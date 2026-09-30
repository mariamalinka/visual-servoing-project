"""Confidence intervals for binomial success rates (standard library only).

Every success rate in this project is an estimate from a finite number of trials.
An observed 10/10 does not mean the true success probability is 100%: with 95%
confidence it is only known to be at least 69.2%. This module provides the exact
Clopper-Pearson interval used for every reported success rate, a PASS rule based
on its lower bound, and the number of trials needed to support a claim.

    python binomial_ci.py 150 150            # interval for 150/150
    python binomial_ci.py --needed 0.95      # all-success trials for >= 95%

Clopper-Pearson (exact) is used rather than Wilson because it never undercovers:
its true coverage is at least the nominal 95% for every n and p. That is the
conservative choice for a reliability claim. Wilson's coverage can dip below 95%
for some true rates. The cost is that Clopper-Pearson is usually a little wider.
(At 100% observed they are close: 150/150 gives a lower bound of 97.6% exact vs
97.5% Wilson.)
"""
from __future__ import annotations

import argparse
import math
import numbers

DEFAULT_CONFIDENCE = 0.95
METHOD = 'clopper-pearson'
METHOD_LABEL = 'Clopper-Pearson (exact)'


def _counts(successes, trials, confidence):
    """Validate and return plain ints (numpy integer counts are accepted)."""
    for name, value in (('successes', successes), ('trials', trials)):
        if isinstance(value, bool) or not isinstance(value, numbers.Integral):
            raise TypeError(f'{name} must be an integer')
    successes, trials = int(successes), int(trials)
    if trials < 0 or not 0 <= successes <= trials:
        raise ValueError('Require 0 <= successes <= trials')
    if isinstance(confidence, bool) or not (isinstance(confidence, (int, float)) and 0 < confidence < 1):
        raise ValueError('confidence must be in (0, 1)')
    return successes, trials


def _log_pmf(k, n, p):
    if p <= 0.0:
        return 0.0 if k == 0 else -math.inf
    if p >= 1.0:
        return 0.0 if k == n else -math.inf
    return (math.lgamma(n + 1) - math.lgamma(k + 1) - math.lgamma(n - k + 1)
            + k * math.log(p) + (n - k) * math.log1p(-p))


def _probability(ks, n, p):
    """P(X in ks) for X ~ Binomial(n, p), summed in log space."""
    logs = [_log_pmf(k, n, p) for k in ks]
    top = max(logs)
    if top == -math.inf:
        return 0.0
    return min(1.0, math.exp(top) * math.fsum(math.exp(v - top) for v in logs))


def _solve(tail, target):
    """Bisection for p in (0, 1) with tail(p) == target; tail is monotone increasing."""
    low, high = 0.0, 1.0
    for _ in range(200):
        middle = (low + high) / 2
        if tail(middle) < target:
            low = middle
        else:
            high = middle
        if high - low < 1e-15:
            break
    return (low + high) / 2


def clopper_pearson(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE):
    """Two-sided exact interval (low, high) for the success probability.

    Returns (None, None) when there are no trials: no data, no estimate.
    The lower bound solves P(X >= k | p) = alpha/2 and the upper bound solves
    P(X <= k | p) = alpha/2. For k = n the lower bound is (alpha/2) ** (1/n);
    for k = 0 the upper bound is 1 - (alpha/2) ** (1/n).
    """
    successes, trials = _counts(successes, trials, confidence)
    if trials == 0:
        return None, None
    k, n, tail = successes, trials, (1 - confidence) / 2
    if k == 0:
        low = 0.0
    elif k == n:
        low = tail ** (1 / n)
    else:  # P(X >= k) increases with p.
        low = _solve(lambda p: _probability(range(k, n + 1), n, p), tail)
    if k == n:
        high = 1.0
    elif k == 0:
        high = 1 - tail ** (1 / n)
    else:  # P(X > k) = 1 - P(X <= k) increases with p.
        high = _solve(lambda p: _probability(range(k + 1, n + 1), n, p), 1 - tail)
    return low, high


def wilson(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE):
    """Two-sided Wilson score interval, kept for comparison with older reports."""
    successes, trials = _counts(successes, trials, confidence)
    if trials == 0:
        return None, None
    from statistics import NormalDist
    z = NormalDist().inv_cdf((1 + confidence) / 2)
    p = successes / trials
    denominator = 1 + z * z / trials
    center = (p + z * z / (2 * trials)) / denominator
    half = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denominator
    return max(0.0, center - half), min(1.0, center + half)


def summary(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE,
            required: float | None = None) -> dict:
    """JSON-ready record: counts, observed rate, interval and (optionally) the verdict."""
    successes, trials = _counts(successes, trials, confidence)
    low, high = clopper_pearson(successes, trials, confidence)
    record = dict(trials=trials, successes=successes, rate=None if not trials else successes / trials,
                  ci_low=low, ci_high=high, confidence=confidence, ci_method=METHOD)
    if required is not None:
        record.update(required_rate=required, meets_required=meets_required(successes, trials, required, confidence))
    return record


def meets_required(successes: int, trials: int, required: float, confidence: float = DEFAULT_CONFIDENCE) -> bool:
    """PASS rule for a minimum success rate: the lower confidence bound must reach it.

    An observed rate at or above the requirement is not enough; the data must
    also rule out, at this confidence, a true rate below the requirement.
    """
    if not 0 <= required <= 1:
        raise ValueError('required must be in [0, 1]')
    low, _ = clopper_pearson(successes, trials, confidence)
    return low is not None and low >= required


def trials_needed(required: float, confidence: float = DEFAULT_CONFIDENCE, failures: int = 0,
                  limit: int = 10_000_000) -> int | None:
    """Smallest n for which (n - failures)/n successes gives a lower bound >= required.

    With no failures this is ceil(log(alpha/2) / log(required)): 72 all-success
    trials for 95% and 368 for 99%, at 95% confidence. For a fixed number of failures
    the lower bound grows with n, so the answer is found by doubling and bisection.
    Returns None if the requirement can never be demonstrated (required >= 1) or
    needs more than `limit` trials.
    """
    if isinstance(failures, bool) or not isinstance(failures, int) or failures < 0:
        raise ValueError('failures must be a nonnegative integer')
    if not 0 <= required <= 1:
        raise ValueError('required must be in [0, 1]')
    if required >= 1:
        return None
    ok = lambda n: meets_required(n - failures, n, required, confidence)
    high = failures + 1
    while not ok(high):
        if high > limit:
            return None
        high *= 2
    low = max(failures, high // 2)  # ok(low) is False (or low has no successes).
    while high - low > 1:
        middle = (low + high) // 2
        if ok(middle):
            high = middle
        else:
            low = middle
    return high


def percent(value, digits=1):
    return 'n/a' if value is None else f'{100 * value:.{digits}f}%'


def format_interval(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE) -> str:
    """'[69.2%, 100.0%]' (or 'n/a' without trials)."""
    low, high = clopper_pearson(successes, trials, confidence)
    return 'n/a' if low is None else f'[{percent(low)}, {percent(high)}]'


def format_rate(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE) -> str:
    """One line for tables: '10/10 (100.0%; 95% CI 69.2-100.0%)'."""
    if not trials:
        return f'{successes}/{trials} (n/a)'
    low, high = clopper_pearson(successes, trials, confidence)
    return (f'{successes}/{trials} ({percent(successes / trials)}; '
            f'{confidence * 100:g}% CI {100 * low:.1f}-{100 * high:.1f}%)')


def format_block(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE) -> str:
    """Multi-line form used in text reports and by the command line."""
    return '\n'.join([f'Trials: {trials}', f'Successes: {successes}',
                      f'Observed success rate: {percent(None if not trials else successes / trials)}',
                      f'{confidence * 100:g}% confidence interval: {format_interval(successes, trials, confidence)}'])


def interpretation(successes: int, trials: int, confidence: float = DEFAULT_CONFIDENCE) -> str:
    """Plain-language reading of a result, never presenting 100% observed as 100% reliable."""
    if not trials:
        return 'No trials: no estimate.'
    low, high = clopper_pearson(successes, trials, confidence)
    if successes == trials:
        return (f'All {trials} succeeded, but that does not prove 100% reliability: with {confidence * 100:g}% '
                f'confidence the true success rate is at least {percent(low)}.')
    if successes == 0:
        return f'None succeeded; with {confidence * 100:g}% confidence the true success rate is at most {percent(high)}.'
    return (f'With {confidence * 100:g}% confidence the true success rate is between {percent(low)} '
            f'and {percent(high)}.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('successes', nargs='?', type=int)
    parser.add_argument('trials', nargs='?', type=int)
    parser.add_argument('--confidence', type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument('--needed', type=float, metavar='RATE',
                        help='Trials needed for the lower bound to reach RATE')
    parser.add_argument('--failures', type=int, default=0, help='With --needed: failures allowed')
    args = parser.parse_args(argv)
    if args.needed is not None:
        n = trials_needed(args.needed, args.confidence, args.failures)
        print('not achievable' if n is None else
              f'{n} trials with at most {args.failures} failure(s) give a {args.confidence * 100:g}% lower bound '
              f'>= {percent(args.needed)} ({format_interval(n - args.failures, n, args.confidence)}).')
        return 0
    if args.successes is None or args.trials is None:
        parser.error('give SUCCESSES TRIALS, or --needed RATE')
    print(format_block(args.successes, args.trials, args.confidence))
    print(interpretation(args.successes, args.trials, args.confidence))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
