# Success rates and confidence intervals

Every success rate in this project comes from a finite number of trials. Reports
therefore show a 95% confidence interval next to every success count, and tests that
require a minimum success rate decide PASS/FAIL on the interval's lower bound, not
on the observed rate. The calculation is in
`outputs/visual-servoing-simulation/binomial_ci.py`, and the unit tests are in
`tests/test_binomial_ci.py`.

```
Trials: 10
Successes: 10
Observed success rate: 100.0%
95% confidence interval: [69.2%, 100.0%]
```

## What the interval means

The true success rate is the long-run fraction of alignments that would succeed
under the tested conditions. The observed rate, successes divided by trials, is an
estimate of it.

A 95% confidence interval comes from a procedure that contains the true rate in at
least 95% of experiments. Read `[69.2%, 100.0%]` as: *these 10 trials are consistent
with any true success rate from 69.2% to 100%.* A true rate of 75% would produce
10/10 fairly often (about 1 time in 18), so 10/10 cannot rule it out.

## Why 100% observed is not 100% reliability

No finite number of successes proves a 100% success rate. It only rules out rates
that are too low. For n trials that all succeed, the lower bound of the 95% interval
is 0.025^(1/n):

| All-success trials | 5 | 10 | 20 | 30 | 50 | 72 | 100 | 150 | 300 | 1000 |
|---|---|---|---|---|---|---|---|---|---|---|
| True rate is at least | 47.8% | 69.2% | 83.2% | 88.4% | 92.9% | 95.0% | 96.4% | 97.6% | 98.8% | 99.6% |

So "10/10 aligned" supports "at least about 69%", and "150/150" supports "at least
97.6%". Reports state this in words whenever every trial succeeded, for example:
*"All 150 succeeded, but that does not prove 100% reliability: with 95% confidence
the true success rate is at least 97.6%."*

## Method

The project uses the exact **Clopper–Pearson** interval for every success rate.
- **Lower bound:** solves P(X ≥ k | p) = 2.5%.
- **Upper bound:** solves P(X ≤ k | p) = 2.5%.
- X is the number of successes in n trials with success probability p.

The implementation uses only the standard library. It matches
`scipy.stats.beta.ppf` to about 10⁻¹².

Why this method rather than Wilson:
- **It is conservative.** Its coverage is at least 95% for every n and every true
  rate; the unit tests check this. Wilson's coverage dips below 95% for some rates.
  For a claim like "at least 95% reliable", erring on the conservative side is the
  right choice.
- **It is well defined at 0% and 100% observed.** Those are exactly the cases this
  project reports most often.
- **The cost is small.** Clopper–Pearson is usually a little wider. At 100% observed
  the two methods are close: 150/150 gives 97.6% (exact) vs 97.5% (Wilson).

The benchmark, comparison, gain and recovery studies previously used Wilson
intervals. They now use Clopper–Pearson, keeping the same summary field names, and
their `summary.json` records `interval_method`. Reports published before this change
still show the Wilson values until they are regenerated.

The lower bound of a two-sided 95% interval is also a one-sided 97.5% lower
confidence bound. Using it for PASS/FAIL is therefore slightly stricter than a
one-sided 95% bound.

## How PASS/FAIL is decided

A test that requires a minimum success rate passes only if the data demonstrate it:

```
lower bound of the 95% Clopper–Pearson interval  >=  required_success_rate
```

It does **not** use `observed_success_rate >= required_success_rate`. With a
requirement of 95%:

| Result | Observed | 95% interval | Demonstrates ≥ 95%? |
|---|---:|---|---|
| 10/10 | 100% | [69.2%, 100.0%] | no: too few trials |
| 71/71 | 100% | [94.9%, 100.0%] | no |
| 72/72 | 100% | [95.0%, 100.0%] | yes |
| 150/150 | 100% | [97.6%, 100.0%] | yes |
| 149/150 | 99.3% | [96.3%, 100.0%] | yes (lower bound) |
| 147/150 | 98.0% | [94.3%, 99.6%] | no |

**Acceptance test** (`tools/acceptance_test.json`, `acceptance_test_long.json`):

| Criterion | Value | Meaning |
|---|---|---|
| `required_success_rate` | 0.95 | The lower bound for each method must be at least 95%. |
| `confidence` | 0.95 | The interval is 95% two-sided. |
| `max_failed_alignments` | 0 | Kept from the original criteria: any failed alignment fails the test. |

The two rules answer different questions:
- The confidence rule asks whether enough trials were run to support a reliability
  claim. A run cut short can fail it even with no failures.
- The zero-failure rule asks whether anything went wrong at all. Every failure in
  this test is something to investigate.

Setting `max_failed_alignments` to `null` would let the confidence rule decide on
its own. For example, 149/150 would then pass. The standard test gives each method
about 150 alignments, so a clean run passes the confidence rule with a lower bound of
about 97.5%.

**Acceptance campaign** (`tools/acceptance_campaign.json`): the same rules, applied
per session.

**Configurations from before this change** have only `success_rate` (the observed
rate) and keep that rule. `--report-only` on past results therefore reproduces
their original verdicts, and those reports now also show the intervals.

**Margin test:** each level has only 10 alignments, which can never demonstrate
95% (it would need 72). The margin is therefore a characterisation. "Tolerated"
still means every alignment at that level succeeded, and the report states that this
only shows a true rate of at least 69.2%. Adding `required_success_rate` to
`tools/margin_test.json` switches the margin test to the lower-bound rule, which
then needs at least 72 alignments per level.

## How many trials a claim needs

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe outputs\visual-servoing-simulation\binomial_ci.py --needed 0.95
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe outputs\visual-servoing-simulation\binomial_ci.py --needed 0.95 --failures 1
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe outputs\visual-servoing-simulation\binomial_ci.py 147 150
```

In code, use `binomial_ci.trials_needed(required, confidence=0.95, failures=0)`.
Trials needed so that the 95% lower bound reaches the requirement:

| Requirement | 0 failures | 1 failure | 2 failures | 5 failures |
|---|---:|---:|---:|---:|
| 90% | 36 | 54 | 70 | 114 |
| 95% | 72 | 110 | 142 | 230 |
| 99% | 368 | 555 | 720 | 1164 |

A 100% requirement can never be demonstrated, so the configuration validation
rejects `required_success_rate = 1.0`.

## Assumptions and limits

The interval assumes independent trials, each with the same success probability.
Keep in mind what that covers here:

- **What population it describes.** The acceptance test repeats alignments from five
  fixed starting poses on one laptop. Its interval is about those poses under those
  conditions, not about every possible start. The randomized studies (benchmark,
  recovery, startup, coverage) sample starts from a declared distribution, so their
  intervals are about that distribution.
- **Declared or deterministic cases.** Studies built from a few hand-picked cases
  (camera delay, collision, natural image, learned) are not random samples. Their
  intervals mainly show how little a handful of trials can establish.
- **Correlated trials.** Alignments by the same sensor worker, or during the same
  low-power period, are not fully independent. That can make an interval narrower
  than it should be. The acceptance test's other criteria (drift, first vs later
  alignments, hardware telemetry) exist partly to catch this.
- **Comparing two methods.** Checking whether two intervals overlap is not a test of
  a difference. The paired studies compare methods on the same starts, using
  improved and regressed trials, for that reason.
