"""Exact binomial confidence intervals and the lower-bound PASS rule (standard library only).

    python -B -m unittest tests.test_binomial_ci -v
"""
import math
import unittest

from binomial_ci import (clopper_pearson, format_block, format_rate, interpretation, meets_required,
                         summary, trials_needed, wilson)

# Reference values: scipy.stats.beta.ppf (Clopper-Pearson), checked to 1e-10.
REFERENCE = {
    (10, 10): (0.6915028921812392, 1.0),
    (150, 150): (0.9757074028338915, 1.0),
    (0, 10): (0.0, 0.3084971078187607),
    (0, 150): (0.0, 0.02429259716610854),
    (5, 10): (0.18708602844739855, 0.8129139715526015),
    (3, 7): (0.09898827844250786, 0.8159484323599169),
    (149, 150): (0.9634168322594198, 0.99983122885689),
    (147, 150): (0.9426657777117942, 0.9958563747187794),
}


class ClopperPearson(unittest.TestCase):
    def test_reference_values(self):
        for (k, n), expected in REFERENCE.items():
            with self.subTest(k=k, n=n):
                low, high = clopper_pearson(k, n)
                self.assertAlmostEqual(low, expected[0], places=10)
                self.assertAlmostEqual(high, expected[1], places=10)

    def test_zero_successes(self):
        for n in (1, 10, 150):
            low, high = clopper_pearson(0, n)
            self.assertEqual(low, 0.0)
            self.assertAlmostEqual(high, 1 - 0.025 ** (1 / n), places=12)

    def test_all_successes_is_never_proof_of_100_percent(self):
        for n in (1, 10, 150, 10_000):
            low, high = clopper_pearson(n, n)
            self.assertEqual(high, 1.0)
            self.assertLess(low, 1.0)
            self.assertAlmostEqual(low, 0.025 ** (1 / n), places=12)

    def test_partial_success_contains_the_observed_rate(self):
        for k, n in ((1, 2), (5, 10), (3, 7), (147, 150), (999, 1000)):
            low, high = clopper_pearson(k, n)
            self.assertLess(low, k / n)
            self.assertGreater(high, k / n)

    def test_ten_of_ten(self):
        low, high = clopper_pearson(10, 10)
        self.assertEqual(round(100 * low, 1), 69.2)
        self.assertEqual(high, 1.0)
        self.assertEqual(format_block(10, 10), 'Trials: 10\nSuccesses: 10\nObserved success rate: 100.0%\n'
                                               '95% confidence interval: [69.2%, 100.0%]')

    def test_150_of_150(self):
        low, _ = clopper_pearson(150, 150)
        self.assertAlmostEqual(low, 0.9757, places=4)
        self.assertEqual(format_rate(150, 150), '150/150 (100.0%; 95% CI 97.6-100.0%)')

    def test_confidence_level_widens_the_interval(self):
        narrow, wide = clopper_pearson(8, 10, 0.90), clopper_pearson(8, 10, 0.99)
        self.assertLess(wide[0], narrow[0])
        self.assertGreater(wide[1], narrow[1])

    def test_symmetry(self):
        for k, n in ((0, 5), (2, 9), (40, 150)):
            low, high = clopper_pearson(k, n)
            low2, high2 = clopper_pearson(n - k, n)
            self.assertAlmostEqual(low, 1 - high2, places=10)
            self.assertAlmostEqual(high, 1 - low2, places=10)

    def test_no_trials_has_no_estimate(self):
        self.assertEqual(clopper_pearson(0, 0), (None, None))
        self.assertEqual(format_rate(0, 0), '0/0 (n/a)')
        self.assertFalse(meets_required(0, 0, 0.5))

    def test_numpy_integer_counts_are_accepted(self):
        import numpy as np
        self.assertEqual(clopper_pearson(np.int64(10), np.int32(10)), clopper_pearson(10, 10))
        self.assertEqual(format_rate(np.int64(3), 3), format_rate(3, 3))
        self.assertIs(type(summary(np.int64(3), np.int64(4))['trials']), int)  # JSON-serialisable.

    def test_invalid_input(self):
        for args in ((11, 10), (-1, 10), (1, -1)):
            with self.assertRaises(ValueError):
                clopper_pearson(*args)
        for args in ((1.0, 10), (True, 10)):
            with self.assertRaises(TypeError):
                clopper_pearson(*args)
        for confidence in (0, 1, 1.5, True):
            with self.assertRaises(ValueError):
                clopper_pearson(5, 10, confidence)

    def test_coverage_is_never_below_nominal(self):
        # The reason for choosing the exact method: for any true p, the interval
        # contains p with probability >= 95%. Wilson does not guarantee this.
        for n in (10, 30):
            intervals = [clopper_pearson(k, n) for k in range(n + 1)]
            for i in range(1, 200):
                p = i / 200
                coverage = sum(math.comb(n, k) * p ** k * (1 - p) ** (n - k)
                               for k, (low, high) in enumerate(intervals) if low <= p <= high)
                self.assertGreaterEqual(coverage, 0.95 - 1e-12, (n, p))

    def test_wilson_is_available_for_comparison(self):
        low, high = wilson(150, 150)
        self.assertAlmostEqual(low, 150 / (150 + 1.959963984540054 ** 2), places=12)
        self.assertEqual(high, 1.0)


class PassRule(unittest.TestCase):
    def test_ten_of_ten_does_not_demonstrate_95_percent(self):
        self.assertFalse(meets_required(10, 10, 0.95))

    def test_enough_all_success_trials_do(self):
        self.assertTrue(meets_required(72, 72, 0.95))
        self.assertFalse(meets_required(71, 71, 0.95))
        self.assertTrue(meets_required(150, 150, 0.95))

    def test_failures_can_still_pass_with_enough_trials(self):
        self.assertTrue(meets_required(149, 150, 0.95))
        self.assertFalse(meets_required(147, 150, 0.95))

    def test_summary_record(self):
        record = summary(10, 10, required=0.95)
        self.assertEqual((record['trials'], record['successes'], record['rate']), (10, 10, 1.0))
        self.assertEqual(record['ci_method'], 'clopper-pearson')
        self.assertFalse(record['meets_required'])

    def test_interpretation_never_claims_certainty(self):
        text = interpretation(150, 150)
        self.assertIn('does not prove 100% reliability', text)
        self.assertIn('97.6%', text)
        self.assertIn('at most 30.8%', interpretation(0, 10))


class TrialsNeeded(unittest.TestCase):
    def test_all_success_closed_form(self):
        for required in (0.8, 0.9, 0.95, 0.99, 0.999):
            with self.subTest(required=required):
                expected = math.ceil(math.log(0.025) / math.log(required))
                self.assertEqual(trials_needed(required), expected)

    def test_known_answers(self):
        self.assertEqual(trials_needed(0.95), 72)
        self.assertEqual(trials_needed(0.99), 368)
        self.assertEqual(trials_needed(0.95, failures=1), 110)

    def test_result_is_the_smallest_sufficient_n(self):
        for required, failures in ((0.9, 0), (0.95, 2), (0.97, 1)):
            n = trials_needed(required, failures=failures)
            self.assertTrue(meets_required(n - failures, n, required))
            self.assertFalse(meets_required(n - 1 - failures, n - 1, required))

    def test_one_hundred_percent_can_never_be_demonstrated(self):
        self.assertIsNone(trials_needed(1.0))


if __name__ == '__main__':
    unittest.main()
