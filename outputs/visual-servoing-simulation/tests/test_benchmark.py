"""Statistical and end-to-end checks for the benchmark."""
import tempfile
import unittest
from pathlib import Path
import numpy as np
from analyze_benchmark import summarize, validate_run, wilson_interval
from benchmark import (BENCH_CONFIG, classify_terminal, project_marker_visibility,
                       read_json, run_trial, sample_plan)
from simulation import Simulation


class BenchmarkTests(unittest.TestCase):
    def test_sampling_seed_prefix_bounds_and_profiles(self):
        config = read_json(BENCH_CONFIG)
        plan = sample_plan(200, 20260906, config)
        self.assertEqual(plan, sample_plan(200, 20260906, config))
        self.assertEqual(plan[:12], sample_plan(12, 20260906, config))
        self.assertNotEqual(plan, sample_plan(200, 42, config))
        self.assertEqual({r["profile"] for r in plan}, set(config["profiles_degrees"]))
        for row in plan:
            self.assertTrue(np.all(np.abs(row["offset_degrees"]) <=
                                   np.array(config["profiles_degrees"][row["profile"]])))

    def test_invalid_sampling(self):
        config = read_json(BENCH_CONFIG)
        with self.assertRaises(ValueError):
            sample_plan(0, 1, config)
        with self.assertRaises(ValueError):
            sample_plan(10, -1, config)

    def test_wilson_edge_cases_and_known_value(self):
        np.testing.assert_allclose(wilson_interval(0, 10), (0, .2775327998628892), atol=1e-12)
        np.testing.assert_allclose(wilson_interval(10, 10), (.7224672001371107, 1), atol=1e-12)
        np.testing.assert_allclose(wilson_interval(5, 10), (.236593090512564, .763406909487436), atol=1e-12)
        self.assertEqual(wilson_interval(0, 0), (None, None))
        with self.assertRaises(ValueError):
            wilson_interval(11, 10)

    def test_invisible_starts_remain_in_overall_denominator(self):
        rows = [
            {"outcome": "converged", "initial_detected": True,
             "terminal_time_s": 4., "settling_time_s": 3.5, "final_error_px": .9},
            {"outcome": "timeout", "initial_detected": True},
            {"outcome": "initial_out_of_view", "initial_detected": False},
            {"outcome": "initial_tracking_loss", "initial_detected": False},
        ]
        result = summarize(rows, .95)
        self.assertEqual(result["success_rate_all"], .25)
        self.assertEqual(result["success_rate_initially_detected"], .5)
        self.assertEqual(sum(result["outcomes"].values()), 4)

    def test_empty_group_is_undefined_not_zero_percent(self):
        result = summarize([], .95)
        self.assertIsNone(result["success_rate_all"])
        self.assertIsNone(result["success_rate_initially_detected"])
        self.assertIsNone(result["median_final_error_px_successes"])

    def test_initial_and_runtime_loss_are_distinct(self):
        self.assertEqual(classify_terminal("tracking_loss", True, False), "initial_out_of_view")
        self.assertEqual(classify_terminal("tracking_loss", True, True), "initial_tracking_loss")
        self.assertEqual(classify_terminal("tracking_loss", False, False), "fov_loss")
        self.assertEqual(classify_terminal("tracking_loss", False, True), "tracking_loss")

    def test_stall_classification_and_precedence(self):
        self.assertEqual(classify_terminal("running", False, True, stalled=True), "stalled")
        self.assertEqual(classify_terminal("running", False, True, True, True), "joint_limit_stall")
        self.assertEqual(classify_terminal("running", False, True, True, False, 1e5), "singularity_stall")
        self.assertEqual(classify_terminal("converged", False, True, True, True, 1e5), "converged")
        self.assertEqual(classify_terminal("timeout", False, True), "timeout")

    def test_incomplete_run_cannot_be_reported_as_complete(self):
        with self.assertRaises(ValueError):
            validate_run(Path("."), {"requested_trials": 200, "completed_trials": 12,
                                    "status": "running"}, [])


class TrialIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()
        cls.desired = cls.sim.marker_corners(cls.sim.image())
        cls.bench = read_json(BENCH_CONFIG)

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_convergence_and_complete_stopped_observation(self):
        spec = {"trial_id": 1, "profile": "test",
                "offset_degrees": self.sim.config["ibvs"]["start_offset_degrees"]}
        summary, trace, before, after = run_trial(self.sim, self.desired, spec, self.bench)
        self.assertEqual(summary["outcome"], "converged")
        self.assertGreater(summary["initial_error_px"], 10)
        self.assertLess(summary["final_error_px"], 1)
        post = trace["phase"] == "post_stop"
        self.assertEqual(int(post.sum()), 30)
        self.assertTrue(np.all(trace["error_px"][post] < 1))
        self.assertTrue(np.all(trace["command_rad_s"][post] == 0))
        self.assertGreater(summary["terminal_time_s"], summary["settling_time_s"])
        self.assertEqual(before.shape, (480, 640, 3))
        self.assertEqual(after.shape, before.shape)
        with tempfile.TemporaryDirectory(prefix="vservo-trace-test-") as temp:
            path = Path(temp) / "trace.npz"
            np.savez_compressed(path, **trace)
            with np.load(path, allow_pickle=False) as saved:
                self.assertEqual(saved["corners_px"].shape, (summary["sample_count"], 4, 2))
                self.assertTrue(np.all(np.diff(saved["time_s"]) > 0))

    def test_out_of_view_start_is_recorded_and_stopped(self):
        spec = {"trial_id": 2, "profile": "test", "offset_degrees": [40, 0, 0, 0, 0, 0]}
        summary, trace, _, _ = run_trial(self.sim, self.desired, spec, self.bench)
        self.assertEqual(summary["outcome"], "initial_out_of_view")
        self.assertFalse(summary["initial_detected"])
        self.assertEqual(summary["sample_count"], 1)
        self.assertIsNone(summary["initial_error_px"])
        self.assertTrue(np.all(trace["command_rad_s"] == 0))
        self.assertFalse(project_marker_visibility(self.sim))
        self.sim.reset()
        self.assertTrue(project_marker_visibility(self.sim))


if __name__ == "__main__":
    unittest.main()

