"""Safety metrics recorded on every physics step (Simulation.safety_record, safety_metrics.py).

    python -B -m unittest tests.test_safety_metrics -v

Headless (no rendering): suitable for CI.
"""
import unittest

import numpy as np

import safety_metrics
from simulation import Simulation


def record(**values):
    base = dict(steps=10, min_environment_slack_m=0.004, min_environment_pair=("a", "b"), min_self_slack_m=0.01,
                min_self_pair=("c", "d"), min_joint_margin_rad=0.5,
                min_joint_margin_joint=1, peak_command_rad_s=0.35, peak_speed_rad_s=0.34, peak_speed_joint=0,
                limits=dict(max_joint_velocity_rad_s=0.6))
    base.update(values)
    return base


class Evaluate(unittest.TestCase):
    def test_every_metric_has_value_limit_verdict_and_requirement(self):
        metrics = safety_metrics.evaluate(record())
        self.assertEqual([m["requirement"] for m in metrics], ["REQ-11", "REQ-11", "REQ-12", "REQ-13", "REQ-13"])
        self.assertTrue(all(m["passed"] is True for m in metrics))
        self.assertAlmostEqual(metrics[0]["value"], 4.0)
        self.assertAlmostEqual(metrics[1]["value"], 10.0)
        self.assertEqual((metrics[0]["limit"], metrics[3]["limit"]), (0.0, 0.6))
        self.assertEqual(safety_metrics.failures(metrics), [])

    def test_violations_fail(self):
        cases = dict(min_environment_slack_m=-0.001, min_self_slack_m=-0.0005, min_joint_margin_rad=-0.01,
                     peak_command_rad_s=0.61,
                     peak_speed_rad_s=0.62)
        for key, value in cases.items():
            with self.subTest(key=key):
                metrics = safety_metrics.evaluate(record(**{key: value}))
                self.assertEqual(sum(m["passed"] is False for m in metrics), 1)
                self.assertEqual(len(safety_metrics.failures(metrics)), 1)

    def test_boundaries_pass(self):
        metrics = safety_metrics.evaluate(record(min_environment_slack_m=0.0, min_self_slack_m=0.0, min_joint_margin_rad=0.0,
                                                 peak_command_rad_s=0.6, peak_speed_rad_s=0.6))
        self.assertTrue(all(m["passed"] for m in metrics))

    def test_missing_values_are_not_recorded_never_pass(self):
        metrics = safety_metrics.evaluate(record(min_environment_slack_m=None))
        self.assertIsNone(metrics[0]["passed"])
        self.assertEqual(safety_metrics.verdict(metrics[0]), "not recorded")
        self.assertTrue(all(m["passed"] is None for m in safety_metrics.evaluate(None)))
        self.assertEqual(safety_metrics.failures(safety_metrics.evaluate(None)), [])
        self.assertIn("not recorded", "\n".join(safety_metrics.table_lines([("old run", None)])))

    def test_merge_keeps_the_worst_session(self):
        merged = safety_metrics.merge([record(min_environment_slack_m=0.002, peak_speed_rad_s=0.2),
                                       None, record(min_joint_margin_rad=0.1, peak_speed_rad_s=0.3, peak_speed_joint=4)])
        self.assertEqual((merged["min_environment_slack_m"], merged["min_joint_margin_rad"]), (0.002, 0.1))
        self.assertEqual((merged["peak_speed_rad_s"], merged["peak_speed_joint"], merged["steps"]), (0.3, 4, 20))
        self.assertIsNone(safety_metrics.merge([None, {}]))

    def test_table_and_csv(self):
        metrics = safety_metrics.evaluate(record(peak_speed_rad_s=0.7))
        text = "\n".join(safety_metrics.table_lines([("SIFT", metrics)]))
        self.assertIn("**FAIL**", text)
        self.assertIn("REQ-13", text)
        row = safety_metrics.flat(metrics)
        self.assertEqual((row["measured_speed_pass"], row["environment_clearance_mm"]), (0, 4.0))


class SimulationRecord(unittest.TestCase):
    """The record reuses the collision guard's own per-step distance check."""

    def test_clearance_matches_the_guard_geometry(self):
        with Simulation(render=False) as sim:
            sim.set_obstacle(True)
            sim.reset()
            sim.reset_safety_record()
            sim.command_velocity(np.array([0.35, 0, 0, 0, 0, 0]))
            lowest = dict(environment=np.inf, self=np.inf)
            for _ in range(300):
                distances, _ = sim.collision.distances(sim.data.qpos)  # Independent check before each step.
                slack = distances - sim.collision.margins
                robot = sim.collision.margins == sim.collision.config["self_clearance_m"]
                lowest["environment"] = min(lowest["environment"], float(np.min(slack[~robot])))
                lowest["self"] = min(lowest["self"], float(np.min(slack[robot])))
                sim.advance(0.002)
            safety = sim.safety_record()
            self.assertEqual(safety["steps"], 300)
            for kind in ("environment", "self"):
                self.assertAlmostEqual(safety[f"min_{kind}_slack_m"], lowest[kind], places=9)
                self.assertIsNotNone(safety[f"min_{kind}_pair"])
                self.assertGreaterEqual(safety[f"min_{kind}_slack_m"], 0)

    def test_joint_margin_and_speeds(self):
        with Simulation(render=False) as sim:
            sim.reset()
            sim.reset_safety_record()
            sim.command_velocity(np.array([0, 0, 0, 0.3, 0, 0]))
            sim.advance(0.5)
            safety = sim.safety_record()
            lower, upper = sim.model.jnt_range.T
            margin = float(np.min(np.minimum(sim.data.qpos - lower, upper - sim.data.qpos)))
            self.assertLessEqual(safety["min_joint_margin_rad"], margin + 1e-12)
            self.assertAlmostEqual(safety["peak_command_rad_s"], 0.3)
            self.assertLessEqual(safety["peak_speed_rad_s"], 0.3 + 1e-6)
            self.assertEqual(safety["peak_speed_joint"], 3)
            self.assertEqual(safety["limits"]["max_joint_velocity_rad_s"], 0.6)
            self.assertTrue(all(m["passed"] for m in safety_metrics.evaluate(safety)))

    def test_disabled_collision_guard_is_not_recorded(self):
        with Simulation(dict(enabled=False), render=False) as sim:
            sim.command_velocity(np.array([0.1, 0, 0, 0, 0, 0]))
            sim.advance(0.1)
            metrics = safety_metrics.evaluate(sim.safety_record())
            self.assertIsNone(metrics[0]["passed"])
            self.assertIsNone(metrics[1]["passed"])


if __name__ == "__main__":
    unittest.main()
