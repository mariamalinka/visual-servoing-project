"""Independent geometry, feedback-isolation and paired simulation checks."""
import copy
import unittest
from pathlib import Path

import cv2
import numpy as np

from analyze_comparison import validate_run
from baselines import (PoseController, estimate_marker_pose, inverse_pose,
                       joint_command, pose_error)
from benchmark import BENCH_CONFIG, read_json, run_trial
from compare_controllers import COMPARISON_CONFIG, run_comparison_trial
from control import marker_object_points
from simulation import ROOT, Simulation


class PoseMathTests(unittest.TestCase):
    def setUp(self):
        self.config = read_json(ROOT / "config.json")
        self.comparison = read_json(COMPARISON_CONFIG)
        self.K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
        self.reference_pose = np.eye(4)
        self.reference_pose[:3, :3] = cv2.Rodrigues(np.array([2.9, .1, -.1]))[0]
        self.reference_pose[:3, 3] = [.02, -.01, .7]
        self.reference = self.project(self.reference_pose)

    def project(self, pose):
        return cv2.projectPoints(marker_object_points(.24), cv2.Rodrigues(pose[:3, :3])[0],
                                 pose[:3, 3], self.K, None)[0].reshape(4, 2)

    def test_pose_estimate_recovers_independent_projection(self):
        estimated, depths = estimate_marker_pose(self.reference, self.K, .24, 3.)
        np.testing.assert_allclose(estimated, self.reference_pose, atol=1e-10)
        np.testing.assert_allclose(estimated @ inverse_pose(estimated), np.eye(4), atol=1e-12)
        np.testing.assert_allclose(depths, (marker_object_points(.24) @
                                   self.reference_pose[:3, :3].T + self.reference_pose[:3, 3])[:, 2])

    def test_pbvs_command_reduces_translation_and_rotation_error(self):
        rng = np.random.default_rng(42)
        for _ in range(20):
            camera = np.eye(4)
            camera[:3, :3] = cv2.Rodrigues(rng.uniform(-.1, .1, 3))[0]
            camera[:3, 3] = rng.uniform(-.02, .02, 3)
            current_marker = inverse_pose(camera) @ self.reference_pose
            controller = PoseController("pbvs", self.reference, self.K, self.config, self.comparison)
            sample = controller.update(self.project(current_marker), np.eye(6), 1/30, camera)
            self.assertEqual(sample.status, "running")
            moved = camera.copy()
            moved[:3, 3] += camera[:3, :3] @ sample.camera_twist[:3] * .001
            moved[:3, :3] = camera[:3, :3] @ cv2.Rodrigues(sample.camera_twist[3:] * .001)[0]
            before, after = pose_error(inverse_pose(camera)), pose_error(inverse_pose(moved))
            self.assertLess(np.linalg.norm(after[:3]), np.linalg.norm(before[:3]))
            self.assertLess(np.linalg.norm(after[3:]), np.linalg.norm(before[3:]))

    def test_open_loop_command_is_independent_of_all_later_images(self):
        camera = np.eye(4)
        camera[:3, 3] = [.015, -.01, -.02]
        first = self.project(inverse_pose(camera) @ self.reference_pose)
        a = PoseController("open_loop", self.reference, self.K, self.config, self.comparison)
        a.update(first, np.eye(6), 1/30, camera)
        b = copy.deepcopy(a)
        frozen_goal = a.world_from_goal.copy()
        for _ in range(8):
            left = a.update(None, np.eye(6), 1/30, camera)
            right = b.update(self.reference + 1000, np.eye(6), 1/30, camera)
            np.testing.assert_array_equal(left.velocity, right.velocity)
            self.assertEqual(left.status, right.status)
            np.testing.assert_array_equal(a.world_from_goal, frozen_goal)
        np.testing.assert_allclose(frozen_goal, np.eye(4), atol=1e-10)

    def test_initial_loss_and_pbvs_runtime_loss_stop(self):
        for method in ("pbvs", "open_loop"):
            control = PoseController(method, self.reference, self.K, self.config, self.comparison)
            sample = control.update(None, np.eye(6), 1/30, np.eye(4))
            self.assertEqual(sample.status, "tracking_loss")
            self.assertFalse(sample.velocity.any())
        pbvs = PoseController("pbvs", self.reference, self.K, self.config, self.comparison)
        pbvs.update(self.reference + 5, np.eye(6), 1/30, np.eye(4))
        self.assertEqual(pbvs.update(None, np.eye(6), 1/30, np.eye(4)).status, "tracking_loss")

    def test_timeout_and_speed_bounds(self):
        velocity, twist, _ = joint_command(np.ones(6)*100, np.diag([1, .01, .001, 1, 1, 1]), self.config["ibvs"])
        self.assertLessEqual(np.max(np.abs(velocity)), .35 + 1e-12)
        self.assertLessEqual(np.linalg.norm(twist[:3]), .12 + 1e-12)
        self.assertLessEqual(np.linalg.norm(twist[3:]), .4 + 1e-12)
        for method in ("pbvs", "open_loop"):
            control = PoseController(method, self.reference, self.K, self.config, self.comparison)
            control.elapsed_seconds = 20.
            sample = control.update(self.reference, np.eye(6), 1/30, np.eye(4))
            self.assertEqual(sample.status, "timeout")
            self.assertFalse(sample.velocity.any())

    def test_incomplete_comparison_is_not_reported(self):
        with self.assertRaises(ValueError):
            validate_run(Path("."), {"status": "running"}, [])


class ComparisonIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()
        cls.desired = cls.sim.marker_corners(cls.sim.image())
        cls.bench = read_json(BENCH_CONFIG)
        cls.comparison = read_json(COMPARISON_CONFIG)
        cls.spec = {"trial_id": 1, "profile": "test",
                    "offset_degrees": cls.sim.config["ibvs"]["start_offset_degrees"]}

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_ibvs_comparison_matches_original_runner_exactly(self):
        previous, old_trace, _, _ = run_trial(self.sim, self.desired, self.spec, self.bench)
        current, new_trace, _, _ = run_comparison_trial(
            self.sim, self.desired, self.spec, self.bench, self.comparison, "ibvs")
        self.assertEqual(current["outcome"], previous["outcome"])
        self.assertEqual(current["terminal_time_s"], previous["terminal_time_s"])
        for name in ("qpos_rad", "command_rad_s", "error_px"):
            np.testing.assert_array_equal(new_trace[name], old_trace[name])

    def test_target_step_feedback_and_restoration(self):
        original = self.sim.model.body_pos[self.sim.model.body("target").id].copy()
        for method in ("ibvs", "pbvs", "open_loop"):
            row, trace, _, _ = run_comparison_trial(
                self.sim, self.desired, self.spec, self.bench, self.comparison,
                method, self.comparison["target_step"])
            self.assertTrue(trace["target_step_applied"].any())
            np.testing.assert_array_equal(self.sim.model.body_pos[self.sim.model.body("target").id], original)
            if method in ("ibvs", "pbvs"):
                self.assertEqual(row["outcome"], "converged", row)
                self.assertLess(row["final_error_px"], 1.)
                self.assertEqual(int((trace["phase"] == "post_stop").sum()), 30)
            else:
                self.assertNotEqual(row["outcome"], "converged", row)
                self.assertGreater(row["final_error_px"], 5.)
            self.assertFalse(self.sim.velocity_command.any())


if __name__ == "__main__":
    unittest.main()
