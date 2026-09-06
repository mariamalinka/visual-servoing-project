"""Independent numerical checks of projection, frames, depth, and stop behavior."""
import json
import unittest

import cv2
import mujoco
import numpy as np

from control import (IBVSController, damped_pinv, estimate_depths, interaction_matrix,
                     marker_object_points, normalized_points)
from simulation import ROOT, Simulation


class ControlMathTests(unittest.TestCase):
    def test_point_matrix_against_camera_motion_finite_differences(self):
        rng = np.random.default_rng(42)
        epsilon = 1e-6
        for _ in range(50):
            depths = rng.uniform(0.2, 2.5, 8)
            xy = rng.uniform(-0.8, 0.8, (8, 2))
            points = np.column_stack((xy * depths[:, None], depths))
            numerical = np.zeros((16, 6))
            for axis in range(6):
                twist = np.eye(6)[axis]
                projected = []
                for h in (epsilon, -epsilon):
                    rotation, _ = cv2.Rodrigues(twist[3:] * h)
                    moved = (points - twist[:3] * h) @ rotation
                    projected.append((moved[:, :2] / moved[:, 2:3]).ravel())
                numerical[:, axis] = (projected[0] - projected[1]) / (2 * epsilon)
            np.testing.assert_allclose(interaction_matrix(xy, depths), numerical,
                                       atol=2e-8, rtol=2e-8)

    def test_pinhole_normalization(self):
        K = np.array([[500, 0, 320], [0, 450, 240], [0, 0, 1.]])
        np.testing.assert_allclose(normalized_points(np.array([[370., 150.]]), K), [[0.1, -0.2]])

    def test_damping_stays_bounded_at_singularity(self):
        rng = np.random.default_rng(13)
        u, _ = np.linalg.qr(rng.normal(size=(6, 6)))
        v, _ = np.linalg.qr(rng.normal(size=(6, 6)))
        matrix = u @ np.diag([1, 0.5, 0.1, 0.01, 1e-14, 0]) @ v.T
        inverse = damped_pinv(matrix, 0.01)
        self.assertTrue(np.isfinite(inverse).all())
        self.assertLessEqual(np.linalg.norm(inverse, 2), 50 + 1e-8)
        np.testing.assert_allclose(matrix @ inverse @ matrix, matrix, atol=0.006)

    def test_known_rotation_depth_recovered_from_pixels(self):
        K = np.array([[461., 0, 320], [0, 461., 240], [0, 0, 1.]])
        obj = marker_object_points(0.24)
        rvec = np.array([3.02, 0.05, 0.10])
        tvec = np.array([0.025, -0.02, 0.65])
        pixels, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
        rotation, _ = cv2.Rodrigues(rvec)
        expected = (obj @ rotation.T + tvec)[:, 2]
        np.testing.assert_allclose(estimate_depths(pixels.reshape(4, 2), K, 0.24, 3),
                                   expected, atol=1e-8)

    def test_bad_math_inputs_rejected(self):
        with self.assertRaises(ValueError):
            interaction_matrix(np.zeros((4, 2)), np.array([1, 1, 0, 1]))
        with self.assertRaises(ValueError):
            damped_pinv(np.eye(6), 0)
        with self.assertRaises(ValueError):
            estimate_depths(np.full((4, 2), np.nan), np.eye(3), 0.24, 3)

    def controller(self):
        config = json.loads((ROOT / "config.json").read_text())
        reference = np.array([[190., 110], [450, 110], [450, 370], [190, 370]])
        K = np.array([[461., 0, 320], [0, 461., 240], [0, 0, 1.]])
        return IBVSController(reference, K, config)

    def test_tracking_loss_stops_until_explicit_restart(self):
        control = self.controller()
        sample = control.update(None, np.eye(6), 1/30)
        self.assertEqual(sample.status, "tracking_loss")
        self.assertFalse(np.any(sample.velocity))
        resumed = control.update(control.desired, np.eye(6), 1/30)
        self.assertEqual(resumed.status, "tracking_loss")
        control.reset()
        self.assertEqual(control.status, "running")

    def test_success_requires_full_observed_hold_window(self):
        control = self.controller()
        for _ in range(15):
            sample = control.update(control.desired, np.eye(6), 1/30)
            self.assertEqual(sample.status, "running")
            self.assertFalse(np.any(sample.velocity))
        sample = control.update(control.desired, np.eye(6), 1/30)
        self.assertEqual(sample.status, "converged")
        self.assertAlmostEqual(control.held_seconds, 0.5)

    def test_timeout_stops(self):
        control = self.controller()
        control.elapsed_seconds = control.config["timeout_s"]
        sample = control.update(control.desired + 10, np.eye(6), 1/30)
        self.assertEqual(sample.status, "timeout")
        self.assertFalse(np.any(sample.velocity))


class CameraFrameTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_camera_origin_jacobian_and_optical_axes(self):
        sim = self.sim
        epsilon = 1e-6
        for offset in ([0, 0, 0, 0, 0, 0], [5, -6, 8, 10, -5, 3], [-8, 5, -7, -10, 8, -6]):
            sim.reset(offset)
            home_q = sim.data.qpos.copy()
            pose = sim.camera_pose()
            analytical = sim.camera_jacobian()
            numerical = np.zeros((6, 6))
            for axis in range(6):
                poses = []
                for h in (epsilon, -epsilon):
                    sim.data.qpos[:] = home_q
                    sim.data.qpos[axis] += h
                    mujoco.mj_forward(sim.model, sim.data)
                    poses.append(sim.camera_pose())
                linear_world = (poses[0][:3, 3] - poses[1][:3, 3]) / (2 * epsilon)
                rdot = (poses[0][:3, :3] - poses[1][:3, :3]) / (2 * epsilon)
                skew_world = rdot @ pose[:3, :3].T
                angular_world = np.array([skew_world[2, 1], skew_world[0, 2], skew_world[1, 0]])
                numerical[:, axis] = np.r_[pose[:3, :3].T @ linear_world,
                                           pose[:3, :3].T @ angular_world]
            np.testing.assert_allclose(analytical, numerical, atol=2e-8, rtol=2e-8)

    def test_full_joint_to_image_chain_against_projection(self):
        sim = self.sim
        sim.reset([3, -4, 6, 5, -3, 4])
        q = sim.data.qpos.copy()
        pose = sim.camera_pose()
        points_optical = np.array([[-0.12, -0.1, .48], [.12, -.1, .49],
                                   [.12, .1, .51], [-.12, .1, .50]])
        world_points = points_optical @ pose[:3, :3].T + pose[:3, 3]
        L = interaction_matrix(points_optical[:, :2] / points_optical[:, 2:3], points_optical[:, 2])
        predicted = L @ sim.camera_jacobian()
        measured = np.zeros((8, 6))
        for axis in range(6):
            pixels = []
            for h in (1e-6, -1e-6):
                sim.data.qpos[:] = q
                sim.data.qpos[axis] += h
                mujoco.mj_forward(sim.model, sim.data)
                shifted = sim.camera_pose()
                optical = (world_points - shifted[:3, 3]) @ shifted[:3, :3]
                pixels.append((optical[:, :2] / optical[:, 2:3]).ravel())
            measured[:, axis] = (pixels[0] - pixels[1]) / 2e-6
        np.testing.assert_allclose(predicted, measured, atol=2e-8, rtol=2e-8)


if __name__ == "__main__":
    unittest.main()
