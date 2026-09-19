"""Recovery behavior, bounded missing-target motion and real app integration."""
import unittest

import numpy as np

from app import Lab
from recovery import ACTIVE_STATES, ReacquiringIBVS, load_recovery_config
from simulation import Simulation


class RecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()
        cls.reference = cls.sim.marker_corners(cls.sim.image())
        cls.home = cls.sim.data.qpos.copy()
        cls.K = cls.sim.camera_intrinsics()
        cls.J = cls.sim.camera_jacobian().copy()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def make_controller(self, **settings):
        config = load_recovery_config()
        config.update(settings)
        c = ReacquiringIBVS(self.reference, self.K, self.sim.config,
                            self.sim.model.jnt_range, self.home, config)
        c.start()
        return c

    def test_loss_brakes_and_requires_consecutive_real_detections(self):
        c = self.make_controller()
        for _ in range(2):
            sample = c.update(None, self.J, self.home, 1/30)
            self.assertEqual(sample.status, "waiting")
            self.assertTrue(np.all(sample.velocity == 0))
        sample = c.update(self.reference, self.J, self.home, 1/30)
        self.assertEqual(sample.status, "confirming")
        self.assertTrue(np.all(sample.velocity == 0))
        c.update(None, self.J, self.home, 1/30)  # Flicker breaks confirmation.
        for _ in range(2):
            self.assertEqual(c.update(self.reference, self.J, self.home, 1/30).status, "confirming")
        sample = c.update(self.reference, self.J, self.home, 1/30)
        self.assertEqual(sample.status, "running")
        self.assertEqual(c.reacquisitions, 1)
        self.assertLess(c.ibvs.held_seconds, c.ibvs.config["success_hold_s"])

    def test_absent_target_times_out_with_bounded_commands(self):
        c = self.make_controller(max_recovery_time_s=1.0, loss_hold_s=0)
        q = self.home.copy()
        q[0] = self.sim.model.jnt_range[0, 1]-.001
        anchor = q.copy()
        states = []
        for _ in range(100):
            sample = c.update(None, self.J, q, 1/30)
            states.append(sample.status)
            self.assertLessEqual(np.max(np.abs(sample.velocity)), .18+1e-12)
            if sample.status not in ACTIVE_STATES:
                break
            q = q + sample.velocity/30  # Encoder-feedback plant for this bound check.
        self.assertEqual(sample.status, "search_timeout")
        self.assertTrue(np.all(sample.velocity == 0))
        self.assertTrue(np.all(np.abs(q-anchor) <= np.deg2rad(60)+1e-12))
        self.assertLessEqual(q[0], anchor[0])  # Starts near the upper limit: only inward motion.
        for _ in range(3):
            sample = c.update(self.reference, self.J, q, 1/30)
            self.assertEqual(sample.status, "search_timeout")
            self.assertTrue(np.all(sample.velocity == 0))

    def test_local_scan_when_remembered_view_no_longer_sees_target(self):
        c = self.make_controller(loss_hold_s=0)
        sample = c.update(None, self.J, self.home, 1/30)
        self.assertEqual(sample.status, "scanning")
        self.assertGreater(np.linalg.norm(sample.velocity), 0)
        self.assertTrue(np.all(sample.velocity[[1,2,3,5]] == 0))
        c.cancel()
        self.assertEqual(c.update(None, self.J, self.home, 1/30).status, "canceled")
        self.assertTrue(np.all(c.update(None, self.J, self.home, 1/30).velocity == 0))

    def test_repeated_losses_and_total_deadline_cannot_restart_forever(self):
        c = self.make_controller(max_recovery_episodes=1)
        c.update(None, self.J, self.home, 1/30)
        for _ in range(3):
            c.update(self.reference, self.J, self.home, 1/30)
        sample = c.update(None, self.J, self.home, 1/30)
        self.assertEqual(sample.status, "recovery_limit")
        self.assertTrue(np.all(sample.velocity == 0))
        c = self.make_controller(max_total_time_s=.1)
        for _ in range(5):
            sample = c.update(None, self.J, self.home, 1/30)
        self.assertEqual(sample.status, "timeout")

    def test_invalid_recovery_configuration_is_rejected(self):
        for change in ({"confirmation_frames":0}, {"max_recovery_time_s":-1},
                       {"max_joint_velocity_rad_s":float("nan")}, {"scan_offsets_degrees":[]}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.make_controller(**change)

    def test_lost_view_button_then_align_reacquires_without_reset(self):
        lab = Lab(self.sim, auto_start=False)
        lab.draw()
        rect = next(r for r,a in lab.buttons if a == "lost_view")
        lab.on_mouse(1, rect[0]+10, rect[1]+10, 0, None)
        self.assertIsNone(self.sim.marker_corners(self.sim.image()))
        start = self.sim.data.qpos.copy()
        lab.align()
        np.testing.assert_array_equal(self.sim.data.qpos, start)
        previous = self.sim.data.qpos.copy()
        states = set()
        for _ in range(1352):
            old_time = self.sim.data.time
            lab.advance(1/30)
            self.assertGreater(self.sim.data.time, old_time)
            self.assertLess(np.max(np.abs(self.sim.data.qpos-previous)), .03)
            previous = self.sim.data.qpos.copy()
            states.add(lab.alignment_status)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status, "converged")
        self.assertTrue({"waiting", "returning", "confirming", "running"} <= states)
        self.assertEqual(lab.controller.reacquisitions, 1)
        for _ in range(30):
            lab.advance(1/30)
            self.assertTrue(np.all(self.sim.velocity_command == 0))
            corners = self.sim.marker_corners(self.sim.image())
            self.assertIsNotNone(corners)
            self.assertLess(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))),1)

    def test_brief_occlusion_during_alignment_resumes(self):
        lab = Lab(self.sim, auto_start=False)
        lab.offset()
        lab.align()
        lab.advance(1/30)
        detector = self.sim.marker_corners
        try:
            self.sim.marker_corners = lambda _: None
            lab.advance(1/30)
            self.assertEqual(lab.alignment_status,"waiting")
            self.assertTrue(np.all(self.sim.velocity_command == 0))
        finally:
            self.sim.marker_corners = detector
        for _ in range(600):
            lab.advance(1/30)
            if not lab.aligning:break
        self.assertEqual(lab.alignment_status, "converged")
        self.assertEqual(lab.controller.reacquisitions,1)

    def test_scan_finds_a_target_that_moved_from_the_taught_view(self):
        import mujoco
        lab = Lab(self.sim, auto_start=False)
        original = self.sim.model.body("target").pos.copy()
        try:
            # Change the simulated scene only; the controller is not told the pose.
            self.sim.model.body("target").pos[1] += .25
            mujoco.mj_forward(self.sim.model, self.sim.data)
            self.assertIsNone(self.sim.marker_corners(self.sim.image()))
            lab.align()
            states = set()
            for _ in range(1352):
                lab.advance(1/30)
                states.add(lab.alignment_status)
                if not lab.aligning:break
            self.assertIn("scanning",states)
            self.assertIn("confirming",states)
            self.assertEqual(lab.alignment_status,"converged")
        finally:
            self.sim.model.body("target").pos[:] = original
            self.sim.reset()

    def test_stop_pause_and_jog_cancel_automatic_recovery(self):
        lab = Lab(self.sim, auto_start=False)
        for action in ("stop", "pause", "jog"):
            with self.subTest(action=action):
                lab.lost_view()
                lab.align()
                for _ in range(12):lab.advance(1/30)
                self.assertEqual(lab.alignment_status, "returning")
                if action == "stop":
                    lab.stop()
                elif action == "pause":
                    lab.toggle_pause()
                    when = self.sim.data.time
                    lab.advance(1)
                    self.assertEqual(self.sim.data.time,when)
                    lab.toggle_pause()
                else:
                    lab.jog(3,1)
                for _ in range(30):lab.advance(1/30)
                self.assertFalse(lab.aligning)
                self.assertEqual(lab.controller.status,"canceled")
                self.assertTrue(np.all(self.sim.velocity_command == 0))


if __name__ == "__main__":
    unittest.main()

