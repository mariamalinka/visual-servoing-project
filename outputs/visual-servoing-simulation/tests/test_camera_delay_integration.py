"""Delayed visual feedback against real MuJoCo motion and app controls."""
import unittest
import json
import tempfile
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from app import Lab
from camera_timing import load_camera_timing
from perception import Observation
from simulation import Simulation


class CameraDelayIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.set_target_mode("aruco")
        self.sim.reset()

    def lab(self, delay=.1, **settings):
        lab = Lab(self.sim, auto_start=False,
                  camera_timing=dict(load_camera_timing(delay), **settings))
        lab.offset()
        lab.align()
        return lab

    def run_until_stopped(self, lab, limit=800):
        for _ in range(limit):
            lab.advance(1/30)
            if not lab.aligning:
                return
        self.fail("Timed-camera run did not finish")

    def test_no_delivery_no_command_and_drawing_cannot_bypass_delay(self):
        lab = self.lab()
        lab.advance(.08)
        self.assertIsNone(lab.camera.latest)
        self.assertIsNone(lab.controller.last_visible_qpos)
        self.assertFalse(self.sim.velocity_command.any())
        q, t = self.sim.data.qpos.copy(), self.sim.data.time
        with patch.object(lab.perception, "observe", side_effect=AssertionError("Drawing cannot detect")):
            for _ in range(3):
                lab.draw()
        np.testing.assert_array_equal(self.sim.data.qpos, q)
        self.assertEqual(self.sim.data.time, t)
        self.assertEqual(lab.camera.delivered_frames, 0)

    def test_motion_continues_during_delay_and_memory_uses_capture_pose(self):
        lab = self.lab()
        lab.advance(.5)
        frame = lab.camera.latest
        self.assertTrue(self.sim.velocity_command.any())
        self.assertGreater(np.linalg.norm(self.sim.data.qpos-frame.qpos), 1e-4)
        self.assertAlmostEqual(frame.available_s-frame.captured_s, .1)
        np.testing.assert_array_equal(lab.controller.last_visible_qpos, frame.qpos)
        # The image shown with the outline is the one actually used for control.
        with patch.object(lab.perception, "observe", side_effect=AssertionError("No display inference")):
            lab.draw()
        np.testing.assert_array_equal(lab.last_rgb, frame.rgb)

    def test_stream_outage_stops_at_capture_age_deadline_and_does_not_restart(self):
        lab = self.lab()
        lab.advance(.5)
        last_capture = lab.camera.last_capture_s
        lab.toggle_camera_stream()
        self.run_until_stopped(lab, 20)
        self.assertEqual(lab.alignment_status, "stale_camera")
        self.assertFalse(self.sim.velocity_command.any())
        stopped = lab.last_camera_failure["simulation_time_s"]
        self.assertLessEqual(stopped-last_capture, .25+self.sim.model.opt.timestep+1e-9)
        self.assertGreaterEqual(stopped-last_capture, .25-1e-9)
        lab.auto_enabled = True
        lab.toggle_camera_stream()
        lab.advance(.5)
        self.assertFalse(lab.aligning)
        self.assertEqual(lab.alignment_status, "stale_camera")
        self.assertFalse(self.sim.velocity_command.any())
        lab.align()
        lab.advance(.3)
        self.assertTrue(lab.aligning)

    def test_delay_beyond_age_budget_never_moves(self):
        lab = self.lab(.3)
        for _ in range(10):
            lab.advance(1/30)
            self.assertFalse(self.sim.velocity_command.any())
        self.assertEqual(lab.alignment_status, "stale_camera")
        self.assertEqual(lab.camera.delivered_frames, 0)

    def test_missing_target_with_live_camera_still_uses_normal_recovery(self):
        lab = self.lab()
        lab.advance(.5)
        board = self.sim.model.geom("target_board")
        alpha = float(board.rgba[3])
        try:
            board.rgba[3] = 0
            lab.advance(.2)
            self.assertEqual(lab.alignment_status, "waiting")
            self.assertFalse(self.sim.velocity_command.any())
            self.assertIsNone(lab.camera.failure(float(self.sim.data.time)))
        finally:
            board.rgba[3] = alpha

    def test_old_queued_frames_cannot_survive_stop_reset_or_mode_change(self):
        lab = self.lab()
        lab.advance(.4)
        self.assertTrue(lab.camera.pending)
        lab.stop()
        self.assertFalse(lab.camera.pending)
        self.assertIsNone(lab.camera.latest)
        lab.advance(.05)
        lab.reset(start_automatically=False)
        self.assertFalse(lab.camera.pending)
        self.assertIsNone(lab.camera.latest)
        self.assertEqual(lab.camera.next_capture_s, 0)
        lab.align(); lab.advance(.3)
        q, t = self.sim.data.qpos.copy(), self.sim.data.time
        lab.toggle_perception()
        self.assertEqual(lab.perception_mode, "natural")
        self.assertFalse(lab.camera.pending)
        self.assertIsNone(lab.camera.latest)
        self.assertFalse(self.sim.velocity_command.any())
        np.testing.assert_array_equal(self.sim.data.qpos, q)
        self.assertEqual(self.sim.data.time, t)

    def test_pause_jog_gain_and_delay_buttons_cancel_pending_motion(self):
        lab = self.lab()
        for action in (lab.toggle_pause, lambda: lab.jog(0, 1), lab.toggle_gain, lab.cycle_camera_delay):
            lab.align(); lab.advance(.3)
            action()
            self.assertFalse(lab.aligning)
            self.assertFalse(lab.camera.pending)
            lab.stop()
            self.assertFalse(self.sim.velocity_command.any())
        lab.align();lab.advance(.3);lab.draw()
        rect = next(rect for rect, name in lab.buttons if name == "camera_delay")
        lab.on_mouse(cv2.EVENT_LBUTTONDOWN, rect[0]+3, rect[1]+3, 0, None)
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())

    def test_repeated_display_frames_cannot_confirm_convergence(self):
        lab = Lab(self.sim, camera_timing=load_camera_timing(.1))
        lab.advance(.25)
        count = lab.camera.delivered_frames
        with patch.object(lab.perception, "observe", side_effect=AssertionError("No new observation")):
            for _ in range(12):
                lab.draw()
        self.assertEqual(lab.camera.delivered_frames, count)
        self.assertNotEqual(lab.alignment_status, "converged")
        lab.toggle_camera_stream()
        self.run_until_stopped(lab, 20)
        self.assertEqual(lab.alignment_status, "stale_camera")

    def test_control_loop_has_a_deadline_even_with_fresh_images(self):
        lab = self.lab(max_run_s=.4)
        self.run_until_stopped(lab, 20)
        self.assertEqual(lab.alignment_status, "camera_run_timeout")
        self.assertFalse(self.sim.velocity_command.any())

    def test_inference_exception_brakes_and_is_reported(self):
        lab = self.lab()
        lab.advance(.4)
        with patch.object(lab.perception, "observe", side_effect=RuntimeError("deliberate camera fault")), \
             patch("traceback.print_exc"):
            lab.advance(.05)
        self.assertEqual(lab.alignment_status, "camera_error")
        self.assertFalse(self.sim.velocity_command.any())
        self.assertIn("deliberate camera fault", lab.message)
        self.assertFalse(lab.camera.stream_enabled)

    def test_failed_inference_result_is_not_treated_as_search_permission(self):
        lab = self.lab()
        lab.advance(.4)
        with patch.object(lab.perception, "observe", return_value=Observation(reason="inference_failed")):
            lab.advance(.2)
        self.assertEqual(lab.alignment_status, "camera_error")
        self.assertFalse(self.sim.velocity_command.any())

    def test_saved_image_metadata_uses_its_capture_state(self):
        lab = self.lab()
        lab.advance(.5)
        lab.draw()
        frame = lab.camera.latest
        pixels = lab.last_rgb.copy()
        lab.advance(.1)
        with tempfile.TemporaryDirectory() as name, patch("app.ROOT", Path(name)):
            lab.save()
            capture = next((Path(name)/"captures").iterdir())
            metadata = json.loads((capture/"camera.json").read_text())
            actual = cv2.cvtColor(cv2.imread(str(capture/"wrist.png")), cv2.COLOR_BGR2RGB)
            np.testing.assert_array_equal(actual, pixels)
            self.assertEqual(metadata["simulation_time_s"], frame.captured_s)
            self.assertEqual(metadata["frame_id"], frame.sequence)
            self.assertEqual(metadata["frame_source"], "delivered_camera")
            self.assertEqual(metadata["saved_simulation_time_s"], self.sim.data.time)
            np.testing.assert_array_equal(metadata["qpos_rad"], frame.qpos)
            np.testing.assert_array_equal(metadata["world_from_optical"], frame.world_from_optical)

    def test_unannounced_clock_reset_stops_old_feedback(self):
        for running in (True, False):
            with self.subTest(running=running):
                self.sim.reset()
                lab = self.lab()
                lab.advance(.5)
                if not running:
                    lab.stop()
                    lab.advance(.05)
                self.assertTrue(lab.camera.pending)
                self.sim.reset()
                lab.advance(.002)
                self.assertEqual(lab.alignment_status, "camera_clock_reset")
                self.assertFalse(self.sim.velocity_command.any())
                self.assertFalse(lab.camera.pending)
                self.assertIsNone(lab.camera.latest)
                self.assertIsNone(lab.last_observation)
                self.assertIsNone(lab.controller.last_visible_qpos)
                self.assertEqual(lab.last_camera_failure["status"], "camera_clock_reset")
                lab.advance(.3)
                self.assertFalse(lab.aligning)
                self.assertFalse(self.sim.velocity_command.any())
                lab.align()
                lab.advance(.3)
                self.assertTrue(lab.aligning)

    def test_controlled_delays_converge_and_hold_on_independent_current_images(self):
        for delay in (0, .05, .1, .2):
            with self.subTest(delay=delay):
                self.sim.reset()
                lab = self.lab(delay)
                self.run_until_stopped(lab)
                self.assertEqual(lab.alignment_status, "converged")
                self.assertGreaterEqual(lab.camera.latest.captured_s-lab._camera_below_since, .5-1e-9)
                for _ in range(30):
                    lab.advance(1/30)
                    corners = self.sim.marker_corners(self.sim.image())
                    self.assertIsNotNone(corners)
                    error = np.sqrt(np.mean(np.sum((corners-lab.reference)**2, axis=1)))
                    self.assertLess(error, 1)
                    self.assertFalse(self.sim.velocity_command.any())


if __name__ == "__main__":
    unittest.main()
