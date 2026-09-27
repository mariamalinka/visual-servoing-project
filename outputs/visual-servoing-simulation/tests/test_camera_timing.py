"""Camera timing contracts independent of the detector and controller."""
import unittest
import numpy as np
from camera_timing import TimedCamera, load_camera_timing
from perception import Observation


class CameraTimingTests(unittest.TestCase):
    def camera(self, **changes):
        return TimedCamera(dict(load_camera_timing(), **changes), 30)

    def submit(self, camera, now, q=0):
        return camera.submit(now, np.zeros((4, 5, 3), np.uint8), np.full(6, q),
                             Observation(corners=np.ones((4, 2))), 12)

    def test_frame_is_delivered_once_at_its_scheduled_time(self):
        camera = self.camera(delay_s=.1)
        captured = self.submit(camera, 0, q=2)
        self.assertIsNone(camera.receive(.099))
        self.assertIs(camera.receive(.1), captured)
        self.assertIsNone(camera.receive(.1))
        self.assertIsNone(camera.receive(.2))
        self.assertEqual(camera.delivered_frames, 1)
        self.assertAlmostEqual(camera.age_s(.2), .2)

    def test_zero_delay_is_available_in_the_capture_tick(self):
        camera = self.camera(delay_s=0)
        self.submit(camera, .2)
        self.assertIsNotNone(camera.receive(.2))

    def test_payload_keeps_capture_pixels_joints_and_measurements(self):
        camera = self.camera()
        rgb = np.zeros((4, 5, 3), np.uint8)
        q = np.ones(6)
        observation = Observation(corners=np.ones((4, 2)))
        frame = camera.submit(0, rgb, q, observation)
        rgb[:] = 255; q[:] = 10; observation.corners[:] = 20
        self.assertEqual(frame.rgb.max(), 0)
        np.testing.assert_array_equal(frame.qpos, np.ones(6))
        np.testing.assert_array_equal(frame.observation.corners, np.ones((4, 2)))

    def test_newest_due_frame_wins_after_a_poll_gap(self):
        camera = self.camera(delay_s=.02)
        for i in range(4):
            self.submit(camera, i / 30)
        received = camera.receive(.15)
        self.assertEqual(received.sequence, 4)
        self.assertEqual(camera.dropped_frames, 3)
        self.assertIsNone(camera.receive(.16))

    def test_queue_is_bounded_and_discards_oldest_pending_frame(self):
        camera = self.camera(delay_s=.2, max_pending_frames=2)
        for i in range(4):
            self.submit(camera, i / 30)
        self.assertEqual(len(camera.pending), 2)
        self.assertEqual(camera.dropped_frames, 2)
        self.assertEqual(camera.pending[0].sequence, 3)

    def test_expired_frame_cannot_refresh_the_watchdog(self):
        camera = self.camera(delay_s=.3)
        self.submit(camera, 0)
        self.assertEqual(camera.failure(.25), "stale_camera")
        self.assertIsNone(camera.receive(.3))
        self.assertEqual(camera.stale_frames, 1)
        self.assertIsNone(camera.latest)

    def test_watchdog_uses_capture_age_not_arrival_time(self):
        camera = self.camera(delay_s=.2)
        self.submit(camera, 0)
        camera.receive(.2)
        self.assertIsNone(camera.failure(.249))
        self.assertEqual(camera.failure(.25), "stale_camera")

    def test_no_images_and_continuous_images_have_bounded_run_time(self):
        camera = self.camera(max_run_s=.2)
        self.assertEqual(camera.failure(.2), "camera_run_timeout")
        camera = self.camera(max_run_s=.2, delay_s=0)
        for i in range(6):
            self.submit(camera, i / 30); camera.receive(i / 30)
        self.assertEqual(camera.failure(.2), "camera_run_timeout")

    def test_reset_flushes_pending_old_frames_and_deadlines(self):
        camera = self.camera()
        self.submit(camera, .2)
        self.assertEqual(camera.failure(.1), "camera_clock_reset")
        camera.reset(0)
        self.assertIsNone(camera.receive(.3))
        self.assertIsNone(camera.latest)
        self.assertTrue(camera.capture_due(0))

    def test_capture_clock_does_not_replay_frames_after_a_gap(self):
        camera = self.camera()
        self.assertTrue(camera.capture_due(0))
        self.assertFalse(camera.capture_due(0))
        self.assertTrue(camera.capture_due(.2))
        self.assertFalse(camera.capture_due(.2))
        camera.stream_enabled = False
        self.assertFalse(camera.capture_due(.3))

    def test_invalid_settings_and_timestamps_are_rejected(self):
        for change in (dict(delay_s=-1), dict(delay_s=float("nan")),
                       dict(max_observation_age_s=0), dict(max_run_s=float("inf")),
                       dict(max_pending_frames=True), dict(max_pending_frames=257)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.camera(**change)
        camera = self.camera()
        self.submit(camera, 0)
        with self.assertRaises(ValueError):
            self.submit(camera, 0)
        with self.assertRaises(ValueError):
            camera.receive(float("nan"))
