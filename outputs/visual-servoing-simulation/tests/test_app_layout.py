"""Lab window layout: every control is reachable, nothing overlaps, and the status reflects the run."""
import unittest

from app import Lab
from camera_timing import load_camera_timing
from simulation import Simulation

ACTIONS = {"align", "pause", "reset", "stop", "perception", "camera_stream", "offset", "cold_start",
           "random_start", "lost_view", "teach", "auto", "gain", "camera_delay", "obstacle", "calibration", "demo"}


class AppLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def click(self, lab, action):
        lab.draw()
        (x, y, w, h), _ = next(item for item in lab.buttons if item[1] == action)
        lab.on_mouse(1, x + w // 2, y + h // 2, 0, None)

    def assert_no_overlap(self, lab):
        canvas = lab.draw()
        rects = list(lab.drawn)
        self.assertTrue({rect for rect, _ in lab.buttons} <= set(rects))
        for i, (x, y, w, h) in enumerate(rects):
            self.assertTrue(0 <= x and 0 <= y and x + w <= canvas.shape[1] and y + h <= canvas.shape[0])
            for (a, b, c, d) in rects[i + 1:]:
                overlap = x < a + c and a < x + w and y < b + d and b < y + h
                self.assertFalse(overlap, f"controls overlap: {(x, y, w, h)} and {(a, b, c, d)}")

    def test_every_control_has_a_click_target_and_none_overlap(self):
        lab = Lab(self.sim, auto_start=False, camera_timing=load_camera_timing(.1))
        lab.draw()
        actions = {action.split(":")[0] for _, action in lab.buttons}
        self.assertTrue(ACTIONS <= actions, ACTIONS - actions)
        self.assertEqual(sum(action.startswith("jog:") for _, action in lab.buttons), 12)
        self.assert_no_overlap(lab)

    def test_picture_controls_are_inert_for_aruco(self):
        lab = Lab(self.sim, auto_start=False)
        lab.draw()
        actions = {action for _, action in lab.buttons}
        self.assertNotIn("matcher", actions)
        self.assertNotIn("matches", actions)
        self.assertNotIn("camera_stream", actions)  # no simulated delay, so no stream to interrupt
        self.assert_no_overlap(lab)

    def test_picture_mode_controls_fit(self):
        lab = Lab(self.sim, auto_start=False, perception_mode="natural")
        lab.toggle_matches()
        lab.draw()
        actions = {action for _, action in lab.buttons}
        self.assertTrue({"matcher", "matches"} <= actions)
        self.assert_no_overlap(lab)

    def test_selected_segment_is_not_clickable(self):
        lab = Lab(self.sim, auto_start=False)
        lab.draw()
        self.assertEqual(sum(action == "auto" for _, action in lab.buttons), 1)
        self.click(lab, "auto")
        self.assertTrue(lab.auto_enabled)

    def test_stop_button_zeroes_motion(self):
        lab = Lab(self.sim, auto_start=False)
        lab.offset()
        lab.align()
        for _ in range(10):
            lab.advance(1 / 30)
        self.assertTrue(self.sim.velocity_command.any())
        self.click(lab, "stop")
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())

    def test_stream_button_interrupts_and_restores_the_delayed_camera(self):
        lab = Lab(self.sim, auto_start=False, camera_timing=load_camera_timing(.1))
        self.click(lab, "camera_stream")
        self.assertFalse(lab.camera.stream_enabled)
        self.click(lab, "camera_stream")
        self.assertTrue(lab.camera.stream_enabled)

    def test_status_reports_a_stale_camera_stop(self):
        lab = Lab(self.sim, auto_start=False, camera_timing=load_camera_timing(.1))
        lab.offset()
        lab.align()
        for _ in range(30):
            lab.advance(1 / 30)
        lab.toggle_camera_stream()
        for _ in range(40):
            lab.advance(1 / 30)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status, "stale_camera")
        state, _ = lab.status_state()
        self.assertEqual(state, "STOPPED - stale camera")
        self.assert_no_overlap(lab)


if __name__ == "__main__":
    unittest.main()
