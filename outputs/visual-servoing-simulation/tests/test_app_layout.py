"""Lab window layout at every UI scale: controls reachable, nothing overlapping, status matching the run."""
import unittest

from app import Lab, fit_scale
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
        for scale in (1.0, 1.25, 1.5, 2.0, 3.0):
            with self.subTest(scale=scale):
                lab = Lab(self.sim, auto_start=False, camera_timing=load_camera_timing(.1), ui_scale=scale)
                canvas = lab.draw()
                self.assertEqual(canvas.shape[:2], (round(830 * scale), round(1200 * scale)))
                actions = {action.split(":")[0] for _, action in lab.buttons}
                self.assertTrue(ACTIONS <= actions, ACTIONS - actions)
                self.assertEqual(sum(action.startswith("jog:") for _, action in lab.buttons), 12)
                self.assert_no_overlap(lab)

    def test_scaled_window_clicks_reach_the_same_actions(self):
        lab = Lab(self.sim, auto_start=False, ui_scale=1.5)
        lab.offset()
        lab.align()
        for _ in range(10):
            lab.advance(1 / 30)
        self.assertTrue(self.sim.velocity_command.any())
        self.click(lab, "stop")
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())
        self.click(lab, "auto")
        self.assertTrue(lab.auto_enabled)

    def test_any_window_shape_keeps_the_layout_proportions(self):
        lab = Lab(self.sim, auto_start=False)
        for width, height in ((1075, 1240), (2560, 900), (1200, 830), (1800, 1245)):
            with self.subTest(window=(width, height)):
                frame = lab.frame_for_window(width, height)
                self.assertEqual(frame.shape[:2], (height, width))
                scale = min(width / 1200, height / 830)
                self.assertAlmostEqual(lab.ui_scale, scale, delta=0.011)
                drawn_w, drawn_h = round(1200 * lab.ui_scale), round(830 * lab.ui_scale)
                self.assertEqual(lab.view_offset, ((width - drawn_w) // 2, (height - drawn_h) // 2))

    def test_clicks_in_a_letterboxed_window_reach_the_right_button(self):
        lab = Lab(self.sim, auto_start=False)
        lab.offset()
        lab.align()
        for _ in range(10):
            lab.advance(1 / 30)
        lab.frame_for_window(1075, 1240)  # tall window: bars above and below the layout
        (x, y, w, h), _ = next(item for item in lab.buttons if item[1] == "stop")
        ox, oy = lab.view_offset
        self.assertGreater(oy, 0)
        lab.on_mouse(1, ox + x + w // 2, oy + y + h // 2, 0, None)
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())

    def test_auto_scale_fits_the_work_area(self):
        self.assertEqual(fit_scale(2560, 1552), 1.75)   # 2560 x 1600 screen, taskbar excluded
        self.assertEqual(fit_scale(1920, 1032), 1.1)
        self.assertEqual(fit_scale(1366, 728), 1.0)     # never below the original size
        self.assertEqual(fit_scale(7680, 4320), 3.0)    # capped
        for width, height in ((2560, 1552), (1920, 1032)):
            scale = fit_scale(width, height)
            self.assertLessEqual(1200 * scale, width - 40)
            self.assertLessEqual(830 * scale, height - 80)

    def test_scale_outside_the_supported_range_is_rejected(self):
        for scale in (0.2, 5.0):
            with self.assertRaises(ValueError):
                Lab(self.sim, auto_start=False, ui_scale=scale)

    def test_picture_controls_are_inert_for_aruco(self):
        lab = Lab(self.sim, auto_start=False)
        lab.draw()
        actions = {action for _, action in lab.buttons}
        self.assertNotIn("matcher", actions)
        self.assertNotIn("matches", actions)
        self.assertNotIn("camera_stream", actions)  # no simulated delay, so no stream to interrupt
        self.assert_no_overlap(lab)

    def test_picture_mode_controls_fit(self):
        for scale in (1.0, 1.5):
            with self.subTest(scale=scale):
                lab = Lab(self.sim, auto_start=False, perception_mode="natural", ui_scale=scale)
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
