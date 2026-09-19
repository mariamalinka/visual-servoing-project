"""Regression checks for marker detection and cursor-independent alignment."""
from __future__ import annotations

import json
import unittest
from pathlib import Path

import cv2
import numpy as np

from app import Lab
from control import estimate_depths
from simulation import Simulation

FIXTURES = Path(__file__).parent / "fixtures"


class AlignmentRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()
        cls.starts = json.loads((FIXTURES / "regression-starts.json").read_text())

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def test_previous_failure_images_decode_the_black_marker(self):
        # Independently measured black-border corner coordinates, not board edges.
        expected = {
            "marker-lost-during-motion.png": [[276, 96], [513, 73], [528, 309], [296, 326]],
            "marker-not-detected-at-start.png": [[82, 127], [333, 116], [351, 351], [105, 379]],
        }
        for name, target in expected.items():
            with self.subTest(image=name):
                bgr = cv2.imread(str(FIXTURES / name))
                corners = self.sim.marker_corners(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                self.assertIsNotNone(corners)
                np.testing.assert_allclose(corners, target, atol=1)
                depths = estimate_depths(corners, self.sim.camera_intrinsics(), .24, 3)
                self.assertTrue(np.all(depths > .03))

    def test_blank_wrong_id_and_occluded_images_are_rejected(self):
        blank = np.full((480, 640, 3), 255, dtype=np.uint8)
        self.assertIsNone(self.sim.marker_corners(blank))
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        wrong = blank.copy()
        wrong[100:340, 200:440] = cv2.aruco.generateImageMarker(dictionary, 8, 240)[..., None]
        self.assertIsNone(self.sim.marker_corners(wrong))
        occluded = cv2.cvtColor(cv2.imread(str(FIXTURES / "marker-lost-during-motion.png")),
                               cv2.COLOR_BGR2RGB)
        occluded[:, 390:] = 255
        self.assertIsNone(self.sim.marker_corners(occluded))

    def run_alignment(self, offset, pointer_mode):
        self.sim.reset()
        lab = Lab(self.sim, auto_start=False)  # Teach the same home reference for every case.
        self.sim.reset(offset)
        lab.draw()
        rect = next(rect for rect, action in lab.buttons if action == "align")
        lab.on_mouse(cv2.EVENT_LBUTTONDOWN, rect[0] + 10, rect[1] + 10, 0, None)
        states = []
        for i in range(600):
            if pointer_mode == "picture":
                lab.on_mouse(cv2.EVENT_MOUSEMOVE, 900 + i % 20, 300, 0, None)
            elif pointer_mode == "toolbar":
                # Moving across Align/Reset must not invoke those buttons.
                lab.on_mouse(cv2.EVENT_MOUSEMOVE, 700 + i % 180, 40, 0, None)
            elif pointer_mode == "outside":
                lab.on_mouse(cv2.EVENT_MOUSEMOVE, -20, -20, 0, None)
            # In "no_events", no further mouse or key input is sent at all.
            lab.advance(1 / 30)
            states.append(self.sim.data.qpos.copy())
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status, "converged", pointer_mode)
        self.assertTrue(np.all(self.sim.velocity_command == 0))
        self.sim.advance(1)
        corners = self.sim.marker_corners(self.sim.image())
        self.assertIsNotNone(corners)
        error = np.sqrt(np.mean(np.sum((corners - lab.reference) ** 2, axis=1)))
        self.assertLess(error, 1)
        return np.asarray(states)

    def test_failed_motion_now_aligns_with_and_without_pointer_events(self):
        offset = self.starts[0]["offset_degrees"]
        reference = self.run_alignment(offset, "no_events")
        for mode in ("picture", "toolbar", "outside"):
            with self.subTest(pointer=mode):
                actual = self.run_alignment(offset, mode)
                np.testing.assert_array_equal(actual, reference)

    def test_previously_undetected_start_now_aligns(self):
        self.run_alignment(self.starts[1]["offset_degrees"], "no_events")


if __name__ == "__main__":
    unittest.main()

