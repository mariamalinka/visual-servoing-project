"""Contracts for paired disturbance plans, delivery ordering, and honest reports."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

import numpy as np
from camera_timing import TimedCamera, load_camera_timing
from camera_robustness import ROOT, make_plan, report, summarize, validate_rows, write_json
from perception import Observation


class CameraDisturbanceTests(unittest.TestCase):
    def camera(self, **settings):
        return TimedCamera(dict(load_camera_timing(), **settings), 30)

    def submit(self, camera, now):
        return camera.submit(now, np.zeros((3, 4, 3), np.uint8), np.zeros(6),
                             Observation(corners=np.ones((4, 2))))

    def test_seeded_loss_and_jitter_replay_after_reset(self):
        camera = self.camera(seed=91, jitter_s=.08, drop_probability=.3)
        def sample():
            frames = [self.submit(camera, i/30) for i in range(80)]
            return [None if f is None else f.available_s-f.captured_s for f in frames]
        first = sample()
        self.assertGreater(camera.lost_frames, 0)
        self.assertLess(camera.lost_frames, 80)
        self.assertTrue(all(.02-1e-9 <= d <= .18+1e-9 for d in first if d is not None))
        camera.reset(0)
        self.assertEqual(first, sample())
        camera.config["seed"] = 92
        camera.reset(0)
        self.assertNotEqual(first, sample())

    def test_newer_frame_can_overtake_old_frame_without_feedback_going_backwards(self):
        camera = self.camera(delay_s=.1, jitter_s=.09)
        camera.rng = Mock()
        camera.rng.random.side_effect = [np.array([.9, 1]), np.array([.9, 0])]
        older = self.submit(camera, 0)
        newer = self.submit(camera, 1/30)
        self.assertGreater(older.available_s, newer.available_s)
        self.assertIs(camera.receive(.05), newer)
        self.assertIsNone(camera.receive(.2))
        self.assertEqual(camera.latest.sequence, 2)
        self.assertEqual(camera.obsolete_frames, 1)
        self.assertGreater(camera.capture_elapsed_s, 0)

    def test_total_packet_loss_never_refreshes_feedback(self):
        camera = self.camera(drop_probability=1)
        for i in range(8):
            self.assertIsNone(self.submit(camera, i/30))
        self.assertFalse(camera.pending)
        self.assertIsNone(camera.receive(.25))
        self.assertEqual(camera.failure(.25), "stale_camera")
        self.assertEqual(camera.lost_frames, camera.sequence)

    def test_jitter_is_clipped_at_zero_delay(self):
        camera = self.camera(delay_s=.01, jitter_s=.1)
        camera.rng = Mock()
        camera.rng.random.return_value = np.array([.9, 0])
        frame = self.submit(camera, 0)
        self.assertEqual(frame.available_s, frame.captured_s)

    def test_outages_use_run_relative_clock_and_restore_capture(self):
        camera = self.camera(outages_s=[[.1, .2]])
        camera.reset(10)
        self.assertTrue(camera.capture_due(10))
        self.assertFalse(camera.capture_due(10.1))
        self.assertFalse(camera.capture_due(10.15))
        self.assertTrue(camera.capture_due(10.2))
        self.assertEqual(camera.outage_slots, 2)

    def test_invalid_disturbances_are_rejected(self):
        for settings in (dict(jitter_s=-1), dict(drop_probability=1.1), dict(seed=-1),
                         dict(seed=True), dict(outages_s=[[.2,.1]]),
                         dict(outages_s=[[.1,.3],[.2,.4]])):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                self.camera(**settings)


class RobustnessPlanTests(unittest.TestCase):
    def config(self):
        return json.loads((ROOT/"robustness_config.json").read_text())

    def test_plan_pairs_poses_and_noise_and_has_repeatable_prefix(self):
        config = self.config()
        plan = make_plan(config, ["aruco","learned"], starts=3, seed=7)
        for case in range(3):
            group = [t for t in plan["trials"] if t["case"] == case]
            self.assertEqual(len({tuple(t["offset_degrees"]) for t in group}), 1)
            self.assertEqual(len({t["timing"]["seed"] for t in group}), 1)
        first = make_plan(config, ["aruco"], starts=1, seed=7, profiles=["baseline"])
        self.assertEqual(first["trials"][0], plan["trials"][0])
        changed = make_plan(config, ["aruco"], starts=1, seed=8)
        self.assertNotEqual(first["trials"][0]["offset_degrees"], changed["trials"][0]["offset_degrees"])

    def test_plan_rejects_misspelled_settings_and_invalid_selection(self):
        config = self.config()
        for options in (dict(starts=0), dict(seed=-1), dict(profiles=["missing"]),
                        dict(profiles=["baseline","baseline"])):
            with self.subTest(options=options), self.assertRaises(ValueError):
                make_plan(config, ["aruco"], **options)
        config["profiles"][0]["drop_probablity"] = .5
        with self.assertRaisesRegex(ValueError, "Unknown camera"):
            make_plan(config, ["aruco"])

    def test_partial_summary_preserves_failures_and_planned_denominator(self):
        plan = make_plan(self.config(), ["aruco"], starts=3, profiles=["baseline"])
        rows = []
        for spec, outcome in zip(plan["trials"], ["converged","stale_camera"]):
            rows.append(dict(**{k:spec[k] for k in ("id","mode","profile","case","expected")},
                             outcome=outcome, expected_pass=outcome=="converged", completion_s=2,
                             final_current_error_px=.5 if outcome=="converged" else 7,
                             safety_violations=[]))
        group = summarize(plan, rows)[0]
        self.assertEqual((group["planned"],group["completed"],group["converged"]), (3,2,1))
        self.assertEqual(group["outcomes"], {"converged":1,"stale_camera":1})
        self.assertEqual(group["median_success_s"], 2)
        with self.assertRaises(ValueError):
            validate_rows(plan, rows+rows[:1])

    def test_report_regenerates_from_json_without_traces(self):
        plan = make_plan(self.config(), ["aruco"], starts=2, profiles=["long-outage"])
        spec = plan["trials"][0]
        rows = [dict(**{k:spec[k] for k in ("id","mode","profile","case","expected")},
                     outcome="stale_camera", expected_pass=True, safety_violations=[])]
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            write_json(directory/"plan.json", plan)
            write_json(directory/"trials.json", rows)
            report(directory)
            summary = json.loads((directory/"summary.json").read_text())[0]
            self.assertEqual(summary["converged"], 0)
            self.assertEqual(summary["expected_outcomes"], 1)
            self.assertEqual(summary["planned"], 2)
            self.assertIn("Completed 1/2", (directory/"REPORT.md").read_text(encoding="utf-8"))
            self.assertTrue((directory/"robustness.png").is_file())

