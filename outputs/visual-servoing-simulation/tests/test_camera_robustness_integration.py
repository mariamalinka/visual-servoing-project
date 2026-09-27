"""Exercise experiment metrics through the real desktop control loop."""
import json
from pathlib import Path
import tempfile
import unittest
from camera_robustness import ROOT, make_plan
from run_camera_robustness import run_trial
from simulation import Simulation


class RobustnessIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def trial(self, profile, **timing):
        config = json.loads((ROOT/"robustness_config.json").read_text())
        plan = make_plan(config, ["aruco"], starts=1, profiles=[profile])
        spec = plan["trials"][0]
        spec["offset_degrees"] = [3,-3,4,3,-2,2]
        spec["timing"].update(timing)
        with tempfile.TemporaryDirectory() as name:
            return run_trial(self.sim, spec, 30, Path(name))

    def test_baseline_measures_stable_alignment_and_joint_motion(self):
        row = self.trial("baseline")
        self.assertTrue(row["expected_pass"], row)
        self.assertEqual(row["outcome"], "converged")
        self.assertGreater(row["joint_travel_rad"], 0)
        self.assertLess(row["max_post_stop_error_px"], 1)
        self.assertEqual(row["post_stop_frames"], 30)
        self.assertFalse(row["safety_violations"])

    def test_outage_pass_requires_deadline_stop_and_restored_capture_without_restart(self):
        row = self.trial("long-outage")
        self.assertEqual(row["outcome"], "stale_camera")
        self.assertTrue(row["expected_pass"], row)
        self.assertTrue(row["capture_restored"])
        self.assertAlmostEqual(row["stopped_capture_age_s"], .25)
        self.assertTrue(row["commands_zero_after_stop"])

    def test_total_packet_loss_is_alignment_failure_but_safe_stop(self):
        row = self.trial("drops", drop_probability=1)
        self.assertEqual(row["outcome"], "stale_camera")
        self.assertFalse(row["expected_pass"])
        self.assertFalse(row["safety_violations"])
        self.assertEqual(row["camera_counts"]["delivered_frames"], 0)
        self.assertTrue(row["commands_zero_after_stop"])

