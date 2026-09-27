"""Real app wiring: refined reference lifecycle and delayed precision hold."""
import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from app import Lab
from simulation import Simulation
from camera_timing import load_camera_timing
from precision import GoalRefinedPerception
from control import ControlSample

class PrecisionIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.sim=Simulation()
    @classmethod
    def tearDownClass(cls):cls.sim.close()
    def setUp(self):
        self.sim.set_obstacle(False);self.sim.reset();self.sim.set_target_mode('aruco')

    def test_default_and_legacy_paths_are_explicit(self):
        lab=Lab(self.sim,auto_start=False)
        self.assertIsInstance(lab.perception,GoalRefinedPerception)
        self.assertIsNotNone(lab.controller.ibvs.precision_stop)
        old=Lab(self.sim,auto_start=False,precision=False)
        self.assertNotIsInstance(old.perception,GoalRefinedPerception)
        self.assertIsNone(old.controller.ibvs.precision_stop)

    def test_gain_calibration_target_changes_keep_precision(self):
        lab=Lab(self.sim,auto_start=False)
        for change in (lab.toggle_gain,lab.cycle_calibration,lambda:lab.select_perception('natural')):
            change()
            self.assertEqual(lab.controller.ibvs.precision_stop,lab.precision_settings['stop'])
            self.assertIsInstance(lab.perception,GoalRefinedPerception)
            np.testing.assert_array_equal(lab.perception.goal,lab.reference)
            self.assertFalse(self.sim.velocity_command.any())

    def test_teach_uses_raw_detector_and_clears_old_reference_and_queue(self):
        lab=Lab(self.sim,auto_start=False,camera_timing=load_camera_timing(.1))
        with tempfile.TemporaryDirectory() as directory:
            lab.reference_path=Path(directory)/'goal.npz'
            lab.offset();lab.align();lab.advance(.4)
            current=self.sim.image();expected=lab.perception.base.observe(current).corners
            self.assertIsNotNone(expected)
            old=lab.perception
            lab.teach_reference()
            self.assertIsNot(old,lab.perception)
            np.testing.assert_array_equal(lab.reference,expected)
            np.testing.assert_array_equal(lab.perception.goal_rgb,current)
            self.assertFalse(lab.camera.pending);self.assertFalse(lab.aligning)
            self.assertFalse(self.sim.velocity_command.any())

    def test_delayed_hold_resets_on_pose_gate_even_below_pixel_threshold(self):
        lab=Lab(self.sim,auto_start=False,camera_timing=load_camera_timing(.1))
        lab.align()
        zero=np.zeros(6)
        eligible=ControlSample('running',.1,zero,zero.copy(),stop_candidate=True)
        ineligible=ControlSample('running',.1,zero,zero.copy(),stop_candidate=False)
        with patch.object(lab.controller,'update',return_value=eligible):lab.advance(.3)
        self.assertIsNotNone(lab._camera_below_since)
        with patch.object(lab.controller,'update',return_value=ineligible):lab.advance(.04)
        self.assertIsNone(lab._camera_below_since)

    def test_draw_does_not_supply_delayed_stop_evidence(self):
        lab=Lab(self.sim,auto_start=False,camera_timing=load_camera_timing(.1))
        lab.align();lab.advance(.15)
        held=lab.controller.ibvs.held_seconds
        with patch.object(lab.perception,'observe',side_effect=AssertionError('display cannot refine')):
            lab.draw();lab.draw()
        self.assertEqual(lab.controller.ibvs.held_seconds,held)

if __name__=='__main__':unittest.main()
