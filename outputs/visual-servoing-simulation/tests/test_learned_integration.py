"""Real learned perception drives the existing controller and app handlers."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from app import Lab
from simulation import Simulation
from reference_image import load_reference
from perception import NaturalImagePerception,NATURAL_REFERENCE
from learned_perception import LearnedUnavailable
from test_learned_perception import AVAILABLE

@unittest.skipUnless(AVAILABLE,"Optional learned dependencies/models are not installed")
class LearnedIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()
    @classmethod
    def tearDownClass(cls):
        cls.sim.close()
    def setUp(self):
        self.sim.set_target_mode("aruco")
        self.sim.reset()

    def finish(self,lab,limit=2200):
        states=set()
        for _ in range(limit):
            lab.advance(1/30);states.add(lab.alignment_status)
            if not lab.aligning:break
        self.assertEqual(lab.alignment_status,"converged")
        for _ in range(30):
            lab.advance(1/30)
            c=lab.detect_target(self.sim.image())
            self.assertIsNotNone(c)
            self.assertLess(np.sqrt(np.mean(np.sum((c-lab.reference)**2,axis=1))),1)
            self.assertFalse(self.sim.velocity_command.any())
        return states

    def test_offset_aligns_without_any_aruco_or_sift_fallback(self):
        with patch.object(self.sim,"marker_corners",side_effect=AssertionError("No ArUco fallback")), \
             patch("cv2.SIFT_create",side_effect=AssertionError("No SIFT fallback")):
            lab=Lab(self.sim,perception_mode="learned")
            lab.offset()
            self.finish(lab)

    def test_cold_start_uses_only_current_images_and_starts_bounded_search(self):
        sim=self.sim;sim.reset([35,0,0,0,0,0])
        with patch.object(sim,"image",side_effect=AssertionError("No live home image")):
            lab=Lab(sim,perception_mode="learned",cold_start=True)
        self.assertIsNone(lab.controller.last_visible_qpos)
        self.assertIsNone(lab.detect_target(sim.image()))
        for _ in range(5):
            lab.advance(1/30)
            self.assertTrue(lab.aligning)
            self.assertEqual(lab.alignment_status,"scanning")
            self.assertLessEqual(np.max(np.abs(sim.velocity_command)),.25)
        lab.stop()
        self.assertFalse(sim.velocity_command.any())
        # Full unseen-start acquisition/convergence is recorded in the paired
        # learned study; keep the unit check focused on startup and cancellation.

    def test_switching_matchers_keeps_pose_goal_and_gain_but_stops_old_command(self):
        lab=Lab(self.sim,perception_mode="natural")
        lab.toggle_gain();lab.offset();lab.advance(1/30)
        q,t=self.sim.data.qpos.copy(),self.sim.data.time
        reference=NATURAL_REFERENCE.read_bytes()
        lab.toggle_matcher()
        self.assertEqual(lab.perception_mode,"learned")
        self.assertEqual(lab.gain_mode,"adaptive")
        self.assertEqual(lab.reference_rgb.tobytes(),np.load(NATURAL_REFERENCE)["rgb"].tobytes())
        self.assertIsNone(lab.controller.last_visible_qpos)
        self.assertFalse(self.sim.velocity_command.any())
        np.testing.assert_array_equal(q,self.sim.data.qpos)
        self.assertEqual(t,self.sim.data.time)
        lab.toggle_perception();self.assertEqual(lab.perception_mode,"aruco")
        lab.toggle_perception();self.assertEqual(lab.perception_mode,"learned")
        lab.toggle_matcher();self.assertEqual(lab.perception_mode,"natural")
        self.assertEqual(NATURAL_REFERENCE.read_bytes(),reference)

    def test_unavailable_matcher_leaves_existing_mode_stopped(self):
        lab=Lab(self.sim,perception_mode="natural")
        lab.offset();lab.advance(1/30)
        reference=lab.reference.copy()
        with patch("app.make_perception",side_effect=LearnedUnavailable("Run setup-learned.cmd")):
            lab.toggle_matcher()
        self.assertEqual(lab.perception_mode,"natural")
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())
        np.testing.assert_array_equal(lab.reference,reference)
        self.assertIn("setup-learned.cmd",lab.message)

    def test_loss_brakes_and_recovers_on_real_restored_images(self):
        lab=Lab(self.sim,perception_mode="learned")
        lab.offset();lab.advance(1/30)
        board=self.sim.model.geom("target_board");alpha=float(board.rgba[3])
        try:
            board.rgba[3]=0
            lab.advance(1/30)
            self.assertEqual(lab.alignment_status,"waiting")
            self.assertFalse(self.sim.velocity_command.any())
        finally:
            board.rgba[3]=alpha
        self.finish(lab)

    def test_drawing_matches_never_advances_search_confirmation(self):
        lab=Lab(self.sim,perception_mode="learned",cold_start=True)
        lab.toggle_matches()
        q,t=self.sim.data.qpos.copy(),self.sim.data.time
        for _ in range(4):lab.draw()
        self.assertEqual(lab.alignment_status,"checking")
        np.testing.assert_array_equal(q,self.sim.data.qpos)
        self.assertEqual(t,self.sim.data.time)

    def test_teaching_preserves_one_shared_picture_goal(self):
        with tempfile.TemporaryDirectory() as folder:
            goal=Path(folder)/"goal.npz";goal.write_bytes(NATURAL_REFERENCE.read_bytes())
            lab=Lab(self.sim,perception_mode="learned",reference_path=goal,auto_start=False)
            lab.teach_reference()
            self.assertEqual(lab.reference_paths["natural"],goal)
            sift=NaturalImagePerception()
            rgb,_=load_reference(goal,self.sim.camera_intrinsics(),sift.reference_config(self.sim.config),
                                 lambda frame:sift.observe(frame).corners)
            np.testing.assert_array_equal(rgb,lab.reference_rgb)
            lab.toggle_matcher()
            self.assertEqual(lab.reference_path,goal)

    def test_manual_interruptions_cancel_learned_automatic_motion(self):
        lab=Lab(self.sim,perception_mode="learned")
        for action in (lab.stop,lab.toggle_pause,lambda:lab.jog(0,1),lab.toggle_gain):
            lab.cold_start();lab.advance(1/30)
            action()
            self.assertFalse(lab.aligning)
            lab.stop()
            self.assertFalse(self.sim.velocity_command.any())

if __name__=="__main__":
    unittest.main()
