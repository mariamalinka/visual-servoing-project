"""Natural mode uses real images, the same motion guards and independent references."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from app import Lab
from simulation import Simulation
from reference_image import DEFAULT_REFERENCE,load_reference,save_reference
from perception import NaturalImagePerception,NATURAL_REFERENCE


class NaturalIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.set_target_mode("aruco")
        self.sim.reset()

    def finish(self,lab,limit=1300):
        states=set()
        for _ in range(limit):
            lab.advance(1/30)
            states.add(lab.alignment_status)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status,"converged")
        for _ in range(30):
            lab.advance(1/30)
            corners=lab.detect_target(self.sim.image())
            self.assertIsNotNone(corners)
            self.assertLess(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))),1)
            self.assertFalse(self.sim.velocity_command.any())
        return states

    def test_fixed_and_adaptive_picture_alignment_stay_aligned_after_stop(self):
        for gain in ("fixed","adaptive"):
            with self.subTest(gain=gain):
                self.sim.reset()
                lab=Lab(self.sim,perception_mode="natural")
                if gain=="adaptive": lab.toggle_gain()
                lab.offset()
                self.finish(lab)

    def test_cold_picture_start_searches_without_a_live_home_observation(self):
        sim=self.sim
        sim.reset([35,0,0,0,0,0])
        initial=sim.data.qpos.copy()
        with patch.object(sim,"image",side_effect=AssertionError("A cold start must not render home")):
            lab=Lab(sim,perception_mode="natural",cold_start=True)
        np.testing.assert_array_equal(sim.data.qpos,initial)
        self.assertIsNone(lab.controller.last_visible_qpos)
        self.assertIsNone(lab.detect_target(sim.image()))
        states=self.finish(lab,limit=2200)
        self.assertIn("scanning",states)
        self.assertIn("confirming",states)
        self.assertIsNotNone(lab.controller.startup.acquired_s)

    def test_switching_modes_stops_command_preserves_pose_and_clears_memory(self):
        sim=self.sim
        lab=Lab(sim)
        lab.offset()
        lab.advance(1/30)
        q,time=sim.data.qpos.copy(),sim.data.time
        marker_reference=DEFAULT_REFERENCE.read_bytes()
        for mode in ("natural","aruco"):
            lab.toggle_perception()
            self.assertEqual(lab.perception_mode,mode)
            self.assertEqual(sim.target_mode,mode)
            np.testing.assert_array_equal(sim.data.qpos,q)
            self.assertEqual(sim.data.time,time)
            self.assertFalse(sim.velocity_command.any())
            self.assertIsNone(lab.controller.last_visible_qpos)
            self.assertEqual(lab.alignment_status,"checking")
        self.assertEqual(DEFAULT_REFERENCE.read_bytes(),marker_reference)

    def test_failed_mode_switch_preserves_material_and_reference_and_stops(self):
        sim=self.sim
        lab=Lab(sim)
        lab.offset()
        lab.advance(1/30)
        before=lab.reference.copy()
        q=sim.data.qpos.copy()
        with tempfile.TemporaryDirectory() as folder:
            lab.reference_paths["natural"]=Path(folder)/"missing.npz"
            lab.toggle_perception()
        self.assertFalse(lab.aligning)
        self.assertFalse(sim.velocity_command.any())
        self.assertEqual(sim.target_mode,"aruco")
        np.testing.assert_array_equal(lab.reference,before)
        np.testing.assert_array_equal(sim.data.qpos,q)

    def test_natural_mode_never_uses_the_aruco_detector(self):
        with patch.object(self.sim, "marker_corners", side_effect=AssertionError("ArUco must not supply picture features")):
            lab=Lab(self.sim,perception_mode="natural")
            lab.offset()
            lab.advance(1/30)
            self.assertIsNotNone(lab.last_observation.corners)
            self.assertTrue(self.sim.velocity_command.any())

    def test_wrong_and_absent_targets_cannot_supply_natural_measurements(self):
        sim=self.sim
        detector=NaturalImagePerception()
        self.assertIsNone(detector.observe(sim.image()).corners)
        sim.set_target_mode("natural")
        self.assertIsNotNone(detector.observe(sim.image()).corners)
        board=sim.model.geom("target_board")
        rgba=board.rgba.copy()
        try:
            board.rgba[3]=0
            self.assertIsNone(detector.observe(sim.image()).corners)
        finally:
            board.rgba[:]=rgba

    def test_actual_tracking_loss_brakes_before_recovery_and_reacquires(self):
        sim=self.sim
        lab=Lab(sim,perception_mode="natural")
        lab.offset()
        lab.advance(1/30)
        self.assertTrue(sim.velocity_command.any())
        board=sim.model.geom("target_board")
        rgba=board.rgba.copy()
        try:
            board.rgba[3]=0
            lab.advance(1/30)
            self.assertIsNone(lab.last_observation.corners)
            self.assertEqual(lab.alignment_status,"waiting")
            self.assertFalse(sim.velocity_command.any())
        finally:
            board.rgba[:]=rgba
        self.finish(lab)

    def test_match_display_cannot_advance_confirmation_or_motion(self):
        sim=self.sim
        lab=Lab(sim,perception_mode="natural",cold_start=True)
        lab.advance(1/30)
        search=lab.controller.startup
        count=search.confirmed_frames
        command=sim.velocity_command.copy()
        q=sim.data.qpos.copy()
        lab.toggle_matches()
        lab.draw()
        lab.draw()
        self.assertEqual(search.confirmed_frames,count)
        np.testing.assert_array_equal(sim.velocity_command,command)
        np.testing.assert_array_equal(sim.data.qpos,q)

    def test_natural_teaching_is_atomic_and_reference_metadata_is_checked(self):
        sim=self.sim
        lab=Lab(sim,perception_mode="natural",auto_start=False)
        config=lab.perception.reference_config(sim.config)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"goal.npz"
            sim.reset([2,-1,2,1,-1,1])
            rgb=sim.image()
            expected=save_reference(path,rgb,sim.camera_intrinsics(),config,lab.detect_target)
            before=path.read_bytes()
            with self.assertRaises(ValueError):
                save_reference(path,np.zeros_like(rgb),sim.camera_intrinsics(),config,lab.detect_target)
            self.assertEqual(path.read_bytes(),before)
            _,actual=load_reference(path,sim.camera_intrinsics(),config,lab.detect_target)
            np.testing.assert_array_equal(actual,expected)
            with self.assertRaises(ValueError):
                load_reference(path,sim.camera_intrinsics(),dict(config,target_image_sha256="0"*64),lab.detect_target)

    def test_stop_pause_jog_and_gain_cancel_natural_search(self):
        sim=self.sim
        for action in ("stop","pause","jog","gain"):
            sim.reset([35,0,0,0,0,0])
            lab=Lab(sim,perception_mode="natural",cold_start=True)
            lab.advance(1/30)
            self.assertTrue(sim.velocity_command.any())
            if action=="stop": lab.stop()
            elif action=="pause": lab.toggle_pause()
            elif action=="jog": lab.jog(3,1)
            else: lab.toggle_gain()
            for _ in range(35): lab.advance(1/30)
            self.assertFalse(lab.aligning)
            self.assertFalse(sim.velocity_command.any())
            self.assertEqual(lab.controller.status,"canceled")


if __name__=="__main__":
    unittest.main()
