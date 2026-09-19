"""Default Auto mode, startup decisions and persistent user interruption."""
import unittest
from unittest.mock import patch
import numpy as np

from app import Lab, random_start_offset
from simulation import Simulation
from startup_search import load_startup_config


class AutomaticStartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def finish(self,lab,steps=4100):
        states=set()
        for _ in range(steps):
            lab.advance(1/30)
            states.add(lab.alignment_status)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status,"converged")
        self.assertFalse(self.sim.velocity_command.any())
        return states

    def test_visible_start_aligns_without_click_or_reset(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        start=sim.data.qpos.copy()
        lab=Lab(sim)
        self.assertTrue(lab.auto_enabled)
        self.assertTrue(lab.aligning)
        np.testing.assert_array_equal(sim.data.qpos,start)
        states=self.finish(lab,600)
        self.assertIn("running",states)
        self.assertNotIn("scanning",states)
        self.assertIsNone(lab.controller.startup)
        sim.advance(1)
        corners=sim.marker_corners(sim.image())
        self.assertIsNotNone(corners)
        self.assertLess(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))),1)

    def test_unseen_start_searches_and_aligns_without_click_or_home_observation(self):
        sim=self.sim
        sim.reset([35,0,0,0,0,0])
        start=sim.data.qpos.copy()
        actual_image=sim.image
        observed=[]
        def image(camera="wrist"):
            observed.append(sim.data.qpos.copy())
            return actual_image(camera)
        with patch.object(sim,"image",side_effect=image):
            lab=Lab(sim)
        self.assertTrue(lab.aligning)
        self.assertIsNone(lab.controller.last_visible_qpos)
        for joints in observed:
            np.testing.assert_array_equal(joints,start)
        states=self.finish(lab)
        self.assertTrue({"scanning","confirming","running"}<=states)
        self.assertNotIn("returning",states)
        self.assertIsNotNone(lab.controller.startup.acquired_s)

    def test_fresh_visible_start_confirms_without_search_motion(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        with patch.object(sim,"image",side_effect=AssertionError("Premature live image")):
            lab=Lab(sim,cold_start=True)
        self.assertIsNone(lab.controller.last_visible_qpos)
        for _ in range(2):
            lab.advance(1/30)
            self.assertEqual(lab.alignment_status,"confirming")
            self.assertFalse(sim.velocity_command.any())
        lab.advance(1/30)
        self.assertEqual(lab.alignment_status,"running")
        self.assertTrue(sim.velocity_command.any())
        self.assertIsNotNone(lab.controller.last_visible_qpos)

    def test_random_button_clears_memory_and_starts_without_an_align_click(self):
        sim=self.sim
        lab=Lab(sim)
        self.assertIsNotNone(lab.controller.last_visible_qpos)
        lab.draw()
        rect=next(rect for rect,action in lab.buttons if action=="random_start")
        with patch("app.random_start_offset",return_value=[35,0,0,0,0,0]):
            with patch.object(sim,"image",side_effect=AssertionError("Unexpected look before start")):
                lab.on_mouse(1,rect[0]+10,rect[1]+10,0,None)
        self.assertTrue(lab.aligning)
        self.assertIsNone(lab.controller.last_visible_qpos)
        lab.advance(1/30)
        self.assertEqual(lab.alignment_status,"scanning")
        self.assertTrue(sim.velocity_command.any())

    def test_random_starts_are_reproducible_and_do_not_filter_hidden_views(self):
        sim=self.sim
        seen=set()
        for seed in range(12):
            offset=random_start_offset(seed)
            np.testing.assert_array_equal(offset,random_start_offset(seed))
            sim.reset(offset)
            seen.add(sim.marker_corners(sim.image()) is not None)
        self.assertEqual(seen,{False,True})
        with self.assertRaises(ValueError):
            random_start_offset(-1)

    def test_user_interruption_does_not_rearm_from_idle(self):
        sim=self.sim
        lab=Lab(sim)
        for action in ("stop","pause","jog","gain"):
            with self.subTest(action=action):
                lab.cold_start()
                lab.advance(1/30)
                self.assertTrue(sim.velocity_command.any())
                if action=="stop":
                    lab.stop()
                elif action=="pause":
                    lab.toggle_pause()
                    lab.advance(1)
                    lab.toggle_pause()
                elif action=="jog":
                    lab.jog(3,1)
                else:
                    lab.toggle_gain()
                for _ in range(60):
                    lab.advance(1/30)
                self.assertFalse(lab.aligning)
                self.assertFalse(sim.velocity_command.any())
                self.assertTrue(lab.auto_enabled)

    def test_auto_off_keeps_new_poses_manual_and_button_on_restarts(self):
        lab=Lab(self.sim,auto_start=False)
        self.assertFalse(lab.aligning)
        for action in (lab.offset,lab.cold_start,lab.reset,lambda:lab.random_start(seed=5)):
            action()
            lab.advance(1/30)
            self.assertFalse(lab.aligning)
            self.assertFalse(self.sim.velocity_command.any())
        lab.cold_start()
        lab.draw()
        rect=next(rect for rect,action in lab.buttons if action=="auto")
        lab.on_mouse(1,rect[0]+10,rect[1]+10,0,None)
        self.assertTrue(lab.auto_enabled)
        lab.advance(1/30)
        self.assertEqual(lab.alignment_status,"scanning")
        lab.toggle_auto()
        for _ in range(30):
            lab.advance(1/30)
        self.assertFalse(lab.auto_enabled)
        self.assertFalse(lab.aligning)
        self.assertFalse(self.sim.velocity_command.any())

    def test_terminal_success_and_missing_target_do_not_restart_forever(self):
        sim=self.sim
        lab=Lab(sim)
        self.finish(lab,60)
        for _ in range(60):
            lab.advance(1/30)
        self.assertEqual(lab.alignment_status,"converged")
        self.assertFalse(lab.aligning)
        board=sim.model.geom("target_board")
        original=board.rgba.copy()
        try:
            board.rgba[3]=0
            sim.reset()
            lab=Lab(sim)
            cfg=load_startup_config()
            cfg["max_search_time_s"]=.2
            lab.controller.startup_config=cfg
            for _ in range(120):
                lab.advance(1/30)
            self.assertEqual(lab.alignment_status,"target_not_found")
            self.assertFalse(lab.aligning)
            self.assertFalse(sim.velocity_command.any())
            self.assertLessEqual(lab.controller.startup.elapsed_s,.2+1e-9)
        finally:
            board.rgba[:]=original

    def test_start_buttons_run_automatically_but_demo_keeps_scripted_control(self):
        sim=self.sim
        lab=Lab(sim)
        lab.offset()
        self.assertTrue(lab.aligning)
        lab.advance(1/30)
        self.assertTrue(sim.velocity_command.any())
        lab.reset()
        self.assertTrue(lab.aligning)
        lab.lost_view()
        self.assertTrue(lab.aligning)
        lab.advance(1/30)
        self.assertEqual(lab.alignment_status,"waiting")
        lab.toggle_demo()
        self.assertTrue(lab.demo)
        self.assertFalse(lab.aligning)
        lab.advance(1/30)
        self.assertTrue(sim.velocity_command.any())


if __name__=="__main__":
    unittest.main()
