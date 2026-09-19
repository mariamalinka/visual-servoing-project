"""Gain policy, feedback equivalence, metric and GUI/recovery integration tests."""
from pathlib import Path
import unittest

import cv2
import numpy as np

from adaptive_gain import (AdaptiveIBVSController, GainPolicy, load_gain_config,
                           make_gain_controller)
from analyze_gain_study import paired_changes, validate_run
from app import Lab
from benchmark import BENCH_CONFIG, read_json
from compare_controllers import run_comparison_trial
from control import IBVSController
from run_gain_study import trace_metrics
from simulation import Simulation


class GainMathTests(unittest.TestCase):
    def test_policy_is_bounded_monotonic_and_has_declared_limits(self):
        policy = GainPolicy(.8, 2.4, 8.)
        errors = np.r_[0., np.geomspace(1e-8, 1e6, 1000)]
        gains = np.array([policy(float(e)) for e in errors])
        self.assertEqual(gains[0], .8)
        self.assertAlmostEqual(gains[-1], 2.4)
        self.assertTrue(np.all(np.diff(gains) >= 0))
        self.assertTrue(np.all((gains >= .8) & (gains <= 2.4)))
        self.assertAlmostEqual(policy(8), .8 + 1.6*(1-np.exp(-1)))

    def test_invalid_policy_and_error_are_rejected(self):
        for values in ((0,2,8),(.8,-1,8),(2,1,8),(.8,2,0),(.8,float("nan"),8),(.8,2,float("inf"))):
            with self.subTest(values=values), self.assertRaises(ValueError):
                GainPolicy(*values)
        for error in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                GainPolicy(.8,2.4,8)(error)

    def test_metric_distinguishes_goal_crossing_from_error_growth(self):
        # Four corners move from +10 px error to -2 px: 20% signed overshoot,
        # although the RMS error never grows above its initial value.
        desired = np.zeros((4,2))
        corners = np.array([np.full((4,2), value, dtype=float) for value in (10, 3, -2)])
        trace = {"corners_px": corners, "error_px": np.array([14.14,4.24,2.83]),
                 "phase": np.array(["control"]*3),
                 "command_rad_s": np.array([np.full(6,v) for v in (.3,.2,.1)]),
                 "gain_per_s": np.array([2.,1.,.8])}
        metrics = trace_metrics(trace,desired,5,1)
        self.assertAlmostEqual(metrics["projected_overshoot_pct"],20)
        self.assertAlmostEqual(metrics["near_goal_command_rms_rad_s"],np.sqrt(.025))
        trace["corners_px"][:] = np.nan
        trace["error_px"][:] = np.nan
        metrics = trace_metrics(trace,desired,5,1)
        self.assertIsNone(metrics["projected_overshoot_pct"])
        self.assertIsNone(metrics["near_goal_command_rms_rad_s"])

    def test_paired_timing_does_not_compare_different_success_groups(self):
        rows=[]
        for trial, times in ((1, (4.,2.,1.)),(2,(8.,4.,2.))):
            for m,t in zip(("fixed","adaptive","fixed_high"),times):
                rows.append({"trial_id":trial,"method":m,"outcome":"converged","settling_time_s":t})
        rows[-2]["outcome"]="timeout"
        result=paired_changes(rows)
        self.assertEqual(result["adaptive"]["common_successes"],1)
        self.assertEqual(result["adaptive"]["median_settling_reduction_pct_vs_fixed"],50)
        self.assertEqual(result["fixed_high"]["common_successes"],2)

    def test_incomplete_gain_study_is_not_reported(self):
        with self.assertRaises(ValueError):
            validate_run(Path("."),{"status":"running"},[])


class GainIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()
        cls.reference=cls.sim.marker_corners(cls.sim.image())
        cls.settings=load_gain_config()
        cls.bench=read_json(BENCH_CONFIG)

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def test_constant_schedule_reproduces_fixed_commands_exactly(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        fixed=IBVSController(self.reference,sim.camera_intrinsics(),sim.config)
        constant=AdaptiveIBVSController(self.reference,sim.camera_intrinsics(),sim.config,
                                        GainPolicy(1.2,1.2,8.))
        for _ in range(180):
            corners=sim.marker_corners(sim.image())
            J=sim.camera_jacobian()
            a=fixed.update(corners,J,1/30)
            b=constant.update(corners,J,1/30)
            np.testing.assert_array_equal(a.velocity,b.velocity)
            self.assertEqual(a.status,b.status)
            sim.command_velocity(a.velocity)
            if a.status!="running":
                break
            sim.advance(1/30)
        self.assertEqual(a.status,"converged")

    def test_gain_updates_before_speed_limits_and_loss_stops(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        c=make_gain_controller("adaptive",self.reference,sim.camera_intrinsics(),sim.config,self.settings)
        corners=sim.marker_corners(sim.image())
        sample=c.update(corners,sim.camera_jacobian(),1/30)
        self.assertGreater(c.config["gain_per_s"],2.)
        self.assertLessEqual(np.max(np.abs(sample.velocity)),.35+1e-12)
        self.assertLessEqual(np.linalg.norm(sample.camera_twist[:3]),.12+1e-12)
        self.assertLessEqual(np.linalg.norm(sample.camera_twist[3:]),.4+1e-12)
        self.assertEqual(c.update(None,sim.camera_jacobian(),1/30).status,"tracking_loss")
        self.assertFalse(c.update(corners,sim.camera_jacobian(),1/30).velocity.any())
        c.reset()
        self.assertEqual(c.config["gain_per_s"],.8)
        c.elapsed_seconds=20
        self.assertEqual(c.update(corners,sim.camera_jacobian(),1/30).status,"timeout")
        self.assertEqual(sim.config["ibvs"]["gain_per_s"],1.2)

    def test_adaptive_trial_aligns_with_saved_gain_and_stopped_evidence(self):
        sim=self.sim
        spec={"trial_id":1,"profile":"test","offset_degrees":sim.config["ibvs"]["start_offset_degrees"]}
        def factory(reference,K,config):
            return make_gain_controller("adaptive",reference,K,config,self.settings)
        row,trace,_,_=run_comparison_trial(sim,self.reference,spec,self.bench,{},"adaptive",
                                          controller_factory=factory)
        self.assertEqual(row["outcome"],"converged",row)
        self.assertGreater(np.ptp(trace["gain_per_s"]),1.)
        self.assertEqual(int((trace["phase"]=="post_stop").sum()),30)
        self.assertTrue(np.all(trace["error_px"][trace["phase"]=="post_stop"]<1))
        self.assertLessEqual(row["max_command_rad_s"],.35+1e-12)

    def test_gain_button_cancels_motion_and_preserves_remembered_view(self):
        sim=self.sim
        lab=Lab(sim, auto_start=False)
        lab.offset()
        lab.align()
        lab.advance(1/30)
        self.assertTrue(np.any(sim.velocity_command))
        last_view=lab.controller.last_visible_qpos.copy()
        lab.draw()
        rect=next(rect for rect,action in lab.buttons if action=="gain")
        lab.on_mouse(cv2.EVENT_LBUTTONDOWN,rect[0]+10,rect[1]+10,0,None)
        self.assertEqual(lab.gain_mode,"adaptive")
        self.assertIsInstance(lab.controller.ibvs,AdaptiveIBVSController)
        self.assertFalse(lab.aligning)
        self.assertFalse(sim.velocity_command.any())
        np.testing.assert_array_equal(lab.controller.last_visible_qpos,last_view)
        lab.toggle_gain()
        self.assertEqual(lab.gain_mode,"fixed")
        self.assertIs(type(lab.controller.ibvs),IBVSController)

    def test_adaptive_mode_reacquires_lost_target_without_pointer_events(self):
        lab=Lab(self.sim, auto_start=False)
        lab.toggle_gain()
        lab.lost_view()
        lab.align()
        for _ in range(1400):
            lab.advance(1/30)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status,"converged")
        self.assertGreater(lab.controller.reacquisitions,0)
        self.assertIsInstance(lab.controller.ibvs,AdaptiveIBVSController)
        self.assertFalse(self.sim.velocity_command.any())
        self.sim.advance(1)
        corners=self.sim.marker_corners(self.sim.image())
        self.assertIsNotNone(corners)
        self.assertLess(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))),1.)


if __name__=="__main__":
    unittest.main()
