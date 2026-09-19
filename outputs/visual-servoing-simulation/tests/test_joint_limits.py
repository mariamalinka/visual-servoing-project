"""Independent optimizer checks, finite retry logic and real failure regressions."""
import itertools
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from app import Lab
from control import ControlSample, IBVSController
from joint_limits import (bounded_least_squares, velocity_bounds, limited_command,
                          JointLimitSupervisor, load_joint_limit_config)
from recovery import ACTIVE_STATES, ReacquiringIBVS, load_recovery_config
from reference_image import DEFAULT_REFERENCE, load_reference
from run_startup_search import trial
from simulation import Simulation
from startup_search import load_startup_config


class JointLimitMathTests(unittest.TestCase):
    def setUp(self):
        self.limits=np.tile([-2.,2.],(6,1))
        self.settings=load_joint_limit_config()
        self.ibvs=dict(max_joint_velocity_rad_s=.35,joint_damping=.005,
                       max_linear_velocity_m_s=.12,max_angular_velocity_rad_s=.4)

    def test_active_set_matches_independent_exhaustive_face_search(self):
        rng=np.random.default_rng(617)
        for _ in range(12):
            A=rng.normal(size=(7,4))
            b=rng.normal(size=7)*3
            lo=rng.uniform(-.6,-.1,4)
            hi=rng.uniform(.1,.7,4)
            prefer=rng.normal(size=4)*.1
            damping=.03
            aug=np.vstack((A,damping*np.eye(4)))
            goal=np.r_[b,damping*prefer]
            best=float("inf")
            for face in itertools.product((-1,0,1),repeat=4):
                face=np.array(face)
                free=face==0
                x=np.where(face==-1,lo,hi).copy()
                if free.any():
                    x[free]=np.linalg.lstsq(aug[:,free],goal-aug[:,~free]@x[~free],rcond=None)[0]
                if np.any(x<lo-1e-9) or np.any(x>hi+1e-9):
                    continue
                best=min(best,float(np.linalg.norm(aug@x-goal)**2))
            actual=bounded_least_squares(A,b,lo,hi,damping,prefer)
            self.assertTrue(np.all(actual>=lo))
            self.assertTrue(np.all(actual<=hi))
            self.assertAlmostEqual(float(np.linalg.norm(aug@actual-goal)**2),best,places=8)

    def test_singular_problem_and_fixed_joint_obey_kkt_conditions(self):
        A=np.ones((3,6))
        b=np.array([1.,1.,1.])
        lo=np.full(6,-.1)
        hi=np.full(6,.2)
        lo[0]=hi[0]=0
        x=bounded_least_squares(A,b,lo,hi,.01)
        self.assertEqual(x[0],0)
        gradient=A.T@(A@x-b)+.01**2*x
        for i in range(1,6):
            if abs(x[i]-lo[i])<1e-9:
                self.assertGreaterEqual(gradient[i],-1e-8)
            elif abs(x[i]-hi[i])<1e-9:
                self.assertLessEqual(gradient[i],1e-8)
            else:
                self.assertLess(abs(gradient[i]),1e-8)
        self.assertTrue(np.isfinite(x).all())

    def test_invalid_optimizer_inputs_are_rejected(self):
        for A,b,lo,hi,damp in (
            (np.eye(2),np.ones(3),[-1]*2,[1]*2,.01),
            (np.eye(2),[1,np.nan],[-1]*2,[1]*2,.01),
            (np.eye(2),[1,1],[2,-1],[1,1],.01),
            (np.eye(2),[1,1],[-1]*2,[1]*2,0)):
            with self.assertRaises(ValueError):
                bounded_least_squares(A,b,lo,hi,damp)

    def test_velocity_damper_slows_outward_commands_but_allows_retreat(self):
        q=np.zeros(6)
        q[0]=1.94-.01
        q[1]=1.999  # Already inside the configured margin.
        q[2]=-1.999
        lo,hi=velocity_bounds(q,self.limits,.35,.06,.4,1/30)
        self.assertAlmostEqual(hi[0],.35*.01/.4)
        self.assertEqual(hi[1],0)
        self.assertEqual(lo[2],0)
        self.assertEqual(lo[1],-.35)
        self.assertEqual(hi[2],.35)
        self.assertLessEqual(q[0]+hi[0]/30,1.94)
        with self.assertRaises(ValueError):
            velocity_bounds(q,self.limits,.35,.06,.4,0)

    def test_feasible_original_command_is_preserved_exactly(self):
        v=np.array([np.nextafter(.35,np.inf),-.03,.02,.06,-.04,.01])
        sample=ControlSample("running",10.,v,.2*v)
        actual,changed=limited_command(sample,.2*np.eye(6),np.zeros(6),self.limits,
                                       self.settings,self.ibvs,1/30)
        np.testing.assert_array_equal(actual,v)
        self.assertFalse(changed)

    def test_constrained_command_obeys_joint_and_actual_camera_speed_limits(self):
        q=np.array([1.939,0,0,0,0,0])
        J=np.diag([4.,.3,2.,1.,2.,1.])
        sample=ControlSample("running",50.,np.full(6,.35),np.ones(6))
        v,changed=limited_command(sample,J,q,self.limits,self.settings,self.ibvs,1/30)
        self.assertTrue(changed)
        lo,hi=velocity_bounds(q,self.limits,.35,.06,np.deg2rad(25),1/30)
        self.assertTrue(np.all(v>=lo-1e-12))
        self.assertTrue(np.all(v<=hi+1e-12))
        self.assertLessEqual(np.linalg.norm((J@v)[:3]),.12+1e-12)
        self.assertLessEqual(np.linalg.norm((J@v)[3:]),.4+1e-12)

    def test_stall_brakes_return_is_bounded_and_confirmation_requires_fresh_frames(self):
        c=JointLimitSupervisor(self.limits)
        sample=ControlSample("running",20.,np.full(6,.05),np.full(6,.05))
        q=np.zeros(6)
        c.filter(sample,np.eye(6),q,1/30,self.ibvs)
        q=np.full(6,.3)
        for _ in range(65):
            out=c.filter(sample,np.eye(6),q,1/30,self.ibvs)
            if out.status=="repositioning":
                break
        self.assertEqual(out.status,"repositioning")
        self.assertFalse(out.velocity.any())
        self.assertEqual(c.retries,1)
        out=c.reposition(None,q,1/30,3)
        self.assertEqual(out.status,"repositioning")
        self.assertLessEqual(np.max(np.abs(out.velocity)),.25)
        self.assertTrue(np.all(out.velocity<0))
        goal=c.reposition_goal
        corners=np.array([[1.,1.],[2.,1.],[2.,2.],[1.,2.]])
        self.assertEqual(c.reposition(corners,goal,1/30,3).status,"retry_confirming")
        c.reposition(None,goal,1/30,3)
        for _ in range(2):
            self.assertEqual(c.reposition(corners,goal,1/30,3).status,"retry_confirming")
        self.assertEqual(c.reposition(corners,goal,1/30,3).status,"retry_ready")
        self.assertTrue(c.alternate)
        for _ in range(65):
            out=c.filter(sample,np.eye(6),goal,1/30,self.ibvs)
            if out.status=="alignment_stalled":
                break
        self.assertEqual(out.status,"alignment_stalled")
        self.assertEqual(c.retries,1)
        self.assertFalse(out.velocity.any())

    def test_unobserved_return_view_has_a_deadline_and_bad_settings_are_rejected(self):
        settings=dict(self.settings,max_reposition_time_s=.1)
        c=JointLimitSupervisor(self.limits,settings)
        c.anchor=np.zeros(6)
        sample=ControlSample("running",10.,np.ones(6),np.ones(6))
        c._begin_retry(sample,np.zeros(6),"stalled")
        for _ in range(4):
            out=c.reposition(None,np.zeros(6),1/30,3)
        self.assertEqual(out.status,"reposition_timeout")
        self.assertFalse(out.velocity.any())
        for setting in ({"max_retries":-1},{"retry_joint_damping":0},
                        {"stall_window_s":float("nan")},{"enabled":"yes"}):
            with self.assertRaises(ValueError):
                JointLimitSupervisor(self.limits,dict(self.settings,**setting))

    def test_optimizer_failure_commands_zero(self):
        c=JointLimitSupervisor(self.limits)
        sample=ControlSample("running",20.,np.ones(6),np.ones(6))
        with patch("joint_limits.bounded_least_squares",side_effect=RuntimeError("test failure")):
            out=c.filter(sample,np.eye(6),np.full(6,1.93),1/30,self.ibvs)
        self.assertEqual(out.status,"solver_failure")
        self.assertFalse(out.velocity.any())


class JointLimitIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()
        _,cls.reference=load_reference(DEFAULT_REFERENCE,cls.sim.camera_intrinsics(),
                                       cls.sim.config,cls.sim.marker_corners)
        cls.starts=json.loads((Path(__file__).parent/"fixtures/joint-limit-starts.json").read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def test_both_previous_joint_limit_failures_now_reposition_and_converge(self):
        for spec in self.starts:
            with self.subTest(trial=spec["trial_id"]):
                row,trace,_,_=trial(self.sim,self.reference,spec,"search",
                                    load_startup_config(),load_recovery_config())
                self.assertEqual(row["outcome"],"converged",row)
                self.assertEqual(row["alignment_retries"],1)
                self.assertTrue({"joint_limited","repositioning","retry_confirming","realigning"}<=set(trace["phase"]))
                self.assertLess(row["alignment_after_acquisition_s"],45)
                self.assertGreater(row["min_joint_margin_rad"],.055)
                self.assertLess(row["final_error_px"],1)
                self.assertTrue(row["stopped_command_zero"])
                returned=[e for e in row["motion_events"] if e["kind"]=="retry_started"]
                self.assertEqual(len(returned),1)
                distance=np.abs(np.array(returned[0]["goal_qpos_rad"])-returned[0]["qpos_rad"])
                self.assertLessEqual(distance.max(),np.deg2rad(140)+1e-12)

    def test_disabled_supervision_reproduces_original_ibvs_commands(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        original=IBVSController(self.reference,sim.camera_intrinsics(),sim.config)
        config=dict(load_joint_limit_config(),enabled=False)
        c=ReacquiringIBVS(self.reference,sim.camera_intrinsics(),sim.config,sim.model.jnt_range,
                          taught_qpos=sim.data.qpos,motion_config=config)
        c.start()
        for _ in range(60):
            corners=sim.marker_corners(sim.image())
            J=sim.camera_jacobian()
            a=original.update(corners,J,1/30)
            b=c.update(corners,J,sim.data.qpos,1/30)
            np.testing.assert_array_equal(a.velocity,b.velocity)
            self.assertEqual(a.status,b.status)
            sim.command_velocity(a.velocity)
            sim.advance(1/30)

    def test_stop_pause_jog_and_gain_cancel_a_stalled_plant_retry(self):
        sim=self.sim
        lab=Lab(sim)
        command=sim.command_velocity
        for action in ("stop","pause","jog","gain"):
            lab.offset()
            # Simulate blocked actuators: images stay fixed despite commands.
            with patch.object(sim,"command_velocity",side_effect=lambda _:command(np.zeros(6))):
                for _ in range(70):
                    lab.advance(1/30)
                    if lab.alignment_status=="repositioning":
                        break
            self.assertEqual(lab.alignment_status,"repositioning")
            if action=="stop":
                lab.stop()
            elif action=="pause":
                lab.toggle_pause()
                lab.toggle_pause()
            elif action=="jog":
                lab.jog(3,1)
            else:
                lab.toggle_gain()
            for _ in range(30):
                lab.advance(1/30)
            self.assertFalse(lab.aligning)
            self.assertFalse(sim.velocity_command.any())
            self.assertEqual(lab.controller.status,"canceled")

    def test_overall_deadline_includes_repositioning_and_is_terminal(self):
        sim=self.sim
        cfg=dict(load_recovery_config(),max_total_time_s=.1)
        c=ReacquiringIBVS(self.reference,sim.camera_intrinsics(),sim.config,sim.model.jnt_range,
                          taught_qpos=sim.data.qpos,recovery_config=cfg)
        c.start()
        c.motion.anchor=sim.data.qpos.copy()
        q=sim.data.qpos.copy()
        q[0]+=.3
        sample=ControlSample("running",20.,np.ones(6),np.ones(6))
        c.motion._begin_retry(sample,q,"stalled")
        c.mode="repositioning"
        for _ in range(8):
            out=c.update(None,sim.camera_jacobian(),q,1/30)
        self.assertEqual(out.status,"timeout")
        self.assertFalse(out.velocity.any())
        self.assertEqual(c.motion.retries,1)
        self.assertEqual(c.update(self.reference,sim.camera_jacobian(),q,1/30).status,"timeout")


if __name__=="__main__":
    unittest.main()
