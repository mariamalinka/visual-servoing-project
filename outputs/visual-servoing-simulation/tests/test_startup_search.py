"""Cold initialization, bounded search, persistence and rendered-camera regressions."""
import tempfile
from pathlib import Path
from unittest import TestCase, main
from unittest.mock import patch

import mujoco
import numpy as np

from app import Lab
from recovery import ReacquiringIBVS
from reference_image import DEFAULT_REFERENCE, load_reference, save_reference
from simulation import Simulation
from startup_search import StartupSearch, load_startup_config


class StartupMathTests(TestCase):
    def setUp(self):
        self.q = np.zeros(6)
        self.limits = np.tile([-2.,2.],(6,1))
        self.corners = np.array([[200,120],[400,120],[400,320],[200,320]],float)

    def make(self, **settings):
        cfg = load_startup_config()
        cfg.update(settings)
        return StartupSearch(self.q,self.limits,cfg)

    def test_scan_is_deterministic_and_expands_from_measured_start(self):
        self.q[:] = [.3,-.1,.2,.1,-.3,.2]
        a,b = self.make(),self.make()
        np.testing.assert_array_equal(a.waypoints,b.waypoints)
        np.testing.assert_array_equal(a.anchor,self.q)
        goals = np.asarray(a.waypoints)
        np.testing.assert_allclose(np.max(np.abs(goals[:,[0,4]]-self.q[[0,4]]),axis=0),
                                   np.deg2rad([48,48]))
        np.testing.assert_allclose(goals[:,[1,2,3,5]],np.tile(self.q[[1,2,3,5]],(len(goals),1)))
        for _ in range(20):
            np.testing.assert_array_equal(a.update(None,self.q,1/30).velocity,
                                          b.update(None,self.q,1/30).velocity)

    def test_confirmation_brakes_and_flicker_restarts_count(self):
        c = self.make()
        self.assertTrue(c.update(None,self.q,1/30).velocity.any())
        self.assertFalse(c.update(self.corners,self.q,1/30).velocity.any())
        c.update(None,self.q,1/30)
        for _ in range(2):
            self.assertEqual(c.update(self.corners,self.q,1/30).status,"confirming")
        result=c.update(self.corners,self.q,1/30)
        self.assertEqual(result.status,"acquired")
        self.assertFalse(result.velocity.any())
        self.assertGreater(c.acquired_s,c.first_detection_s)

    def test_timeout_is_terminal_even_if_marker_appears_later(self):
        c=self.make(max_search_time_s=.1)
        for _ in range(4):
            result=c.update(None,self.q,1/30)
        self.assertEqual(result.status,"target_not_found")
        self.assertEqual(c.stop_reason,"time_budget")
        self.assertFalse(result.velocity.any())
        self.assertFalse(c.update(self.corners,self.q,1/30).velocity.any())
        self.assertIsNone(c.acquired_s)

    def test_coverage_exhaustion_stops_and_rejects_stale_nan_corners(self):
        c=self.make()
        for goal in c.waypoints:
            result=c.update(np.full((4,2),np.nan),goal,1/30)
        self.assertEqual(result.status,"target_not_found")
        self.assertEqual(c.stop_reason,"coverage_complete")
        self.assertIsNone(c.first_detection_s)
        self.assertFalse(result.velocity.any())

    def test_near_limit_start_only_moves_inward_and_obeys_step_bounds(self):
        self.q[0]=1.999
        self.q[4]=-1.999
        c=self.make(max_search_time_s=2)
        q=self.q.copy()
        for _ in range(61):
            result=c.update(None,q,1/30)
            self.assertLessEqual(np.max(np.abs(result.velocity)),.25+1e-12)
            updated=q+result.velocity/30
            self.assertTrue(np.all(updated>=np.minimum(q,c.lower)-1e-12))
            self.assertTrue(np.all(updated<=np.maximum(q,c.upper)+1e-12))
            q=updated
        self.assertEqual(result.status,"target_not_found")
        self.assertTrue(np.all(np.asarray(c.waypoints)>=c.lower))
        self.assertTrue(np.all(np.asarray(c.waypoints)<=c.upper))

    def test_invalid_configuration_and_inputs_fail(self):
        for settings in ({"ring_radii_degrees":[]},{"ring_radii_degrees":[12,6]},
                         {"confirmation_frames":0},{"max_search_time_s":float("nan")},
                         {"max_joint_velocity_rad_s":0}):
            with self.subTest(settings=settings),self.assertRaises(ValueError):
                self.make(**settings)
        c=self.make()
        for dt in (0,-1,float("nan")):
            with self.assertRaises(ValueError):
                c.update(None,self.q,dt)
        with self.assertRaises(ValueError):
            c.update(None,np.zeros(5),1/30)



class StartupEvaluationTests(TestCase):
    def test_plan_keeps_all_draws_pairs_and_negative_controls(self):
        from run_startup_search import make_plan, TARGET_SHIFT_BOUNDS_M
        from benchmark import sample_plan, read_json, BENCH_CONFIG
        a=make_plan(20,42)
        b=make_plan(20,42)
        self.assertEqual(a,b)
        self.assertEqual(len(a),24)
        self.assertEqual(a[:5],make_plan(5,42)[:5])
        original=sample_plan(20,42,read_json(BENCH_CONFIG))
        for spec,old in zip(a[:20],original):
            self.assertEqual(spec["offset_degrees"],old["offset_degrees"])
            self.assertTrue(np.all(np.abs(spec["target_shift_m"])<=TARGET_SHIFT_BOUNDS_M))
        self.assertEqual([s["case"] for s in a[20:]],
                         ["absent_marker","absent_marker","unreachable_target","unreachable_target"])

    def test_incomplete_evaluation_cannot_be_reported(self):
        from run_startup_search import validate_run
        with self.assertRaises(ValueError):
            validate_run(Path("."),{"status":"running"},[],[])

class StartupSimulationTests(TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.reset()

    def test_late_startup_acquisition_has_a_separate_alignment_budget(self):
        sim=self.sim
        _,reference=load_reference(DEFAULT_REFERENCE,sim.camera_intrinsics(),sim.config,sim.marker_corners)
        c=ReacquiringIBVS(reference,sim.camera_intrinsics(),sim.config,sim.model.jnt_range)
        c.start(cold=True)
        q=sim.data.qpos.copy()
        for _ in range(50):
            result=c.update(None,sim.camera_jacobian(),q,1)
            self.assertEqual(result.status,"scanning")
        self.assertIsNone(c.last_visible_qpos)
        for _ in range(4):
            result=c.update(reference,sim.camera_jacobian(),q,1)
        self.assertEqual(result.status,"converged")
        np.testing.assert_array_equal(c.last_visible_qpos,q)
        self.assertGreater(c.elapsed_s,c.config["max_total_time_s"])

    def test_saved_goal_has_no_pose_and_rejects_mismatched_calibration(self):
        sim=self.sim
        with np.load(DEFAULT_REFERENCE,allow_pickle=False) as saved:
            self.assertEqual(set(saved.files),{"rgb","K","metadata"})
        rgb,corners=load_reference(DEFAULT_REFERENCE,sim.camera_intrinsics(),sim.config,sim.marker_corners)
        self.assertEqual(corners.shape,(4,2))
        with self.assertRaises(ValueError):
            load_reference(DEFAULT_REFERENCE,sim.camera_intrinsics()*2,sim.config,sim.marker_corners)
        changed=dict(sim.config,marker_id=8)
        with self.assertRaises(ValueError):
            load_reference(DEFAULT_REFERENCE,sim.camera_intrinsics(),changed,sim.marker_corners)

    def test_failed_teach_preserves_file_and_successful_teach_survives_restart(self):
        sim=self.sim
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"goal.npz"
            rgb=sim.image()
            expected=save_reference(path,rgb,sim.camera_intrinsics(),sim.config,sim.marker_corners)
            before=path.read_bytes()
            with self.assertRaises(ValueError):
                save_reference(path,np.zeros_like(rgb),sim.camera_intrinsics(),sim.config,sim.marker_corners)
            self.assertEqual(path.read_bytes(),before)
            sim.reset([35,0,0,0,0,0])
            lab=Lab(sim, auto_start=False,reference_path=path,cold_start=True)
            np.testing.assert_array_equal(lab.reference,expected)
            self.assertIsNone(lab.controller.last_visible_qpos)

    def test_cold_initialization_never_renders_a_live_home_view(self):
        sim=self.sim
        sim.reset([35,0,0,0,0,0])
        before=sim.data.qpos.copy()
        with patch.object(sim,"image",side_effect=AssertionError("Unexpected live image")):
            lab=Lab(sim, auto_start=False,cold_start=True)
        lab.draw()
        self.assertIsNone(lab.controller.last_visible_qpos)
        lab.align()
        np.testing.assert_array_equal(sim.data.qpos,before)
        self.assertIsNone(lab.controller.last_visible_qpos)

    def test_cold_demo_searches_then_aligns_with_adaptive_gain_without_mouse(self):
        sim=self.sim
        lab=Lab(sim, auto_start=False)
        lab.toggle_gain()
        lab.cold_start()
        lab.draw()
        self.assertIsNone(sim.marker_corners(sim.image()))
        self.assertIsNone(lab.controller.last_visible_qpos)
        lab.align()
        states=set()
        for _ in range(4100):
            lab.advance(1/30)
            states.add(lab.alignment_status)
            if not lab.aligning:
                break
        self.assertEqual(lab.alignment_status,"converged")
        self.assertTrue({"scanning","confirming","running"}<=states)
        self.assertNotIn("returning",states)
        self.assertIsNotNone(lab.controller.startup.acquired_s)
        sim.advance(1)
        corners=sim.marker_corners(sim.image())
        self.assertIsNotNone(corners)
        self.assertLess(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))),1)

    def test_absent_marker_stops_in_real_scene_and_stays_stopped(self):
        sim=self.sim
        board=sim.model.geom("target_board")
        original=board.rgba.copy()
        try:
            board.rgba[3]=0
            sim.reset([35,0,0,0,0,0])
            lab=Lab(sim, auto_start=False,cold_start=True)
            self.assertIsNone(sim.marker_corners(sim.image()))
            cfg=load_startup_config()
            cfg["max_search_time_s"]=.4
            lab.controller.startup_config=cfg
            lab.align()
            for _ in range(45):
                lab.advance(1/30)
            self.assertEqual(lab.alignment_status,"target_not_found")
            self.assertFalse(lab.aligning)
            self.assertFalse(sim.velocity_command.any())
        finally:
            board.rgba[:]=original
            sim.reset()

    def test_stop_pause_jog_and_gain_cancel_startup_search(self):
        sim=self.sim
        lab=Lab(sim, auto_start=False)
        for action in (lab.stop,lab.toggle_pause,lambda:lab.jog(3,1),lab.toggle_gain):
            lab.cold_start()
            lab.align()
            lab.advance(1/30)
            self.assertTrue(sim.velocity_command.any())
            action()
            self.assertFalse(lab.aligning)
            self.assertEqual(lab.controller.status,"canceled")
            for _ in range(30):
                lab.advance(1/30)
            self.assertFalse(sim.velocity_command.any())

    def test_reset_does_not_invent_detection_for_a_moved_target(self):
        sim=self.sim
        original=sim.model.body("target").pos.copy()
        try:
            lab=Lab(sim, auto_start=False)
            sim.model.body("target").pos[1]+=1
            lab.reset()
            self.assertIsNone(sim.marker_corners(sim.image()))
            self.assertIsNone(lab.controller.last_visible_qpos)
        finally:
            sim.model.body("target").pos[:]=original
            sim.reset()

if __name__=="__main__":
    main()
