"""Search gaps, archived-path compatibility, bounds and missed-start regressions."""
import json
from pathlib import Path
import unittest
import numpy as np

from startup_search import StartupSearch, load_startup_config
from simulation import ROOT, Simulation
from app import Lab
from reference_image import DEFAULT_REFERENCE, load_reference
from recovery import load_recovery_config
from run_startup_search import trial

FIXTURES = Path(__file__).parent / "fixtures"


class RefinedSearchTests(unittest.TestCase):
    def setUp(self):
        self.q = np.array([.2, -.4, .8, .1, -.3, .2])
        self.limits = np.tile([-2.5, 2.5], (6, 1))
        self.corners = np.array([[200,120],[400,120],[400,320],[200,320]],float)

    def search(self, **settings):
        return StartupSearch(self.q, self.limits, dict(load_startup_config(), **settings))

    def test_preserves_recorded_coarse_commands_and_legacy_missing_setting(self):
        with np.load(FIXTURES/"coarse-search-prefix.npz",allow_pickle=False) as saved:
            cfg=load_startup_config()
            legacy=dict(cfg)
            legacy.pop("refine_after_coarse")
            legacy["max_search_time_s"]=90
            for config in (cfg,legacy):
                search=StartupSearch(saved["qpos_rad"][0],saved["joint_limits_rad"],config)
                for q,command in zip(saved["qpos_rad"],saved["command_rad_s"]):
                    sample=search.update(None,q,1/30)
                    np.testing.assert_array_equal(sample.velocity,command)
                    self.assertEqual(search.stage,"coarse")
                result=search.update(None,saved["terminal_qpos_rad"],1/30)
                self.assertEqual(result.status,"scanning" if config.get("refine_after_coarse") else "target_not_found")

    def test_finer_pass_finds_a_narrow_view_between_original_rings(self):
        # Synthetic detector depends only on measured angles. Its visible patch is
        # inside the original box but between the original rectangular paths.
        for refine in (False,True):
            search=self.search(refine_after_coarse=refine)
            q=self.q.copy()
            for _ in range(5500):
                offset=np.rad2deg(q-self.q)[[0,4]]
                visible=np.max(np.abs(offset-[18,6]))<1.0
                sample=search.update(self.corners if visible else None,q,1/30)
                q+=sample.velocity/30
                if sample.status not in ("scanning","confirming"):
                    break
            self.assertEqual(sample.status,"acquired" if refine else "target_not_found")
            self.assertFalse(sample.velocity.any())
            if refine:
                self.assertEqual(search.stage,"refined")
                self.assertGreater(search.acquired_s,search.refinement_started_s)

    def test_refinement_confirmation_brakes_flicker_and_cancel(self):
        search=self.search()
        search.index=search.coarse_waypoint_count+1
        self.assertTrue(search.update(None,self.q,1/30).velocity.any())
        for _ in range(2):
            self.assertEqual(search.update(self.corners,self.q,1/30).status,"confirming")
            self.assertFalse(search.update(None,self.q,1/30).status=="acquired")
        for _ in range(2):
            result=search.update(self.corners,self.q,1/30)
            self.assertEqual(result.status,"confirming")
            self.assertFalse(result.velocity.any())
        search.cancel()
        self.assertFalse(search.update(self.corners,self.q,1/30).velocity.any())
        self.assertIsNone(search.acquired_s)

    def test_deadline_covers_both_passes_and_never_restarts(self):
        search=self.search(max_search_time_s=2)
        search.index=search.coarse_waypoint_count
        search.elapsed_s=2-1/30
        search.update(None,self.q,1/30)
        self.assertAlmostEqual(search.refinement_started_s,2-1/30)
        result=search.update(None,self.q,1/30)
        self.assertEqual(result.status,"target_not_found")
        self.assertEqual(search.stop_reason,"time_budget")
        self.assertFalse(search.update(self.corners,self.q,1/30).velocity.any())

    def test_complete_fine_path_is_bounded_near_limits(self):
        self.q[[0,4]]=[2.49,-2.49]
        search=self.search()
        q=self.q.copy()
        stages=set()
        for _ in range(5500):
            result=search.update(None,q,1/30)
            stages.add(search.stage)
            after=q+result.velocity/30
            self.assertLessEqual(np.max(np.abs(result.velocity)),.25+1e-12)
            self.assertTrue(np.all(after>=np.minimum(q,search.lower)-1e-12))
            self.assertTrue(np.all(after<=np.maximum(q,search.upper)+1e-12))
            q=after
            if result.status=="target_not_found":
                break
        self.assertEqual(stages,{"coarse","refined"})
        self.assertEqual(result.status,"target_not_found")
        self.assertFalse(result.velocity.any())
        self.assertLessEqual(search.elapsed_s,180+1/30)

    def test_refinement_uses_configured_gaps_and_requires_boolean(self):
        search=self.search(ring_radii_degrees=[10,30,50])
        np.testing.assert_array_equal(search.refinement_radii_degrees,[5,20,40])
        self.assertLessEqual(np.max(np.abs(np.array(search.waypoints)-self.q)),np.deg2rad(50)+1e-12)
        for invalid in (1,0,"yes",None):
            with self.subTest(invalid=invalid),self.assertRaises(ValueError):
                self.search(refine_after_coarse=invalid)


class RefinedSimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()
        _,cls.reference=load_reference(DEFAULT_REFERENCE,cls.sim.camera_intrinsics(),cls.sim.config,cls.sim.marker_corners)
        cls.cases=json.loads((FIXTURES/"search-missed-starts.json").read_text(encoding="utf-8"))

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def check_missed_case(self,trial_id):
        spec=next(s for s in self.cases if s["trial_id"]==trial_id)
        row,trace,_,_=trial(self.sim,self.reference,spec,"search",load_startup_config(),load_recovery_config())
        self.assertFalse(row["initial_detected"])
        self.assertEqual(row["outcome"],"converged")
        self.assertGreater(row["acquisition_s"],row["search_refinement_started_s"])
        self.assertTrue(row["search_bounds_ok"])
        self.assertLessEqual(row["max_search_command_rad_s"],.25+1e-12)
        self.assertLessEqual(row["max_search_excursion_rad"],np.deg2rad(48)+.005)
        stopped=trace["phase"]=="post_stop"
        self.assertTrue(np.all(trace["error_px"][stopped]<1))
        self.assertFalse(trace["command_rad_s"][stopped].any())
        self.assertTrue(np.all(trace["search_stage"][trace["startup_phase"] & (trace["time_s"]>row["search_refinement_started_s"]+.04)]=="refined"))

    def test_previous_miss_56_finds_and_aligns(self):
        self.check_missed_case(56)

    def test_previous_miss_118_finds_and_aligns(self):
        self.check_missed_case(118)

    def test_gui_stop_pause_jog_and_gain_cancel_fine_pass(self):
        sim=self.sim
        for action in ("stop","pause","jog","gain"):
            sim.reset([35,0,0,0,0,0])
            lab=Lab(sim,cold_start=True)
            lab.advance(1/30)
            search=lab.controller.startup
            search.index=search.coarse_waypoint_count+1
            lab.advance(1/30)
            self.assertIn("(refined)",lab.message)
            self.assertTrue(sim.velocity_command.any())
            if action=="stop": lab.stop()
            elif action=="pause": lab.toggle_pause()
            elif action=="jog": lab.jog(3,1)
            else: lab.toggle_gain()
            for _ in range(35):
                lab.advance(1/30)
            self.assertFalse(lab.aligning)
            self.assertFalse(sim.velocity_command.any())
            self.assertEqual(lab.controller.status,"canceled")


if __name__=="__main__":
    unittest.main()
