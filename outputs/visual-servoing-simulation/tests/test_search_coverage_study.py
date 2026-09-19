"""Coverage reports retain every sampled scene and verify saved evidence."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
from run_search_coverage import summarize,validate,trace_path
from startup_search import load_startup_config

class CoverageStudyTests(unittest.TestCase):
    def test_summary_keeps_failures_and_cohorts_separate(self):
        rows=[]
        for cohort,i,a,b,visible in (("regression",1,"target_not_found","converged",False),
                                    ("holdout",1,"converged","timeout",True),
                                    ("holdout",2,"target_not_found","target_not_found",False)):
            for policy,outcome in (("coarse",a),("refined",b)):
                rows.append(dict(cohort=cohort,trial_id=i,policy=policy,case="random_target",
                                 outcome=outcome,initial_detected=visible,
                                 acquisition_s=None if outcome=="target_not_found" else 4))
        result=summarize(rows)
        self.assertEqual(result["regression"]["improved_trial_ids"],[1])
        self.assertEqual(result["holdout"]["regressed_trial_ids"],[1])
        self.assertEqual(result["holdout"]["refined"]["trials"],2)
        self.assertEqual(result["holdout"]["refined"]["converged"],0)
        self.assertEqual(result["holdout"]["refined"]["initially_undetected"],1)

    def test_incomplete_results_cannot_be_reported(self):
        with self.assertRaises(ValueError):
            validate(Path("."),{"status":"running"},[],[])

    def test_trace_check_rejects_changed_prefix_and_nonzero_stopped_command(self):
        spec=dict(cohort="holdout",trial_id=1,case="random_target",offset_degrees=[0]*6,target_shift_m=[0]*3)
        rows=[dict(spec,policy=p,initial_detected=True,acquisition_s=0.,sample_count=32,
                   outcome="converged",terminal_time_s=1/30) for p in ("coarse","refined")]
        trace=dict(time_s=np.arange(32)/30,qpos_rad=np.zeros((32,6)),
                   corners_px=np.zeros((32,4,2)),command_rad_s=np.zeros((32,6)),
                   phase=np.array(["running","converged"]+["post_stop"]*30),
                   startup_phase=np.zeros(32,bool),detected=np.ones(32,bool),error_px=np.zeros(32))
        manifest=dict(status="complete",completed_rows=2,requested_rows=2,
                      joint_limits_rad=np.tile([-2.,2.],(6,1)).tolist(),
                      search_configs={p:load_startup_config() for p in ("coarse","refined")},
                      recovery_config={"max_total_time_s":45},simulation_config={"camera_hz":30})
        with tempfile.TemporaryDirectory() as name:
            directory=Path(name)
            (directory/"traces").mkdir()
            for r in rows:
                np.savez_compressed(trace_path(directory,r),**trace)
            validate(directory,manifest,[spec],rows)
            changed={k:v.copy() for k,v in trace.items()}
            changed["qpos_rad"][0,0]=.01
            np.savez_compressed(trace_path(directory,rows[1]),**changed)
            with self.assertRaises(AssertionError):
                validate(directory,manifest,[spec],rows)
            trace["command_rad_s"][-1,0]=.01
            for r in rows:
                np.savez_compressed(trace_path(directory,r),**trace)
            with self.assertRaisesRegex(ValueError,"stopped"):
                validate(directory,manifest,[spec],rows)

if __name__=="__main__":
    unittest.main()
