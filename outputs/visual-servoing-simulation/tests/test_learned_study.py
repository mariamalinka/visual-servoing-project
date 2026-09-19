import unittest
from pathlib import Path
from run_learned_study import plan_cases,MODES
from run_natural_image_study import summarize,validate

class LearnedStudyTests(unittest.TestCase):
    def test_predeclared_plan_and_summary_keep_failed_starts(self):
        plan=plan_cases()
        self.assertEqual(len(plan),6)
        self.assertEqual(sum(not r["negative_control"] for r in plan),4)
        self.assertEqual([r["trial_id"] for r in plan],list(range(1,7)))
        rows=[]
        for spec in plan:
            for mode in MODES:
                failure=spec["negative_control"] or (spec["trial_id"]==1 and mode=="learned")
                rows.append(dict(spec,perception_mode=mode,outcome="target_not_found" if failure else "converged",
                    initial_detected=not failure,terminal_time_s=4.,median_detector_call_ms=10.))
        summary=summarize(rows,modes=MODES)
        self.assertEqual(summary["natural"]["converged"],4)
        self.assertEqual(summary["learned"]["cases"],4)
        self.assertEqual(summary["learned"]["converged"],3)
        self.assertEqual(summary["learned"]["negative_outcomes"],{"target_not_found":2})

    def test_incomplete_learned_comparison_is_rejected(self):
        with self.assertRaises(ValueError):
            validate(Path("."),{"status":"running"},[],[],modes=MODES,configs={})

if __name__=="__main__":unittest.main()
