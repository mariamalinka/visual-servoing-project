"""Small perception studies must report all cases, including failures."""
from pathlib import Path
import unittest
from run_natural_image_study import summarize,validate,plan_cases

class NaturalStudyTests(unittest.TestCase):
    def test_plan_and_summary_retain_all_cases(self):
        plan=plan_cases()
        self.assertEqual(len(plan),14)
        self.assertEqual(sum(not p["negative_control"] for p in plan),12)
        self.assertEqual(sum(p["negative_control"] for p in plan),2)
        rows=[]
        for spec in plan:
            for mode in ("aruco","natural"):
                failure=spec["trial_id"]==1 and mode=="natural"
                rows.append(dict(spec,perception_mode=mode,outcome="target_not_found" if spec["negative_control"] or failure else "converged",
                    initial_detected=not failure,terminal_time_s=4.,median_detector_call_ms=10.))
        summary=summarize(rows)
        self.assertEqual(summary["aruco"]["converged"],12)
        self.assertEqual(summary["natural"]["converged"],11)
        self.assertEqual(summary["natural"]["cases"],12)
        self.assertEqual(summary["natural"]["outcomes"]["target_not_found"],1)
        self.assertEqual(summary["natural"]["negative_outcomes"],{"target_not_found":2})

    def test_incomplete_study_cannot_be_reported(self):
        with self.assertRaises(ValueError):
            validate(Path("."),{"status":"running"},[],[])

if __name__=="__main__":
    unittest.main()
