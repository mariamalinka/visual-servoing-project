"""Paired reporting must retain failures and reject unfinished experiments."""
from pathlib import Path
import unittest
from run_joint_limit_study import compare,validate

class JointStudyTests(unittest.TestCase):
    def test_reporting_preserves_failed_starts_and_identifies_regressions(self):
        rows=[]
        for trial,a,b,visible,case in (
            (1,"timeout","converged",False,"random_target"),
            (2,"converged","target_not_found",True,"random_target"),
            (3,"target_not_found","target_not_found",False,"random_target"),
            (4,"target_not_found","target_not_found",False,"absent_marker")):
            for policy,outcome in (("baseline",a),("supervised",b)):
                rows.append(dict(trial_id=trial,policy=policy,outcome=outcome,
                    initial_detected=visible,case=case,alignment_retries=int(policy=="supervised" and trial==1)))
        result=compare(rows)
        self.assertEqual(result["supervised"]["trials"],3)
        self.assertEqual(result["supervised"]["converged"],1)
        self.assertEqual(result["improved_trial_ids"],[1])
        self.assertEqual(result["regressed_trial_ids"],[2])
        self.assertEqual(result["supervised"]["initially_undetected"],2)
        self.assertEqual(result["supervised"]["converged_after_retry"],1)
        self.assertEqual(result["supervised"]["negative_control_outcomes"],{"target_not_found":1})

    def test_incomplete_study_is_rejected(self):
        with self.assertRaises(ValueError):
            validate(Path("."),{"status":"running"},[],[])

if __name__=="__main__":
    unittest.main()
