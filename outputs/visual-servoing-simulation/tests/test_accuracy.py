"""Independent metric, calibration and paired-plan contracts (no rendering required)."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from accuracy import CONFIG_PATH, make_plan, physical_pass, pose_accuracy, report, summarize, validate_rows
from calibration import ControlCalibration
from camera_robustness import write_json


def pose(translation=(0,0,0), rotation_deg=(0,0,0)):
    T=np.eye(4)
    T[:3,:3]=cv2.Rodrigues(np.deg2rad(rotation_deg))[0]
    T[:3,3]=translation
    return T


class PoseAccuracyTests(unittest.TestCase):
    def test_identity_and_known_metric_units(self):
        zero=pose_accuracy(np.eye(4),np.eye(4))
        self.assertEqual(zero["position_error_mm"],0)
        self.assertEqual(zero["orientation_error_deg"],0)
        score=pose_accuracy(pose([.003,.004,0],[0,0,30]),np.eye(4))
        self.assertAlmostEqual(score["position_error_mm"],5)
        self.assertAlmostEqual(score["orientation_error_deg"],30)
        np.testing.assert_allclose(score["translation_error_mm"],[3,4,0])

    def test_translation_components_use_taught_axes(self):
        goal=pose([.5,0,0],[0,0,90])
        current=goal.copy();current[1,3]=.01
        score=pose_accuracy(current,goal)
        np.testing.assert_allclose(score["translation_error_mm"],[10,0,0],atol=1e-10)

    def test_rotation_wrap_and_half_turn(self):
        self.assertAlmostEqual(pose_accuracy(pose(rotation_deg=[0,0,-179]),
                                            pose(rotation_deg=[0,0,179]))["orientation_error_deg"],2)
        self.assertAlmostEqual(pose_accuracy(pose(rotation_deg=[180,0,0]),np.eye(4))["orientation_error_deg"],180)

    def test_common_world_transform_cannot_change_the_score(self):
        current=pose([.1,.2,.3],[5,9,-8]);goal=pose([.11,.18,.3],[6,10,-8])
        world=pose([100,-20,30],[40,-10,60])
        a,b=pose_accuracy(current,goal),pose_accuracy(world@current,world@goal)
        np.testing.assert_allclose(a["translation_error_mm"],b["translation_error_mm"],atol=1e-10)
        self.assertAlmostEqual(a["orientation_error_deg"],b["orientation_error_deg"])

    def test_nonrigid_nonfinite_and_wrong_shape_are_rejected(self):
        bad=[np.eye(3),np.full((4,4),np.nan)]
        shear=np.eye(4);shear[0,1]=.1;bad.append(shear)
        reflected=np.eye(4);reflected[0,0]=-1;bad.append(reflected)
        for value in bad:
            with self.subTest(value=value),self.assertRaises(ValueError):
                pose_accuracy(value,np.eye(4))

    def test_physical_acceptance_requires_both_frames_and_orientation(self):
        fine=dict(position_error_mm=1.,orientation_error_deg=.2)
        self.assertTrue(physical_pass(fine,fine,2,1))
        self.assertFalse(physical_pass(fine,dict(position_error_mm=3.,orientation_error_deg=.2),2,1))
        self.assertFalse(physical_pass(dict(position_error_mm=1.,orientation_error_deg=2.),fine,2,1))


class CalibrationMathTests(unittest.TestCase):
    def test_nominal_calibration_preserves_exact_values(self):
        K=np.array([[460.,0,320],[0,460,240],[0,0,1]])
        J=np.arange(36,dtype=float).reshape(6,6)
        c=ControlCalibration()
        np.testing.assert_array_equal(c.intrinsics(K),K)
        np.testing.assert_array_equal(c.jacobian(J),J)

    def test_intrinsics_and_target_size_change_only_copies(self):
        K=np.array([[460.,0,320],[0,450,240],[0,0,1]])
        config=dict(marker_side_m=.24,ibvs=dict(gain_per_s=1.2))
        c=ControlCalibration(dict(focal_scale=[1.05,.95],principal_offset_px=[5,-3],target_size_scale=1.1))
        out=c.intrinsics(K);cfg=c.controller_config(config)
        np.testing.assert_allclose(out,[[483,0,325],[0,427.5,237],[0,0,1]])
        self.assertAlmostEqual(cfg["marker_side_m"],.264)
        cfg["ibvs"]["gain_per_s"]=7
        self.assertEqual(config["ibvs"]["gain_per_s"],1.2)
        self.assertEqual(K[0,0],460)

    def test_translated_mount_has_the_correct_lever_arm_sign(self):
        c=ControlCalibration(dict(mount_translation_mm=[100,0,0]))
        twist=c.jacobian(np.eye(6))@np.array([0,0,0,0,0,2.])
        np.testing.assert_allclose(twist,[0,.2,0,0,0,2])

    def test_mount_rotation_rotates_both_velocity_blocks(self):
        c=ControlCalibration(dict(mount_rotation_vector_deg=[0,0,90]))
        twist=c.jacobian(np.eye(6))@np.array([1,0,0,1,0,0.])
        np.testing.assert_allclose(twist,[0,-1,0,0,-1,0],atol=1e-12)

    def test_bad_calibration_is_rejected(self):
        for settings in (dict(focal_scale=[0,1]),dict(focal_scale=[1]),dict(focal_scale=[True,1]),dict(mount_translation_mm=[0,np.nan,0]),
                         dict(target_size_scale=True),dict(target_size_scale=-1),dict(typo=1)):
            with self.subTest(settings=settings),self.assertRaises(ValueError):
                ControlCalibration(settings)


class AccuracyPlanTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(CONFIG_PATH.read_text())

    def test_profiles_pair_identical_starts_and_perception_seeds(self):
        plan=make_plan(self.config,["aruco","learned"],starts=2,profiles=["nominal","combined"])
        self.assertEqual(len(plan["trials"]),8)
        for case in (0,1):
            group=[t for t in plan["trials"] if t["case"]==case]
            self.assertTrue(all(t["offset_degrees"]==group[0]["offset_degrees"] for t in group))
            self.assertTrue(all(t["perception_seed"]==group[0]["perception_seed"] for t in group))
        repeated=make_plan(self.config,["aruco"],starts=3,profiles=["nominal"])
        self.assertEqual(plan["trials"][1]["offset_degrees"],repeated["trials"][1]["offset_degrees"])

    def test_invalid_settings_and_missing_baseline_fail(self):
        for overrides in (dict(starts=0),dict(starts=True),dict(seed=-1),dict(profiles=["combined"]),
                          dict(modes=["aruco","aruco"]),dict(profiles=["nominal","typo"])):
            with self.subTest(overrides=overrides),self.assertRaises(ValueError):
                make_plan(self.config,**overrides)
        config=copy.deepcopy(self.config);config["position_tolerance_mm"]=0
        with self.assertRaises(ValueError):make_plan(config)
        config=copy.deepcopy(self.config);config["typo"]=1
        with self.assertRaises(ValueError):make_plan(config)

    def results(self):
        plan=make_plan(self.config,["aruco"],starts=2,profiles=["nominal","combined"])
        score=lambda mm:dict(position_error_mm=mm,orientation_error_deg=.2,translation_error_mm=[mm,0,0])
        rows=[]
        for index,spec in enumerate(plan["trials"]):
            row={k:spec[k] for k in ("id","mode","profile","case")}
            failed=index==3
            mm=1 if index==0 else 3 if index==1 else 2
            row.update(outcome="timeout" if failed else "converged",
                       pixel_success=not failed,physical_success=index==0,
                       safety_violations=[],completion_s=4+index,
                       max_post_stop_error_px=2 if failed else .8,
                       final_accuracy=dict(camera=score(mm),tool=score(mm)))
            rows.append(row)
        return plan,rows

    def test_failures_pixel_only_passes_and_paired_denominators_are_kept(self):
        plan,rows=self.results()
        groups=summarize(plan,rows)
        self.assertEqual(groups[0]["pixel_only"],1)
        self.assertEqual(groups[1]["completed"],2)
        self.assertEqual(groups[1]["outcomes"],dict(converged=1,timeout=1))
        self.assertEqual(groups[1]["paired_successes"],1)
        self.assertEqual(groups[1]["tool"]["paired_median_position_delta_mm"],1)
        self.assertEqual(summarize(plan,rows[:1])[1]["planned"],2)
        self.assertEqual(summarize(plan,rows[:1])[1]["completed"],0)

    def test_duplicate_and_changed_trial_metadata_fail(self):
        plan,rows=self.results()
        with self.assertRaises(ValueError):validate_rows(plan,rows+[rows[0]])
        rows[0]["case"]=10
        with self.assertRaises(ValueError):validate_rows(plan,rows)

    def test_report_rebuilds_from_json_without_raw_traces(self):
        plan,rows=self.results()
        with tempfile.TemporaryDirectory() as folder:
            directory=Path(folder)
            write_json(directory/"plan.json",plan);write_json(directory/"trials.json",rows)
            report(directory)
            self.assertTrue((directory/"sensitivity.png").is_file())
            self.assertTrue((directory/"pixel-vs-physical.png").is_file())
            self.assertIn("timeout",(directory/"REPORT.md").read_text())
            self.assertIn("4/4",(directory/"REPORT.md").read_text())


if __name__=="__main__":
    unittest.main()

