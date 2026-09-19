"""Independent image warps and negative inputs exercise natural target measurements."""
import json
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
from perception import NaturalImagePerception,load_natural_config,NATURAL_REFERENCE
from reference_image import load_reference


class NaturalPerceptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector=NaturalImagePerception()
        cls.expected=np.array([[145,90],[475,105],[460,400],[125,380]],np.float32)
        cls.H=cv2.getPerspectiveTransform(cls.detector.boundary,cls.expected)
        cls.frame=cv2.warpPerspective(cls.detector.template_rgb,cls.H,(640,480),borderValue=(22,22,22))

    def test_known_projective_warp_localizes_picture_in_camera_coordinates(self):
        o=self.detector.observe(self.frame)
        self.assertEqual(o.reason,"accepted")
        self.assertGreaterEqual(o.inliers,12)
        self.assertGreater(o.coverage,.18)
        self.assertLess(np.sqrt(np.mean(np.sum((o.corners-self.expected)**2,axis=1))),1.2)
        self.assertLess(o.reprojection_rms_px,1.5)

    def test_optional_downscaling_returns_original_camera_coordinates(self):
        detector=NaturalImagePerception(config=dict(load_natural_config(),detection_width_px=480))
        o=detector.observe(self.frame)
        self.assertIsNotNone(o.corners)
        self.assertLess(np.sqrt(np.mean(np.sum((o.corners-self.expected)**2,axis=1))),1.5)

    def test_partial_occlusion_can_retain_distributed_matches(self):
        frame=self.frame.copy()
        frame[100:210,145:280]=[40,40,40]
        o=self.detector.observe(frame)
        self.assertIsNotNone(o.corners,o.reason)
        self.assertLess(np.sqrt(np.mean(np.sum((o.corners-self.expected)**2,axis=1))),1.5)

    def test_small_isolated_patch_is_not_sufficient_evidence(self):
        frame=np.full_like(self.frame,22)
        frame[235:305,240:310]=self.frame[235:305,240:310]
        self.assertIsNone(self.detector.observe(frame).corners)

    def test_blank_unrelated_and_mirrored_images_do_not_produce_control_features(self):
        for frame in (np.zeros_like(self.frame),np.full_like(self.frame,190),
                      np.random.default_rng(234).integers(0,256,self.frame.shape,np.uint8),
                      cv2.flip(self.frame,1)):
            with self.subTest(kind=int(frame.mean())):
                self.assertIsNone(self.detector.observe(frame).corners)

    def test_outline_outside_frame_is_rejected_even_with_visible_texture(self):
        expected=self.expected-[190,0]
        H=cv2.getPerspectiveTransform(self.detector.boundary,expected.astype(np.float32))
        frame=cv2.warpPerspective(self.detector.template_rgb,H,(640,480),borderValue=(22,22,22))
        o=self.detector.observe(frame)
        self.assertIsNone(o.corners)
        self.assertEqual(o.reason,"outline_out_of_frame")

    def test_cache_cannot_reuse_detection_after_frame_changes(self):
        frame=self.frame.copy()
        first=self.detector.observe(frame)
        self.assertIsNotNone(first.corners)
        frame[:]=0
        self.assertIsNone(self.detector.observe(frame).corners)

    def test_repeated_calls_are_deterministic(self):
        a=self.detector.observe(self.frame)
        self.detector.observe(np.zeros_like(self.frame))
        b=self.detector.observe(self.frame)
        np.testing.assert_array_equal(a.corners,b.corners)
        np.testing.assert_array_equal(a.template_points,b.template_points)

    def test_picture_contains_no_aruco_code(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        _,ids,_=cv2.aruco.ArucoDetector(dictionary).detectMarkers(self.detector.template_rgb)
        self.assertIsNone(ids)

    def test_invalid_input_and_settings_are_rejected(self):
        for frame in (np.zeros((480,640)),np.zeros((480,640,3),float)):
            with self.assertRaises(ValueError):
                self.detector.observe(frame)
        for override in (dict(min_inliers=3),dict(min_inliers=True),dict(ratio_test=0),
                         dict(min_template_coverage=1),dict(max_features=0),dict(detection_width_px=-1)):
            with self.subTest(override=override),self.assertRaises(ValueError):
                NaturalImagePerception(config=dict(load_natural_config(),**override))

    def test_textureless_template_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"blank.png"
            cv2.imwrite(str(path),np.zeros((384,384,3),np.uint8))
            with self.assertRaisesRegex(ValueError,"too few"):
                NaturalImagePerception(path)

    def test_saved_goal_contains_no_robot_or_target_pose(self):
        with np.load(NATURAL_REFERENCE,allow_pickle=False) as data:
            self.assertEqual(set(data.files),{"rgb","K","metadata"})
            meta=json.loads(str(data["metadata"]))
            self.assertEqual(set(meta),{"perception_mode","camera_width","camera_height","target_side_m","target_image_sha256"})
            self.assertEqual(meta["perception_mode"],"natural")
            self.assertEqual(meta["target_image_sha256"],self.detector.template_sha256)


if __name__=="__main__":
    unittest.main()
