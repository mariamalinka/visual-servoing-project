"""GPU selection, portable CPU operation and installer flavor preservation."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from learned_perception import select_device, LearnedUnavailable, LearnedImagePerception, load_learned_config
import setup_learned


class DeviceSelectionTests(unittest.TestCase):
    def runtime(self,available):
        return SimpleNamespace(device=lambda name:name,cuda=SimpleNamespace(is_available=Mock(return_value=available)))

    def test_auto_uses_gpu_when_available_and_cpu_otherwise(self):
        self.assertEqual(select_device(self.runtime(True),"auto"),"cuda")
        self.assertEqual(select_device(self.runtime(False),"auto"),"cpu")

    def test_explicit_cpu_does_not_initialize_cuda(self):
        runtime=self.runtime(True)
        self.assertEqual(select_device(runtime,"cpu"),"cpu")
        runtime.cuda.is_available.assert_not_called()

    def test_explicit_cuda_does_not_silently_run_on_cpu(self):
        with self.assertRaisesRegex(LearnedUnavailable,"setup-learned.cmd --cuda"):
            select_device(self.runtime(False),"cuda")

    def test_setup_preserves_existing_cuda_and_honors_explicit_overrides(self):
        with patch("setup_learned.importlib.metadata.version",return_value="2.13.0+cu126"):
            self.assertEqual(setup_learned.runtime_requirements().name,"requirements-learned-cuda.txt")
            self.assertEqual(setup_learned.runtime_requirements(cpu=True).name,"requirements-learned.txt")
        with patch("setup_learned.importlib.metadata.version",return_value="2.13.0+cpu"):
            self.assertEqual(setup_learned.runtime_requirements().name,"requirements-learned.txt")
            self.assertEqual(setup_learned.runtime_requirements(cuda=True).name,"requirements-learned-cuda.txt")


    def test_license_only_metadata_cannot_make_setup_downgrade_cuda(self):
        orphan=SimpleNamespace(metadata={},version=None)
        valid=SimpleNamespace(metadata={"Name":"torch"},version="2.13.0+cu126")
        with patch("setup_learned.importlib.metadata.version",return_value=None), \
             patch("setup_learned.importlib.metadata.distributions",return_value=[orphan,valid]):
            self.assertEqual(setup_learned.runtime_requirements().name,"requirements-learned-cuda.txt")


class DeviceInferenceTests(unittest.TestCase):
    def test_cpu_remains_usable_and_keeps_features_on_cpu(self):
        # Optional-dependency availability follows the existing learned tests.
        from test_learned_perception import AVAILABLE
        if not AVAILABLE:self.skipTest("Optional learned dependencies/models missing")
        import cv2
        import numpy as np
        detector=LearnedImagePerception(config=dict(load_learned_config(),device="cpu"))
        self.assertEqual(next(detector.extractor.parameters()).device.type,"cpu")
        self.assertEqual(next(detector.matcher.parameters()).device.type,"cpu")
        self.assertEqual(detector.template_features["keypoints"].device.type,"cpu")
        expected=np.array([[145,90],[475,105],[460,400],[125,380]],np.float32)
        frame=cv2.warpPerspective(detector.template_rgb,cv2.getPerspectiveTransform(detector.boundary,expected),(640,480),borderValue=(22,22,22))
        result=detector.observe(frame)
        self.assertIsNotNone(result.corners,result.reason)
        self.assertLess(np.sqrt(np.mean(np.sum((result.corners-expected)**2,axis=1))),2.)

    def test_auto_places_both_models_and_template_on_available_device(self):
        from test_learned_perception import AVAILABLE
        if not AVAILABLE:self.skipTest("Optional learned dependencies/models missing")
        import torch
        detector=LearnedImagePerception(config=dict(load_learned_config(),device="auto"))
        expected="cuda" if torch.cuda.is_available() else "cpu"
        self.assertEqual(detector.device.type,expected)
        self.assertEqual(next(detector.extractor.parameters()).device.type,expected)
        self.assertEqual(next(detector.matcher.parameters()).device.type,expected)
        self.assertEqual(detector.template_features["keypoints"].device.type,expected)
        self.assertEqual(detector._tensor(detector.template_rgb).device.type,expected)


if __name__=="__main__":unittest.main()
