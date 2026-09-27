"""Learned inference is checked against independent image geometry, without SIFT."""
import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from learned_perception import LearnedImagePerception,LearnedUnavailable,check_model_files,load_learned_config,MODEL_DIR
from perception import PlanarImagePerception,NATURAL_TEMPLATE

AVAILABLE=(importlib.util.find_spec("torch") is not None and importlib.util.find_spec("lightglue") is not None
           and all((MODEL_DIR/(n+".pth")).exists() for n in ("superpoint_v1","superpoint_lightglue")))

class LearnedSetupTests(unittest.TestCase):
    def test_missing_models_give_an_actionable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(LearnedUnavailable,"setup-learned.cmd"):
                check_model_files(directory)

    def test_shared_geometry_rejects_nonfinite_or_insufficient_evidence(self):
        d=PlanarImagePerception(NATURAL_TEMPLATE,load_learned_config())
        self.assertIsNone(d.fit_outline(np.zeros((3,2)),np.zeros((3,2)),(480,640,3)).corners)
        self.assertIsNone(d.fit_outline(np.full((12,2),np.nan),np.zeros((12,2)),(480,640,3)).corners)

@unittest.skipUnless(AVAILABLE,"Optional learned dependencies/models are not installed")
class LearnedPerceptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with patch("urllib.request.urlopen",side_effect=AssertionError("Inference must work offline")), \
             patch("cv2.SIFT_create",side_effect=AssertionError("Learned mode must not use SIFT")):
            cls.detector=LearnedImagePerception()
        cls.expected=np.array([[145,90],[475,105],[460,400],[125,380]],np.float32)
        H=cv2.getPerspectiveTransform(cls.detector.boundary,cls.expected)
        cls.frame=cv2.warpPerspective(cls.detector.template_rgb,H,(640,480),borderValue=(22,22,22))

    def test_real_learned_matches_recover_independent_camera_geometry(self):
        o=self.detector.observe(self.frame)
        self.assertEqual(o.reason,"accepted")
        self.assertGreaterEqual(o.inliers,12)
        self.assertGreaterEqual(o.coverage,.18)
        self.assertLess(np.sqrt(np.mean(np.sum((o.corners-self.expected)**2,axis=1))),2.)
        self.assertEqual(type(self.detector.extractor).__name__,"SuperPoint")
        self.assertEqual(type(self.detector.matcher).__name__,"LightGlue")

    def test_partial_occlusion_still_localizes_the_picture(self):
        frame=self.frame.copy();frame[100:210,145:280]=40
        o=self.detector.observe(frame)
        self.assertIsNotNone(o.corners,o.reason)
        self.assertLess(np.sqrt(np.mean(np.sum((o.corners-self.expected)**2,axis=1))),2.)

    def test_blank_wrong_mirrored_and_tiny_evidence_are_rejected(self):
        tiny=np.full_like(self.frame,22);tiny[235:305,240:310]=self.frame[235:305,240:310]
        for frame in (np.zeros_like(self.frame),np.full_like(self.frame,190),
                      np.random.default_rng(234).integers(0,256,self.frame.shape,np.uint8),
                      cv2.flip(self.frame,1),tiny):
            with self.subTest(mean=float(frame.mean())):
                self.assertIsNone(self.detector.observe(frame).corners)

    def test_outside_boundary_does_not_drive_alignment(self):
        expected=self.expected-[190,0]
        H=cv2.getPerspectiveTransform(self.detector.boundary,expected.astype(np.float32))
        frame=cv2.warpPerspective(self.detector.template_rgb,H,(640,480),borderValue=(22,22,22))
        self.assertIsNone(self.detector.observe(frame).corners)

    def test_cache_reuses_only_identical_pixels_and_failures_clear_features(self):
        d=self.detector
        d.observe(np.zeros_like(self.frame))
        with patch.object(d.matcher,"forward",wraps=d.matcher.forward) as forward:
            a=d.observe(self.frame.copy());b=d.observe(self.frame.copy())
            self.assertIs(a,b)
            self.assertEqual(forward.call_count,1)
        changed=self.frame.copy();changed[0,0]=[1,2,3]
        with patch.object(d.matcher,"forward",side_effect=RuntimeError("Simulated inference failure")):
            self.assertEqual(d.observe(changed).reason,"inference_failed")
            self.assertIsNone(d.observe(changed).corners)

    def test_repeated_inference_is_deterministic(self):
        d=self.detector
        a=d.observe(self.frame)
        d.observe(np.zeros_like(self.frame))
        b=d.observe(self.frame)
        np.testing.assert_allclose(a.corners,b.corners,rtol=0,atol=1e-5)

    def test_byte_upload_preserves_all_intensities_and_noncontiguous_input(self):
        d=self.detector
        rgb=np.tile(np.arange(256,dtype=np.uint8),(24,3,1)).transpose(0,2,1)[::2,::-1]
        self.assertFalse(rgb.flags.c_contiguous)
        tensor=d._tensor(rgb)
        self.assertEqual(tensor.device.type,d.device.type)
        self.assertEqual(tensor.dtype,d.torch.float32)
        expected=np.ascontiguousarray(rgb.transpose(2,0,1)).astype(np.float32)/255.
        old_route=d.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1))).to(
            device=d.device,dtype=d.torch.float32)/255.
        np.testing.assert_array_equal(tensor.cpu().numpy(),old_route.cpu().numpy())
        np.testing.assert_allclose(tensor.cpu().numpy(),expected,rtol=0,atol=1e-7)

    def test_cuda_graph_replay_matches_eager_and_keeps_dynamic_shape_fallback(self):
        d=self.detector
        if not d.cuda_graph_blocks:self.skipTest('CUDA graph backend unavailable on CPU')
        torch=d.torch
        with torch.inference_mode():
            features=d.extractor.extract(d._tensor(cv2.resize(self.frame,(384,288))),resize=None)
            original_blocks=list(d.matcher.transformers)
            for count in (features['keypoints'].shape[1],32):
                current={key:(value[:,:count] if key in ('keypoints','keypoint_scores','descriptors') else value)
                         for key,value in features.items()}
                for block,graph in zip(original_blocks,d.cuda_graph_blocks):block.forward=graph.forward
                try:
                    expected=d.matcher(dict(image0=d.template_features,image1=current))
                    expected={key:(value[0].cpu().numpy().copy() if key in ('matches','scores') else value)
                              for key,value in expected.items() if key in ('matches','scores','stop')}
                finally:
                    for block,graph in zip(original_blocks,d.cuda_graph_blocks):block.forward=graph
                got=d.matcher(dict(image0=d.template_features,image1=current))
                self.assertEqual(got['stop'],expected['stop'])
                np.testing.assert_array_equal(got['matches'][0].cpu().numpy(),expected['matches'])
                np.testing.assert_allclose(got['scores'][0].cpu().numpy(),expected['scores'],rtol=1e-5,atol=1e-7)
        self.assertGreater(sum(g.replays for g in d.cuda_graph_blocks),0)
        self.assertGreater(sum(g.fallbacks for g in d.cuda_graph_blocks),0)
        self.assertEqual(len(d.cuda_graph_blocks),d.matcher.conf.n_layers)

    def test_cuda_graph_buffers_are_reused_and_changed_images_change_matches(self):
        d=self.detector
        if not d.cuda_graph_blocks:self.skipTest('CUDA graph backend unavailable on CPU')
        pointers=[(tuple(t.data_ptr() for t in g.inputs),tuple(t.data_ptr() for t in g.outputs)) for g in d.cuda_graph_blocks]
        first=d.observe(self.frame)
        shifted=cv2.warpAffine(self.frame,np.float32([[1,0,12],[0,1,-7]]),(640,480),borderValue=(22,22,22))
        later=d.observe(shifted)
        self.assertIsNotNone(later.corners,later.reason)
        self.assertGreater(np.linalg.norm(later.corners-first.corners),10.)
        self.assertEqual(pointers,[(tuple(t.data_ptr() for t in g.inputs),tuple(t.data_ptr() for t in g.outputs)) for g in d.cuda_graph_blocks])

    def test_diagnostic_events_preserve_geometry_without_global_synchronization(self):
        d=self.detector
        before=d.observe(self.frame)
        d.observe(np.zeros_like(self.frame))
        d.profile_diagnostics=True
        try:
            with patch.object(d.torch.cuda,'synchronize',side_effect=AssertionError('No extra synchronization')):
                after=d.observe(self.frame)
            np.testing.assert_allclose(after.corners,before.corners,rtol=0,atol=1e-5)
            if d.device.type=='cuda':
                for name in ('extract_cuda_stream_ms','match_cuda_stream_ms','gather_cuda_stream_ms'):
                    self.assertGreaterEqual(after.stage_ms[name],0.)
        finally:
            d.profile_diagnostics=False

    def test_invalid_learned_settings_fail_before_model_loading(self):
        for override in (dict(cpu_threads=0),dict(max_keypoints=3),dict(filter_threshold=2),
                         dict(detection_threshold=float("nan")),dict(depth_confidence=2),dict(device="invalid")):
            with self.subTest(override=override),self.assertRaises(ValueError):
                LearnedImagePerception(config=dict(load_learned_config(),**override))

if __name__=="__main__":
    unittest.main()
