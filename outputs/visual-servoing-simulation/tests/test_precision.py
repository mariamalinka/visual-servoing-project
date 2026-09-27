"""Independent image warps, weak pose directions and fail-closed stop checks."""
import json
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from control import IBVSController
from perception import Observation
from precision import GoalRefinedPerception,load_precision_config,stopping_metrics,validate_stop
from simulation import ROOT

class SensitivityStopTests(unittest.TestCase):
    def setUp(self):
        self.settings=load_precision_config()['stop']
        self.K=np.array([[500.,0,320],[0,500.,240],[0,0,1.]])
        self.points=np.array([[-.12,-.12,.7],[.12,-.12,.7],[.12,.12,.7],[-.12,.12,.7]])
        self.desired=self.project(self.points)

    def project(self,points):
        p=points@self.K.T
        return p[:,:2]/p[:,2:]

    def metrics(self,corners,depths=None):
        return stopping_metrics(corners,self.desired,self.K,self.points[:,2] if depths is None else depths,self.settings)

    def weak_pixels(self):
        t=np.array([.003,0,0]);R,_=cv2.Rodrigues(np.array([0,-.003/.7,0]))
        return self.project((self.points-t)@R)

    def test_exact_reference(self):
        m=self.metrics(self.desired)
        self.assertTrue(m['candidate']);self.assertTrue(m['observable'])
        self.assertEqual(m['position_correction_mm'],0)

    def test_coupled_translation_rotation_under_one_pixel_does_not_stop(self):
        m=self.metrics(self.weak_pixels())
        self.assertLess(m['rms_error_px'],1)
        self.assertGreater(m['position_correction_mm'],2.5)
        self.assertFalse(m['candidate'])

    def test_independent_small_camera_translation_in_mm(self):
        m=self.metrics(self.project(self.points-[.0001,0,0]))
        self.assertAlmostEqual(m['position_correction_mm'],.1,places=4)
        self.assertLess(m['rotation_correction_deg'],1e-5)

    def test_independent_small_rotation_in_degrees(self):
        R,_=cv2.Rodrigues(np.array([0,0,np.deg2rad(.01)]))
        self.assertAlmostEqual(self.metrics(self.project(self.points@R))['rotation_correction_deg'],.01,places=5)

    def test_singular_geometry_is_never_a_candidate_even_at_zero_error(self):
        p=np.repeat([[320.,240.]],4,axis=0)
        m=stopping_metrics(p,p,self.K,np.ones(4),self.settings)
        self.assertFalse(m['observable']);self.assertFalse(m['candidate'])

    def test_invalid_stop_configuration_rejected(self):
        for change in ({'rms_error_px':0},{'rotation_correction_deg':float('nan')},{'position_correction_mm':True},{'extra':1}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                validate_stop(dict(self.settings,**change))

    def controller(self):
        c=json.loads((ROOT/'config.json').read_text());c['precision_stop']=self.settings.copy()
        return IBVSController(self.desired,self.K,c)

    def test_hold_resets_after_any_gate_fails(self):
        c=self.controller()
        for _ in range(10):
            sample=c.update(self.desired,np.eye(6),1/30)
            self.assertTrue(sample.stop_candidate);self.assertEqual(sample.status,'running')
        sample=c.update(self.weak_pixels(),np.eye(6),1/30)
        self.assertFalse(sample.stop_candidate);self.assertIsNone(c.below_since)
        self.assertTrue(sample.velocity.any())
        for _ in range(15):
            self.assertEqual(c.update(self.desired,np.eye(6),1/30).status,'running')
        self.assertEqual(c.update(self.desired,np.eye(6),1/30).status,'converged')

    def test_bad_depth_cannot_pass_at_zero_pixel_error(self):
        with patch('control.estimate_depths',side_effect=ValueError('bad depth')):
            sample=self.controller().update(self.desired,np.eye(6),1/30)
        self.assertEqual(sample.status,'invalid_depth');self.assertFalse(sample.velocity.any())

    def test_sensitivity_failure_brakes_and_stays_terminal(self):
        c=self.controller();c.precision_stop=dict(self.settings,min_scaled_singular_value=1000)
        sample=c.update(self.desired,np.eye(6),1/30)
        self.assertEqual(sample.status,'poor_sensitivity');self.assertFalse(sample.velocity.any())
        self.assertEqual(c.update(self.desired,np.eye(6),1/30).status,'poor_sensitivity')

class ReferenceRefinementTests(unittest.TestCase):
    def setUp(self):
        self.settings=load_precision_config()
        gray=cv2.GaussianBlur(np.random.default_rng(51).integers(0,256,(280,360),np.uint8),(3,3),.8)
        self.rgb=cv2.cvtColor(gray,cv2.COLOR_GRAY2RGB)
        self.goal=np.array([[70.,50],[280,50],[280,230],[70,230]],np.float32)
        self.coarse=self.goal.copy()
        class Detector:
            name='chosen detector';mode='natural'
            def observe(inner,rgb):return Observation(corners=None if self.coarse is None else self.coarse.copy(),reason='accepted')
        self.refiner=GoalRefinedPerception(Detector(),self.rgb,self.goal,self.settings)

    def test_known_projective_warp_refines_biased_corners(self):
        H=np.array([[1.001,.001,.45],[-.0007,.999,.3],[.000003,-.000002,1]],np.float32)
        image=cv2.warpPerspective(self.rgb,H,(360,280))
        true=cv2.perspectiveTransform(self.goal[:,None],H).reshape(4,2)
        self.coarse=true+np.array([[.5,-.4],[-.4,.5],[.5,.4],[-.5,-.3]],np.float32)
        o=self.refiner.observe(image)
        self.assertEqual(o.refinement,'refined');self.assertGreater(o.correlation,self.settings['min_correlation'])
        self.assertLess(np.max(np.linalg.norm(o.corners-true,axis=1)),.1)

    def test_bounded_pyramid_preserves_known_projective_motion(self):
        for shift in (1.,3.,5.):
            H=np.array([[1.002,.002,shift],[-.001,.998,-shift/2],
                        [.000008,-.000009,1]],np.float32)
            image=cv2.warpPerspective(self.rgb,H,(360,280))
            true=cv2.perspectiveTransform(self.goal[:,None],H).reshape(4,2)
            self.coarse=true+np.array([[1.,-.8],[-.8,1.],[.8,.8],[-1.,-.8]],np.float32)
            with self.subTest(shift=shift):
                observed=self.refiner.observe(image)
                self.assertEqual(observed.refinement,'refined')
                self.assertLess(np.max(np.linalg.norm(observed.corners-true,axis=1)),.1)

    def test_cached_result_does_not_report_previous_expensive_stage_times(self):
        self.refiner.observe(self.rgb)
        cached=self.refiner.observe(self.rgb)
        self.assertEqual(cached.stage_ms,{'cache_hit':True})
        self.assertIsNotNone(cached.corners)

    def test_identity_cancels_detector_bias(self):
        self.coarse=self.goal+.5;o=self.refiner.observe(self.rgb)
        self.assertEqual(o.refinement,'refined')
        np.testing.assert_allclose(o.corners,self.goal,atol=.06)

    def test_marker_keeps_its_existing_subpixel_corners(self):
        self.refiner.base.mode="aruco"
        self.coarse=self.goal+.5
        with patch("precision.cv2.findTransformECC",side_effect=AssertionError("Marker uses its own corners")):
            o=self.refiner.observe(self.rgb)
        np.testing.assert_array_equal(o.corners,self.coarse)
        self.assertEqual(o.refinement,"coarse")

    def test_far_target_keeps_acquisition_measurement(self):
        self.coarse=self.goal+20;o=self.refiner.observe(self.rgb)
        self.assertEqual(o.refinement,'coarse');np.testing.assert_array_equal(o.corners,self.coarse)

    def test_missing_detection_never_reuses_corners(self):
        self.refiner.observe(self.rgb);self.coarse=None
        changed=self.rgb.copy();changed[0,0]=0
        self.assertIsNone(self.refiner.observe(changed).corners)

    def test_blank_unrelated_near_images_rejected(self):
        for image in [np.zeros_like(self.rgb),np.random.default_rng(17).integers(0,256,self.rgb.shape,np.uint8)]:
            with self.subTest(mean=image.mean()):
                self.assertIsNone(self.refiner.observe(image).corners)

    def test_ecc_failure_is_closed(self):
        with patch('precision.cv2.findTransformECC',side_effect=cv2.error('no convergence')):
            o=self.refiner.observe(self.rgb)
        self.assertIsNone(o.corners);self.assertEqual(o.reason,'refinement_rejected')

    def test_large_refinement_jump_rejected(self):
        with patch('precision.cv2.findTransformECC',return_value=(1.,np.array([[1,0,10],[0,1,0],[0,0,1]],np.float32))):
            self.assertIsNone(self.refiner.observe(self.rgb).corners)

    def test_teaching_invalidates_cache(self):
        old=self.refiner.observe(self.rgb)
        shifted=cv2.warpAffine(self.rgb,np.float32([[1,0,2],[0,1,0]]),(360,280))
        self.coarse=self.goal+[2,0];self.refiner.set_goal(shifted,self.coarse)
        new=self.refiner.observe(shifted)
        self.assertIsNot(old,new);np.testing.assert_allclose(new.corners,self.coarse,atol=.02)

if __name__=='__main__':unittest.main()
