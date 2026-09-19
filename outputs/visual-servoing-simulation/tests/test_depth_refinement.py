"""Noisy independent geometry exercises refinement without weakening acceptance."""
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from control import estimate_depths,marker_object_points

class DepthRefinementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=json.loads((Path(__file__).parent/"fixtures/learned-depth-noise.json").read_text(encoding="utf-8"))

    def test_noisy_near_frontal_square_recovers_independent_depth(self):
        f=self.fixture
        z=estimate_depths(np.array(f["corners"]),np.array(f["K"]),f["side_m"],3.)
        np.testing.assert_allclose(z,f["expected_depths"],rtol=0,atol=.002)

    def test_refinement_does_not_bypass_the_residual_threshold(self):
        f=self.fixture
        with self.assertRaises(ValueError):
            estimate_depths(np.array(f["corners"]),np.array(f["K"]),f["side_m"],1e-6)

    def test_already_valid_depth_uses_the_original_numeric_path(self):
        K=np.array(self.fixture["K"])
        obj=marker_object_points(.24);r=np.array([3.,.2,-.1]);t=np.array([.02,.01,.7])
        pixels=cv2.projectPoints(obj,r,t,K,None)[0].reshape(4,2)
        expected=(obj@cv2.Rodrigues(r)[0].T+t)[:,2]
        with patch("cv2.solvePnPRefineLM",side_effect=AssertionError("Valid initial estimates stay unchanged")):
            z=estimate_depths(pixels,K,.24,3.)
        np.testing.assert_allclose(z,expected,rtol=0,atol=1e-7)

if __name__=="__main__":unittest.main()
