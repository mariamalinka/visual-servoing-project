"""Interchangeable ArUco and natural planar-picture measurements.

Natural mode uses SIFT correspondences, ratio filtering and a RANSAC homography.
Its four output points are projected image-boundary corners, NOT ArUco detections.
They preserve point identities for the existing four-point IBVS controller.
Only RGB pixels and the canonical picture are available to this matcher.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
NATURAL_TEMPLATE=ROOT/"assets/natural-target.png"
NATURAL_REFERENCE=ROOT/"reference/natural_goal.npz"


@dataclass
class Observation:
    corners: np.ndarray | None = None
    reason: str = "not_detected"
    matches: int = 0
    inliers: int = 0
    inlier_ratio: float = 0.
    coverage: float = 0.
    reprojection_rms_px: float | None = None
    processing_ms: float = 0.
    template_points: np.ndarray = field(default_factory=lambda:np.empty((0,2)))
    image_points: np.ndarray = field(default_factory=lambda:np.empty((0,2)))


class ArucoPerception:
    mode="aruco"
    name="ArUco"
    def __init__(self,detector):
        self.detector=detector

    def observe(self,rgb):
        start=time.perf_counter()
        corners=self.detector(rgb)
        return Observation(corners=corners,reason="accepted" if corners is not None else "not_detected",
                           matches=0 if corners is None else 4,inliers=0 if corners is None else 4,
                           processing_ms=1000*(time.perf_counter()-start))

    def reference_config(self,config):
        return config


def load_natural_config():
    return json.loads((ROOT/"natural_feature_config.json").read_text(encoding="utf-8"))


class PlanarImagePerception:
    """Image-only template, cache, homography gates and inspection display."""
    def __init__(self,template_path,config):
        self.config=dict(config)
        c=self.config
        for key in ("template_size_px","min_inliers","detection_width_px"):
            if type(c[key]) is not int or c[key] < (4 if key=="min_inliers" else 16):
                raise ValueError(f"Invalid {key}")
        for key in ("ransac_reprojection_px","max_reprojection_rms_px",
                    "min_projected_area_px","max_side_ratio"):
            if not np.isfinite(c[key]) or c[key]<=0:
                raise ValueError(f"Invalid {key}")
        for key in ("min_inlier_ratio","min_template_coverage"):
            if not np.isfinite(c[key]) or not 0<c[key]<1:
                raise ValueError(f"Invalid {key}")
        if not np.isfinite(c["image_margin_px"]) or c["image_margin_px"]<0:
            raise ValueError("Invalid image margin")
        self.template_path=Path(template_path)
        raw=self.template_path.read_bytes()
        self.template_sha256=hashlib.sha256(raw).hexdigest()
        bgr=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError("Cannot read the target picture")
        self.template_rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
        size=c["template_size_px"]
        if self.template_rgb.shape!=(size,size,3):
            raise ValueError(f"Target picture must be {size}x{size}")
        # Pixel-edge coordinates span the entire 384-pixel picture, which occupies
        # the same known physical square as the marker. No corner is detected by code.
        self.boundary=np.array([[-.5,-.5],[size-.5,-.5],[size-.5,size-.5],[-.5,size-.5]],np.float32)
        self._last_rgb=None
        self._last_result=None

    def reference_config(self,config):
        return dict(config,perception_mode="natural",target_image_sha256=self.template_sha256)

    def observe(self,rgb):
        rgb=np.asarray(rgb)
        if rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8:
            raise ValueError("Expected an RGB uint8 camera frame")
        if self._last_rgb is not None and np.array_equal(rgb,self._last_rgb):
            return self._last_result
        start=time.perf_counter()
        result=Observation()
        try:
            result=self._detect(rgb)
        except (cv2.error,np.linalg.LinAlgError):
            result.reason="geometry_failure"
        result.processing_ms=1000*(time.perf_counter()-start)
        self._last_rgb=rgb.copy()
        self._last_result=result
        return result

    def fit_outline(self,src,dst,image_shape,result=None):
        """Shared geometric validation for classical and learned correspondences."""
        c=self.config
        result=Observation() if result is None else result
        src,dst=np.asarray(src,np.float32),np.asarray(dst,np.float32)
        result.matches=len(src)
        if src.shape!=dst.shape or src.ndim!=2 or src.shape[1]!=2 or not np.isfinite(src).all() or not np.isfinite(dst).all():
            result.reason="invalid_correspondences"
            return result
        if len(src)<c["min_inliers"]:
            result.reason="too_few_matches"
            return result
        cv2.setRNGSeed(0)
        H,mask=cv2.findHomography(src,dst,cv2.RANSAC,c["ransac_reprojection_px"],maxIters=2000,confidence=.995)
        if H is None or mask is None or not np.isfinite(H).all():
            result.reason="homography_failed"
            return result
        keep=mask.ravel().astype(bool)
        result.inliers=int(keep.sum())
        result.inlier_ratio=result.inliers/len(src)
        result.template_points=src[keep]
        result.image_points=dst[keep]
        if result.inliers<c["min_inliers"] or result.inlier_ratio<c["min_inlier_ratio"]:
            result.reason="weak_consensus"
            return result
        H,_=cv2.findHomography(src[keep],dst[keep],0)
        if H is None or not np.isfinite(H).all():
            result.reason="homography_failed"
            return result
        mapped=cv2.perspectiveTransform(src[keep,None,:],H).reshape(-1,2)
        result.reprojection_rms_px=float(np.sqrt(np.mean(np.sum((mapped-dst[keep])**2,axis=1))))
        if not np.isfinite(result.reprojection_rms_px) or result.reprojection_rms_px>c["max_reprojection_rms_px"]:
            result.reason="large_residual"
            return result
        result.coverage=float(cv2.contourArea(cv2.convexHull(src[keep]))/c["template_size_px"]**2)
        if result.coverage<c["min_template_coverage"]:
            result.reason="clustered_matches"
            return result
        corners=cv2.perspectiveTransform(self.boundary[:,None,:],H).reshape(4,2)
        denominator=np.column_stack((self.boundary,np.ones(4)))@H[2]
        if (not np.isfinite(corners).all() or np.any(np.abs(denominator)<1e-6)
                or not (np.all(denominator>0) or np.all(denominator<0))
                or not cv2.isContourConvex(corners)):
            result.reason="invalid_outline"
            return result
        area=cv2.contourArea(corners,oriented=True)
        sides=np.linalg.norm(np.roll(corners,-1,axis=0)-corners,axis=1)
        if area<c["min_projected_area_px"] or np.min(sides)<5 or np.max(sides)/np.min(sides)>c["max_side_ratio"]:
            result.reason="invalid_outline"
            return result
        height,width=image_shape[:2]
        margin=c["image_margin_px"]
        if (np.any(corners<[margin,margin]) or np.any(corners>[width-1-margin,height-1-margin])):
            result.reason="outline_out_of_frame"
            return result
        result.corners=corners.copy()
        result.reason="accepted"
        return result

    def match_view(self,rgb,observation):
        """An inspection view of accepted geometric inliers; never controller input."""
        left=self.template_rgb
        height=max(left.shape[0],rgb.shape[0])
        canvas=np.full((height,left.shape[1]+rgb.shape[1],3),20,np.uint8)
        canvas[:left.shape[0],:left.shape[1]]=left
        canvas[:rgb.shape[0],left.shape[1]:]=rgb
        for a,b in zip(observation.template_points[:60],observation.image_points[:60]):
            p=tuple(np.round(a).astype(int))
            q=tuple(np.round(b+[left.shape[1],0]).astype(int))
            cv2.line(canvas,p,q,(77,214,192),1,cv2.LINE_AA)
            cv2.circle(canvas,p,3,(255,194,102),-1)
            cv2.circle(canvas,q,3,(77,214,192),-1)
        return canvas


class NaturalImagePerception(PlanarImagePerception):
    mode="natural"
    name="Picture / SIFT"
    def __init__(self,template_path=NATURAL_TEMPLATE,config=None):
        super().__init__(template_path,load_natural_config() if config is None else config)
        c=self.config
        if type(c["max_features"]) is not int or c["max_features"]<16:
            raise ValueError("Invalid max_features")
        if not np.isfinite(c["contrast_threshold"]) or c["contrast_threshold"]<=0:
            raise ValueError("Invalid contrast_threshold")
        if not np.isfinite(c["ratio_test"]) or not 0<c["ratio_test"]<1:
            raise ValueError("Invalid ratio_test")
        self.sift=cv2.SIFT_create(nfeatures=c["max_features"],contrastThreshold=c["contrast_threshold"])
        self.keypoints,self.descriptors=self.sift.detectAndCompute(cv2.cvtColor(self.template_rgb,cv2.COLOR_RGB2GRAY),None)
        if self.descriptors is None or len(self.keypoints)<c["min_inliers"]:
            raise ValueError("Target picture has too few distinctive features")
        self.template_points=np.array([p.pt for p in self.keypoints],np.float32)
        self.matcher=cv2.BFMatcher(cv2.NORM_L2)

    def _detect(self,rgb):
        c=self.config
        result=Observation()
        height,width=rgb.shape[:2]
        scale=min(1.,c["detection_width_px"]/width)
        work=rgb if scale==1 else cv2.resize(rgb,(round(width*scale),round(height*scale)),interpolation=cv2.INTER_AREA)
        keypoints,descriptors=self.sift.detectAndCompute(cv2.cvtColor(work,cv2.COLOR_RGB2GRAY),None)
        # OpenCV resize uses pixel centers. Return all measurements in the original
        # camera coordinates, so K and the existing control law remain unchanged.
        image_points=np.array([((np.array(p.pt)+.5)/[work.shape[1]/width,work.shape[0]/height]-.5)
                               for p in keypoints],np.float32)
        if descriptors is None or len(keypoints)<2:
            result.reason="too_few_features"
            return result
        pairs=self.matcher.knnMatch(self.descriptors,descriptors,k=2)
        candidates=[p[0] for p in pairs if len(p)==2 and p[0].distance<c["ratio_test"]*p[1].distance]
        # Several scale/orientation copies must not inflate the evidence count.
        candidates.sort(key=lambda p:(p.distance,p.queryIdx,p.trainIdx))
        unique=[]
        used=set()
        template_used=set()
        for m in candidates:
            # SIFT can describe the same location with different orientations.
            tp=tuple(np.round(self.template_points[m.queryIdx]).astype(int))
            ip=tuple(np.round(image_points[m.trainIdx]).astype(int))
            if tp not in template_used and ip not in used:
                unique.append(m); template_used.add(tp); used.add(ip)
        result.matches=len(unique)
        if len(unique)<c["min_inliers"]:
            result.reason="too_few_matches"
            return result
        src=np.array([self.template_points[m.queryIdx] for m in unique],np.float32)
        dst=np.array([image_points[m.trainIdx] for m in unique],np.float32)
        return self.fit_outline(src,dst,rgb.shape,result)
