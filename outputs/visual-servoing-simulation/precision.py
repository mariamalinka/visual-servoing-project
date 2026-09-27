"""Reference-image refinement and local sensitivity for final visual alignment.

No simulated pose, joint goal or scene geometry enters this module. The local
motion estimate uses measured image coordinates, assumed K and estimated depth.
"""
from __future__ import annotations
from dataclasses import replace
import json
from pathlib import Path
import time
import cv2
import numpy as np


def load_precision_config():
    c=json.loads(Path(__file__).with_name('precision_config.json').read_text(encoding='utf-8-sig'))
    if type(c['enabled']) is not bool:
        raise ValueError('Precision enabled must be boolean')
    for key in ('entry_error_px','ecc_epsilon','min_correlation','max_refinement_shift_px'):
        if isinstance(c[key],bool) or not np.isfinite(c[key]) or c[key]<=0:
            raise ValueError(f'Invalid precision {key}')
    for key in ('ecc_iterations','ecc_coarse_iterations','roi_margin_px'):
        if type(c[key]) is not int or c[key]<1:
            raise ValueError(f'Invalid precision {key}')
    if isinstance(c['ecc_coarse_scale'],bool) or not np.isfinite(c['ecc_coarse_scale']) or not 0<c['ecc_coarse_scale']<=1:
        raise ValueError('ECC coarse scale must be in (0, 1]')
    if c['min_correlation']>1:
        raise ValueError('Correlation must be at most one')
    validate_stop(c['stop'])
    if c['entry_error_px']<=c['stop']['max_corner_error_px']:
        raise ValueError('Refinement must start before the stopping region')
    return c


def validate_stop(c):
    keys={'rms_error_px','max_corner_error_px','position_correction_mm',
          'rotation_correction_deg','min_scaled_singular_value'}
    if set(c)!=keys or any(isinstance(v,bool) or not np.isfinite(v) or v<=0 for v in c.values()):
        raise ValueError('Precision stopping limits must be positive finite numbers')


def stopping_metrics(corners,desired,K,depths,settings):
    """Undamped local inverse prevents damping from hiding weak image directions.

    Singular values refer to pixel change for motion scaled by the declared
    translation/rotation limits; metres and radians are never mixed unscaled.
    This estimates a residual camera correction, not guaranteed physical error.
    """
    from control import interaction_matrix,normalized_points
    delta=np.asarray(corners,dtype=float)-np.asarray(desired,dtype=float)
    L=interaction_matrix(normalized_points(corners,K),depths)
    # Convert normalized-image velocities into pixels, including intrinsic skew.
    pixel_L=np.einsum('ij,njk->nik',np.asarray(K)[:2,:2],L.reshape(-1,2,6)).reshape(-1,6)
    scales=np.r_[np.full(3,settings['position_correction_mm']/1000),
                 np.full(3,np.deg2rad(settings['rotation_correction_deg']))]
    A=pixel_L*scales
    u,s,vt=np.linalg.svd(A,full_matrices=False)
    observable=bool(len(s)==6 and s[-1]>=settings['min_scaled_singular_value'])
    correction=(vt.T@((u.T@delta.ravel())/np.maximum(s,1e-15)))*scales
    position=float(1000*np.linalg.norm(correction[:3]))
    angle=float(np.rad2deg(np.linalg.norm(correction[3:])))
    rms=float(np.sqrt(np.mean(np.sum(delta**2,axis=1))))
    maximum=float(np.max(np.linalg.norm(delta,axis=1)))
    candidate=bool(observable and rms<settings['rms_error_px'] and maximum<settings['max_corner_error_px']
                   and position<settings['position_correction_mm'] and angle<settings['rotation_correction_deg'])
    return dict(candidate=candidate,observable=observable,position_correction_mm=position,
                rotation_correction_deg=angle,scaled_min_singular_value=float(s[-1]),
                rms_error_px=rms,max_corner_error_px=maximum)


class GoalRefinedPerception:
    """Keep the selected detector as the acquisition gate; refine near its image goal.

    Each result starts from a fresh detector result and an image-to-image warp.
    A rejected local registration supplies no corners, never stale/coarse success.
    """
    def __init__(self,base,rgb,corners,settings):
        self.base=base
        self.settings=settings
        self.set_goal(rgb,corners)

    def __getattr__(self,name):
        return getattr(self.base,name)

    def set_goal(self,rgb,corners):
        self.goal_rgb=np.asarray(rgb).copy()
        self.goal=np.asarray(corners,dtype=np.float32).copy()
        margin=self.settings['roi_margin_px']
        height,width=rgb.shape[:2]
        x0,y0=np.maximum(0,np.floor(self.goal.min(axis=0)-margin)).astype(int)
        x1,y1=np.minimum([width,height],np.ceil(self.goal.max(axis=0)+margin+1)).astype(int)
        self.roi=(int(x0),int(y0),int(x1),int(y1))
        self.gray=cv2.cvtColor(rgb[y0:y1,x0:x1],cv2.COLOR_RGB2GRAY)
        self.origin=np.array([x0,y0],np.float32)
        self._mask_kernel=np.ones((9,9),np.uint8)
        # Prepare the immutable goal pyramid once, including pixel-center geometry.
        h,w=self.gray.shape
        scale=self.settings.get('ecc_coarse_scale',1.)
        self.coarse_size=(max(8,round(w*scale)),max(8,round(h*scale)))
        sx,sy=self.coarse_size[0]/w,self.coarse_size[1]/h
        self._scale=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]],np.float32)
        self._unscale=np.linalg.inv(self._scale).astype(np.float32)
        self.coarse_gray=cv2.resize(self.gray,self.coarse_size,interpolation=cv2.INTER_AREA)
        self.local_goal=self.goal-self.origin
        self._rgb=None;self._result=None

    def observe(self,rgb):
        # Cached pixels are measurement reuse only; TimedCamera owns freshness.
        if self._rgb is not None and np.array_equal(rgb,self._rgb):
            return replace(self._result, stage_ms={"cache_hit": True})
        started=time.perf_counter()
        profile=getattr(self,'profile_diagnostics',False)
        start_cpu=time.thread_time() if profile else 0.
        start_process=time.process_time() if profile else 0.
        coarse=self.base.observe(rgb)
        coarse_finished=time.perf_counter()
        coarse_cpu=time.thread_time() if profile else 0.
        coarse_process=time.process_time() if profile else 0.
        result=replace(coarse,refinement='coarse',correlation=None)
        if coarse.corners is not None and self.base.mode != "aruco":
            error=np.sqrt(np.mean(np.sum((coarse.corners-self.goal)**2,axis=1)))
            if error<=self.settings['entry_error_px']:
                result=self._refine(rgb,coarse)
        refined_cpu=time.thread_time() if profile else 0.
        refined_process=time.process_time() if profile else 0.
        stages=dict(coarse.stage_ms)
        stages.update(coarse_ms=1000*(coarse_finished-started),
                      refinement_ms=1000*(time.perf_counter()-coarse_finished))
        if profile:
            stages.update(coarse_thread_cpu_ms=1000*(coarse_cpu-start_cpu),
                coarse_process_cpu_ms=1000*(coarse_process-start_process),
                refinement_thread_cpu_ms=1000*(refined_cpu-coarse_cpu),
                refinement_process_cpu_ms=1000*(refined_process-coarse_process))
        result=replace(result,processing_ms=1000*(time.perf_counter()-started),stage_ms=stages)
        # Natural/learned already retain an immutable copy for their cache.
        cached=getattr(self.base,'_last_rgb',None)
        self._rgb=cached if cached is not None else np.asarray(rgb).copy()
        self._result=result
        return result

    def _refine(self,rgb,coarse):
        rejected=replace(coarse,corners=None,reason='refinement_rejected',refinement='rejected',correlation=None)
        x0,y0,x1,y1=self.roi
        gray=cv2.cvtColor(rgb[y0:y1,x0:x1],cv2.COLOR_RGB2GRAY)
        current=np.float32(coarse.corners)-self.origin
        mask=np.zeros(gray.shape,np.uint8)
        cv2.fillConvexPoly(mask,np.int32(current),255)
        mask=cv2.erode(mask,self._mask_kernel)
        if np.count_nonzero(mask)<400 or np.std(gray[mask>0])<5:
            return rejected
        try:
            H=cv2.getPerspectiveTransform(self.local_goal,current).astype(np.float32)
            if self.settings.get('ecc_coarse_scale',1.) < 1:
                small=cv2.resize(gray,self.coarse_size,interpolation=cv2.INTER_AREA)
                small_mask=cv2.resize(mask,self.coarse_size,interpolation=cv2.INTER_NEAREST)
                low=(self._scale@H@self._unscale).astype(np.float32)
                _,low=cv2.findTransformECC(self.coarse_gray,small,low,cv2.MOTION_HOMOGRAPHY,
                    (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,
                     self.settings['ecc_coarse_iterations'],self.settings['ecc_epsilon']),small_mask,1)
                H=(self._unscale@low@self._scale).astype(np.float32)
            cc,H=cv2.findTransformECC(self.gray,gray,H,cv2.MOTION_HOMOGRAPHY,
                (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,self.settings['ecc_iterations'],self.settings['ecc_epsilon']),mask,1)
            refined=cv2.perspectiveTransform(self.local_goal[:,None,:],H).reshape(4,2)+self.origin
        except cv2.error:
            return rejected
        displacement=np.max(np.linalg.norm(refined-coarse.corners,axis=1))
        denominator=np.column_stack((self.local_goal,np.ones(4)))@H[2]
        if (not np.isfinite(refined).all() or not np.isfinite(cc) or cc<self.settings['min_correlation']
            or displacement>self.settings['max_refinement_shift_px'] or not cv2.isContourConvex(refined)
            or cv2.contourArea(refined,oriented=True)<=0 or np.any(denominator<=1e-6)
            or np.any(refined<1) or np.any(refined>np.array([rgb.shape[1]-2,rgb.shape[0]-2]))):
            return rejected
        return replace(coarse,corners=refined,reason='accepted',refinement='refined',correlation=float(cc))
