"""Pretrained SuperPoint + LightGlue, followed by the shared planar geometry gates.

Only the canonical picture and RGB camera frame enter this backend. Imports and
weights are lazy, so the original ArUco/SIFT installation remains usable.
"""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from perception import ROOT,NATURAL_TEMPLATE,PlanarImagePerception,Observation

MODEL_DIR=ROOT/"models"
LIGHTGLUE_COMMIT="eb42fee2d71449efb0aa5c10549752b5d75384d8"

class LearnedUnavailable(ValueError):
    pass

def load_learned_config():
    return json.loads((ROOT/"learned_feature_config.json").read_text(encoding="utf-8"))

def check_model_files(directory=MODEL_DIR):
    directory=Path(directory)
    try:
        manifest=json.loads((directory/"manifest.json").read_text(encoding="utf-8"))
        if manifest["lightglue_commit"]!=LIGHTGLUE_COMMIT:
            raise ValueError("Model manifest does not match the pinned implementation")
        expected={"superpoint_v1","superpoint_lightglue"}
        if set(manifest["weights"])!=expected:
            raise ValueError("Incomplete model manifest")
        for name in expected:
            row=manifest["weights"][name]
            path=directory/(name+".pth")
            if row["file"]!=path.name or hashlib.sha256(path.read_bytes()).hexdigest()!=row["sha256"]:
                raise ValueError(f"Model checksum mismatch: {name}")
        return manifest
    except (OSError,ValueError,KeyError) as exc:
        raise LearnedUnavailable("Learned matcher unavailable. Run setup-learned.cmd to install/check its models.") from exc

class LearnedImagePerception(PlanarImagePerception):
    mode="learned"
    name="Picture / SuperPoint + LightGlue"

    def __init__(self,template_path=NATURAL_TEMPLATE,config=None,model_dir=MODEL_DIR):
        super().__init__(template_path,load_learned_config() if config is None else config)
        c=self.config
        for key in ("max_keypoints","cpu_threads","template_extract_width_px"):
            if type(c[key]) is not int or c[key]<1 or (key=="max_keypoints" and c[key]<c["min_inliers"]):
                raise ValueError(f"Invalid {key}")
        if c["device"]!="cpu":
            raise ValueError("This tested configuration uses CPU inference")
        for key in ("detection_threshold","filter_threshold"):
            if not np.isfinite(c[key]) or not 0<c[key]<1:
                raise ValueError(f"Invalid {key}")
        for key in ("depth_confidence","width_confidence"):
            if not np.isfinite(c[key]) or not (c[key]==-1 or 0<c[key]<1):
                raise ValueError(f"Invalid {key}")
        self.model_manifest=check_model_files(model_dir)
        try:
            import torch
            from lightglue import SuperPoint,LightGlue
        except (ImportError,OSError) as exc:
            raise LearnedUnavailable("Learned matcher unavailable. Run setup-learned.cmd to install its dependencies.") from exc
        self.torch=torch
        torch.set_num_threads(c["cpu_threads"])
        # Upstream constructors request URL weights. Resolve those exact URLs to
        # checksum-verified local files with weights_only=True. Never download in
        # a GUI frame or silently substitute another detector.
        by_url={v["url"]:Path(model_dir)/v["file"] for v in self.model_manifest["weights"].values()}
        def local_weights(url,*args,**kwargs):
            if url not in by_url:
                raise LearnedUnavailable("Unexpected model requested; rerun the pinned setup")
            return torch.load(by_url[url],map_location="cpu",weights_only=True)
        try:
            with patch.object(torch.hub,"load_state_dict_from_url",side_effect=local_weights):
                self.extractor=SuperPoint(max_num_keypoints=c["max_keypoints"],
                    detection_threshold=c["detection_threshold"]).eval().to("cpu")
                self.matcher=LightGlue(features="superpoint",flash=True,mp=False,
                    depth_confidence=c["depth_confidence"],width_confidence=c["width_confidence"],
                    filter_threshold=c["filter_threshold"]).eval().to("cpu")
            with torch.inference_mode():
                self.template_features=self.extractor.extract(self._tensor(self.template_rgb),resize=c["template_extract_width_px"])
        except (RuntimeError,OSError) as exc:
            raise LearnedUnavailable("Could not load learned models. Run setup-learned.cmd to check the installation.") from exc
        if self.template_features["keypoints"].shape[1]<c["min_inliers"]:
            raise ValueError("Target picture has too few distinctive learned features")

    def _tensor(self,rgb):
        return self.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1))).float()/255.

    def _detect(self,rgb):
        torch=self.torch
        height,width=rgb.shape[:2]
        scale=min(1.,self.config["detection_width_px"]/width)
        work=rgb if scale==1 else cv2.resize(rgb,(round(width*scale),round(height*scale)),interpolation=cv2.INTER_AREA)
        try:
            with torch.inference_mode():
                current=self.extractor.extract(self._tensor(work),resize=None)
                if current["keypoints"].shape[1]<self.config["min_inliers"]:
                    return Observation(reason="too_few_features")
                output=self.matcher({"image0":self.template_features,"image1":current})
                matches=output["matches"][0].cpu().numpy()
                scores=output["scores"][0].cpu().numpy()
                src=self.template_features["keypoints"][0].cpu().numpy()[matches[:,0]]
                dst=current["keypoints"][0].cpu().numpy()[matches[:,1]]
        except RuntimeError:
            # A failed inference must never reuse the previous accepted corners.
            return Observation(reason="inference_failed")
        # extract(resize=None) uses work-image coordinates. Undo our explicit
        # resize with pixel-center geometry before fitting against camera K.
        dst=(dst+.5)/[work.shape[1]/width,work.shape[0]/height]-.5
        valid=np.isfinite(scores)&(scores>=self.config["filter_threshold"])
        src,dst,scores=src[valid],dst[valid],scores[valid]
        # Retain one correspondence per source and camera location.
        chosen=[]; used_src=set(); used_dst=set()
        for i in np.argsort(-scores,kind="stable"):
            a,b=tuple(np.round(src[i]).astype(int)),tuple(np.round(dst[i]).astype(int))
            if a not in used_src and b not in used_dst:
                chosen.append(i);used_src.add(a);used_dst.add(b)
        return self.fit_outline(src[chosen],dst[chosen],rgb.shape)
