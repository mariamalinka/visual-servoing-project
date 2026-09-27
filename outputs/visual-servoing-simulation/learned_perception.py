"""Pretrained SuperPoint + LightGlue, followed by the shared planar geometry gates.

Only the canonical picture and RGB camera frame enter this backend. Imports and
weights are lazy, so the original ArUco/SIFT installation remains usable.
"""
from __future__ import annotations
import hashlib
import json
import time
from pathlib import Path
from unittest.mock import patch
import cv2
import numpy as np
from perception import ROOT,NATURAL_TEMPLATE,PlanarImagePerception,Observation

MODEL_DIR=ROOT/"models"
LIGHTGLUE_COMMIT="eb42fee2d71449efb0aa5c10549752b5d75384d8"

class LearnedUnavailable(ValueError):
    pass

def unavailable(stage, exc):
    error = LearnedUnavailable(
        f"Learned {stage} failed: {type(exc).__name__}: {exc}. "
        "Run setup-learned.cmd; for unreadable packages use setup-learned.cmd --fresh.")
    error.summary = f"Learned {stage}: {type(exc).__name__}"
    if isinstance(exc, PermissionError) and exc.filename:
        path = Path(exc.filename)
        error.summary += f" ({path.parent.name}/{path.name})"
    return error


def load_dependencies():
    try:
        import torch
        from lightglue import SuperPoint, LightGlue
        return torch, SuperPoint, LightGlue
    except (ImportError, OSError, RuntimeError, AttributeError) as exc:
        raise unavailable("import", exc) from exc


def select_device(torch, requested):
    if requested == "cpu":
        return torch.device("cpu")
    available = torch.cuda.is_available()
    if requested == "cuda" and not available:
        raise LearnedUnavailable("NVIDIA GPU requested but CUDA is unavailable. "
                                 "Run setup-learned.cmd --cuda and check the NVIDIA driver.")
    return torch.device("cuda" if available else "cpu")


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
        raise unavailable("model check", exc) from exc

class LearnedImagePerception(PlanarImagePerception):
    mode="learned"
    name="Picture / SuperPoint + LightGlue"

    def __init__(self,template_path=NATURAL_TEMPLATE,config=None,model_dir=MODEL_DIR,*,cuda_graphs=True):
        if type(cuda_graphs) is not bool:
            raise ValueError('cuda_graphs must be boolean')
        super().__init__(template_path,load_learned_config() if config is None else config)
        c=self.config
        for key in ("max_keypoints","cpu_threads","template_extract_width_px"):
            if type(c[key]) is not int or c[key]<1 or (key=="max_keypoints" and c[key]<c["min_inliers"]):
                raise ValueError(f"Invalid {key}")
        if c["device"] not in ("auto", "cpu", "cuda"):
            raise ValueError("Device must be auto, cpu or cuda")
        for key in ("detection_threshold","filter_threshold"):
            if not np.isfinite(c[key]) or not 0<c[key]<1:
                raise ValueError(f"Invalid {key}")
        for key in ("depth_confidence","width_confidence"):
            if not np.isfinite(c[key]) or not (c[key]==-1 or 0<c[key]<1):
                raise ValueError(f"Invalid {key}")
        self.model_manifest=check_model_files(model_dir)
        torch, SuperPoint, LightGlue = load_dependencies()
        self.torch=torch
        self.device=select_device(torch,c["device"])
        self.name=f"Picture / SuperPoint + LightGlue ({self.device.type.upper()})"
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
                torch.set_num_threads(c["cpu_threads"])
                self.extractor=SuperPoint(max_num_keypoints=c["max_keypoints"],
                    detection_threshold=c["detection_threshold"]).eval().to(self.device)
                self.matcher=LightGlue(features="superpoint",flash=True,mp=False,
                    depth_confidence=c["depth_confidence"],width_confidence=c["width_confidence"],
                    filter_threshold=c["filter_threshold"]).eval().to(self.device)
            with torch.inference_mode():
                self.template_features=self.extractor.extract(self._tensor(self.template_rgb),resize=c["template_extract_width_px"])
        except (ImportError,RuntimeError,OSError,ValueError,AttributeError) as exc:
            raise unavailable("model load", exc) from exc
        if self.template_features["keypoints"].shape[1]<c["min_inliers"]:
            raise ValueError("Target picture has too few distinctive learned features")
        self.cuda_graph_blocks=[]
        graph_started=time.perf_counter()
        if cuda_graphs and self.device.type=='cuda':
            try:
                from learned_cuda_graphs import install_block_graphs
                self.cuda_graph_blocks=install_block_graphs(self)
            except (RuntimeError,ValueError,AttributeError) as exc:
                raise unavailable('CUDA graph initialization',exc) from exc
        self.execution_settings=dict(device=str(self.device),cpu_threads=torch.get_num_threads(),
            cuda_graph_blocks=len(self.cuda_graph_blocks),graph_initialization_ms=1000*(time.perf_counter()-graph_started),
            image_conversion='upload_uint8_then_float32',cudnn_benchmark=torch.backends.cudnn.benchmark,
            cudnn_deterministic=torch.backends.cudnn.deterministic,cudnn_allow_tf32=torch.backends.cudnn.allow_tf32,
            matmul_allow_tf32=torch.backends.cuda.matmul.allow_tf32)

    def _tensor(self,rgb):
        # Upload bytes before converting. Combining device and dtype conversion
        # can cast on the host, waking the CPU pool for every GPU frame. Its
        # idle spinning then competes with GPU submission and the control owner.
        # Keep the configured thread count and exactly the same float32 values.
        return self.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1))).to(
            device=self.device).to(dtype=self.torch.float32)/255.

    def _detect(self,rgb):
        torch=self.torch
        started=time.perf_counter()
        height,width=rgb.shape[:2]
        scale=min(1.,self.config["detection_width_px"]/width)
        work=rgb if scale==1 else cv2.resize(rgb,(round(width*scale),round(height*scale)),interpolation=cv2.INTER_AREA)
        try:
            with torch.inference_mode():
                graph_before=sum(g.replays for g in self.cuda_graph_blocks)
                fallback_before=sum(g.fallbacks for g in self.cuda_graph_blocks)
                events=None
                if getattr(self,'profile_diagnostics',False) and self.device.type=='cuda':
                    if not hasattr(self,'_timing_events'):
                        self._timing_events=[torch.cuda.Event(enable_timing=True) for _ in range(4)]
                    events=self._timing_events
                    events[0].record()
                prepared=time.perf_counter()
                tensor=self._tensor(work)
                uploaded=time.perf_counter()
                current=self.extractor.extract(tensor,resize=None)
                extracted=time.perf_counter()
                if events: events[1].record()
                if current["keypoints"].shape[1]<self.config["min_inliers"]:
                    return Observation(reason="too_few_features")
                output=self.matcher({"image0":self.template_features,"image1":current})
                matched=time.perf_counter()
                if events: events[2].record()
                # Gather on the device and cross the host boundary once. Do not
                # synchronize each score/point array or copy unmatched keypoints.
                matches=output["matches"][0]
                packed_device=torch.cat((self.template_features["keypoints"][0][matches[:,0]],
                    current["keypoints"][0][matches[:,1]],output["scores"][0][:,None]),dim=1)
                if events: events[3].record()
                packed=packed_device.cpu().numpy()
                src,dst,scores=packed[:,:2],packed[:,2:4],packed[:,4]
                transferred=time.perf_counter()
                self.stage_ms.update(prepare_ms=1000*(prepared-started),
                    input_upload_host_ms=1000*(uploaded-prepared),
                    extract_host_ms=1000*(extracted-prepared), match_host_ms=1000*(matched-extracted),
                    transfer_wait_ms=1000*(transferred-matched))
                self.stage_ms.update(cuda_graph_replays=sum(g.replays for g in self.cuda_graph_blocks)-graph_before,
                    cuda_graph_fallbacks=sum(g.fallbacks for g in self.cuda_graph_blocks)-fallback_before)
                # The existing blocking D2H copy completes preceding stream events.
                # Query readiness instead of adding a CUDA-wide synchronization.
                if events and all(event.query() for event in events):
                    self.stage_ms.update(extract_cuda_stream_ms=events[0].elapsed_time(events[1]),
                        match_cuda_stream_ms=events[1].elapsed_time(events[2]),
                        gather_cuda_stream_ms=events[2].elapsed_time(events[3]))
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
        rounded_src=np.rint(src).astype(np.int32)
        rounded_dst=np.rint(dst).astype(np.int32)
        for i in np.argsort(-scores,kind="stable"):
            a,b=tuple(rounded_src[i]),tuple(rounded_dst[i])
            if a not in used_src and b not in used_dst:
                chosen.append(i);used_src.add(a);used_dst.add(b)
        filtered=time.perf_counter()
        self.stage_ms["filter_ms"]=1000*(filtered-transferred)
        result=self.fit_outline(src[chosen],dst[chosen],rgb.shape)
        self.stage_ms["geometry_ms"]=1000*(time.perf_counter()-filtered)
        return result
