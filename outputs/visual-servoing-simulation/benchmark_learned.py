"""Compare uncached CPU/GPU inference on identical saved and independent images."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
import torch
from learned_perception import LearnedImagePerception, load_learned_config
from perception import ROOT, NATURAL_REFERENCE


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--samples",type=int,default=12)
    args=parser.parse_args()
    if args.samples<3: parser.error("Use at least 3 samples")
    output=args.output or ROOT/"results/learned/performance"/datetime.now().strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True,exist_ok=False)
    config=load_learned_config()
    template=cv2.cvtColor(cv2.imread(str(ROOT/"assets/natural-target.png")),cv2.COLOR_BGR2RGB)
    size=config["template_size_px"]
    boundary=np.array([[-.5,-.5],[size-.5,-.5],[size-.5,size-.5],[-.5,size-.5]],np.float32)
    expected=np.array([[145,90],[475,105],[460,400],[125,380]],np.float32)
    frame=cv2.warpPerspective(template,cv2.getPerspectiveTransform(boundary,expected),(640,480),borderValue=(22,22,22))
    occluded=frame.copy();occluded[100:210,145:280]=40
    probes={"saved_camera":(np.load(NATURAL_REFERENCE)["rgb"],None),
            "perspective":(frame,expected),"occluded":(occluded,expected),
            "dimmed":((frame.astype(np.float32)*.55).astype(np.uint8),expected),
            "blank":(np.zeros_like(frame),None)}
    devices=["cpu"]+(["cuda"] if torch.cuda.is_available() else [])
    report=dict(torch=torch.__version__,gpu=torch.cuda.get_device_name() if torch.cuda.is_available() else None,
                config=config,samples_per_probe=args.samples,cache="disabled for each timed call",
                includes="image transfer, extraction, matching, CPU coordinates and geometry; excludes GUI rendering",
                source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
                               ("learned_perception.py","perception.py","learned_feature_config.json")},rows=[])
    for device in devices:
        start=time.perf_counter()
        detector=LearnedImagePerception(config=dict(config,device=device))
        print(device,"initialization",round(time.perf_counter()-start,2),"s",flush=True)
        assert detector.template_features["keypoints"].device.type==device
        for name,(rgb,truth) in probes.items():
            for _ in range(3): detector._detect(rgb)
            samples=[];observations=[]
            for _ in range(args.samples):
                detector._last_rgb=None
                if device=="cuda":torch.cuda.synchronize()
                start=time.perf_counter();observation=detector.observe(rgb)
                if device=="cuda":torch.cuda.synchronize()
                samples.append(1000*(time.perf_counter()-start));observations.append(observation)
            observation=observations[-1]
            row=dict(device=device,probe=name,median_ms=float(np.median(samples)),p95_ms=float(np.percentile(samples,95)),
                     min_ms=min(samples),max_ms=max(samples),inliers=observation.inliers,reason=observation.reason,
                     outline_rms_px=None if truth is None or observation.corners is None else
                     float(np.sqrt(np.mean(np.sum((observation.corners-truth)**2,axis=1)))))
            assert (observation.corners is None)==(name=="blank"),row
            report["rows"].append(row)
            print(json.dumps(row),flush=True)
        del detector
    (output/"benchmark.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print("Saved",output,flush=True)


if __name__=="__main__":main()
