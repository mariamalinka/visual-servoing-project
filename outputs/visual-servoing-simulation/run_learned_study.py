"""Paired SIFT / SuperPoint-LightGlue evaluation with frozen scenes and geometry probes."""
import argparse
from datetime import datetime,timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
import cv2
import numpy as np
from PIL import Image
from benchmark import read_json,write_json
from binomial_ci import format_rate
from simulation import ROOT,Simulation
from perception import NaturalImagePerception,NATURAL_REFERENCE,load_natural_config
from learned_perception import LearnedImagePerception,load_learned_config,check_model_files
from reference_image import load_reference
from run_natural_image_study import plan_cases as previous_plan,validate,summarize,EXTRA_SOURCES,ASSETS
from run_startup_search import trial,SOURCES
from startup_search import load_startup_config
from recovery import load_recovery_config
from joint_limits import load_joint_limit_config

MODES=("natural","learned")
LABELS={"natural":"SIFT","learned":"SuperPoint + LightGlue"}
SOURCE_NAMES=tuple(dict.fromkeys((*SOURCES,*EXTRA_SOURCES,"learned_perception.py","learned_feature_config.json",
    "run_learned_study.py","models/manifest.json","requirements-learned.txt")))

def plan_cases():
    # Small declared functional comparison; no scene is filtered by visibility
    # or by an observed learned outcome. The complete cold search is retained.
    definitions=[
        ("fixed_offset",[3,-3,4,3,-2,2],[0,0,0]),
        ("opposite_offset",[-3,3,-4,-3,2,-2],[0,0,0]),
        ("shifted_picture",[5,-3,4,2,-2,1],[0,.06,.03]),
        ("cold_right",[35,0,0,0,0,0],[0,0,0]),
        ("absent_marker",[0]*6,[0,0,0]),
        ("wrong_target",[0]*6,[0,0,0])]
    result=[]
    for i,(name,offset,shift) in enumerate(definitions,1):
        negative=name in ("absent_marker","wrong_target")
        result.append(dict(trial_id=i,case=name,profile=name,offset_degrees=offset,
            target_shift_m=shift,negative_control=negative,
            search_deadline_s=3 if negative else load_startup_config()["max_search_time_s"]))
    return result

def trace_path(directory,row):
    return directory/"traces"/f"{row['trial_id']:04d}-{row['perception_mode']}.npz"

def image_probes(backends,directory):
    out=directory/"image-probes";out.mkdir()
    boundaries=[
        [[145,90],[475,105],[460,400],[125,380]],
        [[190,110],[440,85],[475,360],[145,380]],
        [[220,155],[420,150],[430,335],[205,345]]]
    rows=[]
    for i,expected in enumerate(boundaries):
        expected=np.asarray(expected,np.float32)
        H=cv2.getPerspectiveTransform(backends["natural"].boundary,expected)
        clean=cv2.warpPerspective(backends["natural"].template_rgb,H,(640,480),borderValue=(22,22,22))
        occluded=clean.copy();occluded[145:225,205:285]=40
        variants={"clean":clean,"dim":np.round(clean*.4).astype(np.uint8),
                  "blur":cv2.GaussianBlur(clean,(5,5),1.2),"occluded":occluded}
        for name,rgb in variants.items():
            Image.fromarray(rgb).save(out/f"{i}-{name}.png")
            for mode,backend in backends.items():
                start=time.perf_counter();o=backend.observe(rgb);ms=1000*(time.perf_counter()-start)
                error=None if o.corners is None else float(np.sqrt(np.mean(np.sum((o.corners-expected)**2,axis=1))))
                rows.append(dict(warp=i,condition=name,mode=mode,accepted=o.corners is not None,
                    corner_rms_px=error,inliers=o.inliers,reason=o.reason,detector_ms=ms,
                    expected_corners=expected.tolist(),estimated_corners=None if o.corners is None else o.corners.tolist()))
    write_json(out/"measurements.json",rows)
    return rows

def analyze(directory):
    manifest=read_json(directory/"manifest.json");plan=read_json(directory/"plan.json")
    rows=[json.loads(s) for s in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    configs=manifest["perception_configs"]
    validate(directory,manifest,plan,rows,modes=MODES,configs=configs)
    probes=read_json(directory/"image-probes/measurements.json")
    expected=[(i,c,m) for i in range(3) for c in ("clean","dim","blur","occluded") for m in MODES]
    if [(r["warp"],r["condition"],r["mode"]) for r in probes]!=expected:
        raise ValueError("Incomplete image probe experiment")
    for row in probes:
        if row["accepted"]:
            computed=float(np.sqrt(np.mean(np.sum((np.array(row["estimated_corners"])-row["expected_corners"])**2,axis=1))))
            if not np.isfinite(computed) or not np.isclose(computed,row["corner_rms_px"],atol=1e-4):
                raise ValueError("Incorrect independent geometry measurement")
    summary=summarize(rows,modes=MODES)
    summary["trace_validation"]="PASS"
    summary["image_probes"]={}
    for mode in MODES:
        group=[r for r in probes if r["mode"]==mode]
        accepted=[r for r in group if r["accepted"]]
        summary["image_probes"][mode]=dict(cases=len(group),accepted=len(accepted),
            median_corner_rms_px=float(np.median([r["corner_rms_px"] for r in accepted])) if accepted else None,
            within_2px=sum(r["corner_rms_px"]<2 for r in accepted))
    write_json(directory/"summary.json",summary)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4.3),layout="constrained")
    fig.suptitle("SIFT and learned matching | initial paired evaluation",fontsize=14,fontweight="bold")
    colors={"natural":"#008b7b","learned":"#6a57b8"}
    bars=axes[0].bar(["SIFT","Learned"],[summary[m]["converged"] for m in MODES],color=list(colors.values()),width=.55)
    axes[0].bar_label(bars,labels=[f"{summary[m]['converged']}/{summary[m]['cases']}" for m in MODES],padding=4)
    axes[0].set(ylabel="Aligned and stable after stopping",ylim=(0,6))
    for mode in MODES:
        row=next(r for r in rows if r["case"]=="fixed_offset" and r["perception_mode"]==mode)
        with np.load(trace_path(directory,row),allow_pickle=False) as t:
            axes[1].plot(t["time_s"],t["error_px"],color=colors[mode],label=LABELS[mode])
        group=[r for r in probes if r["mode"]==mode and r["accepted"]]
        axes[2].scatter([r["warp"]*4+("clean","dim","blur","occluded").index(r["condition"]) for r in group],
                       [r["corner_rms_px"] for r in group],color=colors[mode],label=LABELS[mode],s=28)
    axes[1].axhline(1,color="#888",ls=":",lw=1)
    axes[1].set(xlabel="Simulated time (s)",ylabel="Estimated outline error (px)",yscale="log",title="Same fixed-offset start")
    axes[1].legend(fontsize=8)
    axes[2].axhline(2,color="#888",ls=":",lw=1)
    axes[2].set(xlabel="Declared image-probe index (0â€“11)",ylabel="Independent corner RMS (px)",title="Detected probes only")
    for ax in axes:
        ax.spines[["top","right"]].set_visible(False);ax.grid(alpha=.15);ax.set_axisbelow(True)
    fig.savefig(directory/"comparison.png",dpi=150);fig.savefig(directory/"comparison.svg");plt.close(fig)
    lines=[]
    for spec in plan:
        a,b=[r for r in rows if r["trial_id"]==spec["trial_id"]]
        lines.append(f"| {spec['trial_id']}: {spec['profile']} | {a['outcome']} | {b['outcome']} | {a['terminal_time_s']:.2f} | {b['terminal_time_s']:.2f} |")
    s,l=summary["natural"],summary["learned"]
    fmt=lambda v: "not available" if v is None else f"{v:.2f}"
    report=f"""# Learned matching: initial comparison

Both matchers see the same natural photograph and use the same saved RGB goal,
four virtual boundary corners, geometry gates, fixed-gain IBVS, startup search,
recovery and joint-limit handling. SuperPoint and LightGlue use pretrained weights;
there is no training on these scenes and no fallback to SIFT or ArUco.

Four target-present cases are declared explicitly: the fixed offset, its opposite,
a shifted picture, and a complete unseen start at +35 degrees of base yaw.
The missing/wrong-target controls each use a declared 3-second search deadline.
This is a small functional comparison, with no random-sampling success-rate claim.
All 12 runs and all 24 image-probe measurements are retained. The brackets are 95%
Clopper-Pearson (exact) intervals; with this few cases they are wide, which is why no
success-rate claim is made.

| Measurement | SIFT | SuperPoint + LightGlue |
|---|---:|---:|
| Aligned and stable after stopping | {format_rate(s['converged'],s['cases'])} | {format_rate(l['converged'],l['cases'])} |
| Initially undetected positive cases | {s['initially_undetected']} | {l['initially_undetected']} |
| Median successful total simulated time (s) | {fmt(s['median_total_s_successes'])} | {fmt(l['median_total_s_successes'])} |
| Median of per-trial detector-call medians (ms) | {fmt(s['median_per_trial_detector_ms'])} | {fmt(l['median_per_trial_detector_ms'])} |

![Paired results and independent image geometry](comparison.png)

| Case | SIFT | Learned | SIFT time (s) | Learned time (s) |
|---|---|---|---:|---:|
{chr(10).join(lines)}

## Independent image probes

Three predetermined perspective transforms are each tested clean, dimmed to 40%,
blurred with a 5x5 Gaussian (sigma 1.2), and with a fixed partial occlusion.
These are offline image probes, separate from robot runs. Corners are compared
to independently specified transform geometry, rather than each detector's
own estimate of a goal. Both backends use the same input pixels.

{json.dumps(summary['image_probes'],indent=2)}

## Interpretation and reproducibility

This is a small functional comparison of one known flat photograph. Neither
4/4 robot success nor these twelve image conditions establish that a matcher
is generally better. Rejected probes remain in the denominator. The plot shows
corner errors only where a detection was accepted; the counts above include misses.

Robot success is estimated outline error below 1 px for 0.5 simulated seconds,
followed by a full stopped second below 1 px. It is not a subpixel guarantee of
true physical pose accuracy. The two detected goal outlines differ by
{manifest['reference_outline_difference_px']:.3f} px despite sharing the same RGB goal.

Timing measures CPU detector calls, including identical-frame cache hits.
The camera loop advances at 30 simulated Hz; wall-clock playback can be slower.
No claim of GPU performance is made. The manifest records configs, versions,
model/source/asset hashes and the unmodified goal image. Raw traces include every
matching decision, inlier count, error, joint command and terminal observation.
Trace validation: PASS; negative controls made no false acquisition.

Reproduce: python run_learned_study.py
Rebuild report: python run_learned_study.py --analyze "PATH_TO_RUN"
"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2),flush=True)
    return summary

def run(directory):
    directory.mkdir(parents=True,exist_ok=False)
    for name in ("traces","cases","sources"):(directory/name).mkdir()
    plan=plan_cases()
    configs={"natural":load_natural_config(),"learned":load_learned_config()}
    manifest=dict(status="running",requested_runs=2*len(plan),completed_runs=0,
        created_utc=datetime.now(timezone.utc).isoformat(),perception_configs=configs,
        search_config=load_startup_config(),recovery_config=load_recovery_config(),motion_config=load_joint_limit_config(),
        source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCE_NAMES},
        asset_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ASSETS},
        model_manifest=check_model_files(),versions={n:importlib.metadata.version(n) for n in
            ("torch","torchvision","kornia","lightglue","numpy","mujoco","opencv-python")})
    for name in SOURCE_NAMES:
        target=directory/"sources"/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(ROOT/name,target)
    write_json(directory/"plan.json",plan);write_json(directory/"manifest.json",manifest)
    shutil.copyfile(NATURAL_REFERENCE,directory/"picture_goal.npz")
    manifest["reference_sha256"]=hashlib.sha256(NATURAL_REFERENCE.read_bytes()).hexdigest()
    try:
        backends={"natural":NaturalImagePerception(),"learned":LearnedImagePerception()}
        image_probes(backends,directory)
        with Simulation() as sim:
            refs={}
            for mode,backend in backends.items():
                _,refs[mode]=load_reference(directory/"picture_goal.npz",sim.camera_intrinsics(),
                    backend.reference_config(sim.config),lambda rgb,b=backend:b.observe(rgb).corners)
            manifest["reference_outline_difference_px"]=float(np.sqrt(np.mean(np.sum((refs["natural"]-refs["learned"])**2,axis=1))))
            manifest["simulation_config"]=sim.config
            write_json(directory/"manifest.json",manifest)
            with (directory/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for spec in plan:
                    sim.set_target_mode("aruco" if spec["case"]=="wrong_target" else "natural")
                    for mode,backend in backends.items():
                        observations=[]
                        def detect(rgb):
                            start=time.perf_counter();o=backend.observe(rgb)
                            observations.append((o,1000*(time.perf_counter()-start)))
                            if mode=="learned" and len(observations)%150==0:
                                print(f"  {spec['case']}: {sim.data.time:.1f} simulated s | {o.reason} | {o.inliers} inliers",flush=True)
                            return o.corners
                        cfg=dict(manifest["search_config"],max_search_time_s=spec["search_deadline_s"])
                        row,t,before,after=trial(sim,refs[mode],spec,"search",cfg,manifest["recovery_config"],
                                                manifest["motion_config"],detector=detect)
                        assert len(observations)==row["sample_count"]
                        for key,field in (("matches","matches"),("inliers","inliers"),("inlier_ratio","inlier_ratio"),
                                          ("template_coverage","coverage"),("reprojection_rms_px","reprojection_rms_px"),
                                          ("detection_reason","reason")):
                            t[key]=np.asarray([np.nan if getattr(o,field) is None else getattr(o,field) for o,_ in observations])
                        t["detector_call_ms"]=np.asarray([ms for _,ms in observations])
                        active=t["phase"]!="post_stop"
                        row.update(perception_mode=mode,median_detector_call_ms=float(np.median(t["detector_call_ms"][active])),
                                   p95_detector_call_ms=float(np.percentile(t["detector_call_ms"][active],95)))
                        np.savez_compressed(trace_path(directory,row),**t)
                        case=directory/"cases"/f"{spec['trial_id']:04d}-{mode}";case.mkdir()
                        Image.fromarray(before).save(case/"initial.png");Image.fromarray(after).save(case/"final.png")
                        stream.write(json.dumps(row,allow_nan=False)+"\n");stream.flush()
                        manifest["completed_runs"]+=1;write_json(directory/"manifest.json",manifest)
                        print(f"{spec['trial_id']}/{len(plan)} {mode}: {row['outcome']} | {row['terminal_time_s']:.2f} simulated s",flush=True)
        manifest["status"]="complete"
    except BaseException as exc:
        manifest.update(status="error",error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        manifest["finished_utc"]=datetime.now(timezone.utc).isoformat()
        write_json(directory/"manifest.json",manifest)
    analyze(directory)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analyze",type=Path);parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    if args.analyze:return analyze(args.analyze)
    directory=args.output or ROOT/"results/learned"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    run(directory)
    write_json(ROOT/"results/learned/latest.json",{"path":str(directory.resolve())})
    print(f"Saved {directory}")

if __name__=="__main__":main()
