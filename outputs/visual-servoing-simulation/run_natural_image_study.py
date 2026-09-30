"""Initial paired ArUco / SIFT-picture comparison, with every declared case retained."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import tempfile
import time
import cv2
import mujoco
import numpy as np
from PIL import Image
from benchmark import read_json,write_json
from binomial_ci import format_rate
from perception import ArucoPerception,NaturalImagePerception,NATURAL_REFERENCE,load_natural_config
from reference_image import DEFAULT_REFERENCE,load_reference
from run_startup_search import SOURCES,trial,make_plan
from startup_search import load_startup_config
from recovery import load_recovery_config
from joint_limits import load_joint_limit_config
from simulation import ROOT,Simulation

SEED=20260913
MODES=("aruco","natural")
EXTRA_SOURCES=("perception.py","natural_feature_config.json","app.py","run_natural_image_study.py")
ASSETS=("assets/marker.png","assets/natural-target.png","assets/natural-board.png")


def plan_cases():
    plan=make_plan(8,SEED)[:8]  # Retain all eight randomized target-present draws.
    for name,offset,shift in (
        ("fixed_offset",[3,-3,4,3,-2,2],[0,0,0]),
        ("cold_right",[35,0,0,0,0,0],[0,0,0]),
        ("cold_left",[-35,0,0,0,0,0],[0,0,0]),
        ("shifted_picture",[5,-3,4,2,-2,1],[0,.06,.03]),
        ("absent_marker",[0]*6,[0,0,0]),
        ("wrong_target",[0]*6,[0,0,0])):
        plan.append(dict(trial_id=len(plan)+1,profile=name,case=name,offset_degrees=offset,target_shift_m=shift))
    for spec in plan:
        spec["negative_control"]=spec["case"] in ("absent_marker","wrong_target")
        # These are explicit short timeout controls, not coverage experiments.
        spec["search_deadline_s"]=3 if spec["negative_control"] else load_startup_config()["max_search_time_s"]
    return plan


def summarize(rows,modes=MODES):
    result={}
    for mode in modes:
        group=[r for r in rows if r["perception_mode"]==mode and not r["negative_control"]]
        negative=[r for r in rows if r["perception_mode"]==mode and r["negative_control"]]
        success=[r for r in group if r["outcome"]=="converged"]
        result[mode]=dict(cases=len(group),converged=len(success),
            initially_undetected=sum(not r["initial_detected"] for r in group),
            outcomes=dict(Counter(r["outcome"] for r in group)),
            median_total_s_successes=float(np.median([r["terminal_time_s"] for r in success])) if success else None,
            negative_outcomes=dict(Counter(r["outcome"] for r in negative)),
            median_per_trial_detector_ms=float(np.median([r["median_detector_call_ms"] for r in group])) if group else None)
    return result


def path_for(directory,row):
    return directory/"traces"/f"{row['trial_id']:04d}-{row['perception_mode']}.npz"


def validate(directory,manifest,plan,rows,modes=MODES,configs=None):
    if (manifest["status"]!="complete" or len(rows)!=manifest["completed_runs"]
            or len(rows)!=manifest["requested_runs"] or len(rows)!=2*len(plan)):
        raise ValueError("Cannot report an incomplete natural-image study")
    if [(r["trial_id"],r["perception_mode"]) for r in rows]!=[(s["trial_id"],m) for s in plan for m in modes]:
        raise ValueError("Paired rows differ from the complete plan")
    for spec in plan:
        pair=[r for r in rows if r["trial_id"]==spec["trial_id"]]
        for row in pair:
            for key in ("offset_degrees","target_shift_m","case","search_deadline_s"):
                if row[key]!=spec[key]: raise ValueError(f"Scene setup differs: {key}")
        with np.load(path_for(directory,pair[0]),allow_pickle=False) as a, np.load(path_for(directory,pair[1]),allow_pickle=False) as b:
            np.testing.assert_array_equal(a["qpos_rad"][0],b["qpos_rad"][0])
    for row in rows:
        with np.load(path_for(directory,row),allow_pickle=False) as t:
            if any(len(t[k])!=row["sample_count"] for k in t.files):
                raise ValueError("Trace length mismatch")
            if (not np.isfinite(t["command_rad_s"]).all() or not np.isfinite(t["qpos_rad"]).all()
                    or not np.all(np.diff(t["time_s"])>0) or np.max(np.abs(t["command_rad_s"]))>.35+1e-9):
                raise ValueError("Invalid trajectory or excessive speed")
            stop=t["phase"]=="post_stop"
            terminal=np.flatnonzero(~stop)[-1]
            if stop.sum()!=30 or t["command_rad_s"][stop].any() or t["command_rad_s"][terminal].any():
                raise ValueError("Missing zero command at termination or after stopping")
            if row["outcome"]=="converged" and (not np.isfinite(t["error_px"][stop]).all() or np.any(t["error_px"][stop]>=1)):
                raise ValueError("Reported success did not stay aligned after stopping")
            if not row["search_bounds_ok"] or row["max_search_command_rad_s"]>.25+1e-9:
                raise ValueError("Search exceeded bounds")
            if row["terminal_time_s"]>row["search_deadline_s"]+manifest["recovery_config"]["max_total_time_s"]+.04:
                raise ValueError("Total deadline exceeded")
            accepted=t["detected"]
            if row["perception_mode"]!="aruco":
                cfg=manifest["natural_config"] if configs is None else configs[row["perception_mode"]]
                if (not np.isfinite(t["reprojection_rms_px"][accepted]).all()
                        or np.any(t["inliers"][accepted]<cfg["min_inliers"])
                        or np.any(t["inlier_ratio"][accepted]<cfg["min_inlier_ratio"])
                        or np.any(t["template_coverage"][accepted]<cfg["min_template_coverage"])
                        or np.any(t["reprojection_rms_px"][accepted]>cfg["max_reprojection_rms_px"])):
                    raise ValueError("An accepted picture lacked the required matching evidence")
            if row["negative_control"]:
                if accepted.any() or row["outcome"]!="target_not_found":
                    raise ValueError("Absent/wrong target produced a false acquisition")


def analyze(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(line) for line in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    validate(directory,manifest,plan,rows)
    summary=summarize(rows)
    summary["trace_validation"]="PASS"
    write_json(directory/"summary.json",summary)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(13,4.4),layout="constrained")
    fig.suptitle("Natural-picture alignment | initial paired comparison",fontweight="bold",fontsize=14)
    colors={"aruco":"#ba663c","natural":"#008b7b"}
    bars=axes[0].bar(["ArUco","Picture / SIFT"],[summary[m]["converged"] for m in MODES],color=[colors[m] for m in MODES],width=.55)
    axes[0].bar_label(bars,labels=[f"{summary[m]['converged']}/{summary[m]['cases']}" for m in MODES],padding=4)
    axes[0].set(ylabel="Aligned and stable after stopping",ylim=(0,14))
    for mode in MODES:
        row=next(r for r in rows if r["case"]=="fixed_offset" and r["perception_mode"]==mode)
        with np.load(path_for(directory,row),allow_pickle=False) as t:
            axes[1].plot(t["time_s"],t["error_px"],color=colors[mode],label=mode)
            if mode=="natural":
                axes[2].plot(t["time_s"],t["inliers"],color=colors[mode])
    axes[1].axhline(1,color="#666",ls=":",lw=1)
    axes[1].set(xlabel="Simulated time (s)",ylabel="Outline error (px)",yscale="log",title="Same fixed-offset start")
    axes[1].legend()
    axes[2].set(xlabel="Simulated time (s)",ylabel="Geometric inliers",title="Picture matching evidence")
    for ax in axes:
        ax.spines[["top","right"]].set_visible(False)
        ax.grid(alpha=.15)
        ax.set_axisbelow(True)
    fig.savefig(directory/"comparison.png",dpi=150)
    fig.savefig(directory/"comparison.svg")
    plt.close(fig)
    lines=[]
    for spec in plan:
        a,b=[r for r in rows if r["trial_id"]==spec["trial_id"]]
        lines.append(f"| {spec['trial_id']}: {spec['profile']} | {a['outcome']} | {b['outcome']} | {a['terminal_time_s']:.2f} | {b['terminal_time_s']:.2f} |")
    a,b=summary["aruco"],summary["natural"]
    report=f"""# Initial natural-picture comparison

The same fixed-gain IBVS, startup search, recovery and joint-limit handling are used
with two perception backends: ArUco corners or SIFT matches plus a RANSAC homography
that estimates the four corners of a flat picture. This is classical feature matching,
not learned SuperPoint/LightGlue or direct control of a changing keypoint set.

There are **12 target-present cases**: eight seeded random joint/target placements
(seed {SEED}), the normal offset, two cold starts and one declared target translation.
Every sampled case is retained. Two negative controls (absent target and wrong target)
use an explicitly shortened **3-second search deadline**, followed by the same
one-second stopped observation. This is an initial functional comparison, not a
200-trial success-rate study or a test across many different natural pictures.
Brackets are 95% Clopper-Pearson (exact) intervals: 12/12 only shows a true rate of at
least 73.5%.

| Measurement | ArUco | Picture / SIFT |
|---|---:|---:|
| Aligned and stable for 1 s after stopping | {format_rate(a['converged'],a['cases'])} | {format_rate(b['converged'],b['cases'])} |
| Initially undetected | {a['initially_undetected']} | {b['initially_undetected']} |
| Median total simulated time, successful cases | {a['median_total_s_successes']} | {b['median_total_s_successes']} |
| Median of per-trial detector-call medians (ms) | {a['median_per_trial_detector_ms']} | {b['median_per_trial_detector_ms']} |

Detector timings measure actual calls, including cache hits on identical RGB frames;
they are recorded separately from simulated time. They are machine-dependent.
The camera/control loop advances at 30 simulated Hz; this does not establish
30 frames per second of real-time picture matching on this computer.

![Paired results and feature evidence](comparison.png)

| Case | ArUco | Picture / SIFT | ArUco time (s) | Picture time (s) |
|---|---|---|---:|---:|
{chr(10).join(lines)}

The printed inner square is 0.24 m for both targets. Each uses its own saved image
of the same nominal desired view. The RMS difference between the detected desired
outlines is {manifest['reference_outline_difference_px']:.3f} px; differences in
perception localization are part of this comparison. Depth comes from the observed
square and its known physical size, not the simulator's target pose.

The generated botanical still-life picture contains no fiducial code. Its canonical
image supplies SIFT descriptors. Ambiguous and duplicate matches are removed,
RANSAC rejects inconsistent correspondences, and minimum inliers, spatial coverage,
residual, convexity and full-outline visibility gates decide whether a measurement
is usable. A rejected frame supplies no control features; the existing bounded
search/recovery behavior then applies. No target world pose or saved goal joint
configuration enters perception or the visual error.

Success is RMS error below 1 px in the **estimated outline**, held for 0.5 s, then
remaining below 1 px for a further stopped second. This is not a measured guarantee
of subpixel true physical pose accuracy. Separate synthetic-warp tests compare
outline estimates against known image transforms.

Trace validation: PASS. Starts, scene parameters, search/velocity/deadline bounds,
matching thresholds and stopped observations are checked. No negative control
acquired a false target. Both mode references, source snapshots/hashes, target-asset
hashes, configurations, raw per-frame matching metrics and every outcome are saved.

Reproduce: python run_natural_image_study.py
Rebuild report: python run_natural_image_study.py --analyze "PATH_TO_THIS_RUN"
"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def run(directory):
    plan=plan_cases()
    directory.mkdir(parents=True,exist_ok=False)
    for name in ("traces","cases","sources"):
        (directory/name).mkdir()
    source_names=tuple(dict.fromkeys((*SOURCES,*EXTRA_SOURCES)))
    manifest=dict(status="running",requested_runs=2*len(plan),completed_runs=0,seed=SEED,
        created_utc=datetime.now(timezone.utc).isoformat(),natural_config=load_natural_config(),
        search_config=load_startup_config(),recovery_config=load_recovery_config(),motion_config=load_joint_limit_config(),
        python=platform.python_version(),opencv=cv2.__version__,mujoco=mujoco.__version__,numpy=np.__version__,
        source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in source_names},
        asset_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ASSETS})
    for name in source_names:
        shutil.copyfile(ROOT/name,directory/"sources"/name)
    write_json(directory/"plan.json",plan)
    write_json(directory/"manifest.json",manifest)
    try:
        with Simulation() as sim:
            backends={"aruco":ArucoPerception(sim.marker_corners),"natural":NaturalImagePerception()}
            references={}
            manifest["reference_sha256"]={}
            for mode,path in (("aruco",DEFAULT_REFERENCE),("natural",NATURAL_REFERENCE)):
                saved=directory/f"{mode}_goal.npz"
                shutil.copyfile(path,saved)
                manifest["reference_sha256"][mode]=hashlib.sha256(saved.read_bytes()).hexdigest()
                backend=backends[mode]
                _,references[mode]=load_reference(saved,sim.camera_intrinsics(),backend.reference_config(sim.config),
                                                 lambda rgb,b=backend:b.observe(rgb).corners)
            difference=float(np.sqrt(np.mean(np.sum((references["aruco"]-references["natural"])**2,axis=1))))
            if difference>1:
                raise ValueError("The two saved references use different framing; teach comparable goals before this comparison")
            manifest["reference_outline_difference_px"]=difference
            manifest["simulation_config"]=sim.config
            manifest["joint_limits_rad"]=sim.model.jnt_range.tolist()
            write_json(directory/"manifest.json",manifest)
            with (directory/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for spec in plan:
                    for mode in MODES:
                        backend=backends[mode]
                        physical=("natural" if mode=="aruco" else "aruco") if spec["case"]=="wrong_target" else mode
                        sim.set_target_mode(physical)
                        observed=[]
                        def detect(rgb):
                            start=time.perf_counter()
                            o=backend.observe(rgb)
                            observed.append((o,1000*(time.perf_counter()-start)))
                            return o.corners
                        cfg=dict(manifest["search_config"],max_search_time_s=spec["search_deadline_s"])
                        row,trace,before,after=trial(sim,references[mode],spec,"search",cfg,manifest["recovery_config"],
                                                     manifest["motion_config"],detector=detect)
                        assert len(observed)==row["sample_count"]
                        for key,values in {
                            "matches":[o.matches for o,_ in observed],"inliers":[o.inliers for o,_ in observed],
                            "inlier_ratio":[o.inlier_ratio for o,_ in observed],
                            "template_coverage":[o.coverage for o,_ in observed],
                            "reprojection_rms_px":[np.nan if o.reprojection_rms_px is None else o.reprojection_rms_px for o,_ in observed],
                            "detector_call_ms":[ms for _,ms in observed],"detection_reason":[o.reason for o,_ in observed]}.items():
                            trace[key]=np.asarray(values)
                        active=trace["phase"]!="post_stop"
                        row.update(perception_mode=mode,physical_target=physical,
                                   median_detector_call_ms=float(np.median(trace["detector_call_ms"][active])),
                                   p95_detector_call_ms=float(np.percentile(trace["detector_call_ms"][active],95)),
                                   median_inliers_when_detected=float(np.median(trace["inliers"][trace["detected"]])) if trace["detected"].any() else 0.)
                        np.savez_compressed(path_for(directory,row),**trace)
                        case=directory/"cases"/f"{spec['trial_id']:04d}-{mode}"
                        case.mkdir()
                        Image.fromarray(before).save(case/"initial.png")
                        Image.fromarray(after).save(case/"final.png")
                        stream.write(json.dumps(row,allow_nan=False)+"\n")
                        stream.flush()
                        manifest["completed_runs"]+=1
                        write_json(directory/"manifest.json",manifest)
                        print(f"{spec['trial_id']}/{len(plan)} {mode}: {row['outcome']} | acquisition {row['acquisition_s']} | total {row['terminal_time_s']:.2f} s",flush=True)
        manifest["status"]="complete"
    except BaseException as exc:
        manifest["status"]="interrupted" if isinstance(exc,KeyboardInterrupt) else "error"
        manifest["error"]=f"{type(exc).__name__}: {exc}"
        raise
    finally:
        manifest["finished_utc"]=datetime.now(timezone.utc).isoformat()
        write_json(directory/"manifest.json",manifest)
    analyze(directory)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--analyze",type=Path)
    args=parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    directory=args.output or ROOT/"results/natural-image"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    run(directory)
    write_json(ROOT/"results/natural-image/latest.json",{"path":str(directory.resolve())})
    print(f"Saved natural-picture study: {directory.resolve()}")

if __name__=="__main__":
    main()
