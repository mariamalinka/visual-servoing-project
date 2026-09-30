"""Replay a recorded startup evaluation with joint-limit supervision enabled."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime,timezone
import hashlib
import json
import os
import platform
from pathlib import Path
import shutil
import tempfile

import numpy as np
import cv2
import mujoco
from PIL import Image

from benchmark import read_json,write_json
from binomial_ci import format_rate
from joint_limits import load_joint_limit_config,velocity_bounds
from reference_image import DEFAULT_REFERENCE,load_reference
from run_startup_search import SOURCES,trial
from simulation import ROOT,Simulation

BASELINE=ROOT/"results/startup/20260911-184428-897577-n200-seed20260911"
PHYSICS_SOURCES=("control.py","simulation.py","scene.xml","config.json",
                 "recovery_config.json")
# This historical study explicitly loads its archived single-pass search settings.
# Current default settings are not used. Coarse commands remain backward compatible;
# tests/fixtures/coarse-search-prefix.npz checks the recorded path independently.


def load_baseline(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(line) for line in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    selected=[r for r in rows if r["method"]=="search"]
    if manifest["status"]!="complete" or len(rows)!=manifest["completed_runs"] or len(rows)!=manifest["requested_runs"]:
        raise ValueError("Baseline run is incomplete")
    if [r["trial_id"] for r in selected]!=[p["trial_id"] for p in plan]:
        raise ValueError("Baseline trial IDs do not match its plan")
    for name in PHYSICS_SOURCES:
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=manifest["source_sha256"][name]:
            raise ValueError(f"Baseline physics/search configuration differs: {name}")
    if hashlib.sha256(DEFAULT_REFERENCE.read_bytes()).hexdigest()!=manifest["reference_sha256"]:
        raise ValueError("Baseline uses a different reference image")
    versions=dict(python=platform.python_version(),numpy=np.__version__,
                  opencv=cv2.__version__,mujoco=mujoco.__version__)
    for name,value in versions.items():
        if manifest[name]!=value:
            raise ValueError(f"Recorded baseline runtime differs: {name}")
    return manifest,plan,selected


def compare(rows):
    by_id={}
    summary={}
    for row in rows:
        by_id.setdefault(row["trial_id"],{})[row["policy"]]=row
    for policy in ("baseline","supervised"):
        group=[r for r in rows if r["policy"]==policy and r["case"]=="random_target"]
        success=[r for r in group if r["outcome"]=="converged"]
        unseen=[r for r in group if not r["initial_detected"]]
        negatives=[r for r in rows if r["policy"]==policy and r["case"]!="random_target"]
        summary[policy]=dict(trials=len(group),converged=len(success),
            initially_undetected=len(unseen),
            aligned_initially_undetected=sum(r["outcome"]=="converged" for r in unseen),
            outcomes=dict(Counter(r["outcome"] for r in group)),
            retried=sum(r.get("alignment_retries",0)>0 for r in group),
            converged_after_retry=sum(r.get("alignment_retries",0)>0 for r in success),
            negative_control_outcomes=dict(Counter(r["outcome"] for r in negatives)))
    pairs=[p for p in by_id.values() if p["baseline"]["case"]=="random_target"]
    summary["improved_trial_ids"]=[p["baseline"]["trial_id"] for p in pairs
        if p["baseline"]["outcome"]!="converged" and p["supervised"]["outcome"]=="converged"]
    summary["regressed_trial_ids"]=[p["baseline"]["trial_id"] for p in pairs
        if p["baseline"]["outcome"]=="converged" and p["supervised"]["outcome"]!="converged"]
    return summary


def validate(directory,manifest,plan,rows):
    if manifest["status"]!="complete" or len(rows)!=manifest["completed_rows"] or len(rows)!=2*len(plan):
        raise ValueError("Cannot report an incomplete joint-limit study")
    expected=[(p["trial_id"],m) for p in plan for m in ("baseline","supervised")]
    if [(r["trial_id"],r["policy"]) for r in rows]!=expected:
        raise ValueError("Study trial IDs or ordering differ from the plan")
    cfg=manifest["motion_config"]
    limits=np.array(manifest["joint_limits_rad"])
    for i in range(0,len(rows),2):
        a,b=rows[i:i+2]
        for key in ("offset_degrees","target_shift_m","initial_detected","first_detection_s","acquisition_s"):
            if a[key]!=b[key]:
                raise ValueError(f"Paired starts/search acquisition differ: {key}")
        with np.load(directory/"traces"/f"{a['trial_id']:04d}-baseline.npz",allow_pickle=False) as ta:
            with np.load(directory/"traces"/f"{b['trial_id']:04d}-supervised.npz",allow_pickle=False) as tb:
                np.testing.assert_array_equal(ta["qpos_rad"][0],tb["qpos_rad"][0])
                np.testing.assert_array_equal(ta["corners_px"][0],tb["corners_px"][0])
    for row in rows:
        with np.load(directory/"traces"/f"{row['trial_id']:04d}-{row['policy']}.npz",allow_pickle=False) as t:
            if any(len(t[k])!=row["sample_count"] for k in t.files):
                raise ValueError("Trace length mismatch")
            if not np.all(np.diff(t["time_s"])>0) or not np.isfinite(t["command_rad_s"]).all() or not np.isfinite(t["qpos_rad"]).all():
                raise ValueError("Invalid trace values")
            if np.max(np.abs(t["command_rad_s"]))>.35+1e-9:
                raise ValueError("Joint command speed limit exceeded")
            stop=t["phase"]=="post_stop"
            if stop.sum()!=30 or np.any(t["command_rad_s"][stop]) or np.any(t["command_rad_s"][np.flatnonzero(~stop)[-1]]):
                raise ValueError("Missing zero command at termination or after stopping")
            if row["outcome"]=="converged" and (not np.isfinite(t["error_px"][stop]).all() or np.any(t["error_px"][stop]>=1)):
                raise ValueError("Reported success does not remain aligned after stopping")
            if row["policy"]=="supervised":
                if row["alignment_retries"]>cfg["max_retries"] or row["min_joint_margin_rad"]<cfg["joint_limit_margin_rad"]-.005:
                    raise ValueError("Retry count or actual joint margin violated")
                for index in np.flatnonzero(np.isin(t["phase"],["running","joint_limited","realigning"])):
                    lo,hi=velocity_bounds(t["qpos_rad"][index],limits,.35,
                        cfg["joint_limit_margin_rad"],np.deg2rad(cfg["influence_degrees"]),1/30)
                    if np.any(t["command_rad_s"][index]<lo-1e-9) or np.any(t["command_rad_s"][index]>hi+1e-9):
                        raise ValueError("Alignment command violates the velocity damper")
                moving=np.isin(t["phase"],["repositioning","retry_confirming"])
                if moving.any() and np.max(np.abs(t["command_rad_s"][moving]))>cfg["reposition_velocity_rad_s"]+1e-9:
                    raise ValueError("Repositioning velocity exceeded its cap")
                for event in row["motion_events"]:
                    if event["kind"]=="retry_started":
                        delta=np.array(event["goal_qpos_rad"])-event["qpos_rad"]
                        if np.max(np.abs(delta))>np.deg2rad(cfg["max_reposition_excursion_degrees"])+1e-9:
                            raise ValueError("Retry goal exceeded its excursion bound")
                        later=[e["time_s"] for e in row["motion_events"]
                               if e["kind"]=="retry_confirmed" and e["time_s"]>=event["time_s"]]
                        end=min(later) if later else row["terminal_time_s"]
                        section=moving & (t["time_s"]>=event["time_s"]-.003) & (t["time_s"]<=end+.003)
                        if section.any() and np.max(np.abs(t["qpos_rad"][section]-event["qpos_rad"]))>np.deg2rad(cfg["max_reposition_excursion_degrees"])+.005:
                            raise ValueError("Actual return exceeded its excursion bound")
                if row["terminal_time_s"]>manifest["search_config"]["max_search_time_s"]+manifest["recovery_config"]["max_total_time_s"]+.04:
                    raise ValueError("Overall deadline exceeded")


def analyze(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(line) for line in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    validate(directory,manifest,plan,rows)
    summary=compare(rows)
    summary["trace_validation"]="PASS"
    write_json(directory/"summary.json",summary)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,7.5),layout="constrained")
    fig.suptitle("Joint-limit recovery | the two previous alignment failures",fontsize=15,fontweight="bold")
    for r,trial_id in enumerate((13,167)):
        for policy,color in (("baseline","#ba663c"),("supervised","#008b7b")):
            row=next(x for x in rows if x["trial_id"]==trial_id and x["policy"]==policy)
            with np.load(directory/"traces"/f"{trial_id:04d}-{policy}.npz",allow_pickle=False) as t:
                visible=t["time_s"]>=row["acquisition_s"]
                time=t["time_s"][visible]-row["acquisition_s"]
                axes[r,0].plot(time,t["error_px"][visible],label=policy,color=color,lw=1.7)
                angles=np.max(np.abs(np.rad2deg(t["qpos_rad"][visible][:,[3,5]])),axis=1)
                axes[r,1].plot(time,angles,label=policy,color=color,lw=1.7)
                moving=t["phase"]=="repositioning"
                if policy=="supervised" and moving.any():
                    low,high=t["time_s"][moving][[0,-1]]-row["acquisition_s"]
                    for ax in axes[r]:
                        ax.axvspan(low,high,color=color,alpha=.1,label="return for retry")
        axes[r,0].axhline(1,color="#555",ls=":",lw=1)
        axes[r,0].set(yscale="log",ylabel=f"Trial {trial_id}: error (px)",xlabel="Time after acquisition (simulated s)")
        axes[r,1].axhline(170,color="#555",ls=":",lw=1,label="mechanical limit")
        axes[r,1].set(ylabel="max(|J4|, |J6|) (degrees)",xlabel="Time after acquisition (simulated s)",ylim=(0,180))
        for ax in axes[r]:
            ax.spines[["top","right"]].set_visible(False)
            ax.grid(alpha=.15)
            ax.legend(fontsize=8)
    fig.savefig(directory/"joint_limit_recovery.png",dpi=150)
    fig.savefig(directory/"joint_limit_recovery.svg")
    plt.close(fig)
    a,b=summary["baseline"],summary["supervised"]
    report=f"""# Joint-limit supervision evaluation

The same {a['trials']} randomized target-present scenes and four negative controls
were evaluated. Baseline traces are copied from the completed, earlier startup
study; the supervised controller was run anew. Physics, camera, marker, search,
reference-image and recovery-budget compatibility were checked before the run.
Every sampled scene is retained. Initial detections and acquisition times match
within each pair.

| Result | Recorded baseline | Joint limits + bounded retry |
|---|---:|---:|
| Aligned and stable for 1 s after stopping | {format_rate(a['converged'],a['trials'])} | {format_rate(b['converged'],b['trials'])} |
| Initially undetected, then aligned | {format_rate(a['aligned_initially_undetected'],a['initially_undetected'])} | {format_rate(b['aligned_initially_undetected'],b['initially_undetected'])} |
| Scenes with an alignment retry | 0 | {b['retried']} |
| Converged after retry | 0 | {b['converged_after_retry']} |

Counts show the 95% Clopper-Pearson (exact) confidence interval for the true rate; see docs/STATISTICS.md.

Improved trial IDs: {summary['improved_trial_ids']}.
Regressed trial IDs: {summary['regressed_trial_ids']}.
Supervised outcomes: {b['outcomes']}.
Negative-control outcomes: {b['negative_control_outcomes']}.

![Joint limits and image error](joint_limit_recovery.png)

The original image-based controller and fixed gain 1.2/s are retained. Feasible
original joint commands are unchanged. When needed, bounded least squares
reallocates motion while outward speeds decrease within 25 degrees of a
0.06-radian joint margin. Actual joint positions allow a 0.005-radian actuator
tracking tolerance in validation; requested steps obey the configured bounds.

A two-second image-error window detects insufficient improvement above 3 px.
It compares the best error in each half of the window, requiring at least
1 px or 2% improvement, whichever is larger. Stalls or a 20-second alignment
timeout trigger at most one retry: brake, return toward the first observed
alignment view, confirm three detections, then realign with stronger joint
damping and a mild preference for central joint positions. This changes the
joint-motion calculation, not the desired image or IBVS gain.

The return is capped at 0.25 rad/s, 140 degrees per joint from its entry, and
12 seconds including confirmation. It is joint-feedback motion and can continue
without marker visibility while returning. It does not use a target world pose,
a taught goal joint pose, or teleportation. Startup initialization is the only reset.
The 45-second post-acquisition overall budget includes the return and retry;
each IBVS attempt still has a 20-second timeout. The new method can therefore
use more alignment time than a baseline that stops after its first timeout.

The two failures used during development are part of this regression distribution;
these results are not a held-out success estimate or a guarantee of arbitrary
workspace coverage. Search coverage is unchanged. Robot collisions remain disabled
in this teaching simulation; joint-limit handling is not collision-aware planning.

The baseline manifest, all source hashes, configurations, original plan, reference,
paired outcomes, per-frame NPZ traces and motion events are saved with this run.
Trace validation: PASS.

Reproduce: python run_joint_limit_study.py --baseline "PATH_TO_ORIGINAL_STARTUP_RUN"
Rebuild this report: python run_joint_limit_study.py --analyze "PATH_TO_THIS_RUN"
"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def run(directory,baseline):
    old,plan,prior=load_baseline(baseline)
    cfg=load_joint_limit_config()
    directory.mkdir(parents=True,exist_ok=False)
    (directory/"traces").mkdir()
    (directory/"cases").mkdir()
    sources=(*SOURCES,"run_joint_limit_study.py","app.py")
    manifest=dict(status="running",completed_rows=0,requested_rows=2*len(plan),
        created_utc=datetime.now(timezone.utc).isoformat(),baseline_directory=str(baseline.resolve()),
        baseline_origin="Recorded earlier; copied without rerunning physics",
        motion_config=cfg,search_config=old["search_config"],recovery_config=old["recovery_config"],
        source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in sources},
        reference_sha256=old["reference_sha256"],
        python=platform.python_version(),numpy=np.__version__,opencv=cv2.__version__,mujoco=mujoco.__version__)
    write_json(directory/"baseline_manifest.json",old)
    write_json(directory/"plan.json",plan)
    write_json(directory/"manifest.json",manifest)
    shutil.copyfile(baseline/"goal.npz",directory/"goal.npz")
    if hashlib.sha256((directory/"goal.npz").read_bytes()).hexdigest()!=manifest["reference_sha256"]:
        raise ValueError("Archived baseline reference has changed")
    try:
        with Simulation() as sim:
            _,reference=load_reference(directory/"goal.npz",sim.camera_intrinsics(),sim.config,sim.marker_corners)
            manifest["joint_limits_rad"]=sim.model.jnt_range.tolist()
            manifest["simulation_config"]=sim.config
            write_json(directory/"manifest.json",manifest)
            with (directory/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for spec,previous in zip(plan,prior):
                    previous=dict(previous,policy="baseline")
                    source=baseline/"traces"/f"{spec['trial_id']:04d}-search.npz"
                    shutil.copyfile(source,directory/"traces"/f"{spec['trial_id']:04d}-baseline.npz")
                    row,trace,before,after=trial(sim,reference,spec,"search",old["search_config"],old["recovery_config"],cfg)
                    row["policy"]="supervised"
                    np.savez_compressed(directory/"traces"/f"{spec['trial_id']:04d}-supervised.npz",**trace)
                    for saved in (previous,row):
                        stream.write(json.dumps(saved,allow_nan=False)+"\n")
                    stream.flush()
                    if row["alignment_retries"] or row["outcome"]!="converged":
                        case=directory/"cases"/f"trial-{spec['trial_id']:04d}"
                        case.mkdir()
                        Image.fromarray(before).save(case/"initial.png")
                        Image.fromarray(after).save(case/"final.png")
                        write_json(case/"trial.json",row)
                    manifest["completed_rows"]+=2
                    write_json(directory/"manifest.json",manifest)
                    print(f"{spec['trial_id']}/{len(plan)} | {previous['outcome']} -> {row['outcome']} | retries {row['alignment_retries']}",flush=True)
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
    parser.add_argument("--baseline",type=Path,default=BASELINE)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--analyze",type=Path)
    args=parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    name=datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    directory=args.output or ROOT/"results/joint-limits"/name
    run(directory,args.baseline)
    try:
        location={"relative_to_project":directory.resolve().relative_to(ROOT).as_posix()}
    except ValueError:
        location={"absolute_path":str(directory.resolve())}
    write_json(ROOT/"results/joint-limits/latest.json",location)
    print(f"Saved joint-limit study: {directory.resolve()}")

if __name__=="__main__":
    main()
