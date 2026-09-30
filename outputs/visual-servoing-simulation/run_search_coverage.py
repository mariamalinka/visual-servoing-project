"""Compare the coarse startup scan with refinement, including fresh paired scenes."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import tempfile

import cv2
import mujoco
import numpy as np
from PIL import Image

from benchmark import read_json, write_json
from binomial_ci import format_rate
from reference_image import DEFAULT_REFERENCE, load_reference
from run_startup_search import SOURCES, make_plan, trial
from simulation import ROOT, Simulation
from startup_search import load_startup_config

BASELINE = ROOT/"results/joint-limits/20260912-131706-003699"
UNCHANGED = ("control.py","simulation.py","scene.xml","config.json","reference_image.py",
             "recovery.py","recovery_config.json","joint_limits.py","joint_limit_config.json",
             "benchmark.py","benchmark_config.json")


def load_baseline(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(s) for s in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    selected=[r for r in rows if r["policy"]=="supervised"]
    if (manifest["status"]!="complete" or len(rows)!=manifest["completed_rows"]
            or len(rows)!=manifest["requested_rows"] or len(rows)!=2*len(plan)
            or [r["trial_id"] for r in selected]!=[p["trial_id"] for p in plan]):
        raise ValueError("The recorded joint-limit baseline is incomplete or mismatched")
    for name in UNCHANGED:
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=manifest["source_sha256"][name]:
            raise ValueError(f"Baseline physics/alignment differs: {name}")
    for key,value in dict(python=platform.python_version(),numpy=np.__version__,
                          opencv=cv2.__version__,mujoco=mujoco.__version__).items():
        if manifest[key]!=value:
            raise ValueError(f"Baseline runtime differs: {key}")
    for file in (DEFAULT_REFERENCE,directory/"goal.npz"):
        if hashlib.sha256(file.read_bytes()).hexdigest()!=manifest["reference_sha256"]:
            raise ValueError("Reference image differs from the baseline")
    return manifest,plan,selected


def trace_path(directory,row):
    return directory/"traces"/f"{row['cohort']}-{row['trial_id']:04d}-{row['policy']}.npz"


def summarize(rows):
    result={}
    for cohort in sorted({r["cohort"] for r in rows}):
        group=[r for r in rows if r["cohort"]==cohort]
        stats={}
        for policy in ("coarse","refined"):
            actual=[r for r in group if r["policy"]==policy and r["case"]=="random_target"]
            unseen=[r for r in actual if not r["initial_detected"]]
            negatives=[r for r in group if r["policy"]==policy and r["case"]!="random_target"]
            stats[policy]=dict(trials=len(actual),converged=sum(r["outcome"]=="converged" for r in actual),
                initially_undetected=len(unseen),
                acquired_initially_undetected=sum(r["acquisition_s"] is not None for r in unseen),
                aligned_initially_undetected=sum(r["outcome"]=="converged" for r in unseen),
                refined_pass_used=sum(r.get("search_refinement_started_s") is not None for r in actual),
                outcomes=dict(Counter(r["outcome"] for r in actual)),
                negative_control_outcomes=dict(Counter(r["outcome"] for r in negatives)))
        pairs={r["trial_id"]:{p["policy"]:p for p in group if p["trial_id"]==r["trial_id"]}
               for r in group if r["case"]=="random_target"}
        stats["improved_trial_ids"]=[i for i,p in pairs.items() if p["coarse"]["outcome"]!="converged" and p["refined"]["outcome"]=="converged"]
        stats["regressed_trial_ids"]=[i for i,p in pairs.items() if p["coarse"]["outcome"]=="converged" and p["refined"]["outcome"]!="converged"]
        result[cohort]=stats
    return result


def validate(directory,manifest,plan,rows):
    if (manifest["status"]!="complete" or len(rows)!=manifest["completed_rows"]
            or len(rows)!=manifest["requested_rows"] or len(rows)!=2*len(plan)):
        raise ValueError("Cannot report an incomplete coverage study")
    expected=[(s["cohort"],s["trial_id"],p) for s in plan for p in ("coarse","refined")]
    if [(r["cohort"],r["trial_id"],r["policy"]) for r in rows]!=expected:
        raise ValueError("Plan and result order differ")
    for index in range(0,len(rows),2):
        a,b=rows[index:index+2]
        spec=plan[index//2]
        for key in ("offset_degrees","target_shift_m","case","trial_id","cohort"):
            if a[key]!=spec[key] or b[key]!=spec[key]:
                raise ValueError(f"Paired scene differs: {key}")
        if a["initial_detected"]!=b["initial_detected"]:
            raise ValueError("Initial visibility differs")
        with np.load(trace_path(directory,a),allow_pickle=False) as ta, np.load(trace_path(directory,b),allow_pickle=False) as tb:
            # A refined run must replay the complete old scan up to its failure,
            # or the entire trace when the old policy acquired the marker.
            count=len(ta["time_s"])
            if a["acquisition_s"] is None:
                count=int(np.flatnonzero(ta["phase"]!="post_stop")[-1])
            if len(tb["time_s"])<count:
                raise ValueError("Refined trace ended before the coarse prefix")
            for key in ("time_s","qpos_rad","corners_px","command_rad_s","phase"):
                np.testing.assert_array_equal(ta[key][:count],tb[key][:count])
            if a["acquisition_s"] is not None:
                if a["outcome"]!=b["outcome"] or a["sample_count"]!=b["sample_count"]:
                    raise ValueError("An already-acquired scene changed")
    limits=np.asarray(manifest["joint_limits_rad"])
    dt=1/manifest["simulation_config"]["camera_hz"]
    for row in rows:
        cfg=manifest["search_configs"][row["policy"]]
        with np.load(trace_path(directory,row),allow_pickle=False) as t:
            if any(len(t[k])!=row["sample_count"] for k in t.files):
                raise ValueError("Trace length mismatch")
            if (not np.all(np.diff(t["time_s"])>0) or not np.isfinite(t["qpos_rad"]).all()
                    or not np.isfinite(t["command_rad_s"]).all()
                    or np.max(np.abs(t["command_rad_s"]))>.35+1e-9):
                raise ValueError("Invalid trace or command speed")
            stop=t["phase"]=="post_stop"
            end=np.flatnonzero(~stop)[-1]
            if stop.sum()!=30 or t["command_rad_s"][stop].any() or t["command_rad_s"][end].any():
                raise ValueError("Missing stopped command observation")
            if row["outcome"]=="converged" and (not np.isfinite(t["error_px"][stop]).all() or np.any(t["error_px"][stop]>=1)):
                raise ValueError("Success did not remain aligned for one second")
            if row["terminal_time_s"]>cfg["max_search_time_s"]+manifest["recovery_config"]["max_total_time_s"]+.04:
                raise ValueError("Overall deadline exceeded")
            active=t["startup_phase"] & ~stop
            if active.any():
                q=t["qpos_rad"][active]
                cmd=t["command_rad_s"][active]
                anchor=t["qpos_rad"][0]
                excursion=np.zeros(6)
                excursion[[0,4]]=np.deg2rad(cfg["ring_radii_degrees"][-1])
                margin=cfg["joint_limit_margin_rad"]
                hold=np.clip(anchor,limits[:,0]+margin,limits[:,1]-margin)
                lower=np.minimum(np.maximum(limits[:,0]+margin,anchor-excursion),hold)
                upper=np.maximum(np.minimum(limits[:,1]-margin,anchor+excursion),hold)
                if (np.any(q<np.minimum(anchor,lower)-.005) or np.any(q>np.maximum(anchor,upper)+.005)
                        or np.max(np.abs(cmd))>cfg["max_joint_velocity_rad_s"]+1e-9
                        or np.any(q+dt*cmd<np.minimum(q,lower)-1e-9)
                        or np.any(q+dt*cmd>np.maximum(q,upper)+1e-9)):
                    raise ValueError("Search position/velocity/step bounds exceeded")
                if np.any(t["command_rad_s"][active & t["detected"]]):
                    raise ValueError("Search failed to brake immediately on detection")
                if t["time_s"][active][-1]>cfg["max_search_time_s"]+.04:
                    raise ValueError("Search deadline exceeded")


def analyze(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(s) for s in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    validate(directory,manifest,plan,rows)
    summary=summarize(rows)
    summary["trace_validation"]="PASS"
    write_json(directory/"summary.json",summary)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,9),layout="constrained")
    fig.suptitle("Filling gaps in startup search",fontsize=16,fontweight="bold")
    for ax,trial_id in zip(axes[0],(56,118)):
        pair=[r for r in rows if r["cohort"]=="regression" and r["trial_id"]==trial_id]
        if not pair:
            ax.set_visible(False)
            continue
        for row in pair:
            with np.load(trace_path(directory,row),allow_pickle=False) as t:
                mask=t["startup_phase"] & (t["phase"]!="post_stop")
                delta=np.rad2deg(t["qpos_rad"]-t["qpos_rad"][0])[:,[0,4]]
                if row["policy"]=="coarse":
                    ax.plot(delta[mask,0],delta[mask,1],color="#ba663c",alpha=.85,lw=1.3,label="Original sweep")
                else:
                    start=row["search_refinement_started_s"]
                    fine=mask & (t["time_s"]>=start-.003) if start is not None else np.zeros(len(mask),bool)
                    ax.plot(delta[fine,0],delta[fine,1],color="#008b7b",lw=2,label="Additional finer sweep")
                    if row["acquisition_s"] is not None:
                        i=int(np.argmin(np.abs(t["time_s"]-row["acquisition_s"])))
                        ax.scatter(*delta[i],s=70,marker="*",color="#008b7b",zorder=4,label="Marker acquired")
        ax.scatter(0,0,s=25,color="#222",label="Measured start")
        ax.set(title=f"Previous miss: trial {trial_id}",xlabel="J1 change from start (degrees)",
               ylabel="J5 change from start (degrees)",aspect="equal")
        ax.legend(fontsize=7,loc="lower left")
    for ax,cohort in zip(axes[1],("regression","holdout")):
        if cohort not in summary:
            ax.set_visible(False)
            continue
        group=summary[cohort]
        n=group["coarse"]["trials"]
        bars=ax.bar(["Original scan","With finer pass"],[group[p]["converged"] for p in ("coarse","refined")],
                    color=["#ba663c","#008b7b"],width=.5)
        ax.bar_label(bars,labels=[f"{group[p]['converged']}/{n}" for p in ("coarse","refined")],padding=4)
        ax.set(title="Original scenes" if cohort=="regression" else f"Fresh scenes (seed {manifest['holdout_seed']})",
               ylabel="Aligned and stable after stopping",ylim=(0,n*1.15))
    for ax in axes.flat:
        ax.spines[["right","top"]].set_visible(False)
        ax.grid(alpha=.15)
        ax.set_axisbelow(True)
    fig.savefig(directory/"coverage.png",dpi=150)
    fig.savefig(directory/"coverage.svg")
    plt.close(fig)
    table=[]
    for cohort in ("regression","holdout"):
        if cohort not in summary:
            continue
        g=summary[cohort]
        for key,label in (("converged","Aligned and stable"),("acquired_initially_undetected","Initially invisible, then acquired"),
                          ("aligned_initially_undetected","Initially invisible, then aligned")):
            denominator=g["coarse"]["trials"] if key=="converged" else g["coarse"]["initially_undetected"]
            table.append(f"| {cohort}: {label} | {format_rate(g['coarse'][key], denominator)} | "
                         f"{format_rate(g['refined'][key], denominator)} |")
    details=[]
    for row in rows:
        if row["policy"]=="refined" and row["case"]=="random_target" and row.get("search_refinement_started_s") is not None:
            details.append(f"- {row['cohort']} trial {row['trial_id']}: {row['outcome']}; acquisition {row['acquisition_s']} s; total {row['terminal_time_s']:.2f} s.")
    outcomes="\n".join(f"- {cohort}: improved {summary[cohort]['improved_trial_ids']}; regressed {summary[cohort]['regressed_trial_ids']}; outcomes {summary[cohort]['refined']['outcomes']}; negative controls {summary[cohort]['refined']['negative_control_outcomes']}."
                       for cohort in ("regression","holdout") if cohort in summary)
    report=f"""# Search coverage comparison

Both missed targets were visible within the existing +/-48-degree yaw/pitch box,
between the original coarse scan paths. The new policy preserves the original
12/24/36/48-degree rectangles, then scans the midpoint rings 6/18/30/42 degrees
if no marker was confirmed. It returns toward the measured start between passes.
No target world coordinates, home pose or remembered target view guide the search.

The regression comparison reuses 204 recorded baseline rows from the completed
joint-limit study and runs the refined policy anew. The fresh comparison runs both
policies on {manifest['holdout_trials']} newly sampled target-present scenes
(seed {manifest['holdout_seed']}), plus four negative controls per policy.
The fresh plan is saved before any fresh results are inspected. No sampled starts
are discarded. The two known misses were used during development; the original
200-scene result is a regression result, not a held-out estimate.

| Measurement | Original scan | With finer pass |
|---|---:|---:|
{chr(10).join(table)}

Brackets are 95% Clopper-Pearson (exact) confidence intervals for each rate. The two
policies ran on the same scenes, so compare them by the paired improved/regressed
trials below, not by whether their intervals overlap.

{outcomes}

![Search paths and paired outcomes](coverage.png)

## Time and motion tradeoff

The total search deadline is now 180 simulated seconds instead of 90. The old scan
usually finished in about 76 seconds. The extra pass receives the remaining shared
budget; its transition never resets the clock. It stops earlier if its path completes.
Speed remains 0.25 rad/s per joint, maximum startup excursion remains 48 degrees,
and mechanical margins remain 0.06 radians. Other joints hold the initial position.

Every camera detection interrupts movement immediately. Three consecutive detections
hand off to the unchanged joint-aware IBVS controller. Its separate 45-second budget
still includes any tracking recovery or alignment retry. Overall bounds therefore
permit up to 225 simulated seconds before termination.

{chr(10).join(details)}

## Verification and scope

Trace validation: PASS. Paired initial images/joints and all commands along the
original search prefix match exactly. Every scene acquired by the original scan
retains its entire trace and outcome. Search commands, requested steps, measured
bounds (0.005 rad actuator tolerance), deadlines, braking, terminal zero commands
and one-second stopped observations are checked. Every reported success remains
below 1 pixel during that observation.

Baseline runtime, scene, detector, reference, alignment and joint handling are
checked for compatibility. Configurations, source snapshots/hashes, paired traces,
the saved reference and all outcomes are retained. Recorded baseline trace hashes
are stored separately.

This is a finer finite scan, not exhaustive visibility coverage or guaranteed
alignment from arbitrary poses. Narrower gaps, occlusions and inaccessible views can
still cause misses. Collisions remain disabled in the teaching simulation.

Reproduce: python run_search_coverage.py --holdout-trials {manifest['holdout_trials']} --seed {manifest['holdout_seed']}
Rebuild this report: python run_search_coverage.py --analyze "PATH_TO_THIS_RUN"
"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def run(directory,baseline,holdout_trials,seed):
    old,regression,prior=load_baseline(baseline)
    configs={"coarse":dict(old["search_config"]),"refined":load_startup_config()}
    plan=[dict(s,cohort="regression") for s in regression]
    if holdout_trials:
        plan += [dict(s,cohort="holdout") for s in make_plan(holdout_trials,seed)]
    directory.mkdir(parents=True,exist_ok=False)
    for name in ("traces","cases","sources"):
        (directory/name).mkdir()
    sources=(*SOURCES,"run_search_coverage.py","app.py")
    manifest=dict(status="running",completed_rows=0,requested_rows=2*len(plan),
        created_utc=datetime.now(timezone.utc).isoformat(),baseline_directory=str(baseline.resolve()),
        holdout_trials=holdout_trials,holdout_seed=seed,search_configs=configs,
        recovery_config=old["recovery_config"],motion_config=old["motion_config"],
        source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in sources},
        reference_sha256=old["reference_sha256"],python=platform.python_version(),
        numpy=np.__version__,opencv=cv2.__version__,mujoco=mujoco.__version__,
        baseline_trace_sha256={})
    write_json(directory/"baseline_manifest.json",old)
    write_json(directory/"plan.json",plan)
    write_json(directory/"manifest.json",manifest)
    shutil.copyfile(baseline/"goal.npz",directory/"goal.npz")
    for name in sources:
        shutil.copyfile(ROOT/name,directory/"sources"/name)
    try:
        with Simulation() as sim:
            _,reference=load_reference(directory/"goal.npz",sim.camera_intrinsics(),sim.config,sim.marker_corners)
            manifest["joint_limits_rad"]=sim.model.jnt_range.tolist()
            manifest["simulation_config"]=sim.config
            write_json(directory/"manifest.json",manifest)
            with (directory/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for index,spec in enumerate(plan):
                    for policy in ("coarse","refined"):
                        if spec["cohort"]=="regression" and policy=="coarse":
                            row=dict(prior[index],cohort="regression",policy=policy)
                            source=baseline/"traces"/f"{spec['trial_id']:04d}-supervised.npz"
                            shutil.copyfile(source,trace_path(directory,row))
                            manifest["baseline_trace_sha256"][source.name]=hashlib.sha256(source.read_bytes()).hexdigest()
                        else:
                            row,trace,before,after=trial(sim,reference,spec,"search",configs[policy],old["recovery_config"],old["motion_config"])
                            row["policy"]=policy
                            np.savez_compressed(trace_path(directory,row),**trace)
                            if policy=="refined" and (row["search_refinement_started_s"] is not None or row["outcome"]!="converged"):
                                case=directory/"cases"/f"{spec['cohort']}-{spec['trial_id']:04d}"
                                case.mkdir()
                                Image.fromarray(before).save(case/"initial.png")
                                Image.fromarray(after).save(case/"final.png")
                                write_json(case/"trial.json",row)
                        stream.write(json.dumps(row,allow_nan=False)+"\n")
                        stream.flush()
                        manifest["completed_rows"]+=1
                        write_json(directory/"manifest.json",manifest)
                        print(f"{spec['cohort']} {spec['trial_id']} {policy}: {row['outcome']} | acquired {row['acquisition_s']}",flush=True)
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
    parser.add_argument("--holdout-trials",type=int,default=100)
    parser.add_argument("--seed",type=int,default=20260913)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--analyze",type=Path)
    args=parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    if args.holdout_trials<0:
        parser.error("--holdout-trials must be nonnegative")
    name=datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    directory=args.output or ROOT/"results/search-coverage"/name
    run(directory,args.baseline,args.holdout_trials,args.seed)
    try:
        location={"relative_to_project":directory.resolve().relative_to(ROOT).as_posix()}
    except ValueError:
        location={"absolute_path":str(directory.resolve())}
    write_json(ROOT/"results/search-coverage/latest.json",location)
    print(f"Saved search coverage study: {directory.resolve()}")


if __name__=="__main__":
    main()
