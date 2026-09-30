"""Seeded evaluation of find-and-align, including initially unseen targets."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
import platform
import tempfile
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

from analyze_benchmark import success_interval
from binomial_ci import format_rate, interpretation
from benchmark import BENCH_CONFIG, read_json, sample_plan, write_json
from recovery import ACTIVE_STATES, ReacquiringIBVS, load_recovery_config
from simulation import ROOT, Simulation

SOURCES = ("recovery.py", "recovery_config.json", "run_recovery.py", "app.py",
           "control.py", "simulation.py", "scene.xml", "config.json",
           "benchmark.py", "benchmark_config.json", "analyze_benchmark.py")


def source_hashes():
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCES}


def trial(sim, reference, taught_qpos, spec, cfg):
    sim.reset(spec["offset_degrees"])
    controller = ReacquiringIBVS(reference, sim.camera_intrinsics(), sim.config,
                                sim.model.jnt_range, taught_qpos, cfg)
    controller.start()
    dt = 1/sim.config["camera_hz"]
    data = {key: [] for key in ("time_s", "phase", "error_px", "detected", "corners_px",
                                "qpos_rad", "qvel_rad_s", "command_rad_s", "camera_pose_world")}
    transitions = []
    first = sim.image()
    first_seen = None
    first_confirmed = None

    def record(rgb, corners, phase, command):
        nonlocal first_seen
        error = np.nan if corners is None else float(np.sqrt(np.mean(np.sum((corners-reference)**2, axis=1))))
        if corners is not None and first_seen is None:
            first_seen = float(sim.data.time)
        values = (float(sim.data.time), phase, error, corners is not None,
                  np.full((4,2), np.nan) if corners is None else corners.copy(),
                  sim.data.qpos.copy(), sim.data.qvel.copy(), command.copy(), sim.camera_pose().copy())
        for key, value in zip(data, values):data[key].append(value)
        if not transitions or transitions[-1]["phase"] != phase:
            transitions.append({"time_s":float(sim.data.time), "phase":phase})
        return error

    for tick in range(int(np.ceil(cfg["max_total_time_s"]/dt))+2):
        rgb = first if tick == 0 else sim.image()
        corners = sim.marker_corners(rgb)
        sample = controller.update(corners, sim.camera_jacobian(), sim.data.qpos, dt)
        if controller.reacquisitions and first_confirmed is None:
            first_confirmed = float(sim.data.time)
        record(rgb, corners, sample.status, sample.velocity)
        sim.command_velocity(sample.velocity)
        if sample.status not in ACTIVE_STATES:break
        sim.advance(dt)
    else:
        raise RuntimeError("Controller failed to respect its total deadline")
    outcome = sample.status
    terminal_time = float(sim.data.time)
    if outcome == "converged":
        for _ in range(sim.config["camera_hz"]):
            sim.command_velocity(np.zeros(6))
            sim.advance(dt)
            rgb = sim.image()
            corners = sim.marker_corners(rgb)
            error = record(rgb, corners, "post_stop", np.zeros(6))
            if not np.isfinite(error) or error >= sim.config["ibvs"]["success_error_px"]:
                outcome = "unstable_after_stop"
    sim.command_velocity(np.zeros(6))
    arrays = {key:np.asarray(value) for key,value in data.items()}
    summary = {
        "trial_id":spec["trial_id"], "profile":spec["profile"], "offset_degrees":spec["offset_degrees"],
        "outcome":outcome, "initial_detected":bool(arrays["detected"][0]),
        "initial_error_px":float(arrays["error_px"][0]) if arrays["detected"][0] else None,
        "final_error_px":float(arrays["error_px"][-1]) if arrays["detected"][-1] else None,
        "first_detection_s":first_seen, "first_confirmed_reacquisition_s":first_confirmed,
        "terminal_time_s":terminal_time, "observed_time_s":float(sim.data.time),
        "recovery_episodes":controller.episodes, "reacquisitions":controller.reacquisitions,
        "recovery_time_s":controller.recovery_elapsed_s,
        "used_scan":bool(np.any(arrays["phase"] == "scanning")),
        "sample_count":len(arrays["time_s"]), "transitions":transitions,
    }
    return summary, arrays, first, rgb


def analyze(directory):
    manifest = read_json(directory/"manifest.json")
    rows = [json.loads(line) for line in (directory/"trials.jsonl").read_text().splitlines()]
    plan = read_json(directory/"plan.json")
    if manifest["status"] != "complete" or len(rows) != manifest["requested_trials"] or len(rows) != manifest["completed_trials"]:
        raise ValueError("Incomplete recovery run")
    if [r["trial_id"] for r in rows] != [r["trial_id"] for r in plan]:
        raise ValueError("Trial IDs differ from saved plan")
    for row in rows:
        with np.load(directory/"traces"/f"trial_{row['trial_id']:04d}.npz",allow_pickle=False) as trace:
            if any(len(trace[key]) != row["sample_count"] for key in trace.files):
                raise ValueError("Inconsistent trace lengths")
            if np.any(np.diff(trace["time_s"]) <= 0) or not np.isfinite(trace["command_rad_s"]).all() or not np.isfinite(trace["qpos_rad"]).all():
                raise ValueError("Invalid trace values")
            if np.any(trace["command_rad_s"][-1]):
                raise ValueError("Terminal command must be zero")
            if row["outcome"] == "converged":
                post = trace["phase"] == "post_stop"
                if post.sum() != manifest["simulation_config"]["camera_hz"] or not np.all(trace["error_px"][post] < 1):
                    raise ValueError("Successful trial lacks a stable stopped second")
    success = [r for r in rows if r["outcome"]=="converged"]
    unseen = [r for r in rows if not r["initial_detected"]]
    recovered = [r for r in unseen if r["outcome"]=="converged"]
    counts = dict(Counter(r["outcome"] for r in rows))
    summary = {
        "trials":len(rows), "successes":len(success), "success_rate":len(success)/len(rows),
        "success_ci95":success_interval(len(success),len(rows)),
        "interval_method":"Clopper-Pearson (exact), two-sided",
        "initially_undetected":len(unseen), "undetected_starts_converged":len(recovered),
        "undetected_success_ci95":success_interval(len(recovered),len(unseen)),
        "median_convergence_s_successes":float(np.median([r["terminal_time_s"] for r in success])) if success else None,
        "median_final_error_px_successes":float(np.median([r["final_error_px"] for r in success])) if success else None,
        "median_first_confirmed_reacquisition_s":float(np.median([r["first_confirmed_reacquisition_s"] for r in recovered])) if recovered else None,
        "outcomes":counts, "trace_validation":"PASS",
    }
    write_json(directory/"summary.json", summary)
    with (directory/"trials.csv").open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({k:json.dumps(v) if isinstance(v,list) else v for k,v in row.items()} for row in rows)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11.5,4.6),layout="constrained")
    fig.suptitle(f"Find and align | {len(rows)} random starting poses",fontsize=15,fontweight="bold")
    profiles=list(manifest["sampling_config"]["profiles_degrees"])
    x=np.arange(len(profiles))
    for j,detected in enumerate((True,False)):
        totals=[sum(r["profile"]==p and r["initial_detected"]==detected for r in rows) for p in profiles]
        values=[sum(r["profile"]==p and r["initial_detected"]==detected and r["outcome"]=="converged" for r in rows) for p in profiles]
        bars=axes[0].bar(x+(j-.5)*.36,values,width=.32,color=("#087f8c","#d08b25")[j],
                        label=("Initially detected","Initially undetected")[j])
        axes[0].bar_label(bars,labels=[f"{a}/{b}" for a,b in zip(values,totals)],padding=3)
    axes[0].set(xticks=x,xticklabels=profiles,ylabel="Successful trials",title="Aligned / sampled starts in each group")
    axes[0].set_ylim(0,max(1,axes[0].get_ylim()[1])*1.18)
    axes[0].legend(loc="upper left",fontsize=8)
    for detected,color in ((True,"#087f8c"),(False,"#d08b25")):
        group=[r for r in success if r["initial_detected"]==detected]
        if group:
            axes[1].scatter([r["first_detection_s"] for r in group],[r["terminal_time_s"] for r in group],
                            color=color,s=25,alpha=.7,label="Initially detected" if detected else "Initially undetected")
    axes[1].set(xlabel="First marker detection (simulated s)",ylabel="Convergence (simulated s)",
                title="Successful trials")
    if success:axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.15)
        ax.spines[["top","right"]].set_visible(False)
    fig.savefig(directory/"recovery_overview.png",dpi=160)
    plt.close(fig)
    rate=summary["success_rate"]
    lo,hi=summary["success_ci95"]
    report=f"""# Find-and-align evaluation

Completed {len(rows)} trials with seed {manifest['seed']}. Every sampled start is counted.

- Converged and stayed below 1 px after stopping: **{len(success)}/{len(rows)} ({100*rate:.1f}%)**.
- 95% Clopper-Pearson (exact) interval for the declared distribution: {100*lo:.1f}-{100*hi:.1f}%. {interpretation(len(success),len(rows))}
- Starts without a detected marker: **{len(unseen)}**; converged among those: {format_rate(len(recovered),len(unseen))}.
- Median convergence time, successful trials only: {summary['median_convergence_s_successes']} simulated seconds.
- Median final error, successful trials only: {summary['median_final_error_px_successes']} px.

![Recovery overview](recovery_overview.png)

## Recovery policy and its scope

Each trial is given the measured joint configuration of the taught camera view, where the marker was observed before the trial. During recovery, joint feedback moves toward the most recent observed viewpoint. This is an explicit joint-space recovery phase; it is not pure image-based control while the marker is absent. Once the detector returns three consecutive valid marker frames, unchanged IBVS resumes.

A 0.2-second stationary wait handles brief loss. Recovery commands are bounded to 0.18 rad/s per joint, with a 60-degree per-joint excursion bound, joint-limit margins, a 20-second cumulative recovery budget and a 45-second overall deadline. A local yaw/pitch scan follows if the remembered viewpoint does not reveal the marker. Stop, Pause and manual jogging cancel automatic recovery in the desktop app.

No target pose, ground-truth visibility flag or marker-corner prediction is used for control. Ground-truth joint state supplies the simulated encoder measurements and kinematics. Motion is through velocity actuators; initialization is the only trial reset.

The scene is static, has ideal gravity compensation and disabled robot collisions. The remembered viewpoint is valid in this experiment; this is not evidence for searching an arbitrary environment or collision-safe physical motion.

## Reproduction and saved data

The plan, configuration, taught viewpoint, source hashes and versions are in plan.json and manifest.json. Per-frame traces are numeric NPZ files (load with allow_pickle=False); missing pixel measurements are NaN. trials.jsonl and trials.csv record each outcome and state transitions. summary.json includes all outcomes and confidence intervals. Example images are saved from the first occurrence of each case category.

From the configured project Python environment:

    python run_recovery.py --trials {len(rows)} --seed {manifest['seed']}
    python run_recovery.py --analyze "PATH_TO_THIS_RUN"

"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2))
    return summary


def run(output, plan, config, sampling, seed):
    output.mkdir(parents=True,exist_ok=False)
    (output/"traces").mkdir()
    (output/"cases").mkdir()
    manifest={"status":"running","requested_trials":len(plan),"completed_trials":0,"seed":seed,
              "created_utc":datetime.now(timezone.utc).isoformat(),"recovery_config":config,
              "sampling_config":sampling,"source_sha256":source_hashes(),
              "python":platform.python_version(),"mujoco":mujoco.__version__,
              "opencv":cv2.__version__,"numpy":np.__version__}
    write_json(output/"plan.json",plan)
    write_json(output/"manifest.json",manifest)
    categories=set()
    try:
        with Simulation() as sim:
            reference_rgb=sim.image()
            reference=sim.marker_corners(reference_rgb)
            if reference is None:raise RuntimeError("Taught marker must be detected")
            taught=sim.data.qpos.copy()
            manifest.update(simulation_config=sim.config,desired_corners_px=reference.tolist(),
                            taught_qpos_rad=taught.tolist(),camera_K=sim.camera_intrinsics().tolist())
            write_json(output/"manifest.json",manifest)
            Image.fromarray(reference_rgb).save(output/"reference.png")
            with (output/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for spec in plan:
                    row,arrays,before,after=trial(sim,reference,taught,spec,config)
                    np.savez_compressed(output/"traces"/f"trial_{spec['trial_id']:04d}.npz",**arrays)
                    stream.write(json.dumps(row,allow_nan=False)+"\n")
                    stream.flush()
                    category=row["outcome"]
                    if category=="converged":
                        category="converged_visible" if row["initial_detected"] else "recovered"
                    if category not in categories:
                        case=output/"cases"/category
                        case.mkdir()
                        Image.fromarray(before).save(case/"initial.png")
                        Image.fromarray(after).save(case/"final.png")
                        write_json(case/"trial.json",row)
                        categories.add(category)
                    manifest["completed_trials"]+=1
                    write_json(output/"manifest.json",manifest)
                    if spec["trial_id"]%10==0 or row["outcome"]!="converged":
                        print(f"{manifest['completed_trials']}/{len(plan)} | {row['outcome']} | initially detected {row['initial_detected']} | {row['terminal_time_s']:.2f} s",flush=True)
        manifest["status"]="complete"
    except BaseException as exc:
        manifest["status"]="interrupted" if isinstance(exc,KeyboardInterrupt) else "error"
        manifest["error"]=f"{type(exc).__name__}: {exc}"
        raise
    finally:
        manifest["finished_utc"]=datetime.now(timezone.utc).isoformat()
        write_json(output/"manifest.json",manifest)
    analyze(output)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials",type=int,default=200)
    parser.add_argument("--seed",type=int,default=20260906)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--analyze",type=Path)
    args=parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    config=load_recovery_config()
    sampling=read_json(BENCH_CONFIG)
    plan=sample_plan(args.trials,args.seed,sampling)
    name=datetime.now().strftime("%Y%m%d-%H%M%S-%f")+f"-n{args.trials}-seed{args.seed}"
    output=args.output or ROOT/"results/recovery"/name
    run(output,plan,config,sampling,args.seed)
    (ROOT/"results/recovery").mkdir(parents=True,exist_ok=True)
    try:location={"relative_to_project":output.resolve().relative_to(ROOT).as_posix()}
    except ValueError:location={"absolute_path":str(output.resolve())}
    write_json(ROOT/"results/recovery/latest.json",location)
    print(f"Saved recovery run: {output.resolve()}")


if __name__=="__main__":
    main()

