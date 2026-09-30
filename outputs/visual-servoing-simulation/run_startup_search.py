"""Paired startup-search evaluation with randomized joints and target placement."""
from __future__ import annotations
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import tempfile

import cv2
import mujoco
import numpy as np
from PIL import Image

from benchmark import BENCH_CONFIG, read_json, sample_plan, write_json
from binomial_ci import clopper_pearson, format_rate
from control import IBVSController
from recovery import ACTIVE_STATES, ReacquiringIBVS, load_recovery_config
from reference_image import DEFAULT_REFERENCE, load_reference
from simulation import ROOT, Simulation
from startup_search import load_startup_config
from joint_limits import load_joint_limit_config

TARGET_SHIFT_BOUNDS_M = [.06,.18,.10]
SOURCES = ("startup_search.py","startup_search_config.json","recovery.py",
           "recovery_config.json","reference_image.py","control.py","config.json",
           "simulation.py","scene.xml","benchmark.py","benchmark_config.json",
           "run_startup_search.py","joint_limits.py","joint_limit_config.json")


def make_plan(n, seed):
    # Independent random streams keep joint samples identical to the old benchmark.
    plan=sample_plan(n,seed,read_json(BENCH_CONFIG))
    rng=np.random.default_rng(np.random.SeedSequence([seed,731]))
    for spec in plan:
        spec.update(case="random_target",target_shift_m=rng.uniform(
            -np.array(TARGET_SHIFT_BOUNDS_M),TARGET_SHIFT_BOUNDS_M).tolist())
    for case,offset,shift in (
        ("absent_marker",[0]*6,[0,0,0]),
        ("absent_marker",[35,0,0,0,0,0],[0,0,0]),
        ("unreachable_target",[0]*6,[0,2,0]),
        ("unreachable_target",[-35,0,0,0,0,0],[0,-2,0])):
        plan.append(dict(trial_id=len(plan)+1,profile="negative_control",case=case,
                         offset_degrees=offset,target_shift_m=shift))
    return plan


def trial(sim, reference, spec, method, search_config, recovery_config, motion_config=None, detector=None):
    # The default is the original ArUco path; other perception backends return
    # the same four stable image features without receiving target scene data.
    detect = sim.marker_corners if detector is None else detector
    target=sim.model.body("target")
    board=sim.model.geom("target_board")
    original_pos=target.pos.copy()
    original_rgba=board.rgba.copy()
    try:
        # Scene randomization is evaluation setup, never input to either controller.
        target.pos[:]=original_pos+spec["target_shift_m"]
        if spec["case"]=="absent_marker":
            board.rgba[3]=0
        sim.reset(spec["offset_degrees"])
        if method=="search":
            controller=ReacquiringIBVS(reference,sim.camera_intrinsics(),sim.config,
                sim.model.jnt_range,startup_config=search_config,recovery_config=recovery_config,motion_config=motion_config)
            controller.start(cold=True)
            assert controller.last_visible_qpos is None
        elif method=="direct":
            controller=IBVSController(reference,sim.camera_intrinsics(),sim.config)
        else:
            raise ValueError("Unknown method")
        dt=1/sim.config["camera_hz"]
        keys=("time_s","phase","error_px","detected","corners_px","qpos_rad",
              "qvel_rad_s","command_rad_s","startup_phase","waypoint_index","joint_margin_rad","retry_count","search_stage")
        values={key:[] for key in keys}
        first=sim.image()
        first_detection=None
        acquired=None
        deadline=search_config["max_search_time_s"]+recovery_config["max_total_time_s"]
        def record(rgb,corners,phase,command):
            nonlocal first_detection
            if corners is not None and first_detection is None:
                first_detection=float(sim.data.time)
            startup=(method=="search" and controller.mode=="startup")
            index=controller.startup.index if method=="search" and controller.startup else -1
            error=np.nan if corners is None else float(np.sqrt(np.mean(np.sum((corners-reference)**2,axis=1))))
            row=(float(sim.data.time),phase,error,corners is not None,
                 np.full((4,2),np.nan) if corners is None else corners.copy(),
                 sim.data.qpos.copy(),sim.data.qvel.copy(),command.copy(),startup,index,
                 np.minimum(sim.data.qpos-sim.model.jnt_range[:,0],sim.model.jnt_range[:,1]-sim.data.qpos),
                 controller.motion.retries if method=="search" else 0,
                 controller.startup.stage if method=="search" and controller.startup else "none")
            for k,v in zip(keys,row):
                values[k].append(v)
            return error
        for tick in range(int(np.ceil(deadline/dt))+4):
            rgb=first if tick==0 else sim.image()
            corners=detect(rgb)
            if method=="search":
                sample=controller.update(corners,sim.camera_jacobian(),sim.data.qpos,dt)
                if controller.startup and controller.startup.acquired_s is not None and acquired is None:
                    acquired=float(sim.data.time)
            else:
                sample=controller.update(corners,sim.camera_jacobian(),dt)
                if corners is not None and acquired is None:
                    acquired=float(sim.data.time)
            record(rgb,corners,sample.status,sample.velocity)
            sim.command_velocity(sample.velocity)
            if sample.status not in ACTIVE_STATES:
                break
            sim.advance(dt)
        else:
            raise RuntimeError("Controller exceeded its declared total budget")
        terminal_time=float(sim.data.time)
        outcome=sample.status
        # Every outcome, including failure, is observed for one second after stopping.
        stable=True
        for _ in range(sim.config["camera_hz"]):
            sim.command_velocity(np.zeros(6))
            sim.advance(dt)
            rgb=sim.image()
            corners=detect(rgb)
            error=record(rgb,corners,"post_stop",np.zeros(6))
            if not np.isfinite(error) or error>=sim.config["ibvs"]["success_error_px"]:
                stable=False
        if outcome=="converged" and not stable:
            outcome="unstable_after_stop"
        arrays={k:np.asarray(v) for k,v in values.items()}
        searching=arrays["startup_phase"] & (arrays["phase"]!="post_stop")
        bounds_ok=True
        max_excursion=0.
        search=controller.startup if method=="search" else None
        if search is not None and np.any(searching):
            q=arrays["qpos_rad"][searching]
            # Encoder samples may lag commanded bounds slightly due actuator dynamics.
            bounds_ok=bool(np.all(q>=np.minimum(search.lower,search.anchor)-.005)
                           and np.all(q<=np.maximum(search.upper,search.anchor)+.005))
            max_excursion=float(np.max(np.abs(q-search.anchor)))
        row=dict(spec,method=method,outcome=outcome,
            initial_detected=bool(arrays["detected"][0]),first_detection_s=first_detection,
            acquisition_s=acquired,terminal_time_s=terminal_time,
            alignment_after_acquisition_s=None if acquired is None else terminal_time-acquired,
            final_error_px=None if not np.isfinite(error) else error,
            sample_count=len(arrays["time_s"]),started_without_memory=method=="search",
            search_stop_reason=search.stop_reason if search else None,
            search_refinement_started_s=search.refinement_started_s if search else None,
            search_bounds_ok=bounds_ok,max_search_excursion_rad=max_excursion,
            max_search_command_rad_s=float(np.max(np.abs(arrays["command_rad_s"][searching]))) if np.any(searching) else 0.,
            max_command_rad_s=float(np.max(np.abs(arrays["command_rad_s"]))),
            stopped_command_zero=bool(np.all(arrays["command_rad_s"][arrays["phase"]=="post_stop"]==0)))
        if method=="search":
            row.update(joint_supervision_enabled=controller.motion.config["enabled"],
                       adjusted_alignment_frames=controller.motion.adjusted_frames,
                       alignment_retries=controller.motion.retries,stall_events=controller.motion.stalls,
                       motion_events=controller.motion.events,
                       min_joint_margin_rad=float(np.min(arrays["joint_margin_rad"])))
        return row,arrays,first,rgb
    finally:
        sim.command_velocity(np.zeros(6))
        target.pos[:]=original_pos
        board.rgba[:]=original_rgba
        sim.reset()


def validate_run(directory,manifest,plan,rows):
    if manifest["status"]!="complete" or len(rows)!=manifest["completed_runs"] or len(rows)!=2*len(plan):
        raise ValueError("Cannot report an incomplete paired run")
    if [(r["trial_id"],r["method"]) for r in rows] != [(s["trial_id"],m) for s in plan for m in ("direct","search")]:
        raise ValueError("Trial IDs or pair order differ from saved plan")
    for spec in plan:
        pair=[r for r in rows if r["trial_id"]==spec["trial_id"]]
        if pair[0]["initial_detected"]!=pair[1]["initial_detected"]:
            raise ValueError("Paired runs started with different detection states")
    for row in rows:
        path=directory/"traces"/f"{row['trial_id']:04d}-{row['method']}.npz"
        with np.load(path,allow_pickle=False) as trace:
            if any(len(trace[k])!=row["sample_count"] for k in trace.files):
                raise ValueError("Trace length mismatch")
            if not np.all(np.diff(trace["time_s"])>0) or not np.isfinite(trace["qpos_rad"]).all():
                raise ValueError("Invalid times or joint measurements")
            if not np.isfinite(trace["command_rad_s"]).all() or row["max_command_rad_s"]>.35+1e-9:
                raise ValueError("Invalid or excessive command")
            if not row["search_bounds_ok"] or row["max_search_command_rad_s"]>manifest["search_config"]["max_joint_velocity_rad_s"]+1e-9:
                raise ValueError("Startup scan exceeded declared bounds")
            stop=trace["phase"]=="post_stop"
            if stop.sum()!=manifest["simulation_config"]["camera_hz"] or np.any(trace["command_rad_s"][stop]):
                raise ValueError("Missing stationary post-stop observation")
            if row["outcome"]=="converged":
                if not np.all(np.isfinite(trace["error_px"][stop])) or np.any(trace["error_px"][stop]>=1):
                    raise ValueError("Successful run failed stopped verification")
            terminal=np.flatnonzero(~stop)[-1]
            if np.any(trace["command_rad_s"][terminal]):
                raise ValueError("Terminal command is not zero")


def summarize(rows):
    def median(values):
        return float(np.median(values)) if values else None
    result={}
    for method in ("direct","search"):
        group=[r for r in rows if r["method"]==method and r["case"]=="random_target"]
        successes=[r for r in group if r["outcome"]=="converged"]
        unseen=[r for r in group if not r["initial_detected"]]
        acquired=[r for r in unseen if r["acquisition_s"] is not None]
        negatives=[r for r in rows if r["method"]==method and r["case"]!="random_target"]
        result[method]=dict(trials=len(group),converged=len(successes),
            converged_ci95=clopper_pearson(len(successes),len(group)),interval_method="Clopper-Pearson (exact), two-sided",
            initially_undetected=len(unseen),acquired_initially_undetected=len(acquired),
            aligned_initially_undetected=sum(r["outcome"]=="converged" for r in unseen),
            median_acquisition_s_initially_undetected=median([r["acquisition_s"] for r in acquired]),
            median_total_s_successes=median([r["terminal_time_s"] for r in successes]),
            median_alignment_s_successes=median([r["alignment_after_acquisition_s"] for r in successes]),
            outcomes=dict(Counter(r["outcome"] for r in group)),
            negative_controls=len(negatives),
            negative_control_outcomes=dict(Counter(r["outcome"] for r in negatives)),
            negative_controls_all_stopped=all(r["stopped_command_zero"] for r in negatives))
    return result


def analyze(directory):
    manifest=read_json(directory/"manifest.json")
    plan=read_json(directory/"plan.json")
    rows=[json.loads(line) for line in (directory/"trials.jsonl").read_text(encoding="utf-8").splitlines()]
    validate_run(directory,manifest,plan,rows)
    summary=summarize(rows)
    summary["trace_validation"]="PASS"
    write_json(directory/"summary.json",summary)
    os.environ.setdefault("MPLCONFIGDIR",str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4.8),layout="constrained")
    fig.suptitle("Startup search | randomized joints and target placement",fontsize=15,fontweight="bold")
    methods=("direct","search")
    totals=[summary[m]["trials"] for m in methods]
    bars=axes[0].bar(["Direct IBVS","Search + IBVS"],[summary[m]["converged"] for m in methods],
                     color=["#8b98aa","#098879"],width=.55)
    axes[0].bar_label(bars,labels=[f"{summary[m]['converged']}/{n}" for m,n in zip(methods,totals)],padding=4)
    axes[0].set(ylabel="Aligned and stable after stopping",ylim=(0,max(totals)*1.14))
    for detected,color in ((True,"#098879"),(False,"#cd852a")):
        group=[r for r in rows if r["method"]=="search" and r["case"]=="random_target"
               and r["outcome"]=="converged" and r["initial_detected"]==detected]
        if group:
            axes[1].scatter([r["acquisition_s"] for r in group],[r["terminal_time_s"] for r in group],
                s=24,alpha=.7,color=color,label="Initially visible" if detected else "Initially undetected")
    axes[1].set(xlabel="Confirmed acquisition (simulated s)",ylabel="Total time to align (simulated s)")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.spines[["right","top"]].set_visible(False)
        ax.grid(axis="y",alpha=.15)
    fig.savefig(directory/"overview.png",dpi=150)
    fig.savefig(directory/"overview.svg")
    plt.close(fig)
    a,b=summary["direct"],summary["search"]
    report=f"""# Systematic startup-search evaluation

Seed {manifest['seed']}; {len(plan)-4} randomized scenes, each run with both methods.
No sampled scene was rejected for visibility or reachability. Four negative controls
are reported separately from the randomized target-present group.

| Measurement | Direct IBVS | Startup search + IBVS |
|---|---:|---:|
| Aligned and stable for 1 s after stopping | {format_rate(a['converged'],a['trials'])} | {format_rate(b['converged'],b['trials'])} |
| Initially undetected | {a['initially_undetected']} | {b['initially_undetected']} |
| Initially undetected, then acquired | {format_rate(a['acquired_initially_undetected'],a['initially_undetected'])} | {format_rate(b['acquired_initially_undetected'],b['initially_undetected'])} |
| Initially undetected, then aligned | {format_rate(a['aligned_initially_undetected'],a['initially_undetected'])} | {format_rate(b['aligned_initially_undetected'],b['initially_undetected'])} |

Counts show the 95% Clopper-Pearson (exact) confidence interval for the true rate; see docs/STATISTICS.md.

![Paired results](overview.png)

Search acquisition median among initially undetected scenes that were acquired:
{b['median_acquisition_s_initially_undetected']} simulated seconds.
Successful search runs: median total time {b['median_total_s_successes']} s;
median time from confirmed acquisition to convergence {b['median_alignment_s_successes']} s.
Medians describe their stated groups; they are not paired speed comparisons.

Target translations are independent uniform offsets within +/- {TARGET_SHIFT_BOUNDS_M} m
in world X/Y/Z from the original scene. Robot joint boxes are unchanged from the
original benchmark. Every method receives the same saved image goal and camera
calibration. Search begins without a remembered robot viewpoint. Target position
is used only to arrange the test scene; it is never supplied to the controller.
The reference is not recaptured between trials. Gain is fixed at 1.2/s for both methods.

Two negative controls remove the marker board from rendering; two place the target
two metres sideways, beyond the arm\'s reach. The distant target can still be\nvisible; these cases test bounded failure after detection as well as during search.
Search negative-control outcomes: {b['negative_control_outcomes']}.
All negative controls had zero commanded velocity throughout the one-second post-stop observation:
{b['negative_controls_all_stopped']}.

The search follows expanding yaw/pitch rectangles relative to the measured starting
joints, with configured coarse radii {manifest['search_config']['ring_radii_degrees']} degrees.
Refinement after the coarse pass is {manifest['search_config'].get('refine_after_coarse',False)}:
when enabled, a second pass bisects the gaps between those rings.
It checks images during motion, brakes on detection, and requires
{manifest['search_config']['confirmation_frames']} consecutive frames. Its speed cap is
{manifest['search_config']['max_joint_velocity_rad_s']} rad/s, with
{manifest['search_config']['joint_limit_margin_rad']} rad mechanical margins and a
{manifest['search_config']['max_search_time_s']}-second acquisition deadline.
After acquisition the existing IBVS/recovery controller has its own 45-second
overall budget (IBVS has a 20-second alignment timeout per episode).

This samples a limited static scene distribution; it does not establish arbitrary
workspace coverage. A two-joint scan can fail because of orientation, occlusion,
reachability or limited search time. Robot collisions remain disabled in this
teaching simulation, so these results do not establish collision-safe physical motion.

Outcomes: {b['outcomes']}. Trace validation: PASS.
The manifest records configurations, source/reference hashes and library versions.
plan.json contains every scene; trials.jsonl contains every outcome; traces/ contains
numeric per-frame NPZ logs, including commands, detections, joint feedback and phases.
cases/ contains representative initial and final rendered images.

Reproduce: python run_startup_search.py --trials {len(plan)-4} --seed {manifest['seed']} --reference "PATH_TO_THIS_RUN/goal.npz"
Rebuild this report: python run_startup_search.py --analyze "PATH_TO_THIS_RUN"
"""
    (directory/"REPORT.md").write_text(report,encoding="utf-8")
    print(json.dumps(summary,indent=2),flush=True)
    return summary


def run(directory,plan,seed,reference_path=DEFAULT_REFERENCE):
    directory.mkdir(parents=True,exist_ok=False)
    (directory/"traces").mkdir()
    (directory/"cases").mkdir()
    search_config=load_startup_config()
    recovery_config=load_recovery_config()
    manifest=dict(status="running",seed=seed,requested_runs=2*len(plan),completed_runs=0,
        created_utc=datetime.now(timezone.utc).isoformat(),
        search_config=search_config,recovery_config=recovery_config,motion_config=load_joint_limit_config(),
        sampling_config=read_json(BENCH_CONFIG),target_shift_bounds_m=TARGET_SHIFT_BOUNDS_M,
        source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES},
        reference_sha256=hashlib.sha256(Path(reference_path).read_bytes()).hexdigest(),
        python=platform.python_version(),mujoco=mujoco.__version__,opencv=cv2.__version__,numpy=np.__version__)
    write_json(directory/"plan.json",plan)
    write_json(directory/"manifest.json",manifest)
    (directory/"goal.npz").write_bytes(Path(reference_path).read_bytes())
    categories=set()
    try:
        with Simulation() as sim:
            rgb,reference=load_reference(reference_path,sim.camera_intrinsics(),sim.config,sim.marker_corners)
            manifest.update(simulation_config=sim.config,camera_K=sim.camera_intrinsics().tolist(),
                            desired_corners_px=reference.tolist())
            Image.fromarray(rgb).save(directory/"reference.png")
            write_json(directory/"manifest.json",manifest)
            with (directory/"trials.jsonl").open("w",encoding="utf-8") as stream:
                for spec in plan:
                    for method in ("direct","search"):
                        row,arrays,before,after=trial(sim,reference,spec,method,search_config,recovery_config)
                        np.savez_compressed(directory/"traces"/f"{spec['trial_id']:04d}-{method}.npz",**arrays)
                        stream.write(json.dumps(row,allow_nan=False)+"\n")
                        stream.flush()
                        category=f"{method}-{spec['case']}-{row['outcome']}-{'visible' if row['initial_detected'] else 'unseen'}"
                        if category not in categories:
                            case=directory/"cases"/category
                            case.mkdir()
                            Image.fromarray(before).save(case/"initial.png")
                            Image.fromarray(after).save(case/"final.png")
                            write_json(case/"trial.json",row)
                            categories.add(category)
                        manifest["completed_runs"]+=1
                        write_json(directory/"manifest.json",manifest)
                        if method=="search":
                            print(f"{spec['trial_id']}/{len(plan)} | {row['outcome']} | initial detection {row['initial_detected']} | acquisition {row['acquisition_s']} | {row['terminal_time_s']:.2f}s",flush=True)
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
    parser.add_argument("--trials",type=int,default=200,help="Random target scenes; each has two paired runs, plus four negative controls")
    parser.add_argument("--seed",type=int,default=20260911)
    parser.add_argument("--output",type=Path)
    parser.add_argument("--analyze",type=Path)
    parser.add_argument("--reference",type=Path,default=DEFAULT_REFERENCE)
    args=parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
        return
    name=datetime.now().strftime("%Y%m%d-%H%M%S-%f")+f"-n{args.trials}-seed{args.seed}"
    directory=args.output or ROOT/"results/startup"/name
    run(directory,make_plan(args.trials,args.seed),args.seed,args.reference)
    try:
        location={"relative_to_project":directory.resolve().relative_to(ROOT).as_posix()}
    except ValueError:
        location={"absolute_path":str(directory.resolve())}
    write_json(ROOT/"results/startup/latest.json",location)
    print(f"Saved startup evaluation: {directory.resolve()}")

if __name__=="__main__":
    main()
