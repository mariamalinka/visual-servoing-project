"""Run/resume paired physical accuracy and controller-calibration experiments."""
from __future__ import annotations
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import time
import traceback
import numpy as np
from accuracy import ROOT, CONFIG_PATH, make_plan, pose_accuracy, physical_pass, report, validate_rows
from camera_robustness import digest, write_json
from camera_timing import load_camera_timing
from run_camera_robustness import fingerprint, error_px


def prepare_goals(sim,directory,modes):
    from app import make_perception
    from reference_image import save_reference
    goals = {}
    for target in dict.fromkeys("aruco" if mode=="aruco" else "natural" for mode in modes):
        sim.reset()
        sim.set_target_mode(target)
        perception = make_perception(target,sim)
        path = directory/"reference"/(target+".npz")
        rgb = sim.image()
        save_reference(path,rgb,sim.camera_intrinsics(),perception.reference_config(sim.config),
                       lambda image:perception.observe(image).corners)
        goals[target] = dict(camera_world=sim.camera_pose().tolist(),tool_world=sim.tool_pose().tolist(),
                             tool_frame="tool_roll body origin",qpos_rad=sim.data.qpos.tolist(),
                             reference=path.relative_to(directory).as_posix(),reference_sha256=digest(path),
                             true_K=sim.camera_intrinsics().tolist(),true_target_side_m=sim.config["marker_side_m"])
    write_json(directory/"goals.json",goals)
    return goals


def validate_goals(directory,manifest):
    if digest(directory/"goals.json") != manifest["goals_sha256"]:
        raise ValueError("Evaluation goals changed; create a new experiment")
    goals = json.loads((directory/"goals.json").read_text())
    for goal in goals.values():
        if digest(directory/goal["reference"]) != goal["reference_sha256"]:
            raise ValueError("Private reference changed; create a new experiment")
    return goals


def run_trial(sim,spec,plan,goals,directory):
    import cv2
    from PIL import Image
    from app import Lab
    sim.set_obstacle(False)
    sim.reset()
    cv2.setRNGSeed(spec["perception_seed"])
    goal = goals["aruco" if spec["mode"]=="aruco" else "natural"]
    timing = dict(load_camera_timing(plan["camera_delay_s"]),max_run_s=plan["max_trial_s"])
    lab = Lab(sim,auto_start=False,perception_mode=spec["mode"],
              reference_path=directory/goal["reference"],camera_timing=timing,calibration=spec["calibration"],
              precision=plan.get("precision_enabled",True))
    sim.reset(spec["offset_degrees"])
    lab.align()
    dt = float(sim.model.opt.timestep)
    previous_q = sim.data.qpos.copy()
    travel,peak_command = 0.,0.
    min_slack,contacts = float("inf"),0
    violations = set()
    path = directory/"traces"/(spec["id"]+".jsonl")
    path.parent.mkdir(exist_ok=True)
    last_sequence = 0
    started = time.perf_counter()
    initial_accuracy = dict(camera=pose_accuracy(sim.camera_pose(),goal["camera_world"]),
                            tool=pose_accuracy(sim.tool_pose(),goal["tool_world"]))
    initial_error = error_px(lab.perception.observe(sim.image()).corners,lab.reference)

    def scores():
        return dict(camera=pose_accuracy(sim.camera_pose(),goal["camera_world"]),
                    tool=pose_accuracy(sim.tool_pose(),goal["tool_world"]))

    def audit():
        nonlocal min_slack,contacts
        q = sim.data.qpos
        if not np.isfinite(q).all() or not np.isfinite(sim.velocity_command).all():
            violations.add("nonfinite_state")
            return
        distances,_ = sim.collision.distances(q)
        min_slack = min(min_slack,float(np.min(distances-sim.collision.margins)))
        contacts += len(sim.forbidden_contacts())
        if min_slack < -1e-6:
            violations.add("collision_clearance")
        if contacts:
            violations.add("forbidden_contact")
        if np.any(q < sim.model.jnt_range[:,0]-1e-6) or np.any(q > sim.model.jnt_range[:,1]+1e-6):
            violations.add("joint_limit")
        age = lab.camera.age_s(float(sim.data.time))
        if sim.velocity_command.any() and (age is None or age >= timing["max_observation_age_s"]-1e-10):
            violations.add("motion_without_fresh_feedback")

    audit()
    with path.open("w",encoding="utf-8") as stream:
        while lab.aligning:
            lab.advance(dt)
            audit()
            q = sim.data.qpos.copy()
            travel += float(np.sum(np.abs(q-previous_q)))
            previous_q = q
            peak_command = max(peak_command,float(np.max(np.abs(sim.velocity_command))))
            frame = lab.camera.latest
            if frame is not None and (frame.sequence != last_sequence or not lab.aligning):
                last_sequence = frame.sequence
                row = dict(time_s=float(sim.data.time),state=lab.alignment_status,
                           captured_s=frame.captured_s,age_s=lab.camera.age_s(float(sim.data.time)),
                           observed_error_px=error_px(frame.observation.corners,lab.reference),
                           qpos_rad=q.tolist(),command_rad_s=sim.velocity_command.tolist(),accuracy=scores(),
                           refinement=frame.observation.refinement,correlation=frame.observation.correlation,
                           processing_ms=frame.observation.processing_ms,
                           stop_metrics=lab.controller.ibvs.stop_metrics)
                stream.write(json.dumps(row,allow_nan=False)+"\n")
            if sim.data.time > plan["max_trial_s"]+dt:
                violations.add("run_deadline")
            if violations:
                lab.stop()
                break
    outcome = "safety_violation" if violations else lab.alignment_status
    completion = float(sim.data.time)
    stop_accuracy = scores()
    post_errors,post_scores = [],[stop_accuracy]
    commands_zero = True
    post_precision_passes = []
    for _ in range(plan["post_stop_frames"]):
        end = float(sim.data.time)+1/sim.config["camera_hz"]
        while sim.data.time < end-1e-10:
            lab.advance(dt)
            audit()
            commands_zero = commands_zero and not bool(sim.velocity_command.any()) and not lab.aligning
        observed=lab.perception.observe(sim.image())
        post_errors.append(error_px(observed.corners,lab.reference))
        if lab.precision_enabled:
            from control import estimate_depths
            from precision import stopping_metrics
            try:
                depths=estimate_depths(observed.corners,lab.control_intrinsics(),lab.controller.ibvs.side_m,
                                       lab.controller.ibvs.config["max_pnp_reprojection_error_px"])
                check=stopping_metrics(observed.corners,lab.reference,lab.control_intrinsics(),depths,
                                       lab.controller.ibvs.precision_stop)
                post_precision_passes.append(check["candidate"])
            except (ValueError,cv2.error,np.linalg.LinAlgError):
                post_precision_passes.append(False)
        post_scores.append(scores())
    if not commands_zero:
        violations.add("motion_after_stop")
    if violations:
        outcome = "safety_violation"
    stable = commands_zero and all(e is not None and e < sim.config["ibvs"]["success_error_px"] for e in post_errors)
    stable = stable and all(post_precision_passes)
    if outcome == "converged" and not stable:
        outcome = "unstable_after_stop"
    pixel_success = outcome=="converged" and stable and not violations
    physical_success = pixel_success and all(physical_pass(s["camera"],s["tool"],
        plan["position_tolerance_mm"],plan["orientation_tolerance_deg"]) for s in post_scores)
    snapshot = None
    if spec["case"] == 0 and spec["profile"] in ("nominal","combined"):
        snapshot = spec["id"]+".png"
        Image.fromarray(lab.draw()).save(directory/snapshot)
    worst = {frame:{key:max(s[frame][key] for s in post_scores)
                    for key in ("position_error_mm","orientation_error_deg")} for frame in ("camera","tool")}
    return dict(**{k:spec[k] for k in ("id","mode","profile","case")},
        outcome=outcome,pixel_success=bool(pixel_success),physical_success=bool(physical_success),
        completion_s=completion,initial_error_px=initial_error,
        max_post_stop_error_px=max(post_errors) if all(e is not None for e in post_errors) else None,
        post_stop_frames=len(post_errors),commands_zero_after_stop=commands_zero,
        post_stop_precision_passes=post_precision_passes,
        initial_accuracy=initial_accuracy,stop_accuracy=stop_accuracy,final_accuracy=post_scores[-1],
        worst_post_stop_accuracy=worst,final_camera_world=sim.camera_pose().tolist(),final_tool_world=sim.tool_pose().tolist(),
        reference_features_px=lab.reference.tolist(),assumed_K=lab.control_intrinsics().tolist(),
        assumed_target_side_m=lab.control_config()["marker_side_m"],calibration=lab.calibration.settings,
        minimum_collision_slack_mm=1000*min_slack,forbidden_contacts=contacts,
        safety_violations=sorted(violations),joint_travel_rad=travel,peak_joint_command_rad_s=peak_command,
        matcher=lab.perception.name,precision_enabled=lab.precision_enabled,
        stopping_settings=lab.controller.ibvs.precision_stop,stop_metrics=lab.controller.ibvs.stop_metrics,
        wall_s=time.perf_counter()-started,
        trace=path.relative_to(directory).as_posix(),snapshot=snapshot)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--accuracy-study",action="store_true",help=argparse.SUPPRESS)
    parser.add_argument("--config",type=Path)
    parser.add_argument("--starts",type=int)
    parser.add_argument("--seed",type=int)
    parser.add_argument("--modes",nargs="+",choices=("aruco","natural","learned"))
    parser.add_argument("--profiles",nargs="+",help="Configured profiles including nominal")
    parser.add_argument("--legacy-stop",action="store_true",help="Original 1 px stop without reference-image refinement")
    parser.add_argument("--plan-only",action="store_true")
    target=parser.add_mutually_exclusive_group()
    target.add_argument("--output",type=Path)
    target.add_argument("--resume",type=Path)
    target.add_argument("--report-only",type=Path)
    args=parser.parse_args(argv)
    overrides=(args.config,args.starts,args.seed,args.modes,args.profiles)
    if (args.resume or args.report_only) and (args.plan_only or args.legacy_stop or any(x is not None for x in overrides)):
        parser.error("Resume/report uses the saved plan; omit overrides and --plan-only")
    if args.report_only:
        report(args.report_only.resolve())
        return 0
    if args.resume:
        directory=args.resume.resolve()
        manifest=json.loads((directory/"manifest.json").read_text())
        if manifest["fingerprint"] != fingerprint():
            raise ValueError("Source/assets/runtime changed; create a new experiment")
        if manifest["plan_sha256"] != digest(directory/"plan.json"):
            raise ValueError("Saved plan changed; create a new experiment")
        plan=json.loads((directory/"plan.json").read_text())
        rows=json.loads((directory/"trials.json").read_text())
        validate_rows(plan,rows)
        goals=validate_goals(directory,manifest) if "goals_sha256" in manifest else None
        if rows and goals is None:
            raise ValueError("Completed trials require immutable evaluation goals")
    else:
        config=json.loads((args.config or CONFIG_PATH).read_text(encoding="utf-8"))
        plan=make_plan(config,args.modes,args.starts,args.seed,args.profiles)
        plan["precision_enabled"]=not args.legacy_stop
        if plan["precision_enabled"]:
            from precision import load_precision_config
            plan["precision_settings"]=load_precision_config()
        directory=(args.output or ROOT/"results/accuracy"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")).resolve()
        directory.mkdir(parents=True,exist_ok=False)
        write_json(directory/"plan.json",plan)
        rows=[];goals=None
        write_json(directory/"trials.json",rows)
        manifest=dict(schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),status="planned",
                      fingerprint=fingerprint(),plan_sha256=digest(directory/"plan.json"),
                      planned_trials=len(plan["trials"]),completed_trials=0)
        write_json(directory/"manifest.json",manifest)
    print(f"Accuracy experiment: {directory}\n{len(plan['trials'])} planned; {len(rows)} saved.",flush=True)
    if args.plan_only:
        return 0
    complete={r["id"] for r in rows}
    pending=[s for s in plan["trials"] if s["id"] not in complete]
    manifest["status"]="running"
    write_json(directory/"manifest.json",manifest)
    try:
        if pending or goals is None:
            from simulation import Simulation
            with Simulation() as sim:
                if goals is None:
                    goals=prepare_goals(sim,directory,plan["modes"])
                    manifest["goals_sha256"]=digest(directory/"goals.json")
                    write_json(directory/"manifest.json",manifest)
                for spec in pending:
                    try:
                        row=run_trial(sim,spec,plan,goals,directory)
                    except Exception as exc:
                        sim.command_velocity(np.zeros(6))
                        folder=directory/"traces";folder.mkdir(exist_ok=True)
                        (folder/(spec["id"]+".error.txt")).write_text(traceback.format_exc(),encoding="utf-8")
                        row=dict(**{k:spec[k] for k in ("id","mode","profile","case")},
                                 outcome="error",pixel_success=False,physical_success=False,
                                 safety_violations=[],error=f"{type(exc).__name__}: {exc}")
                    rows.append(row)
                    write_json(directory/"trials.json",rows)
                    manifest["completed_trials"]=len(rows)
                    write_json(directory/"manifest.json",manifest)
                    print(f"[{len(rows)}/{len(plan['trials'])}] {row['id']}: {row['outcome']}; "
                          f"physical tolerance {'met' if row['physical_success'] else 'not met'}",flush=True)
        validate_goals(directory,manifest)
        if manifest["fingerprint"] != fingerprint():
            raise RuntimeError("Experiment inputs changed during execution")
        report(directory)
        manifest["status"]="complete"
        write_json(directory/"manifest.json",manifest)
    except BaseException:
        manifest["status"]="interrupted"
        write_json(directory/"manifest.json",manifest)
        raise
    return int(any(r["outcome"]=="error" or r.get("safety_violations") for r in rows))


if __name__=="__main__":
    raise SystemExit(main())

