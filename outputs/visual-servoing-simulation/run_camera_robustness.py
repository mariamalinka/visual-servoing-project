"""Run or resume seeded closed-loop camera robustness experiments."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import time
import traceback

import numpy as np

from camera_robustness import ROOT, NAMES, digest, make_plan, report, validate_rows, write_json


def fingerprint():
    paths = set(ROOT.glob("*.py")) | set(ROOT.glob("*.json")) | {ROOT/"scene.xml"}
    for folder in ("assets", "reference"):
        paths.update(p for p in (ROOT/folder).rglob("*") if p.is_file())
    paths.add(ROOT/"models/manifest.json")
    versions = {"python": platform.python_version(), "platform": platform.platform()}
    for name in ("numpy", "opencv-python", "mujoco", "matplotlib", "pillow", "torch", "torchvision", "lightglue"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return dict(sha256={p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths) if p.is_file()},
                runtime=versions)


def error_px(corners, reference):
    if corners is None:
        return None
    return float(np.sqrt(np.mean(np.sum((corners-reference)**2, axis=1))))


def run_trial(sim, spec, post_frames, directory):
    import cv2
    from app import Lab
    sim.reset()
    cv2.setRNGSeed(spec["perception_seed"])
    lab = Lab(sim, auto_start=False, perception_mode=spec["mode"], camera_timing=spec["timing"])
    reference_hash = digest(lab.reference_path)
    sim.reset(spec["offset_degrees"])
    lab.align()
    initial_error = error_px(lab.perception.observe(sim.image()).corners, lab.reference)
    trace_path = directory/"traces"/(spec["id"]+".jsonl")
    trace_path.parent.mkdir(exist_ok=True)
    dt = float(sim.model.opt.timestep)
    travel, peak_command = 0.0, 0.0
    margin = float("inf")
    previous_q = sim.data.qpos.copy()
    violations = set()
    collision_slack, contact_count = None, 0

    def inspect_geometry():
        nonlocal collision_slack, contact_count
        distances,_ = sim.collision.distances(sim.data.qpos)
        if len(distances):
            slack = float(np.min(distances-sim.collision.margins))
            collision_slack = slack if collision_slack is None else min(collision_slack,slack)
            if slack < -1e-6:
                violations.add("collision_clearance")
        contact_count += len(sim.forbidden_contacts())
        if contact_count:
            violations.add("forbidden_contact")

    inspect_geometry()
    ages, inference = [], []
    last_frame = 0
    started = time.perf_counter()
    with trace_path.open("w", encoding="utf-8") as trace:
        while lab.aligning:
            lab.advance(dt)
            inspect_geometry()
            now = float(sim.data.time)
            q = sim.data.qpos.copy()
            command = sim.velocity_command
            travel += float(np.sum(np.abs(q-previous_q)))
            previous_q = q
            peak_command = max(peak_command, float(np.max(np.abs(command))))
            margin = min(margin, float(np.min(np.minimum(q-sim.model.jnt_range[:,0],
                                                        sim.model.jnt_range[:,1]-q))))
            if not np.isfinite(q).all() or not np.isfinite(command).all():
                violations.add("nonfinite_state")
            if margin < -1e-6:
                violations.add("joint_limit")
            age = lab.camera.age_s(now)
            if np.any(command) and (age is None or age >= spec["timing"]["max_observation_age_s"]-1e-10):
                violations.add("motion_without_fresh_feedback")
            frame = lab.camera.latest
            is_new = frame is not None and frame.sequence != last_frame
            if is_new:
                last_frame = frame.sequence
                ages.append(age)
                inference.append(frame.inference_ms)
            if is_new or not lab.aligning:
                row = dict(time_s=now, state=lab.alignment_status,
                           captured_s=None if frame is None else frame.captured_s,
                           delivered_s=lab.camera.delivered_s, age_s=age,
                           frame_id=None if frame is None else frame.sequence,
                           observed_error_px=None if frame is None else error_px(frame.observation.corners, lab.reference),
                           current_qpos_rad=q.tolist(), command_rad_s=command.tolist(),
                           captured_qpos_rad=None if frame is None else frame.qpos.tolist(),
                           captured_frames=lab.camera.sequence, dropped_packets=lab.camera.lost_frames,
                           obsolete_frames=lab.camera.obsolete_frames)
                trace.write(json.dumps(row, allow_nan=False)+"\n")
            if violations:
                lab.stop()
                break
    outcome = "safety_violation" if violations else lab.alignment_status
    completion_s = float(sim.data.time)
    event = lab.last_camera_failure
    terminal = error_px(lab.perception.observe(sim.image()).corners, lab.reference)
    counters = {key:getattr(lab.camera,key) for key in
                ("sequence","delivered_frames","dropped_frames","lost_frames","stale_frames","obsolete_frames","outage_slots")}
    # Observe resumed capture without rearming a stopped controller.
    last_outage_end = max((end for _, end in spec["timing"].get("outages_s", [])), default=completion_s)
    validation_end = max(completion_s+post_frames/sim.config["camera_hz"],
                         last_outage_end+2/sim.config["camera_hz"]+spec["timing"]["delay_s"]+spec["timing"].get("jitter_s",0))
    post_errors = []
    commands_zero = True
    next_observe = completion_s
    while sim.data.time < validation_end-1e-10 or len(post_errors) < post_frames:
        lab.advance(dt)
        inspect_geometry()
        commands_zero = commands_zero and not bool(sim.velocity_command.any()) and not lab.aligning
        if sim.data.time+1e-10 >= next_observe and len(post_errors) < post_frames:
            post_errors.append(error_px(lab.perception.observe(sim.image()).corners, lab.reference))
            next_observe += 1/sim.config["camera_hz"]
    if not commands_zero:
        violations.add("motion_after_stop")
    stable = commands_zero and all(e is not None and e < sim.config["ibvs"]["success_error_px"] for e in post_errors)
    if violations:
        outcome = "safety_violation"
    elif outcome == "converged" and not stable:
        outcome = "unstable_after_stop"
    stopped_age = None if event is None else event["simulation_time_s"] - (
        lab.camera.started_s if event["captured_s"] is None else event["captured_s"])
    restored = (lab.camera.latest is not None and lab.camera.latest.captured_s > last_outage_end)
    expected_pass = outcome == spec["expected"] and not violations
    if spec["expected"] == "stale_camera":
        budget = spec["timing"]["max_observation_age_s"]
        expected_pass = expected_pass and stopped_age is not None and budget-1e-9 <= stopped_age <= budget+dt+1e-9 and commands_zero and restored
    if digest(lab.reference_path) != reference_hash:
        raise RuntimeError("Experiment changed its saved reference image")
    return dict(**{key:spec[key] for key in ("id","mode","profile","case","expected")},
        outcome=outcome, expected_pass=bool(expected_pass), safety_violations=sorted(violations),
        completion_s=completion_s, initial_error_px=initial_error, final_current_error_px=terminal,
        max_post_stop_error_px=max(post_errors) if all(e is not None for e in post_errors) else None,
        post_stop_frames=len(post_errors), commands_zero_after_stop=commands_zero,
        joint_travel_rad=travel, peak_joint_command_rad_s=peak_command, minimum_joint_margin_rad=margin,
        delivered_age_p95_ms=1000*float(np.percentile(ages,95)) if ages else None,
        inference_median_ms=float(np.median(inference)) if inference else None,
        minimum_collision_slack_m=collision_slack, forbidden_contacts=contact_count,
        collision_limited_calls=sim.collision.limited_steps, collision_blocked_calls=sim.collision.blocked_steps,
        camera_counts=counters, failure_event=event, stopped_capture_age_s=stopped_age,
        capture_restored=restored, wall_s=time.perf_counter()-started,
        matcher=lab.perception.name, reference_sha256=reference_hash, trace=trace_path.relative_to(directory).as_posix())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robustness", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--config", type=Path, help="Experiment JSON (defaults to robustness_config.json)")
    parser.add_argument("--starts", type=int, help="Paired starting poses per profile/matcher (default 10)")
    parser.add_argument("--seed", type=int, help="Reproducible plan seed")
    parser.add_argument("--modes", nargs="+", choices=tuple(NAMES))
    parser.add_argument("--profiles", nargs="+", help="Named profiles in the configuration")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--output", type=Path, help="New output directory; existing data is never overwritten")
    target.add_argument("--resume", type=Path, help="Resume pending trials from a saved plan")
    target.add_argument("--report-only", type=Path, help="Regenerate figures/report from saved JSON")
    parser.add_argument("--plan-only", action="store_true", help="Save and inspect the plan without rendering")
    args = parser.parse_args(argv)
    if (args.resume or args.report_only) and any(x is not None for x in
        (args.config, args.starts, args.seed, args.modes, args.profiles)):
        parser.error("Resume/report uses the saved plan; omit plan overrides")
    if args.report_only:
        report(args.report_only.resolve())
        return 0
    if args.resume and args.plan_only:
        parser.error("Use --plan-only when creating a new experiment")
    if args.resume:
        directory = args.resume.resolve()
        manifest = json.loads((directory/"manifest.json").read_text())
        if digest(directory/"plan.json") != manifest["plan_sha256"]:
            raise ValueError("Saved plan changed; create a new experiment")
        if fingerprint() != manifest["fingerprint"]:
            raise ValueError("Source, assets, references or runtime changed; create a new experiment")
        plan = json.loads((directory/"plan.json").read_text())
        rows = json.loads((directory/"trials.json").read_text())
        validate_rows(plan, rows)
    else:
        config = json.loads((args.config or ROOT/"robustness_config.json").read_text())
        plan = make_plan(config, args.modes or list(NAMES), args.starts, args.seed, args.profiles)
        directory = (args.output or ROOT/"results/robustness"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")).resolve()
        directory.mkdir(parents=True, exist_ok=False)
        write_json(directory/"plan.json", plan)
        rows = []
        write_json(directory/"trials.json", rows)
        git = subprocess.run(["git","rev-parse","HEAD"], cwd=ROOT, capture_output=True, text=True)
        manifest = dict(schema_version=1, status="planned", created_utc=datetime.now(timezone.utc).isoformat(),
                        git_head=git.stdout.strip(), plan_sha256=digest(directory/"plan.json"),
                        fingerprint=fingerprint(), planned_trials=len(plan["trials"]), completed_trials=0)
        write_json(directory/"manifest.json", manifest)
    print(f"Experiment: {directory}\nPlan: {len(plan['trials'])} trials; {len(rows)} already saved.", flush=True)
    if args.plan_only:
        return 0
    completed = {row["id"] for row in rows}
    pending = [spec for spec in plan["trials"] if spec["id"] not in completed]
    manifest["status"] = "running"
    write_json(directory/"manifest.json", manifest)
    try:
        if pending:
            from simulation import Simulation
            with Simulation() as sim:
                for spec in pending:
                    try:
                        row = run_trial(sim, spec, plan["post_stop_frames"], directory)
                    except Exception as exc:
                        sim.command_velocity(np.zeros(6))
                        folder = directory/"traces"
                        folder.mkdir(exist_ok=True)
                        (folder/(spec["id"]+".error.txt")).write_text(traceback.format_exc(), encoding="utf-8")
                        row = dict(**{k:spec[k] for k in ("id","mode","profile","case","expected")},
                                   outcome="error", expected_pass=False, safety_violations=[],
                                   error=f"{type(exc).__name__}: {exc}")
                    rows.append(row)
                    write_json(directory/"trials.json", rows)
                    manifest["completed_trials"] = len(rows)
                    write_json(directory/"manifest.json", manifest)
                    print(f"[{len(rows)}/{len(plan['trials'])}] {spec['id']}: {row['outcome']} "
                          f"(expected outcome met: {row['expected_pass']})", flush=True)
        if fingerprint() != manifest["fingerprint"]:
            raise RuntimeError("Experiment inputs changed during execution")
        report(directory)
        failed = any(r["outcome"] == "error" or r["safety_violations"] for r in rows)
        manifest.update(status="complete_with_errors" if failed else "complete",
                        completed_utc=datetime.now(timezone.utc).isoformat())
        write_json(directory/"manifest.json", manifest)
        print("Report:", directory/"REPORT.md", flush=True)
        return 2 if failed else 0
    except BaseException as exc:
        manifest.update(status="interrupted" if isinstance(exc, KeyboardInterrupt) else "failed",
                        error=f"{type(exc).__name__}: {exc}")
        write_json(directory/"manifest.json", manifest)
        report(directory)
        raise


if __name__ == "__main__":
    raise SystemExit(main())

