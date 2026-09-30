"""Physical stop and resume response with the simulated actuator dynamics.

    run.cmd --stop-response                 # stops, pause/resume cycles, realtime watchdog stops
    run.cmd --stop-response --accuracy 5    # plus a paired accuracy check, 5 starts per mode

Measures, for each actuator profile in actuator_config.json:
1. Stops from constant speed (0.1, 0.35 and 0.6 rad/s; each joint in both
   directions and all joints together; home and the standard start pose): physical
   stopping time, stopping distance (joints, camera, tool), deceleration and jerk,
   compared with the analytic setpoint prediction.
2. Starts and resumes: time to reach the command, acceleration and jerk.
3. Repeated pause/resume cycles, resuming both before and after standstill.
4. Realtime watchdog stops (stop setting, 400 ms image-age limit): command stop
   latency, physical stopping time and image age at standstill, measured in the
   real runtime.
5. Optionally (--accuracy N), final camera and tool accuracy with and without the
   actuator model on the same starts.

Everything is simulation. The actuator limits are assumptions (docs/ACTUATOR_MODEL.md);
no result here is a statement about a real robot.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

from actuator import CONFIG_PATH, braking_distance, braking_time, load_actuator_config

ROOT = Path(__file__).resolve().parent
SPEEDS = (0.1, 0.35, 0.6)
AGE_LIMIT_S = 0.4          # Image-age limit used by the tests (RuntimeConfig default is 0.25 s).
LATENESS_BOUND_S = 0.05    # Tested bound on command stop latency after the limit.
STOP_FIELDS = ("profile", "pose", "motion", "joint", "requested_rad_s", "speed_at_command_rad_s",
               "limited_by_guard", "setpoint_stop_ms", "predicted_setpoint_stop_ms", "physical_stop_ms",
               "joint_travel_rad", "predicted_travel_bound_rad", "camera_travel_mm", "tool_travel_mm",
               "camera_rotation_deg", "peak_measured_accel_rad_s2", "peak_setpoint_accel_rad_s2",
               "peak_setpoint_jerk_rad_s3", "peak_measured_jerk_rad_s3", "start_reached_ms",
               "start_peak_setpoint_accel_rad_s2", "start_peak_setpoint_jerk_rad_s3", "start_peak_measured_accel_rad_s2")


def profiles():
    return list(json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["profiles"])


def motions():
    """(label, joint index or None, sign vector)."""
    rows = []
    for joint in range(6):
        for sign in (1, -1):
            vector = np.zeros(6)
            vector[joint] = sign
            rows.append((f"joint{joint}{'+' if sign > 0 else '-'}", joint, vector))
    rows.append(("all+", None, np.array([1, 1, -1, 1, 1, 1.])))
    rows.append(("all-", None, np.array([-1, -1, 1, -1, -1, -1.])))
    return rows


def stop_case(sim, pose, vector, speed):
    """Ramp to `speed`, hold, stop; return the stop and start records."""
    sim.reset(pose)
    config = sim.actuator
    ramp = 0.0 if not config.enabled else float(np.max(speed / config.acceleration_limit + config.acceleration_limit / config.jerk_limit))
    sim.command_velocity(vector * speed)
    sim.advance(ramp + 0.25)
    sim.command_velocity(np.zeros(6), reason="study_stop")
    sim.advance(0.6)
    log = list(sim.motion_log)
    start = next(r for r in reversed(log) if r["kind"] == "start")
    stop = next(r for r in reversed(log) if r["kind"] == "stop")
    return stop, start


def run_stops(profile_names):
    from simulation import Simulation
    rows, examples = [], {}
    for name in profile_names:
        with Simulation(render=False, actuator_config=name) as sim:
            model = sim.actuator
            for pose_name, pose in (("home", None), ("start", sim.config["ibvs"]["start_offset_degrees"])):
                for label, joint, vector in motions():
                    for speed in SPEEDS:
                        stop, start = stop_case(sim, pose, vector, speed)
                        index = joint if joint is not None else int(np.argmax(np.abs(stop["setpoint_at_command_rad_s"])))
                        actual = abs(stop["setpoint_at_command_rad_s"][index])
                        if model.enabled:
                            predicted_ms = 1000 * float(np.max(braking_time(np.abs(stop["setpoint_at_command_rad_s"]),
                                                                            model.deceleration_limit, model.jerk_limit)))
                            bound = float(np.max(braking_distance(np.abs(stop["setpoint_at_command_rad_s"]),
                                                                  model.deceleration_limit, model.jerk_limit)
                                                 + np.abs(stop["setpoint_at_command_rad_s"]) * model.servo_lag_allowance_s))
                        else:
                            predicted_ms, bound = 0.0, float(np.max(np.abs(stop["setpoint_at_command_rad_s"]))) * model.servo_lag_allowance_s
                        rows.append(dict(profile=name, pose=pose_name, motion=label, joint=joint, requested_rad_s=speed,
                            speed_at_command_rad_s=stop["speed_at_command_rad_s"],
                            limited_by_guard=bool(actual < 0.95 * speed),
                            setpoint_stop_ms=None if stop["setpoint_stop_time_s"] is None else 1000 * stop["setpoint_stop_time_s"],
                            predicted_setpoint_stop_ms=predicted_ms,
                            physical_stop_ms=None if stop["stop_time_s"] is None else 1000 * stop["stop_time_s"],
                            joint_travel_rad=stop["max_joint_travel_rad"], predicted_travel_bound_rad=bound,
                            camera_travel_mm=stop["camera_travel_mm"], tool_travel_mm=stop["tool_travel_mm"],
                            camera_rotation_deg=stop["camera_rotation_deg"],
                            peak_measured_accel_rad_s2=stop["peak_measured_accel_rad_s2"],
                            peak_setpoint_accel_rad_s2=stop["peak_setpoint_accel_rad_s2"],
                            peak_setpoint_jerk_rad_s3=stop["peak_setpoint_jerk_rad_s3"],
                            peak_measured_jerk_rad_s3=stop["peak_measured_jerk_rad_s3"],
                            start_reached_ms=None if start.get("reached_command_s") is None else 1000 * start["reached_command_s"],
                            start_peak_setpoint_accel_rad_s2=start["peak_setpoint_accel_rad_s2"],
                            start_peak_setpoint_jerk_rad_s3=start["peak_setpoint_jerk_rad_s3"],
                            start_peak_measured_accel_rad_s2=start["peak_measured_accel_rad_s2"],
                            outcome=stop["outcome"], stop_clamps=stop["stop_clamps"]))
                        if pose_name == "home" and label == "joint1-" and speed == 0.35:
                            examples[name] = dict(stop=stop["profile"], start=start["profile"])
    return rows, examples


def run_cycles(profile_names, cycles=20):
    """Repeated pause/resume: hold shorter than the stop (resume while braking) and longer."""
    from simulation import Simulation
    results = []
    vector = np.array([0.35, -0.25, 0.2, 0.15, -0.15, 0.1])
    for name in profile_names:
        with Simulation(render=False, actuator_config=name) as sim:
            for hold_s in (0.04, 0.3):
                sim.reset(sim.config["ibvs"]["start_offset_degrees"])
                first = len(sim.motion_log)
                q0 = sim.data.qpos.copy()
                for cycle in range(cycles):
                    sim.command_velocity(vector * (1 if cycle % 2 == 0 else -1))
                    sim.advance(0.4)
                    sim.command_velocity(np.zeros(6), reason="watchdog_pause")
                    sim.advance(hold_s)
                sim.advance(0.6)
                log = list(sim.motion_log)[first:]
                stops = [r for r in log if r["kind"] == "stop"]
                starts = [r for r in log if r["kind"] == "start"]
                done = [r["stop_time_s"] for r in stops if r["outcome"] == "stopped"]
                results.append(dict(profile=name, hold_s=hold_s, cycles=cycles, stops=len(stops),
                    reached_standstill=len(done), resumed_while_braking=sum(r["outcome"] == "resumed" for r in stops),
                    stop_time_ms_min=1000 * min(done) if done else None, stop_time_ms_max=1000 * max(done) if done else None,
                    peak_setpoint_accel_rad_s2=max(r["peak_setpoint_accel_rad_s2"] for r in log),
                    peak_setpoint_jerk_rad_s3=max(r["peak_setpoint_jerk_rad_s3"] for r in log),
                    peak_measured_accel_rad_s2=max(r["peak_measured_accel_rad_s2"] for r in log),
                    start_reached_ms_max=max((1000 * r["reached_command_s"] for r in starts
                                              if r.get("reached_command_s") is not None), default=None),
                    stop_clamps=sim.actuator.stop_clamps, forbidden_contacts=len(sim.forbidden_contacts()),
                    net_joint_drift_rad=float(np.max(np.abs(sim.data.qpos - q0)))))
    return results


def realtime_session(name, config, offset, until, settle_s=0.6, timeout_s=30):
    """Run one RealtimeSession until `until(session)` is true, let the arm settle, return its report."""
    from realtime import RealtimeSession
    session = RealtimeSession(config=config, offset=offset, actuator=name).start()
    try:
        if not session.ready.wait(60):
            raise RuntimeError("Realtime session did not start")
        deadline = time.perf_counter() + timeout_s
        while not until(session) and time.perf_counter() < deadline:
            if session.snapshot().get("error"):
                raise RuntimeError(session.snapshot()["error"])
            time.sleep(.01)
        time.sleep(settle_s)  # Let the physical stop finish.
    finally:
        session.close()
    return session.report()


def counters(report):
    counts = report["counts"]
    return dict(control_gap_p99_ms=report["control_gap_ms"]["p99"] if report["control_gap_ms"] else None,
                control_deadline_misses=counts["control_deadline_misses"],
                unsafe_motion_ticks=counts["unsafe_motion_ticks"], post_stop_motion_ticks=counts["post_stop_motion_ticks"],
                paused_motion_ticks=counts["paused_motion_ticks"], contacts=counts["contacts"])


def run_realtime(profile_names, runs):
    """The real runtime: watchdog stops (stop setting), manual stops, and holds with resume.

    Watchdog stops trip at different times after Align; manual stops are requested at
    different times; the hold run delays every frame so the watchdog holds and resumes
    repeatedly (the runtime default).
    """
    from realtime import RuntimeConfig
    from stop_response import link
    watchdog, manual, holds = [], [], []
    offsets = ([3, -3, 4, 3, -2, 2], [-4, 3, -3, -3, 3, -2], [5, -2, 5, 4, -3, 3])
    for name in profile_names:
        for run in range(runs):
            fault_after = 0.3 + 0.25 * run
            config = RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, inference_stall_s=.8,
                                   fault_after_s=fault_after, stale_resume_s=0)
            report = realtime_session(name, config, offsets[run % len(offsets)],
                                      lambda s: s.snapshot().get("status") == "stale_camera")
            for row in link(report):
                if row["kind"] == "stop" and row["reason"] == "stale_camera":
                    watchdog.append(dict(profile=name, run=run, fault_after_s=fault_after, **counters(report), **row))
        for run in range(max(1, runs // 2)):
            delay = 0.25 + 0.25 * run
            config = RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0)
            marks = {}

            def until(session, delay=delay, marks=marks):
                state = session.snapshot()
                if state.get("counters", {}).get("accepted", 0) and "first" not in marks:
                    marks["first"] = time.perf_counter()
                if "first" in marks and "sent" not in marks and time.perf_counter() - marks["first"] >= delay:
                    session.command("stop")
                    marks["sent"] = True
                return "sent" in marks and state.get("status") == "stopped"
            report = realtime_session(name, config, offsets[run % len(offsets)], until)
            for row in link(report):
                if row["kind"] == "stop" and row["reason"] == "stopped":
                    manual.append(dict(profile=name, run=run, delay_s=delay, **counters(report), **row))
    return dict(watchdog=watchdog, manual=manual, holds=run_holds(profile_names))


def run_holds(profile_names):
    """Hold and resume (the runtime default) with every frame delayed, so holds repeat."""
    from realtime import RuntimeConfig
    from stop_response import link
    holds = []

    def until(session):
        state = session.snapshot()
        counts = state.get("counters") or {}
        ended = counts.get("accepted", 0) > 0 and not state.get("active") and state.get("status") not in ("idle", "initializing")
        return counts.get("watchdog_resumes", 0) >= 8 or ended
    for name in profile_names:
        config = RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, inference_stall_s=.3, fault_after_s=.3,
                               stale_resume_s=2.)
        report = realtime_session(name, config, [3, -3, 4, 3, -2, 2], until, timeout_s=40)
        rows = link(report)
        pauses = [r for r in rows if r["kind"] == "pause"]
        resumes = [r for r in rows if r["kind"] == "resume"]
        moving = [r for r in pauses if r["moving"]]
        times = [r["physical_stop_ms"] for r in moving if r.get("physical_stop_ms") is not None]
        reached = [r["reached_command_ms"] for r in resumes if r.get("reached_command_ms") is not None]
        holds.append(dict(profile=name, **counters(report), holds=len(pauses), holds_moving=len(moving),
            reached_standstill=sum(r["outcome"] == "stopped" for r in moving),
            resumed_while_braking=sum(r["outcome"] == "resumed" for r in moving),
            other=sum(r["outcome"] not in ("stopped", "resumed") for r in moving),
            physical_stop_ms_max=max(times) if times else None,
            camera_travel_mm_max=max((r.get("camera_travel_mm") or 0 for r in moving), default=0.0),
            resumes=len(resumes), resume_reached_ms_max=max(reached) if reached else None,
            resumes_reached=sum(r["outcome"] == "reached" for r in resumes),
            resume_motion_ms_max=max((r["duration_ms"] for r in resumes if r.get("duration_ms") is not None), default=None),
            resume_peak_speed_rad_s=max((r["peak_speed_rad_s"] for r in resumes if r.get("peak_speed_rad_s") is not None),
                                        default=None),
            peak_setpoint_jerk_rad_s3=max([r.get("peak_setpoint_jerk_rad_s3") or 0 for r in moving + resumes] or [0]),
            peak_setpoint_accel_rad_s2=max([r.get("peak_setpoint_accel_rad_s2") or 0 for r in moving + resumes] or [0]),
            stop_clamps=sum(r.get("stop_clamps", 0) for r in moving + resumes),
            watchdog_pauses=report["counts"]["watchdog_pauses"], watchdog_resumes=report["counts"]["watchdog_resumes"]))
    return holds


DELAY_LEVELS_S = (0.0, 0.05, 0.1, 0.15, 0.25, 0.3)


def run_delay_levels(profile_names, levels=DELAY_LEVELS_S):
    """Whole ArUco alignments in the realtime runtime (hold and resume) with a constant per-frame delay.

    Does the ramped actuator change the outcome when holds become frequent? One
    alignment per level and profile, from the standard offset.
    """
    from realtime import RuntimeConfig
    rows = []

    def ended(session):
        state = session.snapshot()
        counts = state.get("counters") or {}
        return counts.get("accepted", 0) > 0 and not state.get("active") and state.get("status") not in ("idle", "initializing")
    for stall in levels:
        for name in profile_names:
            config = RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, inference_stall_s=stall, fault_after_s=0.0,
                                   stale_resume_s=2.)
            report = realtime_session(name, config, [3, -3, 4, 3, -2, 2], ended, settle_s=0.3, timeout_s=70)
            events = report["events"]
            align = next((e["at_s"] for e in events if e["kind"] == "align"), None)
            stop = next((e for e in events if e["kind"] == "stop" and e.get("reason") != "idle"), None)
            counts = report["counts"]
            rows.append(dict(profile=name, delay_ms=1000 * stall, outcome=None if stop is None else stop["reason"],
                             duration_s=None if stop is None or align is None else stop["at_s"] - align,
                             holds=counts["watchdog_pauses"], paused_s=counts["paused_s"],
                             paused_motion_ticks=counts["paused_motion_ticks"], unsafe_motion_ticks=counts["unsafe_motion_ticks"],
                             contacts=counts["contacts"],
                             final_error_px=next((f["error_px"] for f in reversed(report["frames"])), None)))
            print(f"  delay {1000 * stall:.0f} ms {name}: {rows[-1]['outcome']} after {rows[-1]['duration_s']}", flush=True)
    return rows


def run_collision(profile_names):
    """Drive single joints toward the mapped obstacle and measure clearance through the stop."""
    from collision import CollisionPoseError
    from simulation import Simulation
    rows = []
    for name in profile_names:
        with Simulation(render=False, actuator_config=name) as sim:
            for pose_name, pose in (("home", None), ("start", sim.config["ibvs"]["start_offset_degrees"])):
                for joint in range(3):
                    for sign in (1, -1):
                        for speed in SPEEDS:
                            sim.set_obstacle(False)
                            sim.reset(pose)
                            try:
                                sim.set_obstacle(True)
                            except CollisionPoseError:
                                continue
                            command = np.zeros(6)
                            command[joint] = sign * speed
                            first = len(sim.motion_log)
                            sim.command_velocity(command)
                            slack, engaged, blocked_at = np.inf, False, None
                            for step in range(1500):  # 3 s, checked every 2 ms physics step.
                                sim.advance(0.002)
                                distances, _ = sim.collision.distances(sim.data.qpos)
                                slack = min(slack, float(np.min(distances - sim.collision.margins)))
                                engaged = engaged or sim.collision.last.status in ("limited", "blocked")
                                if sim.collision_event is not None and blocked_at is None:
                                    blocked_at = step
                                    sim.collision_event = None
                                if blocked_at is not None and step - blocked_at >= 250:
                                    break
                            if not engaged:
                                continue  # This motion never came near the obstacle.
                            stops = [r for r in list(sim.motion_log)[first:] if r["kind"] == "stop"
                                     and r["reason"] in ("collision_blocked", "collision_guard")]
                            rows.append(dict(profile=name, pose=pose_name, joint=joint, sign=sign, requested_rad_s=speed,
                                             blocked=blocked_at is not None, minimum_slack_mm=1000 * slack,
                                             forbidden_contacts=len(sim.forbidden_contacts()),
                                             stop_camera_travel_mm=stops[-1]["camera_travel_mm"] if stops else None,
                                             stop_speed_rad_s=stops[-1]["speed_at_command_rad_s"] if stops else None))
                            sim.collision.reset()
    return rows


def run_accuracy(profile_names, starts, directory):
    """Paired accuracy trials: same starts, seeds and goals; only the actuator differs."""
    from accuracy import make_plan
    from run_accuracy_study import prepare_goals, run_trial
    from simulation import Simulation
    config = json.loads((ROOT / "accuracy_config.json").read_text(encoding="utf-8"))
    plan = make_plan(config, ["aruco", "natural"], starts, None, ["nominal"])
    plan["precision_enabled"] = True
    rows, references = [], None
    for name in profile_names:
        folder = directory / "accuracy" / name
        folder.mkdir(parents=True, exist_ok=True)
        with Simulation(actuator_config=name) as sim:
            # Goals and references are rendered at home before any motion: identical for every profile.
            goals = prepare_goals(sim, folder, plan["modes"])
            digests = {mode: goal["reference_sha256"] for mode, goal in goals.items()}
            if references is not None and digests != references:
                raise RuntimeError("Reference images differ between actuator profiles")
            references = digests
            for spec in plan["trials"]:
                row = run_trial(sim, spec, plan, goals, folder)
                stops = [r for r in sim.motion_log if r["kind"] == "stop"]
                final_stop = stops[-1] if stops else None
                rows.append(dict(profile=name, id=spec["id"], mode=spec["mode"], outcome=row["outcome"],
                    physical_success=row["physical_success"], completion_s=row["completion_s"],
                    stop_camera_mm=row["stop_accuracy"]["camera"]["position_error_mm"],
                    final_camera_mm=row["final_accuracy"]["camera"]["position_error_mm"],
                    worst_camera_mm=row["worst_post_stop_accuracy"]["camera"]["position_error_mm"],
                    worst_camera_deg=row["worst_post_stop_accuracy"]["camera"]["orientation_error_deg"],
                    worst_tool_mm=row["worst_post_stop_accuracy"]["tool"]["position_error_mm"],
                    worst_tool_deg=row["worst_post_stop_accuracy"]["tool"]["orientation_error_deg"],
                    max_post_stop_error_px=row["max_post_stop_error_px"],
                    safety_violations=row["safety_violations"],
                    minimum_collision_slack_mm=row["minimum_collision_slack_mm"],
                    final_stop_camera_travel_mm=None if final_stop is None else final_stop["camera_travel_mm"],
                    final_stop_ms=None if final_stop is None or final_stop["stop_time_s"] is None
                    else 1000 * final_stop["stop_time_s"]))
                print(f"  accuracy {name} {spec['id']}: {row['outcome']}, worst camera "
                      f"{rows[-1]['worst_camera_mm']:.2f} mm", flush=True)
    return rows, plan


def worst(rows, key, **match):
    values = [r[key] for r in rows if r.get(key) is not None and all(r.get(k) == v for k, v in match.items())]
    return max(values) if values else None


def write_report(directory, meta, stops, cycles, realtime, accuracy_rows, collision_rows, delay_levels=None):
    names = meta["profiles"]
    lines = ["# Stop and resume response with actuator dynamics", "",
             f"Run {meta['created_utc']}. Simulation only: the actuator limits are assumptions "
             "(`actuator_config.json`, `docs/ACTUATOR_MODEL.md`), not measurements of a real drive.", "",
             *([f"Note: {meta['note']}", ""] if meta.get("note") else []),
             "Definitions:", "",
             "- **Command stop latency**: image-age limit to zero velocity command (wall clock, realtime runtime).",
             "- **Physical stopping time**: zero command to standstill, every joint below "
             f"{meta['stopped_velocity_rad_s']} rad/s for {1000 * meta['stopped_hold_s']:.0f} ms (simulated time).",
             "- **Physical stopping distance**: motion between the zero command and standstill "
             "(largest joint angle, camera and tool displacement).", "",
             "## Profiles", "", "| Profile | Acceleration | Deceleration | Jerk | Worst-case check against the collision guard |",
             "|---|---:|---:|---:|---|"]
    for name in names:
        config = load_actuator_config(name)
        check = meta["braking_checks"][name]
        if config.get("enabled", True):
            lines.append(f"| {name} | {config['max_acceleration_rad_s2']} rad/s² | {config['max_deceleration_rad_s2']} rad/s² | "
                         f"{config['max_jerk_rad_s3']} rad/s³ | {check['equivalent_time_s'] * 1000:.0f} ms equivalent "
                         f"≤ {check['braking_time_s'] * 1000:.0f} ms |")
        else:
            lines.append(f"| {name} | none | none | none | model off (previous behaviour) |")
    lines += ["", "## 1. Stops from constant speed", "",
              f"{len(stops)} stops: 14 motions (each joint both ways, all joints together) x 3 speeds x 2 poses "
              "per profile. Worst case per requested speed:", "",
              "| Profile | Speed | Setpoint stop (predicted) | Physical stop | Largest joint travel (bound) | "
              "Camera travel | Camera rotation | Peak decel (measured) | Peak jerk setpoint / measured |",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for name in names:
        for speed in SPEEDS:
            sub = [r for r in stops if r["profile"] == name and r["requested_rad_s"] == speed]
            lines.append(
                f"| {name} | {speed} rad/s | {max(r['setpoint_stop_ms'] or 0 for r in sub):.0f} ms "
                f"({max(r['predicted_setpoint_stop_ms'] for r in sub):.0f}) | {max(r['physical_stop_ms'] or 0 for r in sub):.0f} ms | "
                f"{max(r['joint_travel_rad'] for r in sub):.4f} rad ({max(r['predicted_travel_bound_rad'] for r in sub):.4f}) | "
                f"{max(r['camera_travel_mm'] for r in sub):.1f} mm | {max(r['camera_rotation_deg'] for r in sub):.2f}° | "
                f"{max(r['peak_measured_accel_rad_s2'] for r in sub):.1f} rad/s² | "
                f"{max(r['peak_setpoint_jerk_rad_s3'] for r in sub):.0f} / {max(r['peak_measured_jerk_rad_s3'] for r in sub):.0f} rad/s³ |")
    exceeded = [r for r in stops if r["joint_travel_rad"] > r["predicted_travel_bound_rad"] + 1e-9]
    unfinished = [r for r in stops if r["outcome"] != "stopped"]
    limited = sum(r["limited_by_guard"] for r in stops)
    lines += ["", f"- Stops that did not reach standstill within 0.6 s: {len(unfinished)}.",
              f"- Stops whose joint travel exceeded the model's bound (setpoint + servo allowance): {len(exceeded)}"
              + ("." if not exceeded else f" (worst {max(r['joint_travel_rad'] - r['predicted_travel_bound_rad'] for r in exceeded):.4f} rad over)."),
              f"- {limited} stops started below 95% of the requested speed because the collision guard or joint-limit "
              "backstop was already slowing the joint; the prediction uses the actual speed.",
              "- Measured jerk is the finite difference of MuJoCo joint accelerations over one 2 ms step. With the "
              "model off it reflects the servo's step response, not a drive limit.", "",
              "Starts (0 to the requested speed):", "",
              "| Profile | Speed | Time to reach command | Peak acceleration setpoint / measured | Peak jerk setpoint |",
              "|---|---:|---:|---:|---:|"]
    for name in names:
        for speed in SPEEDS:
            sub = [r for r in stops if r["profile"] == name and r["requested_rad_s"] == speed]
            reached = [r["start_reached_ms"] for r in sub if r["start_reached_ms"] is not None]
            lines.append(f"| {name} | {speed} rad/s | {max(reached) if reached else float('nan'):.0f} ms | "
                         f"{max(r['start_peak_setpoint_accel_rad_s2'] for r in sub):.1f} / "
                         f"{max(r['start_peak_measured_accel_rad_s2'] for r in sub):.1f} rad/s² | "
                         f"{max(r['start_peak_setpoint_jerk_rad_s3'] for r in sub):.0f} rad/s³ |")
    lines += ["", "## 2. Repeated pause/resume cycles", "",
              "Six joints moving (up to 0.35 rad/s), alternating direction, 0.4 s of motion then a zero command. "
              "A 40 ms hold resumes while the robot is still braking; a 300 ms hold resumes from standstill.", "",
              "| Profile | Hold | Cycles | Reached standstill | Resumed while braking | Stop time range | "
              "Peak accel setpoint / measured | Peak jerk setpoint | Stop clamps | Contacts |",
              "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for row in cycles:
        span = ("—" if row["stop_time_ms_min"] is None else
                f"{row['stop_time_ms_min']:.0f}–{row['stop_time_ms_max']:.0f} ms")
        lines.append(f"| {row['profile']} | {1000 * row['hold_s']:.0f} ms | {row['cycles']} | {row['reached_standstill']} | "
                     f"{row['resumed_while_braking']} | {span} | {row['peak_setpoint_accel_rad_s2']:.1f} / "
                     f"{row['peak_measured_accel_rad_s2']:.1f} rad/s² | {row['peak_setpoint_jerk_rad_s3']:.0f} rad/s³ | "
                     f"{row['stop_clamps']} | {row['forbidden_contacts']} |")
    if collision_rows:
        lines += ["", "## 3. Approaching the obstacle", "",
                  "Single joints (0–2, both directions) driven toward the mapped obstacle from home and the start pose; "
                  "clearance to the collision margin checked after every 2 ms physics step until 0.5 s after the "
                  "guard blocked the motion. Only motions where the guard engaged are listed.", "",
                  "| Profile | Speed | Cases | Blocked | Minimum slack above the margin | Forbidden contacts | "
                  "Camera travel after the block |", "|---|---:|---:|---:|---:|---:|---:|"]
        for name in names:
            for speed in SPEEDS:
                sub = [r for r in collision_rows if r["profile"] == name and r["requested_rad_s"] == speed]
                if not sub:
                    continue
                travel = [r["stop_camera_travel_mm"] for r in sub if r["stop_camera_travel_mm"] is not None]
                lines.append(f"| {name} | {speed} rad/s | {len(sub)} | {sum(r['blocked'] for r in sub)} | "
                             f"{min(r['minimum_slack_mm'] for r in sub):.2f} mm | {sum(r['forbidden_contacts'] for r in sub)} | "
                             f"{max(travel) if travel else 0:.1f} mm |")
        speeds = [r["stop_speed_rad_s"] for r in collision_rows if r["stop_speed_rad_s"] is not None]
        lines += ["", "Negative slack would mean the arm entered the 12 mm (environment) or 6 mm (self) margin. "
                  + (f"The guard had already slowed the arm to at most {max(speeds):.3f} rad/s when the stop began, "
                     "so these cases test the guard's gradual slow-down, not a sudden stop at full speed next to an "
                     "obstacle. " if speeds else "")
                  + "This is a check of the braking assumption in these cases, not a formal guarantee."]
    watchdog, manual, holds = (realtime or {}).get("watchdog", []), (realtime or {}).get("manual", []), \
        (realtime or {}).get("holds", [])
    realtime = dict(realtime or {}, delay_levels=delay_levels)
    if watchdog or manual:
        lines += ["", "## 4. Realtime stops", "",
                  "Watchdog stops (stop setting, 400 ms image-age limit): the sensor stalls 0.8 s at different times after "
                  "Align. Manual stops: Stop is pressed at different times after the first command. Latency is wall "
                  "clock; stopping time is simulated (the runtime integrates wall time).", "",
                  "| Profile | Kind | Run | Speed at command | Command stop latency | Physical stopping time | "
                  "Image age at standstill | Camera travel | Unsafe / post-stop command ticks |",
                  "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
        for kind, rows in (("watchdog", watchdog), ("manual Stop", manual)):
            for row in rows:
                age = (None if row.get("deadline_to_standstill_ms") is None
                       else 1000 * AGE_LIMIT_S + row["deadline_to_standstill_ms"])
                latency = "—" if row.get("command_latency_ms") is None else "%.1f ms" % row["command_latency_ms"]
                stop = ("at rest" if not row["moving"] else "not measured" if row.get("physical_stop_ms") is None
                        else "%.0f ms" % row["physical_stop_ms"])
                lines.append(f"| {row['profile']} | {kind} | {row['run']} | {row.get('speed_at_command_rad_s') or 0:.3f} rad/s | "
                             f"{latency} | {stop} | {'—' if age is None else '%.0f ms' % age} | "
                             f"{row.get('camera_travel_mm') or 0:.1f} mm | "
                             f"{row['unsafe_motion_ticks']} / {row['post_stop_motion_ticks']} |")
    if holds:
        # A resume that never reached its command ended at the next hold: its length is the motion time.
        cut = [r for r in holds if r.get("resumes") and not r.get("resumes_reached") and r.get("resume_motion_ms_max")]
        lines += ["", "Holds and resumes in the runtime default (hold and resume, 2 s window): every frame is delayed "
                  "0.3 s after 0.3 s, so the 400 ms watchdog holds and resumes repeatedly."
                  + "".join(f" With `{r['profile']}`, every resume was followed by the next hold within "
                            f"{r['resume_motion_ms_max']:.0f} ms (the resumed image is itself already close to the age "
                            f"limit), so the arm reached at most {r['resume_peak_speed_rad_s']:.3f} rad/s between holds."
                            for r in cut), "",
                  "| Profile | Holds (while moving) | Reached standstill / resumed while braking / other | "
                  "Longest physical stop | Camera travel | Resumes (reached the command before it changed) | "
                  "Longest time to reach the command | Peak setpoint accel / jerk | Jerk-limit exceptions | "
                  "Motion ticks during a hold |",
                  "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
        for row in holds:
            stop = "—" if row["physical_stop_ms_max"] is None else "%.0f ms" % row["physical_stop_ms_max"]
            reach = "—" if row["resume_reached_ms_max"] is None else "%.0f ms" % row["resume_reached_ms_max"]
            lines.append(f"| {row['profile']} | {row['holds']} ({row['holds_moving']}) | {row['reached_standstill']} / "
                         f"{row['resumed_while_braking']} / {row['other']} | {stop} | {row['camera_travel_mm_max']:.1f} mm | "
                         f"{row['resumes']} ({row.get('resumes_reached', '?')}) | {reach} | "
                         f"{row['peak_setpoint_accel_rad_s2']:.1f} rad/s² / "
                         f"{row['peak_setpoint_jerk_rad_s3']:.0f} rad/s³ | {row['stop_clamps']} | {row['paused_motion_ticks']} |")
    levels = (realtime or {}).get("delay_levels") or []
    if levels:
        lines += ["", "Whole ArUco alignments in the runtime default (hold and resume) with a constant added delay on every "
                  "frame; one alignment per level and profile, container CPU:", "",
                  "| Added delay | " + " | ".join(names_rt := sorted({r["profile"] for r in levels}, key=names.index)) + " |",
                  "|---:|" + "---|" * len(names_rt)]
        for delay in sorted({r["delay_ms"] for r in levels}):
            cells = []
            for name in names_rt:
                row = next((r for r in levels if r["profile"] == name and r["delay_ms"] == delay), None)
                if row is None:
                    cells.append("—")
                    continue
                parts = [str(row["outcome"]) + ("" if row["duration_s"] is None else f" in {row['duration_s']:.1f} s"),
                         f"{row['holds']} holds"]
                if row["final_error_px"] is not None:
                    parts.append(f"last error {row['final_error_px']:.2f} px")
                cells.append(", ".join(parts))
            lines.append(f"| +{delay:.0f} ms | " + " | ".join(cells) + " |")
        lines += ["", "`timeout` is the controller's 20 s alignment limit; the duration is missing where the event log "
                  "(256 entries) no longer held the Align event. Same outcome with and without the model at every level "
                  "in this single-run comparison."
                  if all(len({r["outcome"] for r in levels if r["delay_ms"] == d}) == 1 for d in {r["delay_ms"] for r in levels})
                  else "`timeout` is the controller's 20 s alignment limit. The outcome differs between the profiles at "
                  "some level; see the rows."]
    lines += ["", "## 5. The 400 ms watchdog requirement against physical motion", ""]
    budget = []
    for name in names:
        ibvs = [r for r in stops if r["profile"] == name and r["requested_rad_s"] == 0.35]
        cap = [r for r in stops if r["profile"] == name and r["requested_rad_s"] == 0.6]
        t35, t60 = max(r["physical_stop_ms"] for r in ibvs), max(r["physical_stop_ms"] for r in cap)
        budget.append((name, t35, t60))
        lines.append(f"- **{name}**: worst physical stopping time {t35:.0f} ms from 0.35 rad/s (the controller's joint "
                     f"speed limit) and {t60:.0f} ms from 0.6 rad/s (the absolute cap). With the tested latency bound of "
                     f"{1000 * LATENESS_BOUND_S:.0f} ms, the robot is at standstill by an image age of at most "
                     f"{1000 * (AGE_LIMIT_S + LATENESS_BOUND_S) + t35:.0f} ms ({1000 * (AGE_LIMIT_S + LATENESS_BOUND_S) + t60:.0f} ms at the cap).")
    lines += ["", "What 400 ms means therefore has to be stated explicitly:", "",
              "- *Command* stop by 400 ms image age (the current requirement, REQ-01): unchanged and still met; "
              "the actuator model does not delay the zero command.",
              "- *Physical* standstill by 400 ms image age: not met with the current `max_age_s` = 0.4 s by any "
              "profile, including the model switched off, because the trip itself happens at 400 ms. It would need "
              "the trip at about 400 − 50 − (physical stopping time) ms of image age:"]
    for name, t35, t60 in budget:
        lines.append(f"  {name}: {1000 * (AGE_LIMIT_S - LATENESS_BOUND_S) - t35:.0f} ms for motion at up to 0.35 rad/s, "
                     f"{1000 * (AGE_LIMIT_S - LATENESS_BOUND_S) - t60:.0f} ms at up to 0.6 rad/s.")
    lines += ["", "This study does not change the requirement or `max_age_s`; see docs/ACTUATOR_MODEL.md."]
    if accuracy_rows:
        lines += ["", "## 6. Accuracy with and without the actuator model", "",
                  "Same starts, seeds, goals and perception; nominal calibration; 100 ms camera delay. Worst over the "
                  "30 post-stop frames (accuracy_config.json tolerance: 2 mm and 1°).", "",
                  "| Profile | Mode | Converged within tolerance | Worst camera | Worst tool | Median time to converge | "
                  "Worst camera travel in the final stop |",
                  "|---|---|---:|---:|---:|---:|---:|"]
        for name in names:
            for mode in ("aruco", "natural"):
                sub = [r for r in accuracy_rows if r["profile"] == name and r["mode"] == mode]
                if not sub:
                    continue
                ok = sum(r["physical_success"] for r in sub)
                travel = [r["final_stop_camera_travel_mm"] for r in sub if r["final_stop_camera_travel_mm"] is not None]
                lines.append(f"| {name} | {'SIFT' if mode == 'natural' else 'ArUco'} | {ok}/{len(sub)} | "
                             f"{max(r['worst_camera_mm'] for r in sub):.2f} mm / {max(r['worst_camera_deg'] for r in sub):.3f}° | "
                             f"{max(r['worst_tool_mm'] for r in sub):.2f} mm / {max(r['worst_tool_deg'] for r in sub):.3f}° | "
                             f"{np.median([r['completion_s'] for r in sub]):.1f} s | "
                             f"{max(travel) if travel else 0:.3f} mm |")
        lines += ["", "Learned GPU perception needs the laptop's GPU and is not part of this container run."]
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return lines


def plot(directory, examples):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, axes = plt.subplots(1, 2, figsize=(11, 4))
    for name, profile in examples.items():
        for axis, key in zip(axes, ("start", "stop")):
            data = np.array(profile[key]) if profile[key] else np.zeros((0, 3))
            if len(data):
                axis.plot(data[:, 0], data[:, 1], label=f"{name} measured")
                axis.plot(data[:, 0], data[:, 2], "--", label=f"{name} setpoint")
    for axis, title in zip(axes, ("Start 0 → 0.35 rad/s (shoulder)", "Stop 0.35 rad/s → 0 (shoulder)")):
        axis.set_title(title)
        axis.set_xlabel("time after command (ms)")
        axis.set_ylabel("largest joint speed (rad/s)")
        axis.grid(alpha=.3)
        axis.legend(fontsize=7)
    figure.tight_layout()
    figure.savefig(directory / "speed_profiles.png", dpi=120)
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stop-response", action="store_true", help=argparse.SUPPRESS)  # run.cmd route.
    parser.add_argument("--profiles", nargs="+", default=None, help="Actuator profiles (default: all in the config)")
    parser.add_argument("--realtime", type=int, default=5, metavar="N", help="Realtime watchdog stops per profile (0 = skip)")
    parser.add_argument("--accuracy", type=int, default=0, metavar="N", help="Paired accuracy starts per mode (0 = skip)")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report-only", type=Path, metavar="DIR", help="Rewrite REPORT.md from a saved summary.json")
    args = parser.parse_args(argv)
    if args.report_only:
        for line in report(args.report_only.resolve()):
            print(line)
        return 0
    names = args.profiles or profiles()
    directory = (args.output or ROOT / "results" / "stop-response" / datetime.now().strftime("%Y%m%d-%H%M%S")).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    from actuator import ActuatorModel
    collision = json.loads((ROOT / "collision_config.json").read_text(encoding="utf-8"))
    limit = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))["max_joint_velocity_rad_s"]
    shared = load_actuator_config(names[0])
    meta = dict(created_utc=datetime.now(timezone.utc).isoformat(), profiles=names,
                actuator_config=json.loads(CONFIG_PATH.read_text(encoding="utf-8")),
                stopped_velocity_rad_s=shared["stopped_velocity_rad_s"], stopped_hold_s=shared["stopped_hold_s"],
                braking_checks={n: ActuatorModel(n, max_velocity=limit,
                                                 collision_braking_time_s=collision["braking_time_s"]).braking_check
                                for n in names})
    print(f"Stop-response study: {directory}", flush=True)
    paired = [n for n in names if n in ("ideal", "default")] or names
    result = dict(meta=meta, stops=[], cycles=[], collision=[], realtime={}, accuracy=[], examples={})

    def save():  # After every phase, so an interrupted run keeps what it measured.
        (directory / "summary.json").write_text(json.dumps(result, indent=1, default=float), encoding="utf-8")
    result["stops"], result["examples"] = run_stops(names)
    print(f"  {len(result['stops'])} stops measured", flush=True)
    save()
    result["cycles"] = run_cycles(names)
    result["collision"] = run_collision(names)
    print(f"  {len(result['collision'])} obstacle approaches measured", flush=True)
    save()
    if args.realtime:
        result["realtime"] = run_realtime(paired, args.realtime)
        save()
        result["delay_levels"] = run_delay_levels(paired)
        save()
    if args.accuracy:
        result["accuracy"], plan = run_accuracy(paired, args.accuracy, directory)
        meta["accuracy_plan"] = dict(starts=args.accuracy, seed=plan["seed"], modes=plan["modes"])
        save()
    with (directory / "stops.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(result["stops"][0]))
        writer.writeheader()
        writer.writerows(result["stops"])
    plot(directory, result["examples"])
    for line in report(directory):
        print(line)
    return 0


def report(directory):
    """(Re)write REPORT.md from summary.json."""
    result = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    return write_report(directory, result["meta"], result["stops"], result["cycles"], result.get("realtime"),
                        result.get("accuracy", []), result.get("collision", []), result.get("delay_levels"))


if __name__ == "__main__":
    raise SystemExit(main())
