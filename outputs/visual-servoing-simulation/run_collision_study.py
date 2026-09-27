"""Reproduce collision-aware alignment, cold search and remembered-view recovery."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import traceback

import numpy as np

from camera_robustness import ROOT, digest, write_json
from camera_timing import load_camera_timing
from run_camera_robustness import error_px, fingerprint

CASES = ("alignment", "cold_search_obstacle", "recovery_obstacle")


def run_case(sim, mode, case, directory, timing):
    import cv2
    from PIL import Image
    from app import Lab

    sim.set_obstacle(False)
    sim.reset()
    sim.set_obstacle(case != "alignment")
    cv2.setRNGSeed(20260920)
    cold = case == "cold_search_obstacle"
    if cold:
        sim.reset([35, 0, 0, 0, 0, 0])
    lab = Lab(sim, auto_start=False, cold_start=cold,
              perception_mode=mode, camera_timing=timing)
    reference_hash = digest(lab.reference_path)
    remembered = False
    if case == "alignment":
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
    elif case == "recovery_obstacle":
        # Learn the return view from a delivered image before the explicit test displacement.
        lab.advance(.25)
        remembered = lab.controller.last_visible_qpos is not None
        if not remembered:
            raise RuntimeError("Recovery fixture did not acquire a real visible viewpoint")
        lab.lost_view()
        if lab.controller.last_visible_qpos is None:
            raise RuntimeError("Lost view action discarded the observed recovery viewpoint")
    initial = error_px(lab.perception.observe(sim.image()).corners, lab.reference)
    if case != "alignment" and initial is not None:
        raise RuntimeError("Search fixture must start without a target detection")
    lab.align()
    trial_id = mode + "-" + case
    trace_path = directory/"traces"/(trial_id+".jsonl")
    started = time.perf_counter()
    min_distance, min_slack = float("inf"), float("inf")
    contacts = 0
    violations = set()
    dt = float(sim.model.opt.timestep)
    last_frame, last_state = 0, None
    plot_rows = []
    detour_seen_s = None
    detour_image = None

    def inspect():
        nonlocal min_distance, min_slack, contacts
        q = sim.data.qpos
        if not np.isfinite(q).all() or not np.isfinite(sim.velocity_command).all():
            violations.add("nonfinite_state")
            return
        distances, _ = sim.collision.distances(q)
        min_distance = min(min_distance, float(np.min(distances)))
        min_slack = min(min_slack, float(np.min(distances-sim.collision.margins)))
        contacts += len(sim.forbidden_contacts())
        if min_slack < -1e-6:
            violations.add("collision_clearance")
        if contacts:
            violations.add("forbidden_contact")
        if np.any(q < sim.model.jnt_range[:, 0]-1e-6) or np.any(q > sim.model.jnt_range[:, 1]+1e-6):
            violations.add("joint_limit")
        age = lab.camera.age_s(float(sim.data.time))
        if sim.velocity_command.any() and (age is None or age >= timing["max_observation_age_s"]-1e-10):
            violations.add("motion_without_fresh_feedback")

    inspect()
    with trace_path.open("w", encoding="utf-8") as trace:
        while lab.aligning:
            lab.advance(dt)
            inspect()
            now = float(sim.data.time)
            frame = lab.camera.latest
            frame_id = 0 if frame is None else frame.sequence
            if frame_id != last_frame or lab.alignment_status != last_state or violations:
                distances, _ = sim.collision.distances(sim.data.qpos)
                row = dict(time_s=now, state=lab.alignment_status,
                           observed_error_px=None if frame is None else error_px(frame.observation.corners, lab.reference),
                           captured_s=None if frame is None else frame.captured_s,
                           qpos_rad=sim.data.qpos.tolist(),
                           requested_rad_s=sim.requested_velocity.tolist(),
                           command_rad_s=sim.velocity_command.tolist(),
                           clearance_m=float(np.min(distances)),
                           clearance_slack_m=float(np.min(distances-sim.collision.margins)),
                           guard=sim.collision.last.status, detours=sim.collision.detours)
                trace.write(json.dumps(row, allow_nan=False)+"\n")
                plot_rows.append(row)
                last_frame, last_state = frame_id, lab.alignment_status
            if sim.collision.detours and detour_seen_s is None:
                detour_seen_s = now
            if detour_seen_s is not None and detour_image is None and now >= detour_seen_s+1:
                detour_image = trial_id+"-detour.png"
                Image.fromarray(lab.draw()).save(directory/detour_image)
            if now > timing["max_run_s"]+dt:
                violations.add("run_deadline")
            if violations:
                lab.stop()
                break
    outcome = "safety_violation" if violations else lab.alignment_status
    completion = float(sim.data.time)
    states = sorted(set(r["state"] for r in plot_rows))
    startup = lab.controller.startup
    skipped = lab.controller.blocked_waypoints + (0 if startup is None else startup.blocked_waypoints)
    # Independent current images verify the stopped pose; queued delayed images cannot pass this.
    post_errors, stopped = [], True
    for _ in range(30):
        end = float(sim.data.time)+1/sim.config["camera_hz"]
        while sim.data.time < end-1e-10:
            lab.advance(dt)
            inspect()
            stopped = stopped and not bool(sim.velocity_command.any()) and not lab.aligning
        post_errors.append(error_px(lab.perception.observe(sim.image()).corners, lab.reference))
    stable = stopped and all(e is not None and e < sim.config["ibvs"]["success_error_px"] for e in post_errors)
    if not stopped:
        violations.add("motion_after_stop")
    if violations:
        outcome = "safety_violation"
    elif outcome == "converged" and not stable:
        outcome = "unstable_after_stop"
    if digest(lab.reference_path) != reference_hash:
        raise RuntimeError("Experiment changed its saved reference image")
    final_image = trial_id+"-final.png"
    Image.fromarray(lab.draw()).save(directory/final_image)
    return dict(id=trial_id, mode=mode, case=case, outcome=outcome,
                passed=outcome == "converged" and stable and not violations,
                completion_s=completion, initial_error_px=initial,
                max_post_stop_error_px=max(post_errors) if all(e is not None for e in post_errors) else None,
                post_stop_frames=30, commands_zero_after_stop=stopped,
                minimum_clearance_mm=1000*min_distance, minimum_clearance_slack_mm=1000*min_slack,
                forbidden_contacts=contacts, safety_violations=sorted(violations),
                detours=sim.collision.detours, blocked_waypoints=skipped,
                limited_guard_calls=sim.collision.limited_steps, blocked_guard_calls=sim.collision.blocked_steps,
                remembered_view_before_displacement=remembered, states=states,
                matcher=lab.perception.name, reference_sha256=reference_hash,
                wall_s=time.perf_counter()-started,
                trace=trace_path.relative_to(directory).as_posix(), final_image=final_image,
                detour_image=detour_image), plot_rows


def report(directory, manifest, rows, trajectories):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True)
    colors = dict(aruco="#157a8a", natural="#bf7014", learned="#7556a6")
    styles = dict(alignment="-", cold_search_obstacle="--", recovery_obstacle=":")
    for row in rows:
        trace = trajectories.get(row["id"], [])
        if not trace:
            continue
        t = [r["time_s"] for r in trace]
        kw = dict(color=colors[row["mode"]], linestyle=styles[row["case"]], linewidth=1.3, label=row["id"])
        axes[0].plot(t, [np.nan if r["observed_error_px"] is None else r["observed_error_px"] for r in trace], **kw)
        axes[1].plot(t, [1000*r["clearance_slack_m"] for r in trace], **kw)
        axes[2].plot(t, [np.rad2deg(r["qpos_rad"][1]) for r in trace], **kw)
    axes[0].axhline(1, color="black", linewidth=.7)
    axes[0].set(ylabel="Delivered-image error (px)", yscale="symlog", ylim=(0, None))
    axes[1].axhline(0, color="firebrick", linewidth=1)
    axes[1].set(ylabel="Minimum clearance minus\napplicable margin (mm)")
    axes[2].set(ylabel="Shoulder angle (degrees)", xlabel="Simulation time (s)")
    for axis in axes:
        axis.grid(alpha=.2)
    axes[0].legend(fontsize=7, ncol=2)
    fig.suptitle("Collision-aware control | 100 ms camera delay | checks every 2 ms")
    fig.savefig(directory/"trajectories.png", dpi=155)
    plt.close(fig)
    passed = sum(r["passed"] for r in rows)
    lines = ["# Collision-aware motion validation", "",
             f"{passed}/{len(rows)} cases converged and stayed below 1 px in 30 new current images after stopping.",
             "", "All cases use the same production guard, bounded planner, controller and 100 ms simulated camera delay. "
             "Clearance and forbidden contacts are checked at every 2 ms physics step, including the stopped interval. "
             "Slack is the smallest signed distance minus its applicable 12 mm environment or 6 mm self margin.",
             "", "| Matcher | Case | Outcome | Time (s) | Min distance (mm) | Min slack (mm) | Detours | Skipped | Contacts |",
             "|---|---|---|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        if "error" in r:
            lines.append(f"| {r['mode']} | {r['case']} | ERROR | — | — | — | — | — | — |")
        else:
            lines.append(f"| {r['mode']} | {r['case']} | {r['outcome']} | {r['completion_s']:.2f} | "
                         f"{r['minimum_clearance_mm']:.2f} | {r['minimum_clearance_slack_mm']:.2f} | "
                         f"{r['detours']} | {r['blocked_waypoints']} | {r['forbidden_contacts']} |")
    lines += ["", "![Recorded trajectories](trajectories.png)", "",
              "## Reproduce", "", "From the repository root:", "", "~~~powershell", ".\\run.cmd --collision-study", "~~~", "",
              "Baseline starts at offsets [3, -3, 4, 3, -2, 2] degrees. Both obstacle cases start at +35 degrees yaw. "
              "Cold search has no remembered view. Recovery first remembers a real delivered home image, then the "
              "Lost view action explicitly loads the lost-view pose and flushes queued camera images.",
              "", "The straight joint-space return crosses the column despite clear endpoints. The unit suite separately "
              "checks this swept segment, a three-segment detour, maximum-speed braking, manual-axis preservation, "
              "invalid placement/reset rollback, and bounded failure.",
              "", "The planner searches a finite family of joint detours; a feasible path elsewhere may be missed. "
              "These deterministic cases are regression evidence, not a success-rate estimate over arbitrary scenes "
              "or a hardware safety certification. Host timing is diagnostic; it is not included in simulated delay.",
              "", "Source, asset, reference and dependency fingerprints are saved in [manifest.json](manifest.json). "
              "Every trial, including failures, is retained in [trials.json](trials.json). "
              "Raw per-frame traces are generated locally and ignored by Git.",
              ""]
    (directory/"REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collision-study", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--modes", nargs="+", choices=("aruco", "natural", "learned"), default=["aruco", "natural", "learned"])
    parser.add_argument("--output", type=Path, help="New output directory; existing results are never overwritten")
    args = parser.parse_args(argv)
    directory = (args.output or ROOT/"results/collision"/datetime.now().strftime("%Y%m%d-%H%M%S-%f")).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    (directory/"traces").mkdir()
    timing = load_camera_timing(.1)
    modes = list(dict.fromkeys(args.modes))
    manifest = dict(schema_version=1, created_utc=datetime.now(timezone.utc).isoformat(),
                    fingerprint=fingerprint(), timing=timing, modes=modes, cases=list(CASES),
                    status="running", planned_trials=len(modes)*len(CASES), completed_trials=0)
    write_json(directory/"manifest.json", manifest)
    rows, trajectories = [], {}
    from simulation import Simulation
    print(f"Collision experiment: {directory}", flush=True)
    try:
        with Simulation() as sim:
            for mode in modes:
                for case in CASES:
                    try:
                        row, trace = run_case(sim, mode, case, directory, timing)
                        trajectories[row["id"]] = trace
                    except Exception as exc:
                        sim.command_velocity(np.zeros(6))
                        row = dict(id=mode+"-"+case, mode=mode, case=case, outcome="error",
                                   passed=False, error=f"{type(exc).__name__}: {exc}")
                        (directory/"traces"/(row["id"]+".error.txt")).write_text(traceback.format_exc(), encoding="utf-8")
                    rows.append(row)
                    write_json(directory/"trials.json", rows)
                    manifest["completed_trials"] = len(rows)
                    write_json(directory/"manifest.json", manifest)
                    print(f"[{len(rows)}/{manifest['planned_trials']}] {row['id']}: {row['outcome']}", flush=True)
        if fingerprint() != manifest["fingerprint"]:
            raise RuntimeError("Experiment inputs changed during execution")
        report(directory, manifest, rows, trajectories)
        manifest["status"] = "complete"
        write_json(directory/"manifest.json", manifest)
    except BaseException:
        manifest["status"] = "interrupted"
        write_json(directory/"manifest.json", manifest)
        raise
    return 0 if all(r["passed"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())

