"""Paired simulated camera-delay trials using the desktop application's control path."""
from __future__ import annotations

import argparse
import shutil
import importlib.metadata
from datetime import datetime
import hashlib
import json
from pathlib import Path
import platform
import time

import cv2
import numpy as np
from PIL import Image

from app import Lab
from binomial_ci import format_rate
from camera_timing import load_camera_timing
from simulation import ROOT, Simulation

STARTS = [
    [3, -3, 4, 3, -2, 2],
    [-3, 2, -3, -2, 2, -2],
    [5, -4, 4, -3, 2, 3],
]
NAMES = {"aruco": "ArUco", "natural": "SIFT", "learned": "Learned"}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def error_px(corners, reference):
    return None if corners is None else float(np.sqrt(np.mean(np.sum((corners-reference)**2, axis=1))))


def trial(sim, mode, delay_ms, offsets, case, directory, interrupt=False):
    sim.reset()
    cv2.setRNGSeed(20260919 + case)
    lab = Lab(sim, auto_start=False, perception_mode=mode,
              camera_timing=load_camera_timing(delay_ms/1000))
    goal_digest = hashlib.sha256(lab.reference_path.read_bytes()).hexdigest()
    sim.reset(offsets)
    lab.align()
    initial_error = error_px(lab.perception.observe(sim.image()).corners, lab.reference)
    rows = []
    outage_capture_s = None
    observed_frame = 0
    inference = []
    started = time.perf_counter()
    for step in range(1800):
        if interrupt and outage_capture_s is None and sim.data.time >= .75:
            outage_capture_s = lab.camera.last_capture_s
            lab.toggle_camera_stream()
        lab.advance(1/30)
        frame = lab.camera.latest
        if frame is not None and frame.sequence != observed_frame:
            observed_frame = frame.sequence
            inference.append(frame.inference_ms)
        rows.append(dict(
            time_s=float(sim.data.time), state=lab.alignment_status,
            frame_id=None if frame is None else frame.sequence,
            captured_s=None if frame is None else frame.captured_s,
            delivered_s=lab.camera.delivered_s,
            age_s=lab.camera.age_s(float(sim.data.time)),
            observed_error_px=None if frame is None else error_px(frame.observation.corners, lab.reference),
            command_rad_s=sim.velocity_command.tolist(),
            current_qpos_rad=sim.data.qpos.tolist(),
            capture_qpos_rad=None if frame is None else frame.qpos.tolist()))
        if case == 0 and delay_ms == 100 and not interrupt and step == 30:
            Image.fromarray(lab.draw()).save(directory/f"{mode}-delayed-control.png")
        if not lab.aligning:
            break
    elapsed = time.perf_counter()-started
    outcome = lab.alignment_status if not lab.aligning else "study_timeout"
    finish_s = float(sim.data.time)
    if lab.aligning:
        lab.stop()
    terminal = error_px(lab.perception.observe(sim.image()).corners, lab.reference)
    post_errors = []
    commands_zero = True
    for _ in range(30):
        lab.advance(1/30)
        post_errors.append(error_px(lab.perception.observe(sim.image()).corners, lab.reference))
        commands_zero = commands_zero and not bool(sim.velocity_command.any())
    stable = all(error is not None and error < 1 for error in post_errors) and commands_zero
    if outcome == "converged" and not stable:
        outcome = "unstable_after_stop"
    assert hashlib.sha256(lab.reference_path.read_bytes()).hexdigest() == goal_digest
    trace = directory/"traces"/f"{mode}-{delay_ms:g}ms-{case}{'-outage' if interrupt else ''}.jsonl"
    trace.parent.mkdir(exist_ok=True)
    trace.write_text("".join(json.dumps(row, allow_nan=False)+"\n" for row in rows), encoding="utf-8")
    result = dict(mode=mode, delay_ms=delay_ms, case=case, offset_degrees=offsets,
                  interrupted=interrupt, outcome=outcome, completion_s=finish_s,
                  initial_error_px=initial_error, final_current_error_px=terminal,
                  max_post_stop_error_px=max(post_errors) if all(x is not None for x in post_errors) else None,
                  commands_zero_after_stop=commands_zero, wall_s=elapsed,
                  delivered_frames=lab.camera.delivered_frames,
                  dropped_frames=lab.camera.dropped_frames,
                  outage_last_capture_s=outage_capture_s,
                  failure_event=lab.last_camera_failure,
                  inference_median_ms=float(np.median(inference)) if inference else None,
                  reference_sha256=goal_digest, trace=trace.relative_to(directory).as_posix())
    if interrupt:
        assert outcome == "stale_camera", result
        assert result["failure_event"]["simulation_time_s"] - outage_capture_s <= .252000001, result
        assert commands_zero
    print(f"{NAMES[mode]:7s} {delay_ms:3g} ms case {case}: {outcome}, {finish_s:.3f} s, current error {terminal}", flush=True)
    return result


def report(directory, rows, modes, delays):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lines = [
        "# Camera delay study", "",
        "The same three declared starting offsets were evaluated through the desktop",
        "application's timed-camera path at each delay. All failure outcomes are retained.",
        "These are simulated transport delays; host inference duration is recorded separately.",
        "The loop is not a wall-clock real-time scheduler.",
        "Brackets are 95% Clopper-Pearson (exact) intervals. With three declared offsets per cell,",
        "they mainly show how little three trials can establish; they are not estimates over random starts.", "",
        "| Matcher | Delay, ms | Stable successes | Median completion, simulated s | Maximum final current-image error, px |",
        "|---|---:|---:|---:|---:|"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.7))
    for mode in modes:
        medians = []
        for delay in delays:
            group = [r for r in rows if r["mode"] == mode and r["delay_ms"] == delay and not r["interrupted"]]
            successful = [r for r in group if r["outcome"] == "converged"]
            median = float(np.median([r["completion_s"] for r in successful])) if successful else None
            finite = [r["final_current_error_px"] for r in group if r["final_current_error_px"] is not None]
            worst = max(finite) if finite else None
            completion_text = "—" if median is None else f"{median:.3f}"
            error_text = "—" if worst is None else f"{worst:.3f}"
            lines.append(f"| {NAMES[mode]} | {delay:g} | {format_rate(len(successful), len(group))} | {completion_text} | {error_text} |")
            medians.append(np.nan if median is None else median)
        axes[0].plot(delays, medians, marker="o", label=NAMES[mode])
        representative = next(r for r in rows if r["mode"] == mode and r["delay_ms"] == 100 and r["case"] == 0 and not r["interrupted"]) if 100 in delays else None
        if representative:
            trace = [json.loads(line) for line in (directory/representative["trace"]).read_text().splitlines()]
            visible = [p for p in trace if p["observed_error_px"] is not None]
            axes[1].plot([p["time_s"] for p in visible], [p["observed_error_px"] for p in visible], label=NAMES[mode])
    axes[0].set(xlabel="Configured camera delay (ms)", ylabel="Median completion (simulated s)",
                title="Successful trials only")
    axes[1].set(xlabel="Simulation time (s)", ylabel="Delayed observation error (px)", title="First start, 100 ms delay")
    for ax in axes:
        ax.grid(alpha=.25); ax.legend()
    fig.tight_layout()
    fig.savefig(directory/"camera-delay.png", dpi=160)
    fig.savefig(directory/"camera-delay.svg")
    plt.close(fig)
    lines += ["", "![Timing and delayed feedback](camera-delay.png)", "",
              "Completion means the controller declared convergence and 30 subsequent fresh",
              "current-image observations stayed below 1 px with zero joint commands.",
              "Fresh-current-image checks use the selected detector; they are not independent",
              "physical pose ground truth. Missing completion points mean no successful trials.", "",
              "## Stream interruption", "",
              "One additional 100 ms trial per matcher interrupts capture after 0.75 simulated",
              "seconds. Queued images can still arrive, but the last capture must expire",
              "within the 250 ms budget (plus at most one 2 ms physics tick).", "",
              "| Matcher | Outcome | Stop minus last capture, ms | Commands stayed zero |",
              "|---|---|---:|---|"]
    for row in rows:
        if row["interrupted"]:
            age = 1000*(row["failure_event"]["simulation_time_s"]-row["outage_last_capture_s"])
            lines.append(f"| {NAMES[row['mode']]} | {row['outcome']} | {age:.1f} | {row['commands_zero_after_stop']} |")
    lines += ["", "## Interpretation", "",
              "These small static offsets test implementation and local convergence; they do",
              "not establish a stability bound over arbitrary scenes. A shorter completion",
              "time with delay can result from commands continuing while a newer image is",
              "in flight. It is not evidence that delay generally improves the controller.",
              "No prediction or latency-dependent retuning was applied.", "",
              "[All trial outcomes](trials.json) · [Configuration](manifest.json) · [Measured source and runtime](source_manifest.json)", "",
              "The full local traces are generated under traces/ and are excluded from Git.",
              "Rerun the study to regenerate those traces. Displaying the app never generates",
              "extra feedback in timed mode.", ""]
    failures = [r for r in rows if not r["interrupted"] and r["outcome"] != "converged"]
    if failures:
        lines += ["## Trials without stable convergence", "",
                  "| Matcher | Delay, ms | Start index | Outcome | Final current-image error, px |",
                  "|---|---:|---:|---|---:|"]
        for row in failures:
            error = row["final_current_error_px"]
            value = "—" if error is None else f"{error:.3f}"
            lines.append(f"| {NAMES[row['mode']]} | {row['delay_ms']:g} | {row['case']} | {row['outcome']} | {value} |")
        lines.append("")
    (directory/"REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delay-study", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--modes", nargs="+", choices=tuple(NAMES), default=list(NAMES))
    parser.add_argument("--delays-ms", nargs="+", type=float, default=[0, 50, 100, 200])
    args = parser.parse_args()
    if len(set(args.modes)) != len(args.modes) or len(set(args.delays_ms)) != len(args.delays_ms):
        parser.error("Modes and delays must be unique")
    if any(not np.isfinite(delay) or not 0 <= delay < 250 for delay in args.delays_ms):
        parser.error("Study delays must be finite and in [0, 250) ms")
    directory = args.output or ROOT/"results/camera-delay"/datetime.now().strftime("%Y%m%d-%H%M%S")
    directory.mkdir(parents=True, exist_ok=False)
    source_names = ["app.py", "camera_timing.py", "camera_timing_config.json", "control.py",
                    "recovery.py", "joint_limits.py", "simulation.py", "config.json",
                    "scene.xml", "run_camera_delay_study.py"]
    manifest = dict(status="running", python=platform.python_version(), modes=args.modes,
                    delays_ms=args.delays_ms, starts=STARTS, camera_timing=load_camera_timing(),
                    source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_names})
    source_names += ["perception.py", "learned_perception.py", "reference_image.py", "adaptive_gain.py",
                     "startup_search.py", "natural_feature_config.json", "learned_feature_config.json",
                     "recovery_config.json", "startup_search_config.json", "joint_limit_config.json", "gain_config.json"]
    snapshot = directory/"source"
    snapshot.mkdir()
    for name in source_names:
        shutil.copy2(ROOT/name, snapshot/name)
    versions = {}
    for name in ("numpy", "opencv-python", "mujoco", "torch", "torchvision", "lightglue"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    write_json(directory/"source_manifest.json", dict(
        source_sha256={name: hashlib.sha256((snapshot/name).read_bytes()).hexdigest() for name in source_names},
        versions=versions,
        reference_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
                          ("reference/goal.npz", "reference/natural_goal.npz")}))
    write_json(directory/"manifest.json", manifest)
    rows = []
    try:
        with Simulation() as sim:
            for mode in args.modes:
                for delay in args.delays_ms:
                    for case, offsets in enumerate(STARTS):
                        rows.append(trial(sim, mode, delay, offsets, case, directory))
                        write_json(directory/"trials.json", rows)
                rows.append(trial(sim, mode, 100, STARTS[0], 0, directory, interrupt=True))
                write_json(directory/"trials.json", rows)
        report(directory, rows, args.modes, args.delays_ms)
        manifest["status"] = "complete"
        manifest["completed_trials"] = len(rows)
        write_json(directory/"manifest.json", manifest)
        print("Report:", directory/"REPORT.md")
    except BaseException as exc:
        manifest.update(status="failed", error=f"{type(exc).__name__}: {exc}", completed_trials=len(rows))
        write_json(directory/"manifest.json", manifest)
        raise


if __name__ == "__main__":
    main()
