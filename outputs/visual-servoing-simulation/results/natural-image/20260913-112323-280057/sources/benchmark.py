"""Randomized evaluation of the unchanged step-2 IBVS controller."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

from control import IBVSController
from simulation import ROOT, Simulation

BENCH_CONFIG = ROOT / "benchmark_config.json"
SOURCE_FILES = ("control.py", "simulation.py", "scene.xml", "config.json", "benchmark.py", "benchmark_config.json")
OUTCOMES = (
    "converged", "initial_out_of_view", "initial_tracking_loss", "fov_loss",
    "tracking_loss", "invalid_depth", "joint_limit_stall", "singularity_stall",
    "stalled", "timeout", "unstable_after_stop",
)


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")


def source_hashes() -> dict[str, str]:
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCE_FILES}


def sample_plan(n: int, seed: int, config: dict) -> list[dict]:
    """IID mixture of joint-space boxes. Seed and prefixes are reproducible."""
    if n < 1 or seed < 0:
        raise ValueError("Trial count must be positive and seed nonnegative")
    profiles = config["profiles_degrees"]
    names = list(profiles)
    if not names:
        raise ValueError("At least one sampling profile is required")
    for bounds in profiles.values():
        values = np.asarray(bounds, dtype=float)
        if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0):
            raise ValueError("Each profile requires six positive finite bounds")
    rng = np.random.default_rng(seed)
    plan = []
    for index in range(n):
        name = names[int(rng.integers(len(names)))]
        bounds = np.asarray(profiles[name], dtype=float)
        plan.append({"trial_id": index + 1, "profile": name,
                     "offset_degrees": rng.uniform(-bounds, bounds).tolist()})
    return plan


def project_marker_visibility(sim: Simulation) -> bool:
    """Evaluation only: are all physical marker corners in front and in frame?

    This uses scene ground truth to diagnose FOV loss. Nothing from this function
    is passed to the controller. In-frame does not imply unoccluded/detectable.
    """
    geom = sim.model.geom("target_board")
    h = sim.config["marker_side_m"] / 2
    local = np.array([[-geom.size[0], h, h], [-geom.size[0], -h, h],
                      [-geom.size[0], -h, -h], [-geom.size[0], h, -h]])
    rotation = sim.data.geom_xmat[geom.id].reshape(3, 3)
    world = local @ rotation.T + sim.data.geom_xpos[geom.id]
    pose = sim.camera_pose()
    front_normal = rotation @ np.array([-1., 0, 0])
    if np.dot(pose[:3, 3] - world.mean(axis=0), front_normal) <= 0:
        return False
    points = (world - pose[:3, 3]) @ pose[:3, :3]
    if np.any(points[:, 2] <= 0):
        return False
    projected = points @ sim.camera_intrinsics().T
    pixels = projected[:, :2] / projected[:, 2:3]
    return bool(np.all((pixels[:, 0] >= 0) & (pixels[:, 0] < sim.width) &
                       (pixels[:, 1] >= 0) & (pixels[:, 1] < sim.height)))


def classify_terminal(status: str, initial: bool, in_frame: bool,
                      stalled: bool = False, near_limit: bool = False,
                      condition: float = 0, condition_limit: float = 1e4) -> str:
    """Diagnostic precedence. A stalled error does not prove a local minimum."""
    if status == "tracking_loss":
        if initial:
            return "initial_tracking_loss" if in_frame else "initial_out_of_view"
        return "tracking_loss" if in_frame else "fov_loss"
    if status in ("converged", "invalid_depth"):
        return status
    if stalled:
        if near_limit:
            return "joint_limit_stall"
        if condition >= condition_limit:
            return "singularity_stall"
        return "stalled"
    if status == "timeout":
        return "timeout"
    return "running"


def error_metric(corners, desired) -> float | None:
    if corners is None:
        return None
    return float(np.sqrt(np.mean(np.sum((corners - desired)**2, axis=1))))


def run_trial(sim: Simulation, desired: np.ndarray, spec: dict, bench: dict):
    """Run one trial. Every control input is computed by the existing controller."""
    sim.reset(spec["offset_degrees"])
    controller = IBVSController(desired, sim.camera_intrinsics(), sim.config)
    dt = 1 / sim.config["camera_hz"]
    fields = ("time_s", "error_px", "corners_px", "depth_estimate_m", "qpos_rad", "qvel_rad_s",
              "command_rad_s", "camera_twist", "joint_condition", "camera_pose_world",
              "detected", "in_frame_ground_truth", "phase")
    data = {name: [] for name in fields}
    initial_rgb = sim.image()
    initial_pose = sim.camera_pose().copy()
    initial_detected = False
    below_entry = None
    outcome = "timeout"
    started = time.perf_counter()

    def record(corners, command, twist, depths, condition, in_frame, phase):
        error = error_metric(corners, desired)
        values = {
            "time_s": float(sim.data.time), "error_px": np.nan if error is None else error,
            "corners_px": np.full((4, 2), np.nan) if corners is None else corners.copy(),
            "depth_estimate_m": np.full(4, np.nan) if depths is None else depths.copy(),
            "qpos_rad": sim.data.qpos.copy(), "qvel_rad_s": sim.data.qvel.copy(),
            "command_rad_s": command.copy(), "camera_twist": twist.copy(),
            "joint_condition": condition, "camera_pose_world": sim.camera_pose().copy(),
            "detected": corners is not None, "in_frame_ground_truth": in_frame, "phase": phase,
        }
        for key, value in values.items():
            data[key].append(value)
        return error

    horizon = int(np.ceil(sim.config["ibvs"]["timeout_s"] / dt)) + 2
    for tick in range(horizon):
        rgb = initial_rgb if tick == 0 else sim.image()
        corners = sim.marker_corners(rgb)
        if tick == 0:
            initial_detected = corners is not None
        jacobian = sim.camera_jacobian()
        singular = np.linalg.svd(jacobian, compute_uv=False)
        condition = float(singular[0] / max(singular[-1], 1e-15))
        sample = controller.update(corners, jacobian, dt)
        in_frame = project_marker_visibility(sim)
        error = record(corners, sample.velocity, sample.camera_twist, sample.depths,
                       condition, in_frame, "control")
        if error is not None and error < sim.config["ibvs"]["success_error_px"]:
            if below_entry is None:
                below_entry = float(sim.data.time)
        else:
            below_entry = None
        window = int(np.ceil(bench["stall_window_s"] / dt)) + 1
        stalled = False
        if len(data["error_px"]) >= window and error is not None and error >= sim.config["ibvs"]["success_error_px"]:
            recent = np.asarray(data["error_px"][-window:])
            speeds = np.asarray(data["qvel_rad_s"][-window:])
            stalled = bool(np.isfinite(recent).all() and np.ptp(recent) <= bench["stall_error_span_px"] and
                           np.max(np.abs(speeds)) < bench["stall_actual_joint_speed_rad_s"])
        margin = bench["joint_limit_margin_rad"]
        near_limit = bool(np.any((sim.data.qpos < sim.model.jnt_range[:, 0] + margin) & (sample.velocity < 0)) or
                          np.any((sim.data.qpos > sim.model.jnt_range[:, 1] - margin) & (sample.velocity > 0)))
        outcome = classify_terminal(sample.status, tick == 0, in_frame, stalled, near_limit,
                                     condition, bench["singular_condition_threshold"])
        # An evaluator stop overrides the controller request; record applied input.
        if outcome != "running":
            data["command_rad_s"][-1] = np.zeros(6)
        sim.command_velocity(sample.velocity if outcome == "running" else np.zeros(6))
        if outcome != "running":
            break
        sim.advance(dt)
        if not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all():
            raise FloatingPointError(f"Non-finite physics state in trial {spec['trial_id']}")
    terminal_time = float(sim.data.time)
    terminal_error = error_metric(corners, desired)
    terminal_rgb = rgb.copy()
    if outcome == "converged":
        for _ in range(int(np.ceil(bench["post_stop_observation_s"] / dt))):
            sim.command_velocity(np.zeros(6))
            sim.advance(dt)
            rgb = sim.image()
            corners = sim.marker_corners(rgb)
            sv = np.linalg.svd(sim.camera_jacobian(), compute_uv=False)
            condition = float(sv[0] / max(sv[-1], 1e-15))
            error = record(corners, np.zeros(6), np.zeros(6), None, condition,
                           project_marker_visibility(sim), "post_stop")
            if error is None or error >= sim.config["ibvs"]["success_error_px"]:
                outcome = "unstable_after_stop"
        terminal_rgb = rgb.copy()
    sim.command_velocity(np.zeros(6))
    arrays = {key: np.asarray(values) for key, values in data.items()}
    errors = arrays["error_px"]
    finite_errors = errors[np.isfinite(errors)]
    initial_error = None if not np.isfinite(errors[0]) else float(errors[0])
    final_error = None if not np.isfinite(errors[-1]) else float(errors[-1])
    peak_error = float(np.max(finite_errors)) if len(finite_errors) else None
    summary = {
        "trial_id": spec["trial_id"], "profile": spec["profile"], "outcome": outcome,
        "initial_detected": initial_detected,
        "initial_in_frame_ground_truth": bool(arrays["in_frame_ground_truth"][0]),
        "initial_error_px": initial_error, "terminal_error_px": terminal_error,
        "final_error_px": final_error, "peak_error_px": peak_error,
        "peak_error_increase_pct": (100 * (peak_error / initial_error - 1)
                                    if initial_error is not None and initial_error > 0 else None),
        "terminal_time_s": terminal_time,
        "settling_time_s": below_entry if outcome == "converged" else None,
        "observed_time_s": float(sim.data.time), "sample_count": len(errors),
        "max_joint_condition": float(np.max(arrays["joint_condition"])),
        "max_command_rad_s": float(np.max(np.abs(arrays["command_rad_s"]))),
        "wall_time_s": time.perf_counter() - started,
        "initial_camera_position_m": initial_pose[:3, 3].tolist(),
        "offset_degrees": spec["offset_degrees"],
    }
    return summary, arrays, initial_rgb, terminal_rgb


def run_benchmark(output: Path, plan: list[dict], bench: dict, seed: int | None) -> Path:
    """Checkpoint each completed trial, preserving all prior run directories."""
    output.mkdir(parents=True, exist_ok=False)
    (output / "traces").mkdir()
    (output / "cases").mkdir()
    manifest = {"schema_version": 1, "status": "running", "seed": seed,
                "requested_trials": len(plan), "completed_trials": 0,
                "created_utc": datetime.now(timezone.utc).isoformat(), "benchmark_config": bench,
                "source_sha256": source_hashes(), "python": platform.python_version(),
                "mujoco": mujoco.__version__, "opencv": cv2.__version__, "numpy": np.__version__,
                "ground_truth_use": "FOV diagnostics only; controller uses rendered marker features, estimated depth and robot kinematics."}
    write_json(output / "plan.json", plan)
    write_json(output / "manifest.json", manifest)
    counts = Counter()
    try:
        with Simulation() as sim:
            reference_rgb = sim.image()
            desired = sim.marker_corners(reference_rgb)
            if desired is None:
                raise RuntimeError("The reference marker is not detected")
            manifest["simulation_config"] = sim.config
            manifest["desired_corners_px"] = desired.tolist()
            manifest["camera_K"] = sim.camera_intrinsics().tolist()
            manifest["home_camera_pose_world"] = sim.camera_pose().tolist()
            Image.fromarray(reference_rgb).save(output / "reference.png")
            home = sim.data.qpos.copy()
            for trial in plan:
                q = home + np.deg2rad(trial["offset_degrees"])
                if np.any(q < sim.model.jnt_range[:, 0]) or np.any(q > sim.model.jnt_range[:, 1]):
                    raise ValueError("Sampling plan exceeds robot joint limits")
            write_json(output / "manifest.json", manifest)
            with (output / "trials.jsonl").open("w", encoding="utf-8") as stream:
                for trial in plan:
                    summary, arrays, before, after = run_trial(sim, desired, trial, bench)
                    np.savez_compressed(output / "traces" / f"trial_{trial['trial_id']:04d}.npz", **arrays)
                    if counts[summary["outcome"]] == 0:
                        case = output / "cases" / summary["outcome"]
                        case.mkdir()
                        Image.fromarray(before).save(case / "initial.png")
                        Image.fromarray(after).save(case / "terminal.png")
                        write_json(case / "trial.json", summary)
                    stream.write(json.dumps(summary, allow_nan=False) + "\n")
                    stream.flush()
                    counts[summary["outcome"]] += 1
                    manifest["completed_trials"] += 1
                    write_json(output / "manifest.json", manifest)
                    print(f"{manifest['completed_trials']:3d}/{len(plan)} | trial {trial['trial_id']:03d} | "
                          f"{trial['profile']:6s} | {summary['outcome']:22s} | "
                          f"initial {summary['initial_error_px']} px | final {summary['final_error_px']} px", flush=True)
        manifest["status"] = "complete"
    except BaseException as exc:
        manifest["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "manifest.json", manifest)
    from analyze_benchmark import analyze
    analyze(output)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output", type=Path, help="New directory; existing runs are preserved")
    parser.add_argument("--replay-run", type=Path)
    parser.add_argument("--trial-id", type=int)
    args = parser.parse_args()
    bench = read_json(BENCH_CONFIG)
    if args.replay_run:
        if args.trial_id is None or args.trials is not None or args.seed is not None:
            parser.error("Replay requires --trial-id and cannot change --trials/--seed")
        manifest = read_json(args.replay_run / "manifest.json")
        current = source_hashes()
        for name in SOURCE_FILES:
            if manifest["source_sha256"][name] != current[name]:
                parser.error(f"Replay source differs: {name}. Restore the recorded version first.")
        saved_plan = read_json(args.replay_run / "plan.json")
        plan = [trial for trial in saved_plan if trial["trial_id"] == args.trial_id]
        if not plan:
            parser.error("Trial ID is not in the saved plan")
        bench = manifest["benchmark_config"]
        seed = manifest["seed"]
    else:
        if args.trial_id is not None:
            parser.error("--trial-id requires --replay-run")
        seed = bench["seed"] if args.seed is None else args.seed
        count = bench["trials"] if args.trials is None else args.trials
        plan = sample_plan(count, seed, bench)
    name = datetime.now().strftime("%Y%m%d-%H%M%S-%f") + f"-n{len(plan)}-seed{seed}"
    output = args.output or ROOT / "results" / "benchmark" / name
    run_benchmark(output, plan, bench, seed)
    if not args.replay_run:
        (ROOT / "results" / "benchmark").mkdir(parents=True, exist_ok=True)
        try:
            location = {"relative_to_project": output.resolve().relative_to(ROOT).as_posix()}
        except ValueError:
            location = {"absolute_path": str(output.resolve())}
        write_json(ROOT / "results" / "benchmark" / "latest.json", location)
    print(f"Saved benchmark: {output.resolve()}")


if __name__ == "__main__":
    main()

