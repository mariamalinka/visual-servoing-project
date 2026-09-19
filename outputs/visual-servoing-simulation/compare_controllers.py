"""Paired alignment comparison: python compare_controllers.py [--trials 200].

No target recovery is used. Each controller receives identical starting states.
The target-step demonstration is separate from the randomized static benchmark.
"""
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

from baselines import METHODS, make_controller, update_controller
from benchmark import (BENCH_CONFIG, classify_terminal, error_metric,
                       project_marker_visibility, read_json, sample_plan, write_json)
from simulation import ROOT, Simulation

COMPARISON_CONFIG = ROOT / "comparison_config.json"
SOURCE_FILES = ("baselines.py", "control.py", "simulation.py", "scene.xml", "config.json",
                "benchmark.py", "benchmark_config.json", "compare_controllers.py",
                "comparison_config.json", "analyze_comparison.py")


def run_comparison_trial(sim, desired, spec, bench, comparison, method, target_step=None, *, controller_factory=None):
    """Drive physical velocity actuators; evaluation-only images score every method."""
    target_id = sim.model.body("target").id
    original_target = sim.model.body_pos[target_id].copy()
    try:
        sim.reset(spec["offset_degrees"])
        controller = (make_controller(method, desired, sim.camera_intrinsics(), sim.config, comparison)
                      if controller_factory is None else
                      controller_factory(desired, sim.camera_intrinsics(), sim.config))
        dt = 1 / sim.config["camera_hz"]
        fields = ("time_s", "error_px", "corners_px", "qpos_rad", "qvel_rad_s",
                  "command_rad_s", "camera_twist", "camera_pose_world", "joint_condition",
                  "detected", "in_frame_ground_truth", "phase", "target_step_applied", "gain_per_s")
        data = {name: [] for name in fields}
        initial_rgb = sim.image()
        stepped = False
        step_applied_time = None
        below_entry = None
        started = time.perf_counter()

        def record(corners, velocity, twist, phase):
            error = error_metric(corners, desired)
            sv = np.linalg.svd(sim.camera_jacobian(), compute_uv=False)
            condition = float(sv[0] / max(sv[-1], 1e-15))
            values = {
                "time_s": float(sim.data.time), "error_px": np.nan if error is None else error,
                "corners_px": np.full((4, 2), np.nan) if corners is None else corners.copy(),
                "qpos_rad": sim.data.qpos.copy(), "qvel_rad_s": sim.data.qvel.copy(),
                "command_rad_s": velocity.copy(), "camera_twist": twist.copy(),
                "camera_pose_world": sim.camera_pose().copy(), "joint_condition": condition,
                "detected": corners is not None, "in_frame_ground_truth": project_marker_visibility(sim),
                "phase": phase, "target_step_applied": stepped,
                "gain_per_s": float(controller.config["gain_per_s"]),
            }
            for name, value in values.items():
                data[name].append(value)
            return error, condition

        outcome = "timeout"
        for tick in range(int(np.ceil(sim.config["ibvs"]["timeout_s"] / dt)) + 2):
            if target_step and not stepped and sim.data.time + 1e-9 >= target_step["time_s"]:
                sim.model.body_pos[target_id] = original_target + target_step["translation_world_m"]
                mujoco.mj_forward(sim.model, sim.data)
                stepped = True
                step_applied_time = float(sim.data.time)
            rgb = initial_rgb if tick == 0 else sim.image()
            corners = sim.marker_corners(rgb)
            sample = update_controller(controller, corners, sim.camera_jacobian(), dt, sim.camera_pose())
            error, condition = record(corners, sample.velocity, sample.camera_twist, "control")
            if error is not None and error < sim.config["ibvs"]["success_error_px"]:
                if below_entry is None:
                    below_entry = float(sim.data.time)
            else:
                below_entry = None
            window = int(np.ceil(bench["stall_window_s"] / dt)) + 1
            stalled = False
            if len(data["error_px"]) >= window and error is not None and error >= sim.config["ibvs"]["success_error_px"]:
                recent = np.asarray(data["error_px"][-window:])
                stalled = bool(np.isfinite(recent).all() and np.ptp(recent) <= bench["stall_error_span_px"]
                               and np.max(np.abs(data["qvel_rad_s"][-window:])) < bench["stall_actual_joint_speed_rad_s"])
            margin = bench["joint_limit_margin_rad"]
            near_limit = bool(np.any((sim.data.qpos < sim.model.jnt_range[:, 0] + margin) & (sample.velocity < 0))
                              or np.any((sim.data.qpos > sim.model.jnt_range[:, 1] - margin) & (sample.velocity > 0)))
            if sample.status == "motion_complete":
                outcome = "motion_complete"
            else:
                outcome = classify_terminal(sample.status, tick == 0, data["in_frame_ground_truth"][-1],
                                            stalled, near_limit, condition, bench["singular_condition_threshold"])
            if outcome != "running":
                data["command_rad_s"][-1] = np.zeros(6)
            sim.command_velocity(sample.velocity if outcome == "running" else np.zeros(6))
            if outcome != "running":
                break
            sim.advance(dt)
            if not np.isfinite(sim.data.qpos).all() or not np.isfinite(sim.data.qvel).all():
                raise FloatingPointError("Non-finite physics state")

        terminal_time = float(sim.data.time)
        terminal_error = error_metric(corners, desired)
        # Open-loop's controller declares only motion completion. The evaluator
        # independently decides success; camera measurements never correct it.
        if outcome in ("converged", "motion_complete"):
            observed_hold = below_entry is not None and (
                terminal_time - below_entry + .0021 >= sim.config["ibvs"]["success_hold_s"])
            success_candidate = observed_hold and terminal_error is not None and (
                terminal_error < sim.config["ibvs"]["success_error_px"])
            outcome = "converged" if success_candidate else "open_loop_residual"
            for _ in range(int(np.ceil(bench["post_stop_observation_s"] / dt))):
                sim.command_velocity(np.zeros(6))
                sim.advance(dt)
                rgb = sim.image()
                corners = sim.marker_corners(rgb)
                error, _ = record(corners, np.zeros(6), np.zeros(6), "post_stop")
                if success_candidate and (error is None or error >= sim.config["ibvs"]["success_error_px"]):
                    outcome = "unstable_after_stop"

        arrays = {key: np.asarray(values) for key, values in data.items()}
        errors = arrays["error_px"]
        finite_errors = errors[np.isfinite(errors)]
        initial_error = None if not np.isfinite(errors[0]) else float(errors[0])
        final_error = None if not np.isfinite(errors[-1]) else float(errors[-1])
        peak = float(np.max(finite_errors)) if len(finite_errors) else None
        summary = {
            "trial_id": spec["trial_id"], "method": method, "profile": spec["profile"],
            "scenario": "target_step" if target_step else "static",
            "outcome": outcome, "initial_detected": bool(arrays["detected"][0]),
            "offset_degrees": spec["offset_degrees"], "initial_error_px": initial_error,
            "terminal_error_px": terminal_error, "final_error_px": final_error,
            "peak_error_px": peak, "peak_error_increase_pct": (
                100 * (peak / initial_error - 1) if initial_error is not None and initial_error > 0 else None),
            "terminal_time_s": terminal_time,
            "settling_time_s": below_entry if outcome == "converged" else None,
            "observed_time_s": float(sim.data.time), "sample_count": len(errors),
            "max_command_rad_s": float(np.max(np.abs(arrays["command_rad_s"]))),
            "max_joint_condition": float(np.max(arrays["joint_condition"])),
            "wall_time_s": time.perf_counter() - started, "target_step_time_s": step_applied_time,
        }
        return summary, arrays, initial_rgb, rgb.copy()
    finally:
        sim.command_velocity(np.zeros(6))
        sim.model.body_pos[target_id] = original_target
        mujoco.mj_forward(sim.model, sim.data)


def run_comparison(output: Path, plan: list[dict], bench: dict, comparison: dict, seed: int):
    if tuple(comparison["controllers"]) != METHODS:
        raise ValueError(f"Expected the three comparison methods: {METHODS}")
    output.mkdir(parents=True, exist_ok=False)
    (output / "traces").mkdir()
    (output / "cases").mkdir()
    manifest = {
        "schema_version": 1, "status": "running", "seed": seed, "completed_trials": 0,
        "requested_static_poses": len(plan), "requested_trials": len(plan) * len(METHODS) + len(METHODS),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark_config": bench, "comparison_config": comparison,
        "source_sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in SOURCE_FILES},
        "python": platform.python_version(), "mujoco": mujoco.__version__,
        "opencv": cv2.__version__, "numpy": np.__version__,
        "ground_truth_use": "Scene geometry only diagnoses FOV. Robot FK supplies camera pose/Jacobian. No controller receives simulator target pose or taught joint goal.",
    }
    write_json(output / "plan.json", plan)
    write_json(output / "manifest.json", manifest)
    counts = Counter()
    try:
        with Simulation() as sim:
            reference_rgb = sim.image()
            desired = sim.marker_corners(reference_rgb)
            if desired is None:
                raise RuntimeError("Reference marker not detected")
            manifest["simulation_config"] = sim.config
            manifest["desired_corners_px"] = desired.tolist()
            manifest["camera_K"] = sim.camera_intrinsics().tolist()
            Image.fromarray(reference_rgb).save(output / "reference.png")
            demo = {"trial_id": 0, "profile": "fixed_demo",
                    "offset_degrees": sim.config["ibvs"]["start_offset_degrees"]}
            manifest["target_step_demo"] = demo
            write_json(output / "manifest.json", manifest)
            runs = [(spec, None) for spec in plan] + [(demo, comparison["target_step"])]
            with (output / "trials.jsonl").open("w", encoding="utf-8") as stream:
                for spec, step in runs:
                    for method in METHODS:
                        row, trace, before, after = run_comparison_trial(
                            sim, desired, spec, bench, comparison, method, step)
                        key = f"{row['scenario']}_{method}_{spec['trial_id']:04d}"
                        row["trace"] = f"traces/{key}.npz"
                        np.savez_compressed(output / row["trace"], **trace)
                        case_key = (row["scenario"], method, row["outcome"])
                        if not counts[case_key]:
                            case = output / "cases" / key
                            case.mkdir()
                            Image.fromarray(before).save(case / "initial.png")
                            Image.fromarray(after).save(case / "terminal.png")
                            write_json(case / "trial.json", row)
                        counts[case_key] += 1
                        stream.write(json.dumps(row, allow_nan=False) + "\n")
                        stream.flush()
                        manifest["completed_trials"] += 1
                        write_json(output / "manifest.json", manifest)
                        print(f"{manifest['completed_trials']:3d}/{manifest['requested_trials']} | "
                              f"{key} | {row['outcome']} | final {row['final_error_px']}", flush=True)
        manifest["status"] = "complete"
    except BaseException as exc:
        manifest["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output / "manifest.json", manifest)
    from analyze_comparison import analyze
    return analyze(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output", type=Path, help="New directory; existing results are preserved")
    args = parser.parse_args()
    bench, comparison = read_json(BENCH_CONFIG), read_json(COMPARISON_CONFIG)
    count = bench["trials"] if args.trials is None else args.trials
    seed = bench["seed"] if args.seed is None else args.seed
    plan = sample_plan(count, seed, bench)
    name = datetime.now().strftime("%Y%m%d-%H%M%S-%f") + f"-n{count}-seed{seed}"
    output = args.output or ROOT / "results" / "comparison" / name
    run_comparison(output, plan, bench, comparison, seed)
    pointer = ROOT / "results" / "comparison" / "latest.json"
    pointer.parent.mkdir(exist_ok=True, parents=True)
    try:
        location = {"relative_to_project": output.resolve().relative_to(ROOT).as_posix()}
    except ValueError:
        location = {"absolute_path": str(output.resolve())}
    write_json(pointer, location)
    print(f"Saved comparison: {output.resolve()}")


if __name__ == "__main__":
    main()
