"""Compare fixed, adaptive and fixed-high IBVS gain on identical starting poses."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

from adaptive_gain import METHODS, load_gain_config, make_gain_controller
from benchmark import BENCH_CONFIG, read_json, sample_plan, write_json
from compare_controllers import run_comparison_trial
from simulation import ROOT, Simulation

SOURCE_FILES = ("adaptive_gain.py", "gain_config.json", "control.py", "simulation.py",
                "scene.xml", "config.json", "benchmark.py", "benchmark_config.json",
                "baselines.py", "compare_controllers.py", "run_gain_study.py",
                "analyze_gain_study.py")


def trace_metrics(trace: dict, desired: np.ndarray, near_goal_px: float,
                  success_px: float) -> dict:
    """Overshoot is signed projection past the goal along the initial error.

    It is NOT the peak of the nonnegative RMS error. Different feature components
    can cross their goals without this aggregate projected overshoot being > 0.
    """
    vectors = (trace["corners_px"] - desired).reshape(-1, 8)
    initial = vectors[0]
    denominator = float(initial @ initial)
    finite = np.isfinite(vectors).all(axis=1)
    projected_overshoot = None
    if np.isfinite(denominator) and denominator > 1e-12 and finite.any():
        signed = vectors[finite] @ initial / denominator
        projected_overshoot = float(100 * max(0., -np.min(signed)))
    near = ((trace["phase"] == "control") & (trace["error_px"] >= success_px)
            & (trace["error_px"] <= near_goal_px))
    near_rms = (float(np.sqrt(np.mean(trace["command_rad_s"][near]**2)))
                if near.any() else None)
    return {"projected_overshoot_pct": projected_overshoot,
            "near_goal_command_rms_rad_s": near_rms,
            "min_gain_per_s": float(np.min(trace["gain_per_s"])),
            "max_gain_per_s": float(np.max(trace["gain_per_s"]))}


def run_study(output: Path, plan: list[dict], bench: dict, settings: dict, seed: int):
    output.mkdir(parents=True, exist_ok=False)
    (output / "traces").mkdir()
    (output / "cases").mkdir()
    manifest = {
        "schema_version": 1, "status": "running", "seed": seed,
        "requested_poses": len(plan), "requested_trials": len(plan)*len(METHODS),
        "completed_trials": 0, "methods": list(METHODS),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "benchmark_config": bench, "gain_config": settings,
        "source_sha256": {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCE_FILES},
        "python": platform.python_version(), "mujoco": mujoco.__version__,
        "opencv": cv2.__version__, "numpy": np.__version__,
        "ground_truth_use": "Controller receives detected pixels, estimated depth and robot Jacobian. Scene geometry diagnoses FOV only.",
        "selection": "Gain parameters declared before evaluation; no tuning on the 200-pose results.",
    }
    write_json(output/"plan.json", plan)
    write_json(output/"manifest.json", manifest)
    counts = Counter()
    try:
        with Simulation() as sim:
            image = sim.image()
            desired = sim.marker_corners(image)
            if desired is None:
                raise RuntimeError("Reference marker not detected")
            manifest["simulation_config"] = sim.config
            manifest["camera_K"] = sim.camera_intrinsics().tolist()
            manifest["desired_corners_px"] = desired.tolist()
            Image.fromarray(image).save(output/"reference.png")
            write_json(output/"manifest.json", manifest)
            with (output/"trials.jsonl").open("w", encoding="utf-8") as stream:
                for spec in plan:
                    for method in METHODS:
                        def factory(reference, K, config):
                            return make_gain_controller(method, reference, K, config, settings)
                        row, trace, before, after = run_comparison_trial(
                            sim, desired, spec, bench, {}, method, controller_factory=factory)
                        row.update(trace_metrics(trace, desired, settings["near_goal_error_px"],
                                                 sim.config["ibvs"]["success_error_px"]))
                        row["trace"] = f"traces/{method}_{spec['trial_id']:04d}.npz"
                        np.savez_compressed(output/row["trace"], **trace)
                        key = (method, row["outcome"])
                        if not counts[key]:
                            case = output/"cases"/f"{method}_{spec['trial_id']:04d}"
                            case.mkdir()
                            Image.fromarray(before).save(case/"initial.png")
                            Image.fromarray(after).save(case/"terminal.png")
                            write_json(case/"trial.json", row)
                        counts[key] += 1
                        stream.write(json.dumps(row, allow_nan=False)+"\n")
                        stream.flush()
                        manifest["completed_trials"] += 1
                        write_json(output/"manifest.json", manifest)
                        print(f"{manifest['completed_trials']:3d}/{manifest['requested_trials']} | "
                              f"{method:10s} {spec['trial_id']:03d} | {row['outcome']} | "
                              f"t={row['terminal_time_s']:.2f}s | error={row['final_error_px']}", flush=True)
        manifest["status"] = "complete"
    except BaseException as exc:
        manifest["status"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        write_json(output/"manifest.json", manifest)
    from analyze_gain_study import analyze
    return analyze(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output", type=Path, help="New directory; old runs are preserved")
    args = parser.parse_args()
    bench, settings = read_json(BENCH_CONFIG), load_gain_config()
    count = bench["trials"] if args.trials is None else args.trials
    seed = bench["seed"] if args.seed is None else args.seed
    plan = sample_plan(count, seed, bench)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    output = args.output or ROOT/"results"/"gain"/f"{stamp}-n{count}-seed{seed}"
    run_study(output, plan, bench, settings, seed)
    pointer = ROOT/"results"/"gain"/"latest.json"
    pointer.parent.mkdir(parents=True, exist_ok=True)
    try:
        location = {"relative_to_project": output.resolve().relative_to(ROOT).as_posix()}
    except ValueError:
        location = {"absolute_path": str(output.resolve())}
    write_json(pointer, location)
    print(f"Saved gain study: {output.resolve()}")


if __name__ == "__main__":
    main()
