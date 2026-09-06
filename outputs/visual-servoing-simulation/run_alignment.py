"""Run one reproducible IBVS trial and save measurements, plots and a short GIF."""
from __future__ import annotations

import csv
import json
import os
import platform
import tempfile
from pathlib import Path

import cv2
import mujoco
import numpy as np
from PIL import Image

from app import AMBER, BG, TEAL, label
from control import IBVSController
from simulation import ROOT, Simulation


def plot_trace(directory: Path, rows: list[dict], reference: np.ndarray, threshold: float) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    valid = [row for row in rows if row["error_px"] is not None]
    times = np.array([row["time_s"] for row in valid])
    errors = np.array([row["error_px"] for row in valid])
    with plt.rc_context({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")
        fig.suptitle("Visual servoing | first closed-loop trial", fontsize=15, fontweight="bold")
        axes[0].semilogy(times, np.maximum(errors, 1e-6), color="#087f8c", linewidth=2,
                         label="Measured corner error")
        axes[0].axhline(threshold, linestyle="--", color="#ba6b18", label=f"Success threshold: {threshold:g} px")
        axes[0].set(xlabel="Simulation time (s)", ylabel="RMS corner error (pixels)",
                    title="Image error decreases under feedback")
        axes[0].grid(True, which="both", alpha=0.18)
        axes[0].legend(loc="upper right", fontsize=8)
        colors = ["#087f8c", "#ad4b70", "#7356a4", "#ba6b18"]
        for i in range(4):
            u = [row[f"u{i}"] for row in valid]
            v = [row[f"v{i}"] for row in valid]
            axes[1].plot(u, v, color=colors[i], linewidth=2, label=f"Corner {i}")
            axes[1].scatter(u[0], v[0], color=colors[i], s=28)
            axes[1].scatter(reference[i, 0], reference[i, 1], marker="x", color="black", s=65)
        closed_reference = np.vstack((reference, reference[0]))
        axes[1].plot(closed_reference[:, 0], closed_reference[:, 1], "--", color="gray", alpha=0.45)
        axes[1].set(xlabel="u (pixels)", ylabel="v (pixels)",
                    title="Feature paths | dots: start, crosses: goal", aspect="equal")
        axes[1].invert_yaxis()
        axes[1].grid(True, alpha=0.18)
        axes[1].legend(loc="center", fontsize=8, ncols=2)
        fig.savefig(directory / "convergence.png", dpi=160)
        fig.savefig(directory / "convergence.svg")
        plt.close(fig)


def movie_frame(sim: Simulation, rgb: np.ndarray, corners: np.ndarray | None,
                 reference: np.ndarray, error: float | None, status: str) -> Image.Image:
    annotated = rgb.copy()
    cv2.polylines(annotated, [reference.astype(np.int32)], True, AMBER, 2)
    if corners is not None:
        cv2.polylines(annotated, [corners.astype(np.int32)], True, TEAL, 2)
    canvas = np.full((438, 960, 3), BG, dtype=np.uint8)
    label(canvas, "VISUAL SERVOING / AUTOMATIC ALIGNMENT", 18, 28, 0.6, TEAL)
    label(canvas, f"t = {sim.data.time:.2f} s   |   {status}", 18, 54, 0.48)
    if error is not None:
        label(canvas, f"RMS error: {error:.2f} px", 665, 54, 0.48)
    canvas[66:426, :480] = cv2.resize(sim.image("world"), (480, 360))
    canvas[66:426, 480:] = cv2.resize(annotated, (480, 360))
    return Image.fromarray(canvas)


def main() -> None:
    directory = ROOT / "results" / "alignment"
    directory.mkdir(parents=True, exist_ok=True)
    with Simulation() as sim:
        reference_rgb = sim.image()
        reference = sim.marker_corners(reference_rgb)
        if reference is None:
            raise RuntimeError("Reference marker was not detected")
        Image.fromarray(reference_rgb).save(directory / "reference.png")
        controller = IBVSController(reference, sim.camera_intrinsics(), sim.config)
        offset = sim.config["ibvs"]["start_offset_degrees"]
        sim.reset(offset)
        initial_rgb = sim.image()
        Image.fromarray(initial_rgb).save(directory / "initial.png")
        rows, frames = [], []
        dt = 1 / sim.config["camera_hz"]
        for tick in range(int(sim.config["ibvs"]["timeout_s"] / dt) + 2):
            rgb = sim.image()
            corners = sim.marker_corners(rgb)
            sample = controller.update(corners, sim.camera_jacobian(), dt)
            row = {"time_s": float(sim.data.time), "status": sample.status,
                   "error_px": sample.error_px, "joint_condition": sample.joint_condition}
            for i in range(6):
                row[f"q{i+1}_rad"] = float(sim.data.qpos[i])
                row[f"qdot{i+1}_rad_s"] = float(sample.velocity[i])
                row[f"camera_twist_{i}"] = float(sample.camera_twist[i])
            for i in range(4):
                row[f"u{i}"] = None if corners is None else float(corners[i, 0])
                row[f"v{i}"] = None if corners is None else float(corners[i, 1])
                row[f"depth{i}_m"] = None if sample.depths is None else float(sample.depths[i])
            rows.append(row)
            if tick % 3 == 0 or sample.status != "running":
                frames.append(movie_frame(sim, rgb, corners, reference, sample.error_px, sample.status))
            if tick % 30 == 0 or sample.status != "running":
                print(f"{sim.data.time:5.2f} s | {sample.status:14s} | error {sample.error_px}", flush=True)
            sim.command_velocity(sample.velocity)
            if sample.status != "running":
                break
            sim.advance(dt)
        # Record actual stopped behavior for a further second; no new alignment.
        sim.command_velocity(np.zeros(6))
        sim.advance(1.0)
        final_rgb = sim.image()
        final_corners = sim.marker_corners(final_rgb)
        final_error = (None if final_corners is None else
                       float(np.sqrt(np.mean(np.sum((final_corners - reference)**2, axis=1)))))
        Image.fromarray(final_rgb).save(directory / "final.png")
        frame = movie_frame(sim, final_rgb, final_corners, reference, final_error, sample.status)
        frames.extend([frame.copy() for _ in range(12)])
        frames[0].save(directory / "alignment.gif", save_all=True, append_images=frames[1:],
                       duration=100, loop=0, optimize=True)
        with (directory / "trace.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        summary = {
            "status": sample.status,
            "scope": "One fixed-offset marker IBVS trial; not the randomized benchmark",
            "initial_error_px": rows[0]["error_px"],
            "terminal_error_px": sample.error_px,
            "error_px_after_1s_stopped": final_error,
            "terminal_time_s": rows[-1]["time_s"],
            "observed_hold_s": controller.held_seconds,
            "error_definition": "sqrt(mean over four corners of squared Euclidean pixel distance)",
            "initial_offset_degrees": offset,
            "desired_corners_px": reference.tolist(),
            "camera_K": sim.camera_intrinsics().tolist(),
            "config": sim.config,
            "python": platform.python_version(), "mujoco": mujoco.__version__, "opencv": cv2.__version__,
            "control_uses_target_ground_truth": False
        }
        (directory / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
        plot_trace(directory, rows, reference, sim.config["ibvs"]["success_error_px"])
        print(json.dumps(summary, indent=2))
        if sample.status != "converged" or final_error is None or final_error >= sim.config["ibvs"]["success_error_px"]:
            raise SystemExit("The trial did not establish stable alignment; inspect results/alignment.")


if __name__ == "__main__":
    main()
