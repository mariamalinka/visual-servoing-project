"""Validate saved paired trials and regenerate the comparison report and figures."""
from __future__ import annotations

import argparse
import csv
import json
import os
import tempfile
from collections import Counter
from pathlib import Path

import numpy as np

from analyze_benchmark import wilson_interval
from baselines import LABELS, METHODS
from benchmark import read_json, write_json
from simulation import ROOT


def median(rows, key):
    values = [r[key] for r in rows if r[key] is not None]
    return float(np.median(values)) if values else None


def summarize(rows, confidence=.95):
    successful = [r for r in rows if r["outcome"] == "converged"]
    detected = [r for r in rows if r["initial_detected"]]
    return {
        "trials": len(rows), "initially_detected": len(detected), "successes": len(successful),
        "success_ci95_all": wilson_interval(len(successful), len(rows), confidence),
        "success_ci95_detected": wilson_interval(len(successful), len(detected), confidence),
        "median_convergence_s_successes": median(successful, "terminal_time_s"),
        "median_settling_s_successes": median(successful, "settling_time_s"),
        "median_final_error_px_successes": median(successful, "final_error_px"),
        "median_final_error_px_detected": median(detected, "final_error_px"),
        "outcomes": dict(sorted(Counter(r["outcome"] for r in rows).items())),
    }


def validate_run(directory, manifest, rows):
    if manifest["status"] != "complete" or not (
            len(rows) == manifest["completed_trials"] == manifest["requested_trials"]):
        raise ValueError("Incomplete comparison; no complete-run report may be generated")
    plan = read_json(directory / "plan.json")
    if len(plan) != manifest["requested_static_poses"]:
        raise ValueError("Plan size mismatch")
    by_id = {spec["trial_id"]: spec for spec in plan}
    if len(by_id) != len(plan) or 0 in by_id:
        raise ValueError("Duplicate or reserved trial ID")
    expected = {("static", m, spec["trial_id"]) for spec in plan for m in METHODS}
    expected |= {("target_step", m, 0) for m in METHODS}
    keys = [(r["scenario"], r["method"], r["trial_id"]) for r in rows]
    if len(keys) != len(set(keys)) or set(keys) != expected:
        raise ValueError("Missing or duplicate paired controller trials")
    config = manifest["simulation_config"]["ibvs"]
    for row in rows:
        spec = by_id[row["trial_id"]] if row["scenario"] == "static" else manifest["target_step_demo"]
        if row["offset_degrees"] != spec["offset_degrees"]:
            raise ValueError("Paired trials used different starting poses")
        with np.load(directory / row["trace"], allow_pickle=False) as trace:
            if any(len(trace[k]) != row["sample_count"] for k in trace.files):
                raise ValueError("Inconsistent trace lengths")
            if np.any(np.diff(trace["time_s"]) <= 0):
                raise ValueError("Non-increasing trace time")
            for name in ("qpos_rad", "qvel_rad_s", "command_rad_s", "camera_twist", "camera_pose_world"):
                if not np.isfinite(trace[name]).all():
                    raise ValueError("Non-finite motion data")
            if np.max(np.abs(trace["command_rad_s"])) > config["max_joint_velocity_rad_s"] + 1e-10:
                raise ValueError("Joint command exceeds shared speed limit")
            if row["scenario"] == "target_step" and not trace["target_step_applied"].any():
                raise ValueError("Target-step demonstration never applied its disturbance")
            if row["outcome"] == "converged":
                post = trace["phase"] == "post_stop"
                expected_post = int(np.ceil(manifest["benchmark_config"]["post_stop_observation_s"]
                                            * manifest["simulation_config"]["camera_hz"]))
                if (int(post.sum()) != expected_post or not np.all(trace["command_rad_s"][post] == 0)
                        or not np.all(trace["error_px"][post] < config["success_error_px"])):
                    raise ValueError("Missing stable stopped evidence")
                time = trace["time_s"]
                held = ((time >= row["terminal_time_s"] - config["success_hold_s"] + .0021)
                        & (time <= row["terminal_time_s"]))
                if not held.any() or not np.all(trace["error_px"][held] < config["success_error_px"]):
                    raise ValueError("Success lacks observed image hold")
    for spec in plan:
        paired = [r for r in rows if r["scenario"] == "static" and r["trial_id"] == spec["trial_id"]]
        if len({r["initial_detected"] for r in paired}) != 1:
            raise ValueError("Paired trials have different initial visibility")
        errors = [r["initial_error_px"] for r in paired]
        if any(e != errors[0] for e in errors):
            raise ValueError("Paired trials have different initial image errors")


def generate_figures(directory, rows, summary, manifest):
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"ibvs": "#087f8c", "pbvs": "#7b61a8", "open_loop": "#c97830"}
    with plt.rc_context({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
        fig.suptitle("Three ways to align | identical starting poses", fontsize=15, fontweight="bold")
        for i, method in enumerate(METHODS):
            group = summary["static"][method]
            n, k = group["initially_detected"], group["successes"]
            if n:
                rate = 100*k/n
                lo, hi = group["success_ci95_detected"]
                axes[0].bar(i, rate, color=colors[method])
                axes[0].errorbar(i, rate, yerr=[[max(0, rate-100*lo)], [max(0, 100*hi-rate)]],
                                 color="#333333", capsize=5, fmt="none")
                axes[0].text(i, min(110, 100*hi+3), f"{k}/{n}", ha="center")
            group_rows = [r for r in rows if r["scenario"] == "static" and r["method"] == method
                          and r["initial_detected"] and r["final_error_px"] is not None]
            errors = [max(r["final_error_px"], 1e-4) for r in group_rows]
            if errors:
                # Each dot is an actual trial; deterministic spread only avoids overlap.
                spread = np.linspace(-.18, .18, len(errors))
                axes[1].scatter(i+spread, errors, color=colors[method], alpha=.5, s=18)
                axes[1].plot([i-.22, i+.22], [np.median(errors)]*2, color="#222222", linewidth=2)
        names = ["Image-based\nIBVS", "Pose-based\nPBVS", "Look once\n+ joint feedback"]
        axes[0].set(xticks=range(3), xticklabels=names, ylim=(0, 116),
                    ylabel="Success among initially detected starts (%)",
                    title="95% Wilson intervals; invisible starts reported separately")
        axes[1].axhline(manifest["simulation_config"]["ibvs"]["success_error_px"],
                        color="#555555", linestyle="--", label="Success threshold")
        axes[1].set(xticks=range(3), xticklabels=names, yscale="log",
                    ylabel="Final RMS corner error (pixels)",
                    title="All measured detected starts, including failures")
        axes[1].legend()
        for ax in axes:
            ax.grid(axis="y", alpha=.15)
        fig.savefig(directory / "comparison.png", dpi=160)
        fig.savefig(directory / "comparison.svg")
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(9, 4.7), layout="constrained")
        for row in rows:
            if row["scenario"] != "target_step":
                continue
            with np.load(directory / row["trace"], allow_pickle=False) as trace:
                ax.plot(trace["time_s"], trace["error_px"], color=colors[row["method"]],
                        label=f"{LABELS[row['method']]}: {row['outcome']}", linewidth=2)
        ax.axvline(manifest["comparison_config"]["target_step"]["time_s"], linestyle=":",
                   color="#333333", label=f"Target shifts {1000*np.linalg.norm(manifest['comparison_config']['target_step']['translation_world_m']):g} mm")
        ax.axhline(manifest["simulation_config"]["ibvs"]["success_error_px"], linestyle="--",
                   color="#888888", label="Success threshold")
        ax.set(title="One controlled demonstration | target moves after the first image",
               xlabel="Simulated time (s)", ylabel="RMS corner error (pixels)", yscale="log")
        ax.legend(fontsize=9)
        ax.grid(alpha=.15)
        fig.savefig(directory / "target_step.png", dpi=160)
        plt.close(fig)


def number(value, digits=2):
    return "-" if value is None else f"{value:.{digits}f}"


def analyze(directory):
    directory = Path(directory).resolve()
    manifest = read_json(directory / "manifest.json")
    rows = [json.loads(line) for line in (directory / "trials.jsonl").read_text().splitlines() if line.strip()]
    validate_run(directory, manifest, rows)
    static = [r for r in rows if r["scenario"] == "static"]
    confidence = manifest["benchmark_config"]["confidence_level"]
    summary = {
        "status": "complete", "static_poses": manifest["requested_static_poses"],
        "static": {m: summarize([r for r in static if r["method"] == m], confidence) for m in METHODS},
        "target_step": [r for r in rows if r["scenario"] == "target_step"],
    }
    common_ids = set.intersection(*(
        {r["trial_id"] for r in static if r["method"] == m and r["outcome"] == "converged"} for m in METHODS))
    summary["common_successes"] = {
        "count": len(common_ids),
        "median_convergence_s": {m: median([r for r in static if r["method"] == m
                                           and r["trial_id"] in common_ids], "terminal_time_s") for m in METHODS},
    }
    write_json(directory / "summary.json", summary)
    with (directory / "trials.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    generate_figures(directory, rows, summary, manifest)
    lines = [
        "# Alignment controller comparison", "",
        f"**{len(static)} static trials: {len(static)//3} identical starting poses per controller**, seed {manifest['seed']}. "
        "Three additional target-step runs are a separate demonstration.", "",
        "![Controller comparison](comparison.png)", "",
        "| Controller | Success, all starts | Success, detected starts (95% Wilson CI) | Median convergence (s)* | Median final error (px)* | Median final error, all measured detected starts (px) |",
        "|---|---:|---|---:|---:|---:|",
    ]
    for method, group in summary["static"].items():
        k, n, d = group["successes"], group["trials"], group["initially_detected"]
        lo, hi = group["success_ci95_detected"]
        conditional = f"{k}/{d} ({100*k/d:.1f}%; {100*lo:.1f}-{100*hi:.1f}%)" if d else "No detected starts"
        lines.append(f"| {LABELS[method]} | {k}/{n} | {conditional} | "
                     f"{number(group['median_convergence_s_successes'])} | "
                     f"{number(group['median_final_error_px_successes'], 3)} | "
                     f"{number(group['median_final_error_px_detected'], 3)} |")
    lines += ["", "*Successful trials only; these groups can differ. Convergence includes the 0.5 s hold, "
              "and excludes the additional stopped second. A dash means no successful trials.", "",
              f"On the **{len(common_ids)} starts where all three succeeded**, median convergence times are: "
              + ", ".join(f"{LABELS[m]} {number(summary['common_successes']['median_convergence_s'][m])} s" for m in METHODS) + ".", "",
              "## What is compared", "",
              "- IBVS: unchanged fixed-gain controller; image-point error determines velocity.",
              "- PBVS: IPPE marker pose estimated each frame; camera translation and axis-angle error determine velocity. Image error determines the shared stop/hold criterion.",
              "- Look once: estimate the target once, freeze a world-frame camera goal, follow it using robot forward kinematics. Later images are evaluation-only; they never update its movement or stopping rule.",
              "- Open-loop position/rotation tolerances are in comparison_config.json. Completion of that pose move alone is not success: the evaluator independently checks the same image threshold and hold.", "",
              "## Fairness and limitations", "",
              "- Same reference image, marker detector, intrinsics, gain, speed limits, damped joint inverse, actuator model, 20 s horizon and initial poses. Search/recovery is disabled.",
              "- All randomized starts are retained, including initially invisible targets. The plan is generated before any trial; no visibility rejection or score-based reruns.",
              "- Success requires RMS corner error below 1 px for 0.5 s, followed by 1 s at zero command with every image still below 1 px.",
              "- Robot forward kinematics is available to open-loop motion execution. Simulator target position and the home joint goal are never supplied to controllers.",
              "- Fixed lighting, ideal kinematics and a static planar marker favor open-loop execution. These results do not establish a universally best controller or real-robot performance.",
              "- Planar PnP ambiguity and pixel discretization can affect pose-based methods. Gains are shared, not independently optimized.",
              "- Joint-space sampling is the mixture of boxes in benchmark_config.json, not a uniform distribution of camera positions.", "",
              "## Outcomes", "", "| Outcome | IBVS | PBVS | Look once |", "|---|---:|---:|---:|"]
    for outcome in sorted({r["outcome"] for r in static}):
        lines.append("| " + outcome + " | " + " | ".join(
            str(summary["static"][m]["outcomes"].get(outcome, 0)) for m in METHODS) + " |")
    lines += ["", "An open_loop_residual means the frozen pose goal was reached but the independent image success test failed. "
              "A stall is descriptive, not proof of a mathematical local minimum.", "",
              "## Target movement demonstration", "",
              "![Target movement](target_step.png)", "",
              "The target shifts 20 mm along world Y at 1 s, while each controller is aligning from the same fixed GUI Offset pose. "
              "The desired image stays the same. This is one controlled example per method, not a randomized robustness study.", "",
              "| Controller | Outcome | Final error (px) |", "|---|---|---:|"]
    for row in summary["target_step"]:
        lines.append(f"| {LABELS[row['method']]} | {row['outcome']} | {number(row['final_error_px'], 3)} |")
    lines += ["", "## Reproduction and records", "",
              "- Run all comparisons: run.cmd --compare",
              "- Regenerate the latest report without physics: run.cmd --compare-report",
              "- Custom size: python compare_controllers.py --trials 12 --seed 42",
              "- This report: python analyze_comparison.py PATH_TO_THIS_RUN",
              "- plan.json saves the exact randomized offsets. manifest.json saves source hashes, package versions, configuration and reference features.",
              "- trials.jsonl / trials.csv contain every result. traces/*.npz contain numeric frame-by-frame images' corners, commands, actual joint motion, error, camera poses and FOV diagnostics.",
              "- cases/ saves the first example of each method/scenario/outcome, including initial and final camera images.", "",
              "Equations and conventions: [Chaumette & Hutchinson (2006)](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf); "
              "[OpenCV PnP](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).", ""]
    (directory / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path)
    args = parser.parse_args()
    directory = args.directory
    if directory is None:
        pointer = read_json(ROOT / "results" / "comparison" / "latest.json")
        directory = ROOT / pointer["relative_to_project"] if "relative_to_project" in pointer else Path(pointer["absolute_path"])
    summary = analyze(directory)
    print(f"Validated and regenerated {summary['static_poses']} paired poses in {directory}")


if __name__ == "__main__":
    main()
