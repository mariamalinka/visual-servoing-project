"""Validate and regenerate the gain-scheduling experiment from saved traces."""
from __future__ import annotations

import argparse
from collections import Counter
import csv
import json
import os
from pathlib import Path
import tempfile

import numpy as np

from adaptive_gain import GainPolicy, LABELS, METHODS
from analyze_benchmark import wilson_interval
from analyze_comparison import median, number
from benchmark import OUTCOMES, read_json, write_json
from simulation import ROOT


def validate_run(directory, manifest, rows):
    if manifest["status"] != "complete" or not (
            len(rows) == manifest["requested_trials"] == manifest["completed_trials"]):
        raise ValueError("Incomplete gain study")
    plan = read_json(directory/"plan.json")
    by_id = {r["trial_id"]: r for r in plan}
    if len(by_id) != len(plan) or len(plan) != manifest["requested_poses"]:
        raise ValueError("Invalid starting-pose plan")
    expected = {(r["trial_id"], m) for r in plan for m in METHODS}
    keys = [(r["trial_id"], r["method"]) for r in rows]
    if len(keys) != len(set(keys)) or set(keys) != expected:
        raise ValueError("Missing or duplicate paired trials")
    cfg = manifest["simulation_config"]["ibvs"]
    settings = manifest["gain_config"]
    policy = GainPolicy(**settings["adaptive"])
    for row in rows:
        if row["outcome"] not in OUTCOMES or row["scenario"] != "static":
            raise ValueError("Unexpected outcome or scenario")
        if row["offset_degrees"] != by_id[row["trial_id"]]["offset_degrees"]:
            raise ValueError("Starting pose differs from plan")
        with np.load(directory/row["trace"], allow_pickle=False) as trace:
            if any(len(trace[k]) != row["sample_count"] for k in trace.files):
                raise ValueError("Trace lengths disagree")
            if not len(trace["time_s"]) or np.any(np.diff(trace["time_s"]) <= 0):
                raise ValueError("Trace time must increase")
            for key in ("qpos_rad", "qvel_rad_s", "command_rad_s", "camera_twist", "gain_per_s"):
                if not np.isfinite(trace[key]).all():
                    raise ValueError("Non-finite motion/gain data")
            if np.max(np.abs(trace["command_rad_s"])) > cfg["max_joint_velocity_rad_s"] + 1e-10:
                raise ValueError("Shared joint speed limit exceeded")
            if np.max(np.linalg.norm(trace["camera_twist"][:, :3], axis=1)) > cfg["max_linear_velocity_m_s"] + 1e-10:
                raise ValueError("Shared camera linear speed limit exceeded")
            if np.max(np.linalg.norm(trace["camera_twist"][:, 3:], axis=1)) > cfg["max_angular_velocity_rad_s"] + 1e-10:
                raise ValueError("Shared camera angular speed limit exceeded")
            control = trace["phase"] == "control"
            measured = control & np.isfinite(trace["error_px"])
            if row["method"] == "adaptive":
                expected_gain = [policy(float(e)) for e in trace["error_px"][measured]]
                # Detector features are float32; scheduling uses float64 arithmetic.
                if not np.allclose(trace["gain_per_s"][measured], expected_gain, rtol=2e-7, atol=2e-7):
                    raise ValueError("Recorded gain does not follow the saved policy")
            else:
                gain = cfg["gain_per_s"] if row["method"] == "fixed" else settings["fixed_high_gain_per_s"]
                if not np.all(trace["gain_per_s"] == gain):
                    raise ValueError("A fixed-gain trial changed its gain")
            if row["outcome"] == "converged":
                post = trace["phase"] == "post_stop"
                count = int(np.ceil(manifest["benchmark_config"]["post_stop_observation_s"]
                                    * manifest["simulation_config"]["camera_hz"]))
                if (post.sum() != count or not np.all(trace["command_rad_s"][post] == 0)
                        or not np.all(trace["error_px"][post] < cfg["success_error_px"])):
                    raise ValueError("Success has no complete stable stopped observation")
                held = (control & (trace["time_s"] >= row["terminal_time_s"] - cfg["success_hold_s"] + .0021))
                if not held.any() or not np.all(trace["error_px"][held] < cfg["success_error_px"]):
                    raise ValueError("Success has no valid image hold")
            from run_gain_study import trace_metrics
            derived = trace_metrics(trace, np.asarray(manifest["desired_corners_px"]),
                                    settings["near_goal_error_px"], cfg["success_error_px"])
            for name, value in derived.items():
                if value is None:
                    if row[name] is not None:
                        raise ValueError("Missing trace metric was reported as measured")
                elif not np.isclose(row[name], value, rtol=1e-10, atol=1e-10):
                    raise ValueError(f"Summary differs from trace: {name}")
    for trial_id in by_id:
        paired = [r for r in rows if r["trial_id"] == trial_id]
        if len({r["initial_detected"] for r in paired}) != 1 or len({r["initial_error_px"] for r in paired}) != 1:
            raise ValueError("Paired starting images differ")


def summarize(rows, confidence):
    success = [r for r in rows if r["outcome"] == "converged"]
    detected = [r for r in rows if r["initial_detected"]]
    overshoots = [r["projected_overshoot_pct"] for r in detected if r["projected_overshoot_pct"] is not None]
    growth = [r["peak_error_increase_pct"] for r in detected if r["peak_error_increase_pct"] is not None]
    return {
        "trials": len(rows), "initially_detected": len(detected), "successes": len(success),
        "success_ci95_all": wilson_interval(len(success), len(rows), confidence),
        "success_ci95_detected": wilson_interval(len(success), len(detected), confidence),
        "median_settling_s_successes": median(success, "settling_time_s"),
        "median_completion_s_successes": median(success, "terminal_time_s"),
        "median_final_error_px_successes": median(success, "final_error_px"),
        "max_projected_overshoot_pct_detected": max(overshoots) if overshoots else None,
        "max_error_growth_pct_detected": max(growth) if growth else None,
        "median_near_goal_command_rms_rad_s_successes": median(success, "near_goal_command_rms_rad_s"),
        "outcomes": dict(sorted(Counter(r["outcome"] for r in rows).items())),
    }


def paired_changes(rows):
    indexed = {(r["trial_id"], r["method"]): r for r in rows}
    result = {}
    for method in ("adaptive", "fixed_high"):
        pairs = [(r, indexed[(r["trial_id"], method)]) for r in rows
                 if r["method"] == "fixed" and r["outcome"] == "converged"
                 and indexed[(r["trial_id"], method)]["outcome"] == "converged"]
        valid = [(a, b) for a, b in pairs if a["settling_time_s"] > 0]
        result[method] = {
            "common_successes": len(pairs),
            "median_settling_change_s_vs_fixed": float(np.median(
                [b["settling_time_s"]-a["settling_time_s"] for a,b in pairs])) if pairs else None,
            "median_settling_reduction_pct_vs_fixed": float(np.median(
                [100*(a["settling_time_s"]-b["settling_time_s"])/a["settling_time_s"] for a,b in valid])) if valid else None,
        }
    return result


def generate_figures(directory, rows, manifest, summary):
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir())/"visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"fixed": "#788394", "adaptive": "#087f8c", "fixed_high": "#be7030"}
    settings = manifest["gain_config"]
    cfg = manifest["simulation_config"]["ibvs"]
    policy = GainPolicy(**settings["adaptive"])
    common = set.intersection(*(
        {r["trial_id"] for r in rows if r["method"] == m and r["outcome"] == "converged"} for m in METHODS))
    with plt.rc_context({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
        fig.suptitle("Adaptive gain | speed and correction strength", fontsize=15, fontweight="bold")
        for i,m in enumerate(METHODS):
            values = [r["settling_time_s"] for r in rows if r["method"] == m and r["trial_id"] in common]
            if values:
                axes[0].scatter(i+np.linspace(-.18,.18,len(values)), values, s=18, alpha=.5, color=colors[m])
                axes[0].plot([i-.23,i+.23], [np.median(values)]*2, color="#222222", linewidth=2)
        axes[0].set(xticks=range(3), xticklabels=[LABELS[m] for m in METHODS],
                    ylabel="Settling time (simulated seconds)",
                    title=f"Same {len(common)} successful starts | black lines: medians")
        errors = np.linspace(0, 60, 300)
        axes[1].plot(errors, [policy(float(e)) for e in errors], color=colors["adaptive"], lw=2, label="Adaptive")
        axes[1].axhline(cfg["gain_per_s"], color=colors["fixed"], linestyle="--", label=f"Fixed {cfg['gain_per_s']:g}")
        axes[1].axhline(settings["fixed_high_gain_per_s"], color=colors["fixed_high"], linestyle="--",
                        label=f"Fixed high {settings['fixed_high_gain_per_s']:g}")
        axes[1].set(xlabel="RMS corner error (pixels)", ylabel="Gain (1/s)",
                    title="Gain increases smoothly with image error", ylim=(0, settings["fixed_high_gain_per_s"]*1.2))
        axes[1].legend()
        for ax in axes:
            ax.grid(alpha=.15)
        fig.savefig(directory/"gain_comparison.png", dpi=160)
        fig.savefig(directory/"gain_comparison.svg")
        plt.close(fig)
        detected_ids = sorted({r["trial_id"] for r in rows if r["initial_detected"]})
        if detected_ids:
            example = detected_ids[0]
            fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), layout="constrained")
            fig.suptitle(f"First detected starting pose: trial {example} | no best-case selection", fontsize=14)
            for row in rows:
                if row["trial_id"] != example:
                    continue
                with np.load(directory/row["trace"], allow_pickle=False) as trace:
                    axes[0].plot(trace["time_s"], trace["error_px"], color=colors[row["method"]], label=LABELS[row["method"]])
                    control = trace["phase"] == "control"
                    axes[1].plot(trace["time_s"][control], trace["gain_per_s"][control],
                                 color=colors[row["method"]], label=LABELS[row["method"]])
            axes[0].axhline(cfg["success_error_px"], color="#333333", linestyle=":", label="Success threshold")
            axes[0].set(yscale="log", xlabel="Simulated time (s)", ylabel="RMS corner error (pixels)",
                        title="Error decay, including stopped observation")
            axes[1].set(xlabel="Simulated time (s)", ylabel="Configured gain (1/s)",
                        title="Adaptive gain falls as alignment improves")
            for ax in axes:
                ax.legend()
                ax.grid(alpha=.15)
            fig.savefig(directory/"gain_trace.png", dpi=160)
            plt.close(fig)


def analyze(directory):
    directory = Path(directory).resolve()
    manifest = read_json(directory/"manifest.json")
    rows = [json.loads(s) for s in (directory/"trials.jsonl").read_text().splitlines() if s.strip()]
    validate_run(directory, manifest, rows)
    summary = {
        "status": "complete", "poses": manifest["requested_poses"],
        "methods": {m: summarize([r for r in rows if r["method"] == m],
                                manifest["benchmark_config"]["confidence_level"]) for m in METHODS},
        "paired": paired_changes(rows),
    }
    write_json(directory/"summary.json", summary)
    with (directory/"trials.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    generate_figures(directory, rows, manifest, summary)
    cfg = manifest["simulation_config"]["ibvs"]
    gain = manifest["gain_config"]["adaptive"]
    near = manifest["gain_config"]["near_goal_error_px"]
    lines = [
        "# Adaptive-gain IBVS study", "",
        f"**{len(rows)} trials: {summary['poses']} identical starting poses per gain policy**, seed {manifest['seed']}. Search/recovery disabled.", "",
        "![Gain comparison](gain_comparison.png)", "",
        "| Policy | Success/all starts | Success/detected starts (95% Wilson CI) | Median settling (s)* | Median completion (s)* | Median final error (px)* |",
        "|---|---:|---|---:|---:|---:|",
    ]
    for m,g in summary["methods"].items():
        k,d,n = g["successes"],g["initially_detected"],g["trials"]
        lo,hi = g["success_ci95_detected"]
        rate = f"{k}/{d} ({100*k/d:.1f}%; {100*lo:.1f}-{100*hi:.1f}%)" if d else "No detected starts"
        lines.append(f"| {LABELS[m]} | {k}/{n} | {rate} | {number(g['median_settling_s_successes'])} | "
                     f"{number(g['median_completion_s_successes'])} | {number(g['median_final_error_px_successes'],3)} |")
    lines += ["", "*Successful trials only. Settling is the start of the final sustained below-threshold streak. Completion includes the hold window; stopped observation adds another second.", "",
              "## Paired timing changes", "",
              "| Policy versus fixed baseline | Common successes | Median settling change (s; negative is faster) | Median per-trial settling reduction |",
              "|---|---:|---:|---:|"]
    for m,p in summary["paired"].items():
        lines.append(f"| {LABELS[m]} | {p['common_successes']} | {number(p['median_settling_change_s_vs_fixed'])} | "
                     f"{number(p['median_settling_reduction_pct_vs_fixed'],1)}% |")
    lines += ["", "## Overshoot and commands near the goal", "",
              "| Policy | Maximum projected overshoot (%) | Maximum RMS error growth above initial (%) | Median near-goal joint-command RMS (rad/s)* |",
              "|---|---:|---:|---:|"]
    for m,g in summary["methods"].items():
        lines.append(f"| {LABELS[m]} | {number(g['max_projected_overshoot_pct_detected'],3)} | "
                     f"{number(g['max_error_growth_pct_detected'],3)} | "
                     f"{number(g['median_near_goal_command_rms_rad_s_successes'],5)} |")
    lines += ["",
              "Projected overshoot measures how far the feature-error vector crosses the goal along its initial direction, divided by the initial error magnitude. It is distinct from RMS error growth and does not detect every individual corner crossing.",
              f"*Near-goal command RMS is measured during control frames with error between {cfg['success_error_px']:g} and {near:g} px, successful trials only. It describes requested joint speeds, not a general smoothness guarantee.", ""]
    maxima = [g["max_projected_overshoot_pct_detected"] for g in summary["methods"].values()]
    if all(v is not None and v < 1e-8 for v in maxima):
        lines += ["**No projected overshoot was observed with any policy. This experiment therefore demonstrates no overshoot reduction from adaptive gain.**", ""]
    lines += ["## Declared policy and common conditions", "",
              f"lambda(e) = {gain['near_gain_per_s']:g} + "
              f"({gain['far_gain_per_s']:g} - {gain['near_gain_per_s']:g}) * (1 - exp(-e/{gain['error_scale_px']:g})), "
              "where e is RMS image error in pixels. This is an error-based gain schedule; it does not train a model.",
              f"Fixed baseline gain: {cfg['gain_per_s']:g}/s. Fixed-high gain: {manifest['gain_config']['fixed_high_gain_per_s']:g}/s.",
              "The high fixed gain is included to distinguish adaptation from simply turning up the proportional gain.",
              "Every method uses the existing IBVS equations, the same camera/Jacobian, actuator model, speed limits, reference and exact starting offsets.",
              f"Success requires error below {cfg['success_error_px']:g} px for {cfg['success_hold_s']:g} s, "
              f"then a further {manifest['benchmark_config']['post_stop_observation_s']:g} s stopped below threshold. Timeout: {cfg['timeout_s']:g} s.",
              "Initially invisible starts are retained. Parameters were frozen before the run; no gains were selected by optimizing these results.",
              "Fixed lighting, ideal dynamics and no sensor latency make this a limited gain study. A faster high fixed gain here is not evidence of robustness to noise or delay. The GUI starts in the existing fixed mode.", "",
              "## Example trajectory", ""]
    if any(r["initial_detected"] for r in rows):
        lines += ["![Error and scheduled gain](gain_trace.png)", ""]
    lines += ["## Outcomes", "", "| Outcome | Fixed | Adaptive | Fixed high |", "|---|---:|---:|---:|"]
    for outcome in sorted({r["outcome"] for r in rows}):
        lines.append("| "+outcome+" | "+" | ".join(str(summary["methods"][m]["outcomes"].get(outcome,0)) for m in METHODS)+" |")
    lines += ["", "## Reproduction", "",
              "- run.cmd --gain-study: create a fresh study.",
              "- run.cmd --gain-report: regenerate the latest report without physics.",
              "- python run_gain_study.py --trials 12 --seed 42: custom smaller study.",
              "- python analyze_gain_study.py PATH_TO_RUN: analyze a particular saved run.",
              "- plan.json and manifest.json: exact starting offsets, configurations, versions and source hashes.",
              "- traces/*.npz: numeric per-frame image error, features, gain, commands, measured joints, camera pose and FOV diagnostics.",
              "- trials.jsonl / trials.csv and summary.json: individual metrics and aggregate results.", "",
              "Background on the proportional IBVS law: [Inria visual-servoing course](https://vdrevell.gitlabpages.inria.fr/istic-robm/asservissement-visuel.html). "
              "The increasing-with-error gain schedule here is our explicit design choice, not a claim that all adaptive visual-servoing methods use that schedule.", ""]
    (directory/"REPORT.md").write_text("\n".join(lines),encoding="utf-8")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path)
    args = parser.parse_args()
    directory = args.directory
    if directory is None:
        pointer = read_json(ROOT/"results"/"gain"/"latest.json")
        directory = ROOT/pointer["relative_to_project"] if "relative_to_project" in pointer else Path(pointer["absolute_path"])
    summary = analyze(directory)
    print(f"Validated and regenerated gain study: {summary['poses']} paired poses in {directory}")


if __name__ == "__main__":
    main()
