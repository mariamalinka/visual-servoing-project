"""Regenerate benchmark statistics, tables and figures from saved trial logs."""
from __future__ import annotations

import argparse
import csv
import json
import os
import tempfile
from pathlib import Path
from statistics import NormalDist

import numpy as np

from benchmark import OUTCOMES, ROOT, read_json, write_json
import binomial_ci

LABELS = {
    "converged": "Converged", "initial_out_of_view": "Out of view at start",
    "initial_tracking_loss": "Not detected at start", "fov_loss": "Left camera view",
    "tracking_loss": "Tracking lost in frame", "invalid_depth": "Depth rejected",
    "joint_limit_stall": "Stalled at joint limit", "singularity_stall": "Stalled near singularity",
    "stalled": "Stalled", "timeout": "Timeout", "unstable_after_stop": "Unstable after stopping",
}


def wilson_interval(successes: int, total: int, confidence: float = .95):
    """Two-sided Wilson score interval; no-data denominators stay undefined.

    https://www.itl.nist.gov/div898/handbook/prc/section2/prc241.htm
    """
    if total < 0 or successes < 0 or successes > total or not 0 < confidence < 1:
        raise ValueError("Invalid binomial counts or confidence")
    if total == 0:
        return None, None
    z = NormalDist().inv_cdf((1 + confidence) / 2)
    p = successes / total
    denominator = 1 + z*z / total
    center = (p + z*z / (2*total)) / denominator
    half = z * np.sqrt(p*(1-p)/total + z*z/(4*total*total)) / denominator
    return float(max(0, center-half)), float(min(1, center+half))


def success_interval(successes: int, total: int, confidence: float = .95):
    """The project's interval for every success rate: exact Clopper-Pearson (binomial_ci.py).

    wilson_interval above is kept for code that imports it; reports use this one.
    """
    return binomial_ci.clopper_pearson(successes, total, confidence)


def summarize(rows: list[dict], confidence: float) -> dict:
    n = len(rows)
    successes = [r for r in rows if r["outcome"] == "converged"]
    detected = [r for r in rows if r["initial_detected"]]
    if any(not r["initial_detected"] for r in successes):
        raise ValueError("A successful trial must start with detected features")
    k = len(successes)
    return {
        "trials": n, "initially_detected": len(detected), "successes": k,
        "success_rate_all": k/n if n else None,
        "success_ci95_all": success_interval(k, n, confidence),
        "success_rate_initially_detected": k/len(detected) if detected else None,
        "success_ci95_initially_detected": success_interval(k, len(detected), confidence),
        "median_convergence_time_s_successes": float(np.median([r["terminal_time_s"] for r in successes])) if k else None,
        "median_settling_time_s_successes": float(np.median([r["settling_time_s"] for r in successes])) if k else None,
        "median_final_error_px_successes": float(np.median([r["final_error_px"] for r in successes])) if k else None,
        "outcomes": {name: sum(r["outcome"] == name for r in rows) for name in OUTCOMES},
    }


def validate_run(directory: Path, manifest: dict, rows: list[dict]) -> None:
    expected = manifest["requested_trials"]
    if manifest["status"] != "complete" or len(rows) != expected or manifest["completed_trials"] != expected:
        raise ValueError("Run is incomplete; saved logs are preserved but no complete-run report will be generated")
    plan = read_json(directory / "plan.json")
    expected_ids = [r["trial_id"] for r in plan]
    ids = [r["trial_id"] for r in rows]
    if len(set(ids)) != len(ids) or set(ids) != set(expected_ids):
        raise ValueError("Missing or duplicate trial IDs")
    for row in rows:
        if row["outcome"] not in OUTCOMES:
            raise ValueError("Unknown trial outcome")
        with np.load(directory / "traces" / f"trial_{row['trial_id']:04d}.npz", allow_pickle=False) as trace:
            if any(len(trace[key]) != row["sample_count"] for key in trace.files):
                raise ValueError(f"Inconsistent trace arrays in trial {row['trial_id']}")
            if len(trace["time_s"]) > 1 and np.any(np.diff(trace["time_s"]) <= 0):
                raise ValueError("Trace time must increase")
            if not np.isfinite(trace["qpos_rad"]).all() or not np.isfinite(trace["command_rad_s"]).all():
                raise ValueError("Non-finite robot data")
            if row["outcome"] == "converged":
                threshold = manifest["simulation_config"]["ibvs"]["success_error_px"]
                observed = trace["phase"] == "post_stop"
                if not observed.any() or not np.all(trace["error_px"][observed] < threshold):
                    raise ValueError("Successful trial lacks stable post-stop evidence")


def generate_figures(directory: Path, rows: list[dict], manifest: dict, summary: dict) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "visual-servoing-matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    profiles = list(manifest["benchmark_config"]["profiles_degrees"])
    with plt.rc_context({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False}):
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 5.2), layout="constrained")
        fig.suptitle(f"Visual servoing | {len(rows)} randomized starting poses", fontsize=15, fontweight="bold")
        colors = ("#087f8c", "#d08b25")
        labeled_series = set()
        for i, name in enumerate(profiles):
            group = summary["profiles"][name]
            for j, (rate_key, ci_key, denom) in enumerate((
                ("success_rate_all", "success_ci95_all", group["trials"]),
                ("success_rate_initially_detected", "success_ci95_initially_detected", group["initially_detected"]),
            )):
                rate = group[rate_key]
                if rate is None:
                    continue
                lo, hi = group[ci_key]
                position = i + (j-.5)*.36
                axes[0].bar(position, 100*rate, width=.32, color=colors[j],
                             label=("All sampled starts", "Initially detected starts")[j] if j not in labeled_series else None)
                labeled_series.add(j)
                axes[0].errorbar(position, 100*rate, yerr=[[max(0, 100*(rate-lo))], [max(0, 100*(hi-rate))]],
                                 fmt="none", color="#303030", capsize=4, linewidth=1)
                axes[0].text(position, min(110, 100*hi+2.0), f"{group['successes']}/{denom}",
                              ha="center", fontsize=9)
        axes[0].set(xticks=range(len(profiles)), xticklabels=profiles, ylim=(0, 116),
                     ylabel="Successful alignment (%)", xlabel="Joint-offset sampling profile",
                     title="Success rates with 95% Clopper-Pearson intervals")
        axes[0].set_yticks([0, 25, 50, 75, 100])
        axes[0].legend(loc="lower left", fontsize=9)
        axes[0].grid(axis="y", alpha=.15)
        counts = summary["overall"]["outcomes"]
        present = [name for name in OUTCOMES if counts[name]]
        bars = axes[1].barh([LABELS[name] for name in present], [counts[name] for name in present],
                            color=["#087f8c" if name == "converged" else "#ad6161" for name in present])
        axes[1].bar_label(bars, padding=4)
        axes[1].invert_yaxis()
        axes[1].set(xlabel="Number of trials", title="Every sampled start is counted",
                     xlim=(0, max(counts.values())*1.18+1))
        axes[1].grid(axis="x", alpha=.15)
        fig.savefig(directory / "benchmark_overview.png", dpi=160)
        fig.savefig(directory / "benchmark_overview.svg")
        plt.close(fig)

        fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.8), layout="constrained")
        fig.suptitle("Alignment performance | detected starts", fontsize=15, fontweight="bold")
        successes = [r for r in rows if r["outcome"] == "converged"]
        if successes:
            times = [r["terminal_time_s"] for r in successes]
            axes[0].hist(times, bins=min(15, max(3, int(np.sqrt(len(times))))), color="#087f8c")
            median = float(np.median(times))
            axes[0].axvline(median, color="#9e571d", linestyle="--", label=f"Median {median:.2f} s")
            axes[0].legend()
        else:
            axes[0].text(.5, .5, "No successful trials", transform=axes[0].transAxes, ha="center")
        axes[0].set(xlabel="Time to declared convergence (simulated seconds)",
                     ylabel="Successful trials", title="Includes the 0.5 s hold window")
        for success, color, marker in ((True, "#087f8c", "o"), (False, "#ad6161", "x")):
            group = [r for r in rows if (r["outcome"] == "converged") == success and
                     r["initial_error_px"] is not None and r["final_error_px"] is not None]
            if group:
                axes[1].scatter([r["initial_error_px"] for r in group],
                                 [max(r["final_error_px"], 1e-4) for r in group],
                                 color=color, marker=marker, s=25, alpha=.75,
                                 label="Converged" if success else "Did not converge")
        axes[1].axhline(manifest["simulation_config"]["ibvs"]["success_error_px"],
                        color="#9e571d", linestyle="--", linewidth=1, label="1 px threshold")
        axes[1].set(xlabel="Initial RMS corner error (pixels)", ylabel="Final RMS corner error (pixels)",
                     yscale="log", title="Trials with measurable initial and final error")
        axes[1].legend(fontsize=8)
        for ax in axes:
            ax.grid(alpha=.15)
        fig.savefig(directory / "alignment_performance.png", dpi=160)
        plt.close(fig)


def format_rate(group: dict, conditional: bool = False) -> str:
    key = "initially_detected" if conditional else "all"
    p = group[f"success_rate_{key}"]
    lo, hi = group[f"success_ci95_{key}"]
    return "No data" if p is None else f"{100*p:.1f}% [{100*lo:.1f}, {100*hi:.1f}]"


def analyze(directory: Path) -> dict:
    directory = directory.resolve()
    manifest = read_json(directory / "manifest.json")
    rows = [json.loads(line) for line in (directory / "trials.jsonl").read_text().splitlines() if line.strip()]
    validate_run(directory, manifest, rows)
    confidence = manifest["benchmark_config"]["confidence_level"]
    summary = {"status": "complete", "confidence_level": confidence,
               "interval_method": "Clopper-Pearson (exact), two-sided",
               "interval_note": "Summaries written before this change used the Wilson score interval.",
               "overall": summarize(rows, confidence),
               "profiles": {name: summarize([r for r in rows if r["profile"] == name], confidence)
                            for name in manifest["benchmark_config"]["profiles_degrees"]},
               "definitions": {
                   "success": "Controller holds error <1 px for 0.5 s and all samples during 1 further stopped second also stay <1 px.",
                   "conditional_rate": "Successes divided by starts where OpenCV detected the marker; invisible starts remain in the all-start denominator.",
                   "settling_time": "Start of the final sustained below-threshold streak, successful trials only.",
                   "convergence_time": "Time when the controller declares success, including the hold window but excluding the additional stopped observation.",
                   "stall": "Error span <=0.15 px and all actual joint speeds <0.01 rad/s over 3 s; descriptive, not proof of a local minimum.",
                   "missing_data": "Missing image measurements are null in JSON and NaN in numeric NPZ traces.",
               }}
    write_json(directory / "summary.json", summary)
    with (directory / "trials.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows({key: json.dumps(value) if isinstance(value, list) else value
                          for key, value in row.items()} for row in rows)
    generate_figures(directory, rows, manifest, summary)
    overall = summary["overall"]
    lines = [
        "# Randomized visual-servoing benchmark", "",
        f"Completed **{len(rows)} trials**, RNG seed **{manifest['seed']}**. Controller: unchanged step-2 fixed-gain marker IBVS.", "",
        f"- All starts: **{overall['successes']}/{overall['trials']}** converged; {format_rate(overall)}.",
        f"- Starts with the marker initially detected: **{overall['successes']}/{overall['initially_detected']}**; {format_rate(overall, True)}.",
        "- Brackets show 95% Clopper-Pearson (exact) confidence intervals for the declared starting-pose distribution. "
        "A 100% observed rate is not proof of 100% reliability; the lower bound is what the data supports.", "",
        "![Benchmark overview](benchmark_overview.png)", "",
        "| Profile | Sampled starts | Marker detected at start | Converged | Success, all starts (95% CI) | Success, detected starts (95% CI) |",
        "|---|---:|---:|---:|---|---|",
    ]
    for name, group in summary["profiles"].items():
        lines.append(f"| {name} | {group['trials']} | {group['initially_detected']} | {group['successes']} | {format_rate(group)} | {format_rate(group, True)} |")
    lines += ["", "## Sampling", "", manifest["benchmark_config"]["sampling"], "",
              "Bounds below are symmetric offsets about home, in joint degrees (J1 through J6).", "",
              "| Profile | J1 | J2 | J3 | J4 | J5 | J6 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for name, bounds in manifest["benchmark_config"]["profiles_degrees"].items():
        lines.append("| " + name + " | " + " | ".join(f"+/-{value:g}" for value in bounds) + " |")
    lines += ["", "The plan is saved before simulation. No starts are rejected for visibility and no failed trials are rerun to improve the score.",
              "Joint-space sampling induces a nonuniform camera-pose distribution. These intervals describe this simulation experiment, not physical-robot reliability.",
              "", "## Outcomes", "", "| Outcome | Count | First recorded example |", "|---|---:|---|"]
    for name, count in overall["outcomes"].items():
        example = f"[Images and trial data](cases/{name}/)" if count else "-"
        lines.append(f"| {LABELS[name]} | {count} | {example} |")
    lines += ["", "Examples are the first occurrence of each outcome, not selected for best or worst performance.",
              "Initial loss is distinguished from loss during motion. FOV labels use evaluation-only geometric projection; the controller never receives this ground truth.",
              "A stalled trial is not labeled a proven local minimum. Joint-limit and singularity labels indicate observed conditions during the stall.",
              "", "## Timing and accuracy", "", "![Performance](alignment_performance.png)", ""]
    if overall["successes"]:
        lines += [f"- Median declared-convergence time, successes only: **{overall['median_convergence_time_s_successes']:.2f} simulated seconds**.",
                  f"- Median settling time, successes only: **{overall['median_settling_time_s_successes']:.2f} simulated seconds**.",
                  f"- Median final RMS corner error, successes only: **{overall['median_final_error_px_successes']:.3f} px**.", ""]
    lines += ["## Files and reproduction", "",
              "- plan.json: exact starting offsets and trial IDs.",
              "- manifest.json: source hashes, package versions, sampling and controller configuration.",
              "- trials.jsonl and trials.csv: one summary per sampled start.",
              "- traces/trial_NNNN.npz: numeric per-frame time, corners, depth, commands, actual joint rates, camera pose, conditioning and FOV diagnostics. Load with numpy.load(..., allow_pickle=False).",
              "- summary.json: rates, confidence intervals and per-profile statistics.", "",
              "From the project folder, regenerate the figures without running physics:", "",
              "    python analyze_benchmark.py \"PATH_TO_THIS_RUN\"", "",
              "Replay one exact starting pose (current source hashes must match the saved run):", "",
              "    python benchmark.py --replay-run \"PATH_TO_THIS_RUN\" --trial-id 12", "",
              "This step benchmarks one controller under fixed lighting and idealized dynamics. Baseline comparisons, learned features, noise/latency sweeps and physical validation are not included.", "",
              "Interval method: exact Clopper-Pearson (`binomial_ci.py`; see docs/STATISTICS.md).", ""]
    (directory / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps({"overall": overall, "profiles": summary["profiles"]}, indent=2))
    return summary


def latest_run() -> Path:
    latest = read_json(ROOT / "results" / "benchmark" / "latest.json")
    return ROOT / latest["relative_to_project"] if "relative_to_project" in latest else Path(latest["absolute_path"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="?", type=Path)
    args = parser.parse_args()
    analyze(args.directory or latest_run())


if __name__ == "__main__":
    main()

