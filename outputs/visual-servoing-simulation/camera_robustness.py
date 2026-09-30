"""Repeatable camera robustness plans and reports, without a rendering dependency."""
from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from binomial_ci import format_rate
from camera_timing import TimedCamera, load_camera_timing

ROOT = Path(__file__).resolve().parent
NAMES = {"aruco": "ArUco", "natural": "SIFT", "learned": "Learned"}


def write_json(path, value):
    """Atomic replacement lets a completed trial survive an interrupted run."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def make_plan(config, modes, starts=None, seed=None, profiles=None):
    count = config["starts"] if starts is None else starts
    seed = config["seed"] if seed is None else seed
    if type(count) is not int or not 1 <= count <= 10000:
        raise ValueError("Starts must be an integer in [1, 10000]")
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("Seed must be an integer in [0, 2**32)")
    if not modes or len(set(modes)) != len(modes) or any(m not in NAMES for m in modes):
        raise ValueError("Choose unique modes from aruco, natural, learned")
    limits = np.asarray(config["offset_limit_degrees"], dtype=float)
    if limits.shape != (6,) or not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError("Provide six positive finite offset limits")
    for key in ("max_trial_s", "post_stop_frames"):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if type(config["post_stop_frames"]) is not int:
        raise ValueError("post_stop_frames must be an integer")
    available = config["profiles"]
    names = [p["name"] for p in available]
    if len(set(names)) != len(names) or any(not re.fullmatch(r"[a-z0-9-]+", n) for n in names):
        raise ValueError("Profile names must be unique lowercase names")
    selected = names if profiles is None else profiles
    if not selected or len(set(selected)) != len(selected) or not set(selected) <= set(names):
        raise ValueError("Choose unique configured profiles")
    specs = []
    for profile in available:
        if profile["name"] not in selected:
            continue
        spec = dict(profile)
        name, expected = spec.pop("name"), spec.pop("expected", "converged")
        if expected not in ("converged", "stale_camera"):
            raise ValueError("Expected outcomes must be converged or stale_camera")
        allowed = set(load_camera_timing()) | {"jitter_s", "drop_probability", "outages_s", "seed"}
        if set(spec) - allowed:
            raise ValueError(f"Unknown camera settings: {set(spec) - allowed}")
        timing = dict(load_camera_timing(), **spec)
        timing["max_run_s"] = config["max_trial_s"]
        TimedCamera(timing, 30)
        specs.append(dict(name=name, expected=expected, timing=timing))
    # Independent generators: changing a profile never changes the starting poses.
    rng = np.random.default_rng(seed)
    offsets = rng.uniform(-limits, limits, size=(count, 6)).tolist()
    trials = []
    for mode in modes:
        for spec in specs:
            for case, offset in enumerate(offsets):
                noise_seed = int(np.random.SeedSequence([seed, case, 1]).generate_state(1)[0])
                perception_seed = int(np.random.SeedSequence([seed, case, 2]).generate_state(1)[0] % (2**31))
                timing = dict(spec["timing"], seed=noise_seed)
                trials.append(dict(id=f"{mode}-{spec['name']}-{case:04d}", mode=mode,
                                   profile=spec["name"], case=case, offset_degrees=offset,
                                   timing=timing, perception_seed=perception_seed,
                                   expected=spec["expected"]))
    return dict(schema_version=1, seed=seed, starts=count, modes=modes, profiles=specs,
                post_stop_frames=config["post_stop_frames"], trials=trials)


def validate_rows(plan, rows):
    expected = {t["id"]: t for t in plan["trials"]}
    seen = set()
    for row in rows:
        key = row["id"]
        if key in seen or key not in expected:
            raise ValueError(f"Duplicate or unknown trial: {key}")
        for field in ("mode", "profile", "case", "expected"):
            if row[field] != expected[key][field]:
                raise ValueError(f"Trial metadata differs from the plan: {key}")
        seen.add(key)


def summarize(plan, rows):
    validate_rows(plan, rows)
    groups = []
    for mode in plan["modes"]:
        for profile in plan["profiles"]:
            group = [r for r in rows if r["mode"] == mode and r["profile"] == profile["name"]]
            success = [r for r in group if r["outcome"] == "converged"]
            def metric(field, selected=group, percentile=50):
                values = [r[field] for r in selected if r.get(field) is not None]
                return float(np.percentile(values, percentile)) if values else None
            groups.append(dict(mode=mode, profile=profile["name"], expected=profile["expected"],
                planned=plan["starts"], completed=len(group), converged=len(success),
                expected_outcomes=sum(bool(r.get("expected_pass")) for r in group),
                safety_violations=sum(bool(r.get("safety_violations")) for r in group),
                outcomes=dict(Counter(r["outcome"] for r in group)),
                median_success_s=metric("completion_s", success),
                p95_success_s=metric("completion_s", success, 95),
                p95_final_error_px=metric("final_current_error_px", percentile=95),
                median_joint_travel_rad=metric("joint_travel_rad")))
    return groups


def report(directory):
    plan = json.loads((directory/"plan.json").read_text())
    rows = json.loads((directory/"trials.json").read_text())
    groups = summarize(plan, rows)
    write_json(directory/"summary.json", groups)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    matrix = np.full((len(plan["modes"]), len(plan["profiles"])), np.nan)
    for i, mode in enumerate(plan["modes"]):
        for j, profile in enumerate(plan["profiles"]):
            g = next(g for g in groups if g["mode"] == mode and g["profile"] == profile["name"])
            if g["completed"]:
                matrix[i, j] = g["expected_outcomes"] / g["completed"]
    fig, ax = plt.subplots(figsize=(max(8, len(plan["profiles"])*1.25), 3.4))
    heat = ax.imshow(matrix, vmin=0, vmax=1, cmap="RdYlGn", aspect="auto")
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            g = groups[i*matrix.shape[1]+j]
            text = "pending" if not g["completed"] else f"{g['expected_outcomes']}/{g['completed']}"
            ax.text(j, i, text, ha="center", va="center", color="black")
    ax.set_xticks(range(len(plan["profiles"])), [p["name"] for p in plan["profiles"]], rotation=25, ha="right")
    ax.set_yticks(range(len(plan["modes"])), [NAMES[m] for m in plan["modes"]])
    ax.set_title("Expected outcome met (alignment or specified watchdog stop)")
    fig.colorbar(heat, ax=ax, label="Fraction of completed trials")
    fig.tight_layout()
    fig.savefig(directory/"robustness.png", dpi=160)
    fig.savefig(directory/"robustness.svg")
    plt.close(fig)
    def fmt(value):
        return "—" if value is None else f"{value:.3f}"
    lines = ["# Automated camera robustness experiment", "",
        f"Seed: {plan['seed']}. Completed {len(rows)}/{len(plan['trials'])} planned trials. "
        "Partial runs retain their full planned denominator.", "",
        "The same starting poses and per-capture random draws are paired across matchers. "
        "Offsets are sampled without filtering for visibility or success. Jitter and loss "
        "are simulated transport effects; host inference duration is measured separately.", "",
        "## Disturbances", "",
        "| Profile | Delay / uniform jitter, ms | Independent packet loss | Capture outages, s | Expected |",
        "|---|---:|---:|---|---|"]
    for p in plan["profiles"]:
        c = p["timing"]
        lines.append(f"| {p['name']} | {1000*c['delay_s']:g} / ±{1000*c.get('jitter_s',0):g} | "
                     f"{100*c.get('drop_probability',0):g}% | {c.get('outages_s',[])} | {p['expected']} |")
    lines += ["", "![Expected outcomes](robustness.png)", "",
        "| Matcher | Profile | Completed / planned | Aligned | Expected outcome met | Safety violations | Median / p95 successful time, simulated s | p95 final image error, px |",
        "|---|---|---:|---:|---:|---:|---:|---:|"]
    lines[-2:-2] = ["Aligned and expected-outcome counts show 95% Clopper-Pearson (exact) confidence intervals "
                    "over completed trials; see docs/STATISTICS.md.", ""]
    for g in groups:
        lines.append(f"| {NAMES[g['mode']]} | {g['profile']} | {g['completed']}/{g['planned']} | "
                     f"{format_rate(g['converged'], g['completed'])} | {format_rate(g['expected_outcomes'], g['completed'])} | "
                     f"{g['safety_violations']} | "
                     f"{fmt(g['median_success_s'])} / {fmt(g['p95_success_s'])} | {fmt(g['p95_final_error_px'])} |")
    lines += ["", "## Outcome definitions", "",
        "Aligned means the controller stopped as converged, then all configured fresh "
        "current-image checks remained below the configured pixel threshold with zero commands. "
        "The checks use the selected detector; they are not independent physical pose ground truth.",
        "", "A specified outage passes only with a stale-camera stop at the observation-age "
        "deadline, stationary commands afterwards, and observed capture restoration. "
        "A safe watchdog stop in a convergence profile remains an alignment failure. "
        "Safety checks sample commands and actual joint limits on every 2 ms physics tick.",
        "", "Completion quantiles include successful trials only. Error quantiles include every "
        "available terminal measurement. Small samples are descriptive, not a stability guarantee. "
        "No gain retuning or delay compensation is applied.", "",
        "## Outcomes retained", "",
        "| Matcher | Profile | Outcome counts |",
        "|---|---|---|"]
    for g in groups:
        lines.append(f"| {NAMES[g['mode']]} | {g['profile']} | {g['outcomes']} |")
    lines += ["", "[Plan](plan.json) · [All trials](trials.json) · [Summary](summary.json) · "
              "[Source and runtime manifest](manifest.json)", "",
              "Per-frame traces and exception logs remain local under traces/. Resume uses the "
              "saved plan and requires matching source, reference, asset and runtime fingerprints. "
              "The report can be regenerated from the JSON files without raw traces.", ""]
    (directory/"REPORT.md").write_text("\n".join(lines), encoding="utf-8")

