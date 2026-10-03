"""Safety metrics recorded in every realtime run, judged against their requirements.

The measurements come from Simulation.safety_record(), which keeps the extremes
seen on every 2 ms physics step:
- clearance: the collision guard's own distance check (collision.py) minus the
  required margin, separately to the environment (12 mm margin) and between robot
  parts (6 mm). The guard computes distances only up to its influence range
  (80 mm), so a larger clearance is reported as at most 80 mm minus the margin;
- joint margin: the distance of each joint to its scene.xml range;
- speed: the command after the guard and backstop, and the measured joint velocity.

This module only evaluates those records. It adds no geometry of its own. Each
metric carries the measured value, the limit, PASS/FAIL and the requirement ID
from docs/SAFETY_TRACEABILITY.md. A run that predates the record reports
"not recorded", never PASS.

The values describe the simulation. They are not measurements of a real robot.
"""
from __future__ import annotations

import math

TOLERANCE = 1e-9

# (key, requirement, description, unit, comparison)
DEFINITIONS = (
    ("environment_clearance", "REQ-11", "Minimum clearance to the environment above its 12 mm margin", "mm", ">="),
    ("self_clearance", "REQ-11", "Minimum clearance between robot parts above its 6 mm margin", "mm", ">="),
    ("joint_margin", "REQ-12", "Minimum distance to a joint limit", "rad", ">="),
    ("command_speed", "REQ-13", "Peak commanded joint speed", "rad/s", "<="),
    ("measured_speed", "REQ-13", "Peak measured joint speed", "rad/s", "<="),
)


def merge(records):
    """Worst case over several records (sessions): lowest margins, highest speeds."""
    records = [r for r in records if r]
    if not records:
        return None
    merged = dict(steps=sum(r.get("steps", 0) for r in records), limits=records[0].get("limits"),
                  sessions=len(records))
    for key, detail, pick in (("min_environment_slack_m", "min_environment_pair", min),
                              ("min_self_slack_m", "min_self_pair", min),
                              ("min_joint_margin_rad", "min_joint_margin_joint", min),
                              ("peak_command_rad_s", None, max), ("peak_speed_rad_s", "peak_speed_joint", max)):
        present = [r for r in records if r.get(key) is not None]
        if not present:
            merged[key] = None
            if detail:
                merged[detail] = None
            continue
        worst = pick(present, key=lambda r: r[key])
        merged[key] = worst[key]
        if detail:
            merged[detail] = worst.get(detail)
    return merged


def evaluate(record):
    """List of metric dicts: value, limit, passed (None = not recorded), requirement."""
    limits = (record or {}).get("limits") or {}
    speed_limit = limits.get("max_joint_velocity_rad_s")
    values = {}
    if record:
        for kind in ("environment", "self"):
            slack = record.get(f"min_{kind}_slack_m")
            pair = record.get(f"min_{kind}_pair")
            values[f"{kind}_clearance"] = (None if slack is None else 1000 * slack, 0.0,
                                           None if not pair else " / ".join(pair))
        margin = record.get("min_joint_margin_rad")
        joint = record.get("min_joint_margin_joint")
        values["joint_margin"] = (margin, 0.0, None if joint is None else f"joint {joint}")
        values["command_speed"] = (record.get("peak_command_rad_s"), speed_limit, None)
        joint = record.get("peak_speed_joint")
        values["measured_speed"] = (record.get("peak_speed_rad_s"), speed_limit, None if joint is None else f"joint {joint}")
    metrics = []
    for key, requirement, description, unit, comparison in DEFINITIONS:
        value, limit, detail = values.get(key, (None, None, None))
        if value is None or limit is None or (isinstance(value, float) and not math.isfinite(value)):
            passed = None
        elif comparison == ">=":
            passed = value >= limit - TOLERANCE
        else:
            passed = value <= limit + TOLERANCE
        metrics.append(dict(key=key, requirement=requirement, description=description, unit=unit,
                            value=value, limit=limit, comparison=comparison, passed=passed, detail=detail))
    return metrics


def failures(metrics):
    """Failure messages for gated verdicts; metrics that were not recorded are not failures."""
    return [f"{m['requirement']} {m['description'].lower()} {fmt_value(m)} (limit {m['comparison']} {fmt_limit(m)})"
            for m in metrics if m["passed"] is False]


def fmt_value(metric):
    value = metric["value"]
    if value is None:
        return "not recorded"
    digits = 2 if metric["unit"] == "mm" else 3
    return f"{value:.{digits}f} {metric['unit']}"


def fmt_limit(metric):
    return "n/a" if metric["limit"] is None else f"{metric['limit']:g} {metric['unit']}"


def verdict(metric):
    return {True: "PASS", False: "FAIL", None: "not recorded"}[metric["passed"]]


def table_lines(columns):
    """Markdown table: one row per metric, one column per (label, metrics) pair."""
    lines = ["| Requirement | Safety metric (every physics step) | Limit | " + " | ".join(label for label, _ in columns) + " |",
             "|---|---|---|" + "---|" * len(columns)]
    for index, (key, requirement, description, unit, comparison) in enumerate(DEFINITIONS):
        limit = next((fmt_limit(metrics[index]) for _, metrics in columns if metrics and metrics[index]["limit"] is not None),
                     "n/a")
        cells = []
        for _, metrics in columns:
            if not metrics:
                cells.append("not recorded")
                continue
            m = metrics[index]
            cells.append(f"{fmt_value(m)} **{verdict(m)}**" + (f" ({m['detail']})" if m["detail"] and m["passed"] is not None else ""))
        lines.append(f"| {requirement} | {description} | {comparison} {limit} | " + " | ".join(cells) + " |")
    return lines


def flat(metrics):
    """Flat dict for CSV columns: value and pass per metric."""
    row = {}
    for m in metrics or []:
        row[f"{m['key']}_{m['unit'].replace('/', '_per_')}"] = "" if m["value"] is None else round(m["value"], 6)
        row[f"{m['key']}_pass"] = "" if m["passed"] is None else int(m["passed"])
    return row
