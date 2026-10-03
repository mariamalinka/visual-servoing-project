"""Fault-injection campaign: trigger every covered safety path, repeatedly, and report.

    run.cmd --fault-campaign                 # 3 repeats per scenario
    run.cmd --fault-campaign --repeats 5
    run.cmd --fault-campaign --report-only results/fault-injection/<run>

Writes REPORT.md and summary.json to results/fault-injection/<timestamp>. Each safety
path gets one of these statuses:

- IMPLEMENTED: the code exists, but nothing triggers it (the status before this campaign).
- TESTED: an automated test triggers it, but this run saved no evidence for it.
- VERIFIED: every repeat produced the documented reaction, with every check passing,
  and the injected fault is the fault the requirement is about.
- PARTIALLY VERIFIED: every repeat passed, but the injection covers only part of the
  requirement (the report says which part).
- NOT VERIFIED: at least one repeat did not produce the documented reaction.

Everything is simulation: the realtime timings depend on the computer that runs the
campaign, and nothing here is evidence about a real robot.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import time

ROOT = Path(__file__).resolve().parent

TESTS = dict(
    control_loop_stall="tests/test_fault_injection.py::test_control_loop_stall_stops_with_control_overrun",
    control_loop_stall_near_limit="tests/test_fault_injection.py::test_stall_just_over_the_deadline_still_stops",
    control_loop_stall_below_limit="tests/test_fault_injection.py::test_stall_below_the_deadline_does_not_stop",
    sensor_worker_crash="tests/test_fault_injection.py::test_sensor_worker_crash_stops_the_arm",
    sensor_worker_partial_message="tests/test_fault_injection.py::test_half_sent_frame_cannot_block_the_control_loop",
    sensor_result_read_blocked="tests/test_fault_injection.py::test_blocked_result_read_cannot_block_the_control_loop",
    watchdog_stop="tests/test_fault_injection.py::test_watchdog_stop_reference",
    physics_gap_cap="tests/test_fault_injection.py::test_physics_gap_is_capped_and_counted",
    controller_stall="tests/test_fault_injection.py::test_slow_controller_computation_stops_without_applying_its_command",
    clock_step_back="tests/test_fault_injection.py::test_clock_stepping_back_stops_with_clock_error",
    inference_failure="tests/test_fault_injection.py::test_failed_inference_stops_the_alignment",
    collision_block="tests/test_fault_injection.py::test_collision_block_stops_the_realtime_alignment",
    run_timeout="tests/test_fault_injection.py::test_run_limit_ends_a_moving_alignment",
    speed_clip="tests/test_fault_injection.py::test_speed_command_above_the_limit_is_clipped",
)

# Measurements shown in the summary table (key, label, unit, worst-of: max or min).
TIMINGS = (("stop_after_stall_end_ms", "after the stalled tick"), ("stop_after_computation_ms", "after the late computation"),
           ("stop_after_jump_ms", "after the clock jump"), ("stop_after_failed_result_received_ms", "after the failed result"),
           ("stop_after_block_ms", "after the block"), ("stop_after_limit_ms", "after the run limit"),
           ("stop_after_exit_ms", "after the OS reported the worker exited"),
           ("stop_after_kill_ms", "after the kill request"), ("stop_after_deadline_ms", "after the image-age limit"))


def status_of(runs):
    if not runs:
        return "TESTED"
    if not all(r["passed"] for r in runs):
        return "NOT VERIFIED"
    return "VERIFIED" if all(r["coverage"] == "full" for r in runs) else "PARTIALLY VERIFIED"


def worst(values, pick=max):
    values = [v for v in values if v is not None]
    return pick(values) if values else None


def summarize(name, runs):
    first = runs[0]
    measured = [r["measured"] for r in runs]
    timing_key = next((k for k, _ in TIMINGS if any(k in m for m in measured)), None)
    physical = [m.get("physical_stop") or {} for m in measured]
    safety = [{x["key"]: x for x in (m.get("safety_metrics") or [])} for m in measured]
    speeds = [p["peak_measured_rad_s"] for m in measured for p in m.get("profiles", []) if "note" not in p]
    return dict(
        scenario=name, id=first["id"], requirements=first["requirements"], mechanism=first["mechanism"],
        fault=first["fault"], expected=first["expected"], coverage=first["coverage"], coverage_note=first["coverage_note"],
        test=TESTS.get(name), repeats=len(runs), passed=sum(r["passed"] for r in runs), status=status_of(runs),
        deterministic=bool(first.get("deterministic")),
        failed_checks=sorted({c["check"] for r in runs for c in r["checks"] if not c["passed"]}),
        reaction_ms_max=None if timing_key is None else worst(m.get(timing_key) for m in measured),
        reaction_label=None if timing_key is None else dict(TIMINGS)[timing_key],
        physical_stop_ms_max=worst(p.get("physical_stop_ms") for p in physical),
        camera_travel_mm_max=worst(p.get("camera_travel_mm") for p in physical),
        speed_at_fault_rad_s_max=worst(p.get("speed_at_command_rad_s") for p in physical),
        peak_measured_speed_rad_s=worst([s.get("measured_speed", {}).get("value") for s in safety] + speeds),
        min_environment_clearance_mm=worst([s.get("environment_clearance", {}).get("value") for s in safety], min),
        min_self_clearance_mm=worst([s.get("self_clearance", {}).get("value") for s in safety], min),
        min_joint_margin_rad=worst([s.get("joint_margin", {}).get("value") for s in safety], min),
        discarded_physics_s_max=worst(m.get("discarded_physics_s") for m in measured),
        exit_after_kill_ms_max=worst(m.get("exit_after_kill_ms") for m in measured),
        stop_after_kill_ms_max=worst(m.get("stop_after_kill_ms") for m in measured),
        largest_loop_gap_ms=worst(m.get("largest_loop_gap_ms") for m in measured))


def fmt(value, digits=1, unit=""):
    return "—" if value is None else f"{value:.{digits}f}{unit}"


def write_report(directory):
    data = json.loads((directory / "summary.json").read_text(encoding="utf-8"))
    rows = [summarize(name, runs) for name, runs in data["runs"].items()]  # Always from the raw records.
    counts = {s: sum(r["status"] == s for r in rows) for s in
              ("VERIFIED", "PARTIALLY VERIFIED", "NOT VERIFIED", "TESTED")}
    lines = ["# Fault-injection campaign", "",
             f"Run {data['created_utc']} on {data['host']}, actuator profile `{data['actuator']}`, "
             f"{data['repeats']} repeat(s) per scenario (deterministic scenarios once). Simulation only: timings "
             "depend on this computer, and nothing here is evidence about a real robot.", "",
             f"Source fingerprint: {len(data.get('fingerprint', {}).get('sha256', {}))} files hashed in `summary.json` "
             f"(`realtime.py` {data.get('fingerprint', {}).get('sha256', {}).get('realtime.py', '?')[:12]}, "
             f"`simulation.py` {data.get('fingerprint', {}).get('sha256', {}).get('simulation.py', '?')[:12]}).", "",
             "Before this campaign every path below was IMPLEMENTED only: the code existed, but no test triggered it.",
             "", "**Result:** " + ", ".join(f"{v} {k}" for k, v in counts.items() if v) + ".", "",
             "## Safety paths", "",
             "| Path | Requirement | Fault injected | Expected reaction | Passed | Status |", "|---|---|---|---|---:|---|"]
    for r in rows:
        lines.append(f"| {r['id']} | {', '.join(r['requirements'])} | {r['fault']} | {r['expected']} | "
                     f"{r['passed']}/{r['repeats']} | **{r['status']}** |")
    lines += ["", "## Measured reaction (worst over the repeats)", "",
              "| Path | Stop issued | Speed at the fault | Physical stop | Camera travel | Peak measured speed | "
              "Minimum clearance above margin: environment / robot parts | Minimum joint margin |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        reaction = "—" if r["reaction_ms_max"] is None else f"{r['reaction_ms_max']:.1f} ms {r['reaction_label']}"
        if r.get("largest_loop_gap_ms") is not None:
            reaction = f"no stop (largest loop gap {r['largest_loop_gap_ms']:.1f} ms)"
        if r.get("exit_after_kill_ms_max") is not None:
            reaction += (f"; OS reported the exit up to {r['exit_after_kill_ms_max']:.1f} ms after the kill request; "
                         f"stop up to {r['stop_after_kill_ms_max']:.1f} ms after it")
        lines.append(f"| {r['id']} | {reaction} | {fmt(r['speed_at_fault_rad_s_max'], 3, ' rad/s')} | "
                     f"{fmt(r['physical_stop_ms_max'], 0, ' ms')} | {fmt(r['camera_travel_mm_max'], 2, ' mm')} | "
                     f"{fmt(r['peak_measured_speed_rad_s'], 3, ' rad/s')} | {fmt(r['min_environment_clearance_mm'], 1, ' mm')} / "
                     f"{fmt(r['min_self_clearance_mm'], 1, ' mm')} | "
                     f"{fmt(r['min_joint_margin_rad'], 3, ' rad')} |")
    lines += ["", "## Notes per path", ""]
    for r in rows:
        lines.append(f"- **{r['id']}** — mechanism: {r['mechanism']}. Test: `{r['test']}`."
                     + (f" Failed checks: {'; '.join(r['failed_checks'])}." if r["failed_checks"] else "")
                     + (" Deterministic: run once." if r.get("deterministic") else "")
                     + (f" {'Partial coverage' if r['coverage'] != 'full' else 'Note'}: {r['coverage_note']}"
                        if r["coverage_note"] else ""))
    lines += ["", "## Status definitions", "",
              "- **IMPLEMENTED:** the code exists; nothing triggers it.",
              "- **TESTED:** an automated test triggers it; no saved evidence in this run.",
              "- **VERIFIED:** every repeat produced the documented reaction with all checks passing, and the injected "
              "fault is the fault the requirement is about.",
              "- **PARTIALLY VERIFIED:** every repeat passed, but the injection covers only part of the requirement.",
              "- **NOT VERIFIED:** at least one repeat did not produce the documented reaction.", "",
              "Each repeat's full record (fault, expected, measured, every check) is in `summary.json`. The physical "
              "stop uses the actuator model (docs/ACTUATOR_MODEL.md); clearance and joint margin come from the "
              "per-step safety record (safety_metrics.py)."]
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return lines


def main(argv=None):
    import fault_injection
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fault-campaign", action="store_true", help=argparse.SUPPRESS)  # run.cmd route.
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenarios", nargs="+", choices=tuple(fault_injection.SCENARIOS))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report-only", type=Path, metavar="DIR")
    args = parser.parse_args(argv)
    if args.report_only:
        for line in write_report(args.report_only.resolve()):
            print(line)
        return 0
    if not 1 <= args.repeats <= 50:
        parser.error("--repeats must be in [1, 50]")
    from actuator import load_actuator_config
    directory = (args.output or ROOT / "results" / "fault-injection" / datetime.now().strftime("%Y%m%d-%H%M%S")).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    names = args.scenarios or list(fault_injection.SCENARIOS)
    from run_camera_robustness import fingerprint
    data = dict(created_utc=datetime.now(timezone.utc).isoformat(), host=f"{platform.system()} {platform.machine()}, "
                f"Python {platform.python_version()}", actuator=load_actuator_config()["profile"], repeats=args.repeats,
                fingerprint=fingerprint(), scenarios=[], runs={})
    for name in names:
        runs = []
        for repeat in range(args.repeats):
            if runs and runs[0].get("deterministic"):
                break  # Same inputs, same result: one run is the evidence.
            started = time.perf_counter()
            try:
                run = fault_injection.SCENARIOS[name]()
            except Exception as exc:  # A crashed scenario is a failed repeat, not a lost campaign.
                run = dict(id=name, requirements=[], mechanism="", fault="", expected="", measured={},
                           checks=[dict(check="scenario ran", passed=False, detail=f"{type(exc).__name__}: {exc}")],
                           passed=False, coverage="full", coverage_note="")
            run["wall_s"] = time.perf_counter() - started
            runs.append(run)
            print(f"{name} [{repeat + 1}/{args.repeats}]: {'passed' if run['passed'] else 'FAILED'}", flush=True)
        data["runs"][name] = runs
        data["scenarios"].append(summarize(name, runs))
        (directory / "summary.json").write_text(json.dumps(data, indent=1, default=str), encoding="utf-8")
    if fingerprint() != data["fingerprint"]:
        raise RuntimeError("Source files changed during the campaign; its evidence would not match one version")
    for line in write_report(directory):
        print(line)
    return int(any(s["status"] == "NOT VERIFIED" for s in data["scenarios"]))


if __name__ == "__main__":
    raise SystemExit(main())
