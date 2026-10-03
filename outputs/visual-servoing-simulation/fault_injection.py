"""Fault-injection scenarios for safety paths of the realtime runtime and the simulation.

Each scenario deliberately triggers one fault and returns a record of three things:
- `fault`: what was injected, and how;
- `expected`: the documented reaction;
- `measured`: what the runtime actually did, with timings, speeds and margins.

`checks` compares the two, and `passed` is true only if every check holds.

Injection follows the project's existing patterns:
- **Sensor-side faults** use RuntimeConfig fields, like the existing
  `inference_stall_s`: here `inference_failure`.
- **Control-side faults** replace a class for the duration of one session, as the
  acceptance tool does with `simulation.Simulation`. The sensor worker is a separate
  spawned process and never sees these replacements.
- **Result-pipe faults** replace the worker function the runtime spawns
  (`partial_result_worker`, a module-level function the spawned process imports by
  name) or the parent's `multiprocessing` queue read for one session.

Nothing here changes production behaviour when no fault is injected. The results
describe the simulation, not a real robot.

    tests/test_fault_injection.py   one test per scenario
    run_fault_campaign.py           all scenarios, repeated, with a report
"""
from __future__ import annotations

from contextlib import contextmanager
import os
from pathlib import Path
import sys
import tempfile
import threading
import time

import numpy as np

AGE_LIMIT_S = 0.4
OFFSET = [3, -3, 4, 3, -2, 2]
CONTROL_DEADLINE_S = 0.05


@contextmanager
def replaced(owner, name, value):
    """Temporarily replace owner.name (restored even if the session fails)."""
    original = getattr(owner, name)
    setattr(owner, name, value)
    try:
        yield original
    finally:
        setattr(owner, name, original)


def check(checks, name, ok, detail=""):
    checks.append(dict(check=name, passed=bool(ok), detail=detail))


def run_session(config, until, *, settle_s=0.7, timeout_s=30, actuator=None, obstacle=False):
    """One RealtimeSession until `until(session)`, then settle so the arm stops; return its report."""
    from realtime import RealtimeSession
    session = RealtimeSession(config=config, offset=OFFSET, actuator=actuator, obstacle=obstacle,
                              auto_start=False).start()
    try:
        if not session.ready.wait(90):
            raise RuntimeError("Realtime session did not start")
        # Let the idle loop settle before Align, so a start-up scheduling gap cannot
        # end the alignment before the fault is injected.
        time.sleep(0.5)
        session.command("align")
        deadline = time.perf_counter() + timeout_s
        while not until(session) and time.perf_counter() < deadline:
            if session.snapshot().get("error"):
                break
            time.sleep(.005)
        time.sleep(settle_s)
        final = session.snapshot()
    finally:
        closing = time.perf_counter()
        session.close()  # Raises if the control loop does not shut down within five seconds.
    close_s = time.perf_counter() - closing
    report = session.report()
    report["close_s"] = close_s
    report["final_state"] = {k: final.get(k) for k in ("status", "active", "error")}
    report["final_command"] = None if final.get("command") is None else np.asarray(final["command"]).tolist()
    return report


def stopped(reason):
    return lambda session: session.snapshot().get("status") == reason


def moving(session, accepted=10):
    counts = session.snapshot().get("counters") or {}
    return counts.get("accepted", 0) >= accepted and counts.get("moving_ticks", 0) > 0


def stop_event(report, reason):
    return next((e for e in report["events"] if e["kind"] == "stop" and e.get("reason") == reason), None)


def common_measurements(report, reason, checks, *, runtime_error_expected=False):
    """Measurements and checks shared by every runtime stop scenario.

    runtime_error_expected: the documented reaction ends the runtime with an error
    after the stop (a failed sensor worker), so the final status is runtime_error.
    """
    from safety_metrics import evaluate
    from stop_response import link
    counts = report["counts"]
    event = stop_event(report, reason)
    physical = next((r for r in link(report) if r["kind"] == "stop" and r["reason"] == reason), None)
    measured = dict(
        stop_reason_observed=report["final_state"]["status"],
        stop_event_found=event is not None,
        final_command_zero=report["final_command"] is not None and not np.any(report["final_command"]),
        moving_ticks=counts["moving_ticks"], unsafe_motion_ticks=counts["unsafe_motion_ticks"],
        post_stop_motion_ticks=counts["post_stop_motion_ticks"], contacts=counts["contacts"],
        control_deadline_misses=counts["control_deadline_misses"], discarded_physics_s=counts["discarded_physics_s"],
        physical_stop=None if physical is None else {k: physical.get(k) for k in (
            "moving", "outcome", "speed_at_command_rad_s", "physical_stop_ms", "camera_travel_mm", "max_joint_travel_rad")},
        safety_metrics=None if not report.get("safety") else evaluate(report["safety"]))
    if runtime_error_expected:
        check(checks, "the runtime ended with its error report after the stop",
              measured["stop_reason_observed"] == "runtime_error" and bool(report.get("error")))
    else:
        check(checks, f"stop reason is {reason}", measured["stop_reason_observed"] == reason,
              f"observed {measured['stop_reason_observed']}")
        check(checks, "the runtime kept running (no runtime error)", not report["final_state"].get("error")
              and not report.get("error"), (report["final_state"].get("error") or "")[-120:])
    check(checks, "stop event recorded", event is not None)
    check(checks, "the robot was moving before the fault", counts["moving_ticks"] > 0)
    check(checks, "command zero after the stop, and it stayed zero", measured["final_command_zero"]
          and counts["post_stop_motion_ticks"] == 0, f"post-stop motion ticks {counts['post_stop_motion_ticks']}")
    check(checks, "no unsafe motion or forbidden contact", counts["unsafe_motion_ticks"] == 0 and counts["contacts"] == 0,
          f"unsafe {counts['unsafe_motion_ticks']}, contacts {counts['contacts']}")
    if physical is not None and physical.get("moving") and not runtime_error_expected:
        check(checks, "physical stop reached standstill", physical.get("outcome") == "stopped",
              f"{physical.get('physical_stop_ms')} ms, camera travel {physical.get('camera_travel_mm')} mm")
    if measured["safety_metrics"]:
        failed = [m for m in measured["safety_metrics"] if m["passed"] is False]
        check(checks, "clearance, joint margin and speed within REQ-11 to REQ-13", not failed,
              "; ".join(f"{m['requirement']} {m['key']} {m['value']}" for m in failed))
    return event, measured


def record(identifier, requirements, mechanism, fault, expected, measured, checks, coverage="full", note="",
           deterministic=False):
    return dict(id=identifier, requirements=requirements, mechanism=mechanism, fault=fault, expected=expected,
                measured=measured, checks=checks, passed=all(c["passed"] for c in checks),
                coverage=coverage, coverage_note=note, deterministic=deterministic)


# ----------------------------------------------------------------------------- scenarios

def _stall_session(stall_s, actuator, until_status):
    """A session whose control thread blocks once for stall_s, inside a physics call, while moving."""
    import simulation
    from realtime import RuntimeConfig
    base = simulation.Simulation
    marks = dict(armed=False, stalled=None, largest_step=0.0, step_after_stall=None)

    class StallingSimulation(base):
        def advance(self, seconds):
            marks["largest_step"] = max(marks["largest_step"], seconds)
            if marks["stalled"] is not None and marks["step_after_stall"] is None:
                marks["step_after_stall"] = seconds
            super().advance(seconds)
            if marks["armed"] and marks["stalled"] is None and np.any(self.velocity_command):
                started = time.perf_counter()
                time.sleep(stall_s)  # The control thread itself is blocked.
                marks["stalled"] = (started, time.perf_counter())

    def until(session):
        if not marks["armed"] and moving(session):
            marks["armed"] = True
        return marks["stalled"] is not None and session.snapshot().get("status") == until_status

    with replaced(simulation, "Simulation", StallingSimulation):
        report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0), until,
                             actuator=actuator)
    return report, marks


def control_loop_stall(stall_s=0.15, actuator=None, identifier="control-loop-stall"):
    """REQ-03 and REQ-08: one control tick blocks for stall_s while the robot moves."""
    report, marks = _stall_session(stall_s, actuator, "control_overrun")
    checks = []
    event, measured = common_measurements(report, "control_overrun", checks)
    stall = marks["stalled"]
    measured.update(injected_stall_ms=None if stall is None else 1000 * (stall[1] - stall[0]),
                    largest_physics_step_s=marks["largest_step"], physics_step_after_stall_s=marks["step_after_stall"])
    if stall is not None and event is not None:
        measured["stop_after_stall_end_ms"] = 1000 * (event["stopped_s"] - stall[1])
        check(checks, "stop within one control deadline after the stalled tick",
              0 <= measured["stop_after_stall_end_ms"] <= 1000 * CONTROL_DEADLINE_S,
              f"{measured['stop_after_stall_end_ms']:.2f} ms")
    check(checks, "deadline miss counted", report["counts"]["control_deadline_misses"] >= 1)
    return record(identifier, ["REQ-03"], "realtime.py: gap > max_control_gap_s at the start of a tick "
                  "-> stop('control_overrun')",
                  f"the control thread blocks for {1000 * stall_s:.0f} ms inside one physics call while moving",
                  "alignment stops as control_overrun on the next tick; zero command; no motion afterwards",
                  measured, checks, note="In the simulation the arm cannot move while the control thread is "
                  "blocked, because physics runs in that thread. A real drive would keep executing the last command "
                  "for the whole stall (here about speed x stall, e.g. 0.05 rad/s x 0.15 s = 7.5 mrad).")


def control_loop_stall_near_limit(actuator=None):
    """REQ-03: a stall only just longer than the 50 ms deadline still stops."""
    return control_loop_stall(0.055, actuator, "control-loop-stall-near-limit")


def control_loop_stall_below_limit(stall_s=0.035, actuator=None):
    """REQ-03 negative control: a stall shorter than the deadline must not stop the alignment."""
    report, marks = _stall_session(stall_s, actuator, "converged")
    checks, counts = [], report["counts"]
    stall = marks["stalled"]
    measured = dict(injected_stall_ms=None if stall is None else 1000 * (stall[1] - stall[0]),
                    stop_reason_observed=report["final_state"]["status"],
                    control_deadline_misses=counts["control_deadline_misses"],
                    largest_loop_gap_ms=None if not report["control_gap_ms"] else report["control_gap_ms"]["max"])
    check(checks, "the stall was injected while moving", stall is not None)
    check(checks, "no control_overrun and no deadline miss", counts["control_deadline_misses"] == 0
          and stop_event(report, "control_overrun") is None, f"misses {counts['control_deadline_misses']}")
    check(checks, "the alignment converged", measured["stop_reason_observed"] == "converged",
          f"observed {measured['stop_reason_observed']}")
    return record("control-loop-stall-below-limit", ["REQ-03"], "realtime.py: gap > max_control_gap_s",
                  f"the control thread blocks for {1000 * stall_s:.0f} ms (below the 50 ms deadline) while moving",
                  "no stop: the alignment continues and converges", measured, checks)


def physics_gap_cap(stall_s=0.15, actuator=None):
    """REQ-08: the same stall must not be replayed as one long physics step."""
    base = control_loop_stall(stall_s, actuator)
    measured = base["measured"]
    checks = [c for c in base["checks"] if c["check"].startswith("stop reason")]
    expected_discard = stall_s - CONTROL_DEADLINE_S
    step = measured["physics_step_after_stall_s"]
    check(checks, "the physics call after the stall was capped at exactly max_control_gap_s",
          step is not None and abs(step - CONTROL_DEADLINE_S) < 1e-9, f"{step} s for a {stall_s:.2f} s gap")
    check(checks, "no physics call longer than max_control_gap_s",
          measured["largest_physics_step_s"] <= CONTROL_DEADLINE_S + 1e-9, f"{measured['largest_physics_step_s']:.4f} s")
    check(checks, "the lost time is counted in discarded_physics_s",
          abs(measured["discarded_physics_s"] - expected_discard) <= 0.03,
          f"{measured['discarded_physics_s']:.3f} s (expected about {expected_discard:.3f} s)")
    return record("physics-gap-cap", ["REQ-08"], "realtime.py: sim.advance(min(gap, max_control_gap_s)); "
                  "discarded_physics_s", base["fault"],
                  "the next physics call is capped at 50 ms and the rest of the gap is counted as discarded",
                  measured, checks, note=base["coverage_note"])


def controller_stall(stall_s=0.08, actuator=None):
    """REQ-03: one controller computation takes longer than the control deadline."""
    import recovery
    from realtime import RuntimeConfig
    original = recovery.ReacquiringIBVS.update
    marks = dict(calls=0, stalled=None)

    def slow_update(self, *args, **kwargs):
        marks["calls"] += 1
        result = original(self, *args, **kwargs)
        if marks["calls"] == 6 and marks["stalled"] is None:
            started = time.perf_counter()
            time.sleep(stall_s)
            marks["stalled"] = (started, time.perf_counter())
        return result

    with replaced(recovery.ReacquiringIBVS, "update", slow_update):
        report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0),
                             stopped("control_overrun"), actuator=actuator)
    checks = []
    event, measured = common_measurements(report, "control_overrun", checks)
    stall = marks["stalled"]
    measured["injected_stall_ms"] = None if stall is None else 1000 * (stall[1] - stall[0])
    if stall is not None and event is not None:
        measured["stop_after_computation_ms"] = 1000 * (event["stopped_s"] - stall[1])
        check(checks, "stop issued right after the late computation", 0 <= measured["stop_after_computation_ms"] <= 5,
              f"{measured['stop_after_computation_ms']:.2f} ms")
        late = [f for f in report["frames"] if f["applied_s"] >= stall[0]]
        measured["commands_applied_from_late_computation"] = len(late)
        check(checks, "the late command was never applied", not late, f"{len(late)} applied")
    return record("controller-stall", ["REQ-03"], "realtime.py: computed - last_tick > max_control_gap_s after "
                  "controller.update() -> stop('control_overrun')",
                  f"the 6th controller computation takes {1000 * stall_s:.0f} ms longer",
                  "stop as control_overrun without applying the late command", measured, checks)


class SteppedClock:
    """A stand-in for the realtime module's `time`: perf_counter jumps back once.

    Every perf_counter call made from realtime.py sees the jump (in practice the
    control thread; the test thread does not call it). The sensor worker is a
    separate process and keeps the real clock.
    """

    def __init__(self, jump_s):
        self.jump_s = jump_s
        self.armed = False
        self.jumped_at = None  # (real, reported) at the jump

    def perf_counter(self):
        now = time.perf_counter()
        if self.armed and self.jumped_at is None:
            self.jumped_at = (now, now - self.jump_s)
        return now - self.jump_s if self.jumped_at is not None else now

    def __getattr__(self, name):
        return getattr(time, name)


def clock_step_back(jump_s=1.0, actuator=None):
    """REQ-06: the runtime's monotonic clock appears to run backwards."""
    import realtime
    from realtime import RuntimeConfig
    clock = SteppedClock(jump_s)

    def until(session):
        if not clock.armed and moving(session):
            clock.armed = True
        return session.snapshot().get("status") == "clock_error"

    with replaced(realtime, "time", clock):
        report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0), until,
                             actuator=actuator, settle_s=1.8)  # The first tick after the jump sleeps ~jump_s.
    checks = []
    event, measured = common_measurements(report, "clock_error", checks)
    if clock.jumped_at is not None and event is not None:
        measured["stop_after_jump_ms"] = 1000 * (event["stopped_s"] - clock.jumped_at[1])
        check(checks, "stop at the first clock reading after the jump", 0 <= measured["stop_after_jump_ms"] <= 5,
              f"{measured['stop_after_jump_ms']:.2f} ms (runtime clock)")
    return record("clock-step-back", ["REQ-06"], "realtime.py: CommandLease.failure() -> 'clock_error' when now is "
                  "before the lease start or the last capture",
                  f"time.perf_counter as seen by the control thread jumps back {jump_s:.1f} s while moving",
                  "alignment stops as clock_error; zero command", measured, checks)


def inference_failure(actuator=None):
    """REQ-06: the perception result reports a failed inference (as the Learned GPU matcher can)."""
    from realtime import RuntimeConfig
    report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0, fault_after_s=0.6,
                                       inference_failure=1.0), stopped("inference_failed"), actuator=actuator)
    checks = []
    event, measured = common_measurements(report, "inference_failed", checks)
    faulted = [f for f in report["sensor_frames"] if f["fault_injected"]]
    measured["failed_frames_received"] = len(faulted)
    if faulted and event is not None:
        first = min(faulted, key=lambda f: f["received_s"])
        measured["stop_after_failed_result_received_ms"] = 1000 * (event["stopped_s"] - first["received_s"])
        # The result is released after the 50 ms transport delay, then acted on at the next tick.
        check(checks, "stop when the failed result is delivered (transport delay + one deadline)",
              0 <= measured["stop_after_failed_result_received_ms"] <= 1000 * (0.05 + CONTROL_DEADLINE_S),
              f"{measured['stop_after_failed_result_received_ms']:.1f} ms after receipt")
        applied = [f for f in report["frames"] if f["sequence"] == first["sequence"]]
        check(checks, "no command from the failed result", not applied)
    return record("inference-failure", ["REQ-06"], "realtime.py: observation.reason == 'inference_failed' -> "
                  "stop('inference_failed')",
                  "every frame after 0.6 s returns an 'inference_failed' observation (RuntimeConfig.inference_failure = 1)",
                  "alignment stops as inference_failed; no command from the failed result", measured, checks,
                  coverage="partial", note="The failure is injected in the sensor worker; a real Learned GPU "
                  "RuntimeError inside the matcher is not reproduced.")


def collision_block(actuator=None):
    """REQ-15: the collision guard blocks all motion during a realtime alignment."""
    import simulation
    from collision import CollisionDecision
    from realtime import RuntimeConfig
    base = simulation.Simulation
    marks = dict(armed=False, blocked_at=None)

    class BlockingSimulation(base):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            guard = self.collision
            original = guard.filter_velocity

            def filter_velocity(qpos, qvel, request, limit):
                decision = original(qpos, qvel, request, limit)
                if marks["armed"] and np.any(request):
                    marks["blocked_at"] = marks["blocked_at"] or time.perf_counter()
                    guard.blocked_steps += 1
                    guard.last = CollisionDecision(np.zeros(6), "blocked", decision.clearance_m, ("injected", "fault"))
                    return guard.last
                return decision
            guard.filter_velocity = filter_velocity

    def until(session):
        if not marks["armed"] and moving(session):
            marks["armed"] = True
        return session.snapshot().get("status") == "collision_blocked"

    with replaced(simulation, "Simulation", BlockingSimulation):
        report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0), until,
                             actuator=actuator)
    checks = []
    event, measured = common_measurements(report, "collision_blocked", checks)
    if marks["blocked_at"] and event is not None:
        measured["stop_after_block_ms"] = 1000 * (event["stopped_s"] - marks["blocked_at"])
        check(checks, "stop within one control deadline after the block",
              0 <= measured["stop_after_block_ms"] <= 1000 * CONTROL_DEADLINE_S, f"{measured['stop_after_block_ms']:.2f} ms")
    return record("collision-block", ["REQ-15"], "simulation.py: a 'blocked' guard decision latches the request to "
                  "zero; realtime.py: skip_blocked_motion() is False while tracking -> stop('collision_blocked')",
                  "the collision guard's decision is replaced by 'blocked' for every motion request once moving",
                  "command zero in the same physics step; alignment stops as collision_blocked", measured, checks,
                  coverage="partial", note="The guard decision is injected. Geometric blocking is tested in the "
                  "simulation (tests/test_collision.py) but not in the realtime runtime; the search/recovery "
                  "skip branch is not exercised.")


def run_timeout(max_run_s=1.0, actuator=None):
    """REQ-06, REQ-16: the realtime run limit ends a still-moving alignment."""
    from realtime import RuntimeConfig
    report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0, max_run_s=max_run_s),
                         stopped("run_timeout"), actuator=actuator)
    checks = []
    event, measured = common_measurements(report, "run_timeout", checks)
    align = next((e for e in report["events"] if e["kind"] == "align"), None)
    if align is not None and event is not None:
        measured["stop_after_limit_ms"] = 1000 * (event["stopped_s"] - align["at_s"] - max_run_s)
        check(checks, "stop within one control deadline after the run limit",
              0 <= measured["stop_after_limit_ms"] <= 1000 * CONTROL_DEADLINE_S, f"{measured['stop_after_limit_ms']:.2f} ms")
    return record("run-timeout", ["REQ-06", "REQ-16"], "realtime.py: CommandLease.failure() -> 'run_timeout'",
                  f"max_run_s = {max_run_s:.1f} s, shorter than an alignment (about 4 s)",
                  "alignment stops as run_timeout at the limit; zero command", measured, checks)


def speed_clip(actuator="default"):
    """REQ-13: a command above 0.6 rad/s is clipped before it reaches the robot.

    Checked with the given actuator profile (default: the production default). The
    'ideal' profile (no actuator model) is measured too and reported for information:
    without ramps, MuJoCo's velocity servo overshoots a step command slightly.
    """
    from simulation import Simulation
    checks, rows = [], []
    for profile in dict.fromkeys([actuator or "default", "ideal"]):
        informative = profile == "ideal" and (actuator or "default") != "ideal"
        with Simulation(render=False, actuator_config=profile) as sim:
            limit = sim.config["max_joint_velocity_rad_s"]
            sim.reset(sim.config["ibvs"]["start_offset_degrees"])
            sim.reset_safety_record()
            request = np.array([1.5, -1.5, 1.5, 1.5, -1.5, 1.5])
            sim.command_velocity(request)
            requested = sim.requested_velocity.copy()
            sim.advance(0.8)
            safety = sim.safety_record()
            rows.append(dict(profile=profile, requested_rad_s=float(np.max(np.abs(request))),
                             after_clip_rad_s=float(np.max(np.abs(requested))),
                             peak_command_rad_s=safety["peak_command_rad_s"],
                             peak_measured_rad_s=safety["peak_speed_rad_s"], limit_rad_s=limit))
            if informative:
                rows[-1]["note"] = "information only: no actuator model"
                continue
            check(checks, f"{profile}: request clipped to {limit} rad/s", np.max(np.abs(requested)) <= limit + 1e-12,
                  f"{np.max(np.abs(requested)):.3f}")
            check(checks, f"{profile}: commanded speed never above the limit", safety["peak_command_rad_s"] <= limit + 1e-12,
                  f"{safety['peak_command_rad_s']:.3f}")
            check(checks, f"{profile}: measured speed never above the limit", safety["peak_speed_rad_s"] <= limit + 1e-9,
                  f"{safety['peak_speed_rad_s']:.3f}")
    return record("speed-clip", ["REQ-13"], "simulation.py: np.clip in command_velocity; ctrlrange in scene.xml",
                  "a 1.5 rad/s command on all six joints (2.5x the limit)",
                  "command clipped to 0.6 rad/s; measured joint speed never above 0.6 rad/s",
                  dict(profiles=rows), checks, deterministic=True)


def watchdog_stop(actuator=None):
    """REQ-01 reference: the freshness watchdog (stop setting) when the sensor stalls mid-motion."""
    from realtime import RuntimeConfig
    report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0, fault_after_s=0.6,
                                       inference_stall_s=0.8), stopped("stale_camera"), actuator=actuator, settle_s=1.1)
    checks = []
    event, measured = common_measurements(report, "stale_camera", checks)
    if event is not None and event.get("deadline_s") is not None:
        measured["stop_after_deadline_ms"] = 1000 * (event["stopped_s"] - event["deadline_s"])
        check(checks, "zero command within one control deadline of the image-age limit",
              0 <= measured["stop_after_deadline_ms"] <= 1000 * CONTROL_DEADLINE_S,
              f"{measured['stop_after_deadline_ms']:.2f} ms")
    return record("watchdog-stop", ["REQ-01"], "realtime.py: CommandLease.failure() -> 'stale_camera' -> stop()",
                  "the sensor worker stalls 0.8 s, 0.6 s after Align (stop setting, 400 ms image-age limit)",
                  "zero command at most 50 ms after the image age reaches 400 ms; stays stopped", measured, checks)


def kill_and_watch(session, marks):
    """Kill the sensor worker; record the kill request and when the OS reports it exited.

    marks["killed"] is the moment the kill was requested. marks["died"] is the moment the
    worker's process sentinel (a pipe on POSIX, the process handle on Windows) signals
    exit, which is the earliest the runtime's is_alive() check can see it. Waiting on the
    sentinel does not reap the process, so the runtime's own detection is unchanged.
    """
    from multiprocessing.connection import wait
    process = session.worker

    def watch():
        if wait([process.sentinel], timeout=10):
            marks["died"] = time.perf_counter()

    watcher = threading.Thread(target=watch, name="exit-watcher", daemon=True)
    watcher.start()
    marks["killed"] = time.perf_counter()
    process.kill()


def detection_checks(event, marks, measured, checks):
    """Worker-failure detection time, split by who controls it (changed 2026-10-03).

    - Runtime reaction (gated): stop no later than one control deadline (50 ms) after the
      OS reports the worker exited. This is what realtime.py controls: its is_alive()
      check runs every tick. A negative value means the runtime detected the death before
      the OS reported the exit, for example through the receiver seeing the worker's pipe
      close ("(result pipe closed)", recorded as detected_by); that passes.
    - Platform detection (measured, not gated): kill request to OS-reported exit. It
      depends on the OS: about 10 ms on Linux, 50-130 ms on the Windows laptop.
    - End to end (gated): kill request to stop within the 400 ms image-age limit, the
      outer bound that the freshness watchdog enforces anyway.

    Until 2026-10-03 the gate was kill request to stop <= 50 ms. It was changed after the
    Windows runs showed the runtime reacting within 3 ms of the exit while Windows took
    50-130 ms to report it; see docs/FAULT_INJECTION.md.
    """
    died, killed = marks.get("died"), marks.get("killed")
    stopped = None if event is None else event["stopped_s"]
    if stopped is not None and killed is not None:
        measured["stop_after_kill_ms"] = 1000 * (stopped - killed)
    if died is not None and killed is not None:
        measured["exit_after_kill_ms"] = 1000 * (died - killed)
    if stopped is not None and died is not None:
        measured["stop_after_exit_ms"] = 1000 * (stopped - died)
    kill_ms, exit_ms, reaction_ms = (measured.get(k) for k in ("stop_after_kill_ms", "exit_after_kill_ms",
                                                                  "stop_after_exit_ms"))
    check(checks, "runtime reaction: stop within one control deadline after the OS reported the worker exited",
          reaction_ms is not None and kill_ms is not None and kill_ms >= 0
          and reaction_ms <= 1000 * CONTROL_DEADLINE_S,
          "exit never observed" if exit_ms is None else
          f"OS reported the exit {exit_ms:.2f} ms after the kill request; stop {reaction_ms:.2f} ms after the exit")
    check(checks, "end to end: stop within the 400 ms image-age limit after the kill request",
          kill_ms is not None and 0 <= kill_ms <= 1000 * AGE_LIMIT_S,
          "no stop" if kill_ms is None else f"{kill_ms:.2f} ms")


def sensor_worker_crash(actuator=None):
    """REQ-06: the sensor worker process dies while the arm moves."""
    from realtime import RuntimeConfig
    marks = dict(killed=None, died=None)

    def until(session):
        if marks["killed"] is None and moving(session):
            kill_and_watch(session, marks)
        return marks["killed"] is not None and session.snapshot().get("status") == "runtime_error"

    report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05, stale_resume_s=0), until,
                         actuator=actuator)
    checks = []
    event, measured = common_measurements(report, "worker_failed", checks, runtime_error_expected=True)
    check(checks, "stop reason worker_failed recorded", event is not None)
    error = report.get("error") or ""
    # Which path stopped the arm: the per-tick is_alive() check, or the receiver seeing the
    # worker's pipe close (which can come before the OS reports the process exited).
    measured["detected_by"] = "result_pipe_closed" if "(result pipe closed)" in error else (
        "is_alive" if "Sensor process exited" in error else "other")
    measured["error_tail"] = error[-160:]
    detection_checks(event, marks, measured, checks)
    return record("sensor-worker-crash", ["REQ-06"], "realtime.py: worker not alive -> stop('worker_failed'), then the "
                  "runtime ends with an error report",
                  "the sensor worker process is killed while the arm moves",
                  "zero command within one control deadline of the OS-reported exit, and within the 400 ms image-age limit of the kill request; the runtime then ends with its error report", measured, checks,
                  note="The kill lands at a random moment. A kill in the middle of a result transfer (the "
                  "defect found by the first campaign, now fixed by the receiver thread) is targeted by "
                  "sensor-worker-partial-message and sensor-result-read-blocked. The runtime ends right after "
                  "the stop, so the simulated arm's braking is not measured.")


# ----------------------------------------------------------------------------- result pipe

PARTIAL_TRIGGER_ENV = "VS_FAULT_PARTIAL_RESULT_TRIGGER"


def partial_result_worker(requests, results, shutdown, *args, **kwargs):
    """Spawned test worker: the real sensor worker, except that once the trigger file
    exists, its next frame is left half-written in the results pipe.

    That is the state a worker killed in the middle of a transfer leaves behind. On
    POSIX, multiprocessing frames each message as a 4-byte length followed by the
    pickle, so the reader has the length and then waits for bytes that never come.
    The worker then idles until it is killed. `<trigger>.written` records the moment
    the half message is in the pipe.
    """
    import struct
    from multiprocessing.reduction import ForkingPickler
    import realtime
    trigger = Path(os.environ[PARTIAL_TRIGGER_ENV])
    put_latest = realtime.put_latest

    def faulty_put_latest(queue, value):
        if queue is results and value.get("kind") == "frame" and trigger.exists():
            data = bytes(ForkingPickler.dumps(value))
            pending = memoryview(struct.pack("!i", len(data)) + data[:len(data) // 2])
            descriptor = results._writer.fileno()
            while pending:
                pending = pending[os.write(descriptor, pending):]
            Path(f"{trigger}.written").write_text(f"{time.perf_counter()!r} {len(data)}")
            while True:
                time.sleep(1)
        return put_latest(queue, value)

    realtime.put_latest = faulty_put_latest
    realtime.sensor_worker(requests, results, shutdown, *args, **kwargs)


def _stuck_result_read(identifier, fault, inject, injected_at, receiver_expectation, coverage_note):
    """REQ-06 and REQ-01: a result read that never completes, then the worker dies.

    inject() starts the fault while the arm moves; injected_at() returns its time once
    it is in place. The worker stays alive for one freshness window, so the watchdog
    must still act, and is then killed.
    """
    from realtime import RuntimeConfig
    marks = dict(injected=None, ticks=None, ticks_after=None, paused=None, killed=None, died=None)

    def until(session):
        state = session.snapshot()
        counts = state.get("counters") or {}
        now = time.perf_counter()
        if marks["injected"] is None:
            if marks["ticks"] is None and moving(session):
                inject()
                marks["ticks"] = -1
            elif marks["ticks"] == -1 and injected_at() is not None:
                marks["injected"], marks["ticks"] = injected_at(), counts.get("control_ticks", 0)
        elif marks["killed"] is None:
            # Kill once the freshness watchdog has paused the arm (hold setting), or after
            # one second if it never does (the old runtime's control loop was blocked).
            if state.get("status") == "paused" or now - marks["injected"] > 1.0:
                marks["paused"] = state.get("status") == "paused"
                marks["ticks_after"] = counts.get("control_ticks", 0) - marks["ticks"]
                kill_and_watch(session, marks)
        return marks["killed"] is not None and state.get("status") == "runtime_error"

    report = run_session(RuntimeConfig(max_age_s=AGE_LIMIT_S, transport_s=.05), until, timeout_s=10)
    checks = []
    event, measured = common_measurements(report, "worker_failed", checks, runtime_error_expected=True)
    error = report.get("error") or ""
    measured["detected_by"] = "result_pipe_closed" if "(result pipe closed)" in error else (
        "is_alive" if "Sensor process exited" in error else "other")
    measured.update(fault_injected=marks["injected"] is not None, control_ticks_after_fault=marks["ticks_after"],
                    close_s=report["close_s"], receiver=report.get("receiver"),
                    error_tail=(report.get("error") or "")[-160:])
    check(checks, "the fault was in place while the arm moved", marks["injected"] is not None)
    check(checks, "the control loop kept running after the read blocked",
          (marks["ticks_after"] or 0) >= 50, f"{marks['ticks_after']} control ticks before the kill")
    pause = next((e for e in report["events"] if e["kind"] == "pause" and marks["injected"]
                  and e["at_s"] >= marks["injected"]), None)
    if pause is not None:
        measured["pause_after_deadline_ms"] = 1000 * (pause["at_s"] - pause["deadline_s"])
    check(checks, "the freshness watchdog paused the arm on time while the worker was alive",
          pause is not None and 0 <= measured["pause_after_deadline_ms"] <= 1000 * CONTROL_DEADLINE_S,
          "no pause event" if pause is None else f"{measured['pause_after_deadline_ms']:.2f} ms after the 400 ms limit")
    check(checks, "stop reason worker_failed recorded", event is not None)
    detection_checks(event, marks, measured, checks)
    check(checks, "the error report names the exited sensor process",
          "Sensor process exited" in (report.get("error") or ""), measured["error_tail"])
    check(checks, "shutdown completed", report["close_s"] < 5, f"close() took {report['close_s']:.3f} s")
    receiver = report.get("receiver") or {}
    check(checks, receiver_expectation[0], receiver_expectation[1](receiver), str(receiver))
    return record(identifier, ["REQ-06", "REQ-01"], "realtime.py: the result pipe is read only by the receiver "
                  "thread; the control loop reads its local queue, detects the dead worker (is_alive) and stops "
                  "with 'worker_failed'", fault,
                  "the control loop keeps ticking; the watchdog pauses at the 400 ms image age; zero command within "
                  "one control deadline of the OS-reported exit (and 400 ms of the kill request); the runtime ends "
                  "with its error report; shutdown completes",
                  measured, checks, coverage="full", note=coverage_note)


def sensor_worker_partial_message(actuator=None):
    """The original defect: the worker leaves half a frame in the results pipe, then dies."""
    import realtime
    directory = tempfile.mkdtemp(prefix="vs-partial-")
    trigger = Path(directory, "trigger")
    written = Path(f"{trigger}.written")

    def injected_at():
        if not written.exists():
            return None
        text = written.read_text().split()
        return float(text[0]) if text else None

    previous = os.environ.get(PARTIAL_TRIGGER_ENV)
    os.environ[PARTIAL_TRIGGER_ENV] = str(trigger)
    try:
        with replaced(realtime, "sensor_worker", partial_result_worker):
            return _stuck_result_read(
                "sensor-worker-partial-message",
                "the sensor worker writes a frame's length header and half of its bytes into the real results "
                "pipe, stays alive for one freshness window, then is killed",
                lambda: trigger.write_text("go"), injected_at,
                # "during message": the read was stuck inside the half message, not merely
                # ended by a clean exit (which ends a waiting read with a bare EOFError).
                ("the receiver's read, stuck inside the half message, ended once the worker died",
                 lambda r: r.get("stuck_at_close") is False
                 and "end of file during message" in str(r.get("ended", ""))),
                "POSIX pipe framing only. Python's Windows pipes are message-oriented, so this exact partial "
                "message is not produced there; sensor-result-read-blocked covers the same control-loop "
                "behaviour on every platform.")
    finally:
        if previous is None:
            os.environ.pop(PARTIAL_TRIGGER_ENV, None)
        else:
            os.environ[PARTIAL_TRIGGER_ENV] = previous
        for path in (written, trigger):
            path.unlink(missing_ok=True)
        os.rmdir(directory)


def sensor_result_read_blocked(actuator=None):
    """Portable: the parent's result read blocks forever on a frame, then the worker dies."""
    import multiprocessing.queues as mp_queues
    armed, release = threading.Event(), threading.Event()
    blocked = {}
    original = mp_queues.Queue.get

    def get(self, *args, **kwargs):
        value = original(self, *args, **kwargs)
        if armed.is_set() and isinstance(value, dict) and value.get("kind") == "frame":
            blocked.setdefault("at", time.perf_counter())
            release.wait()  # Never set during the session: the read does not return.
        return value

    try:
        with replaced(mp_queues.Queue, "get", get):
            return _stuck_result_read(
                "sensor-result-read-blocked",
                "the parent's read of the next frame from the results queue never returns (any platform); the "
                "worker stays alive for one freshness window, then is killed",
                armed.set, lambda: blocked.get("at"),
                ("shutdown completed although the receiver thread stayed blocked",
                 lambda r: r.get("stuck_at_close") is True),
                "Simulates a read that never completes inside the parent process rather than producing one in "
                "the pipe; sensor-worker-partial-message produces the real pipe state on POSIX.")
    finally:
        release.set()


SCENARIOS = dict(
    control_loop_stall=control_loop_stall,
    control_loop_stall_near_limit=control_loop_stall_near_limit,
    control_loop_stall_below_limit=control_loop_stall_below_limit,
    physics_gap_cap=physics_gap_cap,
    controller_stall=controller_stall,
    clock_step_back=clock_step_back,
    inference_failure=inference_failure,
    collision_block=collision_block,
    run_timeout=run_timeout,
    sensor_worker_crash=sensor_worker_crash,
    sensor_result_read_blocked=sensor_result_read_blocked,
    watchdog_stop=watchdog_stop,
    speed_clip=speed_clip,
)
if sys.platform != "win32":  # Python's Windows pipes are message-oriented: no partial message.
    SCENARIOS["sensor_worker_partial_message"] = sensor_worker_partial_message
