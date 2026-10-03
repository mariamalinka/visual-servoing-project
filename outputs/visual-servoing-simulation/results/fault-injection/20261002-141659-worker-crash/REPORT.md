# Fault-injection campaign

Run 2026-10-02T12:47:46.707827+00:00 on Linux x86_64, Python 3.12.3, actuator profile `default`, 30 repeat(s) per scenario (deterministic scenarios once). Simulation only: timings depend on this computer, and nothing here is evidence about a real robot.

Source fingerprint: 87 files hashed in `summary.json` (`realtime.py` 3c9476102365, `simulation.py` ac799ddd1b09).

Before this campaign every path below was IMPLEMENTED only: the code existed, but no test triggered it.

**Result:** 2 VERIFIED, 1 NOT VERIFIED.

## Safety paths

| Path | Requirement | Fault injected | Expected reaction | Passed | Status |
|---|---|---|---|---:|---|
| sensor-worker-crash | REQ-06 | the sensor worker process is killed while the arm moves | zero command within one tick; the runtime then ends with its error report | 29/30 | **NOT VERIFIED** |
| sensor-worker-partial-message | REQ-06, REQ-01 | the sensor worker writes a frame's length header and half of its bytes into the real results pipe, stays alive for one freshness window, then is killed | the control loop keeps ticking; the watchdog pauses at the 400 ms image age; zero command within one control deadline of the kill; the runtime ends with its error report; shutdown completes | 30/30 | **VERIFIED** |
| sensor-result-read-blocked | REQ-06, REQ-01 | the parent's read of the next frame from the results queue never returns (any platform); the worker stays alive for one freshness window, then is killed | the control loop keeps ticking; the watchdog pauses at the 400 ms image age; zero command within one control deadline of the kill; the runtime ends with its error report; shutdown completes | 30/30 | **VERIFIED** |

## Measured reaction (worst over the repeats)

| Path | Stop issued | Speed at the fault | Physical stop | Camera travel | Peak measured speed | Minimum clearance above margin: environment / robot parts | Minimum joint margin |
|---|---:|---:|---:|---:|---:|---:|---:|
| sensor-worker-crash | 20.8 ms after the worker died | — | — | — | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| sensor-worker-partial-message | 19.0 ms after the worker died | 0.049 rad/s | — | 0.62 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| sensor-result-read-blocked | 20.2 ms after the worker died | 0.050 rad/s | — | 0.62 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |

## Notes per path

- **sensor-worker-crash** — mechanism: realtime.py: worker not alive -> stop('worker_failed'), then the runtime ends with an error report. Test: `tests/test_fault_injection.py::test_sensor_worker_crash_stops_the_arm`. Failed checks: stop event recorded; stop reason worker_failed recorded; the robot was moving before the fault; the runtime ended with its error report after the stop. Note: The kill lands at a random moment. A kill in the middle of a result transfer (the defect found by the first campaign, now fixed by the receiver thread) is targeted by sensor-worker-partial-message and sensor-result-read-blocked. The runtime ends right after the stop, so the simulated arm's braking is not measured.
- **sensor-worker-partial-message** — mechanism: realtime.py: the result pipe is read only by the receiver thread; the control loop reads its local queue, detects the dead worker (is_alive) and stops with 'worker_failed'. Test: `tests/test_fault_injection.py::test_half_sent_frame_cannot_block_the_control_loop`. Note: POSIX pipe framing only. Python's Windows pipes are message-oriented, so this exact partial message is not produced there; sensor-result-read-blocked covers the same control-loop behaviour on every platform.
- **sensor-result-read-blocked** — mechanism: realtime.py: the result pipe is read only by the receiver thread; the control loop reads its local queue, detects the dead worker (is_alive) and stops with 'worker_failed'. Test: `tests/test_fault_injection.py::test_blocked_result_read_cannot_block_the_control_loop`. Note: Simulates a read that never completes inside the parent process rather than producing one in the pipe; sensor-worker-partial-message produces the real pipe state on POSIX.

## Status definitions

- **IMPLEMENTED:** the code exists; nothing triggers it.
- **TESTED:** an automated test triggers it; no saved evidence in this run.
- **VERIFIED:** every repeat produced the documented reaction with all checks passing, and the injected fault is the fault the requirement is about.
- **PARTIALLY VERIFIED:** every repeat passed, but the injection covers only part of the requirement.
- **NOT VERIFIED:** at least one repeat did not produce the documented reaction.

Each repeat's full record (fault, expected, measured, every check) is in `summary.json`. The physical stop uses the actuator model (docs/ACTUATOR_MODEL.md); clearance and joint margin come from the per-step safety record (safety_metrics.py).
