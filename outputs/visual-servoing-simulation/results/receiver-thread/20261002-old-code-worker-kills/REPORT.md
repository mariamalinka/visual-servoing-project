# Fault-injection campaign

Run 2026-10-02T12:06:36.897501+00:00 on Linux x86_64, Python 3.12.3, actuator profile `default`, 30 repeat(s) per scenario (deterministic scenarios once). Simulation only: timings depend on this computer, and nothing here is evidence about a real robot.

Source fingerprint: 87 files hashed in `summary.json` (`realtime.py` 521a6998689a, `simulation.py` ac799ddd1b09).

Before this campaign every path below was IMPLEMENTED only: the code existed, but no test triggered it.

**Result:** 1 NOT VERIFIED.

## Safety paths

| Path | Requirement | Fault injected | Expected reaction | Passed | Status |
|---|---|---|---|---:|---|
| sensor-worker-crash | REQ-06 | the sensor worker process is killed while the arm moves | zero command within one tick; the runtime then ends with its error report | 29/30 | **NOT VERIFIED** |

## Measured reaction (worst over the repeats)

| Path | Stop issued | Speed at the fault | Physical stop | Camera travel | Peak measured speed | Minimum clearance above margin: environment / robot parts | Minimum joint margin |
|---|---:|---:|---:|---:|---:|---:|---:|
| sensor-worker-crash | 21.7 ms after the worker died | — | — | — | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |

## Notes per path

- **sensor-worker-crash** — mechanism: realtime.py: worker not alive -> stop('worker_failed'), then the runtime ends with an error report. Test: `tests/test_fault_injection.py::test_sensor_worker_crash_stops_the_arm`. Failed checks: scenario ran. Partial coverage: The kill lands at a random moment. A kill in the middle of a result transfer can leave a partial message that blocks the control thread's mailbox read (a known defect, docs/FAULT_INJECTION.md); this scenario does not target that moment. The runtime ends right after the stop, so the simulated arm's braking is not measured.

## Status definitions

- **IMPLEMENTED:** the code exists; nothing triggers it.
- **TESTED:** an automated test triggers it; no saved evidence in this run.
- **VERIFIED:** every repeat produced the documented reaction with all checks passing, and the injected fault is the fault the requirement is about.
- **PARTIALLY VERIFIED:** every repeat passed, but the injection covers only part of the requirement.
- **NOT VERIFIED:** at least one repeat did not produce the documented reaction.

Each repeat's full record (fault, expected, measured, every check) is in `summary.json`. The physical stop uses the actuator model (docs/ACTUATOR_MODEL.md); clearance and joint margin come from the per-step safety record (safety_metrics.py).
