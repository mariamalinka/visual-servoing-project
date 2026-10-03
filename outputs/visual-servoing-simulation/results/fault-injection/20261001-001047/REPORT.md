# Fault-injection campaign

Run 2026-09-30T22:10:47.267804+00:00 on Linux x86_64, Python 3.12.3, actuator profile `default`, 5 repeat(s) per scenario (deterministic scenarios once). Simulation only: timings depend on this computer, and nothing here is evidence about a real robot.

Source fingerprint: 87 files hashed in `summary.json` (`realtime.py` 521a6998689a, `simulation.py` ac799ddd1b09).

Before this campaign every path below was IMPLEMENTED only: the code existed, but no test triggered it.

**Result:** 10 VERIFIED, 2 PARTIALLY VERIFIED.

## Safety paths

| Path | Requirement | Fault injected | Expected reaction | Passed | Status |
|---|---|---|---|---:|---|
| control-loop-stall | REQ-03 | the control thread blocks for 150 ms inside one physics call while moving | alignment stops as control_overrun on the next tick; zero command; no motion afterwards | 5/5 | **VERIFIED** |
| control-loop-stall-near-limit | REQ-03 | the control thread blocks for 55 ms inside one physics call while moving | alignment stops as control_overrun on the next tick; zero command; no motion afterwards | 5/5 | **VERIFIED** |
| control-loop-stall-below-limit | REQ-03 | the control thread blocks for 35 ms (below the 50 ms deadline) while moving | no stop: the alignment continues and converges | 5/5 | **VERIFIED** |
| physics-gap-cap | REQ-08 | the control thread blocks for 150 ms inside one physics call while moving | the next physics call is capped at 50 ms and the rest of the gap is counted as discarded | 5/5 | **VERIFIED** |
| controller-stall | REQ-03 | the 6th controller computation takes 80 ms longer | stop as control_overrun without applying the late command | 5/5 | **VERIFIED** |
| clock-step-back | REQ-06 | time.perf_counter as seen by the control thread jumps back 1.0 s while moving | alignment stops as clock_error; zero command | 5/5 | **VERIFIED** |
| inference-failure | REQ-06 | every frame after 0.6 s returns an 'inference_failed' observation (RuntimeConfig.inference_failure = 1) | alignment stops as inference_failed; no command from the failed result | 5/5 | **PARTIALLY VERIFIED** |
| collision-block | REQ-15 | the collision guard's decision is replaced by 'blocked' for every motion request once moving | command zero in the same physics step; alignment stops as collision_blocked | 5/5 | **PARTIALLY VERIFIED** |
| run-timeout | REQ-06, REQ-16 | max_run_s = 1.0 s, shorter than an alignment (about 4 s) | alignment stops as run_timeout at the limit; zero command | 5/5 | **VERIFIED** |
| sensor-worker-crash | REQ-06 | the sensor worker process is killed while the arm moves | zero command within one tick; the runtime then ends with its error report | 5/5 | **VERIFIED** |
| watchdog-stop | REQ-01 | the sensor worker stalls 0.8 s, 0.6 s after Align (stop setting, 400 ms image-age limit) | zero command at most 50 ms after the image age reaches 400 ms; stays stopped | 5/5 | **VERIFIED** |
| speed-clip | REQ-13 | a 1.5 rad/s command on all six joints (2.5x the limit) | command clipped to 0.6 rad/s; measured joint speed never above 0.6 rad/s | 1/1 | **VERIFIED** |

## Measured reaction (worst over the repeats)

| Path | Stop issued | Speed at the fault | Physical stop | Camera travel | Peak measured speed | Minimum clearance above margin: environment / robot parts | Minimum joint margin |
|---|---:|---:|---:|---:|---:|---:|---:|
| control-loop-stall | 2.9 ms after the stalled tick | 0.052 rad/s | 58 ms | 0.64 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| control-loop-stall-near-limit | 2.5 ms after the stalled tick | 0.050 rad/s | 58 ms | 0.64 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| control-loop-stall-below-limit | no stop (largest loop gap 38.5 ms) | — | — | — | — | — / — | — |
| physics-gap-cap | 2.6 ms after the stalled tick | 0.055 rad/s | 60 ms | 0.71 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| controller-stall | 0.4 ms after the late computation | 0.080 rad/s | 70 ms | 1.26 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| clock-step-back | 0.4 ms after the clock jump | 0.056 rad/s | 58 ms | 0.66 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| inference-failure | 50.3 ms after the failed result | 0.055 rad/s | 60 ms | 0.71 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| collision-block | 0.5 ms after the block | 0.056 rad/s | 60 ms | 0.77 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| run-timeout | 3.7 ms after the run limit | 0.037 rad/s | 52 ms | 0.40 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| sensor-worker-crash | 13.4 ms after the worker died | — | — | — | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| watchdog-stop | 2.4 ms after the image-age limit | 0.055 rad/s | 60 ms | 0.73 mm | 0.082 rad/s | 3.0 mm / 9.0 mm | 0.995 rad |
| speed-clip | — | — | — | — | 0.592 rad/s | — / — | — |

## Notes per path

- **control-loop-stall** — mechanism: realtime.py: gap > max_control_gap_s at the start of a tick -> stop('control_overrun'). Test: `tests/test_fault_injection.py::test_control_loop_stall_stops_with_control_overrun`. Note: In the simulation the arm cannot move while the control thread is blocked, because physics runs in that thread. A real drive would keep executing the last command for the whole stall (here about speed x stall, e.g. 0.05 rad/s x 0.15 s = 7.5 mrad).
- **control-loop-stall-near-limit** — mechanism: realtime.py: gap > max_control_gap_s at the start of a tick -> stop('control_overrun'). Test: `tests/test_fault_injection.py::test_stall_just_over_the_deadline_still_stops`. Note: In the simulation the arm cannot move while the control thread is blocked, because physics runs in that thread. A real drive would keep executing the last command for the whole stall (here about speed x stall, e.g. 0.05 rad/s x 0.15 s = 7.5 mrad).
- **control-loop-stall-below-limit** — mechanism: realtime.py: gap > max_control_gap_s. Test: `tests/test_fault_injection.py::test_stall_below_the_deadline_does_not_stop`.
- **physics-gap-cap** — mechanism: realtime.py: sim.advance(min(gap, max_control_gap_s)); discarded_physics_s. Test: `tests/test_fault_injection.py::test_physics_gap_is_capped_and_counted`. Note: In the simulation the arm cannot move while the control thread is blocked, because physics runs in that thread. A real drive would keep executing the last command for the whole stall (here about speed x stall, e.g. 0.05 rad/s x 0.15 s = 7.5 mrad).
- **controller-stall** — mechanism: realtime.py: computed - last_tick > max_control_gap_s after controller.update() -> stop('control_overrun'). Test: `tests/test_fault_injection.py::test_slow_controller_computation_stops_without_applying_its_command`.
- **clock-step-back** — mechanism: realtime.py: CommandLease.failure() -> 'clock_error' when now is before the lease start or the last capture. Test: `tests/test_fault_injection.py::test_clock_stepping_back_stops_with_clock_error`.
- **inference-failure** — mechanism: realtime.py: observation.reason == 'inference_failed' -> stop('inference_failed'). Test: `tests/test_fault_injection.py::test_failed_inference_stops_the_alignment`. Partial coverage: The failure is injected in the sensor worker; a real Learned GPU RuntimeError inside the matcher is not reproduced.
- **collision-block** — mechanism: simulation.py: a 'blocked' guard decision latches the request to zero; realtime.py: skip_blocked_motion() is False while tracking -> stop('collision_blocked'). Test: `tests/test_fault_injection.py::test_collision_block_stops_the_realtime_alignment`. Partial coverage: The guard decision is injected. Geometric blocking is tested in the simulation (tests/test_collision.py) but not in the realtime runtime; the search/recovery skip branch is not exercised.
- **run-timeout** — mechanism: realtime.py: CommandLease.failure() -> 'run_timeout'. Test: `tests/test_fault_injection.py::test_run_limit_ends_a_moving_alignment`.
- **sensor-worker-crash** — mechanism: realtime.py: worker not alive -> stop('worker_failed'), then the runtime ends with an error report. Test: `tests/test_fault_injection.py::test_sensor_worker_crash_stops_the_arm`. Note: The runtime ends right after the stop, so the simulated arm's braking is not measured; on hardware the drive and REQ-24 decide what happens next.
- **watchdog-stop** — mechanism: realtime.py: CommandLease.failure() -> 'stale_camera' -> stop(). Test: `tests/test_fault_injection.py::test_watchdog_stop_reference`.
- **speed-clip** — mechanism: simulation.py: np.clip in command_velocity; ctrlrange in scene.xml. Test: `tests/test_fault_injection.py::test_speed_command_above_the_limit_is_clipped`. Deterministic: run once.

## Status definitions

- **IMPLEMENTED:** the code exists; nothing triggers it.
- **TESTED:** an automated test triggers it; no saved evidence in this run.
- **VERIFIED:** every repeat produced the documented reaction with all checks passing, and the injected fault is the fault the requirement is about.
- **PARTIALLY VERIFIED:** every repeat passed, but the injection covers only part of the requirement.
- **NOT VERIFIED:** at least one repeat did not produce the documented reaction.

Each repeat's full record (fault, expected, measured, every check) is in `summary.json`. The physical stop uses the actuator model (docs/ACTUATOR_MODEL.md); clearance and joint margin come from the per-step safety record (safety_metrics.py).
