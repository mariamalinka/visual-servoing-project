# Requirements Verification and Safety Traceability

This page connects each requirement the project defines or clearly relies on to five
things:
- the hazard it prevents;
- the mechanism that enforces it;
- the automated test that verifies it;
- the latest saved evidence;
- an honest status.

The requirements come from the existing documentation, configuration files and
acceptance criteria. None is new. Every test named below exists in
`outputs/visual-servoing-simulation/tests/` (written `tests/`) or `tools/`. Every
result folder is under `outputs/visual-servoing-simulation/results/`.

**Scope.** Everything here is verified in simulation (MuJoCo physics, rendered
camera, one Windows laptop). Nothing is verified on a physical robot, and nothing
here is a certification. "Stop" means the controller commands zero joint velocity.
Since 2026-09-30 the simulation includes an actuator dynamics model with assumed
acceleration, deceleration and jerk limits ([actuator model](ACTUATOR_MODEL.md)).
After a zero command the simulated arm therefore takes a measurable time and
distance to stop. That physical response is measured and reported separately from
the command. The model's limits are assumptions, not measurements of a real drive.

**Status definitions**

| Status | Meaning |
|---|---|
| VERIFIED | Automated tests exercise the enforcing mechanism, and saved evidence shows the requirement met under the stated conditions. |
| PARTIALLY VERIFIED | Some verification exists, but a part of the requirement has no test, the evidence covers only some conditions, or the evidence predates the current behaviour. |
| NOT VERIFIED | No automated test and no saved evidence that the mechanism works, or the mechanism is not implemented. |

The Status column, the summary counts, the fresh-restart count and the
[machine-checked status](#machine-checked-status) section are generated from the saved results by
`tools/traceability.py`; see [keeping this page current](#keeping-this-page-current).

<!-- BEGIN GENERATED: summary (do not edit; regenerate with python -B tools/traceability.py --write) -->
**Summary:** 26 requirements. 13 are VERIFIED, 12 are PARTIALLY VERIFIED and 1 is NOT VERIFIED (REQ-24).
<!-- END GENERATED: summary -->
The review was made on 2026-09-29 from the code, tests and saved results.
REQ-25 and REQ-26 and the actuator notes in REQ-01, REQ-05 and REQ-12 were added on
2026-09-30 with the actuator model. On 2026-10-01 the safety envelope record and the
fault-injection campaign ([fault injection](FAULT_INJECTION.md)) moved REQ-03 and REQ-08
to VERIFIED and added evidence to REQ-06, REQ-10 to REQ-13, REQ-15 and REQ-16. On
2026-10-02 the control-loop freeze after a sensor-worker crash (REQ-06) was fixed by
receiving sensor results on a separate thread. The regression scenarios pass 35/35
each on the fixed runtime. The re-run of the whole campaign passed 154 of 156 repeats;
the two failures never injected their fault (gap 2). No status changed: REQ-06 stays PARTIALLY
VERIFIED because the Learned GPU failure is still injected, not reproduced, and the
laptop acceptance evidence predated this runtime (gap 3). On 2026-10-03 laptop runs on the fixed runtime added evidence:
- the acceptance test (`results/acceptance-test/20261002-233959`) passed with the safety envelope recorded;
- the fault campaign added Windows results.

That moved REQ-01 and REQ-19 to VERIFIED. A second laptop acceptance run, after a fresh restart
(`results/acceptance-test/20261003-134526`, 9 min uptime), **failed**. SIFT had one control deadline miss: a 59.5 ms loop gap 45 ms after
an Align in the first session. It stopped that alignment with `control_overrun` before the arm moved, so
the run had 148/149 successes and 1 failed alignment. REQ-03 and REQ-19 therefore went back to PARTIALLY
VERIFIED. A third run (`results/acceptance-test/20261003-150343`, 87 min uptime, so not a fresh restart) passed with 0 misses. It is
the first to save the per-cycle stall breakdown; its largest loop gap was 11.8 ms (sustained Learned GPU session). A fourth run after a fresh
restart (`results/acceptance-test/20261003-154640`, 3 min uptime) also passed with 0 misses; its largest loop gap was 23.6 ms. Under the
procedure's conditions there was then one pass and one failure, and the failure is unexplained. A fifth run (`results/acceptance-test/20261003-161751`,
34 min uptime) passed as well. That makes 2 passes and 1 failure under the procedure's conditions (both passes
followed the same restart). At 95%
confidence (Clopper–Pearson, as in [statistics](STATISTICS.md)), 1 miss in 3 runs puts the per-run
chance of a miss somewhere between 0.8% and 90.6%, so the miss is not shown to be gone.

Four more runs that evening each started 2 min after their own restart. The first,
`results/acceptance-test/20261003-181233`, **failed**: one SIFT control deadline miss (a 62.0 ms gap) in the
sustained session stopped a moving alignment with `control_overrun` 5.4 mm from the goal, and the same moment set
the SIFT processing maximum to 275.6 ms (gate 250 ms). The saved stage breakdown shows that the control loop and the separate sensor-worker process slowed down together for about 0.5 s: every control cycle took 13–62 ms, almost all of it inside the physics and collision step, with no garbage collection and little time descheduled, and the thread executed only about 56 million CPU cycles in the 61 ms cycle (about a quarter of what 3.8 GHz allows); in the same 0.5 s SIFT feature extraction in the worker took 239 and 219 ms instead of about 27 ms.
Two processes slowing at once points to something outside the project's code taking the CPU two minutes after
boot. The logged CPU frequency and temperature (1.3 s samples) show nothing, so the source is not identified.
The next three runs (`20261003-184433`, `20261003-191452`, `20261003-200656`) passed with 0 misses. Runs after a
fresh restart now stand at 5 passes and 2 failures out of 7; at 95% confidence the per-run chance of a miss is
between 3.7% and 71.0%. The failure reset the pre-declared count.

Two more runs that night (`20261003-231335`, `20261003-234632`), each about 4 min after its own restart,
passed with 0 misses. That completed 5 consecutive fresh-restart passes, each after its own restart, so under the
pre-declared criterion REQ-03 and REQ-19 returned to VERIFIED on 2026-10-04 (computed by `tools/traceability.py`).
This does not explain the two misses. Runs after a fresh restart stand at 7 passes and 2 failures out of 9; at
95% confidence the per-run chance of a miss is between 2.8% and 60.0%, so a deadline miss can still fail an
individual acceptance run on this laptop (see the hazard rows for REQ-03 and REQ-19).

**Pre-declared criterion (set 2026-10-03, before further runs):** REQ-03 and REQ-19 return to VERIFIED
after either of these, with every run kept as evidence:
- 5 consecutive fresh-restart acceptance runs (≤ 60 min uptime, at most one run per restart) pass with 0 misses (<!-- BEGIN GENERATED: fresh-streak (do not edit; regenerate with python -B tools/traceability.py --write) -->5 of 5 so far: `20261003-184433`, `20261003-191452`, `20261003-200656`, `20261003-231335`, `20261003-234632`<!-- END GENERATED: fresh-streak -->);
- the cause of the miss in `20261003-134526` is found, fixed and re-tested. The worker-failure detection gate was split
into runtime reaction, platform detection and end-to-end time, after Windows proved to
take 50–130 ms to report a killed process ([fault injection](FAULT_INJECTION.md)).

On 2026-10-03 the statuses became machine-checked (`tools/traceability.py`). The check moved
REQ-07 to PARTIALLY VERIFIED: its saved runs record 0 expired requests and 0 mailbox drops, so
they never exercised the drop path, and the tests are its only verification.

## Traceability table

Abbreviations:
- `RT` = `tests/test_realtime.py`
- `AT` = `tools/test_acceptance_test.py`
- **acceptance run** = `results/acceptance-test/20260929-011129`: standard profile,
  hold-and-resume default, 150 SIFT and 147 Learned GPU alignments.

### Runtime and watchdog safety (`realtime.py`)

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-01 | When the newest accepted camera image reaches the age limit, the robot is commanded to zero velocity no later than one control deadline (≤ 50 ms) after the limit. The limit is 400 ms in the acceptance, margin and latency tests; the runtime and app default is 250 ms. | Motion continues on a view that no longer matches reality. | `CommandLease.failure()` checked on every 2 ms tick and after control computation. `stop()` / `pause()` command zero velocity. | RT `test_expired_initial_lease_is_independent_of_worker`, `test_blocking_inference_cannot_pause_watchdog_or_rearm_motion` (stop setting, lateness < 50 ms), `test_watchdog_pause_zeroes_motion_on_time_and_resumes_on_fresh_images` and `test_watchdog_pause_ends_the_run_when_images_do_not_return` (hold default, lateness < 50 ms). `run.cmd --latency-study` gate ≤ 50 ms. | `results/latency/20260921-validation`: worst lateness **2.034 ms** over the injected-stall trials. That run used the stop setting. Fault-injection campaign `results/fault-injection/20261001-001047` (stop setting, cloud workspace): zero command at most 2.4 ms after the limit in 5/5 runs; laptop campaign `results/fault-injection/20261003-001323`: at most 2.3 ms, 3/3. **Hold default (since 2026-10-02):** the worker-failure scenarios hold the arm at the 400 ms limit while the worker is still alive. The pause came at most 1.81 ms after the limit in 70 cloud runs (`results/fault-injection/20261002-141659-*`) and at most 2.12 ms in 33 laptop runs (`results/fault-injection/20261003-*`). **This requirement is about the command.** The physical standstill that follows is measured and reported, not required. It came 42–72 ms after the zero command in six realtime watchdog trips with the actuator model on (image age at standstill 442–474 ms). The worst case is 152 ms from 0.35 rad/s (`results/stop-response/20260930-004121`, [actuator model](ACTUATOR_MODEL.md)). | VERIFIED |
| REQ-02 | Commands come only from a result of the current alignment that was captured after the last accepted image and is still younger than the age limit. | A late or out-of-order result moves the robot toward where the target used to be. | `CommandLease.eligible()`; generation numbers; re-check after computation. `unsafe_motion_ticks` counts any motion without fresh feedback. | RT `test_duplicates_future_prestart_nan_and_reordered_frames_rejected`, `test_expired_delivery_cannot_extend_capture_age`, `test_stop_latches_and_rearm_rejects_previous_generation`; the process tests assert `unsafe_motion_ticks == 0`. Acceptance gate `unsafe_motion: 0` (AT `test_each_safety_counter_fails`). | Acceptance run: unsafe motion 0 / 0. Margin runs: 0 at every level. `results/latency/20260925-native-scheduling-after`: 0. | VERIFIED |
| REQ-03 | A control-loop gap or controller computation longer than 50 ms stops the alignment (`control_overrun`). Acceptance requires 0 misses. | The last command runs unsupervised while the loop is stalled. | Gap and compute-time checks in `RealtimeSession._run()`; `control_deadline_misses` counter; `native_scheduling.py` reduces stalls. | Fault injection (`tests/test_fault_injection.py`): `test_control_loop_stall_stops_with_control_overrun` (a 150 ms stall inside one control tick), `test_stall_just_over_the_deadline_still_stops` (55 ms), `test_stall_below_the_deadline_does_not_stop` (35 ms) and `test_slow_controller_computation_stops_without_applying_its_command` (an 80 ms controller computation). Harness: AT `test_each_safety_counter_fails` (synthetic counter), `test_production_limits_cannot_be_relaxed`. | Acceptance run: 0 misses; largest loop gap 38.6 ms (Learned GPU) and 11.1 ms (SIFT). Laptop acceptance run on the receiver-thread runtime (`results/acceptance-test/20261002-233959`): 0 misses for both methods. **Fresh-restart laptop run `results/acceptance-test/20261003-134526`: 1 miss (SIFT), so the acceptance FAILED.**<br>• The gap was 59.5 ms, starting 45 ms after the second Align of the first fresh-worker session.<br>• The next tick stopped the alignment with `control_overrun`, 104.6 ms after Align. No command had been applied yet, and the arm was not moving.<br>• The receiver had finished a frame 49 ms before the stop. That frame waited in the mailbox until the control thread ran again, so the control thread itself did not run for 59.5 ms.<br>• The harness did not save the per-cycle stage breakdown, so the cause (scheduling, GC or work in the tick) is unknown. The harness saves it since 2026-10-03.<br>This is a real, uninjected trigger of the mechanism.<br>Run `results/acceptance-test/20261003-150343` (87 min uptime): 0 misses for both methods, largest loop gap 11.8 ms (sustained Learned GPU; 9.1 ms in the first SIFT session), PASS. Fresh-restart run `results/acceptance-test/20261003-154640` (3 min uptime): 0 misses, largest loop gap 23.6 ms over 12 sessions, PASS. Fresh-restart run `results/acceptance-test/20261003-161751` (34 min): 0 misses, largest loop gap 14.3 ms, PASS. **Fresh-restart run `results/acceptance-test/20261003-181233` (2 min): 1 miss (SIFT), FAILED.** A 62.0 ms cycle in the sustained session stopped a moving alignment with `control_overrun`, 0 unsafe motion; the control loop and the separate sensor-worker process slowed down together for about 0.5 s: every control cycle took 13–62 ms, almost all of it inside the physics and collision step, with no garbage collection and little time descheduled, and the thread executed only about 56 million CPU cycles in the 61 ms cycle (about a quarter of what 3.8 GHz allows); in the same 0.5 s SIFT feature extraction in the worker took 239 and 219 ms instead of about 27 ms. Source not identified; a second real, uninjected trigger of the mechanism. Fresh-restart runs `20261003-184433`, `20261003-191452` and `20261003-200656` (2 min each, own restarts): 0 misses, largest loop gaps 27.4, 19.3 and 22.2 ms, PASS. Fresh-restart runs `20261003-231335` and `20261003-234632` (about 4 min each, own restarts): 0 misses, largest loop gaps 13.2 and 14.3 ms, PASS; this completed the pre-declared 5 consecutive passes. Laptop fault campaign `results/fault-injection/20261003-001323`: stalls stopped at most 3.7 ms after the stalled tick, 3/3 each. `20260925-native-scheduling-after` stress campaign: **2 misses** (Learned GPU). Fault-injection campaign `results/fault-injection/20261001-001047` (cloud workspace, 5 repeats each): a 150 ms and a 55 ms loop stall stopped at most 2.9 ms after the stalled tick; a 35 ms stall (negative control) did not stop and the alignment converged; a slow computation stopped at most 0.4 ms after it, and its command was never applied; 0 motion after the stop. In the simulation the arm cannot move during a stall (physics runs in the stalled thread); a real drive would keep executing the last command for the stall. A block that never returns is never detected by this check, by design (`REALTIME_CONTROL.md`); see REQ-24. The worker-crash freeze under REQ-06 was such a block in the result read; since 2026-10-02 that read runs on a separate receiver thread. | VERIFIED |
| REQ-04 | A stop is latched: the command stays zero and no late result restarts motion until a new Align. This is checked for 1.1 s after every stop. | Motion restarts after an operator or watchdog stop, when everyone assumes the robot is stationary. | `CommandLease.stop()` deactivates the lease, and a new generation is needed to move. `post_stop_motion_ticks` counter. Harness post-stop observation (`stop_stayed_latched`). | RT `test_blocking_inference_cannot_pause_watchdog_or_rearm_motion` (stays stopped after the late result arrives), `test_manual_stop_and_rearm_have_distinct_frame_generations`. Acceptance gates `post_stop_motion: 0`, `unlatched_stops: 0`. | Acceptance run: post-stop motion 0 / 0, unlatched stops 0 / 0. Stress campaign: "All stops stayed latched: True / True". | VERIFIED |
| REQ-05 | Hold and resume (default `stale_resume_s` = 2 s):<br>• the command stays zero during a hold;<br>• control resumes only on a newer, fresh image of the same alignment;<br>• with no such image for 2 s, the alignment ends as `stale_camera`.<br>Acceptance allows 0 alignments ended by the watchdog and holds of at most 5% of alignment time. | Resuming on outdated information, or an unbounded "running" hold that restarts much later. | `CommandLease.pause()` and `failure()`, `watchdog()`, resume only through `eligible()`, `paused_motion_ticks`. Harness pause budget. | RT `test_pause_holds_only_within_resume_window`, `test_pause_never_outlives_run_timeout_or_a_stop`, `test_hold_and_resume_is_the_default_and_stop_is_a_setting` and the two process pause tests. AT `test_pause_budget` (the harness verdict only). | Margin runs `20260928-234612-resume2000ms` and `20260929-110852`: 0 motion during holds and 0 alignments ended by the watchdog up to +150 ms, with up to 342 holds per level, all using the 2 s window. **No hold has run out the 2 s window in a real run.** The process test that ends a hold uses a 1 s window. Acceptance run: **0 holds occurred**, so the 5% budget was not exercised. All of this evidence predates the actuator model. With the model, a hold brakes the arm within about 0.1–0.15 s from 0.35 rad/s, and within 70 ms in a realtime hold-and-resume run at typical alignment speeds. A resume ramps up within the acceleration and jerk limits (stop-response study; RT hold test). Under heavy per-frame delay, each resume moved the arm for at most 18 ms before the next hold. Margin run with the model on, `20260930-225125`:<br>• +150 ms tolerated by both methods, unchanged;<br>• 0 motion during holds and 0 unsafe or post-stop motion;<br>• up to 339 holds per level;<br>• more time held under heavy delay (Learned GPU at +150 ms: 53.0 s vs 43.5 s);<br>• not a fresh restart. | PARTIALLY VERIFIED |
| REQ-06 | The alignment stops with zero command on a sensor-worker exit or exception, an inference failure, a clock error, or the 120 s run limit. | Perception fails silently while the controller stays active. | `worker_failed`, `inference_failed`, `clock_error` and `run_timeout` paths in `realtime.py`. Since 2026-10-02 the results pipe is read only by `ResultReceiver` (a separate thread), so a blocked read cannot stop the control loop from detecting the dead worker (`is_alive()`). | RT `test_worker_exit_stops_and_shutdown_does_not_hang`, `test_terminated_waiting_worker_cannot_block_shutdown_notification`, `test_worker_reports_full_error_chain`, `test_run_timeout_even_with_fresh_frames` (1 s limit). Fault injection: `test_sensor_worker_crash_stops_the_arm`, `test_half_sent_frame_cannot_block_the_control_loop` (a real half message in the results pipe; not on Windows), `test_blocked_result_read_cannot_block_the_control_loop`, RT `ResultReceiverTests` (6 tests, no rendering), `test_clock_stepping_back_stops_with_clock_error`, `test_failed_inference_stops_the_alignment` (failure injected in the sensor worker through `RuntimeConfig.inference_failure`), `test_run_limit_ends_a_moving_alignment` (1 s limit in the running process). | No such failure occurred in the saved runs; acceptance runtime errors were 0. Fault-injection campaign `results/fault-injection/20261001-001047`, 5/5 each: `clock_error` 0.4 ms after the clock jump; `inference_failed` at most 50.3 ms after the failed result arrived (50 ms transport delay plus the next tick) with no command from it; `run_timeout` at most 3.7 ms after the limit; a killed sensor worker stopped the arm at most 13.4 ms later. The campaign found and the fix removed a runtime crash after the `clock_error` stop (negative physics step). **Defect fixed 2026-10-02:** in the first campaign 1 of 5 worker kills left the control thread unable to shut down: a partial result message blocked the mailbox read (`reproduce_partial_message.py`). The old runtime froze again in 1 of 30 random kills (`results/receiver-thread/20261002-old-code-worker-kills`). On the fixed runtime (`results/fault-injection/20261002-141659-worker-crash` and `-all`):<br>• a half-sent frame left in the real pipe: 35/35;<br>• a result read that never returns: 35/35;<br>• the random kill: 34/35. The failed repeat never killed the worker, because a start-up `control_overrun` stopped the alignment first.<br>While the read was blocked the loop kept running, the watchdog paused at most 1.8 ms after the 400 ms limit, and shutdown completed. `worker_failed` followed at most 40.1 ms after the kill with a half-sent frame (one outlier; otherwise ≤ 19.0 ms), at most 20.2 ms with a never-returning read, and at most 20.8 ms with a random kill. In the same campaign, the clock step passed 4/5: its failed repeat stopped with `stale_camera` before the clock jump was injected ([fault injection](FAULT_INJECTION.md)).<br>**Windows laptop (2026-10-03):** the then-gate "stop within 50 ms of the kill request" failed in most runs: kill to stop took 37–132 ms (`results/fault-injection/20261003-001323`, `20261003-002419`, `20261003-003238`). Run `20261003-010746` split the time:<br>• Windows took 48–130 ms to report the killed worker exited;<br>• the runtime stopped at most 2.8 ms after that through its per-tick `is_alive()` check, or 7–16 ms before it in the random-kill runs, because the receiver saw the worker's pipe close first.<br>The gate was therefore split into runtime reaction (≤ 50 ms after the OS-reported exit), platform detection (recorded) and end to end (≤ 400 ms). The earlier runs keep their recorded failures. Confirmation run under the new gates (`results/fault-injection/20261003-132018`): random kill 10/10 (all stopped by the receiver seeing the pipe close, 7–15 ms before the OS-reported exit), never-returning read 10/10 (stop 0.8–2.3 ms after the exit); kill request to stop at most 62.7 ms. Repeat after a fresh restart (`20261003-134000`): 10/10 and 10/10, OS-reported exit up to 114.5 ms after the kill request, stop at most 1.9 ms after it (or before it, via the pipe), so the Windows delay does not depend on uptime. The Learned GPU failure itself is injected, not reproduced. | PARTIALLY VERIFIED |
| REQ-07 | No backlog: work older than 50 ms is dropped before rendering and matching, and mailboxes hold one item. | Every command acts on an ever-older image; memory grows. | `put_latest()`, single-item queues, `request_is_expired()`, bounded `pending` (8). Since 2026-10-02 the receiver thread forwards results to a one-item, latest-wins mailbox (`ResultReceiver`): each stage (the pipe and the receiver's mailbox) holds at most one result, and with one request outstanding at most one result is in flight; drops in either are counted (`result_dropped`, `receiver_dropped`). | RT `test_queue_overload_keeps_memory_bounded_and_latest_request`, `test_old_pending_work_is_dropped_before_rendering_or_matching`, `test_slow_worker_skips_capture_slots_and_keeps_actual_snapshots_fresh`, `ResultReceiverTests.test_forwards_messages_and_keeps_only_the_latest_unread_one`. | `20260925-native-scheduling-after`: 0 expired requests and 0 result-mailbox drops. Receiver mailbox (cloud timing comparison `results/receiver-thread/20261002-timing`, 7 sessions of 60 s): 0 drops, never more than one message. | PARTIALLY VERIFIED |
| REQ-08 | A scheduler gap longer than 50 ms is not replayed as one long physics step; the lost time is counted (`discarded_physics_s`). | One long step moves the robot far at once and skips the per-step collision and joint checks. | `sim.advance(min(gap, max_control_gap_s))` in `realtime.py`. | Fault injection: `test_physics_gap_is_capped_and_counted` (a 150 ms stall; asserts that no physics call exceeds 50 ms and that the lost time is counted). | Acceptance run: `discarded_physics_s` = 0 in every session, so the mechanism was never exercised. Fault-injection campaign `results/fault-injection/20261001-001047`, 5/5: the physics call after a 0.15 s stall was exactly 0.050 s, and 0.103 s was counted in `discarded_physics_s`. | VERIFIED |
| REQ-09 | Lab mode (desktop app with simulated camera delay): control stops when capture age reaches 250 ms and does not restart on late frames. | Same as REQ-01, in the desktop lab. | `TimedCamera.failure()` (`camera_timing.py`), `Lab._camera_failure()` (`app.py`). | `tests/test_camera_timing.py`: `test_watchdog_uses_capture_age_not_arrival_time`, `test_expired_frame_cannot_refresh_the_watchdog`. `tests/test_camera_delay_integration.py`: `test_stream_outage_stops_at_capture_age_deadline_and_does_not_restart`, `test_delay_beyond_age_budget_never_moves`. `tests/test_camera_robustness_integration.py`; `tests/test_camera_robustness.py::test_total_packet_loss_never_refreshes_feedback`. | `results/camera-delay/20260919-validation`: `stale_camera` at 250.0 ms for ArUco, SIFT and Learned. `results/robustness/20260920-validation`: long-outage profile ended in the expected stop in 2/2 trials per matcher. | VERIFIED |
| REQ-24 | A physical robot needs an actuator-side command timeout that stops the motors if commands stop arriving (`REALTIME_CONTROL.md`, `WATCHDOG_DECISION.md`). | The motors keep the last velocity if the whole control process hangs or crashes. | **Not implemented.** This is a documented limitation of the simulation. | None. | None. | NOT VERIFIED |

### Workspace safety: collision, joint limits, speed

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-10 | No forbidden contact, meaning robot-environment or robot-self penetration. | Collision damages the robot, camera, target or surroundings. | `CollisionGuard` velocity filter on every 2 ms physics step (`collision.py`, `Simulation._guard_velocity`), `Simulation.forbidden_contacts()`; realtime contact counter. | `tests/test_collision.py`: `test_contacts_enabled_and_unsafe_reset_is_atomic`, `test_maximum_speed_approach_brakes_before_floor_and_stays_stopped`, `test_held_command_is_guarded_after_obstacle_appears`. Acceptance gate `contacts: 0`. | `results/collision/20260920-validation`: 9/9 cases, 0 contacts. Acceptance run: 0 / 0. Since 2026-10-01 every run also records the clearance above the margin (REQ-11). | VERIFIED |
| REQ-11 | Keep at least 12 mm from the environment and 6 mm between non-adjacent robot parts, including along planned detours (`collision_config.json`). | Without the margin there is no room for braking or model error, so the next disturbance becomes a contact. | `CollisionGuard.filter_velocity` (braking reserve), `pose_clear`, swept-segment checks, `plan_path`. The guard also keeps the smallest slack of each check (`last_slack`), recorded per physics step by `Simulation.safety_record()`. | `tests/test_collision.py`: per-tick clearance assertions in the approach tests, `test_detour_has_dense_clearance_and_does_not_change_live_state`, `test_swept_path_rejects_safe_endpoints_with_blocked_middle`, `test_nonadjacent_self_geometry_is_checked`. `tests/test_safety_metrics.py::test_clearance_matches_the_guard_geometry`; acceptance and margin gates `tools/test_acceptance_test.py::test_a_safety_envelope_violation_fails_the_method`, `tools/test_margin_test.py::SafetyEnvelope`. | Collision study: minimum slack **3.00 mm** above the margin in all 9 cases. The published camera-robustness run predates slack logging. `COLLISION_AWARE.md` states the braking reserve is not a formal guarantee.<br>**Recorded in every run since 2026-10-01:** the acceptance test, margin test, acceptance campaign and stress analysis report the smallest clearance above the margin over every 2 ms physics step, with limit, PASS/FAIL and this ID (`safety_metrics.py`; the acceptance test and campaign fail on a FAIL, a margin level is not tolerated). Laptop acceptance run `results/acceptance-test/20261002-233959` (12 of 12 sessions recorded, no obstacle): at least **3.00 mm** to the environment and **9.00 mm** between robot parts, PASS. Fault-injection campaign `results/fault-injection/20261001-001047` (no obstacle): at least 3.0 mm to the environment and 9.0 mm between robot parts in every scenario. **Still partial:** no run with the record has placed an obstacle or driven a detour; the collision study that did predates the actuator model. | PARTIALLY VERIFIED |
| REQ-12 | Joints stay inside the `scene.xml` ranges. Commands slow toward a 0.06 rad margin, and outbound commands are zeroed 0.03 rad before the hard limit. | Stalled alignment and large tracking error; on hardware, driving into mechanical stops. | `velocity_bounds`, `limited_command` and `JointLimitSupervisor` (`joint_limits.py`); backstop in `Simulation.advance`, which with the actuator model triggers earlier by the joint's remaining stopping distance; workspace clipping in recovery and search. | `tests/test_joint_limits.py`: `test_velocity_damper_slows_outward_commands_but_allows_retreat`, `test_constrained_command_obeys_joint_and_actual_camera_speed_limits`, `test_both_previous_joint_limit_failures_now_reposition_and_converge`. Near-limit tests in `test_recovery.py` and `test_startup_search.py`. `tests/test_actuator.py::test_joint_limit_backstop_accounts_for_the_stopping_distance` (backstop at 0.6 rad/s with the actuator model). `tests/test_safety_metrics.py::test_joint_margin_and_speeds`. | `results/joint-limits/20260912-131706-003699`: trace validation PASS (joint margin ≥ 0.06 rad minus 0.005 rad tracking tolerance), 198/200 aligned. Robustness run: 0 safety violations, with joint limits sampled every tick. Both predate the actuator model.<br>**Recorded in every run since 2026-10-01:** the acceptance test, margin test, acceptance campaign and stress analysis report the smallest distance to a joint limit over every 2 ms physics step, with limit, PASS/FAIL and this ID (`safety_metrics.py`; the acceptance test and campaign fail on a FAIL, a margin level is not tolerated). Laptop acceptance run `results/acceptance-test/20261002-233959`: joint margin at least **0.995 rad**, PASS. **Still partial:** no joint came near its limit in that run; the near-limit evidence (joint-limit study) predates the actuator model. | PARTIALLY VERIFIED |
| REQ-13 | Joint speed never exceeds 0.6 rad/s (the actuator range). Per-mode caps are 0.35 rad/s (IBVS), 0.25 (search) and 0.18 (recovery). | Higher speed lengthens stopping distance beyond the clearance margin and braking reserve. | Clip in `Simulation.command_velocity`; per-mode scaling in `control.py`, `startup_search.py`, `recovery.py` and `joint_limits.py`; `ctrlrange` in `scene.xml`. | `tests/test_recovery.py` (≤ 0.18 rad/s), `tests/test_joint_limits.py` bounds, `tests/test_baselines.py::test_timeout_and_speed_bounds`. Fault injection: `test_speed_command_above_the_limit_is_clipped` (a 1.5 rad/s command). `tests/test_safety_metrics.py`. | The joint-limit study checks command bounds and raises on a violation; it passed. Fault-injection campaign `results/fault-injection/20261001-001047`: clipped to 0.600 rad/s; peak measured speed 0.592 rad/s with the default actuator model. **Without the model (`ideal`) the servo overshot a step command to 0.631 rad/s**, so the measured-speed half of this requirement relies on the actuator ramps.<br>**Recorded in every run since 2026-10-01:** the acceptance test, margin test, acceptance campaign and stress analysis report the peak commanded and measured joint speed over every 2 ms physics step, with limit, PASS/FAIL and this ID (`safety_metrics.py`; the acceptance test and campaign fail on a FAIL, a margin level is not tolerated). Laptop acceptance run `results/acceptance-test/20261002-233959`: peak commanded **0.280 rad/s** and peak measured **0.198 rad/s**, PASS; run `results/acceptance-test/20261003-134526`: 0.350 and 0.215 rad/s, PASS; run `results/acceptance-test/20261003-150343`: 0.324 and 0.220 rad/s, PASS; run `results/acceptance-test/20261003-154640`: 0.350 and 0.228 rad/s, PASS; run `results/acceptance-test/20261003-161751`: 0.245 and 0.231 rad/s, PASS; runs `20261003-181233` to `20261003-234632`: at most 0.326 and 0.222 rad/s, PASS; laptop campaign `results/fault-injection/20261003-001323`: a 1.5 rad/s command clipped to 0.600 rad/s, peak measured 0.592 rad/s. **Still partial:** the search (0.25) and recovery (0.18 rad/s) caps have tests but no saved run with the record. | PARTIALLY VERIFIED |
| REQ-25 | The simulated actuator limits hold on the servo setpoint:<br>• acceleration ≤ `max_acceleration_rad_s2` while speeding up;<br>• ≤ `max_deceleration_rad_s2` while braking;<br>• jerk ≤ `max_jerk_rad_s3`, except the counted one-step cut when a stop would otherwise reverse the motion (`stop_clamps`);<br>• no overshoot of a constant command;<br>• a zero command never reverses the motion.<br>Every stop's physical response is measured: stopping time, stopping distance, deceleration and jerk ([actuator model](ACTUATOR_MODEL.md)). | Stops and resumes that are more violent than assumed; stop behaviour that is claimed but not measured. | `ActuatorModel.step()` (`actuator.py`), applied after the collision guard and backstop in `Simulation.advance()`; `Simulation.motion_log`; `stop_response.link()`. | `tests/test_actuator.py`:<br>• 14 unit tests: limits at several speeds and directions, stop while accelerating, reversal, a provoked stop clamp, random sequences with and without stops, repeated cycles, passthrough;<br>• 10 MuJoCo tests: measured stops at 4 joints × 4 speeds, a stop while accelerating, resume before standstill, 8 pause/resume cycles, backstop, reset, close, labels, collision block.<br>`tests/test_stop_response.py`. RT watchdog-stop and hold tests check the physical stop and the resume. | `results/stop-response/20260930-004121`:<br>• 252 constant-speed stops over 3 profiles, with setpoint jerk and deceleration within the limits in every stop;<br>• 0 stop clamps in the stops, the cycles and the realtime runs;<br>• 20-cycle pause/resume with stop times 130–134 ms (`default`);<br>• realtime watchdog stops, manual stops and holds.<br>The stops and cycles are deterministic; the run was made in the cloud workspace. | VERIFIED |
| REQ-26 | The actuator model brakes at least as fast as the collision guard's braking reserve assumes: worst-case stopping travel at 0.6 rad/s ≤ speed × `braking_time_s` (0.12 s), including a 15 ms servo allowance. A profile that fails is rejected at start-up. | Stopping distance longer than the clearance reserve, so a stop that the guard expects to end with clearance ends in contact. | `ActuatorModel.check_braking()` called by `Simulation.__init__`; `Simulation` raises on a failing profile. | `tests/test_actuator.py`: `test_file_profiles_are_valid_and_pass_the_braking_check`, `test_braking_slower_than_the_collision_guard_assumes_is_rejected`, `test_collision_block_is_recorded_as_a_physical_stop`, and the per-stop travel bound in `test_stop_is_measured_for_several_speeds_and_joints`. `tests/test_collision.py` passes with the model on. | Worst-case equivalent 108 ms (`default`) and 111 ms (`gentle`) vs 120 ms. Stop-response study (`20260930-004121`):<br>• measured joint travel within the bound in all 252 stops;<br>• 42 approaches to the obstacle (joints 0–2, 3 speeds, 3 profiles, clearance checked every physics step): minimum slack above the margin 0.36 mm with `default`, 0.32 mm with `ideal`, 0 contacts. The guard had already slowed the arm to ≤ 0.012 rad/s when it blocked, so no full-speed stop next to an obstacle was tested.<br>**Clearance with the model on is not measured in acceptance or stress runs, and the bound is a joint-space linearisation, not a formal guarantee.** | PARTIALLY VERIFIED |
| REQ-14 | Every declared start pose is checked against joint limits and collision clearance before a test run. | A run starts in contact or beyond a limit, which invalidates it and, on hardware, is unsafe from the first command. | `Simulation.reset()` raises on an unsafe pose; preflight loops in the acceptance, margin and campaign tools. | `tests/test_collision.py::test_contacts_enabled_and_unsafe_reset_is_atomic`. **No test of the harness preflight loop.** | Runs refuse to start on a failed preflight, but the manifests do not record the preflight result. | PARTIALLY VERIFIED |
| REQ-15 | When no safe motion exists, the robot stops (`collision_blocked`) with zero command. A search skips at most four blocked waypoints per update. | Pushing against a blocked direction: oscillation at the obstacle or an endless search beside it. | `skip_blocked_motion()`, the `collision_blocked` stop in `realtime.py` and `app.py`, and the planner check budget. | `tests/test_collision.py`: `test_blocked_manual_motion_is_reported_by_app` (lab), `test_all_blocked_waypoints_finish_with_bounded_zero_motion`, `test_planning_budget_fails_closed_and_releases_scratch_budget`. Fault injection: `test_collision_block_stops_the_realtime_alignment` (the guard's decision is replaced by `blocked` while tracking). | Collision study: detours and skipped waypoints are recorded, and all 9 cases ended safely. Fault-injection campaign `results/fault-injection/20261001-001047`, 5/5: realtime stop at most 0.5 ms after the injected block, 0 motion afterwards. The block is injected, geometric blocking in the realtime runtime and the search/recovery skip branch are not exercised. | PARTIALLY VERIFIED |
| REQ-16 | Search and recovery are bounded:<br>• startup search ≤ 180 s;<br>• recovery ≤ 20 s per episode and 45 s in total, at most 3 episodes, at most 60° excursion;<br>• IBVS timeout 20 s;<br>• realtime run limit 120 s. | With the target absent, an unbounded search keeps the robot moving and widens the swept workspace. | Limits in `startup_search.py`, `recovery.py`, `control.py` and `realtime.py`, from their config files. | `tests/test_startup_search.py::test_timeout_is_terminal_even_if_marker_appears_later`, `tests/test_search_coverage.py::test_deadline_covers_both_passes_and_never_restarts`, `tests/test_recovery.py::test_repeated_losses_and_total_deadline_cannot_restart_forever` and `::test_absent_target_times_out_with_bounded_commands`, `tests/test_control.py::test_timeout_stops`. Realtime run limit: `tests/test_fault_injection.py::test_run_limit_ends_a_moving_alignment`. | Startup, search-coverage and joint-limit studies: every negative control ended in a bounded outcome (`target_not_found`, `timeout`, `recovery_limit` or `reposition_timeout`), with zero command. Realtime run limit: fault-injection campaign `results/fault-injection/20261001-001047`, 5/5 stopped at most 3.7 ms after a 1 s limit. | VERIFIED |

### Task performance and acceptance

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-17 | Every attempt ends with camera and tool position error ≤ 2 mm and orientation error ≤ 1° (acceptance test; accuracy study). | A "converged" alignment is physically off, and the error carries into whatever the tool does next. | Precision stopping gates decide when to stop (`precision.py`, `control.py`). The harness measures error against the simulated ground-truth pose and fails the run. | `tests/test_accuracy.py::test_physical_acceptance_requires_both_frames_and_orientation`; `tools/test_acceptance_campaign.py::test_accuracy_and_latency_fail` (position above 2 mm only). **No acceptance-harness test with an error above the limit, and no harness test of the 1° limit.** | Acceptance run: worst 0.463 mm / 0.097° (SIFT) and 0.606 mm / 0.125° (Learned GPU); laptop run `results/acceptance-test/20261002-233959`: 0.458 mm / 0.097° and 0.655 mm / 0.128°. Run `results/acceptance-test/20261003-134526`: the one alignment ended by `control_overrun` stopped 35.4 mm / 4.49° from the goal; converged alignments were within tolerance (Learned GPU worst 0.756 mm / 0.126°). Run `results/acceptance-test/20261003-150343`: 0.464 mm / 0.096° and 0.662 mm / 0.118°. Run `results/acceptance-test/20261003-154640`: 0.460 mm / 0.096° and 0.639 mm / 0.114°. Run `results/acceptance-test/20261003-161751`: 0.457 mm / 0.096° and 0.714 mm / 0.128°. Run `results/acceptance-test/20261003-181233`: the SIFT alignment ended by `control_overrun` stopped 5.38 mm / 0.65° from the goal; converged alignments were within tolerance. Runs `20261003-184433` to `20261003-234632`: at most 0.464 mm / 0.097° (SIFT) and 0.730 mm / 0.135° (Learned GPU). `results/accuracy/20260921-precision-final`: 72/72 within tolerance. Earlier degraded runs with the stop response failed (e.g. 5.19 mm; 36.4 mm / 4.59°). The docs call the thresholds illustrative, not a robot specification. | PARTIALLY VERIFIED |
| REQ-18 | An alignment is reported as converged only after the precision gates hold for 0.5 s, and the error stays below the threshold with zero command after the stop. | False success reported on a transient good frame or while still moving. | IBVS hold (`control.py`), precision gates (`precision.py`), post-stop checks in the study runners. | `tests/test_control.py::test_success_requires_full_observed_hold_window`; `tests/test_precision.py::test_hold_resets_after_any_gate_fails` and `::test_coupled_translation_rotation_under_one_pixel_does_not_stop`; `tests/test_precision_integration.py`; `tests/test_benchmark.py::test_convergence_and_complete_stopped_observation`; `tests/test_camera_delay_integration.py::test_repeated_display_frames_cannot_confirm_convergence`. | `results/accuracy/20260921-precision-final` (precision stopping): 72/72 trials passed, each rechecked through 30 post-stop captures. The earlier recovery study (200/200 stayed below 1 px after stopping) used the older 1 px rule. | VERIFIED |
| REQ-19 | Success rate: each method's 95% Clopper–Pearson lower bound is ≥ 95%, with 0 failed alignments (acceptance test and campaign; [statistics](STATISTICS.md)). | A small, unmeasured failure rate leaves the robot stopped short of the goal in routine use; "100%" from few trials hides it. | Harness gate (`run_acceptance_test.py`, `binomial_ci.py`). | `tests/test_binomial_ci.py`; AT `test_ten_of_ten_is_not_enough_to_claim_95_percent`, `test_success_gate_uses_the_lower_confidence_bound`, `test_failed_alignment_rule_is_separate_from_the_confidence_rule`; `tools/test_acceptance_campaign.py::test_too_few_alignments_cannot_claim_95_percent`. | Laptop acceptance run `results/acceptance-test/20261002-233959`, judged under this rule: 149/149 (SIFT) and 147/147 (Learned GPU), 95% lower bounds **97.6%** and **97.5%**, 0 failed alignments, PASS. **Fresh-restart run `results/acceptance-test/20261003-134526`: SIFT 148/149, lower bound 96.3% but 1 failed alignment (a `control_overrun`, REQ-03), so FAIL; Learned GPU 147/147, lower bound 97.5%, PASS.** Run `results/acceptance-test/20261003-150343` (87 min uptime): 148/148 and 147/147, lower bounds 97.5% and 97.5%, 0 failed alignments, PASS. Fresh-restart run `results/acceptance-test/20261003-154640` (3 min): 148/148 and 147/147, lower bounds 97.5% and 97.5%, PASS. Fresh-restart run `results/acceptance-test/20261003-161751` (34 min): 149/149 and 147/147, lower bounds 97.6% and 97.5%, PASS. **Fresh-restart run `results/acceptance-test/20261003-181233` (2 min): SIFT 148/149 with 1 failed alignment (a `control_overrun`, REQ-03), so FAIL; Learned GPU 147/147.** Fresh-restart runs `20261003-184433`, `20261003-191452`, `20261003-200656`: 148/148 SIFT each and 145/145, 146/146, 146/146 Learned GPU, lower bounds 97.5%, PASS. Fresh-restart runs `20261003-231335`, `20261003-234632`: 148/148 and 147/147 each, lower bounds 97.5%, PASS. The earlier acceptance run was judged on "100% observed". The rule depends on the number of alignments: at least 72 with no failure are needed to reach 95%. | VERIFIED |
| REQ-20 | Perception rejects blank, wrong-ID, wrong, mirrored, occluded and absent targets in the tested images and negative controls: no control features and no false acquisition. This is not a guarantee for every image. | Aligning to the wrong object drives the tool to a wrong pose with full confidence. | ID check (ArUco); inlier, homography, area and coverage gates (`perception.py`, `learned_perception.py`). | `tests/test_alignment_regression.py::test_blank_wrong_id_and_occluded_images_are_rejected`; rejection tests in `tests/test_natural_perception.py`; `tests/test_learned_perception.py::test_blank_wrong_mirrored_and_tiny_evidence_are_rejected`; `tests/test_natural_integration.py::test_wrong_and_absent_targets_cannot_supply_natural_measurements`. | Negative controls in the natural-image, learned, startup, coverage and joint-limit studies: no false acquisition (two to four controls per study). | VERIFIED |
| REQ-21 | Acceptance latency gates:<br>• processing p95 / p99 / max ≤ 150 / 175 / 250 ms;<br>• capture-to-command p99 / max ≤ 250 / 400 ms;<br>• later / first processing p99 ≤ 1.5×. | Commands act on older images: overshoot, more watchdog holds, and gradual degradation of a reused worker. | Harness reporting gates only. At runtime, latency is bounded by REQ-01 and REQ-05. | AT `test_later_alignment_regression_fails_learned_only`; `tools/test_acceptance_campaign.py::test_accuracy_and_latency_fail` (processing max). **No acceptance-harness test that exceeds the p95, p99 or capture limits.** | Acceptance run:<br>• processing 64.9 / 72.3 / 97.3 ms (SIFT), 58.9 / 67.1 / 97.0 ms (Learned);<br>• capture 129.1 / 154.2 and 121.4 / 152.1 ms;<br>• later / first 1.20× and 1.39×.<br>Laptop run `results/acceptance-test/20261002-233959` (receiver-thread runtime): processing 60.8 / 68.7 / 159.7 ms (SIFT) and 55.8 / 62.8 / 113.1 ms (Learned); capture 125.0 / 217.9 and 117.4 / 170.2 ms; later / first 1.00× and 1.25×.<br>Fresh-restart run `results/acceptance-test/20261003-134526`: processing 61.7 / 70.7 / 125.0 ms (SIFT) and 58.5 / 64.9 / 99.3 ms (Learned); capture 126.7 / 186.9 and 119.5 / 154.9 ms; later / first 1.03× and 1.14×; all within the gates. Run `results/acceptance-test/20261003-150343`: processing 56.5 / 62.4 / 90.2 ms and 56.0 / 60.5 / 79.7 ms; capture 119.0 / 156.4 and 115.3 / 133.4 ms; later / first 0.87× and 1.24×. Run `results/acceptance-test/20261003-154640`: processing 57.0 / 64.6 / 191.4 ms and 60.1 / 66.3 / 101.2 ms; capture 121.3 / 250.3 and 121.6 / 157.9 ms; later / first 0.91× and 1.24×. Run `results/acceptance-test/20261003-161751`: processing 60.6 / 67.2 / 89.2 ms and 55.9 / 61.1 / 75.9 ms; capture 124.3 / 166.1 and 115.5 / 133.3 ms; later / first 1.04× and 1.13×.<br>**Run `results/acceptance-test/20261003-181233`: SIFT processing max 275.6 ms (gate 250 ms), FAIL**, from the same system-wide slowdown as its deadline miss (REQ-03); p95 / p99 58.6 / 65.0 ms, capture 121.9 / 182.8 ms. Runs `20261003-184433` to `20261003-234632`: processing p95 / p99 / max at most 60.8 / 66.8 / 174.4 ms, capture at most 124.0 / 233.6 ms, later / first 0.95–1.24×.<br>Run `20260928-173556-long` failed (Learned p95 214.4 ms, 2.29×). Results depend on the hardware state. | PARTIALLY VERIFIED |
| REQ-22 | Protected production inputs (controller, calibration, scene, safety and transport files) match the validated baseline, except reviewed changes pinned in `tools/approved_changes.json`. | A test certifies code other than the code that runs. | `baseline_check()` fingerprint before and during every acceptance and margin run. | AT `test_approved_change_must_match_both_hashes`, `test_checked_in_approval_matches_the_repository`, `test_protected_inputs`. | The acceptance-run report lists the approved `realtime.py` change and identical runtime versions. The actuator-model change (`simulation.py`, `realtime.py`, `actuator.py`, `actuator_config.json`) the safety-record change (`collision.py`, `simulation.py`, `realtime.py`) and the receiver-thread change (`realtime.py`, 2026-10-02) are pinned the same way. The laptop acceptance run `results/acceptance-test/20261002-233959` ran with all of them and lists them as approved changes, with identical runtime versions. | VERIFIED |
| REQ-23 | Timing runs follow `TEST_PROCEDURE.md`: fresh restart (≤ 60 min uptime), AC power, Best performance, other programs closed. | Results measured under unknown load cannot be compared or reproduced; background load has already caused false failures. | AC power is checked (the run is INVALID otherwise). Uptime and power mode are recorded and reported, not gated. Other programs are not checked. | AT `test_conditions_met`, `test_conditions_broken_are_flagged` (report text only). **No test of the AC-power INVALID gate.** | Margin run `20260929-110852`: fresh restart **yes**. Acceptance run: fresh restart **no** (265 min uptime). Laptop run `results/acceptance-test/20261002-233959`: fresh restart **no** (5075 min uptime), AC power and Best performance yes. Run `results/acceptance-test/20261003-134526`: fresh restart **yes** (9 min), AC power and Best performance yes; other programs not checked automatically. Run `results/acceptance-test/20261003-150343`: fresh restart **no** (87 min). Run `results/acceptance-test/20261003-154640`: fresh restart **yes** (3 min), PASS. Run `results/acceptance-test/20261003-161751`: fresh restart **yes** (34 min), PASS. Runs `20261003-181233` to `20261003-234632` (six runs): fresh restart **yes** (2–5 min, each after its own restart), AC power and Best performance yes. | PARTIALLY VERIFIED |

## Hazard analysis

Severity is judged for a physical robot running the same software. In simulation
none of these hazards can cause damage, which is why the scope note above matters.

| ID | Consequence if violated | Severity | Main residual risk |
|---|---|---|---|
| REQ-01 | The arm keeps moving on outdated perception and can overshoot the goal or drive into the target or the workspace. | High | Hold-default lateness is measured only in fault-injection runs (at most 2.12 ms on the laptop), not in a normal acceptance run, which saw no holds. The requirement covers the command only. With the assumed actuator model the arm stops physically up to about 150 ms later from 0.35 rad/s, moving the camera up to 17 mm. Real braking is unknown. |
| REQ-02 | A late or replayed image produces a jump toward an old target position. | High | None known in simulation. It relies on correct timestamps from the camera. |
| REQ-03 | While the loop is stalled nothing checks freshness or collisions, and on hardware the last command keeps executing. | High | Misses were observed under stress, and Windows is not a real-time OS. The stop itself is verified by fault injection in the cloud workspace and on the laptop, and it worked on two real gaps in laptop acceptance runs (59.5 ms, cause not recorded; 62.0 ms, during a system-wide slowdown of unidentified source). VERIFIED under the pre-declared 5-in-a-row rule, but misses still occurred in 2 of 9 fresh-restart runs (95% interval 2.8–60.0% per run), so a stall can end an alignment early. The motion during a stall is not simulated. |
| REQ-04 | Unexpected motion after a stop, when people assume the robot is stationary. | High | None known. |
| REQ-05 | Resuming on stale data, or an alignment that stays "alive" and restarts much later. | Medium | No hold has run out the 2 s window in a real run, and the 5% budget has not been exercised. Repeated stop and go is simulated with the assumed actuator limits only, and untested on real motors. |
| REQ-06 | Perception failure goes unnoticed, and the controller holds a command or waits indefinitely. | High | The Learned GPU failure is injected, not reproduced. A result read that never returns now blocks only the receiver thread; that thread is left behind at shutdown (one per event). The half-sent-frame scenario cannot run on Windows. On Windows the OS takes 50–130 ms to report a killed worker exited, so a hard crash leaves the last command running that long before the stop (the 400 ms watchdog bounds it). |
| REQ-07 | Commands act on steadily older images, and memory grows. | Medium | Dropping and bounding are verified by tests only; no saved run overloaded the pipeline. |
| REQ-08 | One large step skips the per-step collision and limit checks. | High | Verified in the cloud workspace only; the laptop runs never needed it. |
| REQ-09 | Same as REQ-01, in the desktop lab. | Medium | None known in the lab mode. |
| REQ-24 | A hung or crashed controller leaves the motors running. | High | Not implemented. It must be solved in hardware or drive firmware before any physical use. |
| REQ-10 | Physical collision damage. | High | Evidence comes from 9 fixed collision cases plus contact-free acceptance runs. |
| REQ-11 | No margin left for braking or model error, so small disturbances cause contacts. | High | The acceptance record (3.00 mm) has no obstacle; obstacle and detour clearance was last measured before the actuator model. The braking reserve is not a formal guarantee. |
| REQ-12 | Contact with mechanical stops, and alignment failure near limits. | High | The acceptance run stayed 0.995 rad from every limit, so the near-limit slowdown is not exercised there. The backstop is tested at 0.6 rad/s on one joint in simulation. |
| REQ-13 | Longer stopping distance than the collision guard assumes. | Medium | Search and recovery caps have no saved run with the speed record. Without the actuator model the servo overshoots a step command by about 5%. |
| REQ-25 | Violent stops and resumes, or stop behaviour that is claimed but not measured. | Medium | The limits are assumptions. Joints are not synchronised, so the camera path can deviate briefly during ramps. |
| REQ-26 | A stop the guard expects to end with clearance ends in contact. | High | The check is a joint-space approximation with a 15 ms servo allowance measured only in MuJoCo. Clearance with the model on is not measured in acceptance runs. |
| REQ-14 | The first command already starts from an unsafe configuration. | Medium | The preflight is not recorded or tested. |
| REQ-15 | Oscillation or repeated pushing at an obstacle. | Medium | The realtime path is tested only with an injected block. |
| REQ-16 | Endless motion when the target is absent. | Medium | None known. |
| REQ-17 | The task silently fails with the tool placed millimetres off. | Medium (task) | Clean runs only. The thresholds are illustrative, and there is no real sensor or encoder error. |
| REQ-18 | Success is reported falsely, or the robot stops while still moving. | Medium | None known. |
| REQ-19 | Unknown failure rate in routine use. | Medium | <!-- BEGIN GENERATED: rule-runs (do not edit; regenerate with python -B tools/traceability.py --write) -->11 saved runs judged under the rule; 9 passed, including 7 after a fresh restart<!-- END GENERATED: rule-runs -->. The two failed runs, both after a fresh restart, failed for SIFT because of a single control deadline miss each (REQ-03), not a perception failure. VERIFIED under the pre-declared rule; the per-run chance of such a failure is not shown to be small. |
| REQ-20 | Confident alignment to the wrong object. | High | Only a few negative images and controls are tested. |
| REQ-21 | Lag-driven overshoot and more watchdog holds. | Low to medium | Depends on the hardware, and most thresholds lack a test with a value above the limit. |
| REQ-22 | Evidence refers to different code than the code that runs. | Medium | None known. |
| REQ-23 | Unrepeatable or misleading results. | Low | Only AC power is enforced. |

## Requirements without a verification test

- **REQ-24:** actuator-side command timeout (not implemented).
- **REQ-06:** a real Learned GPU inference exception (the fault-injection test replaces
  the result in the sensor worker).
- **REQ-14:** the harness preflight loop.
- **REQ-15:** geometric blocking in the realtime runtime, and the search/recovery skip
  branch; the fault-injection test injects the guard's decision.
- **REQ-17 and REQ-21:** acceptance-harness tests with a pose error above the limit,
  or processing and capture-to-command latency above their limits. Only the
  later/first ratio is tested in the acceptance tool; the campaign tests position
  and processing max.
- **REQ-23:** the AC-power INVALID gate.

## Tests not linked to a documented requirement

These tests are useful but do not verify any requirement above:
- **Setup and tooling:** `tests/test_learned_device.py`, `tests/test_learned_errors.py`
  (device selection, setup errors), `tests/test_automatic_start.py` (UI start
  buttons).
- **Operating system:** `tests/test_native_scheduling.py` (thread priority setup and
  restore). It supports REQ-03 but asserts no timing.
- **Diagnostics:** `tests/test_latency_stress.py`, `tests/test_latency_stress_analysis.py`,
  `tools/test_acceptance_environment.py` (hardware telemetry is "diagnostic, not
  gated").
- **Study bookkeeping:** `tests/test_joint_study.py`, `tests/test_natural_study.py`,
  `tests/test_learned_study.py`, `tests/test_search_coverage_study.py`.
- **Performance features and characterisation:** `tests/test_adaptive_gain.py`,
  `tests/test_calibration_integration.py` (mechanics only), `tools/test_margin_test.py`
  (a characterisation, not a requirement), `tests/test_depth_refinement.py` (depth
  estimation maths), `tests/test_learned_integration.py` (Learned-mode integration).
- **Architecture:** RT `test_frontend_draws_during_initialization_without_live_physics`,
  `test_headless_physics_does_not_create_a_renderer`.

## Documentation claims stronger than the evidence

**Corrected with this page:**
- **`WATCHDOG_DECISION.md`:**
  - It said the acceptance run "meets its 95% requirement". That run was judged on
    the earlier "100% observed" rule.
  - "Roughly doubled" understated Learned GPU (+25 → +150 ms).
  - The "20–40 s" failure time did not match the measured 27–43 s medians.
  - "None of the laptop runs followed the fresh-restart step" is out of date: the
    margin run `20260929-110852` did.
  - It stated 400 ms as if it were the runtime default.
- **`REALTIME_CONTROL.md`:** "A new result cannot revive an expired lease" and "Zero
  commands are retained until another explicit Align" were true only for the stop
  setting.
- **Root `README.md`:** "stops on stale feedback" did not mention hold and resume in
  the realtime runtime.
- **`ACCEPTANCE_TEST.md`:** "The runtime values … cannot be overridden" ignored
  `--watchdog-stop` and `--stale-resume-ms`. The list of rows that differ from the
  campaign omitted the sustained-run row.
- **`MARGIN_TEST.md`:** the "tolerated" definition omitted that excluded
  (low-power) levels are skipped.

**Left as they are, noted here:**
- **Historical Wilson intervals:** `COMPARISON.md`, `ALIGNMENT_FIX.md` and
  `BENCHMARK.md` quote them. They are historical values; current summaries use
  Clopper–Pearson ([statistics](STATISTICS.md)).
- **`CAMERA_DELAY.md`:** "Success still requires the existing 1 px threshold" predates
  precision stopping, which is now the default (`PRECISION_STOPPING.md`).
- **App `README.md`:** "198/200 to 200/200 … with no regressions" omits the caveat in
  `SEARCH_COVERAGE.md` that the two misses were used during development.
- **Startup-search report:** it states a 90 s acquisition deadline; the current
  configuration is 180 s. The report predates the change.
- **Test counts in docs (91, 187, 200, 216, 263):** these are snapshots from different
  dates, not the current total.

## Gaps between implementation, testing and documentation

1. **Safety quantities in the main evidence: one run so far.** Since 2026-10-01
   the acceptance test, margin test, acceptance campaign and stress analysis record
   minimum clearance, minimum joint margin and peak joint speed over every physics
   step, with limits and PASS/FAIL. The laptop acceptance run `results/acceptance-test/20261002-233959`
   is the first saved run with the record (all PASS). It has no obstacle, no joint near a
   limit and no search or recovery, so REQ-11 to REQ-13 stay PARTIALLY VERIFIED. No
   margin or stress run with the record has been saved. The stress runner also refuses changed production
   inputs, so it needs a new baseline manifest first.
2. **Mechanisms triggered only by injected faults.** The fault-injection campaign
   covers `control_overrun` (both triggers), physics-gap capping, `clock_error`,
   `run_timeout`, the realtime `inference_failed` and `collision_blocked` paths and the
   sensor-worker crash and speed clip, 5 repeats each (the worker-failure paths 35 in the 2026-10-02 re-run) ([fault injection](FAULT_INJECTION.md)). Two remain
   partial: the inference failure and the collision block are injected, not produced
   by the real matcher or real geometry. The campaign found two defects, both fixed: a crash after
   a backward clock step, and a control-loop freeze when the sensor worker dies in
   the middle of a result transfer (fixed 2026-10-02 with a receiver thread; regression
   scenarios 35/35 each). In the re-run on the fixed runtime, two repeats (one random
   worker kill and one clock step) failed because an earlier `control_overrun` or
   `stale_camera` stop ended the alignment before the fault was injected. The joint-limit backstop is covered by
   `tests/test_actuator.py` since 2026-09-30. The campaign ran in the cloud workspace;
   a laptop run (`run.cmd --fault-campaign`) would add the target host.
3. **Evidence older than the current behaviour.** Watchdog lateness under the hold
   default is measured only in fault-injection runs. The margin runs held and resumed many times with the 2 s window, but
   no hold has run out the window in a real run. The acceptance run saw no holds, so
   the 5% pause budget has not been exercised. The receiver thread (2026-10-02) moved
   result reception and unpickling off the control thread. The laptop's acceptance,
   margin and stress timing (REQ-03, REQ-07, REQ-21) was measured before that change.
   A cloud comparison with interleaved sessions (`results/receiver-thread/20261002-timing`,
   SIFT only) found no regression:
   - loop-gap percentiles, deadline misses, throughput and capture-to-command latency
     are unchanged within the variation between sessions;
   - IPC p99 is about 0.4–1.1 ms higher;
   - in one test scenario that aligns immediately at start-up, 2–3 of 56 sessions
     on the new runtime ended with a start-up `control_overrun`, against 0 of 56 on
     the old runtime. This is not statistically significant, and the
     diagnosed instance was OS descheduling while the receiver was idle.

   The laptop acceptance run on the new runtime (`results/acceptance-test/20261002-233959`,
   both matchers) confirmed it on the target host: PASS with 0 deadline misses and
   capture-to-command p99 125.0 / 117.4 ms (earlier run 129.1 / 121.4 ms).
   **But the next run, after a fresh restart (`results/acceptance-test/20261003-134526`),
   failed with one SIFT deadline miss** (REQ-03). One miss cannot say whether the
   receiver thread makes such stalls more likely. That is open:
   - the old runtime also had misses on the laptop (2 in the 2026-09-25 stress campaign);
   - the cloud comparison found start-up stalls in 2–3 of 56 new-runtime sessions
     against 0 of 56 old, which is not significant.

   The acceptance harness now saves the per-cycle stage breakdown, so the next miss
   can be traced. The run after it (`results/acceptance-test/20261003-150343`, 87 min uptime, not a fresh
   restart) passed with 0 misses (largest loop gap 11.8 ms), and a further
   fresh-restart run (`results/acceptance-test/20261003-154640`, 3 min uptime) passed with 0 misses
   (largest loop gap 23.6 ms), as did one at 34 min uptime (`20261003-161751`). A second
   fresh-restart miss (`20261003-181233`) came with the breakdown saved: the control loop and the sensor
   worker slowed down together for about 0.5 s, so the stall came from outside the control thread's own work
   (no GC, little descheduling), from a source not identified; three passes followed. The
   pre-declared criterion for returning REQ-03 and REQ-19 to VERIFIED is in the summary. The margin and stress runs have not been repeated on the new runtime.
4. **Command stop and physical stop are now separate, measured quantities.** The
   actuator model (REQ-25, REQ-26) makes the simulated arm brake over a finite time
   and distance. Every stop is measured, and the physical values are reported next to
   the command values in the acceptance, margin and latency reports. Three gaps
   remain:
   - the requirements (REQ-01, REQ-04, REQ-05) are still stated for the command;
   - the actuator limits are assumptions, not drive data;
   - no actuator-side timeout exists (REQ-24).

   A physical standstill requirement such as "stopped by 400 ms of image age" is not
   met with the current settings by any profile, even without the model. The worst
   case with `default` is 602 ms from the controller's 0.35 rad/s and 654 ms at the
   0.6 rad/s cap. It would need either a much shorter image-age limit or
   lower speeds ([actuator model](ACTUATOR_MODEL.md)).
5. **Two stale-image limits.** The runtime and app default is 250 ms; the tests use
   400 ms. Documentation must say which one a statement is about.
6. **Small or deterministic samples:**
   - 9 collision cases;
   - 2 starts per calibration profile in the accuracy study;
   - two to four negative controls per study;
   - 5 fixed poses in the acceptance test.
   Their intervals and conclusions apply to those cases ([statistics](STATISTICS.md)).

## Machine-checked status

<!-- BEGIN GENERATED: status (do not edit; regenerate with python -B tools/traceability.py --write) -->
Computed by `tools/traceability.py` from the registry `docs/traceability.json` and the result files. Statuses in the tables above are copied from here.

| ID | Status | Tests found | Evidence (rule outcome) | Why not VERIFIED |
|---|---|---|---|---|
| REQ-01 | VERIFIED | 4 | 1 measured, 10 pass, 5 superseded | — |
| REQ-02 | VERIFIED | 4 | 5 pass | — |
| REQ-03 | VERIFIED | 6 | 2 fail, 1 measured, 18 pass | — |
| REQ-04 | VERIFIED | 2 | 2 pass | — |
| REQ-05 | PARTIALLY VERIFIED | 4 | 4 pass | gap: hold-window-not-exercised |
| REQ-06 | PARTIALLY VERIFIED | 11 | 3 fail, 1 measured, 3 partial, 14 pass, 10 superseded | gap: learned-failure-injected<br>gap: pre-fault-stops<br>partial coverage: results/fault-injection/20261001-001047 inference-failure<br>partial coverage: results/fault-injection/20261002-141659-all inference-failure<br>partial coverage: results/fault-injection/20261003-001323 inference-failure<br>failure: results/fault-injection/20261002-141659-all clock-step-back<br>failure: results/fault-injection/20261002-141659-worker-crash sensor-worker-crash<br>failure: results/fault-injection/20261003-001323 clock-step-back |
| REQ-07 | PARTIALLY VERIFIED | 4 | 2 measured | no evidence item meets its rule<br>gap: drop-path-not-exercised |
| REQ-08 | VERIFIED | 1 | 1 measured, 1 pass | — |
| REQ-09 | VERIFIED | 6 | 2 pass | — |
| REQ-10 | VERIFIED | 3 | 2 pass | — |
| REQ-11 | PARTIALLY VERIFIED | 6 | 1 measured, 2 pass | gap: no-obstacle-with-record |
| REQ-12 | PARTIALLY VERIFIED | 5 | 3 pass | gap: no-joint-near-limit-with-record |
| REQ-13 | PARTIALLY VERIFIED | 5 | 13 pass | gap: search-recovery-caps-unrecorded |
| REQ-14 | PARTIALLY VERIFIED | 1 | none | no evidence item meets its rule<br>gap: preflight-untested |
| REQ-15 | PARTIALLY VERIFIED | 4 | 1 partial | no evidence item meets its rule<br>gap: block-injected<br>partial coverage: results/fault-injection/20261001-001047 collision-block |
| REQ-16 | VERIFIED | 6 | 2 pass | — |
| REQ-17 | PARTIALLY VERIFIED | 2 | 2 fail, 11 pass | gap: harness-limit-untested<br>gap: overrun-stops-off-goal<br>failure: results/acceptance-test/20261003-134526<br>failure: results/acceptance-test/20261003-181233 |
| REQ-18 | VERIFIED | 6 | 1 pass | — |
| REQ-19 | VERIFIED | 5 | 2 fail, 9 pass | — |
| REQ-20 | VERIFIED | 4 | 3 pass | — |
| REQ-21 | PARTIALLY VERIFIED | 2 | 2 fail, 11 pass | gap: harness-limits-untested<br>gap: system-slowdown-20261003-181233<br>gap: long-run-failed-20260928<br>failure: results/acceptance-test/20261003-181233<br>failure: results/acceptance-test/20260928-173556-long |
| REQ-22 | VERIFIED | 3 | 2 pass | — |
| REQ-23 | PARTIALLY VERIFIED | 2 | 3 fail, 10 pass | gap: ac-gate-untested<br>gap: runs-without-fresh-restart<br>failure: results/acceptance-test/20260929-011129<br>failure: results/acceptance-test/20261002-233959<br>failure: results/acceptance-test/20261003-150343 |
| REQ-24 | NOT VERIFIED | 0 | none | mechanism not implemented |
| REQ-25 | VERIFIED | 2 | 1 pass | — |
| REQ-26 | PARTIALLY VERIFIED | 4 | 1 pass | gap: clearance-with-model-unmeasured |

**Acceptance runs used as evidence** (`verdict.json` and `manifest.json`): the runs a requirement cites, and every full run since `20261002-233959`:

| Run | Uptime at start | Fresh restart (≤ 60 min) | AC power / power mode | Verdict | Judged on the lower bound |
|---|---|---|---|---|---|
| `20260928-173556-long` | — | — | yes / — | FAIL | no |
| `20260929-011129` | 265 min | no | yes / Best performance | PASS | no |
| `20261002-233959` | 5075 min | no | yes / Best performance | PASS | yes |
| `20261003-134526` | 9 min | yes | yes / Best performance | FAIL | yes |
| `20261003-150343` | 87 min | no | yes / Best performance | PASS | yes |
| `20261003-154640` | 3 min | yes | yes / Best performance | PASS | yes |
| `20261003-161751` | 34 min | yes | yes / Best performance | PASS | yes |
| `20261003-181233` | 2 min | yes | yes / Best performance | FAIL | yes |
| `20261003-184433` | 2 min | yes | yes / Best performance | PASS | yes |
| `20261003-191452` | 2 min | yes | yes / Best performance | PASS | yes |
| `20261003-200656` | 2 min | yes | yes / Best performance | PASS | yes |
| `20261003-231335` | 5 min | yes | yes / Best performance | PASS | yes |
| `20261003-234632` | 4 min | yes | yes / Best performance | PASS | yes |

**Pre-declared fresh-restart rule** (REQ-03, REQ-19): 5 consecutive fresh-restart passes after `20261003-134526`.

- Fresh-restart passes in the current streak (raw): 5 (20261003-184433, 20261003-191452, 20261003-200656, 20261003-231335, 20261003-234632)
- Counted under the rule (independent restarts): 5 of 5 (20261003-184433, 20261003-191452, 20261003-200656, 20261003-231335, 20261003-234632)
- Met: yes
- Recorded reading: at most one run per restart counts (decided 2026-10-03: TEST_PROCEDURE.md asks for a restart before every run)

**Fault-injection scenarios cited as evidence** (`summary.json`, status from `status_of`):

| Campaign | Scenario | Cited for | Repeats passed and status | Use |
|---|---|---|---|---|
| `20261001-001047` | clock-step-back | REQ-06 | 5/5 VERIFIED | pass |
| `20261001-001047` | collision-block | REQ-15 | 5/5 PARTIALLY VERIFIED (partial coverage) | partial |
| `20261001-001047` | control-loop-stall | REQ-03 | 5/5 VERIFIED | pass |
| `20261001-001047` | control-loop-stall-below-limit | REQ-03 | 5/5 VERIFIED | pass |
| `20261001-001047` | control-loop-stall-near-limit | REQ-03 | 5/5 VERIFIED | pass |
| `20261001-001047` | controller-stall | REQ-03 | 5/5 VERIFIED | pass |
| `20261001-001047` | inference-failure | REQ-06 | 5/5 PARTIALLY VERIFIED (partial coverage) | partial |
| `20261001-001047` | physics-gap-cap | REQ-08 | 5/5 VERIFIED | pass |
| `20261001-001047` | run-timeout | REQ-06, REQ-16 | 5/5 VERIFIED | pass |
| `20261001-001047` | sensor-worker-crash | REQ-06 | 5/5 VERIFIED | pass |
| `20261001-001047` | speed-clip | REQ-13 | 1/1 VERIFIED | pass |
| `20261001-001047` | watchdog-stop | REQ-01 | 5/5 VERIFIED | pass |
| `20261002-141659-all` | clock-step-back | REQ-06 | 4/5 NOT VERIFIED | fail |
| `20261002-141659-all` | inference-failure | REQ-06 | 5/5 PARTIALLY VERIFIED (partial coverage) | partial |
| `20261002-141659-all` | run-timeout | REQ-06 | 5/5 VERIFIED | pass |
| `20261002-141659-all` | sensor-result-read-blocked | REQ-01, REQ-06 | 5/5 VERIFIED | pass |
| `20261002-141659-all` | sensor-worker-crash | REQ-06 | 5/5 VERIFIED | pass |
| `20261002-141659-all` | sensor-worker-partial-message | REQ-01, REQ-06 | 5/5 VERIFIED | pass |
| `20261002-141659-all` | watchdog-stop | REQ-01 | 5/5 VERIFIED | pass |
| `20261002-141659-worker-crash` | sensor-result-read-blocked | REQ-01, REQ-06 | 30/30 VERIFIED | pass |
| `20261002-141659-worker-crash` | sensor-worker-crash | REQ-06 | 29/30 NOT VERIFIED | fail |
| `20261002-141659-worker-crash` | sensor-worker-partial-message | REQ-01, REQ-06 | 30/30 VERIFIED | pass |
| `20261003-001323` | clock-step-back | REQ-06 | 2/3 NOT VERIFIED | fail |
| `20261003-001323` | control-loop-stall | REQ-03 | 3/3 VERIFIED | pass |
| `20261003-001323` | control-loop-stall-below-limit | REQ-03 | 3/3 VERIFIED | pass |
| `20261003-001323` | control-loop-stall-near-limit | REQ-03 | 3/3 VERIFIED | pass |
| `20261003-001323` | controller-stall | REQ-03 | 3/3 VERIFIED | pass |
| `20261003-001323` | inference-failure | REQ-06 | 3/3 PARTIALLY VERIFIED (partial coverage) | partial |
| `20261003-001323` | run-timeout | REQ-06 | 3/3 VERIFIED | pass |
| `20261003-001323` | sensor-result-read-blocked | REQ-01, REQ-06 | 0/3 NOT VERIFIED | superseded (1) |
| `20261003-001323` | sensor-worker-crash | REQ-06 | 3/3 VERIFIED | superseded (1) |
| `20261003-001323` | speed-clip | REQ-13 | 1/1 VERIFIED | pass |
| `20261003-001323` | watchdog-stop | REQ-01 | 3/3 VERIFIED | pass |
| `20261003-002419` | sensor-result-read-blocked | REQ-01, REQ-06 | 0/10 NOT VERIFIED | superseded (1) |
| `20261003-002419` | sensor-worker-crash | REQ-06 | 2/10 NOT VERIFIED | superseded (1) |
| `20261003-003238` | sensor-result-read-blocked | REQ-01, REQ-06 | 0/10 NOT VERIFIED | superseded (1) |
| `20261003-003238` | sensor-worker-crash | REQ-06 | 3/10 NOT VERIFIED | superseded (1) |
| `20261003-010746` | sensor-result-read-blocked | REQ-01, REQ-06 | 0/10 NOT VERIFIED | superseded (1) |
| `20261003-010746` | sensor-worker-crash | REQ-06 | 0/10 NOT VERIFIED | superseded (1) |
| `20261003-131231` | sensor-result-read-blocked | REQ-01, REQ-06 | 1/10 NOT VERIFIED | superseded (2) |
| `20261003-131231` | sensor-worker-crash | REQ-06 | 0/10 NOT VERIFIED | superseded (2) |
| `20261003-132018` | sensor-result-read-blocked | REQ-01, REQ-06 | 10/10 VERIFIED | pass |
| `20261003-132018` | sensor-worker-crash | REQ-06 | 10/10 VERIFIED | pass |
| `20261003-134000` | sensor-result-read-blocked | REQ-01, REQ-06 | 10/10 VERIFIED | pass |
| `20261003-134000` | sensor-worker-crash | REQ-06 | 10/10 VERIFIED | pass |

(1) judged under the earlier "stop within 50 ms of the kill request" gate, which was split on 2026-10-03 (FAULT_INJECTION.md); re-run under the new gates in 20261003-132018 and 20261003-134000<br>
(2) ran with the previous harness (FAULT_INJECTION.md) under the earlier gate; re-run as 20261003-132018

**Documented gaps** (from the registry; a human decides when one is closed, except where a rule closes it):

- **REQ-03 `fresh-restart-misses`** (closed): Fresh-restart laptop runs failed on one SIFT control deadline miss each: 20261003-134526 (59.5 ms gap, cause not recorded) and 20261003-181233 (62.0 ms gap during a 0.5 s slowdown of both the control loop and the sensor worker, source not identified). Pre-declared criterion (2026-10-03): closed by 5 consecutive fresh-restart passes, or when the cause is found, fixed and re-tested. Closes when the rule `consecutive_fresh_acceptance` is met (met).
- **REQ-05 `hold-window-not-exercised`** (open): No hold has run out the 2 s window in a real run (the process test uses a 1 s window), and the acceptance run had 0 holds, so the 5% pause budget was not exercised (gap 3).
- **REQ-06 `learned-failure-injected`** (open): The Learned GPU inference failure is injected in the sensor worker (RuntimeConfig.inference_failure), not reproduced with the real matcher; the scenario has partial coverage.
- **REQ-06 `pre-fault-stops`** (open): In these repeats an earlier stale_camera or control_overrun stop ended the alignment before the fault was injected, so the fault was not exercised (gap 2 on this page; FAULT_INJECTION.md). status_of counts them as failures; the campaigns were not re-run.
- **REQ-07 `drop-path-not-exercised`** (open): The saved runs record 0 expired requests and 0 mailbox drops, so they show no backlog formed but never exercised the drop path; bounding and dropping are verified by the tests only. Found by the automated check on 2026-10-03 (the page had VERIFIED).
- **REQ-11 `no-obstacle-with-record`** (open): No run with the safety-envelope record has placed an obstacle or driven a detour; the collision study that did predates the actuator model (gap 1).
- **REQ-12 `no-joint-near-limit-with-record`** (open): No joint came near its limit in a run with the record; the near-limit evidence (joint-limit study) predates the actuator model (gap 1).
- **REQ-13 `search-recovery-caps-unrecorded`** (open): The search (0.25 rad/s) and recovery (0.18 rad/s) caps have tests but no saved run with the speed record (gap 1).
- **REQ-14 `preflight-untested`** (open): No test of the harness preflight loop, and manifests do not record the preflight result.
- **REQ-15 `block-injected`** (open): The realtime block is injected; geometric blocking in the realtime runtime and the search/recovery skip branch are not exercised.
- **REQ-17 `harness-limit-untested`** (open): No acceptance-harness test with a pose error above the limit, and no harness test of the 1° limit. The thresholds are illustrative, not a robot specification.
- **REQ-17 `overrun-stops-off-goal`** (open): Alignments ended by control_overrun (REQ-03) stopped off the goal: 35.4 mm / 4.49° in 20261003-134526 and 5.38 mm / 0.65° in 20261003-181233; converged alignments were within tolerance.
- **REQ-19 `fresh-restart-misses`** (closed): Fresh-restart laptop runs failed on one SIFT control deadline miss each: 20261003-134526 (59.5 ms gap, cause not recorded) and 20261003-181233 (62.0 ms gap during a 0.5 s slowdown of both the control loop and the sensor worker, source not identified). Pre-declared criterion (2026-10-03): closed by 5 consecutive fresh-restart passes, or when the cause is found, fixed and re-tested. Closes when the rule `consecutive_fresh_acceptance` is met (met).
- **REQ-21 `harness-limits-untested`** (open): No acceptance-harness test that exceeds the p95, p99 or capture limits; results depend on the hardware state.
- **REQ-21 `long-run-failed-20260928`** (open): The long run 20260928-173556-long failed (Learned p95 214.4 ms, 2.29×).
- **REQ-21 `system-slowdown-20261003-181233`** (open): SIFT processing max 275.6 ms (gate 250 ms) during the same 0.5 s system-wide slowdown that caused the REQ-03 miss in 20261003-181233; source not identified.
- **REQ-23 `ac-gate-untested`** (open): No test of the AC-power INVALID gate; only AC power is enforced, other programs are not checked.
- **REQ-23 `runs-without-fresh-restart`** (open): These timing runs did not follow the fresh-restart step of TEST_PROCEDURE.md.
- **REQ-26 `clearance-with-model-unmeasured`** (open): Clearance with the actuator model on is not measured in acceptance or stress runs, no full-speed stop next to an obstacle was tested, and the bound is a joint-space linearisation.
<!-- END GENERATED: status -->

## Keeping this page current

Statuses come from the evidence, not from this text. `tools/traceability.py` recomputes them and
checks the page; nothing in it marks a requirement VERIFIED because code exists.

**Where the evidence comes from.** The registry [`traceability.json`](traceability.json) lists,
per requirement, the automated tests, the result folders and the documented gaps. The tool reads
only structured result files:
- acceptance test: `verdict.json` (status, completeness and each method's failed gates, as the
  harness judged them) and `manifest.json` (uptime, AC power, power mode, protected inputs);
- fault injection: `summary.json`; each scenario's status is re-derived from its repeats with the
  campaign's own rule (`status_of`) and must match the stored one;
- margin test: `margin.json` (MEASURED only; it has no pass/fail);
- studies (`verification.json`, `summary.json`): only the field values the registry declares,
  with limits taken from the requirement text, `tools/acceptance_test.json` or
  `FRESH_RESTART_S`.

**How a status is calculated.** An evidence item passes, fails, is partial (a fault scenario
with partial coverage), is a measurement (no pass/fail), is superseded (with a reason), or is
invalid (smoke, incomplete or malformed run). Then, following the status definitions above:
- NOT VERIFIED: the mechanism is not implemented, or there is no test and no passing evidence;
- VERIFIED: at least one named test exists, at least one item passes, and there is no partial
  item, no open gap and no unexplained failure;
- PARTIALLY VERIFIED: anything in between.

Whether evidence "covers only some conditions" is a judgment. It is recorded as a named gap in
the registry, and only a person closes it, except where a pre-declared rule closes it: the
REQ-03/REQ-19 gap closes after 5 consecutive fresh-restart passes. Every failing item must be
covered by a gap, or the check fails. Where the wording of a rule allows two readings (for
example two runs after one restart), the tool counts both, reports the difference and does not
upgrade a status on the weaker reading. In the streak, a fresh-restart FAIL resets the count, a
run without a fresh restart neither counts nor resets it, and a pass means an overall PASS (which
already requires 0 misses); a non-fresh FAIL or an incomplete run is reported as an open question
and blocks the rule until `interpretation` in the registry records the intended reading.
Recorded on 2026-10-03: at most one run counts per restart (`same_restart_counts: false`), because
the test procedure asks for a restart before every run. The page shows both the raw number of
fresh-restart passes and the number counted under this reading.

**What is generated.** The Status column, the summary counts, the fresh-restart count, the
run count in the REQ-19 hazard row and the [machine-checked status](#machine-checked-status)
section, all between `BEGIN GENERATED` / `END GENERATED` markers (the Status cells are rewritten
in place). Everything else on this page is written by hand. Selected numbers in the hand-written
cells (uptimes, loop gaps, lower bounds) are checked against the result files through the
registry's `claims`.

**Regenerate** after a new run or a registry change:

```
python -B tools/traceability.py --write
```

**CI** runs `python -B tools/traceability.py --check`, which writes nothing and exits 1 when:
- a status or count differs from the evidence, or a status claims more than it;
- a generated field is stale or was edited by hand;
- a cited result path or named test does not exist, or cited evidence is git-ignored;
- a result folder in a scanned category (acceptance-test, acceptance, fault-injection,
  margin-test, stop-response) is neither cited nor excluded with a reason;
- a requirement ID is missing or duplicated;
- a result file is invalid, incomplete or contradicts itself;
- a checked number no longer matches its file.

A new requirement, test or result folder therefore needs a registry entry. Evidence must be
published (whitelisted in `results/.gitignore`) for CI to see it.
