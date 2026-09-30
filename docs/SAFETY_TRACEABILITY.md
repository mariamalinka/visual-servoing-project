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

**Summary:** 26 requirements. 10 are VERIFIED, 14 are PARTIALLY VERIFIED and 2 are NOT
VERIFIED. The review was made on 2026-09-29 from the code, tests and saved results.
REQ-25 and REQ-26 and the actuator notes in REQ-01, REQ-05 and REQ-12 were added on
2026-09-30 with the actuator model.

## Traceability table

Abbreviations:
- `RT` = `tests/test_realtime.py`
- `AT` = `tools/test_acceptance_test.py`
- **acceptance run** = `results/acceptance-test/20260929-011129`: standard profile,
  hold-and-resume default, 150 SIFT and 147 Learned GPU alignments.

### Runtime and watchdog safety (`realtime.py`)

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-01 | When the newest accepted camera image reaches the age limit, the robot is commanded to zero velocity no later than one control deadline (≤ 50 ms) after the limit. The limit is 400 ms in the acceptance, margin and latency tests; the runtime and app default is 250 ms. | Motion continues on a view that no longer matches reality. | `CommandLease.failure()` checked on every 2 ms tick and after control computation. `stop()` / `pause()` command zero velocity. | RT `test_expired_initial_lease_is_independent_of_worker`, `test_blocking_inference_cannot_pause_watchdog_or_rearm_motion` (stop setting, lateness < 50 ms), `test_watchdog_pause_zeroes_motion_on_time_and_resumes_on_fresh_images` and `test_watchdog_pause_ends_the_run_when_images_do_not_return` (hold default, lateness < 50 ms). `run.cmd --latency-study` gate ≤ 50 ms. | `results/latency/20260921-validation`: worst lateness **2.034 ms** over the injected-stall trials. That run used the stop setting; the current hold default is covered by tests only, with no saved measurement. **This requirement is about the command.** The physical standstill that follows is measured and reported, not required. It came 42–72 ms after the zero command in six realtime watchdog trips with the actuator model on (image age at standstill 442–474 ms). The worst case is 152 ms from 0.35 rad/s (`results/stop-response/20260930-004121`, [actuator model](ACTUATOR_MODEL.md)). | PARTIALLY VERIFIED |
| REQ-02 | Commands come only from a result of the current alignment that was captured after the last accepted image and is still younger than the age limit. | A late or out-of-order result moves the robot toward where the target used to be. | `CommandLease.eligible()`; generation numbers; re-check after computation. `unsafe_motion_ticks` counts any motion without fresh feedback. | RT `test_duplicates_future_prestart_nan_and_reordered_frames_rejected`, `test_expired_delivery_cannot_extend_capture_age`, `test_stop_latches_and_rearm_rejects_previous_generation`; the process tests assert `unsafe_motion_ticks == 0`. Acceptance gate `unsafe_motion: 0` (AT `test_each_safety_counter_fails`). | Acceptance run: unsafe motion 0 / 0. Margin runs: 0 at every level. `results/latency/20260925-native-scheduling-after`: 0. | VERIFIED |
| REQ-03 | A control-loop gap or controller computation longer than 50 ms stops the alignment (`control_overrun`). Acceptance requires 0 misses. | The last command runs unsupervised while the loop is stalled. | Gap and compute-time checks in `RealtimeSession._run()`; `control_deadline_misses` counter; `native_scheduling.py` reduces stalls. | **No runtime test triggers `control_overrun`.** Harness only: AT `test_each_safety_counter_fails` (synthetic counter), `test_production_limits_cannot_be_relaxed`. | Acceptance run: 0 misses; largest loop gap 38.6 ms (Learned GPU) and 11.1 ms (SIFT). `20260925-native-scheduling-after` stress campaign: **2 misses** (Learned GPU). | PARTIALLY VERIFIED |
| REQ-04 | A stop is latched: the command stays zero and no late result restarts motion until a new Align. This is checked for 1.1 s after every stop. | Motion restarts after an operator or watchdog stop, when everyone assumes the robot is stationary. | `CommandLease.stop()` deactivates the lease, and a new generation is needed to move. `post_stop_motion_ticks` counter. Harness post-stop observation (`stop_stayed_latched`). | RT `test_blocking_inference_cannot_pause_watchdog_or_rearm_motion` (stays stopped after the late result arrives), `test_manual_stop_and_rearm_have_distinct_frame_generations`. Acceptance gates `post_stop_motion: 0`, `unlatched_stops: 0`. | Acceptance run: post-stop motion 0 / 0, unlatched stops 0 / 0. Stress campaign: "All stops stayed latched: True / True". | VERIFIED |
| REQ-05 | Hold and resume (default `stale_resume_s` = 2 s):<br>• the command stays zero during a hold;<br>• control resumes only on a newer, fresh image of the same alignment;<br>• with no such image for 2 s, the alignment ends as `stale_camera`.<br>Acceptance allows 0 alignments ended by the watchdog and holds of at most 5% of alignment time. | Resuming on outdated information, or an unbounded "running" hold that restarts much later. | `CommandLease.pause()` and `failure()`, `watchdog()`, resume only through `eligible()`, `paused_motion_ticks`. Harness pause budget. | RT `test_pause_holds_only_within_resume_window`, `test_pause_never_outlives_run_timeout_or_a_stop`, `test_hold_and_resume_is_the_default_and_stop_is_a_setting` and the two process pause tests. AT `test_pause_budget` (the harness verdict only). | Margin runs `20260928-234612-resume2000ms` and `20260929-110852`: 0 motion during holds and 0 alignments ended by the watchdog up to +150 ms, with up to 342 holds per level, all using the 2 s window. **No hold has run out the 2 s window in a real run.** The process test that ends a hold uses a 1 s window. Acceptance run: **0 holds occurred**, so the 5% budget was not exercised. All of this evidence predates the actuator model. With the model, a hold brakes the arm within about 0.1–0.15 s from 0.35 rad/s, and within 70 ms in a realtime hold-and-resume run at typical alignment speeds. A resume ramps up within the acceleration and jerk limits (stop-response study; RT hold test). Under heavy per-frame delay, each resume moved the arm for at most 18 ms before the next hold. Margin run with the model on, `20260930-225125`:<br>• +150 ms tolerated by both methods, unchanged;<br>• 0 motion during holds and 0 unsafe or post-stop motion;<br>• up to 339 holds per level;<br>• more time held under heavy delay (Learned GPU at +150 ms: 53.0 s vs 43.5 s);<br>• not a fresh restart. | PARTIALLY VERIFIED |
| REQ-06 | The alignment stops with zero command on a sensor-worker exit or exception, an inference failure, a clock error, or the 120 s run limit. | Perception fails silently while the controller stays active. | `worker_failed`, `inference_failed`, `clock_error` and `run_timeout` paths in `realtime.py`. | RT `test_worker_exit_stops_and_shutdown_does_not_hang`, `test_terminated_waiting_worker_cannot_block_shutdown_notification`, `test_worker_reports_full_error_chain`, `test_run_timeout_even_with_fresh_frames` (1 s limit). **No test for `clock_error` or for the realtime `inference_failed` stop.** | No such failure occurred in the saved runs; acceptance runtime errors were 0. | PARTIALLY VERIFIED |
| REQ-07 | No backlog: work older than 50 ms is dropped before rendering and matching, and mailboxes hold one item. | Every command acts on an ever-older image; memory grows. | `put_latest()`, single-item queues, `request_is_expired()`, bounded `pending` (8). | RT `test_queue_overload_keeps_memory_bounded_and_latest_request`, `test_old_pending_work_is_dropped_before_rendering_or_matching`, `test_slow_worker_skips_capture_slots_and_keeps_actual_snapshots_fresh`. | `20260925-native-scheduling-after`: 0 expired requests and 0 result-mailbox drops. | VERIFIED |
| REQ-08 | A scheduler gap longer than 50 ms is not replayed as one long physics step; the lost time is counted (`discarded_physics_s`). | One long step moves the robot far at once and skips the per-step collision and joint checks. | `sim.advance(min(gap, max_control_gap_s))` in `realtime.py`. | **No test found.** | Acceptance run: `discarded_physics_s` = 0 in every session, so the mechanism was never exercised. | NOT VERIFIED |
| REQ-09 | Lab mode (desktop app with simulated camera delay): control stops when capture age reaches 250 ms and does not restart on late frames. | Same as REQ-01, in the desktop lab. | `TimedCamera.failure()` (`camera_timing.py`), `Lab._camera_failure()` (`app.py`). | `tests/test_camera_timing.py`: `test_watchdog_uses_capture_age_not_arrival_time`, `test_expired_frame_cannot_refresh_the_watchdog`. `tests/test_camera_delay_integration.py`: `test_stream_outage_stops_at_capture_age_deadline_and_does_not_restart`, `test_delay_beyond_age_budget_never_moves`. `tests/test_camera_robustness_integration.py`; `tests/test_camera_robustness.py::test_total_packet_loss_never_refreshes_feedback`. | `results/camera-delay/20260919-validation`: `stale_camera` at 250.0 ms for ArUco, SIFT and Learned. `results/robustness/20260920-validation`: long-outage profile ended in the expected stop in 2/2 trials per matcher. | VERIFIED |
| REQ-24 | A physical robot needs an actuator-side command timeout that stops the motors if commands stop arriving (`REALTIME_CONTROL.md`, `WATCHDOG_DECISION.md`). | The motors keep the last velocity if the whole control process hangs or crashes. | **Not implemented.** This is a documented limitation of the simulation. | None. | None. | NOT VERIFIED |

### Workspace safety: collision, joint limits, speed

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-10 | No forbidden contact, meaning robot-environment or robot-self penetration. | Collision damages the robot, camera, target or surroundings. | `CollisionGuard` velocity filter on every 2 ms physics step (`collision.py`, `Simulation._guard_velocity`), `Simulation.forbidden_contacts()`; realtime contact counter. | `tests/test_collision.py`: `test_contacts_enabled_and_unsafe_reset_is_atomic`, `test_maximum_speed_approach_brakes_before_floor_and_stays_stopped`, `test_held_command_is_guarded_after_obstacle_appears`. Acceptance gate `contacts: 0`. | `results/collision/20260920-validation`: 9/9 cases, 0 contacts. Acceptance run: 0 / 0. | VERIFIED |
| REQ-11 | Keep at least 12 mm from the environment and 6 mm between non-adjacent robot parts, including along planned detours (`collision_config.json`). | Without the margin there is no room for braking or model error, so the next disturbance becomes a contact. | `CollisionGuard.filter_velocity` (braking reserve), `pose_clear`, swept-segment checks, `plan_path`. | `tests/test_collision.py`: per-tick clearance assertions in the approach tests, `test_detour_has_dense_clearance_and_does_not_change_live_state`, `test_swept_path_rejects_safe_endpoints_with_blocked_middle`, `test_nonadjacent_self_geometry_is_checked`. | Collision study: minimum slack **3.00 mm** above the margin in all 9 cases. **Clearance is not measured in the acceptance, margin or stress runs.** The published camera-robustness run predates slack logging. `COLLISION_AWARE.md` states the braking reserve is not a formal guarantee. | PARTIALLY VERIFIED |
| REQ-12 | Joints stay inside the `scene.xml` ranges. Commands slow toward a 0.06 rad margin, and outbound commands are zeroed 0.03 rad before the hard limit. | Stalled alignment and large tracking error; on hardware, driving into mechanical stops. | `velocity_bounds`, `limited_command` and `JointLimitSupervisor` (`joint_limits.py`); backstop in `Simulation.advance`, which with the actuator model triggers earlier by the joint's remaining stopping distance; workspace clipping in recovery and search. | `tests/test_joint_limits.py`: `test_velocity_damper_slows_outward_commands_but_allows_retreat`, `test_constrained_command_obeys_joint_and_actual_camera_speed_limits`, `test_both_previous_joint_limit_failures_now_reposition_and_converge`. Near-limit tests in `test_recovery.py` and `test_startup_search.py`. `tests/test_actuator.py::test_joint_limit_backstop_accounts_for_the_stopping_distance` (backstop at 0.6 rad/s with the actuator model). | `results/joint-limits/20260912-131706-003699`: trace validation PASS (joint margin ≥ 0.06 rad minus 0.005 rad tracking tolerance), 198/200 aligned. Robustness run: 0 safety violations, with joint limits sampled every tick. Both predate the actuator model. **Joint positions are not measured in acceptance or realtime runs.** | PARTIALLY VERIFIED |
| REQ-13 | Joint speed never exceeds 0.6 rad/s (the actuator range). Per-mode caps are 0.35 rad/s (IBVS), 0.25 (search) and 0.18 (recovery). | Higher speed lengthens stopping distance beyond the clearance margin and braking reserve. | Clip in `Simulation.command_velocity`; per-mode scaling in `control.py`, `startup_search.py`, `recovery.py` and `joint_limits.py`; `ctrlrange` in `scene.xml`. | `tests/test_recovery.py` (≤ 0.18 rad/s), `tests/test_joint_limits.py` bounds, `tests/test_baselines.py::test_timeout_and_speed_bounds`. **No test for the 0.6 rad/s clip itself.** | The joint-limit study checks command bounds and raises on a violation; it passed. **Not measured in acceptance runs.** | PARTIALLY VERIFIED |
| REQ-25 | The simulated actuator limits hold on the servo setpoint:<br>• acceleration ≤ `max_acceleration_rad_s2` while speeding up;<br>• ≤ `max_deceleration_rad_s2` while braking;<br>• jerk ≤ `max_jerk_rad_s3`, except the counted one-step cut when a stop would otherwise reverse the motion (`stop_clamps`);<br>• no overshoot of a constant command;<br>• a zero command never reverses the motion.<br>Every stop's physical response is measured: stopping time, stopping distance, deceleration and jerk ([actuator model](ACTUATOR_MODEL.md)). | Stops and resumes that are more violent than assumed; stop behaviour that is claimed but not measured. | `ActuatorModel.step()` (`actuator.py`), applied after the collision guard and backstop in `Simulation.advance()`; `Simulation.motion_log`; `stop_response.link()`. | `tests/test_actuator.py`:<br>• 14 unit tests: limits at several speeds and directions, stop while accelerating, reversal, a provoked stop clamp, random sequences with and without stops, repeated cycles, passthrough;<br>• 10 MuJoCo tests: measured stops at 4 joints × 4 speeds, a stop while accelerating, resume before standstill, 8 pause/resume cycles, backstop, reset, close, labels, collision block.<br>`tests/test_stop_response.py`. RT watchdog-stop and hold tests check the physical stop and the resume. | `results/stop-response/20260930-004121`:<br>• 252 constant-speed stops over 3 profiles, with setpoint jerk and deceleration within the limits in every stop;<br>• 0 stop clamps in the stops, the cycles and the realtime runs;<br>• 20-cycle pause/resume with stop times 130–134 ms (`default`);<br>• realtime watchdog stops, manual stops and holds.<br>The stops and cycles are deterministic; the run was made in the cloud workspace. | VERIFIED |
| REQ-26 | The actuator model brakes at least as fast as the collision guard's braking reserve assumes: worst-case stopping travel at 0.6 rad/s ≤ speed × `braking_time_s` (0.12 s), including a 15 ms servo allowance. A profile that fails is rejected at start-up. | Stopping distance longer than the clearance reserve, so a stop that the guard expects to end with clearance ends in contact. | `ActuatorModel.check_braking()` called by `Simulation.__init__`; `Simulation` raises on a failing profile. | `tests/test_actuator.py`: `test_file_profiles_are_valid_and_pass_the_braking_check`, `test_braking_slower_than_the_collision_guard_assumes_is_rejected`, `test_collision_block_is_recorded_as_a_physical_stop`, and the per-stop travel bound in `test_stop_is_measured_for_several_speeds_and_joints`. `tests/test_collision.py` passes with the model on. | Worst-case equivalent 108 ms (`default`) and 111 ms (`gentle`) vs 120 ms. Stop-response study (`20260930-004121`):<br>• measured joint travel within the bound in all 252 stops;<br>• 42 approaches to the obstacle (joints 0–2, 3 speeds, 3 profiles, clearance checked every physics step): minimum slack above the margin 0.36 mm with `default`, 0.32 mm with `ideal`, 0 contacts. The guard had already slowed the arm to ≤ 0.012 rad/s when it blocked, so no full-speed stop next to an obstacle was tested.<br>**Clearance with the model on is not measured in acceptance or stress runs, and the bound is a joint-space linearisation, not a formal guarantee.** | PARTIALLY VERIFIED |
| REQ-14 | Every declared start pose is checked against joint limits and collision clearance before a test run. | A run starts in contact or beyond a limit, which invalidates it and, on hardware, is unsafe from the first command. | `Simulation.reset()` raises on an unsafe pose; preflight loops in the acceptance, margin and campaign tools. | `tests/test_collision.py::test_contacts_enabled_and_unsafe_reset_is_atomic`. **No test of the harness preflight loop.** | Runs refuse to start on a failed preflight, but the manifests do not record the preflight result. | PARTIALLY VERIFIED |
| REQ-15 | When no safe motion exists, the robot stops (`collision_blocked`) with zero command. A search skips at most four blocked waypoints per update. | Pushing against a blocked direction: oscillation at the obstacle or an endless search beside it. | `skip_blocked_motion()`, the `collision_blocked` stop in `realtime.py` and `app.py`, and the planner check budget. | `tests/test_collision.py`: `test_blocked_manual_motion_is_reported_by_app` (lab), `test_all_blocked_waypoints_finish_with_bounded_zero_motion`, `test_planning_budget_fails_closed_and_releases_scratch_budget`. **No test of the realtime `collision_blocked` path.** | Collision study: detours and skipped waypoints are recorded, and all 9 cases ended safely. | PARTIALLY VERIFIED |
| REQ-16 | Search and recovery are bounded:<br>• startup search ≤ 180 s;<br>• recovery ≤ 20 s per episode and 45 s in total, at most 3 episodes, at most 60° excursion;<br>• IBVS timeout 20 s;<br>• realtime run limit 120 s. | With the target absent, an unbounded search keeps the robot moving and widens the swept workspace. | Limits in `startup_search.py`, `recovery.py`, `control.py` and `realtime.py`, from their config files. | `tests/test_startup_search.py::test_timeout_is_terminal_even_if_marker_appears_later`, `tests/test_search_coverage.py::test_deadline_covers_both_passes_and_never_restarts`, `tests/test_recovery.py::test_repeated_losses_and_total_deadline_cannot_restart_forever` and `::test_absent_target_times_out_with_bounded_commands`, `tests/test_control.py::test_timeout_stops`. | Startup, search-coverage and joint-limit studies: every negative control ended in a bounded outcome (`target_not_found`, `timeout`, `recovery_limit` or `reposition_timeout`), with zero command. | VERIFIED |

### Task performance and acceptance

| ID | Requirement | Hazard / risk | Enforcement mechanism | Verification method / test | Evidence / latest result | Status |
|---|---|---|---|---|---|---|
| REQ-17 | Every attempt ends with camera and tool position error ≤ 2 mm and orientation error ≤ 1° (acceptance test; accuracy study). | A "converged" alignment is physically off, and the error carries into whatever the tool does next. | Precision stopping gates decide when to stop (`precision.py`, `control.py`). The harness measures error against the simulated ground-truth pose and fails the run. | `tests/test_accuracy.py::test_physical_acceptance_requires_both_frames_and_orientation`; `tools/test_acceptance_campaign.py::test_accuracy_and_latency_fail` (position above 2 mm only). **No acceptance-harness test with an error above the limit, and no harness test of the 1° limit.** | Acceptance run: worst 0.463 mm / 0.097° (SIFT) and 0.606 mm / 0.125° (Learned GPU). `results/accuracy/20260921-precision-final`: 72/72 within tolerance. Earlier degraded runs with the stop response failed (e.g. 5.19 mm; 36.4 mm / 4.59°). The docs call the thresholds illustrative, not a robot specification. | PARTIALLY VERIFIED |
| REQ-18 | An alignment is reported as converged only after the precision gates hold for 0.5 s, and the error stays below the threshold with zero command after the stop. | False success reported on a transient good frame or while still moving. | IBVS hold (`control.py`), precision gates (`precision.py`), post-stop checks in the study runners. | `tests/test_control.py::test_success_requires_full_observed_hold_window`; `tests/test_precision.py::test_hold_resets_after_any_gate_fails` and `::test_coupled_translation_rotation_under_one_pixel_does_not_stop`; `tests/test_precision_integration.py`; `tests/test_benchmark.py::test_convergence_and_complete_stopped_observation`; `tests/test_camera_delay_integration.py::test_repeated_display_frames_cannot_confirm_convergence`. | `results/accuracy/20260921-precision-final` (precision stopping): 72/72 trials passed, each rechecked through 30 post-stop captures. The earlier recovery study (200/200 stayed below 1 px after stopping) used the older 1 px rule. | VERIFIED |
| REQ-19 | Success rate: each method's 95% Clopper–Pearson lower bound is ≥ 95%, with 0 failed alignments (acceptance test and campaign; [statistics](STATISTICS.md)). | A small, unmeasured failure rate leaves the robot stopped short of the goal in routine use; "100%" from few trials hides it. | Harness gate (`run_acceptance_test.py`, `binomial_ci.py`). | `tests/test_binomial_ci.py`; AT `test_ten_of_ten_is_not_enough_to_claim_95_percent`, `test_success_gate_uses_the_lower_confidence_bound`, `test_failed_alignment_rule_is_separate_from_the_confidence_rule`; `tools/test_acceptance_campaign.py::test_too_few_alignments_cannot_claim_95_percent`. | **No saved run has been judged under this rule.** The acceptance run was judged on "100% observed". Re-evaluated with the current rule, the same data passes (lower bounds 97.6% / 97.5%), but that re-evaluation is not saved. | PARTIALLY VERIFIED |
| REQ-20 | Perception rejects blank, wrong-ID, wrong, mirrored, occluded and absent targets in the tested images and negative controls: no control features and no false acquisition. This is not a guarantee for every image. | Aligning to the wrong object drives the tool to a wrong pose with full confidence. | ID check (ArUco); inlier, homography, area and coverage gates (`perception.py`, `learned_perception.py`). | `tests/test_alignment_regression.py::test_blank_wrong_id_and_occluded_images_are_rejected`; rejection tests in `tests/test_natural_perception.py`; `tests/test_learned_perception.py::test_blank_wrong_mirrored_and_tiny_evidence_are_rejected`; `tests/test_natural_integration.py::test_wrong_and_absent_targets_cannot_supply_natural_measurements`. | Negative controls in the natural-image, learned, startup, coverage and joint-limit studies: no false acquisition (two to four controls per study). | VERIFIED |
| REQ-21 | Acceptance latency gates:<br>• processing p95 / p99 / max ≤ 150 / 175 / 250 ms;<br>• capture-to-command p99 / max ≤ 250 / 400 ms;<br>• later / first processing p99 ≤ 1.5×. | Commands act on older images: overshoot, more watchdog holds, and gradual degradation of a reused worker. | Harness reporting gates only. At runtime, latency is bounded by REQ-01 and REQ-05. | AT `test_later_alignment_regression_fails_learned_only`; `tools/test_acceptance_campaign.py::test_accuracy_and_latency_fail` (processing max). **No acceptance-harness test that exceeds the p95, p99 or capture limits.** | Acceptance run:<br>• processing 64.9 / 72.3 / 97.3 ms (SIFT), 58.9 / 67.1 / 97.0 ms (Learned);<br>• capture 129.1 / 154.2 and 121.4 / 152.1 ms;<br>• later / first 1.20× and 1.39×.<br>Run `20260928-173556-long` failed (Learned p95 214.4 ms, 2.29×). Results depend on the hardware state. | PARTIALLY VERIFIED |
| REQ-22 | Protected production inputs (controller, calibration, scene, safety and transport files) match the validated baseline, except reviewed changes pinned in `tools/approved_changes.json`. | A test certifies code other than the code that runs. | `baseline_check()` fingerprint before and during every acceptance and margin run. | AT `test_approved_change_must_match_both_hashes`, `test_checked_in_approval_matches_the_repository`, `test_protected_inputs`. | The acceptance-run report lists the approved `realtime.py` change and identical runtime versions. The actuator-model change (`simulation.py`, `realtime.py`, `actuator.py`, `actuator_config.json`) is pinned the same way; no acceptance run has used it yet. | VERIFIED |
| REQ-23 | Timing runs follow `TEST_PROCEDURE.md`: fresh restart (≤ 60 min uptime), AC power, Best performance, other programs closed. | Results measured under unknown load cannot be compared or reproduced; background load has already caused false failures. | AC power is checked (the run is INVALID otherwise). Uptime and power mode are recorded and reported, not gated. Other programs are not checked. | AT `test_conditions_met`, `test_conditions_broken_are_flagged` (report text only). **No test of the AC-power INVALID gate.** | Margin run `20260929-110852`: fresh restart **yes**. Acceptance run: fresh restart **no** (265 min uptime). | PARTIALLY VERIFIED |

## Hazard analysis

Severity is judged for a physical robot running the same software. In simulation
none of these hazards can cause damage, which is why the scope note above matters.

| ID | Consequence if violated | Severity | Main residual risk |
|---|---|---|---|
| REQ-01 | The arm keeps moving on outdated perception and can overshoot the goal or drive into the target or the workspace. | High | Lateness under the hold default is tested but not recorded in a saved run. The requirement covers the command only. With the assumed actuator model the arm stops physically up to about 150 ms later from 0.35 rad/s, moving the camera up to 17 mm. Real braking is unknown. |
| REQ-02 | A late or replayed image produces a jump toward an old target position. | High | None known in simulation. It relies on correct timestamps from the camera. |
| REQ-03 | While the loop is stalled nothing checks freshness or collisions, and on hardware the last command keeps executing. | High | The overrun path is untested, misses were observed under stress, and Windows is not a real-time OS. |
| REQ-04 | Unexpected motion after a stop, when people assume the robot is stationary. | High | None known. |
| REQ-05 | Resuming on stale data, or an alignment that stays "alive" and restarts much later. | Medium | No hold has run out the 2 s window in a real run, and the 5% budget has not been exercised. Repeated stop and go is simulated with the assumed actuator limits only, and untested on real motors. |
| REQ-06 | Perception failure goes unnoticed, and the controller holds a command or waits indefinitely. | High | `clock_error` and realtime `inference_failed` are untested. |
| REQ-07 | Commands act on steadily older images, and memory grows. | Medium | None known. |
| REQ-08 | One large step skips the per-step collision and limit checks. | High | Never exercised. |
| REQ-09 | Same as REQ-01, in the desktop lab. | Medium | None known in the lab mode. |
| REQ-24 | A hung or crashed controller leaves the motors running. | High | Not implemented. It must be solved in hardware or drive firmware before any physical use. |
| REQ-10 | Physical collision damage. | High | Evidence comes from 9 fixed collision cases plus contact-free acceptance runs. |
| REQ-11 | No margin left for braking or model error, so small disturbances cause contacts. | High | Clearance is not measured in acceptance or stress runs. The braking reserve is not a formal guarantee. |
| REQ-12 | Contact with mechanical stops, and alignment failure near limits. | High | Not measured in acceptance or realtime runs. The backstop is tested at 0.6 rad/s on one joint in simulation. |
| REQ-13 | Longer stopping distance than the collision guard assumes. | Medium | The 0.6 rad/s clip is untested, and speed is not measured in acceptance runs. |
| REQ-25 | Violent stops and resumes, or stop behaviour that is claimed but not measured. | Medium | The limits are assumptions. Joints are not synchronised, so the camera path can deviate briefly during ramps. |
| REQ-26 | A stop the guard expects to end with clearance ends in contact. | High | The check is a joint-space approximation with a 15 ms servo allowance measured only in MuJoCo. Clearance with the model on is not measured in acceptance runs. |
| REQ-14 | The first command already starts from an unsafe configuration. | Medium | The preflight is not recorded or tested. |
| REQ-15 | Oscillation or repeated pushing at an obstacle. | Medium | The realtime path is untested. |
| REQ-16 | Endless motion when the target is absent. | Medium | None known. |
| REQ-17 | The task silently fails with the tool placed millimetres off. | Medium (task) | Clean runs only. The thresholds are illustrative, and there is no real sensor or encoder error. |
| REQ-18 | Success is reported falsely, or the robot stops while still moving. | Medium | None known. |
| REQ-19 | Unknown failure rate in routine use. | Medium | No saved run has been judged under the current rule yet. |
| REQ-20 | Confident alignment to the wrong object. | High | Only a few negative images and controls are tested. |
| REQ-21 | Lag-driven overshoot and more watchdog holds. | Low to medium | Depends on the hardware, and most thresholds lack a test with a value above the limit. |
| REQ-22 | Evidence refers to different code than the code that runs. | Medium | None known. |
| REQ-23 | Unrepeatable or misleading results. | Low | Only AC power is enforced. |

## Requirements without a verification test

- **REQ-08:** physics-gap capping.
- **REQ-24:** actuator-side command timeout (not implemented).
- **REQ-03:** the `control_overrun` stop itself. Only the counter's effect on the
  verdict is tested.
- **REQ-06:** `clock_error` and the realtime `inference_failed` stop.
- **REQ-13:** the 0.6 rad/s speed clip.
- **REQ-14:** the harness preflight loop.
- **REQ-15:** the realtime `collision_blocked` path.
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

1. **Safety quantities not measured where they matter most.** The acceptance, margin
   and stress runs count contacts but do not record minimum clearance, minimum
   joint margin or peak joint speed. REQ-11 to REQ-13 therefore rest on unit tests
   and dedicated studies. Recording these three values per session (the collision
   and robustness runners already compute them) would close the largest gap.
2. **Mechanisms without tests:** `control_overrun`, `clock_error`, the realtime
   `inference_failed` and `collision_blocked` paths, physics-gap capping, the joint
   backstop and the speed clip. Each could be exercised with an injected fault in the
   style of the existing `inference_stall_s` tests.
3. **Evidence older than the current behaviour.** Watchdog lateness was measured only
   with the stop setting. No saved acceptance run has been judged under the current
   success rule. The margin runs held and resumed many times with the 2 s window, but
   no hold has run out the window in a real run. The acceptance run saw no holds, so
   the 5% pause budget has not been exercised.
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

## Keeping this page current

When a requirement, test or criterion changes, update its row. Keep each Evidence
cell pointing at the newest saved result that supports it, and name the run folder.
A status may move up to VERIFIED only when both an automated test and saved evidence
exist for the current behaviour.
