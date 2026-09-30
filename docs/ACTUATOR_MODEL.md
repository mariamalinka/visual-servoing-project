# Actuator dynamics model

- **Status:** added on 2026-09-30. On by default (profile `default`); profile `ideal`
  switches it off and restores the previous behaviour exactly.
- **Code:** `outputs/visual-servoing-simulation/actuator.py`,
  `actuator_config.json`. The model is applied in `Simulation.advance()`, and physical stops
  are measured in `simulation.py` and `stop_response.py`.
- **Evidence:** `results/stop-response/20260930-004121`
  (`run.cmd --stop-response --accuracy 5`), `tests/test_actuator.py` and the realtime
  process tests.

**This is a simulation assumption, not a model of a specific robot.** The limits
below were chosen as plausible values that fit the collision guard. They were not
measured on any drive. Nothing on this page is evidence about the safety of real
hardware. Real stopping behaviour must be measured on the robot.

## Why

The robot model before this change used MuJoCo velocity servos that received the
controller's command directly. When the command dropped to zero, the servo braked at
up to about 200 rad/s² and the arm stopped within about 10 to 100 ms. The
stop therefore looked almost instantaneous. Real drives do not behave like this.
Their trajectory generator limits acceleration and jerk, so the robot keeps moving
for a while after a stop and needs time to reach speed when it starts again. Because
of this, the project's stop requirements described the *command*, and
`SAFETY_TRACEABILITY.md` listed physical stopping as a gap.

## Where the model sits

```
controller / runtime  ->  command_velocity()       clip to 0.6 rad/s
                          collision guard           braking reserve, block
                          joint-limit backstop      now stops early by the stopping distance
                          = velocity_command        "the command" (a stop zeroes it at once)
                          ActuatorModel.step()      acceleration, deceleration and jerk limits   <- new
                          = actuator.velocity       servo setpoint
                          MuJoCo velocity servo     kv gain, force limit (scene.xml)
                          = data.qvel               measured joint velocity
```

This is the least disruptive place for the model:

- **The controller is unchanged.** `control.py`, `recovery.py` and the other
  controllers still produce velocity commands exactly as before, and they still see
  the measured joint state.
- **The runtime logic is unchanged.** Every stop, hold, collision block and shutdown
  still sets the command to zero at the same moment. `realtime.py` only passes the
  stop reason and records a command serial number so each event can be matched to its
  physical stop.
- **The command-level counters keep their meaning.** `unsafe_motion_ticks`,
  `post_stop_motion_ticks` and `paused_motion_ticks` count *commanded* motion. A stop
  still makes that zero immediately, and the counters stayed 0 in every realtime run
  with the model.
- **Every user of `Simulation` gets the model.** This includes the desktop lab, the
  studies, the tests and the realtime runtime.

## The model

For each joint, the setpoint velocity *v* follows the command *c* under these limits:

| Limit | `default` | `gentle` | Applies when |
|---|---:|---:|---|
| `max_acceleration_rad_s2` | 2.0 rad/s² | 1.0 rad/s² | the speed \|v\| increases (start, resume) |
| `max_deceleration_rad_s2` | 5.0 rad/s² | 5.0 rad/s² | the speed \|v\| decreases (stop, hold, braking) |
| `max_jerk_rad_s3` | 150 rad/s³ | 100 rad/s³ | always |

- **No overshoot.** The acceleration is ramped back to zero early, so a constant
  command is reached exactly. The model plans in discrete 2 ms steps, so the limits
  hold on the setpoint itself (tested).
- **A stop never reverses.** If the command drops to zero while the setpoint is
  braking hard through a reversal, the jerk limit would carry it past zero and
  briefly backwards. Instead, the setpoint stops at zero and its acceleration is cut
  to zero in that single step. **This is the one exception to the jerk limit.** Such
  steps are counted as `stop_clamps` and reported.
  - A unit test provokes the case on purpose, and random command sequences with
    frequent stops produce it regularly.
  - None occurred in the study's stops, cycles or realtime runs. There, commands
    change sign only through a stop or slowly.
- **Emergency stops use the same deceleration.** Manual Stop and watchdog stops are
  handled like any other zero command. The model has no separate faster emergency
  deceleration.
- **Limits can be set per joint.** Each limit can be one number for all joints or six
  numbers, one per joint.
- **`ideal` switches the model off.** The command reaches the servo unchanged, which
  is the behaviour before this change.

From constant speed *v*, the setpoint needs *v/D + D/J* seconds to stop and travels
*v·(v/D + D/J)/2* radians. For `default` this is 103 ms and 0.018 rad from 0.35 rad/s,
and 153 ms and 0.046 rad from 0.6 rad/s. After the setpoint reaches zero, the MuJoCo
servo adds a further lag.

### Consistency with the collision guard

The collision guard (`collision.py`) keeps a braking reserve of `closing speed ×
braking_time_s` (0.12 s). In other words, it assumes a joint moving at speed *v*
stops within *v × 0.12 s* of travel. When the collision guard is on, `Simulation`
refuses to start with an actuator profile that breaks this assumption.

The check uses the worst case:
- the joint moves at the 0.6 rad/s cap;
- it is still accelerating at the full limit;
- it then brakes;
- a servo lag allowance of 15 ms (`servo_lag_allowance_s`) is added.

| Profile | Worst-case stop, as equivalent time | Guard assumption |
|---|---:|---:|
| `default` | 108 ms | 120 ms |
| `gentle` | 111 ms | 120 ms |

A profile with a deceleration of 2 rad/s², for example, is rejected with a message
naming `braking_time_s`. In the study, measured joint travel stayed within this bound
in all 252 stops, including the servo lag.

### Joint-limit backstop

`Simulation.advance()` zeroes an outward command 0.03 rad before the hard joint limit.
With the model on, the joint would then keep moving for its stopping distance, so
the backstop now triggers earlier by that distance.

The cut is also latched: the outward command stays zero until the command stops
pointing outward, so a retreat is allowed at once. Without the latch, the
conservative distance would release the command again short of the line. The joint
would then creep toward the line in repeated brake–accelerate cycles, which the
independent review found.

With `ideal` the distance is zero and the original, unlatched backstop runs
unchanged. A test drives joint index 3
(the first wrist joint) at 0.6 rad/s toward its limit and checks that it stops at
least 0.02 rad short of it.

## Three different stop quantities

| Quantity | From | To | Clock | Measured by |
|---|---|---|---|---|
| **Command stop latency** | the image-age limit (or other stop trigger) | zero velocity command | wall clock | `realtime.py` events (`stopped_s − deadline_s`) |
| **Physical stopping time** | zero velocity command | standstill | simulated time | `Simulation.motion_log` |
| **Physical stopping distance** | pose at the zero command | pose at standstill | — | largest joint travel (rad), camera and tool displacement (mm), camera rotation (°) |

**Standstill** means every joint's *measured* velocity (after the servo) stays below
`stopped_velocity_rad_s` = 0.001 rad/s for `stopped_hold_s` = 20 ms. The time is
counted to the start of that 20 ms window. Both values are settings in
`actuator_config.json`.

A stop record also includes:
- the speed at the command;
- the peak measured deceleration and the peak setpoint acceleration;
- jerk, for both the setpoint and the measured motion (finite difference over one
  2 ms step);
- the reason:
  - the runtime's reason (`stale_camera`, `watchdog_pause`, `stopped`,
    `collision_blocked`, `shutdown`, …);
  - `collision_guard` or `joint_limit_backstop` when a safety layer reduced a
    requested motion to zero;
  - otherwise `zero_command`;
- the outcome, one of:
  - `stopped`;
  - `resumed` (a new command arrived before standstill);
  - `reset`;
  - `closed` (the simulation ended first);
  - `stopped_again`.

When joining with runtime events, a zero command can have two other results:
- `at_rest`: nothing to stop;
- `not_measured`: the robot was moving, but no physics step followed, as at shutdown
  or with a resume in the same control tick.

A zero command during a resume that never produced motion is shown as `not_started`.
None of these are reported as a stop of zero length.

Resumes and starts are recorded as `start` records with the time taken to reach the
command. In the realtime runtime, the three quantities are reported per event by
`stop_response.link()`.

The acceptance test, margin test and latency study now show the physical values next
to the command values. They are informational and are not pass criteria.

## Results

Source: `results/stop-response/20260930-004121`. This run was made in the cloud
workspace, not on the laptop.

The stop, cycle and accuracy parts are deterministic and do not depend on the
machine. The realtime part is wall clock. Its latencies are host-dependent, and it
should be repeated on the laptop with `run.cmd --stop-response`.

### Stops from constant speed

The study covers 252 stops:
- 14 motions: each joint in both directions, plus all joints together;
- 3 speeds;
- 2 poses (home and the standard start);
- 3 profiles.

Worst case per speed:

| Profile | From | Physical stopping time | Largest joint travel | Camera travel | Peak deceleration |
|---|---:|---:|---:|---:|---:|
| ideal | 0.1 rad/s | 68 ms | 0.0012 rad | 1.1 mm | 33 rad/s² |
| ideal | 0.35 rad/s | 88 ms | 0.0041 rad | 3.8 mm | 115 rad/s² |
| ideal | 0.6 rad/s | 96 ms | 0.0070 rad | 6.6 mm | 198 rad/s² |
| default | 0.1 rad/s | 96 ms | 0.0035 rad | 2.7 mm | 3.8 rad/s² |
| default | 0.35 rad/s | 152 ms | 0.0212 rad | 17.3 mm | 5.2 rad/s² |
| default | 0.6 rad/s | 204 ms | 0.0514 rad | 44.2 mm | 5.3 rad/s² |
| gentle | 0.35 rad/s | 164 ms | 0.0240 rad | 19.9 mm | 5.1 rad/s² |
| gentle | 0.6 rad/s | 218 ms | 0.0567 rad | 50.2 mm | 5.2 rad/s² |

- **Braking is gentler.** With the model, peak deceleration falls from up to about
  200 rad/s² to the 5 rad/s² limit. Measured jerk falls from up to about
  66,000 rad/s³ (a step response) to at most 170 rad/s³.
- **Stops take longer and travel further.** Stopping time is about 1.4 to 2.1× the
  `ideal` value with `default`, and up to 2.3× with `gentle`. Stopping distance grows much more: about 5× for the camera from
  0.35 rad/s, and about 7× from 0.6 rad/s.
- **The prediction holds.** The setpoint stop time matched the analytic prediction
  to within 2 ms in 250 of 252 stops. In the other two, the collision guard was
  already slowing the joint, so the setpoint stopped sooner than predicted. The
  prediction assumes braking starts from zero acceleration and is conservative.
- **The servo adds up to about 50 ms.** Measured standstill came up to 52 ms after
  the setpoint reached zero. That is for the heavy shoulder joints; for the light
  wrist joints the lag is close to zero. In the shoulder example (0.35 rad/s), the
  measured speed was already below 0.01 rad/s 16 ms after the setpoint reached zero.
  Most of the lag is that slow tail.

**Starts:**

| Profile | Time to reach 0.35 rad/s | Time to reach 0.6 rad/s | Peak acceleration |
|---|---:|---:|---:|
| default | 188 ms | 312 ms | 2.0 rad/s² |
| gentle | 358 ms | 608 ms | 1.0 rad/s² |

With `ideal`, the setpoint reached the command in a single 2 ms step, and the
measured acceleration peaked at about 200 rad/s².

### Repeated pause/resume

The cycle test ran 20 cycles with all six joints moving at up to 0.35 rad/s. Each
cycle moved for 0.4 s, alternating direction, then held at zero:
- A 40 ms hold resumed while the arm was still braking. Of those stops, 19 were
  interrupted and the last reached standstill.
- A 300 ms hold resumed from standstill. All 20 stops reached standstill.

With `default`:
- The stop time varied by at most 4 ms across the 20 cycles (130–134 ms), so nothing
  accumulates.
- Setpoint jerk stayed at or below 150 rad/s³ and acceleration at or below
  5 rad/s², including resumes during braking.
- There were no stop clamps and no forbidden contacts.

### Approaching the obstacle

The study drove joints 0–2 in both directions toward the mapped obstacle, at 3 speeds,
from home and from the start pose. Clearance was checked after every 2 ms physics
step, until 0.5 s after the guard blocked the motion. The guard engaged in 14 cases
per profile.

| | ideal | default | gentle |
|---|---:|---:|---:|
| Minimum slack above the 12 mm / 6 mm margin | 0.32 mm | 0.36 mm | 0.36 mm |
| Forbidden contacts | 0 | 0 | 0 |

The collision guard's braking reserve had already slowed the arm to at most
0.012 rad/s when it blocked the motion. So these cases confirm the gradual slow-down
with the model on. They do not test a sudden stop at full speed next to an obstacle;
that case is covered only by the worst-case braking check above.

### Realtime runtime

All realtime runs used the 400 ms image-age limit. Stops used the stop setting:
- **Watchdog stops:** the sensor was stalled at six different times after Align.
- **Manual stops:** Stop was pressed at three different times after the first
  command.

| | ideal | default |
|---|---:|---:|
| Watchdog: speed at the zero command | 0.018–0.077 rad/s | 0.018–0.081 rad/s |
| Watchdog: command stop latency | 0.2–2.0 ms | 0.4–2.1 ms |
| Watchdog: physical stopping time | 32–50 ms | 42–72 ms |
| Watchdog: image age at standstill | 433–452 ms | 442–474 ms |
| Watchdog: camera travel after the command | ≤ 0.5 mm | ≤ 1.3 mm |
| Manual Stop: physical stopping time (0.05–0.08 rad/s) | 46–50 ms | 64–74 ms |
| Manual Stop: camera travel | ≤ 0.5 mm | ≤ 1.5 mm |
| Unsafe / post-stop command ticks, all runs | 0 / 0 | 0 / 0 |

In these runs the controller was already slowing near the goal when the stop came,
so the arm moved slowly. The constant-speed stops above are the worst case.

**Hold and resume (the runtime default).** In this run every frame was delayed by
0.3 s, so the watchdog held and resumed repeatedly.

| | ideal | default |
|---|---:|---:|
| Holds while moving | 9 | 9 |
| Reached standstill | 8 | 8 |
| Resumed while still braking | 1 | 1 |
| Longest physical stop | 48 ms | 70 ms |
| Camera travel per hold | ≤ 0.5 mm | ≤ 1.1 mm |

With `default`:
- setpoint jerk stayed at or below 150 rad/s³ and setpoint acceleration at or below
  3.4 rad/s² through holds and resumes;
- there were no stop clamps and 0 motion ticks during holds.

**A finding worth knowing about hold and resume.** When every frame is late, each
resumed image is itself already close to the age limit. With `default`, the next
hold came at most 18 ms after each resume, and the arm reached only 0.022 rad/s
between holds. Without the model the servo received the full command at once
during those bursts. So ramped actuators make stop-and-go
progress under heavy latency slower per resume.

To see whether that changes outcomes, the study ran whole ArUco alignments with
hold and resume at six constant per-frame delays, one alignment per level and
profile. The outcome was the same with and without the model at every level:

| Added delay | Outcome, both profiles |
|---|---|
| 0, 50 and 100 ms | converged in 3.5–4.4 s with no holds |
| 150 and 250 ms | the 20 s controller time limit, with about 140–200 holds and 0.5–0.6 px error remaining |
| 300 ms | `alignment_stalled` |

This is one alignment per level, on the cloud workspace's CPU. The laptop check is the
latency-margin test below.

### Latency-margin test on the laptop, with the model on

Run `results/margin-test/20260930-225125` used the `default` profile and hold and
resume. It was compared with the last run before the model, `20260929-110852`.

**The margin is unchanged.** Both methods tolerate +150 ms of added per-frame delay
(10/10 at every level up to +150 ms) and fail at +200 ms, as before. The safety
counters were all 0: no unsafe or post-stop motion, no motion during a hold, no
contacts and no control-deadline misses.

**The cost is slower stop-and-go under heavy delay:**

| Level | Measure | SIFT, before → with model | Learned GPU, before → with model |
|---|---|---:|---:|
| +0 ms | Median time to converge | 4.2 → 4.2 s | 4.3 → 4.3 s |
| +125 ms | Median time to converge | 4.9 → 4.9 s | 6.8 → 8.8 s |
| +125 ms | Time held, 10 alignments | 5.8 → 7.4 s | 19.6 → 30.0 s |
| +150 ms | Median time to converge | 6.8 → 7.1 s | 9.5 → 10.9 s |
| +150 ms | Time held, 10 alignments | 20.1 → 23.1 s | 43.5 → 53.0 s |

- Up to +75 ms the times match the earlier run.
- At +125 and +150 ms, Learned GPU spent 22–53% more time held and took 15–30% longer
  to converge. SIFT's time held and time to converge grew by 0–27%.
- This is the effect the study predicted: each resume gives the ramped arm only a few
  milliseconds of motion before the next hold.

**Physical stops after holds:**
- up to 104 ms to standstill and 4.3 mm of camera travel, both at the failing +200 ms
  level;
- up to +150 ms, at most 88 ms and 1.9 mm.

**Limits:**
- **Not a fresh restart.** The laptop had been on for about 36 hours, so the run
  does not follow `TEST_PROCEDURE.md`. No low-power state was detected.
- **Different conditions.** The earlier run was made the day before, after a
  restart, so part of each difference can come from the laptop's state.
- **Small samples.** Each level has 10 alignments; 10/10 shows a success rate of at
  least 69%.

## The 400 ms watchdog requirement, re-evaluated

REQ-01 in `SAFETY_TRACEABILITY.md` requires a **command** stop: zero velocity at
most 50 ms after the image-age limit. The limit is 400 ms in the tests and 250 ms by
default in the runtime.

**The command requirement is unaffected by the model and still met.** The model
does not delay the zero command. The realtime tests still assert a command stop
latency below 50 ms, and they now also check that the physical stop completes.

**A physical requirement would be a different requirement.** "Stopped within 400 ms"
of image age cannot be met with `max_age_s` = 0.4 s by any profile, even `ideal`,
because the trip itself happens at 400 ms. Worst case, the arm reaches standstill at
an image age of about:

- `default`: 400 + 50 + 152 = **602 ms** for motion at the controller's 0.35 rad/s
  cap, and 654 ms at the 0.6 rad/s absolute cap;
- `ideal`: 538 ms and 546 ms.

The measured realtime trips reached standstill by 443–472 ms (`default`).

To guarantee physical standstill by 400 ms of image age with `default`, the trip
would have to happen at about **198 ms** of image age for 0.35 rad/s motion, or
146 ms at 0.6 rad/s. That is close to the image ages seen in normal operation. In the acceptance run
(`20260929-011129`), capture-to-command was p99 121–129 ms and at most 152–154 ms,
and the held image keeps ageing until the next result arrives. The watchdog would then trip in normal
operation.

**Recommendation:** keep REQ-01 as a command requirement. State the physical
consequence separately as a measured, reported quantity, "image age at standstill".
Neither the requirement nor `max_age_s` has been changed. Choosing a physical
stopping requirement is a decision for real hardware. It needs the robot's measured
braking, and it would more likely be met by limiting speed near obstacles than by a
shorter image-age limit.

## Effect on accuracy

A paired check compared `ideal` and `default` on the same five random starts per
method, with the same seeds, goals and references. The runs used nominal
calibration, a 100 ms camera delay and the accuracy study's 2 mm / 1° tolerance.

| | ArUco ideal | ArUco default | SIFT ideal | SIFT default |
|---|---:|---:|---:|---:|
| Converged within tolerance | 5/5 | 5/5 | 5/5 | 5/5 |
| Worst camera error | 0.66 mm / 0.061° | 0.68 mm / 0.062° | 0.28 mm / 0.037° | 0.27 mm / 0.037° |
| Worst tool error | 0.74 mm / 0.061° | 0.76 mm / 0.062° | 0.31 mm / 0.037° | 0.30 mm / 0.037° |
| Median time to converge | 5.4 s | 5.4 s | 5.6 s | 5.6 s |
| Camera travel after the final stop | ≤ 0.005 mm | ≤ 0.005 mm | ≤ 0.007 mm | ≤ 0.008 mm |

**The model did not measurably change final accuracy or convergence time in these
runs.** Near the goal the controller's commands are already very small, so the final
stop moves the camera by micrometres. The 10–20 mm stopping distances above matter
for stops in mid-motion (watchdog, collision, manual Stop), not for the final
precision stop.

This check has limits:
- It uses five starts per method. With 5/5, the success rate is only known to be at
  least 48% ([statistics](STATISTICS.md)), so it shows no accuracy regression in
  these cases. It does not measure a success rate.
- Learned GPU perception was not included, because it needs the laptop's GPU.
- The acceptance test on the laptop, which runs with the model on by default, is the
  larger check.

## Assumptions and simplifications

1. **Assumed limits.** Acceleration 2 rad/s², deceleration 5 rad/s² and jerk
   150 rad/s³ (`default`) are plausible for a small arm, but they are not taken from
   a data sheet or a measurement.
2. **Independent joints.** Joints are not time-synchronised. While joints ramp, the
   direction of motion in joint space, and therefore the camera path, can differ
   briefly from the commanded direction. Industrial controllers usually synchronise
   joints.
3. **Constant limits.** They do not depend on pose, payload, gravity torque,
   temperature, supply voltage or wear. The MuJoCo servo force limits in `scene.xml`
   still apply after the model. Force saturation was not logged. The measured
   deceleration stayed within 6% of the setpoint limit (peak 5.3 rad/s²), while the
   same servos braked at up to about 200 rad/s² without the model. So the model's
   limit, not the servo, set the braking in the study.
4. **One deceleration for every stop.** No separate emergency (category 0/1) stop
   profile, brake engagement or safe-torque-off is modelled.
5. **Reversal through zero.** When braking continues through zero into the opposite
   direction, the acceleration ramps from the deceleration limit to the acceleration
   limit at the jerk limit. It can briefly exceed the acceleration limit while the
   speed grows again.
6. **Discrete time.** The model is evaluated once per 2 ms physics step, like
   everything else in the simulation.
7. **Standstill threshold.** Standstill is |q̇| < 0.001 rad/s on every joint for
   20 ms. A different threshold gives different stop times: most of the servo's final
   tail is below 0.01 rad/s.
8. **Servo lag allowance.** The 15 ms allowance in the braking check is covered by
   the study (measured travel ≤ bound in all 252 stops), but only for this MuJoCo
   model.
9. **No actuator-side timeout.** The model does not stop the motors if commands stop
   arriving. REQ-24 is still not implemented.
10. **Realtime timing.** The realtime runtime integrates wall time. The physical
    stopping time in realtime reports is simulated time, which matches wall time
    except when `discarded_physics_s` is nonzero.
11. **Cost.** In one measurement during development in the cloud workspace, the model
    and the stop measurement added about 0.06 ms per 2 ms physics step (median). This
    is not part of the saved evidence. The acceptance test's control-deadline and
    loop-gap figures on the laptop are the relevant check.

## Configuration

- **Change a limit:** edit `actuator_config.json`. Values may be one number or six
  per-joint numbers. No code changes are needed. A profile that brakes too slowly for
  the collision guard is rejected when `Simulation` starts.
- **Select a profile:**
  - the whole project: `active_profile`;
  - the desktop app and the realtime runtime:
    `run.cmd --actuator ideal|default|gentle` (also with `--realtime`);
  - code: `Simulation(actuator_config='gentle')` or
    `RealtimeSession(actuator='gentle')`, or pass a dict with the profile fields.
- **Protected files.** `actuator.py` and `actuator_config.json` are protected
  production files in the acceptance test. The changes to `simulation.py` and
  `realtime.py` and the two new files are recorded in `tools/approved_changes.json`.
  Any other edit to them makes an acceptance run INVALID.

## Tests

`tests/test_actuator.py` (24 tests) and `tests/test_stop_response.py` (9 tests):

- **Setpoint limits:**
  - acceleration, deceleration and jerk limits hold on the setpoint;
  - no overshoot and exact arrival;
  - a stop never reverses;
  - speeds 0.05, 0.1, 0.35 and 0.6 rad/s in both directions;
  - a stop while still accelerating, a stop during a reversal, and random command
    sequences;
  - a provoked stop clamp, and random sequences with frequent stops (jerk limit
    held except at counted clamps, never a reversal);
  - 20 stop–start cycles;
  - passthrough with the model off.
- **Configuration:**
  - the braking check;
  - per-joint limits;
  - invalid configurations are rejected.
- **MuJoCo, 4 joints × 4 speeds:**
  - stop time within the prediction plus the servo allowance;
  - joint travel within the bound;
  - `ideal` compared with `default`;
  - a resume before standstill;
  - 8 pause/resume cycles;
  - a stop while still accelerating, within the predicted distance;
  - the joint-limit backstop: one latched stop, and retreat is allowed at once;
  - reset, and closing the simulation during a stop;
  - reason labels;
  - a collision block recorded as a physical stop with clearance kept.
- **Joining events and stops (`test_stop_response.py`):**
  - the three quantities are kept apart;
  - at rest, `not_measured`, `not_started` and evicted records;
  - summaries and merging.

The realtime process tests (`tests/test_realtime.py`) now also check two things:
- **Watchdog stop:** the arm reaches standstill, and within the predicted setpoint
  stop time plus 80 ms of servo and timing allowance. Measured from the image-age
  limit, standstill comes within 50 ms more. Setpoint jerk stays within the limit.
- **Hold and resume:** each hold either reaches standstill or is resumed before it.
  Each resume keeps setpoint jerk within the limit and acceleration within the
  deceleration limit. A resume that starts while the arm is still braking begins at
  the braking deceleration, so the acceleration limit alone would be too strict a
  check.

## Next steps

1. Run `run.cmd --stop-response` on the laptop, so the realtime part is measured on
   the target host.
2. Repeat the acceptance test with the model on; its report now includes the
   physical stop rows. The margin test has been repeated (unchanged at +150 ms; see
   above). Repeating it once after a fresh restart would remove the uptime caveat.
3. Before any hardware use, replace the assumed limits with the drive's measured
   braking and add an actuator-side timeout (REQ-24).
4. Consider a speed limit near obstacles and near the goal. It would bound the
   physical stopping distance directly, without shortening the image-age limit.
