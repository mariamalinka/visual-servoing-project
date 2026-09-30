# Stop and resume response with actuator dynamics

Run 2026-09-29T22:41:21.152398+00:00. Simulation only: the actuator limits are assumptions (`actuator_config.json`, `docs/ACTUATOR_MODEL.md`), not measurements of a real drive.

Note: The hold-and-resume rows and the delay levels were run after the main run, with the final run_stop_response.py: the main run ended its hold sessions before any hold (termination check fixed in run_holds), and the delay levels were added afterwards.

Definitions:

- **Command stop latency**: image-age limit to zero velocity command (wall clock, realtime runtime).
- **Physical stopping time**: zero command to standstill, every joint below 0.001 rad/s for 20 ms (simulated time).
- **Physical stopping distance**: motion between the zero command and standstill (largest joint angle, camera and tool displacement).

## Profiles

| Profile | Acceleration | Deceleration | Jerk | Worst-case check against the collision guard |
|---|---:|---:|---:|---|
| ideal | none | none | none | model off (previous behaviour) |
| default | 2.0 rad/s² | 5.0 rad/s² | 150.0 rad/s³ | 108 ms equivalent ≤ 120 ms |
| gentle | 1.0 rad/s² | 5.0 rad/s² | 100.0 rad/s³ | 111 ms equivalent ≤ 120 ms |

## 1. Stops from constant speed

252 stops: 14 motions (each joint both ways, all joints together) x 3 speeds x 2 poses per profile. Worst case per requested speed:

| Profile | Speed | Setpoint stop (predicted) | Physical stop | Largest joint travel (bound) | Camera travel | Camera rotation | Peak decel (measured) | Peak jerk setpoint / measured |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| ideal | 0.1 rad/s | 2 ms (0) | 68 ms | 0.0012 rad (0.0015) | 1.1 mm | 0.10° | 33.0 rad/s² | 0 / 11050 rad/s³ |
| ideal | 0.35 rad/s | 2 ms (0) | 88 ms | 0.0041 rad (0.0052) | 3.8 mm | 0.36° | 115.4 rad/s² | 0 / 38748 rad/s³ |
| ideal | 0.6 rad/s | 2 ms (0) | 96 ms | 0.0070 rad (0.0090) | 6.6 mm | 0.62° | 197.8 rad/s² | 0 / 66521 rad/s³ |
| default | 0.1 rad/s | 50 ms (52) | 96 ms | 0.0035 rad (0.0041) | 2.7 mm | 0.32° | 3.8 rad/s² | 150 / 170 rad/s³ |
| default | 0.35 rad/s | 102 ms (103) | 152 ms | 0.0212 rad (0.0233) | 17.3 mm | 2.14° | 5.2 rad/s² | 150 / 164 rad/s³ |
| default | 0.6 rad/s | 152 ms (153) | 204 ms | 0.0514 rad (0.0550) | 44.2 mm | 5.40° | 5.3 rad/s² | 150 / 164 rad/s³ |
| gentle | 0.1 rad/s | 62 ms (63) | 102 ms | 0.0041 rad (0.0047) | 3.2 mm | 0.39° | 3.1 rad/s² | 100 / 114 rad/s³ |
| gentle | 0.35 rad/s | 118 ms (120) | 164 ms | 0.0240 rad (0.0262) | 19.9 mm | 2.48° | 5.1 rad/s² | 100 / 112 rad/s³ |
| gentle | 0.6 rad/s | 168 ms (170) | 218 ms | 0.0567 rad (0.0600) | 50.2 mm | 5.99° | 5.2 rad/s² | 100 / 110 rad/s³ |

- Stops that did not reach standstill within 0.6 s: 0.
- Stops whose joint travel exceeded the model's bound (setpoint + servo allowance): 0.
- 2 stops started below 95% of the requested speed because the collision guard or joint-limit backstop was already slowing the joint; the prediction uses the actual speed.
- Measured jerk is the finite difference of MuJoCo joint accelerations over one 2 ms step. With the model off it reflects the servo's step response, not a drive limit.

Starts (0 to the requested speed):

| Profile | Speed | Time to reach command | Peak acceleration setpoint / measured | Peak jerk setpoint |
|---|---:|---:|---:|---:|
| ideal | 0.1 rad/s | 2 ms | 50.0 / 33.0 rad/s² | 0 rad/s³ |
| ideal | 0.35 rad/s | 2 ms | 175.0 / 115.4 rad/s² | 0 rad/s³ |
| ideal | 0.6 rad/s | 2 ms | 300.0 / 197.9 rad/s² | 0 rad/s³ |
| default | 0.1 rad/s | 62 ms | 2.0 / 2.1 rad/s² | 150 rad/s³ |
| default | 0.35 rad/s | 188 ms | 2.0 / 2.1 rad/s² | 150 rad/s³ |
| default | 0.6 rad/s | 312 ms | 2.0 / 2.1 rad/s² | 150 rad/s³ |
| gentle | 0.1 rad/s | 108 ms | 1.0 / 1.1 rad/s² | 100 rad/s³ |
| gentle | 0.35 rad/s | 358 ms | 1.0 / 1.1 rad/s² | 100 rad/s³ |
| gentle | 0.6 rad/s | 608 ms | 1.0 / 1.1 rad/s² | 100 rad/s³ |

## 2. Repeated pause/resume cycles

Six joints moving (up to 0.35 rad/s), alternating direction, 0.4 s of motion then a zero command. A 40 ms hold resumes while the robot is still braking; a 300 ms hold resumes from standstill.

| Profile | Hold | Cycles | Reached standstill | Resumed while braking | Stop time range | Peak accel setpoint / measured | Peak jerk setpoint | Stop clamps | Contacts |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ideal | 40 ms | 20 | 1 | 19 | 74–74 ms | 175.0 / 53.5 rad/s² | 0 rad/s³ | 0 | 0 |
| ideal | 300 ms | 20 | 20 | 0 | 70–74 ms | 175.0 / 53.5 rad/s² | 0 rad/s³ | 0 | 0 |
| default | 40 ms | 20 | 1 | 19 | 134–134 ms | 5.0 / 5.1 rad/s² | 150 rad/s³ | 0 | 0 |
| default | 300 ms | 20 | 20 | 0 | 130–134 ms | 5.0 / 5.1 rad/s² | 150 rad/s³ | 0 | 0 |
| gentle | 40 ms | 20 | 1 | 19 | 146–146 ms | 5.0 / 5.1 rad/s² | 100 rad/s³ | 0 | 0 |
| gentle | 300 ms | 20 | 20 | 0 | 142–146 ms | 5.0 / 4.8 rad/s² | 100 rad/s³ | 0 | 0 |

## 3. Approaching the obstacle

Single joints (0–2, both directions) driven toward the mapped obstacle from home and the start pose; clearance to the collision margin checked after every 2 ms physics step until 0.5 s after the guard blocked the motion. Only motions where the guard engaged are listed.

| Profile | Speed | Cases | Blocked | Minimum slack above the margin | Forbidden contacts | Camera travel after the block |
|---|---:|---:|---:|---:|---:|---:|
| ideal | 0.1 rad/s | 2 | 2 | 0.37 mm | 0 | 0.0 mm |
| ideal | 0.35 rad/s | 6 | 4 | 1.03 mm | 0 | 0.1 mm |
| ideal | 0.6 rad/s | 6 | 6 | 0.32 mm | 0 | 0.1 mm |
| default | 0.1 rad/s | 2 | 2 | 0.36 mm | 0 | 0.0 mm |
| default | 0.35 rad/s | 6 | 4 | 1.27 mm | 0 | 0.1 mm |
| default | 0.6 rad/s | 6 | 5 | 0.37 mm | 0 | 0.2 mm |
| gentle | 0.1 rad/s | 2 | 2 | 0.36 mm | 0 | 0.0 mm |
| gentle | 0.35 rad/s | 6 | 4 | 1.26 mm | 0 | 0.1 mm |
| gentle | 0.6 rad/s | 6 | 5 | 0.55 mm | 0 | 0.2 mm |

Negative slack would mean the arm entered the 12 mm (environment) or 6 mm (self) margin. The guard had already slowed the arm to at most 0.012 rad/s when the stop began, so these cases test the guard's gradual slow-down, not a sudden stop at full speed next to an obstacle. This is a check of the braking assumption in these cases, not a formal guarantee.

## 4. Realtime stops

Watchdog stops (stop setting, 400 ms image-age limit): the sensor stalls 0.8 s at different times after Align. Manual stops: Stop is pressed at different times after the first command. Latency is wall clock; stopping time is simulated (the runtime integrates wall time).

| Profile | Kind | Run | Speed at command | Command stop latency | Physical stopping time | Image age at standstill | Camera travel | Unsafe / post-stop command ticks |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| ideal | watchdog | 0 | 0.077 rad/s | 1.7 ms | 50 ms | 452 ms | 0.5 mm | 0 / 0 |
| ideal | watchdog | 1 | 0.058 rad/s | 2.0 ms | 50 ms | 452 ms | 0.5 mm | 0 / 0 |
| ideal | watchdog | 2 | 0.049 rad/s | 0.9 ms | 46 ms | 447 ms | 0.4 mm | 0 / 0 |
| ideal | watchdog | 3 | 0.027 rad/s | 0.3 ms | 34 ms | 434 ms | 0.1 mm | 0 / 0 |
| ideal | watchdog | 4 | 0.020 rad/s | 0.2 ms | 36 ms | 436 ms | 0.2 mm | 0 / 0 |
| ideal | watchdog | 5 | 0.018 rad/s | 0.6 ms | 32 ms | 433 ms | 0.1 mm | 0 / 0 |
| default | watchdog | 0 | 0.081 rad/s | 1.8 ms | 72 ms | 474 ms | 1.2 mm | 0 / 0 |
| default | watchdog | 1 | 0.063 rad/s | 1.8 ms | 72 ms | 474 ms | 1.3 mm | 0 / 0 |
| default | watchdog | 2 | 0.050 rad/s | 2.1 ms | 64 ms | 466 ms | 1.0 mm | 0 / 0 |
| default | watchdog | 3 | 0.029 rad/s | 1.2 ms | 46 ms | 447 ms | 0.3 mm | 0 / 0 |
| default | watchdog | 4 | 0.019 rad/s | 2.0 ms | 44 ms | 446 ms | 0.3 mm | 0 / 0 |
| default | watchdog | 5 | 0.018 rad/s | 0.4 ms | 42 ms | 442 ms | 0.2 mm | 0 / 0 |
| ideal | manual Stop | 0 | 0.076 rad/s | — | 50 ms | — | 0.5 mm | 0 / 0 |
| ideal | manual Stop | 1 | 0.063 rad/s | — | 50 ms | — | 0.5 mm | 0 / 0 |
| ideal | manual Stop | 2 | 0.050 rad/s | — | 46 ms | — | 0.3 mm | 0 / 0 |
| default | manual Stop | 0 | 0.079 rad/s | — | 66 ms | — | 1.1 mm | 0 / 0 |
| default | manual Stop | 1 | 0.070 rad/s | — | 74 ms | — | 1.5 mm | 0 / 0 |
| default | manual Stop | 2 | 0.051 rad/s | — | 64 ms | — | 0.9 mm | 0 / 0 |

Holds and resumes in the runtime default (hold and resume, 2 s window): every frame is delayed 0.3 s after 0.3 s, so the 400 ms watchdog holds and resumes repeatedly. With `default`, every resume was followed by the next hold within 18 ms (the resumed image is itself already close to the age limit), so the arm reached at most 0.022 rad/s between holds.

| Profile | Holds (while moving) | Reached standstill / resumed while braking / other | Longest physical stop | Camera travel | Resumes (reached the command before it changed) | Longest time to reach the command | Peak setpoint accel / jerk | Jerk-limit exceptions | Motion ticks during a hold |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ideal | 10 (9) | 8 / 1 / 0 | 48 ms | 0.5 mm | 9 (8) | 2 ms | 53.6 rad/s² / 0 rad/s³ | 0 | 0 |
| default | 10 (9) | 8 / 1 / 0 | 70 ms | 1.1 mm | 9 (0) | — | 3.4 rad/s² / 150 rad/s³ | 0 | 0 |

Whole ArUco alignments in the runtime default (hold and resume) with a constant added delay on every frame; one alignment per level and profile, container CPU:

| Added delay | ideal | default |
|---:|---|---|
| +0 ms | converged in 4.4 s, 0 holds, last error 0.20 px | converged in 4.4 s, 0 holds, last error 0.20 px |
| +50 ms | converged in 4.1 s, 0 holds, last error 0.18 px | converged in 4.0 s, 0 holds, last error 0.18 px |
| +100 ms | converged in 3.5 s, 0 holds, last error 0.16 px | converged in 3.6 s, 0 holds, last error 0.13 px |
| +150 ms | timeout, 203 holds, last error 0.59 px | timeout, 204 holds, last error 0.59 px |
| +250 ms | timeout, 143 holds, last error 0.52 px | timeout, 144 holds, last error 0.54 px |
| +300 ms | alignment_stalled in 19.7 s, 53 holds, last error 29.23 px | alignment_stalled in 8.7 s, 21 holds, last error 38.49 px |

`timeout` is the controller's 20 s alignment limit; the duration is missing where the event log (256 entries) no longer held the Align event. Same outcome with and without the model at every level in this single-run comparison.

## 5. The 400 ms watchdog requirement against physical motion

- **ideal**: worst physical stopping time 88 ms from 0.35 rad/s (the controller's joint speed limit) and 96 ms from 0.6 rad/s (the absolute cap). With the tested latency bound of 50 ms, the robot is at standstill by an image age of at most 538 ms (546 ms at the cap).
- **default**: worst physical stopping time 152 ms from 0.35 rad/s (the controller's joint speed limit) and 204 ms from 0.6 rad/s (the absolute cap). With the tested latency bound of 50 ms, the robot is at standstill by an image age of at most 602 ms (654 ms at the cap).
- **gentle**: worst physical stopping time 164 ms from 0.35 rad/s (the controller's joint speed limit) and 218 ms from 0.6 rad/s (the absolute cap). With the tested latency bound of 50 ms, the robot is at standstill by an image age of at most 614 ms (668 ms at the cap).

What 400 ms means therefore has to be stated explicitly:

- *Command* stop by 400 ms image age (the current requirement, REQ-01): unchanged and still met; the actuator model does not delay the zero command.
- *Physical* standstill by 400 ms image age: not met with the current `max_age_s` = 0.4 s by any profile, including the model switched off, because the trip itself happens at 400 ms. It would need the trip at about 400 − 50 − (physical stopping time) ms of image age:
  ideal: 262 ms for motion at up to 0.35 rad/s, 254 ms at up to 0.6 rad/s.
  default: 198 ms for motion at up to 0.35 rad/s, 146 ms at up to 0.6 rad/s.
  gentle: 186 ms for motion at up to 0.35 rad/s, 132 ms at up to 0.6 rad/s.

This study does not change the requirement or `max_age_s`; see docs/ACTUATOR_MODEL.md.

## 6. Accuracy with and without the actuator model

Same starts, seeds, goals and perception; nominal calibration; 100 ms camera delay. Worst over the 30 post-stop frames (accuracy_config.json tolerance: 2 mm and 1°).

| Profile | Mode | Converged within tolerance | Worst camera | Worst tool | Median time to converge | Worst camera travel in the final stop |
|---|---|---:|---:|---:|---:|---:|
| ideal | ArUco | 5/5 | 0.66 mm / 0.061° | 0.74 mm / 0.061° | 5.4 s | 0.005 mm |
| ideal | SIFT | 5/5 | 0.28 mm / 0.037° | 0.31 mm / 0.037° | 5.6 s | 0.007 mm |
| default | ArUco | 5/5 | 0.68 mm / 0.062° | 0.76 mm / 0.062° | 5.4 s | 0.005 mm |
| default | SIFT | 5/5 | 0.27 mm / 0.037° | 0.30 mm / 0.037° | 5.6 s | 0.008 mm |

Learned GPU perception needs the laptop's GPU and is not part of this container run.
