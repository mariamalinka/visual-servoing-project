# Camera delay and feedback age

Timed-camera mode models the delay between taking an image and delivering its
measurement to the controller. The robot continues its simulated motion during
that interval. The immediate-camera mode remains available as the historical
baseline.

## Try it

From the repository root:

~~~powershell
.\run.cmd --learned --camera-delay-ms 100
~~~

Use `--natural` for SIFT, or omit the matcher flag for ArUco.

- **C / Delay** cycles through timed 0, 50, 100 and 200 ms, then the immediate baseline. Changing timing stops the current motion; press **G** or load **Offset [O]** to align.
- **X** interrupts or restores camera capture in timed mode. Already captured frames can still arrive.
- If feedback becomes too old, the status shows **STALE CAMERA** and joint commands become zero. Restoring capture does not restart motion; press **G** explicitly.
- The wrist panel displays the delivered image with its matching outline, frame number, capture time and current age. The world panel shows the current robot.

Saved camera images include their capture-time joints and camera transform,
frame number, delivery time and save time in the accompanying metadata.

The default observation-age limit is **250 ms**, measured from capture, not
delivery. A 200 ms old image has about 50 ms of remaining usable age.
An explicit age limit can be selected with `--max-camera-age-ms`.

## Timing model

Physics runs in 2 ms steps. Camera acquisition follows its own 30 Hz simulated
schedule. Each image owns a copy of its pixels, capture-time joints and
observation. Its delivery time is capture time plus the configured delay.

The controller consumes each delivered frame once. Commands are held between
measurements. Display refreshes do not trigger inference, advance confirmation
counts, or provide a newer observation to the controller. A delayed detection
stores the robot pose at capture for later recovery; joint limits and differential
kinematics still use current measured joints.

The delivery queue has a finite capacity and drops its oldest pending frame on
overflow. If several frames become available between polls, the newest eligible
one is used. Expired frames cannot refresh the watchdog. Stop, reset, matcher
changes and manual interruptions invalidate queued observations. An unexpected
clock reset also clears frames and remembered capture poses before another run.

This is a **deterministic transport-delay model in simulated time**. Inference and
rendering still execute on the application's thread; their host execution time
does not become the configured transport delay. Inference duration is measured
separately. This implementation does not promise wall-clock real-time operation
or model a serial inference worker's throughput.

## Feedback loss and convergence

A watchdog is checked on physics ticks even when no camera measurement arrives.
It stops the run once the last usable observation reaches the age limit.
A first image that cannot arrive within the limit never authorizes movement.
An independent run deadline also advances without camera updates.

A fresh image in which the target is missing remains a live camera measurement:
the existing bounded target-recovery behavior handles it. An unavailable stream
or failed inference stops timed-camera motion. They do not authorize a blind
startup search.

Success still requires the existing 1 px threshold and hold duration. In timed
mode the hold must also span distinct captured images over the configured
duration. Repeated display frames cannot establish convergence.

## Configuration and source

| File | Responsibility |
|---|---|
| [camera_timing_config.json](camera_timing_config.json) | Delay, maximum observation age, queue capacity and run deadline |
| [camera_timing.py](camera_timing.py) | Capture scheduling, owned frames, delayed delivery and watchdog |
| [app.py](app.py) | Physics-tick scheduling, user controls, delivered-image display and cancellation |
| [recovery.py](recovery.py) and [joint_limits.py](joint_limits.py) | Capture-time view memory with current-joint motion limits |
| [run_camera_delay_study.py](run_camera_delay_study.py) | Paired closed-loop trials and stream-interruption checks |

For seeded jitter, packet loss, outages and repeatable starting poses, use the
[automated robustness experiment](CAMERA_ROBUSTNESS.md).

## Reproduce the study

~~~powershell
.\run.cmd --delay-study
~~~

The default study runs ArUco, SIFT and Learned at 0, 50, 100 and 200 ms on three
declared starting poses, plus one stream interruption per matcher. Learned needs
its optional runtime and local model files.

For a base installation:

~~~powershell
.\run.cmd --delay-study --modes aruco natural
~~~

Every outcome is retained. A successful trial must remain below 1 px on 30 fresh
current-image checks after stopping, with zero commands throughout. Traces record
capture and delivery timestamps, observation age, current and captured joints,
delayed image error and commanded velocities. Raw traces remain local under the
[results policy](results/README.md).

## Measured results

Across the three starting poses and four delays, ArUco and SIFT each converged in
12/12 trials. Learned converged in 9/12: all three 200 ms trials timed out.
Its median simulated completion time increased from 5.87 s at zero delay to
10.70 s at 50 ms and 15.63 s at 100 ms. No delay compensation or gain retuning
was applied.

All three stream-interruption checks stopped exactly 250 ms after the final
capture, with commands remaining zero. The full local regression suite passed
187 tests and the scene/application check. The final clock-reset refinement
was then checked with all 25 camera timing and integration tests.

[Measured delay-study results](results/camera-delay/20260919-validation/REPORT.md)
and [verification evidence](results/camera-delay/20260919-validation/verification.json).

The trial set checks local convergence and feedback-loss handling. It does not
establish stability across arbitrary targets, calibration errors, moving scenes
or all possible latency values.
