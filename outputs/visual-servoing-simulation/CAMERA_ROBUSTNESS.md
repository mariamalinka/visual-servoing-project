# Automated camera robustness experiment

Run the full experiment from the repository root:

~~~powershell
.\run.cmd --robustness
~~~

The default plan contains **240 trials**: 10 seeded starting poses, eight camera
profiles, and ArUco, SIFT and Learned. This can take a while, particularly when
Learned approaches the controller timeout. Results are saved after every trial.
The experiment does not change the controller gains or saved target images.

For a smaller run or a base installation:

~~~powershell
.\run.cmd --robustness --starts 2
.\run.cmd --robustness --modes aruco natural --starts 5
~~~

Learned requires its optional runtime and model files. A failed matcher is
reported as an error; it is never replaced with another matcher.

## What is varied

| Profile | Transport conditions |
|---|---|
| baseline | Zero delay |
| delay-100 | 100 ms fixed delay |
| delay-200 | 200 ms fixed delay |
| jitter | 100 ms delay with uniform ±80 ms jitter |
| drops | 100 ms delay with 20% independent packet loss |
| combined | 150 ms delay, uniform ±80 ms jitter, 20% packet loss |
| brief-outage | 100 ms delay; capture off from 0.75 to 0.83 simulated seconds |
| long-outage | 100 ms delay; capture off from 0.75 to 1.35 simulated seconds |

Edit [robustness_config.json](robustness_config.json), or use `--config` with
another JSON file. Select a subset with, for example,
`--profiles baseline jitter combined`.

Jitter changes each frame's delivery timestamp. Negative sampled delays are
clipped to zero. Packet loss happens after capture and inference, while an outage
prevents capture. Delayed frames may overtake each other; an older frame cannot
replace a newer observation already consumed by the controller.

These disturbances use simulated time. Rendering and inference still execute
synchronously on the application thread. Recorded inference durations are
diagnostics, not a measurement of a real camera pipeline's throughput.

## Repeat, interrupt and resume

Starting offsets are uniform within the configured six joint-offset limits.
No pose is rejected because the target is invisible or difficult to align.
Every matcher/profile receives the same poses. Each case also shares a seeded
stream of packet-loss and jitter draws; perception has a separate seed.
GPU results can still vary with the runtime and device.

Inspect a plan before running it:

~~~powershell
.\run.cmd --robustness --starts 10 --seed 20260920 --plan-only --output outputs/visual-servoing-simulation/results/robustness/my-run
.\run.cmd --robustness --resume outputs/visual-servoing-simulation/results/robustness/my-run
~~~

Explicit paths are relative to the directory where you launch the command.
Without `--output`, runs go under `outputs/visual-servoing-simulation/results/robustness/`.

**Ctrl+C** preserves completed trials. Resume runs only pending trial IDs;
a trial interrupted before saving runs again. Saved failures are retained.
Resume rejects a changed plan, source, assets, reference images or runtime.
Use a new output directory when comparing another implementation.

The launcher returns without a keypress: exit 0 means the planned trials
completed, even if some did not align; exit 2 reports trial exceptions or safety
violations. Read the report's outcome counts to assess robustness.

## Measurements and acceptance

The runner uses the application's actual camera/control path. It checks commands
and joint limits at every 2 ms physics tick and saves timestamped traces locally.

- Alignment requires controller convergence and 30 fresh current-image checks
  below the configured error threshold, with zero commands after stopping.
- The long outage requires a stale-camera stop at the 250 ms capture-age limit,
  capture restoration, and no automatic motion restart.
- A stale-camera stop in a profile intended to align is an alignment failure,
  even when the watchdog behaves correctly.
- Results include success counts, all terminal outcomes, median/p95 successful
  settling time, final image error, joint travel, peak commands, observation age,
  frame-loss counters and any safety violations.

Pixel checks use the selected detector; they do not establish physical pose
accuracy. Small trial sets do not establish a stability bound.

## Reports and repository contents

Each run writes `plan.json`, `manifest.json`, `trials.json`, `summary.json`,
`REPORT.md`, and PNG/SVG figures. Raw traces and full exception logs live under
`traces/` and follow the [results policy](results/README.md).

Regenerate a report from saved JSON, including a published run without raw traces:

~~~powershell
.\run.cmd --robustness --report-only outputs/visual-servoing-simulation/results/robustness/my-run
~~~

The source fingerprint and package versions are recorded before the first trial
and checked again at completion. The fixed-delay baseline remains available via
[the original timing study](CAMERA_DELAY.md).


## Measured validation

The [48-trial validation](results/robustness/20260920-validation/VALIDATION.md)
completed with 31 alignments, 15 stale-camera stops and two timeouts. All six
specified long-outage checks passed. No safety violations were recorded, and
the complete regression suite passed 200 tests. These results use two starting
poses per profile/matcher; the default experiment uses ten.

## Collision checks in new runs

Following the collision-aware motion update, new runs also audit forbidden
contacts and minimum signed clearance minus the applicable collision margin
at every physics tick, including the stopped interval. These metrics are saved
in each trial. A margin violation or contact fails the safety checks.
The published 48-trial run above predates this change; its outcomes remain
historical evidence. Its source fingerprint prevents resuming it with a changed
controller. Create a new experiment to evaluate this revision.
[Collision policy and obstacle experiment](COLLISION_AWARE.md).
