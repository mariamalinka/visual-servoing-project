# Visual Servoing Simulation

A six-joint robot aligns its wrist-camera view with a saved reference image.
The project combines MuJoCo physics, OpenCV perception and visual feedback control,
with optional SuperPoint + LightGlue matching on CPU or NVIDIA GPU.

![Robot aligning from visual feedback](outputs/visual-servoing-simulation/results/alignment/alignment.gif)

**[Project report](docs/PROJECT_REPORT.md)**: a one-page overview, then the full account of what was built, the problems found, how they were fixed, and what remains open.

## What it does

- Aligns an ArUco marker or one known flat photograph with a taught image.
- Compares image-based control (IBVS), pose-based control (PBVS), and a look-once baseline.
- Finds an initially unseen target and recovers after losing sight of it.
- Applies joint-limit supervision, bounded recovery and optional adaptive gain.
- Checks mapped collision clearance and routes search/recovery around a test obstacle.
- Models camera transport delay, timestamps observations and never moves on stale feedback: the robot is commanded to zero velocity when the image-age limit is reached. The desktop lab ends the alignment; the wall-clock runtime holds and resumes on fresh images by default ([watchdog decision](docs/WATCHDOG_DECISION.md)).
- Evaluates controllers on repeatable starting poses and records success and failure evidence.
- Measures camera/tool-frame positioning accuracy and sensitivity to controller calibration errors.

This is a simulation project. No physical robot or camera is required.

## Run on Windows

Install **64-bit Python 3.12**, then run these commands from the repository root:

~~~powershell
.\setup.cmd
.\run.cmd
~~~

Choose **Offset [O]** to try automatic alignment. **V** switches between the marker
and photograph. **K** switches between SIFT and the optional Learned matcher.

To install Learned matching with NVIDIA acceleration:

~~~powershell
.\setup-learned.cmd --cuda
.\run.cmd --learned
~~~

Use `--cpu` instead of `--cuda` for CPU-only installation. Models are downloaded
during setup; matching works offline afterwards. The camera status identifies
**Learned GPU** or **Learned CPU**. See the
[installation and troubleshooting guide](outputs/visual-servoing-simulation/LEARNED_MATCHING.md).

The root launchers use the application in `outputs/visual-servoing-simulation/`.
Its existing launchers also work. The application stays at this path so local
environments, saved references and existing workflows keep working.

To experiment with delayed feedback, run `run.cmd --learned --camera-delay-ms 100`.
Use **C** to change delay and **X** to interrupt camera capture.
Run `run.cmd --robustness` for the [automated delay, jitter and loss experiment](outputs/visual-servoing-simulation/CAMERA_ROBUSTNESS.md).
[Camera timing model and controls](outputs/visual-servoing-simulation/CAMERA_DELAY.md).

To try obstacle-aware search, run **run.cmd --obstacle --cold-start --camera-delay-ms 100**.
Press **U** to toggle the column. [Collision controls and validation](outputs/visual-servoing-simulation/COLLISION_AWARE.md).

Run **run.cmd --accuracy-study** to compare pixel convergence with physical pose error.
Press **I** to cycle calibration assumptions in the app.
The default [precision stopping](outputs/visual-servoing-simulation/PRECISION_STOPPING.md)
refines the image near the goal and checks sensitivity to remaining camera motion.
Use `--legacy-stop` to compare the original 1 px rule.
[Physical accuracy and calibration sensitivity](outputs/visual-servoing-simulation/PHYSICAL_ACCURACY.md).

## Explore the project

| Location | Purpose |
|---|---|
| [outputs/visual-servoing-simulation/](outputs/visual-servoing-simulation/) | Application source, configuration, scene and launchers |
| [tests/](outputs/visual-servoing-simulation/tests/) | Mathematics, perception, recovery and integration checks |
| [assets/](outputs/visual-servoing-simulation/assets/) and [reference/](outputs/visual-servoing-simulation/reference/) | Required target images and saved image goals |
| [docs/](docs/README.md) | Documentation and experiment index |
| [results/](outputs/visual-servoing-simulation/results/README.md) | Selected reports, plots and reproducibility metadata |
| [tools/](tools/) | Repository validation and independent geometry analysis |
| [.github/workflows/](.github/workflows/) | Automated repository and control-mathematics checks |

Local `work/`, `archive/`, virtual environments, model weights, caches and
unpublished experiment runs are ignored by Git. Historical ZIP exports are local
backups; they are not required to run the application.

## Results and verification

The [precision-stopping comparison](outputs/visual-servoing-simulation/results/accuracy/20260921-precision-final/COMPARISON.md)
passed 72/72 paired calibration trials and 12/12 trials with new starting poses.
Learned physical acceptance improved from 0/24 to 24/24 at the selected 2 mm /
1 degree limits. Its median tool error among image-converged trials fell from
9.92 mm to 0.168 mm. The historical baseline is preserved.
[Verification: 263 tests](outputs/visual-servoing-simulation/results/accuracy/20260921-precision-final/VERIFICATION.md).

Local verification covers controls, collisions, recovery and camera timing,
including Learned GPU alignment and CPU fallback.
[Collision and regression evidence](outputs/visual-servoing-simulation/results/collision/20260920-validation/VERIFICATION.md). On the tested RTX 3050 laptop, GPU picture matching
takes approximately **55–57 ms per image**, about **8 times faster** than the paired
CPU measurement. These are measured inference timings, not a promise of a
30 Hz application loop. [Timing and alignment evidence](outputs/visual-servoing-simulation/LEARNED_PERFORMANCE.md).

~~~powershell
.\run.cmd --verify
.\run.cmd --benchmark
.\run.cmd --compare
~~~

Benchmarks write new local results. Reports and a small representative alignment
trace are included in Git; bulk per-trial traces are generated locally.
[Experiments and reproduction](docs/README.md#experiments).

## Scope

The photograph matcher recognizes one known planar target. Collision checks use
known scene geometry and a bounded local planner. Simulation playback can be
slower than real time.
Pixel alignment does not establish physical positioning accuracy.
[Controller details and limitations](outputs/visual-servoing-simulation/README.md).

For development commands and the policy for generated files, see
[CONTRIBUTING.md](CONTRIBUTING.md).

## Actual camera processing latency

Run **run.cmd --realtime --learned --max-camera-age-ms 400** for wall-clock control with rendering and matching in a separate process. Press **O**, then **G** to align. The original 250 ms app default remains available; the explicit 400 ms budget includes processing and the interval between results. Run **run.cmd --latency-study** to measure alignment, transport and blocked-camera behavior.

[Runtime, controls and timing limits](outputs/visual-servoing-simulation/REALTIME_CONTROL.md).

[Perception tail latency and the 50 ms transport comparison](outputs/visual-servoing-simulation/PERCEPTION_LATENCY.md).

[Sustained latency testing and spike diagnostics](outputs/visual-servoing-simulation/LATENCY_STRESS.md).

The simulated arm has actuator dynamics: a stop zeroes the command at once, and the arm then brakes within assumed acceleration and jerk limits. Run **run.cmd --stop-response** to measure stopping time, stopping distance and pause/resume behaviour; `--actuator ideal` switches the model off. [Actuator model, assumptions and results](docs/ACTUATOR_MODEL.md) (simulation only, not real-robot data).

## Acceptance test

Run **acceptance-test.cmd** (about 30 minutes, on AC power) for the automated SIFT and Learned GPU acceptance test: five starting poses, repeated alignments and a 12-minute reused-worker run per method, ending in a PASS/FAIL report. [Plan, criteria and recorded evidence](docs/ACCEPTANCE_TEST.md).

[Requirements Verification and Safety Traceability](docs/SAFETY_TRACEABILITY.md): each safety and acceptance requirement with its hazard, enforcement, tests, evidence and verification status, including the requirements that are not yet verified.

[Fault injection and the safety envelope](docs/FAULT_INJECTION.md): every acceptance, margin and stress run now records clearance, joint-limit margin and joint speed against their requirements, and **run.cmd --fault-campaign** triggers each safety stop on purpose and reports the reaction.
