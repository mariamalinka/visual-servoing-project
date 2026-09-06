# Visual Servoing Simulation

A six-joint robot uses a simulated wrist camera to align an ArUco marker with a reference image. Runs locally on Windows using MuJoCo, OpenCV and Python. No robot, webcam, Arduino or Raspberry Pi is required.

**Completed:** step 1 (scene and manual motion), step 2 (marker-based IBVS), step 3 (200 randomized evaluation trials).

[Read the randomized benchmark results](BENCHMARK.md). The benchmark tests Align from new starting poses, beyond the fixed Offset example.

![Automatic alignment](results/alignment/alignment.gif)

## Try automatic alignment

1. Double-click **run.cmd** in this folder. Close any older simulator window first so you load the new version.
2. Click **Offset [O]** to load a starting pose away from the goal.
3. Click **Align [G]**. The controller moves the arm using the camera images.
4. Watch the green detected corners approach the amber reference outline.
5. The status changes to **CONVERGED** once the error remains below 1 pixel for 0.5 simulated seconds. Joint velocity commands then remain zero.

You can also jog a joint manually, then click Align. Reset instantly restores home; Align instead returns using the visual feedback loop. **Demo** remains a separate scripted joint-motion demonstration.

The software is already installed on this computer. The launcher finds the project-local environment at `../../work/vservo-venv`; renaming the project folder does not break that relative path.

## Fixed-offset example (step 2)

One reproducible trial starts with joint offsets of **[3, -3, 4, 3, -2, 2] degrees** from home. This is the same pose loaded by the Offset button.

| Measurement | Result |
|---|---:|
| Initial RMS corner error | 45.14 px |
| Error when convergence was declared | 0.986 px |
| Time until convergence, including the hold window | 3.63 simulated seconds |
| Observed window below 1 px | 0.50 seconds |
| Error after a further second with zero command | 0.986 px |
| Control mathematics unit tests | 10 passed |
| App alignment, manual controls and tracking-loss stop | Passed |

![Error decay and feature paths](results/alignment/convergence.png)

Error is the root mean square of the four corners' Euclidean distances from their corresponding reference points, in pixels. It measures the complete marker configuration, including scale and rotation, rather than just its center. The reference goal is the observed home image, not a mathematically perfect square centered at exactly (320, 240).

This is the original single-trial example. The separate [200-trial benchmark](BENCHMARK.md) measures successes and failures across randomized joint offsets; neither experiment implements marker-free perception.

## Reproduce and inspect

From PowerShell in this folder:

```powershell
.\run.cmd --verify
.\run.cmd --trial
.\run.cmd --benchmark
.\run.cmd --report
```

`--verify` runs all 20 maths, benchmark and integration tests, original scene checks and the app's Offset/Align click handlers. It also verifies a stable stopped result and automatic stopping when the marker is no longer detected.

`--trial` regenerates the following files from a fresh physics simulation:

| Output | Contents |
|---|---|
| results/alignment/summary.json | Outcome, measured errors, timing, parameters and versions |
| results/alignment/trace.csv | Per-frame error, corners, depths, camera twist and joint commands |
| results/alignment/convergence.png | Log-scale error curve and image-plane feature paths |
| results/alignment/convergence.svg | Vector version of the same plots |
| results/alignment/alignment.gif | Actual rendered world and wrist-camera frames |
| results/alignment/reference.png | Unannotated desired camera image |
| results/alignment/initial.png | Unannotated offset camera image |
| results/alignment/final.png | Unannotated stopped camera image |
| verification/results.json | Scene and app-level acceptance measurements |

`--benchmark` runs 200 seeded trials, saves every sampled starting pose and its outcome in a new timestamped directory under `results/benchmark/`, then produces a report and plots. Previous runs are preserved. `--report` regenerates the latest run's report and plots from saved logs without running physics.

The verification, trial and benchmark commands run without the desktop control window but still render camera images through OpenGL. The GIF samples the trajectory at 10 frames/second and holds the stopped final image briefly for viewing.

## Controls

| Control | Result |
|---|---|
| O / Offset | Load the declared offset starting pose |
| G / Align | Start IBVS from the current pose |
| Joint - / + buttons | Send a 0.25 s joint velocity pulse |
| 1 through 6, then A / D | Select and jog a joint |
| R / Reset | Instantly restore home and reset simulation time |
| M / Demo | Toggle scripted motion, starting at home |
| Space / Pause | Pause or resume; cancels active motion commands |
| Stop | Cancel alignment or demo and command zero velocity |
| S | Save a raw camera image and metadata under captures/ |
| Esc / close window | Exit |

If the target disappears, the controller commands zero velocity and reports tracking loss. Reset or jog to recover visibility, then start Align again. Invalid depth estimates and the 20 s timeout also stop alignment. The benchmark additionally records whether loss happened at the start or during motion, whether the marker geometry remained in view, and whether a trial stalled, timed out or became unstable after stopping. Align does not search for an unseen marker.

## How the controller works

1. At startup, detect the four marker corners in the rendered home image. These coordinates define the desired visual configuration.
2. Every camera frame, detect the marker with subpixel corner refinement.
3. Normalize current and desired image points using the camera calibration K.
4. Estimate each corner's optical depth from its image coordinates, known marker size and OpenCV IPPE-square PnP. Reject invalid depth or excessive reprojection error.
5. Stack the point-feature interaction matrices L. Compute camera velocity as `v_c = -gain * damped_pinv(L) * (s - s_desired)`.
6. Obtain joint velocities with a damped inverse of the camera-origin robot Jacobian. Bound speeds, apply velocity actuators, then render the next image.

The feedback error is in the image: this is proportional IBVS. PnP supplies only depths for L. The control calculation does not use the simulator's target position or command the home joint angles. Simulator kinematics supply the calibrated robot Jacobian and camera axes.

For a normalized point (x, y) at optical depth Z, the two rows of L are:

```text
[-1/Z,     0, x/Z,       x*y, -(1+x*x),  y]
[    0, -1/Z, y/Z,   1+y*y,       -x*y, -x]
```

The damped inverse uses SVD gains `sigma / (sigma^2 + damping^2)`. Velocity vectors are scaled uniformly when saturated to preserve their direction.

Defaults in `config.json`: gain 1.2/s, interaction damping 0.0001, joint damping 0.005, camera limits 0.12 m/s and 0.4 rad/s, and joint limit 0.35 rad/s for alignment. The Jacobians mix linear and angular units; these damping values are for this model and SI unit convention.

## Numerical verification

The tests compare formulas with independently perturbed geometry:

- Interaction matrix versus central finite differences of camera motion, with 50 seeded random sets of eight points; tolerance 2e-8 absolute and relative.
- Camera-origin Jacobian versus perturbed robot joints at three poses, including the wrist-to-camera offset and optical-axis conversion.
- Complete `L * J_camera` chain versus direct projection after joint perturbations.
- Depth recovery from a known rotated marker projection.
- Bounded damped inverse at a manufactured singularity.
- Invalid-input rejection, tracking-loss stop, timeout, and an observed 0.5 s hold window before success.

App acceptance checks call the same click handlers used by Offset and Align, advance actual MuJoCo physics, and inspect images detected by OpenCV. The rendered app layout has been inspected; native desktop mouse interaction has not been verified remotely.

## Files and conventions

| File | Purpose |
|---|---|
| scene.xml | Original teaching arm, six joints, camera and marker target |
| config.json | Camera, manual jog and IBVS parameters |
| simulation.py | Physics, rendering, image detection and camera Jacobian |
| control.py | Interaction matrix, damped inverse, depth estimation and IBVS |
| app.py | Desktop views and controls |
| run_alignment.py | Reproducible trial, CSV export, plots and GIF |
| tests/test_control.py | Independent maths and controller behavior tests |
| benchmark.py | Seeded trial generation, per-frame logging and exact-pose replay |
| benchmark_config.json | Sampling bounds, number of trials and evaluation thresholds |
| analyze_benchmark.py | Log validation, rates, confidence intervals, CSV and plots |
| tests/test_benchmark.py | Sampling, statistics, failure classification and trial integration tests |
| BENCHMARK.md | Results and links to the completed 200-trial dataset |
| verify.py | Scene and app integration checks |
| requirements.txt | Pinned dependencies, including Matplotlib for plots |

Units are meters, radians and seconds. Camera twist ordering is `[vx, vy, vz, wx, wy, wz]`. Optical axes are +X right, +Y down, +Z forward. The Jacobian is evaluated at the camera origin and rotated into optical axes. Its lever arm is already included, so no second translation transform is applied.

Physics runs at 500 Hz in simulated time, with 30 Hz camera samples. If rendering is slow, wall-clock playback slows; 30 real-time FPS is not guaranteed. Normal motion is integrated through velocity actuators. Position assignments are used only to initialize/reset the scene and in isolated finite-difference tests.

The 640 x 480 camera sees a 0.32 m board containing a 0.24 m ArUco marker (DICT_4X4_50, ID 7). The scene texture compensates for mirrored box-face UVs; the sensor image is never mirrored to force detection.

## Setup on another computer

Install 64-bit Python 3.12, then run **setup.cmd** followed by **run.cmd**. Initial setup downloads dependencies into `.venv`. Subsequent runs work offline. An OpenGL-capable graphics driver is needed. Keep the entire project folder.

Equivalent commands:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe app.py
```

MuJoCo replaces the original Gazebo/ROS 2 stack for the agreed Windows version. ROS topics and ros2_control are not implemented.

## Scope and next step

This is a teaching-arm model with ideal gravity compensation, simple geometry, disabled robot collisions and fixed lighting. Noise, distortion, artificial latency, learned features and controller baselines are still future work. No real-world performance claim is made.

**Step 3 is complete:** randomized evaluation, structured traces, failure diagnostics, confidence intervals and report generation.

**Next:** investigate the recorded detection failures and compare controller/perception improvements on the same saved starting poses. This provides measured evidence for improvements before adding learned features or more realistic sensor effects.

## Sources

- [Chaumette & Hutchinson, Visual Servo Control, Part I (2006)](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf), equations 4 and 9-11.
- [MuJoCo Jacobian API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac).
- [OpenCV PnP and IPPE-square corner ordering](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).
