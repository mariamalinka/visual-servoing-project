# Comparing three ways to align the robot

Completed 200 identical randomized starting poses per method: **600 static trials**, plus **3 separate target-movement demonstrations**. The RNG seed is 20260906.

[Full generated report](results/comparison/20260907-005242-736914-n200-seed20260906/REPORT.md) | [Code explained](CODE_GUIDE.md) | [All trial rows](results/comparison/20260907-005242-736914-n200-seed20260906/trials.csv)

| Method | Successful / all starts | Successful / initially detected | Median completion time* | Median final image error* |
|---|---:|---:|---:|---:|
| Image-based feedback (IBVS) | 86/200 | 86/86 | 4.05 s | 0.974 px |
| Pose-based feedback (PBVS) | 86/200 | 86/86 | 4.27 s | 0.943 px |
| Look once + joint feedback | 83/200 | 83/86 | 5.47 s | 0.530 px |

*Medians are for successful trials, in simulated time. The shared success threshold is 1 px. Completion includes a 0.5 s hold; every success also passed a further 1 s stopped-image check.

On the 83 starts where all three succeeded, median completion times were **4.07 s for IBVS, 4.27 s for PBVS and 5.47 s for look-once**.

112 starts were outside the camera view and 2 more had no marker detection. They remain in the denominator for every method. Search was disabled so this experiment measures alignment. The interactive app still uses the earlier remembered-view recovery.

![Controller comparison](results/comparison/20260907-005242-736914-n200-seed20260906/comparison.png)

The error plot includes failed runs when their final error can be measured. Dots are individual trials; black horizontal lines are medians.

## What the results mean

- Both feedback methods succeeded on all 86 detected starts in this experiment. Their detected-start 95% Wilson success intervals are 95.7–100%; this is not a guarantee on unseen starting poses.
- Look-once succeeded on 83/86 detected starts (96.5%; 95% Wilson interval 90.2–98.8%). In 3 cases it reached its predicted camera pose but missed the image threshold.
- Look-once can be accurate in this ideal, static simulation. Its lower median error also reflects its tighter internal pose stopping tolerance: it continues moving after a feedback method is allowed to stop below 1 px. The timing/error table compares these configured implementations.
- These results support keeping IBVS as our current default. They do not establish that IBVS is universally faster or more accurate than PBVS.

## When the target moves

The target shifts 20 mm sideways after 1 s. Each controller starts from the same declared Offset pose, with the same reference image.

| Method | Final error | Result |
|---|---:|---|
| Image-based feedback (IBVS) | 0.924 px | converged |
| Pose-based feedback (PBVS) | 0.939 px | converged |
| Look once + joint feedback | 20.854 px | open_loop_residual |

![Response to moving target](results/comparison/20260907-005242-736914-n200-seed20260906/target_step.png)

IBVS keeps measuring image error. PBVS keeps estimating the target's 3D pose. Both update their motion after the target shifts. Look-once follows the destination calculated before the shift; joint feedback gets it there, but later images cannot correct that destination.

This is one controlled example per method, kept separate from the randomized success rates. Noise, latency, uncertain calibration and more target motions still need separate robustness experiments.

## Important code

- **simulation.py** renders the camera, detects the marker and moves the simulated motors.
- **control.py / IBVSController.update()** turns current-versus-reference image error into camera velocity and then joint velocities.
- **baselines.py / PoseController.update()** implements PBVS and look-once. The difference is whether the visual estimate is updated every frame or frozen at the start.
- **compare_controllers.py / run_comparison_trial()** runs the observe/move loop, records every frame and checks success after stopping.
- **analyze_comparison.py / analyze()** validates paired trials and regenerates statistics, tables and plots.
- **tests/test_baselines.py** checks geometry, open-loop independence from later images, limits and actual simulated behavior.

Start with [the illustrated code walkthrough](CODE_GUIDE.md) for explanations of the important functions and equations.

## Run it

From this simulation folder:

~~~powershell
.\run.cmd --compare
.\run.cmd --compare-report
.\run.cmd --verify
~~~

The first command creates a fresh 200-pose experiment. The second rebuilds the latest report from saved logs. The third runs tests and scene/application checks. Existing result directories are preserved.

**Verification:** all 41 unit/integration tests passed (8 added for this step), the scene/application acceptance check passed, and the completed comparison's 603 records passed report validation. The new IBVS runner exactly matched the original runner's trajectory in the regression test.
