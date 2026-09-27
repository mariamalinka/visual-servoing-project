# Physical accuracy and calibration sensitivity

This experiment measures physical pose accuracy **in simulation**. It reports
millimetres and degrees alongside image error, and tests how errors in the
controller's calibration affect convergence and final positioning.

## Run and inspect

From the repository root:

~~~powershell
.\run.cmd --accuracy-study
.\run.cmd --accuracy-study --starts 2
.\run.cmd --accuracy-study --starts 2 --modes aruco natural
~~~

The default is **108 trials**: three paired starts × 12 calibration profiles ×
three matchers. The selected [72-trial validation](results/accuracy/20260921-validation/REPORT.md)
uses two starts per profile/matcher. See the [measured findings](results/accuracy/20260921-validation/FINDINGS.md)
for the acceptance counts and interpretation. Learned requires the optional installed
models/runtime. Every outcome is retained, including timeouts, rejected starts,
camera failures and physical-accuracy misses.

Try a calibration assumption interactively:

~~~powershell
.\run.cmd --calibration-profile combined --camera-delay-ms 100
.\run.cmd --learned --calibration-profile focal-plus-5 --camera-delay-ms 100
~~~

**Calibration [I]** cycles the configured profiles. Changing calibration stops
motion and clears queued camera frames while preserving the robot pose, image
goal, selected gain and a previously observed recovery viewpoint. Press **G** to
run again. The status row names the active assumption; saved captures include
the assumed intrinsics and calibration settings.

## What is measured

The experiment takes a private reference image at a known simulated teaching
pose. Each trial starts from a declared joint offset, runs the existing app
controller, and measures the actual camera and tool poses when it stops.

| Quantity | Definition |
|---|---|
| Camera position error | Distance between actual and taught optical origins, mm |
| Tool-frame position error | Distance between actual and taught tool_roll body origins, mm |
| Orientation error | Shortest rotation angle between actual and taught frame axes, degrees |
| Signed position error | Three translation components in the corresponding taught frame's axes |
| Pixel error | RMS displacement of the four detected target corners |
| Time to stop | Simulation time when the controller declares its terminal outcome |
| Physical acceptance | Image convergence plus both frames within the declared physical tolerances |

The tool frame is explicitly the **tool_roll body origin**. No gripper TCP is
modelled. Camera axes are right/down/forward; tool axes come from its body frame.
The camera and tool have different origins, so their position errors can differ.
Their orientation-error magnitudes agree because they are rigidly attached.

For actual pose (R, p) and taught pose (R*, p*), signed position error is
1000 R*ᵀ(p − p*) and position error is its Euclidean norm. Orientation uses the
SO(3) geodesic angle of R*ᵀR; it does not subtract Euler angles.
[accuracy.py](accuracy.py) validates rigid transforms and implements these scores.

The configured acceptance thresholds are **2 mm and 1° for both frames**.
They are an illustrative engineering rubric, not a certified robot specification.
The original validation used a **1 px for 0.5 s** image criterion. The current
app defaults to [precision stopping](PRECISION_STOPPING.md), which adds
reference-image refinement and sensitivity-aware checks. Use `--legacy-stop`
with the app or accuracy runner to compare the original stopping policy.
After stopping, the evaluator checks **30 fresh current captures**, zero commands
and physical tolerance at every sampled pose, including the stop pose. A trial
can pass image convergence and fail physical acceptance.

## Calibration errors under test

Only the controller's assumptions change. The camera still renders with its
original pinhole model; target geometry, nominal robot kinematics and collision
supervision remain fixed.

| Profiles | Perturbation |
|---|---|
| nominal | No calibration error |
| focal-minus/plus-5 | Both assumed focal lengths scaled by −5% / +5% |
| principal-minus/plus-5 | Assumed horizontal principal point shifted −5 / +5 px |
| mount-x-minus/plus-10 | Assumed camera origin shifted −10 / +10 mm along optical X |
| mount-yaw-minus/plus-5 | Assumed camera axes rotated −5 / +5° about optical Y |
| size-minus/plus-10 | Assumed target side length scaled by −10% / +10% |
| combined | The positive perturbation from every group above |

[accuracy_config.json](accuracy_config.json) declares the profiles, sampling seed,
offset bounds, 100 ms camera delay, 60 s run deadline and acceptance thresholds.
The implementation also accepts separate fx/fy scales, two principal-point
offsets, and three-component mount translation/rotation vectors. Unknown keys,
nonfinite values and invalid scales are rejected.

[calibration.py](calibration.py) applies assumed intrinsics to both current and
desired normalized features, and assumed target size to the controller's depth
estimator. The unchanged image goal is still the control objective.

For camera-mount error ΔT = [Rδ, tδ], expressed in nominal optical axes, the
assumed point velocity is Rδᵀ(v + ω × tδ); angular velocity is Rδᵀω.
The calibrated Jacobian shifts the evaluation point once and then rotates both
blocks. Its sign and lever arm are checked against finite differences of an
independently constructed virtual camera pose in MuJoCo.

## Reproducibility and preservation

The same seeded starting poses and perception seeds are reused across every
profile and matcher. Poses are not resampled to remove difficult starts.
Pairwise differences compare a perturbed trial to nominal for the same matcher
and start, only when both pass image convergence; the report shows that paired
denominator. Failed trials remain in the overall counts.

Existing saved application goals contain image/calibration metadata without the
original physical teaching pose. They cannot supply an exact ground-truth
accuracy reference. The experiment therefore creates its own reference captures
under its result directory. Existing goals are preserved. Ground-truth teaching
poses are stored separately in **goals.json** for evaluation; the controller
receives only the private reference image and detected features.

Each result directory contains the plan, source/runtime fingerprints, private
references, teaching poses, all trial outcomes, summary metrics, figures, selected
app snapshots and local per-frame traces. Completed trials are written atomically.

~~~powershell
.\run.cmd --accuracy-study --plan-only --starts 2 --output outputs/visual-servoing-simulation/results/accuracy/my-run
.\run.cmd --accuracy-study --resume outputs/visual-servoing-simulation/results/accuracy/my-run
.\run.cmd --accuracy-study --report-only outputs/visual-servoing-simulation/results/accuracy/my-run
~~~

New output directories must not already exist. Resume checks the plan, source,
runtime, assets, teaching poses and private reference hashes before continuing.
It skips completed trials, including recorded failures. Report-only rebuilds
plots/tables from JSON without running the simulator or requiring bulk traces.
Completed negative outcomes are valid study results; exceptions or safety
violations produce a failing process exit code.

## Interpreting the results

Physical-error medians and sample p95 values describe pixel-converged trials.
Terminal outcomes, physical acceptance, and pixel-only passes are counted
separately. Signed mean error and paired differences are also saved in the JSON
summary. With only two validation starts, these are observed sample statistics,
not population bounds or an ISO repeatability measurement.

Calibration error can affect transients and robustness while leaving the final
image objective unchanged. Sub-pixel image agreement can still coexist with
larger physical pose errors. The [independent projection check](results/accuracy/20260921-validation/GEOMETRY_CHECK.md)
found 14 image-converged trials with under 1 px of ideal geometric corner
displacement but tool-position error above 2 mm. In this viewing geometry,
coupled translation and rotation can leave a very similar planar-target image;
feature bias is not required to explain that the image threshold permits a
physical miss. Physical acceptance is therefore measured independently of the
detector's success indication. The separation between feature measurements and approximate
control models follows the [visual servoing formulation](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf).

This study does not cover real hardware, lens distortion, encoder errors,
uncertain collision geometry, or true camera intrinsics changing after teaching.
Those need separate experiments. Host rendering/matching duration is diagnostic;
it is not added to simulated transport delay.

## Verification and code

- [Metric/calibration/plan tests](tests/test_accuracy.py): independent rigid-pose
  examples, frame invariance, angle wrapping, lever-arm sign, paired statistics
  and report generation.
- [App integration tests](tests/test_calibration_integration.py): unchanged
  rendering/geometry, finite-difference calibrated Jacobians, actual command
  changes, profile controls, gain/target switching and tool-frame definition.
- [run_accuracy_study.py](run_accuracy_study.py): execution, physics-tick safety
  auditing, independent physical scoring, interrupted-run resume and reporting.
- [MuJoCo Jacobian conventions](https://mujoco.readthedocs.io/en/latest/programming/simulation.html#jacobians).

Run **run.cmd --verify** for the complete local suite. The CI mathematics job
includes the metric/calibration/plan tests; rendered integration remains a local
check. [Published verification](results/accuracy/20260921-validation/VERIFICATION.md).

