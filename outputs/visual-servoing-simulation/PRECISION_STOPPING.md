# Precision stopping and pose sensitivity

The app now refines alignment against the saved camera image near the goal and
checks the estimated remaining camera motion before declaring success. Run the
normal launcher to use it. For a controlled comparison, `--legacy-stop` selects
the original detector-only, 1 px stopping rule.

~~~powershell
.\run.cmd --learned --camera-delay-ms 100
.\run.cmd --learned --legacy-stop --camera-delay-ms 100
.\run.cmd --accuracy-study --starts 2
.\run.cmd --accuracy-study --starts 2 --legacy-stop
~~~

## Image feedback and stopping

The selected detector still acquires the target and checks its identity. ArUco
retains its existing subpixel corner detector. For SIFT and Learned, when the
four-corner RMS error is at most 8 px, a local homography
registers the current full-resolution target patch to the saved reference image.
It refines the detector's coordinates using image intensities. It does not load
the taught robot pose or read scene geometry.

The [ECC registration](https://docs.opencv.org/4.x/dc/d6b/group__video__track.html)
uses a cropped target region, up to 8 half-resolution and 12 full-resolution
iterations, and the detected target's interior mask. Correlation must reach 0.97; the refined outline must remain valid
and within 8 px of the acquired corners. Invalid registration supplies no control
features and follows the existing bounded recovery behavior. Each frame starts
from its own detector measurement; previous accepted corners are not substituted.

A stop requires all of these checks continuously for the existing 0.5 s hold:

- RMS image error below **0.25 px**.
- Every corner's displacement below **0.5 px**.
- Estimated camera translation correction below **0.8 mm**.
- Estimated camera rotation correction below **0.1 degree**.
- Sufficient local image sensitivity, defined below.

Commands are zero during the hold. A failed check resets it. With camera delay,
the hold must span distinct captured frames; display redraws cannot advance it.
The accuracy runner repeats the precision checks on every post-stop capture.
Changing calibration, matcher or gain retains the selected stopping policy.
Teaching replaces the image reference and clears pending old frames.

The final-alignment status shows estimated correction in mm and degrees. These
are **image-derived camera estimates**, not measured physical accuracy and not a
robot TCP specification. Independent simulation camera/tool errors remain the
acceptance measure in the accuracy study.

## Sensitivity to pose changes

Four corner coordinates provide eight image measurements. The interaction
matrix predicts how a small six-axis camera motion changes them. The new stop
check uses an **undamped** local inverse: damping used to keep commanded motion
stable must not hide a weakly measured error direction when deciding to stop.

Translation columns are scaled by 0.8 mm and rotation columns by 0.1 degree.
SVD is applied to that pixel-space matrix, so metres and radians are not mixed
without explicit scales. A minimum scaled singular value below 0.01 px makes
near-goal stopping unreliable; the app brakes with `poor_sensitivity` instead of
reporting alignment. The remaining correction comes from the same local image
model, using the controller's assumed calibration and image-estimated depths.

The [independent pose probes](results/accuracy/20260921-precision-final/POSE_SENSITIVITY.md)
use exact projection and a numerical Jacobian only for evaluation. In that taught
view, four of 45 probes lie outside 2 mm / 1 degree while satisfying the old
1 px gate. None satisfies the new gate. Coupled translation and rotation explain
these weak directions. This result covers instantaneous checks in one scene,
not convergence from arbitrary starts.

Depth estimation also handles a frontal-square initialization failure: if IPPE
and its refinement cannot produce an acceptable estimate, an independent planar
initializer is tried. Positive depth and the existing reprojection-residual
limit still apply.

## Measured result

All 72 paired calibration trials passed physical acceptance, plus 12 trials with
new starting poses. Among image-converged cases, median tool error changed from
0.505 to 0.406 mm for ArUco, 0.512 to 0.053 mm for SIFT, and 9.917 to 0.168 mm for
Learned. The original Learned run had 16 image passes and no physical passes;
the updated run had 24 of each. These denominators differ, so the
[complete paired plot and outcomes](results/accuracy/20260921-precision-final/COMPARISON.md)
are retained. [263-test verification](results/accuracy/20260921-precision-final/VERIFICATION.md).

## Reproduction and evidence

- [Original 72-trial baseline](results/accuracy/20260921-validation/FINDINGS.md).
- [Precision calibration run](results/accuracy/20260921-precision-final/REPORT.md).
- [Pose sensitivity calculations](results/accuracy/20260921-precision-final/pose-sensitivity.json).
- [Settings](precision_config.json), [refinement and sensitivity](precision.py),
  [controller](control.py), and [experiment runner](run_accuracy_study.py).
- [Independent image/math tests](tests/test_precision.py) and
  [app integration checks](tests/test_precision_integration.py).

The new and historical runs retain their own plans and fingerprints. Results
include every planned outcome. The historical baseline is not overwritten;
`--report-only` can rebuild old JSON reports, while `--resume` rejects changed
source/settings. The current `--legacy-stop` path uses today's validated depth
estimator, so it is explicitly distinct from the historical source revision.

## Limits

This improves final image feedback and stopping in the tested simulation. It does
not add depth sensing, multiple targets, hardware calibration or lens distortion
compensation. Estimated correction can still be biased by calibration or feature
errors. The singular-value threshold is a configured engineering gate, not a
probabilistic confidence bound. Rejecting uncertain registration can stop or
extend a run; the unchanged time and safety budgets still apply.

Refinement adds processing near the goal. In an [eight-frame paired timing check](results/accuracy/20260921-precision-final/TIMING.md),
Learned GPU increased from 64.8 ms to 102.3 ms and SIFT from 44.9 ms to 78.0 ms.
It uses a bounded crop and iteration count, but no real-time wall-clock guarantee is made. Simulated camera transport
delay still excludes host inference/rendering time.

The original precision accuracy campaign used the earlier 45-iteration refinement.
[Current tail-latency optimization and validation](PERCEPTION_LATENCY.md) document
the bounded pyramid replacement; image and pose stopping thresholds are unchanged.
