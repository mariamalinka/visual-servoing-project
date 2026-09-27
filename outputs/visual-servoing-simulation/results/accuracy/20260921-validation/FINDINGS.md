# Measured findings

The 72-trial simulated sweep exposed a gap between image convergence and physical
positioning in the Learned matcher/controller combination. ArUco and SIFT met the
selected physical tolerances in every tested case; Learned did not.

| Matcher | Image convergence | Physical acceptance | Final tool-position error among image passes |
|---|---:|---:|---:|
| ArUco | 24/24 | 24/24 | 0.321–0.717 mm |
| SIFT | 24/24 | 24/24 | 0.260–0.792 mm |
| Learned | 16/24 | 0/24 | 5.505–12.723 mm |

Physical acceptance requires image convergence and **both camera and tool-frame
errors within 2 mm and 1°**, checked at stopping and across 30 fresh post-stop
captures. The tool frame is the `tool_roll` body origin. All 72 trials remained
within the monitored collision/joint/feedback constraints; no forbidden contacts
were recorded.

## Calibration sensitivity

The same two seeded starts were paired across 12 profiles and three matchers,
with 100 ms camera transport delay. Profiles cover nominal calibration, focal
length ±5%, principal point ±5 px, mount translation ±10 mm, mount rotation ±5°,
target-size assumption ±10%, and combined positive errors. Only controller
assumptions changed; rendering and physical geometry remained fixed.

For ArUco/SIFT, the largest absolute per-profile median paired tool-error change
was about **0.190 mm** (SIFT, target-size assumption −10%). Every tested profile
still passed. These cases suggest tolerance to the tested model errors for this
scene and these starts.

Learned already missed physical tolerance at nominal calibration: one start
timed out and the other converged with **9.896 mm** tool-position error. Across
all profiles it had seven timeouts, one invalid-depth stop and 16 image passes.
The −10% target-size profile produced no image passes. Some perturbations changed
which start converged, so comparing only successful medians would hide failures.
The [full report](REPORT.md) retains their denominators and paired-success counts.
With only one nominal Learned image pass, its paired deltas are particularly
limited and do not support a ranking of calibration corrections.

## Why a small pixel error can still miss physically

For `learned-combined-0001`, detected post-stop error was **0.722 px**, while the
true projected target corners changed by only **0.778 px**. Nevertheless, the
final tool error was **11.291 mm** and **1.132°**. Across the study, 14 image passes
combined true geometric corner change below 1 px with tool error above 2 mm.

This independent calculation shows that the current image threshold permits
physical misses in this viewing geometry. Translation and rotation together can
leave almost the same image of a planar target. It does not prove that calibration
error alone caused the difference, or rule out additional feature-localization
error. [Projection check and every trial](GEOMETRY_CHECK.md).

![Calibration comparison](sensitivity.png)

## Engineering implication and limits

The next control improvement should target the image stopping criterion and
sensitivity to pose changes, using this experiment as an unchanged baseline.
Candidates include more precise final feature refinement and additional spatial
or depth information to distinguish nearly identical planar views. Tightening
one pixel threshold alone needs testing for noise sensitivity and timeouts.
Ground truth should remain an evaluation input, not a hidden controller shortcut.

These are observations from one simulated scene, one teaching pose per target
and two starts, without lens distortion or encoder noise. They are not hardware
accuracy claims, statistical reliability bounds or an ISO repeatability test.
Physical acceptance failures remain in the saved data; no controller tuning was
performed to make this validation pass.

[Verification evidence](VERIFICATION.md) · [Method and usage](../../../PHYSICAL_ACCURACY.md)

Reproduce from the original repository root:

~~~powershell
.\run.cmd --accuracy-study --starts 2
~~~

The default command without `--starts 2` runs three starts (108 trials).
