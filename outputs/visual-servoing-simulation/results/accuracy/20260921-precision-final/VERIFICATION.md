# Precision stopping verification

The final implementation passed **263 tests in 448.828 s**, followed by app/scene
verification. [Full log](full-tests.txt). Eighteen new math/image tests and five
rendered integration tests cover the new behavior.

## Physical validation

- **72/72** paired calibration trials passed the declared 2 mm / 1 degree limits
  for both camera and tool frames. All three matchers passed 24/24.
- **12/12** additional trials passed with seed 20260922, two new starts, all three
  matchers and nominal/combined calibration. [New-start report](../20260921-precision-fresh-starts/REPORT.md).
- All **2,520** fresh post-stop captures passed the configured precision checks.
  Commands stayed zero. There were zero forbidden contacts and zero recorded
  collision, joint-limit, stale-feedback or post-stop-motion violations.
- The 72-trial comparison checks identical plans, target images, teaching poses and
  scene geometry against the preserved historical run. All failures from the
  original run remain visible. [Paired comparison](COMPARISON.md).
- In 45 independent pose/projection probes, the old 1 px gate admitted four poses
  outside physical tolerance; the new gate admitted none. These are instantaneous
  eligibility checks, not extra closed-loop trials. [Sensitivity report](POSE_SENSITIVITY.md).

## Behavior checked

Tests cover a real coupled translation/rotation with sub-pixel image displacement,
known pose units, singular geometry, complete hold/reset behavior, valid depth,
independent synthetic projective warps, missing/blank/unrelated images, failed
registration, excessive warp corrections, reference-cache replacement, ArUco
subpixel preservation, gain/calibration/matcher changes, safe teaching and
capture-time hold accounting. The full suite also covers camera outages,
collision supervision, search/recovery, Learned inference and existing controls.

A current legacy-mode CLI run passed; completed-run resume preserved its result
bytes and report-only rebuilt the output. [Smoke evidence](legacy-smoke.txt).

## Provenance and reporting

Runtime, control/perception sources, assets, plans, teaching records and private
reference hashes were checked against the execution manifests. Original saved
application references and historical experiment files are preserved.

After execution, garbled unit symbols in the report formatter were corrected to
UTF-8. An AST check confirmed that only non-ASCII display strings inside the
report function changed; control, scoring and experiment logic did not change.
The 16 accuracy/report tests passed again. [Correction record](report-format-correction.json),
[execution-time report source](report-source-at-execution.txt),
[report test output](report-format-tests.txt). Manifests retain their original
execution fingerprints. Use report-only for completed historical runs; strict
resume rejects any source change, including formatting.

The comparison plot, sensitivity plot and final application snapshot were
visually checked. These measurements cover one simulated scene and a small set
of starts; they do not establish hardware accuracy or statistical reliability.

Repository checks passed for the prospective Git file set: Python syntax,
relative documentation links and generated-file exclusions. `git diff --check`
also passed. [Processing-cost check](TIMING.md).

[Machine-readable verification](verification.json).
