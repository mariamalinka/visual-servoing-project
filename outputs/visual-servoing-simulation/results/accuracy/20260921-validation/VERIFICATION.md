# Verification: physical accuracy and calibration sensitivity

The experiment completed all **72/72 trials**. The software checks passed;
physical acceptance was **48/72**, with the Learned misses retained as measured
results. See [findings](FINDINGS.md) and the [per-profile report](REPORT.md).

## Software and integration

- `run.cmd --verify`: **240 tests passed in 376.961 s**, followed by a passing
  app/scene verification. [Full output](full-tests.txt).
- The suite includes 16 metric/calibration/plan tests and six rendered app
  integration tests. Coverage includes pose units and frame invariance, rotation
  wrapping, mount lever arms, independently finite-differenced virtual-camera
  Jacobians, real command changes, unchanged true rendering/geometry, and
  calibration persistence through UI/gain/target changes.
- A real two-trial ArUco run was interrupted after its first completed trial.
  Resume preserved the first result and trace bytes and completed the second.
  Report-only regeneration passed. A modified teaching-goal record was rejected
  before execution. [Resume check output](resume-check.txt).
- Final source/assets/runtime fingerprint, plan hash, teaching-goal hash and
  private reference hashes match the completed manifest. All 72 planned IDs are
  present exactly once.

Repository syntax/link/generated-file checks passed for 738 prospective Git files
(60.6 MiB). `git diff --check` passed. Archives, environments and bulk traces remain
excluded. Existing project changes were preserved.

## Experiment invariants

- ArUco: 24/24 image passes; 24/24 physical passes.
- SIFT: 24/24 image passes; 24/24 physical passes.
- Learned: 16/24 image passes; 0/24 physical passes; seven timeouts and one
  invalid-depth stop.
- All trials include 30 fresh post-stop captures and zero post-stop commands.
- Zero forbidden contacts and zero recorded safety violations. The smallest
  clearance above the configured collision margin was 3.000 mm.
- The original saved application references remained unchanged. The experiment
  used private captures and kept evaluation teaching poses outside the controller.

## Independent geometry check

The [projection analysis](GEOMETRY_CHECK.md) uses actual camera poses and true
intrinsics, independently of matching and PnP. Known pinhole examples, translated
camera coordinates and behind-camera rejection passed. Its script and input
hashes are recorded. Fourteen image-converged trials had geometric corner change
below 1 px but tool-frame position error above 2 mm.

The three published figures and the application calibration controls were
visually inspected. These measurements apply to the simulated scene and two
seeded starts; they do not establish hardware accuracy or population reliability.

[Machine-readable verification](verification.json) includes provenance and
per-mode counts. Bulk traces remain local and ignored by Git.
