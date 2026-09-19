# Randomized visual-servoing benchmark

> Published snapshot: bulk per-frame traces remain local. See the [dataset policy](../../README.md) before regenerating this report.


Completed **100 trials**, RNG seed **42**. Controller: unchanged step-2 fixed-gain marker IBVS.

- All starts: **44/100** converged; 44.0% [34.7, 53.8].
- Starts with the marker initially detected: **44/44**; 100.0% [92.0, 100.0].
- Brackets show 95% Wilson confidence intervals for the declared starting-pose distribution.

![Benchmark overview](benchmark_overview.png)

| Profile | Sampled starts | Marker detected at start | Converged | Success, all starts (95% CI) | Success, detected starts (95% CI) |
|---|---:|---:|---:|---|---|
| small | 30 | 26 | 26 | 86.7% [70.3, 94.7] | 100.0% [87.1, 100.0] |
| medium | 37 | 17 | 17 | 45.9% [31.0, 61.6] | 100.0% [81.6, 100.0] |
| large | 33 | 1 | 1 | 3.0% [0.5, 15.3] | 100.0% [20.7, 100.0] |

## Sampling

Each trial chooses a profile uniformly, then samples six independent uniform joint offsets within that profile. No invisible starts are rejected. Camera poses follow the joint kinematics; they are not uniform in Cartesian space.

Bounds below are symmetric offsets about home, in joint degrees (J1 through J6).

| Profile | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| small | +/-3 | +/-3 | +/-4 | +/-4 | +/-3 | +/-4 |
| medium | +/-8 | +/-8 | +/-10 | +/-12 | +/-8 | +/-12 |
| large | +/-18 | +/-18 | +/-25 | +/-30 | +/-18 | +/-30 |

The plan is saved before simulation. No starts are rejected for visibility and no failed trials are rerun to improve the score.
Joint-space sampling induces a nonuniform camera-pose distribution. These intervals describe this simulation experiment, not physical-robot reliability.

## Outcomes

| Outcome | Count | First recorded example |
|---|---:|---|
| Converged | 44 | [Images and trial data](cases/converged/) |
| Out of view at start | 56 | [Images and trial data](cases/initial_out_of_view/) |
| Not detected at start | 0 | - |
| Left camera view | 0 | - |
| Tracking lost in frame | 0 | - |
| Depth rejected | 0 | - |
| Stalled at joint limit | 0 | - |
| Stalled near singularity | 0 | - |
| Stalled | 0 | - |
| Timeout | 0 | - |
| Unstable after stopping | 0 | - |

Examples are the first occurrence of each outcome, not selected for best or worst performance.
Initial loss is distinguished from loss during motion. FOV labels use evaluation-only geometric projection; the controller never receives this ground truth.
A stalled trial is not labeled a proven local minimum. Joint-limit and singularity labels indicate observed conditions during the stall.

## Timing and accuracy

![Performance](alignment_performance.png)

- Median declared-convergence time, successes only: **4.03 simulated seconds**.
- Median settling time, successes only: **3.53 simulated seconds**.
- Median final RMS corner error, successes only: **0.976 px**.

## Files and reproduction

- plan.json: exact starting offsets and trial IDs.
- manifest.json: source hashes, package versions, sampling and controller configuration.
- trials.jsonl and trials.csv: one summary per sampled start.
- traces/trial_NNNN.npz: numeric per-frame time, corners, depth, commands, actual joint rates, camera pose, conditioning and FOV diagnostics. Load with numpy.load(..., allow_pickle=False).
- summary.json: rates, confidence intervals and per-profile statistics.

From the project folder, regenerate the figures without running physics:

    python analyze_benchmark.py "PATH_TO_THIS_RUN"

Replay one exact starting pose (current source hashes must match the saved run):

    python benchmark.py --replay-run "PATH_TO_THIS_RUN" --trial-id 12

This step benchmarks one controller under fixed lighting and idealized dynamics. Baseline comparisons, learned features, noise/latency sweeps and physical validation are not included.

[Wilson interval method: NIST](https://www.itl.nist.gov/div898/handbook/prc/section2/prc241.htm).
