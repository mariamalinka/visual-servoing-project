# Randomized alignment benchmark

**Stage 4:** the desktop Align button now includes automatic target reacquisition. The experiments below remain the pure-IBVS baselines. [Find-and-align results and instructions](RECOVERY.md). Run `.\run.cmd --recovery` to evaluate the new behavior.

The benchmark places the simulated robot in different starting poses, runs the camera-based Align controller, and saves each attempt's outcome, timing, error and motion trace.

**Latest development:** an intermittent detection failure has been fixed. See [the explanation and validation](ALIGNMENT_FIX.md).

## Recorded experiments

| Experiment | Trials | Initially detected | Aligned | Detection lost during movement |
|---|---:|---:|---:|---:|
| Original detector, seed 20260906 | 200 | 75 | 34 | 41 |
| Fixed detector, same exact poses | 200 | 86 | 86 | 0 |
| Fixed detector, new seed 42 | 100 | 44 | 44 | 0 |

All sampled starts are counted, including those with no marker in view. Successful trials must remain below 1 pixel of error for the controller's 0.5-second hold and one additional stopped second.

- [Original step-3 report](results/benchmark/20260906-155544-117220-n200-seed20260906/REPORT.md)
- [Same-pose report after the detector fix](results/benchmark/20260906-164904-307850-n200-seed20260906/REPORT.md)
- [Separate 100-pose validation report](results/benchmark/20260906-165404-850153-n100-seed42/REPORT.md)
- [Before/after comparison](results/alignment-fix/comparison.json)

The after-fix 200-trial run achieved 43% success over all starts and 86/86 success among initially detected starts. Its remaining failures were 112 out-of-view starts and 2 in-frame starts without a detection. These numbers apply to the declared sampling distribution and idealized scene.

## Run a new benchmark

Open PowerShell in this project folder:

~~~powershell
.\run.cmd --benchmark
~~~

This uses 200 trials and the seed/bounds in [benchmark_config.json](benchmark_config.json). A new timestamped output directory is created each time. Prior results are preserved.

To regenerate the latest report from logs, or run checks:

~~~powershell
.\run.cmd --report
.\run.cmd --verify
~~~

For custom counts or exact replay, use the project's installed Python. On this computer:

~~~powershell
..\..\work\vservo-venv\Scripts\python.exe benchmark.py --trials 20 --seed 42
..\..\work\vservo-venv\Scripts\python.exe benchmark.py --replay-run "results/benchmark/20260906-164904-307850-n200-seed20260906" --trial-id 4
~~~

On another computer set up with setup.cmd, substitute .\.venv\Scripts\python.exe.

Replay checks source hashes, so a baseline run requires the baseline source from the original step-3 archive. Its earlier measurements have been preserved. Replaying the new run uses the current source. Numerical replay can depend on platform, graphics driver and package versions.

## What each run contains

- plan.json: exact joint offsets, profile and trial ID, saved before execution.
- manifest.json: source hashes, versions, reference, calibration and settings.
- trials.jsonl and trials.csv: one outcome per sampled start.
- traces/trial_NNNN.npz: time, corners, commands, depth, joint motion and camera pose.
- summary.json: overall and per-profile statistics with 95% Wilson intervals.
- REPORT.md and PNG/SVG plots: readable results.
- cases/: unmodified camera images from the first occurrence of each outcome.

These are joint-space samples around home, with no visibility filtering. Camera poses are not uniform in Cartesian space. The reports distinguish missing initial features from failures during motion.
