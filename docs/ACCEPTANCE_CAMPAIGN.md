# Automated acceptance campaign

From the repository root, run:

```powershell
& './outputs/visual-servoing-simulation/.venv/Scripts/python.exe' -B tools/run_acceptance_campaign.py --output outputs/visual-servoing-simulation/results/acceptance/my-run
```

Choose a new output directory for every run. Existing evidence is never overwritten. The process exits **0 only for a complete acceptance PASS**, otherwise **1**. The optional `--smoke` runs a short harness check with status `SMOKE_ONLY`; it cannot certify acceptance. Its successful completion can exit 0 even when full-duration criteria are unmet.

The checked-in `tools/acceptance_campaign.json` is the predeclared contract. The default schedule is SIFT, Learned GPU, SIFT, Learned GPU: **four 30-minute sessions**, about two hours plus startup and final-attempt completion. Each session keeps one sensor process and CUDA context alive throughout all its alignments. There are no per-alignment worker restarts. A future config may increase session length to 60 minutes; at least two sessions per method are required.

Three joint offsets repeat within every session, in degrees from the taught home pose:

1. `[3, -3, 4, 3, -2, 2]` (the validated latency starting offset).
2. `[1.5, -1.5, 2, 1.5, -1, 1]`.
3. `[-3, 3, -4, -3, 2, -2]`.

Only explicit offset resets vary through a harness-local Simulation subclass. Home/reference capture, collision checks, production files, controllers, calibration, perception/model configuration, and the single initial worker warm-up remain unchanged. Every pose is checked against joint limits and collision clearance before execution.

## Acceptance criteria fixed before execution

Every session must meet every criterion:

| Criterion | Limit |
|---|---:|
| Alignment success: 95% lower confidence bound (exact Clopper–Pearson) | ≥ 95% |
| Failed alignments | 0 |
| Reused-worker duration | At least 1,800 seconds |
| Coverage | At least 20 attempts per declared pose |
| Camera and tool position error, every attempt | ≤2 mm |
| Camera and tool orientation error, every attempt | ≤1° |
| Capture-to-command p99 / maximum | ≤250 / 400 ms |
| Uncached processing p95 / p99 / maximum | ≤150 / 175 / 250 ms |
| Later processing p99, relative to first attempt at the same pose | ≤1.5× |
| Freshness trips / control deadline misses | 0 / 0 |
| Unsafe motion / post-stop motion / forbidden contacts | 0 / 0 / 0 |
| Unlatched stops / worker restarts / runtime errors / telemetry loss | 0 |
| Clearance above the collision margin, distance to joint limits, peak commanded and measured joint speed (REQ-11 to REQ-13), every physics step | ≥ 0 mm, ≥ 0 rad, ≤ 0.6 rad/s |

The last row applies to sessions that record the safety envelope (since 2026-10-01).
Older sessions show "not recorded" and keep their verdicts. The report has a
per-session safety-envelope table.

The success criterion uses the lower bound of the 95% confidence interval, not the
observed rate; see [Success rates and confidence intervals](STATISTICS.md). A 30-minute
session gives a few hundred alignments, well above the 72 all-success alignments
needed to show 95%. Campaigns declared before this change used "100% observed". Their
saved configuration keeps that rule, and continuing such a campaign
(`--continue-from`) requires the same configuration.

The runtime freshness watchdog remains **400 ms**, the control deadline **50 ms**, and added transport delay **50 ms**. Acceptance latency gates are reporting gates; they do not change control behavior or replace those safety limits. Physical thresholds reuse the existing accuracy study's 2 mm / 1° contract. Latency targets add explicit engineering headroom below the freshness ceiling; they are not claimed hard-real-time guarantees.

Safe watchdog/deadline stops are counted as failures, and the scheduled campaign continues to characterize sustained behavior. Unsafe motion, forbidden contact, an unlatched stop, or a runtime failure aborts the remaining schedule. An incomplete campaign cannot pass. Failed criteria are never relaxed automatically.

## Measurements and retained evidence

- `manifest.json`: frozen criteria, runtime configuration, source and model hashes, dependency/OS/Python versions, launch and actual Windows executable paths, virtual environment, selected CUDA GPU inventory, driver and power metadata, and harness/config hashes.
- `progress.json`: current worker PID, session elapsed time, alignment counts and cumulative counters.
- Per-session JSON: success rate, image error, physical errors, capture-to-command latency, uncached processing, first/later performance, matched-pose comparisons, failures and timing-data checksum.
- `REPORT.md` and `verdict.json`: explicit pass/fail comparison; refreshed after each session and on completion/error.
- `traces/*jsonl.gz`: raw sensor stages, accepted commands, control intervals, events, GC/context timing diagnostics; failed and late frames are retained.
- `traces/*-attempts.json`: endpoint joint poses, outcome, worker PID, image stopping metrics, independently scored camera/tool pose errors.
- `traces/*-ending.json`: final counters, execution settings and runtime errors.
- `traces/gpu.csv`: continuous GPU clock, utilization, power, temperature and VRAM telemetry.

Processing percentiles exclude cache hits and include unsuccessful/late active frames. Capture-to-command statistics cover accepted commands; discarded results remain in sensor telemetry and counters. First/later aggregate statistics describe the first alignment versus all subsequent alignments in each worker. The regression gate separately compares each pose's first attempt with later attempts at that pose, preventing a different starting pose from masquerading as degradation.

Physical accuracy is evaluated offline from the post-stop joint snapshot using a separate, non-rendering simulation and the existing SE(3) accuracy scorer. Both camera and tool frames are compared with the taught home pose. This ground truth never reaches the image controller. Failed alignments remain in physical-error totals.

## Bounded telemetry without restarting workers

The original runtime's bounded telemetry deques cannot retain an entire half-hour. A harness subclass asks the control owner to copy and clear telemetry at its normal inactive end-of-tick publication, after the existing 1.1-second post-stop observation. The main harness serializes the chunk to gzip before rearming; no file I/O runs in the active control loop. The worker, streaming policy, CUDA context, and original bounded request/result mailboxes remain alive and unchanged. There is no unbounded writer or perception queue. This adds measured inactive serialization time between alignments; it is not an exact replay of the older two-minute timing study.

Counters are never reset when buffers are drained. Analysis reconciles received-frame and accepted-command counts, detects duplicate sequence IDs, checks stable worker PIDs and latched stops, and rejects telemetry evictions. Disk/serialization/runtime errors prevent a PASS. Full-session aggregation and physical scoring occur after the worker closes.

## Future regression runs

The default command requires the exact validated source/dependency/input fingerprint from the last passing latency campaign. To evaluate an intentional Python implementation change against the same criteria, add `--allow-code-changes`. This permits changed Python source while still pinning dependency versions, scene, configuration, assets/reference and model-manifest inputs. Every new source hash is recorded, and source must stay unchanged during the campaign. Changes to controller configuration, model settings, dependencies or acceptance criteria require a separately declared validation contract; do not silently mix their results with this campaign.

Raw timing/GPU evidence remains local under ignored `traces` directories. Keep small manifests, summaries and reports for GitHub. A passing finite campaign supports repeatability under its recorded conditions; it does not prove an absolute worst-case execution bound or establish hardware safety certification.

Failure-gate unit tests:

```powershell
& './outputs/visual-servoing-simulation/.venv/Scripts/python.exe' -B -m unittest discover -s tools -p test_acceptance_campaign.py -v
```
