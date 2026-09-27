# Published experiment evidence

This folder contains a curated set of reports, figures, plans, metrics and
representative observations. The allowlist in [.gitignore](.gitignore) controls
what is included in Git. New experiment directories are local by default.

## Published studies

| Study | Report |
|---|---|
| Perception tail latency | [Paired before/after transport and validation](latency/20260921-tail-optimization/REPORT.md) |
| Precision stopping | [72 paired trials, 12 new-start trials and pose sensitivity](accuracy/20260921-precision-final/COMPARISON.md) |
| Physical accuracy and calibration | [Paired physical-pose evaluation](accuracy/20260921-validation/REPORT.md) |
| Collision-aware motion | [Alignment, cold search and recovery](collision/20260920-validation/REPORT.md) |
| Camera robustness | [48-trial disturbance validation](robustness/20260920-validation/VALIDATION.md) |
| Camera delay | [Timestamped feedback and stream interruption](camera-delay/20260919-validation/REPORT.md) |
| Initial IBVS benchmark | [200 starts](benchmark/20260906-155544-117220-n200-seed20260906/REPORT.md) |
| Updated IBVS benchmark | [200 starts](benchmark/20260906-164904-307850-n200-seed20260906/REPORT.md) |
| IBVS holdout | [100 starts, seed 42](benchmark/20260906-165404-850153-n100-seed42/REPORT.md) |
| Controller comparison | [IBVS, PBVS and look-once](comparison/20260907-005242-736914-n200-seed20260906/REPORT.md) |
| Adaptive gain | [Fixed and adaptive policies](gain/20260911-173457-385483-n200-seed20260906/REPORT.md) |
| Recovery | [Find-and-align study](recovery/20260906-212540-109662-n200-seed20260906/REPORT.md) |
| Startup search | [Initially unseen targets](startup/20260911-184428-897577-n200-seed20260911/REPORT.md) |
| Joint-limit supervision | [Bounded alignment retry](joint-limits/20260912-131706-003699/REPORT.md) |
| Search coverage | [Coarse and refined scans](search-coverage/20260913-020201-974031/REPORT.md) |
| Natural image | [ArUco and SIFT](natural-image/20260913-112323-280057/REPORT.md) |
| Learned matching | [SIFT and Learned](learned/20260913-162607-253226/REPORT.md) |
| GPU performance | [Paired measurements and actual alignment](../LEARNED_PERFORMANCE.md) |

The single-run [alignment trace](alignment/trace.csv) is retained as a small,
complete example. Required baseline plans, manifests, trial outcomes and saved
goals remain included for the historical joint-limit and coverage studies.

## Raw traces

Bulk `traces/*.npz` files, duplicate runs, smoke experiments and setup logs are
kept locally and excluded from Git. Original local files were preserved during
the repository cleanup. A fresh checkout contains the published evidence,
not all frame-by-frame data from every historical run.

Run a study to generate a new complete dataset before using its report-only
command. Historical report validators require their complete raw traces; a
published snapshot alone is not sufficient to rerun those validators.
Exact replay studies also enforce recorded source/runtime versions.

The original reports describe their complete local datasets. Their numerical
results have not been recalculated as part of this repository cleanup.

## Add a result intentionally

1. Run the experiment into a new directory.
2. Validate the complete local run and inspect its report.
3. Add a directory or file entry to this folder's allowlist.
4. Keep reports, essential metadata and a few useful figures; leave bulk traces
   and machine-local `latest.json` pointers excluded.
5. Run `tools/check_repository.py` from the repository root before committing.

Downloadable source snapshots can be generated from Git tags. Old step ZIPs are
local archives and are not part of the application or its runtime dependencies.

- [Actual camera latency and worker-stall validation](latency/20260921-validation/REPORT.md). Source/runtime fingerprints and per-trial timing evidence are retained.

- [Sustained latency campaign](latency/20260922-sustained/REPORT.md): 20 sessions per method; the longer run exposed repeated Learned freshness failures and occasional control overruns. [Diagnosis and limits](latency/20260922-sustained/DIAGNOSIS.md).

- [Reused Learned GPU worker candidate](latency/20260923-reuse-after/COMPARISON.md): 40 sessions, 10,506 uncached Learned frames and 14,297 uncached SIFT frames. Average Learned latency improved, but sustained stability failed. [State, clock, profiler and control-overrun diagnosis](latency/20260923-reuse-after/DIAGNOSIS.md).

- [Native scheduling iteration](latency/20260925-native-scheduling-after/COMPARISON.md):
  40 sessions; fewer control misses, continued Learned degradation and increased
  freshness trips. [Native/GC diagnosis](latency/20260925-native-scheduling-after/DIAGNOSIS.md).

- [Learned worker lifetime diagnostics](latency/20260925-worker-lifetime/DIAGNOSIS.md): fresh/reused comparison, continuous GPU telemetry and native CPU attribution; no production changes.

- [GPU/power-settings rerun](latency/20260926-power-settings/POWER_COMPARISON.md): matched reused-worker comparison after laptop settings changes.

- [Underlying Python executable GPU-settings rerun](latency/20260926-base-executable-settings/SETTINGS_COMPARISON.md): three-way comparison; 184/184 reused-worker alignments and zero freshness trips in the matched campaign.
