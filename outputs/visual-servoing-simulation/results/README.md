# Published experiment evidence

This folder contains a curated set of reports, figures, plans, metrics and
representative observations. The allowlist in [.gitignore](.gitignore) controls
what is included in Git. New experiment directories are local by default.

## Published studies

| Study | Report |
|---|---|
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
