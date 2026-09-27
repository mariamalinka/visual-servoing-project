# Optimization validation

Changes and experiments were performed in the original project folder. Previous project changes and historical experiment evidence are preserved. This report accompanies the [before/after latency comparison](REPORT.md).

## Root cause and selected implementation

The original 277.5 ms SIFT and 227.5 ms Learned frames lacked internal stage traces. Timing-only baseline instrumentation reproduced long frames: a 236.9 ms SIFT processing interval contained 175.9 ms refinement, and a 292.3 ms Learned interval contained 221.2 ms refinement. These are newly measured reproductions, not decompositions of the original two frames.

A replay of 86 captured poses compared refinement schedules. The selected 8-iteration half-resolution plus 12-iteration full-resolution schedule preserved the original 1e-6 ECC epsilon and quality gates. Replay refinement maxima fell from 220.1 to 67.2 ms for SIFT and from 287.4 to 75.4 ms for Learned; maximum corner changes relative to the old algorithm were 0.0327 and 0.0231 px. A single-resolution shortcut shifted a Learned corner by about 0.6 px and was rejected. [Replay measurements](refinement-probe.json).

SIFT coordinate conversion and duplicate filtering are vectorized. Learned gathers matched points and scores on the GPU and transfers one compact array to the CPU. The goal pyramid and mask kernel are prepared once, and redundant RGB caching is reduced. The runtime skips busy capture slots, keeps bounded mailboxes, and rejects expired copied requests before expensive work. No image resolution, feature limit, confidence gate or stopping tolerance was relaxed. [Implementation details](../../../PERCEPTION_LATENCY.md).

## Freshness and overload

5/6 final trials aligned with 50 ms added wall-clock transport and the unchanged 400 ms budget. There were 0 freshness stops and 1 control deadline misses. There were no forbidden contacts, unsafe motion ticks, or commands after stopping. Maximum held-feedback-age bounds are reported in the comparison table.

One final Learned trial stopped on a 65.9 ms control-loop gap after four accepted frames; command application took 40.5 ms around that event, while the surrounding sensor processing intervals were about 79 ms. The saved wall-clock timings do not distinguish time computing from time descheduled in the command/guard stage. This remaining control-path spike is not resolved by the perception optimization. The failed trial is retained, not replaced by a passing rerun. A 400 ms camera budget alone therefore does not establish reliable uninterrupted control. Both accepted samples and all-active sensor completions are reported because stopped trials are censored. Three short trials per matcher are insufficient to establish a hard upper bound or a reliable deployment p99.

All four 800 ms rendering/inference stall trials stopped while the worker remained busy. Watchdog lateness ranged from 1.00 to 5.47 ms. Late results did not resume motion, and all workers shut down. [Stall report](../tail-stall-validation-v2/REPORT.md).

## Physical accuracy and regression

All 12 seeded accuracy trials passed: SIFT and Learned, three starts each, with nominal and combined calibration perturbations. The original 2 mm / 1 degree physical tolerances and stricter image/estimated-pose stopping gates remained enabled. All 360 fresh post-stop checks passed with zero commands and no safety violations. This independent simulation-time accuracy study uses 100 ms simulated transport; it is separate from the 50 ms wall-clock experiment.

| Method | Maximum camera error (mm) | Maximum tool error (mm) | Maximum angle (degrees) |
|---|---:|---:|---:|
| SIFT | 0.261 | 0.287 | 0.0346 |
| Learned | 0.364 | 0.434 | 0.0659 |

[Physical accuracy report](../../accuracy/20260922-tail-precision/REPORT.md). These errors use simulator ground truth for evaluation only; the controller does not use the taught physical pose.

The final full regression suite passed **281 tests** in 428.8 s. It includes independently warped image truth, invalid registration rejection, exact-pixel cache behavior, request expiration, slow-worker queue bounds, and watchdog/rearm tests. [Full test log](full-tests.txt). An initial timing-instrumentation cache-identity regression was repaired. A termination-during-wait diagnostic then found shutdown blocked in multiprocessing.Event.set/Condition.notify_all after the worker died. Closing a dedicated pipe now wakes the worker without that shared condition, and result receipts replace the shared readiness event. The full suite covers the reproduced failure. [Captured pre-fix shutdown stack](worker-termination-before.txt).

## Audit and reproduction

Source and dependency fingerprints match across the final transport, stall and accuracy manifests and the validated source. Scene, reference images, controller and calibration hashes are unchanged from the baseline. Test-source hashes and raw evidence hashes are in [verification.json](verification.json).

- [Baseline manifest](../tail-before-profiled/manifest.json) and [baseline trials](../tail-before-profiled/REPORT.md).
- [Final transport manifest](../tail-after-final-v2/manifest.json) and [all final trials](../tail-after-final-v2/REPORT.md).
- [Stall manifest](../tail-stall-validation-v2/manifest.json).
- [Accuracy plan](../../accuracy/20260922-tail-precision/plan.json) and [accuracy manifest](../../accuracy/20260922-tail-precision/manifest.json).

Run each command from the repository root. Use new output directories for timing runs; do not run competing benchmarks concurrently.

```powershell
.\run.cmd --latency-study --modes natural learned --profiles transport-50ms --repeats 3 --max-camera-age-ms 400
.\run.cmd --latency-study --modes natural learned --profiles inference-stall render-stall
.\run.cmd --accuracy-study --modes natural learned --profiles nominal combined --starts 3
Push-Location outputs/visual-servoing-simulation
.\.venv\Scripts\python.exe -B -m unittest discover -s tests -v
Pop-Location
```

Timing runs warm the models before arming. Python, Windows and GPU scheduling can still stall; iteration limits and these measured results are not a hard real-time guarantee. The existing freshness watchdog remains necessary.
