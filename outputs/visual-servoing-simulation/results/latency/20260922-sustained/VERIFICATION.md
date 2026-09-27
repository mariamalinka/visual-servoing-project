# Verification

**The implementation checks passed; the sustained timing requirement failed.**

- Full regression suite: **290 tests passed in 373.914 seconds**, exit code 0.
- All 40 raw trace checksums verified. Capture slots and every acquired request reconcile with completion, expiry and drop counters.
- Source/runtime fingerprint is unchanged from campaign start through completion and after the full regression suite. Controller, calibration, scene, references and model inputs match the preceding comparison.
- No frame, command, loop, slow-control-cycle or parent-GC telemetry eviction was recorded. Worker GC evidence remains the bounded last-eight-event context described in the guide.
- All stops stayed latched; no unsafe-motion, post-stop-motion or forbidden-contact ticks were recorded.
- Main campaign: 20 sessions per method. The 10,000-uncached-frame target was met by SIFT and missed by Learned before the predeclared cap.
- The separate probe-disabled and GPU-sampling runs also reproduced Learned freshness failures. They are excluded from the main statistical totals.
- Windows denied kernel profiling policy activation (`0xc5585011`); exact scheduler/native-wait attribution remains unavailable.

New checks cover high-percentile computation, retaining late failed work, resetting feedback-age bounds between attempts, bounded diagnostic storage, CUDA-event geometry preservation without a new global synchronization, whole-session resampling and attribution around watchdog stops.

[Full test output](full-tests.txt), [verification data and analysis hashes](verification.json), [statistical report](REPORT.md), [diagnostic interpretation](DIAGNOSIS.md).
