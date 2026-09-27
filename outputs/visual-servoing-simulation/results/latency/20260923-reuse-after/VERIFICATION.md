# Verification of the reused-worker candidate

The implementation passed **294 regression tests** in 398.215 seconds before the
campaign. [Full test output](full-tests.txt) includes Learned/eager graph output
equivalence, dynamic-shape fallback, fixed-buffer reuse, input-value equivalence,
physical-accuracy, collision, watchdog and stop-latching checks. No application
source changed during or after the completed campaign.

[Machine-readable verification](verification.json) records:

- All 40 new and 40 baseline raw trace checksums verified.
- Identical campaign plans, controller/calibration/scene/model inputs and
  dependency versions; intentional implementation changes separately identified.
- Complete measured frame/command/loop telemetry and reconciled capture counts.
- 73,542 transformer-block graph replays and zero eager fallbacks in active
  uncached Learned frames; nine graphs initialized in every Learned worker.
- AC observed at all 80 before/after session boundaries, without continuous
  power-source monitoring.
- Zero recorded unsafe motion ticks, motion after stop or forbidden contacts;
  every explicit stop remained latched through its observation window.

**Timing validation failed:** Learned had 663 freshness stops and 14 control
misses; SIFT had 34 and 10. See [comparison](COMPARISON.md) and
[diagnosis](DIAGNOSIS.md). Successful evidence collection and passing functional
tests are not a successful sustained timing validation.

Native WPR attribution remains unavailable because Windows denied profiling
privileges (`0xc5585011`). Raw traces and large profiler JSON files remain local;
reports, compact metrics, checksums and selected diagnostic samples are curated
for Git. The original failed campaign has not been rewritten.
