# 30 random worker kills on the code before the fix

This folder holds the `sensor_worker_crash` scenario repeated 30 times on the runtime
*before* the receiver thread (`realtime.py` 521a6998689a, the hash in `summary.json`).
It was run in the cloud workspace on 2026-10-02 to compare the kill-to-stop time with
the fixed runtime.

**Result:** 29/30 passed. Repeat 6 failed with "Control loop did not shut down within
five seconds". That is the original defect: the first fault campaign saw it once in
5 kills. Here it reappeared from a random kill, not a targeted one.

| Runtime | Random kills | Control loop froze | Kill to `worker_failed` stop: p50 / p90 / max |
|---|---|---|---|
| Before (this folder) | 30 | 1 | 12.0 / 14.9 / 21.7 ms (29 measured) |
| After (`../../fault-injection/20261002-141659-worker-crash` and `-all`) | 34 | 0 | 12.1 / 14.3 / 20.8 ms |

On the fixed runtime, one more repeat (35 in all) never killed the worker: a start-up
`control_overrun` ended the alignment first. At about 1 in 30, these counts alone cannot show the defect is gone. That is shown by
the deterministic scenarios `sensor-worker-partial-message` and
`sensor-result-read-blocked`, 35/35 each on the fixed runtime. The table shows that
detecting a dead worker takes as long as it did before.
