# Wall-clock latency and overload experiment

Measured on the recorded host. Each matcher uses the same joint offset and precision stop. The configured camera budget includes queueing, rendering, inference, IPC and transport. The control loop requests a 2 ms period; measured gaps are reported below.

| Matcher | Profile | Budget (ms) | Outcome | Pass | Command age p95 / p99 (ms) | Loop gap p99 (ms) | Watchdog lateness (ms) |
|---|---|---:|---|---|---:|---:|---:|
| natural | transport-50ms | 400 | converged | yes | 177.2 / 178.5 | 5.70 | -- |
| learned | transport-50ms | 400 | control_overrun | NO | 132.2 / 136.1 | 7.85 | -- |
| natural | transport-50ms | 400 | converged | yes | 170.0 / 191.3 | 6.35 | -- |
| learned | transport-50ms | 400 | converged | yes | 180.7 / 188.4 | 6.27 | -- |
| natural | transport-50ms | 400 | converged | yes | 163.7 / 165.5 | 6.07 | -- |
| learned | transport-50ms | 400 | converged | yes | 187.0 / 194.2 | 6.86 | -- |

Passed: 5/6.

The two stall profiles block the actual sensor worker for 800 ms after two seconds. A passing stall trial must first issue motion, stop while the worker is still busy, return zero commands after stopping, and remain stopped when its late result arrives. The maximum allowed watchdog lateness is 50 ms; this is a test bound, not a hard real-time guarantee.

Raw accepted-frame timings and all received sensor timestamps are in each trial JSON. Initialization and model warm-up occur with zero command before arming. Accepted-frame percentiles exclude rejected stale frames; sensor_frames retains their timings. All buffers are bounded (one pending request, one pending result, eight delayed results; 4096 telemetry frames and 100000 loop gaps). Busy capture slots are skipped before acquisition; capture timestamps are never moved forward on an old frame.

These are visible-start wall-clock trials, not a repeat of the full physical calibration or cold-search campaigns. Host load changes timing. Results should be repeated on the deployment machine.
