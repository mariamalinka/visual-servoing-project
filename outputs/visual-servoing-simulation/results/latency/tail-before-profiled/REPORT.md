# Wall-clock latency and overload experiment

Measured on the recorded host. Each matcher uses the same joint offset and precision stop. The configured camera budget includes queueing, rendering, inference, IPC and transport. The control loop requests a 2 ms period; measured gaps are reported below.

| Matcher | Profile | Budget (ms) | Outcome | Pass | Command age p95 / p99 (ms) | Loop gap p99 (ms) | Watchdog lateness (ms) |
|---|---|---:|---|---|---:|---:|---:|
| natural | transport-50ms | 400 | stale_camera | NO | 191.4 / 307.3 | 5.53 | 1.43 |
| learned | transport-50ms | 400 | stale_camera | NO | 195.9 / 265.5 | 9.29 | 0.54 |
| natural | transport-50ms | 400 | stale_camera | NO | 182.2 / 256.6 | 5.82 | 1.61 |
| learned | transport-50ms | 400 | converged | yes | 176.7 / 178.5 | 6.40 | -- |
| natural | transport-50ms | 400 | stale_camera | NO | 184.6 / 262.1 | 6.15 | 1.57 |
| learned | transport-50ms | 400 | stale_camera | NO | 181.4 / 195.0 | 6.31 | 0.99 |

Passed: 1/6.

The two stall profiles block the actual sensor worker for 800 ms after two seconds. A passing stall trial must first issue motion, stop while the worker is still busy, return zero commands after stopping, and remain stopped when its late result arrives. The maximum allowed watchdog lateness is 50 ms; this is a test bound, not a hard real-time guarantee.

Raw accepted-frame timings and all received sensor timestamps are in each trial JSON. Initialization and model warm-up occur with zero command before arming. Accepted-frame percentiles exclude rejected stale frames; sensor_frames retains their timings. All buffers are bounded (one pending request, one pending result, eight delayed results; 4096 telemetry frames and 100000 loop gaps).

These are visible-start wall-clock trials, not a repeat of the full physical calibration or cold-search campaigns. Host load changes timing. Results should be repeated on the deployment machine.
