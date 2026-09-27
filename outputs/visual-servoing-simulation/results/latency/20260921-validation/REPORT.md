# Wall-clock latency and overload experiment

Measured on the recorded host. Each matcher uses the same joint offset and precision stop. The configured camera budget includes queueing, rendering, inference, IPC and transport. The control loop requests a 2 ms period; measured gaps are reported below.

| Matcher | Profile | Budget (ms) | Outcome | Pass | Command age p95 / p99 (ms) | Loop gap p99 (ms) | Watchdog lateness (ms) |
|---|---|---:|---|---|---:|---:|---:|
| aruco | nominal | 400 | converged | yes | 16.5 / 19.4 | 6.74 | -- |
| aruco | transport-50ms | 400 | converged | yes | 64.5 / 71.8 | 7.03 | -- |
| aruco | inference-stall | 400 | stale_camera | yes | 18.5 / 22.4 | 6.87 | 2.03 |
| aruco | render-stall | 400 | stale_camera | yes | 18.7 / 21.2 | 6.97 | 0.54 |
| natural | nominal | 400 | converged | yes | 141.1 / 188.6 | 6.07 | -- |
| natural | transport-50ms | 400 | stale_camera | NO | 174.9 / 187.6 | 6.40 | 2.65 |
| natural | inference-stall | 400 | stale_camera | yes | 128.2 / 132.4 | 7.36 | 1.40 |
| natural | render-stall | 400 | stale_camera | yes | 116.5 / 128.1 | 6.36 | 0.98 |
| learned | nominal | 400 | converged | yes | 125.9 / 238.3 | 7.54 | -- |
| learned | transport-50ms | 400 | stale_camera | NO | 176.1 / 192.6 | 7.44 | 1.52 |
| learned | inference-stall | 400 | stale_camera | yes | 116.0 / 123.1 | 6.64 | 0.85 |
| learned | render-stall | 400 | stale_camera | yes | 123.7 / 127.3 | 6.64 | 1.40 |

Passed: 10/12.

The two stall profiles block the actual sensor worker for 800 ms after two seconds. A passing stall trial must first issue motion, stop while the worker is still busy, return zero commands after stopping, and remain stopped when its late result arrives. The maximum allowed watchdog lateness is 50 ms; this is a test bound, not a hard real-time guarantee.

Raw accepted-frame timings and all received sensor timestamps are in each trial JSON. Initialization and model warm-up occur with zero command before arming. Accepted-frame percentiles exclude rejected stale frames; sensor_frames retains their timings. All buffers are bounded (one pending request, one pending result, eight delayed results; 4096 telemetry frames and 100000 loop gaps).

These are visible-start wall-clock trials, not a repeat of the full physical calibration or cold-search campaigns. Host load changes timing. Results should be repeated on the deployment machine.

[Validation, regression checks and interpretation](VERIFICATION.md).

![Latency and watchdog measurements](latency-and-watchdog.png)
