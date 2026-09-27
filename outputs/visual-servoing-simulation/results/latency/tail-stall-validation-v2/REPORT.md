# Wall-clock latency and overload experiment

Measured on the recorded host. Each matcher uses the same joint offset and precision stop. The configured camera budget includes queueing, rendering, inference, IPC and transport. The control loop requests a 2 ms period; measured gaps are reported below.

| Matcher | Profile | Budget (ms) | Outcome | Pass | Command age p95 / p99 (ms) | Loop gap p99 (ms) | Watchdog lateness (ms) |
|---|---|---:|---|---|---:|---:|---:|
| natural | inference-stall | 400 | stale_camera | yes | 115.1 / 115.6 | 6.13 | 1.00 |
| natural | render-stall | 400 | stale_camera | yes | 123.9 / 129.6 | 7.48 | 5.47 |
| learned | inference-stall | 400 | stale_camera | yes | 118.5 / 119.7 | 6.54 | 4.85 |
| learned | render-stall | 400 | stale_camera | yes | 119.8 / 132.7 | 7.00 | 1.35 |

Passed: 4/4.

The two stall profiles block the actual sensor worker for 800 ms after two seconds. A passing stall trial must first issue motion, stop while the worker is still busy, return zero commands after stopping, and remain stopped when its late result arrives. The maximum allowed watchdog lateness is 50 ms; this is a test bound, not a hard real-time guarantee.

Raw accepted-frame timings and all received sensor timestamps are in each trial JSON. Initialization and model warm-up occur with zero command before arming. Accepted-frame percentiles exclude rejected stale frames; sensor_frames retains their timings. All buffers are bounded (one pending request, one pending result, eight delayed results; 4096 telemetry frames and 100000 loop gaps). Busy capture slots are skipped before acquisition; capture timestamps are never moved forward on an old frame.

These are visible-start wall-clock trials, not a repeat of the full physical calibration or cold-search campaigns. Host load changes timing. Results should be repeated on the deployment machine.
