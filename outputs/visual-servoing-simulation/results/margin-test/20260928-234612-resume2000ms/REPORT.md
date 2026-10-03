# Latency-margin test: MEASURED

Freshness watchdog response: **hold and resume** (`stale_resume_s` = 2 s). At the same 400 ms limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and ends as `stale_camera` only if none arrives within 2 s. Holds are reported separately; the margin counts alignments the watchdog ended.

How much extra per-frame perception latency each method tolerates before alignment fails. A fixed delay is added to every active frame; the production controller, 400 ms freshness watchdog, 50 ms control deadline and 50 ms transport delay are unchanged.

| Method | Normal processing p50 / p99 (without injected delay) | Tolerated added delay | Equivalent slowdown | First failing level |
|---|---:|---:|---:|---:|
| SIFT | 45.5 / 59.8 ms | +150 ms | 4.3x slower perception | +200 ms |
| Learned GPU | 44.6 / 58.1 ms | +150 ms | 4.4x slower perception | +200 ms |

"Tolerated" is the largest level where it and every lower unaffected level aligned 100% with no alignment ended by the freshness watchdog. Equivalent slowdown = (normal p50 + tolerated delay) / normal p50.

## SIFT

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 45.5 / 59.8 | +0 | 117.0 | 4.22 s | no |
| +25 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 76.9 / 95.8 | +31 | 151.0 | 4.14 s | no |
| +50 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 103.3 / 121.9 | +58 | 176.9 | 3.93 s | no |
| +75 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 126.3 / 149.9 | +81 | 205.8 | 3.83 s | no |
| +100 ms | 10/10 | 0 (0 / 0) | 12 (0.2 s) | 151.1 / 169.2 | +106 | 225.4 | 3.71 s | no |
| +125 ms | 10/10 | 0 (0 / 0) | 144 (5.4 s) | 176.5 / 196.8 | +131 | 252.3 | 4.77 s | no |
| +150 ms | 10/10 | 0 (0 / 0) | 256 (20.1 s) | 204.2 / 218.3 | +159 | 273.8 | 6.81 s | no |
| +200 ms | 0/10 | 0 (0 / 0) | 1591 (256.7 s) | 255.0 / 271.5 | +209 | 327.3 | 42.62 s | no |

## Learned GPU

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 44.6 / 58.1 | +0 | 113.0 | 4.33 s | no |
| +25 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 79.2 / 127.1 | +35 | 181.8 | 4.38 s | no |
| +50 ms | 10/10 | 0 (0 / 0) | 1 (0.0 s) | 106.8 / 159.2 | +62 | 212.5 | 3.99 s | no |
| +75 ms | 10/10 | 0 (0 / 0) | 112 (4.4 s) | 171.1 / 195.2 | +126 | 249.4 | 4.87 s | no |
| +100 ms | 10/10 | 0 (0 / 0) | 131 (10.8 s) | 196.6 / 275.4 | +152 | 285.5 | 6.00 s | no |
| +125 ms | 10/10 | 0 (0 / 0) | 245 (27.4 s) | 227.4 / 292.1 | +183 | 338.3 | 8.33 s | no |
| +150 ms | 10/10 | 0 (0 / 0) | 342 (52.3 s) | 254.9 / 288.3 | +210 | 335.4 | 10.12 s | no |
| +200 ms | 0/10 | 0 (0 / 0) | 599 (150.2 s) | 298.1 / 324.4 | +254 | 377.1 | 31.91 s | no |

## Test procedure

Procedure: `docs/TEST_PROCEDURE.md`. Conditions this run could check:

- Fresh restart: NO (computer running 180 min when the test started; the procedure asks for at most 60 min)
- AC power at every session start: yes
- Windows power mode Best performance: yes
- Not checked automatically: other programs and browser tabs closed, laptop maker's utility in performance mode, ventilation. Note in your results whether these were followed.

## Safety and validity

- Unsafe motion, post-stop motion, forbidden contacts and unlatched stops across all levels: 0. Control deadline misses: 0.
- No laptop low-power state was detected during any level.
- "At start" trips happened before the first command of an alignment: the first fresh result must arrive within 400 ms of Align, and it can queue behind a frame already in progress. "Mid-alignment" trips happened after the robot was already moving.
- The injected delay is constant. Real slower hardware also has longer tails, so the tolerated delay is an upper bound for hardware with the same median slowdown.
- Each level has 10 alignments over 5 poses on a fresh worker; a 100% level is limited evidence, not a guarantee.
- Windows power mode at every session start: Best performance
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files changed: app.py, realtime.py, realtime_app.py; runtime versions identical; approved change to `realtime.py`: freshness watchdog hold-and-resume (RuntimeConfig.stale_resume_s, default 0 = original stop)
- Python `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`, CUDA GPU NVIDIA GeForce RTX 3050 6GB Laptop GPU.

Per-level data: `margin.csv` (for plotting) and `margin.json`; per-session evidence in `margin-*.json` and `traces/`.
