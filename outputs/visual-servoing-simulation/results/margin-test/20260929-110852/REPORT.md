# Latency-margin test: MEASURED

Freshness watchdog response: **hold and resume** (`stale_resume_s` = 2 s). At the same 400 ms limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and ends as `stale_camera` only if none arrives within 2 s. Holds are reported separately; the margin counts alignments the watchdog ended.

How much extra per-frame perception latency each method tolerates before alignment fails. A fixed delay is added to every active frame; the production controller, 400 ms freshness watchdog, 50 ms control deadline and 50 ms transport delay are unchanged.

| Method | Normal processing p50 / p99 (without injected delay) | Tolerated added delay | Equivalent slowdown | First failing level |
|---|---:|---:|---:|---:|
| SIFT | 45.2 / 56.9 ms | +150 ms (excluding +125 ms) | 4.3x slower perception | +200 ms |
| Learned GPU | 44.8 / 55.9 ms | +150 ms (excluding +125 ms) | 4.3x slower perception | +200 ms |

"Tolerated" is the largest level where it and every lower unaffected level aligned 100% with no alignment ended by the freshness watchdog. Equivalent slowdown = (normal p50 + tolerated delay) / normal p50.

## SIFT

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 45.2 / 56.9 | +0 | 113.5 | 4.21 s | no |
| +25 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 77.3 / 94.2 | +32 | 150.0 | 4.10 s | no |
| +50 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 101.3 / 122.9 | +56 | 178.5 | 3.93 s | no |
| +75 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 127.2 / 141.6 | +82 | 197.2 | 3.83 s | no |
| +100 ms | 10/10 | 0 (0 / 0) | 21 (0.4 s) | 153.5 / 172.7 | +108 | 228.2 | 3.71 s | no |
| +125 ms | 10/10 | 0 (0 / 0) | 151 (5.8 s) | 177.0 / 194.4 | +132 | 251.2 | 4.85 s | YES - excluded from the margin |
| +150 ms | 10/10 | 0 (0 / 0) | 251 (20.1 s) | 203.4 / 223.2 | +158 | 278.9 | 6.79 s | no |
| +200 ms | 0/10 | 0 (0 / 0) | 1600 (254.9 s) | 254.3 / 269.9 | +209 | 325.5 | 42.80 s | no |

## Learned GPU

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 44.8 / 55.9 | +0 | 110.5 | 4.32 s | no |
| +25 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 78.6 / 131.2 | +34 | 184.5 | 4.16 s | no |
| +50 ms | 10/10 | 0 (0 / 0) | 0 (0.0 s) | 104.7 / 157.0 | +60 | 210.4 | 4.07 s | no |
| +75 ms | 10/10 | 0 (0 / 0) | 111 (4.2 s) | 171.8 / 193.3 | +127 | 247.2 | 4.86 s | no |
| +100 ms | 10/10 | 0 (0 / 0) | 124 (10.3 s) | 192.6 / 219.3 | +148 | 273.6 | 6.03 s | no |
| +125 ms | 10/10 | 0 (0 / 0) | 205 (19.6 s) | 200.9 / 317.8 | +156 | 368.5 | 6.79 s | YES - excluded from the margin |
| +150 ms | 10/10 | 0 (0 / 0) | 320 (43.5 s) | 253.6 / 336.9 | +209 | 391.8 | 9.48 s | no |
| +200 ms | 0/10 | 0 (0 / 0) | 644 (160.1 s) | 298.1 / 321.6 | +253 | 375.1 | 26.95 s | no |

## Test procedure

Procedure: `docs/TEST_PROCEDURE.md`. Conditions this run could check:

- Fresh restart: yes (computer running 4 min when the test started; the procedure asks for at most 60 min)
- AC power at every session start: yes
- Windows power mode Best performance: yes
- Not checked automatically: other programs and browser tabs closed, laptop maker's utility in performance mode, ventilation. Note in your results whether these were followed.

## Safety and validity

- Unsafe motion, post-stop motion, forbidden contacts and unlatched stops across all levels: 0. Control deadline misses: 0.
- A level marked "laptop low-power state" had 30 s windows with the GPU busy below 400 MHz or the CPU below 85% of maximum frequency. Its result mixes the injected delay with a real slowdown; repeat the run before relying on it.
- "At start" trips happened before the first command of an alignment: the first fresh result must arrive within 400 ms of Align, and it can queue behind a frame already in progress. "Mid-alignment" trips happened after the robot was already moving.
- The injected delay is constant. Real slower hardware also has longer tails, so the tolerated delay is an upper bound for hardware with the same median slowdown.
- Each level has 10 alignments over 5 poses on a fresh worker; a 100% level is limited evidence, not a guarantee.
- Windows power mode at every session start: Best performance
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files changed: app.py, realtime.py, realtime_app.py, run_latency_stress.py, run_latency_study.py; runtime versions identical; approved change to `realtime.py`: freshness watchdog hold-and-resume, default on (stale_resume_s = 2 s; 0 = original stop)
- Python `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`, CUDA GPU NVIDIA GeForce RTX 3050 6GB Laptop GPU.

Per-level data: `margin.csv` (for plotting) and `margin.json`; per-session evidence in `margin-*.json` and `traces/`.
