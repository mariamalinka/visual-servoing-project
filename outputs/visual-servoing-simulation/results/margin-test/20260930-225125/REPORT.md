# Latency-margin test: MEASURED

Freshness watchdog response: **hold and resume** (`stale_resume_s` = 2 s). At the same 400 ms limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and ends as `stale_camera` only if none arrives within 2 s. Holds are reported separately; the margin counts alignments the watchdog ended.

How much extra per-frame perception latency each method tolerates before alignment fails. A fixed delay is added to every active frame; the production controller, 400 ms freshness watchdog, 50 ms control deadline and 50 ms transport delay are unchanged.

| Method | Normal processing p50 / p99 (without injected delay) | Tolerated added delay | Equivalent slowdown | First failing level |
|---|---:|---:|---:|---:|
| SIFT | 47.9 / 70.3 ms | +150 ms | 4.1x slower perception | +200 ms |
| Learned GPU | 45.3 / 58.6 ms | +150 ms | 4.3x slower perception | +200 ms |

"Tolerated" is the largest level where it and every lower unaffected level aligned 100% (observed), with no alignment ended by the freshness watchdog. Equivalent slowdown = (normal p50 + tolerated delay) / normal p50.

With 10 alignments per level, 10/10 means the true success rate at that level is at least 69.2% with 95% confidence, not 100%. Showing at least 95% would need 72 alignments per level, so the margin is a characterisation, not a reliability guarantee.

"Aligned" shows the 95% exact Clopper-Pearson confidence interval for the true success rate at each level.

## SIFT

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 47.9 / 70.3 | +0 | 128.1 | 4.22 s | no |
| +25 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 79.8 / 102.9 | +32 | 159.2 | 4.11 s | no |
| +50 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 100.5 / 119.8 | +53 | 175.1 | 3.93 s | no |
| +75 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 128.2 / 145.5 | +80 | 201.3 | 3.87 s | no |
| +100 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 32 (0.7 s) | 156.3 / 171.9 | +108 | 227.8 | 3.88 s | no |
| +125 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 166 (7.4 s) | 181.9 / 207.9 | +134 | 261.4 | 4.90 s | no |
| +150 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 267 (23.1 s) | 208.1 / 231.9 | +160 | 288.3 | 7.11 s | no |
| +200 ms | 0/10 [0.0%, 30.8%] | 0 (0 / 0) | 1544 (275.1 s) | 261.4 / 307.5 | +214 | 357.6 | 42.89 s | no |

## Learned GPU

| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | Held and resumed (total held) | Processing p50 / p99 (ms) | p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| +0 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 45.3 / 58.6 | +0 | 113.3 | 4.30 s | no |
| +25 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 0 (0.0 s) | 79.5 / 132.7 | +34 | 187.3 | 4.23 s | no |
| +50 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 9 (0.6 s) | 106.4 / 185.9 | +61 | 250.5 | 4.02 s | no |
| +75 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 111 (4.4 s) | 168.4 / 199.5 | +123 | 251.8 | 4.63 s | no |
| +100 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 135 (10.7 s) | 181.2 / 285.7 | +136 | 333.4 | 5.71 s | no |
| +125 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 247 (30.0 s) | 230.6 / 291.7 | +185 | 331.6 | 8.77 s | no |
| +150 ms | 10/10 [69.2%, 100.0%] | 0 (0 / 0) | 339 (53.0 s) | 255.8 / 288.6 | +211 | 339.7 | 10.88 s | no |
| +200 ms | 0/10 [0.0%, 30.8%] | 0 (0 / 0) | 514 (126.5 s) | 298.0 / 336.6 | +253 | 380.7 | 16.44 s | no |

## Test procedure

Procedure: `docs/TEST_PROCEDURE.md`. Conditions this run could check:

- Fresh restart: NO (computer running 2147 min when the test started; the procedure asks for at most 60 min)
- AC power at every session start: yes
- Windows power mode Best performance: yes
- Not checked automatically: other programs and browser tabs closed, laptop maker's utility in performance mode, ventilation. Note in your results whether these were followed.

## Safety and validity

- Unsafe motion, post-stop motion, forbidden contacts and unlatched stops across all levels: 0. Control deadline misses: 0.
- No laptop low-power state was detected during any level.
- "At start" trips happened before the first command of an alignment: the first fresh result must arrive within 400 ms of Align, and it can queue behind a frame already in progress. "Mid-alignment" trips happened after the robot was already moving.
- The injected delay is constant. Real slower hardware also has longer tails, so the tolerated delay is an upper bound for hardware with the same median slowdown.
- Each level has 10 alignments over 5 poses on a fresh worker, so a level where all of them succeeded is limited evidence, not a guarantee (see the confidence intervals).
- SIFT, physical response with the simulated actuator model `default`: after a hold up to 86 ms to standstill and 4.3 mm of camera travel. The zero command itself is immediate; these are simulated values, not real-robot data.
- Learned GPU, physical response with the simulated actuator model `default`: after a hold up to 104 ms to standstill and 3.6 mm of camera travel. The zero command itself is immediate; these are simulated values, not real-robot data.
- Windows power mode at every session start: Best performance
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files changed: accuracy.py, actuator.py, actuator_config.json, analyze_benchmark.py, analyze_comparison.py, analyze_gain_study.py, app.py, binomial_ci.py, camera_robustness.py, realtime.py, realtime_app.py, run_camera_delay_study.py, run_collision_study.py, run_joint_limit_study.py, run_latency_stress.py, run_latency_study.py, run_learned_study.py, run_natural_image_study.py, run_recovery.py, run_search_coverage.py, run_startup_search.py, run_stop_response.py, simulation.py, stop_response.py; runtime versions identical; approved change to `realtime.py`: hold-and-resume default (stale_resume_s = 2 s) plus actuator-model stop reasons and physical-stop reporting; approved change to `simulation.py`: actuator dynamics between command and servo; physical stop measurement; approved change to `actuator.py`: new: actuator dynamics model; approved change to `actuator_config.json`: new: actuator profiles (ideal, default, gentle)
- Python `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`, CUDA GPU NVIDIA GeForce RTX 3050 6GB Laptop GPU.

Per-level data: `margin.csv` (for plotting) and `margin.json`; per-session evidence in `margin-*.json` and `traces/`.
