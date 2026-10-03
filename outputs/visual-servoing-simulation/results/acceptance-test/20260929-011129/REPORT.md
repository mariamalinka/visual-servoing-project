# Acceptance test: PASS

Profile: standard. Fresh-worker repeatability on 5 poses, then a 12-minute sustained run with one reused worker per method.

Freshness watchdog response: **hold and resume** (`stale_resume_s` = 2 s). At the same 400 ms limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and ends as `stale_camera` only if none arrives within 2 s. Holds are allowed only within the pause budget in the criteria; an alignment the watchdog ends fails.

| Metric | SIFT | Learned GPU |
|---|---:|---:|
| Verdict | **PASS** | **PASS** |
| Alignment success | 150/150 (100.0%) | 147/147 (100.0%) |
|   fresh-worker poses | 15/15 | 15/15 |
|   sustained reused worker | 135/135 in 12.1 min | 132/132 in 12.1 min |
| Processing p95 / p99 / max (ms) | 64.9 / 72.3 / 97.3 | 58.9 / 67.1 / 97.0 |
| Capture-to-command p99 / max (ms) | 129.1 / 154.2 | 121.4 / 152.1 |
| Freshness watchdog trips | 0 | 0 |
| Paused time (share of alignment time) | 0.0 s (0.0%) | 0.0 s (0.0%) |
| Control deadline misses | 0 | 0 |
| First-alignment processing p95 / p99 / max | 56.2 / 61.1 / 65.5 | 45.1 / 48.7 / 68.2 |
| Later-alignment processing p95 / p99 / max | 65.5 / 73.1 / 97.3 | 59.6 / 67.6 / 97.0 |
| Later / first processing p99 | 1.20x | 1.39x |
| First / later capture-to-command p99 | 116.6 / 130.1 | 103.5 / 122.0 |
| Time to converge, first / later (median) | 4.32 s (n=6) / 4.19 s (n=134) | 4.42 s (n=6) / 4.26 s (n=131) |
| Sustained drift, last / first window p99 | 0.81x | 1.18x |
| Sustained worst window p99 (window start) | 75.5 ms (minute 8) | 71.0 ms (minute 10) |
| Unsafe / post-stop motion ticks | 0 / 0 | 0 / 0 |
| Motion ticks during a watchdog hold | 0 | 0 |
| Forbidden contacts / unlatched stops | 0 / 0 | 0 / 0 |
| Physical error max (mm / deg) | 0.463 / 0.097 | 0.606 / 0.125 |
| Stop reasons | converged 150 | converged 147 |

Hardware telemetry (diagnostic, not gated). A sample counts as clock-limited when nvidia-smi reports a software power cap, software/hardware thermal slowdown, hardware slowdown or power brake.

| Hardware | SIFT | Learned GPU |
|---|---:|---:|
| GPU clock-limit samples, s (SW power / SW thermal / HW) | 0 / 0 / 0 | 0 / 0 / 0 |
| Failed alignments during GPU clock limit | 0/0 | 0/0 |
| GPU busy below 400 MHz, s / busy s | 0 / 0 | 7 / 598 |
| Failed alignments: GPU below 400 MHz / CPU below 85% max | 0/0 / 0/0 | 0/0 / 0/0 |
| Slow frames (>= 150 ms) during GPU clock limit | 0/0 | 0/0 |
| GPU busy SM clock min / max temperature | n/a / 56 C | 315 MHz / 58 C |
| Hottest thermal zone / min passive limit | 85 C / 100% | 82 C / 100% |
| CPU frequency min / median (% of max) | 66% / 97% | 57% / 94% |

**SIFT failed criteria:** none
**Learned GPU failed criteria:** none

## Test procedure

Procedure: `docs/TEST_PROCEDURE.md`. Conditions this run could check:

- Fresh restart: NO (computer running 265 min when the test started; the procedure asks for at most 60 min)
- AC power at every session start: yes
- Windows power mode Best performance: yes
- Not checked automatically: other programs and browser tabs closed, laptop maker's utility in performance mode, ventilation. Note in your results whether these were followed.

## Conditions

Freshness watchdog 400 ms, control deadline 50 ms, transport delay 50 ms (production values, asserted before the run). Protected production files match the validated baseline except reviewed changes listed in `tools/approved_changes.json`: `realtime.py` (freshness watchdog hold-and-resume, default on (stale_resume_s = 2 s; 0 = original stop)).
Gates: 100% alignment; processing p95/p99/max <= 150/175/250 ms; capture-to-command p99/max <= 250/400 ms; 0 alignments ended by the watchdog, paused <= 5% of alignment time, 0 deadline misses, 0 unsafe/post-stop motion; <= 2 mm / 1 deg; later/first p99 <= 1.5x.

- Python: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe` (venv `C:\Users\marys\Documents\Codex\2026-09-06\visual-servoing-project\outputs\visual-servoing-simulation\.venv`)
- Sensor worker executable: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`
- CUDA GPU: NVIDIA GeForce RTX 3050 6GB Laptop GPU (cuda:0, UUID e148b270-8b60-34ee-4cc6-7ac0cdbc9758, driver 577.02)
- Versions: Python 3.12.14, torch 2.13.0+cu126, CUDA 12.6, cuDNN 91002, mujoco 3.12.0, OpenCV 5.0.0.93, numpy 2.5.2, Windows-11-10.0.26200-SP0
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files changed: app.py, realtime.py, realtime_app.py, run_latency_stress.py, run_latency_study.py; runtime versions identical; approved change to `realtime.py`: freshness watchdog hold-and-resume, default on (stale_resume_s = 2 s; 0 = original stop)
- Windows power mode at every session start: Best performance
- Started 2026-09-28T23:11:29.085469+00:00; complete: True

Processing = render finished to perception result, every uncached active frame including failed/late ones. Capture-to-command covers accepted commands. First = first alignment of each worker (5 fresh workers + the sustained worker); later = every subsequent alignment of the sustained reused worker. Details: `verdict.json`, `manifest.json`, `test-config.json`; raw per-frame evidence under `traces/`.
