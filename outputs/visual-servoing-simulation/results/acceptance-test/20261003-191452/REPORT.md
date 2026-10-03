# Acceptance test: PASS

Profile: standard. Fresh-worker repeatability on 5 poses, then a 12-minute sustained run with one reused worker per method.

Freshness watchdog response: **hold and resume** (`stale_resume_s` = 2 s). At the same 400 ms limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and ends as `stale_camera` only if none arrives within 2 s. Holds are allowed only within the pause budget in the criteria; an alignment the watchdog ends fails.

| Metric | SIFT | Learned GPU |
|---|---:|---:|
| Verdict | **PASS** | **PASS** |
| Alignment success | 148/148 (100.0%) | 146/146 (100.0%) |
|   95% confidence interval, exact Clopper-Pearson | [97.5%, 100.0%] | [97.5%, 100.0%] |
|   success rate demonstrated | yes: lower bound 97.5% >= 95.0% | yes: lower bound 97.5% >= 95.0% |
|   fresh-worker poses | 15/15 [78.2%, 100.0%] | 15/15 [78.2%, 100.0%] |
|   sustained reused worker | 133/133 [97.3%, 100.0%] in 12.0 min | 131/131 [97.2%, 100.0%] in 12.0 min |
| Processing p95 / p99 / max (ms) | 59.2 / 65.8 / 78.4 | 56.9 / 62.8 / 92.1 |
| Capture-to-command p99 / max (ms) | 122.6 / 150.0 | 117.9 / 150.8 |
| Freshness watchdog trips | 0 | 0 |
| Paused time (share of alignment time) | 0.0 s (0.0%) | 0.0 s (0.0%) |
| Control deadline misses | 0 | 0 |
| First-alignment processing p95 / p99 / max | 60.6 / 67.4 / 71.1 | 46.1 / 50.7 / 59.2 |
| Later-alignment processing p95 / p99 / max | 59.0 / 65.7 / 78.4 | 57.4 / 63.0 / 92.1 |
| Later / first processing p99 | 0.98x | 1.24x |
| First / later capture-to-command p99 | 124.5 / 122.5 | 104.3 / 118.4 |
| Time to converge, first / later (median) | 4.28 s (n=6) / 4.25 s (n=132) | 4.35 s (n=6) / 4.32 s (n=130) |
| Sustained drift, last / first window p99 | 0.97x | 1.07x |
| Sustained worst window p99 (window start) | 66.9 ms (minute 2) | 67.5 ms (minute 8) |
| Unsafe / post-stop motion ticks | 0 / 0 | 0 / 0 |
| Motion ticks during a watchdog hold | 0 | 0 |
| Actuator model (simulated) | default | default |
| Physical stop after a stop: max time / camera travel | none while moving | none while moving |
| Physical stop after a hold: max time / camera travel | none while moving | none while moving |
| Forbidden contacts / unlatched stops | 0 / 0 | 0 / 0 |
| Physical error max (mm / deg) | 0.463 / 0.097 | 0.607 / 0.125 |
| Stop reasons | converged 148 | converged 146 |

**Success rates are estimates.** The interval is the two-sided 95% exact Clopper-Pearson confidence interval for the true success rate (docs/STATISTICS.md).
- SIFT: All 148 succeeded, but that does not prove 100% reliability: with 95% confidence the true success rate is at least 97.5%.
- Learned GPU: All 146 succeeded, but that does not prove 100% reliability: with 95% confidence the true success rate is at least 97.5%.
- A success-rate claim of at least 95.0% passes only if the lower bound reaches it; that needs at least 72 alignments with no failure.

## Safety envelope

Extremes over every 2 ms physics step of every session (worst session per method). Clearance is the collision guard's own distance check minus the required margin (12 mm to the environment, 6 mm between robot parts); joint margin is the distance to the `scene.xml` range. A FAIL fails the test. Per-session values: `safety.csv`. Simulation values, not real-robot measurements.

Recorded in 12 of 12 sessions.

| Requirement | Safety metric (every physics step) | Limit | SIFT | Learned GPU |
|---|---|---|---|---|
| REQ-11 | Minimum clearance to the environment above its 12 mm margin | >= 0 mm | 3.00 mm **PASS** (pedestal / upper_arm) | 3.00 mm **PASS** (pedestal / upper_arm) |
| REQ-11 | Minimum clearance between robot parts above its 6 mm margin | >= 0 mm | 9.00 mm **PASS** (wrist_roll_link / tool_link) | 9.00 mm **PASS** (wrist_roll_link / tool_link) |
| REQ-12 | Minimum distance to a joint limit | >= 0 rad | 0.995 rad **PASS** (joint 1) | 0.995 rad **PASS** (joint 1) |
| REQ-13 | Peak commanded joint speed | <= 0.6 rad/s | 0.087 rad/s **PASS** | 0.243 rad/s **PASS** |
| REQ-13 | Peak measured joint speed | <= 0.6 rad/s | 0.085 rad/s **PASS** (joint 2) | 0.201 rad/s **PASS** (joint 3) |

Hardware telemetry (diagnostic, not gated). A sample counts as clock-limited when nvidia-smi reports a software power cap, software/hardware thermal slowdown, hardware slowdown or power brake.

| Hardware | SIFT | Learned GPU |
|---|---:|---:|
| GPU clock-limit samples, s (SW power / SW thermal / HW) | 0 / 0 / 0 | 0 / 0 / 0 |
| Failed alignments during GPU clock limit | 0/0 | 0/0 |
| GPU busy below 400 MHz, s / busy s | 0 / 0 | 3 / 586 |
| Failed alignments: GPU below 400 MHz / CPU below 85% max | 0/0 / 0/0 | 0/0 / 0/0 |
| Slow frames (>= 150 ms) during GPU clock limit | 0/0 | 0/0 |
| GPU busy SM clock min / max temperature | n/a / 55 C | 322 MHz / 57 C |
| Hottest thermal zone / min passive limit | 82 C / 100% | 79 C / 100% |
| CPU frequency min / median (% of max) | 68% / 94% | 43% / 92% |

**SIFT failed criteria:** none
**Learned GPU failed criteria:** none

## Test procedure

Procedure: `docs/TEST_PROCEDURE.md`. Conditions this run could check:

- Fresh restart: yes (computer running 2 min when the test started; the procedure asks for at most 60 min)
- AC power at every session start: yes
- Windows power mode Best performance: yes
- Not checked automatically: other programs and browser tabs closed, laptop maker's utility in performance mode, ventilation. Note in your results whether these were followed.

## Conditions

Freshness watchdog 400 ms, control deadline 50 ms, transport delay 50 ms (production values, asserted before the run). Protected production files match the validated baseline except reviewed changes listed in `tools/approved_changes.json`: `realtime.py` (hold-and-resume default (stale_resume_s = 2 s), actuator-model stop reporting, and sensor results received on a separate thread); `simulation.py` (actuator dynamics between command and servo; physical stop measurement); `actuator.py` (new: actuator dynamics model); `actuator_config.json` (new: actuator profiles (ideal, default, gentle)); `collision.py` (keep the smallest clearance slack of each check for the safety record).
Gates: 95% lower confidence bound on alignment success >= 95.0% and at most 0 failed alignment(s); processing p95/p99/max <= 150/175/250 ms; capture-to-command p99/max <= 250/400 ms; 0 alignments ended by the watchdog, paused <= 5% of alignment time, 0 deadline misses, 0 unsafe/post-stop motion; clearance, joint-limit margin and joint speed within REQ-11 to REQ-13 (when recorded); <= 2 mm / 1 deg; later/first p99 <= 1.5x.

- Python: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe` (venv `C:\Users\marys\Documents\Codex\2026-09-06\visual-servoing-project\outputs\visual-servoing-simulation\.venv`)
- Sensor worker executable: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`
- CUDA GPU: NVIDIA GeForce RTX 3050 6GB Laptop GPU (cuda:0, UUID e148b270-8b60-34ee-4cc6-7ac0cdbc9758, driver 577.02)
- Versions: Python 3.12.14, torch 2.13.0+cu126, CUDA 12.6, cuDNN 91002, mujoco 3.12.0, OpenCV 5.0.0.93, numpy 2.5.2, Windows-11-10.0.26200-SP0
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files changed: accuracy.py, actuator.py, actuator_config.json, analyze_benchmark.py, analyze_comparison.py, analyze_gain_study.py, app.py, binomial_ci.py, camera_robustness.py, collision.py, fault_injection.py, latency_stress.py, realtime.py, realtime_app.py, run_camera_delay_study.py, run_collision_study.py, run_fault_campaign.py, run_joint_limit_study.py, run_latency_stress.py, run_latency_study.py, run_learned_study.py, run_natural_image_study.py, run_recovery.py, run_search_coverage.py, run_startup_search.py, run_stop_response.py, safety_metrics.py, simulation.py, stop_response.py; runtime versions identical; approved change to `realtime.py`: hold-and-resume default (stale_resume_s = 2 s), actuator-model stop reporting, and sensor results received on a separate thread; approved change to `simulation.py`: actuator dynamics between command and servo; physical stop measurement; approved change to `actuator.py`: new: actuator dynamics model; approved change to `actuator_config.json`: new: actuator profiles (ideal, default, gentle); approved change to `collision.py`: keep the smallest clearance slack of each check for the safety record
- Windows power mode at every session start: Best performance
- Started 2026-10-03T17:14:52.377296+00:00; complete: True

Processing = render finished to perception result, every uncached active frame including failed/late ones. Capture-to-command covers accepted commands. First = first alignment of each worker (5 fresh workers + the sustained worker); later = every subsequent alignment of the sustained reused worker. Details: `verdict.json`, `manifest.json`, `test-config.json`; raw per-frame evidence under `traces/`.
