# Acceptance test (long): FAIL

Profile: long. Fresh-worker repeatability on 5 poses, then a 26-minute sustained run with one reused worker per method.

| Metric | SIFT | Learned GPU |
|---|---:|---:|
| Verdict | **FAIL** | **FAIL** |
| Alignment success | 298/324 (92.0%) | 81/511 (15.9%) |
|   fresh-worker poses | 15/15 | 15/15 |
|   sustained reused worker | 283/309 in 26.0 min | 66/496 in 26.0 min |
| Processing p95 / p99 / max (ms) | 107.7 / 139.2 / 343.4 | 214.4 / 292.7 / 365.8 |
| Capture-to-command p99 / max (ms) | 194.6 / 297.3 | 258.2 / 365.6 |
| Freshness watchdog trips | 26 | 430 |
| Control deadline misses | 0 | 0 |
| First-alignment processing p95 / p99 / max | 94.8 / 125.7 / 147.8 | 107.6 / 128.7 / 148.5 |
| Later-alignment processing p95 / p99 / max | 108.0 / 139.7 / 343.4 | 223.2 / 294.4 / 365.8 |
| Later / first processing p99 | 1.11x | 2.29x |
| First / later capture-to-command p99 | 181.5 / 194.9 | 183.4 / 259.2 |
| Time to converge, first / later (median) | 4.07 s (n=6) / 4.04 s (n=282) | 3.98 s (n=6) / 4.22 s (n=65) |
| Sustained drift, last / first window p99 | 1.64x | 1.05x |
| Sustained worst window p99 (window start) | 189.6 ms (minute 20) | 328.8 ms (minute 20) |
| Unsafe / post-stop motion ticks | 0 / 0 | 0 / 0 |
| Forbidden contacts / unlatched stops | 0 / 0 | 0 / 0 |
| Physical error max (mm / deg) | 2.965 / 0.397 | 36.445 / 4.591 |
| Stop reasons | converged 298, stale_camera 26 | stale_camera 430, converged 81 |

Hardware telemetry (diagnostic, not gated). A sample counts as clock-limited when nvidia-smi reports a software power cap, software/hardware thermal slowdown, hardware slowdown or power brake.

| Hardware | SIFT | Learned GPU |
|---|---:|---:|
| GPU clock-limit samples, s (SW power / SW thermal / HW) | 50 / 50 / 0 | 107 / 89 / 0 |
| Failed alignments during GPU clock limit | 0/26 | 12/430 |
| Slow frames (>= 150 ms) during GPU clock limit | 0/76 | 34/1210 |
| GPU busy SM clock min / max temperature | 52 MHz / 61 C | 52 MHz / 65 C |
| Hottest thermal zone / min passive limit | 88 C / 100% | 87 C / 100% |
| CPU frequency min (% of max) | 50% | 49% |

GPU clock-limit episodes of 2 s or more (time after the session's worker was ready; negative = during start-up):

- SIFT `repeatability-natural-pose1` at 0 s for 10 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 4.0%
- SIFT `repeatability-natural-pose2` at 0 s for 10 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 3.0%
- SIFT `repeatability-natural-pose3` at 0 s for 10 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 4.0%
- SIFT `repeatability-natural-pose4` at 0 s for 10 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 4.0%
- SIFT `sustained-natural` at 0 s for 10 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 6.0%
- Learned GPU `repeatability-learned-pose0` at -5 s for 22 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 73.0%
- Learned GPU `repeatability-learned-pose1` at -5 s for 5 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 0.0%
- Learned GPU `repeatability-learned-pose4` at 12 s for 5 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 990.0 MHz, utilisation up to 48.0%
- Learned GPU `sustained-learned` at 103 s for 2 s: sw_power_cap; SM clock down to 238.0 MHz, utilisation up to 89.0%
- Learned GPU `sustained-learned` at 109 s for 5 s: sw_power_cap; SM clock down to 221.0 MHz, utilisation up to 94.0%
- Learned GPU `sustained-learned` at 245 s for 58 s: sw_power_cap, sw_thermal_slowdown; SM clock down to 390.0 MHz, utilisation up to 75.0%
- Learned GPU `sustained-learned` at 1526 s for 4 s: sw_power_cap; SM clock down to 88.0 MHz, utilisation up to 62.0%

**SIFT failed criteria:** alignment success 298/324; processing max 343.4 ms > 250; freshness_trips 26; position error max 2.965 mm
**Learned GPU failed criteria:** alignment success 81/511; processing p95 214.4 ms > 150; processing p99 292.7 ms > 175; processing max 365.8 ms > 250; capture p99 258.2 ms > 250; freshness_trips 430; position error max 36.445 mm; orientation error max 4.591 deg; later/first processing p99 2.29x

## Conditions

Freshness watchdog 400 ms, control deadline 50 ms, transport delay 50 ms (production values, asserted before the run). Production files were not modified.
Gates: 100% alignment; processing p95/p99/max <= 150/175/250 ms; capture-to-command p99/max <= 250/400 ms; 0 freshness trips, 0 deadline misses, 0 unsafe/post-stop motion; <= 2 mm / 1 deg; later/first p99 <= 1.5x.

- Python: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe` (venv `C:\Users\marys\Documents\Codex\2026-09-06\visual-servoing-project\outputs\visual-servoing-simulation\.venv`)
- Sensor worker executable: `C:\Users\marys\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe`
- CUDA GPU: NVIDIA GeForce RTX 3050 6GB Laptop GPU (cuda:0, UUID e148b270-8b60-34ee-4cc6-7ac0cdbc9758, driver 577.02)
- Versions: Python 3.12.14, torch 2.13.0+cu126, CUDA 12.6, cuDNN 91002, mujoco 3.12.0, OpenCV 5.0.0.93, numpy 2.5.2, Windows-11-10.0.26200-SP0
- Versus validated baseline `20260926-base-executable-settings`: source/config/model files identical; runtime versions identical
- Started 2026-09-28T15:35:56.138184+00:00; complete: True

Processing = render finished to perception result, every uncached active frame including failed/late ones. Capture-to-command covers accepted commands. First = first alignment of each worker (5 fresh workers + the sustained worker); later = every subsequent alignment of the sustained reused worker. Details: `verdict.json`, `manifest.json`, `test-config.json`; raw per-frame evidence under `traces/`.
