# Acceptance test: SIFT and Learned GPU

Follow the [test procedure](TEST_PROCEDURE.md) before every run: restart, close other programs and tabs, AC power, Best performance.

One command, from the repository root, on AC power, with nothing else using the GPU:

```powershell
.\acceptance-test.cmd
```

It takes about 30 minutes and writes a new folder
`outputs/visual-servoing-simulation/results/acceptance-test/<date-time>/`. The exit code
is **0 for PASS**, **1 for FAIL** and **2 for INVALID or INCOMPLETE**. `REPORT.md` in that
folder is the short PASS/FAIL report.

Options: `--smoke` runs a 4-minute harness check (status `SMOKE_ONLY`, never a pass).
`--method sift` or `--method learned` runs one method for diagnosis (status `PARTIAL`
at best). `--no-system-monitor` turns off the CPU/thermal sampler. `--report-only <folder>` recomputes the report from saved evidence after
verifying the raw-trace checksums. `--output <new folder>` chooses the folder.

`--stale-resume-ms 2000` runs with the watchdog set to **hold and resume** instead of
stop (see `outputs/visual-servoing-simulation/REALTIME_CONTROL.md`). The criteria do not
change: every hold still counts as a freshness watchdog trip, so a PASS still requires
0. The report shows how many trips ended an alignment and how many were held and
resumed, and any motion during a hold fails the run as a safety violation.

## Long run (under 1 hour)

```powershell
.\acceptance-test.cmd --long
```

The long profile (`tools/acceptance_test_long.json`) keeps everything in the standard
test the same except the sustained run:

- The same poses, fresh-worker phase, runtime limits and PASS criteria.
- Each method's reused worker runs for **26 minutes** instead of 12, and needs at
  least 20 attempts per pose.
- With start-up and the fresh-worker phase, that is about 28–29 minutes per method and
  about 57 minutes in total.
- Results go to `results/acceptance-test/<date-time>-long/`. The report also shows the
  worst 2-minute processing window, so a slowdown late in the run stands out.

`--long --smoke` checks the setup in about 4 minutes. The two-hour
`tools/run_acceptance_campaign.py` remains available for even longer runs.

## What runs

The same fixed plan runs every time (`tools/acceptance_test.json`):

1. **Fresh-worker repeatability, SIFT then Learned GPU.** For each of the five
   starting poses, a new sensor worker starts, performs the existing one-image
   warm-up, auto-arms and aligns. Offset/Align then repeats that pose three times.
2. **Sustained reused worker, SIFT then Learned GPU.** One sensor worker (for Learned,
   one CUDA context) is kept for 12 minutes, cycling through the five poses with
   Offset/Align. The worker is never restarted, and no extra warm-up is added.

| Pose | Joint offsets from home (degrees) |
|---|---|
| 0 | 3, -3, 4, 3, -2, 2 (production default) |
| 1 | 1.5, -1.5, 2, 1.5, -1, 1 |
| 2 | -3, 3, -4, -3, 2, -2 |
| 3 | -2, -3, 3, -2, 2, 3 |
| 4 | 2, 3, -3, 2, -2, -3 |

Every pose is checked against the joint limits and collision clearance before the
run. All five start with the photograph in view.

After every stop, the existing 1.1-second post-stop observation checks that the stop
stayed latched with zero command. Then the next Offset/Align is issued. Safe watchdog
stops count as failures and the test continues. Unsafe motion, post-stop motion, a
forbidden contact, an unlatched stop or a runtime error cancels the remaining sessions.

## What stays unchanged

The harness imports the production `RealtimeSession` as it is. The controller,
calibration, scene, reference images, perception settings, worker warm-up, 50 ms
transport delay, **400 ms freshness watchdog** and **50 ms control deadline** are
unchanged. The runtime values are asserted before the run and cannot be overridden.

The harness changes three things, and only in its own process:

- It chooses which declared offset an explicit Offset reset uses.
- It copies the runtime's bounded telemetry buffers at the controller's normal
  inactive publish, after each stop.
- It writes that telemetry to disk while the robot is stopped.

No file I/O runs in the active control loop, and no queue is made unbounded.
By default the runtime settings are exactly production; `--stale-resume-ms` adds only
`stale_resume_s`.

The controller, calibration, scene, safety and transport files and their
configuration are compared with the validated fingerprint in
`results/latency/20260926-base-executable-settings/manifest.json`. If any of them
differs, the run is **INVALID** and does not start. The only exception is a reviewed
change listed in `tools/approved_changes.json`, pinned to both the baseline hash and
the new hash (currently the opt-in watchdog hold-and-resume in `realtime.py`, which
is off by default). The report names every approved change it relied on. Perception and model code may
differ; every file hash is recorded. Source, runner and configuration are rechecked
before every session. A change mid-run makes the result INVALID. Use `--baseline` to
certify against a newer validated manifest.

## PASS criteria

Each method must meet every criterion. Overall PASS requires both methods to pass
and the whole plan to complete.

| Criterion | Limit |
|---|---:|
| Alignment success (fresh and sustained) | 100% |
| Processing p95 / p99 / max | ≤ 150 / 175 / 250 ms |
| Capture-to-command p99 / max | ≤ 250 / 400 ms |
| Freshness watchdog trips / control deadline misses | 0 / 0 |
| Unsafe motion / post-stop motion / forbidden contacts / unlatched stops | 0 / 0 / 0 / 0 |
| Motion during a watchdog hold (hold-and-resume runs only) | 0 |
| Camera and tool position / orientation error, every attempt | ≤ 2 mm / 1° |
| Later-alignment processing p99 ÷ first-alignment processing p99 | ≤ 1.5× |
| Sustained run: duration, one worker PID, attempts per pose | ≥ 720 s, 1, ≥ 10 |
| Telemetry loss, missing endpoints, runtime errors | 0 |

These limits are the ones already predeclared in `tools/acceptance_campaign.json`.

**Processing** runs from render completion to the perception result. It covers every
uncached active frame, including failed and late ones. **Capture-to-command** covers
accepted commands.

**First** is the first alignment of every worker (five fresh workers and the
sustained worker). **Later** is every subsequent alignment of the sustained reused
worker. This measures the reused-worker regression directly against the fresh-worker
baseline taken in the same run.

The report also includes these figures, which are not gated:

- Time to converge, first and later.
- Capture-to-command p99, first and later.
- Processing p99 in 2-minute windows through the sustained run (drift).
- Stop reasons.
- Physical error.

## Recorded with every run

- `manifest.json` holds the test configuration and its hash, and the runner hash.
- It records the launch executable, the actual Windows executable, the base
  interpreter, the multiprocessing spawn executable and the venv. It also records
  the sensor worker's executable, read from its process.
- It records the CUDA device selected by torch, matched to `nvidia-smi` by UUID.
  It also stores the driver, `CUDA_VISIBLE_DEVICES`, and the Windows
  `UserGpuPreferences` entries for the executables. These entries are read only.
- It records versions: Python, OS, numpy, OpenCV, MuJoCo, torch, CUDA, cuDNN,
  LightGlue and kornia.
- It records the source/config/model fingerprint, its comparison with the validated
  baseline, model weight hashes and AC status at every session boundary.
- `test-config.json` is an exact copy of the configuration used.
- `REPORT.md` is the concise PASS/FAIL report. `verdict.json` holds all statistics
  and failed criteria. One summary JSON is written per session.
- `traces/` holds raw per-frame telemetry, attempt endpoints and session endings. It
  also holds `gpu.csv`, `system.csv` and `nvidia-smi-start.txt`/`-end.txt` (see below).
  It stays local.

## Hardware telemetry

This is diagnostic evidence for explaining a failure. It is not a PASS/FAIL criterion.

- **`gpu.csv`:** `nvidia-smi` once a second. It records SM/memory clocks, utilisation,
  temperature, power draw and enforced power limit, and the clock-event-reason bitmask.
  The report decodes the bitmask into software power cap, software thermal slowdown,
  hardware slowdown, hardware thermal slowdown and power brake.
- **`system.csv`:** Windows thermal zones (temperature, passive-cooling limit, throttle
  reasons) and processor frequency as % of maximum, once a second. They come from the
  Performance Monitor counters through PowerShell, and need no administrator rights.
  Laptops expose different zones; some expose none, and the report then says so.
- **`nvidia-smi-start.txt` / `-end.txt`:** `nvidia-smi -q` readouts of performance,
  power, temperature and clock limits before and after the sessions. `manifest.json`
  also records whether the ACPI temperature class is readable.

Each controller session records a clock anchor, so frames, stops and hardware samples
share one timeline. For each method the report gives:

- Seconds of each clock-limit reason.
- How many failed alignments and slow frames (≥ 150 ms) happened while the GPU was
  clock-limited, within 1.5 s of a sample.
- Minimum busy SM clock and maximum GPU temperature.
- Hottest thermal zone, lowest passive limit and lowest CPU frequency.
- Every clock-limit episode, timed from when that session's worker became ready.

The report also marks **low-performance periods**: 30-second windows where the GPU was busy
below 400 MHz or the CPU ran below 85% of its maximum frequency. It counts how many
failed alignments fell inside them. The Windows power mode (Settings → Power mode) is
recorded at every session start.

The samplers are separate processes and change no settings. They are tied to the test
process through a Windows job object, so they stop even if the test window is closed.
When a run starts, it also stops any samplers that an earlier, killed run left behind,
and records this in the manifest. The PowerShell sampler
adds a little CPU load; use `--no-system-monitor` to measure without it. Earlier runs
can be re-analysed with `--report-only`: GPU correlation works for them, but they
have no `system.csv`.

## Interpreting a result

A PASS supports repeatability under the recorded conditions only. It is not a
worst-case timing guarantee. The sustained run is 12 minutes. The longer
`tools/run_acceptance_campaign.py` (2 × 30 minutes per method) remains available
for longer-duration evidence.

Keep the laptop on AC and close other GPU applications. Results on this laptop have
depended on system GPU and power settings: see the
[GPU-settings comparison](../outputs/visual-servoing-simulation/results/latency/20260926-base-executable-settings/SETTINGS_COMPARISON.md).
The manifest records those settings so that different runs can be compared.

Unit tests for the gates and the report run without the simulator:

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B -m unittest discover -s tools -p test_acceptance_test.py -v
```
