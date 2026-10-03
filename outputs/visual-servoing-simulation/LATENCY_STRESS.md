# Sustained latency testing

The short transport comparison is a useful regression check, but three
alignments cannot characterize a rare scheduling or GPU stall. The sustained
runner repeats alignment for two minutes per initialized worker, with at least
12 sessions per method. SIFT and Learned GPU alternate between sessions.

## Run the campaign

From the repository root, with the existing Python environment:

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B outputs/visual-servoing-simulation/run_latency_stress.py --output outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN --min-sessions 12 --max-sessions 20 --session-seconds 120 --target-uncached-frames 10000
```

The plan is written before the first trial. A complete round finishes before
the sample-based stopping rule is checked: at least 12 sessions and 10,000
uncached active captures **per method**, or a maximum of 20 sessions. Failures
do not cause early completion or replacement of a session. The completion file
states whether the sample target was actually reached.

Each process uses the preceding experiment's initialization and model warm-up,
then auto-arms at the same offset. After each alignment or watchdog stop, it
observes the stopped system for 1.1 seconds, explicitly resets the offset and
starts another alignment. A stop stays latched until that explicit command.
Sessions end at an attempt boundary after the minimum duration. Model warm-up
and the controller's stopping hold are not extended to improve the scores.

The transport delay remains 50 ms, the capture-age budget 400 ms and the
control deadline 50 ms. The source/runtime manifest verifies the scene,
calibration, reference images, model hashes, controller settings and software
versions against the preceding comparison. Diagnostic additions are recorded
as their own source fingerprint. Source hashes are rechecked at every session
boundary. Never edit application source, configuration or reference files
while a campaign is running.

An interrupted campaign can use `--resume` with the **same arguments and source
fingerprint**. Completed session evidence is retained. An interrupted session
that did not finish and save its trace is not a completed observation; record
the interruption when interpreting a resumed campaign. Use a new directory
for changed conditions, tuning or a new implementation.

## Analyze after the campaign

Do not run the bootstrap, a test suite, another matcher benchmark or a native
profiler concurrently with the statistical campaign.

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B tools/analyze_latency_stress.py outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN
```

The analyzer verifies every raw trace checksum, includes all failed attempts
and retains perception completed after a watchdog stop. It reports:

- Sessions, attempts and successful alignments, including the first attempt in
  each process separately from later attempts using that worker.
- All-active and uncached processing mean, p95, p99, p99.9 and maximum.
- Accepted capture-to-command latency and the maximum held feedback age
  between accepted updates or stopping, including initial waiting.
- Freshness trips, control overruns, skipped acquisition slots, replacements,
  expiry, shutdown accounting and late completions after each kind of stop.
- Whole-session bootstrap intervals and chronological per-session trends.
- Perception and control stages around every watchdog event and the slowest
  recorded frames/commands.
- The safety envelope (worst session per method): clearance above the collision
  margin, distance to the joint limits and peak commanded and measured joint speed,
  over every physics step, with PASS/FAIL against REQ-11 to REQ-13. Sessions recorded
  before 2026-10-01 show "not recorded".

Note that `run_latency_stress.py` refuses to run when a production input differs
from its baseline manifest. Since the actuator model and the safety record changed
`simulation.py`, a new stress campaign needs a current `--baseline`.

Frames within a session are correlated. Resampling individual frames would
make uncertainty look too small, so the bootstrap resamples whole sessions.
Even 10,000 observations supply only about ten samples in the upper 0.1% tail.
A measured maximum is not a worst-case execution-time guarantee. Zero observed
failures cannot prove the true failure probability is zero.

## Diagnostics and their limits

The opt-in probes are inactive in normal interactive use. Sensor frames retain
request queue, acquisition/render, inference, IPC and transport timestamps.
Accepted commands retain controller computation and application durations;
their stage intervals sum to capture-to-command latency. Matcher diagnostics
separate preprocessing, extraction, matching, transfer, geometry and precision
refinement. A cache hit is explicitly identified.

Learned GPU diagnostics record reusable CUDA events on the existing stream.
They read completed events after the existing required device-to-host transfer,
without adding `cuda.synchronize()`. CUDA stream elapsed time includes gaps in
host submission and idle time; it is not a sum of kernel execution durations.
Host and process CPU counters complement the wall-clock measurements.

Control cycles of at least 10 ms retain individual stage timestamps, raw
Windows thread CPU-cycle counters, requested wake-up lateness and native thread
IDs. GC callbacks retain pause intervals. Windows thread CPU time was measured
in 15.625 ms increments on this host; a zero reading for a short interval does
not prove that the thread was off CPU. Raw CPU cycles provide finer evidence
but should not be converted to milliseconds without a frequency model.

Frame/command telemetry is bounded to 8,192 rows per session, loop gaps to
150,000 and slow-cycle/GC records to 2,048 each. Eviction counters are reported.
Worker frames carry only the latest eight GC events as diagnostic context.
Trace compression, JSON reports and GPU status queries occur outside active
sessions. The probes themselves still impose some timing overhead, which must
be acknowledged when comparing them with older runs without probes.

For scheduling and GPU-clock attribution, run a **separate** diagnostic session:

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B tools/trace_latency_diagnostic.py --output outputs/visual-servoing-simulation/results/latency/NEW-DIAGNOSTIC --seconds 120 --wpr
```

This uses installed Windows Performance Recorder CPU context-switch/sampling
events and samples NVIDIA clock, power, temperature and limiting reasons. WPR
requires the relevant Windows recording privileges. The launcher refuses to
touch an existing recording and stops only the recording it started. Omit
`--wpr` for GPU sampling alone. It changes no power, clock, priority or affinity
settings. These profiler-affected results are explicitly excluded from the
statistical campaign.

The preceding 65.9 ms control miss did not record scheduling or CPU counters.
Its exact historical cause cannot be recovered from elapsed timestamps alone;
a similar future miss must be supported by its own evidence.

## Saved evidence

[Sustained campaign report](results/latency/20260922-sustained/REPORT.md).
The manifest records the predeclared plan and source/runtime hashes;
`completion.json` records completion and source integrity, not a blanket
alignment pass. A zero runner exit code means the evidence collection completed
without loss of the measured frame/command/loop telemetry and without a source
change. Inspect watchdog counts and alignment outcomes to judge performance.

Raw per-frame gzip traces stay local under `traces/` and are excluded from Git.
The curated report, plots, diagnostics, session summaries and checksums are
small enough to retain as reviewable evidence. Previous experiments remain
preserved.

## Reused-worker execution candidate, 2026-09-23

The [complete before/after campaign](results/latency/20260923-reuse-after/COMPARISON.md)
keeps the failed 20260922 run as baseline. The new run completed 20 sessions per
method and met both 10,000-uncached-frame targets. Learned's average improved,
but reused-worker degradation and watchdog stops remain; it did not pass the
stability requirement. [Diagnosis and permission limits](results/latency/20260923-reuse-after/DIAGNOSIS.md).

To compare against a sustained manifest, add
`--baseline outputs/visual-servoing-simulation/results/latency/20260922-sustained/manifest.json`
to the campaign command. The runner checks the same fixed inputs and records
the exact baseline hash. It now captures Windows AC/battery status at session
boundaries. These snapshots do not provide continuous power telemetry.

CUDA Learned initialization now captures nine fixed-shape transformer graphs;
the existing one-image model warm-up remains unchanged. All graph setup runs
before arming. No extra per-alignment warm-up or process restart was added.

After analysis, `tools/compare_worker_reuse.py BEFORE AFTER` produces the paired
tables and audit. `tools/summarize_worker_probes.py LATENCY_DIRECTORY OUTPUT_JSON`
and `tools/analyze_replay_clocks.py LATENCY_DIRECTORY OUTPUT_DIRECTORY` summarize
the separate diagnostics. Never run these analyses alongside active measurement.

## Native scheduling candidate, 2026-09-25

The [completed 40-session comparison](results/latency/20260925-native-scheduling-after/COMPARISON.md)
shows fewer control misses but continuing Learned degradation and more freshness
trips. [Native scheduler/GPU evidence and remaining GC pauses](results/latency/20260925-native-scheduling-after/DIAGNOSIS.md)
supersede the earlier native-recording permission limitation. Safety stayed
latched, but the sustained performance criterion failed.

`tools/record_reuse_native.py --output <new results/latency directory>` records
a separate 120-second reused-worker diagnostic using `tools/reuse_native.wprp`.
It requires Windows recording privileges, refuses an existing recording, and
saves the bounded circular trace at the first control miss or after 100 seconds.
Save I/O and profiler overhead exclude this run from validation statistics.
Earlier trace events can be overwritten even when the lost-event count is zero.
The system-wide ETL remains local. `tools/read_gpu_profile.py EXECUTABLE --output
OUTPUT_JSON` reads the NVIDIA application profile without changing it; run it
outside a timing campaign.

The comparison command is `tools/compare_native_scheduling.py BEFORE AFTER
--tests TEST_LOG`. Its audit checks the specific scheduling-only source delta,
unchanged fixed inputs, raw checksums, safety and recorded scheduling API readback.
