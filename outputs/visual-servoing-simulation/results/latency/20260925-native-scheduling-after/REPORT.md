# Sustained perception latency validation

The same starting offset, 50 ms added transport, 400 ms capture-age budget, 50 ms control-loop deadline, controller, calibration, scene, model settings and dependency versions as the preceding three-run comparison. Each process uses the same initial model warm-up. Explicit offset/align cycles then reuse that worker; the stopping criterion and 1.1 s post-stop observation window are unchanged.

Predeclared stopping rule: at least 12 sessions and 10,000 uncached active frames per method, or 20 sessions maximum. Each session runs for at least 120 s after ready and ends at an attempt boundary. Modes alternate in each complete round; all failures remain included.

| Method | Sessions | Aligned / attempts | First attempt in each process | Subsequent attempts | Active / ready minutes | Uncached active frames |
|---|---:|---:|---:|---:|---:|---:|
| SIFT | 20 | 444/497 | 17/20 | 427/477 | 31.4 / 41.1 | 15,531 |
| Learned GPU | 20 | 110/812 | 18/20 | 92/792 | 24.9 / 41.0 | 10,327 |

Observed 755 unsuccessful alignment attempts, 2 control deadline misses and 753 freshness watchdog trips. Sample target met: True. Source unchanged throughout the campaign: True.

## Processing latency

| Method | Cohort | Samples | Mean / p95 / p99 / p99.9 / maximum (ms) |
|---|---|---:|---|
| SIFT | All active captures | 19,272 | 69.6 / 122.3 / 151.3 / 203.2 / 284.4 |
| SIFT | Uncached active captures | 15,531 | 86.3 / 125.7 / 156.3 / 214.6 / 284.4 |
| Learned GPU | All active captures | 13,954 | 75.8 / 213.6 / 292.6 / 334.5 / 355.2 |
| Learned GPU | Uncached active captures | 10,327 | 102.3 / 237.9 / 301.0 / 336.9 / 355.2 |

Processing is the complete wall interval from image acquisition completion to the finished perception result. It includes refinement, host/device waits and preemption. Active captures remain counted when processing completes after a stop. Idle captures are excluded. Cache hits are shown in the all-active distribution and excluded explicitly in the second distribution.

## Capture to command and held feedback age

| Method | Accepted commands | Mean / p95 / p99 / maximum (ms) | Maximum held feedback age bound (ms) |
|---|---:|---|---:|
| SIFT | 18,731 | 129.1 / 181.2 / 208.7 / 288.7 | 408.7 |
| Learned GPU | 12,914 | 125.2 / 235.4 / 268.5 / 378.1 | 409.6 |

Capture-to-command uses actual accepted command application timestamps. Rejected results have no command timestamp; their processing remains in the preceding table and their capture-to-delivery distribution is in summary.json. The held-age bound uses the preceding capture through the next application or stop, and includes initial waiting after arming. It is evaluated separately for every attempt and includes the interval between accepted updates.

## Watchdogs, bounded work and completeness

| Counter | SIFT | Learned GPU |
|---|---:|---:|
| Acquisition slots | 71352 | 70802 |
| Captured requests | 35566 | 33643 |
| Skipped busy acquisition slots | 35786 | 37159 |
| Pending request replacements | 0 | 0 |
| Transport results replaced | 1 | 3 |
| Result mailbox drops | 0 | 0 |
| Pending transport entries cancelled at stops | 1247 | 1787 |
| Obsolete generation results | 385 | 645 |
| Expired requests before inference | 0 | 0 |
| Expired results | 0 | 7 |
| Completed but unreceived results | 0 | 0 |
| Requests never started | 0 | 0 |
| Control deadline misses | 0 | 2 |
| Unsafe motion ticks | 0 | 0 |
| Motion ticks after stop | 0 | 0 |
| Forbidden contacts | 0 | 0 |
| Freshness watchdog trips | 53 | 700 |
| Late completions after freshness stop | 44 | 565 |
| Late completions after control stop | 0 | 0 |
| All stops stayed latched | True | True |
| Frame/command/loop telemetry complete | True | True |
| Extended diagnostics enabled | True | True |
| Control-cycle/parent-GC buffers complete | True | True |

Counters cover entire sessions, including post-stop observation and draining. Skipped slots do not acquire an image. Pending cancellation at a stop and obsolete generations after an explicit reset are intentional cleanup, not evidence of growing queues. Worker expiry acknowledgments and worker expiry totals are two views of the same events and must not be summed. Request/result mailboxes are bounded to one and transport buffering to eight. Capture-slot counts refer to actual runtime acquisition opportunities; busy skips count opportunities suppressed by an occupied worker. Control scheduling can also reduce the rate of those opportunities below the nominal 30 Hz. Every acquired request and runtime capture slot is reconciled against completion, expiry and drop counters.

## Statistical uncertainty and stability

95% percentile bootstrap intervals resample whole sessions (2,000 replicates, fixed seed); they do not treat correlated frames within an alignment or session as independent. These intervals assume sessions are sufficiently exchangeable. Chronological host/thermal effects can still limit that assumption. The observed maximum is not a worst-case execution-time guarantee.

| Method | Metric | Estimate (ms) | Session bootstrap 95% interval (ms) |
|---|---|---:|---:|
| SIFT | Uncached processing mean | 86.3 | 83.7–88.9 |
| SIFT | Uncached processing p95 | 125.7 | 121.9–129.9 |
| SIFT | Uncached processing p99 | 156.3 | 148.3–163.4 |
| SIFT | Uncached processing p99.9 | 214.6 | 190.2–229.5 |
| SIFT | Capture to command mean | 129.1 | 127.0–131.1 |
| SIFT | Capture to command p95 | 181.2 | 177.7–184.4 |
| SIFT | Capture to command p99 | 208.7 | 203.8–213.8 |
| Learned GPU | Uncached processing mean | 102.3 | 89.5–117.8 |
| Learned GPU | Uncached processing p95 | 237.9 | 205.5–258.5 |
| Learned GPU | Uncached processing p99 | 301.0 | 289.8–310.5 |
| Learned GPU | Uncached processing p99.9 | 336.9 | 327.6–341.2 |
| Learned GPU | Capture to command mean | 125.2 | 120.5–130.7 |
| Learned GPU | Capture to command p95 | 235.4 | 223.6–244.3 |
| Learned GPU | Capture to command p99 | 268.5 | 261.5–276.0 |

SIFT p99.9 has only approximately 15.5 uncached observations in its upper 0.1% tail. It is much better sampled than three short trials but remains sensitive to rare events.


Learned GPU p99.9 has only approximately 10.3 uncached observations in its upper 0.1% tail. It is much better sampled than three short trials but remains sensitive to rare events.

| Method | Worker cohort | Uncached processing mean / p95 / p99 / p99.9 / maximum (ms) |
|---|---|---|
| SIFT | First alignment | 85.9 / 129.9 / 160.1 / 237.0 / 275.8 |
| SIFT | Subsequent alignments | 86.3 / 125.5 / 156.1 / 212.5 / 284.4 |
| Learned GPU | First alignment | 76.2 / 125.6 / 147.8 / 205.5 / 242.9 |
| Learned GPU | Subsequent alignments | 104.2 / 242.2 / 302.2 / 337.1 / 355.2 |

| Method | Half of session sequence | Processing p95 / p99 / max (ms), uncached | Command p95 / p99 / max (ms) |
|---|---|---|---|
| SIFT | first | 127.7 / 157.3 / 284.4 | 182.5 / 210.7 / 288.7 |
| SIFT | second | 124.2 / 152.9 / 251.7 | 179.9 / 205.1 / 274.4 |
| Learned GPU | first | 241.5 / 302.8 / 340.9 | 236.4 / 271.2 / 378.0 |
| Learned GPU | second | 232.6 / 297.3 / 355.2 | 234.2 / 265.2 / 378.1 |

![Latency by chronological session](session-latency.png)

## Spike attribution

See diagnostics.json for every watchdog event, overlapping control cycle, parent/worker GC activity, concurrent sensor work and nearby commands. The ten slowest processing frames, oldest commands and longest active control cycles are retained per method.

| Method | Session / sequence | Processing (ms) | Coarse / refinement (ms) | Extract / match / transfer host (ms) |
|---|---|---:|---|---|
| SIFT | natural-session-002 / 873 | 284.4 | 216.3 / 67.7 | — / — / — |
| SIFT | natural-session-002 / 18 | 275.8 | 226.4 / 49.1 | — / — / — |
| SIFT | natural-session-008 / 90 | 257.1 | 195.8 / 61.0 | — / — / — |
| Learned GPU | learned-session-015 / 326 | 355.2 | 310.4 / 44.3 | 138.5 / 165.3 / 0.5 |
| Learned GPU | learned-session-015 / 259 | 348.7 | 310.0 / 38.5 | 133.4 / 172.3 / 0.4 |
| Learned GPU | learned-session-014 / 747 | 345.8 | 309.7 / 35.9 | 133.6 / 169.0 / 0.5 |

**SIFT, natural-session-000, generation 4: stale_camera.** No retained preceding control stage. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-002, generation 18: control_overrun.** Nearby control cycle 116.92 ms; largest interval `receive_deserialize` 112.67 ms, thread CPU 0.000 ms and 4749594 raw CPU cycles. Overlapping parent GC events: 1; worker GC events: 0.

**Learned GPU, learned-session-017, generation 37: control_overrun.** Nearby control cycle 73.73 ms; largest interval `receive_deserialize` 67.97 ms, thread CPU 62.500 ms and 190668798 raw CPU cycles. Overlapping parent GC events: 1; worker GC events: 0.

**Learned GPU, learned-session-000, generation 8: stale_camera.** No retained preceding control stage. Overlapping parent GC events: 0; worker GC events: 0.

CUDA event intervals are recorded on the existing stream and read only after the existing required device-to-host copy. No global CUDA synchronization is added. They measure stream elapsed intervals, including host submission gaps/idle periods, not the sum of GPU kernel execution times. Host extraction/matching and transfer intervals are reported separately. Coarse and refinement timings contain their respective child stages and must not be added to those children. Process CPU time sums work across threads and can exceed wall time.

Windows thread CPU time on this host was empirically quantized to 15.625 ms. A zero sub-tick CPU reading does not prove a wait. Control-stage raw QueryThreadCycleTime counters provide finer evidence but are not converted to milliseconds because clock frequency varies. A large wall interval with few CPU cycles points to off-CPU delay; distinguishing scheduler preemption from a lock/device wait requires native scheduling traces. GC callbacks record observed pauses; worker frames carry the last eight GC events as bounded context.

The historical 65.9 ms Learned control miss had no CPU/scheduler trace. Its exact historical cause cannot be reconstructed from the saved 40.5 ms command-application interval alone. A recurring stage can be localized by this campaign; it should not be retroactively asserted to be the cause of the old event without matching evidence.

## Reproduction and evidence

Run from the repository root with the existing environment:

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B outputs/visual-servoing-simulation/run_latency_stress.py --output outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN --min-sessions 12 --max-sessions 20 --session-seconds 120 --target-uncached-frames 10000
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B tools/analyze_latency_stress.py outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN
```

[Plan, runtime versions and source hashes](manifest.json), [completion checks](completion.json), [pooled distributions and session results](summary.json), [failure/spike evidence](diagnostics.json), [individual session summaries](sessions.json). Raw per-frame gzip traces remain local under `traces/`; every trace has a SHA-256 in its session summary. Source files and runtime are checked at every session boundary and completion.
