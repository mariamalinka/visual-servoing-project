# Sustained perception latency validation

The same starting offset, 50 ms added transport, 400 ms capture-age budget, 50 ms control-loop deadline, controller, calibration, scene, model settings and dependency versions as the preceding three-run comparison. Each process uses the same initial model warm-up. Explicit offset/align cycles then reuse that worker; the stopping criterion and 1.1 s post-stop observation window are unchanged.

Predeclared stopping rule: at least 12 sessions and 10,000 uncached active frames per method, or 20 sessions maximum. Each session runs for at least 120 s after ready and ends at an attempt boundary. Modes alternate in each complete round; all failures remain included.

| Method | Sessions | Aligned / attempts | First attempt in each process | Subsequent attempts | Active / ready minutes | Uncached active frames |
|---|---:|---:|---:|---:|---:|---:|
| SIFT | 20 | 454/498 | 18/20 | 436/478 | 31.1 / 40.8 | 14,297 |
| Learned GPU | 20 | 119/796 | 19/20 | 100/776 | 25.2 / 40.9 | 10,506 |

Observed 721 unsuccessful alignment attempts, 24 control deadline misses and 697 freshness watchdog trips. Sample target met: True. Source unchanged throughout the campaign: True.

## Processing latency

| Method | Cohort | Samples | Mean / p95 / p99 / p99.9 / maximum (ms) |
|---|---|---:|---|
| SIFT | All active captures | 17,975 | 73.1 / 126.5 / 149.3 / 195.9 / 306.6 |
| SIFT | Uncached active captures | 14,297 | 91.9 / 129.6 / 152.9 / 201.5 / 306.6 |
| Learned GPU | All active captures | 14,008 | 78.4 / 212.4 / 291.8 / 329.6 / 382.2 |
| Learned GPU | Uncached active captures | 10,506 | 104.5 / 239.7 / 298.7 / 334.2 / 382.2 |

Processing is the complete wall interval from image acquisition completion to the finished perception result. It includes refinement, host/device waits and preemption. Active captures remain counted when processing completes after a stop. Idle captures are excluded. Cache hits are shown in the all-active distribution and excluded explicitly in the second distribution.

## Capture to command and held feedback age

| Method | Accepted commands | Mean / p95 / p99 / maximum (ms) | Maximum held feedback age bound (ms) |
|---|---:|---|---:|
| SIFT | 17,449 | 133.7 / 186.5 / 207.9 / 287.1 | 455.7 |
| Learned GPU | 12,987 | 128.8 / 236.4 / 268.5 / 375.1 | 412.3 |

Capture-to-command uses actual accepted command application timestamps. Rejected results have no command timestamp; their processing remains in the preceding table and their capture-to-delivery distribution is in summary.json. The held-age bound uses the preceding capture through the next application or stop, and includes initial waiting after arming. It is evaluated separately for every attempt and includes the interval between accepted updates.

## Watchdogs, bounded work and completeness

| Counter | SIFT | Learned GPU |
|---|---:|---:|
| Acquisition slots | 69932 | 69869 |
| Captured requests | 34220 | 33158 |
| Skipped busy acquisition slots | 35712 | 36711 |
| Pending request replacements | 0 | 0 |
| Transport results replaced | 0 | 2 |
| Result mailbox drops | 0 | 0 |
| Pending transport entries cancelled at stops | 1257 | 1657 |
| Obsolete generation results | 334 | 533 |
| Expired requests before inference | 0 | 0 |
| Expired results | 0 | 7 |
| Completed but unreceived results | 0 | 0 |
| Requests never started | 0 | 0 |
| Control deadline misses | 10 | 14 |
| Unsafe motion ticks | 0 | 0 |
| Motion ticks after stop | 0 | 0 |
| Forbidden contacts | 0 | 0 |
| Freshness watchdog trips | 34 | 663 |
| Late completions after freshness stop | 27 | 575 |
| Late completions after control stop | 7 | 11 |
| All stops stayed latched | True | True |
| Frame/command/loop telemetry complete | True | True |
| Extended diagnostics enabled | True | True |
| Control-cycle/parent-GC buffers complete | True | True |

Counters cover entire sessions, including post-stop observation and draining. Skipped slots do not acquire an image. Pending cancellation at a stop and obsolete generations after an explicit reset are intentional cleanup, not evidence of growing queues. Worker expiry acknowledgments and worker expiry totals are two views of the same events and must not be summed. Request/result mailboxes are bounded to one and transport buffering to eight. Capture-slot counts refer to actual runtime acquisition opportunities; busy skips count opportunities suppressed by an occupied worker. Control scheduling can also reduce the rate of those opportunities below the nominal 30 Hz. Every acquired request and runtime capture slot is reconciled against completion, expiry and drop counters.

## Statistical uncertainty and stability

95% percentile bootstrap intervals resample whole sessions (2,000 replicates, fixed seed); they do not treat correlated frames within an alignment or session as independent. These intervals assume sessions are sufficiently exchangeable. Chronological host/thermal effects can still limit that assumption. The observed maximum is not a worst-case execution-time guarantee.

| Method | Metric | Estimate (ms) | Session bootstrap 95% interval (ms) |
|---|---|---:|---:|
| SIFT | Uncached processing mean | 91.9 | 89.7–94.1 |
| SIFT | Uncached processing p95 | 129.6 | 125.9–133.0 |
| SIFT | Uncached processing p99 | 152.9 | 148.2–155.8 |
| SIFT | Uncached processing p99.9 | 201.5 | 189.6–209.9 |
| SIFT | Capture to command mean | 133.7 | 132.0–135.5 |
| SIFT | Capture to command p95 | 186.5 | 183.3–189.5 |
| SIFT | Capture to command p99 | 207.9 | 203.5–211.2 |
| Learned GPU | Uncached processing mean | 104.5 | 99.9–109.2 |
| Learned GPU | Uncached processing p95 | 239.7 | 228.3–250.4 |
| Learned GPU | Uncached processing p99 | 298.7 | 294.1–304.5 |
| Learned GPU | Uncached processing p99.9 | 334.2 | 325.6–339.5 |
| Learned GPU | Capture to command mean | 128.8 | 127.3–130.3 |
| Learned GPU | Capture to command p95 | 236.4 | 232.6–240.3 |
| Learned GPU | Capture to command p99 | 268.5 | 265.0–274.3 |

SIFT p99.9 has only approximately 14.3 uncached observations in its upper 0.1% tail. It is much better sampled than three short trials but remains sensitive to rare events.


Learned GPU p99.9 has only approximately 10.5 uncached observations in its upper 0.1% tail. It is much better sampled than three short trials but remains sensitive to rare events.

| Method | Worker cohort | Uncached processing mean / p95 / p99 / p99.9 / maximum (ms) |
|---|---|---|
| SIFT | First alignment | 84.1 / 127.4 / 156.3 / 181.8 / 189.7 |
| SIFT | Subsequent alignments | 92.2 / 129.8 / 152.5 / 202.1 / 306.6 |
| Learned GPU | First alignment | 63.5 / 105.0 / 120.0 / 144.5 / 204.7 |
| Learned GPU | Subsequent alignments | 107.9 / 246.2 / 300.3 / 334.8 / 382.2 |

| Method | Half of session sequence | Processing p95 / p99 / max (ms), uncached | Command p95 / p99 / max (ms) |
|---|---|---|---|
| SIFT | first | 132.9 / 154.6 / 306.6 | 189.9 / 211.3 / 287.1 |
| SIFT | second | 125.2 / 149.0 / 244.4 | 182.3 / 203.6 / 267.4 |
| Learned GPU | first | 229.0 / 298.4 / 348.8 | 231.8 / 267.8 / 375.1 |
| Learned GPU | second | 250.4 / 298.7 / 382.2 | 240.8 / 269.1 / 369.1 |

![Latency by chronological session](session-latency.png)

## Spike attribution

See diagnostics.json for every watchdog event, overlapping control cycle, parent/worker GC activity, concurrent sensor work and nearby commands. The ten slowest processing frames, oldest commands and longest active control cycles are retained per method.

| Method | Session / sequence | Processing (ms) | Coarse / refinement (ms) | Extract / match / transfer host (ms) |
|---|---|---:|---|---|
| SIFT | natural-session-003 / 617 | 306.6 | 270.2 / 36.1 | — / — / — |
| SIFT | natural-session-013 / 1144 | 244.4 | 68.9 / 175.2 | — / — / — |
| SIFT | natural-session-005 / 96 | 233.8 | 146.2 / 87.2 | — / — / — |
| Learned GPU | learned-session-012 / 531 | 382.2 | 330.8 / 51.2 | 140.2 / 186.2 / 0.3 |
| Learned GPU | learned-session-016 / 1146 | 350.3 | 299.8 / 50.3 | 133.0 / 160.7 / 0.5 |
| Learned GPU | learned-session-002 / 1195 | 348.8 | 304.1 / 44.4 | 137.0 / 161.1 / 0.5 |

**SIFT, natural-session-002, generation 25: control_overrun.** Nearby control cycle 50.61 ms; largest interval `receive_deserialize` 44.94 ms, thread CPU 46.875 ms and 146405247 raw CPU cycles. Overlapping parent GC events: 1; worker GC events: 0.

**SIFT, natural-session-003, generation 4: control_overrun.** Nearby control cycle 50.29 ms; largest interval `physics_collision` 45.03 ms, thread CPU 0.000 ms and 73069107 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-003, generation 10: control_overrun.** Nearby control cycle 113.09 ms; largest interval `physics_collision` 92.29 ms, thread CPU 15.625 ms and 11400237 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-005, generation 2: control_overrun.** Nearby control cycle 55.37 ms; largest interval `physics_collision` 52.62 ms, thread CPU 15.625 ms and 24476988 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-006, generation 22: control_overrun.** Nearby control cycle 11.65 ms; largest interval `supervision` 0.65 ms, thread CPU 0.000 ms and 2313351 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-008, generation 22: control_overrun.** Nearby control cycle 80.51 ms; largest interval `physics_collision` 77.45 ms, thread CPU 15.625 ms and 103094886 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-013, generation 12: control_overrun.** Nearby control cycle 68.24 ms; largest interval `controller_compute` 44.47 ms, thread CPU 15.625 ms and 13212161 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-014, generation 21: control_overrun.** Nearby control cycle 27.98 ms; largest interval `sleep_or_descheduled` 16.86 ms, thread CPU 0.000 ms and 499537 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-015, generation 1: control_overrun.** Nearby control cycle 57.57 ms; largest interval `receive_deserialize` 50.46 ms, thread CPU 31.250 ms and 119153308 raw CPU cycles. Overlapping parent GC events: 1; worker GC events: 0.

**SIFT, natural-session-018, generation 4: control_overrun.** Nearby control cycle 73.24 ms; largest interval `command_application` 46.93 ms, thread CPU 0.000 ms and 3488802 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**SIFT, natural-session-000, generation 10: stale_camera.** Nearby control cycle 11.30 ms; largest interval `physics_collision` 8.56 ms, thread CPU 0.000 ms and 21453586 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-000, generation 6: control_overrun.** Nearby control cycle 49.39 ms; largest interval `receive_deserialize` 24.57 ms, thread CPU 0.000 ms and 1302139 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-000, generation 13: control_overrun.** Nearby control cycle 54.22 ms; largest interval `physics_collision` 50.67 ms, thread CPU 0.000 ms and 45504614 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-002, generation 31: control_overrun.** Nearby control cycle 70.34 ms; largest interval `sleep_or_descheduled` 68.59 ms, thread CPU 0.000 ms and 325818 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-005, generation 1: control_overrun.** Nearby control cycle 119.09 ms; largest interval `physics_collision` 116.25 ms, thread CPU 15.625 ms and 26610547 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-005, generation 24: control_overrun.** Nearby control cycle 64.89 ms; largest interval `physics_collision` 61.12 ms, thread CPU 0.000 ms and 30333002 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-006, generation 7: control_overrun.** Nearby control cycle 52.51 ms; largest interval `physics_collision` 30.98 ms, thread CPU 15.625 ms and 37543114 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-008, generation 8: control_overrun.** Nearby control cycle 49.94 ms; largest interval `physics_collision` 44.33 ms, thread CPU 0.000 ms and 20345650 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-014, generation 11: control_overrun.** Nearby control cycle 69.05 ms; largest interval `physics_collision` 65.66 ms, thread CPU 15.625 ms and 43591058 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-014, generation 34: control_overrun.** Nearby control cycle 62.01 ms; largest interval `physics_collision` 48.22 ms, thread CPU 0.000 ms and 51035199 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-014, generation 43: control_overrun.** Nearby control cycle 61.21 ms; largest interval `sleep_or_descheduled` 58.15 ms, thread CPU 0.000 ms and 300405 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-015, generation 2: control_overrun.** Nearby control cycle 52.66 ms; largest interval `physics_collision` 50.25 ms, thread CPU 0.000 ms and 14369405 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-015, generation 23: control_overrun.** Nearby control cycle 71.40 ms; largest interval `controller_compute` 33.55 ms, thread CPU 0.000 ms and 8973102 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-016, generation 26: control_overrun.** Nearby control cycle 25.83 ms; largest interval `supervision` 0.83 ms, thread CPU 0.000 ms and 2399830 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-018, generation 4: control_overrun.** Nearby control cycle 58.20 ms; largest interval `physics_collision` 32.57 ms, thread CPU 0.000 ms and 27616836 raw CPU cycles. Overlapping parent GC events: 0; worker GC events: 0.

**Learned GPU, learned-session-000, generation 12: stale_camera.** No retained preceding control stage. Overlapping parent GC events: 0; worker GC events: 0.

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

[Diagnostic follow-up and attribution limits](DIAGNOSIS.md).

[Complete before/after comparison](COMPARISON.md). [Functional and evidence verification](VERIFICATION.md).

For an exact repeat against the failed sustained baseline, add
`--baseline outputs/visual-servoing-simulation/results/latency/20260922-sustained/manifest.json`
to the command above. The recorded campaign includes graph capture at model
initialization and retains the original one-image warm-up.
