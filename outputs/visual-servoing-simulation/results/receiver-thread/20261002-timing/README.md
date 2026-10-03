# Receiver thread: timing before and after

Does moving the result-pipe read and frame unpickling from the control thread to the
new receiver thread (`realtime.py` `ResultReceiver`) change control-loop timing,
jitter, frame latency, deadline misses, throughput or CPU use?

**Answer in this environment: no meaningful regression.**
- **Receive step:** about 8× cheaper at the median and about 40× cheaper at p99.9.
- **Unchanged within session-to-session variation:** control-loop gap percentiles,
  throughput and capture-to-command latency.
- **CPU:** the receiver thread uses about 0.8% of one core, and the control thread
  uses about 1.8 points less.
- **The consistent cost:** IPC p99 rose by about 0.4–1.1 ms. That is small next to
  the 2 ms control period and the 50 ms deadline, and it does not show in
  capture-to-command latency.

These measurements do not justify moving images to shared memory.

## Method

- **Versions:**
  - before = `realtime.py` 521a6998689a (the control thread reads the pipe);
  - after = 3c9476102365 (the receiver thread, final version).

  Every other source file is identical. Each session JSON records its `realtime_sha`.
- **Procedure:** the project's sustained latency session (`run_latency_stress.run_session`):
  - SIFT (natural) matcher;
  - 400 ms age limit, 50 ms transport delay, stop setting;
  - repeated offset and Align for 60 s.

  Summaries use `latency_stress.summarize_session`.
- **Interleaving:** before and after alternate, each in a fresh process, so drift
  affects both.
  - Set 1 records every control cycle: the cycle probe's threshold is set to 0 ms in
    both versions.
  - Set 2 uses the stock diagnostics, which record only cycles of 10 ms or more, as
    the stress procedure does. Set 2 starts with "after" to balance the order.
- **CPU:** user + system time per thread from `/proc`, from 1 s after start-up until
  `close()`, as a percentage of one core.
- **Scripts:**
  - `receiver_compare.py`: one session;
  - `aggregate.py`: the tables;
  - `pause-scenario/`: the watchdog-pause test's session, repeated (`pause_margin.py`,
    `pause_status.py`, `pause_diag.py` and their outputs).

Two identical comparisons of intermediate versions gave the same conclusions:
- a 4-slot mailbox instead of the final one-item mailbox;
- 6b150fda0407, which differs from the final version only in error-path handling.

They are superseded and not published.

## Results

The full tables are in `comparison.md`. The per-session data are in `sessions/`.

| Measure (mean of sessions; set 1 / set 2) | Before | After |
|---|---|---|
| Loop gap p50 (ms) | 2.01 / 2.00 | 2.00 / 2.00 |
| Loop gap p99 (ms) | 8.27 / 8.03 | 7.95 / 7.91 |
| Loop gap p99.9 (ms) | 11.53 / 10.95 | 11.25 / 10.76 |
| Loop gap max (ms) | 43.3 / 23.2 | 42.7 / 24.5 |
| Control deadline misses (> 50 ms), all sessions | 1 / 0 | 0 / 0 |
| Receive step per tick, p50 / p99.9 (ms), set 1 | 0.024 / 1.57 | 0.003 / 0.036 |
| Busy time per tick, p99 / p99.9 (ms), set 1 | 5.35 / 8.57 | 5.04 / 8.07 |
| Wake-up lateness p99 (ms), set 1 | 4.32 | 4.16 |
| Frames received per second | 9.13 / 9.23 | 9.21 / 9.27 |
| IPC (worker finished → control loop has it), p50 (ms) | 2.55 / 2.53 | 2.32 / 2.34 |
| IPC p99 (ms) | 4.56 / 4.23 | 4.99 / 5.30 |
| IPC max (ms) | 6.1 / 7.0 | 22.4 / 7.8 |
| Capture to command p50 (ms) | 175.8 / 175.2 | 174.7 / 173.9 |
| Capture to command p99 (ms) | 220.8 / 215.0 | 211.8 / 211.3 |
| Capture to command max (ms) | 234.1 / 232.9 | 222.5 / 231.9 |
| CPU, control thread (% of a core) | 30.8 / 30.2 | 29.0 / 28.4 |
| CPU, receiver thread | — | 0.79 / 0.80 |
| CPU, whole control process | 31.2 / 30.6 | 30.2 / 29.6 |
| CPU, sensor worker process | 120.8 / 122.4 | 121.8 / 121.6 |

With the receiver thread, a frame spends 1.8 ms (p50) in the pipe read and
unpickling on the receiver thread. It then waits 0.5 ms (p50) in the mailbox until
the next control tick takes it. In these sessions the mailbox never had to drop a
message (`receiver_dropped` = 0), and the receiver was never stuck at shutdown.

**Interpretation, with the limits of four and three sessions per version:**
- **Control timing and jitter:** not worse. The loop-gap percentiles, per-tick busy
  time and wake-up lateness overlap between versions.
- **Deadline misses:** 1 before, 0 after. The two superseded comparisons also had
  misses only before (2 and 1). That is too few events to claim a reduction.
- **Worst single tick:** both versions have single ticks of 5–29 ms of control-thread
  work, in the receive stage and elsewhere. Set 1 creates a record for every cycle,
  which makes Python's garbage collection heavier, so these maxima are not attributed
  to the change in either direction.
- **IPC p50 and p99:** p50 is about 0.2 ms lower. p99 is about 0.4–1.1 ms higher in
  every comparison so far. This is the measured cost of the hand-off between two
  threads.
- **IPC max, set 1:** higher with the receiver thread (mean 22 ms, single events up
  to 36 ms). These are frames that waited in the mailbox during one long control tick
  (mailbox wait max up to 34 ms in one session). That is the same kind of long tick
  that, before the change, delayed the pipe read itself. Set 2, without per-cycle
  recording, shows no such difference (7.8 vs 7.0 ms). Capture-to-command latency
  includes this wait and is not higher.
- **CPU:** the work moved; the total did not grow.
- **Throughput:** unchanged.
- **Freshness-watchdog stops** (stop setting): 8 before and 5 after over all sessions.
  SIFT on 2 cores is sometimes slower than the 400 ms budget. This is a property of
  the environment, not of the change.

## Watchdog-pause test margin

`tests/test_realtime.py::test_watchdog_pause_zeroes_motion_on_time_and_resumes_on_fresh_images`
injects about 350 ms of latency per frame. It requires every capture-to-command
latency to stay under 400 ms, and it requires no stop other than holds. It failed once
in an earlier full-suite run on an intermediate version, and it passed in the final
full suite (`full-test-suite.txt`: 390 tests, 0 failures, 22 GPU tests skipped). The
same session was repeated many times per version, interleaved (`pause-scenario/`).

**Latency margin** (`margin_6_per_version.txt`, final code):

| | Worst frame per run (ms) | Frames at or over 400 ms |
|---|---|---|
| Before | 396.4–400.8 | 2 |
| After | 397.0–400.5 (one run ended early, see below) | 1 |

The test sits at its 400 ms limit on this 2-core machine in both versions, so its
occasional latency failure is not caused by the change. The test was not changed.

**Early `control_overrun` stops in this scenario.** The session aligns immediately at
start-up. On the final code it ended with `control_overrun` 0.25–0.7 s after Align in
2 or 3 of the 56 sessions here:
- 1 of 20 in `status_20_per_version.txt`;
- 1 of 30 in `diagnosed_30_after.txt`;
- probably the run with only 4 frames in `margin_6_per_version.txt`.

On the old code it happened in none of the 56.

One instance was captured with the project's cycle and GC probes. The stalled tick
spent 54.8 ms in `sleep_or_descheduled`: it slept, and the operating system did not
run it again for 54 ms (wake-up 53.8 ms late, 0.8 ms of thread CPU). No garbage
collection ran within 0.3 s, and no frame passed through the receiver within 0.3 s,
so the receiver thread was blocked in the kernel, waiting for data.

The sustained sessions above point the other way: 1 deadline miss before and 0
after (the two superseded comparisons also had misses only before). Neither
difference is statistically significant (Fisher's exact test on 3 of 56 vs 0 of 56:
two-sided p ≈ 0.24, one-sided p ≈ 0.12).
The diagnosed stall shows no receiver activity. These measurements cannot show that
the receiver thread causes start-up stalls, and they cannot fully exclude a small
effect. In every case the stop itself was correct: zero command and no unsafe
motion. The laptop acceptance run, which requires 0 deadline misses, is the
decisive check.

## Limits

- **Cloud workspace, not the laptop:**
  - 2 CPU cores, Linux, no GPU;
  - the Learned GPU matcher was not run. The frame size, and therefore the transfer
    cost, is the same for both matchers.
  - Windows timing on the laptop must be confirmed with `acceptance-test.cmd` and the
    latency stress run.
- **Short sessions:** 7 sessions of 60 s per version. This measures typical behaviour
  and large regressions, not rare tail events.
