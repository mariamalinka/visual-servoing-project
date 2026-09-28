# Latency-margin test

Follow the [test procedure](TEST_PROCEDURE.md) before every run: restart, close other programs and tabs, AC power, Best performance.

How much slower can perception get before alignment fails? This test answers that
with a measured number for SIFT and for Learned GPU, instead of depending on how fast
the laptop happens to be.

From the repository root, on AC power, with nothing else using the GPU:

```powershell
.\margin-test.cmd
```

It takes about 15–20 minutes and writes
`outputs/visual-servoing-simulation/results/margin-test/<date-time>/`. `REPORT.md` is
the summary and `margin.csv` has one row per level, ready for plotting. `--smoke`
checks the setup in about 2 minutes, and `--method sift` or `--method learned`
measures one method. `--report-only <folder>` rebuilds the report from saved evidence.

## Method

Each level adds a fixed delay to **every active perception frame**. It uses the
runtime's existing fault injection: `RuntimeConfig.inference_stall_s`, applied in the
sensor worker between rendering and matching, with `fault_after_s = 0`. Frames
captured while the robot is idle are not delayed.

- **Levels:** +0, 25, 50, 75, 100, 125, 150 and 200 ms (`tools/margin_test.json`).
- **Per level and method:** a fresh worker and 10 alignments, cycling the five
  acceptance-test poses.
- **Order:** SIFT and Learned GPU alternate at each level, so both see the same
  laptop conditions.
- **Stopping:** a method stops escalating after a level where nothing aligned.
- **Unchanged:** the controller, calibration, scene, 50 ms transport delay, 400 ms
  freshness watchdog, 50 ms control deadline and safety logic.
- **Protected files:** the same fingerprint check as the acceptance test applies.

## Results

For each method, the report gives:

- **Tolerated added delay:** the largest level where it, and every lower level,
  aligned 10/10 and no alignment was ended by the freshness watchdog.
- **Equivalent slowdown:** (normal processing p50 + tolerated delay) ÷ normal p50.
  For example, "2.0× slower perception" means hardware with half this laptop's
  perception speed is at the edge.
- **Per level:**
  - Alignments and freshness trips, split into "at start" (before the first command)
    and "mid-alignment".
  - Processing and capture-to-command latency, and time to converge.
  - Whether the laptop's own low-power state was detected during that level.

## Hold and resume

```powershell
.\margin-test.cmd --stale-resume-ms 2000
```

runs the same levels with the watchdog set to **hold and resume** instead of stop
(see `REALTIME_CONTROL.md`). The folder name ends in `-resume2000ms`. The 400 ms
trip, 50 ms deadline and all other limits are the same; only what happens after a
trip changes. The report then has a "Held and resumed" column, and a level fails
only if an alignment did not converge or was ended by the watchdog. Compare the
tolerated delay with a normal run made under the same conditions.

In the cloud check on a slower CPU, SIFT without injected delay took about 82 ms
per frame. With the default stop, +100 ms aligned 0/5. With hold and resume it
aligned 5/5 (139 holds, 15 s held, median 7.8 s to converge instead of 3.8 s).
At +150 ms and above it still failed: holds happened on almost every frame, and
the controller ended each alignment as stalled or timed out. There was no unsafe
motion at any level.

## Limits

- The injected delay is constant. Slower real hardware also has longer tails, so the
  tolerated delay is an upper bound for hardware with the same median slowdown.
- Ten alignments per level is a characterisation, not a guarantee.
- A level marked "laptop low-power state" mixes the injected delay with a real
  slowdown; repeat the run before relying on it.
- Hardware logging, the power-mode record and the cleanup of leftover samplers work
  as in the [acceptance test](ACCEPTANCE_TEST.md).
