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

- **Tolerated added delay:** the largest level where it, and every lower level not
  excluded for a laptop low-power state, aligned 10/10 and no alignment was ended by the freshness watchdog. 10/10 shows a
  true success rate of at least 69.2% at that level (95% confidence), not 100%.
  Each level's "Aligned" cell shows its exact 95% interval, and `margin.csv` has
  `success_ci_low` and `success_ci_high` columns. The margin is a characterisation.
  Demonstrating 95% per level would need 72 alignments; adding
  `required_success_rate` to `tools/margin_test.json` switches to that rule. See
  [Success rates and confidence intervals](STATISTICS.md).
- **Equivalent slowdown:** (normal processing p50 + tolerated delay) ÷ normal p50.
  For example, "2.0× slower perception" means hardware with half this laptop's
  perception speed is at the edge.
- **Per level:**
  - Alignments and freshness trips, split into "at start" (before the first command)
    and "mid-alignment".
  - Processing and capture-to-command latency, and time to converge.
  - Whether the laptop's own low-power state was detected during that level.

## Watchdog response

The test uses the default response, **hold and resume**: a trip stops the robot at
400 ms, and the alignment continues on the next fresh image. A level fails only if an
alignment did not converge or was ended by the watchdog. Holds and the time held are
reported in their own column.

```powershell
.\margin-test.cmd --watchdog-stop
```

runs the same levels with the original stop response (the folder name ends in
`-stop`), for comparison with earlier results. On this laptop, back to back, the
tolerated delay was:

| | Stop | Hold and resume |
|---|---:|---:|
| SIFT | +75 ms | +150 ms |
| Learned GPU | +25 ms | +150 ms |

Why this is the default, and what it costs: [watchdog decision](WATCHDOG_DECISION.md).

Since 2026-09-30 the simulated arm has actuator dynamics, so a hold also takes time
and distance to stop physically. The report's safety section and `margin.csv` show
the longest physical stopping time and camera travel after a hold, per level. These
figures are informational, not pass criteria ([actuator model](ACTUATOR_MODEL.md)).
The results in the table above were recorded before the model. With the model on
(`results/margin-test/20260930-225125`), both methods still tolerate +150 ms. Under
heavy delay they spend more time held: Learned GPU at +150 ms was held 53 s instead
of 43.5 s ([actuator model](ACTUATOR_MODEL.md)).

## Limits

- The injected delay is constant. Slower real hardware also has longer tails, so the
  tolerated delay is an upper bound for hardware with the same median slowdown.
- Ten alignments per level is a characterisation, not a guarantee.
- A level marked "laptop low-power state" mixes the injected delay with a real
  slowdown; repeat the run before relying on it.
- Hardware logging, the power-mode record and the cleanup of leftover samplers work
  as in the [acceptance test](ACCEPTANCE_TEST.md).
