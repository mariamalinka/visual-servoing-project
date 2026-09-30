# Decision: hold and resume after a freshness-watchdog trip

- **Status:** adopted as the default on 2026-09-29.
- **Setting:** `RuntimeConfig.stale_resume_s` (default 2 s). Setting it to 0 restores
  the original stop exactly.
- **Where:** `outputs/visual-servoing-simulation/realtime.py`, recorded in
  `tools/approved_changes.json`.

## Summary

When the newest camera image becomes too old, the robot must stop moving. The limit
is 400 ms in the tested configuration; the runtime and app default is 250 ms. That
rule is unchanged. What changed is what happens **after** the stop:

| | Stop (original, now a setting) | Hold and resume (new default) |
|---|---|---|
| At the image-age limit | Zero velocity | Zero velocity, at the same moment |
| Alignment | Ended as `stale_camera` | Kept alive, robot held still |
| Next fresh image | Ignored; the robot waits for a new Align | Control continues from where it stopped |
| No fresh image for 2 s | Already ended | Ended as `stale_camera` |

On the same laptop, back to back, hold and resume raised the extra perception delay
the system tolerates before alignment fails from +75 to +150 ms for SIFT and from +25
to +150 ms for Learned GPU. Safety counters and
final accuracy were unchanged.

## Problem

The acceptance and margin tests showed that success depended on how fast the laptop
happened to be at the moment:

- Learned GPU failed when the GPU clock dropped, when Windows used a low-power state,
  or when browser tabs were open. It passed after a restart with everything closed.
- The margin test found very little headroom with the stop response. Learned GPU
  tolerated only +25 ms of extra per-frame delay; SIFT tolerated +75 ms.

The cause is structural, not a bug in one method. The sensor worker processes one
frame at a time, so the image the controller holds can be about two frame times
plus the 50 ms transport delay old before the next result arrives. That leaves
about 155 ms per frame. A single slow frame, typically during precision refinement
near the goal, pushes the held image past 400 ms. Under the stop response, that one
frame ended the entire alignment, sometimes with the camera still about 20 mm from
the goal.

A system that needs a specific GPU and a quiet computer to succeed is fragile. The
watchdog was doing its safety job correctly; the problem was that a brief sensing
delay was treated as a task failure.

## Options considered

1. **Hold and resume** (chosen). Keep the safety stop and keep the alignment.
2. **Adaptive perception quality.** Use cheaper matching when frames run late.
3. **Latency-scaled speed.** Move slower when images are older.
4. **Two sensor workers.** Overlap frames so results arrive more often.
5. **Start-up hardware self-check.** Refuse to run on hardware that is too slow.

Hold and resume came first for four reasons:

- It is the smallest change.
- It leaves the controller, perception and every safety limit untouched.
- It addresses exactly the failure observed (occasional slow frames).
- It can be switched off with a single setting.

Options 2–4 change perception or motion behaviour and need their own validation.
Option 5 only reports the problem. They remain possible next steps.

## Design

The safety properties are preserved exactly:

- **Same trip.** The watchdog fires at the same checks and the same image age as
  before, and commands zero velocity immediately. Tests assert that the zero command
  comes less than 50 ms after the limit in both settings. The recorded worst case
  (2.03 ms) was measured with the stop setting.
- **Same eligibility rules.** A held alignment resumes only on a result that belongs
  to the same alignment, was captured after the last accepted image and is itself
  less than 400 ms old. An old or delayed result can never restart motion.
- **Bounded.** The hold ends the alignment as `stale_camera` after 2 s without a
  fresh image. The 120 s run limit, the 50 ms control deadline, manual Stop, a new
  Align, collisions and every other stop reason still end it immediately.
- **No hidden motion.** `paused_motion_ticks` counts any command while held. It must
  be 0, and the tests treat any other value as a safety violation.
- **Controller continuity.** The success hold restarts after a pause. The first
  controller step after a pause is given at most `max_age_s` of elapsed time, the
  largest step a normal run can see, so controller timers do not count the hold as
  motion.
- **Visible.** Every hold is recorded: `watchdog_pauses`, `watchdog_resumes`,
  `paused_s` and pause/resume events with their timestamps.

## Evidence

**Latency-margin test on this laptop** (RTX 3050 Laptop GPU, Best performance, AC
power). The two runs were made back to back:
- `results/margin-test/20260928-233454` is the stop run.
- `results/margin-test/20260928-234612-resume2000ms` is the hold-and-resume run.

Normal processing was about 45 ms per frame in both runs.

| Largest added delay with 10/10 aligned | Stop | Hold and resume |
|---|---:|---:|
| SIFT | +75 ms (2.7× slower perception) | +150 ms (4.3×) |
| Learned GPU | +25 ms (1.6×) | +150 ms (4.4×) |

| Learned GPU level | Stop: aligned | Hold and resume: aligned | Holds | Time to converge |
|---|---:|---:|---:|---:|
| +0 ms | 10/10 [69.2–100%] | 10/10 [69.2–100%] | 0 | 4.3 s |
| +75 ms | 2/10 [2.5–55.6%] | 10/10 [69.2–100%] | 112 | 4.9 s |
| +100 ms | 1/10 [0.3–44.5%] | 10/10 [69.2–100%] | 131 | 6.0 s |
| +150 ms | not reached | 10/10 | 342 | 10.1 s |
| +200 ms | not reached | 0/10 (stalled) | 599 | — |

Brackets are 95% exact Clopper–Pearson intervals ([statistics](STATISTICS.md)). Ten
alignments per level is a small sample: 10/10 only shows a true success rate of at
least 69%. The difference between the two responses is still clear. At +75 ms and
+100 ms the intervals do not overlap, and Fisher's exact test on the two counts gives
p = 0.0007 (2/10 vs 10/10) and p = 0.0001 (1/10 vs 10/10). That is very unlikely to
be chance. What 10 alignments cannot show is the exact success rate with hold and
resume at those levels.

**Accuracy and safety:**
- Every converged alignment finished within 0.9 mm and 0.15°.
- The stop run's failed alignments ended up to 22 mm from the goal.
- In both runs there was no unsafe motion, no motion during a hold, no motion after
  a stop, no forbidden contact and no control deadline miss.

**Acceptance test with the new default**
(`results/acceptance-test/20260929-011129`): **PASS** for both methods. The run
used the standard profile: five poses on fresh workers, then 12 minutes on one reused
worker per method, with hold and resume on (`stale_resume_s` = 2 s).

| | SIFT | Learned GPU |
|---|---:|---:|
| Aligned (95% interval) | 150/150 [97.6–100%] | 147/147 [97.5–100%] |
| Processing p99 / max | 72 / 97 ms | 67 / 97 ms |
| Capture-to-command p99 / max | 129 / 154 ms | 121 / 152 ms |
| Watchdog trips / holds / time held | 0 / 0 / 0 s | 0 / 0 / 0 s |
| Control deadline misses; unsafe, post-stop or held motion | 0; 0 | 0; 0 |
| Worst physical error | 0.46 mm / 0.10° | 0.61 mm / 0.13° |
| Later / first processing p99 (limit 1.5×) | 1.20× | 1.39× |

- **Normal operation is unchanged.** Latency, time to converge (about 4.2–4.4 s) and
  accuracy match the earlier passing run with the stop response
  (`results/acceptance-test/20260928-134939`).
- **Holds were never needed.** No frame was late enough to trip the watchdog, so
  the new default costs nothing when perception keeps up. This run shows that normal
  operation is unchanged. It does not test behaviour during holds; that evidence is
  the margin test above.
- **Laptop conditions.** There was no GPU power or thermal limiting and no laptop
  low-power period.
- **Headroom.** Learned GPU's later/first ratio (1.39×) is close to its 1.5× limit.
  Later alignments were no slower than before (p99 67.6 ms vs 69.1 ms). The ratio
  is high because the first alignments were unusually fast (p99 48.7 ms vs 57.5 ms).

**Cloud check on a slower CPU** (SIFT, about 82 ms per frame): +100 ms went from 0/5
aligned with stop to 5/5 with hold and resume.

**Limits of this evidence:**
- The two margin runs compared above and the acceptance run did not follow the
  fresh-restart step of the test procedure. Uptime was about 3 hours for the margin
  runs and 4.4 hours for the acceptance run. The two margin runs were made under the
  same conditions, so the comparison between them holds.
- A later margin run with hold and resume, 4 minutes after a restart
  (`results/margin-test/20260929-110852`), reproduced the result: +150 ms tolerated
  for both methods. A +125 ms level was excluded because of a brief GPU low-clock
  period, and it still aligned 10/10. The acceptance run should still be repeated
  once after a restart.
- There were 10 alignments per level, and the injected delay was constant. Real slow
  hardware also has longer tails.
- Success counts are estimates, not proof of 100% reliability. The acceptance run
  was judged under the rule in force at the time, "100% observed". Re-evaluated
  under the current rule (95% lower bound ≥ 95%, no failures), the same data shows
  at least 97.5% for each method and passes. That re-evaluation is not saved as a
  run ([statistics](STATISTICS.md)). A margin level with 10/10 shows only at least
  69%.

## Consequences

**What gets better**
- Success depends much less on hardware and background load. A slow frame costs a
  short pause instead of the whole alignment.
- The robot no longer stops partway to the goal because of a single late image.
- Normal operation is unchanged: with fast frames, no holds occurred.

**What gets worse, and how it is handled**
- *Slower, stop-and-go motion when perception is slow.* Pauses and pause time are
  measured. The acceptance test limits them to 5% of alignment time, so a system
  that is barely coping still fails.
- *Perception that is too slow on every frame still fails.* In the +200 ms margin
  levels it took 27–43 s (median per level) of stop-and-go before the controller's
  own stall or timeout ended the alignment,
  instead of about 1 s. The failure is still safe and reported. Adaptive quality
  (option 2) is the planned fix for this case.
- *Real actuators.* Repeated stops and restarts are harder on motors than one stop.
  Since 2026-09-30 the simulation models this with assumed acceleration, deceleration
  and jerk limits ([actuator model](ACTUATOR_MODEL.md)):
  - a hold now brakes the arm within about 0.1–0.15 s from 0.35 rad/s, and within
    70 ms at the slower speeds of a realtime hold-and-resume run;
  - a resume ramps up smoothly;
  - 20 consecutive pause/resume cycles stayed within the limits with no drift in
    stop time.

  The evidence on this page was recorded before that model. The safety side of the
  hold decision does not depend on it, because the zero command still comes at the
  same moment. The margin figures may: under heavy per-frame delay each resume moves
  the ramped arm for only a few milliseconds before the next hold. A single-run
  comparison gave the same alignment outcomes with and without the model at six
  delay levels.

  The margin test with the model on (`results/margin-test/20260930-225125`) confirmed
  the margin: +150 ms for both methods, as before. The cost is slower progress under
  heavy delay. At +150 ms, Learned GPU was held 53 s instead of 43.5 s and needed
  10.9 s instead of 9.5 s to converge ([actuator model](ACTUATOR_MODEL.md)). That run
  was not made after a fresh restart.

  A physical robot would still need its own measured ramps and an actuator-side
  command timeout.
- *More behaviour to validate.* The mechanism is covered by unit tests and
  real-process tests, and by the margin runs above.

## Changes to the tests

- **Acceptance test.** The criterion "0 freshness watchdog trips" is replaced by:
  - 0 alignments ended by the watchdog;
  - total hold time at most 5% of alignment time (`paused_fraction`);
  - 0 motion ticks during a hold.

  Every other criterion is unchanged. The report still shows the total trips and
  splits them into stops and holds. Configurations from before this change keep the
  old rule, and `--report-only` on old results reproduces their original verdicts.
- **Margin test.** A level fails if an alignment does not converge or is ended by the
  watchdog. Holds are reported in their own column.
- **Historical validation tools stay on stop:**
  - `run.cmd --latency-study`
  - the latency stress runner
  - `tools/run_acceptance_campaign.py`

  They measure the stop response and keep reproducing their recorded evidence.
- **Protected files.** `realtime.py` differs from the validated baseline only by this
  change, which is pinned to both hashes in `tools/approved_changes.json`. Any other
  edit to a protected file still makes a test INVALID.

## Using the stop setting

| Where | Hold and resume (default) | Stop |
|---|---|---|
| Code | `RuntimeConfig()` | `RuntimeConfig(stale_resume_s=0)` |
| Interactive app | `run.cmd --realtime ...` | `run.cmd --realtime ... --watchdog-stop` |
| Acceptance test | `acceptance-test.cmd` | `acceptance-test.cmd --watchdog-stop` (folder ends in `-stop`) |
| Margin test | `margin-test.cmd` | `margin-test.cmd --watchdog-stop` (folder ends in `-stop`) |

`--stale-resume-ms N` sets a different hold window. Use stop for diagnosis, where
every slow period should appear as a failure, and to compare against earlier
results.

## Next steps

1. Repeat `acceptance-test.cmd` once after a fresh restart, following
   `TEST_PROCEDURE.md`. The first run with the default passed, but with 4.4 hours of
   uptime.
2. Consider adaptive perception quality (option 2) for hardware that is too slow on
   every frame.
3. Repeat the acceptance run with the actuator model on (the default since
   2026-09-30). The margin run has been repeated (`20260930-225125`); the margin is
   unchanged.
4. For a physical robot, replace the assumed actuator limits with measured ones and
   add an actuator-side timeout.
