# Fault injection and the safety envelope

This page covers two additions from 2026-10-01 to the project's safety evidence:

1. **Safety envelope in every run.** The acceptance test, margin test, acceptance
   campaign and stress analysis now record clearance, joint-limit margin and joint
   speed over every physics step.
2. **Fault-injection campaign.** The campaign deliberately triggers every safety stop
   that previously had no direct test, and records the reaction.

On 2026-10-02 the control-loop freeze found by the campaign (finding 2) was fixed:
sensor results are now received on a separate thread. Two regression scenarios target
that failure, and the campaign was re-run on the fixed runtime.

Both are simulation evidence. The timings depend on the computer that runs them.
Nothing here says how a real robot would behave.

## 1. Safety envelope (REQ-11 to REQ-13)

### What is recorded

These values were already computed by several side studies:
- clearance above the collision margin in the collision, robustness and accuracy
  runners;
- joint margin in the robustness, startup and joint-limit studies.

The main runs recorded neither. The main runs now reuse the calculation that runs
anyway: the collision guard checks every distance on every 2 ms physics step, and
it now keeps the smallest slack of each check. `Simulation.safety_record()` keeps the
extremes. No new geometry code was added.

| Metric | Source | Limit | Requirement |
|---|---|---|---|
| Minimum clearance to the environment above its 12 mm margin | the guard's own check (`collision.py`), every physics step | ≥ 0 mm | REQ-11 |
| Minimum clearance between robot parts above its 6 mm margin | the same check | ≥ 0 mm | REQ-11 |
| Minimum distance to a joint limit | position after each physics step vs `scene.xml` ranges | ≥ 0 rad | REQ-12 |
| Peak commanded joint speed | command after the guard and the backstop | ≤ 0.6 rad/s | REQ-13 |
| Peak measured joint speed | MuJoCo joint velocity | ≤ 0.6 rad/s | REQ-13 |

`safety_metrics.py` gives each metric its value, limit, PASS/FAIL and requirement ID.

Some details of the measurement:
- **Moment of the checks.** Clearance is checked on the pose before each physics
  step, as the guard does. Joint margin and speed are checked after the step.
- **Upper bound.** The guard only computes distances up to its 80 mm influence
  range, so a larger clearance is reported as at most 80 mm minus the margin.
- **Session scope.** Each value is the worst over the whole session, including idle
  time.
- **Mixed runs.** When only some sessions have a record, the report says in how many
  of them it was recorded.

### Where it appears

| Run | Report | Per-run data | Effect of a FAIL |
|---|---|---|---|
| Acceptance test | "Safety envelope" table per method | `safety.csv` (per session), `verdict.json` | the method fails |
| Acceptance campaign | per-session table | `verdict.json` | the session fails |
| Margin test | per-level table per method | `margin.csv` columns, `margin.json` | the level is not tolerated |
| Stress analysis | worst session per method | `summary.json` (per method), `sessions.json` (per session) | reported, not gated (the stress run has no pass/fail) |

Runs made before this change show "not recorded". That never counts as a pass or a
failure, so re-reporting old results keeps their verdicts.

**First saved run with the record:** the laptop acceptance run
`results/acceptance-test/20261002-233959` (2026-10-02, PASS) recorded the envelope in 12 of 12
sessions:
- clearance at least 3.00 mm to the environment and 9.00 mm between robot parts;
- joint margin at least 0.995 rad;
- peak commanded speed 0.280 rad/s and peak measured speed 0.198 rad/s.

All are PASS. No obstacle was placed and no joint came near its limit, so this run does
not exercise detours or the joint-limit slowdown. REQ-11 to REQ-13 therefore stay
PARTIALLY VERIFIED (see [traceability](SAFETY_TRACEABILITY.md)). No margin or stress
run with the record has been saved yet.

The stress runner also refuses changed production inputs. Since `simulation.py` has
changed, it needs a baseline manifest made with the current inputs before it will
run.

## 2. Fault-injection campaign

```powershell
.\run.cmd --fault-campaign              # every scenario, 3 repeats
.\run.cmd --fault-campaign --repeats 5
```

The runner writes `REPORT.md` and `summary.json` to `results/fault-injection/<time>`.
Every repeat's record contains:
- the fault that was injected;
- the documented reaction;
- the measured reaction;
- every check made on it.

### How faults are injected

Injection reuses the project's existing patterns. No path is triggered by editing
production logic:

- **Sensor side:** a `RuntimeConfig` field, like the existing `inference_stall_s`.
  `inference_failure = 1` makes every faulted frame return an `inference_failed`
  observation. The default is 0, and negative values are rejected.
- **Control side:** a class is replaced for one session, as the acceptance tool
  already does with `simulation.Simulation`. Examples:
  - a `Simulation` subclass that blocks the control thread once;
  - a slow controller update;
  - a stand-in clock for `realtime.py` that jumps back;
  - a guard that answers "blocked".

  The sensor worker is a separate process and is not affected.
- **Worker crash:** the sensor worker process is killed.
- **Result pipe** (since 2026-10-02):
  - `sensor_worker_partial_message`: the runtime spawns a test worker,
    `fault_injection.partial_result_worker`. It is the real worker, except that on a
    signal it writes a frame's length header and half of its bytes into the real
    results pipe. That is the state a worker killed mid-transfer leaves behind. This
    needs POSIX pipe framing: Python's Windows pipes are message-oriented, so the
    scenario is not offered on Windows.
  - `sensor_result_read_blocked`: the parent's read of the next frame never
    returns. This works on every platform.

  In both, the worker stays alive for one freshness window and is then killed.

Every fault is injected while the arm is moving, at one of these points:
- after 10 accepted frames (loop stalls, clock step, collision block, worker crash);
- at the 6th controller update (controller stall);
- 0.6 s or 1 s after Align (inference failure, watchdog, run limit).

Every realtime scenario checks that the arm was moving before the fault.

### Results after the receiver-thread fix (2026-10-02)

Two campaigns ran on the fixed runtime (`realtime.py` 3c9476102365) in the cloud
workspace with the default actuator profile:
- `results/fault-injection/20261002-141659-worker-crash`: the three worker-failure
  scenarios, 30 repeats each;
- `results/fault-injection/20261002-141659-all`: every scenario, 5 repeats each.

154 of 156 repeats passed.

| Path | Requirement | Repeats passed | Measured reaction (worst of passed repeats) | Campaign status |
|---|---|---|---|---|
| Half-sent frame in the results pipe, then the worker dies | REQ-06, REQ-01 | 30/30 and 5/5 | the control loop ran 146–168 ticks during the blocked transfer; the watchdog paused at most 1.7 ms after the 400 ms limit; `worker_failed` at most 40.1 ms after the kill (next highest 19.0 ms); the receiver's read ended with "end of file during message" every time; shutdown completed | VERIFIED |
| Result read that never returns, then the worker dies | REQ-06, REQ-01 | 30/30 and 5/5 | 148–169 ticks during the blocked read; pause at most 1.8 ms after the limit; `worker_failed` at most 20.2 ms after the kill; shutdown left the blocked receiver behind after its 0.5 s bounded join every time | VERIFIED |
| Sensor worker crash (random moment) | REQ-06 | 29/30 and 5/5 | `worker_failed` at most 20.8 ms after the kill | NOT VERIFIED in the 30-repeat campaign (see below); VERIFIED in the 5-repeat one |

The rest of the 5-repeat campaign:
- control-loop stalls: stop at most 2.8 ms after the stalled tick;
- physics-gap cap: 2.7 ms;
- slow computation: 0.6 ms;
- collision block: 0.6 ms;
- run limit: 2.1 ms;
- watchdog stop: 1.9 ms after the image-age limit;
- failed inference: 50.4 ms after the failed result reached the control loop;
- clock step: 4/5 passed (0.4 ms after the jump in the passed repeats).

**The two failed repeats never injected their fault.** An earlier, correct safety stop
ended the alignment first:
- **Worker crash, repeat 14 of 30:** a control-loop gap over 50 ms right after Align
  stopped the alignment with `control_overrun` before the arm moved. The scenario
  waits for motion before killing the worker, so the kill never happened.
- **Clock step, repeat 1 of 5:** the arm moved, but a frame took longer than the
  400 ms image-age limit (SIFT on 2 cores), so `stale_camera` stopped it before the
  clock jump.

In both, the command was zero and there was no unsafe motion. The campaign's rule
counts any failed repeat, so these two paths are NOT VERIFIED in this run. They were
not re-run until they passed.

Both stops also occur in this environment without the change. The interleaved
sessions in `results/receiver-thread/20261002-timing` had:
- `stale_camera` stops in both versions;
- 1 deadline miss on the old code and 0 on the new.

The only clustering seen was at start-up in one test scenario: 2–3 of 56 sessions on
the new code against 0 of 56 on the old, which is not statistically significant. The diagnosed instance was the operating system not scheduling the
control thread while the receiver was idle (see that folder's README). The scenarios
could mark a repeat whose fault was never injected as invalid instead of failed.
That would be a change to the harness, and it has not been made.

The 40.1 ms worst kill-to-stop in the half-sent-frame scenario is a single outlier in
104 kills on the final code; the next highest is 20.8 ms. The dead worker is detected
by the same `is_alive()` check as before, at the top of each tick. The saved records
do not show whether process teardown or a scheduling gap caused it. It is inside the
50 ms deadline.

**Old code, same random kill.** For comparison, 30 random kills ran on the runtime
before the fix (`results/receiver-thread/20261002-old-code-worker-kills`):

| Runtime | Control loop froze | Kill to stop, p50 / p90 / max |
|---|---|---|
| Before the fix | 1 of 30 ("did not shut down within five seconds") | 12.0 / 14.9 / 21.7 ms |
| After the fix | 0 of 34 kills | 12.1 / 14.3 / 20.8 ms |

The freeze reappeared on the old code from a random kill. Detecting a dead worker
takes as long as before.

### Windows laptop results and the detection-time criterion (2026-10-03)

The fixed runtime (`realtime.py` 3c9476102365) was run on the laptop (Windows 11,
RTX 3050):
- **Acceptance test:** `results/acceptance-test/20261002-233959`, **PASS** for both
  methods. 149/149 and 147/147 alignments, 0 control deadline misses, 0 watchdog trips,
  capture-to-command p99 125.0 / 117.4 ms. The laptop had been running 5075 min instead
  of being freshly restarted (REQ-23).
- **Full campaign, 3 repeats:** `results/fault-injection/20261003-001323`.
  - **Stops matched the cloud workspace:** control-loop stalls stopped at most 3.7 ms
    after the stalled tick, the clock step 0.5 ms after the jump, the watchdog stop
    2.3 ms after the limit, and the speed clip held at 0.600 rad/s (peak measured
    0.592 rad/s).
  - **Clock step passed 2/3.** The failed repeat again stopped with `stale_camera`
    before the jump was injected.
  - **The never-returning-read scenario failed 0/3**, on the 50 ms detection gate
    described below.

**Worker-failure detection on Windows.** The gate then in force was "stop within 50 ms
of the kill request". On the laptop it failed:
- in `20261003-001323`, 0/3 for the never-returning read;
- in `20261003-002419` (10 repeats), 2/10 for the random kill and 0/10 for the
  never-returning read;
- in `20261003-003238` (10 repeats), 3/10 and 0/10. That run used the harness before
  the exit timing was added: the file had been overwritten on the laptop just before
  the run.

Kill request to stop was 37–132 ms. In every run the control loop kept ticking, the
watchdog paused at most 2.12 ms after the 400 ms limit (33 measurements), and the arm
stopped with `worker_failed` and zero command.

To find where the time goes, the harness now also records when the OS reports the
worker exited (`kill_and_watch`). It waits on the process sentinel, which does not
reap the process. Run `20261003-010746` (10 repeats each) split the time:

| Scenario | Kill request → OS reports exit | Exit → stop (the runtime) |
|---|---|---|
| Result read that never returns | 54.6–129.8 ms | 0.4–2.8 ms |
| Random kill | 48.2–113.4 ms | −16.1 to −11.4 ms |

**Interpretation.**
- **Windows is the slow part.** Windows takes about 50–130 ms to report a killed worker
  exited, against about 10 ms on Linux. The runtime stops within 3 ms after that.
- **The negative values are early detection.** In the random-kill runs the arm
  stopped 7–16 ms *before* Windows reported the process exited. The receiver thread saw
  the dying worker's pipe close before the process handle signalled, and the runtime
  stopped on that ("Sensor process exited with code -15 (result pipe closed)"). The
  confirmation run below records this path (`detected_by`) for all 10 random kills. (An earlier version
  of this page credited `is_alive()` and every run's error text; that was wrong. The
  random-kill scenario did not save its error text, so those runs carry no evidence
  for either explanation. Since 2026-10-03 the scenarios record the path as
  `detected_by`.) In the never-returning-read scenario the receiver is blocked, so
  the stop comes from `is_alive()`, 0.4–2.8 ms after the exit.
- **This is not a receiver-thread effect.** Detection uses the same per-tick
  `is_alive()` check as before the fix. No old-code run on Windows exists to compare
  against.

**Consequence for a real system:** if the perception process dies hard on Windows, the
last command keeps running for about 50–130 ms before the stop, because the OS needs
that long to report the exit. The 400 ms freshness watchdog bounds it in any case.
A faster detector would watch for missing messages (a short heartbeat from the
worker) rather than for process exit. That has not been built.

**Criterion change (decided 2026-10-03, after these runs).** The single gate "stop within
50 ms of the kill request" measured mostly the operating system. It would fail on this
laptop whatever the runtime did. It was replaced by three measurements:

| Measurement | Gate |
|---|---|
| Runtime reaction: OS-reported exit → stop | ≤ 50 ms (one control deadline). A negative value, where the runtime detected the death before the OS reported it, passes. |
| Platform detection: kill request → OS-reported exit | recorded, not gated |
| End to end: kill request → stop | ≤ 400 ms (the image-age limit, the outer bound) |

The runs above were judged under the old gate and keep their recorded failures. They
are not re-scored. Under the new gates, the `20261003-010746` measurements would pass
20/20.

**Confirmation run under the new gates:** `results/fault-injection/20261003-132018`
(laptop, harness `fault_injection.py` 83570808869e, `realtime.py` 3c9476102365):

| Scenario | Passed | Kill request → OS reports exit | Exit → stop | Kill request → stop | Detected by |
|---|---|---|---|---|---|
| Random kill | **10/10** | 45.2–77.5 ms | −14.8 to −7.2 ms | 34.5–62.7 ms | result pipe closed, 10 of 10 |
| Result read that never returns | **10/10** | 46.4–61.5 ms | 0.8–2.3 ms | 47.7–62.6 ms | `is_alive()`, 10 of 10 |

In the never-returning-read runs, the control loop ran 199–204 ticks while the read
was blocked. The watchdog paused at most 1.9 ms after the 400 ms limit. Shutdown left
the blocked receiver behind in every run, as designed.

Both paths are VERIFIED on the laptop under the new gates. The two detection routes
complement each other:
- **Pipe closed (receiver):** reacts before the OS reports the exit when the receiver
  is free to read.
- **`is_alive()` (per tick):** reacts within 2.3 ms of the OS-reported exit when the
  receiver is blocked.

**Repeat after a fresh restart:** `results/fault-injection/20261003-134000`, same
harness and runtime, run after restarting the laptop. Uptime is not recorded by the
campaign; the restart is as reported by the operator.

| Scenario | Passed | Kill request → OS reports exit | Exit → stop | Kill request → stop | Detected by |
|---|---|---|---|---|---|
| Random kill | **10/10** | 49.4–114.5 ms | −14.1 to −9.5 ms | 37.7–103.7 ms | result pipe closed, 10 of 10 |
| Result read that never returns | **10/10** | 46.7–106.3 ms | 0.8–1.9 ms | 47.5–107.8 ms | `is_alive()`, 10 of 10 |

The OS still took up to about 115 ms to report the exit, so the Windows delay is not
caused by a long uptime. The watchdog paused at most 1.3 ms after the limit.

The run before these, `20261003-131231`, used the previous harness because an update had
not reached the laptop. It is kept, with its failures under the old gate.

### Results of the first campaign (2026-10-01)

These results are from `results/fault-injection/20261001-001047`: cloud workspace,
default actuator profile, 5 repeats each. The speed clip is deterministic and ran
once. All 56 repeats passed. The source hashes of the run are in `summary.json`. After the run,
`fault_injection.py` changed only to mark the worker-crash scenario as partial coverage.

| Path | Requirement | Fault | Measured reaction (worst of 5) | Campaign status |
|---|---|---|---|---|
| Control-loop stall | REQ-03 | control thread blocked 150 ms while moving | `control_overrun` 2.9 ms after the stalled tick; arm stopped in 58 ms, camera travel 0.6 mm | VERIFIED |
| Stall just over the deadline | REQ-03 | blocked 55 ms | `control_overrun` 2.5 ms after the stalled tick | VERIFIED |
| Stall under the deadline (negative control) | REQ-03 | blocked 35 ms | no stop, no deadline miss (largest loop gap 38.5 ms); the alignment converged | VERIFIED |
| Physics-gap cap | REQ-08 | blocked 150 ms | the physics call after the stall was exactly 50 ms; 0.103 s counted as discarded | VERIFIED |
| Slow controller computation | REQ-03 | one computation 80 ms late | `control_overrun` 0.4 ms after it; its command never applied | VERIFIED |
| Clock step back | REQ-06 | runtime clock jumps back 1 s | `clock_error` 0.4 ms after the jump | VERIFIED |
| Failed inference | REQ-06 | every faulted frame returns `inference_failed` | stop at most 50.3 ms after the failed result arrived (50 ms transport delay plus the next tick); no command from it | PARTIALLY VERIFIED |
| Realtime collision block | REQ-15 | guard decision replaced by `blocked` | `collision_blocked` 0.5 ms after the block | PARTIALLY VERIFIED |
| Run limit | REQ-06, REQ-16 | `max_run_s` = 1 s | `run_timeout` 3.7 ms after the limit | VERIFIED |
| Sensor worker crash | REQ-06 | worker process killed | `worker_failed` stop at most 13.4 ms after the kill; runtime ends with its error report | VERIFIED in this run (overall NOT VERIFIED at the time, see finding 2; fixed 2026-10-02) |
| Watchdog stop (reference) | REQ-01 | sensor stalls 0.8 s | zero command 2.4 ms after the 400 ms image-age limit | VERIFIED |
| Speed clip | REQ-13 | 1.5 rad/s command on all joints | clipped to 0.600 rad/s; peak measured speed 0.592 rad/s | VERIFIED |

In every realtime scenario except the worker crash, which ends the runtime by
design:
- the runtime kept running;
- the command stayed zero after the stop;
- there was no unsafe or post-stop motion and no contact;
- the arm reached standstill.

The clearance above the margin never dropped below 3.0 mm to the environment or
9.0 mm between robot parts. No obstacle was placed. The 3.0 mm environment value is
the pedestal and the upper arm, which are 3.0 mm above their margin both at home and
at the start pose, so it is the same in every run.

**Why two paths are partial:**
- **Failed inference:** the failure is injected in the sensor worker. A real
  exception inside the Learned GPU matcher is not reproduced.
- **Collision block:** the guard's decision is injected. Geometric blocking is
  tested in the simulation (`tests/test_collision.py`) but not in the realtime
  runtime, and the search/recovery skip branch is not exercised.

**Statuses** follow the campaign report:
- **IMPLEMENTED:** code only.
- **TESTED:** an automated test exists, but there is no saved evidence.
- **VERIFIED:** all repeats passed, and the injected fault is the fault the
  requirement is about.
- **PARTIALLY VERIFIED:** all repeats passed, but the injection covers only part of
  the requirement.
- **NOT VERIFIED:** a repeat failed.

Before this campaign every path above was IMPLEMENTED only. The watchdog stop was the
exception: it already had tests.

### Findings

**1. The runtime crashed after a clock step. Fixed.** In the first campaign run, the
clock scenario issued the `clock_error` stop correctly and the command was zero.
Then the runtime crashed on the next tick, because a negative control gap was passed
to the physics step. A negative gap is now integrated as zero (`realtime.py`), and
the scenario passes 5/5.

After a backward clock step, the loop pauses once for about the size of the step,
and that time is counted in `discarded_physics_s`.

**2. A sensor-worker crash could freeze the control loop. Fixed on 2026-10-02.** In the
previous full run (log kept as `earlier-run-20261001-000218.txt`), 1 of 5 worker
kills left the control thread unable to shut down within 5 s. Neither the cause nor
the last command was captured.

The most likely cause is a platform behaviour, shown by the reproducer
(`reproduce_partial_message.py` and its output):
- The worker sends each result (an 0.9 MB image) through a `multiprocessing.Queue`.
- If the worker is killed in the middle of a message, the partial message stays in
  the pipe.
- The control thread's `results.get_nowait()` then blocks forever.

The watchdog, the deadline check and the physics all run in that same thread, so
nothing would stop the arm. In the simulation, physics would also stop and the arm
would stand still; on hardware, the last command would keep running. That is the
REQ-24 hazard.

The 40 further isolated kills and the final campaign's 5 did not hit that moment.

**The fix** (`realtime.py` `ResultReceiver`; [realtime control](../outputs/visual-servoing-simulation/REALTIME_CONTROL.md)):
- **Receiver thread.** It is the only reader of the results pipe. It forwards whole
  messages to a one-item, latest-wins mailbox, which the control loop reads without
  blocking.
- **Detection unchanged.** The control loop still detects the dead worker with
  `is_alive()` and stops with `worker_failed`.
- **Pipe ownership.** The parent closes its copy of the pipe's send end after
  starting the worker. Once the worker dies, a half-read message then ends with
  end-of-file instead of blocking.
- **Bounded shutdown.** Shutdown waits at most 0.5 s for the receiver. A receiver that
  is still blocked is left behind and recorded (`report()["receiver"]`).

The messages, the worker and the watchdog logic are unchanged. The receiver runs at
normal priority. The control thread keeps its above-normal priority on Windows.

**Regression evidence:**
- Both new scenarios froze the old runtime ("Control loop did not shut down within
  five seconds").
- On the fixed runtime they passed 35/35 each.
- An independent review of the change led to three corrections before the final
  runs:
  - a failing receiver now always delivers its error;
  - the worker's final error message is no longer dropped while the receiver holds
    the read lock;
  - receiver-mailbox drops are counted (`receiver_dropped`).
- `tests/test_realtime.py::ResultReceiverTests` covers the receiver on its own,
  without rendering:
  - latest-wins delivery;
  - end-of-file on a half-written message;
  - an error traceback that is never displaced;
  - a bounded close when a read never returns.

**Timing before and after** (`results/receiver-thread/20261002-timing`, cloud
workspace, SIFT, interleaved sessions):
- **Control loop:** loop-gap percentiles, deadline misses (1 before, 0 after),
  throughput and capture-to-command latency are unchanged within the variation
  between sessions.
- **Receive step on the control thread:** p99.9 fell from 1.57 to 0.036 ms.
- **CPU:** the receiver thread uses about 0.8% of one core, and the control thread
  about 1.8 points less.
- **Measured cost:** IPC p99 rose by about 0.4–1.1 ms.

That gives no measured reason to move images to shared memory. The freeze is fixed:
the targeted scenarios pass 35/35 each. The random-kill path had one invalid repeat
(below).

**3. Measured speed depends on the actuator model.** With the model off (`ideal`),
MuJoCo's velocity servo overshoots a step to the 0.6 rad/s cap and reaches
0.631 rad/s. With the default ramps it stays at 0.592 rad/s. The acceptance gate on
measured speed would therefore fail an `ideal` run that reached the cap. The
production default and the acceptance test use `default`.

**4. Stall motion is understated in the simulation.** The physics runs in the
control thread, so the simulated arm cannot move during a stall. The reported
stopping distances start after the stall. A real drive would keep executing the last
command during the stall: about speed × stall, for example 0.05 rad/s × 0.15 s =
7.5 mrad.

## Tests and CI

The tests are:
- `tests/test_safety_metrics.py` (9 tests);
- `tests/test_fault_injection.py` (15 tests: one per scenario, plus the settings check;
  the half-sent-frame test is skipped on Windows);
- `tests/test_realtime.py::ResultReceiverTests` (6 tests of the receiver thread, no
  rendering; the end-of-file test is skipped on Windows);
- the safety-envelope tests in `tools/test_acceptance_test.py` and
  `tools/test_margin_test.py`;
- `tests/test_actuator.py` and `tests/test_stop_response.py`.

The realtime fault scenarios start the sensor worker, which renders with OpenGL.
They are not suitable for the headless CI runner, so they run locally and through
the campaign.

The headless tests can run in CI. `.github/workflows/checks.yml` is protected, so add
these lines to the "Verify control mathematics" step by hand:

```yaml
          python -B -m unittest discover -s tests -p test_safety_metrics.py -v
          if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
          python -B -m unittest discover -s tests -p test_fault_injection.py -k SimulationFaults -v
          if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
          python -B -m unittest discover -s tests -p test_actuator.py -v
          if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
          python -B -m unittest discover -s tests -p test_stop_response.py -v
          if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
          python -B -m unittest discover -s tests -p test_realtime.py -k ResultReceiverTests -v
```

Put a `if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }` line between the current last
line (`test_binomial_ci.py`) and the new ones. Every line ends in ` -v`. A line that ends
in a lone ` -` makes unittest treat `-` as the start directory, and the step fails with
"Start directory is not importable: '-'".

## Limits

- **Two hosts.** The campaigns ran in the cloud workspace (Linux, 2 cores) and, from
  2026-10-03, on the laptop (Windows). Process-exit detection differs between them by
  a factor of about 10.
- **Small samples.** Five repeats per path (35 for the worker-failure paths) show the
  mechanism works under these conditions. They are not a failure-rate estimate.
- **Windows pipes.** The half-sent-frame scenario needs POSIX pipe framing, so on
  the laptop only the never-returning-read scenario and the random kill run. Python
  documents its Windows pipes as message-oriented, so the original freeze may not
  occur there at all; the fix does not depend on it.
- **Blocked receiver at shutdown.** If a read never returns, the receiver thread
  stays blocked after the session ends. That is one daemon thread per such event,
  which is harmless in the simulation, but it is not reclaimed until the process
  exits.
- **Low speeds.** Speeds at the fault were low (≤ 0.08 rad/s), because the faults
  are injected early in an alignment from the standard start offset, which only
  needs slow motion. The worst-case stopping distance is in
  [actuator model](ACTUATOR_MODEL.md).
- **Real hardware.** REQ-24 (actuator-side timeout) is not implemented. Nothing here
  replaces real hardware testing.
