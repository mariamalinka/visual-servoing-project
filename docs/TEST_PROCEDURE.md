# Test procedure for timing measurements

The acceptance test, the long run and the latency-margin test measure timing on a
laptop. Background programs and power management change those timings a lot. For
example, the same margin test gave Learned GPU **+25 ms** of tolerated delay with
browser tabs open and **+50 ms** after a restart with them closed. Total CPU load
during the SIFT level was 48% before and 17% after. So every run that is reported
follows this procedure, and its results are stated as measured *under these
conditions*.

## Before every run

1. **Restart the laptop.** Start the test within an hour of the restart. This clears
   background load and any power state the laptop has entered.
2. **Close other programs,** including all browser tabs, chat and music apps,
   cloud-sync clients, games and IDEs. Leave only the terminal that runs the test.
   Don't use the laptop while the test runs.
3. **AC power.** Plug in the charger and keep it plugged in. The tests refuse to start
   on battery, and a run where power is lost is INVALID.
4. **Windows power mode: Best performance.** Settings → System → Power & battery →
   Power mode.
5. **Laptop maker's utility:** a fixed Performance/Turbo mode, not Auto, Balanced or
   Quiet.
6. **Ventilation:** lid open, on a hard surface, vents unobstructed.
7. **Check the setup first:** run the command with `--smoke`, then start the full run.

| Test | Command | Duration |
|---|---|---|
| Acceptance test | `.\acceptance-test.cmd` | about 30 min |
| Long run | `.\acceptance-test.cmd --long` | about 57 min |
| Latency margin | `.\margin-test.cmd` | about 15–20 min |

## What the tests check and record

At start-up the test prints this checklist. It warns if the computer has been running
for more than an hour or the Windows power mode isn't Best performance. Every report
has a **Test procedure** section showing:

- **Fresh restart:** time since the computer started, taken from Windows.
- **AC power** at every session start.
- **Windows power mode** at every session start.
- **Not checked automatically:** other programs closed, the laptop utility mode and
  ventilation. Note in your results whether you followed these.

The hardware telemetry (GPU clocks and throttle reasons, CPU frequency, thermal zone)
and the "low-performance period" markers show whether the laptop slowed down anyway.
Treat a run that shows such periods, or that breaks the procedure, as an observation
of those conditions, not as the representative result.

## Reporting results

State results with their conditions, for example: *"RTX 3050 Laptop GPU, fresh
restart, other programs closed, AC, Best performance: Learned GPU tolerated +50 ms
per frame (≈2.1× slower perception)."* Keep runs that didn't follow the procedure as
evidence of sensitivity to background load, not as the headline result.
