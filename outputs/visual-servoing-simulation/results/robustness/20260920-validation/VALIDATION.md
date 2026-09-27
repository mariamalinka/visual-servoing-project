# Robustness validation

The validation run completed **48 trials** using two seeded starting poses per
profile and matcher. The default experiment uses ten poses and runs 240 trials.

| Outcome | Trials |
|---|---:|
| Stable alignment | 31 |
| Stale-camera stop | 15 |
| Controller timeout | 2 |
| Safety violation or execution error | 0 |

All six specified long-outage checks stopped at the capture-age deadline,
observed capture restoration, and remained stationary. The other nine stale
stops occurred in convergence profiles and count as alignment failures.
Learned timed out in both 200 ms trials. Every matcher aligned in the zero-delay,
100 ms, jitter-only and brief-outage cases. Combined delay, jitter and loss
triggered stale-camera stops in all six cases.

These two poses are a validation sample, not an estimated stability envelope.
The default ten-pose run provides a larger repeatable experiment. Controller
gains and saved goal images were preserved.

The full local verification passed **200 tests** plus the scene/application check.
All 38 camera tests passed. An additional real interruption/resume check confirmed
that the saved trial was preserved, only the pending trial ran, completed runs
did not repeat trials, changed plans/source fingerprints were rejected, and
report-only regeneration succeeded.

The source, runtime, assets and reference fingerprints matched from plan creation
through completion. Recorded trace timestamps were monotonic, and nonzero commands
in the traces had feedback younger than 250 ms. The runner also checked these
conditions on every physics tick.

The regression suite overlapped the experiment. Host inference/wall timings are
diagnostics; the comparison uses simulated time.

[Detailed results and figure](REPORT.md) · [Machine-readable verification](verification.json)
· [Regression output](full-verification.txt) · [Camera test output](camera-tests.txt)
· [Interruption/resume verification](resume-verification.json)
