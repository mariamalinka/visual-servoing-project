# Documentation

## Start here

- [Run the simulator and learn its controls](../outputs/visual-servoing-simulation/README.md)
- [Understand the source and control mathematics](../outputs/visual-servoing-simulation/CODE_GUIDE.md)
- [Install and troubleshoot Learned matching](../outputs/visual-servoing-simulation/LEARNED_MATCHING.md)
- [Measured CPU/GPU performance](../outputs/visual-servoing-simulation/LEARNED_PERFORMANCE.md)
- [Development and verification](../CONTRIBUTING.md)
- [Repository layout and local archives](REPOSITORY.md)

- [Camera delay and observation-age controls](../outputs/visual-servoing-simulation/CAMERA_DELAY.md)

## Experiments

Run the commands below from the repository root. Each guide explains its method,
recorded results and limitations.

| Study | Guide | Reproduce |
|---|---|---|
| Wall-clock latency and overload | [Runtime and timing](../outputs/visual-servoing-simulation/REALTIME_CONTROL.md) | **run.cmd --latency-study** |
| Physical accuracy and calibration | [Metrics and sensitivity study](../outputs/visual-servoing-simulation/PHYSICAL_ACCURACY.md) | **run.cmd --accuracy-study** |
| Collision-aware motion | [Guard and checked search](../outputs/visual-servoing-simulation/COLLISION_AWARE.md) | **run.cmd --collision-study** |
| Camera robustness | [Automated experiment](../outputs/visual-servoing-simulation/CAMERA_ROBUSTNESS.md) | `run.cmd --robustness` |
| Camera delay | [Timing study](../outputs/visual-servoing-simulation/CAMERA_DELAY.md) | `run.cmd --delay-study` |
| Baseline IBVS | [Benchmark](../outputs/visual-servoing-simulation/BENCHMARK.md) | `run.cmd --benchmark` |
| IBVS, PBVS and look-once | [Controller comparison](../outputs/visual-servoing-simulation/COMPARISON.md) | `run.cmd --compare` |
| Adaptive gain | [Gain study](../outputs/visual-servoing-simulation/ADAPTIVE_GAIN.md) | `run.cmd --gain-study` |
| Lost-view recovery | [Recovery](../outputs/visual-servoing-simulation/RECOVERY.md) | `run.cmd --recovery` |
| Unseen startup | [Startup search](../outputs/visual-servoing-simulation/STARTUP_SEARCH.md) | `run.cmd --startup-study` |
| Joint limits and retry | [Joint-limit study](../outputs/visual-servoing-simulation/JOINT_LIMITS.md) | `run.cmd --joint-study` |
| Finer search coverage | [Coverage study](../outputs/visual-servoing-simulation/SEARCH_COVERAGE.md) | `run.cmd --coverage-study` |
| Photograph matching | [SIFT study](../outputs/visual-servoing-simulation/NATURAL_IMAGE.md) | `run.cmd --natural-study` |
| SIFT and Learned | [Learned study](../outputs/visual-servoing-simulation/LEARNED_MATCHING.md) | `run.cmd --learned-study` |

The joint-limit and coverage studies use preserved baseline plans and metadata
and enforce their recorded runtime/source versions.

[Published evidence and raw-trace policy](../outputs/visual-servoing-simulation/results/README.md).

- [Precision stopping and pose sensitivity](../outputs/visual-servoing-simulation/PRECISION_STOPPING.md).

- [Perception tail latency: causes, bounded refinement and paired measurements](../outputs/visual-servoing-simulation/PERCEPTION_LATENCY.md).

[Sustained latency testing and spike diagnostics](../outputs/visual-servoing-simulation/LATENCY_STRESS.md).

[Reused Learned GPU workers: complete before/after stress campaign and remaining limits](../outputs/visual-servoing-simulation/results/latency/20260923-reuse-after/COMPARISON.md).

[Native scheduling validation: reduced control misses, unresolved reused Learned degradation](../outputs/visual-servoing-simulation/results/latency/20260925-native-scheduling-after/COMPARISON.md).

- [Automated acceptance campaign](ACCEPTANCE_CAMPAIGN.md): repeatable multi-pose, long-running SIFT/Learned GPU validation with predeclared pass/fail criteria.

- [Test procedure for timing measurements](TEST_PROCEDURE.md): conditions every reported acceptance, long or margin run must follow.

- [Latency-margin test](MARGIN_TEST.md): one command (`margin-test.cmd`, about 15–20 minutes) that measures how much extra per-frame perception latency SIFT and Learned GPU tolerate before alignment fails.

- [Requirements Verification and Safety Traceability](SAFETY_TRACEABILITY.md): every safety and acceptance requirement linked to its hazard, enforcing code, tests, latest evidence and status (verified, partially verified or not verified), with the known gaps.

- [Success rates and confidence intervals](STATISTICS.md): what the 95% intervals next to every success count mean, why 100% observed is not 100% reliability, how PASS/FAIL uses the lower bound, and how many trials a claim needs.

- [Actuator dynamics model](ACTUATOR_MODEL.md): simulated acceleration, deceleration and jerk limits between the velocity command and the servos; the difference between command stop latency, physical stopping time and stopping distance; measured stops, resumes and pause/resume cycles; the 400 ms watchdog requirement re-evaluated against physical motion; assumptions (not real-robot data).

- [Fault injection and the safety envelope](FAULT_INJECTION.md): clearance, joint-limit margin and joint speed recorded in every acceptance, margin, campaign and stress run with PASS/FAIL against REQ-11 to REQ-13, and a fault-injection campaign that triggers each safety stop on purpose and records the reaction.

- [Watchdog decision](WATCHDOG_DECISION.md): why a freshness-watchdog trip now holds and resumes the alignment instead of ending it, with the evidence and trade-offs. Stop remains a setting.

- [Acceptance test](ACCEPTANCE_TEST.md): one command (`acceptance-test.cmd`, about 30 minutes) for SIFT and Learned GPU: five starting poses with fresh workers, then a 12-minute reused-worker run per method, with a PASS/FAIL report.
