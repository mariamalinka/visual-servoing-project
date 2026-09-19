# Documentation

## Start here

- [Run the simulator and learn its controls](../outputs/visual-servoing-simulation/README.md)
- [Understand the source and control mathematics](../outputs/visual-servoing-simulation/CODE_GUIDE.md)
- [Install and troubleshoot Learned matching](../outputs/visual-servoing-simulation/LEARNED_MATCHING.md)
- [Measured CPU/GPU performance](../outputs/visual-servoing-simulation/LEARNED_PERFORMANCE.md)
- [Development and verification](../CONTRIBUTING.md)
- [Repository layout and local archives](REPOSITORY.md)

## Experiments

Run the commands below from the repository root. Each guide explains its method,
recorded results and limitations.

| Study | Guide | Reproduce |
|---|---|---|
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
