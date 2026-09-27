# GPU/power-settings rerun

Same eight 120-second reused-worker sessions and 32 interleaved fresh-worker controls. Code, dependencies, runner, warm-up, scene/model, 50 ms transport, 400 ms freshness and 50 ms control deadline match the preserved baseline. Statistics below concern reused workers only.

| Metric | Previous baseline | After settings change |
|---|---:|---:|
| Alignment success | 12/364 (3.3%) | 156/194 (80.4%) |
| Freshness trips | 352 | 38 |
| Processing p95 / p99 / max (ms) | 272.0 / 313.5 / 358.3 | 85.3 / 178.3 / 356.9 |
| First alignment processing p95 / p99 / max (ms) | 131.7 / 147.8 / 183.8 | 66.1 / 74.1 / 79.8 |
| Later alignment processing p95 / p99 / max (ms) | 275.0 / 314.0 / 358.3 | 86.8 / 180.7 / 356.9 |
| Slow-frame median SM clock (MHz) | 95.0 | 97.0 |

Material improvement, but not a stable fix. The first seven reused sessions passed 154/154 alignments with no freshness trips; the last passed 2/40 with 38 trips. Later-alignment maximum latency and slow-frame GPU clocks remained near baseline levels.

Uncached active-frame processing, retaining failed and late frames; first/later refer to alignment position within each reused worker. Slow threshold >=200 ms, GPU samples strictly within those processing intervals.

The slow-frame clock comparison uses 430 baseline samples and 52 rerun samples. No slow frame occurred during first alignments. The final two fresh-worker controls also hit freshness stops; the degraded period therefore was not confined to a single reused worker. Individual settings were not isolated experimentally. Different attempt counts result from the same fixed session durations: failed alignments stop sooner.

The complete audit and distributions are in POWER_COMPARISON.json and comparison.json; raw per-frame and GPU correlation data remain local under traces.
