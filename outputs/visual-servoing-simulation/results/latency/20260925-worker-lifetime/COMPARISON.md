| Worker / alignment group | Processing p95 / p99 / max (ms) | Freshness trips | Control misses | GPU SM clock for slow frames, median (MHz) | GPU states for slow frames |
|---|---:|---:|---:|---:|---|
| Fresh worker for every alignment | 132.6 / 149.8 / 197.2 | 1 | 0 | No in-frame sample | {} |
| Reused: first alignment | 131.7 / 147.8 / 183.8 | 0 | 0 | No in-frame sample | {} |
| Reused: later alignments | 275.0 / 314.0 / 358.3 | 352 | 0 | 95.0 | {'P3': 188, 'P0': 162, 'P5': 80} |

Processing excludes cache hits and retains failed/late frames. Every fresh alignment has a new worker; “later” for fresh refers to its position within the comparison block. GPU clock/state samples are observational.
