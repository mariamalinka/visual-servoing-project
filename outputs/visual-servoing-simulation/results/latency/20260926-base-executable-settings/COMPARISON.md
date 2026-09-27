| Worker / alignment group | Processing p95 / p99 / max (ms) | Freshness trips | Control misses | GPU SM clock for slow frames, median (MHz) | GPU states for slow frames |
|---|---:|---:|---:|---:|---|
| Fresh worker for every alignment | 107.4 / 118.6 / 143.6 | 0 | 0 | No in-frame sample | {} |
| Reused: first alignment | 107.3 / 115.9 / 125.8 | 0 | 0 | No in-frame sample | {} |
| Reused: later alignments | 107.6 / 115.8 / 130.8 | 0 | 0 | No in-frame sample | {} |

Processing excludes cache hits and retains failed/late frames. Every fresh alignment has a new worker; “later” for fresh refers to its position within the comparison block. GPU clock/state samples are observational.
