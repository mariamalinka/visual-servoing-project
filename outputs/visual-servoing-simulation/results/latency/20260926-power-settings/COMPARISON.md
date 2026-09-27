| Worker / alignment group | Processing p95 / p99 / max (ms) | Freshness trips | Control misses | GPU SM clock for slow frames, median (MHz) | GPU states for slow frames |
|---|---:|---:|---:|---:|---|
| Fresh worker for every alignment | 79.5 / 118.8 / 172.6 | 2 | 0 | No in-frame sample | {} |
| Reused: first alignment | 66.1 / 74.1 / 79.8 | 0 | 0 | No in-frame sample | {} |
| Reused: later alignments | 86.8 / 180.7 / 356.9 | 38 | 0 | 97.0 | {'P3': 46, 'P5': 6} |

Processing excludes cache hits and retains failed/late frames. Every fresh alignment has a new worker; “later” for fresh refers to its position within the comparison block. GPU clock/state samples are observational.
