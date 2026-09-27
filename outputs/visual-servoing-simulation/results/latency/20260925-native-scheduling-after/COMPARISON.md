| Method | Campaign | Alignment success | Freshness trips | Control misses | Processing p95 / p99 / max (ms) | Capture to command p95 / p99 / max (ms) |
|---|---|---:|---:|---:|---:|---:|
| SIFT | Before | 454/498 (91.2%) | 34 | 10 | 129.6 / 152.9 / 306.6 | 186.5 / 207.9 / 287.1 |
| SIFT | After | 444/497 (89.3%) | 53 | 0 | 125.7 / 156.3 / 284.4 | 181.2 / 208.7 / 288.7 |
| Learned GPU | Before | 119/796 (14.9%) | 663 | 14 | 239.7 / 298.7 / 382.2 | 236.4 / 268.5 / 375.1 |
| Learned GPU | After | 110/812 (13.5%) | 700 | 2 | 237.9 / 301.0 / 355.2 | 235.4 / 268.5 / 378.1 |

Processing includes every uncached active frame, including failed and late work. Capture-to-command latency covers accepted commands.

| Method | Campaign | First alignment processing p95 / p99 / max (ms) | Later alignment processing p95 / p99 / max (ms) |
|---|---|---:|---:|
| SIFT | Before | 127.4 / 156.3 / 189.7 | 129.8 / 152.5 / 306.6 |
| SIFT | After | 129.9 / 160.1 / 275.8 | 125.5 / 156.1 / 284.4 |
| Learned GPU | Before | 105.0 / 120.0 / 204.7 | 246.2 / 300.3 / 382.2 |
| Learned GPU | After | 125.6 / 147.8 / 242.9 | 242.2 / 302.2 / 355.2 |
