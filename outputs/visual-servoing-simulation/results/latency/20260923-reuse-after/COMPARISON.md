# Sustained worker reuse: before versus after

The failed 20260922 campaign is retained as the baseline. This comparison checks identical campaign plans, dependency versions, controller/calibration/scene/model inputs, and every raw trace checksum. Deliberate execution changes are listed in [comparison-audit.json](comparison-audit.json).

The plan alternates SIFT and Learned GPU, uses at least 12 two-minute sessions per method, targets 10,000 uncached active frames per method, and caps at 20 sessions. A session reuses one worker across explicit alignments. Transport is 50 ms, the capture-age budget is 400 ms, and the control deadline is 50 ms. Initialization now includes graph capture; the one-camera-image warm-up and all subsequent rearming procedures are unchanged.

## Outcomes

| Method | Campaign | Sessions | Aligned / attempts | First attempts | Later attempts | Freshness trips | Control misses | Maximum held age (ms) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| SIFT | Before | 20 | 480/487 | 20/20 | 460/467 | 1 | 6 | 405.204 |
| SIFT | After | 20 | 454/498 | 18/20 | 436/478 | 34 | 10 | 455.704 |
| Learned GPU | Before | 20 | 55/849 | 20/20 | 35/829 | 793 | 1 | 419.169 |
| Learned GPU | After | 20 | 119/796 | 19/20 | 100/776 | 663 | 14 | 412.277 |

## Processing latency

All values are milliseconds. All active frames include legitimate identical-pixel cache hits. Uncached frames, including failed and late work, are reported separately to expose inference tails.

| Method | Campaign | Cohort | Frames | Mean | p95 | p99 | p99.9 | Maximum |
|---|---|---|---:|---:|---:|---:|---:|---:|
| SIFT | Before | All active | 20204 | 67.535 | 115.520 | 131.081 | 154.395 | 232.340 |
| SIFT | Before | Uncached | 16195 | 84.183 | 117.697 | 133.377 | 155.389 | 232.340 |
| SIFT | Before | First, uncached | 724 | 79.892 | 115.479 | 132.862 | 143.925 | 145.817 |
| SIFT | Before | Later, uncached | 15471 | 84.384 | 117.749 | 133.419 | 155.878 | 232.340 |
| SIFT | After | All active | 17975 | 73.118 | 126.511 | 149.277 | 195.854 | 306.641 |
| SIFT | After | Uncached | 14297 | 91.852 | 129.636 | 152.914 | 201.491 | 306.641 |
| SIFT | After | First, uncached | 639 | 84.122 | 127.407 | 156.331 | 181.827 | 189.667 |
| SIFT | After | Later, uncached | 13658 | 92.214 | 129.755 | 152.460 | 202.099 | 306.641 |
| Learned GPU | Before | All active | 12053 | 91.199 | 239.330 | 299.571 | 334.737 | 369.704 |
| Learned GPU | Before | Uncached | 8687 | 126.435 | 261.987 | 312.376 | 337.424 | 369.704 |
| Learned GPU | Before | First, uncached | 617 | 95.531 | 129.931 | 141.038 | 152.873 | 156.882 |
| Learned GPU | Before | Later, uncached | 8070 | 128.798 | 266.565 | 313.446 | 339.586 | 369.704 |
| Learned GPU | After | All active | 14008 | 78.426 | 212.411 | 291.839 | 329.649 | 382.230 |
| Learned GPU | After | Uncached | 10506 | 104.469 | 239.688 | 298.735 | 334.157 | 382.230 |
| Learned GPU | After | First, uncached | 807 | 63.512 | 105.031 | 119.981 | 144.471 | 204.678 |
| Learned GPU | After | Later, uncached | 9699 | 107.877 | 246.164 | 300.308 | 334.769 | 382.230 |

## Capture to accepted command

| Method | Campaign | Commands | Mean (ms) | p95 | p99 | Maximum |
|---|---|---:|---:|---:|---:|---:|
| SIFT | Before | 19715 | 127.368 | 175.139 | 191.259 | 237.372 |
| SIFT | After | 17449 | 133.701 | 186.462 | 207.877 | 287.095 |
| Learned GPU | Before | 10911 | 137.557 | 245.442 | 275.079 | 375.412 |
| Learned GPU | After | 12987 | 128.756 | 236.374 | 268.545 | 375.063 |

These accepted-command distributions exclude rejected work. Processing distributions above include that work; the complete report also includes capture-to-delivery latency for all active results. The held-age maximum includes waiting until the next accepted command or a latched stop, so it can exceed 400 ms when the watchdog detects expiry.

## Bounded-work accounting

| Counter | SIFT before | SIFT after | Learned before | Learned after |
|---|---:|---:|---:|---:|
| Skipped busy acquisition slots | 34934 | 35712 | 38876 | 36711 |
| Replaced pending acquisition | 0 | 0 | 0 | 0 |
| Replaced/dropped transport results | 3 | 0 | 1 | 2 |
| Dropped result mailbox work | 0 | 0 | 0 | 0 |
| Cancelled pending transport on stop/reset | 1248 | 1257 | 1832 | 1657 |
| Expired requests | 1 | 0 | 0 | 0 |
| Expired results | 0 | 0 | 6 | 7 |
| Obsolete-generation results | 354 | 334 | 639 | 533 |
| Completed but unreceived | 0 | 0 | 0 | 0 |
| Requests not started | 0 | 0 | 0 | 0 |
| Unsafe motion ticks | 0 | 0 | 0 | 0 |
| Motion ticks after stop | 0 | 0 | 0 | 0 |
| Late after freshness stop | 0 | 27 | 653 | 575 |
| Late after control stop | 4 | 7 | 0 | 11 |
| All stops stayed latched | True | True | True | True |
| Late after any stop (includes convergence) | 4 | 36 | 653 | 586 |

Counters can overlap: an expired-request acknowledgement and the worker expiry counter describe the same event. Cancellation/obsolescence is expected after explicit stops; late completion must never reactivate motion.

## Interpretation and evidence

Read [DIAGNOSIS.md](DIAGNOSIS.md) for GPU clocks, CPU/submission measurements, the power-source confound, rejected alternatives, and native scheduling limits. [REPORT.md](REPORT.md) contains whole-session bootstrap intervals and per-session trends; [verification.json](verification.json) records validation.

Frame samples are correlated within a session. p99.9 has only about ten tail observations per 10,000 frames, and an observed maximum is not a guaranteed worst-case bound. Zero observed events do not establish a zero event rate.
