# GPU-settings campaign comparison

Uncached active-frame processing; first/later means alignment position within each reused worker. Slow means >=200 ms; GPU clocks use samples strictly inside those processing intervals. All runs use eight 120-second reused sessions and 32 interleaved fresh-worker controls.

| Metric | 20260925-worker-lifetime | 20260926-power-settings | 20260926-base-executable-settings |
|---|---:|---:|---:|
| Alignment success | 12/364 (3.3%) | 156/194 (80.4%) | 184/184 (100.0%) |
| Freshness trips | 352 | 38 | 0 |
| Processing p95 / p99 / max (ms) | 272.0 / 313.5 / 358.3 | 85.3 / 178.3 / 356.9 | 107.6 / 115.8 / 130.8 |
| First alignment processing p95 / p99 / max (ms) | 131.7 / 147.8 / 183.8 | 66.1 / 74.1 / 79.8 | 107.3 / 115.9 / 125.8 |
| Later alignment processing p95 / p99 / max (ms) | 275.0 / 314.0 / 358.3 | 86.8 / 180.7 / 356.9 | 107.6 / 115.8 / 130.8 |
| Median SM clock during slow frames | 95 MHz | 97 MHz | No frames >=200 ms |

Code, dependency fingerprints, runner hash, test plan and telemetry queries are identical. The 400 ms freshness limit, 50 ms control deadline and 50 ms transport delay are unchanged. Raw traces remain local under ignored traces directories.

Different attempt counts reflect fixed session durations; failed alignments can stop sooner. The campaigns were sequential observations, not a randomized isolation of each driver or power setting. A passing campaign does not prove the absence of rare future failures.

The newest settings materially improved sustained repeatability in this campaign: all eight reused sessions passed (184/184 alignments), with zero freshness trips and no >=200 ms processing frames. First-versus-later processing latency remained comparable. Relative to the immediately preceding settings run, processing p95 increased from 85.3 to 107.6 ms, while p99 decreased from 178.3 to 115.8 ms and the maximum decreased from 356.9 to 130.8 ms. The final reused session passed 23/23, compared with 2/40 previously. All 32 fresh-worker controls also passed. Safety audits passed and AC was recorded at every session boundary.

Windows UserGpuPreferences was checked during the campaign: both the project .venv launcher and the underlying codex-primary-runtime Python executable had GpuPreference=2. This confirms the new preference was present; it does not isolate that setting from other time-varying system conditions as the cause of improvement.
