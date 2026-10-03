### Every control cycle recorded (cycle probe threshold 0 ms, same in both)

Sessions: 4 before, 4 after; each 60 s SIFT (natural), interleaved.
Cells: mean of per-session values (range across sessions).

| Metric | Before (control thread reads the pipe) | After (receiver thread) |
|---|---|---|
| Control-loop gap p50 (ms) | 2.01 (2.00–2.01) | 2.00 (2.00–2.00) |
| Control-loop gap p99 (ms) | 8.27 (8.00–9.04) | 7.95 (7.79–8.00) |
| Control-loop gap p99.9 (ms) | 11.53 (10.71–13.55) | 11.25 (10.72–11.64) |
| Control-loop gap max (ms) | 43.30 (37.17–52.93) | 42.72 (34.72–51.53) |
| Control deadline misses (> 50 ms) | 1 total (per session 1, 0, 0, 0) | 0 total (per session 0, 0, 0, 0) |
| Freshness-watchdog stops | 4 total (per session 3, 0, 1, 0) | 2 total (per session 0, 0, 1, 1) |
| Frames received per second | 9.13 (8.92–9.24) | 9.21 (9.17–9.28) |
| Alignments converged / attempts | 48 / 53 | 50 / 52 |
| IPC: worker finished → control loop has it, p50 (ms) | 2.55 (2.49–2.63) | 2.32 (2.30–2.36) |
| IPC p99 (ms) | 4.56 (4.06–5.22) | 4.99 (4.69–5.40) |
| IPC max (ms) | 6.12 (4.60–7.44) | 22.35 (7.89–35.56) |
| Capture to command p50 (ms) | 175.75 (172.44–178.16) | 174.73 (174.18–175.36) |
| Capture to command p99 (ms) | 220.76 (214.05–237.96) | 211.75 (207.77–216.50) |
| Capture to command max (ms) | 234.09 (220.36–262.15) | 222.52 (218.60–226.98) |
| Maximum feedback age (ms) | 402.32 (399.70–408.58) | 399.79 (393.72–404.91) |
| CPU: control thread (% of one core) | 30.80 (30.31–31.09) | 29.01 (28.66–29.17) |
| CPU: receiver thread (% of one core) | n/a | 0.79 (0.78–0.80) |
| CPU: whole control process (% of one core) | 31.22 (30.72–31.52) | 30.23 (29.89–30.39) |
| CPU: sensor worker process (% of one core) | 120.75 (116.45–122.66) | 121.79 (120.68–122.79) |
| Per tick: receive step wall p50 (ms) | 0.024 (0.024–0.024) | 0.003 (0.003–0.003) |
| Per tick: receive step wall p99.9 (ms) | 1.573 (1.326–2.114) | 0.036 (0.031–0.042) |
| Per tick: receive step wall max (ms) | 7.652 (4.842–15.406) | 10.142 (3.372–28.738) |
| Per tick: receive step CPU max (ms) | 5.003 (1.543–15.298) | 7.127 (0.445–26.726) |
| Per tick: busy time p50 (ms) | 0.61 (0.61–0.61) | 0.57 (0.56–0.58) |
| Per tick: busy time p99 (ms) | 5.35 (5.14–5.88) | 5.04 (4.91–5.15) |
| Per tick: busy time p99.9 (ms) | 8.57 (7.98–9.95) | 8.07 (7.60–8.35) |
| Per tick: busy time max (ms) | 32.47 (16.21–43.23) | 23.46 (11.58–32.61) |
| Per tick: wake-up lateness p99 (ms) | 4.32 (4.13–4.73) | 4.16 (4.03–4.26) |
| Per tick: wake-up lateness max (ms) | 16.53 (7.60–24.64) | 20.26 (7.91–49.64) |
| After only: pipe read + unpickle in receiver, p50 / max (ms) | — | 1.78 (1.74–1.79) / 13.83 (4.50–30.89) |
| After only: wait in local queue until the control loop takes it, p50 / max (ms) | — | 0.48 (0.46–0.51) / 10.70 (2.79–33.96) |
| After only: receiver mailbox drops / stuck at close | — | 0 / 0 of 4 |

Runtime errors: 0

### Stock diagnostics (cycle probe records spikes ≥ 10 ms only), as in the stress procedure

Sessions: 3 before, 3 after; each 60 s SIFT (natural), interleaved.
Cells: mean of per-session values (range across sessions).

| Metric | Before (control thread reads the pipe) | After (receiver thread) |
|---|---|---|
| Control-loop gap p50 (ms) | 2.00 (2.00–2.01) | 2.00 (2.00–2.00) |
| Control-loop gap p99 (ms) | 8.03 (7.79–8.29) | 7.91 (7.81–8.01) |
| Control-loop gap p99.9 (ms) | 10.95 (10.36–11.25) | 10.76 (10.32–11.28) |
| Control-loop gap max (ms) | 23.18 (14.06–36.36) | 24.45 (19.56–31.63) |
| Control deadline misses (> 50 ms) | 0 total (per session 0, 0, 0) | 0 total (per session 0, 0, 0) |
| Freshness-watchdog stops | 4 total (per session 1, 2, 1) | 3 total (per session 0, 2, 1) |
| Frames received per second | 9.23 (9.10–9.33) | 9.27 (9.23–9.29) |
| Alignments converged / attempts | 35 / 39 | 36 / 39 |
| IPC: worker finished → control loop has it, p50 (ms) | 2.53 (2.52–2.55) | 2.34 (2.33–2.36) |
| IPC p99 (ms) | 4.23 (3.78–4.56) | 5.30 (4.51–5.97) |
| IPC max (ms) | 6.97 (6.10–7.43) | 7.75 (7.03–8.66) |
| Capture to command p50 (ms) | 175.15 (174.11–176.99) | 173.91 (172.41–174.97) |
| Capture to command p99 (ms) | 214.97 (210.62–219.73) | 211.25 (207.68–216.28) |
| Capture to command max (ms) | 232.88 (220.34–254.55) | 231.89 (213.39–253.15) |
| Maximum feedback age (ms) | 403.74 (401.76–406.65) | 397.45 (387.67–402.61) |
| CPU: control thread (% of one core) | 30.16 (29.62–30.74) | 28.36 (28.20–28.52) |
| CPU: receiver thread (% of one core) | n/a | 0.80 (0.78–0.81) |
| CPU: whole control process (% of one core) | 30.56 (29.99–31.14) | 29.60 (29.43–29.77) |
| CPU: sensor worker process (% of one core) | 122.43 (121.90–122.92) | 121.57 (120.45–122.56) |
| After only: pipe read + unpickle in receiver, p50 / max (ms) | — | 1.80 (1.78–1.82) / 4.98 (4.48–5.41) |
| After only: wait in local queue until the control loop takes it, p50 / max (ms) | — | 0.50 (0.50–0.50) / 4.37 (3.51–5.12) |
| After only: receiver mailbox drops / stuck at close | — | 0 / 0 of 3 |

Runtime errors: 0

