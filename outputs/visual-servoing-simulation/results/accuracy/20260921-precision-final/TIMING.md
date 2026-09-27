# Near-goal processing cost

Eight saved near-goal images per matcher were measured twice with warmed models:
matcher only, then matcher plus precision refinement. Each call forced fresh
inference, so repeated-pixel caches did not distort the comparison. Rendering
and simulated camera transport delay are excluded.

| Matcher | Matcher-only median | With refinement median |
|---|---:|---:|
| SIFT | 44.9 ms | 78.0 ms |
| Learned GPU | 64.8 ms | 102.3 ms |

All 16 refined images passed registration. This small paired timing check shows
the extra computation near the goal, not a real-time guarantee. Outside the 8 px
refinement region the selected matcher remains on its normal acquisition path.
ArUco retains its existing subpixel detector. Timings depend on the machine and
load; camera transport delay still does not include this host processing time.

The physical improvement therefore has a per-frame processing cost. In the
72-trial experiment, median simulated stopping time among Learned image passes
fell from 11.603 s to 5.820 s; this is a different quantity from wall-clock frame
time and the successful-trial denominators differ (16 versus 24).

[Raw paired timings](refinement-timing.json) · [Saved camera-pose inputs](timing-input.json)

Reproduce from the repository root with the original runtime/models:

~~~powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B outputs/visual-servoing-simulation/results/accuracy/20260921-precision-final/timing-script.py
~~~

The script records its own hash, the input hash and the private reference hash.
Bulk experiment traces are not needed once the compact timing input is saved.
