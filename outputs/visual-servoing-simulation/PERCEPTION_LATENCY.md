# Perception tail latency

SIFT and Learned now use bounded coarse-to-fine registration near the image goal.
The selected matcher still supplies a fresh target detection on every changed
image. The image/pose stopping tolerances, confidence gates, model weights,
feature-count limits and controller gains are unchanged.

```powershell
.\run.cmd --realtime --learned --camera-delay-ms 50 --max-camera-age-ms 400
.\run.cmd --latency-study --modes natural learned --profiles transport-50ms --repeats 3 --max-camera-age-ms 400
```

Press **O**, then **G**, for the standard alignment offset.

## Where the long frames came from

The original 277.5 ms SIFT and 227.5 ms Learned measurements did not include
internal stage traces. Three new baseline runs per method reproduced the same
class of long frames. Stage timing identified the shared ECC refinement as the
main source: slow reproduced refinements took roughly 150-221 ms, while their
feature matching stages took about 55-71 ms. GPU-to-host transfer was a fraction
of a millisecond in those examples. It was not the dominant cause.

An offline replay selects the eight slowest refinements and a spread of other
refined frames from each baseline run, reconstructing their recorded robot states
and rendering the same scene. It compared six schedules on 86 captured poses.
The former 45-iteration full-resolution refinement had replay maxima of about
220 ms for SIFT and 287 ms for Learned. A single-resolution 12-iteration shortcut
moved a Learned corner by about 0.6 px, so it was rejected.

The selected schedule uses up to **8 iterations at half resolution, then 12 at
full resolution**, retaining the same 1e-6 ECC convergence epsilon. The full
homography and original acceptance gates are retained. Replay refinement maxima
were about 67/75 ms; the largest corner difference from the old result was below
0.033 px. Those differences are relative to the old algorithm, not independent
physical ground truth. Separate image-warp and physical-accuracy checks validate
quality.

## What changed

- `precision.py` prepares the saved goal's pyramid, scale transforms and erosion
  kernel once. Each frame starts from its own detected homography, refines at the
  two resolutions and checks the original correlation and geometry thresholds.
  Resizing uses pixel-center transforms, including odd ROI dimensions.
- `perception.py` converts and scales all SIFT keypoints together and rounds
  candidate locations in batches. It preserves matching order, feature limits,
  geometric gates and original camera coordinates. The refiner can reuse the
  base detector's immutable image cache instead of retaining another full copy.
- `learned_perception.py` gathers matched points and scores on the device and
  copies one compact array to the CPU, replacing four separate transfers. There
  are no new CUDA-wide synchronization calls; the necessary final transfer still
  waits for completion. Upstream LightGlue's adaptive decisions are unchanged.
- `realtime.py` acquires a new state snapshot only when the worker can accept it.
  Busy 30 Hz capture slots are skipped. A copied state waiting over 50 ms, or
  already unable to satisfy the age budget after transport, is discarded before
  rendering and matching. Request/result mailboxes remain bounded to one item.
  Capture time always belongs to the actual snapshot; queued images are never
  assigned a newer timestamp.
- Readiness is acknowledged by result delivery. Closing a dedicated shutdown
  pipe wakes the worker through EOF, avoiding shared multiprocessing event locks.
  Forced termination during a wait exposed a dead-event notification hang; the
  regression suite now covers that case.

`Observation.stage_ms` and the runtime's sensor records distinguish preparation,
feature extraction/matching, geometry and image refinement. Normal Learned stage clocks
measure host submission/wait intervals. The opt-in sustained diagnostics also
record CUDA stream events, which include submission gaps and are not sums of
individual kernel execution times. The measured end-to-end interval includes
device completion.
Cached images are marked separately so an earlier long duration is not reported
as newly performed work.

## Evidence and interpretation

[Paired before/after measurements](results/latency/20260921-tail-optimization/REPORT.md)
report mean, p95, p99, maximum, deadline trips, expired results and skipped/dropped
work. The 50 ms transport and 400 ms budget are identical in both versions. Runs
are complete and include failures. The report distinguishes actual command ages
from sensor results that arrived after control stopped, and separates skipped
capture opportunities from already-acquired work dropped from a queue.

The final batch had no camera-age watchdog trips: SIFT aligned in 3/3 runs and
Learned in 2/3. One Learned run stopped on a 65.9 ms control-loop gap, with a
40.5 ms command-application interval; this remaining control-path spike is
retained in the report. Learned p95 also rose slightly, so not every percentile
improved. The perception changes do not establish uninterrupted real-time
control on this host.

The original [wall-clock validation](results/latency/20260921-validation/REPORT.md)
is preserved. Its source fingerprint refers to the previous implementation.
Repeated host measurements and bounded iteration counts do not establish a hard
worst-case execution time; operating-system or device stalls still trigger the
existing freshness watchdog.

Recreate the comparison after producing the two sets of trial JSON:

```powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B tools/compare_perception_latency.py BEFORE AFTER OUTPUT
```

The comparison tool verifies paired trial IDs, starting offsets, age/transport
budgets, runtime versions, scene, references and controller inputs. The replay tool
`tools/probe_ecc_tail.py INPUT OUTPUT_JSON` explores refinement schedules outside
the armed control loop.

## Sustained validation

Use the [longer stress-test workflow](LATENCY_STRESS.md) to measure repeated
alignments across multiple warmed worker sessions. It retains late results and
failed attempts, reports p99.9 processing latency, and provides stage/CPU/GC
evidence with confidence intervals based on whole sessions.

The [20-session-per-method campaign](results/latency/20260922-sustained/REPORT.md)
did not validate sustained operation within the 400 ms budget. SIFT aligned
480/487 times with one freshness stop and six control overruns; Learned aligned
55/849 times with 793 freshness stops and one control overrun. First alignments
passed 20/20 per method, exposing a substantial difference from reused-worker
operation. [Follow-up diagnosis](results/latency/20260922-sustained/DIAGNOSIS.md).

## Reused Learned worker follow-up

The worker now uploads uint8 images before GPU float32 conversion and replays
bounded fixed-shape LightGlue transformer graphs. Model settings and adaptive
stopping are unchanged; other shapes use an eager fallback. This removes
measured CPU conversion/thread-pool work and reduces host kernel submissions.

The [40-session comparison](results/latency/20260923-reuse-after/COMPARISON.md)
still fails sustained freshness: Learned aligned 119/796 times, with 663
freshness stops and 14 control misses. Uncached mean improved to 104.469 ms,
but p99 remained 298.735 ms and maximum reached 382.230 ms. SIFT also worsened
in the new campaign despite unchanged perception code. These observations
do not establish a stable GPU execution regime or a causal clock mechanism.
[State, profiler and clock evidence](results/latency/20260923-reuse-after/DIAGNOSIS.md).

The latest [native scheduling comparison](results/latency/20260925-native-scheduling-after/COMPARISON.md)
completed 40 sustained sessions. Control misses decreased, but Learned GPU
tail latency and repeated freshness stops remain unresolved; SIFT freshness
non-regression was not demonstrated. [Detailed evidence](results/latency/20260925-native-scheduling-after/DIAGNOSIS.md).
