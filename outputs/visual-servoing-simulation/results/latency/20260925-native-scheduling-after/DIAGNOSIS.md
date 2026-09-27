# Native scheduling investigation, 25 September 2026

The sustained stability requirement **failed**. The scoped scheduling change
reduced control misses from 10 to 0 for SIFT and from 14 to 2 for Learned GPU.
It did not fix reused-worker degradation: Learned passed 110/812 alignments,
with 700 freshness trips. SIFT passed 444/497, with 53 freshness trips, compared
with 454/498 and 34 previously. There is no demonstrated SIFT non-regression.
The [requested metric comparison](COMPARISON.md) includes all failed attempts.

## What changed

`native_scheduling.py` uses documented Windows HighQoS requests for the owned
sensor process and control thread. Only the control thread receives a relative
ABOVE_NORMAL priority of 1; the sensor retains normal base priority. The code
records API readback/errors and restores previous settings at shutdown. The
existing controller, inference math/model, worker reuse, warm-up, queues,
400 ms freshness limit and 50 ms control deadline are unchanged. No restart,
keepalive, affinity, power-plan change or GPU clock override was introduced.

The baseline's context switches recorded both relevant threads as Unimportant.
In the candidate native recording, all 21,839 observed control switch-ins and
all 16,338 sensor switch-ins were Important. Control dynamic priorities were
9–11, confirming that the request affected actual scheduling. This is separate
from the statistical evidence of fewer control misses; host background load
was not controlled by the experiment.

## Native control evidence

The baseline diagnostic captured a 75.629 ms control cycle. Its native timer
wait ended at trace time 46.1368514 s, but the thread did not run until
46.2000713 s: **63.2199 ms ready to run**, following **3.0564 ms waiting**.
The previous switch-out stack passed through Python, KernelBase and the kernel.
This directly establishes OS dispatch delay for this reproduced miss. It does
not establish that inference blocked the control owner. The process whose
context contains the timer-ready event is not necessarily responsible.

The two remaining control misses in the unprofiled candidate campaign have
direct garbage-collection evidence:

| Session | Cycle | Receive/deserialization | Recorded parent generation-2 collection |
|---|---:|---:|---:|
| Learned 002 | 116.924 ms | 112.675 ms | 111.280 ms on another parent thread |
| Learned 017 | 73.727 ms | 67.975 ms | 67.761 ms on the control thread |

The former had only 4.75 million control-thread CPU cycles in the long receive
interval; the latter recorded 62.5 ms thread CPU in both collection and receive.
No ETW trace was running during this statistical campaign, so precise native
wait/dispatch subdivisions for those two particular events are unavailable.
Their matching GC intervals support collection-induced pauses, not inference
blocking. A future GC change must explicitly bound retained memory and be
validated with this full campaign; globally disabling collection is not a
validated remedy.

## Reused GPU worker remains degraded

In the candidate native trace, a 319.884 ms frame spent approximately
302.980 ms running on a CPU, 16.606 ms ready and 0.299 ms in native waits.
Its extraction/matching CUDA stream spans were 147.040/168.209 ms. A second
303.241 ms frame had 260.025 ms running, 42.980 ms ready and 0.237 ms waiting.
CPU submission starvation alone cannot account for the complete slow frames.
CPU running time can include driver spinning, and CUDA stream spans include
device scheduling and host submission gaps; these are not execution-only GPU
measurements. Timeline alignment to Python has approximately 2 ms uncertainty.

The [separate clock recording](../20260924-native-learned-004/gpu-timeline.png)
again drops into a low-clock regime at about 44 s after the first alignment,
alongside repeated long frames. Of 289 samples within armed intervals, the
median SM clock was 247 MHz; GPU temperatures were at most 54 C. No sampled
thermal-slowdown or power-brake reason was active. Six samples reported a
software power cap, and 275 reported GPU idle. Samples include gaps between
submissions and cannot prove the cause of the low clocks. Both boundary power
snapshots were AC. These observations do not justify a thermal or power-brake
root-cause claim.

DxgKrnl hardware-queue submission/completion and context rundown events were
captured. The baseline slow-frame window contains multiple DmaPacket and
QueuePacket schemas with the same event name; payload length is needed to
interpret them. The installed legacy WPA GPU table does not expose CUDA HWS
execution, and the modern GPU Work table exported no usable rows for the
selected candidate interval. Thus GPU execution slowdown versus GPU scheduler
preemption remains unresolved. The trace is preserved for further decoding;
there is no claimed per-kernel scheduling attribution.

A later read-only NVAPI lookup found no explicit profile for the project's
Python executable and returned SETTING_NOT_FOUND for the power-management
entry in the base/global profile. Those values are represented as unavailable,
not zero or a known effective policy. No driver settings were changed.

Earlier probes already found stable model/template hashes, bounded allocator
usage, no allocator retry/OOM, and no graph fallback; changing streams,
removing recurring OpenGL rendering and changing CPU pool waits did not remove
the degraded regime. See the preserved
[earlier investigation](../20260923-reuse-after/DIAGNOSIS.md).

## Evidence and limitations

[Native evidence index](native-evidence.json),
[GPU clock/power summary](../20260924-native-learned-004/gpu-correlation.json),
[driver profile read](driver-profile.json),
[campaign event diagnosis](diagnostics.json), and
[verification](VERIFICATION.md) retain the evidence behind these conclusions.

Native recordings are diagnostic-only and excluded from campaign percentiles.
Their bounded circular buffers overwrote earlier events even though lost-event
counters were zero. Saving a trace also adds I/O; subsequent diagnostic outcomes
cannot be pooled with unprofiled results. Large system-wide ETLs, raw native
exports and per-frame campaign traces stay local and excluded from Git.

The repeatability goal remains unmet. This iteration retains the measured
scheduling improvement but does not declare the system ready for sustained
Learned control or claim that freshness failures were fixed.

## References for API semantics

- [Windows quality of service](https://learn.microsoft.com/en-us/windows/win32/procthread/quality-of-service)
- [SetThreadInformation](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-setthreadinformation)
- [SetProcessInformation](https://learn.microsoft.com/en-us/windows/win32/api/processthreadsapi/nf-processthreadsapi-setprocessinformation)
- [NVIDIA driver profile API](https://docs.nvidia.com/nvapi/group__drsapi.html)

These references explain API behavior, not the measured root cause.
