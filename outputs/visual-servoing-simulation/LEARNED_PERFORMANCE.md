# Learned matcher performance — 19 September 2026

The original project now uses the NVIDIA GPU automatically when CUDA is available.
The camera status identifies **Learned GPU** or **Learned CPU**. The CPU path remains
available through the device setting; an explicit CUDA request fails clearly if
CUDA is unavailable.

This laptop has an AMD Ryzen 7 8845HS CPU and an NVIDIA RTX 3050 6 GB GPU, with
NVIDIA driver 577.02. The prior PyTorch installation was CPU-only. Profiling found
roughly 421 ms in SuperPoint extraction and 101 ms in matching at eight CPU threads.
One, two and four threads were slower here, so the CPU thread setting stays at eight.

The environment now contains PyTorch **2.13.0+cu126** and torchvision
**0.28.0+cu126**. Setup preserves an existing CUDA build, verifies real inference,
and offers explicit **--cuda** and **--cpu** options. Dependencies and backups
remain inside this original project; no source changes were made in the other
workspace during this performance task.

## Paired inference timings

Both devices ran the same images and settings in the same benchmark process.
Each image received three warm-up calls and 12 uncached timed calls. CUDA was
synchronized for timing. These measurements include upload, neural inference,
coordinate transfer and geometric checks; they exclude GUI rendering and startup.

| Image | CPU median, ms | GPU median, ms | Speedup |
|---|---:|---:|---:|
| saved camera | 436.8 | 57.1 | 7.65x |
| perspective | 434.6 | 54.7 | 7.94x |
| occluded | 425.1 | 55.1 | 7.71x |
| dimmed | 437.4 | 56.5 | 7.74x |
| blank | 325.0 | 13.6 | 23.85x |

Picture matching improved by approximately **8x**. The earlier CPU-only build
measured 509–537 ms on the same picture probes during the download; the table
uses the later paired CPU/GPU measurements for a controlled comparison.

Native camera images remain 640x480; the working image remains 384x288, the
keypoint limit remains 256, and all geometry/confidence gates retain their previous
values. GPU arithmetic produces slightly different matches, not identical outputs.
Independent GPU outline RMS errors were 1.13 px for the perspective view, 1.73 px
under occlusion, and 0.76 px under dimming. Both devices rejected the blank image.

[Raw paired benchmark](results/learned/performance-20260919/cpu-gpu/benchmark.json)
· [earlier CPU-only benchmark](results/learned/performance-20260919/cpu-before/benchmark.json)

## Actual switching and alignment

The real mouse-button callbacks switched SIFT to Learned, aligned from the offset,
and switched back to SIFT. The selected device was **cuda**. The test prohibited
SIFT/ArUco fallback and network downloads throughout the learned run.

- Full verification wall time: **25.06 s**, compared with **108.96 s** in the prior CPU verification. These totals include initialization, switching and saved screenshots; they are not pure inference timings.
- Initial RMS image error: **44.63 px**.
- Final and maximum post-stop error: **0.561 px**, below the unchanged 1 px threshold.
- Pose/time were preserved on switching, the saved goal remained byte-identical,
  and velocity commands stayed zero over 30 post-stop observations.

The first PyTorch/CUDA/model load still has startup overhead. Steady matching is
much faster; this GPU configuration does not promise a full 30 Hz application loop.

[Alignment evidence](results/learned/performance-20260919/gpu-alignment/summary.json)
· [GPU screenshot](results/learned/performance-20260919/gpu-alignment/learned-aligned.png)
· [prior CPU evidence](results/learned/performance-20260919/prior-cpu-alignment.json)

## Validation and use

**All 162 tests passed in 168.699 seconds**, with CUDA and CPU fallback exercised
and no skipped tests. This includes the existing geometry, offline inference,
wrong-target rejection, loss recovery, cancellation, teaching, controller and
search tests, plus device placement and installer flavor selection regressions.
`pip check` and `git diff --check` also passed.

[Full test log](results/learned/performance-20260919/full-tests.log)

Close an older simulator window and reopen **run.cmd**. Select Picture with **V**,
Learned with **K**, and use **O** (Auto on) or **G** to align. The status should say
**Learned GPU** on this laptop.

Reproduce timings with **benchmark_learned.py** and a new output directory. Reproduce
the button/alignment check with **verify_learned_repair.py --output <new-directory>**.
Use this project's **.venv/Scripts/python.exe**. Installation commands and primary
implementation references are in [the learned-matching guide](LEARNED_MATCHING.md#gpu-acceleration).
