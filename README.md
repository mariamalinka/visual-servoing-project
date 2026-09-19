# Visual Servoing Simulation

A six-joint robot aligns its wrist-camera view with a saved reference image.
The project combines MuJoCo physics, OpenCV perception and visual feedback control,
with optional SuperPoint + LightGlue matching on CPU or NVIDIA GPU.

![Robot aligning from visual feedback](outputs/visual-servoing-simulation/results/alignment/alignment.gif)

## What it does

- Aligns an ArUco marker or one known flat photograph with a taught image.
- Compares image-based control (IBVS), pose-based control (PBVS), and a look-once baseline.
- Finds an initially unseen target and recovers after losing sight of it.
- Applies joint-limit supervision, bounded recovery and optional adaptive gain.
- Evaluates controllers on repeatable starting poses and records success and failure evidence.

This is a simulation project. No physical robot or camera is required.

## Run on Windows

Install **64-bit Python 3.12**, then run these commands from the repository root:

~~~powershell
.\setup.cmd
.\run.cmd
~~~

Choose **Offset [O]** to try automatic alignment. **V** switches between the marker
and photograph. **K** switches between SIFT and the optional Learned matcher.

To install Learned matching with NVIDIA acceleration:

~~~powershell
.\setup-learned.cmd --cuda
.\run.cmd --learned
~~~

Use `--cpu` instead of `--cuda` for CPU-only installation. Models are downloaded
during setup; matching works offline afterwards. The camera status identifies
**Learned GPU** or **Learned CPU**. See the
[installation and troubleshooting guide](outputs/visual-servoing-simulation/LEARNED_MATCHING.md).

The root launchers use the application in `outputs/visual-servoing-simulation/`.
Its existing launchers also work. The application stays at this path so local
environments, saved references and existing workflows keep working.

## Explore the project

| Location | Purpose |
|---|---|
| [outputs/visual-servoing-simulation/](outputs/visual-servoing-simulation/) | Application source, configuration, scene and launchers |
| [tests/](outputs/visual-servoing-simulation/tests/) | Mathematics, perception, recovery and integration checks |
| [assets/](outputs/visual-servoing-simulation/assets/) and [reference/](outputs/visual-servoing-simulation/reference/) | Required target images and saved image goals |
| [docs/](docs/README.md) | Documentation and experiment index |
| [results/](outputs/visual-servoing-simulation/results/README.md) | Selected reports, plots and reproducibility metadata |
| [tools/](tools/) | Repository validation |
| [.github/workflows/](.github/workflows/) | Automated repository and control-mathematics checks |

Local `work/`, `archive/`, virtual environments, model weights, caches and
unpublished experiment runs are ignored by Git. Historical ZIP exports are local
backups; they are not required to run the application.

## Results and verification

The most recent local verification passed **162 tests**, including Learned GPU
alignment and CPU fallback. On the tested RTX 3050 laptop, GPU picture matching
takes approximately **55–57 ms per image**, about **8 times faster** than the paired
CPU measurement. These are measured inference timings, not a promise of a
30 Hz application loop. [Timing and alignment evidence](outputs/visual-servoing-simulation/LEARNED_PERFORMANCE.md).

~~~powershell
.\run.cmd --verify
.\run.cmd --benchmark
.\run.cmd --compare
~~~

Benchmarks write new local results. Reports and a small representative alignment
trace are included in Git; bulk per-trial traces are generated locally.
[Experiments and reproduction](docs/README.md#experiments).

## Scope

The photograph matcher recognizes one known planar target. Robot collisions are
currently disabled, and simulation time can advance more slowly than wall time.
Pixel alignment does not establish physical positioning accuracy.
[Controller details and limitations](outputs/visual-servoing-simulation/README.md).

For development commands and the policy for generated files, see
[CONTRIBUTING.md](CONTRIBUTING.md).
