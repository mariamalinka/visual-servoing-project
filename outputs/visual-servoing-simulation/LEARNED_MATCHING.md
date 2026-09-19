# Learned image matching

The photograph can now be tracked using **SuperPoint + LightGlue**, alongside the
existing SIFT and ArUco options. Both neural networks use pretrained weights.
The application does not train them or teach them a target location.

## Try it

Close any older simulator window and reopen **run.cmd**.

1. Press **V** to select the photograph.
2. Press **K** or click **SIFT [K]** to select **Learned [K]**.
3. Press **O** for an offset start. Auto ON begins alignment.
4. Press **F** to inspect matched details in the template and current camera.
5. Press **N** to start outside the view with no remembered viewpoint.
6. Press **K** again to return to SIFT. V switches back to ArUco and remembers
   your selected picture matcher for the next switch.

Or launch it directly:

~~~powershell
.\run.cmd --learned
.\run.cmd --learned --cold-start
.\run.cmd --learned --random-start --seed 42
.\run.cmd --learned-study
.\run.cmd --verify
~~~

Switching matchers stops the old command, keeps the robot pose and selected gain,
clears runtime viewpoint memory, and reloads the **same saved picture goal**.
Auto ON then checks the current camera; Auto OFF leaves the robot stopped.
A failed model load keeps the previous mode and goal, with zero movement command.

SIFT and Learned share **reference/natural_goal.npz** and the canonical photograph.
Teach image [H] changes that shared image goal. ArUco retains its separate goal.

## How it works

**SuperPoint** is a neural network that locates distinctive points in an image and
describes the surrounding appearance. **LightGlue** compares those descriptions
and their context to decide which points correspond between the known photograph
and camera image. Their pretrained weights stay fixed during the simulation.

RANSAC then finds a perspective mapping supported by the matches. The existing
geometry checks reject weak, concentrated or inconsistent evidence and outlines
outside the frame. The accepted mapping estimates the picture's four boundary
corners, keeping stable feature identities for the original IBVS controller.

The controller compares current and desired image corners and generates bounded
joint velocities. If visual evidence disappears, the existing search/recovery
supervisor takes over. Learned matching provides a new way to recognize the
picture; the search does not rely on a location learned by the neural network.

The learned backend receives the canonical picture and RGB camera frames. It has
no access to simulator target coordinates, a visibility flag or a desired joint
pose. There is no hidden SIFT or ArUco fallback.

## Depth estimation and noisy features

Learned feature localization can make the inferred outline noisier than the
classical detector's outline. The existing analytic IPPE depth initializer can
then exceed its reprojection threshold even when a good square pose is nearby.

**control.py / estimate_depths** now refines an initially rejected finite estimate
with OpenCV's iterative least-squares pose refinement. The **same 3 px threshold**
and positive-depth checks still apply. Already-acceptable estimates follow their
original numeric path. PnP supplies depth to IBVS; it does not replace image error
with a target-pose error.

An independent noisy-square regression case and the actual learned alignment
tests verify this behavior.

## Important code

| File / function | Responsibility |
|---|---|
| learned_perception.py / LearnedImagePerception | Load verified pretrained models, prepare template features, run camera inference |
| learned_perception.py / _detect | Resize pixels, extract/match features, undo image scaling, discard weak/duplicate correspondences |
| perception.py / PlanarImagePerception.fit_outline | Shared RANSAC, spatial coverage, residual, shape and frame-boundary checks |
| app.py / make_perception | Lazily create the selected backend; normal startup does not import PyTorch |
| app.py / Lab.toggle_matcher and select_perception | Switch matcher, preserve the goal/pose/gain, cancel old motion and reset view memory |
| control.py / estimate_depths | Estimate square depths and refine only otherwise-rejected analytic estimates |
| learned_feature_config.json | CPU, image size, keypoint budget and matching/geometry thresholds |
| models/manifest.json | Official weight URLs and exact SHA-256 checksums |
| setup_learned.py and download_learned_models.py | Explicit dependency installation and verified model downloads |
| run_learned_study.py | Paired robot runs and independent image-transform probes |
| record_learned_demo.py | Exercise real GUI buttons and record alignment |

Template features are computed once per detector instance. The canonical image
remains 384Ă—384; extraction can resize it internally while returning coordinates
in its original pixel space. Camera extraction uses a smaller working image,
then maps points back to 640Ă—480 before geometric estimation. This keeps the
existing camera calibration and physical 0.24 m picture size meaningful.

Identical RGB frames reuse their inference result. A changed frame or failed
inference cannot reuse the previous accepted measurement. Displaying matches
does not advance the controller or count as another confirmation frame.

## Current configuration and checks

| Setting | Learned mode |
|---|---:|
| Native camera image | 640×480 |
| Working camera image | 384×288 |
| Template extraction image | 192×192 |
| Maximum requested keypoints per image | 256 |
| PyTorch CPU threads | 8 |
| Minimum geometric inliers | 12 |
| Minimum inlier fraction | 55% |
| Minimum template coverage | 18% |
| Maximum inlier reprojection RMS | 1.5 px |

The feature and outline gates retain the SIFT thresholds. The smaller extraction
images and keypoint budget reduce CPU work. The saved picture and goal image
keep their original resolutions.

**All 149 automated tests passed** in 291.053 seconds, with the optional learned
tests enabled. They cover independent geometry, partial occlusion, negative
images, cache invalidation, offline weights, no SIFT/ArUco fallback, cancellation,
matcher switching, shared teaching, tracking-loss recovery and the original
controller/search regressions. [Complete test log](results/learned/full-test-suite.txt).

## Installation and offline use

The optional runtime is installed in this project's existing virtual environment.
For another Windows x64 computer with Python 3.12, run **setup.cmd**, then
**setup-learned.cmd**. It installs the pinned CPU PyTorch, torchvision, Kornia and
LightGlue packages, downloads the two official weight files and verifies hashes.

The installer uses Windows extended paths for PyTorch's deeply nested license
files. It does not change Windows registry settings. Existing ArUco and SIFT
usage does not require these optional packages.

After setup, inference works offline. The GUI resolves upstream model requests
to verified local files and loads only tensor weights; it never downloads during
motion. Missing or changed weights produce an installation message.

The weight files are kept locally under **models/** and excluded from Git;
**models/manifest.json** and the setup scripts are retained for reproduction.
Dependency pins and archive hashes are in **requirements-learned.txt**.

## Limits and interpretation

This still recognizes **one known flat photograph**, not arbitrary objects.
Both methods control four inferred picture corners. Direct control of a changing
set of learned keypoints is a separate future step.

CPU inference is slower than SIFT and the application can run slower than real
time. The control loop still advances at 30 Hz in **simulated time**; an animation
played at simulated speed is not a claim about real-time CPU performance.

Matching confidence and small outline error do not establish true physical pose
accuracy. Independent image-transform probes assess localization separately.
Tests with altered images are distinct from closed-loop robot tests with moving
targets, variable lighting, latency or camera noise.

## Primary references and model provenance

- [Official LightGlue implementation and configuration](https://github.com/cvg/LightGlue),
  pinned to commit eb42fee2d71449efb0aa5c10549752b5d75384d8.
- [Official SuperPoint implementation](https://github.com/cvg/LightGlue/blob/eb42fee2d71449efb0aa5c10549752b5d75384d8/lightglue/superpoint.py).
- [OpenCV pose estimation and refinement](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).
- [Official PyTorch CPU installation versions](https://pytorch.org/get-started/previous-versions/).
- [Upstream licensing for code and pretrained weights](https://github.com/cvg/LightGlue#license).

The target photograph is the existing generated asset from the SIFT step.
[Its source and exact generation prompt](assets/natural-target-source.json) are
unchanged; no new picture was generated for this comparison.
