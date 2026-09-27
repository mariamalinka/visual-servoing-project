# Physical accuracy and calibration sensitivity

Reference-image refinement and local sensitivity checks are enabled. Stopping requires all configured pixel and estimated-camera-correction limits for the hold window, and every post-stop capture is rechecked. These estimates are not physical ground truth.

Completed 12/12 predeclared trials; 12 passed image convergence and 12 also met the declared physical tolerances.

Physical acceptance requires both camera and tool-frame position errors ≤ 2 mm and orientation errors ≤ 1° through 30 post-stop captures. These are illustrative experiment thresholds, not a certified robot specification.

| Matcher | Profile | Pixel / done | Physical / done | Camera mm | Tool mm | Angle ° | Time s | Paired Δtool mm |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| ArUco | nominal | 2/2 | 2/2 | 0.219 | 0.231 | 0.029 | 5.120 | 0.000 |
| ArUco | combined | 2/2 | 2/2 | 0.283 | 0.301 | 0.028 | 5.019 | 0.070 |
| SIFT | nominal | 2/2 | 2/2 | 0.072 | 0.063 | 0.023 | 5.436 | 0.000 |
| SIFT | combined | 2/2 | 2/2 | 0.075 | 0.056 | 0.024 | 5.269 | -0.007 |
| Learned | nominal | 2/2 | 2/2 | 0.229 | 0.299 | 0.044 | 5.536 | 0.000 |
| Learned | combined | 2/2 | 2/2 | 0.083 | 0.090 | 0.025 | 5.519 | -0.209 |

Physical columns and time are medians over pixel-converged trials; all failures remain in the denominators and outcome counts below. Paired deltas use the same starting pose and matcher, where both nominal and perturbed cases passed pixel convergence. Negative Δ means a smaller error than nominal, not a statistical finding.

![Calibration comparison](sensitivity.png)

![Pixel error and tool-frame error](pixel-vs-physical.png)

## Outcomes

- ArUco / nominal: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / combined: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / nominal: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / combined: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- Learned / nominal: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- Learned / combined: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.

## Method and limits

The study pairs 2 seeded starts across every selected profile/matcher, at 100 ms simulated transport delay. Camera rendering, target geometry and collision supervision remain nominal. Only controller intrinsics, camera-mount Jacobian and assumed target size change.

Each target has a private reference capture at a known simulated teaching pose. The controller receives only the reference image and its detected features. Ground-truth camera and tool poses stay in evaluation records; saved application goals are preserved. Position is the Euclidean displacement of the defined frame origin; orientation is the SO(3) geodesic angle. Signed position components use the corresponding taught frame's axes. The tool frame is the tool_roll body origin, not an unmodelled gripper TCP.

Image and physical success are scored separately. All post-stop checks require zero commanded motion. Collision clearance and forbidden contacts are audited every physics step. Physical scores are evaluated at the stop and on fresh current captures, never on delayed PnP pose estimates.

These deterministic, small samples measure sensitivity for one static planar target and fixed scene. Their endpoint spread is across initial poses, not an ISO repeatability test. No real hardware, distortion, encoder error, moving-obstacle uncertainty, or change of true camera calibration after teaching is modelled. A perturbed control model can change transients without causing systematic endpoint bias because the image goal remains the same.

The [plan](plan.json), [manifest](manifest.json), [all outcomes](trials.json), [summary metrics](summary.json) and [evaluation goals](goals.json) are saved. Per-frame traces are generated locally and ignored by Git. Reports can be rebuilt from saved JSON without them.

Run from the project root:

~~~powershell
.\run.cmd --accuracy-study
.\run.cmd --accuracy-study --report-only <saved-run-directory>
~~~
