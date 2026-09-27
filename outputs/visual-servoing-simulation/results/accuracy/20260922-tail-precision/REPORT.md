# Physical accuracy and calibration sensitivity

Reference-image refinement and local sensitivity checks are enabled. Stopping requires all configured pixel and estimated-camera-correction limits for the hold window, and every post-stop capture is rechecked. These estimates are not physical ground truth.

Completed 12/12 predeclared trials; 12 passed image convergence and 12 also met the declared physical tolerances.

Physical acceptance requires both camera and tool-frame position errors ≤ 2 mm and orientation errors ≤ 1° through 30 post-stop captures. These are illustrative experiment thresholds, not a certified robot specification.

| Matcher | Profile | Pixel / done | Physical / done | Camera mm | Tool mm | Angle ° | Time s | Paired Δtool mm |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| SIFT | nominal | 3/3 | 3/3 | 0.078 | 0.058 | 0.018 | 5.370 | 0.000 |
| SIFT | combined | 3/3 | 3/3 | 0.078 | 0.054 | 0.022 | 5.102 | -0.011 |
| Learned | nominal | 3/3 | 3/3 | 0.133 | 0.185 | 0.037 | 5.736 | 0.000 |
| Learned | combined | 3/3 | 3/3 | 0.066 | 0.059 | 0.024 | 5.670 | -0.117 |

Physical columns and time are medians over pixel-converged trials; all failures remain in the denominators and outcome counts below. Paired deltas use the same starting pose and matcher, where both nominal and perturbed cases passed pixel convergence. Negative Δ means a smaller error than nominal, not a statistical finding.

![Calibration comparison](sensitivity.png)

![Pixel error and tool-frame error](pixel-vs-physical.png)

## Outcomes

- SIFT / nominal: {'converged': 3}; pixel-only passes 0; safety failures 0; paired successes 3.
- SIFT / combined: {'converged': 3}; pixel-only passes 0; safety failures 0; paired successes 3.
- Learned / nominal: {'converged': 3}; pixel-only passes 0; safety failures 0; paired successes 3.
- Learned / combined: {'converged': 3}; pixel-only passes 0; safety failures 0; paired successes 3.

## Method and limits

The study pairs 3 seeded starts across every selected profile/matcher, at 100 ms simulated transport delay. Camera rendering, target geometry and collision supervision remain nominal. Only controller intrinsics, camera-mount Jacobian and assumed target size change.

Each target has a private reference capture at a known simulated teaching pose. The controller receives only the reference image and its detected features. Ground-truth camera and tool poses stay in evaluation records; saved application goals are preserved. Position is the Euclidean displacement of the defined frame origin; orientation is the SO(3) geodesic angle. Signed position components use the corresponding taught frame's axes. The tool frame is the tool_roll body origin, not an unmodelled gripper TCP.

Image and physical success are scored separately. All post-stop checks require zero commanded motion. Collision clearance and forbidden contacts are audited every physics step. Physical scores are evaluated at the stop and on fresh current captures, never on delayed PnP pose estimates.

These deterministic, small samples measure sensitivity for one static planar target and fixed scene. Their endpoint spread is across initial poses, not an ISO repeatability test. No real hardware, distortion, encoder error, moving-obstacle uncertainty, or change of true camera calibration after teaching is modelled. A perturbed control model can change transients without causing systematic endpoint bias because the image goal remains the same.

The [plan](plan.json), [manifest](manifest.json), [all outcomes](trials.json), [summary metrics](summary.json) and [evaluation goals](goals.json) are saved. Per-frame traces are generated locally and ignored by Git. Reports can be rebuilt from saved JSON without them.

Run from the project root:

~~~powershell
.\run.cmd --accuracy-study
.\run.cmd --accuracy-study --report-only <saved-run-directory>
~~~
