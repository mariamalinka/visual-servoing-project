# Physical accuracy and calibration sensitivity

Completed 72/72 predeclared trials; 64 passed image convergence and 48 also met the declared physical tolerances.

Physical acceptance requires both camera and tool-frame position errors ≤ 2 mm and orientation errors ≤ 1° through 30 post-stop captures. These are illustrative experiment thresholds, not a certified robot specification.

| Matcher | Profile | Pixel / done | Physical / done | Camera mm | Tool mm | Angle ° | Time s | Paired Δtool mm |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| ArUco | nominal | 2/2 | 2/2 | 0.527 | 0.496 | 0.065 | 4.469 | 0.000 |
| ArUco | focal-minus-5 | 2/2 | 2/2 | 0.425 | 0.482 | 0.103 | 4.420 | -0.013 |
| ArUco | focal-plus-5 | 2/2 | 2/2 | 0.690 | 0.671 | 0.044 | 4.520 | 0.175 |
| ArUco | principal-minus-5 | 2/2 | 2/2 | 0.493 | 0.464 | 0.070 | 4.486 | -0.032 |
| ArUco | principal-plus-5 | 2/2 | 2/2 | 0.548 | 0.514 | 0.059 | 4.469 | 0.019 |
| ArUco | mount-x-minus-10 | 2/2 | 2/2 | 0.546 | 0.515 | 0.062 | 4.469 | 0.020 |
| ArUco | mount-x-plus-10 | 2/2 | 2/2 | 0.499 | 0.475 | 0.069 | 4.486 | -0.020 |
| ArUco | mount-yaw-minus-5 | 2/2 | 2/2 | 0.565 | 0.552 | 0.066 | 4.470 | 0.056 |
| ArUco | mount-yaw-plus-5 | 2/2 | 2/2 | 0.521 | 0.495 | 0.072 | 4.436 | -0.001 |
| ArUco | size-minus-10 | 2/2 | 2/2 | 0.615 | 0.590 | 0.050 | 4.703 | 0.094 |
| ArUco | size-plus-10 | 2/2 | 2/2 | 0.461 | 0.438 | 0.074 | 4.352 | -0.057 |
| ArUco | combined | 2/2 | 2/2 | 0.588 | 0.556 | 0.062 | 4.303 | 0.060 |
| SIFT | nominal | 2/2 | 2/2 | 0.549 | 0.497 | 0.050 | 4.486 | 0.000 |
| SIFT | focal-minus-5 | 2/2 | 2/2 | 0.383 | 0.324 | 0.076 | 4.420 | -0.173 |
| SIFT | focal-plus-5 | 2/2 | 2/2 | 0.689 | 0.664 | 0.035 | 4.520 | 0.167 |
| SIFT | principal-minus-5 | 2/2 | 2/2 | 0.550 | 0.493 | 0.052 | 4.486 | -0.004 |
| SIFT | principal-plus-5 | 2/2 | 2/2 | 0.523 | 0.462 | 0.054 | 4.452 | -0.035 |
| SIFT | mount-x-minus-10 | 2/2 | 2/2 | 0.498 | 0.432 | 0.054 | 4.469 | -0.065 |
| SIFT | mount-x-plus-10 | 2/2 | 2/2 | 0.563 | 0.509 | 0.053 | 4.469 | 0.012 |
| SIFT | mount-yaw-minus-5 | 2/2 | 2/2 | 0.584 | 0.526 | 0.055 | 4.436 | 0.029 |
| SIFT | mount-yaw-plus-5 | 2/2 | 2/2 | 0.538 | 0.490 | 0.052 | 4.453 | -0.007 |
| SIFT | size-minus-10 | 2/2 | 2/2 | 0.713 | 0.687 | 0.038 | 4.669 | 0.190 |
| SIFT | size-plus-10 | 2/2 | 2/2 | 0.416 | 0.340 | 0.068 | 4.336 | -0.157 |
| SIFT | combined | 2/2 | 2/2 | 0.581 | 0.531 | 0.050 | 4.320 | 0.034 |
| Learned | nominal | 1/2 | 0/2 | 7.983 | 9.896 | 1.003 | 9.270 | 0.000 |
| Learned | focal-minus-5 | 1/2 | 0/2 | 8.201 | 10.201 | 1.046 | 14.370 | — |
| Learned | focal-plus-5 | 2/2 | 0/2 | 8.992 | 11.154 | 1.136 | 20.619 | 2.828 |
| Learned | principal-minus-5 | 1/2 | 0/2 | 8.961 | 11.130 | 1.135 | 10.836 | 1.234 |
| Learned | principal-plus-5 | 1/2 | 0/2 | 6.843 | 8.508 | 0.873 | 9.670 | -1.388 |
| Learned | mount-x-minus-10 | 2/2 | 0/2 | 7.453 | 9.250 | 0.940 | 8.836 | -0.463 |
| Learned | mount-x-plus-10 | 1/2 | 0/2 | 8.709 | 10.833 | 1.109 | 12.936 | — |
| Learned | mount-yaw-minus-5 | 1/2 | 0/2 | 8.034 | 9.939 | 1.016 | 26.270 | 0.043 |
| Learned | mount-yaw-plus-5 | 2/2 | 0/2 | 6.151 | 7.676 | 0.801 | 16.270 | -4.391 |
| Learned | size-minus-10 | 0/2 | 0/2 | — | — | — | — | — |
| Learned | size-plus-10 | 2/2 | 0/2 | 8.051 | 10.001 | 1.020 | 6.136 | -0.384 |
| Learned | combined | 2/2 | 0/2 | 8.622 | 10.683 | 1.076 | 12.436 | 1.396 |

Physical columns and time are medians over pixel-converged trials; all failures remain in the denominators and outcome counts below. Paired deltas use the same starting pose and matcher, where both nominal and perturbed cases passed pixel convergence. Negative Δ means a smaller error than nominal, not a statistical finding.

![Calibration comparison](sensitivity.png)

![Pixel error and tool-frame error](pixel-vs-physical.png)

## Outcomes

- ArUco / nominal: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / focal-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / focal-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / principal-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / principal-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / mount-x-minus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / mount-x-plus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / mount-yaw-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / mount-yaw-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / size-minus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / size-plus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- ArUco / combined: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / nominal: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / focal-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / focal-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / principal-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / principal-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / mount-x-minus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / mount-x-plus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / mount-yaw-minus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / mount-yaw-plus-5: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / size-minus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / size-plus-10: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- SIFT / combined: {'converged': 2}; pixel-only passes 0; safety failures 0; paired successes 2.
- Learned / nominal: {'timeout': 1, 'converged': 1}; pixel-only passes 1; safety failures 0; paired successes 1.
- Learned / focal-minus-5: {'converged': 1, 'timeout': 1}; pixel-only passes 1; safety failures 0; paired successes 0.
- Learned / focal-plus-5: {'converged': 2}; pixel-only passes 2; safety failures 0; paired successes 1.
- Learned / principal-minus-5: {'timeout': 1, 'converged': 1}; pixel-only passes 1; safety failures 0; paired successes 1.
- Learned / principal-plus-5: {'timeout': 1, 'converged': 1}; pixel-only passes 1; safety failures 0; paired successes 1.
- Learned / mount-x-minus-10: {'converged': 2}; pixel-only passes 2; safety failures 0; paired successes 1.
- Learned / mount-x-plus-10: {'converged': 1, 'timeout': 1}; pixel-only passes 1; safety failures 0; paired successes 0.
- Learned / mount-yaw-minus-5: {'timeout': 1, 'converged': 1}; pixel-only passes 1; safety failures 0; paired successes 1.
- Learned / mount-yaw-plus-5: {'converged': 2}; pixel-only passes 2; safety failures 0; paired successes 1.
- Learned / size-minus-10: {'invalid_depth': 1, 'timeout': 1}; pixel-only passes 0; safety failures 0; paired successes 0.
- Learned / size-plus-10: {'converged': 2}; pixel-only passes 2; safety failures 0; paired successes 1.
- Learned / combined: {'converged': 2}; pixel-only passes 2; safety failures 0; paired successes 1.

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
