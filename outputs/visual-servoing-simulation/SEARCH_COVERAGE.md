# Finer startup-search coverage

If the camera has never seen the marker and the original scan finds nothing, the robot now makes a finer second pass automatically. Reopen **run.cmd** to load the change. Auto mode, Cold start and Random all use it.

## Why the two targets were missed

The old scan moved J1 (base yaw) and J5 (wrist pitch) around rectangles at 12, 24, 36 and 48 degrees from the measured starting pose. That covers paths within a box, but it does not look from every point inside the box.

An offline rendered-camera check found valid detections for both missed scenes inside that same box, between the old paths. The small visible regions were near yaw change +18 degrees and pitch change +6 or -6 degrees. Those diagnostic coordinates are saved with the evaluation; the controller never receives them.

## What the robot does now

1. Run the original **coarse** scan at 12, 24, 36 and 48 degrees.
2. If nothing is detected, move back toward the measured starting joint positions.
3. Run a **refined** scan at 6, 18, 30 and 42 degrees, filling gaps between the original rectangles.
4. Brake as soon as the camera detects the marker. Require three consecutive detections, then align using the existing image feedback and joint-limit handling.

The midpoint radii are calculated from the configured coarse rings. This gives a general second pass rather than a destination chosen for either failed scene. Both stages use current encoder readings and actual camera detections. They do not use target world coordinates, a home pose, or a saved target viewpoint.

The app message changes from **Startup search (coarse)** to **Startup search (refined)**. Stop, Pause, manual jogging and changing gain cancel either stage. Auto does not restart a stopped or failed run from idle.

## Time and motion limits

| Setting | Previous search | Updated search |
|---|---:|---:|
| Maximum J1/J5 change from the measured start | 48 degrees | 48 degrees |
| Joint-speed limit during search | 0.25 rad/s | 0.25 rad/s |
| Margin inside mechanical limits | 0.06 rad | 0.06 rad |
| Confirming detections | 3 consecutive frames | 3 consecutive frames |
| Total startup-search deadline | 90 s | 180 s |
| Search passes | 1 | 2 |

Both passes share the 180-second deadline. Search stops earlier when its complete path is exhausted. The other four joints hold their starting positions, moving inward only if needed for a joint margin.

A missing target now receives more search time. A target acquired on the original pass uses the original path and acquisition time. After acquisition, the separate 45-second alignment/recovery budget still applies, including any alignment retry.

## Measured results

The completed comparison recovered **both search misses**, improving the original scene set from **198/200 to 200/200 aligned starts**, with no regressions.

| Result | Original scan | With finer pass |
|---|---:|---:|
| Original scenes: aligned and stable after stopping | 198/200 | 200/200 |
| Original initially invisible targets: found and aligned | 120/122 | 122/122 |
| Fresh scenes: aligned and stable after stopping | 100/100 | 100/100 |
| Fresh initially invisible targets: found and aligned | 59/59 | 59/59 |

The two recovered targets were acquired after **91.33 s** and **90.47 s** of simulated time. Alignment finished at **96.67 s** and **95.43 s**, respectively. Both stopped below 1 px error and remained there for the following second.

The fresh scenes all acquired the marker during the coarse pass. They check preserved behavior; the observed extra coverage comes from the two known misses. Those misses were used during development, so 200/200 is a regression result and does not guarantee arbitrary target coverage.

Both absent-marker controls completed the refined path at about **141.6 s** and stopped. The unreachable-target controls also terminated within their bounds. Every scene acquired by the original scan retained its complete original motion trace; formerly missed scenes retained their original coarse prefix before the extra pass.

All **103 automated tests passed** (100 control/integration tests plus three reporting tests), and the scene/app acceptance check passed. An additional recorded app run recovered trial 56 using **Auto + adaptive gain**, ending below 1 px after a one-second stopped observation.

Read the [full comparison report](results/search-coverage/20260913-020201-974031/REPORT.md), [verification evidence](results/search-coverage/verification.json), or [recorded app result](results/search-coverage/demo/final.png).

![Search paths and paired results](results/search-coverage/20260913-020201-974031/coverage.png)

## Important code

| File / function | What it does |
|---|---|
| startup_search.py / StartupSearch.__init__ | Build the coarse waypoints, then optional midpoint rectangles within the same bounds. |
| startup_search.py / StartupSearch.update | Move using measured joints, inspect detections every frame, confirm them, and enforce the shared deadline. |
| startup_search.py / StartupSearch.stage | Report whether the current waypoint belongs to coarse or refined search. |
| startup_search_config.json | Set refine_after_coarse, ring radii, speed and the total search deadline. |
| app.py / Lab.advance | Display the search stage and use the normal Auto/cancellation flow. |
| run_search_coverage.py | Compare the recorded 198/200 controller with the update, then compare both policies on fresh scenes. |
| tests/test_search_coverage.py | Check old recorded commands, a narrow synthetic viewing gap, bounds, cancellation and both real missed-start regressions. |

A configuration without **refine_after_coarse**, or with it set to false, retains the original single-pass algorithm. Historical studies explicitly use their archived search settings.

## Reproduce the comparison

~~~powershell
.\run.cmd --coverage-study
~~~

This replays the original 200 target-present scenes and four negative controls against the recorded baseline. It also runs both policies on 100 fresh scenes with seed 20260913 and four further negative controls. The fresh plan is saved before results are inspected.

For custom counts or to regenerate a saved report:

~~~powershell
..\..\work\vservo-venv\Scripts\python.exe run_search_coverage.py --holdout-trials 100 --seed 20260913
..\..\work\vservo-venv\Scripts\python.exe run_search_coverage.py --analyze "PATH_TO_SAVED_RUN"
~~~

Results live under **results/search-coverage/**. The report includes every sampled scene, acquisition and alignment outcomes, exact-prefix checks, commands, joint feedback, stopped observations, reference image, configurations and source snapshots/hashes.

This is a finite refinement, so narrower visibility gaps, occlusion or inaccessible views may still cause a miss. The simulator remains a marker-based teaching scene with robot collisions disabled.
