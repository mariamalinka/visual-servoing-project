# How the important code works

This project simulates a robot with six rotating joints and a camera attached to its wrist. The target is a printed square marker inside the simulated scene. The task is to move the camera until the marker looks like a previously saved reference view.

Everything runs on your computer. MuJoCo simulates the robot, its motors and the camera; OpenCV reads the resulting camera images.

## The main loop

The important sequence in compare_controllers.py, inside run_comparison_trial(), is:

~~~python
rgb = sim.image()
corners = sim.marker_corners(rgb)
sample = update_controller(controller, corners, sim.camera_jacobian(),
                           dt, sim.camera_pose())
sim.command_velocity(sample.velocity)
sim.advance(dt)
~~~

In plain language: **take a picture → find the marker → choose a movement → command the motors → simulate the movement → repeat**.

The actual function also checks stopping conditions and records data between these steps. There is one camera measurement every 1/30 simulated second; physics takes smaller steps at 500 Hz. Simulation time can run faster or slower than a real clock.

~~~mermaid
flowchart LR
    A[MuJoCo wrist-camera image] --> B[OpenCV marker corners]
    B --> C{Alignment method}
    C --> D[IBVS: pixel error every frame]
    C --> E[PBVS: estimated 3D pose every frame]
    C --> F[Look once: freeze destination at first frame]
    D --> G[Camera velocity]
    E --> G
    F --> G
    G --> H[Robot Jacobian and speed limits]
    H --> I[Six joint velocity commands]
    I --> J[MuJoCo motors and physics]
    J --> A
~~~

For the look-once method, later images feed the evaluator only. Joint positions still provide feedback for executing its original destination.

## 1. simulation.py: the robot and its camera

**Simulation.image()** asks MuJoCo to render what the moving wrist camera sees. The marker's position in this image really changes when the robot moves.

**Simulation.marker_corners()** runs OpenCV's ArUco detector on those pixels. If marker ID 7 is found, it returns four pairs of numbers: each corner's horizontal and vertical image coordinates. If detection fails, it returns None.

**Simulation.camera_intrinsics()** describes the camera lens mathematically. The matrix K contains focal lengths and image-center coordinates, allowing pixels to be related to directions through the lens.

**Simulation.camera_jacobian()** describes how the camera moves when each joint rotates. It is a 6-by-6 matrix:

~~~python
camera_velocity = J @ joint_velocity
~~~

The six camera components are three linear velocities and three angular velocities. Because the controller chooses the camera movement, we solve this relationship in the opposite direction to obtain joint commands.

**Simulation.command_velocity() / advance()** send joint speeds to the simulated motors and let physics move the robot. During alignment, the code does not teleport the joints to their destination.

**Simulation.camera_pose()** provides the camera position and orientation from the robot's current geometry. Look-once execution uses this as ideal robot forward kinematics. It does not read the target position from the simulator.

**scene.xml** defines the links, joints, motor limits, wrist camera, target and lighting. **config.json** sets the camera rate, detector settings and shared controller parameters.

## 2. control.py: our existing image-based controller

**IBVSController.update()** is the main decision-making function. It compares the currently observed marker corners with their reference positions. Its key equations are:

~~~python
error = (current - self.desired_normalized).ravel()
twist = -gain * damped_pinv(L, damping) @ error
~~~

- error: how far each observed corner is from its desired image position.
- L: the interaction matrix, which predicts how camera motion moves points in the image.
- twist: the requested camera movement: vx, vy, vz, wx, wy, wz.
- gain: how strongly to correct the error.

The internal control error uses normalized image coordinates. The displayed error uses pixels so it is easier to interpret.

**interaction_matrix()** builds L. Depth matters because a nearby point moves farther across the image than a distant point for the same sideways camera motion.

**estimate_depths()** estimates those depths from the marker's known size and its image corners. IBVS uses this depth estimate, but its feedback error is still the difference between image coordinates.

**damped_pinv()** solves the movement equations while limiting amplification near singular configurations, where some camera motions are difficult for the arm to make. Additional speed limits bound the actual command.

**ControlSample** packages the result of one controller update: status, measured pixel error, six joint speeds, camera twist and optional diagnostics.

## 3. baselines.py: the two added methods

### Pose-based feedback: PBVS

**estimate_marker_pose()** uses OpenCV solvePnP with IPPE_SQUARE to estimate the marker's position and orientation relative to the camera from its four image corners. This is an estimate from the rendered image, not the simulator's true target pose.

**inverse_pose()** reverses a position-and-orientation transformation. A transformation called T_A_B converts coordinates expressed in frame B into frame A.

**PoseController.update(), with method="pbvs",** does this on every frame:

1. Estimate the marker's current pose relative to the camera.
2. Combine it with the marker pose from the reference image.
3. Calculate how the camera needs to translate and rotate to recreate that reference.
4. Convert that displacement into a velocity command.

The key composition is:

~~~python
current_from_goal = current_from_marker @ self.marker_from_goal
~~~

Here, goal means the desired camera viewpoint relative to the marker.

**pose_error()** extracts the required camera translation and axis-angle rotation. The controller multiplies these by the shared gain.

PBVS uses image error for its stopping criterion, so all methods are judged against the same final image alignment. Its movement commands come from the estimated 3D displacement.

### Look once, then move

The same PoseController with method="open_loop" makes the initial visual estimate once and stores:

~~~python
self.world_from_goal = world_from_camera @ current_from_goal
~~~

This freezes the estimated camera destination in world coordinates. On later frames:

~~~python
current_from_goal = inverse_pose(world_from_camera) @ self.world_from_goal
~~~

The robot follows that fixed destination using its joint-derived camera pose. Later target images cannot change its destination, command or stop decision.

This is **open-loop with respect to vision**. It still uses joint feedback, as a robot normally would when executing a motion. It does not receive the taught home joints as an answer.

Once the frozen goal is reached within the configured position and rotation tolerances, the controller reports motion_complete. The evaluator independently checks whether the image is actually aligned. Reaching the predicted pose can leave an image error.

### Shared movement conversion

**joint_command()** applies the same camera speed limits, damped Jacobian inverse and joint speed limit used by IBVS. This prevents one controller from winning simply because it is permitted to move faster.

**make_controller() / update_controller()** provide a small common entry point for the three methods while keeping the existing IBVS implementation intact.

## 4. compare_controllers.py: the experiment

**run_comparison()** generates one experiment across all three methods. It saves a reference image, then runs every sampled starting pose with IBVS, PBVS and look-once. Each trial resets the same robot and target state.

The exact offsets are generated by the existing **benchmark.sample_plan()** and saved to plan.json before the trials. The seed makes the sequence reproducible. Initially invisible targets remain in the results.

**run_comparison_trial()** owns the observe/control/move loop. It:

- Starts from the declared joint offsets.
- Applies the selected controller's commands through MuJoCo.
- Logs image error, detected corners, requested speeds, actual joint movement and camera pose.
- Detects failure, timeout or completion.
- Checks the stopped image for a further second after a candidate success.
- Stops the robot and restores any target movement before returning.

The measured error is the RMS distance between corresponding current and reference corners. Roughly, 1 px means the corners differ by about one pixel on average.

The shared success test requires error below 1 px for 0.5 s, plus a further 1 s stopped with every measured frame still below 1 px.

**project_marker_visibility()**, imported from benchmark.py, reads scene geometry only to explain a failure: was the marker outside the camera view, or was it in view but not detected? Its answer never guides a controller.

The separate target-step demonstration shifts the target 20 mm sideways after 1 s. This scene change tests whether a controller reacts to new visual information. It is kept separate from the 200-pose static experiment.

## 5. analyze_comparison.py: turning measurements into results

**validate_run()** checks that the run finished, each starting pose has exactly one result per controller, starting measurements match, traces are consistent, commands respect limits and claimed successes have stopped-image evidence.

**summarize()** calculates success counts, confidence intervals, convergence times, final error and failure counts. It reports both all starts and initially detected starts.

**analyze()** writes summary.json, trials.csv, REPORT.md and the plots. It also reports timing on the subset of starts where all three methods succeeded, avoiding a misleading speed comparison between different successful groups.

The raw traces remain available as numeric NPZ files. Reports can be regenerated without running the robot again.

## 6. tests/test_baselines.py: checking the new work

These checks cover:

- Recovering a known marker pose from independently projected image points.
- Confirming that PBVS commands reduce both translation and rotation errors.
- Giving look-once two completely different sequences of later images and verifying identical commands.
- Stopping on initial tracking loss and PBVS runtime tracking loss.
- Speed limits and timeout.
- Refusing to report an unfinished experiment.
- Exact agreement between the new IBVS runner and the original benchmark trajectory.
- Closed-loop reaction to the moved target, and restoration of the scene afterwards.

The older tests continue to check the interaction matrix, robot Jacobian, detector regression, GUI alignment and target recovery.

## Running it

From the simulation project folder:

~~~powershell
.\run.cmd --compare
.\run.cmd --compare-report
.\run.cmd --verify
~~~

The first command runs the 200-pose comparison and three additional target-step demonstrations. The second rebuilds the latest report. The third runs the tests and scene/application checks.

Double-clicking run.cmd still opens the existing interactive simulator. Its Align button uses the image-based controller with remembered-view recovery; the three-controller comparison runs through the benchmark command.

**Read control.py first to understand the original controller; then baselines.py to understand what changes between alignment methods.**

Sources for the mathematical conventions: [Chaumette & Hutchinson, Visual Servo Control, Part I](https://web.mit.edu/amcp/OldFiles/drg/Chaumette_Part_I.pdf) and [OpenCV PnP documentation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html).


## Adaptive gain: the next step

The app now has a **Fixed gain / Adaptive gain [T]** button. It starts in fixed mode. Changing mode stops the current movement; start Align again to use the selected mode. The selected mode also works after remembered-view target recovery.

**adaptive_gain.py / GainPolicy** calculates the correction strength from the latest image error:

~~~python
gain = 0.8 + (2.4 - 0.8) * (1 - exp(-error_px / 8.0))
~~~

At zero error the scheduled gain is 0.8; for increasingly large errors it approaches 2.4. The transition is continuous. The values live in **gain_config.json**. Because the controller stops below 1 px, a nonzero scheduled gain at zero error does not cause motion.

**AdaptiveIBVSController.update()** measures the current RMS corner error, stores the scheduled gain in its private controller configuration, then calls the existing **IBVSController.update()**. This is the important design choice: adaptive mode uses the same interaction matrix, depth estimator, joint inverse, speed limits and stopping logic. It changes only the proportional gain.

This is an error-dependent gain schedule. It does not train a machine-learning model or estimate unknown robot parameters.

**app.py / Lab.toggle_gain()** first calls Stop, then replaces the IBVS controller inside the existing recovery wrapper. The wrapper's remembered viewpoint is retained. New commands are generated only after the user starts Align.

**run_gain_study.py** evaluates three policies on the same starting poses: fixed 1.2, adaptive 0.8–2.4, and fixed high 2.4. Including fixed high distinguishes adapting the gain from simply increasing it.

The trial loop is shared with **compare_controllers.run_comparison_trial()**, which now accepts an optional controller factory and records the gain at each frame. The existing three-method comparison retains its default controllers.

**trace_metrics()** measures projected overshoot across the goal and joint-command RMS while image error is between 1 and 5 px. The error norm itself cannot be negative, so its peak is not the same thing as crossing the desired feature configuration. Projected overshoot uses the initial feature-error direction; individual corners can cross their goals without this aggregate metric reporting overshoot.

**analyze_gain_study.py** validates every recorded gain against the declared policy, checks speed limits and success evidence, recalculates trace metrics and reports paired settling-time differences. It generates both a gain-policy plot and an actual error/gain trajectory.

The 10 new checks in **tests/test_adaptive_gain.py** include constant-gain equivalence to the old controller, malformed parameters, limits, tracking loss, overshoot arithmetic, paired statistics, safe mode switching and recovery without mouse events.

Use the launcher with **--gain-study** to run a fresh study, or **--gain-report** to regenerate the latest report. [Read the measured results](ADAPTIVE_GAIN.md).


## Startup search

The app now loads the desired image from reference/goal.npz instead of taking a new home reference at startup. The file has no robot joint positions. reference_image.py checks the camera and marker settings and detects the desired corners from those saved pixels.

When ReacquiringIBVS has no observed viewpoint, startup_search.py builds an expanding yaw/pitch path around the measured starting joints. StartupSearch.update checks every fresh detection while following the path, brakes on detection and requires three consecutive frames. Only then does recovery.py hand the current corners to the existing IBVSController.

Cold start [N] clears runtime viewpoint memory and loads an out-of-view demonstration pose. Launching run.cmd --cold-start sets that pose before the app reads a live camera image. Teach image [H] explicitly changes the persistent image goal.

[Full workflow, configuration and evaluation](STARTUP_SEARCH.md).


## Automatic startup and random starts

Lab now starts with auto_start=True. After loading the saved reference and observing only the current starting view, _start_if_auto calls align. The existing ReacquiringIBVS then selects visual alignment, startup search, or remembered-view recovery using its detections and runtime memory.

Auto also starts a run after Offset, Reset, Lost view, Cold start or Random. Cold start and Random clear remembered viewpoints. Random uses the existing benchmark's joint-offset distribution without checking visibility before accepting a sample.

There is no automatic restart in Lab.advance when the controller is idle. Stop, Pause, a completed run or a failure therefore remain stopped. The Auto [B] button can disable automatic starts; Align [G] still starts an explicit run. Existing manual-workflow tests pass auto_start=False, while test_automatic_start.py tests the default automatic behavior.


## Joint limits and stalled alignment

ReacquiringIBVS._align_visible now calls the existing IBVS controller and then JointLimitSupervisor.filter. Feasible joint commands pass through unchanged. Otherwise, velocity_bounds gradually reduces outward movement near the joint stops, and bounded_least_squares finds a camera-motion approximation inside the allowed joint speeds.

The supervisor also records image-error progress. A stall or an alignment timeout can request one return toward the first valid observed alignment view. ReacquiringIBVS handles that explicit repositioning state before confirming detections and resetting only the per-attempt IBVS timer. The overall timer continues.

The retry increases joint damping and adds a mild preference for central joint positions. It keeps the same reference image and selected gain policy. [Parameters, code responsibilities and evaluation](JOINT_LIMITS.md).

## Filling gaps in startup search

**startup_search.py** constructs the existing coarse rectangles first, then optional midpoint rectangles. The midpoint radii are calculated from the configured gaps; they are not coordinates learned from a failed target. Both passes stay centered on the measured starting joints and share the same excursion bounds.

**StartupSearch.update** continues checking each camera detection while moving. It brakes and confirms detections exactly as before. The new **stage** property describes coarse or refined search; **refinement_started_s** records when the extra pass begins. Neither starts a new timeout clock.

**startup_search_config.json** enables refinement and sets the combined search deadline to 180 simulated seconds. **app.py / Lab.advance** displays the stage. Auto still starts alignment immediately when the marker is already visible.

**run_search_coverage.py** compares against the saved 198/200 controller and also runs both policies on fresh scenes. It checks that every earlier successful scene keeps its original complete trace, and every earlier search miss keeps the original coarse prefix before adding movement.

[Explanation, tradeoffs and results](SEARCH_COVERAGE.md).


## Natural-picture alignment

**perception.py / NaturalImagePerception._detect** is the new perception core. It extracts SIFT descriptors from the canonical photograph and current RGB frame, rejects ambiguous or duplicate matches, then fits a RANSAC homography. A homography is the perspective mapping between two views of the same flat image. Geometric inliers must be numerous, distributed across the picture and consistent with the final fit.

The four output features are the projected picture boundary corners. They keep stable point identities even though the set of matched texture details changes. **control.py / IBVSController.update** therefore receives the same shape of input as before and uses the same error, gain, depth calculation and velocity limits. No learned matcher or variable-keypoint interaction matrix is introduced in this version.

**app.py / Lab.detect_target** routes camera frames to the selected backend. **Lab.toggle_perception** stops motion, validates the other saved image, swaps the board material and clears runtime view memory while keeping the measured robot pose. With Auto ON, the existing search/alignment flow then checks only the current view. **Lab.toggle_matches** changes the inspection display without taking controller steps.

**reference_image.py** gives each target its own compatible image goal. The natural goal also records the canonical picture hash. Neither goal stores robot joint positions. **simulation.py / set_target_mode** changes only the board material; perception does not receive its position.

**run_natural_image_study.py** supplies each backend to the shared startup-search trial loop and records matching evidence alongside the existing motion trace. The paired scenes, negative controls, source snapshots and stopped observations are retained.

[Controls, detailed thresholds, results and limitations](NATURAL_IMAGE.md).


## Pretrained learned matching

**learned_perception.py / LearnedImagePerception** replaces SIFT extraction/matching with pretrained SuperPoint and LightGlue. The auto device setting selects an available CUDA GPU, otherwise CPU. Both models, template features and input tensors stay on the selected device; only matched coordinates/scores return to CPU for geometry. Template features are extracted once; each changed camera frame supplies new matches. Resizing is reversed before geometric checks, so all controller features remain in the calibrated 640×480 camera coordinates.

**perception.py / PlanarImagePerception** contains the shared image cache, canonical target handling, RANSAC homography and rejection gates. The SIFT backend now inherits this common code. Learned inference cannot silently call SIFT or ArUco.

**app.py / Lab.toggle_matcher** selects SIFT or Learned with K. **select_perception** stops motion, validates the model and saved image, preserves the current joints and gain mode, then clears viewpoint memory. SIFT and Learned share the same physical picture and saved RGB goal.

**control.py / estimate_depths** adds an iterative refinement only when the initial analytic IPPE estimate exceeds the existing reprojection threshold. The same final residual and positive-depth gates remain mandatory. Estimates that already pass retain their original numeric path.

**run_learned_study.py** records a small paired robot comparison and independent perspective/brightness/blur/occlusion image probes. CPU wall time is recorded separately from simulated time.

[Usage, model installation, limitations and full explanation](LEARNED_MATCHING.md).
