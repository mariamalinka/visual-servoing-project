# Collision-aware motion and search

Collision checks are enabled by default for manual jogging, visual alignment,
startup search, remembered-view recovery, retries and demo motion. MuJoCo also
generates physical contacts for the robot.

## Try it

From the repository root:

~~~powershell
.\run.cmd --obstacle --cold-start --camera-delay-ms 100
.\run.cmd --learned --obstacle --cold-start --camera-delay-ms 100
.\run.cmd --collision-study
~~~

Press **U** or **Obstacle [U]** to add/remove the column. Changing the obstacle
stops motion and clears remembered views. **N** starts a search without a remembered
viewpoint; with Auto OFF, press **G** to begin. **L** demonstrates recovery after
a visible view has been observed. **C** and **X** retain their camera-delay and
stream-interruption controls.

The status row shows the guard state, monitored pair's clearance, detours planned
and search waypoints skipped. During limiting, the displayed clearance belongs
to the pair restricting motion; it is not necessarily the global minimum.
A blocked alignment or manual command stops and names the limiting pair.
Choose another path or starting view before pressing Align again.

Reset and preset buttons are explicit pose initialization, not planned motion.
A pose inside a clearance margin is rejected without moving the arm. Obstacle
placement is rejected atomically if it violates the current robot clearance.
Normal movement always uses velocity actuators and physics integration.

## How it works

[collision.py](collision.py) builds collision pairs from the scene's contact masks.
It checks the arm and wrist camera against the floor, pedestal, target board and
stand, optional column, and nonadjacent robot geometry. Rigidly attached shapes
and direct parent/child robot bodies are excluded, following the scene's MuJoCo
contact convention. The explicit base_shell/pedestal exclusion permits the fixed
mounting gap; other arm/pedestal pairs remain checked.

The guard owns kinematic scratch state. It uses measured joints and known scene
geometry to query signed separation and closest points, then forms distance
gradients from point Jacobians. It does not move the live robot or use mapped
target geometry as evidence that a target has been detected.

[collision_config.json](collision_config.json) defines the policy:

| Setting | Default | Purpose |
|---|---:|---|
| Environment clearance | 12 mm | Margin from scene objects |
| Nonadjacent self clearance | 6 mm | Margin between robot shapes |
| Influence distance | 80 mm | Start considering approach constraints |
| Velocity lookahead | 250 ms | Convert available separation to a closing-speed limit |
| Braking reserve | 120 ms × measured closing speed | Leave room for actuator deceleration |
| Detour offsets | 12°, 24° | Bounded extra joint motion for route candidates |
| Query budget | 2,048 per route | Reject a route when planning work is exhausted |

At every 2 ms physics tick, the guard checks the held command again. It slows the
requested joint direction uniformly, preserving a manual jog's axis. Retreating
motion remains possible while the pose is outside the margins. Commands reduced
below 2% of the requested norm are stopped and latched off until another command
is issued. Zero requests never generate escape motion. If the pose is already
inside a margin, nonzero commands fail closed. Mechanical joint-limit bounds
and the stale-camera watchdog also remain active.

## Checked search paths

[motion_path.py](motion_path.py) follows a cached sequence of checked waypoints.
Startup search, remembered-view returns, local recovery scans and joint-limit
repositioning all use it. The first candidate is a straight joint-space segment.
If blocked, the planner tries bounded three-segment routes:

1. Move one joint away from the start.
2. Traverse toward a similarly offset goal.
3. Move back to the requested goal.

Both endpoint clearance and the space between endpoints are checked. Adaptive
subdivision uses conservative joint-radius motion bounds to certify clearance
between samples. Uncertified segments fail closed when depth/query budgets are
exhausted. Every detour stays inside the caller's allowed joint workspace.
Startup search permits up to 24° extra motion in otherwise held joints, retaining
its original yaw/pitch coverage and overall deadline.

Unreachable search waypoints are skipped. At most four blocked waypoints are
processed in one camera update. If all available routes are blocked, search stops
with **collision_blocked**. Live detection interrupts search and brakes before
confirmation. The physics-tick guard stays active while following cached routes,
including between delayed camera deliveries.

## Validation

Run **run.cmd --verify** for the local regression suite. Collision tests cover
physical contacts, nonadjacent self geometry, unsafe reset/placement rollback,
safe endpoints with an unsafe connecting segment, dense detour verification,
planning-budget exhaustion, maximum-speed braking, newly inserted obstacles,
manual-axis preservation, detection priority, and bounded blocked-search failure.

The [nine-case experiment](results/collision/20260920-validation/REPORT.md) runs
ordinary alignment, cold search around the column, and remembered-view recovery
with ArUco, SIFT and Learned. All use 100 ms camera delay. It checks clearance,
contacts, joint limits and feedback age every physics tick, then requires 30 new
current images below 1 px and zero commands after convergence.

Each run creates a new directory with source/runtime fingerprints, all trial
outcomes, plots, app screenshots and local raw traces. It returns a failing exit
code if a case fails. Use **--modes aruco natural** without the optional Learned
runtime, or **--output <new-directory>** for an explicit destination.
Existing directories are never overwritten.

New [camera robustness runs](CAMERA_ROBUSTNESS.md) also record minimum collision
clearance slack and forbidden contacts. Historical studies retain their original
outcomes and fingerprints; create a new run for this revision.

The full regression run passed 216 tests. After the final Lost view correction,
18 collision tests and 28 existing recovery/automatic-start/gain tests passed.
That correction preserves previously delivered view memory with camera delay
and rejects an unsafe Lost view preset without first resetting through home.
[Verification records](results/collision/20260920-validation/VERIFICATION.md).

## Limits

This is a mapped static-scene collision layer for this six-hinge teaching arm.
Geometry and joint calibration are treated as known. The online velocity damper
is a local first-order policy with an empirical braking reserve; it is not a
formal guarantee for arbitrary speeds, moving obstacles, uncertain geometry or
real actuators. The finite detour family can miss a feasible path, and image-based
alignment can stop at an obstacle even when a more complex visual route exists.

Search still requires actual rendered target detections. It does not receive a
target pose from the map. These deterministic cases are regression evidence, not
a general success-rate estimate or a physical safety certification. Camera delay
remains simulated transport delay; host matching/planning time is separate.

Geometry API references: [MuJoCo signed geometry distance](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-geomdistance),
[point Jacobians](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac),
and [collision detection and pair filtering](https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection).

