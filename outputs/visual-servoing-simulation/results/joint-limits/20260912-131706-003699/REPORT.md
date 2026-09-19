# Joint-limit supervision evaluation

> Published snapshot: bulk per-frame traces remain local. See the [dataset policy](../../README.md) before regenerating this report.


The same 200 randomized target-present scenes and four negative controls
were evaluated. Baseline traces are copied from the completed, earlier startup
study; the supervised controller was run anew. Physics, camera, marker, search,
reference-image and recovery-budget compatibility were checked before the run.
Every sampled scene is retained. Initial detections and acquisition times match
within each pair.

| Result | Recorded baseline | Joint limits + bounded retry |
|---|---:|---:|
| Aligned and stable for 1 s after stopping | 196/200 | 198/200 |
| Initially undetected, then aligned | 118/122 | 120/122 |
| Scenes with an alignment retry | 0 | 2 |
| Converged after retry | 0 | 2 |

Improved trial IDs: [13, 167].
Regressed trial IDs: [].
Supervised outcomes: {'converged': 198, 'target_not_found': 2}.
Negative-control outcomes: {'target_not_found': 2, 'recovery_limit': 1, 'reposition_timeout': 1}.

![Joint limits and image error](joint_limit_recovery.png)

The original image-based controller and fixed gain 1.2/s are retained. Feasible
original joint commands are unchanged. When needed, bounded least squares
reallocates motion while outward speeds decrease within 25 degrees of a
0.06-radian joint margin. Actual joint positions allow a 0.005-radian actuator
tracking tolerance in validation; requested steps obey the configured bounds.

A two-second image-error window detects insufficient improvement above 3 px.
It compares the best error in each half of the window, requiring at least
1 px or 2% improvement, whichever is larger. Stalls or a 20-second alignment
timeout trigger at most one retry: brake, return toward the first observed
alignment view, confirm three detections, then realign with stronger joint
damping and a mild preference for central joint positions. This changes the
joint-motion calculation, not the desired image or IBVS gain.

The return is capped at 0.25 rad/s, 140 degrees per joint from its entry, and
12 seconds including confirmation. It is joint-feedback motion and can continue
without marker visibility while returning. It does not use a target world pose,
a taught goal joint pose, or teleportation. Startup initialization is the only reset.
The 45-second post-acquisition overall budget includes the return and retry;
each IBVS attempt still has a 20-second timeout. The new method can therefore
use more alignment time than a baseline that stops after its first timeout.

The two failures used during development are part of this regression distribution;
these results are not a held-out success estimate or a guarantee of arbitrary
workspace coverage. Search coverage is unchanged. Robot collisions remain disabled
in this teaching simulation; joint-limit handling is not collision-aware planning.

The baseline manifest, all source hashes, configurations, original plan, reference,
paired outcomes, per-frame NPZ traces and motion events are saved with this run.
Trace validation: PASS.

Reproduce: python run_joint_limit_study.py --baseline "PATH_TO_ORIGINAL_STARTUP_RUN"
Rebuild this report: python run_joint_limit_study.py --analyze "PATH_TO_THIS_RUN"
