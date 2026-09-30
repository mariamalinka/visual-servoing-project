"""Physical pose scoring, paired calibration plans and portable experiment reports."""
from __future__ import annotations
from collections import Counter
import json
from pathlib import Path
import re
import numpy as np
from binomial_ci import clopper_pearson, format_rate
from calibration import ControlCalibration, CONFIG_PATH
from camera_robustness import NAMES, write_json

ROOT = Path(__file__).resolve().parent


def checked_pose(pose):
    T = np.asarray(pose,dtype=float)
    if T.shape != (4,4) or not np.isfinite(T).all():
        raise ValueError("Pose must be a finite 4x4 rigid transform")
    R = T[:3,:3]
    if not np.allclose(T[3],[0,0,0,1],atol=1e-9,rtol=0) or not np.allclose(R.T@R,np.eye(3),atol=1e-7,rtol=0) or not np.isclose(np.linalg.det(R),1,atol=1e-7,rtol=0):
        raise ValueError("Pose must contain a proper rotation and homogeneous last row")
    return T


def pose_accuracy(current, goal):
    """Position in the taught frame and SO(3) geodesic angle, not Euler subtraction."""
    current, goal = checked_pose(current), checked_pose(goal)
    delta = goal[:3,:3].T@(current[:3,3]-goal[:3,3])
    R = goal[:3,:3].T@current[:3,:3]
    sine = .5*np.linalg.norm([R[2,1]-R[1,2], R[0,2]-R[2,0], R[1,0]-R[0,1]])
    cosine = np.clip((np.trace(R)-1)/2,-1.,1.)
    angle = float(np.rad2deg(np.arctan2(sine,cosine)))
    return dict(translation_error_mm=(1000*delta).tolist(),
                position_error_mm=1000*float(np.linalg.norm(delta)), orientation_error_deg=angle)


def physical_pass(camera, tool, position_mm, orientation_deg):
    return all(score["position_error_mm"] <= position_mm and score["orientation_error_deg"] <= orientation_deg
               for score in (camera,tool))


def make_plan(config, modes=None, starts=None, seed=None, profiles=None):
    allowed = {"seed","starts","offset_limit_degrees","camera_delay_s","max_trial_s",
               "post_stop_frames","position_tolerance_mm","orientation_tolerance_deg","profiles"}
    if set(config) != allowed:
        raise ValueError(f"Unexpected/missing experiment settings: {sorted(set(config)^allowed)}")
    count = config["starts"] if starts is None else starts
    seed = config["seed"] if seed is None else seed
    if type(count) is not int or not 1 <= count <= 10000:
        raise ValueError("Starts must be an integer in [1, 10000]")
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("Seed must be an integer in [0, 2**32)")
    modes = list(NAMES) if modes is None else list(modes)
    if not modes or len(set(modes)) != len(modes) or not set(modes) <= set(NAMES):
        raise ValueError("Choose unique modes from aruco, natural, learned")
    limits = np.asarray(config["offset_limit_degrees"],dtype=float)
    if limits.shape != (6,) or not np.isfinite(limits).all() or np.any(limits<=0):
        raise ValueError("Provide six positive finite offset limits")
    for key in ("max_trial_s","position_tolerance_mm","orientation_tolerance_deg"):
        value = config[key]
        if isinstance(value,bool) or not np.isscalar(value) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if type(config["post_stop_frames"]) is not int or config["post_stop_frames"] < 1:
        raise ValueError("Post-stop frames must be a positive integer")
    delay = config["camera_delay_s"]
    if isinstance(delay,bool) or not np.isfinite(delay) or not 0 <= delay < .25:
        raise ValueError("Camera delay must be in [0, 0.25) seconds")
    available = {}
    for p in config["profiles"]:
        if set(p) != {"name","calibration"} or not isinstance(p["name"],str) or not re.fullmatch(r"[a-z0-9-]+",p["name"]) or p["name"] in available:
            raise ValueError("Profiles need unique lowercase names and calibration settings")
        available[p["name"]] = ControlCalibration(p["calibration"]).settings
    if available.get("nominal") != ControlCalibration().settings:
        raise ValueError("Include an unperturbed nominal profile")
    chosen = list(available) if profiles is None else list(profiles)
    if "nominal" not in chosen or len(set(chosen)) != len(chosen) or not set(chosen) <= set(available):
        raise ValueError("Choose unique configured profiles, including nominal for paired comparison")
    selected = [dict(name=n,calibration=available[n]) for n in available if n in chosen]
    offsets = np.random.default_rng(seed).uniform(-limits,limits,size=(count,6)).tolist()
    trials = []
    for mode in modes:
        for p in selected:
            for case, offset in enumerate(offsets):
                trials.append(dict(id=f"{mode}-{p['name']}-{case:04d}", mode=mode,
                    profile=p["name"], case=case, offset_degrees=offset,
                    calibration=p["calibration"],
                    perception_seed=int(np.random.SeedSequence([seed,case]).generate_state(1)[0]%(2**31))))
    return dict(schema_version=1, seed=seed, starts=count, modes=modes, profiles=selected,
                camera_delay_s=delay, max_trial_s=config["max_trial_s"],
                post_stop_frames=config["post_stop_frames"],
                position_tolerance_mm=config["position_tolerance_mm"],
                orientation_tolerance_deg=config["orientation_tolerance_deg"], trials=trials)


def validate_rows(plan,rows):
    expected = {t["id"]:t for t in plan["trials"]}
    seen = set()
    for row in rows:
        key = row["id"]
        if key in seen or key not in expected:
            raise ValueError("Duplicate or unknown trial result")
        if any(row[k] != expected[key][k] for k in ("mode","profile","case")):
            raise ValueError("Trial metadata differs from plan")
        seen.add(key)


def summarize(plan, rows):
    validate_rows(plan,rows)
    baseline = {(r["mode"],r["case"]):r for r in rows if r["profile"]=="nominal"}
    groups = []
    for mode in plan["modes"]:
        for profile in plan["profiles"]:
            group = [r for r in rows if r["mode"]==mode and r["profile"]==profile["name"]]
            converged = [r for r in group if r.get("pixel_success")]
            pairs = [(r,baseline[(mode,r["case"])]) for r in group
                     if r.get("pixel_success") and baseline.get((mode,r["case"]),{}).get("pixel_success")]
            def values(frame,metric,selected=converged):
                return [r["final_accuracy"][frame][metric] for r in selected if r.get("final_accuracy")]
            def percentile(values,q=50):
                return float(np.percentile(values,q)) if values else None
            item = dict(mode=mode,profile=profile["name"],planned=plan["starts"],completed=len(group),
                        pixel_successes=len(converged),physical_successes=sum(r.get("physical_success",False) for r in group),
                        physical_success_ci95=clopper_pearson(sum(r.get("physical_success",False) for r in group),len(group)),
                        pixel_only=sum(r.get("pixel_success",False) and not r.get("physical_success",False) for r in group),
                        outcomes=dict(Counter(r["outcome"] for r in group)),
                        safety_failures=sum(bool(r.get("safety_violations")) for r in group),
                        median_success_s=percentile([r["completion_s"] for r in converged]),
                        paired_successes=len(pairs),
                        paired_median_time_delta_s=percentile([r["completion_s"]-b["completion_s"] for r,b in pairs]))
            for frame in ("camera","tool"):
                item[frame] = dict(
                    median_position_mm=percentile(values(frame,"position_error_mm")),
                    p95_position_mm=percentile(values(frame,"position_error_mm"),95),
                    median_angle_deg=percentile(values(frame,"orientation_error_deg")),
                    p95_angle_deg=percentile(values(frame,"orientation_error_deg"),95),
                    paired_median_position_delta_mm=percentile([r["final_accuracy"][frame]["position_error_mm"]-
                                                               b["final_accuracy"][frame]["position_error_mm"] for r,b in pairs]),
                    mean_translation_error_mm=np.mean(values(frame,"translation_error_mm"),axis=0).tolist() if converged else None)
            groups.append(item)
    return groups


def report(directory):
    directory = Path(directory)
    plan = json.loads((directory/"plan.json").read_text())
    rows = json.loads((directory/"trials.json").read_text())
    groups = summarize(plan,rows)
    write_json(directory/"summary.json",groups)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    profiles = [p["name"] for p in plan["profiles"]]
    fig, axes = plt.subplots(2,2,figsize=(max(12,len(profiles)*1.1),8),constrained_layout=True)
    metrics = (("camera","median_position_mm","Camera position (mm)"),
               ("tool","median_position_mm","Tool-frame position (mm)"),
               ("camera","median_angle_deg","Orientation (degrees)"),
               (None,None,"Physical acceptance / completed trials"))
    for ax,(frame,metric,title) in zip(axes.ravel(),metrics):
        data = np.full((len(plan["modes"]),len(profiles)),np.nan)
        labels = {}
        for i,mode in enumerate(plan["modes"]):
            for j,profile in enumerate(profiles):
                g = next(g for g in groups if g["mode"]==mode and g["profile"]==profile)
                value = (g[frame][metric] if frame else g["physical_successes"]/g["completed"] if g["completed"] else None)
                if value is not None:
                    data[i,j] = value
                labels[i,j] = ("—" if value is None else f"{value:.2f}") if frame else f"{g['physical_successes']}/{g['completed']}"
        heat = ax.imshow(data,aspect="auto",cmap="viridis" if frame else "RdYlGn",vmin=0,vmax=None if frame else 1)
        for (i,j),text in labels.items():
            ax.text(j,i,text,ha="center",va="center",fontsize=7,
                    color="white" if frame and np.isfinite(data[i,j]) and data[i,j] < np.nanmax(data)*.5 else "black")
        ax.set_xticks(range(len(profiles)),profiles,rotation=55,ha="right",fontsize=7)
        ax.set_yticks(range(len(plan["modes"])),[NAMES[m] for m in plan["modes"]])
        ax.set_title(title,fontsize=11)
        fig.colorbar(heat,ax=ax,shrink=.65)
    fig.suptitle("Calibration sensitivity | physical-error medians among pixel-converged trials",fontsize=13)
    fig.savefig(directory/"sensitivity.png",dpi=165)
    plt.close(fig)

    fig,ax = plt.subplots(figsize=(8,5),constrained_layout=True)
    colors = dict(aruco="#157a8a",natural="#bf7014",learned="#7556a6")
    for mode in plan["modes"]:
        for nominal in (False,True):
            selected=[r for r in rows if r["mode"]==mode and (r["profile"]=="nominal")==nominal
                      and r.get("max_post_stop_error_px") is not None and r.get("final_accuracy")]
            ax.scatter([r["max_post_stop_error_px"] for r in selected],
                       [r["final_accuracy"]["tool"]["position_error_mm"] for r in selected],
                       color=colors[mode],marker="*" if nominal else "o",s=110 if nominal else 25,alpha=.8,
                       label=NAMES[mode]+(" nominal" if nominal else " perturbed"))
    ax.axvline(1,color="black",linestyle="--",linewidth=1,label="1 px image threshold")
    ax.axhline(plan["position_tolerance_mm"],color="firebrick",linestyle="--",linewidth=1,label="Position tolerance")
    ax.set(xlabel="Worst post-stop pixel error (px)",ylabel="Final tool-frame position error (mm)",
           title="Image convergence versus physical positioning",xlim=(0,None),ylim=(0,None))
    ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(directory/"pixel-vs-physical.png",dpi=165)
    plt.close(fig)
    def fmt(value):
        return "—" if value is None else f"{value:.3f}"
    lines=["# Physical accuracy and calibration sensitivity","",
           f"Completed {len(rows)}/{len(plan['trials'])} predeclared trials; "
           f"{sum(g['pixel_successes'] for g in groups)} passed image convergence and "
           f"{sum(g['physical_successes'] for g in groups)} also met the declared physical tolerances.","",
           f"Physical acceptance requires both camera and tool-frame position errors ≤ {plan['position_tolerance_mm']:g} mm "
           f"and orientation errors ≤ {plan['orientation_tolerance_deg']:g}° through {plan['post_stop_frames']} post-stop captures. "
           "These are illustrative experiment thresholds, not a certified robot specification.","",
           "| Matcher | Profile | Pixel / done | Physical / done | Camera mm | Tool mm | Angle ° | Time s | Paired Δtool mm |",
           "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    policy = ("Reference-image refinement and local sensitivity checks are enabled. "
              "Stopping requires all configured pixel and estimated-camera-correction limits for the hold window, "
              "and every post-stop capture is rechecked. These estimates are not physical ground truth."
              if plan.get("precision_enabled",False) else
              "This run uses the original 1 px image-only stopping rule without reference-image refinement.")
    lines[2:2] = [policy, ""]
    for g in groups:
        lines.append(f"| {NAMES[g['mode']]} | {g['profile']} | {format_rate(g['pixel_successes'], g['completed'])} | "
                     f"{format_rate(g['physical_successes'], g['completed'])} | {fmt(g['camera']['median_position_mm'])} | "
                     f"{fmt(g['tool']['median_position_mm'])} | {fmt(g['camera']['median_angle_deg'])} | "
                     f"{fmt(g['median_success_s'])} | {fmt(g['tool']['paired_median_position_delta_mm'])} |")
    lines += ["","Success counts show 95% Clopper-Pearson (exact) confidence intervals for the true rate "
              "(docs/STATISTICS.md); an observed 100% is not proof of 100% reliability.",
              "","Physical columns and time are medians over pixel-converged trials; all failures remain in the denominators "
              "and outcome counts below. Paired deltas use the same starting pose and matcher, where both nominal and "
              "perturbed cases passed pixel convergence. Negative Δ means a smaller error than nominal, not a statistical finding.",
              "", "![Calibration comparison](sensitivity.png)","",
              "![Pixel error and tool-frame error](pixel-vs-physical.png)","",
              "## Outcomes",""]
    for g in groups:
        lines.append(f"- {NAMES[g['mode']]} / {g['profile']}: {g['outcomes']}; pixel-only passes {g['pixel_only']}; "
                     f"safety failures {g['safety_failures']}; paired successes {g['paired_successes']}.")
    lines += ["","## Method and limits","",
              f"The study pairs {plan['starts']} seeded starts across every selected profile/matcher, at "
              f"{plan['camera_delay_s']*1000:g} ms simulated transport delay. Camera rendering, target geometry and "
              "collision supervision remain nominal. Only controller intrinsics, camera-mount Jacobian and assumed target size change.",
              "", "Each target has a private reference capture at a known simulated teaching pose. The controller receives "
              "only the reference image and its detected features. Ground-truth camera and tool poses stay in evaluation records; "
              "saved application goals are preserved. Position is the Euclidean displacement of the defined frame origin; "
              "orientation is the SO(3) geodesic angle. Signed position components use the corresponding taught frame's axes. "
              "The tool frame is the tool_roll body origin, not an unmodelled gripper TCP.",
              "", "Image and physical success are scored separately. All post-stop checks require zero commanded motion. "
              "Collision clearance and forbidden contacts are audited every physics step. Physical scores are evaluated at "
              "the stop and on fresh current captures, never on delayed PnP pose estimates.",
              "", "These deterministic, small samples measure sensitivity for one static planar target and fixed scene. "
              "Their endpoint spread is across initial poses, not an ISO repeatability test. "
              "No real hardware, distortion, encoder error, moving-obstacle uncertainty, or change of true camera calibration "
              "after teaching is modelled. A perturbed control model can change transients without causing systematic endpoint "
              "bias because the image goal remains the same.",
              "", "The [plan](plan.json), [manifest](manifest.json), [all outcomes](trials.json), "
              "[summary metrics](summary.json) and [evaluation goals](goals.json) are saved. "
              "Per-frame traces are generated locally and ignored by Git. Reports can be rebuilt from saved JSON without them.",
              "", "Run from the project root:", "", "~~~powershell",
              ".\\run.cmd --accuracy-study",
              ".\\run.cmd --accuracy-study --report-only <saved-run-directory>",
              "~~~",""]
    (directory/"REPORT.md").write_text("\n".join(lines),encoding="utf-8")
    return groups

