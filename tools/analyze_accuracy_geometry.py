"""Check ideal target-corner projection against physical error in a completed accuracy run.

This is an evaluation-only follow-up; it never feeds geometry to the controller.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco

APP=Path(__file__).resolve().parents[1]/"outputs/visual-servoing-simulation"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def project(world,pose,K):
    T=np.asarray(pose,dtype=float)
    points=(world-T[:3,3])@T[:3,:3]
    if np.any(points[:,2]<=0):
        return None
    pixels=points@np.asarray(K).T
    return pixels[:,:2]/pixels[:,2:]


def analyze(directory):
    directory=Path(directory).resolve()
    manifest=json.loads((directory/"manifest.json").read_text())
    if manifest["status"]!="complete":
        raise ValueError("Run the complete accuracy experiment before this geometry check")
    for name in ("scene.xml","config.json"):
        if digest(APP/name)!=manifest["fingerprint"]["sha256"][name]:
            raise ValueError("Scene/config changed; use the experiment's recorded revision")
    if mujoco.__version__!=manifest["fingerprint"]["runtime"]["mujoco"]:
        raise ValueError("Use the recorded MuJoCo version")
    goals=json.loads((directory/"goals.json").read_text())
    if digest(directory/"goals.json")!=manifest["goals_sha256"]:
        raise ValueError("Evaluation goals changed")
    trials=json.loads((directory/"trials.json").read_text())
    model=mujoco.MjModel.from_xml_path(str(APP/"scene.xml"))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    board=model.geom("target_board").id
    rotation=data.geom_xmat[board].reshape(3,3)
    center=data.geom_xpos[board]
    rows=[]
    for trial in trials:
        row={k:trial[k] for k in ("id","mode","profile","case","outcome","pixel_success","physical_success")}
        row.update(geometric_error_px=None,detected_post_stop_error_px=trial.get("max_post_stop_error_px"),
                   tool_position_mm=None,camera_position_mm=None,angle_deg=None)
        if trial.get("final_camera_world") is not None:
            goal=goals["aruco" if trial["mode"]=="aruco" else "natural"]
            h=goal["true_target_side_m"]/2
            # Square edges on the board's front (-X) face. The physical texture
            # square spans 384/512 of the 0.32 m board, i.e. the declared 0.24 m.
            x=-float(model.geom_size[board,0])
            local=np.array([[x,h,h],[x,-h,h],[x,-h,-h],[x,h,-h]])
            world=local@rotation.T+center
            desired=project(world,goal["camera_world"],goal["true_K"])
            current=project(world,trial["final_camera_world"],goal["true_K"])
            if desired is not None and current is not None:
                row["geometric_error_px"]=float(np.sqrt(np.mean(np.sum((current-desired)**2,axis=1))))
            row.update(tool_position_mm=trial["final_accuracy"]["tool"]["position_error_mm"],
                       camera_position_mm=trial["final_accuracy"]["camera"]["position_error_mm"],
                       angle_deg=trial["final_accuracy"]["camera"]["orientation_error_deg"])
        rows.append(row)
    record=dict(method="Ideal pinhole projection of fixed physical target corners; evaluator only",
                analysis_sha256=digest(__file__),trials_sha256=digest(directory/"trials.json"),
                goals_sha256=digest(directory/"goals.json"),rows=rows)
    (directory/"geometry-check.json").write_text(json.dumps(record,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,5),constrained_layout=True)
    names=dict(aruco="ArUco",natural="SIFT",learned="Learned")
    colors=dict(aruco="#157a8a",natural="#bf7014",learned="#7556a6")
    for mode in names:
        for success in (True,False):
            selected=[r for r in rows if r["mode"]==mode and r["pixel_success"]==success and r["geometric_error_px"] is not None]
            if selected:
                ax.scatter([r["geometric_error_px"] for r in selected],[r["tool_position_mm"] for r in selected],
                           color=colors[mode],marker="o" if success else "x",alpha=.75,
                           label=names[mode]+(" image pass" if success else " image failure"))
    plan=json.loads((directory/"plan.json").read_text())
    ax.axvline(1,color="black",linestyle="--",linewidth=1,label="1 px geometric change")
    ax.axhline(plan["position_tolerance_mm"],color="firebrick",linestyle="--",linewidth=1,label="Position tolerance")
    ax.set(xlabel="True geometric corner displacement (px RMS)",ylabel="Final tool-frame position error (mm)",
           title="Projection geometry versus physical position",xlim=(0,None),ylim=(0,None))
    ax.grid(alpha=.2);ax.legend(fontsize=8)
    zoom=ax.inset_axes([.43,.12,.51,.48])
    for mode in names:
        for success in (True,False):
            selected=[r for r in rows if r["mode"]==mode and r["pixel_success"]==success and r["geometric_error_px"] is not None]
            zoom.scatter([r["geometric_error_px"] for r in selected],[r["tool_position_mm"] for r in selected],
                         color=colors[mode],marker="o" if success else "x",alpha=.75,s=18)
    zoom.axvline(1,color="black",linestyle="--",linewidth=1)
    zoom.axhline(plan["position_tolerance_mm"],color="firebrick",linestyle="--",linewidth=1)
    zoom.set(xlim=(.5,1.3),ylim=(0,14),title="Detail near the 1 px threshold")
    zoom.tick_params(labelsize=8);zoom.grid(alpha=.2)
    fig.savefig(directory/"geometry-vs-physical.png",dpi=165);plt.close(fig)
    examples=[r for r in rows if r["pixel_success"] and r["geometric_error_px"] is not None
              and r["geometric_error_px"]<1 and r["tool_position_mm"]>plan["position_tolerance_mm"]]
    lines=["# Independent projection-geometry check","",
           "This follow-up uses exact simulated camera poses, the real camera intrinsics and the physical "
           "square's corners on the board's front face. It compares ideal pinhole projections at the final "
           "and taught poses. It does not use the matcher, PnP or assumed controller calibration.","",
           "The result measures geometric change in the image; it does not model rasterization or prove that "
           "a detector's corner errors are unbiased. Detected error below is the worst of the post-stop captures; "
           "geometric error uses the final pose.",""]
    if examples:
        example=max(examples,key=lambda r:r["tool_position_mm"])
        lines += [f"**Example: {example['id']}.** Detected image error is {example['detected_post_stop_error_px']:.3f} px; "
                  f"true geometric image change is {example['geometric_error_px']:.3f} px. Tool-frame position error is "
                  f"{example['tool_position_mm']:.3f} mm and orientation error is {example['angle_deg']:.3f}°.","",
                  "Thus even true sub-pixel corner agreement can coexist with a physical-tolerance miss in this "
                  "viewing geometry. Coupled camera translation and rotation can preserve a very similar image "
                  "of this planar target. This observation does not establish that calibration bias alone caused "
                  "the endpoint difference, or that all targets/viewpoints behave this way.",""]
    lines += ["![Geometric image error and physical error](geometry-vs-physical.png)","",
              "| Trial | Outcome | Detected px | Geometric px | Camera mm | Tool mm | Angle ° |",
              "|---|---|---:|---:|---:|---:|---:|"]
    def fmt(value):return "—" if value is None else f"{value:.3f}"
    for r in rows:
        lines.append(f"| {r['id']} | {r['outcome']} | {fmt(r['detected_post_stop_error_px'])} | "
                     f"{fmt(r['geometric_error_px'])} | {fmt(r['camera_position_mm'])} | "
                     f"{fmt(r['tool_position_mm'])} | {fmt(r['angle_deg'])} |")
    lines += ["","All outcomes remain in this table. Missing projections are marked —.",
              "[Recorded calculation inputs and results](geometry-check.json) · [Main sensitivity report](REPORT.md)","",
              "Reproduce from the repository root:","", "~~~powershell",
              ".\\outputs\\visual-servoing-simulation\\.venv\\Scripts\\python.exe -B tools/analyze_accuracy_geometry.py <completed-run-directory>",
              "~~~",""]
    (directory/"GEOMETRY_CHECK.md").write_text("\n".join(lines),encoding="utf-8")
    return dict(trials=len(rows),subpixel_physical_misses=len(examples))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    print(json.dumps(analyze(parser.parse_args().directory)))

