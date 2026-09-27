"""Independently project known pose perturbations to evaluate image-stop sensitivity.

Ground truth is used only by this offline evaluator. No controller input changes.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
from accuracy import pose_accuracy,physical_pass
from precision import load_precision_config,stopping_metrics


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def analyze(baseline,output):
    baseline=Path(baseline).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((baseline/'manifest.json').read_text())
    if digest(APP/'scene.xml')!=manifest['fingerprint']['sha256']['scene.xml']:
        raise ValueError('Scene must match the recorded teaching geometry')
    if digest(baseline/'goals.json')!=manifest['goals_sha256']:
        raise ValueError('Teaching records changed')
    goal=json.loads((baseline/'goals.json').read_text())['natural']
    model=mujoco.MjModel.from_xml_path(str(APP/'scene.xml'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    board=model.geom('target_board').id;h=goal['true_target_side_m']/2;x=-model.geom_size[board,0]
    world=np.array([[x,h,h],[x,-h,h],[x,-h,-h],[x,h,-h]])@data.geom_xmat[board].reshape(3,3).T+data.geom_xpos[board]
    taught=np.asarray(goal['camera_world']);tool=np.asarray(goal['tool_world']);K=np.asarray(goal['true_K'])
    camera_tool=np.linalg.inv(taught)@tool
    settings=load_precision_config()['stop']
    def project(pose):
        points=(world-pose[:3,3])@pose[:3,:3];p=points@K.T
        return p[:,:2]/p[:,2:],points[:,2]
    def moved(twist):
        delta=np.eye(4);delta[:3,:3]=cv2.Rodrigues(np.asarray(twist)[3:])[0];delta[:3,3]=twist[:3]
        return taught@delta
    desired,depths=project(taught)
    # Numerical pixel Jacobian is independent of the controller interaction matrix.
    J=np.zeros((8,6));eps=1e-6
    for axis in range(6):
        step=np.eye(6)[axis]*eps
        J[:,axis]=(project(moved(step))[0]-project(moved(-step))[0]).ravel()/(2*eps)
    scales=np.r_[np.full(3,.001),np.full(3,np.deg2rad(.1))]
    u,s,vt=np.linalg.svd(J*scales,full_matrices=False)
    weak=vt[-1]*scales;weak/=np.linalg.norm(weak[:3])
    probes=[('identity',np.zeros(6))]
    for axis in range(6):
        for magnitude in ((.0005,.001,.003) if axis<3 else np.deg2rad([.05,.2,1])):
            for sign in (-1,1):
                step=np.eye(6)[axis]*sign*magnitude
                probes.append((('translation-' if axis<3 else 'rotation-')+'xyz'[axis%3]+f'-{sign*magnitude:g}',step))
    for mm in (-5,-3,-1,-.5,.5,1,3,5):probes.append((f'weak-direction-{mm:g}mm',weak*mm/1000))
    rows=[]
    for name,step in probes:
        pose=moved(step);pixels,z=project(pose)
        metrics=stopping_metrics(pixels,desired,K,z,settings)
        camera=pose_accuracy(pose,taught);tip=pose_accuracy(pose@camera_tool,tool)
        physical=physical_pass(camera,tip,2,1)
        rows.append(dict(name=name,camera_twist=step.tolist(),camera=camera,tool=tip,
                         image_error_px=metrics['rms_error_px'],legacy_candidate=metrics['rms_error_px']<1,
                         precision_candidate=metrics['candidate'],within_physical_limits=physical,metrics=metrics))
    legacy_misses=sum(r['legacy_candidate'] and not r['within_physical_limits'] for r in rows)
    precision_misses=sum(r['precision_candidate'] and not r['within_physical_limits'] for r in rows)
    # Operator norms give worst linear amplification of an 8-coordinate error
    # with L2 norm 0.05 px; this is a sensitivity example, not a noise covariance.
    inverse=np.linalg.pinv(J)
    amplification=dict(feature_vector_l2_px=.05,
        max_translation_mm=float(np.linalg.norm(inverse[:3],2)*.05*1000),
        max_rotation_deg=float(np.rad2deg(np.linalg.norm(inverse[3:],2)*.05)))
    record=dict(method='Exact pinhole projection at perturbed camera poses; independent numerical Jacobian',
        source_sha256=digest(__file__),scene_sha256=digest(APP/'scene.xml'),goals_sha256=digest(baseline/'goals.json'),
        precision_config_sha256=digest(APP/'precision_config.json'),mujoco_version=mujoco.__version__,
        dimensionless_pose_scales=scales.tolist(),scaled_singular_values_px=s.tolist(),
        weak_camera_direction=weak.tolist(),feature_error_amplification=amplification,
        probes=len(rows),legacy_false_candidates=legacy_misses,precision_false_candidates=precision_misses,rows=rows)
    (output/'pose-sensitivity.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4.4),constrained_layout=True)
    axes[0].bar(np.arange(1,7),s,color='#157a8a');axes[0].set_yscale('log')
    axes[0].set(xlabel='Scaled camera-motion direction (SVD)',ylabel='Pixel-vector change per unit scaled motion',
                title='Image sensitivity: 1 mm / 0.1 degree scales')
    for kind,label,color in [('translation','Translation','#157a8a'),('rotation','Rotation','#bf7014'),('weak','Weak coupled direction','#7556a6')]:
        selected=[r for r in rows if r['name'].startswith(kind)]
        axes[1].scatter([r['image_error_px'] for r in selected],[r['tool']['position_error_mm'] for r in selected],label=label,color=color,alpha=.8)
    axes[1].axvline(1,color='black',ls='--',lw=1,label='Legacy 1 px threshold')
    axes[1].axhline(2,color='firebrick',ls='--',lw=1,label='2 mm physical tolerance')
    axes[1].set(xlabel='Exact projected corner RMS (px)',ylabel='Actual tool-position change (mm)',title='Small image change can hide tool motion')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.2)
    fig.savefig(output/'pose-sensitivity.png',dpi=165);plt.close(fig)
    lines=['# Sensitivity to physical pose changes','',
        f'{len(rows)} independent projection probes; {legacy_misses} legacy stopping candidates lie outside the physical limits, versus {precision_misses} precision candidates.','',
        'These are instantaneous eligibility checks at fixed simulated poses, not closed-loop success rates. Exact depths and true K are used only in this evaluator; the application estimates depths from images and uses its configured calibration.','',
        'The numerical pixel Jacobian is differentiated from exact projection. Translation columns are scaled by 1 mm and rotation columns by 0.1 degree before SVD. The weakest direction combines translation and rotation.','',
        f'A hypothetical feature-error vector with L2 norm 0.05 px can amplify to {amplification["max_translation_mm"]:.3f} mm of local translation error or {amplification["max_rotation_deg"]:.3f} degrees of rotation error. These are separate worst-direction linear sensitivities, not confidence intervals or measured sensor noise.','',
        '![Pose sensitivity](pose-sensitivity.png)','',
        '| Perturbation | Exact px | Tool mm | Camera degrees | Legacy candidate | Precision candidate | Physical tolerance |',
        '|---|---:|---:|---:|---|---|---|']
    for r in rows:lines.append(f'| {r["name"]} | {r["image_error_px"]:.3f} | {r["tool"]["position_error_mm"]:.3f} | {r["camera"]["orientation_error_deg"]:.3f} | {r["legacy_candidate"]} | {r["precision_candidate"]} | {r["within_physical_limits"]} |')
    lines+=['','[Inputs and all scores](pose-sensitivity.json). This covers one teaching view and planar geometry; it does not prove observability for every target, or remove calibration/noise uncertainty.','',
        'Reproduce from the repository root:', '', '~~~powershell',
        '.\\outputs\\visual-servoing-simulation\\.venv\\Scripts\\python.exe -B tools/analyze_pose_sensitivity.py --baseline outputs/visual-servoing-simulation/results/accuracy/20260921-validation --output <result-directory>', '~~~','']
    (output/'POSE_SENSITIVITY.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps({k:record[k] for k in ('probes','legacy_false_candidates','precision_false_candidates','feature_error_amplification')}))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();analyze(args.baseline,args.output)
