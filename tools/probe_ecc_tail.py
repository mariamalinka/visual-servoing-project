"""Replay captured robot states to compare bounded ECC schedules, outside control."""
import argparse
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
from app import make_perception
from reference_image import load_reference
from perception import NATURAL_REFERENCE
from precision import GoalRefinedPerception,load_precision_config
from simulation import Simulation
import mujoco


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    cv2.setNumThreads(4)
    original=cv2.findTransformECC
    schedules={'original':(45,1e-6,0), '12-fine':(12,1e-6,0),
               '20-relaxed':(20,1e-5,0),'45-relaxed':(45,1e-5,0),
               '8-coarse-12-fine':(12,1e-6,8), '6-coarse-8-fine':(8,1e-6,6)}
    rows=[]
    with Simulation() as sim:
        sim.set_target_mode('natural')
        for mode in ('natural','learned'):
            detector=make_perception(mode,sim)
            goal,corners=load_reference(NATURAL_REFERENCE,sim.camera_intrinsics(),
                detector.reference_config(sim.config),lambda rgb:detector.observe(rgb).corners)
            source=[]
            for path in args.input.glob(mode+'-transport-50ms-*.json'):
                data=json.loads(path.read_text(encoding='utf-8'))
                frames=[f for f in data['sensor_frames'] if f['refinement']=='refined' and
                        not f['stage_ms'].get('cache_hit')]
                # Fixed selection: eight largest refinements and a spread through each run.
                chosen=sorted(frames,key=lambda f:f['stage_ms'].get('refinement_ms',0),reverse=True)[:8]
                chosen+=frames[::max(1,len(frames)//8)]
                seen=set()
                for f in chosen:
                    if f['sequence'] not in seen:
                        source.append((path.name,f));seen.add(f['sequence'])
            refiners={name:GoalRefinedPerception(detector,goal,corners,dict(load_precision_config(),ecc_coarse_scale=1.)) for name in schedules}
            precomputed={}
            for name,refiner in refiners.items():
                h,w=refiner.gray.shape;size=(round(w/2),round(h/2))
                sx,sy=size[0]/w,size[1]/h
                S=np.array([[sx,0,(sx-1)/2],[0,sy,(sy-1)/2],[0,0,1]],np.float32)
                precomputed[name]=(cv2.resize(refiner.gray,size,interpolation=cv2.INTER_AREA),size,S,np.linalg.inv(S))
            for source_name,frame in source:
                sim.data.qpos[:]=frame['qpos'];mujoco.mj_forward(sim.model,sim.data)
                rgb=sim.image();coarse=detector.observe(rgb)
                if coarse.corners is None:
                    continue
                baseline=None
                for name,(iterations,epsilon,coarse_iterations) in schedules.items():
                    def ecc(template,current,H,motion,criteria,mask,blur):
                        if coarse_iterations:
                            low,size,S,Si=precomputed[name]
                            small=cv2.resize(current,size,interpolation=cv2.INTER_AREA)
                            lowmask=cv2.resize(mask,size,interpolation=cv2.INTER_NEAREST)
                            _,Hlow=original(low,small,(S@H@Si).astype(np.float32),motion,
                                (criteria[0],coarse_iterations,epsilon),lowmask,blur)
                            H=(Si@Hlow@S).astype(np.float32)
                        return original(template,current,H,motion,(criteria[0],iterations,epsilon),mask,blur)
                    started=time.perf_counter()
                    with patch('precision.cv2.findTransformECC',ecc):
                        result=refiners[name]._refine(rgb,coarse)
                    elapsed=1000*(time.perf_counter()-started)
                    if name=='original':
                        baseline=result.corners
                    difference=(None if baseline is None or result.corners is None else
                        float(np.max(np.linalg.norm(result.corners-baseline,axis=1))))
                    rows.append(dict(mode=mode,source=source_name,sequence=frame['sequence'],schedule=name,
                        refinement_ms=elapsed,accepted=result.corners is not None,max_corner_delta_px=difference))
            print(mode,'frames',len(source),flush=True)
    summary=[]
    for mode in ('natural','learned'):
        for name in schedules:
            group=[r for r in rows if r['mode']==mode and r['schedule']==name]
            times=[r['refinement_ms'] for r in group]
            differences=[r['max_corner_delta_px'] for r in group if r['max_corner_delta_px'] is not None]
            summary.append(dict(mode=mode,schedule=name,count=len(group),accepted=sum(r['accepted'] for r in group),
                p95_ms=float(np.percentile(times,95)),max_ms=max(times),
                max_corner_delta_px=max(differences,default=None)))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(dict(summary=summary,frames=rows),indent=2)+'\n',encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
