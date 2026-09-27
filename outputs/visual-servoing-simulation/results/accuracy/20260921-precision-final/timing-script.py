"""Reproduce warmed matcher/refinement timing on saved near-goal camera poses."""
from pathlib import Path
import hashlib,json,sys,time
import cv2,mujoco,numpy as np
RUN=Path(__file__).resolve().parent
APP=RUN.parents[2]
sys.path.insert(0,str(APP))
from app import Lab
from simulation import Simulation

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    # The compact timing input avoids depending on ignored bulk experiment traces.
    input_path=RUN/'timing-input.json'
    if input_path.exists():
        inputs=json.loads(input_path.read_text(encoding='utf-8'))
    else:
        inputs={}
        for mode in ('natural','learned'):
            trace=RUN/'traces'/f'{mode}-nominal-0000.jsonl'
            rows=[json.loads(line) for line in trace.read_text(encoding='utf-8').splitlines()]
            rows=[r for r in rows if r.get('refinement')=='refined' and any(r['command_rad_s'])]
            inputs[mode]=[rows[i]['qpos_rad'] for i in np.linspace(0,len(rows)-1,8,dtype=int)]
        input_path.write_text(json.dumps(inputs,indent=2)+'\n',encoding='utf-8')
    results={}
    with Simulation() as sim:
        for mode,qposes in inputs.items():
            lab=Lab(sim,auto_start=False,perception_mode=mode,reference_path=RUN/'reference/natural.npz')
            refiner=lab.perception;base=refiner.base
            images=[]
            for q in qposes:
                sim.data.qpos[:]=q;mujoco.mj_forward(sim.model,sim.data);images.append(sim.image())
            refiner.observe(images[0])
            times={'matcher_only':[],'matcher_and_refinement':[]};accepted=[]
            for image in images:
                for refined in (False,True):
                    base._last_rgb=None;refiner._rgb=None
                    start=time.perf_counter();result=(refiner if refined else base).observe(image)
                    times['matcher_and_refinement' if refined else 'matcher_only'].append(1000*(time.perf_counter()-start))
                    if refined:accepted.append(result.refinement=='refined')
            results[mode]=dict(matcher=base.name,frames=len(images),refined_frames=sum(accepted),
                milliseconds=times,median_ms={k:float(np.median(v)) for k,v in times.items()})
    record=dict(method='Eight saved near-goal poses per matcher; warmed model, forced fresh inference, paired same RGB; excludes rendering and simulated transport delay',
        source_sha256=digest(__file__),input_sha256=digest(input_path),reference_sha256=digest(RUN/'reference/natural.npz'),
        results=results)
    (RUN/'refinement-timing.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results,indent=2))
if __name__=='__main__':main()
