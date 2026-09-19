"""Record actual GUI frames for the cold-start workflow without opening a window."""
from pathlib import Path
import json
import numpy as np
from PIL import Image

from app import Lab
from simulation import ROOT, Simulation

def main():
    directory=ROOT/"results/startup/demo"
    directory.mkdir(parents=True,exist_ok=True)
    frames=[]
    with Simulation() as sim:
        sim.reset([35,0,0,0,0,0])
        lab=Lab(sim, auto_start=False,cold_start=True)
        lab.message="Cold start: no remembered viewpoint. Click Align [G] to search from here."
        Image.fromarray(lab.draw()).save(directory/"initial.png")
        lab.align()
        previous=None
        transitions=[]
        for tick in range(4100):
            if tick%6==0:
                frame=Image.fromarray(lab.draw()).resize((900,574))
                frames.append(frame)
            lab.advance(1/30)
            if lab.alignment_status!=previous:
                transitions.append(dict(time_s=float(sim.data.time),status=lab.alignment_status))
                previous=lab.alignment_status
            if not lab.aligning:
                break
        if lab.alignment_status!="converged":
            raise RuntimeError(f"Demo did not converge: {lab.alignment_status}")
        sim.advance(1)
        final=Image.fromarray(lab.draw())
        final.save(directory/"final.png")
        final.save(ROOT/"preview.png")
        frames.append(final.resize((900,574)))
        frames[0].save(directory/"startup_search.gif",save_all=True,append_images=frames[1:],
                       duration=[100]*(len(frames)-1)+[1800],loop=0,optimize=False)
        result=dict(outcome=lab.alignment_status,acquisition_s=lab.controller.startup.acquired_s,
                    transitions=transitions,reference_contains_robot_pose=False,
                    animation_speed="2x simulated time; one frame per 0.2 simulated seconds")
        (directory/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()
