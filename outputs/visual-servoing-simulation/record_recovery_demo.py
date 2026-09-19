"""Record the actual desktop find-and-align flow as images and an animation."""
import json
import numpy as np
from PIL import Image
from app import Lab
from simulation import ROOT, Simulation

def main():
    directory=ROOT/"results/recovery/demo"
    directory.mkdir(parents=True,exist_ok=True)
    frames=[]
    states=[]
    with Simulation() as sim:
        lab=Lab(sim, auto_start=False)
        lab.lost_view()
        initial=lab.draw()
        Image.fromarray(initial).save(directory/"initial.png")
        first=Image.fromarray(initial).resize((900,575),Image.Resampling.LANCZOS)
        frames.extend([first]*6)
        lab.align()
        saved_search=False
        for tick in range(1352):
            lab.advance(1/30)
            if not states or states[-1]["status"] != lab.alignment_status:
                states.append({"time_s":float(sim.data.time),"status":lab.alignment_status})
            if tick%3==0:
                rgb=lab.draw()
                frames.append(Image.fromarray(rgb).resize((900,575),Image.Resampling.LANCZOS))
                if lab.alignment_status=="returning" and not saved_search:
                    Image.fromarray(rgb).save(directory/"searching.png")
                    saved_search=True
            if not lab.aligning:break
        if lab.alignment_status!="converged":
            raise RuntimeError(f"Demonstration failed: {lab.alignment_status}")
        sim.advance(1)
        final=lab.draw()
        Image.fromarray(final).save(directory/"final.png")
        frames.extend([Image.fromarray(final).resize((900,575),Image.Resampling.LANCZOS)]*12)
        frames[0].save(directory/"recovery.gif",save_all=True,append_images=frames[1:],
                       duration=100,loop=0,optimize=False)
        corners=sim.marker_corners(sim.image())
        error=float(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1))))
        report={"status":lab.alignment_status,"final_error_px":error,"transitions":states,
                "note":"Recorded live MuJoCo state and the desktop draw/click logic. No pose reset occurs during recovery."}
        (directory/"summary.json").write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))

if __name__=="__main__":
    main()

