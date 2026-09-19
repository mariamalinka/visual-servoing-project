"""Record actual matcher-switch buttons and learned alignment in the GUI."""
import json
import cv2
import numpy as np
from PIL import Image
from app import Lab
from simulation import ROOT,Simulation

def main():
    out=ROOT/"results/learned/demo";out.mkdir(parents=True,exist_ok=True)
    with Simulation() as sim:
        lab=Lab(sim,auto_start=False)
        def click(action):
            lab.draw()
            rect=next(r for r,name in lab.buttons if name==action)
            lab.on_mouse(cv2.EVENT_LBUTTONDOWN,rect[0]+10,rect[1]+10,0,None)
        click("perception");click("matcher")
        assert lab.perception_mode=="learned"
        click("offset");click("matches")
        Image.fromarray(lab.draw()).save(out/"matches.png")
        click("matches");click("auto")
        frames=[];states=[]
        for i in range(1400):
            lab.advance(1/30)
            if not states or states[-1]["state"]!=lab.alignment_status:
                states.append(dict(time_s=float(sim.data.time),state=lab.alignment_status))
            if i%3==0:
                frames.append(Image.fromarray(lab.draw()).resize((900,575)))
            if not lab.aligning:break
        assert lab.alignment_status=="converged",lab.alignment_status
        errors=[]
        for _ in range(30):
            lab.advance(1/30);o=lab.detect_target(sim.image())
            assert o is not None and not sim.velocity_command.any()
            errors.append(float(np.sqrt(np.mean(np.sum((o-lab.reference)**2,axis=1)))))
        assert max(errors)<1
        final=Image.fromarray(lab.draw())
        final.save(out/"final.png");final.save(ROOT/"preview.png")
        frames.extend([final.resize((900,575))]*10)
        frames[0].save(out/"alignment.gif",save_all=True,append_images=frames[1:],duration=100,loop=0)
        result=dict(outcome=lab.alignment_status,mode=lab.perception_mode,auto_enabled=lab.auto_enabled,
            transitions=states,post_stop_errors_px=errors,stopped_command_zero=True,
            playback="GIF uses simulated time; CPU playback is slower")
        (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result,indent=2))

if __name__=="__main__":main()
