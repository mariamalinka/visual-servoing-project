"""Record actual application controls and natural-picture alignment."""
import json
import cv2
import numpy as np
from PIL import Image
from app import Lab
from simulation import ROOT,Simulation

def main():
    out=ROOT/"results/natural-image/demo"
    out.mkdir(parents=True,exist_ok=True)
    results={}
    with Simulation() as sim:
        lab=Lab(sim,auto_start=False)
        def click(action):
            lab.draw()
            rect=next(r for r,name in lab.buttons if name==action)
            lab.on_mouse(cv2.EVENT_LBUTTONDOWN,rect[0]+10,rect[1]+10,0,None)
        click("perception")
        assert lab.perception_mode=="natural"
        click("offset")
        Image.fromarray(lab.draw()).save(out/"offset-initial.png")
        click("matches")
        Image.fromarray(lab.draw()).save(out/"matches.png")
        click("matches")
        click("auto")
        for name in ("offset","cold"):
            if name=="cold":
                click("cold_start")
                assert lab.controller.last_visible_qpos is None
                assert lab.detect_target(sim.image()) is None
                Image.fromarray(lab.draw()).save(out/"cold-initial.png")
            transitions=[]
            frames=[]
            saved=set()
            for tick in range(6800):
                lab.advance(1/30)
                state=lab.alignment_status
                if not transitions or transitions[-1]["state"]!=state:
                    transitions.append(dict(time_s=float(sim.data.time),state=state))
                if state in ("scanning","confirming","running","converged") and state not in saved:
                    Image.fromarray(lab.draw()).save(out/f"{name}-{state}.png")
                    saved.add(state)
                if name=="offset" and tick%3==0:
                    frames.append(Image.fromarray(lab.draw()).resize((900,575)))
                if not lab.aligning:
                    break
            assert lab.alignment_status=="converged", (name,lab.alignment_status)
            errors=[]
            for _ in range(30):
                lab.advance(1/30)
                corners=lab.detect_target(sim.image())
                assert corners is not None and not sim.velocity_command.any()
                errors.append(float(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1)))))
            assert max(errors)<1
            frame=Image.fromarray(lab.draw())
            frame.save(out/f"{name}-final.png")
            if name=="offset":
                frames.extend([frame.resize((900,575))]*10)
                frames[0].save(out/"alignment.gif",save_all=True,append_images=frames[1:],duration=100,loop=0)
            else:
                frame.save(ROOT/"preview.png")
            results[name]=dict(outcome=lab.alignment_status,auto_enabled=lab.auto_enabled,
                perception_mode=lab.perception_mode,transitions=transitions,
                post_stop_errors_px=errors,terminal_command_zero=True)
        (out/"summary.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
        print(json.dumps(results,indent=2))

if __name__=="__main__":
    main()
