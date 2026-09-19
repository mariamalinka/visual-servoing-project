"""Record the real GUI during refined search, without pointer interaction."""
import json
import numpy as np
from PIL import Image
from app import Lab
from simulation import ROOT,Simulation

def main():
    spec=json.loads((ROOT/"tests/fixtures/search-missed-starts.json").read_text(encoding="utf-8"))[0]
    out=ROOT/"results/search-coverage/demo"
    out.mkdir(parents=True,exist_ok=True)
    transitions=[]
    saved=set()
    with Simulation() as sim:
        sim.model.body("target").pos[:]+=spec["target_shift_m"]
        sim.reset(spec["offset_degrees"])
        lab=Lab(sim,cold_start=True,auto_start=False)
        lab.toggle_gain()
        lab.toggle_auto()
        for _ in range(6800):
            lab.advance(1/30)
            search=lab.controller.startup
            phase=lab.alignment_status
            stage=search.stage if search and lab.controller.mode=="startup" else phase
            if not transitions or transitions[-1]["state"]!=stage:
                transitions.append(dict(time_s=float(sim.data.time),state=stage,phase=phase))
            if stage in ("coarse","refined","running","converged") and stage not in saved:
                Image.fromarray(lab.draw()).save(out/f"{stage}.png")
                saved.add(stage)
            if not lab.aligning:
                break
        if lab.alignment_status!="converged":
            raise RuntimeError(f"GUI demo did not converge: {lab.alignment_status}")
        errors=[]
        for _ in range(30):
            lab.advance(1/30)
            corners=sim.marker_corners(sim.image())
            errors.append(float(np.sqrt(np.mean(np.sum((corners-lab.reference)**2,axis=1)))) if corners is not None else None)
        assert all(e is not None and e<1 for e in errors)
        assert not sim.velocity_command.any()
        frame=lab.draw()
        Image.fromarray(frame).save(out/"final.png")
        Image.fromarray(frame).save(ROOT/"preview.png")
        result=dict(trial_id=spec["trial_id"],gain_mode=lab.gain_mode,auto_enabled=lab.auto_enabled,
                    outcome=lab.alignment_status,transitions=transitions,
                    search_acquired_s=search.acquired_s,refinement_started_s=search.refinement_started_s,
                    post_stop_errors_px=errors,terminal_command_zero=True)
        (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()
