"""Save actual GUI states while the first failed-start regression is retried."""
import json
from pathlib import Path
from PIL import Image

from app import Lab
from simulation import ROOT,Simulation

def main():
    cases=json.loads((ROOT/"tests/fixtures/joint-limit-starts.json").read_text(encoding="utf-8"))
    spec=cases[0]
    out=ROOT/"results/joint-limits/demo"
    out.mkdir(parents=True,exist_ok=True)
    saved=set()
    transitions=[]
    with Simulation() as sim:
        sim.model.body("target").pos[:]+=spec["target_shift_m"]
        sim.reset(spec["offset_degrees"])
        lab=Lab(sim,cold_start=True)
        for _ in range(4100):
            lab.advance(1/30)
            phase=lab.alignment_status
            if not transitions or transitions[-1]["phase"]!=phase:
                transitions.append(dict(time_s=float(sim.data.time),phase=phase))
            if phase in ("joint_limited","repositioning","retry_confirming","realigning","converged") and phase not in saved:
                Image.fromarray(lab.draw()).save(out/f"{phase}.png")
                saved.add(phase)
            if not lab.aligning:
                break
        if lab.alignment_status!="converged":
            raise RuntimeError(f"Demo failed: {lab.alignment_status}")
        sim.advance(1)
        Image.fromarray(lab.draw()).save(out/"final.png")
        Image.fromarray(lab.draw()).save(ROOT/"preview.png")
        result=dict(outcome=lab.alignment_status,trial_id=spec["trial_id"],transitions=transitions,
                    motion_events=lab.controller.motion.events,
                    retries=lab.controller.motion.retries,
                    min_goal_description="One saved reference image, no taught goal joint pose")
        (out/"summary.json").write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(json.dumps(result,indent=2))

if __name__=="__main__":
    main()
