"""Exercise the real GUI callbacks and save evidence without changing the goal."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import cv2
import numpy as np
from PIL import Image
from app import Lab
from perception import NATURAL_REFERENCE
from simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    goal_hash = hashlib.sha256(NATURAL_REFERENCE.read_bytes()).hexdigest()
    started = time.perf_counter()
    with Simulation() as sim:
        lab = Lab(sim, perception_mode="natural", auto_start=False)
        def click(action):
            lab.draw()
            rect = next(rect for rect, name in lab.buttons if name == action)
            lab.on_mouse(cv2.EVENT_LBUTTONDOWN, rect[0]+10, rect[1]+10, 0, None)
        click("offset")
        pose, sim_time = sim.data.qpos.copy(), float(sim.data.time)
        # A real learned run must not call either classical detector or network.
        with patch("cv2.SIFT_create", side_effect=AssertionError("No SIFT fallback")), \
             patch.object(sim, "marker_corners", side_effect=AssertionError("No ArUco fallback")), \
             patch("urllib.request.urlopen", side_effect=AssertionError("Offline inference")):
            click("matcher")
            assert lab.perception_mode == "learned", lab.message
            np.testing.assert_array_equal(pose, sim.data.qpos)
            assert float(sim.data.time) == sim_time
            assert not sim.velocity_command.any()
            initial = lab.detect_target(sim.image())
            assert initial is not None
            initial_error = float(np.sqrt(np.mean(np.sum((initial-lab.reference)**2, axis=1))))
            click("matches")
            Image.fromarray(lab.draw()).save(args.output / "learned-selected.png")
            click("align")
            states = []
            for step in range(2200):
                lab.advance(1/30)
                if not states or states[-1]["state"] != lab.alignment_status:
                    states.append({"state": lab.alignment_status, "simulation_s": float(sim.data.time)})
                    print(states[-1], flush=True)
                if step % 40 == 0:
                    print(f"step {step}, elapsed {time.perf_counter()-started:.1f}s", flush=True)
                if not lab.aligning:
                    break
            assert lab.alignment_status == "converged", lab.alignment_status
            errors = []
            for _ in range(30):
                lab.advance(1/30)
                corners = lab.detect_target(sim.image())
                assert corners is not None
                errors.append(float(np.sqrt(np.mean(np.sum((corners-lab.reference)**2, axis=1)))))
                assert not sim.velocity_command.any()
            assert max(errors) < 1, errors
            Image.fromarray(lab.draw()).save(args.output / "learned-aligned.png")
            mode = lab.perception_mode
            device = str(lab.perception.device)
        click("matcher")
        assert lab.perception_mode == "natural"
        assert hashlib.sha256(NATURAL_REFERENCE.read_bytes()).hexdigest() == goal_hash
        result = dict(outcome="converged", alignment_mode=mode, switch_back=lab.perception_mode,
                      python=sys.executable, device=device, initial_error_px=initial_error,
                      final_error_px=errors[-1], post_stop_max_error_px=max(errors),
                      pose_preserved_on_switch=True, saved_goal_preserved=True,
                      offline=True, classical_fallback=False, stopped_command_zero=True,
                      transitions=states, wall_seconds=time.perf_counter()-started)
        (args.output / "summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
