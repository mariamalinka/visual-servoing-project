"""Acceptance check: actuator commands move the arm and its rendered camera image.

Checks the scene, manual commands, and the app's automatic IBVS alignment path.
Run: python verify.py
"""
from __future__ import annotations

import json
import platform

import cv2
import mujoco
import numpy as np
from PIL import Image

from app import Lab
from simulation import ROOT, Simulation


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    directory = ROOT / "verification"
    directory.mkdir(exist_ok=True)
    with Simulation() as sim:
        require(sim.model.nv == 6 and sim.model.nu == 6, "Expected six actuated joints")
        start_q = sim.data.qpos.copy()
        pose_before = sim.camera_pose()
        before = sim.image()
        corners_before = sim.marker_corners(before)
        require(corners_before is not None, "Marker must be detected in the initial rendered image")
        Image.fromarray(before).save(directory / "wrist-before.png")
        Image.fromarray(sim.image("world")).save(directory / "scene.png")

        # Command an actuator; do not assign qpos to manufacture movement.
        sim.command_velocity(np.array([0.15, 0, 0, 0, 0, 0]))
        sim.advance(0.5)
        sim.command_velocity(np.zeros(6))
        sim.advance(0.25)
        after = sim.image()
        corners_after = sim.marker_corners(after)
        require(corners_after is not None, "Marker must remain detectable after the test motion")
        joint_motion = float(np.rad2deg(sim.data.qpos[0] - start_q[0]))
        camera_motion = float(np.linalg.norm(sim.camera_pose()[:3, 3] - pose_before[:3, 3]))
        pixel_motion = float(np.linalg.norm(corners_after.mean(axis=0) - corners_before.mean(axis=0)))
        require(joint_motion > 2, "Joint command did not move joint 1 sufficiently")
        require(camera_motion > 0.01, "Wrist camera did not move with the arm")
        require(pixel_motion > 10, "Rendered marker did not move sufficiently in the camera image")
        require(np.isfinite(sim.data.qpos).all(), "Non-finite physics state")
        Image.fromarray(after).save(directory / "wrist-after.png")

        # Check all six joints are genuinely driven by actuator commands.
        response = []
        for i in range(6):
            sim.reset()
            command = np.zeros(6)
            command[i] = 0.10
            sim.command_velocity(command)
            sim.advance(0.3)
            delta = float(sim.data.qpos[i] - start_q[i])
            require(delta > 0.015, f"Joint {i+1} failed to respond to velocity input")
            response.append(float(np.rad2deg(delta)))

        # A zero command at the gravity-compensated home pose should hold still.
        sim.reset()
        sim.advance(2.0)
        drift = float(np.max(np.abs(sim.data.qpos - start_q)))
        require(drift < 0.002, "Arm drifted with zero joint velocity command")

        # Reset must restore reproducible RGB and pose, not just visual labels.
        sim.reset()
        reset_rgb = sim.image()
        require(np.array_equal(before, reset_rgb), "Reset did not reproduce the initial image")
        require(np.allclose(sim.camera_pose(), pose_before), "Reset did not restore camera pose")

        # Exercise the same manual pulse used by the desktop buttons.
        lab = Lab(sim, auto_start=False)
        lab.draw()
        rect = next(rect for rect, action in lab.buttons if action == "jog:0:1")
        lab.on_mouse(cv2.EVENT_LBUTTONDOWN, rect[0] + 10, rect[1] + 10, 0, None)
        for _ in range(30):
            lab.advance(1 / 30)
        require(not np.any(sim.velocity_command), "Manual jog failed to stop automatically")
        require(0.025 < sim.data.qpos[0] - start_q[0] < 0.06, "Manual jog duration is incorrect")

        # Pause must stop simulation time; resume must not restart an old pulse.
        lab.toggle_pause()
        paused_at = sim.data.time
        lab.advance(0.5)
        require(sim.data.time == paused_at, "Paused simulation advanced")
        lab.toggle_pause()
        lab.advance(0.1)
        require(not np.any(sim.velocity_command), "Resume restarted a canceled command")

        # Smoke check scripted motion keeps the initial demonstration usable.
        lab.toggle_demo()
        for _ in range(120):
            lab.advance(1 / 30)
        require(sim.marker_corners(sim.image()) is not None, "Demo lost the marker")
        lab.stop()
        require(not np.any(sim.velocity_command), "Stop did not cancel demo velocity")

        sim.reset()
        lab = Lab(sim, auto_start=False)
        # Exercise the real app click dispatch and controller, from an offset.
        lab.draw()
        for action in ("offset", "align"):
            rect = next(rect for rect, name in lab.buttons if name == action)
            lab.on_mouse(cv2.EVENT_LBUTTONDOWN, rect[0] + 10, rect[1] + 10, 0, None)
            lab.draw()
        initial_alignment_error = float(np.sqrt(np.mean(np.sum(
            (sim.marker_corners(sim.image()) - lab.reference)**2, axis=1))))
        for _ in range(600):
            lab.advance(1/30)
            if not lab.aligning:
                break
        require(lab.alignment_status == "converged", "App alignment did not converge")
        sim.advance(1)
        final_alignment_error = float(np.sqrt(np.mean(np.sum(
            (sim.marker_corners(sim.image()) - lab.reference)**2, axis=1))))
        require(initial_alignment_error > 10 and final_alignment_error < 1,
                "App did not remove the initial image error")
        require(not np.any(sim.velocity_command), "App failed to stop after alignment")
        Image.fromarray(lab.draw()).save(ROOT / "preview.png")
        sim.reset([40, 0, 0, 0, 0, 0])
        lab.align()
        lab.advance(1/30)
        require(lab.alignment_status == "waiting" and lab.aligning, "App did not begin marker recovery")
        require(not np.any(sim.velocity_command), "App did not brake immediately on lost tracking")
        sim.reset()
        report = {
            "status": "PASS",
            "scope": "Scene, manual motion, alignment and immediate recovery braking checks",
            "python": platform.python_version(), "mujoco": mujoco.__version__, "opencv": cv2.__version__,
            "joint_count": sim.model.nv,
            "camera_resolution": [sim.width, sim.height],
            "physics_hz": 1 / sim.model.opt.timestep,
            "camera_sample_hz_simulated": sim.config["camera_hz"],
            "j1_motion_degrees": joint_motion,
            "camera_translation_m": camera_motion,
            "marker_centroid_shift_px": pixel_motion,
            "each_joint_response_degrees": response,
            "zero_command_drift_rad_2s": drift,
            "reset_image_exact_match": True,
            "manual_pulse_stops": True,
            "pause_resume_stop_and_demo": True,
            "app_alignment_initial_error_px": initial_alignment_error,
            "app_alignment_final_error_px": final_alignment_error,
            "app_alignment_and_recovery_braking": True,
            "K": sim.camera_intrinsics().tolist(),
            "world_from_optical_at_home": sim.camera_pose().tolist()
        }
        (directory / "results.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
