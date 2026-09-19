"""Visual servoing desktop lab: manual jogging and automatic marker alignment."""
from __future__ import annotations

import argparse
import json
import time
from datetime import datetime

import cv2
import numpy as np
from PIL import Image

from simulation import ROOT, Simulation
from adaptive_gain import load_gain_config, make_gain_controller
from benchmark import BENCH_CONFIG, read_json, sample_plan
from recovery import ACTIVE_STATES, ReacquiringIBVS
from reference_image import DEFAULT_REFERENCE, load_reference, save_reference
from startup_search import load_startup_config

WINDOW = "Visual Servoing Simulation | Auto align and search"
# Colors are RGB, because MuJoCo returns RGB.
BG = (16, 22, 31)
PANEL = (26, 34, 46)
TEXT = (225, 234, 243)
MUTED = (151, 169, 188)
TEAL = (77, 214, 192)
AMBER = (246, 194, 102)
JOINT_NAMES = ["Base yaw", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Tool roll"]


def label(canvas: np.ndarray, text: str, x: int, y: int,
          size: float = 0.5, color: tuple = TEXT, weight: int = 1) -> None:
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size,
                color, weight, cv2.LINE_AA)


def random_start_offset(seed=None):
    """Use the existing joint-offset distribution, without filtering visibility."""
    if seed is None:
        seed = int(np.random.default_rng().integers(0, 2**32))
    return sample_plan(1, seed, read_json(BENCH_CONFIG))[0]["offset_degrees"]


class Lab:
    def __init__(self, sim: Simulation, reference_path=DEFAULT_REFERENCE,
                 cold_start=False, auto_start=True) -> None:
        self.sim = sim
        self.auto_enabled = auto_start
        self.selected = 0
        self.gain_mode = "fixed"
        self.gain_settings = load_gain_config()
        self.paused = False
        self.demo = False
        self.demo_start = 0.0
        self.pulse_end = 0.0
        self.buttons: list[tuple[tuple[int, int, int, int], str]] = []
        self.message = "Jog or load a starting pose, then Align. Lost view [L] demonstrates target recovery."
        self.last_rgb: np.ndarray | None = None
        self.demo_origin = sim.data.qpos.copy()
        self.reference_path = reference_path
        self.cold_start_pending = cold_start
        self.reference_rgb, self.reference = load_reference(
            reference_path, sim.camera_intrinsics(), sim.config, sim.marker_corners)
        self.controller = ReacquiringIBVS(self.reference, sim.camera_intrinsics(), sim.config,
                                         sim.model.jnt_range)
        if not cold_start:
            self.controller.remember_view(sim.marker_corners(sim.image()), sim.data.qpos)
        self.aligning = False
        self.alignment_status = "idle"
        self._start_if_auto()

    def _start_if_auto(self) -> None:
        # Only explicit start events arm a run. Never restart from the idle loop.
        if self.auto_enabled:
            self.align()

    def toggle_auto(self) -> None:
        self.auto_enabled = not self.auto_enabled
        if self.auto_enabled:
            self.align()
        else:
            self.stop()
            self.message = "Auto OFF. Jog or choose a starting pose, then click Align [G]."

    def toggle_gain(self) -> None:
        # Stop first: switching mode must never retain an active motor command.
        self.stop()
        self.gain_mode = "adaptive" if self.gain_mode == "fixed" else "fixed"
        self.controller.ibvs = make_gain_controller(
            self.gain_mode, self.reference, self.sim.camera_intrinsics(),
            self.sim.config, self.gain_settings)
        self.message = f"{self.gain_mode.capitalize()} gain selected. Click Align [G] to start."

    def offset(self) -> None:
        self.stop()
        self.sim.reset(self.sim.config["ibvs"]["start_offset_degrees"])
        self.paused = False
        self.alignment_status = "idle"
        self.message = "Offset pose loaded. Click Align [G] to close the visual feedback loop."
        self._start_if_auto()

    def lost_view(self) -> None:
        self.reset(start_automatically=False)
        self.sim.reset(self.controller.config["lost_view_demo_offset_degrees"])
        self.message = "Marker is outside the view. Click Align [G] to find it and align."
        self._start_if_auto()

    def cold_start(self) -> None:
        self.stop()
        self.sim.reset(load_startup_config()["demo_offset_degrees"])
        self.controller.last_visible_qpos = None
        self.cold_start_pending = True
        self.paused = False
        self.message = "Cold start: no remembered viewpoint. Click Align [G] to search from here."
        self._start_if_auto()

    def random_start(self, seed=None) -> None:
        self.stop()
        self.sim.reset(random_start_offset(seed))
        self.controller.last_visible_qpos = None
        self.cold_start_pending = True
        self.paused = False
        self.message = "Random start: no remembered viewpoint. Click Align [G] to check the camera."
        self._start_if_auto()

    def teach_reference(self) -> None:
        self.stop()
        rgb = self.sim.image()
        try:
            reference = save_reference(self.reference_path, rgb, self.sim.camera_intrinsics(),
                                       self.sim.config, self.sim.marker_corners)
        except (ValueError, OSError) as exc:
            self.message = str(exc)
            return
        self.reference_rgb, self.reference = rgb.copy(), reference
        self.controller.ibvs = make_gain_controller(
            self.gain_mode, reference, self.sim.camera_intrinsics(), self.sim.config, self.gain_settings)
        self.cold_start_pending = False
        self.controller.remember_view(reference, self.sim.data.qpos)
        self.message = "Reference image saved for future sessions. This visible view is now the alignment goal."

    def align(self) -> None:
        self.stop()
        self.paused = False
        self.controller.start(cold=self.cold_start_pending)
        self.cold_start_pending = False
        self.aligning = True
        self.alignment_status = "checking"
        self.message = "Checking camera: align if the marker is visible; search or recover if it is missing."

    def reset(self, start_automatically=True) -> None:
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        self.sim.reset()
        self.cold_start_pending = False
        self.controller.last_visible_qpos = None
        self.controller.remember_view(self.sim.marker_corners(self.sim.image()), self.sim.data.qpos)
        self.demo = False
        self.paused = False
        self.pulse_end = 0
        self.message = "Home pose restored. Align uses detections from the current scene."
        if start_automatically:
            self._start_if_auto()

    def jog(self, joint: int, direction: int) -> None:
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        self.selected = joint
        self.demo = False
        self.paused = False
        command = np.zeros(6)
        command[joint] = direction * self.sim.config["jog_velocity_rad_s"]
        self.sim.command_velocity(command)
        self.pulse_end = self.sim.data.time + self.sim.config["jog_duration_s"]
        self.message = f"J{joint + 1}: {command[joint]:+.2f} rad/s for {self.sim.config['jog_duration_s']:.2f} s."

    def toggle_demo(self) -> None:
        if self.demo:
            self.stop()
        else:
            self.reset(start_automatically=False)
            self.demo = True
            self.demo_start = self.sim.data.time
            self.demo_origin = self.sim.data.qpos.copy()
            self.message = "Scripted motion demo. Click Stop, then Align to return using image feedback."

    def stop(self) -> None:
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        self.demo = False
        self.pulse_end = 0
        self.sim.command_velocity(np.zeros(6))
        self.message = "Stopped. Click Align [G] or load a new starting pose to begin another run."

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        # Pausing cancels the current pulse and the demo; resuming is stationary.
        self.stop()
        self.message = "Simulation paused." if self.paused else "Simulation resumed."

    def advance(self, seconds: float) -> None:
        if self.paused:
            return
        if self.aligning:
            corners = self.sim.marker_corners(self.sim.image())
            sample = self.controller.update(corners, self.sim.camera_jacobian(),
                                            self.sim.data.qpos, seconds)
            self.sim.command_velocity(sample.velocity)
            self.alignment_status = sample.status
            if sample.status in ACTIVE_STATES:
                self.message = {
                    "running": "Aligning from camera images. Green corners approach the amber reference.",
                    "joint_limited": "Aligning while keeping room at the joint limits.",
                    "repositioning": "Alignment needs a retry. Returning toward a previously observed view.",
                    "retry_confirming": "Checking the returned view before retrying alignment.",
                    "realigning": "Retrying alignment with less wrist rotation.",
                    "waiting": "Marker lost. Holding still briefly before recovery.",
                    "returning": "Finding marker: moving toward the last visible viewpoint.",
                    "scanning": "Finding marker: scanning nearby views.",
                    "confirming": "Marker found. Confirming detection before resuming alignment.",
                }[sample.status]
                if self.controller.mode == "startup":
                    search = self.controller.startup
                    detail = "confirming marker" if sample.status == "confirming" else "no remembered viewpoint"
                    self.message = (f"Startup search ({search.stage}): view {min(search.index+1,len(search.waypoints))}/{len(search.waypoints)}"
                                    f" | {search.elapsed_s:.1f}/{search.config['max_search_time_s']:g} s | {detail}")
            else:
                self.aligning = False
                self.alignment_status = sample.status
                if sample.status == "converged":
                    cfg = self.sim.config["ibvs"]
                    self.message = (f"Aligned: {sample.error_px:.2f} px RMS corner error, held below "
                                    f"{cfg['success_error_px']:g} px for {cfg['success_hold_s']:g} s.")
                elif sample.status == "target_not_found":
                    self.message = "Target not found within the search area/budget. Stopped. Jog to a new start and try Align."
                else:
                    self.message = f"Stopped: {sample.status}. Jog to another view, then try Align again."
        elif self.demo:
            t = self.sim.data.time - self.demo_start
            amplitudes = np.array([0.026, 0.016, 0.020, 0.025, 0.016, 0.03])
            frequencies = np.array([0.8, 0.6, 0.7, 0.6, 0.9, 0.7])
            target = self.demo_origin + amplitudes * np.sin(frequencies * t)
            feedforward = amplitudes * frequencies * np.cos(frequencies * t)
            self.sim.command_velocity(feedforward + 3 * (target - self.sim.data.qpos))
        elif self.pulse_end > self.sim.data.time:
            # Split at the pulse deadline so one click has a fixed duration.
            pulse_duration = min(seconds, self.pulse_end - self.sim.data.time)
            self.sim.advance(pulse_duration)
            seconds -= pulse_duration
            if self.sim.data.time >= self.pulse_end - self.sim.model.opt.timestep:
                self.sim.command_velocity(np.zeros(6))
                self.pulse_end = 0
        else:
            self.sim.command_velocity(np.zeros(6))
        if seconds > 1e-10:
            self.sim.advance(seconds)

    def button(self, canvas: np.ndarray, rect: tuple[int, int, int, int],
               text: str, action: str, active: bool = False) -> None:
        x, y, w, h = rect
        cv2.rectangle(canvas, (x, y), (x+w, y+h), TEAL if active else PANEL, -1)
        label(canvas, text, x+12, y+h//2+5, color=BG if active else TEXT)
        self.buttons.append((rect, action))

    def draw(self) -> np.ndarray:
        canvas = np.full((766, 1200, 3), BG, dtype=np.uint8)
        self.buttons.clear()
        label(canvas, "Visual Servoing", 24, 36, 0.85, TEAL, 2)
        gain = self.controller.ibvs.config["gain_per_s"]
        label(canvas, f"IMAGE-BASED ALIGNMENT / {self.gain_mode.upper()} GAIN {gain:.2f}", 24, 61, 0.44, MUTED)
        self.button(canvas, (390, 23, 138, 34),
                    "Auto ON [B]" if self.auto_enabled else "Auto OFF [B]", "auto", self.auto_enabled)
        self.button(canvas, (542, 23, 114, 34), "Offset [O]", "offset")
        self.button(canvas, (668, 23, 114, 34), "Align [G]", "align", self.aligning)
        self.button(canvas, (794, 23, 114, 34), "Reset [R]", "reset")
        self.button(canvas, (920, 23, 114, 34), "Demo [M]", "demo", self.demo)
        self.button(canvas, (1046, 23, 130, 34), "Play" if self.paused else "Pause", "pause", self.paused)
        label(canvas, "WORLD VIEW", 24, 92, 0.46, MUTED)
        label(canvas, "WRIST CAMERA / 640 x 480", 608, 92, 0.46, MUTED)
        rgb = self.sim.image()
        self.last_rgb = rgb
        world = self.sim.image("world")
        annotated = rgb.copy()
        cv2.polylines(annotated, [self.reference.astype(np.int32)], True, AMBER, 2, cv2.LINE_AA)
        for point in self.reference:
            cv2.drawMarker(annotated, tuple(point.astype(int)), AMBER, cv2.MARKER_CROSS, 12, 1)
        corners = self.sim.marker_corners(rgb)
        if not self.aligning and not self.cold_start_pending:
            self.controller.remember_view(corners, self.sim.data.qpos)
        if corners is not None:
            cv2.polylines(annotated, [corners.astype(np.int32)], True, TEAL, 2, cv2.LINE_AA)
            for i, corner in enumerate(corners):
                cv2.circle(annotated, tuple(corner.astype(int)), 4, TEAL, -1)
                label(annotated, str(i), int(corner[0])+7, int(corner[1])-6, 0.45, TEAL)
        cv2.drawMarker(annotated, (320, 240), (230, 163, 81), cv2.MARKER_CROSS, 18, 1)
        canvas[102:528, 24:592] = cv2.resize(world, (568, 426))
        canvas[102:528, 608:1176] = cv2.resize(annotated, (568, 426))
        active_state = {"joint_limited": "ALIGNING WITH JOINT LIMITS",
                        "repositioning": "REPOSITIONING FOR RETRY",
                        "retry_confirming": "CONFIRMING RETRY",
                        "realigning": "RETRYING ALIGNMENT",
                        "checking": "CHECKING CAMERA", "running": "ALIGNING", "waiting": "WAITING FOR MARKER",
                        "returning": "RETURNING TO LAST VIEW", "scanning": "SEARCHING",
                        "confirming": "CONFIRMING MARKER"}.get(self.alignment_status, "ALIGNING")
        state = ("PAUSED" if self.paused else active_state if self.aligning else "DEMO" if self.demo
                 else self.alignment_status.upper() if self.alignment_status != "idle" else "MANUAL")
        label(canvas, f"{state}   |   simulation time {self.sim.data.time:7.2f} s", 24, 551, 0.45, TEAL)
        if corners is None:
            message = "Marker not detected - finding target" if self.aligning else "Marker not detected - Align [G] can try to recover"
            label(canvas, message, 608, 551, 0.42, AMBER)
        else:
            error = np.sqrt(np.mean(np.sum((corners - self.reference)**2, axis=1)))
            label(canvas, f"Marker 7 visible   |   RMS corner error {error:.2f} px",
                  608, 551, 0.45, TEAL)
        cv2.line(canvas, (24, 571), (1176, 571), PANEL, 1)
        self.button(canvas, (24, 580, 170, 28), "Cold start [N]", "cold_start", self.cold_start_pending)
        self.button(canvas, (208, 580, 170, 28), "Random [P]", "random_start")
        self.button(canvas, (392, 580, 200, 28), "Teach image [H]", "teach")
        self.button(canvas, (608, 580, 238, 28),
                    "Adaptive gain [T]" if self.gain_mode == "adaptive" else "Fixed gain [T]",
                    "gain", self.gain_mode == "adaptive")
        self.button(canvas, (860, 580, 172, 28), "Lost view [L]", "lost_view")
        self.button(canvas, (1046, 580, 130, 28), "Stop", "stop")
        for i, name in enumerate(JOINT_NAMES):
            col, row = i // 3, i % 3
            x, y = 24 + 584 * col, 615 + 37 * row
            color = TEAL if i == self.selected else TEXT
            label(canvas, f"J{i+1}  {name}", x, y+21, 0.5, color)
            label(canvas, f"{np.rad2deg(self.sim.data.qpos[i]):+7.1f} deg", x+250, y+21, 0.47, MUTED)
            self.button(canvas, (x+406, y, 65, 29), "-", f"jog:{i}:-1")
            self.button(canvas, (x+483, y, 65, 29), "+", f"jog:{i}:1")
        label(canvas, self.message, 24, 738, 0.44, MUTED)
        label(canvas, "B: auto   P: random   N: cold start   H: teach   T: gain   O/L: offsets   G: align   1-6: joint   A/D: jog   Space: pause   S: save   Esc: quit", 24, 758, 0.39, MUTED)
        return canvas

    def on_mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for (bx, by, w, h), action in self.buttons:
            if bx <= x <= bx+w and by <= y <= by+h:
                if action.startswith("jog:"):
                    _, joint, direction = action.split(":")
                    self.jog(int(joint), int(direction))
                else:
                    {"reset": self.reset, "demo": self.toggle_demo,
                     "pause": self.toggle_pause, "stop": self.stop,
                     "offset": self.offset, "align": self.align, "lost_view": self.lost_view,
                     "gain": self.toggle_gain, "cold_start": self.cold_start,
                     "teach": self.teach_reference, "auto": self.toggle_auto,
                     "random_start": self.random_start}[action]()
                break

    def save(self) -> None:
        if self.last_rgb is None:
            return
        directory = ROOT / "captures" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        directory.mkdir(parents=True)
        Image.fromarray(self.last_rgb).save(directory / "wrist.png")
        metadata = {"simulation_time_s": self.sim.data.time,
                    "gain_mode": self.gain_mode,
                    "gain_per_s": self.controller.ibvs.config["gain_per_s"],
                    "qpos_rad": self.sim.data.qpos.tolist(),
                    "K": self.sim.camera_intrinsics().tolist(),
                    "world_from_optical": self.sim.camera_pose().tolist()}
        (directory / "camera.json").write_text(json.dumps(metadata, indent=2))
        self.message = "Saved raw wrist.png and camera.json in captures/."

    def run(self) -> None:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
        cv2.resizeWindow(WINDOW, 1200, 766)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        period = 1 / self.sim.config["camera_hz"]
        try:
            while True:
                start = time.perf_counter()
                self.advance(period)
                cv2.imshow(WINDOW, cv2.cvtColor(self.draw(), cv2.COLOR_RGB2BGR))
                key = cv2.waitKeyEx(max(1, int((period - (time.perf_counter()-start))*1000)))
                if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if ord("1") <= key <= ord("6"):
                    self.selected = key - ord("1")
                elif key in (ord("a"), ord("A")):
                    self.jog(self.selected, -1)
                elif key in (ord("d"), ord("D")):
                    self.jog(self.selected, 1)
                elif key in (ord("r"), ord("R")):
                    self.reset()
                elif key in (ord("m"), ord("M")):
                    self.toggle_demo()
                elif key in (ord("o"), ord("O")):
                    self.offset()
                elif key in (ord("l"), ord("L")):
                    self.lost_view()
                elif key in (ord("t"), ord("T")):
                    self.toggle_gain()
                elif key in (ord("b"), ord("B")):
                    self.toggle_auto()
                elif key in (ord("p"), ord("P")):
                    self.random_start()
                elif key in (ord("n"), ord("N")):
                    self.cold_start()
                elif key in (ord("h"), ord("H")):
                    self.teach_reference()
                elif key in (ord("g"), ord("G")):
                    self.align()
                elif key == 32:
                    self.toggle_pause()
                elif key in (ord("s"), ord("S")):
                    self.save()
        finally:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", action="store_true", help="Render a preview without opening a window")
    starts = parser.add_mutually_exclusive_group()
    starts.add_argument("--cold-start", action="store_true", help="Start out of view with no remembered viewpoint")
    starts.add_argument("--random-start", action="store_true", help="Start at a sampled pose without a remembered viewpoint")
    parser.add_argument("--seed", type=int, help="Reproduce a random start (requires --random-start)")
    parser.add_argument("--manual", action="store_true", help="Start with Auto OFF and wait for Align")
    args = parser.parse_args()
    if args.seed is not None and (not args.random_start or args.seed < 0):
        parser.error("--seed must be nonnegative and used with --random-start")
    with Simulation() as sim:
        if args.cold_start:
            # Set the start before any live image can supply a remembered view.
            sim.reset(load_startup_config()["demo_offset_degrees"])
        elif args.random_start:
            sim.reset(random_start_offset(args.seed))
        lab = Lab(sim, cold_start=args.cold_start or args.random_start, auto_start=not args.manual)
        if args.manual and (args.cold_start or args.random_start):
            lab.message = "Starting pose loaded without a remembered view. Click Align [G] when ready."
        if args.snapshot:
            Image.fromarray(lab.draw()).save(ROOT / "preview.png")
        else:
            lab.run()


if __name__ == "__main__":
    main()
