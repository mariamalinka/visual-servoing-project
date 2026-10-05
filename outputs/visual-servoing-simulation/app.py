"""Visual servoing desktop lab: manual jogging and automatic marker alignment."""
from __future__ import annotations

import argparse
import json
import sys
import time
import traceback
from datetime import datetime
from dataclasses import replace
from camera_timing import TimedCamera, load_camera_timing
from collision import CollisionPoseError
from calibration import ControlCalibration, calibration_profiles
from precision import GoalRefinedPerception, load_precision_config

import cv2
import numpy as np
from PIL import Image

from simulation import ROOT, Simulation
from adaptive_gain import load_gain_config, make_gain_controller
from benchmark import BENCH_CONFIG, read_json, sample_plan
from recovery import ACTIVE_STATES, ReacquiringIBVS
from reference_image import DEFAULT_REFERENCE, load_reference, save_reference
from startup_search import load_startup_config
from perception import ArucoPerception, NaturalImagePerception, NATURAL_REFERENCE, Observation

WINDOW = "Visual Servoing Simulation | Auto align and search"
# Colors are RGB, because MuJoCo returns RGB.
BG = (16, 22, 31)
PANEL = (26, 34, 46)
TEXT = (225, 234, 243)
MUTED = (151, 169, 188)
TEAL = (77, 214, 192)
AMBER = (246, 194, 102)
PANEL_BG = (22, 29, 40)
BUTTON = (30, 39, 52)
BORDER = (42, 52, 66)
SELECTED = (36, 65, 63)
TEAL_LIGHT = (127, 230, 214)
DIM = (110, 123, 140)
STOP_RED = (201, 58, 46)
WARN = (232, 162, 58)
JOINT_NAMES = ["Base yaw", "Shoulder", "Elbow", "Wrist roll", "Wrist pitch", "Tool roll"]


def label(canvas: np.ndarray, text: str, x: int, y: int,
          size: float = 0.5, color: tuple = TEXT, weight: int = 1) -> None:
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size,
                color, weight, cv2.LINE_AA)


def text_width(text: str, size: float, weight: int = 1) -> int:
    return cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, weight)[0][0]


def rounded_rect(canvas: np.ndarray, x: int, y: int, w: int, h: int, color: tuple,
                 border: tuple | None = None, radius: int = 5, border_px: int = 1) -> None:
    """Filled rectangle with rounded corners (optionally with a border)."""
    r = max(0, min(radius, w // 2, h // 2))
    for c, inset in (((border, 0),) if border is not None else ()) + ((color, border_px if border is not None else 0),):
        x0, y0, x1, y1, rr = x + inset, y + inset, x + w - 1 - inset, y + h - 1 - inset, max(0, r - inset)
        if rr == 0:
            cv2.rectangle(canvas, (x0, y0), (x1, y1), c, -1)
            continue
        cv2.rectangle(canvas, (x0 + rr, y0), (x1 - rr, y1), c, -1)
        cv2.rectangle(canvas, (x0, y0 + rr), (x1, y1 - rr), c, -1)
        for cx, cy in ((x0 + rr, y0 + rr), (x1 - rr, y0 + rr), (x0 + rr, y1 - rr), (x1 - rr, y1 - rr)):
            cv2.circle(canvas, (cx, cy), rr, c, -1, cv2.LINE_AA)


UI_SIZE = (1200, 830)  # base layout in pixels at scale 1


class Painter:
    """Draws the base 1200 x 830 layout scaled by `scale`; every coordinate passed in is a base unit.

    Text is drawn at the scaled size rather than stretched afterwards, so it stays sharp.
    """

    def __init__(self, canvas: np.ndarray, scale: float):
        self.canvas, self.s = canvas, scale

    def px(self, value: float) -> int:
        return int(round(value * self.s))

    def thick(self, weight: int) -> int:
        # Keep strokes thin until the scale is large enough for a heavier one to read well.
        return weight if self.s < 1.75 else max(1, int(round(weight * self.s * 0.75)))

    def width(self, text: str, size: float, weight: int = 1) -> float:
        return text_width(text, size * self.s, self.thick(weight)) / self.s

    def text(self, text: str, x: float, y: float, size: float = 0.5, color: tuple = TEXT, weight: int = 1) -> None:
        label(self.canvas, text, self.px(x), self.px(y), size * self.s, color, self.thick(weight))

    def scaled(self, rect):
        x, y, w, h = rect
        return (self.px(x), self.px(y), self.px(x + w) - self.px(x), self.px(y + h) - self.px(y))

    def rect(self, x, y, w, h, color, border=None, radius=5) -> None:
        rounded_rect(self.canvas, *self.scaled((x, y, w, h)), color, border=border, radius=self.px(radius),
                     border_px=max(1, int(self.s)))

    def line(self, p1, p2, color, weight=1) -> None:
        cv2.line(self.canvas, (self.px(p1[0]), self.px(p1[1])), (self.px(p2[0]), self.px(p2[1])), color,
                 self.thick(weight), cv2.LINE_AA)

    def circle(self, center, radius, color, weight=-1) -> None:
        cv2.circle(self.canvas, (self.px(center[0]), self.px(center[1])), max(1, self.px(radius)), color,
                   weight if weight < 0 else self.thick(weight), cv2.LINE_AA)

    def box(self, p1, p2, color, weight=1) -> None:
        cv2.rectangle(self.canvas, (self.px(p1[0]), self.px(p1[1])), (self.px(p2[0]), self.px(p2[1])), color,
                      self.thick(weight))

    def image(self, x, y, w, h, image: np.ndarray) -> None:
        sx, sy, sw, sh = self.scaled((x, y, w, h))
        interpolation = cv2.INTER_AREA if sw < image.shape[1] else cv2.INTER_LINEAR
        self.canvas[sy:sy + sh, sx:sx + sw] = cv2.resize(image, (sw, sh), interpolation=interpolation)


def screen_fit_scale() -> float:
    """Largest UI scale (in 0.05 steps, 1.0 to 3.0) whose window fits the Windows work area.

    Makes the process DPI-aware first, so the scale is measured in real pixels and Windows does not
    stretch (and blur) the window afterwards. Other platforms keep 1.0.
    """
    if sys.platform != "win32":
        return 1.0
    import ctypes
    from ctypes import wintypes
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(2)
    except (AttributeError, OSError):
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except (AttributeError, OSError):
            pass
    area = wintypes.RECT()
    if not ctypes.windll.user32.SystemParametersInfoW(0x30, 0, ctypes.byref(area), 0):  # SPI_GETWORKAREA
        return 1.0
    return fit_scale(area.right - area.left, area.bottom - area.top)


def fit_scale(width: int, height: int) -> float:
    """Largest scale in 0.05 steps, from 1 to 3, whose window fits a work area of this size."""
    fit = min((width - 40) / UI_SIZE[0], (height - 80) / UI_SIZE[1])  # room for borders and title bar
    return float(min(3.0, max(1.0, np.floor(fit * 20 + 1e-9) / 20)))


def random_start_offset(seed=None):
    """Use the existing joint-offset distribution, without filtering visibility."""
    if seed is None:
        seed = int(np.random.default_rng().integers(0, 2**32))
    return sample_plan(1, seed, read_json(BENCH_CONFIG))[0]["offset_degrees"]


def make_perception(mode,sim):
    if mode=="aruco":
        return ArucoPerception(lambda rgb: sim.marker_corners(rgb))
    if mode=="natural":
        return NaturalImagePerception()
    if mode=="learned":
        from learned_perception import LearnedImagePerception
        return LearnedImagePerception()
    raise ValueError("Unknown perception mode")


def report_perception_error(mode, exc):
    """Keep the full exception chain even when the GUI retains its old mode."""
    details = (f"\n{datetime.now().isoformat()} selecting {mode}\n"
               f"Python: {sys.executable}\n" + traceback.format_exc())
    print(details, file=sys.stderr, flush=True)
    log_path = ROOT / "logs" / "learned-matcher.log"
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(details)
        location = "logs/learned-matcher.log"
    except OSError as log_error:
        print(f"Could not save {log_path}: {log_error}", file=sys.stderr, flush=True)
        location = "console (log write failed)"
    return f"Mode unchanged: {getattr(exc, 'summary', str(exc))}. Details: {location}"


class Lab:
    def __init__(self, sim: Simulation, reference_path=None,
                 cold_start=False, auto_start=True, perception_mode="aruco", camera_timing=None, calibration=None, precision=True,
                 ui_scale: float = 1.0) -> None:
        if not 0.5 <= ui_scale <= 4.0:
            raise ValueError("ui_scale must be between 0.5 and 4")
        self.ui_scale = float(ui_scale)
        self.view_offset = (0, 0)  # where the drawn layout sits inside the window (letterboxing)
        self.sim = sim
        self.precision_settings = load_precision_config()
        self.precision_enabled = bool(precision and self.precision_settings["enabled"])
        self.calibration = ControlCalibration(calibration)
        self.calibration_name = next((name for name,settings in calibration_profiles().items()
                                      if settings == self.calibration.settings),"custom")
        self.camera = None if camera_timing is None else TimedCamera(camera_timing, sim.config["camera_hz"])
        if self.camera is not None:
            self.camera.reset(float(sim.data.time))
        self._camera_below_since = None
        self.last_camera_failure = None
        self.auto_enabled = auto_start
        self.selected = 0
        self.gain_mode = "fixed"
        self.gain_settings = load_gain_config()
        self.paused = False
        self.demo = False
        self.demo_start = 0.0
        self.pulse_end = 0.0
        self.buttons: list[tuple[tuple[int, int, int, int], str]] = []
        self.drawn: list[tuple[int, int, int, int]] = []
        self.message = "Jog or load a starting pose, then Align. Lost view [L] demonstrates target recovery."
        self.last_rgb: np.ndarray | None = None
        self.last_image_info = None
        self.demo_origin = sim.data.qpos.copy()
        if perception_mode not in ("aruco", "natural", "learned"):
            raise ValueError("Perception mode must be aruco, natural or learned")
        self.perception_mode = perception_mode
        self.perception = make_perception(perception_mode,sim)
        self.picture_mode = "learned" if perception_mode=="learned" else "natural"
        self.reference_paths = {"aruco": DEFAULT_REFERENCE, "natural": NATURAL_REFERENCE, "learned": NATURAL_REFERENCE}
        if reference_path is not None:
            self.reference_paths[perception_mode] = reference_path
            if perception_mode!="aruco":
                self.reference_paths["natural"] = self.reference_paths["learned"] = reference_path
        self.reference_path = self.reference_paths[perception_mode]
        self.show_matches = False
        self.last_observation = None
        sim.set_target_mode("aruco" if perception_mode=="aruco" else "natural")
        self.cold_start_pending = cold_start
        self.reference_rgb, self.reference = load_reference(
            self.reference_path, sim.camera_intrinsics(), self.perception.reference_config(sim.config), self.detect_target)
        self._refine_perception()
        self.controller = ReacquiringIBVS(self.reference, self.control_intrinsics(), self.control_config(),
                                         sim.model.jnt_range, path_planner=sim.collision.plan_path,
                                         collision_detour_degrees=sim.collision.config["search_detour_degrees"])
        if not cold_start and self.camera is None:
            self.controller.remember_view(self.detect_target(sim.image()), sim.data.qpos)
        self.aligning = False
        self.alignment_status = "idle"
        self._start_if_auto()

    def control_intrinsics(self):
        return self.calibration.intrinsics(self.sim.camera_intrinsics())

    def _refine_perception(self):
        base = self.perception.base if isinstance(self.perception,GoalRefinedPerception) else self.perception
        self.perception = (GoalRefinedPerception(base,self.reference_rgb,self.reference,self.precision_settings)
                           if self.precision_enabled else base)

    def control_config(self):
        config = self.calibration.controller_config(self.sim.config)
        if self.precision_enabled:
            config["precision_stop"] = self.precision_settings["stop"].copy()
        return config

    def control_jacobian(self):
        return self.calibration.jacobian(self.sim.camera_jacobian())

    def cycle_calibration(self):
        try:
            profiles = calibration_profiles()
            names = list(profiles)
            index = names.index(self.calibration_name) if self.calibration_name in names else -1
            name = names[(index+1)%len(names)]
            calibration = ControlCalibration(profiles[name])
        except (OSError,ValueError) as exc:
            self.stop()
            self.message = f"Calibration profile unavailable: {exc}"
            return
        self.stop()
        view = self.controller.last_visible_qpos
        self.calibration, self.calibration_name = calibration, name
        self.controller = ReacquiringIBVS(self.reference,self.control_intrinsics(),self.control_config(),
            self.sim.model.jnt_range,path_planner=self.sim.collision.plan_path,
            collision_detour_degrees=self.sim.collision.config["search_detour_degrees"])
        self.controller.ibvs = make_gain_controller(self.gain_mode,self.reference,
            self.control_intrinsics(),self.control_config(),self.gain_settings)
        self.controller.last_visible_qpos = None if view is None else view.copy()
        self._clear_camera()
        self.message = f"Calibration assumption: {name}. Click Align [G] to test this controller model."

    def detect_target(self, rgb):
        self.last_observation = self.perception.observe(rgb)
        return self.last_observation.corners

    def toggle_perception(self) -> None:
        self.select_perception(self.picture_mode if self.perception_mode=="aruco" else "aruco")

    def toggle_matcher(self) -> None:
        if self.perception_mode=="aruco":
            self.message = "Select Picture [V], then use K to switch SIFT / Learned matching."
            return
        self.select_perception("learned" if self.perception_mode=="natural" else "natural")

    def select_perception(self, mode) -> None:
        self.stop()
        try:
            perception = make_perception(mode,self.sim)
            path = self.reference_paths[mode]
            rgb, reference = load_reference(path, self.sim.camera_intrinsics(),
                perception.reference_config(self.sim.config), lambda frame: perception.observe(frame).corners)
        except (OSError, ValueError) as exc:
            self.message = report_perception_error(mode, exc)
            return
        self.sim.set_target_mode("aruco" if mode=="aruco" else "natural")
        self.perception_mode, self.perception = mode, perception
        if mode!="aruco":
            self.picture_mode=mode
        self.reference_path, self.reference_rgb, self.reference = path, rgb, reference
        self._refine_perception()
        self.last_observation = None
        self.show_matches = False
        self.controller = ReacquiringIBVS(reference, self.control_intrinsics(),
                                          self.control_config(), self.sim.model.jnt_range,
                                          path_planner=self.sim.collision.plan_path,
                                          collision_detour_degrees=self.sim.collision.config["search_detour_degrees"])
        self.controller.ibvs = make_gain_controller(self.gain_mode, reference,
            self.control_intrinsics(), self.control_config(), self.gain_settings)
        self.cold_start_pending = True
        self.message = f"{perception.name} selected. Align [G] checks the current view."
        self._start_if_auto()

    def toggle_matches(self) -> None:
        if self.perception_mode != "aruco":
            self.show_matches = not self.show_matches
        else:
            self.message = "Select Picture [V] to inspect natural-feature matches."

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
            self.gain_mode, self.reference, self.control_intrinsics(),
            self.control_config(), self.gain_settings)
        self.message = f"{self.gain_mode.capitalize()} gain selected. Click Align [G] to start."

    def offset(self) -> None:
        self.stop()
        if not self._reset_pose(self.sim.config["ibvs"]["start_offset_degrees"]):
            return
        self._clear_camera()
        self.paused = False
        self.alignment_status = "idle"
        self.message = "Offset pose loaded. Click Align [G] to close the visual feedback loop."
        self._start_if_auto()

    def lost_view(self) -> None:
        # Retain only an actually observed viewpoint, including with delayed delivery.
        # Validate the requested preset before changing the current pose.
        self.stop()
        if not self._reset_pose(self.controller.config["lost_view_demo_offset_degrees"]):
            return
        self._clear_camera()
        self.cold_start_pending = False
        self.paused = False
        self.message = "Target is outside the view. Click Align [G] to find it and align."
        self._start_if_auto()

    def cold_start(self) -> None:
        self.stop()
        if not self._reset_pose(load_startup_config()["demo_offset_degrees"]):
            return
        self._clear_camera()
        self.controller.last_visible_qpos = None
        self.cold_start_pending = True
        self.paused = False
        self.message = "Cold start: no remembered viewpoint. Click Align [G] to search from here."
        self._start_if_auto()

    def random_start(self, seed=None) -> None:
        self.stop()
        if not self._reset_pose(random_start_offset(seed)):
            return
        self._clear_camera()
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
                                       self.perception.reference_config(self.sim.config),
                                       lambda frame: (self.perception.base if isinstance(self.perception,GoalRefinedPerception) else self.perception).observe(frame).corners)
        except (ValueError, OSError) as exc:
            self.message = str(exc)
            return
        self.reference_rgb, self.reference = rgb.copy(), reference
        self._refine_perception()
        self._clear_camera()
        self.controller.ibvs = make_gain_controller(
            self.gain_mode, reference, self.control_intrinsics(), self.control_config(), self.gain_settings)
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
        self.message = "Checking camera: align if the target is visible; search or recover if it is missing."

    def reset(self, start_automatically=True) -> None:
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        if not self._reset_pose():
            return
        self._clear_camera()
        self.cold_start_pending = False
        self.controller.last_visible_qpos = None
        if self.camera is None:
            self.controller.remember_view(self.detect_target(self.sim.image()), self.sim.data.qpos)
        self.demo = False
        self.paused = False
        self.pulse_end = 0
        self.message = "Home pose restored. Align uses detections from the current scene."
        if start_automatically:
            self._start_if_auto()

    def jog(self, joint: int, direction: int) -> None:
        self._clear_camera()
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
        self._clear_camera()
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        self.demo = False
        self.pulse_end = 0
        self.sim.command_velocity(np.zeros(6))
        self.sim.collision_event = None
        self.message = "Stopped. Click Align [G] or load a new starting pose to begin another run."

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        # Pausing cancels the current pulse and the demo; resuming is stationary.
        self.stop()
        self.message = "Simulation paused." if self.paused else "Simulation resumed."


    def _reset_pose(self, offset=None):
        try:
            self.sim.reset(offset)
            return True
        except CollisionPoseError as exc:
            self.sim.command_velocity(np.zeros(6))
            self.demo = False
            self.pulse_end = 0
            self.message = str(exc)
            return False

    def toggle_obstacle(self):
        self.stop()
        try:
            self.sim.set_obstacle(not self.sim.obstacle_enabled)
        except CollisionPoseError as exc:
            self.message = str(exc)
            return
        self.controller.last_visible_qpos = None
        self.message = ("Obstacle enabled. Cold start [N] demonstrates checked search paths." if self.sim.obstacle_enabled
                        else "Obstacle removed. Collision checks remain active.")

    def _handle_collision(self):
        event = self.sim.collision_event
        if event is None:
            return
        self.sim.collision_event = None
        if self.aligning and self.controller.skip_blocked_motion():
            self.sim.command_velocity(np.zeros(6))
            self.message = "Search path blocked by geometry. Checking another bounded waypoint."
            return
        if self.alignment_status in ("stale_camera","camera_error","camera_run_timeout"):
            return
        self.stop()
        self.alignment_status = "collision_blocked"
        pair = " / ".join(event["pair"] or ())
        self.message = f"Stopped for collision clearance: {pair}. Choose another path and Align [G]."

    def _clear_camera(self):
        if self.camera is not None:
            self.camera.reset(float(self.sim.data.time))
            self.last_observation = None
            self.last_rgb = None
            self.last_image_info = None
            self._camera_below_since = None
            self.last_camera_failure = None

    def cycle_camera_delay(self):
        self.stop()
        presets = (None, 0.0, 0.05, 0.1, 0.2)
        current = None if self.camera is None else self.camera.config["delay_s"]
        index = next((i for i, value in enumerate(presets) if value == current), 0)
        delay = presets[(index + 1) % len(presets)]
        self.camera = (None if delay is None else
                       TimedCamera(load_camera_timing(delay), self.sim.config["camera_hz"]))
        self._clear_camera()
        self.message = ("Immediate camera selected. Click Align [G]." if delay is None else
                        f"Camera delay {delay*1000:.0f} ms. Click Align [G]; X interrupts the camera stream.")

    def toggle_camera_stream(self):
        if self.camera is None:
            self.message = "Select a camera delay with C before interrupting its stream."
            return
        self.camera.stream_enabled = not self.camera.stream_enabled
        self.message = ("Camera stream restored. Align [G] starts a stopped run." if self.camera.stream_enabled
                        else "Camera stream interrupted. Motion stops when the last observation expires.")

    def _camera_failure(self, status):
        self.last_camera_failure = dict(status=status, simulation_time_s=float(self.sim.data.time),
                                        captured_s=None if self.camera.latest is None else self.camera.latest.captured_s)
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = status
        self.demo = False
        self.pulse_end = 0
        self.sim.command_velocity(np.zeros(6))
        self.message = ("Stopped: camera feedback is too old. Restore the stream [X], then Align [G]."
                        if status == "stale_camera" else "Stopped: timed-camera run deadline reached.")

    def _camera_tick(self):
        camera = self.camera
        now = float(self.sim.data.time)
        if camera.failure(now) == "camera_clock_reset":
            self._camera_failure("camera_clock_reset")
            failure = self.last_camera_failure
            self._clear_camera()
            self.controller.last_visible_qpos = None
            self.last_camera_failure = failure
            self.message = "Stopped: simulation clock changed. Align [G] starts with a fresh camera queue."
            return
        if camera.capture_due(now):
            rgb = self.sim.image()
            qpos = self.sim.data.qpos.copy()
            camera_pose = self.sim.camera_pose()
            started = time.perf_counter()
            try:
                observation = self.perception.observe(rgb)
                camera.submit(now, rgb, qpos, observation, 1000*(time.perf_counter()-started), camera_pose)
            except (RuntimeError, OSError, ValueError, cv2.error) as exc:
                self._camera_failure("camera_error")
                camera.stream_enabled = False
                camera.pending.clear()
                self.message = f"Stopped: camera error {type(exc).__name__}: {str(exc)[:95]}. See console."
                traceback.print_exc()
                return
        frame = camera.receive(now)
        if frame is not None:
            self.last_observation = frame.observation
            if frame.observation.reason == "inference_failed":
                self._camera_failure("camera_error")
                camera.stream_enabled = False
                camera.pending.clear()
                self.message = "Stopped: camera inference failed. Check the matcher, then restart Align [G]."
                return
            if self.aligning:
                sample = self.controller.update(
                    frame.observation.corners, self.control_jacobian(),
                    self.sim.data.qpos, camera.capture_elapsed_s, observation_qpos=frame.qpos)
                # The hold window must span distinct captured images, not GUI
                # draws, repeated reads, or a period with no new camera frames.
                if sample.stop_candidate:
                    if self._camera_below_since is None:
                        self._camera_below_since = frame.captured_s
                    held = frame.captured_s - self._camera_below_since
                    if sample.status == "converged" and held + 1e-9 < self.sim.config["ibvs"]["success_hold_s"]:
                        sample = replace(sample, status="running")
                        self.controller.status = self.controller.ibvs.status = "running"
                else:
                    self._camera_below_since = None
                self._apply_control_sample(sample)
            elif not self.cold_start_pending:
                self.controller.remember_view(frame.observation.corners, frame.qpos)
        if self.aligning:
            failure = camera.failure(now)
            if failure is not None:
                self._camera_failure(failure)

    def advance(self, seconds: float) -> None:
        if self.camera is None:
            self._advance_immediate(seconds)
            return
        if not np.isfinite(seconds) or seconds < 0:
            raise ValueError("Duration must be finite and nonnegative")
        if self.paused:
            return
        # Rendering and MuJoCo stay on the context's owning thread. Delivery and
        # the watchdog are checked at every physics tick, including without an
        # observation. Held commands therefore move the arm during camera delay.
        remaining = seconds
        while remaining > 1e-10:
            self._camera_tick()
            step = min(float(self.sim.model.opt.timestep), remaining)
            if self.aligning:
                self.sim.advance(step)
            else:
                self._advance_immediate(step)
            self._handle_collision()
            remaining -= step
        # Enforce expiry at the endpoint too; a caller cannot observe an expired
        # nonzero command between calls to advance().
        if self.aligning and self.camera.failure(float(self.sim.data.time)) is not None:
            self._camera_failure(self.camera.failure(float(self.sim.data.time)))

    def _apply_control_sample(self, sample):
        self.sim.command_velocity(sample.velocity)
        self.alignment_status = sample.status
        if sample.status in ACTIVE_STATES:
            self.message = {
                "running": "Aligning from camera images. Green corners approach the amber reference.",
                "joint_limited": "Aligning while keeping room at the joint limits.",
                "repositioning": "Alignment needs a retry. Returning toward a previously observed view.",
                "retry_confirming": "Checking the returned view before retrying alignment.",
                "realigning": "Retrying alignment with less wrist rotation.",
                "waiting": "Target lost. Holding still briefly before recovery.",
                "returning": "Finding target: moving toward the last visible viewpoint.",
                "scanning": "Finding target: scanning nearby views.",
                "confirming": "Target found. Confirming detection before resuming alignment.",
            }[sample.status]
            if sample.stop_metrics is not None and sample.error_px <= self.precision_settings["entry_error_px"]:
                m=sample.stop_metrics
                self.message=(f"Final alignment: {sample.error_px:.2f} px | estimated camera correction "
                              f"{m['position_correction_mm']:.2f} mm / {m['rotation_correction_deg']:.2f} deg.")
            if self.controller.mode == "startup":
                search = self.controller.startup
                detail = "confirming target" if sample.status == "confirming" else "no remembered viewpoint"
                self.message = (f"Startup search ({search.stage}): view {min(search.index+1,len(search.waypoints))}/{len(search.waypoints)}"
                                f" | {search.elapsed_s:.1f}/{search.config['max_search_time_s']:g} s | {detail}")
        else:
            self.aligning = False
            self.alignment_status = sample.status
            if sample.status == "converged":
                cfg = self.sim.config["ibvs"]
                self.message = (f"Aligned: {sample.error_px:.2f} px; precision image/motion checks held {cfg['success_hold_s']:g} s." if self.precision_enabled else
                                f"Aligned: {sample.error_px:.2f} px RMS corner error, held below "
                                f"{cfg['success_error_px']:g} px for {cfg['success_hold_s']:g} s.")
            elif sample.status == "target_not_found":
                self.message = "Target not found within the search area/budget. Stopped. Jog to a new start and try Align."
            else:
                self.message = f"Stopped: {sample.status}. Jog to another view, then try Align again."

    def _advance_immediate(self, seconds: float) -> None:
        if self.paused:
            return
        if self.aligning:
            corners = self.detect_target(self.sim.image())
            sample = self.controller.update(corners, self.control_jacobian(),
                                            self.sim.data.qpos, seconds)
            self._apply_control_sample(sample)
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
        self._handle_collision()

    # ------------------------------------------------------------------ drawing helpers

    def button(self, canvas: np.ndarray, rect: tuple[int, int, int, int], text: str, action: str | None,
               active: bool = False, key: str | None = None, style: str = "normal") -> None:
        """Draw a button and register its click area. style: normal, primary, stop or disabled.

        `active` marks a switched-on toggle. A disabled button (or action None) is drawn only.
        """
        p = self.p
        x, y, w, h = rect
        fill, color, key_color = BUTTON, TEXT, MUTED
        if style == "primary" or (style == "normal" and active):
            fill, color, key_color = (TEAL, BG, BG) if style == "primary" else (SELECTED, TEAL_LIGHT, TEAL_LIGHT)
        elif style == "stop":
            fill, color, key_color = STOP_RED, (255, 255, 255), (255, 255, 255)
        elif style == "disabled":
            color, key_color = DIM, DIM
        elif style == "warn":
            fill, color, key_color = (58, 42, 20), AMBER, AMBER
        p.rect(x, y, w, h, fill, border=None if style in ("primary", "stop") or active else BORDER)
        size = 0.5 if h >= 34 and w >= 140 else 0.46 if h >= 30 else 0.42
        weight = 2 if style in ("primary", "stop") else 1
        ty = y + h // 2 + int(10 * size)
        tx = x + 12
        if style == "stop":
            p.box((x + 14, y + h // 2 - 6), (x + 26, y + h // 2 + 6), color, 2)
            tx = x + 36
        if key:
            p.text(text, tx, ty, size, color, weight)
            kw = p.width(key, 0.36)
            p.text(key, x + w - 10 - kw, ty - 1, 0.36, key_color)
        else:
            p.text(text, x + (w - p.width(text, size, weight)) // 2, ty, size, color, weight)
        self.drawn.append(p.scaled(rect))
        if action is not None and style != "disabled":
            self.buttons.append((p.scaled(rect), action))

    def segmented(self, canvas, x, y, h, options, key=None, label_size=0.38):
        """Side-by-side options; the selected one is drawn highlighted and is not clickable.

        options: (text, selected, action) per segment. Returns the right edge.
        """
        p = self.p
        for text, selected, action in options:
            w = p.width(text, label_size) + 18
            p.rect(x, y, w, h, SELECTED if selected else BUTTON, border=BORDER, radius=3)
            self.drawn.append(p.scaled((x, y, w, h)))
            p.text(text, x + 9, y + h // 2 + 4, label_size, TEAL_LIGHT if selected else MUTED)
            if not selected and action is not None:
                self.buttons.append((p.scaled((x, y, w, h)), action))
            x += w
        if key:
            p.text(key, x + 5, y + h // 2 + 4, 0.34, DIM)
            x += p.width(key, 0.34) + 5
        return x

    def status_state(self) -> tuple[str, tuple]:
        active_state = {"joint_limited": "ALIGNING WITH JOINT LIMITS",
                        "repositioning": "REPOSITIONING FOR RETRY",
                        "retry_confirming": "CONFIRMING RETRY",
                        "realigning": "RETRYING ALIGNMENT",
                        "checking": "CHECKING CAMERA", "running": "ALIGNING", "waiting": "WAITING FOR TARGET",
                        "returning": "RETURNING TO LAST VIEW", "scanning": "SEARCHING",
                        "confirming": "CONFIRMING TARGET"}.get(self.alignment_status, "ALIGNING")
        if self.paused:
            return "PAUSED", MUTED
        if self.aligning:
            return active_state, TEAL
        if self.demo:
            return "DEMO", TEAL
        if self.alignment_status == "converged":
            return "CONVERGED", TEAL
        if self.alignment_status == "idle":
            return "MANUAL", TEXT
        return "STOPPED - " + self.alignment_status.replace("_", " "), AMBER

    # ------------------------------------------------------------------ frame

    def draw(self) -> np.ndarray:
        p = self.p = Painter(np.full((round(UI_SIZE[1] * self.ui_scale), round(UI_SIZE[0] * self.ui_scale), 3),
                                     BG, dtype=np.uint8), self.ui_scale)
        canvas = p.canvas
        self.buttons.clear()
        self.drawn = []  # every control drawn this frame, clickable or not (layout checks)
        state, state_color = self.status_state()

        # Header: title and the run controls (Stop set apart).
        p.text("Visual Servoing", 24, 38, 0.85, TEAL, 2)
        p.text("IMAGE-BASED ALIGNMENT / LAB MODE", 24, 60, 0.4, MUTED)
        self.button(canvas, (650, 16, 132, 44), "Align", "align", key="G",
                    style="primary" if not self.aligning else "normal", active=self.aligning)
        self.button(canvas, (790, 16, 120, 44), "Play" if self.paused else "Pause", "pause", self.paused, key="Space")
        self.button(canvas, (918, 16, 104, 44), "Reset", "reset", key="R")
        p.line((1029, 22), (1029, 54), BORDER)
        self.button(canvas, (1036, 16, 140, 44), "STOP", "stop", key="E", style="stop")

        # View captions with the camera's own controls.
        matching_view = self.show_matches and self.perception_mode != "aruco"
        p.text("TEMPLATE / CAMERA MATCHES" if matching_view else "WORLD VIEW", 24, 86, 0.4, MUTED)
        sim_time = f"{self.sim.data.time:.2f} s"
        p.text(sim_time, 592 - p.width(sim_time, 0.42), 86, 0.42, TEXT)
        p.text("sim time", 592 - p.width(sim_time, 0.42) - p.width("sim time ", 0.4), 86, 0.4, MUTED)
        p.text("WRIST CAMERA", 608, 86, 0.4, MUTED)
        # Camera controls, placed right to left so they never overlap the caption.
        picture = "disabled" if self.perception_mode == "aruco" else "normal"
        stream_on = self.camera is not None and self.camera.stream_enabled
        right = 1176
        for text, action, active, key, style in (
                ("Stream" if stream_on or self.camera is None else "No stream", "camera_stream", False, "X",
                 "disabled" if self.camera is None else "normal" if stream_on else "warn"),
                ("Matches", "matches", matching_view, "F", picture),
                ("Learned" if self.perception_mode == "learned" else "SIFT", "matcher",
                 self.perception_mode == "learned", "K", picture)):
            w = p.width(text, 0.42) + p.width(key, 0.36) + 34
            right -= w
            self.button(canvas, (right, 68, w, 24), text, action, active, key=key, style=style)
            right -= 6
        options = [("ArUco", self.perception_mode == "aruco", "perception"),
                   ("Picture", self.perception_mode != "aruco", "perception")]
        width = sum(p.width(t, 0.38) + 18 for t, _, _ in options) + p.width("V", 0.34) + 5
        self.segmented(canvas, right - 2 - width, 68, 24, options, key="V")

        frame = None if self.camera is None else self.camera.latest
        rgb = self.sim.image() if frame is None else frame.rgb
        self.last_rgb = rgb
        self.last_image_info = dict(
            simulation_time_s=float(self.sim.data.time) if frame is None else frame.captured_s,
            frame_id=None if frame is None else frame.sequence,
            frame_source="live_preview" if frame is None else "delivered_camera",
            delivered_simulation_time_s=None if frame is None else self.camera.delivered_s,
            qpos_rad=(self.sim.data.qpos if frame is None else frame.qpos).tolist(),
            world_from_optical=(self.sim.camera_pose().tolist() if frame is None else
                                None if frame.world_from_optical is None else frame.world_from_optical.tolist()))
        world = None if matching_view else self.sim.image("world")
        annotated = rgb.copy()
        cv2.polylines(annotated, [self.reference.astype(np.int32)], True, AMBER, 2, cv2.LINE_AA)
        for point in self.reference:
            cv2.drawMarker(annotated, tuple(point.astype(int)), AMBER, cv2.MARKER_CROSS, 12, 1)
        corners = (self.detect_target(rgb) if self.camera is None else
                   None if frame is None else frame.observation.corners)
        if self.camera is None and not self.aligning and not self.cold_start_pending:
            self.controller.remember_view(corners, self.sim.data.qpos)
        if corners is not None:
            cv2.polylines(annotated, [corners.astype(np.int32)], True, TEAL, 2, cv2.LINE_AA)
            for i, corner in enumerate(corners):
                cv2.circle(annotated, tuple(corner.astype(int)), 4, TEAL, -1)
                label(annotated, str(i), int(corner[0])+7, int(corner[1])-6, 0.45, TEAL)
        observation = self.last_observation or Observation(reason="waiting_for_camera")
        if self.perception_mode != "aruco" and observation is not None:
            color = TEAL if corners is not None else AMBER
            for point in observation.image_points:
                cv2.circle(annotated, tuple(np.round(point).astype(int)), 2, color, -1)
        if matching_view:
            view = self.perception.match_view(rgb, observation)
            scale = min(568/view.shape[1], 426/view.shape[0])
            resized = cv2.resize(view, (round(view.shape[1]*scale), round(view.shape[0]*scale)))
            world = np.full((426,568,3), BG, np.uint8)
            y, x = (426-resized.shape[0])//2, (568-resized.shape[1])//2
            world[y:y+resized.shape[0], x:x+resized.shape[1]] = resized
        cv2.drawMarker(annotated, (320, 240), (230, 163, 81), cv2.MARKER_CROSS, 18, 1)
        p.image(24, 100, 568, 426, world)
        camera_view = annotated
        now = float(self.sim.data.time)
        age = None if self.camera is None else self.camera.age_s(now)
        limit = None if self.camera is None else self.camera.config["max_observation_age_s"]
        stale = age is not None and age >= limit
        no_frame = self.camera is not None and (stale or not self.camera.stream_enabled)
        if no_frame:
            # Dim the last delivered image so it cannot be mistaken for a live view.
            camera_view = (camera_view * 0.35 + np.array(BG) * 0.65).astype(np.uint8)
        p.image(608, 100, 568, 426, camera_view)
        if no_frame:
            p.box((609, 101), (1174, 524), WARN, 2)
            title = "No new frame"
            detail = ("Waiting for the first delivered image" if age is None else
                      f"Last delivered image is {age*1000:.0f} ms old" + ("" if self.camera.stream_enabled else "; stream off"))
            p.text(title, 892 - p.width(title, 0.6, 2) / 2, 305, 0.6, AMBER, 2)
            p.text(detail, 892 - p.width(detail, 0.45) / 2, 332, 0.45, TEXT)

        # Status strip: alignment, camera age, collision guard, target and precision.
        self.draw_status(canvas, state, state_color, corners, observation, frame, age, limit)

        # Bottom panels.
        self.draw_start_panel(canvas)
        self.draw_jog_panel(canvas)
        self.draw_settings_panel(canvas)

        failed = state_color == AMBER
        p.text(self.message[:150], 24, 820, 0.42, AMBER if failed else MUTED)
        hint = "S save capture   Esc quit"
        p.text(hint, 1176 - p.width(hint, 0.36), 820, 0.36, DIM)
        return canvas

    def draw_status(self, canvas, state, state_color, corners, observation, frame, age, limit):
        p = self.p
        x0, y0, h = 24, 536, 66
        widths = (258, 330, 282, 282)
        p.rect(x0, y0, 1152, h, BORDER, radius=6)
        x = x0
        cells = []
        for i, w in enumerate(widths):
            left = x + (0 if i == 0 else 1)
            right = x + w - (0 if i == len(widths) - 1 else 0)
            p.rect(left + 1, y0 + 1, right - left - 1, h - 2, PANEL_BG,
                         radius=5 if i in (0, len(widths) - 1) else 0)
            cells.append(left + 14)
            x += w
        caption = 0.36
        # Alignment
        p.text("ALIGNMENT", cells[0], y0 + 20, caption, MUTED)
        p.circle((cells[0] + 5, y0 + 43), 5, state_color)
        p.text(state[:28], cells[0] + 16, y0 + 48, 0.48 if len(state) < 20 else 0.4, state_color, 1)
        # Camera image age against the freshness limit
        cx, cw = cells[1], widths[1] - 28
        p.text("CAMERA IMAGE AGE", cx, y0 + 20, caption, MUTED)
        bar_y = y0 + 30
        p.rect(cx, bar_y, cw, 8, (37, 47, 61), radius=4)
        if self.camera is None:
            value, note, color = "live", "no simulated delay (C adds one)", MUTED
        elif age is None:
            value, note, color = f"- / {limit*1000:.0f} ms", "waiting for the first delivered image", MUTED
        else:
            over = age >= limit
            color = WARN if over else TEAL
            value = f"{age*1000:.0f} / {limit*1000:.0f} ms"
            fill = int(cw * min(age / limit, 1.0))
            if fill > 0:
                p.rect(cx, bar_y, max(fill, 8), 8, color, radius=4)
            note = ("over the limit" if over else "fresh") + f" | frame {frame.sequence}" if frame is not None else ""
        p.text(value, cx + cw - p.width(value, 0.36), y0 + 20, 0.36, AMBER if color == WARN else
              TEXT if color == TEAL else MUTED)
        p.text(note, cx, y0 + 56, 0.36, MUTED)
        # Collision guard
        decision = self.sim.collision.last
        gx = cells[2]
        p.text("COLLISION GUARD", gx, y0 + 20, caption, MUTED)
        warn = decision.status in ("limited", "blocked")
        status = decision.status.upper()
        p.text(status, gx, y0 + 40, 0.45, AMBER if warn else TEAL, 1)
        distance = "not measured" if decision.clearance_m is None else f"{decision.clearance_m*1000:.1f} mm"
        p.text(f"clearance {distance}", gx + p.width(status, 0.45) + 10, y0 + 40, 0.4, TEXT)
        skips = self.controller.blocked_waypoints + (0 if self.controller.startup is None else self.controller.startup.blocked_waypoints)
        p.text(f"detours {self.sim.collision.detours} | skipped {skips}", gx, y0 + 56, 0.36, MUTED)
        # Target and precision
        px = cells[3]
        target = "MARKER 7" if self.perception_mode == "aruco" else (
            ("LEARNED" if self.perception_mode == "learned" else "SIFT") + " PICTURE")
        p.text(f"TARGET / {target}", px, y0 + 20, caption, MUTED)
        if corners is None:
            reason = "not detected" if self.perception_mode == "aruco" else observation.reason.replace("_", " ")
            p.text(reason[:32], px, y0 + 40, 0.42, AMBER)
        else:
            error = np.sqrt(np.mean(np.sum((corners - self.reference)**2, axis=1)))
            text = f"RMS {error:.2f} px"
            if self.perception_mode != "aruco":
                text += f" | {observation.inliers} inliers | {observation.processing_ms:.0f} ms"
            p.text(text, px, y0 + 40, 0.42, TEXT)
        hold = getattr(self.controller.ibvs, "held_seconds", 0.0) or 0.0
        hold_s = self.sim.config["ibvs"]["success_hold_s"]
        moving = float(np.abs(self.sim.velocity_command).max()) > 1e-9
        p.text(f"hold {hold:.1f} / {hold_s:g} s | command {'moving' if moving else 'zero'}",
              px, y0 + 56, 0.36, MUTED)

    def draw_panel(self, canvas, x, y, w, h, title, note=None):
        p = self.p
        p.rect(x, y, w, h, PANEL_BG, border=BORDER, radius=6)
        p.text(title, x + 12, y + 20, 0.36, MUTED)
        if note:
            p.text(note, x + w - 12 - p.width(note, 0.34), y + 20, 0.34, DIM)

    def draw_start_panel(self, canvas):
        x, y, w = 24, 612, 296
        self.draw_panel(canvas, x, y, w, 190, "STARTING POSE")
        starts = (("Offset", "offset", "O", False), ("Cold start", "cold_start", "N", self.cold_start_pending),
                  ("Random", "random_start", "P", False), ("Lost view", "lost_view", "L", False),
                  ("Teach image", "teach", "H", False))
        bw = (w - 24 - 8) // 2
        for i, (text, action, key, active) in enumerate(starts):
            col, row = i % 2, i // 2
            self.button(canvas, (x + 12 + col * (bw + 8), y + 32 + row * 42, bw, 34), text, action, active, key=key)

    def draw_jog_panel(self, canvas):
        p = self.p
        x, y, w = 336, 612, 524
        self.draw_panel(canvas, x, y, w, 190, "JOG JOINTS", "1-6 select | A / D jog")
        col_w = (w - 24 - 16) // 2
        for i, name in enumerate(JOINT_NAMES):
            col, row = i // 3, i % 3
            jx, jy = x + 12 + col * (col_w + 16), y + 34 + row * 46
            color = TEAL if i == self.selected else TEXT
            p.text(f"J{i+1}", jx, jy + 21, 0.42, TEAL if i == self.selected else MUTED)
            p.text(name, jx + 28, jy + 21, 0.45, color)
            angle = f"{np.rad2deg(self.sim.data.qpos[i]):+.1f}"
            ax = jx + col_w - 76 - 8 - p.width(angle, 0.42) - 6
            p.text(angle, ax, jy + 21, 0.42, TEXT)
            p.circle((ax + p.width(angle, 0.42) + 4, jy + 10), 2, TEXT, 1)
            self.button(canvas, (jx + col_w - 76, jy, 36, 32), "-", f"jog:{i}:-1")
            self.button(canvas, (jx + col_w - 36, jy, 36, 32), "+", f"jog:{i}:1")

    def draw_settings_panel(self, canvas):
        p = self.p
        x, y, w = 876, 612, 300
        self.draw_panel(canvas, x, y, w, 190, "SETTINGS")
        gain = self.controller.ibvs.config["gain_per_s"]
        delay = "Off" if self.camera is None else f"{self.camera.config['delay_s']*1000:.0f} ms"
        rows = (
            ("Auto start", "B", [("On", self.auto_enabled, "auto"), ("Off", not self.auto_enabled, "auto")]),
            (f"Gain {gain:.2f}", "T", [("Fixed", self.gain_mode == "fixed", "gain"),
                                       ("Adaptive", self.gain_mode == "adaptive", "gain")]),
            ("Camera delay", "C", ("cycle", delay, "camera_delay", self.camera is not None)),
            ("Obstacle", "U", [("Off", not self.sim.obstacle_enabled, "obstacle"),
                               ("On", self.sim.obstacle_enabled, "obstacle")]),
            ("Calibration", "I", ("cycle", self.calibration_name, "calibration", self.calibration_name != "nominal")),
            ("Demo loop", "M", [("Off", not self.demo, "demo"), ("On", self.demo, "demo")]),
        )
        for i, (name, key, control) in enumerate(rows):
            ry = y + 32 + i * 26
            p.text(name, x + 12, ry + 15, 0.4, TEXT)
            p.text(key, x + 12 + p.width(name, 0.4) + 6, ry + 15, 0.34, DIM)
            if isinstance(control, tuple):
                _, value, action, changed = control
                text = f"{value} >"
                bw = p.width(text, 0.38) + 18
                self.button(canvas, (x + w - 12 - bw, ry, bw, 21), text, action, changed)
            else:
                width = sum(p.width(t, 0.38) + 18 for t, _, _ in control)
                self.segmented(canvas, x + w - 12 - width, ry, 21, control)

    def frame_for_window(self, width: int, height: int) -> np.ndarray:
        """The layout drawn to fit a window of this size, centred with background bars.

        The layout keeps its proportions at any window size: it is redrawn at the largest scale that
        fits (so text stays sharp) and the rest of the window is filled, never stretched.
        """
        fit = min(width / UI_SIZE[0], height / UI_SIZE[1])
        self.ui_scale = float(min(4.0, max(0.5, np.floor(fit * 100) / 100)))
        canvas = self.draw()
        h, w = canvas.shape[:2]
        if w > width or h > height:  # window smaller than the minimum scale: show the layout as is
            self.view_offset = (0, 0)
            return canvas
        frame = np.full((height, width, 3), BG, dtype=np.uint8)
        x, y = (width - w) // 2, (height - h) // 2
        frame[y:y + h, x:x + w] = canvas
        self.view_offset = (x, y)
        return frame

    def on_mouse(self, event: int, x: int, y: int, flags: int, param: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        x, y = x - self.view_offset[0], y - self.view_offset[1]
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
                     "random_start": self.random_start, "perception": self.toggle_perception,
                     "matches": self.toggle_matches, "matcher": self.toggle_matcher,
                     "camera_delay": self.cycle_camera_delay, "obstacle": self.toggle_obstacle,
                     "calibration": self.cycle_calibration,
                     "camera_stream": self.toggle_camera_stream}[action]()
                break

    def save(self) -> None:
        if self.last_rgb is None:
            return
        directory = ROOT / "captures" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        directory.mkdir(parents=True)
        Image.fromarray(self.last_rgb).save(directory / "wrist.png")
        metadata = {"simulation_time_s": self.sim.data.time,
                    "precision_enabled": self.precision_enabled,
                    "precision_settings": self.precision_settings if self.precision_enabled else None,
                    "stop_metrics": self.controller.ibvs.stop_metrics,
                    "calibration_profile": self.calibration_name,
                    "controller_calibration": self.calibration.settings,
                    "assumed_K": self.control_intrinsics().tolist(),
                    "gain_mode": self.gain_mode,
                    "perception_mode": self.perception_mode,
                    "gain_per_s": self.controller.ibvs.config["gain_per_s"],
                    "qpos_rad": self.sim.data.qpos.tolist(),
                    "K": self.sim.camera_intrinsics().tolist(),
                    "world_from_optical": self.sim.camera_pose().tolist()}
        metadata.update(self.last_image_info or {})
        metadata["saved_simulation_time_s"] = float(self.sim.data.time)
        (directory / "camera.json").write_text(json.dumps(metadata, indent=2))
        self.message = "Saved raw wrist.png and camera.json in captures/."

    def run(self) -> None:
        # The window may be resized freely; every frame is drawn at the window's own size (see
        # frame_for_window), so the image is never stretched out of proportion.
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, round(UI_SIZE[0] * self.ui_scale), round(UI_SIZE[1] * self.ui_scale))
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        period = 1 / self.sim.config["camera_hz"]
        try:
            while True:
                start = time.perf_counter()
                self.advance(period)
                _, _, width, height = cv2.getWindowImageRect(WINDOW)
                frame = (self.frame_for_window(width, height) if width > 0 and height > 0 else self.draw())
                cv2.imshow(WINDOW, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
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
                elif key in (ord("v"), ord("V")):
                    self.toggle_perception()
                elif key in (ord("k"), ord("K")):
                    self.toggle_matcher()
                elif key in (ord("f"), ord("F")):
                    self.toggle_matches()
                elif key in (ord("i"), ord("I")):
                    self.cycle_calibration()
                elif key in (ord("u"), ord("U")):
                    self.toggle_obstacle()
                elif key in (ord("c"), ord("C")):
                    self.cycle_camera_delay()
                elif key in (ord("x"), ord("X")):
                    self.toggle_camera_stream()
                elif key in (ord("h"), ord("H")):
                    self.teach_reference()
                elif key in (ord("g"), ord("G")):
                    self.align()
                elif key == 32:
                    self.toggle_pause()
                elif key in (ord("e"), ord("E")):
                    self.stop()
                elif key in (ord("s"), ord("S")):
                    self.save()
        finally:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--natural", action="store_true", help="Use the picture with SIFT matching")
    modes.add_argument("--learned", action="store_true", help="Use the picture with pretrained SuperPoint + LightGlue")
    parser.add_argument("--legacy-stop", action="store_true", help="Use the original 1 px stopping rule without image refinement")
    parser.add_argument("--snapshot", action="store_true", help="Render a preview without opening a window")
    starts = parser.add_mutually_exclusive_group()
    starts.add_argument("--cold-start", action="store_true", help="Start out of view with no remembered viewpoint")
    starts.add_argument("--random-start", action="store_true", help="Start at a sampled pose without a remembered viewpoint")
    parser.add_argument("--seed", type=int, help="Reproduce a random start (requires --random-start)")
    parser.add_argument("--calibration-profile", choices=tuple(calibration_profiles()), default="nominal",
                        help="Assumed controller calibration; rendering and collision geometry stay fixed")
    parser.add_argument("--obstacle", action="store_true", help="Enable the mapped obstacle column for collision-aware search")
    parser.add_argument("--manual", action="store_true", help="Start with Auto OFF and wait for Align")
    parser.add_argument("--ui-scale", default="auto",
                        help="Window scale: a number from 1 to 3 (1 = 1200 x 830), or 'auto' to fill the screen (default)")
    parser.add_argument("--camera-delay-ms", type=float, help="Enable timestamped camera delivery with this simulated delay")
    parser.add_argument("--max-camera-age-ms", type=float, default=250, help="Stop if observation age reaches this bound (default: 250 ms)")
    parser.add_argument("--stale-resume-ms", type=float, default=2000, help="--realtime only: after a freshness trip, hold at zero velocity and resume on fresh images for up to this long (default: 2000 ms; 0 = stop, same as --watchdog-stop)")
    parser.add_argument("--watchdog-stop", action="store_true", help="--realtime only: a freshness trip ends the alignment (original behaviour) instead of hold and resume")
    parser.add_argument("--realtime", action="store_true", help="Run wall-clock control with isolated rendering/perception and an independent freshness watchdog")
    parser.add_argument("--actuator", help="Simulated actuator dynamics profile from actuator_config.json "
                        "(default: its active_profile; 'ideal' = no dynamics, the previous behaviour)")
    args = parser.parse_args()
    if args.actuator is not None:
        from actuator import load_actuator_config
        try:
            load_actuator_config(args.actuator)
        except ValueError as exc:
            parser.error(str(exc))
    if args.watchdog_stop:
        args.stale_resume_ms = 0
    if args.ui_scale == "auto":
        # The realtime runtime has its own window and does not use this scale.
        ui_scale = 1.0 if args.snapshot or args.realtime else screen_fit_scale()
    else:
        try:
            ui_scale = float(args.ui_scale)
        except ValueError:
            parser.error("--ui-scale must be a number or 'auto'")
        if not 1.0 <= ui_scale <= 3.0:
            parser.error("--ui-scale must be between 1 and 3")
    timing = None
    if args.camera_delay_ms is not None:
        timing = dict(load_camera_timing(args.camera_delay_ms/1000), max_observation_age_s=args.max_camera_age_ms/1000)
        try:
            TimedCamera(timing, 30)
        except (ValueError, TypeError) as exc:
            parser.error(str(exc))
    if args.seed is not None and (not args.random_start or args.seed < 0):
        parser.error("--seed must be nonnegative and used with --random-start")
    if args.realtime:
        if args.snapshot:
            parser.error("--realtime is interactive; omit --snapshot")
        from realtime import RuntimeConfig
        from realtime_app import run_interactive
        try:
            RuntimeConfig(transport_s=(args.camera_delay_ms or 0)/1000,
                          max_age_s=args.max_camera_age_ms/1000,
                          stale_resume_s=args.stale_resume_ms/1000)
        except ValueError as exc:
            parser.error(str(exc))
        run_interactive(args)
        return
    with Simulation(actuator_config=args.actuator) as sim:
        try:
            if args.obstacle:
                sim.set_obstacle(True)
            if args.cold_start:
                # Set the start before any live image can supply a remembered view.
                sim.reset(load_startup_config()["demo_offset_degrees"])
            elif args.random_start:
                sim.reset(random_start_offset(args.seed))
        except CollisionPoseError as exc:
            parser.error(str(exc))
        lab = Lab(sim, cold_start=args.cold_start or args.random_start, auto_start=not args.manual,
                  perception_mode="learned" if args.learned else "natural" if args.natural else "aruco", camera_timing=timing,
                  calibration=calibration_profiles()[args.calibration_profile],precision=not args.legacy_stop,
                  ui_scale=ui_scale)
        if args.manual and (args.cold_start or args.random_start):
            lab.message = "Starting pose loaded without a remembered view. Click Align [G] when ready."
        if args.snapshot:
            Image.fromarray(lab.draw()).save(ROOT / "preview.png")
        else:
            lab.run()


if __name__ == "__main__":
    main()
