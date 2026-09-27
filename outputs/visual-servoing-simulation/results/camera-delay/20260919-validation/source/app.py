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
                 cold_start=False, auto_start=True, perception_mode="aruco", camera_timing=None) -> None:
        self.sim = sim
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
        self.message = "Jog or load a starting pose, then Align. Lost view [L] demonstrates target recovery."
        self.last_rgb: np.ndarray | None = None
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
        self.controller = ReacquiringIBVS(self.reference, sim.camera_intrinsics(), sim.config,
                                         sim.model.jnt_range)
        if not cold_start and self.camera is None:
            self.controller.remember_view(self.detect_target(sim.image()), sim.data.qpos)
        self.aligning = False
        self.alignment_status = "idle"
        self._start_if_auto()

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
        self.last_observation = None
        self.show_matches = False
        self.controller = ReacquiringIBVS(reference, self.sim.camera_intrinsics(),
                                          self.sim.config, self.sim.model.jnt_range)
        self.controller.ibvs = make_gain_controller(self.gain_mode, reference,
            self.sim.camera_intrinsics(), self.sim.config, self.gain_settings)
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
            self.gain_mode, self.reference, self.sim.camera_intrinsics(),
            self.sim.config, self.gain_settings)
        self.message = f"{self.gain_mode.capitalize()} gain selected. Click Align [G] to start."

    def offset(self) -> None:
        self.stop()
        self.sim.reset(self.sim.config["ibvs"]["start_offset_degrees"])
        self._clear_camera()
        self.paused = False
        self.alignment_status = "idle"
        self.message = "Offset pose loaded. Click Align [G] to close the visual feedback loop."
        self._start_if_auto()

    def lost_view(self) -> None:
        self.reset(start_automatically=False)
        self.sim.reset(self.controller.config["lost_view_demo_offset_degrees"])
        self._clear_camera()
        self.message = "Target is outside the view. Click Align [G] to find it and align."
        self._start_if_auto()

    def cold_start(self) -> None:
        self.stop()
        self.sim.reset(load_startup_config()["demo_offset_degrees"])
        self._clear_camera()
        self.controller.last_visible_qpos = None
        self.cold_start_pending = True
        self.paused = False
        self.message = "Cold start: no remembered viewpoint. Click Align [G] to search from here."
        self._start_if_auto()

    def random_start(self, seed=None) -> None:
        self.stop()
        self.sim.reset(random_start_offset(seed))
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
                                       self.perception.reference_config(self.sim.config), self.detect_target)
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
        self.message = "Checking camera: align if the target is visible; search or recover if it is missing."

    def reset(self, start_automatically=True) -> None:
        self.controller.cancel()
        self.aligning = False
        self.alignment_status = "idle"
        self.sim.reset()
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
        self.message = "Stopped. Click Align [G] or load a new starting pose to begin another run."

    def toggle_pause(self) -> None:
        self.paused = not self.paused
        # Pausing cancels the current pulse and the demo; resuming is stationary.
        self.stop()
        self.message = "Simulation paused." if self.paused else "Simulation resumed."


    def _clear_camera(self):
        if self.camera is not None:
            self.camera.reset(float(self.sim.data.time))
            self.last_observation = None
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
        self.sim.command_velocity(np.zeros(6))
        self.message = ("Stopped: camera feedback is too old. Restore the stream [X], then Align [G]."
                        if status == "stale_camera" else "Stopped: timed-camera run deadline reached.")

    def _camera_tick(self):
        camera = self.camera
        now = float(self.sim.data.time)
        if camera.capture_due(now):
            rgb = self.sim.image()
            qpos = self.sim.data.qpos.copy()
            started = time.perf_counter()
            try:
                observation = self.perception.observe(rgb)
                camera.submit(now, rgb, qpos, observation, 1000*(time.perf_counter()-started))
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
                    frame.observation.corners, self.sim.camera_jacobian(),
                    self.sim.data.qpos, camera.capture_elapsed_s, observation_qpos=frame.qpos)
                # The hold window must span distinct captured images, not GUI
                # draws, repeated reads, or a period with no new camera frames.
                if sample.error_px is not None and sample.error_px < self.sim.config["ibvs"]["success_error_px"]:
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
                self.message = (f"Aligned: {sample.error_px:.2f} px RMS corner error, held below "
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
            sample = self.controller.update(corners, self.sim.camera_jacobian(),
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
        matching_view = self.show_matches and self.perception_mode != "aruco"
        label(canvas, "TEMPLATE / CAMERA MATCHES" if matching_view else "WORLD VIEW", 24, 92, 0.46, MUTED)
        if self.perception_mode!="aruco":
            self.button(canvas, (228,73,122,24),
                        "Learned [K]" if self.perception_mode=="learned" else "SIFT [K]", "matcher",
                        self.perception_mode=="learned")
        self.button(canvas, (362, 73, 230, 24),
                    "Picture [V]" if self.perception_mode != "aruco" else "ArUco [V]", "perception",
                    self.perception_mode != "aruco")
        self.button(canvas, (1000, 73, 176, 24), "Matches [F]", "matches", matching_view)
        label(canvas, "WRIST CAMERA / 640 x 480", 608, 92, 0.46, MUTED)
        delay_label = "Delay [C]" if self.camera is None else f"{self.camera.config['delay_s']*1000:.0f} ms [C]"
        self.button(canvas, (870,73,118,24), delay_label, "camera_delay", self.camera is not None)
        frame = None if self.camera is None else self.camera.latest
        rgb = self.sim.image() if frame is None else frame.rgb
        self.last_rgb = rgb
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
        canvas[102:528, 24:592] = cv2.resize(world, (568, 426))
        canvas[102:528, 608:1176] = cv2.resize(annotated, (568, 426))
        active_state = {"joint_limited": "ALIGNING WITH JOINT LIMITS",
                        "repositioning": "REPOSITIONING FOR RETRY",
                        "retry_confirming": "CONFIRMING RETRY",
                        "realigning": "RETRYING ALIGNMENT",
                        "checking": "CHECKING CAMERA", "running": "ALIGNING", "waiting": "WAITING FOR TARGET",
                        "returning": "RETURNING TO LAST VIEW", "scanning": "SEARCHING",
                        "confirming": "CONFIRMING TARGET"}.get(self.alignment_status, "ALIGNING")
        state = ("PAUSED" if self.paused else active_state if self.aligning else "DEMO" if self.demo
                 else self.alignment_status.upper() if self.alignment_status != "idle" else "MANUAL")
        label(canvas, f"{state}   |   simulation time {self.sim.data.time:7.2f} s", 24, 551, 0.45, TEAL)
        if self.perception_mode != "aruco":
            if corners is None:
                message = f"Picture: {observation.reason.replace('_', ' ')} | {observation.inliers} inliers"
                label(canvas, message, 608, 551, 0.40, AMBER)
            else:
                error = np.sqrt(np.mean(np.sum((corners-self.reference)**2,axis=1)))
                backend = ("Learned GPU" if self.perception.device.type == "cuda" else "Learned CPU") if self.perception_mode == "learned" else "Picture"
                label(canvas, f"{backend} | {observation.inliers} inliers | error {error:.2f} px | {observation.processing_ms:.0f} ms",
                      608, 551, 0.42, TEAL)
        elif corners is None:
            message = "Marker not detected - finding target" if self.aligning else "Marker not detected - Align [G] can try to recover"
            label(canvas, message, 608, 551, 0.42, AMBER)
        else:
            error = np.sqrt(np.mean(np.sum((corners - self.reference)**2, axis=1)))
            label(canvas, f"Marker 7 visible   |   RMS corner error {error:.2f} px",
                  608, 551, 0.45, TEAL)
        if self.camera is not None:
            age = self.camera.age_s(float(self.sim.data.time))
            timing = ("Waiting for delivered image" if frame is None else
                      f"Frame {frame.sequence} | captured {frame.captured_s:.2f}s | age {age*1000:.0f} ms")
            stream = "ON" if self.camera.stream_enabled else "OFF"
            label(canvas, f"{timing} | camera {stream} [X]", 608, 566, 0.34, MUTED)
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
        label(canvas, "V: target   K: matcher   F: matches   B: auto   P: random   N: cold   H: teach   T: gain   O/L: offsets   G: align   1-6: joint   A/D: jog   Space: pause   Esc: quit", 24, 758, 0.39, MUTED)
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
                     "random_start": self.random_start, "perception": self.toggle_perception,
                     "matches": self.toggle_matches, "matcher": self.toggle_matcher,
                     "camera_delay": self.cycle_camera_delay}[action]()
                break

    def save(self) -> None:
        if self.last_rgb is None:
            return
        directory = ROOT / "captures" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        directory.mkdir(parents=True)
        Image.fromarray(self.last_rgb).save(directory / "wrist.png")
        metadata = {"simulation_time_s": self.sim.data.time,
                    "gain_mode": self.gain_mode,
                    "perception_mode": self.perception_mode,
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
                elif key in (ord("v"), ord("V")):
                    self.toggle_perception()
                elif key in (ord("k"), ord("K")):
                    self.toggle_matcher()
                elif key in (ord("f"), ord("F")):
                    self.toggle_matches()
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
                elif key in (ord("s"), ord("S")):
                    self.save()
        finally:
            cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--natural", action="store_true", help="Use the picture with SIFT matching")
    modes.add_argument("--learned", action="store_true", help="Use the picture with pretrained SuperPoint + LightGlue")
    parser.add_argument("--snapshot", action="store_true", help="Render a preview without opening a window")
    starts = parser.add_mutually_exclusive_group()
    starts.add_argument("--cold-start", action="store_true", help="Start out of view with no remembered viewpoint")
    starts.add_argument("--random-start", action="store_true", help="Start at a sampled pose without a remembered viewpoint")
    parser.add_argument("--seed", type=int, help="Reproduce a random start (requires --random-start)")
    parser.add_argument("--manual", action="store_true", help="Start with Auto OFF and wait for Align")
    parser.add_argument("--camera-delay-ms", type=float, help="Enable timestamped camera delivery with this simulated delay")
    parser.add_argument("--max-camera-age-ms", type=float, default=250, help="Stop if observation age reaches this bound (default: 250 ms)")
    args = parser.parse_args()
    timing = None
    if args.camera_delay_ms is not None:
        timing = dict(load_camera_timing(args.camera_delay_ms/1000), max_observation_age_s=args.max_camera_age_ms/1000)
        try:
            TimedCamera(timing, 30)
        except (ValueError, TypeError) as exc:
            parser.error(str(exc))
    if args.seed is not None and (not args.random_start or args.seed < 0):
        parser.error("--seed must be nonnegative and used with --random-start")
    with Simulation() as sim:
        if args.cold_start:
            # Set the start before any live image can supply a remembered view.
            sim.reset(load_startup_config()["demo_offset_degrees"])
        elif args.random_start:
            sim.reset(random_start_offset(args.seed))
        lab = Lab(sim, cold_start=args.cold_start or args.random_start, auto_start=not args.manual,
                  perception_mode="learned" if args.learned else "natural" if args.natural else "aruco", camera_timing=timing)
        if args.manual and (args.cold_start or args.random_start):
            lab.message = "Starting pose loaded without a remembered view. Click Align [G] when ready."
        if args.snapshot:
            Image.fromarray(lab.draw()).save(ROOT / "preview.png")
        else:
            lab.run()


if __name__ == "__main__":
    main()
