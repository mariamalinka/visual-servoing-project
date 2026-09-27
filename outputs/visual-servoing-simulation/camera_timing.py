"""Timestamped camera transport in simulated time.

This deterministic model delays delivery without delaying the physics clock.
Host inference duration is measured separately, not added to the configured
transport delay. It is not a wall-clock real-time scheduler.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import copy
import json
from pathlib import Path

import numpy as np

CONFIG_PATH = Path(__file__).resolve().with_name("camera_timing_config.json")


def load_camera_timing(delay_s=None):
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    if delay_s is not None:
        config["delay_s"] = delay_s
    return config


@dataclass(frozen=True)
class CameraFrame:
    sequence: int
    captured_s: float
    available_s: float
    rgb: np.ndarray
    qpos: np.ndarray
    observation: object
    inference_ms: float
    world_from_optical: np.ndarray | None = None


class TimedCamera:
    """Bounded camera transport; consume new observations once, in capture order."""

    def __init__(self, config, camera_hz):
        self.config = dict(jitter_s=0.0, drop_probability=0.0, seed=0, outages_s=[])
        self.config.update(config)
        for key in ("delay_s", "max_observation_age_s", "max_run_s", "jitter_s", "drop_probability"):
            value = self.config[key]
            if isinstance(value, bool) or not np.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        if self.config["max_observation_age_s"] <= 0 or self.config["max_run_s"] <= 0:
            raise ValueError("Age and run deadlines must be positive")
        if self.config["drop_probability"] > 1:
            raise ValueError("drop_probability must be in [0, 1]")
        if type(self.config["seed"]) is not int or not 0 <= self.config["seed"] < 2**32:
            raise ValueError("seed must be an integer in [0, 2**32)")
        previous_end = -1.0
        for window in self.config["outages_s"]:
            if len(window) != 2 or not np.isfinite(window).all():
                raise ValueError("Outages must be finite [start, end] intervals")
            start, end = window
            if start < 0 or start >= end or start < previous_end:
                raise ValueError("Outages must be positive-duration, sorted and nonoverlapping")
            previous_end = end
        limit = self.config["max_pending_frames"]
        if type(limit) is not int or not 1 <= limit <= 256:
            raise ValueError("max_pending_frames must be an integer in [1, 256]")
        if isinstance(camera_hz, bool) or not np.isfinite(camera_hz) or camera_hz <= 0:
            raise ValueError("camera_hz must be positive and finite")
        self.period_s = 1.0 / camera_hz
        self.stream_enabled = True
        self.reset(0.0)

    def reset(self, now):
        self._time(now)
        self.pending = deque()
        self.next_capture_s = float(now)
        self.started_s = float(now)
        self.latest = None
        self.delivered_s = None
        self.last_capture_s = None
        self.sequence = 0
        self.delivered_frames = 0
        self.dropped_frames = 0
        self.stale_frames = 0
        self.lost_frames = 0
        self.obsolete_frames = 0
        self.outage_slots = 0
        self.rng = np.random.default_rng(self.config["seed"])
        self.capture_elapsed_s = self.period_s

    @staticmethod
    def _time(now):
        if not np.isfinite(now) or now < 0:
            raise ValueError("Camera timestamp must be finite and nonnegative")

    def capture_due(self, now):
        self._time(now)
        if now + 1e-10 < self.next_capture_s:
            return False
        # Skip missed slots rather than manufacturing duplicate captures.
        missed = max(1, int(np.floor((now - self.next_capture_s + 1e-10) / self.period_s)) + 1)
        self.next_capture_s += missed * self.period_s
        elapsed = now - self.started_s
        if any(start <= elapsed + 1e-10 < end for start, end in self.config["outages_s"]):
            self.outage_slots += 1
            return False
        return self.stream_enabled

    def submit(self, now, rgb, qpos, observation, inference_ms=0.0, world_from_optical=None):
        self._time(now)
        if self.last_capture_s is not None and now <= self.last_capture_s:
            raise ValueError("Capture timestamps must increase strictly")
        pixels = np.array(rgb, copy=True)
        joints = np.array(qpos, dtype=float, copy=True)
        if pixels.ndim != 3 or pixels.shape[2] != 3 or pixels.dtype != np.uint8:
            raise ValueError("Expected an RGB uint8 camera frame")
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError("Expected six finite joints at capture")
        if not np.isfinite(inference_ms) or inference_ms < 0:
            raise ValueError("Inference duration must be finite and nonnegative")
        pose = None if world_from_optical is None else np.array(world_from_optical, dtype=float, copy=True)
        if pose is not None:
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                raise ValueError("Expected a finite capture-time camera transform")
            pose.setflags(write=False)
        measured = copy.deepcopy(observation)
        pixels.setflags(write=False)
        joints.setflags(write=False)
        self.sequence += 1
        self.last_capture_s = float(now)
        # Draw both variables at every capture to pair profiles and matchers.
        loss_draw, jitter_draw = self.rng.random(2)
        if loss_draw < self.config["drop_probability"]:
            self.lost_frames += 1
            return None
        delay = max(0.0, self.config["delay_s"] + (2*jitter_draw-1)*self.config["jitter_s"])
        frame = CameraFrame(self.sequence, float(now), float(now + delay),
                            pixels, joints, measured, float(inference_ms), pose)
        if len(self.pending) >= self.config["max_pending_frames"]:
            self.pending.popleft()
            self.dropped_frames += 1
        self.pending.append(frame)
        return frame

    def receive(self, now):
        self._time(now)
        ready = None
        waiting = deque()
        for candidate in self.pending:
            if self.latest is not None and candidate.captured_s <= self.latest.captured_s:
                self.obsolete_frames += 1
                continue
            if now - candidate.captured_s >= self.config["max_observation_age_s"] - 1e-10:
                self.stale_frames += 1
                continue
            if candidate.available_s > now + 1e-10:
                waiting.append(candidate)
                continue
            if ready is not None:
                self.dropped_frames += 1
            ready = candidate  # Capture order is retained even when arrival order differs.
        self.pending = waiting
        if ready is None:
            return None
        self.capture_elapsed_s = (self.period_s if self.latest is None
                                  else ready.captured_s - self.latest.captured_s)
        self.latest = ready
        self.delivered_s = float(now)
        self.delivered_frames += 1
        return ready

    def age_s(self, now):
        self._time(now)
        return None if self.latest is None else float(now - self.latest.captured_s)

    def failure(self, now):
        """Independent clock: no new frames cannot pause a safety deadline."""
        self._time(now)
        if now < self.started_s or (self.last_capture_s is not None and now < self.last_capture_s):
            return "camera_clock_reset"
        if now - self.started_s >= self.config["max_run_s"] - 1e-10:
            return "camera_run_timeout"
        origin = self.started_s if self.latest is None else self.latest.captured_s
        if now - origin >= self.config["max_observation_age_s"] - 1e-10:
            return "stale_camera"
        return None
