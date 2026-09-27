"""Error-dependent gain for the existing IBVS control law.

This is gain scheduling, not learning a robot model. Only lambda changes:
lambda(e) = near + (far-near) * (1-exp(-e/scale)), with e in RMS pixels.
The image interaction matrix, joint inverse, limits and stopping logic remain
those in control.IBVSController. Parameters are frozen before evaluation.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from control import IBVSController

CONFIG_PATH = Path(__file__).resolve().parent / "gain_config.json"
METHODS = ("fixed", "adaptive", "fixed_high")
LABELS = {"fixed": "Fixed baseline", "adaptive": "Adaptive gain",
          "fixed_high": "Fixed high gain"}


@dataclass(frozen=True)
class GainPolicy:
    near_gain_per_s: float
    far_gain_per_s: float
    error_scale_px: float

    def __post_init__(self) -> None:
        values = (self.near_gain_per_s, self.far_gain_per_s, self.error_scale_px)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Gain parameters must be finite and positive")
        if self.near_gain_per_s > self.far_gain_per_s:
            raise ValueError("Near gain must not exceed far gain")

    def __call__(self, error_px: float) -> float:
        if not np.isfinite(error_px) or error_px < 0:
            raise ValueError("Image error must be finite and nonnegative")
        weight = -np.expm1(-error_px / self.error_scale_px)
        return float(self.near_gain_per_s +
                     (self.far_gain_per_s - self.near_gain_per_s) * weight)


def load_gain_config() -> dict:
    settings = json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))
    GainPolicy(**settings["adaptive"])
    for key in ("fixed_high_gain_per_s", "near_goal_error_px"):
        if not np.isfinite(settings[key]) or settings[key] <= 0:
            raise ValueError(f"{key} must be positive and finite")
    return settings


class AdaptiveIBVSController(IBVSController):
    """Choose the proportional gain from each fresh image, then run normal IBVS."""

    def __init__(self, desired_pixels, K, config, policy: GainPolicy):
        self.policy = policy
        super().__init__(desired_pixels, K, config)

    def reset(self) -> None:
        super().reset()
        self.config["gain_per_s"] = self.policy.near_gain_per_s

    def update(self, corners, camera_jacobian, dt):
        if self.status == "running" and corners is not None:
            points = np.asarray(corners, dtype=float)
            if points.shape != self.desired.shape or not np.isfinite(points).all():
                raise ValueError("Expected four finite marker corners")
            error_px = float(np.sqrt(np.mean(np.sum((points-self.desired)**2, axis=1))))
            self.config["gain_per_s"] = self.policy(error_px)
        return super().update(corners, camera_jacobian, dt)


def make_gain_controller(method, desired, K, config, settings):
    if method == "adaptive":
        return AdaptiveIBVSController(desired, K, config, GainPolicy(**settings["adaptive"]))
    if method not in ("fixed", "fixed_high"):
        raise ValueError(f"Unknown gain method: {method}")
    controller = IBVSController(desired, K, config)
    if method == "fixed_high":
        controller.config["gain_per_s"] = settings["fixed_high_gain_per_s"]
    return controller
