"""Image-triggered, bounded startup search with no taught robot viewpoint."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from control import ControlSample

CONFIG_PATH = Path(__file__).resolve().parent / "startup_search_config.json"


def load_startup_config():
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))


class StartupSearch:
    """Coarse then finer yaw/pitch rectangles around the measured start.

    Input: encoder positions and current detected corners. No reference pose,
    target coordinates, simulator object or home configuration is accepted.
    Detection interrupts motion even between waypoints.
    """
    def __init__(self, qpos, joint_limits, config=None):
        self.config = dict(load_startup_config() if config is None else config)
        c = self.config
        for key in ("max_search_time_s", "max_joint_velocity_rad_s",
                    "position_gain_per_s", "arrival_tolerance_rad", "joint_limit_margin_rad"):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if type(c["confirmation_frames"]) is not int or c["confirmation_frames"] < 1:
            raise ValueError("confirmation_frames must be a positive integer")
        radii = np.asarray(c["ring_radii_degrees"], dtype=float)
        if radii.ndim != 1 or not len(radii) or not np.isfinite(radii).all():
            raise ValueError("Search needs finite increasing ring radii")
        if radii[0] <= 0 or np.any(np.diff(radii) <= 0):
            raise ValueError("Ring radii must be positive and strictly increasing")
        # Missing setting means the original single-pass policy for archived runs.
        refine = c.get("refine_after_coarse", False)
        if type(refine) is not bool:
            raise ValueError("refine_after_coarse must be a boolean")
        self.anchor = self._q(qpos).copy()
        limits = np.asarray(joint_limits, dtype=float)
        margin = c["joint_limit_margin_rad"]
        if limits.shape != (6, 2) or not np.isfinite(limits).all() or np.any(np.diff(limits)[:, 0] <= 2*margin):
            raise ValueError("Expected six joint ranges with room for the margin")
        if np.any(self.anchor < limits[:, 0]) or np.any(self.anchor > limits[:, 1]):
            raise ValueError("Starting joints are outside mechanical limits")
        excursion = np.zeros(6)
        excursion[[0, 4]] = np.deg2rad(radii[-1])
        self.lower = np.maximum(limits[:, 0]+margin, self.anchor-excursion)
        self.upper = np.minimum(limits[:, 1]-margin, self.anchor+excursion)
        # Other joints hold their starting position, moving inward if necessary.
        hold = np.clip(self.anchor, limits[:, 0]+margin, limits[:, 1]-margin)
        self.lower = np.minimum(self.lower, hold)
        self.upper = np.maximum(self.upper, hold)
        self.waypoints = [hold]
        def append_rings(scan_radii):
            for r in scan_radii:
                for yaw, pitch in ((-r,0),(-r,-r),(r,-r),(r,r),(-r,r),(-r,0)):
                    goal = hold.copy()
                    goal[[0,4]] = self.anchor[[0,4]] + np.deg2rad([yaw,pitch])
                    goal = np.clip(goal, self.lower, self.upper)
                    if np.max(np.abs(goal-self.waypoints[-1])) > 1e-8:
                        self.waypoints.append(goal)

        append_rings(radii)
        self.coarse_waypoint_count = len(self.waypoints)
        self.refinement_radii_degrees = np.array([])
        if refine:
            # Bisect every gap, including center-to-first-ring. This is a generic
            # second pass, independent of any target location or reference pose.
            self.refinement_radii_degrees = (np.r_[0.,radii[:-1]]+radii)/2
            if np.max(np.abs(hold-self.waypoints[-1])) > 1e-8:
                self.waypoints.append(hold.copy())
            append_rings(self.refinement_radii_degrees)
        self.refinement_started_s = None
        self.index = 0
        self.elapsed_s = 0.
        self.first_detection_s = None
        self.acquired_s = None
        self.confirmed_frames = 0
        self.status = "scanning"
        self.stop_reason = None

    @property
    def stage(self):
        return "refined" if self.index >= self.coarse_waypoint_count and len(self.refinement_radii_degrees) else "coarse"

    @staticmethod
    def _q(qpos):
        q = np.asarray(qpos, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Expected six finite measured joints")
        return q

    def _sample(self, status, velocity=None):
        self.status = status
        return ControlSample(status, None, np.zeros(6) if velocity is None else velocity,
                             np.zeros(6))

    def cancel(self):
        self.status = "canceled"

    def update(self, corners, qpos, dt):
        q = self._q(qpos)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Frame duration must be positive and finite")
        if self.status not in ("scanning", "confirming"):
            return self._sample(self.status)
        if self.elapsed_s + 1e-9 >= self.config["max_search_time_s"]:
            self.stop_reason = "time_budget"
            return self._sample("target_not_found")
        stamp = self.elapsed_s
        self.elapsed_s += dt
        valid = (corners is not None and np.shape(corners) == (4,2)
                 and np.isfinite(corners).all())
        if valid:
            if self.first_detection_s is None:
                self.first_detection_s = stamp
            self.confirmed_frames += 1
            if self.confirmed_frames >= self.config["confirmation_frames"]:
                self.acquired_s = stamp
                return self._sample("acquired")
            return self._sample("confirming")
        self.confirmed_frames = 0
        while self.index < len(self.waypoints):
            if self.stage == "refined" and self.refinement_started_s is None:
                self.refinement_started_s = stamp
            delta = self.waypoints[self.index] - q
            if np.max(np.abs(delta)) > self.config["arrival_tolerance_rad"]:
                velocity = self.config["position_gain_per_s"]*delta
                # Uniform scaling keeps coordinated yaw/pitch paths straight.
                limit = self.config["max_joint_velocity_rad_s"]
                velocity *= min(1., limit/max(float(np.max(np.abs(velocity))), 1e-12))
                velocity = np.clip(velocity, np.minimum(0., (self.lower-q)/dt),
                                   np.maximum(0., (self.upper-q)/dt))
                return self._sample("scanning", velocity)
            self.index += 1
        self.stop_reason = "coverage_complete"
        return self._sample("target_not_found")
