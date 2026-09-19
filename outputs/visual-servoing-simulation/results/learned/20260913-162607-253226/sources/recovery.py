"""Startup search, marker reacquisition and joint-aware alignment supervision.

During lost-feature recovery, joint feedback returns toward a previously observed
viewpoint, followed by a local scan. No target pose or simulator visibility flag
is used. Once the marker is confirmed, ordinary IBVS takes over.
"""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
from control import ControlSample, IBVSController
from startup_search import StartupSearch
from joint_limits import JointLimitSupervisor

ACTIVE_STATES = frozenset(("running", "waiting", "returning", "scanning", "confirming",
                           "joint_limited", "repositioning", "retry_confirming", "realigning"))
CONFIG_PATH = Path(__file__).resolve().parent / "recovery_config.json"


def load_recovery_config() -> dict:
    return json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))


class ReacquiringIBVS:
    def __init__(self, desired, K, simulation_config, joint_limits,
                 taught_qpos=None, recovery_config=None, startup_config=None, motion_config=None):
        self.ibvs = IBVSController(desired, K, simulation_config)
        self.config = dict(recovery_config or load_recovery_config())
        self.startup_config = startup_config
        self.limits = np.asarray(joint_limits, dtype=float).copy()
        if self.limits.shape != (6, 2) or not np.isfinite(self.limits).all():
            raise ValueError("Expected six finite joint limits")
        margin = self.config["joint_limit_margin_rad"]
        if not np.isfinite(margin) or margin <= 0 or np.any(self.limits[:, 1]-self.limits[:, 0] <= 2*margin):
            raise ValueError("Joint limits need room for the recovery margin")
        for key in ("max_recovery_time_s", "max_total_time_s", "max_joint_velocity_rad_s",
                    "position_gain_per_s", "arrival_tolerance_rad", "max_excursion_degrees"):
            if not np.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if self.config["loss_hold_s"] < 0 or not np.isfinite(self.config["loss_hold_s"]):
            raise ValueError("Hold duration must be finite and nonnegative")
        for key in ("confirmation_frames", "max_recovery_episodes"):
            if not isinstance(self.config[key], int) or self.config[key] < 1:
                raise ValueError(f"{key} must be a positive integer")
        offsets = np.asarray(self.config["scan_offsets_degrees"], dtype=float)
        if offsets.ndim != 2 or offsets.shape[1] != 2 or not len(offsets) or not np.isfinite(offsets).all():
            raise ValueError("Scan needs finite yaw/pitch offset pairs")
        self.last_visible_qpos = None if taught_qpos is None else self._q(taught_qpos).copy()
        self.motion = JointLimitSupervisor(self.limits, motion_config)
        self.cancel()

    @staticmethod
    def _q(qpos):
        q = np.asarray(qpos, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Expected six finite measured joint positions")
        return q

    def remember_view(self, corners, qpos):
        """Store measured joints only when a real marker detection is available."""
        if corners is not None:
            points = np.asarray(corners)
            if points.shape == (4, 2) and np.isfinite(points).all():
                self.last_visible_qpos = self._q(qpos).copy()

    def cancel(self):
        self.status = "canceled"
        if getattr(self, "startup", None) is not None:
            self.startup.cancel()
        self.startup = None
        self.startup_elapsed_s = 0.
        self.mode = None
        self.elapsed_s = 0.0
        self.recovery_elapsed_s = 0.0
        self.episodes = 0
        self.reacquisitions = 0
        self.confirmed_frames = 0
        self.waypoints = []
        self.ibvs.reset()
        self.motion.reset()

    def start(self, cold=False):
        """Start; cold=True explicitly discards any remembered viewpoint."""
        self.cancel()
        if cold:
            self.last_visible_qpos = None
        self.status = "running"

    def _sample(self, status, command=None):
        self.status = status
        zero = np.zeros(6)
        return ControlSample(status, None, zero if command is None else command, zero.copy())

    def _begin_recovery(self, qpos):
        self.motion.clear_progress()
        self.episodes += 1
        self.mode = "waiting"
        self.hold_elapsed_s = 0.0
        self.confirmed_frames = 0
        margin = self.config["joint_limit_margin_rad"]
        excursion = np.deg2rad(self.config["max_excursion_degrees"])
        self.lower = np.maximum(self.limits[:, 0] + margin, qpos - excursion)
        self.upper = np.minimum(self.limits[:, 1] - margin, qpos + excursion)
        if np.any(self.lower > self.upper):
            raise ValueError("Measured joints are outside the recovery workspace")
        view = qpos if self.last_visible_qpos is None else self.last_visible_qpos
        self.return_goal = np.clip(view, self.lower, self.upper)
        self.waypoints = []
        for yaw, pitch in self.config["scan_offsets_degrees"]:
            goal = self.return_goal.copy()
            goal[[0, 4]] += np.deg2rad([yaw, pitch])
            goal = np.clip(goal, self.lower, self.upper)
            if not self.waypoints or np.linalg.norm(goal-self.waypoints[-1]) > 1e-8:
                self.waypoints.append(goal)
        self.waypoint_index = 0

    def _toward(self, qpos, goal, dt):
        difference = goal - qpos
        limit = self.config["max_joint_velocity_rad_s"]
        command = np.clip(self.config["position_gain_per_s"] * difference, -limit, limit)
        # Bound the requested step, including at a mechanical or search-box edge.
        return np.clip(command, np.minimum(0, (self.lower-qpos)/dt),
                       np.maximum(0, (self.upper-qpos)/dt))

    def _align_visible(self, corners, camera_jacobian, qpos, dt):
        sample = self.ibvs.update(corners, camera_jacobian, dt)
        sample = self.motion.filter(sample, camera_jacobian, qpos, dt, self.ibvs.config, self.elapsed_s - dt)
        if sample.status == "repositioning":
            self.mode = "repositioning"
        if sample.status in ("running", "joint_limited", "realigning", "converged"):
            self.remember_view(corners, qpos)
        self.status = sample.status
        return sample

    def update(self, corners, camera_jacobian, qpos, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Frame duration must be positive and finite")
        qpos = self._q(qpos)
        if self.status not in ACTIVE_STATES:
            return self._sample(self.status)
        if self.elapsed_s - self.startup_elapsed_s + 1e-9 >= self.config["max_total_time_s"]:
            return self._sample("timeout")
        self.elapsed_s += dt

        if self.mode == "repositioning":
            sample = self.motion.reposition(corners, qpos, dt, self.config["confirmation_frames"], self.elapsed_s - dt)
            if sample.status == "retry_ready":
                self.mode = None
                self.ibvs.reset()
                return self._align_visible(corners, camera_jacobian, qpos, dt)
            self.status = sample.status
            return sample

        if self.last_visible_qpos is None and self.mode is None:
            self.startup = StartupSearch(qpos, self.limits, self.startup_config)
            self.mode = "startup"
        if self.mode == "startup":
            sample = self.startup.update(corners, qpos, dt)
            self.startup_elapsed_s = self.startup.elapsed_s
            if sample.status != "acquired":
                self.status = sample.status
                return sample
            self.remember_view(corners, qpos)
            self.mode = None
            self.ibvs.reset()

        if self.mode is None and corners is not None:
            return self._align_visible(corners, camera_jacobian, qpos, dt)

        if self.mode is None:
            self._begin_recovery(qpos)
        if self.episodes > self.config["max_recovery_episodes"]:
            return self._sample("recovery_limit")
        if self.recovery_elapsed_s + 1e-9 >= self.config["max_recovery_time_s"]:
            return self._sample("search_timeout")
        self.recovery_elapsed_s += dt

        # Brake immediately on detection and demand consecutive fresh frames.
        # Never keep a stale IBVS command or use predicted/ground-truth corners.
        if corners is not None:
            self.confirmed_frames += 1
            if self.confirmed_frames < self.config["confirmation_frames"]:
                return self._sample("confirming")
            self.mode = None
            self.reacquisitions += 1
            self.ibvs.reset()
            return self._align_visible(corners, camera_jacobian, qpos, dt)
        self.confirmed_frames = 0

        if self.mode == "waiting":
            if self.hold_elapsed_s + 1e-9 < self.config["loss_hold_s"]:
                self.hold_elapsed_s += dt
                return self._sample("waiting")
            self.mode = "returning"

        tolerance = self.config["arrival_tolerance_rad"]
        if self.mode == "returning":
            if np.max(np.abs(qpos-self.return_goal)) > tolerance:
                return self._sample("returning", self._toward(qpos, self.return_goal, dt))
            self.mode = "scanning"

        while self.waypoint_index < len(self.waypoints):
            goal = self.waypoints[self.waypoint_index]
            if np.max(np.abs(qpos-goal)) > tolerance:
                return self._sample("scanning", self._toward(qpos, goal, dt))
            self.waypoint_index += 1
        return self._sample("search_exhausted")

