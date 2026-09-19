"""Pose-based and visually open-loop baselines, using pixels and robot kinematics.

T_A_B maps coordinates from B into A. Optical axes: right, down, forward.
PnP convention: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
PBVS background: Chaumette & Hutchinson (2006), Visual Servo Control, Part I.
"""
from __future__ import annotations

import cv2
import numpy as np

from control import ControlSample, IBVSController, damped_pinv, marker_object_points

METHODS = ("ibvs", "pbvs", "open_loop")
LABELS = {"ibvs": "Image-based (IBVS)", "pbvs": "Pose-based (PBVS)",
          "open_loop": "Look once + joint feedback"}


def inverse_pose(pose: np.ndarray) -> np.ndarray:
    """Rigid inverse: R^-1 = R.T, inverse translation = -R.T @ t."""
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ pose[:3, 3]
    return result


def estimate_marker_pose(corners: np.ndarray, K: np.ndarray, side_m: float,
                         max_error_px: float) -> tuple[np.ndarray, np.ndarray]:
    """Estimate camera-from-marker using the same IPPE solver as marker IBVS."""
    pixels = np.asarray(corners, dtype=float)
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
        raise ValueError("Expected four finite marker corners")
    points = marker_object_points(side_m)
    ok, rvec, tvec = cv2.solvePnP(points, pixels, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("Marker pose estimation failed")
    rotation, _ = cv2.Rodrigues(rvec)
    depths = (points @ rotation.T + tvec.reshape(3))[:, 2]
    projected, _ = cv2.projectPoints(points, rvec, tvec, K, None)
    residual = np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels)**2, axis=1)))
    if (not np.isfinite(rotation).all() or not np.isfinite(depths).all()
            or np.any(depths < .03) or not np.isfinite(residual) or residual > max_error_px):
        raise ValueError("Unreliable marker pose")
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = rotation, tvec.reshape(3)
    return pose, depths


def pose_error(current_from_goal: np.ndarray) -> np.ndarray:
    """Desired camera displacement and axis-angle rotation, in current camera axes.

    For a fixed goal, v = gain*t reduces world position error exponentially.
    w = gain*log(R) rotates toward the goal; both signs are positive because this
    is desired camera displacement, rather than current-minus-desired features.
    """
    rotation_vector, _ = cv2.Rodrigues(current_from_goal[:3, :3])
    return np.r_[current_from_goal[:3, 3], rotation_vector.ravel()]


def joint_command(twist: np.ndarray, jacobian: np.ndarray, config: dict):
    """Same uniform speed scaling and damped joint inverse as the IBVS baseline."""
    twist = np.asarray(twist, dtype=float).copy()
    J = np.asarray(jacobian, dtype=float)
    if J.shape != (6, 6) or not np.isfinite(J).all():
        raise ValueError("Expected a finite 6x6 camera Jacobian")
    scale = max(1., np.linalg.norm(twist[:3]) / config["max_linear_velocity_m_s"],
                np.linalg.norm(twist[3:]) / config["max_angular_velocity_rad_s"])
    twist /= scale
    velocity = damped_pinv(J, config["joint_damping"]) @ twist
    velocity /= max(1., np.max(np.abs(velocity)) / config["max_joint_velocity_rad_s"])
    singular = np.linalg.svd(J, compute_uv=False)
    condition = float(singular[0] / max(singular[-1], 1e-15))
    return velocity, twist, condition


class PoseController:
    """PBVS, or a frozen visual goal tracked using joint-derived camera pose.

    Open-loop means open with respect to VISION. Robot joint feedback remains
    active. After initialization, later pixels never change its command or stop
    decision. The evaluator can still render images to measure its error.
    """

    def __init__(self, method: str, desired: np.ndarray, K: np.ndarray,
                 config: dict, comparison: dict):
        if method not in ("pbvs", "open_loop"):
            raise ValueError("Unknown pose controller")
        self.method = method
        self.desired = np.asarray(desired, dtype=float).copy()
        self.K = np.asarray(K, dtype=float).copy()
        self.config = config["ibvs"].copy()
        self.side_m = config["marker_side_m"]
        self.comparison = comparison
        reference, _ = estimate_marker_pose(
            self.desired, self.K, self.side_m, self.config["max_pnp_reprojection_error_px"])
        self.marker_from_goal = inverse_pose(reference)
        self.world_from_goal = None
        self.elapsed_seconds = 0.
        self.below_since = None
        self.status = "running"

    def update(self, corners, camera_jacobian, dt, world_from_camera):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Frame duration must be positive and finite")
        zero = np.zeros(6)
        if self.status != "running":
            return ControlSample(self.status, None, zero, zero.copy())
        if self.elapsed_seconds + 1e-9 >= self.config["timeout_s"]:
            self.status = "timeout"
            return ControlSample(self.status, None, zero, zero.copy())

        # PBVS remeasures every frame. Open-loop only takes its first measurement.
        needs_pixels = self.method == "pbvs" or self.world_from_goal is None
        error_px, depths = None, None
        if needs_pixels:
            if corners is None:
                self.status = "tracking_loss"
                return ControlSample(self.status, None, zero, zero.copy())
            error_px = float(np.sqrt(np.mean(np.sum((corners - self.desired)**2, axis=1))))
            try:
                current_from_marker, depths = estimate_marker_pose(
                    corners, self.K, self.side_m, self.config["max_pnp_reprojection_error_px"])
            except (ValueError, cv2.error):
                self.status = "invalid_depth"
                return ControlSample(self.status, error_px, zero, zero.copy())
            current_from_goal = current_from_marker @ self.marker_from_goal
            if self.method == "open_loop":
                # Only the INITIAL observed target and CURRENT robot FK enter.
                # The taught/home robot pose is never supplied as a goal.
                self.world_from_goal = world_from_camera @ current_from_goal
        else:
            current_from_goal = inverse_pose(world_from_camera) @ self.world_from_goal

        displacement = pose_error(current_from_goal)
        if self.method == "pbvs":
            close = error_px < self.config["success_error_px"]
        else:
            close = (np.linalg.norm(displacement[:3]) < self.comparison["open_loop_position_tolerance_m"]
                     and np.linalg.norm(displacement[3:]) < self.comparison["open_loop_rotation_tolerance_rad"])
        if close:
            if self.below_since is None:
                self.below_since = self.elapsed_seconds
            if self.elapsed_seconds - self.below_since + 1e-9 >= self.config["success_hold_s"]:
                self.status = "converged" if self.method == "pbvs" else "motion_complete"
            self.elapsed_seconds += dt
            return ControlSample(self.status, error_px, zero, zero.copy(), depths)
        self.below_since = None
        velocity, twist, condition = joint_command(
            self.config["gain_per_s"] * displacement, camera_jacobian, self.config)
        self.elapsed_seconds += dt
        return ControlSample(self.status, error_px, velocity, twist, depths, condition)


def make_controller(method, desired, K, config, comparison):
    if method == "ibvs":
        return IBVSController(desired, K, config)
    return PoseController(method, desired, K, config, comparison)


def update_controller(controller, corners, jacobian, dt, camera_pose):
    if isinstance(controller, IBVSController):
        return controller.update(corners, jacobian, dt)
    return controller.update(corners, jacobian, dt, camera_pose)
