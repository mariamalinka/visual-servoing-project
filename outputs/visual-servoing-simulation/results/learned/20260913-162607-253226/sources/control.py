"""Point-feature image-based visual servoing (IBVS).

Conventions follow Chaumette & Hutchinson, Visual Servo Control, Part I,
IEEE Robotics & Automation Magazine 13(4), 2006, equations (4), (9), (11).
Error is current minus desired normalized image coordinates. Camera velocity
is [vx, vy, vz, wx, wy, wz] in optical axes (+X right, +Y down, +Z forward).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


def normalized_points(pixels: np.ndarray, K: np.ndarray) -> np.ndarray:
    points = np.asarray(pixels, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError("Expected finite Nx2 image coordinates")
    homogeneous = np.column_stack((points, np.ones(len(points))))
    rays = np.linalg.solve(np.asarray(K, dtype=float), homogeneous.T).T
    return rays[:, :2] / rays[:, 2:3]


def interaction_matrix(points: np.ndarray, depths: np.ndarray) -> np.ndarray:
    """Stack 2x6 point matrices so s_dot = L @ camera_twist.

    For stationary scene points P, camera motion gives P_dot = -v - w x P.
    Differentiating the pinhole projection x=X/Z, y=Y/Z gives these rows.
    Depths are per-feature optical Z, not Euclidean range.
    """
    xy = np.asarray(points, dtype=float)
    z = np.asarray(depths, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or z.shape != (len(xy),):
        raise ValueError("Expected Nx2 normalized points and N depths")
    if not np.isfinite(xy).all() or not np.isfinite(z).all() or np.any(z <= 0):
        raise ValueError("Points must be finite and depths positive")
    x, y = xy.T
    L = np.empty((2 * len(xy), 6))
    L[0::2] = np.column_stack((-1/z, np.zeros_like(z), x/z, x*y, -(1+x*x), y))
    L[1::2] = np.column_stack((np.zeros_like(z), -1/z, y/z, 1+y*y, -x*y, -x))
    return L


def damped_pinv(matrix: np.ndarray, damping: float) -> np.ndarray:
    """SVD damped least squares: singular gain sigma/(sigma^2 + damping^2).

    For positive damping, each gain is bounded by 1/(2*damping), including at
    singular configurations. Equivalent to A.T @ inv(A @ A.T + damping^2 I).
    """
    A = np.asarray(matrix, dtype=float)
    if A.ndim != 2 or not np.isfinite(A).all() or not np.isfinite(damping) or damping <= 0:
        raise ValueError("Expected a finite matrix and positive finite damping")
    u, singular, vt = np.linalg.svd(A, full_matrices=False)
    return (vt.T * (singular / (singular**2 + damping**2))) @ u.T


def marker_object_points(side_m: float) -> np.ndarray:
    if not np.isfinite(side_m) or side_m <= 0:
        raise ValueError("Marker side must be positive")
    h = side_m / 2
    # OpenCV SOLVEPNP_IPPE_SQUARE ordering matches ArUco's canonical corners.
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=float)


def estimate_depths(corners: np.ndarray, K: np.ndarray, side_m: float,
                    max_reprojection_error: float) -> np.ndarray:
    """Recover feature Z from observed corners and known marker size, not sim pose.

    PnP supplies only the depths for L. The feedback error remains in the image;
    no target-pose error, home joint angles or simulator target position is used.
    https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
    """
    pixels = np.asarray(corners, dtype=float)
    if pixels.shape != (4, 2) or not np.isfinite(pixels).all():
        raise ValueError("Expected four finite marker corners")
    object_points = marker_object_points(side_m)
    ok, rvec, tvec = cv2.solvePnP(object_points, pixels, K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("Marker depth estimation failed")
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, None)
    residual = np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels)**2, axis=1)))
    if np.isfinite(residual) and residual > max_reprojection_error:
        # IPPE is an analytic initializer. Small feature noise near a frontal
        # square can leave its initial reprojection error unnecessarily large.
        # Refine only an otherwise rejected estimate; keep the same acceptance
        # threshold and preserve the original path for already-valid estimates.
        rvec, tvec = cv2.solvePnPRefineLM(object_points, pixels, K, None, rvec, tvec)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, None)
        residual = np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - pixels)**2, axis=1)))
    rotation, _ = cv2.Rodrigues(rvec)
    z = (object_points @ rotation.T + tvec.reshape(3))[:, 2]
    if not np.isfinite(z).all() or np.any(z < 0.03) or not np.isfinite(residual) or residual > max_reprojection_error:
        raise ValueError("Unreliable marker depth estimate")
    return z


@dataclass
class ControlSample:
    status: str
    error_px: float | None
    velocity: np.ndarray
    camera_twist: np.ndarray
    depths: np.ndarray | None = None
    joint_condition: float | None = None


class IBVSController:
    """Fixed-gain proportional IBVS followed by damped differential kinematics."""

    def __init__(self, desired_pixels: np.ndarray, K: np.ndarray, config: dict) -> None:
        self.desired = np.asarray(desired_pixels, dtype=float).copy()
        if self.desired.shape != (4, 2) or not np.isfinite(self.desired).all():
            raise ValueError("The reference must contain four finite marker corners")
        self.K = np.asarray(K, dtype=float).copy()
        self.desired_normalized = normalized_points(self.desired, self.K)
        self.config = config["ibvs"].copy()
        self.side_m = config["marker_side_m"]
        self.reset()

    def reset(self) -> None:
        self.status = "running"
        self.held_seconds = 0.0
        self.below_since: float | None = None
        self.elapsed_seconds = 0.0

    def update(self, corners: np.ndarray | None, camera_jacobian: np.ndarray,
               dt: float) -> ControlSample:
        """One fresh camera measurement per update; tracking failure stops motion.

        The caller applies the returned joint command for dt simulated seconds.
        A success is reported only after the error stays below threshold for the
        configured hold window; zero command is applied during that window.
        """
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError("Frame duration must be positive and finite")
        zero = np.zeros(6)
        if self.status != "running":
            return ControlSample(self.status, None, zero, zero.copy())
        if corners is None:
            self.status = "tracking_loss"
            return ControlSample(self.status, None, zero, zero.copy())
        J = np.asarray(camera_jacobian, dtype=float)
        if J.shape != (6, 6) or not np.isfinite(J).all():
            raise ValueError("Expected a finite 6x6 optical-frame camera Jacobian")
        error_px = float(np.sqrt(np.mean(np.sum((corners - self.desired)**2, axis=1))))
        if self.elapsed_seconds >= self.config["timeout_s"]:
            self.status = "timeout"
            return ControlSample(self.status, error_px, zero, zero.copy())
        if error_px < self.config["success_error_px"]:
            if self.below_since is None:
                self.below_since = self.elapsed_seconds
            self.held_seconds = self.elapsed_seconds - self.below_since
            if self.held_seconds + 1e-9 >= self.config["success_hold_s"]:
                self.status = "converged"
            self.elapsed_seconds += dt
            return ControlSample(self.status, error_px, zero, zero.copy())
        self.held_seconds = 0.0
        self.below_since = None
        try:
            z = estimate_depths(corners, self.K, self.side_m,
                                self.config["max_pnp_reprojection_error_px"])
        except (ValueError, cv2.error):
            self.status = "invalid_depth"
            return ControlSample(self.status, error_px, zero, zero.copy())
        current = normalized_points(corners, self.K)
        L = interaction_matrix(current, z)
        error = (current - self.desired_normalized).ravel()
        twist = -self.config["gain_per_s"] * damped_pinv(L, self.config["interaction_damping"]) @ error
        # Scale uniformly to preserve the twist direction when saturating.
        scale = max(1.0, np.linalg.norm(twist[:3]) / self.config["max_linear_velocity_m_s"],
                    np.linalg.norm(twist[3:]) / self.config["max_angular_velocity_rad_s"])
        twist /= scale
        velocity = damped_pinv(J, self.config["joint_damping"]) @ twist
        joint_scale = max(1.0, np.max(np.abs(velocity)) / self.config["max_joint_velocity_rad_s"])
        velocity /= joint_scale
        self.elapsed_seconds += dt
        singular = np.linalg.svd(J, compute_uv=False)
        condition = float(singular[0] / max(singular[-1], 1e-15))
        return ControlSample(self.status, error_px, velocity, twist, z, condition)
