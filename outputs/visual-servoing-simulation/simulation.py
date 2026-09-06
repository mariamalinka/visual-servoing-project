"""MuJoCo scene and velocity-command interface for the visual servoing project.

Camera images are rendered from the moving camera, not drawn from target-coordinate
ground truth. The IBVS controller lives in control.py.
"""
from __future__ import annotations

import json
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent


def make_marker(config: dict) -> None:
    """Create the physical scene texture: 384 px marker + 64 px white borders."""
    asset_dir = ROOT / "assets"
    asset_dir.mkdir(exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config["marker_dictionary"]))
    marker = cv2.aruco.generateImageMarker(dictionary, config["marker_id"], 384)
    board = np.full((512, 512), 255, dtype=np.uint8)
    board[64:448, 64:448] = marker
    # The negative-X face of MuJoCo's box has mirrored UVs. Compensate in
    # the scene texture so the rendered physical marker has the proper code.
    # Never mirror the sensor image or the detected coordinates.
    if not cv2.imwrite(str(asset_dir / "marker.png"), cv2.flip(board, 1)):
        raise RuntimeError("Cannot write marker texture")


class Simulation:
    """Six revolute joints with gravity compensation and velocity actuators.

    SI units: seconds, meters, radians, radians/second. Rendering must happen on
    the thread that constructed this object (OpenGL context ownership).
    """

    def __init__(self) -> None:
        self.config = json.loads((ROOT / "config.json").read_text())
        make_marker(self.config)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.width = self.config["camera_width"]
        self.height = self.config["camera_height"]
        self.camera_id = self.model.camera("wrist").id
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.world_camera = mujoco.MjvCamera()
        self.world_camera.lookat[:] = [0.53, 0, 0.34]
        self.world_camera.distance = 1.95
        self.world_camera.azimuth = 65
        self.world_camera.elevation = -23
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.config["marker_dictionary"])
        )
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.velocity_command = np.zeros(self.model.nu)
        self._requested_time = 0.0
        self.reset()

    def reset(self, offset_degrees: np.ndarray | None = None) -> None:
        """Explicit reset only; normal motion always uses actuators + mj_step."""
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        if offset_degrees is not None:
            offset = np.asarray(offset_degrees, dtype=float)
            if offset.shape != (6,) or not np.isfinite(offset).all():
                raise ValueError("Initial offset must contain six finite angles in degrees")
            position = self.data.qpos + np.deg2rad(offset)
            if np.any(position < self.model.jnt_range[:, 0]) or np.any(position > self.model.jnt_range[:, 1]):
                raise ValueError("Initial pose is outside the joint limits")
            self.data.qpos[:] = position
        self.velocity_command[:] = 0
        self._requested_time = 0.0
        mujoco.mj_forward(self.model, self.data)

    def command_velocity(self, command: np.ndarray) -> None:
        values = np.asarray(command, dtype=float)
        if values.shape != (self.model.nu,) or not np.isfinite(values).all():
            raise ValueError(f"Expected {self.model.nu} finite joint velocities")
        limit = self.config["max_joint_velocity_rad_s"]
        self.velocity_command[:] = np.clip(values, -limit, limit)

    def advance(self, seconds: float) -> None:
        if not np.isfinite(seconds) or seconds < 0:
            raise ValueError("Duration must be finite and nonnegative")
        # Accumulate the fractional step: 30 Hz cameras with 500 Hz physics.
        self._requested_time += seconds
        dt = self.model.opt.timestep
        while self.data.time + dt <= self._requested_time + 1e-10:
            cmd = self.velocity_command.copy()
            lower, upper = self.model.jnt_range.T
            margin = 0.03  # Stop outbound velocity before the mechanical limit.
            cmd[(self.data.qpos < lower + margin) & (cmd < 0)] = 0
            cmd[(self.data.qpos > upper - margin) & (cmd > 0)] = 0
            self.data.ctrl[:] = cmd
            mujoco.mj_step(self.model, self.data)
        # mj_step leaves position-dependent fields at the integration-stage pose.
        # Forward refresh makes camera pose and RGB correspond to current qpos.
        mujoco.mj_forward(self.model, self.data)

    def image(self, camera: str = "wrist") -> np.ndarray:
        """Unannotated uint8 RGB image. Available camera names: wrist, world."""
        if camera not in ("wrist", "world"):
            raise ValueError("Camera must be wrist or world")
        self.renderer.update_scene(
            self.data, camera=self.camera_id if camera == "wrist" else self.world_camera
        )
        return self.renderer.render().copy()

    def marker_corners(self, rgb: np.ndarray) -> np.ndarray | None:
        """Detect and refine marker corners from the rendered camera pixels."""
        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            for detected, marker_id in zip(corners, ids.ravel()):
                if int(marker_id) == self.config["marker_id"]:
                    return detected.reshape(4, 2).copy()
        return None

    def camera_intrinsics(self) -> np.ndarray:
        """Pinhole K; square pixels, center at image width/2 and height/2."""
        fovy = np.deg2rad(self.model.cam_fovy[self.camera_id])
        focal = self.height / (2 * np.tan(fovy / 2))
        return np.array([[focal, 0, self.width / 2],
                         [0, focal, self.height / 2], [0, 0, 1]])

    def camera_pose(self) -> np.ndarray:
        """World-from-optical transform (+X right, +Y down, +Z forward).

        Simulator ground truth, for evaluation/kinematics; not perception input.
        MuJoCo camera axes are +X right, +Y up, -Z forward.
        """
        transform = np.eye(4)
        transform[:3, :3] = (
            self.data.cam_xmat[self.camera_id].reshape(3, 3) @ np.diag([1, -1, -1])
        )
        transform[:3, 3] = self.data.cam_xpos[self.camera_id]
        return transform

    def camera_jacobian(self) -> np.ndarray:
        """Map joint rates to [linear; angular] velocity at the optical origin.

        mj_jac evaluates a point on the camera body, with axes aligned to world.
        Rotate both 3-row blocks into optical axes. Because the point is already
        the camera origin, its wrist-to-camera lever arm is already included;
        an additional translation adjoint would incorrectly count it twice.
        https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-jac
        """
        linear = np.zeros((3, self.model.nv))
        angular = np.zeros_like(linear)
        mujoco.mj_jac(self.model, self.data, linear, angular,
                      self.data.cam_xpos[self.camera_id],
                      int(self.model.cam_bodyid[self.camera_id]))
        optical_from_world = self.camera_pose()[:3, :3].T
        return np.vstack((optical_from_world @ linear, optical_from_world @ angular))

    def close(self) -> None:
        self.renderer.close()

    def __enter__(self) -> Simulation:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
