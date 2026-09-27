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
from collision import CollisionGuard, CollisionPoseError

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

    def __init__(self, collision_config=None, *, render=True) -> None:
        self.config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
        if render:
            make_marker(self.config)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.collision = CollisionGuard(self.model, collision_config)
        if not self.collision.enabled:
            self.model.geom_contype[:] = 0
            self.model.geom_conaffinity[:] = 0
        self.obstacle_enabled = False
        self.obstacle_mocap_id = int(self.model.body("obstacle").mocapid[0])
        self.data.mocap_pos[self.obstacle_mocap_id] = self.collision.config["obstacle_position_m"]
        self.collision.sync_environment(self.data)
        self.collision_event = None
        self.target_mode = "aruco"
        self.width = self.config["camera_width"]
        self.height = self.config["camera_height"]
        self.camera_id = self.model.camera("wrist").id
        self.renderer = (mujoco.Renderer(self.model, height=self.height, width=self.width)
                         if render else None)
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
        # The default 0.125 merges the marker and its surrounding board contours
        # at some viewing angles, suppressing a valid decode. Keep these distinct.
        parameters.minMarkerDistanceRate = self.config["marker_detection"]["min_marker_distance_rate"]
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.velocity_command = np.zeros(self.model.nu)
        self.requested_velocity = np.zeros(self.model.nu)
        self._requested_time = 0.0
        self.reset()

    def set_target_mode(self, mode: str) -> None:
        """Swap only the board material, preserving measured joints and target pose."""
        if mode not in ("aruco", "natural"):
            raise ValueError("Target mode must be aruco or natural")
        self.command_velocity(np.zeros(6))
        material = "marker_mat" if mode == "aruco" else "natural_mat"
        self.model.geom("target_board").matid[0] = self.model.material(material).id
        self.target_mode = mode
        mujoco.mj_forward(self.model, self.data)

    def reset(self, offset_degrees: np.ndarray | None = None) -> None:
        """Explicit reset only; normal motion always uses actuators + mj_step."""
        position = self.model.key("home").qpos.copy()
        if offset_degrees is not None:
            offset = np.asarray(offset_degrees, dtype=float)
            if offset.shape != (6,) or not np.isfinite(offset).all():
                raise ValueError("Initial offset must contain six finite angles in degrees")
            position = position + np.deg2rad(offset)
            if np.any(position < self.model.jnt_range[:, 0]) or np.any(position > self.model.jnt_range[:, 1]):
                raise ValueError("Initial pose is outside the joint limits")
        self.collision.sync_environment(self.data)
        if not self.collision.pose_clear(position):
            distances,_ = self.collision.distances(position)
            index = int(np.argmin(distances-self.collision.margins))
            pair = tuple(self.collision.geom_names[i] for i in self.collision.pairs[index])
            raise CollisionPoseError(f"Starting pose violates collision clearance: {pair}, {distances[index]*1000:.1f} mm")
        environment = self.data.mocap_pos.copy(),self.data.mocap_quat.copy()
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.data.mocap_pos[:],self.data.mocap_quat[:] = environment
        self.data.qpos[:] = position
        self.velocity_command[:] = 0
        self.requested_velocity[:] = 0
        self.collision_event = None
        self.collision.reset()
        self._requested_time = 0.0
        mujoco.mj_forward(self.model, self.data)

    def command_velocity(self, command: np.ndarray) -> None:
        values = np.asarray(command, dtype=float)
        if values.shape != (self.model.nu,) or not np.isfinite(values).all():
            raise ValueError(f"Expected {self.model.nu} finite joint velocities")
        limit = self.config["max_joint_velocity_rad_s"]
        self.requested_velocity[:] = np.clip(values, -limit, limit)
        self._guard_velocity()

    def _guard_velocity(self):
        self.collision.sync_environment(self.data)
        decision = self.collision.filter_velocity(self.data.qpos, self.data.qvel,
                    self.requested_velocity, self.config["max_joint_velocity_rad_s"])
        self.velocity_command[:] = decision.velocity
        if decision.status == "blocked" and np.any(self.requested_velocity):
            self.collision_event = dict(time_s=float(self.data.time), pair=decision.pair,
                                        clearance_m=decision.clearance_m)
            # Blocked motion is latched off. A later command requires explicit control input.
            self.requested_velocity[:] = 0

    def set_obstacle(self, enabled, position=None):
        """Change the mapped obstacle only when the resulting pose has clearance."""
        if type(enabled) is not bool:
            raise ValueError("Obstacle enabled must be boolean")
        geom = self.model.geom("obstacle_box").id
        old = (self.data.mocap_pos[self.obstacle_mocap_id].copy(), int(self.model.geom_contype[geom]),
               int(self.model.geom_conaffinity[geom]), self.model.geom_rgba[geom].copy())
        if position is not None:
            position = np.asarray(position,dtype=float)
            if position.shape != (3,) or not np.isfinite(position).all():
                raise ValueError("Obstacle position must contain three finite coordinates")
            self.data.mocap_pos[self.obstacle_mocap_id] = position
        self.model.geom_contype[geom] = int(enabled and self.collision.enabled)
        self.model.geom_conaffinity[geom] = int(enabled and self.collision.enabled)
        self.model.geom_rgba[geom] = [.9,.32,.18,1 if enabled else 0]
        self.collision.refresh_pairs()
        self.collision.sync_environment(self.data)
        if enabled and not self.collision.pose_clear(self.data.qpos):
            self.data.mocap_pos[self.obstacle_mocap_id], self.model.geom_contype[geom], self.model.geom_conaffinity[geom], self.model.geom_rgba[geom] = old
            self.collision.refresh_pairs()
            self.collision.sync_environment(self.data)
            raise CollisionPoseError("Obstacle would overlap the robot or its clearance zone")
        self.obstacle_enabled = enabled
        mujoco.mj_forward(self.model,self.data)

    def forbidden_contacts(self):
        if not self.data.ncon:
            return []
        pairs = self.collision.pair_keys
        return [self.data.contact[i] for i in range(self.data.ncon)
                if tuple(sorted(self.data.contact[i].geom.tolist())) in pairs
                and self.data.contact[i].dist < -1e-7]

    def advance(self, seconds: float) -> None:
        if not np.isfinite(seconds) or seconds < 0:
            raise ValueError("Duration must be finite and nonnegative")
        # Accumulate the fractional step: 30 Hz cameras with 500 Hz physics.
        self._requested_time += seconds
        dt = self.model.opt.timestep
        while self.data.time + dt <= self._requested_time + 1e-10:
            self._guard_velocity()
            cmd = self.velocity_command.copy()
            lower, upper = self.model.jnt_range.T
            margin = 0.03  # Stop outbound velocity before the mechanical limit.
            cmd[(self.data.qpos < lower + margin) & (cmd < 0)] = 0
            cmd[(self.data.qpos > upper - margin) & (cmd > 0)] = 0
            self.velocity_command[:] = cmd
            self.data.ctrl[:] = cmd
            mujoco.mj_step(self.model, self.data)
        # mj_step leaves position-dependent fields at the integration-stage pose.
        # Forward refresh makes camera pose and RGB correspond to current qpos.
        mujoco.mj_forward(self.model, self.data)

    def image(self, camera: str = "wrist") -> np.ndarray:
        """Unannotated uint8 RGB image. Available camera names: wrist, world."""
        if camera not in ("wrist", "world"):
            raise ValueError("Camera must be wrist or world")
        if self.renderer is None:
            raise RuntimeError("Rendering is disabled for this physics instance")
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

    def tool_pose(self) -> np.ndarray:
        """Evaluation pose of the tool_roll body origin, in its local tool axes.

        This is a defined tool frame, not an unmodelled gripper TCP.
        """
        body = self.model.body("tool_roll").id
        pose = np.eye(4)
        pose[:3,:3] = self.data.xmat[body].reshape(3,3)
        pose[:3,3] = self.data.xpos[body]
        return pose

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
        if self.renderer is not None:
            self.renderer.close()

    def __enter__(self) -> Simulation:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
