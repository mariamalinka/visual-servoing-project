"""MuJoCo scene and velocity-command interface for the visual servoing project.

Camera images are rendered from the moving camera, not drawn from target-coordinate
ground truth. The IBVS controller lives in control.py.
"""
from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import cv2
import mujoco
import numpy as np
from actuator import ActuatorModel
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

    Command path, once per 2 ms physics step:
      requested_velocity  clipped controller/runtime request
      velocity_command    after the collision guard and the joint-limit backstop
                          ("the command"; a stop makes this exactly zero at once)
      actuator.velocity   servo setpoint after the actuator model (actuator.py):
                          acceleration- and jerk-limited, so the robot brakes and
                          starts over a finite time
      data.qvel           measured joint velocity after MuJoCo's velocity servo
    Every transition to a zero command while moving is measured as a physical stop
    (motion_log); see docs/ACTUATOR_MODEL.md.
    """

    def __init__(self, collision_config=None, *, render=True, actuator_config=None) -> None:
        self.config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
        if render:
            make_marker(self.config)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "scene.xml"))
        self.data = mujoco.MjData(self.model)
        self.collision = CollisionGuard(self.model, collision_config)
        # actuator_config: None (actuator_config.json's active profile), a profile name
        # or a dict. Rejected if it brakes slower than the collision guard assumes.
        self.actuator = ActuatorModel(actuator_config, self.model.nu,
            max_velocity=self.config["max_joint_velocity_rad_s"],
            collision_braking_time_s=self.collision.config["braking_time_s"] if self.collision.enabled else None)
        self.motion_log = deque(maxlen=2048)
        self.motion_evicted = 0
        self.command_serial = 0  # Counts command_velocity calls; links runtime events to motion_log.
        self._motion = None
        self._command_reason = None
        self._command_was_zero = True
        self._backstop = np.zeros(self.model.nu)  # Latched joint-limit cut per joint (+1 upper, -1 lower).
        self._kinematics = mujoco.MjData(self.model)
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
        self._finish_motion("reset")
        environment = self.data.mocap_pos.copy(),self.data.mocap_quat.copy()
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        self.data.mocap_pos[:],self.data.mocap_quat[:] = environment
        self.data.qpos[:] = position
        self.velocity_command[:] = 0
        self.requested_velocity[:] = 0
        self.actuator.reset()
        self._backstop[:] = 0
        self._command_reason = None
        self._command_was_zero = True
        self.collision_event = None
        self.collision.reset()
        self._requested_time = 0.0
        mujoco.mj_forward(self.model, self.data)

    def command_velocity(self, command: np.ndarray, reason: str | None = None) -> None:
        """Set the joint velocity command. `reason` labels a zero command (a stop) in motion_log."""
        values = np.asarray(command, dtype=float)
        if values.shape != (self.model.nu,) or not np.isfinite(values).all():
            raise ValueError(f"Expected {self.model.nu} finite joint velocities")
        limit = self.config["max_joint_velocity_rad_s"]
        self.requested_velocity[:] = np.clip(values, -limit, limit)
        self._command_reason = None if np.any(values) else (reason or "zero_command")
        self.command_serial += 1
        if self._motion is not None and self._motion["kind"] == "stop" and not np.any(values):
            self._motion["serials"].append(self.command_serial)  # A further stop during the same braking.
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
            self._command_reason = "collision_blocked"

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
            guard_zero = not np.any(cmd)
            lower, upper = self.model.jnt_range.T
            margin = 0.03  # Stop outbound velocity before the mechanical limit.
            if not self.actuator.enabled:
                cmd[(self.data.qpos < lower + margin) & (cmd < 0)] = 0
                cmd[(self.data.qpos > upper - margin) & (cmd > 0)] = 0
            else:
                # The joint keeps moving after a zero command: cut the outward command
                # early by the distance it still needs, and keep it cut (latched) until
                # the command stops pointing outward, so the joint does not creep
                # toward the line in repeated brake/accelerate cycles.
                reach = self.actuator.stopping_distance()
                setpoint = self.actuator.velocity
                self._backstop[(self.data.qpos + np.where(setpoint > 0, reach, 0) > upper - margin) & (cmd > 0)] = 1
                self._backstop[(self.data.qpos - np.where(setpoint < 0, reach, 0) < lower + margin) & (cmd < 0)] = -1
                self._backstop[self._backstop * cmd <= 0] = 0
                cmd[self._backstop != 0] = 0
            self.velocity_command[:] = cmd
            self._monitor_command(cmd, guard_zero)
            self.data.ctrl[:] = self.actuator.step(cmd, dt)
            qvel = self.data.qvel.copy()
            mujoco.mj_step(self.model, self.data)
            self._monitor_step(cmd, qvel, dt)
        # mj_step leaves position-dependent fields at the integration-stage pose.
        # Forward refresh makes camera pose and RGB correspond to current qpos.
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------ physical stop measurement

    def physical_stops(self) -> list:
        """Measured stop episodes (oldest first); see _finish_motion for the fields."""
        return [dict(row) for row in self.motion_log if row["kind"] == "stop"]

    def is_moving(self) -> bool:
        """True unless every joint is below the standstill threshold and the setpoint is zero."""
        return bool(np.any(np.abs(self.data.qvel) >= self.actuator.stopped_velocity)
                    or np.any(self.actuator.velocity))

    _moving = is_moving

    def _monitor_command(self, cmd, guard_zero=False):
        """Open a stop episode when the command becomes zero while the robot moves,
        and a start episode when motion is commanded again."""
        zero = not np.any(cmd)
        kind = None if self._motion is None else self._motion["kind"]
        if zero and kind != "stop" and self._moving():
            if not np.any(self.requested_velocity):
                reason = self._command_reason or "zero_command"
            else:  # Motion was requested; a safety layer reduced it to zero.
                reason = "collision_guard" if guard_zero else "joint_limit_backstop"
            self._finish_motion("stopped_again")
            self._begin_motion("stop", reason)
        elif not zero and (kind == "stop" or (kind is None and self._command_was_zero)):
            self._finish_motion("resumed")
            self._begin_motion("start", "command")
        self._command_was_zero = zero

    def _begin_motion(self, kind, reason):
        self._motion = dict(kind=kind, reason=reason, started_s=float(self.data.time),
            qpos=self.data.qpos.copy(), qvel=self.data.qvel.copy(),
            setpoint=self.actuator.velocity.copy(), setpoint_acceleration=self.actuator.acceleration.copy(),
            peak_speed=float(np.max(np.abs(self.data.qvel))), peak_measured_accel=0.0, peak_measured_jerk=0.0,
            peak_setpoint_accel=0.0, peak_setpoint_jerk=0.0, previous_accel=None,
            setpoint_zero_s=None, below_s=None, profile=[], steps=0,
            stop_clamps=self.actuator.stop_clamps, serials=[self.command_serial])

    def _monitor_step(self, cmd, previous_qvel, dt):
        motion = self._motion
        if motion is None:
            return
        qvel = self.data.qvel
        accel = (qvel - previous_qvel) / dt
        motion["steps"] += 1
        motion["peak_speed"] = max(motion["peak_speed"], float(np.max(np.abs(qvel))))
        motion["peak_measured_accel"] = max(motion["peak_measured_accel"], float(np.max(np.abs(accel))))
        if motion["previous_accel"] is not None:
            motion["peak_measured_jerk"] = max(motion["peak_measured_jerk"],
                                               float(np.max(np.abs(accel - motion["previous_accel"]))) / dt)
        motion["previous_accel"] = accel
        motion["peak_setpoint_accel"] = max(motion["peak_setpoint_accel"], float(np.max(np.abs(self.actuator.acceleration))))
        motion["peak_setpoint_jerk"] = max(motion["peak_setpoint_jerk"], float(np.max(np.abs(self.actuator.jerk))))
        elapsed = float(self.data.time) - motion["started_s"]
        if motion["steps"] % 2 == 1 and len(motion["profile"]) < 100:  # Every 4 ms, first 0.4 s.
            motion["profile"].append((round(1000 * elapsed, 1), float(np.max(np.abs(qvel))),
                                      float(np.max(np.abs(self.actuator.velocity)))))
        if motion["kind"] == "stop":
            if motion["setpoint_zero_s"] is None and not np.any(self.actuator.velocity):
                motion["setpoint_zero_s"] = elapsed
            if self._moving():
                motion["below_s"] = None
            elif motion["below_s"] is None:
                motion["below_s"] = elapsed
            if motion["below_s"] is not None and elapsed - motion["below_s"] >= self.actuator.stopped_hold_s - 1e-9:
                self._finish_motion("stopped")
        elif not np.any(np.abs(cmd - self.actuator.velocity) > 1e-12) or elapsed >= 2.0:
            motion["reached_s"] = elapsed if elapsed < 2.0 else None
            self._finish_motion("reached" if elapsed < 2.0 else "unfinished")

    def _pose_points(self, qpos):
        """Camera optical origin, tool origin and camera rotation for qpos (scratch data)."""
        scratch = self._kinematics
        scratch.qpos[:] = qpos
        scratch.mocap_pos[:], scratch.mocap_quat[:] = self.data.mocap_pos, self.data.mocap_quat
        mujoco.mj_kinematics(self.model, scratch)
        mujoco.mj_camlight(self.model, scratch)
        return (scratch.cam_xpos[self.camera_id].copy(), scratch.xpos[self.model.body("tool_roll").id].copy(),
                scratch.cam_xmat[self.camera_id].reshape(3, 3).copy())

    def _finish_motion(self, outcome):
        """Close the open episode and append its record to motion_log.

        Stop records: stop_time_s is the time from the zero command until every joint
        stays below stopped_velocity_rad_s (measured, after the servo) for
        stopped_hold_s; setpoint_stop_time_s is when the actuator setpoint reached
        zero. Distances compare the pose at the zero command with the pose now.
        """
        motion, self._motion = self._motion, None
        if motion is None:
            return
        elapsed = float(self.data.time) - motion["started_s"]
        camera0, tool0, rotation0 = self._pose_points(motion["qpos"])
        camera1, tool1, rotation1 = self._pose_points(self.data.qpos)
        cosine = np.clip((np.trace(rotation0.T @ rotation1) - 1) / 2, -1, 1)
        travel = self.data.qpos - motion["qpos"]
        record = dict(kind=motion["kind"], reason=motion["reason"], outcome=outcome,
            started_s=motion["started_s"], duration_s=elapsed,
            speed_at_command_rad_s=float(np.max(np.abs(motion["qvel"]))),
            qvel_at_command_rad_s=motion["qvel"].tolist(), setpoint_at_command_rad_s=motion["setpoint"].tolist(),
            peak_speed_rad_s=motion["peak_speed"],
            peak_measured_accel_rad_s2=motion["peak_measured_accel"],
            peak_measured_jerk_rad_s3=motion["peak_measured_jerk"],
            peak_setpoint_accel_rad_s2=motion["peak_setpoint_accel"],
            peak_setpoint_jerk_rad_s3=motion["peak_setpoint_jerk"],
            joint_travel_rad=travel.tolist(), max_joint_travel_rad=float(np.max(np.abs(travel))),
            camera_travel_mm=float(1000 * np.linalg.norm(camera1 - camera0)),
            tool_travel_mm=float(1000 * np.linalg.norm(tool1 - tool0)),
            camera_rotation_deg=float(np.degrees(np.arccos(cosine))),
            stop_clamps=self.actuator.stop_clamps - motion["stop_clamps"],
            command_serials=motion["serials"], profile=motion["profile"])
        if motion["kind"] == "stop":
            record.update(stopped=outcome == "stopped",
                          stop_time_s=motion["below_s"] if outcome == "stopped" else None,
                          setpoint_stop_time_s=motion["setpoint_zero_s"])
        else:
            record.update(reached_command_s=motion.get("reached_s"))
        self.motion_evicted += len(self.motion_log) == self.motion_log.maxlen
        self.motion_log.append(record)

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
        self._finish_motion("closed")  # An unfinished stop is still recorded.
        if self.renderer is not None:
            self.renderer.close()

    def __enter__(self) -> Simulation:
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
