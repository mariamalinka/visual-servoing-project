"""Mapped-geometry collision clearance, velocity damping and bounded detours.

Collision geometry is known simulation geometry, not visual target evidence.
Owned kinematic scratch data keeps planning from changing the live robot.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import numpy as np

CONFIG_PATH = Path(__file__).resolve().with_name("collision_config.json")


def load_collision_config(changes=None):
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    config.update(changes or {})
    return config


class CollisionPoseError(ValueError):
    pass


class _PathBudget(RuntimeError):
    pass


@dataclass
class CollisionDecision:
    velocity: np.ndarray
    status: str
    clearance_m: float | None
    pair: tuple[str, str] | None


def scale_velocity(request, gradients, lower, limit, velocity_lower=None, velocity_upper=None):
    """Keep the requested joint direction; slow enough to satisfy every clearance bound."""
    lo = -limit if velocity_lower is None else velocity_lower
    hi = limit if velocity_upper is None else velocity_upper
    velocity = np.clip(np.asarray(request,dtype=float),lo,hi)
    if len(gradients):
        rates = gradients@velocity
        approaching = rates < -1e-12
        if np.any(approaching):
            scale = min(1.,float(np.min(lower[approaching]/rates[approaching])))
            velocity *= max(0.,scale)
    return velocity


class CollisionGuard:
    def __init__(self, model, config=None):
        self.model = model
        self.config = load_collision_config(config)
        c = self.config
        if type(c["enabled"]) is not bool:
            raise ValueError("Collision enabled must be boolean")
        for key in ("clearance_m", "self_clearance_m", "influence_m", "lookahead_s",
                    "braking_time_s", "search_detour_degrees"):
            if isinstance(c[key], bool) or not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        if c["influence_m"] <= max(c["clearance_m"], c["self_clearance_m"]):
            raise ValueError("Collision influence must exceed the clearances")
        for key in ("max_segment_checks", "max_segment_depth", "max_plan_checks"):
            if type(c[key]) is not int or c[key] < 1:
                raise ValueError(f"{key} must be a positive integer")
        offsets = np.asarray(c["detour_degrees"], dtype=float)
        if offsets.ndim != 1 or not len(offsets) or not np.isfinite(offsets).all() or np.any(offsets <= 0):
            raise ValueError("Provide positive finite detour angles")
        if not c["detour_axes"] or len(set(c["detour_axes"])) != len(c["detour_axes"]) or any(type(a) is not int or not 0 <= a < model.nv for a in c["detour_axes"]):
            raise ValueError("Detour axes must be unique joint indices")
        if model.nq != 6 or model.nv != 6 or np.any(model.jnt_type != mujoco.mjtJoint.mjJNT_HINGE):
            raise ValueError("Collision guard requires this six-hinge teaching arm")
        self.data = mujoco.MjData(model)
        self.enabled = c["enabled"]
        self.cap = max(.2, c["influence_m"])
        self.geom_names = [model.geom(i).name or f"geom_{i}" for i in range(model.ngeom)]
        self.radii = self._joint_radii()
        self.refresh_pairs()
        self.reset()

    def reset(self):
        self._remaining_checks = None
        self.limited_steps = 0
        self.blocked_steps = 0
        self.plans = 0
        self.detours = 0
        self.rejected_paths = 0
        self.last = CollisionDecision(np.zeros(6), "clear" if self.enabled else "disabled", None, None)
        # Smallest distance above its required margin in the latest check (m) and its
        # pair, for the environment and for robot-robot pairs separately. Recorded for
        # safety evidence only; no decision uses them.
        self.last_slack = dict(environment=None, self=None)
        self.last_slack_pair = dict(environment=None, self=None)

    def refresh_pairs(self):
        model = self.model
        ignored = {frozenset(pair) for pair in self.config["ignored_geom_pairs"]}
        if any(not pair <= set(self.geom_names) for pair in ignored):
            raise ValueError("Ignored collision pair names must exist in the model")
        pairs, margins, robot_pairs = [], [], []
        if self.enabled:
            for i in range(model.ngeom):
                for j in range(i+1, model.ngeom):
                    if frozenset((self.geom_names[i], self.geom_names[j])) in ignored:
                        continue
                    if not ((model.geom_contype[i] & model.geom_conaffinity[j]) or
                            (model.geom_contype[j] & model.geom_conaffinity[i])):
                        continue
                    robot_i,robot_j = np.any(self.radii[i]),np.any(self.radii[j])
                    if not robot_i and not robot_j:
                        continue
                    a = int(model.body_weldid[model.geom_bodyid[i]])
                    b = int(model.body_weldid[model.geom_bodyid[j]])
                    if a == b:
                        continue
                    # Match MuJoCo's direct parent/weld exclusions; retain world contacts.
                    if a and b and (model.body_parentid[a] == b or model.body_parentid[b] == a):
                        continue
                    pairs.append((i,j))
                    margins.append(self.config["self_clearance_m"] if robot_i and robot_j else self.config["clearance_m"])
                    robot_pairs.append(bool(robot_i and robot_j))
        self.pairs = np.asarray(pairs, dtype=int).reshape(-1,2)
        self.margins = np.asarray(margins)
        self.self_pairs = np.asarray(robot_pairs, dtype=bool)  # Robot-robot pairs (self_clearance_m).
        self.pair_keys = {tuple(pair) for pair in pairs}
        self.left,self.right = self.pairs.T
        self.radius_sums = model.geom_rbound[self.left]+model.geom_rbound[self.right]
        self.plane_left = np.flatnonzero(model.geom_type[self.left] == mujoco.mjtGeom.mjGEOM_PLANE)
        self.plane_right = np.flatnonzero(model.geom_type[self.right] == mujoco.mjtGeom.mjGEOM_PLANE)
        self.bodies_left = model.geom_bodyid[self.left]
        self.bodies_right = model.geom_bodyid[self.right]
        # Shared upstream joint rotations move both shapes rigidly: distance is invariant.
        if len(pairs):
            a, b = self.radii[self.pairs[:,0]], self.radii[self.pairs[:,1]]
            self.motion_bounds = np.where((a > 0) & (b > 0), 0, a+b)
        else:
            self.motion_bounds = np.zeros((0,6))

    def _joint_radii(self):
        model = self.model
        radii = np.zeros((model.ngeom,6))
        for geom in range(model.ngeom):
            radius = float(np.linalg.norm(model.geom_pos[geom]) + model.geom_rbound[geom])
            body = int(model.geom_bodyid[geom])
            while body:
                for joint in range(model.body_jntadr[body], model.body_jntadr[body]+model.body_jntnum[body]):
                    radii[geom,model.jnt_dofadr[joint]] = radius+np.linalg.norm(model.jnt_pos[joint])
                radius += float(np.linalg.norm(model.body_pos[body]))
                body = int(model.body_parentid[body])
        return radii

    def _q(self, qpos):
        q = np.asarray(qpos, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Collision queries require six finite measured joints")
        return q

    def sync_environment(self, data):
        self.data.mocap_pos[:] = data.mocap_pos
        self.data.mocap_quat[:] = data.mocap_quat

    def distances(self, qpos, witnesses=False):
        if self._remaining_checks is not None:
            if self._remaining_checks <= 0:
                raise _PathBudget("Bounded collision-planning budget exhausted")
            self._remaining_checks -= 1
        self.data.qpos[:] = self._q(qpos)
        mujoco.mj_kinematics(self.model, self.data)
        cap = self.config["influence_m"] if witnesses else self.cap
        distances = np.full(len(self.pairs), cap)
        segments = np.zeros((len(self.pairs),6)) if witnesses else None
        if not len(self.pairs):
            return distances, segments
        a,b = self.left,self.right
        centers = self.data.geom_xpos
        broad = np.linalg.norm(centers[a]-centers[b], axis=1)-self.radius_sums
        for rows,planes,others in ((self.plane_left,a,b),(self.plane_right,b,a)):
            normals = self.data.geom_xmat[planes[rows]].reshape(-1,3,3)[:,:,2]
            broad[rows] = np.einsum("ij,ij->i",normals,centers[others[rows]]-centers[planes[rows]]) - self.model.geom_rbound[others[rows]]
        for index in np.flatnonzero(broad < cap):
            i,j = self.pairs[index]
            distance = mujoco.mj_geomDistance(self.model, self.data, int(i), int(j), cap,
                                             None if segments is None else segments[index])
            if not np.isfinite(distance):
                raise RuntimeError("Collision distance query returned a nonfinite result")
            distances[index] = distance
        return distances, segments

    def minimum(self, qpos):
        distances,_ = self.distances(qpos)
        if not len(distances):
            return None,None
        index = int(np.argmin(distances))
        return float(distances[index]), tuple(self.geom_names[i] for i in self.pairs[index])

    def pose_clear(self, qpos):
        q = self._q(qpos)
        if np.any(q < self.model.jnt_range[:,0]) or np.any(q > self.model.jnt_range[:,1]):
            return False
        distances,_ = self.distances(q)
        return bool(np.all(distances >= self.margins-1e-10))

    def segment_clear(self, start, goal):
        """Conservative adaptive swept-path check, including between samples.

        Joint-radius bounds limit how far either shape can move over a subsegment.
        Uncertified subsegments are subdivided; exhausting the budget rejects them.
        """
        start,goal = self._q(start),self._q(goal)
        if not self.pose_clear(start) or not self.pose_clear(goal):
            return False
        d0,_ = self.distances(start)
        d1,_ = self.distances(goal)
        stack = [(start,goal,d0,d1,0)]
        checked = 0
        while stack:
            a,b,da,db,depth = stack.pop()
            bound = self.motion_bounds@np.abs(b-a)
            if np.all(np.minimum(da,db)-.5*bound >= self.margins-1e-10):
                continue
            if depth >= self.config["max_segment_depth"] or checked >= self.config["max_segment_checks"]:
                return False
            mid = (a+b)/2
            dm,_ = self.distances(mid)
            checked += 1
            if np.any(dm < self.margins-1e-10):
                return False
            stack.append((mid,b,dm,db,depth+1))
            stack.append((a,mid,da,dm,depth+1))
        return True

    def plan_path(self, start, goal, lower, upper):
        self._remaining_checks = self.config["max_plan_checks"]
        try:
            return self._plan_path(start,goal,lower,upper)
        except _PathBudget:
            self.rejected_paths += 1
            return None
        finally:
            self._remaining_checks = None

    def _plan_path(self, start, goal, lower, upper):
        """Direct path, then bounded three-segment joint detours; no goal oracle."""
        self.plans += 1
        start,goal = self._q(start),self._q(goal)
        lower,upper = self._q(lower),self._q(upper)
        if np.any(lower > upper) or np.any(goal < lower) or np.any(goal > upper):
            return None
        if self.segment_clear(start,goal):
            return [goal.copy()]
        if not self.pose_clear(start) or not self.pose_clear(goal):
            self.rejected_paths += 1
            return None
        for degrees in self.config["detour_degrees"]:
            for axis in self.config["detour_axes"]:
                for sign in (-1,1):
                    a,b = start.copy(),goal.copy()
                    a[axis] += sign*np.deg2rad(degrees)
                    b[axis] += sign*np.deg2rad(degrees)
                    if np.any(a < lower) or np.any(a > upper) or np.any(b < lower) or np.any(b > upper):
                        continue
                    if self.segment_clear(start,a) and self.segment_clear(a,b) and self.segment_clear(b,goal):
                        self.detours += 1
                        return [a,b,goal.copy()]
        self.rejected_paths += 1
        return None

    def filter_velocity(self, qpos, qvel, request, limit):
        request = self._q(request)
        qvel = self._q(qvel)
        if not self.enabled:
            self.last = CollisionDecision(request.copy(), "disabled", None, None)
            return self.last
        distances, segments = self.distances(qpos, witnesses=True)
        slack = distances-self.margins
        for kind, mask in (("environment", ~self.self_pairs), ("self", self.self_pairs)):
            if np.any(mask):
                tightest = int(np.flatnonzero(mask)[np.argmin(slack[mask])])
                self.last_slack[kind] = float(slack[tightest])
                self.last_slack_pair[kind] = tuple(self.geom_names[i] for i in self.pairs[tightest])
            else:
                self.last_slack[kind] = self.last_slack_pair[kind] = None
        if not len(distances):
            self.last = CollisionDecision(request.copy(), "clear", None, None)
            return self.last
        closest = int(np.argmin(distances))
        pair = tuple(self.geom_names[i] for i in self.pairs[closest])
        clearance = float(distances[closest])
        if not np.any(request):
            self.last = CollisionDecision(np.zeros(6), "clear", clearance, pair)
            return self.last
        if np.any(distances < self.margins-1e-7):
            index = int(np.argmin(distances-self.margins))
            pair = tuple(self.geom_names[i] for i in self.pairs[index])
            self.blocked_steps += 1
            self.last = CollisionDecision(np.zeros(6), "blocked", float(distances[index]), pair)
            return self.last
        gradients, bounds, indices = [], [], []
        mujoco.mj_comPos(self.model, self.data)
        for index in np.flatnonzero(distances < self.config["influence_m"]):
            distance = distances[index]
            if distance <= 1e-10:
                continue
            i,j = self.pairs[index]
            normal = (segments[index,3:]-segments[index,:3])/distance
            jac_a,jac_b = np.zeros((3,6)),np.zeros((3,6))
            mujoco.mj_jac(self.model,self.data,jac_a,None,segments[index,:3],int(self.model.geom_bodyid[i]))
            mujoco.mj_jac(self.model,self.data,jac_b,None,segments[index,3:],int(self.model.geom_bodyid[j]))
            gradient = normal@(jac_b-jac_a)
            closing = max(0.,-float(gradient@qvel))
            reserve = distance-self.margins[index]-closing*self.config["braking_time_s"]
            gradients.append(gradient)
            bounds.append(-max(0.,reserve)/self.config["lookahead_s"])
            indices.append(index)
        gradients = np.asarray(gradients).reshape(-1,6)
        bounds = np.asarray(bounds)
        lo,hi = np.full(6,-limit),np.full(6,limit)
        q = self.data.qpos
        lo[q < self.model.jnt_range[:,0]+.03] = 0
        hi[q > self.model.jnt_range[:,1]-.03] = 0
        bounded = np.clip(request,lo,hi)
        velocity = scale_velocity(bounded,gradients,bounds,limit,lo,hi)
        status = "clear"
        if not np.allclose(velocity,bounded,atol=1e-10,rtol=0):
            self.limited_steps += 1
            status = "limited"
            if len(indices):
                index = indices[int(np.argmin(gradients@request-bounds))]
                pair = tuple(self.geom_names[i] for i in self.pairs[index])
                clearance = float(distances[index])
            if np.linalg.norm(velocity) < max(1e-6,.02*np.linalg.norm(request)):
                velocity[:] = 0
                status = "blocked"
                self.blocked_steps += 1
        self.last = CollisionDecision(velocity,status,clearance,pair)
        return self.last

