"""Simulated actuator dynamics between the velocity command and the joint servos.

The controller, the safety layers and the realtime runtime decide a joint velocity
command. Before this model existed, MuJoCo's velocity servos received that command
unchanged, so a zero command started braking at once with whatever torque the
servo could produce (about 10-90 ms to standstill). Real drives do not follow a
step change in velocity: the drive's trajectory generator limits acceleration and
jerk. This module models that generator.

    command (rad/s) --> ActuatorModel.step() --> servo setpoint (rad/s) --> MuJoCo

Per joint, the setpoint velocity v tracks the command with
- |dv/dt| <= max_acceleration_rad_s2 while the speed |v| increases,
- |dv/dt| <= max_deceleration_rad_s2 while the speed |v| decreases (stops, braking),
- |d2v/dt2| <= max_jerk_rad_s3 at all times.
The acceleration is reduced before the command is reached, so the setpoint does
not overshoot a constant command.

Simplifications (documented in docs/ACTUATOR_MODEL.md):
- Joints are independent. They are not time-synchronised, so during a ramp the
  joint-space direction of motion can differ from the commanded direction.
- The limits are constant: no dependence on pose, payload, torque or temperature.
- The MuJoCo velocity servo (kv gains, force limits in scene.xml) still sits
  after this model and adds its own small lag.
- The parameters are simulation assumptions, not measured values of a real drive.

`enabled: false` passes the command through unchanged: the previous behaviour.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
CONFIG_PATH = ROOT / "actuator_config.json"
LIMIT_KEYS = ("max_acceleration_rad_s2", "max_deceleration_rad_s2", "max_jerk_rad_s3")


def load_actuator_config(selection=None, path=CONFIG_PATH) -> dict:
    """Resolve an actuator configuration.

    selection: None (the file's active_profile), a profile name, or a dict with the
    profile fields (an explicit configuration, e.g. from a study). The result always
    contains the shared fields (stopped threshold, hold time, servo allowance).
    """
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    shared = {key: document[key] for key in ("stopped_velocity_rad_s", "stopped_hold_s",
                                              "servo_lag_allowance_s")}
    if selection is None:
        selection = document["active_profile"]
    if isinstance(selection, str):
        if selection not in document["profiles"]:
            raise ValueError(f"Unknown actuator profile {selection!r}; "
                             f"choose from {sorted(document['profiles'])}")
        profile = dict(document["profiles"][selection], profile=selection)
    elif isinstance(selection, dict):
        profile = dict(selection)
        profile.setdefault("profile", "custom")
    else:
        raise TypeError("Actuator selection must be None, a profile name or a dict")
    return {**shared, **profile}


def _per_joint(value, name, joints):
    array = np.broadcast_to(np.asarray(value, dtype=float), (joints,)).copy() \
        if np.ndim(value) == 0 or len(value) == joints else None
    if array is None:
        raise ValueError(f"{name} must be a number or {joints} numbers")
    if not np.isfinite(array).all() or np.any(array <= 0):
        raise ValueError(f"{name} must be finite and positive")
    return array


def braking_time(speed, deceleration, jerk, acceleration=0.0):
    """Time for the setpoint to reach zero from `speed`, starting at `acceleration`.

    `acceleration` > 0 means currently speeding up; it must first be ramped to zero.
    Worst case used for the checks: braking starts from zero acceleration after that.
    """
    speed, deceleration, jerk = (np.asarray(x, dtype=float) for x in (speed, deceleration, jerk))
    rising = np.maximum(np.asarray(acceleration, dtype=float), 0)
    t1 = rising / jerk
    u = np.abs(speed) + rising ** 2 / (2 * jerk)
    full = u >= deceleration ** 2 / jerk
    return t1 + np.where(full, u / deceleration + deceleration / jerk, 2 * np.sqrt(u / jerk))


def braking_distance(speed, deceleration, jerk, acceleration=0.0):
    """Conservative distance (rad) the setpoint travels until it reaches zero.

    Phase 1 ramps a rising acceleration down to zero (speed grows by a^2/2J).
    Phase 2 brakes with a jerk-limited profile that is symmetric in time, so its
    distance is exactly speed * duration / 2. Braking that is already under way
    is ignored, which only overestimates the distance.
    """
    speed, deceleration, jerk = (np.asarray(x, dtype=float) for x in (speed, deceleration, jerk))
    rising = np.maximum(np.asarray(acceleration, dtype=float), 0)
    t1 = rising / jerk
    u = np.abs(speed) + rising ** 2 / (2 * jerk)
    phase1 = u * t1  # Upper bound: speed never exceeds u during phase 1.
    full = u >= deceleration ** 2 / jerk
    t2 = np.where(full, u / deceleration + deceleration / jerk, 2 * np.sqrt(u / jerk))
    return phase1 + u * t2 / 2


class ActuatorModel:
    """Rate-, acceleration- and jerk-limited velocity setpoint for each joint."""

    def __init__(self, config=None, joints=6, *, max_velocity=None, collision_braking_time_s=None):
        self.config = load_actuator_config(config) if not isinstance(config, dict) or \
            "stopped_velocity_rad_s" not in config else dict(config)
        self.joints = joints
        self.enabled = self.config.get("enabled", True)
        if type(self.enabled) is not bool:
            raise ValueError("enabled must be true or false")
        self.stopped_velocity = float(self.config["stopped_velocity_rad_s"])
        self.stopped_hold_s = float(self.config["stopped_hold_s"])
        self.servo_lag_allowance_s = float(self.config["servo_lag_allowance_s"])
        if not (self.stopped_velocity > 0 and self.stopped_hold_s >= 0 and self.servo_lag_allowance_s >= 0):
            raise ValueError("Invalid stopped threshold, hold time or servo allowance")
        if self.enabled:
            self.acceleration_limit = _per_joint(self.config["max_acceleration_rad_s2"],
                                                 "max_acceleration_rad_s2", joints)
            self.deceleration_limit = _per_joint(self.config["max_deceleration_rad_s2"],
                                                 "max_deceleration_rad_s2", joints)
            self.jerk_limit = _per_joint(self.config["max_jerk_rad_s3"], "max_jerk_rad_s3", joints)
        else:
            self.acceleration_limit = self.deceleration_limit = self.jerk_limit = np.full(joints, np.inf)
        self.braking_check = None
        if max_velocity is not None and collision_braking_time_s is not None:
            self.braking_check = self.check_braking(max_velocity, collision_braking_time_s)
            if not self.braking_check["ok"]:
                raise ValueError(
                    "Actuator braking is slower than the collision guard assumes: worst-case "
                    f"stop {self.braking_check['equivalent_time_s']:.3f} s equivalent > braking_time_s "
                    f"{collision_braking_time_s:.3f} s. Raise max_deceleration_rad_s2 or max_jerk_rad_s3, "
                    "or raise braking_time_s in collision_config.json.")
        self.velocity = np.zeros(joints)
        self.acceleration = np.zeros(joints)
        self.jerk = np.zeros(joints)
        self.stop_clamps = 0  # Steps where a stop cut the acceleration to avoid reversing.

    def reset(self):
        self.stop_clamps = 0
        self.velocity[:] = 0
        self.acceleration[:] = 0
        self.jerk[:] = 0

    def describe(self) -> dict:
        """JSON-ready record of the active parameters (for reports)."""
        record = {key: value for key, value in self.config.items() if key not in ("note",)}
        if self.braking_check is not None:
            record["braking_check"] = self.braking_check
        return record

    def check_braking(self, max_velocity, braking_time_s) -> dict:
        """Compare worst-case stopping with the collision guard's braking assumption.

        The guard (collision.py) keeps a reserve of closing_speed * braking_time_s,
        i.e. it assumes a joint moving at speed v stops within v * braking_time_s.
        Worst case here: moving at max_velocity while still accelerating at the full
        limit, plus the servo lag allowance (v * servo_lag_allowance_s).
        """
        if not self.enabled:
            return dict(ok=True, equivalent_time_s=self.servo_lag_allowance_s, braking_time_s=braking_time_s,
                        note="Actuator model disabled: only the servo lag applies.")
        speed = float(max_velocity)
        distance = braking_distance(speed, self.deceleration_limit, self.jerk_limit, self.acceleration_limit)
        equivalent = distance / speed + self.servo_lag_allowance_s
        worst = float(np.max(equivalent))
        return dict(ok=bool(worst <= braking_time_s + 1e-12), equivalent_time_s=worst,
                    braking_time_s=float(braking_time_s), max_velocity_rad_s=speed,
                    worst_stop_distance_rad=float(np.max(distance)),
                    worst_stop_time_s=float(np.max(braking_time(speed, self.deceleration_limit, self.jerk_limit,
                                                                self.acceleration_limit))))

    def stopping_distance(self) -> np.ndarray:
        """Per-joint conservative distance the setpoint still travels if commanded to zero now."""
        if not self.enabled:
            return np.zeros(self.joints)
        rising = self.acceleration * np.sign(self.velocity)
        return braking_distance(self.velocity, self.deceleration_limit, self.jerk_limit, rising) \
            + np.abs(self.velocity) * self.servo_lag_allowance_s

    def step(self, command, dt) -> np.ndarray:
        """Advance the setpoint by dt toward the command; returns the new setpoint."""
        target = np.asarray(command, dtype=float)
        if not self.enabled:
            self.jerk[:] = 0
            self.acceleration[:] = 0 if dt <= 0 else (target - self.velocity) / dt
            self.velocity[:] = target
            return self.velocity.copy()
        if dt <= 0:
            return self.velocity.copy()
        v, a, jerk_step = self.velocity, self.acceleration, self.jerk_limit * dt
        error = target - v
        # Speed falls when the change opposes the current motion (braking, reversal).
        braking = (v * error < 0) | ((v == 0) & (a * error < 0))
        limit = np.where(braking, self.deceleration_limit, self.acceleration_limit)
        # Largest acceleration x from which ramping back to zero in steps of J*dt
        # (with an exact final step) changes the velocity by at most |error|. With
        # E = |error|/(J*dt^2) and n = floor((sqrt(8E+1)-1)/2): x = J*dt*(E + n(n+1)/2)/(n+1).
        steps = np.abs(error) / (jerk_step * dt)
        n = np.floor((np.sqrt(8 * steps + 1) - 1) / 2)
        reachable = jerk_step * (steps + n * (n + 1) / 2) / (n + 1)
        desired = np.sign(error) * np.minimum(limit, reachable)
        new_a = a + np.clip(desired - a, -jerk_step, jerk_step)
        # Land exactly on the command when this step can reach it within the jerk limit
        # and the acceleration can return to zero on the next step.
        landing = error / dt
        land = (np.abs(landing - a) <= jerk_step * (1 + 1e-9)) & (np.abs(landing) <= jerk_step * (1 + 1e-9))
        new_a = np.where(land, landing, new_a)
        new_v = np.where(land, target, v + new_a * dt)
        # If the command changed faster than the jerk limit allows, the setpoint passes
        # it and comes back, as a jerk-limited drive would. The exception is a stop: a
        # zero command never reverses the motion. There the setpoint stops at zero and
        # its acceleration is cut to zero in one step, exceeding the jerk limit for
        # that step; this is counted in stop_clamps.
        reverse = (target == 0) & ((v * new_v < 0) | ((v == 0) & (new_v != 0)))
        if np.any(reverse):
            self.stop_clamps += int(np.count_nonzero(reverse))
            new_a = np.where(reverse, 0.0, new_a)
            new_v = np.where(reverse, 0.0, new_v)
        self.jerk[:] = (new_a - a) / dt
        self.acceleration[:] = new_a
        self.velocity[:] = new_v
        return self.velocity.copy()


def prediction(speed, config=None) -> dict:
    """Analytic setpoint stop from constant speed (zero acceleration) for reports."""
    model = ActuatorModel(config)
    if not model.enabled:
        return dict(stop_time_s=0.0, stop_distance_rad=0.0)
    time = braking_time(speed, model.deceleration_limit, model.jerk_limit)
    distance = braking_distance(speed, model.deceleration_limit, model.jerk_limit)
    return dict(stop_time_s=np.asarray(time).tolist(), stop_distance_rad=np.asarray(distance).tolist())


if __name__ == "__main__":
    for name in json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["profiles"]:
        model = ActuatorModel(name, max_velocity=0.6, collision_braking_time_s=0.12)
        print(name, json.dumps(model.describe(), indent=1))
