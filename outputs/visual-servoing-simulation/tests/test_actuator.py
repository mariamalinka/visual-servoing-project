"""Actuator dynamics model (actuator.py) and physical stop measurement in simulation.py.

    python -B -m unittest tests.test_actuator -v

The limits tested here are simulation assumptions (actuator_config.json), not
properties of a real drive.
"""
import json
import unittest
from pathlib import Path

import numpy as np

from actuator import ActuatorModel, braking_distance, braking_time, load_actuator_config

DT = 0.002
SPEEDS = (0.05, 0.1, 0.35, 0.6)
PROFILES = ("default", "gentle")


def run(model, segments, joint=0):
    """Step `model` through [(command, seconds), ...]; return setpoint v, a, j arrays."""
    velocity, acceleration, jerk = [], [], []
    for command, seconds in segments:
        target = np.zeros(model.joints)
        target[:] = command
        for _ in range(round(seconds / DT)):
            velocity.append(model.step(target, DT)[joint])
            acceleration.append(model.acceleration[joint])
            jerk.append(model.jerk[joint])
    return np.array(velocity), np.array(acceleration), np.array(jerk)


class Limits(unittest.TestCase):
    def check_limits(self, model, velocity, previous=0.0, previous_accel=0.0, tolerance=1e-9):
        """The limits hold on the setpoint itself, not only on the model's bookkeeping."""
        v = np.r_[previous, velocity]
        a = np.diff(v) / DT
        j = np.diff(np.r_[previous_accel, a]) / DT
        self.assertLessEqual(np.max(np.abs(j)), model.jerk_limit[0] * (1 + tolerance) + tolerance)
        self.assertLessEqual(np.max(np.abs(a)), max(model.acceleration_limit[0], model.deceleration_limit[0]) + 1e-9)
        return a, j

    def test_start_and_stop_from_several_speeds(self):
        for profile in PROFILES:
            for speed in SPEEDS:
                for sign in (1, -1):
                    with self.subTest(profile=profile, speed=sign * speed):
                        model = ActuatorModel(profile)
                        v, a, _ = run(model, [(sign * speed, 1.0), (0.0, 0.5)])
                        self.check_limits(model, v)
                        start, stop = v[:500], v[500:]
                        # No instant change: the first step is limited by the jerk.
                        self.assertLessEqual(abs(start[0]), model.jerk_limit[0] * DT * DT + 1e-12)
                        self.assertLessEqual(abs(stop[0] - start[-1]), model.jerk_limit[0] * DT * DT + 1e-12)
                        # Speeding up obeys the acceleration limit, braking the deceleration limit.
                        self.assertLessEqual(np.max(np.abs(a[:500])), model.acceleration_limit[0] + 1e-9)
                        self.assertLessEqual(np.max(np.abs(a[500:])), model.deceleration_limit[0] + 1e-9)
                        # Reaches the command exactly, never overshoots it, stops exactly.
                        self.assertEqual(start[-1], sign * speed)
                        self.assertLessEqual(np.max(sign * start), speed)
                        self.assertEqual(stop[-1], 0.0)
                        self.assertGreaterEqual(np.min(sign * stop), 0.0)  # A stop never reverses.

    def test_stop_time_and_distance_match_the_analytic_prediction(self):
        for profile in PROFILES:
            for speed in SPEEDS:
                with self.subTest(profile=profile, speed=speed):
                    model = ActuatorModel(profile)
                    run(model, [(speed, 1.0)])
                    v, _, _ = run(model, [(0.0, 0.5)])
                    steps = int(np.argmax(v == 0)) + 1
                    predicted_time = float(braking_time(speed, model.deceleration_limit[0], model.jerk_limit[0]))
                    predicted_distance = float(braking_distance(speed, model.deceleration_limit[0], model.jerk_limit[0]))
                    self.assertAlmostEqual(steps * DT, predicted_time, delta=2 * DT)
                    distance = float(np.sum(v) * DT)
                    self.assertLessEqual(distance, predicted_distance + 1e-9)  # Conservative bound.
                    self.assertGreater(distance, 0.85 * predicted_distance)

    def test_stop_while_still_accelerating(self):
        for profile in PROFILES:
            with self.subTest(profile=profile):
                model = ActuatorModel(profile)
                run(model, [(0.35, 0.06)])
                self.assertGreater(model.acceleration[0], 0)
                bound = float(model.stopping_distance()[0] - abs(model.velocity[0]) * model.servo_lag_allowance_s)
                before, accel = model.velocity[0], model.acceleration[0]
                v, _, _ = run(model, [(0.0, 0.5)])
                self.check_limits(model, v, previous=before, previous_accel=accel)
                self.assertEqual(v[-1], 0.0)
                self.assertGreaterEqual(np.min(v), 0.0)
                self.assertLessEqual(float(np.sum(v) * DT), bound + 1e-9)

    def test_reversal_and_random_commands_respect_jerk(self):
        rng = np.random.default_rng(7)
        for profile in PROFILES:
            model = ActuatorModel(profile)
            segments = [(float(c), float(n) * DT) for c, n in zip(rng.uniform(-.6, .6, 200), rng.integers(1, 80, 200))]
            v, _, _ = run(model, segments + [(0.3, 0.6), (-0.3, 0.8)])
            self.check_limits(model, v)
            self.assertEqual(v[-1], -0.3)
            self.assertLessEqual(np.max(np.abs(v)), 0.6 + 1e-12)

    def test_small_corrections_are_smooth(self):
        model = ActuatorModel("default")
        v, _, _ = run(model, [(0.2, 1.0), (0.21, 0.1), (0.19, 0.1), (0.2, 0.2)])
        self.check_limits(model, v)
        self.assertEqual(v[-1], 0.2)

    def test_repeated_stop_start_cycles_do_not_accumulate(self):
        model = ActuatorModel("default")
        for cycle in range(20):
            v, _, _ = run(model, [(0.35, 0.3), (0.0, 0.2)])
            self.check_limits(model, v)
            self.assertEqual(model.velocity[0], 0.0)
            self.assertEqual(model.acceleration[0], 0.0)
        self.assertEqual(model.stop_clamps, 0)

    def test_stop_during_a_reversal_never_reverses(self):
        model = ActuatorModel("default")
        run(model, [(0.3, 0.3), (-0.3, 0.12)])
        self.assertLess(model.velocity[0], 0)
        v, _, _ = run(model, [(0.0, 0.5)])
        self.assertTrue(np.all(v <= 0))
        self.assertEqual(v[-1], 0.0)

    def test_zero_command_just_before_a_reversal_crosses_zero(self):
        # The one documented exception to the jerk limit: braking hard through zero
        # toward the opposite direction when a stop arrives. The setpoint must not
        # reverse, so the acceleration is cut in one step, and this is counted.
        model = ActuatorModel("default")
        run(model, [(0.3, 0.3)])
        target = np.full(6, -0.3)
        while not (0 < model.velocity[0] < 0.01 and model.acceleration[0] < -3):
            model.step(target, DT)
        v, _, j = run(model, [(0.0, 0.3)])
        self.assertTrue(np.all(v >= 0))
        self.assertEqual(v[-1], 0.0)
        self.assertEqual(model.stop_clamps, 6)  # One step on each of the six joints.
        self.assertEqual(np.count_nonzero(np.abs(j) > model.jerk_limit[0] * (1 + 1e-9)), 1)

    def test_random_commands_with_stops_limit_jerk_except_counted_clamps(self):
        rng = np.random.default_rng(11)
        for profile in PROFILES:
            model = ActuatorModel(profile)
            commands = np.where(rng.random(300) < 0.3, 0.0, rng.uniform(-.6, .6, 300))
            exceptions = 0
            for command, steps in zip(commands, rng.integers(1, 60, 300)):
                for _ in range(steps):
                    before, clamps = model.velocity[0], model.stop_clamps
                    velocity = model.step(np.full(6, command), DT)[0]
                    clamped = model.stop_clamps > clamps
                    exceptions += clamped
                    if not clamped:
                        self.assertLessEqual(abs(model.jerk[0]), model.jerk_limit[0] * (1 + 1e-9))
                    if command == 0:
                        self.assertGreaterEqual(velocity * np.sign(before), 0)  # Never reverses on a stop.
            self.assertEqual(model.stop_clamps, 6 * exceptions)

    def test_disabled_model_passes_the_command_through(self):
        model = ActuatorModel("ideal")
        self.assertFalse(model.enabled)
        for command in ([0.6] * 6, [0.0] * 6, [-0.3, 0.1, 0, 0.2, -0.6, 0.5]):
            np.testing.assert_array_equal(model.step(np.array(command), DT), command)
        np.testing.assert_array_equal(model.stopping_distance(), np.zeros(6))


class Configuration(unittest.TestCase):
    def test_file_profiles_are_valid_and_pass_the_braking_check(self):
        document = json.loads(Path("actuator_config.json").read_text(encoding="utf-8"))
        collision = json.loads(Path("collision_config.json").read_text(encoding="utf-8"))
        limit = json.loads(Path("config.json").read_text(encoding="utf-8"))["max_joint_velocity_rad_s"]
        self.assertIn(document["active_profile"], document["profiles"])
        for name in document["profiles"]:
            with self.subTest(profile=name):
                model = ActuatorModel(name, max_velocity=limit, collision_braking_time_s=collision["braking_time_s"])
                self.assertTrue(model.braking_check["ok"])

    def test_per_joint_limits(self):
        config = load_actuator_config(dict(enabled=True, max_acceleration_rad_s2=[1, 2, 3, 4, 5, 6],
                                           max_deceleration_rad_s2=6, max_jerk_rad_s3=200))
        model = ActuatorModel(config)
        np.testing.assert_array_equal(model.acceleration_limit, [1, 2, 3, 4, 5, 6])
        run(model, [(0.5, 0.05)])
        self.assertLessEqual(model.acceleration[0], 1 + 1e-12)
        self.assertGreater(abs(model.acceleration[5]), 1)

    def test_invalid_configurations_are_rejected(self):
        base = dict(enabled=True, max_acceleration_rad_s2=2, max_deceleration_rad_s2=5, max_jerk_rad_s3=150)
        for change in (dict(max_acceleration_rad_s2=0), dict(max_jerk_rad_s3=-1), dict(max_deceleration_rad_s2=[1, 2]),
                       dict(max_jerk_rad_s3=float("nan")), dict(enabled="yes")):
            with self.subTest(change=change), self.assertRaises(ValueError):
                ActuatorModel(load_actuator_config({**base, **change}))
        with self.assertRaises(ValueError):
            load_actuator_config("no-such-profile")

    def test_braking_slower_than_the_collision_guard_assumes_is_rejected(self):
        slow = load_actuator_config(dict(enabled=True, max_acceleration_rad_s2=2, max_deceleration_rad_s2=2,
                                         max_jerk_rad_s3=150))
        ActuatorModel(slow)  # Usable on its own (e.g. for analysis) ...
        with self.assertRaisesRegex(ValueError, "braking_time_s"):
            ActuatorModel(slow, max_velocity=0.6, collision_braking_time_s=0.12)  # ... but not with the guard.


class SimulatedStops(unittest.TestCase):
    """Physical stop measurement on the MuJoCo arm (no rendering)."""

    @classmethod
    def setUpClass(cls):
        from simulation import Simulation
        cls.Simulation = Simulation
        cls.sims = {name: Simulation(render=False, actuator_config=name) for name in ("ideal", "default")}

    @classmethod
    def tearDownClass(cls):
        for sim in cls.sims.values():
            sim.close()

    def stop(self, profile, joint, speed, *, run_s=0.5):
        sim = self.sims[profile]
        sim.reset()
        command = np.zeros(6)
        command[joint] = speed
        sim.command_velocity(command)
        sim.advance(run_s)
        sim.command_velocity(np.zeros(6), reason="test_stop")
        self.assertFalse(sim.velocity_command.any())  # The command stop is immediate ...
        self.assertTrue(np.any(sim.actuator.velocity))  # ... the physical stop is not.
        sim.advance(0.5)
        return sim.physical_stops()[-1]

    def test_stop_is_measured_for_several_speeds_and_joints(self):
        sim = self.sims["default"]
        for joint in (0, 1, 3, 5):
            for speed in (0.1, 0.35, -0.35, 0.6):
                with self.subTest(joint=joint, speed=speed):
                    record = self.stop("default", joint, speed)
                    self.assertEqual((record["reason"], record["outcome"]), ("test_stop", "stopped"))
                    self.assertTrue(record["stopped"])
                    # Speed at the stop command (the collision guard may have slowed the joint).
                    speed = abs(record["setpoint_at_command_rad_s"][joint])
                    self.assertGreater(speed, 0.05)
                    predicted = float(braking_time(speed, 5.0, 150.0))
                    self.assertAlmostEqual(record["setpoint_stop_time_s"], predicted, delta=2 * DT)
                    # The servo adds a small lag after the setpoint reaches zero.
                    self.assertGreaterEqual(record["stop_time_s"], record["setpoint_stop_time_s"] - 1e-9)
                    self.assertLessEqual(record["stop_time_s"], predicted + 0.08)
                    travel = abs(record["joint_travel_rad"][joint])
                    bound = float(braking_distance(abs(speed), 5.0, 150.0)) + abs(speed) * sim.actuator.servo_lag_allowance_s
                    self.assertLessEqual(travel, bound)
                    self.assertGreater(travel, 0.5 * float(braking_distance(abs(speed), 5.0, 150.0)))
                    self.assertLessEqual(record["peak_setpoint_jerk_rad_s3"], 150.0 * (1 + 1e-9))
                    self.assertLessEqual(record["peak_setpoint_accel_rad_s2"], 5.0 + 1e-9)
                    self.assertGreater(record["camera_travel_mm"] + record["camera_rotation_deg"], 0)

    def test_actuator_stops_take_longer_and_travel_further_than_ideal(self):
        ideal, modelled = self.stop("ideal", 1, 0.35), self.stop("default", 1, 0.35)
        self.assertLess(ideal["stop_time_s"], modelled["stop_time_s"])
        self.assertLess(ideal["max_joint_travel_rad"], modelled["max_joint_travel_rad"])
        self.assertLess(modelled["peak_measured_accel_rad_s2"], ideal["peak_measured_accel_rad_s2"])

    def test_resume_is_smooth_and_interrupts_an_unfinished_stop(self):
        sim = self.sims["default"]
        sim.reset()
        command = np.array([0.3, 0, 0, 0, 0, 0])
        sim.command_velocity(command)
        sim.advance(0.6)
        sim.command_velocity(np.zeros(6), reason="watchdog_pause")
        sim.advance(0.04)  # Resume before the robot has stopped.
        sim.command_velocity(command)
        sim.advance(0.6)
        stop, start = [r for r in sim.motion_log if r["kind"] == "stop"][-1], list(sim.motion_log)[-1]
        self.assertEqual((stop["reason"], stop["outcome"], stop["stop_time_s"]), ("watchdog_pause", "resumed", None))
        self.assertEqual((start["kind"], start["outcome"]), ("start", "reached"))
        self.assertLessEqual(start["peak_setpoint_jerk_rad_s3"], 150.0 * (1 + 1e-9))
        self.assertLessEqual(start["peak_setpoint_accel_rad_s2"], 5.0 + 1e-9)

    def test_repeated_pause_resume_cycles(self):
        sim = self.sims["default"]
        sim.reset()
        command = np.array([0.2, -0.2, 0.2, 0, 0, 0])
        for cycle in range(8):
            sim.command_velocity(command)
            sim.advance(0.35)
            sim.command_velocity(np.zeros(6), reason="watchdog_pause")
            sim.advance(0.3)
        stops = sim.physical_stops()[-8:]
        self.assertEqual([r["outcome"] for r in stops], ["stopped"] * 8)
        times = [r["stop_time_s"] for r in stops]
        self.assertLess(max(times) - min(times), 0.02)  # Each cycle stops the same way.
        self.assertTrue(all(r["peak_setpoint_jerk_rad_s3"] <= 150 * (1 + 1e-9) for r in sim.motion_log))
        self.assertEqual(sim.actuator.stop_clamps, 0)

    def test_joint_limit_backstop_accounts_for_the_stopping_distance(self):
        sim = self.sims["default"]
        sim.reset()
        lower, upper = sim.model.jnt_range[3]
        command = np.zeros(6)
        command[3] = 0.6
        sim.command_velocity(command)
        first = len(sim.motion_log)
        peak = -np.inf
        for _ in range(int((upper - sim.data.qpos[3]) / 0.6 / 0.05) + 40):
            sim.advance(0.05)
            peak = max(peak, sim.data.qpos[3])
        self.assertLess(peak, upper - 0.02)  # The 0.03 rad backstop still holds (within servo lag).
        self.assertGreater(peak, upper - 0.1)  # ... without stopping far too early.
        stops = [r for r in list(sim.motion_log)[first:] if r["kind"] == "stop"]
        # One latched stop, not repeated brake/accelerate cycles toward the line.
        self.assertEqual([(r["reason"], r["outcome"]) for r in stops], [("joint_limit_backstop", "stopped")])
        sim.command_velocity(-command)  # Retreat is allowed at once.
        sim.advance(0.3)
        self.assertLess(sim.data.qpos[3], peak - 0.01)

    def test_stop_while_still_accelerating_stays_within_the_predicted_distance(self):
        sim = self.sims["default"]
        for joint in (0, 1, 5):
            with self.subTest(joint=joint):
                sim.reset()
                command = np.zeros(6)
                command[joint] = 0.35
                sim.command_velocity(command)
                sim.advance(0.06)
                self.assertGreater(sim.actuator.acceleration[joint], 1.0)  # Still accelerating.
                bound = float(sim.actuator.stopping_distance()[joint])
                sim.command_velocity(np.zeros(6), reason="test_stop")
                sim.advance(0.5)
                record = sim.physical_stops()[-1]
                self.assertEqual(record["outcome"], "stopped")
                self.assertLessEqual(abs(record["joint_travel_rad"][joint]), bound)

    def test_closing_records_an_unfinished_stop(self):
        sim = self.Simulation(render=False, actuator_config="default")
        sim.command_velocity(np.array([0.3, 0, 0, 0, 0, 0]))
        sim.advance(0.3)
        sim.command_velocity(np.zeros(6), reason="shutdown")
        self.assertTrue(sim.is_moving())
        sim.advance(0.02)
        sim.close()
        self.assertEqual((sim.motion_log[-1]["reason"], sim.motion_log[-1]["outcome"]), ("shutdown", "closed"))

    def test_guard_and_backstop_stops_are_labelled(self):
        sim = self.sims["default"]
        sim.reset()
        sim.command_velocity(np.array([0.3, 0, 0, 0, 0, 0]))
        sim.advance(0.3)
        sim.command_velocity(np.zeros(6))  # No reason given.
        sim.advance(0.4)
        self.assertEqual(sim.physical_stops()[-1]["reason"], "zero_command")

    def test_reset_clears_the_actuator_and_closes_the_episode(self):
        sim = self.sims["default"]
        sim.reset()
        sim.command_velocity(np.array([0.3, 0, 0, 0, 0, 0]))
        sim.advance(0.3)
        sim.command_velocity(np.zeros(6), reason="stopped")
        sim.advance(0.02)
        sim.reset()
        self.assertEqual(sim.motion_log[-1]["outcome"], "reset")
        self.assertFalse(np.any(sim.actuator.velocity))
        self.assertFalse(np.any(sim.actuator.acceleration))

    def test_collision_block_is_recorded_as_a_physical_stop(self):
        sim = self.Simulation(render=False, actuator_config="default")
        try:
            sim.set_obstacle(True)
            direction = np.zeros(6)
            for joint, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
                sim.reset()
                sim.set_obstacle(True)
                direction[:] = 0
                direction[joint] = 0.35 * sign
                sim.command_velocity(direction)
                for _ in range(80):
                    sim.advance(0.05)
                    if sim.collision_event is not None:
                        break
                if sim.collision_event is not None:
                    break
            if sim.collision_event is None:
                self.skipTest("No single-joint motion reaches the obstacle from home")
            sim.advance(0.5)
            record = sim.physical_stops()[-1]
            self.assertEqual(record["reason"], "collision_blocked")
            self.assertTrue(record["stopped"])
            self.assertEqual(len(sim.forbidden_contacts()), 0)
            distances, _ = sim.collision.distances(sim.data.qpos)
            self.assertTrue(np.all(distances > 0))
        finally:
            sim.close()


if __name__ == "__main__":
    unittest.main()
