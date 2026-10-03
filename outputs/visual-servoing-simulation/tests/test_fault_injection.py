"""Fault injection: each safety path is triggered deliberately and its reaction checked.

    python -B -m unittest tests.test_fault_injection -v

Every scenario (fault_injection.py) returns what was injected, the expected
reaction and the measured reaction; a test fails if any check fails and prints
all of them. The realtime scenarios start the sensor worker (rendering), so only
`SimulationFaults` is suitable for headless CI. `run.cmd --fault-campaign` runs
them all repeatedly and writes the evidence report.
"""
import sys
import unittest

import fault_injection as fi


class FaultTestCase(unittest.TestCase):
    def assertScenario(self, record):
        failed = [f"{c['check']}: {c['detail']}" for c in record["checks"] if not c["passed"]]
        self.assertTrue(record["checks"], "no checks were made")
        self.assertFalse(failed, f"{record['id']} ({record['fault']}):\n  " + "\n  ".join(failed))


class SimulationFaults(FaultTestCase):
    """No sensor worker or rendering: suitable for headless CI."""

    def test_speed_command_above_the_limit_is_clipped(self):
        record = fi.speed_clip()
        self.assertScenario(record)
        default = next(r for r in record["measured"]["profiles"] if r["profile"] == "default")
        self.assertEqual(default["after_clip_rad_s"], default["limit_rad_s"])


class RealtimeFaults(FaultTestCase):
    """The real runtime with its sensor worker; each fault while the arm is moving."""

    def test_control_loop_stall_stops_with_control_overrun(self):
        self.assertScenario(fi.control_loop_stall())

    def test_stall_just_over_the_deadline_still_stops(self):
        self.assertScenario(fi.control_loop_stall_near_limit())

    def test_stall_below_the_deadline_does_not_stop(self):
        self.assertScenario(fi.control_loop_stall_below_limit())

    def test_sensor_worker_crash_stops_the_arm(self):
        self.assertScenario(fi.sensor_worker_crash())

    @unittest.skipIf(sys.platform == "win32", "Python's Windows pipes are message-oriented: no partial message")
    def test_half_sent_frame_cannot_block_the_control_loop(self):
        # Regression for the defect found in the first fault campaign: a worker killed
        # mid-transfer left half a frame in the pipe and froze the control loop.
        self.assertScenario(fi.sensor_worker_partial_message())

    def test_blocked_result_read_cannot_block_the_control_loop(self):
        self.assertScenario(fi.sensor_result_read_blocked())

    def test_watchdog_stop_reference(self):
        self.assertScenario(fi.watchdog_stop())

    def test_physics_gap_is_capped_and_counted(self):
        self.assertScenario(fi.physics_gap_cap())

    def test_slow_controller_computation_stops_without_applying_its_command(self):
        self.assertScenario(fi.controller_stall())

    def test_clock_stepping_back_stops_with_clock_error(self):
        self.assertScenario(fi.clock_step_back())

    def test_failed_inference_stops_the_alignment(self):
        self.assertScenario(fi.inference_failure())

    def test_collision_block_stops_the_realtime_alignment(self):
        self.assertScenario(fi.collision_block())

    def test_run_limit_ends_a_moving_alignment(self):
        self.assertScenario(fi.run_timeout())

    def test_injection_setting_is_off_by_default_and_validated(self):
        # A clean run with the hooks off is the below-limit stall test's convergence.
        from realtime import RuntimeConfig
        self.assertEqual(RuntimeConfig().inference_failure, 0.0)
        for value in (1.5, -0.1):
            with self.assertRaises(ValueError):
                RuntimeConfig(inference_failure=value)


if __name__ == "__main__":
    unittest.main()
