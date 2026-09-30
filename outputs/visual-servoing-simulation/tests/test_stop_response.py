"""Joining runtime stop events with measured physical stops (stop_response.py).

    python -B -m unittest tests.test_stop_response -v
"""
import unittest

from stop_response import format_lines, link, merge, summarize


def stop_record(serials, stop_s=0.1, outcome='stopped', travel=0.02, camera=12.0, kind='stop'):
    record = dict(kind=kind, reason='x', outcome=outcome, started_s=1.0, duration_s=stop_s + 0.02,
                  speed_at_command_rad_s=0.3, peak_speed_rad_s=0.3, max_joint_travel_rad=travel,
                  camera_travel_mm=camera, tool_travel_mm=camera, camera_rotation_deg=1.0,
                  peak_measured_accel_rad_s2=5.0, peak_setpoint_accel_rad_s2=5.0, peak_setpoint_jerk_rad_s3=150.0,
                  peak_measured_jerk_rad_s3=160.0, command_serials=serials)
    if kind == 'stop':
        record.update(stop_time_s=stop_s if outcome == 'stopped' else None, setpoint_stop_time_s=stop_s - 0.04)
    else:
        record.update(reached_command_s=0.19)
    return record


class Link(unittest.TestCase):
    def test_three_quantities_are_kept_apart(self):
        events = [dict(kind='stop', reason='stale_camera', at_s=10.0, deadline_s=10.0, stopped_s=10.002,
                       command_serial=7)]
        row, = link(dict(events=events, motion=[stop_record([7], stop_s=0.15)]))
        self.assertAlmostEqual(row['command_latency_ms'], 2.0)
        self.assertAlmostEqual(row['physical_stop_ms'], 150.0)
        self.assertAlmostEqual(row['deadline_to_standstill_ms'], 152.0)
        self.assertEqual(row['camera_travel_mm'], 12.0)
        self.assertTrue(row['moving'])

    def test_a_zero_command_at_rest_has_no_physical_stop(self):
        row, = link(dict(events=[dict(kind='stop', reason='stopped', at_s=1.0, command_serial=3)], motion=[]))
        self.assertEqual((row['moving'], row['outcome'], row['physical_stop_ms']), (False, 'at_rest', 0.0))

    def test_a_second_stop_during_the_same_braking_links_to_it(self):
        # A collision block opens the stop; the runtime's stop command follows while braking.
        motion = [stop_record([4, 5])]
        row, = link(dict(events=[dict(kind='stop', reason='collision_blocked', at_s=1.0, command_serial=5)],
                         motion=motion))
        self.assertTrue(row['moving'])

    def test_hold_resume_and_interrupted_stop(self):
        events = [dict(kind='pause', reason='stale_camera', at_s=1.0, deadline_s=1.0, stopped_s=1.001, command_serial=2),
                  dict(kind='resume', at_s=1.05, command_serial=3)]
        motion = [stop_record([2], outcome='resumed'), stop_record([3], kind='start')]
        rows = link(dict(events=events, motion=motion))
        self.assertEqual([r['kind'] for r in rows], ['pause', 'resume'])
        self.assertIsNone(rows[0]['physical_stop_ms'])
        self.assertNotIn('deadline_to_standstill_ms', rows[0])
        self.assertAlmostEqual(rows[1]['reached_command_ms'], 190.0)
        summary = summarize(rows)
        self.assertEqual((summary['pause']['interrupted'], summary['pause']['completed']), (1, 0))
        self.assertEqual(summary['resume']['count'], 1)

    def test_evicted_records_are_not_reported_as_at_rest(self):
        events = [dict(kind='stop', reason='stopped', at_s=1.0, command_serial=1),
                  dict(kind='resume', at_s=1.0, command_serial=1)]
        rows = link(dict(events=events, motion=[stop_record([9])], motion_evicted=3))
        self.assertEqual([(r['kind'], r['outcome']) for r in rows], [('stop', 'not_recorded')])
        self.assertEqual(summarize(rows)['stop']['not_recorded'], 1)

    def test_moving_without_a_following_physics_step_is_not_measured(self):
        # Shutdown, or a hold resumed in the same control tick: never reported as "at rest".
        events = [dict(kind='stop', reason='shutdown', at_s=1.0, command_serial=3, moving_at_command=True),
                  dict(kind='resume', at_s=1.0, command_serial=4)]
        rows = link(dict(events=events, motion=[]))
        self.assertEqual([(r['kind'], r['outcome'], r.get('moving')) for r in rows],
                         [('stop', 'not_measured', True), ('resume', 'not_started', None)])
        summary = summarize(rows)
        self.assertEqual((summary['stop']['moving'], summary['stop']['other']), (1, 1))
        self.assertIsNone(summary['stop']['physical_stop_ms'])
        self.assertEqual(summary['resume']['not_started'], 1)

    def test_runs_before_the_actuator_model_have_nothing_to_link(self):
        self.assertEqual(link(dict(events=[dict(kind='stop', reason='stopped', at_s=1.0)], motion=None)), [])


class Summaries(unittest.TestCase):
    def test_merge_takes_counts_and_worst_values(self):
        a = summarize(link(dict(events=[dict(kind='stop', reason='r', at_s=0, command_serial=1)],
                                motion=[stop_record([1], stop_s=0.1, camera=5.0)])))
        b = summarize(link(dict(events=[dict(kind='stop', reason='r', at_s=0, command_serial=1)],
                                motion=[stop_record([1], stop_s=0.2, camera=9.0)])))
        merged = merge([a, None, b])
        self.assertEqual(merged['stop']['count'], 2)
        self.assertAlmostEqual(merged['stop']['physical_stop_ms']['max'], 200.0)
        self.assertEqual(merged['stop']['camera_travel_mm']['max'], 9.0)
        text = '\n'.join(format_lines(merged, dict(profile='default', enabled=True, max_acceleration_rad_s2=2,
                                                    max_deceleration_rad_s2=5, max_jerk_rad_s3=150)))
        self.assertIn('physical stopping time', text)
        self.assertIn('simulation assumption', text)

    def test_no_events(self):
        self.assertEqual(summarize([]), {})
        self.assertIn('No stops', format_lines({})[0])


if __name__ == '__main__':
    unittest.main()
