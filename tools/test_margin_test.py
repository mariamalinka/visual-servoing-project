"""Latency-margin test: configuration contract and margin arithmetic (no simulator needed).

    python -B -m unittest discover -s tools -p test_margin_test.py -v
"""
import copy
import unittest
from unittest.mock import patch

import run_margin_test as m

CONFIG = m.read(m.DEFAULT_CONFIG)


def session(delay, aligned, attempts=10, trips=0, p50=60.0, low_power=False):
    return dict(environment=dict(low_state_episodes=[dict(start_s=0, duration_s=30, state=['cpu_low_frequency'])]) if low_power else {},id=f'margin-learned-{delay:03d}ms', mode='learned', delay_ms=delay, attempts=attempts, aligned=aligned,
                freshness_trips=trips, control_misses=0, runtime_errors=0, unsafe_motion=0, post_stop_motion=0,
                contacts=0, unlatched_stops=0, counts=dict(fault_results=0 if delay == 0 else 100),
                processing_ms=dict(p50=p50 + delay, p99=p50 + delay + 20, max=p50 + delay + 30, count=100, mean=p50 + delay),
                capture_ms=dict(p99=120 + delay), later_alignment_s=dict(p50=4.2, count=9), first_alignment_s=4.3)


class Contract(unittest.TestCase):
    def test_checked_in_config_is_valid(self):
        self.assertEqual(m.validate(CONFIG), [])

    def test_production_limits_and_levels_are_enforced(self):
        for mutate in (lambda c: c['runtime'].update(max_age_s=0.5), lambda c: c.update(added_delay_ms=[25, 50]),
                       lambda c: c.update(added_delay_ms=[0, 50, 25]), lambda c: c.update(alignments_per_level=2),
                       lambda c: c.update(fault_after_s=2.0)):
            c = copy.deepcopy(CONFIG); mutate(c)
            self.assertTrue(m.validate(c))

    def test_only_the_injected_delay_changes(self):
        runtime = m.session_config(CONFIG, 75)['runtime']
        self.assertEqual({k: runtime[k] for k in m.rat.PRODUCTION_RUNTIME}, m.rat.PRODUCTION_RUNTIME)
        self.assertEqual((runtime['inference_stall_s'], runtime['fault_after_s']), (0.075, 0.0))
        self.assertEqual(CONFIG['runtime'], m.rat.PRODUCTION_RUNTIME)  # Not mutated.

    def test_levels_alternate_methods(self):
        plan = m.build_plan(CONFIG)
        self.assertEqual([s['mode'] for s in plan['sessions'][:4]], ['natural', 'learned', 'natural', 'learned'])
        self.assertEqual(plan['sessions'][0]['delay_ms'], 0)


class Margin(unittest.TestCase):
    def margin(self, sessions):
        with patch.object(m, 'trip_phases', return_value=dict(at_start=0, mid_alignment=0)):
            return m.method_margin('learned', sessions, CONFIG, None)

    def test_tolerated_is_last_clean_level_before_first_failure(self):
        r = self.margin([session(0, 10), session(25, 10), session(50, 10), session(75, 7, trips=3), session(100, 0, trips=10)])
        self.assertEqual((r['tolerated_delay_ms'], r['first_failing_delay_ms']), (50, 75))
        self.assertAlmostEqual(r['equivalent_slowdown'], (60 + 50) / 60)

    def test_a_later_lucky_pass_does_not_extend_the_margin(self):
        r = self.margin([session(0, 10), session(25, 9, trips=1), session(50, 10)])
        self.assertEqual((r['tolerated_delay_ms'], r['first_failing_delay_ms']), (0, 25))

    def test_trips_without_failed_alignment_still_count_as_failure(self):
        r = self.margin([session(0, 10), session(25, 10, trips=1)])
        self.assertEqual(r['tolerated_delay_ms'], 0)

    def test_low_power_levels_are_excluded_and_reference_moves_up(self):
        r = self.margin([session(0, 3, trips=7, p50=95, low_power=True), session(25, 10), session(50, 10), session(75, 4, trips=6)])
        self.assertEqual((r['tolerated_delay_ms'], r['first_failing_delay_ms'], r['excluded_levels_ms']), (50, 75, [0]))
        self.assertEqual(r['baseline_processing_ms']['from_level_ms'], 25)
        self.assertAlmostEqual(r['baseline_processing_ms']['p50'], 60)
        self.assertAlmostEqual(r['equivalent_slowdown'], 110 / 60)

    def test_failing_baseline_has_no_margin(self):
        r = self.margin([session(0, 8, trips=2), session(25, 10)])
        self.assertIsNone(r['tolerated_delay_ms'])
        self.assertEqual(r['first_failing_delay_ms'], 0)


class HoldAndResume(unittest.TestCase):
    def test_session_config_adds_only_the_resume_window(self):
        self.assertNotIn('stale_resume_s', m.session_config(CONFIG, 75)['runtime'])
        runtime = m.session_config(CONFIG, 75, 2.0)['runtime']
        self.assertEqual(runtime['stale_resume_s'], 2.0)
        self.assertEqual({k: runtime[k] for k in m.rat.PRODUCTION_RUNTIME}, m.rat.PRODUCTION_RUNTIME)

    def test_holds_are_reported_and_only_ended_alignments_fail_a_level(self):
        held = dict(session(25, 10, trips=4), stale_stops=0, watchdog_pauses=4, paused_s=0.9)
        ended = dict(session(50, 9, trips=3), stale_stops=1, watchdog_pauses=2, paused_s=2.4)
        with patch.object(m, 'trip_phases', return_value=dict(at_start=0, mid_alignment=0)):
            r = m.method_margin('learned', [session(0, 10), held, ended], CONFIG, None)
        self.assertEqual((r['tolerated_delay_ms'], r['first_failing_delay_ms']), (25, 50))
        self.assertEqual([(l['holds'], l['freshness_trips']) for l in r['levels']], [(0, 0), (4, 0), (2, 1)])

    def test_motion_during_a_hold_is_unsafe(self):
        with patch.object(m, 'trip_phases', return_value=dict(at_start=0, mid_alignment=0)):
            r = m.method_margin('learned', [dict(session(0, 10), paused_motion=2)], CONFIG, None)
        self.assertEqual(r['unsafe_events'], 2)


if __name__ == '__main__':
    unittest.main()
