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


class ConfidenceIntervals(unittest.TestCase):
    def margin(self, sessions, config=CONFIG):
        with patch.object(m, 'trip_phases', return_value=dict(at_start=0, mid_alignment=0)):
            return m.method_margin('learned', sessions, config, None)

    def test_every_level_reports_its_interval(self):
        r = self.margin([session(0, 10), session(25, 7, trips=3)])
        first, second = r['levels']
        self.assertAlmostEqual(first['success_ci']['ci_low'], 0.6915, places=4)
        self.assertEqual(first['success_ci']['ci_high'], 1.0)
        self.assertLess(second['success_ci']['ci_low'], 0.7)
        self.assertEqual(m.ci_csv(first), (0.6915, 1.0))

    def test_default_margin_rule_is_observed_and_says_so(self):
        rule, text = m.tolerated_rule_text(CONFIG, 10)
        self.assertEqual(rule, 'aligned 100% (observed)')
        self.assertIn('69.2%', text)
        self.assertIn('not 100%', text)
        self.assertIn('72 alignments per level', text)

    def test_a_declared_required_rate_uses_the_lower_bound(self):
        config = dict(copy.deepcopy(CONFIG), required_success_rate=0.95)
        r = self.margin([session(0, 10), session(25, 10)], config)
        self.assertIsNone(r['tolerated_delay_ms'])  # 10/10 cannot show 95%.
        r = self.margin([session(0, 80, attempts=80), session(25, 80, attempts=80)], config)
        self.assertEqual(r['tolerated_delay_ms'], 25)


class HoldAndResume(unittest.TestCase):
    def test_session_config_adds_only_the_resume_window(self):
        self.assertEqual(m.session_config(CONFIG, 75)['runtime']['stale_resume_s'], 2.0)  # Default: hold and resume.
        self.assertEqual(m.session_config(CONFIG, 75, 0)['runtime']['stale_resume_s'], 0)  # Stop setting.
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



class PhysicalResponse(unittest.TestCase):
    def test_hold_physical_stop_is_reported_not_gated(self):
        block = dict(physical_stop_ms=dict(count=3, max=131.0), camera_travel_mm=dict(count=3, max=4.25))
        level = dict(physical_stops=dict(pause=block), actuator=dict(profile='default'))
        self.assertEqual(m.physical_csv(level), (131.0, 4.25))
        self.assertEqual(m.physical_csv(dict(physical_stops=None)), ('', ''))
        line, = m.physical_lines([dict(label='SIFT', levels=[level])])
        self.assertIn('131 ms', line)
        self.assertIn('`default`', line)
        self.assertIn('not real-robot data', line)
        self.assertEqual(m.physical_lines([dict(label='SIFT', levels=[dict(physical_stops=None)])]), [])



class SafetyEnvelope(unittest.TestCase):
    def margin(self, sessions):
        with patch.object(m, 'trip_phases', return_value=dict(at_start=0, mid_alignment=0)):
            return m.method_margin('learned', sessions, CONFIG, None)

    def test_a_safety_failure_ends_the_tolerated_range(self):
        import safety_metrics
        good = dict(steps=5, min_environment_slack_m=0.003, min_self_slack_m=0.01, min_joint_margin_rad=0.4, peak_command_rad_s=0.35,
                    peak_speed_rad_s=0.34, limits=dict(max_joint_velocity_rad_s=0.6))
        bad = dict(good, min_environment_slack_m=-0.001)
        levels = [session(0, 10), session(25, 10), session(50, 10)]
        for level, record in zip(levels, (good, good, bad)):
            level.update(safety=record, safety_metrics=safety_metrics.evaluate(record))
        r = self.margin(levels)
        self.assertEqual((r['tolerated_delay_ms'], r['first_failing_delay_ms']), (25, 50))
        self.assertEqual(m.safety_csv(r['levels'][2])[1], 0)  # environment_clearance_pass
        text = '\n'.join(m.safety_lines([r]))
        self.assertIn('**FAIL**', text)
        self.assertIn('REQ-11', text)

    def test_levels_without_a_record_are_unaffected(self):
        r = self.margin([session(0, 10), session(25, 10)])
        self.assertEqual(r['tolerated_delay_ms'], 25)
        self.assertEqual(m.safety_lines([r]), [])
        self.assertEqual(set(m.safety_csv(r['levels'][0])), {''})


if __name__ == '__main__':
    unittest.main()
