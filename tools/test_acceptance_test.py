"""The acceptance test must reject failures, missing evidence and changed conditions.

Runs without the simulator, GPU or rendering:
    python -B -m unittest discover -s tools -p test_acceptance_test.py -v
"""
import copy
import gzip
import json
import tempfile
import unittest
from pathlib import Path

import run_acceptance_test as t

CONFIG = t.read(t.DEFAULT_CONFIG)


def frames(generation, count, processing_ms, start_s, sequence):
    rows = []
    for i in range(count):
        captured = start_s + i * 0.05
        rows.append(dict(sequence=sequence + i, generation=generation, captured_s=captured,
                         rendered_s=captured + 0.01, finished_s=captured + 0.01 + processing_ms / 1000,
                         capture_active=True, stage_ms={}))
    return rows


def commands(generation, count, capture_ms):
    return [dict(generation=generation, capture_to_command_ms=capture_ms) for _ in range(count)]


def write_session(directory, spec, attempts_spec, *, start_s=0.0, stale=False, extra_counts=None, duration=None):
    """Synthetic evidence in exactly the format run_session() writes."""
    chunks, attempts, sequence, now = [], [], 0, start_s
    for index, (pose, processing, outcome) in enumerate(attempts_spec):
        generation = index + 1
        sensor = frames(generation, 60, processing, now, sequence); sequence += 60
        cmds = commands(generation, 40, processing + 60)
        reason = 'stale_camera' if outcome == 'stale_camera' else outcome
        events = [dict(kind='align', at_s=now, generation=generation),
                  dict(kind='stop', reason=reason, at_s=now + 3.0, generation=generation)]
        chunks.append(dict(sensor_frames=sensor, frames=cmds, events=events, loop_gaps_ms=[2.0] * 100))
        attempts.append(dict(index=index, generation=generation, outcome=outcome, pose_index=pose, qpos=[0] * 6,
                             error_px=0.2, worker_pid=1234, stop_stayed_latched=True,
                             physical_accuracy=dict(camera=dict(position_error_mm=0.4, orientation_error_deg=0.1),
                                                    tool=dict(position_error_mm=0.5, orientation_error_deg=0.1))))
        now += 5.0
    counts = dict(sensor_received=sum(len(c['sensor_frames']) for c in chunks),
                  accepted=sum(len(c['frames']) for c in chunks), sensor_rows_evicted=0, command_rows_evicted=0,
                  loop_rows_evicted=0, control_deadline_misses=0, unsafe_motion_ticks=0, post_stop_motion_ticks=0,
                  contacts=0)
    counts.update(extra_counts or {})
    raw = directory / 'traces' / (spec['id'] + '.jsonl.gz')
    with gzip.open(raw, 'wt', encoding='utf-8') as stream:
        for chunk in chunks:
            stream.write(json.dumps(chunk) + '\n')
    ending = dict(counts=counts, duration_s=now - start_s if duration is None else duration, error=None,
                  harness_error=None, worker_alive=False, worker_executable='python.exe')
    t.write(directory / 'traces' / (spec['id'] + '-attempts.json'), attempts)
    t.write(directory / 'traces' / (spec['id'] + '-ending.json'), ending)
    result, _ = t.summarize(raw, attempts, ending, {k: spec[k] for k in ('id', 'phase', 'mode', 'pose_ids')},
                            CONFIG, score=False)
    return result


class Evidence:
    """A complete synthetic run: fresh sessions per pose, then one sustained session per method."""
    def __init__(self, directory, *, learned_later_ms=80, sustained_s=730, learned_outcome='converged'):
        self.directory = directory; (directory / 'traces').mkdir()
        self.plan = t.build_plan(CONFIG, smoke=False)
        self.sessions = []
        poses = self.plan['pose_ids']
        for spec in self.plan['sessions']:
            if spec['phase'] == 'repeatability':
                spec_attempts = [(spec['pose_ids'][0], 80, 'converged')] * self.plan['alignments_per_pose']
                result = write_session(directory, spec, spec_attempts)
            else:
                later = learned_later_ms if spec['mode'] == 'learned' else 80
                outcome = learned_outcome if spec['mode'] == 'learned' else 'converged'
                spec_attempts = [(poses[0], 80, 'converged')] + [
                    (poses[i % len(poses)], later, outcome) for i in range(1, 12 * len(poses))]
                result = write_session(directory, spec, spec_attempts, duration=sustained_s)
            self.sessions.append(result)

    def finalize(self, **kwargs):
        return t.finalize(self.directory, self.sessions, CONFIG, self.plan, dict(plan=self.plan), True, **kwargs)


class ConfigurationContract(unittest.TestCase):
    def test_checked_in_config_is_valid(self):
        self.assertEqual(t.validate_config(CONFIG), [])

    def test_production_limits_cannot_be_relaxed(self):
        for key, value in (('max_age_s', 0.5), ('max_control_gap_s', 0.1), ('transport_s', 0.0)):
            with self.subTest(key=key):
                c = copy.deepcopy(CONFIG); c['runtime'][key] = value
                self.assertTrue(t.validate_config(c))

    def test_both_methods_three_poses_and_10_to_15_minutes_required(self):
        c = copy.deepcopy(CONFIG); c['methods'] = ['natural']
        self.assertTrue(t.validate_config(c))
        c = copy.deepcopy(CONFIG); c['poses_degrees'] = c['poses_degrees'][:2]
        self.assertTrue(t.validate_config(c))
        for seconds in (300, 1200):
            c = copy.deepcopy(CONFIG); c['sustained']['seconds'] = seconds
            self.assertTrue(t.validate_config(c))

    def test_long_profile_fits_in_one_hour(self):
        long = t.read(t.LONG_CONFIG)
        self.assertEqual(t.validate_config(long), [])
        self.assertEqual(long['profile'], 'long')
        # Everything except duration and coverage must match the standard test.
        for key in ('runtime', 'poses_degrees', 'criteria', 'post_stop_seconds', 'repeatability', 'methods', 'schedule'):
            self.assertEqual(long[key], CONFIG[key], key)
        # Measured on this laptop: ~215 s for both repeatability phases, ~10 s per sustained start/stop.
        estimate = 215 + 2 * (long['sustained']['seconds'] + 10) + 60
        self.assertLess(estimate, 3600)
        self.assertGreaterEqual(long['sustained']['seconds'] + 105, 27 * 60)  # About 30 minutes per method.

    def test_profile_duration_ranges_are_enforced(self):
        long = t.read(t.LONG_CONFIG)
        for seconds in (720, 1800):
            c = copy.deepcopy(long); c['sustained']['seconds'] = seconds
            self.assertTrue(t.validate_config(c), seconds)
        c = copy.deepcopy(long); c['profile'] = 'forever'
        self.assertTrue(t.validate_config(c))

    def test_long_and_config_flags_conflict(self):
        with self.assertRaises(SystemExit):
            t.main(['--long', '--config', str(t.DEFAULT_CONFIG), '--report-only', 'x'])

    def test_protected_inputs(self):
        for name in ('control.py', 'calibration.py', 'scene.xml', 'realtime.py', 'config.json', 'reference/natural_goal.npz'):
            self.assertTrue(t.is_protected(name), name)
        for name in ('learned_perception.py', 'perception.py', 'learned_cuda_graphs.py', 'app.py'):
            self.assertFalse(t.is_protected(name), name)

    def test_approved_change_must_match_both_hashes(self):
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            t.write(d / 'base.json', dict(fingerprint=dict(sha256={'realtime.py': 'old', 'control.py': 'c'}, runtime={})))
            t.write(d / 'approved.json', dict(changes=[dict(file='realtime.py', title='hold', baseline_sha256='old', sha256='new')]))
            ok = t.baseline_check(dict(sha256={'realtime.py': 'new', 'control.py': 'c'}, runtime={}), d / 'base.json', d / 'approved.json')
            self.assertEqual((ok['protected_changed'], ok['approved_changes']), ([], [dict(file='realtime.py', title='hold')]))
            for current in ({'realtime.py': 'other', 'control.py': 'c'}, {'realtime.py': 'new', 'control.py': 'x'}):
                bad = t.baseline_check(dict(sha256=current, runtime={}), d / 'base.json', d / 'approved.json')
                self.assertTrue(bad['protected_changed'], current)
            t.write(d / 'base2.json', dict(fingerprint=dict(sha256={'realtime.py': 'older', 'control.py': 'c'}, runtime={})))
            stale = t.baseline_check(dict(sha256={'realtime.py': 'new', 'control.py': 'c'}, runtime={}), d / 'base2.json', d / 'approved.json')
            self.assertEqual(stale['protected_changed'], ['realtime.py'])

    def test_checked_in_approval_matches_the_repository(self):
        from run_camera_robustness import fingerprint
        check = t.baseline_check(fingerprint(), t.DEFAULT_BASELINE)
        self.assertEqual(check['protected_changed'], [])

    def test_watchdog_response_is_explicit_and_keeps_production_limits(self):
        self.assertEqual(t.DEFAULT_STALE_RESUME_MS, 2000.0)
        self.assertEqual(t.with_stale_resume(CONFIG, 0)['runtime']['stale_resume_s'], 0.0)
        runtime = t.with_stale_resume(CONFIG, t.DEFAULT_STALE_RESUME_MS)['runtime']
        self.assertEqual({k: runtime[k] for k in t.PRODUCTION_RUNTIME}, t.PRODUCTION_RUNTIME)
        self.assertEqual(runtime['stale_resume_s'], 2.0)
        self.assertNotIn('stale_resume_s', CONFIG['runtime'])  # Not mutated.
        self.assertIn('**stop**', t.watchdog_line({}))
        self.assertIn('hold and resume', t.watchdog_line(dict(runtime_config=dict(stale_resume_s=2.0))))

    def test_plan(self):
        plan = t.build_plan(CONFIG, smoke=False)
        rep = [s for s in plan['sessions'] if s['phase'] == 'repeatability']
        self.assertEqual(len(rep), 2 * len(CONFIG['poses_degrees']))
        self.assertTrue(all(s['max_attempts'] == 3 and len(s['pose_ids']) == 1 for s in rep))
        sus = [s for s in plan['sessions'] if s['phase'] == 'sustained']
        self.assertEqual([s['mode'] for s in sus], ['natural', 'learned'])
        self.assertTrue(all(s['seconds'] == CONFIG['sustained']['seconds'] and s['max_attempts'] is None for s in sus))
        only = t.build_plan(CONFIG, smoke=True, only='learned')
        self.assertTrue(all(s['mode'] == 'learned' for s in only['sessions']))


class SessionAnalysis(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.directory = Path(self.tmp.name); (self.directory / 'traces').mkdir()
        self.spec = dict(id='s', phase='sustained', mode='learned', pose_ids=[0, 1, 2])

    def tearDown(self):
        self.tmp.cleanup()

    def test_first_later_split_timing_and_freshness(self):
        r = write_session(self.directory, self.spec, [(0, 60, 'converged'), (1, 200, 'stale_camera'), (2, 200, 'converged')])
        self.assertEqual((r['attempts'], r['aligned'], r['freshness_trips']), (3, 2, 1))
        self.assertAlmostEqual(r['first_processing_ms']['max'], 60, places=6)
        self.assertAlmostEqual(r['later_processing_ms']['p50'], 200, places=6)
        self.assertAlmostEqual(r['first_alignment_s'], 3.0)
        self.assertEqual(r['stop_reasons'], {'converged': 2, 'stale_camera': 1})
        self.assertEqual(r['telemetry_lost'], 0)

    def test_watchdog_holds_count_as_freshness_trips(self):
        r = write_session(self.directory, self.spec, [(0, 60, 'converged'), (1, 60, 'converged')],
                          extra_counts=dict(watchdog_pauses=3, paused_s=0.6, paused_motion_ticks=0))
        self.assertEqual((r['aligned'], r['stale_stops'], r['watchdog_pauses'], r['freshness_trips']), (2, 0, 3, 3))
        self.assertAlmostEqual(r['paused_s'], 0.6)

    def test_older_evidence_without_hold_counters_is_unchanged(self):
        r = write_session(self.directory, self.spec, [(0, 60, 'converged'), (1, 200, 'stale_camera')])
        self.assertEqual((r['freshness_trips'], r['stale_stops'], r['watchdog_pauses'], r['paused_motion']), (1, 1, 0, 0))

    def test_lost_telemetry_is_detected_not_crashed(self):
        r = write_session(self.directory, self.spec, [(0, 60, 'converged')], extra_counts=dict(sensor_received=999))
        self.assertGreater(r['telemetry_lost'], 0)


class Verdicts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.directory = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def verdict(self):
        return t.read(self.directory / 'verdict.json')

    def test_clean_run_passes_and_report_has_every_requested_metric(self):
        self.assertEqual(Evidence(self.directory).finalize(), 'PASS')
        report = (self.directory / 'REPORT.md').read_text(encoding='utf-8')
        for text in ('Alignment success', 'Processing p95 / p99 / max', 'Capture-to-command p99 / max',
                     'Freshness watchdog trips', 'Control deadline misses', 'First-alignment processing',
                     'Later-alignment processing', 'Unsafe / post-stop motion', 'SIFT', 'Learned GPU'):
            self.assertIn(text, report)

    def test_later_alignment_regression_fails_learned_only(self):
        self.assertEqual(Evidence(self.directory, learned_later_ms=130).finalize(), 'FAIL')
        methods = {m['mode']: m for m in self.verdict()['methods']}
        self.assertEqual(methods['natural']['verdict'], 'PASS')
        self.assertTrue(any('later/first' in f for f in methods['learned']['failures']))

    def test_freshness_failures_fail(self):
        Evidence(self.directory, learned_outcome='stale_camera').finalize()
        learned = {m['mode']: m for m in self.verdict()['methods']}['learned']
        self.assertTrue(any(f.startswith('alignment success') for f in learned['failures']))
        self.assertTrue(any(f.startswith('watchdog_stops') for f in learned['failures']))

    def test_success_gate_uses_the_lower_confidence_bound(self):
        evidence = Evidence(self.directory)
        n = sum(s['attempts'] for s in evidence.sessions if s['mode'] == 'learned')
        self.assertGreaterEqual(n, 72)  # Enough all-success trials to demonstrate 95%.
        self.assertEqual(evidence.finalize(), 'PASS')
        method = {m['mode']: m for m in self.verdict()['methods']}['learned']
        self.assertTrue(method['success_ci']['meets_required'])
        self.assertAlmostEqual(method['success_ci']['ci_low'], 0.025 ** (1 / n), places=9)

    def test_a_safety_envelope_violation_fails_the_method(self):
        evidence = Evidence(self.directory)
        record = dict(steps=100, min_environment_slack_m=0.004, min_environment_pair=('a', 'b'), min_self_slack_m=0.01, min_joint_margin_rad=0.3,
                      min_joint_margin_joint=2, peak_command_rad_s=0.35, peak_speed_rad_s=0.34, peak_speed_joint=1,
                      limits=dict(max_joint_velocity_rad_s=0.6))
        sessions = copy.deepcopy(evidence.sessions)
        for s in sessions:
            bad = s['mode'] == 'learned' and s['phase'] == 'sustained'
            s['safety'] = dict(record, min_environment_slack_m=-0.002) if bad else record
            s['safety_metrics'] = t.safety_metrics_of(s['safety'])
        methods = {mode: t.method_summary(mode, sessions, self.directory, CONFIG, evidence.plan)
                   for mode in ('natural', 'learned')}
        self.assertEqual(methods['natural']['verdict'], 'PASS')
        self.assertEqual(methods['learned']['verdict'], 'FAIL')
        self.assertTrue(any(f.startswith('REQ-11') for f in methods['learned']['failures']), methods['learned']['failures'])
        self.assertAlmostEqual(methods['learned']['safety']['min_environment_slack_m'], -0.002)

    def test_ten_of_ten_is_not_enough_to_claim_95_percent(self):
        evidence = Evidence(self.directory)
        sessions = copy.deepcopy(evidence.sessions)
        for s in sessions:  # Same data, but only 10 successful alignments per method.
            s['attempts'] = s['aligned'] = 2 if s['phase'] == 'repeatability' else 0
        methods = [t.method_summary(mode, sessions, self.directory, CONFIG, evidence.plan) for mode in ('natural', 'learned')]
        for m in methods:
            self.assertEqual((m['aligned'], m['attempts'], m['success_rate']), (10, 10, 1.0))
            self.assertTrue(any('lower bound 69.2% < required 95.0%' in f for f in m['failures']), m['failures'])

    def test_failed_alignment_rule_is_separate_from_the_confidence_rule(self):
        evidence = Evidence(self.directory)
        sessions = copy.deepcopy(evidence.sessions)
        sustained = next(s for s in sessions if s['mode'] == 'learned' and s['phase'] == 'sustained')
        # One failure in 175 alignments: the confidence rule passes (lower bound about 96.9%),
        # but the separate zero-failure rule does not.
        sustained['attempts'] += 100; sustained['aligned'] += 99
        m = t.method_summary('learned', sessions, self.directory, CONFIG, evidence.plan)
        self.assertTrue(m['success_ci']['meets_required'])
        self.assertTrue(any(f.startswith('failed alignments 1') for f in m['failures']))
        relaxed = copy.deepcopy(CONFIG); relaxed['criteria']['max_failed_alignments'] = None
        m = t.method_summary('learned', sessions, self.directory, relaxed, evidence.plan)
        self.assertFalse(any('alignment' in f for f in m['failures']), m['failures'])

    def test_configs_from_before_confidence_intervals_use_the_observed_rate(self):
        evidence = Evidence(self.directory)
        legacy = copy.deepcopy(CONFIG)
        for key in ('required_success_rate', 'confidence', 'max_failed_alignments'):
            legacy['criteria'].pop(key)
        legacy['criteria']['success_rate'] = 1.0
        sessions = copy.deepcopy(evidence.sessions)
        for s in sessions:
            s['attempts'] = s['aligned'] = 2 if s['phase'] == 'repeatability' else 0
        m = t.method_summary('learned', sessions, self.directory, legacy, evidence.plan)
        self.assertFalse(any('alignment success' in f for f in m['failures']))
        self.assertIsNotNone(m['success_ci']['ci_low'])  # Still reported.

    def test_report_states_the_interval_and_its_meaning(self):
        Evidence(self.directory).finalize()
        report = (self.directory / 'REPORT.md').read_text(encoding='utf-8')
        for text in ('95% confidence interval, exact Clopper-Pearson', 'success rate demonstrated',
                     'does not prove 100% reliability', 'needs at least 72 alignments',
                     '95% lower confidence bound on alignment success >= 95.0%'):
            self.assertIn(text, report)

    def test_config_cannot_require_100_percent_as_a_confidence_claim(self):
        bad = copy.deepcopy(CONFIG); bad['criteria']['required_success_rate'] = 1.0
        self.assertTrue(t.validate_config(bad))

    def test_pause_budget(self):
        evidence = Evidence(self.directory)
        for paused_s, expected in ((0.0, 'PASS'), (4.0, 'PASS'), (40.0, 'FAIL')):
            with self.subTest(paused_s=paused_s):
                sessions = copy.deepcopy(evidence.sessions)
                for s in sessions:  # About 3 s per alignment; 4 s held of ~490 s total is < 5%.
                    s['watchdog_pauses'] = 2 if paused_s else 0
                    s['freshness_trips'] = s['watchdog_pauses']
                    s['paused_s'] = paused_s / len(sessions)
                status = t.finalize(self.directory, sessions, CONFIG, evidence.plan, dict(plan=evidence.plan), True)
                self.assertEqual(status, expected)
        self.assertTrue(any(f.startswith('paused ') for m in self.verdict()['methods'] for f in m['failures']))

    def test_configs_from_before_hold_and_resume_fail_every_trip(self):
        evidence = Evidence(self.directory)
        legacy = copy.deepcopy(CONFIG)
        legacy['criteria'].pop('watchdog_stops'); legacy['criteria'].pop('paused_fraction')
        legacy['criteria']['freshness_trips'] = 0
        sessions = copy.deepcopy(evidence.sessions)
        sessions[-1].update(freshness_trips=1, watchdog_pauses=1, stale_stops=0, paused_s=0.1)
        self.assertEqual(t.finalize(self.directory, sessions, legacy, evidence.plan, dict(plan=evidence.plan), True), 'FAIL')

    def test_short_sustained_run_fails(self):
        self.assertEqual(Evidence(self.directory, sustained_s=500).finalize(), 'FAIL')
        self.assertTrue(any('sustained duration' in f for m in self.verdict()['methods'] for f in m['failures']))

    def test_each_safety_counter_fails(self):
        evidence = Evidence(self.directory)
        for key in ('control_misses', 'unsafe_motion', 'post_stop_motion', 'contacts', 'unlatched_stops',
                    'telemetry_lost', 'runtime_errors', 'missing_endpoints', 'paused_motion', 'stale_stops'):
            with self.subTest(key=key):
                sessions = copy.deepcopy(evidence.sessions)
                sessions[-1][key] = 1
                status = t.finalize(self.directory, sessions, CONFIG, evidence.plan, dict(plan=evidence.plan), True)
                self.assertEqual(status, 'FAIL')

    def test_incomplete_invalid_and_smoke_never_pass(self):
        evidence = Evidence(self.directory)
        self.assertEqual(t.finalize(self.directory, evidence.sessions, CONFIG, evidence.plan, {}, False), 'INCOMPLETE')
        self.assertEqual(evidence.finalize(invalid_reason='source changed'), 'INVALID')
        self.assertEqual(evidence.finalize(smoke=True), 'SMOKE_ONLY')
        partial = dict(evidence.plan, only_method='natural')
        sift = [s for s in evidence.sessions if s['mode'] == 'natural']
        self.assertEqual(t.finalize(self.directory, sift, CONFIG, partial, {}, True), 'PARTIAL')

    def test_worker_restart_in_sustained_run_fails(self):
        evidence = Evidence(self.directory)
        evidence.sessions[-1]['worker_pids'] = [1, 2]
        self.assertEqual(evidence.finalize(), 'FAIL')


class Procedure(unittest.TestCase):
    def manifest(self, uptime, ac=1, guid='ded574b5-45a0-4f42-8737-46345c09c238'):
        return dict(uptime_at_start_s=uptime, session_boundaries=[
            dict(power=dict(ACLineStatus=ac), power_mode=dict(effective_overlay=guid))] * 2)

    def test_conditions_met(self):
        text = '\n'.join(t.procedure_lines(self.manifest(1200)))
        self.assertIn('Fresh restart: yes', text)
        self.assertIn('AC power at every session start: yes', text)
        self.assertIn('Best performance: yes', text)
        self.assertIn('Not checked automatically', text)

    def test_conditions_broken_are_flagged(self):
        text = '\n'.join(t.procedure_lines(self.manifest(5 * 3600, ac=0, guid='961cc777-2547-4f9d-8174-7d86181b8a7a')))
        self.assertIn('Fresh restart: NO', text)
        self.assertIn('AC power at every session start: NO', text)
        self.assertIn('Best performance: NO (Best power efficiency)', text)

    def test_older_runs_say_not_recorded(self):
        self.assertIn('Fresh restart: not recorded', '\n'.join(t.procedure_lines({})))



class PhysicalStops(unittest.TestCase):
    """Actuator-model stop measurements are reported, never gated, and absent for older runs."""

    def test_physical_stop_rows(self):
        motion = [dict(kind='stop', reason='watchdog_pause', outcome='stopped', started_s=1.0, duration_s=0.14,
                       stop_time_s=0.12, setpoint_stop_time_s=0.08, speed_at_command_rad_s=0.2,
                       max_joint_travel_rad=0.01, camera_travel_mm=6.5, tool_travel_mm=6.0, camera_rotation_deg=0.8,
                       peak_measured_accel_rad_s2=5.0, peak_setpoint_accel_rad_s2=5.0, peak_setpoint_jerk_rad_s3=150.0,
                       peak_measured_jerk_rad_s3=160.0, command_serials=[4])]
        events = [dict(kind='pause', reason='stale_camera', at_s=5.0, deadline_s=5.0, stopped_s=5.001, command_serial=4)]
        summary = t.physical_stops(events, dict(actuator=dict(profile='default'), motion=motion))
        method = dict(physical_stops=summary)
        self.assertEqual(t.physical_stop_text(method, 'pause'), '120 ms / 6.5 mm (1 while moving)')
        self.assertEqual(t.physical_stop_text(method, 'stop'), 'none while moving')

    def test_runs_before_the_actuator_model_are_not_measured(self):
        self.assertIsNone(t.physical_stops([dict(kind='stop', reason='converged', at_s=1.0)], dict(counts={})))
        self.assertEqual(t.physical_stop_text(dict(physical_stops=None), 'stop'), 'not measured')

    def test_actuator_files_are_protected(self):
        for name in ('actuator.py', 'actuator_config.json', 'simulation.py', 'realtime.py'):
            self.assertTrue(t.is_protected(name), name)
        self.assertFalse(t.is_protected('stop_response.py'))  # Analysis only.



class SafetyEnvelope(unittest.TestCase):
    RECORD = dict(steps=100, min_environment_slack_m=0.004, min_environment_pair=('a', 'b'), min_self_slack_m=0.01, min_joint_margin_rad=0.3,
                  min_joint_margin_joint=2, peak_command_rad_s=0.35, peak_speed_rad_s=0.36, peak_speed_joint=1,
                  limits=dict(max_joint_velocity_rad_s=0.6))

    def test_recorded_metrics_are_judged_and_old_runs_are_not(self):
        metrics = t.safety_metrics_of(self.RECORD)
        self.assertEqual([m['passed'] for m in metrics], [True] * 5)
        self.assertIsNone(t.safety_metrics_of(None))

    def test_report_section_and_csv(self):
        bad = t.safety_metrics_of(dict(self.RECORD, min_joint_margin_rad=-0.02))
        good = t.safety_metrics_of(self.RECORD)
        methods = [dict(label='SIFT', safety_metrics=good), dict(label='Learned GPU', safety_metrics=bad)]
        sessions = [dict(id='rep-1', mode='natural', phase='repeatability', safety=self.RECORD, safety_metrics=good),
                    dict(id='old', mode='learned', phase='sustained')]
        with tempfile.TemporaryDirectory() as folder:
            lines = t.safety_section(Path(folder), methods, sessions)
            csv_text = (Path(folder) / 'safety.csv').read_text(encoding='utf-8')
        text = '\n'.join(lines)
        self.assertIn('REQ-12', text)
        self.assertIn('**FAIL**', text)
        self.assertIn('rep-1', csv_text)
        self.assertIn('old', csv_text)

    def test_a_violation_is_a_failed_criterion(self):
        import safety_metrics
        bad = t.safety_metrics_of(dict(self.RECORD, peak_speed_rad_s=0.65))
        self.assertEqual(len(safety_metrics.failures(bad)), 1)
        self.assertIn('REQ-13', safety_metrics.failures(bad)[0])


if __name__ == '__main__':
    unittest.main()
