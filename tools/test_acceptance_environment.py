"""GPU throttle decoding and failure correlation for the acceptance test telemetry.

    python -B -m unittest discover -s tools -p test_acceptance_environment.py -v
"""
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

import acceptance_environment as env

# Format written by nvidia-smi --format=csv,nounits (local timestamps), including the
# pstate read error seen on this laptop.
GPU_CSV = """timestamp, index, pstate, clocks.current.sm [MHz], clocks.current.memory [MHz], utilization.gpu [%], temperature.gpu, power.draw [W], power.limit [W], enforced.power.limit [W], memory.used [MiB], clocks_event_reasons.active
2026/09/28 02:50:40.000, 0, P3, 772, 5000, 40, 58, 16.01, 60.00, 60.00, 418, 0x0000000000000001
2026/09/28 02:50:41.000, 0, P3, 780, 5000, 29, 58, 15.46, 60.00, 60.00, 418, 0x0000000000000000
2026/09/28 02:50:42.000, 0, P3, 243, 5000, 77, 58, 13.77, 60.00, 60.00, 418, 0x0000000000000024
2026/09/28 02:50:43.000, 0, P3, 247, 5000, 81, 58, 14.38, 60.00, 60.00, 418, 0x0000000000000024
2026/09/28 02:50:44.000, 0, P3, 99, 5000, 15, 57, 14.31, 60.00, 60.00, 418, 0x0000000000000001
2026/09/28 02:50:45.000, 0, P3, 52, 5000, 94, 57, 14.55, 60.00, 60.00, 418, 0x0000000000000024
2026/09/28 02:50:50.000, 0, [Unknown Error], 990, 5500, 0, 56, 11.23, 60.00, 60.00, 0, 0x0000000000000001
"""
OFFSET = 7200  # Europe/Warsaw summer time
T0 = datetime(2026, 9, 28, 0, 50, 40, tzinfo=timezone.utc).timestamp()
SYSTEM_CSV = env.SYSTEM_HEADER + """
1790556640000,zone,\\_TZ.THRM,331,3311,100,0
1790556640000,cpu,_Total,2496,98,110,40
1790556642000,zone,\\_TZ.THRM,358,3584,72,1
1790556642000,cpu,_Total,1197,47,48,55
garbage line
"""


class Decoding(unittest.TestCase):
    def test_reason_bits(self):
        self.assertEqual(env.decode(0x24), ['sw_power_cap', 'sw_thermal_slowdown'])
        self.assertEqual(env.decode(0x1), ['gpu_idle'])
        self.assertEqual(env.decode(0xC8), ['hw_slowdown', 'hw_thermal_slowdown', 'hw_power_brake'])
        self.assertEqual(env.decode(0), [])

    def test_gpu_csv_on_utc_clock(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'gpu.csv'; path.write_text(GPU_CSV, encoding='utf-8')
            samples, offset = env.read_gpu(path, OFFSET)
        self.assertEqual((len(samples), offset), (7, OFFSET))
        self.assertAlmostEqual(samples[0]['t'], T0)
        self.assertEqual(samples[2]['reasons'], ['sw_power_cap', 'sw_thermal_slowdown'])
        self.assertEqual(samples[2]['sm_mhz'], 243)
        self.assertEqual(samples[-1]['pstate'], '[Unknown Error]')

    def test_offset_inferred_for_older_runs(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'gpu.csv'; path.write_text(GPU_CSV, encoding='utf-8')
            _, offset = env.read_gpu(path, None, '2026-09-28T00:50:37.000000+00:00')
        self.assertEqual(offset, OFFSET)

    def test_system_csv(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'system.csv'; path.write_text(SYSTEM_CSV, encoding='utf-8')
            zones, cpu = env.read_system(path)
        self.assertEqual((len(zones), len(cpu)), (2, 2))
        self.assertAlmostEqual(zones[1]['temp_c'], 85.25, places=2)
        self.assertEqual(zones[1]['passive_limit_pct'], 72)
        self.assertEqual(cpu[1]['pct_max_frequency'], 47)


class Correlation(unittest.TestCase):
    def setUp(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'gpu.csv'; path.write_text(GPU_CSV, encoding='utf-8')
            self.gpu, _ = env.read_gpu(path, OFFSET)
        # Controller clock: perf 100.0 s == T0; worker ready at T0.
        self.ending = dict(clock_anchor=dict(perf_s=100.0, utc_epoch_s=T0),
                           started_utc=datetime.fromtimestamp(T0 - 5, timezone.utc).isoformat(),
                           elapsed_s=15.0, duration_s=10.0)
        self.events = [dict(kind='align', at_s=100.0, generation=1), dict(kind='stop', reason='converged', at_s=101.0, generation=1),
                       dict(kind='align', at_s=101.5, generation=2), dict(kind='stop', reason='stale_camera', at_s=102.6, generation=2),
                       dict(kind='align', at_s=103.0, generation=3), dict(kind='stop', reason='stale_camera', at_s=105.2, generation=3)]
        self.attempts = [dict(generation=1, outcome='converged'), dict(generation=2, outcome='stale_camera'),
                         dict(generation=3, outcome='stale_camera')]
        self.frames = [dict(rendered_s=100.5, finished_s=100.56), dict(rendered_s=102.1, finished_s=102.40),
                       dict(rendered_s=105.0, finished_s=105.30)]

    def test_failures_and_slow_frames_matched_to_clock_limit(self):
        e = env.session_environment(self.frames, self.events, self.attempts, self.ending, self.gpu, [], [])
        self.assertEqual((e['failed_alignments'], e['failed_during_throttle']), (2, 2))
        self.assertEqual((e['slow_frames'], e['slow_frames_during_throttle']), (2, 2))
        self.assertEqual(e['limiting_reason_seconds']['sw_thermal_slowdown'], 3)
        self.assertEqual(e['gpu_busy_sm_mhz']['min'], 52)

    def test_short_gaps_join_one_episode_timed_from_ready(self):
        e = env.session_environment(self.frames, self.events, self.attempts, self.ending, self.gpu, [], [])
        self.assertEqual(len(e['throttle_episodes']), 1)
        episode = e['throttle_episodes'][0]
        self.assertEqual((episode['start_s'], episode['min_sm_mhz']), (2.0, 52))

    def test_no_clock_limit_means_no_attribution(self):
        quiet = [dict(s, reasons=['gpu_idle']) for s in self.gpu]
        e = env.session_environment(self.frames, self.events, self.attempts, self.ending, quiet, [], [])
        self.assertEqual(e['failed_during_throttle'], 0)
        self.assertEqual(e['throttle_episodes'], [])

    def test_older_runs_use_ready_time_as_anchor(self):
        ending = dict(self.ending); ending.pop('clock_anchor')
        e = env.session_environment(self.frames, self.events, self.attempts, ending, self.gpu, [], [])
        self.assertEqual(e['failed_during_throttle'], 2)

    def test_missing_telemetry_is_reported_not_guessed(self):
        e = env.session_environment(self.frames, self.events, self.attempts, self.ending, [], [], [])
        self.assertEqual((e['failed_during_throttle'], e['failed_without_gpu_sample']), (0, 2))
        self.assertIsNone(e['gpu_busy_sm_mhz'])


class PowerModes(unittest.TestCase):
    def test_guid_names_match_windows_settings(self):
        self.assertEqual(env.power_mode_name(dict(effective_overlay='ded574b5-45a0-4f42-8737-46345c09c238')), 'Best performance')
        self.assertEqual(env.power_mode_name(dict(effective_overlay='961cc777-2547-4f9d-8174-7d86181b8a7a')), 'Best power efficiency')
        self.assertEqual(env.power_mode_name(dict(effective_overlay='00000000-0000-0000-0000-000000000000')), 'Balanced')

    def test_guid_overrides_a_wrong_stored_label(self):
        record = dict(effective_overlay='ded574b5-45a0-4f42-8737-46345c09c238', mode='Best power efficiency')
        self.assertEqual(env.power_mode_name(record), 'Best performance')


class LeftoversAndLowStates(unittest.TestCase):
    def test_only_our_samplers_are_recognised(self):
        import base64
        ours = env._powershell(env.SYSTEM_SCRIPT)[-1]
        legacy = base64.b64encode(env.LEGACY_SCRIPT_PREFIX.encode('utf-16-le')).decode() + 'AAAA'
        self.assertTrue(env.is_our_sampler('powershell.exe', 'powershell.exe -EncodedCommand ' + ours))
        self.assertTrue(env.is_our_sampler('powershell.exe', 'powershell.exe -EncodedCommand ' + legacy))
        self.assertFalse(env.is_our_sampler('powershell.exe', 'powershell.exe -NoProfile'))
        self.assertFalse(env.is_our_sampler('powershell.exe', None))
        self.assertTrue(env.is_our_sampler('nvidia-smi.exe', 'nvidia-smi --query-gpu=' + env.GPU_FIELDS + ' --loop-ms=1000'))
        self.assertFalse(env.is_our_sampler('nvidia-smi.exe', 'nvidia-smi -q'))

    def test_low_state_windows(self):
        gpu = [dict(t=float(t), util=60, sm_mhz=247 if t >= 60 else 765) for t in range(0, 120)]
        cpu = [dict(t=float(t), pct_max_frequency=75 if t >= 60 else 99) for t in range(0, 120)]
        episodes = env.low_state_episodes(gpu, cpu, 0, 120, origin=0)
        self.assertEqual(episodes, [dict(start_s=60, duration_s=60, state=['cpu_low_frequency', 'gpu_low_clock'])])


if __name__ == '__main__':
    unittest.main()
