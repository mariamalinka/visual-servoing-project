"""Latency evidence must retain failed/late frames and reset leases per attempt."""
import unittest
import numpy as np
from latency_stress import distribution,feedback_age_bound,late_results,active_frames
from deadline_diagnostics import CycleProbe,thread_cycles
from run_latency_stress import validate_fixed_inputs


class StressEvidenceTests(unittest.TestCase):
    def test_execution_change_does_not_relax_fixed_model_or_controller_inputs(self):
        before=dict(fingerprint=dict(runtime={'torch':'pinned'},sha256={
            'learned_perception.py':'old','control.py':'fixed','learned_feature_config.json':'fixed'}))
        current=dict(runtime={'torch':'pinned'},sha256=dict(before['fingerprint']['sha256'],**{'learned_perception.py':'new'}))
        validate_fixed_inputs(before,current)
        for name in ('control.py','learned_feature_config.json'):
            changed=dict(current,sha256=dict(current['sha256'],**{name:'changed'}))
            with self.assertRaises(ValueError):validate_fixed_inputs(before,changed)
        with self.assertRaises(ValueError):validate_fixed_inputs(before,dict(current,runtime={'torch':'different'}))

    def test_high_percentiles_and_nonfinite_rejection(self):
        s=distribution(range(10000))
        self.assertEqual(s['count'],10000)
        self.assertAlmostEqual(s['p99_9'],9989.001)
        self.assertEqual(s['max'],9999)
        self.assertIsNone(distribution([]))
        with self.assertRaises(ValueError):distribution([float('nan')])

    def test_feedback_bound_resets_after_explicit_new_alignment(self):
        row=dict(events=[dict(kind='align',generation=1,at_s=10.),
            dict(kind='stop',generation=1,stopped_s=10.4),
            dict(kind='align',generation=2,at_s=20.),
            dict(kind='stop',generation=2,stopped_s=20.2)],
            frames=[dict(generation=1,captured_s=10.01,applied_s=10.1),
                    dict(generation=2,captured_s=20.01,applied_s=20.1)])
        self.assertAlmostEqual(feedback_age_bound(row),390.)

    def test_failed_late_processing_is_retained_and_idle_cache_separated(self):
        f=dict(sequence=1,generation=1,capture_active=True,captured_s=10.1,
            render_started_s=10.11,rendered_s=10.12,finished_s=10.6,received_s=10.61,
            stage_ms={},cpu_ms={})
        row=dict(events=[dict(kind='stop',generation=1,reason='stale_camera',stopped_s=10.4)],
            sensor_frames=[f,dict(f,sequence=2,capture_active=False),dict(f,sequence=3,stage_ms={'cache_hit':True})])
        self.assertEqual(len(active_frames(row)),2)
        self.assertEqual(len(active_frames(row,True)),1)
        self.assertEqual(len(late_results(row)),2)
        self.assertEqual(late_results(row)[0]['stop_reason'],'stale_camera')

    def test_bounded_spike_evidence_keeps_stage_and_counts_eviction(self):
        probe=CycleProbe(threshold_ms=0,capacity=2)
        for _ in range(3):
            probe.start(True,7);probe.mark('command_application');probe.finish()
        self.assertEqual(len(probe.rows),2)
        self.assertEqual(probe.dropped,1)
        self.assertEqual(probe.rows[-1]['stages'][0]['stage'],'command_application')
        self.assertEqual(probe.rows[-1]['generation'],7)
        if thread_cycles() is not None:
            self.assertGreaterEqual(probe.rows[-1]['cpu_cycles'],0)


if __name__=='__main__':unittest.main()
