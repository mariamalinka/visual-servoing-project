"""Regression checks for the published stress-analysis statistics and attribution."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from analyze_latency_stress import bootstrap_sessions, diagnose_stop


class ClusterStatisticsTests(unittest.TestCase):
    def test_bootstrap_preserves_session_clusters_and_is_reproducible(self):
        # Two internally identical but mutually different sessions. IID-frame
        # resampling would spuriously narrow this interval around 50 ms.
        groups=[[0.]*100,[100.]*100]
        first=bootstrap_sessions(groups,1000,17)
        self.assertEqual(first,bootstrap_sessions(groups,1000,17))
        self.assertEqual(first['mean'],[0.,100.])
        self.assertNotIn('max',first)

    def test_single_cluster_preserves_pooled_quantiles(self):
        values=list(range(101))
        got=bootstrap_sessions([values],10,19)
        self.assertEqual(got['mean'],[50.,50.])
        np.testing.assert_allclose(got['p99_9'],[99.9,99.9])
        self.assertIsNone(bootstrap_sessions([[]],10,19))

    def test_control_diagnosis_uses_same_generation_and_overlapping_gc(self):
        stage=dict(stage='command_application',started_s=10.005,finished_s=10.06,
            wall_ms=55.,thread_cpu_ms=0.,cpu_cycles=1234)
        cycle=dict(generation=2,started_s=10.,finished_s=10.06,wall_ms=60.,stages=[stage])
        wrong=dict(cycle,generation=1,wall_ms=99.)
        gc=dict(started_s=10.02,finished_s=10.04,thread_id=5)
        stop=dict(kind='stop',generation=2,reason='control_overrun',stopped_s=10.0601)
        row=dict(id='fixture',cycle_spikes=[wrong,cycle],sensor_frames=[],frames=[],gc_events=[gc])
        found=diagnose_stop(row,stop)
        self.assertEqual(found['cycle'],cycle)
        self.assertEqual(found['dominant_stage']['stage'],'command_application')
        self.assertEqual(found['overlapping_parent_gc'],[gc])

    def test_freshness_stop_does_not_blame_a_cycle_starting_after_stop(self):
        # A stop can be observed just before the next probe starts. That later
        # cycle cannot explain it, even if its timestamp is within 2 ms.
        cycle=dict(generation=2,started_s=10.001,finished_s=10.06,wall_ms=59.,stages=[])
        stop=dict(kind='stop',generation=2,reason='stale_camera',stopped_s=10.)
        row=dict(id='fixture',cycle_spikes=[cycle],sensor_frames=[],frames=[],gc_events=[])
        found=diagnose_stop(row,stop)
        self.assertIsNone(found['cycle'])
        self.assertIsNone(found['dominant_stage'])


if __name__=='__main__':unittest.main()
