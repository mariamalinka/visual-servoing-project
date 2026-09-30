"""Acceptance gates must reject failures, missing data and incomplete runs."""
import copy
import unittest
import tempfile
from unittest.mock import patch
from pathlib import Path
from run_acceptance_campaign import evaluate, read, write


class AcceptanceGates(unittest.TestCase):
    def setUp(self):
        self.config = read(Path(__file__).with_name('acceptance_campaign.json'))
        d = dict(count=100, mean=80, p95=90, p99=100, max=120)
        self.result = dict(attempts=80, alignments=80, success_rate=1., freshness_trips=0, control_misses=0,
            unsafe_motion=0, post_stop_motion=0, contacts=0, unlatched_stops=0,
            telemetry_lost=0, runtime_errors=0, worker_restarts=0, duration_s=1800,
            pose_attempts={'0':20,'1':20,'2':20}, capture_ms=d, processing_ms=d,
            position_mm=dict(max=1.), orientation_deg=dict(max=.5),
            by_pose_first_later={str(i):dict(first=d,later=d) for i in range(3)})

    def test_valid_contract(self):
        self.assertEqual(evaluate(self.result,self.config), [])

    def test_each_safety_fault_fails(self):
        for key in ('freshness_trips','control_misses','unsafe_motion','post_stop_motion','contacts',
                    'unlatched_stops','telemetry_lost','runtime_errors','worker_restarts'):
            with self.subTest(key=key):
                r=copy.deepcopy(self.result);r[key]=1
                self.assertIn(key,evaluate(r,self.config))

    def test_missing_latency_fails(self):
        r=copy.deepcopy(self.result);r['capture_ms']=None
        self.assertIn('capture_p99_ms',evaluate(r,self.config))

    def test_incomplete_or_uncovered_fails(self):
        r=copy.deepcopy(self.result);r['duration_s']=1799;r['pose_attempts']['2']=19
        self.assertIn('session duration',evaluate(r,self.config))
        self.assertIn('pose 2 coverage',evaluate(r,self.config))

    def test_later_pose_regression_fails(self):
        r=copy.deepcopy(self.result);r['by_pose_first_later']['1']['later']=dict(p99=151)
        self.assertIn('pose 1 later/first p99',evaluate(r,self.config))

    def test_accuracy_and_latency_fail(self):
        r=copy.deepcopy(self.result);r['position_mm']['max']=2.01;r['processing_ms']['max']=251
        self.assertIn('position_mm',evaluate(r,self.config))
        self.assertIn('processing_max_ms',evaluate(r,self.config))

    def test_failed_alignment_fails(self):
        r=copy.deepcopy(self.result);r['alignments']=79;r['success_rate']=79/80
        self.assertIn('failed alignments',evaluate(r,self.config))

    def test_too_few_alignments_cannot_claim_95_percent(self):
        r=copy.deepcopy(self.result);r['attempts']=r['alignments']=60  # 60/60: lower bound 94.0%.
        self.assertIn('alignment success rate',evaluate(r,self.config))

    def test_campaigns_declared_before_confidence_intervals_keep_their_rule(self):
        legacy=copy.deepcopy(self.config)
        for key in ('required_success_rate','confidence','max_failed_alignments'): legacy['criteria'].pop(key)
        legacy['criteria']['success_rate']=1.0
        r=copy.deepcopy(self.result);r['attempts']=r['alignments']=60
        self.assertEqual(evaluate(r,legacy),[])
        r['success_rate']=.99
        self.assertIn('alignment success rate',evaluate(r,legacy))

    def test_missing_endpoint_and_system_pause_fail(self):
        r=copy.deepcopy(self.result);r['missing_endpoints']=1;r['continuity_gaps']=1
        self.assertIn('missing_endpoints',evaluate(r,self.config))
        self.assertIn('continuity_gaps',evaluate(r,self.config))

    def test_atomic_write_retries_transient_windows_sharing_errors(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'progress.json'; original=Path.replace; calls=[]
            def replace(source, target):
                calls.append(1)
                if len(calls)<3: raise PermissionError('transient reader lock')
                return original(source,target)
            with patch.object(Path,'replace',replace), patch('run_acceptance_campaign.time.sleep'):
                write(path,{'attempts':12})
            self.assertEqual(read(path),{'attempts':12})
            self.assertEqual(len(calls),3)
            self.assertFalse(path.with_suffix('.json.tmp').exists())

    def test_persistent_write_error_remains_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(Path,'replace',side_effect=PermissionError('persistent')) as replace, patch('run_acceptance_campaign.time.sleep'):
                with self.assertRaises(PermissionError):write(Path(directory)/'progress.json',{})
            self.assertEqual(replace.call_count,40)


if __name__=='__main__':unittest.main()
