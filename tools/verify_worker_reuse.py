"""Verify saved worker-reuse evidence after collection and analysis have ended."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
from run_camera_robustness import fingerprint
from compare_worker_reuse import extra


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path)
    p.add_argument('--tests',type=Path,required=True)
    args=p.parse_args()
    manifest=read(args.directory/'manifest.json')
    completion=read(args.directory/'completion.json')
    audit=read(args.directory/'comparison-audit.json')
    summary=read(args.directory/'summary.json')
    sessions=read(args.directory/'sessions.json')
    assert fingerprint()==manifest['fingerprint'], 'Source/runtime changed after campaign'
    assert completion['source_unchanged'] and completion['telemetry_complete']
    assert len(sessions)==completion['sessions']
    assert audit['plan_identical'] and audit['runtime_versions_identical']
    current=extra(args.directory)
    assert current==audit['after'], 'Trace or audit changed'
    native=read(args.directory.parent/'20260923-native-diagnostic/metadata.json')
    tests=args.tests.read_text(encoding='utf-8-sig')
    match=re.search(r'Ran (\d+) tests in ([\d.]+)s\s+OK\s*$',tests)
    assert match, 'Test log does not end in success'
    metrics=summary['methods']
    for row in metrics.values():
        assert row['all_stops_stayed_latched']
        assert row['counters']['unsafe_motion_ticks']==0
        assert row['counters']['post_stop_motion_ticks']==0
        assert row['counters']['contacts']==0
    learned_settings={key:value for key,value in current['execution'].items() if key.startswith('learned-')}
    assert learned_settings and all(v['cuda_graph_blocks']==9 for v in learned_settings.values())
    assert all(v['image_conversion']=='upload_uint8_then_float32' for v in learned_settings.values())
    boundaries=[row[point] for row in current['power_boundaries'] for point in ('before','after')]
    ac_observed=all(row is not None and row['ACLineStatus']==1 for row in boundaries)
    timing_passed=all(row['freshness_watchdog_trips']==0 and row['counters']['control_deadline_misses']==0 for row in metrics.values())
    tool_names=('analyze_latency_stress.py','analyze_gpu_latency.py','compare_worker_reuse.py',
                'summarize_worker_probes.py','verify_worker_reuse.py','probe_worker_reuse.py','analyze_replay_clocks.py')
    record=dict(regression_tests=int(match[1]),regression_duration_s=float(match[2]),
        regression_test_log_sha256=hashlib.sha256(args.tests.read_bytes()).hexdigest(),
        sessions=len(sessions),raw_trace_checksums_verified=len(sessions),
        baseline_raw_trace_checksums_verified=len(audit['before']['execution']),
        all_capture_accounting_reconciled=True,all_stops_stayed_latched=True,
        unsafe_motion_ticks=0,post_stop_motion_ticks=0,contacts=0,
        source_and_runtime_unchanged_after_collection=True,
        fixed_inputs_and_campaign_plan_identical=True,baseline_manifest_sha256=audit['baseline_manifest_sha256'],
        timing_requirement_passed=timing_passed,sample_targets_met=completion['sample_targets_met'],
        graph_initialization_verified=True,graph_calls=current['graph_calls'],
        ac_at_all_observed_boundaries=ac_observed,power_boundary_observations=len(boundaries),
        power_observation_limit='Boundary samples only; not continuous AC monitoring.',
        full_kernel_scheduling_attribution_available=False,wpr_start=native['wpr_start'],
        analysis_sha256={name:hashlib.sha256((ROOT/'tools'/name).read_bytes()).hexdigest() for name in tool_names})
    (args.directory/'verification.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(record,indent=2))


if __name__=='__main__':
    main()
