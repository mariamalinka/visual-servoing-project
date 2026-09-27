"""Compare completed, identically configured worker-lifetime campaigns."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def triple(values):
    values = np.asarray(list(values), dtype=float)
    return dict(zip(('p95', 'p99', 'max'), map(float, np.percentile(values, [95, 99, 100]))))


def summarize(directory):
    completion = read(directory / 'completion.json')
    assert completion['source_unchanged'] and completion['all_safety_audits_passed']
    comparison = read(directory / 'comparison.json')
    with gzip.open(directory / 'traces/frame-gpu-correlation.json.gz', 'rt', encoding='utf-8') as stream:
        frames = [f for f in json.load(stream) if f['arm'] == 'reused']
    sessions = [s for s in read(directory / 'sessions.json') if s['arm'] == 'reused']
    assert len(sessions) == 8
    first = comparison['cohorts']['reused_first']
    later = comparison['cohorts']['reused_later']
    attempts = first['attempts'] + later['attempts']
    successes = first['alignments'] + later['alignments']
    slow = later['slow_gpu']['in_processing']
    return {
        'sessions': len(sessions), 'attempts': attempts, 'alignments': successes,
        'success_rate_percent': 100 * successes / attempts,
        'freshness_trips': first['freshness_trips'] + later['freshness_trips'],
        'processing_ms': triple(f['processing_ms'] for f in frames),
        'first_alignment_processing_ms': first['processing_ms'],
        'later_alignment_processing_ms': later['processing_ms'],
        'first_alignment_successes': first['alignments'], 'later_alignment_successes': later['alignments'],
        'slow_frame_gpu': slow,
        'per_session': [{'id': s['id'], 'alignments': s['alignments'], 'attempts': s['attempts'],
                         'freshness_trips': s['freshness_watchdog_trips']} for s in sessions],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('after', type=Path)
    args = parser.parse_args()
    before_manifest = read(args.baseline / 'manifest.json')
    after_manifest = read(args.after / 'manifest.json')
    for key in ('fingerprint', 'plan', 'tool_sha256', 'gpu_query_fields'):
        assert before_manifest[key] == after_manifest[key], f'Changed experiment input: {key}'
    result = {'baseline': summarize(args.baseline), 'after': summarize(args.after),
              'identical_code_dependencies_plan_runner_and_telemetry': True,
              'definition': 'Uncached active-frame processing, retaining failed and late frames; first/later refer to alignment position within each reused worker. Slow threshold >=200 ms, GPU samples strictly within those processing intervals.',
              'interpretation': 'Material aggregate improvement, but sustained stability is not established: the final reused session degraded after seven successful sessions. The low-clock regime remains reproducible. Settings are user-reported; this comparison does not isolate an individual power/driver setting as the cause.'}
    (args.after / 'POWER_COMPARISON.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    def latency(data):
        return ' / '.join(f'{data[k]:.1f}' for k in ('p95', 'p99', 'max'))
    b, a = result['baseline'], result['after']
    rows = [
        ('Alignment success', f"{b['alignments']}/{b['attempts']} ({b['success_rate_percent']:.1f}%)", f"{a['alignments']}/{a['attempts']} ({a['success_rate_percent']:.1f}%)"),
        ('Freshness trips', str(b['freshness_trips']), str(a['freshness_trips'])),
        ('Processing p95 / p99 / max (ms)', latency(b['processing_ms']), latency(a['processing_ms'])),
        ('First alignment processing p95 / p99 / max (ms)', latency(b['first_alignment_processing_ms']), latency(a['first_alignment_processing_ms'])),
        ('Later alignment processing p95 / p99 / max (ms)', latency(b['later_alignment_processing_ms']), latency(a['later_alignment_processing_ms'])),
        ('Slow-frame median SM clock (MHz)', str(b['slow_frame_gpu']['clocks.sm']['p50']), str(a['slow_frame_gpu']['clocks.sm']['p50'])),
    ]
    lines = ['# GPU/power-settings rerun', '',
             'Same eight 120-second reused-worker sessions and 32 interleaved fresh-worker controls. Code, dependencies, runner, warm-up, scene/model, 50 ms transport, 400 ms freshness and 50 ms control deadline match the preserved baseline. Statistics below concern reused workers only.', '',
             '| Metric | Previous baseline | After settings change |', '|---|---:|---:|']
    lines += [f'| {label} | {before} | {after} |' for label, before, after in rows]
    lines += ['', 'Material improvement, but not a stable fix. The first seven reused sessions passed 154/154 alignments with no freshness trips; the last passed 2/40 with 38 trips. Later-alignment maximum latency and slow-frame GPU clocks remained near baseline levels.', '', result['definition'], '',
              'The slow-frame clock comparison uses 430 baseline samples and 52 rerun samples. No slow frame occurred during first alignments. The final two fresh-worker controls also hit freshness stops; the degraded period therefore was not confined to a single reused worker. Individual settings were not isolated experimentally. Different attempt counts result from the same fixed session durations: failed alignments stop sooner.', '',
              'The complete audit and distributions are in POWER_COMPARISON.json and comparison.json; raw per-frame and GPU correlation data remain local under traces.']
    report = '\n'.join(lines) + '\n'
    (args.after / 'POWER_COMPARISON.md').write_text(report, encoding='utf-8')
    print(report)


if __name__ == '__main__':
    main()
