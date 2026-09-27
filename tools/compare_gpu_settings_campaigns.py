"""Compare completed, matched GPU-settings campaigns without changing runtime code."""
import argparse
import json
from pathlib import Path

from compare_worker_power_settings import read, summarize


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('campaigns', type=Path, nargs='+')
    args = p.parse_args()
    assert len(args.campaigns) >= 2
    reference = read(args.campaigns[0] / 'manifest.json')
    results = []
    for directory in args.campaigns:
        manifest = read(directory / 'manifest.json')
        for key in ('fingerprint', 'plan', 'tool_sha256', 'gpu_query_fields'):
            assert manifest[key] == reference[key], f'Changed {key}: {directory}'
        data = summarize(directory)
        assert data['first_alignment_processing_ms']['max'] < 200, 'Slow first frames must be included in GPU sample aggregation'
        sessions = read(directory / 'sessions.json')
        power = [s[k].get('power') for s in sessions for k in ('boundary_before', 'boundary_after')]
        data.update(directory=directory.name, created_utc=manifest['created_utc'],
                    ac_at_all_session_boundaries=all(x and x['ACLineStatus'] == 1 for x in power))
        results.append(data)
    output = args.campaigns[-1]
    definition = 'Uncached active-frame processing; first/later means alignment position within each reused worker. Slow means >=200 ms; GPU clocks use samples strictly inside those processing intervals. All runs use eight 120-second reused sessions and 32 interleaved fresh-worker controls.'
    (output / 'SETTINGS_COMPARISON.json').write_text(json.dumps({
        'matched_inputs': True, 'definition': definition, 'campaigns': results}, indent=2) + '\n', encoding='utf-8')
    def latency(d):
        return ' / '.join(f'{d[k]:.1f}' for k in ('p95', 'p99', 'max'))
    def clocks(d):
        c = d['slow_frame_gpu']['clocks.sm']
        return f"{c['p50']:.0f} MHz" if c else 'No frames >=200 ms'
    metrics = [
        ('Alignment success', lambda d: f"{d['alignments']}/{d['attempts']} ({d['success_rate_percent']:.1f}%)"),
        ('Freshness trips', lambda d: str(d['freshness_trips'])),
        ('Processing p95 / p99 / max (ms)', lambda d: latency(d['processing_ms'])),
        ('First alignment processing p95 / p99 / max (ms)', lambda d: latency(d['first_alignment_processing_ms'])),
        ('Later alignment processing p95 / p99 / max (ms)', lambda d: latency(d['later_alignment_processing_ms'])),
        ('Median SM clock during slow frames', clocks),
    ]
    lines = ['# GPU-settings campaign comparison', '', definition, '',
             '| Metric | ' + ' | '.join(d['directory'] for d in results) + ' |',
             '|---|' + '---:|' * len(results)]
    for name, formatter in metrics:
        lines.append('| ' + name + ' | ' + ' | '.join(formatter(d) for d in results) + ' |')
    lines += ['', 'Code, dependency fingerprints, runner hash, test plan and telemetry queries are identical. The 400 ms freshness limit, 50 ms control deadline and 50 ms transport delay are unchanged. Raw traces remain local under ignored traces directories.', '',
              'Different attempt counts reflect fixed session durations; failed alignments can stop sooner. The campaigns were sequential observations, not a randomized isolation of each driver or power setting. A passing campaign does not prove the absence of rare future failures.']
    report = '\n'.join(lines) + '\n'
    (output / 'SETTINGS_COMPARISON.md').write_text(report, encoding='utf-8')
    print(report)


if __name__ == '__main__':
    main()
