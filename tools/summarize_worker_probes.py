"""Summarize separate worker-reuse diagnostics, never during a timing campaign."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'outputs/visual-servoing-simulation'))
from latency_stress import distribution


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def profile(path):
    events = json.loads(path.read_text(encoding='utf-8'))['traceEvents']
    runtime = [e for e in events if e.get('cat') == 'cuda_runtime' and 'dur' in e]
    kernels = [e for e in events if e.get('cat') == 'kernel' and 'dur' in e]
    names = Counter(e['name'] for e in runtime)
    return dict(sha256=sha(path), runtime={name: dict(count=count,
        total_ms=sum(e['dur'] for e in runtime if e['name'] == name)/1000)
        for name, count in sorted(names.items())}, kernel_count=len(kernels),
        kernel_duration_sum_ms=sum(e['dur'] for e in kernels)/1000,
        kernel_span_ms=(max(e['ts']+e['dur'] for e in kernels)-min(e['ts'] for e in kernels))/1000 if kernels else None,
        interpretation='Separate instrumented frames; durations may include GPU preemption. The kernel sum, stream span and host API waits are not additive.')


def summarize(directory):
    path=directory/'replay.json'
    data=json.loads(path.read_text(encoding='utf-8'))
    rows=[r for r in data['rows'] if not r['profiled']]
    memory={key: dict(min=min(r['memory'][key] for r in rows),
                     max=max(r['memory'][key] for r in rows))
            for key in rows[0]['memory']}
    phases={}
    for index in sorted({r.get('phase_index',0) for r in rows}):
        selected=[r for r in rows if r.get('phase_index',0)==index]
        phases[str(index)]=dict(phase=selected[0].get('phase','baseline'),
            blocktime_ms=sorted({r['blocktime_ms'] for r in selected if 'blocktime_ms' in r}),
            **{key:distribution(r[key] for r in selected)
                for key in ('processing_ms','process_cpu_ms','thread_cpu_ms','gap_ms')})
    return dict(raw_sha256=sha(path), excluded_from_campaign=data['excluded_from_campaign'],
        fingerprint=data['fingerprint'],tool_sha256=data.get('tool_sha256'),environment=data['environment'],
        options={key:data.get(key) for key in ('alternate_stream','device_cast','benchmark',
            'channels_last','blas_lt','graphs','prerender','ablation')},
        before=data['before'],after=data['after'],cohorts=data['cohorts'],
        memory_over_unprofiled_frames=memory,phases=phases,
        profiles={p.name:profile(p) for p in sorted(directory.glob('profile-*.json'))})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('latency_directory',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    result={p.name:summarize(p) for p in sorted(args.latency_directory.glob('20260923-replay-*'))
            if (p/'replay.json').exists()}
    args.output.write_text(json.dumps(dict(probes=result,
        interpretation='Separate diagnostics, excluded from sustained before/after statistics. Variants ran sequentially under changing host/GPU conditions; comparisons are observational, not randomized treatment effects.'),
        indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(f'Summarized {len(result)} diagnostic variants: {args.output}')


if __name__=='__main__':
    main()
