"""Analyze complete sustained sessions, including failed and late sensor work.

Run only after the timing campaign, so bootstrap/plotting load cannot perturb it.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'outputs/visual-servoing-simulation'
sys.path.insert(0, str(APP))
from latency_stress import active_frames, distribution, feedback_age_bound, late_results
from binomial_ci import format_interval, interpretation

NAMES = {'natural': 'SIFT', 'learned': 'Learned GPU'}
METRICS = ('mean', 'p95', 'p99', 'p99_9')


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def compact_frame(frame, session_id):
    return dict(session=session_id, **{k: v for k, v in frame.items()
        if k not in ('qpos', 'rgb', 'observation', 'worker_gc')})


def overlap(rows, start, end):
    return [r for r in rows if r['started_s'] < end and r['finished_s'] > start]


def session_duration(row):
    active_s = 0.
    for event in row['events']:
        if event['kind'] != 'align':
            continue
        stop = next((e for e in row['events'] if e['kind'] == 'stop'
                     and e['generation'] == event['generation']), None)
        if stop:
            active_s += stop['stopped_s'] - event['at_s']
    return active_s


def bootstrap_sessions(groups, repeats, seed):
    """Whole-session percentile bootstrap; no IID-frame assumption.

    Point estimates pool frames. Every replicate pools all samples from randomly
    drawn sessions, including repeated sessions, preserving within-session order
    and dependence. A CI for the observed maximum is deliberately not generated.
    """
    groups = [np.asarray(g, dtype=float) for g in groups]
    if not groups or not any(len(g) for g in groups) or repeats < 1:
        return None
    rng = np.random.default_rng(seed)
    estimates = []
    for _ in range(repeats):
        selected = rng.integers(0, len(groups), len(groups))
        values = np.concatenate([groups[i] for i in selected])
        if not len(values):
            continue
        estimates.append([values.mean(), *np.percentile(values, [95, 99, 99.9])])
    if not estimates:
        return None
    bounds = np.percentile(estimates, [2.5, 97.5], axis=0)
    return dict(zip(METRICS, ([float(lo), float(hi)] for lo, hi in zip(*bounds))))


def diagnose_stop(row, stop):
    at = stop['stopped_s']
    nearby = [c for c in row['cycle_spikes']
              if c['generation'] == stop['generation']
              and c['started_s'] < at and c['finished_s'] >= at - .12]
    cycle = max(nearby, key=lambda c: 1000*(min(at,c['finished_s'])-c['started_s']), default=None)
    stages=[] if cycle is None else [dict(s,pre_stop_wall_ms=1000*(min(at,s['finished_s'])-s['started_s']))
        for s in cycle['stages'] if s['started_s']<at]
    dominant = max(stages, key=lambda s: s['pre_stop_wall_ms']) if stages else None
    started = at - .12 if cycle is None else cycle['started_s']
    ended = at if cycle is None else cycle['finished_s']
    sensor = [f for f in row['sensor_frames'] if f['capture_active']
              and f['generation'] == stop['generation']
              and f['render_started_s'] < ended and f['finished_s'] > started]
    commands = [f for f in row['frames'] if f['generation'] == stop['generation']
                and abs(f['applied_s'] - at) < .2]
    parent_gc = overlap(row['gc_events'], started, ended)
    unique_gc = {(g['thread_id'], g['started_s']): g for f in sensor
                 for g in f.get('worker_gc', [])}
    worker_gc = overlap(unique_gc.values(), started, ended)
    return dict(session=row['id'], stop=stop, cycle=cycle, dominant_stage=dominant,
                overlapping_parent_gc=parent_gc, overlapping_worker_gc=worker_gc,
                concurrent_sensor_frames=[compact_frame(f, row['id']) for f in sensor],
                nearby_commands=commands)


def stress_safety(sessions):
    """Worst safety envelope over a method's sessions; None if the runs predate the record."""
    import safety_metrics
    merged = safety_metrics.merge([s.get('safety') for s in sessions])
    return None if merged is None else safety_metrics.evaluate(merged)


def aggregate(directory, repeats=2000, seed=20260922):
    manifest = json.loads((directory / 'manifest.json').read_text(encoding='utf-8'))
    completion = json.loads((directory / 'completion.json').read_text(encoding='utf-8'))
    summaries = json.loads((directory / 'sessions.json').read_text(encoding='utf-8'))
    assert len({r['id'] for r in summaries}) == len(summaries) == completion['sessions']
    buckets = {mode: dict(summaries=[], groups=defaultdict(list), stages=defaultdict(list),
        counters=Counter(), attempts=[], active_s=0., ready_s=0., elapsed_s=0.,
        outliers=[], command_outliers=[], diagnoses=[], late=[], cohorts=defaultdict(list),
        session_rows=[], sensor_count=0, cached=0, max_age=0., gc=[], spikes=[],
        backend=set(), telemetry_complete=True, diagnostics_complete=True,
        cycle_evictions=0, gc_evictions=0, unsafe_stops=[], errors=[])
        for mode in manifest['plan']['modes']}
    for summary in summaries:
        path = directory / summary['trace']
        assert path.resolve().is_relative_to(directory.resolve()), path
        raw = path.read_bytes()
        assert hashlib.sha256(raw).hexdigest() == summary['trace_sha256'], path
        row = json.loads(gzip.decompress(raw))
        assert row['id'] == summary['id'] and row['mode'] == summary['mode']
        assert row['config'] == manifest['plan']['config']
        assert row['offset_degrees'] == manifest['plan']['offset_degrees']
        counts=summary['counters']
        assert counts['capture_slots']==counts['captures']+counts['busy_capture_skips']
        assert counts['captures']==sum(counts[k] for k in ('sensor_received','worker_requests_expired',
            'input_dropped','result_mailbox_dropped','completed_but_unreceived','requests_not_started')), row['id']
        bucket = buckets[row['mode']]
        bucket['summaries'].append(summary)
        bucket['backend'].add(row['backend'])
        bucket['counters'].update(summary['counters'])
        bucket['active_s'] += session_duration(row)
        bucket['ready_s'] += row['ready_elapsed_s'] or 0.
        bucket['elapsed_s'] += row['elapsed_s']
        bucket['telemetry_complete'] &= summary['telemetry_complete']
        bucket['cycle_evictions'] += row['cycle_spikes_evicted']
        bucket['gc_evictions'] += row['gc_events_evicted']
        bucket['diagnostics_complete'] &= not (row['cycle_spikes_evicted'] or row['gc_events_evicted'])
        if row['error'] or row['initialization_error']:
            bucket['errors'].append(dict(session=row['id'], error=row['error'], orchestration_error=row['initialization_error']))
        for attempt in row['attempts']:
            bucket['attempts'].append(dict(session=row['id'], **attempt))
            if not attempt['stop_stayed_latched']:
                bucket['unsafe_stops'].append(dict(session=row['id'], **attempt))
        active = active_frames(row)
        uncached = active_frames(row, uncached=True)
        processing = [1000 * (f['finished_s'] - f['rendered_s']) for f in active]
        uncached_processing = [1000 * (f['finished_s'] - f['rendered_s']) for f in uncached]
        commands = [f['capture_to_command_ms'] for f in row['frames']]
        delivery = [1000 * (max(f['received_s'], f['finished_s'] + row['config']['transport_s'])
                    - f['captured_s']) for f in active]
        for key, values in [('processing_ms', processing), ('uncached_processing_ms', uncached_processing),
                            ('capture_to_command_ms', commands), ('all_active_capture_to_delivery_ms', delivery)]:
            bucket['groups'][key].append(values)
        bucket['sensor_count'] += len(row['sensor_frames'])
        bucket['cached'] += len(active) - len(uncached)
        bucket['max_age'] = max(bucket['max_age'], feedback_age_bound(row) or 0)
        bucket['late'].extend(dict(session=row['id'], **f) for f in late_results(row))
        first_generation = row['attempts'][0]['generation'] if row['attempts'] else None
        for cohort, selected in [('first_attempt', [f for f in uncached if f['generation'] == first_generation]),
                                 ('reused_worker', [f for f in uncached if f['generation'] != first_generation])]:
            bucket['cohorts'][cohort].extend(1000*(f['finished_s']-f['rendered_s']) for f in selected)
        for f in active:
            for name, value in f['stage_ms'].items():
                if name.endswith('_ms'):
                    bucket['stages'][name].append(value)
            for name, value in f['cpu_ms'].items():
                bucket['stages'][name+'_cpu_ms'].append(value)
            for name, a, b in [('all_active_request_queue_ms','captured_s','render_started_s'),
                                ('all_active_render_ms','render_started_s','rendered_s'),
                                ('all_active_ipc_ms','finished_s','received_s')]:
                bucket['stages'][name].append(1000*(f[b]-f[a]))
        for f in row['frames']:
            parts = ['queue_ms','render_ms','inference_ms','ipc_ms','delivery_wait_ms','control_ms','apply_ms']
            assert abs(sum(f[k] for k in parts) - f['capture_to_command_ms']) < 1e-4
            for name in parts + (['control_thread_cpu_ms','apply_thread_cpu_ms'] if row['diagnostics'] else []):
                bucket['stages']['accepted_'+name].append(f[name])
        worst = sorted(active, key=lambda f: f['finished_s']-f['rendered_s'], reverse=True)[:10]
        for f in worst:
            item = compact_frame(f, row['id'])
            item['processing_ms'] = 1000 * (f['finished_s']-f['rendered_s'])
            item['overlapping_worker_gc'] = overlap(f.get('worker_gc', []), f['rendered_s'], f['finished_s'])
            bucket['outliers'].append(item)
        bucket['command_outliers'].extend(compact_frame(f,row['id']) for f in
            sorted(row['frames'], key=lambda f:f['capture_to_command_ms'], reverse=True)[:5])
        for stop in row['events']:
            if stop['kind'] == 'stop' and stop['reason'] in ('stale_camera', 'control_overrun'):
                bucket['diagnoses'].append(diagnose_stop(row, stop))
        bucket['gc'].extend(dict(session=row['id'], **g) for g in row['gc_events'])
        bucket['spikes'].extend(dict(session=row['id'], **c) for c in row['cycle_spikes'] if c['active'])
        bucket['session_rows'].append(dict(id=row['id'], round=row['round'], started_utc=row['started_utc'],
            attempts=len(row['attempts']), alignments=summary['alignments'],
            first_attempt_outcome=row['attempts'][0]['outcome'] if row['attempts'] else None,
            active_s=session_duration(row), ready_s=row['ready_elapsed_s'],
            processing_ms=distribution(processing), uncached_processing_ms=distribution(uncached_processing),
            capture_to_command_ms=distribution(commands), maximum_feedback_age_ms=feedback_age_bound(row),
            control_deadline_misses=summary['counters']['control_deadline_misses'],
            freshness_watchdog_trips=summary['freshness_watchdog_trips'],
            gpu_before=row['gpu_before'], gpu_after=row['gpu_after']))
    results = {}
    for mode, bucket in buckets.items():
        attempts = bucket['attempts']
        sessions = bucket['summaries']
        n = len(sessions)
        first = [a for a in attempts if a['index'] == 0]
        reused = [a for a in attempts if a['index'] != 0]
        metrics = {key: distribution(v for g in groups for v in g)
                   for key, groups in bucket['groups'].items()}
        ci = {key: bootstrap_sessions(groups, repeats, seed)
              for key, groups in bucket['groups'].items() if key != 'all_active_capture_to_delivery_ms'}
        counts = bucket['counters']
        outcomes = dict(Counter(a['outcome'] for a in attempts))
        clean_sessions = sum(not s['counters']['control_deadline_misses'] and not s['freshness_watchdog_trips']
                             and s['attempts'] == s['alignments'] and not s['error'] and not s['initialization_error']
                             for s in sessions)
        split = max(1, n//2)
        halves = {key: dict(first=distribution(v for g in groups[:split] for v in g),
                           second=distribution(v for g in groups[split:] for v in g))
                  for key,groups in bucket['groups'].items() if key != 'all_active_capture_to_delivery_ms'}
        top_spikes = sorted(bucket['spikes'], key=lambda c:c['wall_ms'], reverse=True)[:10]
        results[mode] = dict(sessions=n, attempts=len(attempts), alignments=outcomes.get('converged',0),
            outcomes=outcomes, sessions_without_failure=clean_sessions,
            first_attempt=dict(attempts=len(first),aligned=sum(a['outcome']=='converged' for a in first)),
            reused_worker_attempts=dict(attempts=len(reused),aligned=sum(a['outcome']=='converged' for a in reused)),
            backends=sorted(bucket['backend']), active_s=bucket['active_s'],ready_s=bucket['ready_s'],
            elapsed_s=bucket['elapsed_s'], sensor_frames=bucket['sensor_count'], cached_active_frames=bucket['cached'],
            **metrics, cluster_bootstrap_95pct=ci, bootstrap_replicates=repeats, bootstrap_seed=seed,
            first_second_half=halves, cohort_uncached_processing_ms={k:distribution(v) for k,v in bucket['cohorts'].items()},
            maximum_feedback_age_ms=bucket['max_age'], counters=dict(counts),
            freshness_watchdog_trips=sum(s['freshness_watchdog_trips'] for s in sessions),
            late_after_freshness_stop=sum(f['stop_reason']=='stale_camera' for f in bucket['late']),
            late_after_control_stop=sum(f['stop_reason']=='control_overrun' for f in bucket['late']),
            all_stops_stayed_latched=not bucket['unsafe_stops'], unsafe_stops=bucket['unsafe_stops'],
            telemetry_complete=bucket['telemetry_complete'], diagnostics_enabled=manifest['plan']['diagnostics'],
            diagnostics_complete=bucket['diagnostics_complete'],
            cycle_spikes_evicted=bucket['cycle_evictions'], gc_events_evicted=bucket['gc_evictions'],
            errors=bucket['errors'], stages_ms={k:distribution(v) for k,v in bucket['stages'].items()},
            zero_event_session_probability_upper_95pct=(1-.05**(1/n)) if clean_sessions==n else None,
            session_rows=bucket['session_rows'],
            longest_processing_frames=sorted(bucket['outliers'],key=lambda f:f['processing_ms'],reverse=True)[:10],
            oldest_accepted_commands=sorted(bucket['command_outliers'],key=lambda f:f['capture_to_command_ms'],reverse=True)[:10],
            deadline_diagnoses=bucket['diagnoses'], late_results=bucket['late'], longest_control_cycles=top_spikes,
            parent_gc_ms=distribution(1000*(g['finished_s']-g['started_s']) for g in bucket['gc']),
            safety_metrics=stress_safety(sessions))
    return dict(manifest=manifest, completion=completion, methods=results)


def describe(stats, keys=('mean','p95','p99','p99_9','max')):
    return '—' if stats is None else ' / '.join(f'{stats[k]:.1f}' for k in keys)


def report(result, directory):
    methods=result['methods']; plan=result['manifest']['plan']; complete=result['completion']
    lines=['# Sustained perception latency validation','',
        'The same starting offset, 50 ms added transport, 400 ms capture-age budget, 50 ms control-loop deadline, '
        'controller, calibration, scene, model settings and dependency versions as the preceding three-run comparison. '
        'Each process uses the same initial model warm-up. Explicit offset/align cycles then reuse that worker; '
        'the stopping criterion and 1.1 s post-stop observation window are unchanged.','',
        f"Predeclared stopping rule: at least {plan['minimum_sessions_per_method']} sessions and "
        f"{plan['target_uncached_active_frames_per_method']:,} uncached active frames per method, "
        f"or {plan['maximum_sessions_per_method']} sessions maximum. Each session runs for at least "
        f"{plan['minimum_session_s']:g} s after ready and ends at an attempt boundary. "
        'Modes alternate in each complete round; all failures remain included.','',
        'Success counts show the 95% Clopper-Pearson (exact) confidence interval for the true success rate.','',
        '| Method | Sessions | Aligned / attempts | First attempt in each process | Subsequent attempts | Active / ready minutes | Uncached active frames |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for mode,r in methods.items():
        f=r['first_attempt'];s=r['reused_worker_attempts']
        lines.append(f"| {NAMES[mode]} | {r['sessions']} | {r['alignments']}/{r['attempts']} {format_interval(r['alignments'], r['attempts'])} | "
            f"{f['aligned']}/{f['attempts']} {format_interval(f['aligned'], f['attempts'])} | "
            f"{s['aligned']}/{s['attempts']} {format_interval(s['aligned'], s['attempts'])} | "
            f"{r['active_s']/60:.1f} / {r['ready_s']/60:.1f} | {r['uncached_processing_ms']['count']:,} |")
    failures=sum(r['attempts']-r['alignments'] for r in methods.values())
    lines += [''] + [f"- {NAMES[mode]}: {interpretation(r['alignments'], r['attempts'])}" for mode, r in methods.items()]
    missed=sum(r['counters']['control_deadline_misses'] for r in methods.values())
    fresh=sum(r['freshness_watchdog_trips'] for r in methods.values())
    lines += ['',f'Observed {failures} unsuccessful alignment attempts, {missed} control deadline misses and '
        f'{fresh} freshness watchdog trips. Sample target met: {complete["sample_targets_met"]}. '
        f'Source unchanged throughout the campaign: {complete["source_unchanged"]}.','',
        '## Processing latency','',
        '| Method | Cohort | Samples | Mean / p95 / p99 / p99.9 / maximum (ms) |',
        '|---|---|---:|---|']
    for mode,r in methods.items():
        for title,key in [('All active captures','processing_ms'),('Uncached active captures','uncached_processing_ms')]:
            s=r[key];lines.append(f"| {NAMES[mode]} | {title} | {s['count']:,} | {describe(s)} |")
    lines += ['', 'Processing is the complete wall interval from image acquisition completion to the finished perception result. '
        'It includes refinement, host/device waits and preemption. Active captures remain counted when processing completes after a stop. '
        'Idle captures are excluded. Cache hits are shown in the all-active distribution and excluded explicitly in the second distribution.','',
        '## Capture to command and held feedback age','',
        '| Method | Accepted commands | Mean / p95 / p99 / maximum (ms) | Maximum held feedback age bound (ms) |',
        '|---|---:|---|---:|']
    for mode,r in methods.items():
        s=r['capture_to_command_ms'];lines.append(f"| {NAMES[mode]} | {s['count']:,} | "
            f"{describe(s,('mean','p95','p99','max'))} | {r['maximum_feedback_age_ms']:.1f} |")
    lines += ['', 'Capture-to-command uses actual accepted command application timestamps. Rejected results have no command timestamp; '
        'their processing remains in the preceding table and their capture-to-delivery distribution is in summary.json. '
        'The held-age bound uses the preceding capture through the next application or stop, and includes initial waiting after arming. '
        'It is evaluated separately for every attempt and includes the interval between accepted updates.','',
        '## Watchdogs, bounded work and completeness','',
        '| Counter | '+' | '.join(NAMES[m] for m in methods)+' |',
        '|---|'+'---:|'*len(methods)]
    counter_names=[('Acquisition slots','capture_slots'),('Captured requests','captures'),('Skipped busy acquisition slots','busy_capture_skips'),
        ('Pending request replacements','input_dropped'),('Transport results replaced','transport_dropped'),
        ('Result mailbox drops','result_mailbox_dropped'),('Pending transport entries cancelled at stops','pending_cancelled'),
        ('Obsolete generation results','obsolete_results'),('Expired requests before inference','pre_inference_dropped'),
        ('Expired results','stale_results'),('Completed but unreceived results','completed_but_unreceived'),
        ('Requests never started','requests_not_started'),('Control deadline misses','control_deadline_misses'),
        ('Unsafe motion ticks','unsafe_motion_ticks'),('Motion ticks after stop','post_stop_motion_ticks'),('Forbidden contacts','contacts')]
    for title,key in counter_names:
        lines.append('| '+title+' | '+' | '.join(str(methods[m]['counters'].get(key,0)) for m in NAMES if m in methods)+' |')
    for title,key in [('Freshness watchdog trips','freshness_watchdog_trips'),('Late completions after freshness stop','late_after_freshness_stop'),
                      ('Late completions after control stop','late_after_control_stop'),('All stops stayed latched','all_stops_stayed_latched'),
                      ('Frame/command/loop telemetry complete','telemetry_complete'),('Extended diagnostics enabled','diagnostics_enabled'),
                      ('Control-cycle/parent-GC buffers complete','diagnostics_complete')]:
        lines.append('| '+title+' | '+' | '.join(str(methods[m][key]) for m in NAMES if m in methods)+' |')
    import safety_metrics
    lines += ['', 'Safety envelope (worst session; extremes over every 2 ms physics step; simulation values):', '']
    lines += safety_metrics.table_lines([(NAMES[m], methods[m].get('safety_metrics')) for m in NAMES if m in methods])
    lines += ['', 'Counters cover entire sessions, including post-stop observation and draining. Skipped slots do not acquire an image. '
        'Pending cancellation at a stop and obsolete generations after an explicit reset are intentional cleanup, not evidence of growing queues. '
        'Worker expiry acknowledgments and worker expiry totals are two views of the same events and must not be summed. '
        'Request/result mailboxes are bounded to one and transport buffering to eight. '
        'Capture-slot counts refer to actual runtime acquisition opportunities; busy skips count opportunities suppressed by an occupied worker. '
        'Control scheduling can also reduce the rate of those opportunities below the nominal 30 Hz. '
        'Every acquired request and runtime capture slot is reconciled against completion, expiry and drop counters.','',
        '## Statistical uncertainty and stability','',
        '95% percentile bootstrap intervals resample whole sessions (2,000 replicates, fixed seed); they do not treat correlated frames '
        'within an alignment or session as independent. These intervals assume sessions are sufficiently exchangeable. '
        'Chronological host/thermal effects can still limit that assumption. The observed maximum is not a worst-case execution-time guarantee.','',
        '| Method | Metric | Estimate (ms) | Session bootstrap 95% interval (ms) |','|---|---|---:|---:|']
    for mode,r in methods.items():
        for key,title in [('uncached_processing_ms','Uncached processing'),('capture_to_command_ms','Capture to command')]:
            for metric in ('mean','p95','p99','p99_9') if key=='uncached_processing_ms' else ('mean','p95','p99'):
                lo,hi=r['cluster_bootstrap_95pct'][key][metric]
                lines.append(f"| {NAMES[mode]} | {title} {metric.replace('_9','.9')} | {r[key][metric]:.1f} | {lo:.1f}–{hi:.1f} |")
    for mode,r in methods.items():
        n=r['uncached_processing_ms']['count']
        lines += ['',f"{NAMES[mode]} p99.9 has only approximately {n*.001:.1f} uncached observations in its upper 0.1% tail. "
            'It is much better sampled than three short trials but remains sensitive to rare events.','']
        upper=r['zero_event_session_probability_upper_95pct']
        if upper is not None:
            lines += [f"With zero failed sessions out of {r['sessions']}, the one-sided exact 95% upper bound on a session's "
                f"failure probability is {100*upper:.1f}% **if sessions are independent and identically distributed**. "
                'Zero observed misses is not proof of a zero miss rate.','']
    lines += ['| Method | Worker cohort | Uncached processing mean / p95 / p99 / p99.9 / maximum (ms) |',
        '|---|---|---|']
    for mode,r in methods.items():
        for key,label in [('first_attempt','First alignment'),('reused_worker','Subsequent alignments')]:
            lines.append(f"| {NAMES[mode]} | {label} | {describe(r['cohort_uncached_processing_ms'][key])} |")
    lines += ['', '| Method | Half of session sequence | Processing p95 / p99 / max (ms), uncached | Command p95 / p99 / max (ms) |',
              '|---|---|---|---|']
    for mode,r in methods.items():
        for half in ('first','second'):
            lines.append(f"| {NAMES[mode]} | {half} | "
                f"{describe(r['first_second_half']['uncached_processing_ms'][half],('p95','p99','max'))} | "
                f"{describe(r['first_second_half']['capture_to_command_ms'][half],('p95','p99','max'))} |")
    lines += ['', '![Latency by chronological session](session-latency.png)','',
        '## Spike attribution','',
        'See diagnostics.json for every watchdog event, overlapping control cycle, parent/worker GC activity, concurrent sensor work '
        'and nearby commands. The ten slowest processing frames, oldest commands and longest active control cycles are retained per method.','',
        '| Method | Session / sequence | Processing (ms) | Coarse / refinement (ms) | Extract / match / transfer host (ms) |',
        '|---|---|---:|---|---|']
    for mode,r in methods.items():
        for f in r['longest_processing_frames'][:3]:
            s=f['stage_ms'];pair=lambda keys:' / '.join(f'{s[k]:.1f}' if k in s else '—' for k in keys)
            lines.append(f"| {NAMES[mode]} | {f['session']} / {f['sequence']} | {f['processing_ms']:.1f} | "
                f"{pair(['coarse_ms','refinement_ms'])} | {pair(['extract_host_ms','match_host_ms','transfer_wait_ms'])} |")
    for mode,r in methods.items():
        controls=[d for d in r['deadline_diagnoses'] if d['stop']['reason']=='control_overrun']
        freshness=[d for d in r['deadline_diagnoses'] if d['stop']['reason']=='stale_camera']
        selected=controls+freshness[:1]
        for d in selected:
            s=d['dominant_stage'];c=d['cycle']
            lines += ['',f"**{NAMES[mode]}, {d['session']}, generation {d['stop']['generation']}: {d['stop']['reason']}.** "
                +(f"Nearby control cycle {c['wall_ms']:.2f} ms; largest interval `{s['stage']}` {s['wall_ms']:.2f} ms, "
                  f"thread CPU {s['thread_cpu_ms']:.3f} ms and {s['cpu_cycles']} raw CPU cycles. " if c and s else 'No retained preceding control stage. ')
                +f"Overlapping parent GC events: {len(d['overlapping_parent_gc'])}; worker GC events: {len(d['overlapping_worker_gc'])}."]
    lines += ['', 'CUDA event intervals are recorded on the existing stream and read only after the existing required device-to-host copy. '
        'No global CUDA synchronization is added. They measure stream elapsed intervals, including host submission gaps/idle periods, '
        'not the sum of GPU kernel execution times. Host extraction/matching and transfer intervals are reported separately. '
        'Coarse and refinement timings contain their respective child stages and must not be added to those children. '
        'Process CPU time sums work across threads and can exceed wall time.','',
        'Windows thread CPU time on this host was empirically quantized to 15.625 ms. A zero sub-tick CPU reading does not prove a wait. '
        'Control-stage raw QueryThreadCycleTime counters provide finer evidence but are not converted to milliseconds because clock frequency varies. '
        'A large wall interval with few CPU cycles points to off-CPU delay; distinguishing scheduler preemption from a lock/device wait requires '
        'native scheduling traces. GC callbacks record observed pauses; worker frames carry the last eight GC events as bounded context.','',
        'The historical 65.9 ms Learned control miss had no CPU/scheduler trace. Its exact historical cause cannot be reconstructed '
        'from the saved 40.5 ms command-application interval alone. A recurring stage can be localized by this campaign; it should not be '
        'retroactively asserted to be the cause of the old event without matching evidence.','',
        '## Reproduction and evidence','',
        'Run from the repository root with the existing environment:', '', '```powershell',
        '.\\outputs\\visual-servoing-simulation\\.venv\\Scripts\\python.exe -B outputs/visual-servoing-simulation/run_latency_stress.py --output outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN --min-sessions 12 --max-sessions 20 --session-seconds 120 --target-uncached-frames 10000',
        '.\\outputs\\visual-servoing-simulation\\.venv\\Scripts\\python.exe -B tools/analyze_latency_stress.py outputs/visual-servoing-simulation/results/latency/NEW-CAMPAIGN',
        '```','',
        '[Plan, runtime versions and source hashes](manifest.json), [completion checks](completion.json), '
        '[pooled distributions and session results](summary.json), [failure/spike evidence](diagnostics.json), '
        '[individual session summaries](sessions.json). Raw per-frame gzip traces remain local under `traces/`; '
        'every trace has a SHA-256 in its session summary. Source files and runtime are checked at every session boundary and completion.','']
    for name,label in [('DIAGNOSIS.md','Diagnostic follow-up and attribution limits'),('VERIFICATION.md','Implementation verification')]:
        if (directory/name).is_file():
            lines.extend([f'[{label}]({name}).',''])
    (directory/'REPORT.md').write_text('\n'.join(lines),encoding='utf-8')


def plot(result,directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,len(result['methods']),figsize=(6*len(result['methods']),7),sharex=True,squeeze=False)
    for column,(mode,r) in enumerate(result['methods'].items()):
        rows=r['session_rows'];x=np.arange(1,len(rows)+1)
        for row,key in enumerate(('uncached_processing_ms','capture_to_command_ms')):
            ax=axes[row,column]
            for metric,label,color in [('p95','p95','#2376a4'),('p99','p99','#dc8b24'),('max','Maximum','#a82e35')]:
                ax.plot(x,[s[key][metric] for s in rows],'.-',label=label,color=color)
            if row==0:ax.set_title(NAMES[mode])
            if row==1:
                ax.axhline(400,color='#444',linestyle='--',linewidth=1,label='400 ms capture-age budget')
                ax.set_xlabel('Chronological session')
            ax.set_ylabel(('Uncached processing' if row==0 else 'Capture to command')+' (ms)')
            ax.grid(alpha=.2);ax.legend(fontsize=8)
            for i,s in enumerate(rows):
                if s['control_deadline_misses'] or s['freshness_watchdog_trips']:
                    ax.axvspan(i+.8,i+1.2,color='#c73e3e',alpha=.15)
    fig.suptitle('50 ms transport delay; unchanged controller and warm-up\nShaded sessions contain watchdog stops; all failed attempts retained')
    fig.tight_layout()
    fig.savefig(directory/'session-latency.png',dpi=160)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--bootstrap',type=int,default=2000)
    args=parser.parse_args()
    if args.bootstrap<1:parser.error('At least one bootstrap replicate is required')
    result=aggregate(args.directory,args.bootstrap)
    diagnostics={mode:{k:r[k] for k in ('deadline_diagnoses','late_results','longest_processing_frames',
        'oldest_accepted_commands','longest_control_cycles')} for mode,r in result['methods'].items()}
    summary={mode:{k:v for k,v in r.items() if k not in diagnostics[mode]} for mode,r in result['methods'].items()}
    write_json(args.directory/'summary.json',dict(completion=result['completion'],methods=summary))
    write_json(args.directory/'diagnostics.json',diagnostics)
    report(result,args.directory);plot(result,args.directory)
    print(json.dumps({mode:{k:r[k] for k in ('sessions','attempts','alignments','processing_ms','uncached_processing_ms',
        'capture_to_command_ms','maximum_feedback_age_ms','freshness_watchdog_trips','counters')} for mode,r in result['methods'].items()},indent=2))


if __name__=='__main__':
    main()
