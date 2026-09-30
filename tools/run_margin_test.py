"""Latency-margin test for SIFT and Learned GPU.

How much slower may perception get before alignment fails? Each level adds a fixed
delay to every active perception frame using the runtime's existing fault injection
(RuntimeConfig.inference_stall_s, applied in the sensor worker between rendering and
matching, from the start of every alignment). Everything else is production:
controller, calibration, scene, 50 ms transport delay, 400 ms freshness watchdog and
50 ms control deadline. Idle frames are not delayed.

Levels run from 0 ms upwards, alternating SIFT and Learned GPU at each level. Every
level uses a fresh worker and the same declared poses. A method stops escalating
after a level where nothing aligned. The result is a measured margin, not PASS/FAIL.

Exit codes: 0 measured (or a completed --smoke run), 2 INVALID/INCOMPLETE.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import traceback

import run_acceptance_test as rat
from run_acceptance_test import env_monitor, read, write, digest, fmt

DEFAULT_CONFIG = rat.ROOT / 'tools' / 'margin_test.json'
DEFAULT_RESULTS = rat.APP / 'results' / 'margin-test'
SMOKE = dict(added_delay_ms=[0, 120], alignments_per_level=3)


def validate(config):
    problems = []
    if config.get('runtime') != rat.PRODUCTION_RUNTIME:
        problems.append(f"runtime {config.get('runtime')} differs from production {rat.PRODUCTION_RUNTIME}")
    if sorted(config.get('methods', [])) != ['learned', 'natural']:
        problems.append('both SIFT (natural) and Learned GPU (learned) are required')
    delays = config.get('added_delay_ms', [])
    if not delays or delays[0] != 0 or delays != sorted(set(delays)) or not all(0 <= d <= 400 for d in delays):
        problems.append('added_delay_ms must start at 0, increase strictly and stay within 0-400 ms')
    if config.get('alignments_per_level', 0) < 5:
        problems.append('at least 5 alignments per level are required')
    if len(config.get('poses_degrees', [])) < 3:
        problems.append('at least three starting poses are required')
    if config.get('fault_after_s') != 0.0:
        problems.append('fault_after_s must be 0 so every active frame is delayed')
    if config.get('post_stop_seconds') != 1.1:
        problems.append('post-stop observation must stay 1.1 s')
    return problems


def build_plan(config, smoke=False, only=None):
    delays = SMOKE['added_delay_ms'] if smoke else config['added_delay_ms']
    per_level = SMOKE['alignments_per_level'] if smoke else config['alignments_per_level']
    modes = [m for m in config['methods'] if not only or m == only]
    sessions = [dict(id=f'margin-{mode}-{d:03d}ms', phase='margin', mode=mode, delay_ms=d,
                     pose_ids=list(range(len(config['poses_degrees']))), max_attempts=per_level)
                for d in delays for mode in modes]
    return dict(delays_ms=delays, alignments_per_level=per_level, smoke=smoke, only_method=only, sessions=sessions)


def session_config(config, delay_ms, stale_resume_s=rat.DEFAULT_STALE_RESUME_MS / 1000):
    """Production runtime plus the existing fault-injection delay; nothing else changes.

    stale_resume_s is the watchdog response: > 0 hold and resume (default), 0 stop.
    """
    c = json.loads(json.dumps(config))
    c['runtime'] = dict(config['runtime'], inference_stall_s=delay_ms / 1000, fault_after_s=config['fault_after_s'],
                        stale_resume_s=stale_resume_s)  # Always explicit: 0 = stop.
    return c


def stale_stops(session):
    """Alignments the freshness watchdog ended (older results have no holds, so every trip was a stop)."""
    return session.get('stale_stops', session['freshness_trips'])


# ----------------------------------------------------------------------------- analysis

def trip_phases(directory, session_id):
    """Split freshness trips into 'before the first command' (start-up) and 'mid-alignment'."""
    import gzip
    commands, trips = {}, []
    with gzip.open(directory / 'traces' / (session_id + '.jsonl.gz'), 'rt', encoding='utf-8') as stream:
        for line in stream:
            chunk = json.loads(line)
            for c in chunk['frames']:
                commands[c['generation']] = commands.get(c['generation'], 0) + 1
            trips += [e['generation'] for e in chunk['events'] if e['kind'] == 'stop' and e['reason'] == 'stale_camera']
    at_start = sum(commands.get(g, 0) == 0 for g in trips)
    return dict(at_start=at_start, mid_alignment=len(trips) - at_start)


def level_confidence(config):
    return float(config.get('confidence', rat.binomial_ci.DEFAULT_CONFIDENCE))


def level_succeeded(aligned, attempts, config):
    """A level counts as tolerated by its observed rate (characterisation) or, if the
    configuration declares required_success_rate, by the lower confidence bound."""
    if 'required_success_rate' in config:
        return rat.binomial_ci.meets_required(aligned, attempts, config['required_success_rate'], level_confidence(config))
    return attempts > 0 and aligned / attempts >= config['tolerated_success_rate']


def tolerated_rule_text(config, per_level):
    """(rule, caveat): what 'tolerated' means, and what a level's count can and cannot show."""
    b, confidence = rat.binomial_ci, level_confidence(config)
    if 'required_success_rate' in config:
        return (f"reached a {confidence * 100:g}% lower confidence bound of "
                f"{b.percent(config['required_success_rate'])} on alignment success"), ''
    rate = config['tolerated_success_rate']
    caveat = ''
    if per_level:
        low, _ = b.clopper_pearson(per_level, per_level, confidence)
        caveat = (f"With {per_level} alignments per level, {per_level}/{per_level} means the true success rate at that "
                  f"level is at least {b.percent(low)} with {confidence * 100:g}% confidence, not 100%. Showing at least "
                  f"95% would need {b.trials_needed(0.95, confidence)} alignments per level, so the margin is a "
                  'characterisation, not a reliability guarantee.')
    return f"aligned {100 * rate:.0f}% (observed)", caveat


def confounded(session):
    e = session.get('environment') or {}
    return bool(e.get('low_state_episodes'))


def method_margin(mode, sessions, config, directory):
    """Margin from levels unaffected by the laptop's own low-power state.

    Levels where that state was detected are shown but excluded, so a slow start-up
    period cannot hide (or fake) the injected-delay margin.
    """
    rows = sorted((s for s in sessions if s['mode'] == mode), key=lambda s: s['delay_ms'])
    if not rows:
        return None
    usable = [s for s in rows if not confounded(s) and s['processing_ms']]
    ref = usable[0] if usable else None
    ref_p50 = None if ref is None else ref['processing_ms']['p50'] - ref['delay_ms']
    ref_p99 = None if ref is None else ref['processing_ms']['p99'] - ref['delay_ms']
    levels, tolerated, first_failure, ok_so_far = [], None, None, True
    for s in rows:
        rate = s['aligned'] / s['attempts'] if s['attempts'] else 0.0
        ci = rat.binomial_ci.summary(s['aligned'], s['attempts'], level_confidence(config))
        clean = level_succeeded(s['aligned'], s['attempts'], config) and stale_stops(s) == 0 and not s['runtime_errors']
        excluded = confounded(s)
        if not excluded:
            if clean and ok_so_far:
                tolerated = s['delay_ms']
            elif not clean and first_failure is None:
                first_failure = s['delay_ms']
            ok_so_far = ok_so_far and clean
        p50 = s['processing_ms']['p50'] if s['processing_ms'] else None
        levels.append(dict(delay_ms=s['delay_ms'], id=s['id'], attempts=s['attempts'], aligned=s['aligned'],
                           success_rate=rate, success_ci=ci, freshness_trips=stale_stops(s), control_misses=s['control_misses'],
                           holds=s.get('watchdog_pauses', 0), held_s=s.get('paused_s', 0.0),
                           processing_ms=s['processing_ms'], capture_ms=s['capture_ms'],
                           measured_added_ms=None if p50 is None or ref_p50 is None else p50 - ref_p50,
                           converge_s=s['later_alignment_s'] or (None if s['first_alignment_s'] is None else
                                                                 dict(p50=s['first_alignment_s'], count=1)),
                           unsafe=(s['unsafe_motion'] + s['post_stop_motion'] + s['contacts'] + s['unlatched_stops']
                                   + s.get('paused_motion', 0)),
                           trips=trip_phases(directory, s['id']), delayed_frames=s['counts'].get('fault_results', 0),
                           confounded=excluded, actuator=s.get('actuator'), physical_stops=s.get('physical_stops')))
    factor = None if tolerated is None or not ref_p50 else (ref_p50 + tolerated) / ref_p50
    return dict(mode=mode, label=config['method_labels'][mode],
                baseline_processing_ms=None if ref is None else dict(p50=ref_p50, p99=ref_p99, from_level_ms=ref['delay_ms']),
                tolerated_delay_ms=tolerated, first_failing_delay_ms=first_failure, equivalent_slowdown=factor,
                excluded_levels_ms=[l['delay_ms'] for l in levels if l['confounded']],
                levels=levels, any_confounded=any(l['confounded'] for l in levels),
                unsafe_events=sum(l['unsafe'] for l in levels), control_misses=sum(l['control_misses'] for l in levels))


def ci_csv(level):
    ci = level.get('success_ci') or rat.binomial_ci.summary(level['aligned'], level['attempts'])
    return tuple('' if ci[k] is None else round(ci[k], 4) for k in ('ci_low', 'ci_high'))


def physical_csv(level):
    block = ((level.get('physical_stops') or {}).get('pause') or {})
    time_ms, travel = block.get('physical_stop_ms'), block.get('camera_travel_mm')
    return ('' if not time_ms else round(time_ms['max'], 1), '' if not travel else round(travel['max'], 2))


def physical_lines(margins):
    """Simulated physical response to holds and stops (informational, not gated)."""
    lines = []
    for m in margins:
        levels = [l for l in m['levels'] if l.get('physical_stops') is not None]
        if not levels:
            continue
        actuator = next((l['actuator'] for l in levels if l.get('actuator')), {}) or {}
        parts = []
        for kind, name in (('pause', 'hold'), ('stop', 'stop')):
            blocks = [l['physical_stops'].get(kind) for l in levels if (l['physical_stops'].get(kind) or {}).get('physical_stop_ms')]
            if blocks:
                parts.append(f"after a {name} up to {max(b['physical_stop_ms']['max'] for b in blocks):.0f} ms to standstill "
                             f"and {max(b['camera_travel_mm']['max'] for b in blocks):.1f} mm of camera travel")
        lines.append(f"- {m['label']}, physical response with the simulated actuator model "
                     f"`{actuator.get('profile', '?')}`: " + ('; '.join(parts) or 'no stop or hold while moving') + '. '
                     'The zero command itself is immediate; these are simulated values, not real-robot data.')
    return lines


def write_report(directory, status, reason, margins, manifest, config):
    write(directory / 'margin.json', dict(status=status, reason=reason, methods=margins))
    lines = ['delay_ms,method,attempts,aligned,success_rate,freshness_trips,processing_p50_ms,processing_p99_ms,'
             'p50_minus_normal_ms,capture_p99_ms,converge_median_s,trips_at_start,trips_mid_alignment,delayed_frames,low_power_state,'
             'watchdog_holds,held_s,success_ci_low,success_ci_high,hold_physical_stop_max_ms,hold_camera_travel_max_mm']
    for m in margins:
        for l in m['levels']:
            p, c, v = l['processing_ms'] or {}, l['capture_ms'] or {}, l['converge_s'] or {}
            lines.append(','.join(str(x) for x in (
                l['delay_ms'], m['label'], l['attempts'], l['aligned'], round(l['success_rate'], 3), l['freshness_trips'],
                round(p.get('p50', float('nan')), 1), round(p.get('p99', float('nan')), 1),
                '' if l['measured_added_ms'] is None else round(l['measured_added_ms'], 1),
                round(c.get('p99', float('nan')), 1), round(v.get('p50', float('nan')), 2), l['trips']['at_start'],
                l['trips']['mid_alignment'], l['delayed_frames'], int(l['confounded']), l.get('holds', 0),
                round(l.get('held_s', 0.0), 2), *ci_csv(l), *physical_csv(l))))
    (directory / 'margin.csv').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    resume = rat.stale_resume_s(dict(runtime_config=manifest.get('watchdog') or {}))
    L = [f'# Latency-margin test: {status}', '', rat.watchdog_line(dict(runtime_config=manifest.get('watchdog') or {}), holds_gated=False), '']
    if reason:
        L += [reason, '']
    modes = sorted({env_monitor.power_mode_name(b.get('power_mode')) for b in manifest.get('session_boundaries', [])} - {None})
    if modes and modes != ['Best performance']:
        L += [f"**Warning: Windows power mode was {' / '.join(modes)}, not Best performance.** Windows may lower CPU "
              'and GPU speed, which shrinks the measured margin. Set Settings > System > Power & battery > Power mode '
              'to Best performance and repeat the run.', '']
    L += ['How much extra per-frame perception latency each method tolerates before alignment fails. A fixed '
          'delay is added to every active frame; the production controller, 400 ms freshness watchdog, '
          '50 ms control deadline and 50 ms transport delay are unchanged.', '',
          '| Method | Normal processing p50 / p99 (without injected delay) | Tolerated added delay | Equivalent slowdown | First failing level |',
          '|---|---:|---:|---:|---:|']
    for m in margins:
        base = m['baseline_processing_ms']
        tol = ('none' if m['tolerated_delay_ms'] is None else f"+{m['tolerated_delay_ms']} ms")
        first = 'not reached' if m['first_failing_delay_ms'] is None else f"+{m['first_failing_delay_ms']} ms"
        factor = 'n/a' if m['equivalent_slowdown'] is None else f"{m['equivalent_slowdown']:.1f}x slower perception"
        ref = '' if not base or not base['from_level_ms'] else f" (from the +{base['from_level_ms']} ms level)"
        tol = tol if not m['excluded_levels_ms'] else tol + ' (excluding ' + ', '.join(f'+{d}' for d in m['excluded_levels_ms']) + ' ms)'
        L.append(f"| {m['label']} | {fmt(base, ('p50', 'p99'))} ms{ref} | {tol} | {factor} | {first} |")
    rule, caveat = tolerated_rule_text(config, (manifest.get('plan') or {}).get('alignments_per_level'))
    L += ['', f'"Tolerated" is the largest level where it and every lower unaffected level {rule}, '
          'with no alignment ended by the freshness watchdog. Equivalent slowdown = '
          '(normal p50 + tolerated delay) / normal p50.', '']
    L += [caveat, ''] if caveat else []
    L += [
          f"\"Aligned\" shows the {level_confidence(config) * 100:g}% exact Clopper-Pearson confidence interval "
          'for the true success rate at each level.', '']
    for m in margins:
        L += [f"## {m['label']}", '',
              '| Added delay | Aligned | Stopped by freshness watchdog (at start / mid-alignment) | '
              + ('Held and resumed (total held) | ' if resume else '') + 'Processing p50 / p99 (ms) | '
              'p50 minus normal (ms) | Capture-to-command p99 (ms) | Time to converge (median) | Laptop low-power state |',
              '|---:|---:|---:|' + ('---:|' if resume else '') + '---:|---:|---:|---:|---|']
        for l in m['levels']:
            conv = 'n/a' if not l['converge_s'] else f"{l['converge_s']['p50']:.2f} s"
            added = 'n/a' if l['measured_added_ms'] is None else f"{l['measured_added_ms']:+.0f}"
            L.append(f"| +{l['delay_ms']} ms | {l['aligned']}/{l['attempts']} {rat.ci_text(l.get('success_ci'))} | {l['freshness_trips']} "
                     f"({l['trips']['at_start']} / {l['trips']['mid_alignment']}) | "
                     + (f"{l.get('holds', 0)} ({l.get('held_s', 0.0):.1f} s) | " if resume else '')
                     + f"{fmt(l['processing_ms'], ('p50', 'p99'))} | {added} | {fmt(l['capture_ms'], ('p99',))} | {conv} | "
                     f"{'YES - excluded from the margin' if l['confounded'] else 'no'} |")
        L.append('')
    L += ['## Test procedure', ''] + rat.procedure_lines(manifest) + ['']
    unsafe = sum(m['unsafe_events'] for m in margins)
    misses = sum(m['control_misses'] for m in margins)
    L += ['## Safety and validity', '',
          f"- Unsafe motion, post-stop motion, forbidden contacts and unlatched stops across all levels: {unsafe}. "
          f"Control deadline misses: {misses}.",
          '- A level marked "laptop low-power state" had 30 s windows with the GPU busy below 400 MHz or the CPU below '
          '85% of maximum frequency. Its result mixes the injected delay with a real slowdown; repeat the run before '
          'relying on it.' if any(m['any_confounded'] for m in margins) else
          '- No laptop low-power state was detected during any level.',
          '- "At start" trips happened before the first command of an alignment: the first fresh result must arrive '
          'within 400 ms of Align, and it can queue behind a frame already in progress. "Mid-alignment" trips happened '
          'after the robot was already moving.',
          '- The injected delay is constant. Real slower hardware also has longer tails, so the tolerated delay is '
          'an upper bound for hardware with the same median slowdown.',
          f"- Each level has {manifest['plan']['alignments_per_level']} alignments over "
          f"{len(config['poses_degrees'])} poses on a fresh worker, so a level where all of them succeeded is limited "
          'evidence, not a guarantee (see the confidence intervals).',
          *physical_lines(margins),
          rat.power_mode_line(manifest), rat.baseline_line(manifest.get('baseline_check') or {}),
          f"- Python `{(manifest.get('python') or {}).get('windows_executable')}`, CUDA GPU "
          f"{(manifest.get('selected_cuda_gpu') or {}).get('name', 'n/a')}.", '',
          'Per-level data: `margin.csv` (for plotting) and `margin.json`; per-session evidence in `margin-*.json` and `traces/`.']
    (directory / 'REPORT.md').write_text('\n'.join(L) + '\n', encoding='utf-8')


def finalize(directory, sessions, config, manifest, complete, invalid=None):
    rat.attach_environment(directory, sessions, manifest)
    for s in sessions:
        write(directory / (s['id'] + '.json'), s)
    margins = [m for m in (method_margin(mode, sessions, config, directory) for mode in config['methods']) if m]
    plan = manifest['plan']
    if invalid:
        status, reason = 'INVALID', invalid
    elif plan.get('smoke'):
        status, reason = 'SMOKE_ONLY', 'Short harness check with two levels; not a margin measurement.'
    elif not complete:
        status, reason = 'INCOMPLETE', 'The test did not complete; levels measured before it stopped are shown.'
    else:
        status, reason = 'MEASURED', None
    write_report(directory, status, reason, margins, manifest, config)
    return status


# ----------------------------------------------------------------------------- orchestration

def run(args):
    config = read(args.config)
    problems = validate(config)
    if problems:
        print('INVALID configuration: ' + '; '.join(problems), flush=True)
        return 2
    only = {'sift': 'natural', 'learned': 'learned', None: None}[args.method]
    plan = build_plan(config, args.smoke, only)
    from run_camera_robustness import fingerprint
    from realtime import RuntimeConfig
    from simulation import Simulation
    current = fingerprint()
    check = rat.baseline_check(current, args.baseline)
    if check['protected_changed']:
        print('INVALID: controller/calibration/scene/safety inputs differ from the validated baseline: '
              + ', '.join(check['protected_changed']), flush=True)
        return 2
    for d in plan['delays_ms']:
        RuntimeConfig(**session_config(config, d)['runtime'])  # Validates every level before starting.
    with Simulation(render=False) as sim:
        for pose in config['poses_degrees']:
            sim.reset(pose)
    power = rat.power_status()
    if power is not None and power.get('ACLineStatus') != 1:
        print('INVALID: connect AC power.', flush=True)
        return 2
    probe, inventory = rat.probe_cuda(), rat.nvidia_inventory()
    selected = rat.select_gpu(probe, inventory)
    if only != 'natural' and not args.smoke and not (selected and probe.get('cuda_available')):
        print('INVALID: CUDA is not available to this Python environment: ' + json.dumps(probe), flush=True)
        return 2
    uptime = rat.procedure_preflight()
    stamp = (datetime.now().strftime('%Y%m%d-%H%M%S') + ('-smoke' if args.smoke else '') + (f'-{args.method}' if args.method else '')
             + ('-stop' if not args.stale_resume_ms else '' if args.stale_resume_ms == rat.DEFAULT_STALE_RESUME_MS
                else f'-resume{args.stale_resume_ms:g}ms'))
    resume_s = args.stale_resume_ms / 1000
    directory = args.output or DEFAULT_RESULTS / stamp
    directory.mkdir(parents=True, exist_ok=False); (directory / 'traces').mkdir()
    write(directory / 'test-config.json', config)
    hashes = {p.name: digest(p) for p in (Path(__file__), Path(rat.__file__), rat.ENVIRONMENT_MODULE, args.config)}
    base_exe = getattr(sys, '_base_executable', sys.executable)
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(), status='running', command=sys.argv, plan=plan,
        config_path=str(args.config), hashes=hashes, local_utc_offset_s=env_monitor.utc_offset_s(),
        uptime_at_start_s=uptime, procedure=rat.PROCEDURE_DOC,
        runtime_configs={str(d): asdict(RuntimeConfig(**session_config(config, d, resume_s)['runtime'])) for d in plan['delays_ms']},
        watchdog=dict(stale_resume_s=resume_s),
        python=dict(executable=sys.executable, windows_executable=rat.this_executable(), base_executable=base_exe,
                    prefix=sys.prefix, version=sys.version),
        environment=rat.runtime_versions(), fingerprint=current, baseline_check=check,
        cuda_probe=probe, nvidia_smi=inventory, selected_cuda_gpu=selected,
        gpu_preferences=rat.gpu_preferences([sys.executable, base_exe, rat.this_executable()]), power_at_start=power,
        method='Fixed delay per active frame via RuntimeConfig.inference_stall_s (fault_after_s=0); fresh worker per '
               'level; levels alternate methods; declared poses cycle; production limits unchanged.')
    manifest['leftover_samplers'] = env_monitor.stop_leftover_samplers()
    manifest['snapshot_start'] = env_monitor.snapshot(directory, 'start')
    write(directory / 'manifest.json', manifest)
    print(f'Latency-margin test -> {directory}', flush=True)
    keep_awake = None
    if os.name == 'nt':
        import ctypes
        keep_awake = ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    monitor = env_monitor.GpuMonitor(directory / 'traces' / 'gpu.csv', config['gpu_sample_interval_ms'],
                                     None if not selected else (selected.get('nvidia_smi') or {}).get('index')).start()
    system = env_monitor.SystemMonitor(directory / 'traces' / 'system.csv', config['gpu_sample_interval_ms'])
    if not args.no_system_monitor:
        system.start()
    manifest['monitors'] = dict(gpu=monitor.metadata(), system=system.metadata())
    sessions, boundaries, stopped, complete, invalid, harness_error = [], [], set(), False, None, None
    try:
        for spec in plan['sessions']:
            if spec['mode'] in stopped:
                continue
            if fingerprint() != current or any(digest(Path(p)) != h for p, h in zip(
                    (Path(__file__), Path(rat.__file__), rat.ENVIRONMENT_MODULE, args.config), hashes.values())):
                invalid = 'Source, runner or configuration changed during the test.'
                break
            status_now = rat.power_status()
            boundaries.append(dict(session=spec['id'], utc=datetime.now(timezone.utc).isoformat(), power=status_now,
                                   power_mode=env_monitor.power_mode()))
            if status_now is not None and status_now.get('ACLineStatus') != 1:
                invalid = 'AC power was disconnected during the test.'
                break
            print(f"START {spec['id']}", flush=True)
            raw, attempts, ending = rat.run_session(spec['mode'], spec['pose_ids'], session_config(config, spec['delay_ms'], resume_s),
                                                    directory, spec['id'], max_attempts=spec['max_attempts'])
            result, attempts = rat.summarize(raw, attempts, ending, {k: spec[k] for k in ('id', 'phase', 'mode', 'pose_ids', 'delay_ms')},
                                             config)
            write(directory / 'traces' / (spec['id'] + '-attempts.json'), attempts)
            sessions.append(result)
            print(f"END {spec['id']}: aligned {result['aligned']}/{result['attempts']}, watchdog stops {stale_stops(result)}, holds {result.get('watchdog_pauses', 0)}, "
                  f"processing p50/p99 {fmt(result['processing_ms'], ('p50', 'p99'))} ms", flush=True)
            manifest['session_boundaries'] = boundaries
            finalize(directory, sessions, config, manifest, False)
            if result['unsafe_motion'] or result['post_stop_motion'] or result['contacts'] or result['unlatched_stops']:
                print('Safety violation: test stopped.', flush=True)
                break
            if result['runtime_errors']:
                print('Runtime error: test stopped.\n' + str(result['harness_error']), flush=True)
                break
            if config['stop_method_after_zero_success'] and result['aligned'] == 0 and spec['delay_ms'] > 0:
                stopped.add(spec['mode'])
                print(f"{config['method_labels'][spec['mode']]}: nothing aligned at +{spec['delay_ms']} ms; "
                      'higher levels skipped.', flush=True)
        else:
            complete = fingerprint() == current
            if not complete:
                invalid = 'Source changed during the test.'
    except KeyboardInterrupt:
        harness_error = 'Interrupted by user'; print('Interrupted.', flush=True)
    except Exception:
        harness_error = traceback.format_exc(); print(harness_error, flush=True)
    finally:
        if keep_awake:
            import ctypes
            ctypes.windll.kernel32.SetThreadExecutionState(keep_awake)
        monitor.stop(); system.stop()
        manifest['snapshot_end'] = env_monitor.snapshot(directory, 'end')
    manifest.update(status='finished', finished_utc=datetime.now(timezone.utc).isoformat(), complete=complete,
                    session_boundaries=boundaries, skipped_after_zero_success=sorted(stopped), harness_error=harness_error)
    write(directory / 'manifest.json', manifest)
    status = finalize(directory, sessions, config, manifest, complete, invalid)
    print((directory / 'REPORT.md').read_text(encoding='utf-8'), flush=True)
    return 0 if status in ('MEASURED', 'SMOKE_ONLY') and complete else 2


def report_only(directory):
    config, manifest = read(directory / 'test-config.json'), read(directory / 'manifest.json')
    sessions = []
    for spec in manifest['plan']['sessions']:
        path = directory / (spec['id'] + '.json')
        if not path.exists():
            continue
        saved = read(path)
        raw = directory / 'traces' / (spec['id'] + '.jsonl.gz')
        if digest(raw) != saved['raw_sha256']:
            print(f"INVALID: raw evidence changed for {spec['id']}", flush=True)
            return 2
        attempts = read(directory / 'traces' / (spec['id'] + '-attempts.json'))
        ending = read(directory / 'traces' / (spec['id'] + '-ending.json'))
        result, _ = rat.summarize(raw, attempts, ending, {k: spec[k] for k in ('id', 'phase', 'mode', 'pose_ids', 'delay_ms')},
                                  config, score=not all('physical_accuracy' in a for a in attempts))
        sessions.append(result)
    status = finalize(directory, sessions, config, manifest, manifest.get('complete', False))
    print((directory / 'REPORT.md').read_text(encoding='utf-8'), flush=True)
    return 0 if status in ('MEASURED', 'SMOKE_ONLY') else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, help='New output directory (default: results/margin-test/<timestamp>)')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--baseline', type=Path, default=rat.DEFAULT_BASELINE)
    parser.add_argument('--smoke', action='store_true', help='Two levels, 3 alignments each: checks the setup')
    parser.add_argument('--method', choices=('sift', 'learned'), help='Measure one method only')
    parser.add_argument('--no-system-monitor', action='store_true')
    parser.add_argument('--report-only', type=Path, metavar='DIRECTORY')
    parser.add_argument('--stale-resume-ms', type=float, default=rat.DEFAULT_STALE_RESUME_MS, metavar='MS',
                        help='Watchdog hold-and-resume window (default: 2000 ms). 0 = stop, same as --watchdog-stop')
    parser.add_argument('--watchdog-stop', action='store_true',
                        help='A freshness trip ends the alignment (original behaviour) instead of hold and resume')
    args = parser.parse_args(argv)
    if args.watchdog_stop:
        args.stale_resume_ms = 0.0
    if not (0 <= args.stale_resume_ms <= 10000):
        parser.error('--stale-resume-ms must be between 0 and 10000')
    if args.report_only:
        return report_only(args.report_only)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
