"""Predeclared acceptance campaign. Production controller and worker are unchanged."""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'outputs/visual-servoing-simulation'
sys.path.insert(0, str(APP))
import binomial_ci  # noqa: E402
import safety_metrics  # noqa: E402


def write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    # Windows readers/AV may briefly hold a destination without FILE_SHARE_DELETE.
    # Bounded retries occur only in the inactive harness, never in the control loop.
    for attempt in range(40):
        try:
            temporary.replace(path)
            break
        except PermissionError:
            if attempt == 39: raise
            time.sleep(.05)


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def dist(values):
    import numpy as np
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite measurement')
    return dict(count=len(a), mean=float(a.mean()), **dict(zip(
        ('p95', 'p99', 'max'), map(float, np.percentile(a, [95, 99, 100])))))


def evaluate(result, config):
    c = config['criteria']; failures = []
    def require(ok, message):
        if not ok: failures.append(message)
    if 'required_success_rate' in c:
        # The lower confidence bound, not the observed rate, must reach the requirement.
        aligned = result.get('alignments', round(result['success_rate'] * result['attempts']))
        require(binomial_ci.meets_required(aligned, result['attempts'], c['required_success_rate'],
                                           c.get('confidence', binomial_ci.DEFAULT_CONFIDENCE)), 'alignment success rate')
        if c.get('max_failed_alignments') is not None:
            require(result['attempts'] - aligned <= c['max_failed_alignments'], 'failed alignments')
    else:  # Campaigns declared before confidence intervals: observed rate.
        require(result['attempts'] > 0 and result['success_rate'] >= c['success_rate'], 'alignment success rate')
    for key in ('freshness_trips', 'control_misses', 'unsafe_motion', 'post_stop_motion'):
        require(result[key] <= c[key], key)
    for key in ('contacts', 'unlatched_stops', 'telemetry_lost', 'runtime_errors', 'worker_restarts', 'missing_endpoints', 'continuity_gaps'):
        require(result.get(key, 0) == 0, key)
    require(result['duration_s'] >= config['session_seconds'], 'session duration')
    for i in range(len(config['poses_degrees'])):
        require(result['pose_attempts'].get(str(i), 0) >= c['minimum_attempts_per_pose_per_session'], f'pose {i} coverage')
    # Safety envelope (REQ-11 to REQ-13) when the session recorded it; older sessions keep their verdicts.
    for message in safety_metrics.failures(result.get('safety_metrics') or []):
        require(False, message)
    for name, quantile, limit in (
        ('capture_ms', 'p99', 'capture_p99_ms'), ('capture_ms', 'max', 'capture_max_ms'),
        ('processing_ms', 'p95', 'processing_p95_ms'), ('processing_ms', 'p99', 'processing_p99_ms'),
        ('processing_ms', 'max', 'processing_max_ms'),
        ('position_mm', 'max', 'position_mm'), ('orientation_deg', 'max', 'orientation_deg')):
        value = result[name]
        require(value is not None and value[quantile] <= c[limit], limit)
    for pose in range(len(config['poses_degrees'])):
        pair = result['by_pose_first_later'].get(str(pose), {})
        first, later = pair.get('first'), pair.get('later')
        require(bool(first and later) and later['p99'] <= first['p99'] * c['later_to_first_p99_ratio'], f'pose {pose} later/first p99')
    return failures


def summarize(raw, ending, attempts, duration, config, score=True):
    sensors, commands, events = [], [], []
    max_gap = 0.; continuity_gaps = 0
    with gzip.open(raw, 'rt', encoding='utf-8') as stream:
        for line in stream:
            chunk = json.loads(line)
            sensors.extend(chunk['sensor_frames']); commands.extend(chunk['frames']); events.extend(chunk['events'])
            max_gap = max(max_gap, max(chunk['loop_gaps_ms'], default=0))
            continuity_gaps += sum(gap > 1000 for gap in chunk['loop_gaps_ms'])
    assert len({f['sequence'] for f in sensors}) == len(sensors), 'Duplicate sensor telemetry'
    bygen = {a['generation']: a for a in attempts}
    started_generations = {e['generation'] for e in events if e['kind'] == 'align'}
    missing = started_generations - set(bygen)
    active = [f for f in sensors if f['capture_active'] and not f['stage_ms'].get('cache_hit', False)]
    first_generation = attempts[0]['generation'] if attempts else None
    first_by_pose = {}
    for a in attempts: first_by_pose.setdefault(a['pose_index'], a['generation'])
    paired = {}
    for pose, generation in first_by_pose.items():
        selected = [f for f in active if f['generation'] in bygen and bygen[f['generation']]['pose_index'] == pose]
        paired[str(pose)] = {label: dist([1000*(f['finished_s']-f['rendered_s']) for f in selected
                                        if (f['generation'] == generation) == (label == 'first')])
                            for label in ('first', 'later')}
    scores = []
    if score:
        import mujoco
        from simulation import Simulation
        from accuracy import pose_accuracy
        with Simulation(render=False) as sim:
            goal_camera, goal_tool = sim.camera_pose(), sim.tool_pose()
            for attempt in attempts:
                sim.data.qpos[:] = attempt['qpos']; mujoco.mj_forward(sim.model, sim.data)
                attempt['physical_accuracy'] = {
                    'camera': pose_accuracy(sim.camera_pose(), goal_camera),
                    'tool': pose_accuracy(sim.tool_pose(), goal_tool)}
                scores.extend(attempt['physical_accuracy'].values())
    counts = ending['counts']
    assert len(sensors) == counts['sensor_received'], 'Lost received-frame telemetry'
    assert len(commands) == counts['accepted'], 'Lost accepted-command telemetry'
    result = dict(attempts=len(started_generations), alignments=sum(a['outcome'] == 'converged' for a in attempts),
        success_rate=sum(a['outcome'] == 'converged' for a in attempts)/max(1, len(started_generations)),
        success_ci=binomial_ci.summary(sum(a['outcome'] == 'converged' for a in attempts), len(started_generations),
                                       config['criteria'].get('confidence', binomial_ci.DEFAULT_CONFIDENCE),
                                       config['criteria'].get('required_success_rate')),
        missing_endpoints=len(missing), continuity_gaps=continuity_gaps, maximum_control_tick_gap_ms=max_gap,
        duration_s=duration, pose_attempts=dict(Counter(str(a['pose_index']) for a in attempts)),
        processing_ms=dist([1000*(f['finished_s']-f['rendered_s']) for f in active]),
        capture_ms=dist([f['capture_to_command_ms'] for f in commands]),
        image_error_px=dist([a['error_px'] for a in attempts if a.get('error_px') is not None]),
        position_mm=dist([s['position_error_mm'] for s in scores]),
        orientation_deg=dist([s['orientation_error_deg'] for s in scores]),
        first_processing_ms=dist([1000*(f['finished_s']-f['rendered_s']) for f in active if f['generation'] == first_generation]),
        later_processing_ms=dist([1000*(f['finished_s']-f['rendered_s']) for f in active if f['generation'] != first_generation]),
        by_pose_first_later=paired,
        freshness_trips=sum(e.get('reason') == 'stale_camera' for e in events),
        control_misses=counts['control_deadline_misses'], unsafe_motion=counts['unsafe_motion_ticks'],
        post_stop_motion=counts['post_stop_motion_ticks'], contacts=counts['contacts'],
        unlatched_stops=sum(not a['stop_stayed_latched'] for a in attempts),
        telemetry_lost=sum(counts[k] for k in ('sensor_rows_evicted','command_rows_evicted','loop_rows_evicted')) + ending['cycle_spikes_evicted'] + ending['gc_events_evicted'],
        runtime_errors=int(bool(ending.get('error') or ending.get('harness_error') or ending['worker_alive'])),
        worker_restarts=int(len({a['worker_pid'] for a in attempts}) > 1),
        counts=counts, worker_counts=ending['worker_counts'], execution_settings=ending['execution_settings'],
        raw_sha256=digest(raw), samples=dict(sensor=len(sensors), uncached_active=len(active), commands=len(commands)),
        safety=ending.get('safety'),
        safety_metrics=None if not ending.get('safety') else safety_metrics.evaluate(ending['safety']))
    result['failures'] = evaluate(result, config)
    return result


def run_case(mode, seconds, config, directory, identifier):
    import numpy as np
    import simulation
    from realtime import RealtimeSession, RuntimeConfig
    from run_latency_stress import wait_for
    original = simulation.Simulation
    pose_log = []

    class CampaignSimulation(original):
        def reset(self, offset_degrees=None):
            # Only explicitly requested offset resets are varied. Home/reference capture stays unchanged.
            if offset_degrees is not None:
                index = len(pose_log) % len(config['poses_degrees'])
                offset_degrees = config['poses_degrees'][index]
                pose_log.append(index)
            return super().reset(offset_degrees)

    class CampaignSession(RealtimeSession):
        def __init__(self, *args, **kwargs):
            self.flush_requested = threading.Event(); self.flush_ready = threading.Event(); self.chunk = None
            super().__init__(*args, **kwargs)

        def take_chunk(self):
            chunk = {}
            for source, name in (('sensor_rows','sensor_frames'),('frame_rows','frames'),('events','events'),('loop_gaps','loop_gaps_ms')):
                rows = getattr(self, source); chunk[name] = list(rows); rows.clear()
            for source, name in (('cycle_probe','cycle_spikes'),('gc_probe','gc_events')):
                probe = getattr(self, source)
                chunk[name] = []
                if probe is not None:
                    while probe.rows: chunk[name].append(probe.rows.popleft())
            return chunk

        def _publish(self, **state):
            super()._publish(**state)
            # Controller owns these buffers. Copy only at its normal inactive end-of-tick publish.
            if 'qpos' in state and 'command' in state and not state.get('active', True) and self.flush_requested.is_set():
                self.chunk = self.take_chunk(); self.flush_requested.clear(); self.flush_ready.set()

    simulation.Simulation = CampaignSimulation
    # The campaign's criteria predate hold-and-resume: it measures the stop response.
    session = CampaignSession(mode, RuntimeConfig(**config['runtime'], stale_resume_s=0), offset=config['poses_degrees'][0],
                              diagnostics=True, telemetry_capacity=8192)
    attempts = []; ready_at = None; error = None; worker_pid = None
    raw = directory / 'traces' / (identifier + '.jsonl.gz')
    started = time.perf_counter()
    try:
        with gzip.open(raw, 'wt', encoding='utf-8', compresslevel=1) as output:
            try:
                session.start()
                if not session.ready.wait(125): raise TimeoutError('Initialization timeout')
                state = session.snapshot()
                if state.get('error'): raise RuntimeError(state['error'])
                ready_at = time.perf_counter(); worker_pid = session.worker.pid
                if mode == 'learned' and state['execution_settings'].get('device') != 'cuda':
                    raise RuntimeError('Learned did not select CUDA')
                while True:
                    generation = state.get('generation', 1)
                    state = wait_for(session, lambda s: s.get('generation', 0) >= generation and s.get('ready')
                                     and not s.get('active') and s['status'] != 'idle', 123)
                    if state.get('error'): raise RuntimeError(state['error'])
                    time.sleep(config['post_stop_seconds'])
                    settled = session.snapshot()
                    attempts.append(dict(index=len(attempts), generation=state['generation'], outcome=state['status'],
                        pose_index=pose_log[-1], qpos=settled['qpos'].tolist(), error_px=state.get('error_px'),
                        stop_metrics=state.get('stop_metrics'), worker_pid=session.worker.pid,
                        stop_stayed_latched=bool(not settled['active'] and settled['status'] == state['status'] and not np.any(settled['command']))))
                    session.flush_ready.clear(); session.flush_requested.set()
                    if not session.flush_ready.wait(5): raise TimeoutError('Inactive telemetry flush timeout')
                    output.write(json.dumps(session.chunk, allow_nan=False) + '\n'); output.flush(); session.chunk = None
                    # Disk I/O occurs here, while stopped, with no control or image queue expansion.
                    elapsed = time.perf_counter() - ready_at
                    write(directory / 'progress.json', dict(session=identifier, mode=mode, elapsed_s=elapsed,
                          attempts=len(attempts), alignments=sum(a['outcome']=='converged' for a in attempts),
                          counters=dict(settled['counters']), worker_pid=worker_pid))
                    if any(settled['counters'][k] for k in ('unsafe_motion_ticks','post_stop_motion_ticks','contacts')) or not attempts[-1]['stop_stayed_latched']:
                        raise RuntimeError('Safety violation; campaign stopped')
                    if elapsed >= seconds: break
                    next_pose = len(pose_log) % len(config['poses_degrees'])
                    desired = np.asarray(settled['home_qpos']) + np.deg2rad(config['poses_degrees'][next_pose])
                    session.command('offset')
                    state = wait_for(session, lambda s: s['status']=='idle' and not s['active'] and np.allclose(s['qpos'], desired, atol=1e-8, rtol=0), 5)
                    previous = state['generation']; session.command('align')
                    state = wait_for(session, lambda s: s.get('generation',0)>previous, 5)
                session.command('stream')
                wait_for(session, lambda s: not s.get('stream',True) and s.get('sensor_available',False), 5)
            except Exception as exc:
                error = traceback.format_exc(); session.command('stop')
            finally:
                session.close()
                output.write(json.dumps(session.take_chunk(), allow_nan=False) + '\n')
    finally:
        simulation.Simulation = original
    ending = session.report(); ending['harness_error'] = error
    duration = 0 if ready_at is None else time.perf_counter() - ready_at
    result = summarize(raw, ending, attempts, duration, config)
    result.update(id=identifier, mode=mode, worker_pid=worker_pid, harness_error=error,
                  elapsed_s=time.perf_counter()-started)
    write(directory / (identifier + '.json'), result)
    write(directory / 'traces' / (identifier + '-attempts.json'), attempts)
    write(directory / 'traces' / (identifier + '-ending.json'), ending)
    return result


def report(directory, results, complete, smoke=False, interrupted=None):
    status = 'SMOKE_ONLY' if smoke else ('PASS' if complete and all(not r['failures'] for r in results) else 'FAIL')
    if interrupted and not smoke: status='FAIL'
    write(directory/'verdict.json', dict(status=status, complete=complete, sessions=results, interrupted_sessions=interrupted or []))
    def triple(d): return 'missing' if d is None else ' / '.join(f'{d[k]:.1f}' for k in ('p95','p99','max'))
    lines = ['# Acceptance campaign: '+status, '',
             'Latencies are p95 / p99 / max in ms. Physical maxima include all attempts, both camera and tool frames.', '',
             'Aligned shows the 95% Clopper-Pearson (exact) confidence interval for the true success rate: all '
             'alignments succeeding is not proof of 100% reliability.', '',
             '| Session | Aligned | Position / angle max (mm / deg) | Capture latency | Processing latency | Freshness / control misses | Unsafe / post-stop | Verdict |',
             '|---|---:|---:|---|---|---:|---:|---|']
    for r in results:
        pos = r['position_mm']; ang = r['orientation_deg']
        physical = 'missing' if not pos or not ang else f"{pos['max']:.3f} / {ang['max']:.3f}"
        lines.append(f"| {r['id']} | {r['alignments']}/{r['attempts']} {binomial_ci.format_interval(r['alignments'], r['attempts'])} | {physical} | {triple(r['capture_ms'])} | {triple(r['processing_ms'])} | {r['freshness_trips']} / {r['control_misses']} | {r['unsafe_motion']} / {r['post_stop_motion']} | {'FAIL' if r['failures'] else 'PASS'} |")
    lines += ['', '| Session | First processing | Later processing | Failed criteria |', '|---|---|---|---|']
    for r in results:
        lines.append(f"| {r['id']} | {triple(r['first_processing_ms'])} | {triple(r['later_processing_ms'])} | {', '.join(r['failures']) or 'none'} |")
    lines += ['', 'Safety envelope per session (extremes over every 2 ms physics step; a FAIL fails the session; '
              'simulation values):', '']
    lines += safety_metrics.table_lines([(r['id'], r.get('safety_metrics')) for r in results])
    total, aligned = sum(r['attempts'] for r in results), sum(r['alignments'] for r in results)
    if total:
        lines += ['', f"All sessions together: {binomial_ci.format_rate(aligned, total)}. {binomial_ci.interpretation(aligned, total)}"]
    lines += ['', 'All criteria are fixed in manifest.json before execution. No thresholds are relaxed after failures. Per-pose first/later comparisons, versions, source hashes and counters are in the JSON artifacts; raw timing and endpoint evidence is local under traces. Interrupted/incomplete campaigns cannot pass.']
    if interrupted:
        lines += ['', 'Interrupted attempts remain failures; replacement full sessions do not erase them.']
        for r in interrupted:
            lines.append(f"- Interrupted {r['id']}: {r['alignments']}/{r['attempts']}, {r['missing_endpoints']} missing endpoints, {r['control_misses']} control misses, maximum control gap {r['maximum_control_tick_gap_ms']:.1f} ms. Evidence: {r['source_directory']}.")
    (directory/'REPORT.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return status


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--config', type=Path, default=ROOT/'tools/acceptance_campaign.json')
    p.add_argument('--smoke', action='store_true', help='30-second sessions; never an acceptance PASS')
    p.add_argument('--allow-code-changes', action='store_true', help='Future regression runs: allow changed Python source, still pin non-code inputs and dependency versions')
    p.add_argument('--continue-from', type=Path, help='Carry fully completed sessions; rerun interrupted session slots from zero and retain all failures')
    args = p.parse_args(); config = read(args.config)
    assert config['runtime'] == dict(max_age_s=.4, transport_s=.05, max_control_gap_s=.05)
    assert 1800 <= config['session_seconds'] <= 3600 and config['sessions_per_method'] >= 2
    assert config['modes'] == ['natural','learned'] and len(config['poses_degrees']) >= 3
    from run_camera_robustness import fingerprint
    from run_latency_stress import gpu_status, power_status
    from realtime import RuntimeConfig
    from simulation import Simulation
    import ctypes
    current = fingerprint()
    baseline = APP/'results/latency/20260926-base-executable-settings/manifest.json'
    saved = read(baseline)['fingerprint']
    if args.allow_code_changes:
        assert current['runtime'] == saved['runtime'], 'Dependency/runtime versions changed'
        assert {k:v for k,v in current['sha256'].items() if not k.endswith('.py')} == {k:v for k,v in saved['sha256'].items() if not k.endswith('.py')}, 'Scene/model/configuration inputs changed'
    else:
        assert current == saved, 'Validated production configuration changed; use --allow-code-changes only for intentional future source regression tests'
    with Simulation(render=False) as sim:
        for pose in config['poses_degrees']: sim.reset(pose)
    executable = ctypes.create_unicode_buffer(32768)
    ctypes.windll.kernel32.GetModuleFileNameW(None, executable, len(executable))
    args.output.mkdir(parents=True, exist_ok=False); (args.output/'traces').mkdir()
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(), config=config,
        runtime_config=asdict(RuntimeConfig(**config['runtime'], stale_resume_s=0)), fingerprint=current,
        runner_sha256=digest(Path(__file__)), config_sha256=digest(args.config), baseline_sha256=digest(baseline),
        executable=sys.executable, windows_executable=executable.value, base_executable=sys._base_executable,
        virtual_environment=sys.prefix, gpu=gpu_status(), smoke_only=args.smoke,
        warmup='Unchanged sensor_worker initialization and single initial observe; no worker restart within a session.',
        telemetry='Bounded runtime buffers copied by control owner while inactive; gzip disk serialization between alignments. Worker/context and streaming remain alive.',
        pose_variation='Harness-only Simulation reset adapter; cycles declared joint offsets, leaving reference/home and production files unchanged.')
    inventory = subprocess.run(['nvidia-smi','--query-gpu=index,name,uuid,driver_version','--format=csv,noheader'], capture_output=True,text=True,timeout=10)
    assert inventory.returncode == 0 and len(inventory.stdout.strip().splitlines()) == 1, 'This validated campaign requires the single NVIDIA GPU environment'
    assert not os.environ.get('CUDA_VISIBLE_DEVICES'), 'Unexpected CUDA device override'
    manifest['selected_cuda_gpu'] = dict(logical_device='cuda:0', inventory=inventory.stdout.strip(), verified_by='Single NVIDIA device inventory plus each Learned worker execution_settings.device == cuda')
    manifest['model_weights_sha256'] = {p.relative_to(APP).as_posix():digest(p) for p in (APP/'models').rglob('*') if p.is_file()}
    manifest['allow_code_changes'] = args.allow_code_changes
    write(args.output/'manifest.json', manifest)
    results = []; interrupted = []; monitor = None; complete = False
    if args.continue_from:
        prior_manifest=read(args.continue_from/'manifest.json')
        assert prior_manifest['config']==config and prior_manifest['fingerprint']==current, 'Cannot continue changed inputs or criteria'
        assert not args.smoke and not prior_manifest['smoke_only'], 'Smoke runs cannot be continued as acceptance'
        prior=read(args.continue_from/'verdict.json')
        interrupted.extend(prior.get('interrupted_sessions', []))
        for old in prior['sessions']:
            evidence_directory=Path(old.get('source_directory',args.continue_from))
            raw=evidence_directory/'traces'/(old['id']+'.jsonl.gz')
            assert digest(raw)==old['raw_sha256'], 'Prior raw evidence changed'
            refreshed=summarize(raw,read(evidence_directory/'traces'/(old['id']+'-ending.json')),
                                read(evidence_directory/'traces'/(old['id']+'-attempts.json')),old['duration_s'],config)
            refreshed.update(id=old['id'],mode=old['mode'],worker_pid=old['worker_pid'], source_directory=str(evidence_directory.resolve()))
            if refreshed['duration_s'] >= config['session_seconds'] and not any(refreshed[k] for k in ('runtime_errors','missing_endpoints','continuity_gaps','worker_restarts')):
                results.append(refreshed)
            else:
                interrupted.append(refreshed)
        manifest['continued_from']=dict(directory=str(args.continue_from.resolve()), manifest_sha256=digest(args.continue_from/'manifest.json'),
            carried_sessions=[r['id'] for r in results], interrupted_sessions=[r['id'] for r in interrupted])
        write(args.output/'manifest.json',manifest)
    # Request system availability, not GPU work or a clock/power-policy change.
    previous_execution_state=ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)
    assert previous_execution_state, 'Cannot request uninterrupted system availability'
    manifest['system_awake_request']=dict(flags='ES_CONTINUOUS | ES_SYSTEM_REQUIRED', previous_state=previous_execution_state,
        scope='Campaign main thread; restored on exit. Does not prevent manual sleep or run GPU keepalive work.')
    write(args.output/'manifest.json',manifest)
    try:
        with (args.output/'traces/gpu.csv').open('w', encoding='utf-8') as log:
            monitor = subprocess.Popen(['nvidia-smi','--query-gpu=timestamp,name,driver_version,pstate,clocks.sm,clocks.mem,utilization.gpu,temperature.gpu,power.draw,memory.used',
                                        '--format=csv,nounits','--loop-ms=250'], stdout=log, stderr=subprocess.STDOUT,
                                       creationflags=subprocess.CREATE_NO_WINDOW)
            for i in range(1 if args.smoke else config['sessions_per_method']):
                for mode in config['modes']:
                    assert fingerprint() == current, 'Production source changed'
                    assert digest(Path(__file__)) == manifest['runner_sha256'] and digest(args.config) == manifest['config_sha256'], 'Runner or criteria changed during campaign'
                    assert power_status()['ACLineStatus'] == 1, 'AC power required'
                    identifier = f'{mode}-{i:02d}'
                    if any(r['id']==identifier for r in results):
                        print('CARRIED '+identifier,flush=True);continue
                    print('START '+identifier, flush=True)
                    result = run_case(mode, 30 if args.smoke else config['session_seconds'], config, args.output, identifier)
                    results.append(result); report(args.output, results, False, args.smoke, interrupted)
                    print(f"END {identifier}: {result['alignments']}/{result['attempts']} failures={result['failures']}", flush=True)
                    assert monitor.poll() is None, 'GPU telemetry stopped'
                    if result['unsafe_motion'] or result['post_stop_motion'] or result['unlatched_stops'] or result['runtime_errors']:
                        raise RuntimeError('Safety/runtime error: remaining campaign cancelled')
            assert fingerprint() == current, 'Production source changed'
            assert manifest['model_weights_sha256'] == {p.relative_to(APP).as_posix():digest(p) for p in (APP/'models').rglob('*') if p.is_file()}, 'Model weights changed'
            complete = True
    except Exception as exc:
        write(args.output/'campaign-error.json', dict(error=repr(exc)))
        print(repr(exc), flush=True)
    finally:
        ctypes.windll.kernel32.SetThreadExecutionState(previous_execution_state)
        if monitor is not None:
            if monitor.poll() is None: monitor.terminate()
            monitor.wait(10)
    status = report(args.output, results, complete, args.smoke, interrupted)
    print(status, flush=True)
    return 0 if status == 'PASS' or (args.smoke and complete) else 1


if __name__ == '__main__':
    raise SystemExit(main())
