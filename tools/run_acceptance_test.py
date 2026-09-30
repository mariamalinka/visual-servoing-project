"""SIFT / Learned GPU acceptance test.

Runs fixed starting poses with a fresh sensor worker per pose, then one reused
worker per method for a sustained run. Production source, settings and the
controller are used unchanged; this harness only issues the existing Offset/Align
commands, varies which declared offset an explicit Offset reset uses, and reads
the runtime's own telemetry while the robot is stopped.

Exit codes: 0 PASS (or a completed --smoke run), 1 FAIL, 2 INVALID/INCOMPLETE.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import gzip
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import threading
import time
import traceback

import acceptance_environment as env_monitor

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'outputs' / 'visual-servoing-simulation'
sys.path.insert(0, str(APP))
import binomial_ci  # noqa: E402  (lives in the application directory)
ENVIRONMENT_MODULE = Path(env_monitor.__file__).resolve()

DEFAULT_CONFIG = ROOT / 'tools' / 'acceptance_test.json'
LONG_CONFIG = ROOT / 'tools' / 'acceptance_test_long.json'
# Allowed sustained reused-worker durations (seconds) for each declared profile.
PROFILES = {'standard': (600, 900, '10-15 minutes'), 'long': (1500, 1650, '25-27.5 minutes, keeping the whole test under 1 hour')}
DEFAULT_BASELINE = APP / 'results' / 'latency' / '20260926-base-executable-settings' / 'manifest.json'
DEFAULT_RESULTS = APP / 'results' / 'acceptance-test'
# Reviewed, intentional changes to protected files, each pinned to exact before/after hashes.
APPROVED_CHANGES = ROOT / 'tools' / 'approved_changes.json'
# The production values this test certifies. They are asserted, never overridden.
PRODUCTION_RUNTIME = dict(max_age_s=0.4, transport_s=0.05, max_control_gap_s=0.05)
# Controller, calibration, scene, safety and transport logic. A difference from the
# validated baseline makes the run INVALID instead of silently testing other code.
PROTECTED = ('control.py', 'recovery.py', 'startup_search.py', 'adaptive_gain.py', 'precision.py',
             'calibration.py', 'realtime.py', 'simulation.py', 'collision.py', 'joint_limits.py',
             'motion_path.py', 'scene.xml', 'config.json', 'precision_config.json', 'recovery_config.json',
             'collision_config.json', 'joint_limit_config.json', 'startup_search_config.json',
             'camera_timing_config.json', 'actuator.py', 'actuator_config.json', 'assets/', 'reference/')
SMOKE = dict(poses=2, alignments_per_pose=1, sustained_seconds=40, minimum_attempts_per_pose=1)


# ----------------------------------------------------------------------------- utilities

def write(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for attempt in range(40):  # Windows readers/antivirus can briefly hold the target.
        try:
            temporary.replace(path)
            return
        except PermissionError:
            if attempt == 39:
                raise
            time.sleep(.05)


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def dist(values):
    import numpy as np
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite measurement')
    return dict(count=int(a.size), mean=float(a.mean()), **dict(zip(
        ('p50', 'p95', 'p99', 'max'), map(float, np.percentile(a, [50, 95, 99, 100])))))


def ratio(later, first, key='p99'):
    if not later or not first or first[key] <= 0:
        return None
    return later[key] / first[key]


def is_protected(name):
    return any(name == p or (p.endswith('/') and name.startswith(p)) for p in PROTECTED)


def label(config, mode):
    return config['method_labels'][mode]


# ----------------------------------------------------------------------------- environment

GPU_PROBE = r'''
import json, os, importlib.metadata as m
out = dict(env={k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "CUDA_DEVICE_ORDER", "MUJOCO_GL")})
for name in ("torch", "torchvision", "lightglue", "kornia"):
    try: out[name] = m.version(name)
    except m.PackageNotFoundError: out[name] = None
try:
    import torch
    out["torch_cuda"] = torch.version.cuda
    out["cudnn"] = torch.backends.cudnn.version() if torch.backends.cudnn.is_available() else None
    out["cuda_available"] = torch.cuda.is_available()
    if out["cuda_available"]:
        index = torch.cuda.current_device(); p = torch.cuda.get_device_properties(index)
        out["selected"] = dict(index=index, name=p.name, uuid=str(getattr(p, "uuid", "")) or None,
            capability=f"{p.major}.{p.minor}", total_memory_mib=p.total_memory // 2**20,
            multiprocessors=p.multi_processor_count)
        out["device_count"] = torch.cuda.device_count()
except Exception as exc:
    out["error"] = f"{type(exc).__name__}: {exc}"
print(json.dumps(out))
'''


def probe_cuda():
    """Separate short-lived process: no CUDA context is left in the harness."""
    try:
        run = subprocess.run([sys.executable, '-c', GPU_PROBE], capture_output=True, text=True, timeout=180)
        return json.loads(run.stdout.strip().splitlines()[-1]) if run.returncode == 0 else dict(error=run.stderr[-2000:])
    except (OSError, subprocess.TimeoutExpired, ValueError, IndexError) as exc:
        return dict(error=repr(exc))


def nvidia_inventory():
    try:
        run = subprocess.run(['nvidia-smi', '--query-gpu=index,name,uuid,driver_version,pci.bus_id,power.limit',
                              '--format=csv,noheader'], capture_output=True, text=True, timeout=15)
        rows = [dict(zip(('index', 'name', 'uuid', 'driver', 'pci_bus_id', 'power_limit'),
                         [c.strip() for c in line.split(',')])) for line in run.stdout.strip().splitlines()]
        return dict(returncode=run.returncode, gpus=rows)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(error=repr(exc), gpus=[])


def select_gpu(probe, inventory):
    """Match the CUDA device torch selects with nvidia-smi by UUID."""
    selected = probe.get('selected')
    if not selected:
        return None
    uuid = (selected.get('uuid') or '').lower().removeprefix('gpu-')
    match = [g for g in inventory.get('gpus', []) if g['uuid'].lower().removeprefix('gpu-') == uuid] if uuid else []
    return dict(cuda_index=selected['index'], name=selected['name'], uuid=selected.get('uuid'),
                capability=selected['capability'], total_memory_mib=selected['total_memory_mib'],
                nvidia_smi=match[0] if len(match) == 1 else None, uuid_matched=len(match) == 1)


def power_status():
    if os.name != 'nt':
        return None
    from run_latency_stress import power_status as status
    return status()


def process_image(pid):
    try:
        if os.name == 'nt':
            import ctypes
            from ctypes import wintypes
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.OpenProcess.restype = wintypes.HANDLE
            handle = kernel.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
            if not handle:
                return None
            try:
                size = wintypes.DWORD(32768); buffer = ctypes.create_unicode_buffer(size.value)
                ok = kernel.QueryFullProcessImageNameW(handle, 0, buffer, ctypes.byref(size))
                return buffer.value if ok else None
            finally:
                kernel.CloseHandle(handle)
        return os.readlink(f'/proc/{pid}/exe')
    except OSError:
        return None


def gpu_preferences(paths):
    """Read-only Windows per-executable graphics preference (UserGpuPreferences)."""
    if os.name != 'nt':
        return None
    import winreg
    found = {}
    try:
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER, r'Software\Microsoft\DirectX\UserGpuPreferences') as key:
            for path in sorted({p for p in paths if p}):
                try:
                    found[path] = winreg.QueryValueEx(key, path)[0]
                except OSError:
                    found[path] = None
    except OSError:
        return {p: None for p in paths if p}
    return found


def this_executable():
    if os.name == 'nt':
        import ctypes
        buffer = ctypes.create_unicode_buffer(32768)
        ctypes.windll.kernel32.GetModuleFileNameW(None, buffer, len(buffer))
        return buffer.value
    return os.path.realpath(sys.executable)


def runtime_versions():
    versions = dict(python=sys.version, implementation=platform.python_implementation(),
                    platform=platform.platform(), machine=platform.machine(), processor=platform.processor())
    for name in ('numpy', 'opencv-python', 'opencv-python-headless', 'mujoco', 'torch', 'torchvision',
                 'lightglue', 'kornia', 'matplotlib', 'pillow'):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


# ----------------------------------------------------------------------------- one session

def run_session(mode, pose_ids, config, directory, identifier, *, max_attempts=None, seconds=None):
    """One sensor worker (process + GPU context) for all its alignments.

    Declared poses cycle in order on each explicit Offset reset. Telemetry is copied
    by the control owner at its normal inactive end-of-tick publish, after the
    post-stop observation; gzip I/O happens here while the robot is stopped.
    """
    import numpy as np
    import simulation
    from realtime import RealtimeSession, RuntimeConfig
    from run_latency_stress import wait_for

    poses = [config['poses_degrees'][i] for i in pose_ids]
    original = simulation.Simulation
    pose_log = []

    class TestSimulation(original):
        def reset(self, offset_degrees=None):
            # Only explicit offset resets change; home and reference capture are untouched.
            if offset_degrees is not None:
                k = len(pose_log) % len(poses)
                offset_degrees = poses[k]
                pose_log.append(pose_ids[k])
            return super().reset(offset_degrees)

    class TestSession(RealtimeSession):
        def __init__(self, *args, **kwargs):
            self.flush_requested = threading.Event(); self.flush_ready = threading.Event(); self.chunk = None
            super().__init__(*args, **kwargs)

        def take_chunk(self):
            chunk = {}
            for source, name in (('sensor_rows', 'sensor_frames'), ('frame_rows', 'frames'),
                                 ('events', 'events'), ('loop_gaps', 'loop_gaps_ms')):
                rows = getattr(self, source); chunk[name] = list(rows); rows.clear()
            chunk['events_full'] = len(chunk['events']) >= (self.events.maxlen or 10**9)
            return chunk

        def _publish(self, **state):
            super()._publish(**state)
            if ('qpos' in state and 'command' in state and not state.get('active', True)
                    and self.flush_requested.is_set()):
                self.chunk = self.take_chunk(); self.flush_requested.clear(); self.flush_ready.set()

    simulation.Simulation = TestSimulation
    runtime = RuntimeConfig(**config['runtime'])
    session = TestSession(mode, runtime, offset=poses[0], diagnostics=True, telemetry_capacity=8192)
    attempts, error, worker_pid, worker_exe, ready_at, execution, anchor = [], None, None, None, None, {}, None
    raw = directory / 'traces' / (identifier + '.jsonl.gz')
    started = time.perf_counter(); started_utc = datetime.now(timezone.utc).isoformat()
    try:
        with gzip.open(raw, 'wt', encoding='utf-8', compresslevel=1) as output:
            try:
                session.start()
                if not session.ready.wait(125):
                    raise TimeoutError('Sensor initialization exceeded 125 s')
                state = session.snapshot()
                if state.get('error'):
                    raise RuntimeError(state['error'])
                ready_at = time.perf_counter(); worker_pid = session.worker.pid
                anchor = dict(perf_s=time.perf_counter(), utc_epoch_s=time.time())
                worker_exe = process_image(worker_pid)
                execution = dict(state.get('execution_settings', {}))
                if mode == 'learned' and execution.get('device') != 'cuda':
                    raise RuntimeError(f"Learned selected {execution.get('device')!r}, not CUDA")
                while True:
                    generation = state.get('generation', 1)
                    state = wait_for(session, lambda s: s.get('generation', 0) >= generation and s.get('ready')
                                     and not s.get('active') and s['status'] != 'idle', 123)
                    if state.get('error'):
                        raise RuntimeError(state['error'])
                    time.sleep(config['post_stop_seconds'])  # Existing post-stop observation window.
                    settled = session.snapshot()
                    attempts.append(dict(index=len(attempts), generation=state['generation'], outcome=state['status'],
                        pose_index=pose_log[-1], qpos=settled['qpos'].tolist(), error_px=state.get('error_px'),
                        worker_pid=session.worker.pid,
                        stop_stayed_latched=bool(not settled['active'] and settled['status'] == state['status']
                                                 and not np.any(settled['command']))))
                    session.flush_ready.clear(); session.flush_requested.set()
                    if not session.flush_ready.wait(5):
                        raise TimeoutError('Inactive telemetry flush timed out')
                    output.write(json.dumps(session.chunk, allow_nan=False) + '\n'); output.flush(); session.chunk = None
                    elapsed = time.perf_counter() - ready_at
                    write(directory / 'progress.json', dict(session=identifier, mode=mode, elapsed_s=round(elapsed, 1),
                          attempts=len(attempts), aligned=sum(a['outcome'] == 'converged' for a in attempts),
                          last_outcome=state['status'], worker_pid=worker_pid, counters=dict(settled['counters'])))
                    counters = settled['counters']
                    if (counters['unsafe_motion_ticks'] or counters['post_stop_motion_ticks'] or counters['contacts']
                            or counters.get('paused_motion_ticks', 0)
                            or not attempts[-1]['stop_stayed_latched']):
                        raise RuntimeError('Safety violation observed; test stopped')
                    if (max_attempts is not None and len(attempts) >= max_attempts) or (
                            seconds is not None and elapsed >= seconds):
                        break
                    desired = np.asarray(settled['home_qpos']) + np.deg2rad(poses[len(pose_log) % len(poses)])
                    session.command('offset')
                    state = wait_for(session, lambda s: s['status'] == 'idle' and not s['active']
                                     and np.allclose(s['qpos'], desired, atol=1e-8, rtol=0), 5)
                    previous = state['generation']; session.command('align')
                    state = wait_for(session, lambda s: s.get('generation', 0) > previous, 5)
                session.command('stream')  # Stop acquiring and drain the one outstanding result.
                wait_for(session, lambda s: not s.get('stream', True) and s.get('sensor_available', False), 5)
            except BaseException:
                error = traceback.format_exc(); session.command('stop')
                if not isinstance(sys.exc_info()[1], Exception):
                    session.close(); raise
            finally:
                if not session.closed.is_set():
                    session.close()
                output.write(json.dumps(session.take_chunk(), allow_nan=False) + '\n')
    finally:
        simulation.Simulation = original
    ending = session.report()
    for key in ('frames', 'sensor_frames', 'events', 'cycle_spikes', 'gc_events'):
        ending.pop(key, None)  # Already streamed to the raw trace; keep the ending file small.
    for record in ending.get('motion') or []:
        record.pop('profile', None)  # Physical stop records stay; their speed traces are not needed here.
    ending.update(harness_error=error, worker_executable=worker_exe, started_utc=started_utc, clock_anchor=anchor,
                  duration_s=0.0 if ready_at is None else time.perf_counter() - ready_at,
                  elapsed_s=time.perf_counter() - started, execution_settings=execution or ending.get('execution_settings'))
    write(directory / 'traces' / (identifier + '-attempts.json'), attempts)
    write(directory / 'traces' / (identifier + '-ending.json'), ending)
    return raw, attempts, ending


# ----------------------------------------------------------------------------- analysis

def score_physical(attempts):
    import mujoco
    from simulation import Simulation
    from accuracy import pose_accuracy
    with Simulation(render=False) as sim:
        goal_camera, goal_tool = sim.camera_pose(), sim.tool_pose()
        for attempt in attempts:
            sim.data.qpos[:] = attempt['qpos']; mujoco.mj_forward(sim.model, sim.data)
            attempt['physical_accuracy'] = dict(camera=pose_accuracy(sim.camera_pose(), goal_camera),
                                                tool=pose_accuracy(sim.tool_pose(), goal_tool))


def summarize(raw, attempts, ending, meta, config, score=True):
    """Everything is recomputed from the raw trace, so --report-only reproduces it exactly."""
    sensors, commands, events, gaps, events_full = [], [], [], [], False
    with gzip.open(raw, 'rt', encoding='utf-8') as stream:
        for line in stream:
            chunk = json.loads(line)
            sensors += chunk['sensor_frames']; commands += chunk['frames']; events += chunk['events']
            gaps += chunk['loop_gaps_ms']; events_full |= bool(chunk.get('events_full'))
    counts = ending['counts']
    by_generation = {a['generation']: a for a in attempts}
    aligns = {e['generation']: e['at_s'] for e in events if e['kind'] == 'align'}
    stops = {}
    for e in events:
        if e['kind'] == 'stop':
            stops.setdefault(e['generation'], e)
    first_generation = attempts[0]['generation'] if attempts else None
    for a in attempts:
        stop = stops.get(a['generation'])
        a['stop_reason'] = None if stop is None else stop['reason']
        a['alignment_s'] = None if stop is None or a['generation'] not in aligns else stop['at_s'] - aligns[a['generation']]

    def processing(frame):
        return 1000 * (frame['finished_s'] - frame['rendered_s'])
    active = [f for f in sensors if f['capture_active'] and not f['stage_ms'].get('cache_hit', False)]
    first = [f for f in active if f['generation'] == first_generation]
    later = [f for f in active if f['generation'] != first_generation and f['generation'] in by_generation]
    t0 = aligns.get(first_generation)
    window = config['sustained']['drift_window_seconds']
    windows = {}
    if t0 is not None:
        for f in active:
            windows.setdefault(int((f['captured_s'] - t0) // window), []).append(processing(f))
    drift = [dict(start_s=k * window, processing_ms=dist(v)) for k, v in sorted(windows.items())]
    per_pose = {}
    for pose in sorted({a['pose_index'] for a in attempts}):
        gens = {a['generation'] for a in attempts if a['pose_index'] == pose}
        pa = [a for a in attempts if a['pose_index'] == pose]
        per_pose[str(pose)] = dict(attempts=len(pa), aligned=sum(a['outcome'] == 'converged' for a in pa),
            processing_ms=dist([processing(f) for f in active if f['generation'] in gens]))
    if score and attempts:
        score_physical(attempts)
    physical = [s for a in attempts if 'physical_accuracy' in a for s in a['physical_accuracy'].values()]
    started = set(aligns)
    missing = started - set(by_generation)
    telemetry_lost = (sum(counts[k] for k in ('sensor_rows_evicted', 'command_rows_evicted', 'loop_rows_evicted'))
                      + abs(len(sensors) - counts['sensor_received']) + abs(len(commands) - counts['accepted'])
                      + (len(sensors) - len({f['sequence'] for f in sensors})) + int(events_full))
    later_attempts = [a for a in attempts if a['generation'] != first_generation]
    result = dict(meta,
        attempts=len(started), aligned=sum(a['outcome'] == 'converged' for a in attempts),
        outcomes=dict(Counter(a['outcome'] for a in attempts)),
        stop_reasons=dict(Counter(e['reason'] for e in events if e['kind'] == 'stop')),
        pose_attempts=dict(Counter(str(a['pose_index']) for a in attempts)), per_pose=per_pose,
        duration_s=ending['duration_s'], worker_pids=sorted({a['worker_pid'] for a in attempts}),
        worker_executable=ending.get('worker_executable'), execution_settings=ending.get('execution_settings'),
        processing_ms=dist([processing(f) for f in active]),
        capture_ms=dist([c['capture_to_command_ms'] for c in commands]),
        first_processing_ms=dist([processing(f) for f in first]),
        later_processing_ms=dist([processing(f) for f in later]),
        first_capture_ms=dist([c['capture_to_command_ms'] for c in commands if c['generation'] == first_generation]),
        later_capture_ms=dist([c['capture_to_command_ms'] for c in commands
                               if c['generation'] != first_generation and c['generation'] in by_generation]),
        first_alignment_s=next((a['alignment_s'] for a in attempts[:1]), None),
        first_alignment_converged=bool(attempts) and attempts[0]['outcome'] == 'converged',
        later_alignment_s=dist([a['alignment_s'] for a in later_attempts
                                if a['outcome'] == 'converged' and a['alignment_s'] is not None]),
        drift=drift,
        image_error_px=dist([a['error_px'] for a in attempts if a.get('error_px') is not None]),
        position_mm=dist([s['position_error_mm'] for s in physical]),
        orientation_deg=dist([s['orientation_error_deg'] for s in physical]),
        # Every watchdog activation counts: an alignment it ended, or a zero-velocity hold it resumed from.
        freshness_trips=sum(e['kind'] == 'stop' and e['reason'] == 'stale_camera' for e in events)
                        + counts.get('watchdog_pauses', 0),
        stale_stops=sum(e['kind'] == 'stop' and e['reason'] == 'stale_camera' for e in events),
        watchdog_pauses=counts.get('watchdog_pauses', 0), paused_s=counts.get('paused_s', 0.0),
        alignment_time_s=sum(a['alignment_s'] for a in attempts if a.get('alignment_s') is not None),
        paused_motion=counts.get('paused_motion_ticks', 0),
        control_misses=counts['control_deadline_misses'], unsafe_motion=counts['unsafe_motion_ticks'],
        post_stop_motion=counts['post_stop_motion_ticks'], contacts=counts['contacts'],
        unlatched_stops=sum(not a['stop_stayed_latched'] for a in attempts),
        missing_endpoints=len(missing), telemetry_lost=telemetry_lost,
        continuity_gaps=sum(g > 1000 for g in gaps), maximum_control_gap_ms=max(gaps, default=0.0),
        runtime_errors=int(bool(ending.get('error') or ending.get('harness_error') or ending.get('worker_alive'))),
        harness_error=ending.get('harness_error'), counts=counts, worker_counts=ending.get('worker_counts'),
        raw_sha256=digest(raw), samples=dict(sensor=len(sensors), uncached_active=len(active), commands=len(commands)),
        actuator=ending.get('actuator'), physical_stops=physical_stops(events, ending))
    return result, attempts


def physical_stops(events, ending):
    """Measured physical response to each stop and hold (informational, not gated).

    Runs from before the actuator model have no measurements: None.
    """
    if 'actuator' not in ending:
        return None
    import stop_response
    return stop_response.summarize(stop_response.link(dict(events=events, motion=ending.get('motion'),
                                                           motion_evicted=ending.get('motion_evicted'))))


def physical_stop_text(m, kind):
    stops = m.get('physical_stops')
    if stops is None:
        return 'not measured'
    block = stops.get(kind)
    if not block or not block.get('physical_stop_ms'):
        return 'none while moving'
    distance = block.get('camera_travel_mm') or {}
    return (f"{block['physical_stop_ms']['max']:.0f} ms / {distance.get('max', 0.0):.1f} mm "
            f"({block['moving']} while moving)")


def pooled(values_by_session):
    values = [v for group in values_by_session for v in group]
    return dist(values)


def raw_values(directory, session, first_only=None):
    """Processing/capture samples of one session for pooled per-method statistics."""
    raw = directory / 'traces' / (session['id'] + '.jsonl.gz')
    attempts = read(directory / 'traces' / (session['id'] + '-attempts.json'))
    gens = {a['generation'] for a in attempts}
    first = attempts[0]['generation'] if attempts else None
    processing, capture = [], []
    with gzip.open(raw, 'rt', encoding='utf-8') as stream:
        for line in stream:
            chunk = json.loads(line)
            for f in chunk['sensor_frames']:
                if not f['capture_active'] or f['stage_ms'].get('cache_hit', False):
                    continue
                if first_only is not None and ((f['generation'] == first) != first_only or f['generation'] not in gens):
                    continue
                processing.append(1000 * (f['finished_s'] - f['rendered_s']))
            for c in chunk['frames']:
                if first_only is not None and ((c['generation'] == first) != first_only or c['generation'] not in gens):
                    continue
                capture.append(c['capture_to_command_ms'])
    return processing, capture


def method_summary(mode, sessions, directory, config, plan):
    """Pool one method's sessions and apply the predeclared criteria."""
    import stop_response
    c = config['criteria']
    rep = [s for s in sessions if s['mode'] == mode and s['phase'] == 'repeatability']
    sus = [s for s in sessions if s['mode'] == mode and s['phase'] == 'sustained']
    mine = rep + sus
    all_samples = [raw_values(directory, s) for s in mine]
    first_samples = [raw_values(directory, s, True) for s in mine]
    later_samples = [raw_values(directory, s, False) for s in sus]
    total = lambda key: sum(s[key] for s in mine)
    summary = dict(mode=mode, label=label(config, mode),
        attempts=total('attempts'), aligned=total('aligned'),
        repeatability=dict(attempts=sum(s['attempts'] for s in rep), aligned=sum(s['aligned'] for s in rep),
                           sessions=len(rep)),
        sustained=None if not sus else dict(attempts=sus[0]['attempts'], aligned=sus[0]['aligned'],
                                            duration_s=sus[0]['duration_s'], worker_pids=sus[0]['worker_pids'],
                                            pose_attempts=sus[0]['pose_attempts'], drift=sus[0]['drift'],
                                            processing_ms=sus[0]['processing_ms'], capture_ms=sus[0]['capture_ms']),
        processing_ms=pooled(p for p, _ in all_samples), capture_ms=pooled(k for _, k in all_samples),
        first_processing_ms=pooled(p for p, _ in first_samples), later_processing_ms=pooled(p for p, _ in later_samples),
        first_capture_ms=pooled(k for _, k in first_samples), later_capture_ms=pooled(k for _, k in later_samples),
        first_alignment_s=dist([s['first_alignment_s'] for s in mine
                                if s['first_alignment_converged'] and s['first_alignment_s'] is not None]),
        later_alignment_s=None if not sus else sus[0]['later_alignment_s'],
        stop_reasons=dict(sum((Counter(s['stop_reasons']) for s in mine), Counter())),
        actuator=next((s['actuator'] for s in mine if s.get('actuator')), None),
        physical_stops=None if all(s.get('physical_stops') is None for s in mine) else
            stop_response.merge([s.get('physical_stops') for s in mine]),
        position_mm_max=max((s['position_mm']['max'] for s in mine if s['position_mm']), default=None),
        orientation_deg_max=max((s['orientation_deg']['max'] for s in mine if s['orientation_deg']), default=None))
    for key in ('freshness_trips', 'control_misses', 'unsafe_motion', 'post_stop_motion', 'contacts', 'unlatched_stops',
                'missing_endpoints', 'telemetry_lost', 'continuity_gaps', 'runtime_errors'):
        summary[key] = total(key)
    summary['stale_stops'] = sum(s.get('stale_stops', s['freshness_trips']) for s in mine)  # Older runs: all stops.
    for key in ('watchdog_pauses', 'paused_s', 'paused_motion', 'alignment_time_s'):
        summary[key] = sum(s.get(key, 0) for s in mine)
    summary['paused_fraction'] = (summary['paused_s'] / summary['alignment_time_s']
                                  if summary['alignment_time_s'] > 0 else 0.0)
    summary['success_rate'] = summary['aligned'] / summary['attempts'] if summary['attempts'] else 0.0
    confidence = success_confidence(c)
    summary['success_ci'] = binomial_ci.summary(summary['aligned'], summary['attempts'], confidence,
                                                c.get('required_success_rate'))
    for phase in ('repeatability', 'sustained'):
        if summary[phase]:
            summary[phase]['success_ci'] = binomial_ci.summary(summary[phase]['aligned'], summary[phase]['attempts'],
                                                               confidence)
    summary['later_to_first_p99'] = ratio(summary['later_processing_ms'], summary['first_processing_ms'])
    drift = summary['sustained']['drift'] if sus else []
    usable = [w for w in drift if w['processing_ms'] and w['processing_ms']['count'] >= 50]
    summary['drift_last_to_first_p99'] = (ratio(usable[-1]['processing_ms'], usable[0]['processing_ms'])
                                          if len(usable) >= 2 else None)
    summary['environment'] = environment_summary(mine)
    summary['failures'] = evaluate_method(summary, sessions, mode, config, plan)
    summary['verdict'] = 'FAIL' if summary['failures'] else 'PASS'
    return summary


def evaluate_method(s, sessions, mode, config, plan):
    c = config['criteria']; failures = []

    def require(ok, message):
        if not ok:
            failures.append(message)
    if 'required_success_rate' in c:
        # Reliability claim: the lower confidence bound, not the observed rate, must reach the requirement.
        low, _ = binomial_ci.clopper_pearson(s['aligned'], s['attempts'], success_confidence(c))
        require(binomial_ci.meets_required(s['aligned'], s['attempts'], c['required_success_rate'], success_confidence(c)),
                f"alignment success {s['aligned']}/{s['attempts']}: {success_confidence(c) * 100:g}% lower bound "
                f"{binomial_ci.percent(low)} < required {binomial_ci.percent(c['required_success_rate'])}")
        if c.get('max_failed_alignments') is not None:
            failed = s['attempts'] - s['aligned']
            require(failed <= c['max_failed_alignments'],
                    f"failed alignments {failed} > {c['max_failed_alignments']}")
    else:  # Configurations from before confidence intervals: observed rate only.
        require(s['attempts'] > 0 and s['success_rate'] >= c['success_rate'],
                f"alignment success {s['aligned']}/{s['attempts']}")
    for name, quantile, limit in (('processing_ms', 'p95', 'processing_p95_ms'), ('processing_ms', 'p99', 'processing_p99_ms'),
                                  ('processing_ms', 'max', 'processing_max_ms'), ('capture_ms', 'p99', 'capture_p99_ms'),
                                  ('capture_ms', 'max', 'capture_max_ms')):
        value = s[name]
        require(value is not None and value[quantile] <= c[limit],
                f"{name.split('_')[0]} {quantile} {'missing' if value is None else f'{value[quantile]:.1f}'} ms > {c[limit]:g}")
    if 'watchdog_stops' in c:
        # Hold-and-resume criteria: no alignment may be ended by the watchdog, and
        # holds may cost only a small, declared share of alignment time.
        require(s.get('stale_stops', s['freshness_trips']) <= c['watchdog_stops'],
                f"watchdog_stops {s.get('stale_stops', s['freshness_trips'])}")
        require(s.get('paused_fraction', 0.0) <= c['paused_fraction'],
                f"paused {100 * s.get('paused_fraction', 0.0):.1f}% of alignment time > {100 * c['paused_fraction']:g}%")
    else:  # Configurations from before hold-and-resume: every trip fails.
        require(s['freshness_trips'] <= c['freshness_trips'], f"freshness_trips {s['freshness_trips']}")
    for key, limit in (('control_misses', 'control_misses'),
                       ('unsafe_motion', 'unsafe_motion'), ('post_stop_motion', 'post_stop_motion'),
                       ('contacts', 'contacts'), ('unlatched_stops', 'unlatched_stops')):
        require(s[key] <= c[limit], f'{key} {s[key]}')
    for key in ('missing_endpoints', 'telemetry_lost', 'continuity_gaps', 'runtime_errors', 'paused_motion'):
        require(s.get(key, 0) == 0, f'{key} {s.get(key, 0)}')
    require(s['position_mm_max'] is not None and s['position_mm_max'] <= c['position_mm'],
            'position error max ' + ('missing' if s['position_mm_max'] is None else f"{s['position_mm_max']:.3f}") + ' mm')
    require(s['orientation_deg_max'] is not None and s['orientation_deg_max'] <= c['orientation_deg'],
            'orientation error max ' + ('missing' if s['orientation_deg_max'] is None else f"{s['orientation_deg_max']:.3f}") + ' deg')
    ratio_value = s['later_to_first_p99']
    require(ratio_value is not None and ratio_value <= c['later_to_first_p99_ratio'],
            'later/first processing p99 ' + ('missing' if ratio_value is None else f'{ratio_value:.2f}x'))
    # Coverage: every declared session ran to its plan, and the sustained worker was never replaced.
    rep = [x for x in sessions if x['mode'] == mode and x['phase'] == 'repeatability']
    expected = {str(i) for i in plan['pose_ids']}
    require({x['pose_ids'][0] for x in rep} == {int(i) for i in expected}
            and all(x['attempts'] >= plan['alignments_per_pose'] for x in rep), 'repeatability coverage incomplete')
    sus = [x for x in sessions if x['mode'] == mode and x['phase'] == 'sustained']
    require(len(sus) == 1, 'sustained run missing')
    if len(sus) == 1:
        x = sus[0]
        require(x['duration_s'] >= plan['sustained_seconds'], f"sustained duration {x['duration_s']:.0f} s")
        require(len(x['worker_pids']) == 1, 'sustained worker replaced')
        require(all(x['pose_attempts'].get(i, 0) >= plan['minimum_attempts_per_pose'] for i in expected),
                'sustained pose coverage')
    return failures


# ----------------------------------------------------------------------------- report

def fmt(d, keys=('p95', 'p99', 'max')):
    return 'n/a' if not d else ' / '.join(f'{d[k]:.1f}' for k in keys)


def seconds(d):
    return 'n/a' if d is None else (f'{d:.2f} s' if isinstance(d, (int, float)) else f"{d['p50']:.2f} s (n={d['count']})")


def worst_window(m):
    windows = [w for w in ((m.get('sustained') or {}).get('drift') or []) if w['processing_ms'] and w['processing_ms']['count'] >= 50]
    if not windows:
        return 'n/a'
    w = max(windows, key=lambda w: w['processing_ms']['p99'])
    return f"{w['processing_ms']['p99']:.1f} ms (minute {w['start_s'] / 60:g})"


PROCEDURE_DOC = 'docs/TEST_PROCEDURE.md'
FRESH_RESTART_S = 3600
PROCEDURE_SUMMARY = ('Test procedure (' + PROCEDURE_DOC + '): restart the laptop, close other programs and browser tabs, '
                     'AC power, Windows power mode Best performance, laptop utility in performance mode.')


def procedure_preflight():
    """Print the checklist and warn about conditions that can be checked. Never blocks the run."""
    print(PROCEDURE_SUMMARY, flush=True)
    uptime = env_monitor.uptime_s()
    if uptime is not None and uptime > FRESH_RESTART_S:
        print(f'WARNING: the computer has been running for {uptime / 3600:.1f} h. The procedure asks for a fresh restart.',
              flush=True)
    mode = env_monitor.power_mode_name(env_monitor.power_mode())
    if mode and mode != 'Best performance':
        print(f'WARNING: Windows power mode is "{mode}"; the procedure asks for Best performance.', flush=True)
    return uptime


def procedure_lines(manifest):
    """Which procedure conditions were met; manual items are listed as not checked."""
    L = [f'Procedure: `{PROCEDURE_DOC}`. Conditions this run could check:', '']
    uptime = manifest.get('uptime_at_start_s')
    if uptime is None:
        L.append('- Fresh restart: not recorded')
    else:
        ok = uptime <= FRESH_RESTART_S
        L.append(f"- Fresh restart: {'yes' if ok else 'NO'} (computer running {uptime / 60:.0f} min when the test started; "
                 f'the procedure asks for at most {FRESH_RESTART_S // 60} min)')
    powers = [b.get('power') for b in manifest.get('session_boundaries', []) if b.get('power')]
    if powers:
        ok = all(p.get('ACLineStatus') == 1 for p in powers)
        L.append(f"- AC power at every session start: {'yes' if ok else 'NO'}")
    else:
        L.append('- AC power: not recorded')
    modes = sorted({env_monitor.power_mode_name(b.get('power_mode')) for b in manifest.get('session_boundaries', [])} - {None})
    L.append('- Windows power mode Best performance: ' + ('not recorded' if not modes else
             'yes' if modes == ['Best performance'] else 'NO (' + ', '.join(modes) + ')'))
    L.append('- Not checked automatically: other programs and browser tabs closed, laptop maker\'s utility in '
             'performance mode, ventilation. Note in your results whether these were followed.')
    return L


def success_confidence(criteria):
    return float(criteria.get('confidence', binomial_ci.DEFAULT_CONFIDENCE))


def ci_text(record):
    """'[97.6%, 100.0%]' from a stored binomial summary (older verdicts may lack it)."""
    if not record or record.get('ci_low') is None:
        return 'n/a'
    return f"[{binomial_ci.percent(record['ci_low'])}, {binomial_ci.percent(record['ci_high'])}]"


def with_ci(block):
    if not block:
        return 'n/a'
    record = block.get('success_ci') or binomial_ci.summary(block['aligned'], block['attempts'])
    return f"{block['aligned']}/{block['attempts']} {ci_text(record)}"


def reliability_text(m, criteria):
    required = criteria.get('required_success_rate')
    record = m.get('success_ci') or binomial_ci.summary(m['aligned'], m['attempts'], success_confidence(criteria))
    if required is None:
        return f"not gated (lower bound {binomial_ci.percent(record['ci_low'])})"
    ok = binomial_ci.meets_required(m['aligned'], m['attempts'], required, success_confidence(criteria))
    return (f"{'yes' if ok else 'NO'}: lower bound {binomial_ci.percent(record['ci_low'])} "
            f"{'>=' if ok else '<'} {binomial_ci.percent(required)}")


def success_rate_lines(methods, criteria):
    """Plain-language reading of each method's success rate, placed right under the table."""
    confidence = success_confidence(criteria)
    lines = [f"**Success rates are estimates.** The interval is the two-sided {confidence * 100:g}% exact "
             "Clopper-Pearson confidence interval for the true success rate (docs/STATISTICS.md)."]
    for m in methods:
        lines.append(f"- {m['label']}: {binomial_ci.interpretation(m['aligned'], m['attempts'], confidence)}")
    required = criteria.get('required_success_rate')
    if required is not None:
        lines.append(f"- A success-rate claim of at least {binomial_ci.percent(required)} passes only if the lower bound "
                     f"reaches it; that needs at least {binomial_ci.trials_needed(required, confidence)} alignments "
                     'with no failure.')
    return lines


def stale_resume_s(manifest):
    return float((manifest.get('runtime_config') or {}).get('stale_resume_s', 0.0))


def success_gate_text(c):
    if 'required_success_rate' not in c:
        return f"{100 * c['success_rate']:g}% alignment (observed)"
    text = (f"{success_confidence(c) * 100:g}% lower confidence bound on alignment success >= "
            f"{binomial_ci.percent(c['required_success_rate'])}")
    if c.get('max_failed_alignments') is not None:
        text += f" and at most {c['max_failed_alignments']} failed alignment(s)"
    return text


def production_files_sentence(manifest):
    check = manifest.get('baseline_check') or {}
    approved = check.get('approved_changes') or []
    if not approved:
        return 'Protected production files match the validated baseline.'
    return ('Protected production files match the validated baseline except reviewed changes listed in '
            '`tools/approved_changes.json`: ' + '; '.join(f"`{c['file']}` ({c['title']})" for c in approved) + '.')


def watchdog_line(manifest, holds_gated=True):
    resume = stale_resume_s(manifest)
    if not resume:
        return ('Freshness watchdog response: **stop** (`stale_resume_s` = 0; the original behaviour, now the '
                '`--watchdog-stop` setting). The 400 ms limit ends the alignment.')
    return (f'Freshness watchdog response: **hold and resume** (`stale_resume_s` = {resume:g} s). At the same 400 ms '
            'limit the robot is commanded to zero velocity; the alignment continues when a fresh image arrives and '
            f'ends as `stale_camera` only if none arrives within {resume:g} s. '
            + ('Holds are allowed only within the pause budget in the criteria; an alignment the watchdog ends fails.'
               if holds_gated else
               'Holds are reported separately; the margin counts alignments the watchdog ended.'))


def power_mode_line(manifest):
    modes = [env_monitor.power_mode_name(b.get('power_mode')) for b in manifest.get('session_boundaries', [])]
    modes = [m for m in modes if m]
    if not modes:
        return '- Windows power mode: not recorded'
    distinct = sorted(set(modes))
    return ('- Windows power mode at every session start: ' + distinct[0] if len(distinct) == 1
            else '- Windows power mode CHANGED during the test: ' + ' -> '.join(modes))


def baseline_line(check):
    if not check:
        return '- Baseline comparison: n/a'
    files = 'identical' if not check['changed_files'] else 'changed: ' + ', '.join(check['changed_files'])
    runtime = ('identical' if not check['runtime_differences'] else
               'different: ' + ', '.join(f'{k} {v[0]} -> {v[1]}' for k, v in sorted(check['runtime_differences'].items())))
    approved = ''.join(f"; approved change to `{c['file']}`: {c['title']}" for c in check.get('approved_changes', []))
    return (f"- Versus validated baseline `{Path(check['baseline']).parent.name}`: source/config/model files {files}; "
            f"runtime versions {runtime}{approved}")


def environment_rows():
    def env(m, f, gpu=False, system=False):
        e = m.get('environment')
        if not e:
            return 'n/a'
        if gpu and not e['gpu_samples']:
            return 'no GPU samples'
        if system and not e['system_samples']:
            return 'no system samples'
        return f(e)

    def limit_seconds(e):
        r = e['limiting_reason_seconds']
        hw = r['hw_slowdown'] + r['hw_thermal_slowdown'] + r['hw_power_brake']
        return f"{r['sw_power_cap']} / {r['sw_thermal_slowdown']} / {hw}"
    optional = lambda value, unit='': 'n/a' if value is None else f'{value:.0f}{unit}'
    return [
        ('GPU clock-limit samples, s (SW power / SW thermal / HW)', lambda m: env(m, limit_seconds, gpu=True)),
        ('Failed alignments during GPU clock limit', lambda m: env(m, lambda e: f"{e['failed_during_throttle']}/{e['failed_alignments']}", gpu=True)),
        ('GPU busy below 400 MHz, s / busy s', lambda m: env(m, lambda e: f"{e['gpu_low_clock_seconds']} / {e['gpu_busy_seconds']}", gpu=True)),
        ('Failed alignments: GPU below 400 MHz / CPU below 85% max', lambda m: env(m, lambda e: (
            f"{e['failed_during_gpu_low_clock']}/{e['failed_alignments']} / "
            f"{e['failed_during_cpu_low_frequency']}/{e['failed_alignments']}"), gpu=True)),
        ('Slow frames (>= 150 ms) during GPU clock limit', lambda m: env(m, lambda e: f"{e['slow_frames_during_throttle']}/{e['slow_frames']}", gpu=True)),
        ('GPU busy SM clock min / max temperature', lambda m: env(m, lambda e: f"{optional(e['gpu_busy_sm_min_mhz'], ' MHz')} / {optional(e['gpu_max_temp_c'], ' C')}", gpu=True)),
        ('Hottest thermal zone / min passive limit', lambda m: env(m, lambda e: (
            'n/a' if not e['thermal_zone_max_c'] else f"{max(e['thermal_zone_max_c'].values()):.0f} C") + ' / ' + optional(e['min_passive_limit_pct'], '%'), system=True)),
        ('CPU frequency min / median (% of max)', lambda m: env(m, lambda e: f"{optional(e['cpu_min_pct_max_frequency'], '%')} / {optional(e.get('cpu_median_pct_max_frequency'), '%')}", system=True)),
    ]


def write_report(directory, status, reason, methods, sessions, manifest, config, complete, telemetry=None):
    verdict = dict(status=status, reason=reason, complete=complete, telemetry=telemetry, methods=methods,
                   sessions=[{k: v for k, v in s.items() if k not in ('per_pose',)} for s in sessions])
    write(directory / 'verdict.json', verdict)
    env = manifest.get('environment', {})
    gpu = manifest.get('selected_cuda_gpu') or {}
    profile = config.get('profile', 'standard')
    L = [f'# Acceptance test{"" if profile == "standard" else f" ({profile})"}: {status}', '',
         f"Profile: {profile}{' (smoke)' if manifest.get('smoke') else ''}. Fresh-worker repeatability on "
         f"{len((manifest.get('plan') or {}).get('pose_ids') or config['poses_degrees'])} poses, then a "
         f"{(manifest.get('plan') or {}).get('sustained_seconds', config['sustained']['seconds']) / 60:.3g}-minute "
         'sustained run with one reused worker per method.', '']
    L += [watchdog_line(manifest), '']
    if reason:
        L += [reason, '']
    L += ['| Metric | ' + ' | '.join(m['label'] for m in methods) + ' |', '|---|' + '---:|' * len(methods)]
    rows = [
        ('Verdict', lambda m: f"**{m['verdict']}**"),
        ('Alignment success', lambda m: f"{m['aligned']}/{m['attempts']} ({100 * m['success_rate']:.1f}%)"),
        (f"  {success_confidence(config['criteria']) * 100:g}% confidence interval, exact Clopper-Pearson",
            lambda m: ci_text(m.get('success_ci') or binomial_ci.summary(m['aligned'], m['attempts'],
                                                                         success_confidence(config['criteria'])))),
        ('  success rate demonstrated', lambda m: reliability_text(m, config['criteria'])),
        ('  fresh-worker poses', lambda m: with_ci(m['repeatability'])),
        ('  sustained reused worker', lambda m: 'n/a' if not m['sustained'] else
            f"{with_ci(m['sustained'])} in {m['sustained']['duration_s'] / 60:.1f} min"),
        ('Processing p95 / p99 / max (ms)', lambda m: fmt(m['processing_ms'])),
        ('Capture-to-command p99 / max (ms)', lambda m: fmt(m['capture_ms'], ('p99', 'max'))),
        ('Freshness watchdog trips', lambda m: str(m['freshness_trips']) + (
            '' if not m.get('watchdog_pauses') else f" ({m['stale_stops']} ended the alignment, "
            f"{m['watchdog_pauses']} held and resumed, {m['paused_s']:.1f} s held)")),
        ('Paused time (share of alignment time)', lambda m: f"{m.get('paused_s', 0.0):.1f} s "
            f"({100 * m.get('paused_fraction', 0.0):.1f}%)"),
        ('Control deadline misses', lambda m: str(m['control_misses'])),
        ('First-alignment processing p95 / p99 / max', lambda m: fmt(m['first_processing_ms'])),
        ('Later-alignment processing p95 / p99 / max', lambda m: fmt(m['later_processing_ms'])),
        ('Later / first processing p99', lambda m: 'n/a' if m['later_to_first_p99'] is None else f"{m['later_to_first_p99']:.2f}x"),
        ('First / later capture-to-command p99', lambda m: f"{fmt(m['first_capture_ms'], ('p99',))} / {fmt(m['later_capture_ms'], ('p99',))}"),
        ('Time to converge, first / later (median)', lambda m: f"{seconds(m['first_alignment_s'])} / {seconds(m['later_alignment_s'])}"),
        ('Sustained drift, last / first window p99', lambda m: 'n/a' if m['drift_last_to_first_p99'] is None else f"{m['drift_last_to_first_p99']:.2f}x"),
        ('Sustained worst window p99 (window start)', lambda m: worst_window(m)),
        ('Unsafe / post-stop motion ticks', lambda m: f"{m['unsafe_motion']} / {m['post_stop_motion']}"),
        ('Motion ticks during a watchdog hold', lambda m: str(m.get('paused_motion', 0))),
        ('Actuator model (simulated)', lambda m: 'not modelled' if not m.get('actuator') else
            m['actuator'].get('profile', '?') + ('' if m['actuator'].get('enabled', True) else ' (off)')),
        ('Physical stop after a stop: max time / camera travel', lambda m: physical_stop_text(m, 'stop')),
        ('Physical stop after a hold: max time / camera travel', lambda m: physical_stop_text(m, 'pause')),
        ('Forbidden contacts / unlatched stops', lambda m: f"{m['contacts']} / {m['unlatched_stops']}"),
        ('Physical error max (mm / deg)', lambda m: 'n/a' if m['position_mm_max'] is None else
            f"{m['position_mm_max']:.3f} / {m['orientation_deg_max']:.3f}"),
        ('Stop reasons', lambda m: ', '.join(f'{k} {v}' for k, v in sorted(m['stop_reasons'].items(), key=lambda kv: -kv[1])) or 'n/a'),
    ]
    for name, f in rows:
        L.append(f'| {name} | ' + ' | '.join(f(m) for m in methods) + ' |')
    L += [''] + success_rate_lines(methods, config['criteria'])
    L += ['', 'Hardware telemetry (diagnostic, not gated). A sample counts as clock-limited when nvidia-smi reports a '
          'software power cap, software/hardware thermal slowdown, hardware slowdown or power brake.', '',
          '| Hardware | ' + ' | '.join(m['label'] for m in methods) + ' |', '|---|' + '---:|' * len(methods)]
    for name, f in environment_rows():
        L.append(f'| {name} | ' + ' | '.join(f(m) for m in methods) + ' |')
    shown = [(m['label'], ep) for m in methods for ep in ((m.get('environment') or {}).get('episodes') or [])
             if ep['duration_s'] >= 2]
    if shown:
        L += ['', 'GPU clock-limit episodes of 2 s or more (time after the session\'s worker was ready; negative = during start-up):', '']
        for name, ep in shown[:15]:
            L.append(f"- {name} `{ep['session']}` at {ep['start_s']:.0f} s for {ep['duration_s']:.0f} s: "
                     f"{', '.join(ep['reasons'])}; SM clock down to {ep['min_sm_mhz']} MHz, utilisation up to {ep['max_util']}%")
    lows = [(m['label'], ep) for m in methods for ep in ((m.get('environment') or {}).get('low_state_episodes') or [])
            if ep['session'].startswith('sustained')]
    if lows:
        L += ['', 'Low-performance periods in the sustained runs (30 s windows; GPU busy below 400 MHz or CPU below 85% of maximum frequency):', '']
        for name, ep in lows[:20]:
            L.append(f"- {name}: minute {ep['start_s'] / 60:.1f} for {ep['duration_s'] / 60:.1f} min: {', '.join(ep['state'])}")
    L += ['']
    for m in methods:
        L.append(f"**{m['label']} failed criteria:** {'; '.join(m['failures']) or 'none'}")
    c = config['criteria']
    L += ['', '## Test procedure', ''] + procedure_lines(manifest)
    L += ['', '## Conditions', '',
          f"Freshness watchdog {1000 * config['runtime']['max_age_s']:.0f} ms, control deadline "
          f"{1000 * config['runtime']['max_control_gap_s']:.0f} ms, transport delay {1000 * config['runtime']['transport_s']:.0f} ms "
          '(production values, asserted before the run). ' + production_files_sentence(manifest),
          f"Gates: {success_gate_text(c)}; processing p95/p99/max <= {c['processing_p95_ms']:g}/{c['processing_p99_ms']:g}/{c['processing_max_ms']:g} ms; "
          f"capture-to-command p99/max <= {c['capture_p99_ms']:g}/{c['capture_max_ms']:g} ms; "
          + (f"0 alignments ended by the watchdog, paused <= {100 * c['paused_fraction']:g}% of alignment time, "
             if 'watchdog_stops' in c else '0 freshness trips, ')
          + f"0 deadline misses, "
          f"0 unsafe/post-stop motion; <= {c['position_mm']:g} mm / {c['orientation_deg']:g} deg; later/first p99 <= {c['later_to_first_p99_ratio']:g}x.",
          '',
          f"- Python: `{manifest.get('python', {}).get('windows_executable') or manifest.get('python', {}).get('executable')}` "
          f"(venv `{manifest.get('python', {}).get('prefix')}`)",
          f"- Sensor worker executable: `{', '.join(sorted({s.get('worker_executable') or '?' for s in sessions})) or 'n/a'}`",
          f"- CUDA GPU: {gpu.get('name', 'n/a')} (cuda:{gpu.get('cuda_index', '?')}, UUID {gpu.get('uuid', '?')}, "
          f"driver {(gpu.get('nvidia_smi') or {}).get('driver', '?')})",
          f"- Versions: Python {env.get('python', '?').split()[0]}, torch {env.get('torch')}, CUDA {manifest.get('cuda_probe', {}).get('torch_cuda')}, "
          f"cuDNN {manifest.get('cuda_probe', {}).get('cudnn')}, mujoco {env.get('mujoco')}, OpenCV {env.get('opencv-python') or env.get('opencv-python-headless')}, "
          f"numpy {env.get('numpy')}, {env.get('platform')}",
          baseline_line(manifest.get('baseline_check') or {}),
          power_mode_line(manifest),
          f"- Started {manifest.get('created_utc')}; complete: {complete}",
          '',
          'Processing = render finished to perception result, every uncached active frame including failed/late ones. '
          f"Capture-to-command covers accepted commands. First = first alignment of each worker ({len(config['poses_degrees']) if not manifest.get('smoke') else len(manifest['plan']['pose_ids'])} fresh workers + the sustained worker); "
          'later = every subsequent alignment of the sustained reused worker. Details: `verdict.json`, `manifest.json`, `test-config.json`; '
          'raw per-frame evidence under `traces/`.']
    (directory / 'REPORT.md').write_text('\n'.join(L) + '\n', encoding='utf-8')


# ----------------------------------------------------------------------------- orchestration

def build_plan(config, smoke, only=None):
    pose_ids = list(range(len(config['poses_degrees'])))
    plan = dict(pose_ids=pose_ids, alignments_per_pose=config['repeatability']['alignments_per_pose'],
                sustained_seconds=config['sustained']['seconds'],
                minimum_attempts_per_pose=config['sustained']['minimum_attempts_per_pose'], smoke=smoke)
    if smoke:
        plan.update(pose_ids=pose_ids[:SMOKE['poses']], alignments_per_pose=SMOKE['alignments_per_pose'],
                    sustained_seconds=SMOKE['sustained_seconds'], minimum_attempts_per_pose=SMOKE['minimum_attempts_per_pose'])
    specs = []
    for item in config['schedule']:
        phase, mode = item.split(':')
        if only and mode != only:
            continue
        if phase == 'repeatability':
            specs += [dict(id=f'{phase}-{mode}-pose{p}', phase=phase, mode=mode, pose_ids=[p],
                           max_attempts=plan['alignments_per_pose'], seconds=None) for p in plan['pose_ids']]
        else:
            specs.append(dict(id=f'{phase}-{mode}', phase=phase, mode=mode, pose_ids=plan['pose_ids'],
                              max_attempts=None, seconds=plan['sustained_seconds']))
    plan['sessions'] = specs
    plan['only_method'] = only
    return plan


DEFAULT_STALE_RESUME_MS = 2000.0  # Matches RuntimeConfig's default (hold and resume).


def with_stale_resume(config, stale_resume_ms):
    """Record the watchdog response explicitly; every certified production limit stays as declared."""
    return dict(config, runtime=dict(config['runtime'], stale_resume_s=stale_resume_ms / 1000))


def validate_config(config):
    problems = []
    if config.get('runtime') != PRODUCTION_RUNTIME:
        problems.append(f"runtime {config.get('runtime')} differs from production {PRODUCTION_RUNTIME}")
    if sorted(config.get('methods', [])) != ['learned', 'natural']:
        problems.append('both SIFT (natural) and Learned GPU (learned) are required')
    if len(config.get('poses_degrees', [])) < 3:
        problems.append('at least three starting poses are required')
    profile = PROFILES.get(config.get('profile', 'standard'))
    if profile is None:
        problems.append(f"unknown profile {config.get('profile')!r}")
    elif not profile[0] <= config['sustained']['seconds'] <= profile[1]:
        problems.append(f"sustained run for the {config.get('profile', 'standard')} profile must be {profile[2]}")
    if config['post_stop_seconds'] != 1.1:
        problems.append('post-stop observation must stay 1.1 s')
    c = config.get('criteria', {})
    if 'required_success_rate' in c:
        if not 0 < c['required_success_rate'] < 1:
            problems.append('required_success_rate must be in (0, 1): 100% can never be demonstrated from finite trials')
        if not 0 < c.get('confidence', binomial_ci.DEFAULT_CONFIDENCE) < 1:
            problems.append('confidence must be in (0, 1)')
    elif 'success_rate' not in c:
        problems.append('a success-rate criterion is required')
    return problems


def approved_changes(path=APPROVED_CHANGES):
    return read(path)['changes'] if Path(path).exists() else []


def baseline_check(current, baseline_path, approved_path=APPROVED_CHANGES):
    """A protected file may differ only by an approved change pinned to both hashes."""
    saved = read(baseline_path)['fingerprint']
    changed = sorted(k for k in set(saved['sha256']) | set(current['sha256'])
                     if saved['sha256'].get(k) != current['sha256'].get(k))
    approved = [c for c in approved_changes(approved_path) if c['file'] in changed
                and c['baseline_sha256'] == saved['sha256'].get(c['file'])
                and c['sha256'] == current['sha256'].get(c['file'])]
    approved_files = {c['file'] for c in approved}
    return dict(baseline=str(baseline_path), baseline_sha256=digest(baseline_path),
                all_identical=not changed and saved['runtime'] == current['runtime'],
                changed_files=changed,
                protected_changed=[k for k in changed if is_protected(k) and k not in approved_files],
                approved_changes=[dict(file=c['file'], title=c['title']) for c in approved],
                runtime_differences={k: [saved['runtime'].get(k), current['runtime'].get(k)]
                                     for k in set(saved['runtime']) | set(current['runtime'])
                                     if saved['runtime'].get(k) != current['runtime'].get(k)})


def attach_environment(directory, sessions, manifest):
    """Place GPU/CPU/thermal samples on each session's clock (diagnostic, not gated)."""
    gpu, offset = env_monitor.read_gpu(directory / 'traces' / 'gpu.csv', manifest.get('local_utc_offset_s'),
                                       manifest.get('created_utc'))
    zones, cpu = env_monitor.read_system(directory / 'traces' / 'system.csv')
    for s in sessions:
        try:
            frames, events = [], []
            with gzip.open(directory / 'traces' / (s['id'] + '.jsonl.gz'), 'rt', encoding='utf-8') as stream:
                for line in stream:
                    chunk = json.loads(line)
                    frames += [f for f in chunk['sensor_frames']
                               if f['capture_active'] and not f['stage_ms'].get('cache_hit', False)]
                    events += chunk['events']
            s['environment'] = env_monitor.session_environment(
                frames, events, read(directory / 'traces' / (s['id'] + '-attempts.json')),
                read(directory / 'traces' / (s['id'] + '-ending.json')), gpu, zones, cpu)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            s['environment'] = dict(error=repr(exc))
    return dict(gpu_samples=len(gpu), system_zone_samples=len(zones), system_cpu_samples=len(cpu), utc_offset_s=offset)


def environment_summary(sessions):
    envs = [(s['id'], s['environment']) for s in sessions if s.get('environment') and 'error' not in s['environment']]
    if not envs:
        return None
    total = lambda key: sum(e[key] for _, e in envs)
    zones = {}
    for _, e in envs:
        for name, value in e['thermal_zone_max_c'].items():
            zones[name] = max(zones.get(name, -1e9), value)
    optional = lambda values: min(values) if values else None
    return dict(
        limiting_reason_seconds={r: sum(e['limiting_reason_seconds'][r] for _, e in envs) for r in env_monitor.LIMITING},
        failed_alignments=total('failed_alignments'), failed_during_throttle=total('failed_during_throttle'),
        failed_without_gpu_sample=total('failed_without_gpu_sample'),
        failed_during_gpu_low_clock=sum(e.get('failed_during_gpu_low_clock', 0) for _, e in envs),
        failed_during_cpu_low_frequency=sum(e.get('failed_during_cpu_low_frequency', 0) for _, e in envs),
        gpu_low_clock_seconds=sum(e.get('gpu_low_clock_seconds', 0) for _, e in envs),
        gpu_busy_seconds=sum(e.get('gpu_busy_seconds', 0) for _, e in envs),
        low_state_episodes=[dict(session=sid, **ep) for sid, e in envs for ep in e.get('low_state_episodes', [])],
        slow_frames=total('slow_frames'), slow_frames_during_throttle=total('slow_frames_during_throttle'),
        gpu_samples=total('gpu_samples'), system_samples=total('system_samples'),
        gpu_max_temp_c=max((e['gpu_max_temp_c'] for _, e in envs if e['gpu_max_temp_c'] is not None), default=None),
        gpu_busy_sm_min_mhz=optional([e['gpu_busy_sm_mhz']['min'] for _, e in envs if e['gpu_busy_sm_mhz']]),
        thermal_zone_max_c=zones,
        min_passive_limit_pct=optional([e['min_passive_limit_pct'] for _, e in envs if e['min_passive_limit_pct'] is not None]),
        cpu_min_pct_max_frequency=optional([e['cpu_pct_max_frequency']['min'] for _, e in envs if e['cpu_pct_max_frequency']]),
        cpu_median_pct_max_frequency=next((e['cpu_pct_max_frequency']['median'] for sid, e in reversed(envs)
                                           if e['cpu_pct_max_frequency'] and sid.startswith('sustained')), None),
        episodes=[dict(session=sid, **ep) for sid, e in envs for ep in e['throttle_episodes']])


def finalize(directory, sessions, config, plan, manifest, complete, invalid_reason=None, smoke=False):
    telemetry = attach_environment(directory, sessions, manifest)
    methods = [method_summary(mode, sessions, directory, config, plan) for mode in config['methods']
               if any(s['mode'] == mode for s in sessions)]
    failed = any(m['failures'] for m in methods)
    if invalid_reason:
        status, reason = 'INVALID', invalid_reason
    elif smoke:
        status, reason = 'SMOKE_ONLY', 'Short harness check; it cannot certify acceptance.'
    elif plan.get('only_method') and not failed and complete:
        status, reason = 'PARTIAL', 'Only one method was run; acceptance requires both.'
    elif failed:
        status = 'FAIL'
        reason = None if complete else 'The test did not complete; failures observed before it stopped are listed.'
    elif not complete or len(methods) != len(config['methods']):
        status, reason = 'INCOMPLETE', 'The test did not complete, so it cannot pass.'
    else:
        status, reason = 'PASS', None
    write_report(directory, status, reason, methods, sessions, manifest, config, complete, telemetry)
    return status


def run(args):
    config = read(args.config)
    problems = validate_config(config)
    if problems:
        print('INVALID configuration: ' + '; '.join(problems), flush=True)
        return 2
    only = {'sift': 'natural', 'learned': 'learned', None: None}[args.method]
    plan = build_plan(config, args.smoke, only)
    from run_camera_robustness import fingerprint
    from realtime import RuntimeConfig
    from simulation import Simulation
    current = fingerprint()
    check = baseline_check(current, args.baseline)
    if check['protected_changed']:
        print('INVALID: controller/calibration/scene/safety inputs differ from the validated baseline: '
              + ', '.join(check['protected_changed']), flush=True)
        return 2
    with Simulation(render=False) as sim:  # Joint-limit and collision clearance for every declared pose.
        for pose in config['poses_degrees']:
            sim.reset(pose)
    power = power_status()
    if power is not None and power.get('ACLineStatus') != 1:
        print('INVALID: connect AC power; laptop GPU behaviour on battery is not the tested condition.', flush=True)
        return 2
    probe, inventory = probe_cuda(), nvidia_inventory()
    selected = select_gpu(probe, inventory)
    if only != 'natural' and not args.smoke and not (selected and probe.get('cuda_available')):
        print('INVALID: CUDA is not available to this Python environment: ' + json.dumps(probe), flush=True)
        return 2
    uptime = procedure_preflight()
    stamp = (datetime.now().strftime('%Y%m%d-%H%M%S') + (f"-{config['profile']}" if config.get('profile', 'standard') != 'standard' else '')
             + ('-smoke' if args.smoke else '') + (f'-{args.method}' if args.method else '')
             + ('-stop' if not args.stale_resume_ms else '' if args.stale_resume_ms == DEFAULT_STALE_RESUME_MS
                else f'-resume{args.stale_resume_ms:g}ms'))
    session_config = with_stale_resume(config, args.stale_resume_ms)
    directory = args.output or DEFAULT_RESULTS / stamp
    directory.mkdir(parents=True, exist_ok=False); (directory / 'traces').mkdir()
    write(directory / 'test-config.json', config)
    runner_hash, config_hash = digest(Path(__file__)), digest(args.config)
    environment_hash = digest(ENVIRONMENT_MODULE)
    base_exe = getattr(sys, '_base_executable', sys.executable)
    import multiprocessing.spawn as spawn
    manifest = dict(created_utc=datetime.now(timezone.utc).isoformat(), status='running', command=sys.argv,
        smoke=args.smoke, plan=plan, config_path=str(args.config), config_sha256=config_hash, runner_sha256=runner_hash,
        environment_module_sha256=environment_hash, local_utc_offset_s=env_monitor.utc_offset_s(),
        uptime_at_start_s=uptime, procedure=PROCEDURE_DOC,
        runtime_config=asdict(RuntimeConfig(**session_config['runtime'])),
        python=dict(executable=sys.executable, windows_executable=this_executable(), base_executable=base_exe,
                    spawn_executable=os.fsdecode(spawn.get_executable()), prefix=sys.prefix, version=sys.version),
        environment=runtime_versions(), fingerprint=current, baseline_check=check,
        cuda_probe=probe, nvidia_smi=inventory, selected_cuda_gpu=selected,
        gpu_preferences=gpu_preferences([sys.executable, base_exe, this_executable()]),
        power_at_start=power, model_weights_sha256={p.relative_to(APP).as_posix(): digest(p)
                                                    for p in sorted((APP / 'models').glob('*.pth'))},
        method='Fresh worker per declared pose (repeatability), then one reused worker per method (sustained). '
               'Unmodified RealtimeSession, controller, calibration, scene, warm-up and watchdogs; only the offset used '
               'by an explicit Offset reset varies. Telemetry copied at inactive publish; disk I/O while stopped.')
    write(directory / 'manifest.json', manifest)
    print(f'Acceptance test -> {directory}', flush=True)
    keep_awake = None
    if os.name == 'nt':
        import ctypes
        keep_awake = ctypes.windll.kernel32.SetThreadExecutionState(0x80000001)  # ES_CONTINUOUS|ES_SYSTEM_REQUIRED
    manifest['leftover_samplers'] = env_monitor.stop_leftover_samplers()
    if manifest['leftover_samplers']['stopped']:
        print(f"Stopped {len(manifest['leftover_samplers']['stopped'])} sampler process(es) left running by an earlier, "
              'interrupted test.', flush=True)
    if manifest['leftover_samplers']['error'] and manifest['leftover_samplers']['found']:
        print('INVALID: samplers from an earlier test are still running and could not be stopped: '
              + manifest['leftover_samplers']['error'], flush=True)
        write(directory / 'manifest.json', manifest)
        return 2
    manifest['snapshot_start'] = env_monitor.snapshot(directory, 'start')
    monitor = env_monitor.GpuMonitor(directory / 'traces' / 'gpu.csv', config['gpu_sample_interval_ms'],
                                     None if not selected else (selected.get('nvidia_smi') or {}).get('index')).start()
    system = env_monitor.SystemMonitor(directory / 'traces' / 'system.csv', config['gpu_sample_interval_ms'])
    if not args.no_system_monitor:
        system.start()
    manifest['monitors'] = dict(gpu=monitor.metadata(), system=system.metadata(), system_enabled=not args.no_system_monitor)
    write(directory / 'manifest.json', manifest)
    sessions, complete, invalid, boundaries, harness_error = [], False, None, [], None
    try:
        for spec in plan['sessions']:
            if (fingerprint() != current or digest(Path(__file__)) != runner_hash or digest(args.config) != config_hash
                    or digest(ENVIRONMENT_MODULE) != environment_hash):
                invalid = 'Source, runner or configuration changed during the test.'
                break
            status_now = power_status()
            boundaries.append(dict(session=spec['id'], utc=datetime.now(timezone.utc).isoformat(), power=status_now,
                                   power_mode=env_monitor.power_mode()))
            if status_now is not None and status_now.get('ACLineStatus') != 1:
                invalid = 'AC power was disconnected during the test.'
                break
            print(f"START {spec['id']}", flush=True)
            raw, attempts, ending = run_session(spec['mode'], spec['pose_ids'], session_config, directory, spec['id'],
                                                max_attempts=spec['max_attempts'], seconds=spec['seconds'])
            meta = {k: spec[k] for k in ('id', 'phase', 'mode', 'pose_ids')}
            result, attempts = summarize(raw, attempts, ending, meta, config)
            write(directory / 'traces' / (spec['id'] + '-attempts.json'), attempts)
            write(directory / (spec['id'] + '.json'), result)
            sessions.append(result)
            print(f"END {spec['id']}: aligned {result['aligned']}/{result['attempts']}, "
                  f"freshness {result['freshness_trips']}, misses {result['control_misses']}, "
                  f"processing p99 {fmt(result['processing_ms'], ('p99',))} ms", flush=True)
            finalize(directory, sessions, config, plan, manifest, False, smoke=args.smoke)
            if (result['unsafe_motion'] or result['post_stop_motion'] or result['contacts'] or result['unlatched_stops']
                    or result['paused_motion']):
                print('Safety violation: remaining sessions cancelled.', flush=True)
                break
            if result['runtime_errors']:
                print('Runtime error: remaining sessions cancelled.\n' + str(result['harness_error']), flush=True)
                break
        else:
            complete = fingerprint() == current
            if not complete:
                invalid = 'Source changed during the test.'
    except KeyboardInterrupt:
        print('Interrupted.', flush=True)
        harness_error = 'Interrupted by user'
    except Exception:
        harness_error = traceback.format_exc()
        print(harness_error, flush=True)
    finally:
        if keep_awake:
            import ctypes
            ctypes.windll.kernel32.SetThreadExecutionState(keep_awake)
        monitor_ok, system_ok = monitor.alive(), system.alive()
        monitor.stop(); system.stop()
        manifest['snapshot_end'] = env_monitor.snapshot(directory, 'end')
    manifest.update(status='finished', finished_utc=datetime.now(timezone.utc).isoformat(), complete=complete,
                    session_boundaries=boundaries, gpu_monitor_alive_until_end=monitor_ok,
                    system_monitor_alive_until_end=system_ok, harness_error=harness_error)
    write(directory / 'manifest.json', manifest)
    status = finalize(directory, sessions, config, plan, manifest, complete, invalid, args.smoke)
    print((directory / 'REPORT.md').read_text(encoding='utf-8'), flush=True)
    print(f'{status}: {directory / "REPORT.md"}', flush=True)
    if status == 'PASS' or (status in ('SMOKE_ONLY', 'PARTIAL') and complete):
        return 0
    return 1 if status == 'FAIL' else 2


def report_only(directory):
    """Recompute every statistic and the verdict from the saved raw evidence."""
    config, manifest = read(directory / 'test-config.json'), read(directory / 'manifest.json')
    plan = manifest['plan']
    sessions = []
    for spec in plan['sessions']:
        path = directory / (spec['id'] + '.json')
        if not path.exists():
            continue
        saved = read(path)
        raw = directory / 'traces' / (spec['id'] + '.jsonl.gz')
        if digest(raw) != saved['raw_sha256']:
            print(f'INVALID: raw evidence changed for {spec["id"]}', flush=True)
            return 2
        attempts = read(directory / 'traces' / (spec['id'] + '-attempts.json'))
        ending = read(directory / 'traces' / (spec['id'] + '-ending.json'))
        result, _ = summarize(raw, attempts, ending, {k: spec[k] for k in ('id', 'phase', 'mode', 'pose_ids')},
                              config, score=not all('physical_accuracy' in a for a in attempts))
        sessions.append(result)
    status = finalize(directory, sessions, config, plan, manifest, manifest.get('complete', False),
                      None, manifest.get('smoke', False))
    print((directory / 'REPORT.md').read_text(encoding='utf-8'), flush=True)
    return 0 if status in ('PASS', 'SMOKE_ONLY', 'PARTIAL') else (1 if status == 'FAIL' else 2)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, help='New output directory (default: results/acceptance-test/<timestamp>)')
    parser.add_argument('--config', type=Path, help='Test configuration (default: tools/acceptance_test.json)')
    parser.add_argument('--long', action='store_true',
                        help='Long profile: 26-minute sustained run per method, under 1 hour in total')
    parser.add_argument('--baseline', type=Path, default=DEFAULT_BASELINE,
                        help='Manifest whose fingerprint defines the protected production inputs')
    parser.add_argument('--smoke', action='store_true', help='About 4 minutes; checks the harness, never an acceptance PASS')
    parser.add_argument('--no-system-monitor', action='store_true',
                        help='Do not log CPU frequency/thermal zones (removes the PowerShell sampler)')
    parser.add_argument('--method', choices=('sift', 'learned'), help='Diagnostic: run one method only (never PASS)')
    parser.add_argument('--report-only', type=Path, metavar='DIRECTORY', help='Recompute the report from saved evidence')
    parser.add_argument('--stale-resume-ms', type=float, default=DEFAULT_STALE_RESUME_MS, metavar='MS',
                        help='Watchdog hold-and-resume window (default: 2000 ms). 0 = stop, same as --watchdog-stop')
    parser.add_argument('--watchdog-stop', action='store_true',
                        help='A freshness trip ends the alignment (original behaviour) instead of hold and resume')
    args = parser.parse_args(argv)
    if args.watchdog_stop:
        args.stale_resume_ms = 0.0
    if not (0 <= args.stale_resume_ms <= 10000):
        parser.error('--stale-resume-ms must be between 0 and 10000')
    if args.long and args.config:
        parser.error('--long selects tools/acceptance_test_long.json; do not combine it with --config')
    args.config = args.config or (LONG_CONFIG if args.long else DEFAULT_CONFIG)
    if args.report_only:
        return report_only(args.report_only)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
