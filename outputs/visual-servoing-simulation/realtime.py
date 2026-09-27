"""Wall-clock visual servoing: one physics owner and one isolated sensor process.

The worker owns its OpenGL context and matcher. It receives copies of robot
state, never the live MjData. All deadlines use time.perf_counter() in seconds;
Windows/Python use the same monotonic clock across these processes.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, asdict, replace
import multiprocessing as mp
from queue import Empty, Full, Queue
import threading
import time
import traceback

import numpy as np


def put_latest(queue, value):
    """Nonblocking bounded mailbox. A feeder race may drop the new item too."""
    try:
        queue.put_nowait(value)
        return 0
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            return 1
        try:
            queue.put_nowait(value)
        except Full:
            pass
        return 1


@dataclass(frozen=True)
class RuntimeConfig:
    camera_hz: float = 30.0
    control_hz: float = 500.0
    max_age_s: float = .250
    transport_s: float = 0.0
    max_control_gap_s: float = .050
    max_run_s: float = 120.0
    max_queue_age_s: float = .050
    # Real fault injection, after this many seconds since the latest Align.
    fault_after_s: float = 2.0
    inference_stall_s: float = 0.0
    render_stall_s: float = 0.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not np.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ('camera_hz', 'control_hz', 'max_age_s', 'max_control_gap_s', 'max_run_s', 'max_queue_age_s'):
            if getattr(self, key) <= 0:
                raise ValueError(f"{key} must be positive")
        if self.control_hz > 1000 or self.camera_hz > self.control_hz:
            raise ValueError("Require camera_hz <= control_hz <= 1000")


def request_is_expired(request, now, config):
    age=now-request['captured_s']
    return (not np.isfinite(age) or age < 0 or age >= config.max_queue_age_s
            or age+config.transport_s >= config.max_age_s)


class CommandLease:
    """A stopped run cannot be revived by a late result or queue backlog."""
    def __init__(self, config):
        self.config = config
        self.generation = 0
        self.active = False
        self.started_s = 0.0
        self.captured_s = None
        self.status = 'idle'

    def start(self, now):
        self.generation += 1
        self.started_s = now
        self.captured_s = None
        self.active = True
        self.status = 'checking'

    def stop(self, reason):
        self.active = False
        self.status = reason

    def failure(self, now):
        if not self.active:
            return None
        if now < self.started_s or (self.captured_s is not None and now < self.captured_s):
            return 'clock_error'
        if now - self.started_s >= self.config.max_run_s:
            return 'run_timeout'
        origin = self.started_s if self.captured_s is None else self.captured_s
        return 'stale_camera' if now - origin >= self.config.max_age_s else None

    def eligible(self, frame, now):
        captured = frame['captured_s']
        return (self.active and frame['generation'] == self.generation
                and np.isfinite(captured) and self.started_s <= captured <= now
                and (self.captured_s is None or captured > self.captured_s)
                and now - captured < self.config.max_age_s)


class ShutdownReceiver:
    """The parent's pipe closure wakes the worker without a shared condition lock.

    Only the receive endpoint is passed to the spawned worker. EOF stays readable,
    so poll also provides an interruptible wait for injected sensor stalls.
    """
    def __init__(self, reader):
        self.reader = reader

    def is_set(self):
        return self.reader.poll(0)

    def wait(self, timeout):
        return self.reader.poll(timeout)


def sensor_worker(requests, results, shutdown, mode, precision, obstacle, config, diagnostics=False, worker_counts=None):
    """Spawn entry point; no live simulation objects cross the process boundary."""
    worker_gc=None
    scheduling=None
    try:
        from native_scheduling import SchedulingLease
        scheduling=SchedulingLease('sensor')
        if diagnostics:
            from deadline_diagnostics import GcProbe
            worker_gc=GcProbe().start()
        import cv2
        import mujoco
        from simulation import Simulation
        from app import make_perception
        from perception import NATURAL_REFERENCE
        from reference_image import DEFAULT_REFERENCE, load_reference
        from precision import GoalRefinedPerception, load_precision_config
        # Bound CPU parallelism while retaining parallel SIFT/ECC kernels.
        cv2.setNumThreads(4)
        with Simulation() as sensor:
            sensor.set_target_mode('aruco' if mode == 'aruco' else 'natural')
            if obstacle:
                sensor.set_obstacle(True)
            detector = make_perception(mode, sensor)
            path = DEFAULT_REFERENCE if mode == 'aruco' else NATURAL_REFERENCE
            rgb, corners = load_reference(path, sensor.camera_intrinsics(),
                detector.reference_config(sensor.config), lambda image: detector.observe(image).corners)
            detector.profile_diagnostics = diagnostics
            if precision:
                detector = GoalRefinedPerception(detector, rgb, corners, load_precision_config())
            detector.profile_diagnostics = diagnostics
            # Initialization, model load and first kernels happen at zero command.
            detector.observe(sensor.image())
            put_latest(results, dict(kind='ready', reference=corners, reference_rgb=rgb, backend=detector.name,
                execution_settings=dict(getattr(detector,'execution_settings',{}),scheduling=scheduling.metadata)))
            while not shutdown.is_set():
                try:
                    request = requests.get(timeout=.1)
                except Empty:
                    continue
                started = time.perf_counter()
                start_cpu = time.thread_time() if diagnostics else 0.
                start_process = time.process_time() if diagnostics else 0.
                if worker_counts is not None:
                    worker_counts[0] += 1
                if request_is_expired(request, started, config):
                    if worker_counts is not None:
                        worker_counts[3] += 1
                    put_latest(results, dict(kind='skipped', sequence=request['sequence'],
                        captured_s=request['captured_s'], generation=request['generation'],
                        reason='queue_expired'))
                    continue
                sensor.data.qpos[:] = request['qpos']
                sensor.data.qvel[:] = request['qvel']
                sensor.data.mocap_pos[:] = request['mocap_pos']
                sensor.data.mocap_quat[:] = request['mocap_quat']
                mujoco.mj_forward(sensor.model, sensor.data)
                fault = request['run_elapsed_s'] >= config.fault_after_s
                if fault and shutdown.wait(config.render_stall_s):
                    break
                rgb = sensor.image()
                rendered = time.perf_counter()
                rendered_cpu = time.thread_time() if diagnostics else 0.
                rendered_process = time.process_time() if diagnostics else 0.
                if fault and shutdown.wait(config.inference_stall_s):
                    break
                observation = detector.observe(rgb)
                finished = time.perf_counter()
                finished_cpu = time.thread_time() if diagnostics else 0.
                finished_process = time.process_time() if diagnostics else 0.
                if worker_counts is not None:
                    worker_counts[1] += 1
                result_drops = put_latest(results, dict(kind='frame', generation=request['generation'],
                    sequence=request['sequence'], captured_s=request['captured_s'],
                    qpos=request['qpos'], rgb=rgb, observation=observation,
                    stage_ms=observation.stage_ms, refinement=observation.refinement,
                    capture_active=request.get('capture_active',False),
                    worker_gc=[] if worker_gc is None else list(worker_gc.rows)[-8:],
                    cpu_ms=dict(render_thread=1000*(rendered_cpu-start_cpu),
                        render_process=1000*(rendered_process-start_process),
                        inference_thread=1000*(finished_cpu-rendered_cpu),
                        inference_process=1000*(finished_process-rendered_process)) if diagnostics else {},
                    render_started_s=started, rendered_s=rendered, finished_s=finished,
                    available_s=finished + config.transport_s, fault_injected=fault and
                    bool(config.render_stall_s or config.inference_stall_s)))
                if worker_counts is not None:
                    worker_counts[2] += result_drops
    except BaseException:
        put_latest(results, dict(kind='error', traceback=traceback.format_exc()))
    finally:
        if worker_gc: worker_gc.close()
        if scheduling: scheduling.close()


class RealtimeSession:
    """UI-independent runtime. The control thread exclusively owns live physics.

    UI and experiment code may send commands or read published copies. They must
    never access the live MjData. No rendering, inference, GUI or file I/O is done
    in the armed control loop. This is a measured software loop, not hard real time.
    """
    def __init__(self, mode='aruco', config=None, *, precision=True, calibration=None,
                 obstacle=False, offset=None, auto_start=True, diagnostics=False, telemetry_capacity=4096):
        if mode not in ('aruco', 'natural', 'learned'):
            raise ValueError('Unknown matcher')
        if type(diagnostics) is not bool or type(telemetry_capacity) is not int or not 256<=telemetry_capacity<=20000:
            raise ValueError('Invalid diagnostic configuration')
        self.diagnostics=diagnostics
        self.worker_counts=None
        self.cycle_probe=None
        self.gc_probe=None
        self.mode = mode
        self.config = config or RuntimeConfig()
        self.precision = precision
        self.calibration = calibration
        self.obstacle = obstacle
        self.offset = offset
        self.auto_start = auto_start
        self.commands = Queue(maxsize=8)
        self.stop_requested = threading.Event()
        self.closed = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self._state = dict(status='initializing', active=False, error=None, rgb=None)
        self.thread = None
        self.worker = None
        self.frame_rows = deque(maxlen=telemetry_capacity)
        self.sensor_rows = deque(maxlen=telemetry_capacity)
        self.loop_gaps = deque(maxlen=150000 if diagnostics else 100000)
        self.events = deque(maxlen=4096 if diagnostics else 256)
        self.counts = dict(captures=0, capture_slots=0, busy_capture_skips=0,
            input_dropped=0, transport_dropped=0, pending_cancelled=0, sensor_received=0,
            sensor_rows_evicted=0, command_rows_evicted=0, loop_rows_evicted=0,
            stale_results=0, obsolete_results=0, pre_inference_dropped=0, accepted=0, control_ticks=0,
            moving_ticks=0, unsafe_motion_ticks=0, contacts=0, post_stop_motion_ticks=0,
            control_deadline_misses=0, fault_results=0, discarded_physics_s=0.0)

    def start(self):
        if self.thread is not None:
            raise RuntimeError('A session can only be started once')
        self.thread = threading.Thread(target=self._run, name='servo-control', daemon=True)
        self.thread.start()
        return self

    def command(self, action):
        if action == 'stop':
            self.stop_requested.set()
        elif action in ('align', 'offset', 'reset', 'stream'):
            put_latest(self.commands, action)
        else:
            raise ValueError('Unknown runtime command')

    def snapshot(self):
        with self.lock:
            return dict(self._state)

    def close(self):
        self.closed.set()
        if self.thread is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError('Control loop did not shut down within five seconds')

    def _publish(self, **state):
        with self.lock:
            self._state.update(state)

    def _run(self):
        from simulation import Simulation
        from recovery import ReacquiringIBVS, ACTIVE_STATES
        from calibration import ControlCalibration
        from precision import load_precision_config
        context = mp.get_context('spawn')
        requests, results = context.Queue(1), context.Queue(1)
        shutdown_reader, shutdown_writer = context.Pipe(duplex=False)
        shutdown = ShutdownReceiver(shutdown_reader)
        sensor_available = True
        self.worker_counts=context.RawArray('q',4)
        if self.diagnostics:
            from deadline_diagnostics import CycleProbe, GcProbe
            self.cycle_probe=CycleProbe()
            self.gc_probe=GcProbe().start()
        probe=self.cycle_probe
        sim = None
        lease = CommandLease(self.config)
        pending = deque(maxlen=8)
        sequence = 0
        stream = True
        below_since = None
        last_frame = None
        next_capture = 0.0
        last_publish = 0.0
        stopped_at = None
        scheduling=None
        try:
            from native_scheduling import SchedulingLease
            scheduling=SchedulingLease('control')
            self._publish(control_scheduling=scheduling.metadata)
            self.worker = context.Process(target=sensor_worker,
                args=(requests, results, shutdown, self.mode, self.precision, self.obstacle, self.config, self.diagnostics, self.worker_counts),
                name='servo-sensor', daemon=True)
            self.worker.start()
            shutdown_reader.close()  # The worker owns the duplicated receive endpoint.
            # Constructed on this thread; headless physics has no OpenGL context.
            sim = Simulation(render=False)
            sim.set_target_mode('aruco' if self.mode == 'aruco' else 'natural')
            if self.obstacle:
                sim.set_obstacle(True)
            if self.offset is not None:
                sim.reset(self.offset)
            startup_deadline = time.perf_counter() + 120
            while not self.closed.is_set():
                try:
                    initial = results.get(timeout=.05)
                except Empty:
                    if not self.worker.is_alive() or time.perf_counter() >= startup_deadline:
                        raise RuntimeError('Sensor initialization failed or exceeded 120 seconds')
                    continue
                if initial['kind'] == 'error':
                    raise RuntimeError(initial['traceback'])
                if initial['kind'] == 'ready':
                    break
            else:
                return
            calibration = ControlCalibration(self.calibration)
            settings = calibration.controller_config(sim.config)
            precision_settings = load_precision_config()
            if self.precision:
                settings['precision_stop'] = precision_settings['stop'].copy()
            controller = ReacquiringIBVS(initial['reference'], calibration.intrinsics(sim.camera_intrinsics()),
                settings, sim.model.jnt_range, path_planner=sim.collision.plan_path,
                collision_detour_degrees=sim.collision.config['search_detour_degrees'])
            self._publish(reference_rgb=initial['reference_rgb'], reference=initial['reference'],
                          execution_settings=initial.get('execution_settings',{}),
                          status='idle', ready=True, backend=initial['backend'], home_qpos=sim.model.key('home').qpos.copy())
            period = 1 / self.config.control_hz
            last_tick = time.perf_counter()
            next_tick = last_tick + period

            def stop(reason, now):
                nonlocal below_since, stopped_at
                was_active = lease.active
                origin = lease.started_s if lease.captured_s is None else lease.captured_s
                lease.stop(reason)
                controller.cancel()
                sim.command_velocity(np.zeros(6))
                self.counts['pending_cancelled'] += len(pending)
                pending.clear()
                below_since = None
                if was_active:
                    stopped_at = now
                    self.events.append(dict(kind='stop', reason=reason, at_s=now, generation=lease.generation,
                        counters=dict(self.counts),
                        last_capture_s=lease.captured_s,
                        deadline_s=origin + self.config.max_age_s if reason == 'stale_camera' else None,
                        stopped_s=time.perf_counter()))

            def align(now):
                nonlocal below_since, next_capture, stopped_at
                stop('idle', now)
                lease.start(now)
                controller.start(cold=True)
                below_since = None
                next_capture = now
                stopped_at = None
                self.events.append(dict(kind='align', at_s=now, generation=lease.generation))

            if self.auto_start:
                align(last_tick)
            self.ready.set()
            while not self.closed.is_set():
                now = time.perf_counter()
                gap = now - last_tick
                if probe:
                    probe.finish()
                    probe.start(lease.active,lease.generation)
                if len(self.loop_gaps)==self.loop_gaps.maxlen:
                    self.counts['loop_rows_evicted'] += 1
                self.loop_gaps.append(gap * 1000)
                self.counts['control_ticks'] += 1
                last_tick = now
                if lease.active and gap > self.config.max_control_gap_s:
                    self.counts['control_deadline_misses'] += 1
                    stop('control_overrun', now)
                failure = lease.failure(now)
                if failure:
                    stop(failure, now)
                if not self.worker.is_alive():
                    stop('worker_failed', now)
                    try:
                        failed = results.get(timeout=.02)
                    except Empty:
                        failed = {}
                    raise RuntimeError(failed.get('traceback',
                        f'Sensor process exited with code {self.worker.exitcode}'))
                if self.stop_requested.is_set():
                    stop('stopped', now)
                    while True:
                        try:
                            self.commands.get_nowait()
                        except Empty:
                            break
                    self.stop_requested.clear()
                else:
                    try:
                        action = self.commands.get_nowait()
                    except Empty:
                        action = None
                    if action == 'align':
                        align(now)
                    elif action in ('reset', 'offset'):
                        stop('idle', now)
                        sim.reset(None if action == 'reset' else sim.config['ibvs']['start_offset_degrees'])
                    elif action == 'stream':
                        stream = not stream
                if probe: probe.mark('supervision')
                if now >= next_capture:
                    next_capture = now + 1 / self.config.camera_hz
                    worker_available=stream and sensor_available
                    if stream:
                        self.counts['capture_slots'] += 1
                        if not worker_available:
                            self.counts['busy_capture_skips'] += 1
                    if worker_available:
                        # Acquire a NEW robot-state snapshot only when the worker
                        # can consume it. Never re-date a queued image/state.
                        sensor_available = False
                        captured = time.perf_counter()
                        sequence += 1
                        request = dict(sequence=sequence, generation=lease.generation,
                            captured_s=captured, capture_active=lease.active, qpos=sim.data.qpos.copy(), qvel=sim.data.qvel.copy(),
                            mocap_pos=sim.data.mocap_pos.copy(), mocap_quat=sim.data.mocap_quat.copy(),
                            run_elapsed_s=captured-lease.started_s if lease.active else -1)
                        self.counts['input_dropped'] += put_latest(requests, request)
                        self.counts['captures'] += 1
                if probe: probe.mark('acquisition_enqueue')
                try:
                    received = results.get_nowait()
                except Empty:
                    received = None
                if received is not None:
                    if received['kind'] in ('frame', 'skipped'):
                        sensor_available = True
                    if received['kind'] == 'skipped':
                        self.counts['pre_inference_dropped'] += 1
                    if received['kind'] == 'error':
                        stop('worker_failed', time.perf_counter())
                        raise RuntimeError(received['traceback'])
                    if received['kind'] == 'frame':
                        received['received_s'] = time.perf_counter()
                        self.counts['fault_results'] += int(received['fault_injected'])
                        self.counts['sensor_received'] += 1
                        if len(self.sensor_rows)==self.sensor_rows.maxlen:
                            self.counts['sensor_rows_evicted'] += 1
                        self.sensor_rows.append({key: received[key] for key in
                            ('sequence', 'generation', 'captured_s', 'render_started_s',
                             'rendered_s', 'finished_s', 'received_s', 'fault_injected', 'stage_ms', 'refinement', 'capture_active', 'cpu_ms', 'worker_gc')})
                        self.sensor_rows[-1]['qpos']=received['qpos'].tolist()
                        if len(pending) == pending.maxlen:
                            self.counts['transport_dropped'] += 1
                        pending.append(received)
                if probe: probe.mark('receive_deserialize')
                now = time.perf_counter()
                frame = None
                waiting = deque(maxlen=8)
                for candidate in pending:
                    if candidate['generation'] != lease.generation:
                        self.counts['obsolete_results'] += 1
                    elif now - candidate['captured_s'] >= self.config.max_age_s:
                        self.counts['stale_results'] += 1
                    elif candidate['available_s'] > now:
                        waiting.append(candidate)
                    elif frame is None or candidate['captured_s'] > frame['captured_s']:
                        if frame is not None:
                            self.counts['transport_dropped'] += 1
                        frame = candidate
                pending = waiting
                # Check the old lease BEFORE allowing a new frame to refresh it.
                failure = lease.failure(now)
                if failure:
                    stop(failure, now)
                if probe: probe.mark('transport_dispatch')
                if frame is not None:
                    last_frame = frame
                    observation = frame['observation']
                    if observation.reason == 'inference_failed':
                        stop('inference_failed', now)
                    elif lease.eligible(frame, now):
                        dt = (1/self.config.camera_hz if lease.captured_s is None
                              else frame['captured_s']-lease.captured_s)
                        compute_started = time.perf_counter()
                        compute_cpu = time.thread_time() if self.diagnostics else 0.
                        sample = controller.update(observation.corners,
                            calibration.jacobian(sim.camera_jacobian()), sim.data.qpos.copy(), dt,
                            observation_qpos=frame['qpos'])
                        computed = time.perf_counter()
                        computed_cpu = time.thread_time() if self.diagnostics else 0.
                        if probe: probe.mark('controller_compute')
                        # No late command may extend an expired lease, including a
                        # controller/path planner that itself takes too long.
                        failure = lease.failure(computed)
                        if computed - last_tick > self.config.max_control_gap_s:
                            self.counts['control_deadline_misses'] += 1
                            stop('control_overrun', computed)
                        elif failure or not lease.eligible(frame, computed):
                            stop(failure or 'stale_camera', computed)
                        else:
                            if sample.stop_candidate:
                                if below_since is None:
                                    below_since = frame['captured_s']
                                if sample.status == 'converged' and (frame['captured_s']-below_since <
                                        settings['ibvs']['success_hold_s']):
                                    sample = replace(sample, status='running')
                                    controller.status = controller.ibvs.status = 'running'
                            else:
                                below_since = None
                            # Lease refresh and command application are one owner operation.
                            lease.captured_s = frame['captured_s']
                            sim.command_velocity(sample.velocity)
                            applied = time.perf_counter()
                            applied_cpu = time.thread_time() if self.diagnostics else 0.
                            if probe: probe.mark('command_application')
                            lease.status = sample.status
                            self.counts['accepted'] += 1
                            if len(self.frame_rows)==self.frame_rows.maxlen:
                                self.counts['command_rows_evicted'] += 1
                            self.frame_rows.append(dict(sequence=frame['sequence'], generation=lease.generation,
                                captured_s=frame['captured_s'], applied_s=applied,
                                queue_ms=1000*(frame['render_started_s']-frame['captured_s']),
                                render_ms=1000*(frame['rendered_s']-frame['render_started_s']),
                                inference_ms=1000*(frame['finished_s']-frame['rendered_s']),
                                ipc_ms=1000*(frame['received_s']-frame['finished_s']),
                                delivery_wait_ms=1000*(compute_started-frame['received_s']),
                                control_ms=1000*(computed-compute_started),
                                control_thread_cpu_ms=1000*(computed_cpu-compute_cpu),
                                apply_thread_cpu_ms=1000*(applied_cpu-computed_cpu),
                                apply_ms=1000*(applied-computed),
                                capture_to_command_ms=1000*(applied-frame['captured_s']),
                                error_px=sample.error_px, status=sample.status))
                            self._publish(error_px=sample.error_px, stop_metrics=sample.stop_metrics)
                            if sample.status not in ACTIVE_STATES:
                                stop(sample.status, applied)
                if probe: probe.mark('command_postprocess')
                # Recheck after command filtering and all queue/control work.
                now = time.perf_counter()
                failure = lease.failure(now)
                if failure:
                    stop(failure, now)
                moving = bool(np.any(sim.velocity_command))
                fresh = lease.captured_s is not None and now-lease.captured_s < self.config.max_age_s
                self.counts['moving_ticks'] += int(moving)
                self.counts['unsafe_motion_ticks'] += int(moving and (not lease.active or not fresh))
                self.counts['post_stop_motion_ticks'] += int(moving and stopped_at is not None)
                # Integrate measured elapsed time. Never replay a long backlog of
                # previously issued velocity commands after a scheduler stall.
                self.counts['discarded_physics_s'] += max(0, gap-self.config.max_control_gap_s)
                if probe: probe.mark('safety_checks')
                sim.advance(min(gap, self.config.max_control_gap_s))
                self.counts['contacts'] += len(sim.forbidden_contacts())
                if sim.collision_event is not None:
                    if lease.active and controller.skip_blocked_motion():
                        sim.command_velocity(np.zeros(6))
                        sim.collision_event = None
                    else:
                        stop('collision_blocked', time.perf_counter())
                        sim.collision_event = None
                if probe: probe.mark('physics_collision')
                now = time.perf_counter()
                if now-last_publish >= 1/30:
                    self._publish(status=lease.status, active=lease.active, stream=stream,
                        now_s=now, started_s=lease.started_s, simulation_s=float(sim.data.time),
                        age_ms=None if lease.captured_s is None else 1000*(now-lease.captured_s),
                        qpos=sim.data.qpos.copy(), command=sim.velocity_command.copy(),
                        rgb=None if last_frame is None else last_frame['rgb'],
                        observation=None if last_frame is None else last_frame['observation'],
                        counters=dict(self.counts), generation=lease.generation,
                        worker_pid=self.worker.pid, sensor_available=sensor_available)
                    last_publish = now
                if probe: probe.mark('state_publish')
                next_tick += period
                if next_tick < now:
                    next_tick = now + period
                # Python 3.11+ sleep uses a high-resolution Windows timer; Event.wait
                # can otherwise quantize a 2 ms control period to about 16 ms.
                if probe:
                    probe.mark('loop_bookkeeping')
                    probe.sleep_until=next_tick
                time.sleep(max(0, next_tick-time.perf_counter()))
        except BaseException:
            details = traceback.format_exc()
            self._publish(status='runtime_error', active=False, error=details)
            print(details, flush=True)
        finally:
            if probe: probe.finish()
            if self.gc_probe: self.gc_probe.close()
            if sim is not None:
                sim.command_velocity(np.zeros(6))
                self._publish(command=sim.velocity_command.copy(), active=False,
                              qpos=sim.data.qpos.copy(), counters=dict(self.counts))
                sim.close()
            # EOF wakes a live worker; no notification can wait on a dead worker.
            shutdown_writer.close()
            shutdown_reader.close()
            if self.worker is not None:
                self.worker.join(timeout=1)
                if self.worker.is_alive():
                    self.worker.terminate()
                    self.worker.join(timeout=1)
            # Unconsumed large frames must not keep a queue feeder alive on exit.
            for mailbox in (requests, results):
                mailbox.cancel_join_thread()
                mailbox.close()
            if scheduling: scheduling.close()
            self.ready.set()

    def report(self):
        """Read after close: raw measurements are retained for audit and plotting."""
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('Close the session before collecting its report')
        def percentiles(values):
            return None if not values else dict(mean=float(np.mean(values)), count=len(values),
                **dict(zip(('p50', 'p95', 'p99', 'p99_9', 'max'),
                map(float, np.percentile(values, [50, 95, 99, 99.9, 100])))))
        rows = list(self.frame_rows)
        fields = ('queue_ms', 'render_ms', 'inference_ms', 'ipc_ms', 'delivery_wait_ms',
                  'control_ms', 'apply_ms', 'capture_to_command_ms')
        return dict(mode=self.mode, backend=self.snapshot().get('backend'),
            execution_settings=self.snapshot().get('execution_settings',{}),
            control_scheduling=self.snapshot().get('control_scheduling',{}),
            config=asdict(self.config), counts=dict(self.counts),
            latency_ms={key: percentiles([row[key] for row in rows]) for key in fields},
            control_gap_ms=percentiles(list(self.loop_gaps)), events=list(self.events), frames=rows,
            sensor_frames=list(self.sensor_rows),
            diagnostics=self.diagnostics,
            cycle_spikes=[] if self.cycle_probe is None else list(self.cycle_probe.rows),
            cycle_spikes_evicted=0 if self.cycle_probe is None else self.cycle_probe.dropped,
            gc_events=[] if self.gc_probe is None else list(self.gc_probe.rows),
            gc_events_evicted=0 if self.gc_probe is None else self.gc_probe.dropped,
            worker_counts=None if self.worker_counts is None else dict(zip(
                ('requests_started','frames_completed','result_dropped','requests_expired'),self.worker_counts[:])),
            error=self.snapshot().get('error'), worker_alive=self.worker.is_alive() if self.worker else False)
