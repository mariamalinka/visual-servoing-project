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
    # Fraction (0..1) of faulted frames whose perception result is replaced by an
    # 'inference_failed' observation, as a failed Learned GPU inference returns.
    # 1 = every frame after fault_after_s; 0.5 = every second one (deterministic).
    inference_failure: float = 0.0
    # Freshness watchdog response (see docs/WATCHDOG_DECISION.md). The trip itself
    # is identical either way: zero velocity at the same moment and image age.
    # > 0 (default): hold and resume. The alignment is paused, resumes on the next
    #   fresh frame, and ends as 'stale_camera' only if none arrives in this many s.
    # 0: stop. The trip ends the alignment (the original behaviour).
    stale_resume_s: float = 2.0

    def __post_init__(self):
        for key, value in asdict(self).items():
            if isinstance(value, bool) or not np.isfinite(value) or value < 0:
                raise ValueError(f"{key} must be finite and nonnegative")
        for key in ('camera_hz', 'control_hz', 'max_age_s', 'max_control_gap_s', 'max_run_s', 'max_queue_age_s'):
            if getattr(self, key) <= 0:
                raise ValueError(f"{key} must be positive")
        if self.control_hz > 1000 or self.camera_hz > self.control_hz:
            raise ValueError("Require camera_hz <= control_hz <= 1000")
        if self.inference_failure > 1:
            raise ValueError("inference_failure must be a fraction in [0, 1]")


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
        self.paused_s = None
        self.status = 'idle'

    def start(self, now):
        self.generation += 1
        self.started_s = now
        self.captured_s = None
        self.paused_s = None
        self.active = True
        self.status = 'checking'

    def stop(self, reason):
        self.active = False
        self.paused_s = None
        self.status = reason

    def pause(self, now):
        """Zero-velocity hold after a freshness trip; the alignment stays alive."""
        self.paused_s = now
        self.status = 'paused'

    def failure(self, now):
        if not self.active:
            return None
        if now < self.started_s or (self.captured_s is not None and now < self.captured_s):
            return 'clock_error'
        if now - self.started_s >= self.config.max_run_s:
            return 'run_timeout'
        origin = self.started_s if self.captured_s is None else self.captured_s
        if now - origin < self.config.max_age_s:
            return None
        # Already holding at zero velocity: only the resume window can end the run.
        if self.paused_s is not None and now - self.paused_s < self.config.stale_resume_s:
            return None
        return 'stale_camera'

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


class ResultReceiver:
    """The only reader of the sensor results pipe; forwards whole messages to `local`.

    A pipe read can block for good: a worker killed halfway through sending a frame
    leaves a length header and part of the message, and the reader waits for the rest.
    (Python's timeout only bounds the wait for data to appear, not for a message to
    finish.) So the control thread never reads the pipe. It takes messages from
    `local` without blocking and detects a dead worker itself (worker.is_alive).

    The thread runs at normal priority, does no physics and shares nothing with the
    control loop except `local`. Unpickling a frame happens here, not in the loop.
    If it stays blocked at shutdown, it is a daemon and is left behind.
    """
    POLL_S = .1

    def __init__(self, results):
        self.results = results
        # A one-item, latest-wins mailbox, like the pipe it replaces.
        self.local = Queue(maxsize=1)
        self.stopping = threading.Event()
        self.forwarded = 0
        self.dropped = 0  # Unread messages replaced by a newer one (none expected).
        self.ended = None
        self.thread = threading.Thread(target=self._run, name='servo-receiver', daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _run(self):
        try:
            while not self.stopping.is_set():
                try:
                    message = self.results.get(timeout=self.POLL_S)
                except Empty:
                    continue
                message['receiver_s'] = time.perf_counter()
                self._deliver(message)
                self.forwarded += 1
            self.ended = 'stopped'
        except (EOFError, OSError) as exc:
            # Every send end is closed: the worker exited, possibly mid-message. This
            # notice never displaces an unread message (the worker's error traceback);
            # the control loop then detects the exit through is_alive() instead.
            self.ended = f'pipe_closed ({type(exc).__name__}: {exc})'
            self._notify(dict(kind='pipe_closed', detail=self.ended))
        except BaseException:
            if self.stopping.is_set():
                self.ended = 'stopped'  # The queue was closed during shutdown.
            else:
                # The thread ends here and no further results arrive, so this must
                # reach the control loop even if a message is still unread.
                self.ended = 'error'
                self._deliver(dict(kind='receiver_error', traceback=traceback.format_exc()))

    def _deliver(self, message):
        """Latest wins. This thread is the only producer, so the retry always ends, and
        unlike put_latest() the new message is never the one dropped."""
        while True:
            try:
                self.local.put_nowait(message)
                return
            except Full:
                try:
                    self.local.get_nowait()
                    self.dropped += 1
                except Empty:
                    pass  # The control loop took it in between.

    def _notify(self, message):
        try:
            self.local.put_nowait(message)
        except Full:
            pass

    def get_nowait(self):
        try:
            return self.local.get_nowait()
        except Empty:
            return None

    def worker_error(self, worker, wait_s=.02):
        """The dead worker's own traceback if it sent one in time, else its exit code."""
        deadline = time.perf_counter() + wait_s
        while True:
            try:
                message = self.local.get(timeout=max(0., deadline - time.perf_counter()))
            except Empty:
                break
            if message.get('kind') in ('error', 'receiver_error'):
                return message['traceback']
        worker.join(timeout=max(0., deadline - time.perf_counter()))
        return f'Sensor process exited with code {worker.exitcode}'

    def close(self, timeout=.5):
        """Stop and wait a bounded time; never wait on a read that cannot finish."""
        self.stopping.set()
        joined = time.perf_counter()
        if self.thread.is_alive():
            self.thread.join(timeout)
        return dict(forwarded=self.forwarded, dropped=self.dropped,
                    ended=self.ended, stuck_at_close=self.thread.is_alive(),
                    join_ms=1000*(time.perf_counter()-joined))


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
        from perception import NATURAL_REFERENCE, Observation
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
            failure_budget = 0.0  # Deterministic spacing of injected inference failures.
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
                failed = False
                if fault and config.inference_failure:
                    failure_budget += config.inference_failure
                    failed = failure_budget >= 1 - 1e-9
                    failure_budget -= failed
                observation = Observation(reason='inference_failed') if failed else detector.observe(rgb)
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
                    bool(config.render_stall_s or config.inference_stall_s or failed)))
                if worker_counts is not None:
                    worker_counts[2] += result_drops
    except BaseException:
        # Bounded wait, not put_latest: the parent's receiver holds the read lock while it
        # waits for data, so put_latest could not displace an unread frame and would
        # drop the traceback. The receiver drains the pipe, so the put normally succeeds.
        try:
            results.put(dict(kind='error', traceback=traceback.format_exc()), timeout=1)
        except Full:
            pass
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
                 obstacle=False, offset=None, auto_start=True, diagnostics=False, telemetry_capacity=4096,
                 actuator=None):
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
        # Actuator dynamics (actuator.py): None = actuator_config.json's active profile,
        # or a profile name / dict. Stops still zero the command at once; the robot
        # then brakes physically, and each stop is measured in the report.
        self.actuator = actuator
        self.actuator_description = None
        self.motion_log = ()
        self.motion_evicted = None
        self.safety_source = None  # Simulation.safety_record: clearance, joint margin, speed extremes.
        self.commands = Queue(maxsize=8)
        self.stop_requested = threading.Event()
        self.closed = threading.Event()
        self.ready = threading.Event()
        self.lock = threading.Lock()
        self._state = dict(status='initializing', active=False, error=None, rgb=None)
        self.thread = None
        self.worker = None
        self.receiver_state = None  # ResultReceiver.close(): forwarded, backlog, whether it was stuck.
        self.frame_rows = deque(maxlen=telemetry_capacity)
        self.sensor_rows = deque(maxlen=telemetry_capacity)
        self.loop_gaps = deque(maxlen=150000 if diagnostics else 100000)
        self.events = deque(maxlen=4096 if diagnostics else 256)
        self.counts = dict(captures=0, capture_slots=0, busy_capture_skips=0,
            input_dropped=0, transport_dropped=0, pending_cancelled=0, sensor_received=0,
            sensor_rows_evicted=0, command_rows_evicted=0, loop_rows_evicted=0,
            stale_results=0, obsolete_results=0, pre_inference_dropped=0, accepted=0, control_ticks=0,
            moving_ticks=0, unsafe_motion_ticks=0, contacts=0, post_stop_motion_ticks=0,
            control_deadline_misses=0, fault_results=0, discarded_physics_s=0.0,
            watchdog_pauses=0, watchdog_resumes=0, paused_s=0.0, paused_motion_ticks=0,
            receiver_dropped=0)  # Results replaced unread in the receiver's mailbox.

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
        receiver = None
        try:
            from native_scheduling import SchedulingLease
            scheduling=SchedulingLease('control')
            self._publish(control_scheduling=scheduling.metadata)
            self.worker = context.Process(target=sensor_worker,
                args=(requests, results, shutdown, self.mode, self.precision, self.obstacle, self.config, self.diagnostics, self.worker_counts),
                name='servo-sensor', daemon=True)
            self.worker.start()
            shutdown_reader.close()  # The worker owns the duplicated receive endpoint.
            # Likewise the worker owns the send end of the results pipe; the parent never
            # sends on it. Without the parent's copy, a worker that dies mid-message closes
            # the last send end, so a blocked read ends (EOF) instead of waiting forever.
            results._writer.close()
            receiver = ResultReceiver(results).start()
            # Constructed on this thread; headless physics has no OpenGL context.
            sim = Simulation(render=False, actuator_config=self.actuator)
            self.actuator_description = sim.actuator.describe()
            self.motion_log = sim.motion_log
            self.motion_evicted = lambda: sim.motion_evicted
            self.safety_source = sim.safety_record
            sim.set_target_mode('aruco' if self.mode == 'aruco' else 'natural')
            if self.obstacle:
                sim.set_obstacle(True)
            if self.offset is not None:
                sim.reset(self.offset)
            startup_deadline = time.perf_counter() + 120
            while not self.closed.is_set():
                try:
                    initial = receiver.local.get(timeout=.05)
                except Empty:
                    initial = dict(kind='pending')
                if initial['kind'] in ('error', 'receiver_error'):
                    raise RuntimeError(initial['traceback'])
                if initial['kind'] in ('pending', 'pipe_closed'):
                    if not self.worker.is_alive() or time.perf_counter() >= startup_deadline:
                        raise RuntimeError('Sensor initialization failed or exceeded 120 seconds')
                    continue
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
                paused_for = None if lease.paused_s is None else now - lease.paused_s
                if paused_for is not None:
                    self.counts['paused_s'] += paused_for
                lease.stop(reason)
                controller.cancel()
                sim.command_velocity(np.zeros(6), reason=reason)
                self.counts['pending_cancelled'] += len(pending)
                pending.clear()
                below_since = None
                if was_active:
                    stopped_at = now
                    self.events.append(dict(kind='stop', reason=reason, at_s=now, generation=lease.generation,
                        counters=dict(self.counts),
                        last_capture_s=lease.captured_s,
                        deadline_s=None if reason != 'stale_camera' else origin + self.config.max_age_s
                                   if paused_for is None else now - paused_for + self.config.stale_resume_s,
                        paused_for_s=paused_for,
                        stopped_s=time.perf_counter(), command_serial=sim.command_serial,
                        moving_at_command=sim.is_moving()))

            def pause(now):
                """Freshness trip with stale_resume_s > 0: identical zero command, run kept."""
                nonlocal below_since
                origin = lease.started_s if lease.captured_s is None else lease.captured_s
                lease.pause(now)
                sim.command_velocity(np.zeros(6), reason='watchdog_pause')
                below_since = None  # The success hold restarts on fresh images.
                self.counts['watchdog_pauses'] += 1
                self.events.append(dict(kind='pause', reason='stale_camera', at_s=now, generation=lease.generation,
                    counters=dict(self.counts), last_capture_s=lease.captured_s,
                    deadline_s=origin + self.config.max_age_s,
                    resume_deadline_s=now + self.config.stale_resume_s,
                    stopped_s=time.perf_counter(), command_serial=sim.command_serial,
                    moving_at_command=sim.is_moving()))

            def watchdog(reason, now):
                """Route a lease failure: a freshness trip pauses when enabled, else stops."""
                if reason == 'stale_camera' and self.config.stale_resume_s > 0 and lease.active and (
                        lease.paused_s is None or now - lease.paused_s < self.config.stale_resume_s):
                    if lease.paused_s is None:
                        pause(now)
                    return
                stop(reason, now)

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
                    watchdog(failure, now)
                if not self.worker.is_alive():
                    stop('worker_failed', now)
                    raise RuntimeError(receiver.worker_error(self.worker))
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
                received = receiver.get_nowait()  # Never the pipe: that read can block.
                if received is not None:
                    if received['kind'] in ('frame', 'skipped'):
                        sensor_available = True
                    if received['kind'] == 'skipped':
                        self.counts['pre_inference_dropped'] += 1
                    if received['kind'] in ('error', 'receiver_error'):
                        stop('worker_failed', time.perf_counter())
                        raise RuntimeError(received['traceback'])
                    if received['kind'] == 'pipe_closed':
                        # The worker's send end closed before is_alive() reported its exit.
                        stop('worker_failed', time.perf_counter())
                        raise RuntimeError(receiver.worker_error(self.worker) + ' (result pipe closed)')
                    if received['kind'] == 'frame':
                        received['received_s'] = time.perf_counter()
                        self.counts['fault_results'] += int(received['fault_injected'])
                        self.counts['sensor_received'] += 1
                        if len(self.sensor_rows)==self.sensor_rows.maxlen:
                            self.counts['sensor_rows_evicted'] += 1
                        self.sensor_rows.append({key: received[key] for key in
                            ('sequence', 'generation', 'captured_s', 'render_started_s',
                             'rendered_s', 'finished_s', 'receiver_s', 'received_s', 'fault_injected', 'stage_ms', 'refinement', 'capture_active', 'cpu_ms', 'worker_gc')})
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
                    watchdog(failure, now)
                if probe: probe.mark('transport_dispatch')
                if frame is not None:
                    last_frame = frame
                    observation = frame['observation']
                    if observation.reason == 'inference_failed':
                        stop('inference_failed', now)
                    elif lease.eligible(frame, now):
                        dt = (1/self.config.camera_hz if lease.captured_s is None
                              else frame['captured_s']-lease.captured_s)
                        if lease.paused_s is not None:
                            # A normal run never exceeds max_age_s between accepted
                            # frames; the controller does not integrate over the hold.
                            dt = min(dt, self.config.max_age_s)
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
                            if lease.paused_s is not None and not failure:
                                self.counts['stale_results'] += 1  # Went stale during compute; hold stays.
                            watchdog(failure or 'stale_camera', computed)
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
                            if lease.paused_s is not None:
                                self.counts['watchdog_resumes'] += 1
                                self.counts['paused_s'] += computed - lease.paused_s
                                self.events.append(dict(kind='resume', at_s=computed, generation=lease.generation,
                                    paused_for_s=computed - lease.paused_s, captured_s=frame['captured_s'],
                                    command_serial=sim.command_serial + 1))  # The command applied next.
                                lease.paused_s = None
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
                    watchdog(failure, now)
                moving = bool(np.any(sim.velocity_command))
                fresh = lease.captured_s is not None and now-lease.captured_s < self.config.max_age_s
                self.counts['moving_ticks'] += int(moving)
                self.counts['unsafe_motion_ticks'] += int(moving and (not lease.active or not fresh))
                self.counts['post_stop_motion_ticks'] += int(moving and stopped_at is not None)
                self.counts['paused_motion_ticks'] += int(moving and lease.paused_s is not None)
                # Integrate measured elapsed time. Never replay a long backlog of
                # previously issued velocity commands after a scheduler stall.
                self.counts['discarded_physics_s'] += max(0, gap-self.config.max_control_gap_s)
                if probe: probe.mark('safety_checks')
                # A clock that stepped backwards (a clock_error stop) gives a negative
                # gap; integrate nothing rather than failing the runtime.
                sim.advance(min(max(gap, 0.0), self.config.max_control_gap_s))
                self.counts['contacts'] += len(sim.forbidden_contacts())
                if sim.collision_event is not None:
                    if lease.active and controller.skip_blocked_motion():
                        sim.command_velocity(np.zeros(6), reason='collision_blocked')
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
                sim.command_velocity(np.zeros(6), reason='shutdown')
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
            # After the worker has gone, so a read blocked on a half message has ended
            # (EOF). A receiver that is still blocked is left behind (daemon thread).
            if receiver is not None:
                self.receiver_state = receiver.close()
                self.counts['receiver_dropped'] = self.receiver_state['dropped']
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
            error=self.snapshot().get('error'), worker_alive=self.worker.is_alive() if self.worker else False,
            actuator=self.actuator_description, motion=list(self.motion_log),
            motion_evicted=0 if self.motion_evicted is None else self.motion_evicted(),
            receiver=self.receiver_state,
            safety=None if self.safety_source is None else self.safety_source())
