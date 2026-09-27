"""Clock/queue contracts and real-process watchdog integration."""
import time
import unittest
from unittest.mock import patch
import threading
from queue import Queue, Empty
import numpy as np
from realtime import CommandLease, RuntimeConfig, RealtimeSession, put_latest, request_is_expired


class LeaseTests(unittest.TestCase):
    def setUp(self):
        self.lease = CommandLease(RuntimeConfig())
        self.lease.start(10.)

    def frame(self, captured=10.1, generation=1):
        return dict(captured_s=captured, generation=generation)

    def test_expired_delivery_cannot_extend_capture_age(self):
        self.assertTrue(self.lease.eligible(self.frame(), 10.2))
        self.lease.captured_s = 10.1
        self.assertEqual(self.lease.failure(10.351), 'stale_camera')
        self.assertFalse(self.lease.eligible(self.frame(), 10.351))

    def test_expired_initial_lease_is_independent_of_worker(self):
        self.assertIsNone(self.lease.failure(10.249))
        self.assertEqual(self.lease.failure(10.25), 'stale_camera')

    def test_stop_latches_and_rearm_rejects_previous_generation(self):
        self.lease.stop('stale_camera')
        self.assertFalse(self.lease.eligible(self.frame(), 10.2))
        self.lease.start(11.)
        self.assertFalse(self.lease.eligible(self.frame(11.01), 11.02))
        self.assertTrue(self.lease.eligible(self.frame(11.01, 2), 11.02))

    def test_duplicates_future_prestart_nan_and_reordered_frames_rejected(self):
        self.lease.captured_s = 10.1
        for stamp in (9., 10.05, 10.1, 11., float('nan')):
            with self.subTest(stamp=stamp):
                self.assertFalse(self.lease.eligible(self.frame(stamp), 10.2))

    def test_run_timeout_even_with_fresh_frames(self):
        lease = CommandLease(RuntimeConfig(max_run_s=1))
        lease.start(10.)
        lease.captured_s = 10.99
        self.assertEqual(lease.failure(11.), 'run_timeout')

    def test_queue_overload_keeps_memory_bounded_and_latest_request(self):
        queue = Queue(maxsize=1)
        drops = sum(put_latest(queue, i) for i in range(100))
        self.assertEqual(drops, 99)
        self.assertEqual(queue.get_nowait(), 99)
        with self.assertRaises(Empty):
            queue.get_nowait()

    def test_old_pending_work_is_dropped_before_rendering_or_matching(self):
        config=RuntimeConfig(max_age_s=.4,transport_s=.05)
        request={'captured_s':10.}
        self.assertFalse(request_is_expired(request,10.02,config))
        self.assertTrue(request_is_expired(request,10.051,config))
        self.assertTrue(request_is_expired(request,11.,config))
        self.assertTrue(request_is_expired(request,9.,config))
        tight=RuntimeConfig(max_age_s=.06,transport_s=.05)
        self.assertTrue(request_is_expired(request,10.02,tight))

    def test_invalid_configuration(self):
        for values in ({'max_age_s':0}, {'camera_hz':501}, {'control_hz':1001},
                       {'transport_s':-1}, {'inference_stall_s':float('nan')}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                RuntimeConfig(**values)

    def test_worker_reports_full_error_chain(self):
        from realtime import sensor_worker
        request, result = Queue(1), Queue(1)
        with patch('simulation.Simulation', side_effect=RuntimeError('test initialization failure')):
            sensor_worker(request, result, threading.Event(), 'aruco', True, False, RuntimeConfig())
        message = result.get_nowait()
        self.assertEqual(message['kind'], 'error')
        self.assertIn('Traceback', message['traceback'])
        self.assertIn('RuntimeError: test initialization failure', message['traceback'])

    def test_frontend_draws_during_initialization_without_live_physics(self):
        from realtime_app import draw_state
        image = draw_state(dict(status='initializing'), 'learned', RuntimeConfig())
        self.assertEqual(image.shape, (650,1280,3))
        self.assertEqual(image.dtype, np.uint8)


class RealtimeProcessTests(unittest.TestCase):
    def wait_state(self, session, predicate, seconds=15):
        deadline = time.perf_counter()+seconds
        while time.perf_counter() < deadline:
            state = session.snapshot()
            if state.get('error'):
                self.fail(state['error'])
            if predicate(state):
                return state
            time.sleep(.01)
        self.fail(str(session.snapshot()))

    def test_blocking_inference_cannot_pause_watchdog_or_rearm_motion(self):
        session = RealtimeSession(config=RuntimeConfig(inference_stall_s=.8, fault_after_s=.6),
                                  offset=[3,-3,4,3,-2,2]).start()
        try:
            self.assertTrue(session.ready.wait(30))
            state = self.wait_state(session, lambda s:s['status']=='stale_camera')
            self.assertGreater(state['counters']['accepted'], 0)
            self.assertGreater(state['counters']['moving_ticks'], 0)
            stopped_ticks = state['counters']['control_ticks']
            time.sleep(1.05)  # Let the obsolete result actually return.
            state = session.snapshot()
            self.assertEqual(state['status'], 'stale_camera')
            self.assertGreater(state['counters']['control_ticks']-stopped_ticks, 100)
            self.assertFalse(state['command'].any())
        finally:
            session.close()
        report = session.report()
        self.assertEqual(report['counts']['unsafe_motion_ticks'], 0)
        self.assertEqual(report['counts']['post_stop_motion_ticks'], 0)
        self.assertFalse(report['worker_alive'])
        event = next(e for e in report['events'] if e.get('reason')=='stale_camera')
        self.assertLess(event['stopped_s']-event['deadline_s'], .05)
        self.assertTrue(all(f['applied_s'] < event['stopped_s'] for f in report['frames']))

    def test_manual_stop_and_rearm_have_distinct_frame_generations(self):
        session = RealtimeSession(offset=[3,-3,4,3,-2,2]).start()
        try:
            self.assertTrue(session.ready.wait(30))
            self.wait_state(session, lambda s:s.get('counters',{}).get('accepted',0)>=2)
            session.command('stop')
            stopped = self.wait_state(session, lambda s:s['status']=='stopped')
            self.assertFalse(stopped['command'].any())
            first_generation = stopped['generation']
            time.sleep(.1)
            session.command('align')
            self.wait_state(session, lambda s:s.get('generation',0)>first_generation and
                            s.get('active') and s.get('age_ms') is not None)
        finally:
            session.close()
        report = session.report()
        self.assertEqual(report['counts']['unsafe_motion_ticks'], 0)
        self.assertFalse(report['worker_alive'])

    def test_slow_worker_skips_capture_slots_and_keeps_actual_snapshots_fresh(self):
        session=RealtimeSession(config=RuntimeConfig(max_age_s=.4,
            inference_stall_s=.07,fault_after_s=0),offset=[3,-3,4,3,-2,2]).start()
        try:
            self.assertTrue(session.ready.wait(30))
            state=self.wait_state(session,lambda s:s.get('counters',{}).get('accepted',0)>=8)
            self.assertGreater(state['counters']['busy_capture_skips'],0)
            self.assertEqual(state['counters']['input_dropped'],0)
            self.assertEqual(state['counters']['capture_slots'],
                state['counters']['captures']+state['counters']['busy_capture_skips'])
        finally:
            session.close()
        report=session.report()
        self.assertTrue(all(f['queue_ms']<50 for f in report['frames']))
        self.assertEqual(report['counts']['unsafe_motion_ticks'],0)

    def test_worker_exit_stops_and_shutdown_does_not_hang(self):
        session = RealtimeSession(offset=[3,-3,4,3,-2,2]).start()
        try:
            self.assertTrue(session.ready.wait(30))
            self.wait_state(session, lambda s:s.get('counters',{}).get('moving_ticks',0)>10)
            session.worker.terminate()
            deadline = time.perf_counter()+3
            while time.perf_counter()<deadline and session.snapshot()['status']!='runtime_error':
                time.sleep(.01)
            self.assertEqual(session.snapshot()['status'], 'runtime_error')
        finally:
            session.close()
        self.assertFalse(session.snapshot()['command'].any())
        self.assertIn('Sensor process exited', session.report()['error'])

    def test_terminated_waiting_worker_cannot_block_shutdown_notification(self):
        # Killing a waiter on multiprocessing.Event used to strand notify_all.
        session = RealtimeSession(config=RuntimeConfig(max_age_s=.4,
            inference_stall_s=.8, fault_after_s=.25), offset=[3,-3,4,3,-2,2]).start()
        try:
            self.assertTrue(session.ready.wait(30))
            state = self.wait_state(session, lambda s:s.get('now_s',0)-s.get('started_s',0)>.4)
            self.assertGreater(state['counters']['moving_ticks'], 0)
            session.worker.terminate()
            self.wait_state_after_worker_exit(session)
        finally:
            session.close()
        self.assertFalse(session.thread.is_alive())
        self.assertFalse(session.snapshot()['command'].any())
        self.assertFalse(session.report()['worker_alive'])

    def wait_state_after_worker_exit(self, session):
        deadline = time.perf_counter()+3
        while time.perf_counter()<deadline and session.snapshot()['status']!='runtime_error':
            time.sleep(.01)
        self.assertEqual(session.snapshot()['status'], 'runtime_error')

    def test_headless_physics_does_not_create_a_renderer(self):
        from simulation import Simulation
        with Simulation(render=False) as sim:
            self.assertIsNone(sim.renderer)
            sim.advance(.01)
            with self.assertRaisesRegex(RuntimeError, 'disabled'):
                sim.image()


if __name__ == '__main__':
    unittest.main()
