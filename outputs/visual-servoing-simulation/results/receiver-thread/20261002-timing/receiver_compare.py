"""One latency-stress session from a given source tree, with per-thread CPU and per-tick stage timing.

Usage: receiver_compare.py TREE LABEL SECONDS FULL_PROBE(0/1) OUT.json

Reuses run_latency_stress.run_session (the project's sustained-session procedure:
SIFT/natural, 400 ms age limit, 50 ms transport, stop setting, repeated offset+align)
and latency_stress.summarize_session. FULL_PROBE=1 replaces the cycle probe's 10 ms
spike threshold by 0 so every control cycle's stages are recorded (same in both trees).
"""
def main():
    import json
    import os
    import sys
    import threading
    import time

    tree, label, seconds, full_probe, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4] == '1', sys.argv[5]
    sys.path.insert(0, tree)
    os.chdir(tree)
    import numpy as np
    import deadline_diagnostics as dd

    if full_probe:
        class EveryCycle(dd.CycleProbe):
            def __init__(self, threshold_ms=10., capacity=2048):
                super().__init__(threshold_ms=0., capacity=60000)
        dd.CycleProbe = EveryCycle

    import realtime
    import run_latency_stress as rls
    from latency_stress import summarize_session, distribution

    TICK = os.sysconf('SC_CLK_TCK')


    def stat_cpu(path):
        try:
            fields = open(path).read().rsplit(')', 1)[1].split()
        except (FileNotFoundError, ProcessLookupError, TypeError):
            return None
        return (int(fields[11]) + int(fields[12])) / TICK


    def sample(session):
        names = {t.name: t.native_id for t in threading.enumerate()}
        return dict(t=time.perf_counter(), process=time.process_time(),
                    control=stat_cpu(f'/proc/self/task/{session.thread.native_id}/stat'),
                    receiver=None if 'servo-receiver' not in names else stat_cpu(f"/proc/self/task/{names['servo-receiver']}/stat"),
                    worker=None if session.worker is None else stat_cpu(f'/proc/{session.worker.pid}/stat'))


    samples = {}
    original_start, original_close = realtime.RealtimeSession.start, realtime.RealtimeSession.close


    def start(self):
        result = original_start(self)

        def first():
            self.ready.wait(130)
            time.sleep(1.0)  # Past the auto-start transient.
            samples['start'] = sample(self)
        threading.Thread(target=first, daemon=True).start()
        return result


    def close(self):
        samples['end'] = sample(self)
        return original_close(self)


    realtime.RealtimeSession.start, realtime.RealtimeSession.close = start, close
    rls.RealtimeSession = realtime.RealtimeSession

    row = rls.run_session('natural', seconds, [3.0, -3.0, 4.0, 3.0, -2.0, 2.0], True)
    row.update(id=f'{label}-session', mode='natural', round=0)
    summary = summarize_session(row)

    a, b = samples.get('start'), samples.get('end')
    elapsed = b['t'] - a['t']
    cpu = {k: None if a.get(k) is None or b.get(k) is None else 100 * (b[k] - a[k]) / elapsed
           for k in ('process', 'control', 'receiver', 'worker')}

    cycles = [c for c in row['cycle_spikes'] if c['started_s'] >= a['t']] if full_probe else []


    def stage(name, key='wall_ms'):
        return distribution(s[key] for c in cycles for s in c['stages'] if s['stage'] == name) if cycles else None


    busy = [sum(s['wall_ms'] for s in c['stages'] if s['stage'] != 'sleep_or_descheduled') for c in cycles]
    busy_cpu = [sum(s['thread_cpu_ms'] for s in c['stages'] if s['stage'] != 'sleep_or_descheduled') for c in cycles]
    wake = [c['wake_lateness_ms'] for c in cycles if c['wake_lateness_ms'] is not None]
    frames = [f for f in row['sensor_frames'] if f['received_s'] >= a['t']]
    pipe = [1000 * (f['receiver_s'] - f['finished_s']) for f in frames if f.get('receiver_s')]
    handoff = [1000 * (f['received_s'] - f['receiver_s']) for f in frames if f.get('receiver_s')]
    ipc = [1000 * (f['received_s'] - f['finished_s']) for f in frames]

    result = dict(label=label, tree=tree, seconds=seconds, full_probe=full_probe, measured_s=elapsed,
                  realtime_sha=__import__('hashlib').sha256(open(os.path.join(tree, 'realtime.py'), 'rb').read()).hexdigest()[:12],
                  cpu_percent=cpu,
                  control_gap_ms=row['control_gap_ms'],
                  control_ticks=row['counts']['control_ticks'],
                  control_deadline_misses=row['counts']['control_deadline_misses'],
                  stops=summary['outcomes'], attempts=summary['attempts'], alignments=summary['alignments'],
                  error=row.get('error'), initialization_error=row.get('initialization_error'),
                  frames_received=row['counts']['sensor_received'],
                  frames_per_s=len(frames) / elapsed,
                  ipc_ms=distribution(ipc), pipe_and_unpickle_ms=distribution(pipe) if pipe else None,
                  local_queue_wait_ms=distribution(handoff) if handoff else None,
                  latency_ms=row['latency_ms'],
                  capture_to_command_ms=summary['capture_to_command_ms'],
                  maximum_feedback_age_ms=summary['maximum_feedback_age_ms'],
                  freshness_watchdog_trips=summary['freshness_watchdog_trips'],
                  unsafe_motion_ticks=row['counts']['unsafe_motion_ticks'],
                  receive_stage_wall_ms=stage('receive_deserialize'),
                  receive_stage_cpu_ms=stage('receive_deserialize', 'thread_cpu_ms'),
                  busy_per_tick_ms=distribution(busy) if busy else None,
                  busy_cpu_per_tick_ms=distribution(busy_cpu) if busy_cpu else None,
                  wake_lateness_ms=distribution(wake) if wake else None,
                  cycles_recorded=len(cycles), cycles_dropped=row['cycle_spikes_evicted'],
                  receiver=row.get('receiver'))
    with open(out, 'w') as stream:
        json.dump(result, stream, indent=1, default=float)
    print(label, 'done', json.dumps(dict(cpu=cpu, misses=result['control_deadline_misses'], gap=row['control_gap_ms'],
                                         error=(row.get('error') or '')[-100:])))


if __name__ == '__main__':
    main()
