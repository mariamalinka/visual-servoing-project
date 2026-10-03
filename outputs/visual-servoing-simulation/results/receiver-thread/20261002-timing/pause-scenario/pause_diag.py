"""Pause scenario with diagnostics; on a control_overrun, dump what the stalled tick contained.
Usage: pause_diag.py TREE LABEL"""


def main():
    import json, os, sys, time
    tree, label = sys.argv[1], sys.argv[2]
    sys.path.insert(0, tree); os.chdir(tree)
    from realtime import RealtimeSession, RuntimeConfig
    session = RealtimeSession(config=RuntimeConfig(max_age_s=.4, transport_s=.05, inference_stall_s=.3, fault_after_s=.3,
                                                   stale_resume_s=2.), offset=[3, -3, 4, 3, -2, 2], diagnostics=True).start()
    try:
        session.ready.wait(30)
        deadline = time.perf_counter() + 15
        while time.perf_counter() < deadline:
            s = session.snapshot()
            if s.get('counters', {}).get('watchdog_resumes', 0) >= 3 or s.get('status') == 'control_overrun':
                break
            time.sleep(.01)
    finally:
        session.close()
    r = session.report()
    stops = [e for e in r['events'] if e['kind'] == 'stop' and e['reason'] == 'control_overrun']
    out = dict(label=label, overrun=bool(stops), gapmax=r['control_gap_ms']['max'])
    if stops:
        at = stops[0]['at_s']
        spikes = [c for c in r['cycle_spikes'] if abs(c['finished_s'] - at) < .3]
        out['spikes'] = [dict(rel_ms=round(1000 * (c['started_s'] - at), 1), wall_ms=round(c['wall_ms'], 1),
                              cpu_ms=round(c['thread_cpu_ms'], 1), wake_late_ms=c['wake_lateness_ms'],
                              stages=[(s['stage'], round(s['wall_ms'], 1), round(s['thread_cpu_ms'], 1))
                                      for s in c['stages'] if s['wall_ms'] > 1]) for c in spikes]
        out['gc'] = [dict(g, rel_ms=round(1000 * (g.get('started_s', g.get('start_s', 0)) - at), 1))
                     for g in r['gc_events'] if abs(g.get('started_s', g.get('start_s', 0)) - at) < .3]
        out['frames_near'] = [dict(seq=f['sequence'], receiver_rel_ms=None if f.get('receiver_s') is None else round(1000 * (f['receiver_s'] - at), 1),
                                   received_rel_ms=round(1000 * (f['received_s'] - at), 1),
                                   finished_rel_ms=round(1000 * (f['finished_s'] - at), 1))
                              for f in r['sensor_frames'] if abs(f['received_s'] - at) < .3]
        out['align_to_stop_ms'] = round(1000 * (at - next(e['at_s'] for e in r['events'] if e['kind'] == 'align')), 1)
    print(json.dumps(out, default=str))


if __name__ == '__main__':
    main()
