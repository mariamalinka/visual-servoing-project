"""The watchdog-pause test's session, reporting its capture-to-command breakdown. Usage: pause_margin.py TREE LABEL"""
def main():
    import json, os, sys
    tree, label = sys.argv[1], sys.argv[2]
    sys.path.insert(0, tree); os.chdir(tree)
    import time
    from realtime import RealtimeSession, RuntimeConfig
    session = RealtimeSession(config=RuntimeConfig(max_age_s=.4, transport_s=.05, inference_stall_s=.3, fault_after_s=.3,
                                                   stale_resume_s=2.), offset=[3, -3, 4, 3, -2, 2]).start()
    try:
        session.ready.wait(30)
        deadline = time.perf_counter() + 15
        while time.perf_counter() < deadline and session.snapshot().get('counters', {}).get('watchdog_resumes', 0) < 3:
            time.sleep(.01)
    finally:
        session.close()
    r = session.report()
    print(json.dumps(dict(label=label, frames_n=len(r["frames"]), status=session.snapshot().get("status"), error=(r.get("error") or "")[-150:], stops=[(e["reason"], round(e["at_s"]-r["events"][0]["at_s"],3)) for e in r["events"] if e["kind"]=="stop"], pauses=r["counts"]["watchdog_pauses"], resumes=r["counts"]["watchdog_resumes"], misses=r["counts"]["control_deadline_misses"], accepted=r["counts"]["accepted"], gapmax=r["control_gap_ms"]["max"])))
    frames = r["frames"]
    worst = max(frames, key=lambda f: f['capture_to_command_ms'])
    keys = ('capture_to_command_ms', 'queue_ms', 'render_ms', 'inference_ms', 'ipc_ms', 'delivery_wait_ms', 'control_ms', 'apply_ms')
    print(json.dumps(dict(label=label, frames=len(frames), over_400=sum(f['capture_to_command_ms'] >= 400 for f in frames),
                          worst={k: round(worst[k], 1) for k in keys},
                          ipc_max=round(max(f['ipc_ms'] for f in frames), 2))))


if __name__ == '__main__':
    main()
