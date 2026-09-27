"""Small interactive front end for the wall-clock runtime."""
from __future__ import annotations
import cv2
import numpy as np
from realtime import RealtimeSession, RuntimeConfig


def draw_state(state, mode, config):
    from app import label
    canvas = np.full((650, 1280, 3), (16, 22, 31), np.uint8)
    for x, key in ((0, 'rgb'), (640, 'reference_rgb')):
        rgb = state.get(key)
        if rgb is not None:
            canvas[40:520, x:x+640] = rgb
    label(canvas, f"Live camera | {mode} | WALL CLOCK", 16, 26, .65)
    label(canvas, 'Saved image goal', 656, 26, .65)
    status = state['status']
    age = state.get('age_ms')
    error = state.get('error_px')
    label(canvas, f"Status: {status}   Age: {'--' if age is None else f'{age:.0f}'} ms"
          f" / {config.max_age_s*1000:.0f} ms   Error: {'--' if error is None else f'{error:.3f}'} px",
          16, 546, .65)
    counts = state.get('counters', {})
    label(canvas, f"Accepted: {counts.get('accepted',0)}   Busy captures skipped: {counts.get('busy_capture_skips',0)}   Replaced: "
          f"{counts.get('input_dropped',0)}   Transport: {config.transport_s*1000:.0f} ms"
          f"   Camera: {'on' if state.get('stream',True) else 'off'}", 16, 574)
    label(canvas, 'G: Align   Space: Stop   O: Offset   R: Reset   X: Camera on/off   Esc: Exit',
          16, 604, .55)
    label(canvas, '1: ArUco   2: SIFT   3: Learned (switching stops motion; press G after loading)',
          16, 632, .52)
    if state.get('error'):
        label(canvas, 'Runtime failed. Full error printed in the console. Switch mode or exit.',
              20, 110, .7, (255, 150, 110))
    return canvas


def run_interactive(args):
    from app import random_start_offset
    from calibration import calibration_profiles
    from startup_search import load_startup_config
    from simulation import ROOT
    import json
    mode = 'learned' if args.learned else 'natural' if args.natural else 'aruco'
    offset = (load_startup_config()['demo_offset_degrees'] if args.cold_start else
              random_start_offset(args.seed) if args.random_start else None)
    config = RuntimeConfig(transport_s=(args.camera_delay_ms or 0)/1000,
                           max_age_s=args.max_camera_age_ms/1000)
    def create(selected, start, auto):
        return RealtimeSession(selected, config, precision=not args.legacy_stop,
            calibration=calibration_profiles()[args.calibration_profile],
            obstacle=args.obstacle, offset=start, auto_start=auto).start()
    session = create(mode, offset, not args.manual)
    window = 'Visual servoing | Wall-clock control'
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1280, 650)
        while True:
            state = session.snapshot()
            canvas = draw_state(state, mode, config)
            cv2.imshow(window, cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(25) & 0xff
            if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            action = {ord('g'): 'align', 32: 'stop', ord('o'): 'offset',
                      ord('r'): 'reset', ord('x'): 'stream'}.get(key)
            if action:
                session.command(action)
            if key in (ord('1'), ord('2'), ord('3')):
                selected = {ord('1'): 'aruco', ord('2'): 'natural', ord('3'): 'learned'}[key]
                session.close()
                final = session.snapshot()
                # Preserve the actual stopped joints when selecting a different matcher.
                home = final.get('home_qpos')
                qpos = final.get('qpos')
                start = None if home is None or qpos is None else np.rad2deg(qpos-home).tolist()
                mode, session = selected, create(selected, start, False)
    finally:
        session.close()
        cv2.destroyAllWindows()
        directory = ROOT / 'logs'
        directory.mkdir(exist_ok=True)
        (directory/'realtime-last.json').write_text(json.dumps(session.report(), indent=2,
            allow_nan=False), encoding='utf-8')
