"""Measure actual latency and overload response (nondeterministic wall clock)."""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import numpy as np
from camera_robustness import write_json
from realtime import RealtimeSession, RuntimeConfig

PROFILES = {
    'nominal': {},
    'transport-50ms': dict(transport_s=.05),
    'inference-stall': dict(inference_stall_s=.8),
    'render-stall': dict(render_stall_s=.8),
}


def run_trial(mode, profile, config, offset, preview_path=None):
    session = RealtimeSession(mode, config, offset=offset).start()
    outcome = 'initialization_timeout'
    last_active = None
    final = {}
    try:
        if session.ready.wait(125):
            deadline = time.perf_counter()+config.max_run_s+3
            while time.perf_counter() < deadline:
                state = session.snapshot()
                if state.get('active') and state.get('rgb') is not None:
                    last_active = state
                if state.get('error') or (state.get('ready') and not state['active'] and
                                          state['status'] != 'idle'):
                    outcome = state['status']
                    break
                time.sleep(.01)
            else:
                outcome = 'experiment_timeout'
            # Keep physics and perception alive so late results actually return.
            # The settling window checks commands, not whether qvel is instantly zero.
            time.sleep(1.1)
            final = session.snapshot()
    finally:
        session.close()
    if preview_path is not None and last_active is not None:
        from PIL import Image
        from realtime_app import draw_state
        Image.fromarray(draw_state(last_active, mode, config)).save(preview_path)
    report = session.report()
    expected = 'stale_camera' if 'stall' in profile else 'converged'
    stops = [e for e in report['events'] if e.get('reason') == 'stale_camera']
    lateness = None if not stops else 1000*(stops[-1]['stopped_s']-stops[-1]['deadline_s'])
    fault_frames = [f for f in report['sensor_frames'] if f['fault_injected']]
    stopped_during_fault = bool(stops and any(f['render_started_s'] < stops[-1]['stopped_s'] <
                                            f['finished_s'] for f in fault_frames))
    counts = report['counts']
    safe = not any(counts[key] for key in ('unsafe_motion_ticks', 'post_stop_motion_ticks', 'contacts'))
    fault_pass = ('stall' not in profile or (stopped_during_fault and lateness is not None
                  and 0 <= lateness <= config.max_control_gap_s*1000 and counts['moving_ticks'] > 0))
    # Physical response to the watchdog stop (simulated actuator model; informational, not gated).
    from stop_response import link
    physical = next((r for r in link(report) if r['kind'] == 'stop' and r['reason'] == 'stale_camera'), None)
    report.update(profile=profile, expected=expected, outcome=outcome,
        offset_degrees=offset, final_error_px=final.get('error_px'), physical_stop=physical,
        watchdog_lateness_ms=lateness, stopped_while_sensor_busy=stopped_during_fault,
        expected_pass=bool(outcome == expected and final.get('status') == outcome and safe
                           and fault_pass and not report['worker_alive'] and not report['error']))
    return report


def physical_text(row):
    stop = row.get('physical_stop')
    if not stop:
        return '--'
    if not stop['moving']:
        return 'at rest'
    return ('not reached' if stop['physical_stop_ms'] is None else
            f"{stop['physical_stop_ms']:.0f} / {stop['camera_travel_mm']:.1f}")


def write_report(directory, rows):
    def metric(row, field, percentile='p95'):
        values = row['latency_ms'].get(field)
        return '--' if values is None else f"{values[percentile]:.1f}"
    lines = ['# Wall-clock latency and overload experiment', '',
        'Measured on the recorded host. Each matcher uses the same joint offset and precision stop. '
        'The configured camera budget includes queueing, rendering, inference, IPC and transport. '
        'The control loop requests a 2 ms period; measured gaps are reported below.', '',
        '| Matcher | Profile | Budget (ms) | Outcome | Pass | Command age p95 / p99 (ms) | Loop gap p99 (ms) | Watchdog lateness (ms) '
        '| Physical stop (ms) / camera travel (mm) |',
        '|---|---|---:|---|---|---:|---:|---:|---:|']
    for row in rows:
        gap = row['control_gap_ms']
        gap_text = '--' if gap is None else f"{gap['p99']:.2f}"
        late = row['watchdog_lateness_ms']
        late_text = '--' if late is None else f'{late:.2f}'
        lines.append(f"| {row['mode']} | {row['profile']} | {row['config']['max_age_s']*1000:.0f} | {row['outcome']} | "
            f"{'yes' if row['expected_pass'] else 'NO'} | {metric(row,'capture_to_command_ms')} / "
            f"{metric(row,'capture_to_command_ms','p99')} | {gap_text} | {late_text} | {physical_text(row)} |")
    lines += ['', f"Passed: {sum(row['expected_pass'] for row in rows)}/{len(rows)}.", '',
        'The two stall profiles block the actual sensor worker for 800 ms after two seconds. '
        'A passing stall trial must first issue motion, stop while the worker is still busy, '
        'return zero commands after stopping, and remain stopped when its late result arrives. '
        'The maximum allowed watchdog lateness is 50 ms; this is a test bound, not a hard real-time guarantee.', '',
        'Watchdog lateness is the command stop latency (limit to zero command). The physical stop column is the '
        'simulated time from that zero command until the arm is at standstill, and the camera displacement meanwhile, '
        'with the actuator model in actuator_config.json (docs/ACTUATOR_MODEL.md). It is informational, not a pass '
        'criterion, and not real-robot data.', '',
        'Raw accepted-frame timings and all received sensor timestamps are in each trial JSON. '
        'Initialization and model warm-up occur with zero command before arming. '
        'Accepted-frame percentiles exclude rejected stale frames; sensor_frames retains their timings. '
        'All buffers are bounded (one pending request, one pending result, eight delayed results; '
        '4096 telemetry frames and 100000 loop gaps). Busy capture slots are skipped before acquisition; '
        'capture timestamps are never moved forward on an old frame.', '',
        'These are visible-start wall-clock trials, not a repeat of the full physical calibration '
        'or cold-search campaigns. Host load changes timing. Results should be repeated on the deployment machine.']
    (directory/'REPORT.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--latency-study', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--modes', nargs='+', choices=('aruco','natural','learned'), default=['aruco','natural','learned'])
    parser.add_argument('--profiles', nargs='+', choices=tuple(PROFILES), default=list(PROFILES))
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--max-camera-age-ms', type=float, default=400,
                        help='Experiment freshness budget, including time between results (default: 400 ms)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if len(args.modes)!=len(set(args.modes)) or len(args.profiles)!=len(set(args.profiles)):
        parser.error('Modes and profiles must be unique')
    if not 1 <= args.repeats <= 100:
        parser.error('--repeats must be in [1, 100]')
    try:
        RuntimeConfig(max_age_s=args.max_camera_age_ms/1000, stale_resume_s=0)
    except ValueError as exc:
        parser.error(str(exc))
    from simulation import ROOT
    from run_camera_robustness import fingerprint
    output = args.output or ROOT/'results'/'latency'/datetime.now().strftime('%Y%m%d-%H%M%S')
    # Never overwrite completed evidence or mix fingerprints in one directory.
    output.mkdir(parents=True, exist_ok=False)
    offset = json.loads((ROOT/'config.json').read_text(encoding='utf-8'))['ibvs']['start_offset_degrees']
    specs = [dict(id=f'{mode}-{profile}-{repeat:02d}', mode=mode, profile=profile,
                  # Validates the stop response: stall trials must stop and stay stopped.
                  config=asdict(RuntimeConfig(max_age_s=args.max_camera_age_ms/1000, stale_resume_s=0,
                                              **PROFILES[profile])))
             for repeat in range(args.repeats) for mode in args.modes for profile in args.profiles]
    write_json(output/'manifest.json', dict(schema_version=1,
        started_utc=datetime.now(timezone.utc).isoformat(), fingerprint=fingerprint(),
        offset_degrees=offset, trials=specs, clock=time.get_clock_info('perf_counter').implementation))
    rows = []
    for spec in specs:
        row = run_trial(spec['mode'], spec['profile'], RuntimeConfig(**dict(dict(stale_resume_s=0), **spec['config'])), offset,
                        output/(spec['id']+'.png') if spec['profile']=='nominal' else None)
        row['id'] = spec['id']
        write_json(output/(spec['id']+'.json'), row)
        rows.append(row)
        write_json(output/'summary.json', [{key:value for key,value in r.items()
            if key not in ('frames','sensor_frames')} for r in rows])
        write_report(output, rows)
        print(f"{spec['id']}: {row['outcome']}; pass={row['expected_pass']}; "
              f"age p95={row['latency_ms']['capture_to_command_ms']}; "
              f"watchdog late={row['watchdog_lateness_ms']}", flush=True)
    return 0 if all(row['expected_pass'] for row in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
