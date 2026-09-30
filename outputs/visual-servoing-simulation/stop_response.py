"""Physical stop response in realtime reports (analysis only; no production logic).

A realtime stop has three different durations that must not be confused:

    image-age limit reached (deadline_s)
      | command stop latency   deadline -> zero velocity command   (wall clock)
    zero command (stopped_s)
      | physical stopping time zero command -> robot at standstill (simulated time)
    standstill: every joint below stopped_velocity_rad_s for stopped_hold_s

The physical stopping distance is the motion between the zero command and
standstill (joint angles, camera and tool displacement). RealtimeSession records
the zero command in its events and Simulation measures the motion in its
motion_log; this module joins them by the command serial number that both record.

The numbers describe the simulated arm with the modelled actuator dynamics
(actuator_config.json). They are not measurements of a real robot.
"""
from __future__ import annotations

import numpy as np

STOP_KINDS = ('stop', 'pause')


def link(report: dict) -> list[dict]:
    """One row per zero command issued by the runtime while an alignment was active."""
    motion = report.get('motion') or []
    stops = {serial: row for row in motion if row['kind'] == 'stop' for serial in row['command_serials']}
    starts = {serial: row for row in motion if row['kind'] == 'start' for serial in row['command_serials']}
    # With a full motion log the oldest records were evicted: those events cannot be judged.
    oldest = min((row['command_serials'][0] for row in motion), default=None) if report.get('motion_evicted') else None
    rows = []
    for event in report.get('events', []):
        if 'command_serial' not in event:
            continue  # A run from before the actuator model: nothing was measured.
        key = event['command_serial']
        if oldest is not None and key < oldest:
            if event['kind'] == 'resume':
                continue
            rows.append(dict(kind=event['kind'], reason=event.get('reason'), at_s=event['at_s'], moving=None,
                             outcome='not_recorded', command_latency_ms=None, physical_stop_ms=None))
            continue
        if event['kind'] == 'resume':
            start = starts.get(key)
            rows.append(dict(kind='resume', reason='watchdog_resume', at_s=event['at_s'],
                             # No start record: the resumed command was zero, or was blocked or
                             # stopped before the next physics step.
                             outcome='not_started' if start is None else start['outcome'],
                             stop_clamps=0 if start is None else start.get('stop_clamps', 0),
                             duration_ms=None if start is None else 1000 * start['duration_s'],
                             peak_speed_rad_s=None if start is None else start['peak_speed_rad_s'],
                             reached_command_ms=None if start is None or start.get('reached_command_s') is None
                             else 1000 * start['reached_command_s'],
                             peak_setpoint_accel_rad_s2=None if start is None else start['peak_setpoint_accel_rad_s2'],
                             peak_setpoint_jerk_rad_s3=None if start is None else start['peak_setpoint_jerk_rad_s3'],
                             peak_measured_accel_rad_s2=None if start is None else start['peak_measured_accel_rad_s2']))
            continue
        if event['kind'] not in STOP_KINDS:
            continue
        episode = stops.get(key)
        lateness = None
        if event.get('deadline_s') is not None and event.get('stopped_s') is not None:
            lateness = 1000 * (event['stopped_s'] - event['deadline_s'])
        row = dict(kind=event['kind'], reason=event.get('reason'), at_s=event['at_s'],
                   command_latency_ms=lateness, moving=episode is not None)
        if episode is None and event.get('moving_at_command'):
            # Moving, but no physics step followed (shutdown, or resumed in the same tick).
            row.update(moving=True, outcome='not_measured', physical_stop_ms=None)
        elif episode is None:  # Already at rest when the zero command came.
            row.update(outcome='at_rest', physical_stop_ms=0.0, setpoint_stop_ms=0.0, speed_at_command_rad_s=0.0,
                       max_joint_travel_rad=0.0, camera_travel_mm=0.0, tool_travel_mm=0.0,
                       camera_rotation_deg=0.0, peak_measured_accel_rad_s2=0.0, peak_setpoint_jerk_rad_s3=0.0,
                       peak_measured_jerk_rad_s3=0.0)
        else:
            row.update(outcome=episode['outcome'],
                       physical_stop_ms=None if episode.get('stop_time_s') is None else 1000 * episode['stop_time_s'],
                       setpoint_stop_ms=None if episode.get('setpoint_stop_time_s') is None
                       else 1000 * episode['setpoint_stop_time_s'],
                       duration_ms=1000 * episode['duration_s'], stop_clamps=episode.get('stop_clamps', 0),
                       **{k: episode[k] for k in ('speed_at_command_rad_s', 'max_joint_travel_rad', 'camera_travel_mm',
                                                  'tool_travel_mm', 'camera_rotation_deg',
                                                  'peak_measured_accel_rad_s2', 'peak_setpoint_accel_rad_s2',
                                                  'peak_setpoint_jerk_rad_s3', 'peak_measured_jerk_rad_s3')})
        if lateness is not None and row['physical_stop_ms'] is not None:
            row['deadline_to_standstill_ms'] = lateness + row['physical_stop_ms']
        rows.append(row)
    return rows


def _stats(values):
    values = [v for v in values if v is not None]
    if not values:
        return None
    return dict(count=len(values), median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values)))


def summarize(rows: list[dict]) -> dict:
    """Counts and distributions of the physical stop metrics, by kind."""
    result = {}
    for kind in ('stop', 'pause', 'resume'):
        subset = [r for r in rows if r['kind'] == kind]
        if not subset:
            continue
        if kind == 'resume':
            result[kind] = dict(count=len(subset), not_started=sum(r['outcome'] == 'not_started' for r in subset),
                                stop_clamps=sum(r.get('stop_clamps', 0) for r in subset),
                                reached_command_ms=_stats([r['reached_command_ms'] for r in subset]),
                                peak_setpoint_accel_rad_s2=_stats([r['peak_setpoint_accel_rad_s2'] for r in subset]),
                                peak_setpoint_jerk_rad_s3=_stats([r['peak_setpoint_jerk_rad_s3'] for r in subset]))
            continue
        moving = [r for r in subset if r['moving']]
        result[kind] = dict(
            count=len(subset), moving=len(moving), not_recorded=sum(r['outcome'] == 'not_recorded' for r in subset),
            completed=sum(r['outcome'] == 'stopped' for r in moving),
            interrupted=sum(r['outcome'] == 'resumed' for r in moving),
            other=sum(r['outcome'] not in ('stopped', 'resumed') for r in moving),
            stop_clamps=sum(r.get('stop_clamps', 0) for r in moving),
            reasons=sorted({str(r['reason']) for r in subset}),
            command_latency_ms=_stats([r.get('command_latency_ms') for r in subset]),
            physical_stop_ms=_stats([r.get('physical_stop_ms') for r in moving]),
            deadline_to_standstill_ms=_stats([r.get('deadline_to_standstill_ms') for r in moving]),
            speed_at_command_rad_s=_stats([r.get('speed_at_command_rad_s') for r in moving]),
            max_joint_travel_rad=_stats([r.get('max_joint_travel_rad') for r in moving]),
            camera_travel_mm=_stats([r.get('camera_travel_mm') for r in moving]),
            camera_rotation_deg=_stats([r.get('camera_rotation_deg') for r in moving]),
            peak_measured_accel_rad_s2=_stats([r.get('peak_measured_accel_rad_s2') for r in moving]),
            peak_setpoint_jerk_rad_s3=_stats([r.get('peak_setpoint_jerk_rad_s3') for r in moving]))
    return result


def merge(summaries: list[dict]) -> dict:
    """Not a re-computation: combine counts and take the worst (max) of each metric."""
    merged = {}
    for summary in summaries:
        for kind, block in (summary or {}).items():
            target = merged.setdefault(kind, {})
            for key, value in block.items():
                if value is None:
                    target.setdefault(key, None)
                elif isinstance(value, dict):
                    old = target.get(key)  # None if an earlier summary had no values for this key.
                    target[key] = value if old is None else dict(count=old['count'] + value['count'],
                                                                 max=max(old['max'], value['max']))
                elif isinstance(value, list):
                    target[key] = sorted(set(target.get(key, [])) | set(value))
                elif isinstance(value, (int, float)):
                    target[key] = target.get(key, 0) + value
    return merged


def format_lines(summary: dict, actuator: dict | None = None) -> list[str]:
    """Plain-text lines for reports."""
    def ms(block):
        if not block:
            return 'n/a'
        return (f"median {block['median']:.0f} / max {block['max']:.0f} ms" if 'median' in block
                else f"max {block['max']:.0f} ms")
    lines = []
    if actuator is not None:
        if actuator.get('enabled', True):
            lines.append(f"Actuator model: {actuator.get('profile')} (acceleration {actuator['max_acceleration_rad_s2']}, "
                         f"deceleration {actuator['max_deceleration_rad_s2']} rad/s^2, jerk "
                         f"{actuator['max_jerk_rad_s3']} rad/s^3; simulation assumption)")
        else:
            lines.append(f"Actuator model: {actuator.get('profile')} (disabled: command applied to the servo at once)")
    if not summary:
        return lines + ['No stops or holds while an alignment was active.']
    for kind, title in (('stop', 'Stops'), ('pause', 'Holds')):
        block = summary.get(kind)
        if not block:
            continue
        lines.append(f"{title}: {block['count']} ({block['moving']} while moving; {block['completed']} reached "
                     f"standstill, {block['interrupted']} resumed first, {block.get('other', 0)} other; "
                     f"{block.get('stop_clamps', 0)} jerk-limit exceptions)")
        lines.append(f"  command stop latency after the image-age limit: {ms(block.get('command_latency_ms'))}")
        lines.append(f"  physical stopping time (zero command to standstill): {ms(block.get('physical_stop_ms'))}")
        lines.append(f"  image-age limit to standstill: {ms(block.get('deadline_to_standstill_ms'))}")
        travel = block.get('camera_travel_mm')
        if travel:
            lines.append(f"  stopping distance: camera max {travel['max']:.1f} mm, joints max "
                         f"{block['max_joint_travel_rad']['max']:.4f} rad")
    resume = summary.get('resume')
    if resume:
        lines.append(f"Resumes: {resume['count']}; time to reach the command {ms(resume.get('reached_command_ms'))}")
    return lines
