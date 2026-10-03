"""Aggregate receiver_compare.py sessions into a before/after table (markdown + JSON)."""
import glob
import json
import os
import sys

import numpy as np

folder = sys.argv[1]


def load(kind, label):
    return [json.load(open(p)) for p in sorted(glob.glob(os.path.join(folder, f'{kind}-{label}-*.json')))]


def pooled(rows, getter):
    values = [getter(r) for r in rows]
    return [v for v in values if v is not None]


def cell(values, fmt='{:.2f}'):
    if not values:
        return 'n/a'
    if len(values) == 1:
        return fmt.format(values[0])
    return f"{fmt.format(np.mean(values))} ({fmt.format(min(values))}–{fmt.format(max(values))})"


def dist(rows, key, stat):
    return pooled(rows, lambda r: (r.get(key) or {}).get(stat))


def table(kind):
    before, after = load(kind, 'before'), load(kind, 'after')
    lines = [f"Sessions: {len(before)} before, {len(after)} after; each 60 s SIFT (natural), interleaved.",
             "Cells: mean of per-session values (range across sessions).", "",
             "| Metric | Before (control thread reads the pipe) | After (receiver thread) |", "|---|---|---|"]
    rows = [
        ("Control-loop gap p50 (ms)", lambda r: dist(r, 'control_gap_ms', 'p50')),
        ("Control-loop gap p99 (ms)", lambda r: dist(r, 'control_gap_ms', 'p99')),
        ("Control-loop gap p99.9 (ms)", lambda r: dist(r, 'control_gap_ms', 'p99_9')),
        ("Control-loop gap max (ms)", lambda r: dist(r, 'control_gap_ms', 'max')),
        ("Control deadline misses (> 50 ms)", lambda r: pooled(r, lambda x: x['control_deadline_misses'])),
        ("Freshness-watchdog stops", lambda r: pooled(r, lambda x: x['freshness_watchdog_trips'])),
        ("Frames received per second", lambda r: pooled(r, lambda x: x['frames_per_s'])),
        ("Alignments converged / attempts", None),
        ("IPC: worker finished → control loop has it, p50 (ms)", lambda r: dist(r, 'ipc_ms', 'p50')),
        ("IPC p99 (ms)", lambda r: dist(r, 'ipc_ms', 'p99')),
        ("IPC max (ms)", lambda r: dist(r, 'ipc_ms', 'max')),
        ("Capture to command p50 (ms)", lambda r: dist(r, 'capture_to_command_ms', 'p50')),
        ("Capture to command p99 (ms)", lambda r: dist(r, 'capture_to_command_ms', 'p99')),
        ("Capture to command max (ms)", lambda r: dist(r, 'capture_to_command_ms', 'max')),
        ("Maximum feedback age (ms)", lambda r: pooled(r, lambda x: x['maximum_feedback_age_ms'])),
        ("CPU: control thread (% of one core)", lambda r: pooled(r, lambda x: x['cpu_percent']['control'])),
        ("CPU: receiver thread (% of one core)", lambda r: pooled(r, lambda x: x['cpu_percent']['receiver'])),
        ("CPU: whole control process (% of one core)", lambda r: pooled(r, lambda x: x['cpu_percent']['process'])),
        ("CPU: sensor worker process (% of one core)", lambda r: pooled(r, lambda x: x['cpu_percent']['worker'])),
    ]
    if kind == 'full':
        rows += [
            ("Per tick: receive step wall p50 (ms)", lambda r: dist(r, 'receive_stage_wall_ms', 'p50')),
            ("Per tick: receive step wall p99.9 (ms)", lambda r: dist(r, 'receive_stage_wall_ms', 'p99_9')),
            ("Per tick: receive step wall max (ms)", lambda r: dist(r, 'receive_stage_wall_ms', 'max')),
            ("Per tick: receive step CPU max (ms)", lambda r: dist(r, 'receive_stage_cpu_ms', 'max')),
            ("Per tick: busy time p50 (ms)", lambda r: dist(r, 'busy_per_tick_ms', 'p50')),
            ("Per tick: busy time p99 (ms)", lambda r: dist(r, 'busy_per_tick_ms', 'p99')),
            ("Per tick: busy time p99.9 (ms)", lambda r: dist(r, 'busy_per_tick_ms', 'p99_9')),
            ("Per tick: busy time max (ms)", lambda r: dist(r, 'busy_per_tick_ms', 'max')),
            ("Per tick: wake-up lateness p99 (ms)", lambda r: dist(r, 'wake_lateness_ms', 'p99')),
            ("Per tick: wake-up lateness max (ms)", lambda r: dist(r, 'wake_lateness_ms', 'max')),
        ]
    rows += [("After only: pipe read + unpickle in receiver, p50 / max (ms)", None),
             ("After only: wait in local queue until the control loop takes it, p50 / max (ms)", None),
             ("After only: receiver mailbox drops / stuck at close", None)]
    summary = {}
    for label, getter in rows:
        if label.startswith('Alignments'):
            b = f"{sum(r['alignments'] for r in before)} / {sum(r['attempts'] for r in before)}"
            a = f"{sum(r['alignments'] for r in after)} / {sum(r['attempts'] for r in after)}"
        elif label.startswith('After only: pipe'):
            b, a = '—', f"{cell(dist(after, 'pipe_and_unpickle_ms', 'p50'))} / {cell(dist(after, 'pipe_and_unpickle_ms', 'max'))}"
        elif label.startswith('After only: wait'):
            b, a = '—', f"{cell(dist(after, 'local_queue_wait_ms', 'p50'))} / {cell(dist(after, 'local_queue_wait_ms', 'max'))}"
        elif label.startswith('After only: receiver'):
            b = '—'
            a = (f"{sum((r['receiver'] or {}).get('dropped', 0) for r in after)} / "
                 f"{sum(bool((r['receiver'] or {}).get('stuck_at_close')) for r in after)} of {len(after)}")
        else:
            if 'misses' in label or 'stops' in label:
                vb, va = getter(before), getter(after)
                b = f"{sum(vb)} total (per session {', '.join(map(str, vb))})"
                a = f"{sum(va)} total (per session {', '.join(map(str, va))})"
            else:
                fmt = '{:.3f}' if 'receive step' in label else '{:.2f}'
                b, a = cell(getter(before), fmt), cell(getter(after), fmt)
        summary[label] = dict(before=b, after=a)
        lines.append(f"| {label} | {b} | {a} |")
    errors = [(r['label'], r['error'] or r['initialization_error']) for r in before + after
              if r['error'] or r['initialization_error']]
    lines += ["", f"Runtime errors: {len(errors)}"] + [f"- {l}: {(e or '')[-200:]}" for l, e in errors]
    return lines, summary


out = {}
text = []
for kind, title in (('full', 'Every control cycle recorded (cycle probe threshold 0 ms, same in both)'),
                    ('spike', 'Stock diagnostics (cycle probe records spikes ≥ 10 ms only), as in the stress procedure')):
    if not glob.glob(os.path.join(folder, f'{kind}-*.json')):
        continue
    lines, summary = table(kind)
    text += [f"### {title}", ""] + lines + [""]
    out[kind] = summary
print('\n'.join(text))
json.dump(out, open(os.path.join(folder, 'aggregate.json'), 'w'), indent=1)
