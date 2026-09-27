"""Aggregation for sustained latency sessions; failed attempts remain in all totals."""
from collections import Counter
import numpy as np


def distribution(values):
    values=np.asarray(list(values),dtype=float)
    if not len(values):
        return None
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite timing evidence')
    return dict(count=len(values),mean=float(values.mean()),**dict(zip(
        ('p50','p95','p99','p99_9','max'),map(float,np.percentile(values,[50,95,99,99.9,100])))))


def active_frames(row, uncached=False):
    return [f for f in row['sensor_frames'] if f['capture_active'] and
            (not uncached or not f['stage_ms'].get('cache_hit',False))]


def feedback_age_bound(row):
    ages=[]
    for align in (e for e in row['events'] if e['kind']=='align'):
        generation=align['generation']
        origin=align['at_s']
        stop=next((e for e in row['events'] if e['kind']=='stop' and
                   e['generation']==generation and e['stopped_s']>=origin),None)
        for f in (f for f in row['frames'] if f['generation']==generation):
            ages.append(1000*(f['applied_s']-origin))
            origin=f['captured_s']
        if stop:
            ages.append(1000*(stop['stopped_s']-origin))
    return max(ages,default=None)


def late_results(row):
    stops={e['generation']:e for e in row['events'] if e['kind']=='stop' and
           e['reason'] in ('stale_camera','control_overrun')}
    return [dict(sequence=f['sequence'],generation=f['generation'],
        stopped_s=stops[f['generation']]['stopped_s'],stop_reason=stops[f['generation']]['reason'],
        **{key:f[key] for key in ('captured_s','render_started_s','rendered_s','finished_s','received_s','stage_ms','cpu_ms')})
        for f in row['sensor_frames'] if f['capture_active'] and f['generation'] in stops and
        f['captured_s']<=stops[f['generation']]['stopped_s']<f['finished_s']]


def summarize_session(row):
    active=active_frames(row)
    uncached=active_frames(row,True)
    stage_values={}
    for f in active:
        for name,value in f['stage_ms'].items():
            if name.endswith('_ms'):
                stage_values.setdefault(name,[]).append(value)
        for name,value in f['cpu_ms'].items():
            stage_values.setdefault(name+'_cpu_ms',[]).append(value)
    wc=row['worker_counts']
    counters=dict(row['counts'])
    counters.update(result_mailbox_dropped=wc['result_dropped'],
        worker_requests_expired=wc['requests_expired'],
        completed_but_unreceived=max(0,wc['frames_completed']-counters['sensor_received']-wc['result_dropped']),
        requests_not_started=max(0,counters['captures']-wc['requests_started']-counters['input_dropped']))
    late=late_results(row)
    return dict(id=row['id'],mode=row['mode'],attempts=len(row['attempts']),
        alignments=sum(a['outcome']=='converged' for a in row['attempts']),
        outcomes=dict(Counter(a['outcome'] for a in row['attempts'])),
        elapsed_s=row['elapsed_s'],initialization_error=row.get('initialization_error'),
        processing_ms=distribution(1000*(f['finished_s']-f['rendered_s']) for f in active),
        uncached_processing_ms=distribution(1000*(f['finished_s']-f['rendered_s']) for f in uncached),
        capture_to_command_ms=distribution(f['capture_to_command_ms'] for f in row['frames']),
        stage_ms={key:distribution(values) for key,values in stage_values.items()},
        maximum_feedback_age_ms=feedback_age_bound(row),
        freshness_watchdog_trips=sum(e.get('reason')=='stale_camera' for e in row['events'] if e['kind']=='stop'),
        late_after_freshness_stop=sum(f['stop_reason']=='stale_camera' for f in late),
        late_after_control_stop=sum(f['stop_reason']=='control_overrun' for f in late),
        late_results=late,counters=counters,worker_alive=row['worker_alive'],error=row['error'],
        telemetry_complete=not any(counters[k] for k in ('sensor_rows_evicted','command_rows_evicted','loop_rows_evicted')),
        diagnostic_spikes_evicted=row['cycle_spikes_evicted'],gc_events_evicted=row['gc_events_evicted'])
