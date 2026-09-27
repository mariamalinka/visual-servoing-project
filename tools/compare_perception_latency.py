"""Compare complete transport runs, retaining stopped trials and drop accounting."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def stats(values):
    values=list(values)
    if not values:
        return None
    return dict(count=len(values),mean=float(np.mean(values)),
        **dict(zip(('p95','p99','max'),map(float,np.percentile(values,[95,99,100])))))


def load_trials(directory):
    manifest=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    specs=manifest['trials']
    rows=[json.loads((directory/(spec['id']+'.json')).read_text(encoding='utf-8')) for spec in specs]
    assert len({r['id'] for r in rows})==len(rows)
    return manifest,rows


def summarize(rows,mode):
    selected=[r for r in rows if r['mode']==mode]
    applied=[f['capture_to_command_ms'] for r in selected for f in r['frames']]
    stages={};sensor=[];fresh_age=[];held_age=[]
    for r in selected:
        stop=next(e for e in r['events'] if e['kind']=='stop')['stopped_s']
        active=[f for f in r['sensor_frames'] if f['captured_s']<=stop]
        origin=next(e for e in r['events'] if e['kind']=='align')['at_s']
        for frame in r['frames']:
            # Conservative upper bound: the preceding lease ends before application.
            held_age.append(1000*(frame['applied_s']-origin))
            origin=frame['captured_s']
        held_age.append(1000*(stop-origin))
        # Include frames still processing when the run stopped: no survivor filter.
        sensor.extend(1000*(f['finished_s']-f['rendered_s']) for f in active)
        fresh_age.extend(1000*(max(f['finished_s']+r['config']['transport_s'],f['received_s'])-
                              f['captured_s']) for f in active)
        for f in active:
            for key,value in f.get('stage_ms',{}).items():
                if key.endswith('_ms'):
                    stages.setdefault(key,[]).append(value)
    count=lambda key:sum(r['counts'].get(key,0) for r in selected)
    return dict(trials=len(selected),aligned=sum(r['outcome']=='converged' for r in selected),
        capture_to_command_ms=stats(applied),inference_ms=stats(sensor),
        maximum_held_feedback_age_bound_ms=max(held_age),
        all_active_capture_to_delivery_ms=stats(fresh_age),stage_ms={k:stats(v) for k,v in stages.items()},
        freshness_deadline_trips=sum(e.get('reason')=='stale_camera' for r in selected for e in r['events']),
        control_deadline_misses=count('control_deadline_misses'),captures=count('captures'),
        capture_slots=sum(r['counts'].get('capture_slots',r['counts']['captures']) for r in selected),
        pending_replaced=count('input_dropped'),busy_capture_skips=count('busy_capture_skips'),
        pre_inference_dropped=count('pre_inference_dropped'),expired_results=count('stale_results'),
        transport_replaced=count('transport_dropped'),unsafe_motion_ticks=count('unsafe_motion_ticks'),
        post_stop_motion_ticks=count('post_stop_motion_ticks'),contacts=count('contacts'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before',type=Path)
    parser.add_argument('after',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    manifests=[];runs=[]
    for folder in (args.before,args.after):
        manifest,rows=load_trials(folder);manifests.append(manifest);runs.append(rows)
        assert all(r['profile']=='transport-50ms' and r['config']['transport_s']==.05 and
                   r['config']['max_age_s']==.4 for r in rows)
    assert {r['id'] for r in runs[0]}=={r['id'] for r in runs[1]},'Pair the same modes and repeats'
    assert manifests[0]['offset_degrees']==manifests[1]['offset_degrees']
    assert manifests[0]['fingerprint']['runtime']==manifests[1]['fingerprint']['runtime']
    for key,sha in manifests[0]['fingerprint']['sha256'].items():
        if key.startswith(('assets/','reference/','models/')) or key in ('control.py','calibration.py','scene.xml','config.json'):
            assert manifests[1]['fingerprint']['sha256'][key]==sha,key
    summary={phase:{mode:summarize(rows,mode) for mode in ('natural','learned')}
             for phase,rows in zip(('before','after'),runs)}
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    names={'natural':'SIFT','learned':'Learned GPU'}
    totals={key:sum(r[key] for r in summary['after'].values())
            for key in ('aligned','trials','freshness_deadline_trips','control_deadline_misses')}
    regressions=[]
    for mode in names:
        for metric in ('mean','p95','p99','max'):
            before=summary['before'][mode]['capture_to_command_ms'][metric]
            after=summary['after'][mode]['capture_to_command_ms'][metric]
            if after>before:
                regressions.append(f'{names[mode]} {metric} ({before:.1f} to {after:.1f} ms)')
    assessment=(f"The optimized runs aligned {totals['aligned']}/{totals['trials']} times. "
        f"Freshness trips: {totals['freshness_deadline_trips']}; "
        f"control deadline misses: {totals['control_deadline_misses']}. ")
    if regressions:
        assessment+='Higher measured command latencies after optimization: '+', '.join(regressions)+'. '
        assessment+='This batch does not establish an improvement in every percentile. '
    assessment+=('Percentiles from these short trials need longer deployment measurements; '
                 'all-active sensor timings below also retain frames completed after a watchdog stop.')
    lines=['# Perception tail latency: paired transport comparison','',
        'Three runs per method, same visible starting offset, 50 ms added transport and an unchanged '
        '400 ms capture-age budget. The same Python/library versions, scene, references, controller '
        'and calibration are verified by the comparison script. Each run includes model warm-up '
        'before arming and a 1.1 s observation period after stopping.','',
        '| Method | Version | Aligned | Commands | Mean (ms) | p95 | p99 | Maximum | Freshness trips | Control misses |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for mode in names:
        for phase in summary:
            r=summary[phase][mode];s=r['capture_to_command_ms']
            lines.append(f"| {names[mode]} | {phase} | {r['aligned']}/{r['trials']} | {s['count']} | "
                f"{s['mean']:.1f} | {s['p95']:.1f} | {s['p99']:.1f} | {s['max']:.1f} | "
                f"{r['freshness_deadline_trips']} | {r['control_deadline_misses']} |")
    lines += ['', 'The latency table measures actual capture-to-command time for accepted observations. '
        'A rejected frame has no command latency. Failed baseline runs stop earlier, so the sample counts '
        'and trajectories differ. Percentiles pool frame samples across all three runs; they are '
        'measurements on this host, not worst-case execution-time guarantees. Freshness trips count '
        'latched camera-age watchdog stops; control misses count loop/computation overruns above 50 ms.', '',
        assessment, '',
        '## Feedback age between commands', '',
        '| Method | Before maximum age bound (ms) | After maximum age bound (ms) |',
        '|---|---:|---:|']
    for mode in names:
        ages=[summary[p][mode]['maximum_held_feedback_age_bound_ms'] for p in ('before','after')]
        lines.append(f'| {names[mode]} | {ages[0]:.1f} | {ages[1]:.1f} |')
    lines += ['', 'A command uses its observation until the next accepted update or stop. '
        'These bounds use the preceding capture timestamp and the next command application/stop '
        'timestamp, including initial waiting from arming. They include the interval between '
        'deliveries, which explains why a run can trip the 400 ms freshness watchdog even when '
        'every accepted command age is less than 400 ms.', '',
        '## Bounded work and dropped frames','',
        '| Method | Version | Capture slots | Acquired | Busy slots skipped | Pending replaced | Expired before inference | Expired results | Transport replaced |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for mode in names:
        for phase in summary:
            r=summary[phase][mode]
            values=[r[k] for k in ('capture_slots','captures','busy_capture_skips','pending_replaced',
                    'pre_inference_dropped','expired_results','transport_replaced')]
            lines.append(f"| {names[mode]} | {phase} | "+' | '.join(map(str,values))+' |')
    lines += ['', 'Counters cover each full trial, including its observation period after stopping. '
        'Before optimization every capture slot enqueues a copied state; full mailboxes replace pending '
        'states. After optimization a busy worker suppresses acquisition at that slot, and the next '
        'accepted slot takes a new state snapshot. A skipped slot is not a processed/dropped camera image. '
        'The old queued-work drop counter alone therefore is not a fair throughput comparison. '
        'There remains at most one pending request, one result and eight transport entries. '
        'The baseline did not separately count result-mailbox replacement or in-flight work '
        'discarded on session shutdown; zero values refer only to the listed counters.', '',
        '## Include the frames that missed control','',
        '| Method | Version | All active sensor frames | Processing mean / p95 / p99 / max (ms) | Capture to delivery mean / p95 / p99 / max (ms) |',
        '|---|---|---:|---:|---:|']
    for mode in names:
        for phase in summary:
            r=summary[phase][mode]
            describe=lambda s:' / '.join(f'{s[k]:.1f}' for k in ('mean','p95','p99','max'))
            lines.append(f"| {names[mode]} | {phase} | {r['inference_ms']['count']} | "
                f"{describe(r['inference_ms'])} | {describe(r['all_active_capture_to_delivery_ms'])} |")
    lines += ['', 'This second table includes every received sensor frame captured before the stop, '
        'including results completed after the watchdog tripped. Delivery time is the later of IPC '
        'receipt and scheduled transport completion; it is not a command application timestamp.', '',
        'Stage distributions are in comparison.json. GPU stage clocks measure host submission/wait '
        'time, not independent CUDA kernel durations. Total processing and capture-to-command times '
        'use wall-clock timestamps and include necessary device completion. Cached identical images '
        'are marked cache_hit instead of inheriting previous expensive-stage durations.', '',
        '![Before and after latency distributions](tail-latency-comparison.png)', '',
        '[Optimization and validation](VERIFICATION.md).']
    (args.output/'REPORT.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    fig,axes=plt.subplots(1,2,figsize=(11,4.3),layout='constrained')
    colors={'before':'#bb5e3c','after':'#087e8b'}
    for ax,mode in zip(axes,names):
        for phase,rows in zip(('before','after'),runs):
            values=np.sort([f['capture_to_command_ms'] for r in rows if r['mode']==mode for f in r['frames']])
            ax.plot(values,100*np.arange(1,len(values)+1)/len(values),label=phase,color=colors[phase])
        ax.axhline(95,color='grey',ls=':',lw=.8);ax.axhline(99,color='grey',ls=':',lw=.8)
        ax.axvline(400,color='#a32727',ls='--',lw=1,label='400 ms age limit')
        ax.set(title=names[mode],xlabel='Accepted capture-to-command latency (ms)',ylabel='Cumulative frames (%)',ylim=(0,101),xlim=(40,410))
        ax.spines[['top','right']].set_visible(False);ax.legend(frameon=False,loc='lower right')
    fig.savefig(args.output/'tail-latency-comparison.png',dpi=170)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
