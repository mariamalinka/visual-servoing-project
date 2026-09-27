"""Compare completed campaigns; retain failed work and audit fixed conditions."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path


def read(path):return json.loads(path.read_text(encoding='utf-8'))

def extra(directory):
    late=Counter();execution={};power=[];graphs=Counter()
    for s in read(directory/'sessions.json'):
        path=directory/s['trace'];blob=path.read_bytes()
        assert hashlib.sha256(blob).hexdigest()==s['trace_sha256']
        row=json.loads(gzip.decompress(blob));mode=row['mode']
        execution[row['id']]=row.get('execution_settings',{})
        power.append(dict(session=row['id'],before=row['gpu_before'].get('power'),after=row['gpu_after'].get('power')))
        stops={e['generation']:e for e in row['events'] if e['kind']=='stop'}
        for f in row['sensor_frames']:
            e=stops.get(f['generation'])
            if e and f['capture_active'] and f['finished_s']>e['stopped_s']:
                late[(mode,e['reason'])]+=1
            if f['capture_active'] and not f['stage_ms'].get('cache_hit'):
                graphs[(mode,'replays')]+=f['stage_ms'].get('cuda_graph_replays',0)
                graphs[(mode,'fallbacks')]+=f['stage_ms'].get('cuda_graph_fallbacks',0)
    return dict(late_all_stops={mode:{reason:n for (m,reason),n in late.items() if m==mode} for mode in ('natural','learned')},
        execution=execution,power_boundaries=power,graph_calls={mode:{kind:n for (m,kind),n in graphs.items() if m==mode} for mode in ('natural','learned')})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path)
    args=p.parse_args()
    old,new=read(args.before/'summary.json'),read(args.after/'summary.json')
    bm,am=read(args.before/'manifest.json'),read(args.after/'manifest.json')
    assert bm['plan']==am['plan'],'Campaign plan differs'
    assert bm['fingerprint']['runtime']==am['fingerprint']['runtime']
    assert am['baseline_manifest_sha256']==hashlib.sha256((args.before/'manifest.json').read_bytes()).hexdigest()
    changes={name for name,sha in bm['fingerprint']['sha256'].items() if am['fingerprint']['sha256'].get(name)!=sha}
    assert changes<={'learned_perception.py','realtime.py','run_latency_stress.py'},changes
    additions=set(am['fingerprint']['sha256'])-set(bm['fingerprint']['sha256'])
    assert additions=={'learned_cuda_graphs.py'},additions
    supplement={phase:extra(directory) for phase,directory in [('before',args.before),('after',args.after)]}
    audit=dict(plan_identical=True,runtime_versions_identical=True,changed_source=sorted(changes),added_source=sorted(additions),
        controller_calibration_scene_model_configuration_and_assets_identical=True,
        baseline_manifest_sha256=am['baseline_manifest_sha256'],**supplement)
    (args.after/'comparison-audit.json').write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    lines=['# Sustained worker reuse: before versus after','',
        'The failed 20260922 campaign is retained as the baseline. This comparison checks identical campaign plans, dependency versions, controller/calibration/scene/model inputs, and every raw trace checksum. Deliberate execution changes are listed in [comparison-audit.json](comparison-audit.json).','',
        'The plan alternates SIFT and Learned GPU, uses at least 12 two-minute sessions per method, targets 10,000 uncached active frames per method, and caps at 20 sessions. A session reuses one worker across explicit alignments. Transport is 50 ms, the capture-age budget is 400 ms, and the control deadline is 50 ms. Initialization now includes graph capture; the one-camera-image warm-up and all subsequent rearming procedures are unchanged.','',
        '## Outcomes','',
        '| Method | Campaign | Sessions | Aligned / attempts | First attempts | Later attempts | Freshness trips | Control misses | Maximum held age (ms) |',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|']
    datasets=[('Before',old),('After',new)]
    names={'natural':'SIFT','learned':'Learned GPU'}
    for mode in names:
        for label,dataset in datasets:
            r=dataset['methods'][mode];f=r['first_attempt'];l=r['reused_worker_attempts']
            lines.append(f"| {names[mode]} | {label} | {r['sessions']} | {r['alignments']}/{r['attempts']} | {f['aligned']}/{f['attempts']} | {l['aligned']}/{l['attempts']} | {r['freshness_watchdog_trips']} | {r['counters']['control_deadline_misses']} | {r['maximum_feedback_age_ms']:.3f} |")
    lines+=['','## Processing latency','',
        'All values are milliseconds. All active frames include legitimate identical-pixel cache hits. Uncached frames, including failed and late work, are reported separately to expose inference tails.','',
        '| Method | Campaign | Cohort | Frames | Mean | p95 | p99 | p99.9 | Maximum |',
        '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for mode in names:
        for label,dataset in datasets:
            r=dataset['methods'][mode]
            for cohort,d in [('All active',r['processing_ms']),('Uncached',r['uncached_processing_ms']),
                ('First, uncached',r['cohort_uncached_processing_ms']['first_attempt']),
                ('Later, uncached',r['cohort_uncached_processing_ms']['reused_worker'])]:
                lines.append(f"| {names[mode]} | {label} | {cohort} | {d['count']} | "+' | '.join(f'{d[k]:.3f}' for k in ('mean','p95','p99','p99_9','max'))+' |')
    lines+=['','## Capture to accepted command','',
        '| Method | Campaign | Commands | Mean (ms) | p95 | p99 | Maximum |',
        '|---|---|---:|---:|---:|---:|---:|']
    for mode in names:
        for label,dataset in datasets:
            d=dataset['methods'][mode]['capture_to_command_ms']
            lines.append(f"| {names[mode]} | {label} | {d['count']} | "+' | '.join(f'{d[k]:.3f}' for k in ('mean','p95','p99','max'))+' |')
    lines+=['','These accepted-command distributions exclude rejected work. Processing distributions above include that work; the complete report also includes capture-to-delivery latency for all active results. The held-age maximum includes waiting until the next accepted command or a latched stop, so it can exceed 400 ms when the watchdog detects expiry.','',
        '## Bounded-work accounting','',
        '| Counter | SIFT before | SIFT after | Learned before | Learned after |',
        '|---|---:|---:|---:|---:|']
    metrics=[('Skipped busy acquisition slots','busy_capture_skips'),('Replaced pending acquisition','input_dropped'),
        ('Replaced/dropped transport results','transport_dropped'),('Dropped result mailbox work','result_mailbox_dropped'),
        ('Cancelled pending transport on stop/reset','pending_cancelled'),('Expired requests','worker_requests_expired'),
        ('Expired results','stale_results'),('Obsolete-generation results','obsolete_results'),
        ('Completed but unreceived','completed_but_unreceived'),('Requests not started','requests_not_started'),
        ('Unsafe motion ticks','unsafe_motion_ticks'),('Motion ticks after stop','post_stop_motion_ticks')]
    ordered=[old['methods']['natural'],new['methods']['natural'],old['methods']['learned'],new['methods']['learned']]
    for label,key in metrics:lines.append('| '+label+' | '+' | '.join(str(r['counters'][key]) for r in ordered)+' |')
    for label,key in [('Late after freshness stop','late_after_freshness_stop'),('Late after control stop','late_after_control_stop'),('All stops stayed latched','all_stops_stayed_latched')]:
        lines.append('| '+label+' | '+' | '.join(str(r[key]) for r in ordered)+' |')
    all_late=[sum(supplement[phase]['late_all_stops'][mode].values()) for mode,phase in [('natural','before'),('natural','after'),('learned','before'),('learned','after')]]
    lines.append('| Late after any stop (includes convergence) | '+' | '.join(map(str,all_late))+' |')
    lines+=['','Counters can overlap: an expired-request acknowledgement and the worker expiry counter describe the same event. Cancellation/obsolescence is expected after explicit stops; late completion must never reactivate motion.','',
        '## Interpretation and evidence','',
        'Read [DIAGNOSIS.md](DIAGNOSIS.md) for GPU clocks, CPU/submission measurements, the power-source confound, rejected alternatives, and native scheduling limits. [REPORT.md](REPORT.md) contains whole-session bootstrap intervals and per-session trends; [verification.json](verification.json) records validation.','',
        'Frame samples are correlated within a session. p99.9 has only about ten tail observations per 10,000 frames, and an observed maximum is not a guaranteed worst-case bound. Zero observed events do not establish a zero event rate.','']
    (args.after/'COMPARISON.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps({k:v for k,v in audit.items() if k not in ('before','after')},indent=2))

if __name__=='__main__':main()
