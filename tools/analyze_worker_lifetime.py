"""Audit and summarize the completed worker-lifetime comparison and GPU telemetry."""
from pathlib import Path
from datetime import datetime,timezone,timedelta
from collections import defaultdict,Counter
import argparse,csv,gzip,hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation';sys.path.insert(0,str(APP))
from latency_stress import active_frames,distribution

def read(path):return json.loads(path.read_text(encoding='utf-8'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();directory=a.directory
    manifest=read(directory/'manifest.json');meta=read(directory/'metadata.json');completed=read(directory/'completion.json')
    assert completed['source_unchanged'] and completed['all_safety_audits_passed']
    sessions=read(directory/'sessions.json');assert completed['sessions']==len(sessions)
    if not manifest['native_diagnostic_only']:
        assert len(sessions)==40 and sum(s['arm']=='fresh' for s in sessions)==32
    fields=manifest['gpu_query_fields'];offset=timezone(timedelta(seconds=manifest['local_utc_offset_seconds']))
    anchors=[meta['before'],meta['after']]
    for s in sessions:anchors.extend([s['boundary_before'],s['boundary_after']])
    anchors=sorted(anchors,key=lambda p:p['time_ns'])
    ax=np.array([p['time_ns']/1e9 for p in anchors]);ay=np.array([p['perf_counter_s'] for p in anchors])
    assert np.all(np.diff(ax)>0) and np.all(np.diff(ay)>0)
    samples=[];invalid=[]
    with (directory/'gpu-samples.csv').open(encoding='utf-8',newline='') as stream:
        reader=csv.reader(stream);next(reader)
        for values in reader:
            if len(values)!=len(fields):raise ValueError('GPU telemetry malformed')
            row=dict(zip(fields,(v.strip() for v in values)))
            utc=datetime.strptime(row['timestamp'],'%Y/%m/%d %H:%M:%S.%f').replace(tzinfo=offset).timestamp()
            row['perf_s']=float(np.interp(utc,ax,ay));row['utc_s']=utc
            for k in ('clocks.sm','clocks.mem','temperature.gpu','power.draw','utilization.gpu','memory.used','memory.free','memory.total'):
                try:row[k]=float(row[k]);assert np.isfinite(row[k])
                except (ValueError,AssertionError):invalid.append({'timestamp':row['timestamp'],'field':k,'value':row[k]});row[k]=None
            samples.append(row)
    st=np.array([s['perf_s'] for s in samples]);assert len(st)>1 and np.all(np.diff(st)>0)
    cohorts=defaultdict(lambda:{'frames':[],'attempts':[],'sessions':set()});allframes=[]
    safety=Counter();checksums=[]
    for s in sessions:
        raw=(directory/s['trace']).read_bytes();digest=hashlib.sha256(raw).hexdigest();assert digest==s['trace_sha256']
        checksums.append({'id':s['id'],'sha256':digest});row=json.loads(gzip.decompress(raw))
        assert row['initialization_error'] is None and not row['error']
        assert not row['worker_alive'],'Previous worker remained alive at session close'
        assert all(attempt['stop_stayed_latched'] for attempt in row['attempts'])
        for k in ('unsafe_motion_ticks','post_stop_motion_ticks','contacts','sensor_rows_evicted','command_rows_evicted','loop_rows_evicted'):
            safety[k]+=row['counts'][k];assert row['counts'][k]==0
        pergen={attempt['generation']:i for i,attempt in enumerate(row['attempts'])}
        stops={e['generation']:e for e in row['events'] if e['kind']=='stop'}
        aligns={e['generation']:e for e in row['events'] if e['kind']=='align'}
        def cohort(generation):
            if s['arm']=='fresh':return 'fresh_first_in_block' if s['arm_index']==0 else 'fresh_later_in_block'
            return 'reused_first' if pergen[generation]==0 else 'reused_later'
        for attempt in row['attempts']:
            g=attempt['generation'];stop=stops[g];name=cohort(g)
            cohorts[name]['sessions'].add(s['id'])
            cohorts[name]['attempts'].append({'session':s['id'],'generation':g,'outcome':attempt['outcome'],
                'freshness':stop['reason']=='stale_camera','control':stop['reason']=='control_overrun',
                'duration_ms':1000*(stop['stopped_s']-aligns[g]['at_s'])})
        for f in active_frames(row,True):
            g=f['generation'];name=cohort(g);mid=(f['rendered_s']+f['finished_s'])/2
            pos=int(np.searchsorted(st,mid));choices=[i for i in (pos-1,pos) if 0<=i<len(st)]
            i=min(choices,key=lambda j:abs(st[j]-mid));separation=abs(st[i]-mid)
            lo=int(np.searchsorted(st,f['rendered_s']));hi=int(np.searchsorted(st,f['finished_s'],side='right'))
            item={'session':s['id'],'arm':s['arm'],'block':s['block'],'cohort':name,'sequence':f['sequence'],'generation':g,
                'rendered_s':f['rendered_s'],'finished_s':f['finished_s'],
                'captured_s':f['captured_s'],'qpos':f.get('qpos'),
                'processing_ms':1000*(f['finished_s']-f['rendered_s']),'stage_ms':f['stage_ms'],
                'sample_separation_ms':1000*separation,'gpu':samples[i] if separation<=.3 else None,
                'gpu_samples_within_processing':samples[lo:hi]}
            cohorts[name]['frames'].append(item);allframes.append(item)
    def gpu_summary(frames):
        matched=[f for f in frames if f['gpu'] is not None]
        result={'frames':len(frames),'matched':len(matched),'states':dict(Counter(f['gpu']['pstate'] for f in matched))}
        for k in ('clocks.sm','clocks.mem','utilization.gpu','power.draw','temperature.gpu','memory.used'):
            result[k]=distribution(f['gpu'][k] for f in matched if f['gpu'][k] is not None)
        result['sample_separation_ms']=distribution(f['sample_separation_ms'] for f in matched)
        result['with_in_interval_sample']=sum(bool(f['gpu_samples_within_processing']) for f in matched)
        inside=[s for f in frames for s in f['gpu_samples_within_processing']]
        result['in_processing']={'samples':len(inside),'states':dict(Counter(s['pstate'] for s in inside)),
            **{k:distribution(s[k] for s in inside if s[k] is not None) for k in ('clocks.sm','clocks.mem','utilization.gpu','power.draw','temperature.gpu','memory.used')}}
        return result
    output={}
    def summarize(data):
        frames=data['frames'];attempts=data['attempts']
        first_frames={}
        for f in sorted(frames,key=lambda f:f['captured_s']):first_frames.setdefault((f['session'],f['generation']),f)
        poses=[f['qpos'] for f in first_frames.values() if f['qpos'] is not None]
        return {'sessions':sorted(data['sessions']),'attempts':len(attempts),'alignments':sum(t['outcome']=='converged' for t in attempts),
            'freshness_trips':sum(t['freshness'] for t in attempts),'control_misses':sum(t['control'] for t in attempts),
            'processing_ms':distribution(f['processing_ms'] for f in frames),
            'first_uncached_frame_processing_ms':distribution(f['processing_ms'] for f in first_frames.values()),
            'first_uncached_frame_poses':poses,
            'alignment_duration_ms':distribution(t['duration_ms'] for t in attempts),
            'slow_gpu':gpu_summary([f for f in frames if f['processing_ms']>=200]),
            'fast_gpu':gpu_summary([f for f in frames if f['processing_ms']<120])}
    for name,data in cohorts.items():output[name]=summarize(data)
    fresh_data={'frames':[],'attempts':[],'sessions':set()}
    for name,data in cohorts.items():
        if not name.startswith('fresh_'):continue
        fresh_data['frames'].extend(data['frames']);fresh_data['attempts'].extend(data['attempts']);fresh_data['sessions'].update(data['sessions'])
    result={'cohorts':output,'slow_threshold_ms':200,'gpu_sample_spacing_ms':distribution(1000*np.diff(st)),
        'gpu_invalid_values':invalid,'safety':dict(safety),'checksums':checksums,
        'clock_anchor_drift_ms':distribution(1000*((ax-ax[0])-(ay-ay[0]))),
        'clock_association':'Nearest sample to frame midpoint, at most 300 ms away. In-interval samples also retained. Association does not prove clock causality.',
        'definition':'Uncached active processing, including frames that finish after a stop. Fresh later means another newly created worker later in the same block.',
        'native_diagnostic_only':manifest['native_diagnostic_only']}
    result['fresh_all']=summarize(fresh_data) if fresh_data['attempts'] else None
    assert sum(v['attempts'] for v in output.values())==sum(s['attempts'] for s in sessions)
    assert sum(v['alignments'] for v in output.values())==sum(s['alignments'] for s in sessions)
    assert sum(v['freshness_trips'] for v in output.values())==sum(s['freshness_watchdog_trips'] for s in sessions)
    assert sum(v['control_misses'] for v in output.values())==sum(s['counters']['control_deadline_misses'] for s in sessions)
    (directory/'comparison.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    with gzip.open(directory/'traces/frame-gpu-correlation.json.gz','wt',encoding='utf-8') as stream:json.dump(allframes,stream,allow_nan=False)
    def triple(v):return ' / '.join(f'{v[k]:.1f}' for k in ('p95','p99','max')) if v else 'unavailable'
    lines=['| Worker / alignment group | Processing p95 / p99 / max (ms) | Freshness trips | Control misses | GPU SM clock for slow frames, median (MHz) | GPU states for slow frames |',
        '|---|---:|---:|---:|---:|---|']
    labels={'fresh_all':'Fresh worker for every alignment','reused_first':'Reused: first alignment','reused_later':'Reused: later alignments'}
    displayed=dict(output)
    if result['fresh_all']:displayed['fresh_all']=result['fresh_all']
    for name in labels:
        if name not in displayed:continue
        r=displayed[name];clock=r['slow_gpu']['in_processing']['clocks.sm']
        lines.append(f"| {labels[name]} | {triple(r['processing_ms'])} | {r['freshness_trips']} | {r['control_misses']} | {clock['p50'] if clock else 'No in-frame sample'} | {r['slow_gpu']['in_processing']['states']} |")
    lines+=['','Processing excludes cache hits and retains failed/late frames. Every fresh alignment has a new worker; “later” for fresh refers to its position within the comparison block. GPU clock/state samples are observational.']
    text='\n'.join(lines)+'\n';(directory/'COMPARISON.md').write_text(text,encoding='utf-8');print(text)
    print('Samples',len(samples),'frames',len(allframes),'cohorts',[(k,v['attempts'],v['alignments']) for k,v in output.items()])

if __name__=='__main__':main()
