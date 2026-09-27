"""Attribute recorded CPU time using WPA precise switches and sampling stacks."""
import argparse,csv,gzip,json,re
from pathlib import Path
from datetime import datetime,timezone
from collections import defaultdict,Counter

def number(s):return float(s.strip().replace('\u00a0','').replace(',','.'))
def read_csv(path):
    with path.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--switches',type=Path,required=True)
    p.add_argument('--samples',type=Path,required=True);p.add_argument('--timespan',type=Path,required=True)
    a=p.parse_args();d=a.directory
    meta=json.loads((d/'metadata.json').read_text());summary=json.loads((d/'sessions.json').read_text())[0]
    row=json.loads(gzip.decompress((d/summary['trace']).read_bytes()))
    header=a.timespan.read_text(encoding='utf-8-sig')
    stamp=re.search(r'Start time \(UTC\)\s*:\s*(\d{4}/\d\d/\d\d:\d\d:\d\d:\d\d\.\d+)',header)[1]
    whole,fraction=stamp.split('.')
    utc=datetime.strptime(whole,'%Y/%m/%d:%H:%M:%S').replace(tzinfo=timezone.utc).timestamp()+float('0.'+fraction)
    anchor=meta['wpr_start_before'];origin=anchor['perf_counter_s']+utc-anchor['time_ns']/1e9
    tids={'control':row['control_scheduling']['thread_id'],'sensor':row['execution_settings']['scheduling']['thread_id']}
    switches=read_csv(a.switches);samples=read_csv(a.samples)
    sw=[]
    for s in switches:
        if s.get('Count')!='1' or not s.get('Switch-In Time (s)'):continue
        x=dict(s);x['tid']=int(s['New Thread Id'])
        for key,label in [('in','Switch-In Time (s)'),('out','Next Switch-Out Time (s)'),('last','Last Switch-Out Time (s)'),('ready','Ready Time (s)')]:
            x[key]=number(s[label])+origin if s.get(label) else None
        if all(x[k] is not None for k in ('in','out','last','ready')):sw.append(x)
    assert sw,'No individual switch rows decoded'
    sp=[]
    for s in samples:
        if s.get('Count')!='1' or not s.get('TimeStamp (s)'):continue
        sp.append(dict(tid=int(s['Thread ID']),at=number(s['TimeStamp (s)'])+origin,
            module=s.get('Module',''),function=s.get('Function',''),stack=s.get('Stack',''),
            dpc_isr=s.get('DPC/ISR',''),weight_ms=number(s.get('Weight (in view) (ms)','0'))))
    assert sp,'No individual CPU samples decoded'
    assert all(any(s['tid']==tid for s in sp) for tid in tids.values()),'CPU samples missing for a target thread; check WPA keep-filter setting'
    segments=defaultdict(list)
    for s in sw:
        segments[s['tid']].extend([(s['in'],s['out'],'running'),(s['last'],min(s['ready'],s['in']),'waiting'),
            (max(s['last'],s['ready']),s['in'],'ready')])
    def interval(tid,start,end):
        counts=defaultdict(float)
        for x,y,kind in segments[tid]:counts[kind]+=max(0,min(end,y)-max(start,x))*1000
        points=[s for s in sp if s['tid']==tid and start<=s['at']<=end]
        leaf=Counter();paths=Counter();interrupts=Counter()
        for s in points:leaf[s['module']]+=s['weight_ms'];paths[s['stack']]+=s['weight_ms'];interrupts[s['dpc_isr']]+=1
        return {'cpu_ms':dict(counts),'samples':len(points),'sample_weight_ms_by_leaf_module':dict(leaf),
            'sample_dpc_isr_counts':dict(interrupts),'top_sample_stacks_ms':paths.most_common(5)}
    coverage={role:[min(s['in'] for s in sw if s['tid']==tid),max(s['out'] for s in sw if s['tid']==tid)] for role,tid in tids.items()}
    frames=[]
    for f in row['sensor_frames']:
        start,end=f['rendered_s'],f['finished_s'];lo,hi=coverage['sensor']
        if not f['capture_active'] or f['stage_ms'].get('cache_hit',False) or not lo<=start<end<=hi:continue
        frames.append(dict(sequence=f['sequence'],generation=f['generation'],started_s=start,finished_s=end,
            processing_ms=1000*(end-start),stages=f['stage_ms'],
            recording_transition=not(meta['wpr_start_after']['perf_counter_s']<=start<end<=meta['wpr_end_before']['perf_counter_s']),
            **interval(tids['sensor'],start,end)))
    slow=[f for f in frames if f['processing_ms']>=200]
    stable_slow=[f for f in slow if not f['recording_transition']]
    for f in frames:
        assert abs(sum(f['cpu_ms'].values())-f['processing_ms'])<0.01,'CPU interval accounting does not close'
    slow_cpu={key:sum(f['cpu_ms'].get(key,0) for f in stable_slow) for key in ('running','ready','waiting')}
    slow_modules=Counter()
    for f in stable_slow:slow_modules.update(f['sample_weight_ms_by_leaf_module'])
    waits={}
    for role,tid in tids.items():
        own=[s for s in sw if s['tid']==tid]
        waits[role]={}
        for label,column in [('waiting','Waits (µs)'),('ready','Ready (µs)')]:
            selected=sorted(own,key=lambda s:number(s[column]),reverse=True)[:8]
            waits[role][label]=[{'duration_ms':number(s[column])/1000,'switch_in_s':s['in'],
                'previous_state':s['New Prev State'],'reason':s['New Prev Wait Reason'],
                'switch_out_stack':s['New Thread Stack'],'ready_stack':s['Ready Thread Stack']} for s in selected]
    result={'origin_perf_s':origin,'timeline_alignment_uncertainty_ms':2.,'thread_ids':tids,'coverage_s':coverage,
        'sample_rows':len(sp),'switch_rows':len(sw),'uncached_frames':len(frames),'slow_frames':len(slow),
        'slowest_frames':sorted(frames,key=lambda f:f['processing_ms'],reverse=True)[:8],
        'all_frames':frames,'longest_off_cpu':waits,
        'steady_recording_slow_frames':len(stable_slow),'steady_recording_slow_cpu_ms':slow_cpu,
        'steady_recording_slow_sample_leaf_ms':dict(slow_modules),
        'limitations':['CPU-running intervals include interruptions and native spin waits; sampled stacks refine that attribution.',
            'Kernel/device execution and GPU preemption are not separated by CPU samples or CUDA stream spans.',
            'Thread waits during inactive intervals are not evidence of a perception bottleneck.']}
    (d/'native-attribution.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print('Frames',len(frames),'slow',len(slow),'samples',len(sp))
    for f in result['slowest_frames'][:3]:print(json.dumps(f,indent=2))
    for role in waits:
        for kind in waits[role]:print(role,kind,[(round(s['duration_ms'],3),s['reason']) for s in waits[role][kind][:3]])

if __name__=='__main__':main()
