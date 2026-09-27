"""Summarize separately sampled replay clocks without inventing frame pairing."""
import argparse
from collections import Counter
import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'outputs/visual-servoing-simulation'))
from latency_stress import distribution


def read_samples(directory):
    meta=json.loads((directory/'metadata.json').read_text(encoding='utf-8'))
    fields=meta['gpu_query_fields']
    samples=[];invalid=[]
    with (directory/'gpu-samples.csv').open(encoding='utf-8',newline='') as f:
        rows=csv.reader(f);next(rows)
        for values in rows:
            if len(values)!=len(fields):
                invalid.append(values);continue
            s=dict(zip(fields,(v.strip() for v in values)))
            try:
                s['time']=datetime.strptime(s['timestamp'],'%Y/%m/%d %H:%M:%S.%f')
                s['sm']=float(s['clocks.sm']);s['temperature']=float(s['temperature.gpu'])
            except ValueError:
                invalid.append(values);continue
            samples.append(s)
    for s in samples:s['elapsed_s']=(s['time']-samples[0]['time']).total_seconds()
    phases={}
    for name,lo,hi in [('first_30s',0,30),('later',30,float('inf'))]:
        selected=[s for s in samples if lo<=s['elapsed_s']<hi]
        phases[name]=dict(samples=len(selected),sm_mhz=distribution(s['sm'] for s in selected),
            minimum_sm_mhz=min((s['sm'] for s in selected),default=None),
            below_300_mhz=sum(s['sm']<300 for s in selected),
            temperatures_c=distribution(s['temperature'] for s in selected),
            states=dict(Counter(s['pstate'] for s in selected)),
            active_reasons={key:sum(s[key]=='Active' for s in selected) for key in fields
                if key.startswith('clocks_event_reasons.') and key!='clocks_event_reasons.active'})
    return samples,dict(phases=phases,invalid_samples=invalid,
        power_before=meta['before']['power'],power_after=meta['after']['power'],
        csv_sha256=hashlib.sha256((directory/'gpu-samples.csv').read_bytes()).hexdigest())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('latency_directory',type=Path);p.add_argument('output',type=Path)
    args=p.parse_args()
    result={};traces={}
    for directory in sorted(args.latency_directory.glob('20260923-replay-*')):
        if not (directory/'gpu-samples.csv').exists():continue
        traces[directory.name],result[directory.name]=read_samples(directory)
    document=dict(interpretation='Whole separate diagnostic replay, including intentional idle intervals. Relative time begins at the first NVIDIA sample. No exact frame-to-clock pairing is inferred; not part of the sustained campaign.',probes=result)
    (args.output/'replay-clock-summary.json').write_text(json.dumps(document,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(11,6))
    for ax,name in zip(axes,('20260923-replay-ablation','20260923-replay-graphs')):
        rows=traces[name]
        ax.plot([s['elapsed_s'] for s in rows],[s['sm'] for s in rows],color='#2679a3',lw=1)
        ax.axhline(300,ls='--',lw=.8,color='#a73736')
        ax.set_title(name.removeprefix('20260923-'));ax.set_ylabel('SM clock (MHz)')
        ax.set_xlabel('Seconds from first NVIDIA sample');ax.grid(alpha=.2)
    fig.suptitle('Low GPU clocks recur in separate reused-worker diagnostics\nIncludes intentional idle intervals; no exact frame pairing or causal attribution')
    fig.tight_layout();fig.savefig(args.output/'replay-clocks.png',dpi=150);plt.close(fig)
    print(json.dumps({name:{cohort:dict(samples=v['samples'],below_300_mhz=v['below_300_mhz'],min=v['minimum_sm_mhz'],median=(v['sm_mhz'] or {}).get('p50')) for cohort,v in r['phases'].items()} for name,r in result.items()},indent=2))


if __name__=='__main__':main()
