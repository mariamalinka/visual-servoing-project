"""Correlate a separate NVIDIA sampling run with recorded perception timings."""
import argparse
from collections import Counter
import csv
from datetime import datetime,timezone,timedelta
import gzip
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'outputs/visual-servoing-simulation'))
from latency_stress import active_frames,distribution


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--utc-offset-hours',type=float,required=True,
        help='UTC offset used by the NVIDIA CSV local timestamps at recording time')
    args=parser.parse_args()
    meta=json.loads((args.directory/'metadata.json').read_text(encoding='utf-8'))
    row=json.loads(gzip.decompress((args.directory/'traces/session.json.gz').read_bytes()))
    fields=meta['gpu_query_fields']
    tz=timezone(timedelta(hours=args.utc_offset_hours))
    samples=[];invalid_samples=[]
    with (args.directory/'gpu-samples.csv').open(encoding='utf-8',newline='') as stream:
        reader=csv.reader(stream);next(reader)
        for values in reader:
            if len(values)!=len(fields):
                raise ValueError('Malformed NVIDIA sample: '+repr(values))
            s=dict(zip(fields,(v.strip() for v in values)))
            utc=datetime.strptime(s['timestamp'],'%Y/%m/%d %H:%M:%S.%f').replace(tzinfo=tz).timestamp()
            s['perf_s']=meta['before']['perf_counter_s']+utc-meta['before']['time_ns']/1e9
            try:
                for key in ('clocks.sm','clocks.mem','temperature.gpu','power.draw','utilization.gpu'):
                    s[key]=float(s[key])
                    if not np.isfinite(s[key]):raise ValueError('Nonfinite '+key)
            except ValueError as exc:
                # NVML can report an unavailable value at a context transition.
                # Preserve the rejected sample and expose the gap in the report.
                invalid_samples.append(dict(raw=dict(zip(fields,values)),error=str(exc)))
                continue
            samples.append(s)
    if not samples:raise ValueError('No complete numeric GPU samples')
    times=np.array([s['perf_s'] for s in samples])
    assert np.all(np.diff(times)>0)
    frames=active_frames(row,True)
    matched=[]
    for f in frames:
        mid=(f['rendered_s']+f['finished_s'])/2
        i=int(np.abs(times-mid).argmin());s=samples[i]
        separation=abs(times[i]-mid)
        if separation>.5:continue
        matched.append(dict(sequence=f['sequence'],generation=f['generation'],
            captured_s=f['captured_s'],mid_s=mid,
            processing_ms=1000*(f['finished_s']-f['rendered_s']),
            coarse_ms=f['stage_ms'].get('coarse_ms'),refinement_ms=f['stage_ms'].get('refinement_ms'),
            gpu=s,sample_separation_ms=1000*separation))
    active_samples=[s for s in samples if any(e['kind']=='align' and e['at_s']<=s['perf_s']<=
        next(x['stopped_s'] for x in row['events'] if x['kind']=='stop' and x['generation']==e['generation'])
        for e in row['events'])]
    reasons=[key for key in fields if key.startswith('clocks_event_reasons.') and key!='clocks_event_reasons.active']
    summary=dict(excluded_from_statistical_campaign=True,utc_offset_hours=args.utc_offset_hours,
        sample_count=len(samples),invalid_sample_count=len(invalid_samples),invalid_samples=invalid_samples,
        active_sample_count=len(active_samples),matched_uncached_frames=len(matched),
        clock_pair_drift_ms=1000*((meta['after']['time_ns']-meta['before']['time_ns'])/1e9-
            (meta['after']['perf_counter_s']-meta['before']['perf_counter_s'])),
        sample_spacing_ms=distribution(1000*np.diff(times)),
        active_gpu={k:distribution(s[k] for s in active_samples)
            for k in ('clocks.sm','clocks.mem','temperature.gpu','power.draw','utilization.gpu')},
        active_states=dict(Counter(s['pstate'] for s in active_samples)),
        active_reason_counts={key:sum(s[key]=='Active' for s in active_samples) for key in reasons},
        sm_clock_groups={label:dict(frames=len(selected),processing_ms=distribution(f['processing_ms'] for f in selected))
            for label,selected in [('below_300_mhz',[f for f in matched if f['gpu']['clocks.sm']<300]),
                                  ('300_to_799_mhz',[f for f in matched if 300<=f['gpu']['clocks.sm']<800]),
                                  ('800_mhz_or_more',[f for f in matched if f['gpu']['clocks.sm']>=800])]},
        longest_processing_frames=sorted(matched,key=lambda f:f['processing_ms'],reverse=True)[:15],
        power_before=meta['before']['power'],power_after=meta['after']['power'],
        interpretation='Nearest 250 ms GPU samples are observational associations. They do not separate kernel time from host gaps or establish causality. Native sampling is excluded from main campaign statistics.')
    (args.directory/'gpu-correlation.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    start=next(e['at_s'] for e in row['events'] if e['kind']=='align')
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    axes[0].scatter([f['mid_s']-start for f in matched],[f['processing_ms'] for f in matched],s=10,c='#246b96')
    axes[0].set_ylabel('Uncached processing (ms)')
    axes[1].plot(times-start,[s['clocks.sm'] for s in samples],color='#ba7625',linewidth=1)
    axes[1].set_ylabel('GPU SM clock (MHz)')
    axes[2].plot(times-start,[s['temperature.gpu'] for s in samples],color='#598441')
    axes[2].set_ylabel('GPU temperature (°C)');axes[2].set_xlabel('Seconds after first alignment started')
    for ax in axes:
        ax.grid(alpha=.2)
        for e in row['events']:
            if e.get('reason') in ('stale_camera','control_overrun'):
                ax.axvline(e['stopped_s']-start,color='#b32c2c',alpha=.25,linewidth=.7)
        ax.set_xlim(0,max(f['mid_s'] for f in matched)-start+1)
    fig.suptitle('Separate GPU diagnostic run: timing and clocks\nRed lines mark watchdog stops; correlation does not establish causality')
    fig.tight_layout();fig.savefig(args.directory/'gpu-timeline.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in summary.items() if k!='longest_processing_frames'},indent=2))


if __name__=='__main__':main()
