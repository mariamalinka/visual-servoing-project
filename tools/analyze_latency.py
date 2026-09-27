"""Plot published wall-clock command ages and actual worker-stall responses."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    summary=json.loads((args.directory/'summary.json').read_text(encoding='utf-8'))
    rows=[json.loads((args.directory/(r['id']+'.json')).read_text(encoding='utf-8')) for r in summary]
    colors={'aruco':'#087e8b','natural':'#ef8354','learned':'#7765b3'}
    names={'aruco':'ArUco','natural':'SIFT','learned':'Learned'}
    fig,(ax,bx)=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    for row in rows:
        if row['profile']!='nominal' or not row['frames']:
            continue
        start=row['events'][0]['at_s']
        ax.plot([f['applied_s']-start for f in row['frames']],
                [f['capture_to_command_ms'] for f in row['frames']],
                color=colors[row['mode']],label=names[row['mode']],alpha=.85)
    budgets=sorted(set(r['config']['max_age_s']*1000 for r in rows))
    for budget in budgets:
        ax.axhline(budget,color='#a32727',ls='--',lw=1,label=f'{budget:.0f} ms budget')
    ax.set(xlabel='Time since Align (s)',ylabel='Capture to command (ms)',
           title='Nominal alignment: actual measurement age')
    ax.legend(frameon=False)
    stalls=[r for r in rows if 'stall' in r['profile']]
    labels=[]
    for y,row in enumerate(stalls):
        events=[e for e in row['events'] if e.get('reason')=='stale_camera']
        if not events:
            labels.append(names[row['mode']]+' / '+row['profile']+' (no stale stop)')
            continue
        stop=events[-1]
        origin=stop['deadline_s']
        faults=[f for f in row['sensor_frames'] if f['fault_injected'] and
                f['render_started_s']<stop['stopped_s']<f['finished_s']]
        if faults:
            f=faults[0]
            bx.plot([1000*(f['render_started_s']-origin),1000*(f['finished_s']-origin)],
                    [y,y],lw=8,color=colors[row['mode']],alpha=.35)
        bx.scatter(1000*(stop['stopped_s']-origin),y,color=colors[row['mode']],marker='D',s=35)
        labels.append(names[row['mode']]+' / '+row['profile'].replace('-stall',''))
    bx.axvline(0,color='#a32727',ls='--',lw=1,label='Freshness deadline')
    bx.set_yticks(range(len(stalls)),labels)
    bx.set(xlabel='Time relative to freshness deadline (ms)',
           title='Worker busy interval; diamond = zero command')
    bx.invert_yaxis();bx.legend(frameon=False)
    for chart in (ax,bx):
        chart.spines[['top','right']].set_visible(False)
        chart.grid(axis='x',alpha=.15)
    fig.savefig(args.directory/'latency-and-watchdog.png',dpi=170)
    print(args.directory/'latency-and-watchdog.png')


if __name__=='__main__':
    main()
