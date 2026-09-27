"""Compare completed accuracy runs with matched plans and teaching images."""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def compare(baseline,current):
    baseline=Path(baseline).resolve();current=Path(current).resolve()
    def load(directory):
        m=json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
        p=json.loads((directory/'plan.json').read_text(encoding='utf-8'))
        r=json.loads((directory/'trials.json').read_text(encoding='utf-8'))
        if m['status']!='complete' or len(r)!=len(p['trials']):raise ValueError('Both runs must be complete')
        if digest(directory/'plan.json')!=m['plan_sha256'] or digest(directory/'goals.json')!=m['goals_sha256']:
            raise ValueError('Plan or teaching record changed')
        expected={x['id'] for x in p['trials']}
        if {x['id'] for x in r}!=expected or len(expected)!=len(r):raise ValueError('Trial IDs must match exactly')
        goals=json.loads((directory/'goals.json').read_text())
        for goal in goals.values():
            if digest(directory/goal['reference'])!=goal['reference_sha256']:raise ValueError('Private reference changed')
        return m,p,r
    old_manifest,old_plan,old=load(baseline);new_manifest,new_plan,new=load(current)
    def comparable(p):return {k:v for k,v in p.items() if k not in ('precision_enabled','precision_settings')}
    if comparable(old_plan)!=comparable(new_plan):raise ValueError('Starting poses, profiles or evaluation limits differ')
    if old_manifest['goals_sha256']!=new_manifest['goals_sha256']:raise ValueError('Teaching images/poses differ')
    if old_manifest['fingerprint']['sha256']['scene.xml']!=new_manifest['fingerprint']['sha256']['scene.xml']:
        raise ValueError('Physical scene differs')
    names=dict(aruco='ArUco',natural='SIFT',learned='Learned');summaries=[]
    def metrics(rows):
        passed=[r for r in rows if r['pixel_success']]
        tool=[r['final_accuracy']['tool']['position_error_mm'] for r in passed]
        return dict(trials=len(rows),image_passes=len(passed),physical_passes=sum(r['physical_success'] for r in rows),
            outcomes=dict(Counter(r['outcome'] for r in rows)),safety_failures=sum(bool(r['safety_violations']) for r in rows),
            tool_median_mm=float(np.median(tool)) if tool else None,tool_max_mm=max(tool) if tool else None,
            tool_min_mm=min(tool) if tool else None,
            stop_median_s=float(np.median([r['completion_s'] for r in passed])) if passed else None)
    for mode in names:
        a=[r for r in old if r['mode']==mode];b=[r for r in new if r['mode']==mode]
        summaries.append(dict(mode=mode,baseline=metrics(a),precision=metrics(b)))
    record=dict(baseline_manifest_sha256=digest(baseline/'manifest.json'),current_manifest_sha256=digest(current/'manifest.json'),
        baseline_trials_sha256=digest(baseline/'trials.json'),current_trials_sha256=digest(current/'trials.json'),
        comparison_script_sha256=digest(__file__),plans_matched=True,teaching_hashes_matched=True,scene_matched=True,groups=summaries)
    (current/'comparison.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,4),constrained_layout=True)
    lookup={r['id']:r for r in old}
    for ax,mode in zip(axes,names):
        for row in [r for r in new if r['mode']==mode]:
            before=lookup[row['id']]
            a=before.get('final_accuracy',{}).get('tool',{}).get('position_error_mm')
            b=row.get('final_accuracy',{}).get('tool',{}).get('position_error_mm')
            if a is None or b is None:continue
            ax.plot([0,1],[a,b],color='#8393a1',alpha=.4,lw=.8)
            ax.scatter(0,a,marker='o' if before['pixel_success'] else 'x',s=23,color='#bf7014')
            ax.scatter(1,b,marker='o' if row['pixel_success'] else 'x',s=23,color='#157a8a')
        ax.axhline(2,color='firebrick',ls='--',lw=1)
        ax.set_xticks([0,1],['Historical 1 px','Precision'])
        ax.set_yscale('log');ax.set_ylim(.005,20);ax.set_title(names[mode]);ax.grid(axis='y',alpha=.2)
    axes[0].set_ylabel('Final tool-position error (mm, log scale)')
    fig.suptitle('Same 24 starts/profiles per matcher | circles: image pass; crosses: image failure')
    fig.savefig(current/'stopping-comparison.png',dpi=165);plt.close(fig)
    def f(value):return '—' if value is None else f'{value:.3f}'
    lines=['# Stopping-rule comparison','',
        'The historical and precision runs use the same 72 planned trials, teaching images, calibration profiles, 100 ms transport delay, 2 mm / 1 degree physical limits and 30 post-stop captures. The original result files are preserved.','',
        '| Matcher | Historical image / physical passes | Precision image / physical passes | Historical median tool mm | Precision median tool mm | Precision max tool mm |',
        '|---|---:|---:|---:|---:|---:|']
    for g in summaries:
        a,b=g['baseline'],g['precision']
        lines.append(f'| {names[g["mode"]]} | {a["image_passes"]}/{a["trials"]} / {a["physical_passes"]}/{a["trials"]} | {b["image_passes"]}/{b["trials"]} / {b["physical_passes"]}/{b["trials"]} | {f(a["tool_median_mm"])} | {f(b["tool_median_mm"])} | {f(b["tool_max_mm"])} |')
    lines+=['','Error statistics in the table are conditional on image convergence. Every planned trial remains in the pass counts and outcome lists. The plot includes all endpoints, including failures.','',
        '![Paired stopping comparison](stopping-comparison.png)','', '## Outcomes and convergence time','']
    for g in summaries:
        a,b=g['baseline'],g['precision']
        lines.append(f'- {names[g["mode"]]}: historical {a["outcomes"]}; precision {b["outcomes"]}. Median simulated time among image passes: {f(a["stop_median_s"])} s → {f(b["stop_median_s"])} s. Precision safety violations: {b["safety_failures"]}.')
    lines+=['','## What changed','',
        'SIFT and Learned acquire the target with their selected matcher, then refine its image alignment against the saved camera image. ArUco retains its own subpixel corners. All modes require the tighter per-corner/RMS limits plus a small image-derived camera correction and sufficient local sensitivity for a full hold. A valid-depth fallback also handles the exactly frontal IPPE initialization failure.','',
        'This comparison measures the combined implementation change; it does not isolate one threshold or attribute improvement to a single component. The local estimates are not ground truth. The physical scores use independent simulated poses.','',
        'The [45 pose probes](POSE_SENSITIVITY.md) show why translation and rotation together can pass a 1 px gate while missing the physical limits. Calibration error, noise, target geometry and real hardware remain separate sources of uncertainty. Two starting poses per profile are a small validation sample, not a reliability bound.','',
        '[Per-profile results](REPORT.md) · [Comparison data and hashes](comparison.json) · [Method and usage](../../../PRECISION_STOPPING.md)','']
    (current/'COMPARISON.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(summaries,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline',required=True,type=Path);parser.add_argument('--current',required=True,type=Path)
    args=parser.parse_args();compare(args.baseline,args.current)
