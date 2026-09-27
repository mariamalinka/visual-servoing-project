"""Audit the unchanged sustained campaign and report the requested latency metrics."""
from pathlib import Path
import argparse,gzip,hashlib,json,re


def read(p):return json.loads(p.read_text(encoding='utf-8'))
def ms(d):return ' / '.join(f'{d[k]:.1f}' for k in ('p95','p99','max'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('before',type=Path);p.add_argument('after',type=Path)
    p.add_argument('--tests',type=Path,required=True)
    args=p.parse_args();bm=read(args.before/'manifest.json');am=read(args.after/'manifest.json')
    assert bm['plan']==am['plan'],'Campaign plan changed'
    assert bm['fingerprint']['runtime']==am['fingerprint']['runtime']
    assert am['baseline_manifest_sha256']==hashlib.sha256((args.before/'manifest.json').read_bytes()).hexdigest()
    changed={n for n,s in bm['fingerprint']['sha256'].items() if am['fingerprint']['sha256'].get(n)!=s}
    added=set(am['fingerprint']['sha256'])-set(bm['fingerprint']['sha256'])
    assert changed=={'realtime.py'},changed
    assert added=={'native_scheduling.py'},added
    summary={phase:read(directory/'summary.json') for phase,directory in [('Before',args.before),('After',args.after)]}
    audits={}
    for phase,directory in [('Before',args.before),('After',args.after)]:
        records=[]
        for s in read(directory/'sessions.json'):
            raw=(directory/s['trace']).read_bytes();assert hashlib.sha256(raw).hexdigest()==s['trace_sha256']
            row=json.loads(gzip.decompress(raw))
            records.append(dict(id=row['id'],control=row.get('control_scheduling'),
                sensor=row.get('execution_settings',{}).get('scheduling'),
                ac=[row[key].get('power') for key in ('gpu_before','gpu_after')]))
            assert row['counts']['unsafe_motion_ticks']==0
            assert row['counts']['post_stop_motion_ticks']==0
            assert row['counts']['contacts']==0
            assert all(a['stop_stayed_latched'] for a in row['attempts'])
            if phase=='After':
                assert records[-1]['control']['qos_applied']
                assert records[-1]['control']['priority_observed']>=1
                assert records[-1]['sensor']['qos_applied']
                assert not records[-1]['sensor']['priority_applied']
                assert not records[-1]['control']['errors'] and not records[-1]['sensor']['errors']
        audits[phase]=records
    tests=args.tests.read_text(encoding='utf-8-sig')
    match=re.search(r'Ran (\d+) tests in ([\d.]+)s\s+OK\s*$',tests);assert match
    completion=read(args.after/'completion.json')
    assert completion['source_unchanged'] and completion['telemetry_complete']
    audit=dict(plan_identical=True,dependencies_and_fixed_inputs_identical=True,changed=sorted(changed),added=sorted(added),
        baseline_manifest_sha256=am['baseline_manifest_sha256'],sessions=audits,
        tests=int(match[1]),test_log_sha256=hashlib.sha256(args.tests.read_bytes()).hexdigest(),
        all_stops_latched=True,unsafe_motion_ticks=0,post_stop_motion_ticks=0,contacts=0)
    (args.after/'comparison-audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    lines=['| Method | Campaign | Alignment success | Freshness trips | Control misses | Processing p95 / p99 / max (ms) | Capture to command p95 / p99 / max (ms) |',
        '|---|---|---:|---:|---:|---:|---:|']
    for mode,name in [('natural','SIFT'),('learned','Learned GPU')]:
        for phase in summary:
            r=summary[phase]['methods'][mode]
            lines.append(f"| {name} | {phase} | {r['alignments']}/{r['attempts']} ({100*r['alignments']/r['attempts']:.1f}%) | {r['freshness_watchdog_trips']} | {r['counters']['control_deadline_misses']} | {ms(r['uncached_processing_ms'])} | {ms(r['capture_to_command_ms'])} |")
    lines+=['','Processing includes every uncached active frame, including failed and late work. Capture-to-command latency covers accepted commands.','',
        '| Method | Campaign | First alignment processing p95 / p99 / max (ms) | Later alignment processing p95 / p99 / max (ms) |',
        '|---|---|---:|---:|']
    for mode,name in [('natural','SIFT'),('learned','Learned GPU')]:
        for phase in summary:
            r=summary[phase]['methods'][mode]['cohort_uncached_processing_ms']
            lines.append(f"| {name} | {phase} | {ms(r['first_attempt'])} | {ms(r['reused_worker'])} |")
    text='\n'.join(lines)+'\n';(args.after/'COMPARISON.md').write_text(text,encoding='utf-8');print(text)


if __name__=='__main__':main()
