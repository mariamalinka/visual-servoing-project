"""Bounded native CPU/GPU/power recording, separate from timing validation."""
from __future__ import annotations
import argparse
import ctypes
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
from run_latency_stress import run_session,write_json,validate_fixed_inputs
from run_camera_robustness import fingerprint
from latency_stress import summarize_session
from trace_latency_diagnostic import GPU_FIELDS,clock_pair


def command(args,timeout=30):
    result=subprocess.run(args,capture_output=True,text=True,errors='replace',timeout=timeout)
    return dict(args=args,returncode=result.returncode,stdout=result.stdout,stderr=result.stderr)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=('learned','natural'),default='learned')
    args=p.parse_args()
    if not args.output.resolve().is_relative_to((APP/'results/latency').resolve()):
        p.error('Native recordings must stay under project results/latency')
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'traces').mkdir()
    baseline=APP/'results/latency/20260923-reuse-after/manifest.json'
    saved=json.loads(baseline.read_text(encoding='utf-8'))
    initial=fingerprint()
    validate_fixed_inputs(saved,initial)
    meta=dict(excluded_from_statistical_campaign=True,diagnostic_overhead=True,
        source_fingerprint=initial,baseline_manifest_sha256=hashlib.sha256(baseline.read_bytes()).hexdigest(),
        mode=args.mode,requested_session_s=120,process_id=os.getpid(),
        changed_source=[name for name,sha in initial['sha256'].items() if saved['fingerprint']['sha256'].get(name)!=sha],
        elevated=bool(ctypes.windll.shell32.IsUserAnAdmin()),before=clock_pair(),
        gpu_query_fields=GPU_FIELDS,gpu_sample_interval_ms=250,
        recording_mode='WPR bounded circular memory; not filemode',
        trace_scope='System-wide native trace retained locally, not uploaded or published')
    owned=False;monitor=None;log=None;trace_thread=None
    try:
        meta['privileges']=command(['whoami','/priv'])
        meta['wpr_status_before']=command(['wpr','-status'])
        if 'not recording' not in (meta['wpr_status_before']['stdout']+meta['wpr_status_before']['stderr']).lower():
            raise RuntimeError('Cannot confirm an idle WPR recorder; existing trace left untouched')
        meta['wpr_profiles']={profile:command(['wpr','-profiledetails',profile]) for profile in ('CPU','GPU','Power')}
        meta['wpr_start_before']=clock_pair()
        profile=ROOT/'tools/reuse_native.wprp'
        meta['wpr_profile_sha256']=hashlib.sha256(profile.read_bytes()).hexdigest()
        meta['wpr_start']=command(['wpr','-start',str(profile)+'!ReuseNative'])
        if meta['wpr_start']['returncode']:raise RuntimeError('Native recording failed to start')
        owned=True
        meta['wpr_start_after']=clock_pair()
        # Save the bounded ring immediately on the first control miss, so the
        # relevant context switches cannot be overwritten before session end.
        import realtime
        trigger=threading.Event()
        original_publish=realtime.RealtimeSession._publish
        def observed_publish(session,**state):
            original_publish(session,**state)
            if state.get('counters',{}).get('control_deadline_misses',0):
                trigger.set()
        realtime.RealtimeSession._publish=observed_publish
        def save_trace():
            hit=trigger.wait(100)
            meta['native_capture_trigger']='control_miss' if hit else '100_second_capture_cap'
            meta['native_capture_end']=clock_pair()
            meta['wpr_status_end']=command(['wpr','-status','collectors','-details'])
            meta['wpr_stop']=command(['wpr','-stop',str((args.output/'traces/native.etl').resolve()),
                'Reused worker native scheduler diagnosis','-skipPdbGen'],timeout=240)
        trace_thread=threading.Thread(target=save_trace,daemon=True)
        trace_thread.start()
        log=(args.output/'gpu-samples.csv').open('w',encoding='utf-8')
        monitor=subprocess.Popen(['nvidia-smi','--query-gpu='+','.join(GPU_FIELDS),
            '--format=csv,nounits','--loop-ms=250'],stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
        row=run_session(args.mode,120,saved['plan']['offset_degrees'],diagnostics=True)
        row.update(id=args.mode+'-native-diagnostic',mode=args.mode,round=0)
        raw=args.output/'traces/session.json.gz'
        with gzip.open(raw,'wt',encoding='utf-8') as f:json.dump(row,f,allow_nan=False)
        summary=summarize_session(row)
        summary.update(trace='traces/session.json.gz',trace_sha256=hashlib.sha256(raw.read_bytes()).hexdigest())
        write_json(args.output/'session.json',summary)
        print(json.dumps({k:summary[k] for k in ('attempts','alignments','outcomes','uncached_processing_ms')},indent=2),flush=True)
    except BaseException as exc:
        meta['error']=repr(exc)
        raise
    finally:
        if trace_thread is not None:
            trigger.set()
            trace_thread.join(timeout=250)
        if monitor is not None:
            if monitor.poll() is None:monitor.terminate()
            monitor.wait(timeout=10)
        if log is not None:log.close()
        meta['after']=clock_pair()
        if owned and trace_thread is None:
            try:
                meta['wpr_status_end']=command(['wpr','-status','collectors','-details'])
                meta['wpr_stop']=command(['wpr','-stop',str((args.output/'traces/native.etl').resolve()),
                    'Reused visual servoing worker latency diagnosis','-skipPdbGen'],timeout=240)
            except BaseException as exc:meta['wpr_stop_error']=repr(exc)
        meta['source_unchanged']=fingerprint()==initial
        write_json(args.output/'metadata.json',meta)
    if not meta.get('wpr_stop') or meta['wpr_stop']['returncode']:
        raise RuntimeError('Native trace was not successfully saved; inspect metadata')
    return 0


if __name__=='__main__':raise SystemExit(main())
