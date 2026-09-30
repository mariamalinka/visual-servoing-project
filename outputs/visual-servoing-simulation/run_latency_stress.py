"""Run predeclared sustained sessions without changing alignment or safety settings."""
from __future__ import annotations
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np
from realtime import RealtimeSession, RuntimeConfig
from run_camera_robustness import fingerprint
from simulation import ROOT
from latency_stress import summarize_session


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def power_status():
    if os.name!='nt':return None
    import ctypes
    class Status(ctypes.Structure):
        _fields_=[('ACLineStatus',ctypes.c_ubyte),('BatteryFlag',ctypes.c_ubyte),
            ('BatteryLifePercent',ctypes.c_ubyte),('SystemStatusFlag',ctypes.c_ubyte),
            ('BatteryLifeTime',ctypes.c_ulong),('BatteryFullLifeTime',ctypes.c_ulong)]
    status=Status()
    if not ctypes.windll.kernel32.GetSystemPowerStatus(ctypes.byref(status)):
        return dict(error='GetSystemPowerStatus failed')
    return {name:getattr(status,name) for name,_ in status._fields_}


def gpu_status():
    try:
        result=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,power.draw,clocks.sm',
            '--format=csv,noheader'],capture_output=True,text=True,timeout=10)
        return dict(returncode=result.returncode,stdout=result.stdout.strip(),stderr=result.stderr.strip(),power=power_status())
    except (OSError,subprocess.TimeoutExpired) as exc:
        return dict(error=str(exc),power=power_status())


def wait_for(session,predicate,timeout):
    deadline=time.perf_counter()+timeout
    while time.perf_counter()<deadline:
        state=session.snapshot()
        if state.get('error') or predicate(state):
            return state
        time.sleep(.01)
    raise TimeoutError('Runtime state transition timed out')


def run_session(mode,seconds,offset,diagnostics=True):
    # Match the old experiment exactly: warm-up in the worker, then auto-arm.
    session=RealtimeSession(mode,RuntimeConfig(max_age_s=.4,transport_s=.05,stale_resume_s=0),offset=offset,
        diagnostics=diagnostics,telemetry_capacity=8192).start()
    attempts=[]
    initial_error=None
    begin=time.perf_counter()
    started_utc=datetime.now(timezone.utc).isoformat()
    ready_at=None
    try:
        if not session.ready.wait(125):
            raise TimeoutError('Initialization exceeded the existing timeout')
        state=session.snapshot()
        if state.get('error'):
            initial_error=state['error']
        else:
            ready_at=time.perf_counter()
            deadline=ready_at+seconds
            while True:
                generation=state.get('generation',1)
                # Auto-start may precede the first 30 Hz published state.
                state=wait_for(session,lambda s:s.get('generation',0)>=generation and
                    s.get('ready') and not s.get('active') and s['status']!='idle',123)
                attempts.append(dict(index=len(attempts),generation=state.get('generation',generation),
                    outcome=state['status'],observed_stop_s=time.perf_counter(),error_px=state.get('error_px'),
                    counters=dict(state.get('counters',{}))))
                time.sleep(1.1)  # Preserve the original post-stop observation window.
                settled=session.snapshot()
                attempts[-1]['stop_stayed_latched']=bool(not settled.get('active') and
                    settled['status']==state['status'] and not np.any(settled.get('command',0)))
                if settled.get('error') or time.perf_counter()>=deadline:
                    break
                session.command('offset')
                desired=np.asarray(settled['home_qpos'])+np.deg2rad(offset)
                state=wait_for(session,lambda s:s['status']=='idle' and not s['active'] and
                    np.allclose(s['qpos'],desired,atol=1e-8,rtol=0),5)
                if state.get('error'):
                    break
                previous=state['generation']
                session.command('align')
                state=wait_for(session,lambda s:s.get('generation',0)>previous,5)
                if state.get('error'):
                    break
            # Stop acquiring, drain the one outstanding result, then close.
            session.command('stream')
            wait_for(session,lambda s:not s.get('stream',True) and s.get('sensor_available',False),5)
    except (TimeoutError,RuntimeError) as exc:
        initial_error=repr(exc)
        session.command('stop')
    finally:
        session.close()
    row=session.report()
    row.update(attempts=attempts,initialization_error=initial_error,started_utc=started_utc,
        elapsed_s=time.perf_counter()-begin,ready_elapsed_s=None if ready_at is None else time.perf_counter()-ready_at,
        requested_session_s=seconds,offset_degrees=offset)
    return row


def validate_fixed_inputs(baseline, current):
    if current['runtime']!=baseline['fingerprint']['runtime']:
        raise ValueError('Software versions differ from the preceding comparison')
    for name,digest in baseline['fingerprint']['sha256'].items():
        if name in ('realtime.py','learned_perception.py','precision.py','run_latency_stress.py'):
            continue  # Execution/instrumentation changes; every version is fingerprinted.
        if current['sha256'].get(name)!=digest:
            raise ValueError('A pre-existing input changed: '+name)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline',type=Path,default=ROOT/'results/latency/tail-after-final-v2/manifest.json')
    parser.add_argument('--min-sessions',type=int,default=12)
    parser.add_argument('--max-sessions',type=int,default=20)
    parser.add_argument('--session-seconds',type=float,default=120)
    parser.add_argument('--target-uncached-frames',type=int,default=10000)
    parser.add_argument('--modes',nargs='+',choices=('natural','learned'),default=['natural','learned'])
    parser.add_argument('--no-diagnostics',action='store_true')
    parser.add_argument('--resume',action='store_true')
    args=parser.parse_args()
    if not 1<=args.min_sessions<=args.max_sessions<=100 or not 1<=args.session_seconds<=120 or args.target_uncached_frames<0 or len(set(args.modes))!=len(args.modes):
        parser.error('Invalid campaign bounds')
    baseline=json.loads(args.baseline.read_text(encoding='utf-8'))
    baseline_digest=hashlib.sha256(args.baseline.read_bytes()).hexdigest()
    current=fingerprint()
    validate_fixed_inputs(baseline,current)
    cfg=asdict(RuntimeConfig(max_age_s=.4,transport_s=.05,stale_resume_s=0))  # Historical stop response.
    if 'plan' in baseline:
        if baseline['plan']['config']!=cfg:raise ValueError('Runtime conditions differ from the baseline')
        offset=baseline['plan']['offset_degrees']
    else:
        if not all(s['config']==cfg for s in baseline['trials']):raise ValueError('Runtime conditions differ from the baseline')
        offset=baseline['offset_degrees']
    plan=dict(schema_version=1,minimum_sessions_per_method=args.min_sessions,
        maximum_sessions_per_method=args.max_sessions,minimum_session_s=args.session_seconds,
        target_uncached_active_frames_per_method=args.target_uncached_frames,modes=args.modes,
        diagnostics=not args.no_diagnostics,config=cfg,offset_degrees=offset,
        stop_rule='Finish a complete round after minimum sessions AND uncached-frame targets, or maximum sessions. Never stop early based on successes or latency.',
        warmup='Identical worker initialization and one initial detector.observe(sensor.image()) before auto-arming each new process. Subsequent cycles reuse the worker.',
        post_stop_s=1.1,telemetry_frames=8192,
        specs=[dict(id=f'{mode}-session-{i:03d}',mode=mode,round=i) for i in range(args.max_sessions) for mode in args.modes])
    if args.resume:
        saved=json.loads((args.output/'manifest.json').read_text(encoding='utf-8'))
        if saved['plan']!=plan or saved['fingerprint']!=current or saved['baseline_manifest_sha256']!=baseline_digest:
            raise ValueError('Cannot resume changed plan/source/runtime')
    else:
        args.output.mkdir(parents=True,exist_ok=False)
        (args.output/'traces').mkdir()
        write_json(args.output/'manifest.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),
            fingerprint=current,baseline_manifest=str(args.baseline.resolve()),
            baseline_manifest_sha256=baseline_digest,
            plan=plan,clock={name:vars(time.get_clock_info(name)) for name in ('perf_counter','thread_time','process_time')},
            hardware=gpu_status()))
    summaries=[]
    for spec in plan['specs']:
        summary_path=args.output/(spec['id']+'.json')
        if summary_path.exists():
            summary=json.loads(summary_path.read_text(encoding='utf-8'))
        else:
            if fingerprint()!=current:
                raise RuntimeError('Source changed during the campaign')
            before_gpu=gpu_status()
            row=run_session(spec['mode'],args.session_seconds,plan['offset_degrees'],not args.no_diagnostics)
            row.update(spec)
            row['gpu_before']=before_gpu;row['gpu_after']=gpu_status()
            summary=summarize_session(row)
            summary['gpu_before']=before_gpu;summary['gpu_after']=row['gpu_after']
            raw=args.output/'traces'/(spec['id']+'.json.gz')
            with gzip.open(raw,'wt',encoding='utf-8') as stream:
                json.dump(row,stream,allow_nan=False)
            summary['trace']=raw.relative_to(args.output).as_posix()
            summary['trace_sha256']=hashlib.sha256(raw.read_bytes()).hexdigest()
            write_json(summary_path,summary)
        summaries.append(summary)
        write_json(args.output/'sessions.json',summaries)
        print(f"{spec['id']}: aligned {summary['alignments']}/{summary['attempts']}; "
            f"uncached={0 if summary['uncached_processing_ms'] is None else summary['uncached_processing_ms']['count']}; "
            f"freshness={summary['freshness_watchdog_trips']}; control_misses={summary['counters']['control_deadline_misses']}",flush=True)
        if spec['mode']==args.modes[-1]:
            counts={mode:sum((r['uncached_processing_ms'] or {}).get('count',0) for r in summaries if r['mode']==mode) for mode in args.modes}
            print('Uncached totals: '+str(counts),flush=True)
            if spec['round']+1>=args.min_sessions and all(v>=args.target_uncached_frames for v in counts.values()):
                break
    complete=dict(completed_utc=datetime.now(timezone.utc).isoformat(),sessions=len(summaries),
        sample_targets_met=all(sum((r['uncached_processing_ms'] or {}).get('count',0) for r in summaries if r['mode']==mode)>=args.target_uncached_frames for mode in args.modes),
        telemetry_complete=all(r['telemetry_complete'] for r in summaries),
        all_alignments_succeeded=all(r['alignments']==r['attempts'] and not r['initialization_error'] for r in summaries),
        control_deadline_misses=sum(r['counters']['control_deadline_misses'] for r in summaries),
        freshness_watchdog_trips=sum(r['freshness_watchdog_trips'] for r in summaries),
        source_unchanged=fingerprint()==current)
    write_json(args.output/'completion.json',complete)
    print(json.dumps(complete,indent=2),flush=True)
    return 0 if complete['telemetry_complete'] and complete['source_unchanged'] else 1


if __name__=='__main__':
    raise SystemExit(main())
