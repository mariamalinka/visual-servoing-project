"""Controlled fresh/reused Learned worker comparison; no runtime modifications.

Native recording is a separate instrumented run, excluded from the comparison.
Fresh workers are an experimental arm, not a production recovery strategy.
"""
from pathlib import Path
from collections import deque
from datetime import datetime,timezone
import argparse,gzip,hashlib,json,os,subprocess,sys,threading,time

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
import run_latency_stress as stress
from latency_stress import summarize_session
from run_camera_robustness import fingerprint
from trace_latency_diagnostic import GPU_FIELDS,clock_pair

def command(args,timeout=30):
    r=subprocess.run(args,capture_output=True,text=True,errors='replace',timeout=timeout)
    return {'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr,'args':args}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--native',action='store_true')
    args=p.parse_args()
    if not args.output.resolve().is_relative_to((APP/'results/latency').resolve()):p.error('Output must be in results/latency')
    baseline=APP/'results/latency/20260925-native-scheduling-after/manifest.json'
    initial=fingerprint();saved=json.loads(baseline.read_text(encoding='utf-8'))
    if initial!=saved['fingerprint']:raise RuntimeError('Source or dependencies changed since the completed scheduling campaign')
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'traces').mkdir()
    fields=GPU_FIELDS+['memory.used','memory.free','memory.total']
    manifest={'created_utc':datetime.now(timezone.utc).isoformat(),'fingerprint':initial,
        'baseline_manifest_sha256':hashlib.sha256(baseline.read_bytes()).hexdigest(),
        'plan':{'blocks':8,'fresh_per_block':4,'reused_seconds':120,'offset_degrees':saved['plan']['offset_degrees'],
            'max_age_s':.4,'transport_s':.05,'max_control_gap_s':.05,'post_stop_observation_s':1.1,
            'warmup':'Unmodified RealtimeSession initialization and one-image warm-up',
            'order':'Even blocks: fresh group then reuse; odd blocks: reuse then fresh group',
            'fresh_session_seconds':0,'fresh_semantics':'Existing runner completes its first alignment and post-stop observation before checking the zero duration',
            'slow_frame_ms':200,'gpu_sample_interval_ms':250},
        'native_diagnostic_only':args.native,'gpu_query_fields':fields,'before':clock_pair(),
        'local_utc_offset_seconds':int(datetime.now().astimezone().utcoffset().total_seconds()),
        'tool_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    stress.write_json(args.output/'manifest.json',manifest)
    meta={'before':manifest['before'],'gpu_query_fields':fields,'excluded_from_statistical_campaign':args.native}
    trigger=threading.Event();stop_trace=threading.Event();native_thread=None
    if args.native:
        import ctypes
        if not ctypes.windll.shell32.IsUserAnAdmin():raise RuntimeError('Native tracing requires an elevated Windows token')
        status=command(['wpr','-status']);meta['wpr_status_before']=status
        if 'not recording' not in (status['stdout']+status['stderr']).lower():raise RuntimeError('Existing WPR recording left untouched')
        def record_native():
            owned=False
            try:
                hit=trigger.wait(80)
                if stop_trace.is_set():return
                meta['trigger']='three_slow_reused_frames' if hit else '80_second_fallback'
                meta['wpr_start_before']=clock_pair()
                profile=ROOT/'tools/worker_lifetime_native.wprp'
                meta['profile_sha256']=hashlib.sha256(profile.read_bytes()).hexdigest()
                started=command(['wpr','-start',str(profile)+'!ReuseNative']);meta['wpr_start']=started
                if started['returncode']:raise RuntimeError('WPR start failed')
                owned=True;meta['wpr_start_after']=clock_pair()
                stop_trace.wait(20)
            except BaseException as exc:meta['native_error']=repr(exc)
            finally:
                if owned:
                    meta['wpr_end_before']=clock_pair()
                    meta['wpr_status_end']=command(['wpr','-status','collectors','-details'])
                    meta['wpr_stop']=command(['wpr','-stop',str(args.output.resolve()/'traces/native.etl'),
                        'Worker lifetime degraded CPU samples, context switches and GPU scheduling','-skipPdbGen'],240)
                    meta['wpr_end_after']=clock_pair()
        native_thread=threading.Thread(target=record_native,daemon=True)

    class DiagnosticSession(stress.RealtimeSession):
        def __init__(self,*a,**kw):
            self._slow_times=deque(maxlen=8);self._seen_sequence=None
            super().__init__(*a,**kw)
        def _publish(self,**state):
            super()._publish(**state)
            if not args.native or trigger.is_set() or not self.sensor_rows:return
            f=self.sensor_rows[-1]
            if f['sequence']==self._seen_sequence:return
            self._seen_sequence=f['sequence']
            if f['capture_active'] and f['generation']>1 and not f['stage_ms'].get('cache_hit',False) and f['finished_s']-f['rendered_s']>=.2:
                now=time.perf_counter();self._slow_times.append(now)
                if sum(now-t<=15 for t in self._slow_times)>=3:
                    meta['degraded_trigger_frame']={'sequence':f['sequence'],'generation':f['generation'],'at_s':now}
                    trigger.set()
        def report(self):
            row=super().report();row['worker_pid']=None if self.worker is None else self.worker.pid
            return row
    stress.RealtimeSession=DiagnosticSession
    monitor=None;summaries=[]
    try:
        with (args.output/'gpu-samples.csv').open('w',encoding='utf-8') as log:
            monitor=subprocess.Popen(['nvidia-smi','--query-gpu='+','.join(fields),'--format=csv,nounits','--loop-ms=250'],
                stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
            if native_thread:native_thread.start()
            specs=[]
            for block in range(1 if args.native else 8):
                fresh=[('fresh',block,j,0) for j in range(4)]
                reused=[('reused',block,0,120)]
                specs.extend(reused if args.native else (fresh+reused if block%2==0 else reused+fresh))
            for arm,block,index,seconds in specs:
                identifier=f'{arm}-b{block:02d}-a{index:02d}'
                print('Starting '+identifier,flush=True)
                before=clock_pair()
                row=stress.run_session('learned',seconds,manifest['plan']['offset_degrees'],diagnostics=True)
                row.update(id=identifier,mode='learned',round=block,arm=arm,block=block,arm_index=index,
                    boundary_before=before,boundary_after=clock_pair())
                if arm=='fresh' and len(row['attempts'])!=1:raise RuntimeError('Fresh arm did not contain exactly one alignment')
                target=args.output/'traces'/(identifier+'.json.gz')
                with gzip.open(target,'wt',encoding='utf-8') as stream:json.dump(row,stream,allow_nan=False)
                summary=summarize_session(row)
                summary.update(arm=arm,block=block,arm_index=index,worker_pid=row['worker_pid'],trace='traces/'+target.name,
                    trace_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),boundary_before=before,boundary_after=row['boundary_after'])
                summaries.append(summary);stress.write_json(args.output/'sessions.json',summaries)
                if row['initialization_error'] or row['error'] or not summary['telemetry_complete']:raise RuntimeError('Invalid session; saved evidence retained')
                if any(row['counts'][key] for key in ('unsafe_motion_ticks','post_stop_motion_ticks','contacts')) or not all(a['stop_stayed_latched'] for a in row['attempts']):
                    raise RuntimeError('Safety audit failed')
                print(f"{identifier}: {summary['alignments']}/{summary['attempts']}; freshness={summary['freshness_watchdog_trips']}; control={summary['counters']['control_deadline_misses']}",flush=True)
                if monitor.poll() is not None:raise RuntimeError('GPU telemetry exited before test completion')
    except BaseException as exc:
        meta['error']=repr(exc);raise
    finally:
        stop_trace.set();trigger.set()
        if native_thread and native_thread.ident:native_thread.join(270)
        if monitor:
            if monitor.poll() is None:monitor.terminate()
            monitor.wait(10)
        meta['after']=clock_pair();meta['source_unchanged']=fingerprint()==initial
        stress.write_json(args.output/'metadata.json',meta)
    if args.native and (meta.get('native_error') or meta.get('wpr_stop',{}).get('returncode')!=0):raise RuntimeError('Native recording incomplete')
    if not meta['source_unchanged']:raise RuntimeError('Source changed during comparison')
    stress.write_json(args.output/'completion.json',dict(completed_utc=datetime.now(timezone.utc).isoformat(),sessions=len(summaries),
        source_unchanged=True,native=args.native,all_safety_audits_passed=True))

if __name__=='__main__':main()
