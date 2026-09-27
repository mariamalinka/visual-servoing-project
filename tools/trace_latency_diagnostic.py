"""Separate native diagnostic run; never pool its timings with the stress campaign.

Records GPU clock/power samples and optionally WPR CPU scheduling events. Does
not change power, GPU clock, affinity, priority, controller or model settings.
"""
from __future__ import annotations
import argparse
import ctypes
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))
from run_latency_stress import run_session, write_json
from latency_stress import summarize_session
from run_camera_robustness import fingerprint

GPU_FIELDS=['timestamp','name','pstate','temperature.gpu','utilization.gpu','utilization.memory',
    'clocks.sm','clocks.mem','power.draw','power.limit','enforced.power.limit',
    'clocks_event_reasons.active','clocks_event_reasons.gpu_idle',
    'clocks_event_reasons.sw_power_cap','clocks_event_reasons.sw_thermal_slowdown',
    'clocks_event_reasons.hw_thermal_slowdown','clocks_event_reasons.hw_power_brake_slowdown']


def power_status():
    class Status(ctypes.Structure):
        _fields_=[('ACLineStatus',ctypes.c_ubyte),('BatteryFlag',ctypes.c_ubyte),
            ('BatteryLifePercent',ctypes.c_ubyte),('SystemStatusFlag',ctypes.c_ubyte),
            ('BatteryLifeTime',ctypes.c_ulong),('BatteryFullLifeTime',ctypes.c_ulong)]
    status=Status()
    if not ctypes.windll.kernel32.GetSystemPowerStatus(ctypes.byref(status)):
        return {'error':'GetSystemPowerStatus failed'}
    return {name:getattr(status,name) for name,_ in status._fields_}


def clock_pair():
    return dict(perf_counter_s=time.perf_counter(),time_ns=time.time_ns(),
        utc=datetime.now(timezone.utc).isoformat(),power=power_status())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=120)
    parser.add_argument('--wpr',action='store_true')
    parser.add_argument('--no-diagnostics',action='store_true',help='Disable in-process probes; keep external GPU/native recording')
    args=parser.parse_args()
    if not 1<=args.seconds<=120:parser.error('seconds must be in [1,120]')
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'traces').mkdir()
    baseline=json.loads((APP/'results/latency/tail-after-final-v2/manifest.json').read_text(encoding='utf-8'))
    initial=fingerprint()
    meta=dict(excluded_from_statistical_campaign=True,diagnostic_overhead=True,
        source_fingerprint=initial,gpu_query_fields=GPU_FIELDS,gpu_sample_interval_ms=250,
        before=clock_pair(),wpr_requested=args.wpr,in_process_diagnostics=not args.no_diagnostics)
    owned_wpr=False
    monitor=None
    log=None
    try:
        if args.wpr:
            status=subprocess.run(['wpr','-status'],capture_output=True,text=True)
            meta['wpr_status_before']=dict(returncode=status.returncode,stdout=status.stdout,stderr=status.stderr)
            if 'not recording' not in (status.stdout+status.stderr).lower():
                raise RuntimeError('WPR status did not confirm no existing recording; leaving it untouched')
            started=subprocess.run(['wpr','-start','CPU'],capture_output=True,text=True)
            meta['wpr_start']=dict(returncode=started.returncode,stdout=started.stdout,stderr=started.stderr)
            if started.returncode:
                raise RuntimeError('WPR CPU start failed; see diagnostic metadata')
            owned_wpr=True
            meta['wpr_started_clock']=clock_pair()
        log=(args.output/'gpu-samples.csv').open('w',encoding='utf-8')
        monitor=subprocess.Popen(['nvidia-smi','--query-gpu='+','.join(GPU_FIELDS),
            '--format=csv,nounits','--loop-ms=250'],stdout=log,stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NO_WINDOW)
        row=run_session('learned',args.seconds,baseline['offset_degrees'],not args.no_diagnostics)
        row.update(id='learned-native-diagnostic',mode='learned',round=0)
        raw=args.output/'traces/session.json.gz'
        with gzip.open(raw,'wt',encoding='utf-8') as stream:
            json.dump(row,stream,allow_nan=False)
        summary=summarize_session(row)
        summary['trace']='traces/session.json.gz'
        summary['trace_sha256']=hashlib.sha256(raw.read_bytes()).hexdigest()
        write_json(args.output/'session.json',summary)
        print(json.dumps({k:summary[k] for k in ('attempts','alignments','outcomes','uncached_processing_ms','counters')},indent=2),flush=True)
    except Exception as exc:
        meta['error']=repr(exc)
        raise
    finally:
        if monitor is not None:
            if monitor.poll() is None:monitor.terminate()
            monitor.wait(timeout=10)
            meta['gpu_monitor_returncode']=monitor.returncode
        if log is not None:log.close()
        meta['after']=clock_pair()
        try:
            if owned_wpr:
                stopped=subprocess.run(['wpr','-stop',str((args.output/'traces/cpu.etl').resolve())],
                    capture_output=True,text=True,timeout=180)
                meta['wpr_stop']=dict(returncode=stopped.returncode,stdout=stopped.stdout,stderr=stopped.stderr)
        except Exception as exc:
            meta['wpr_stop_error']=repr(exc)
            raise
        finally:
            meta['source_unchanged']=fingerprint()==initial
            write_json(args.output/'metadata.json',meta)
    return 0


if __name__=='__main__':raise SystemExit(main())
