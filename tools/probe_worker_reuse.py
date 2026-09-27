"""Bounded diagnostic replay, excluded from control validation statistics."""
from __future__ import annotations
import argparse
import ctypes
from contextlib import contextmanager
import gzip
import hashlib
import json
import os
from pathlib import Path
import sys
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]
APP=ROOT/'outputs/visual-servoing-simulation'
sys.path.insert(0,str(APP))

@contextmanager
def gpu_record(output):
    from trace_latency_diagnostic import GPU_FIELDS,clock_pair
    metadata=dict(before=clock_pair(),gpu_query_fields=GPU_FIELDS)
    with (output/'gpu-samples.csv').open('w',encoding='utf-8') as log:
        monitor=subprocess.Popen(['nvidia-smi','--query-gpu='+','.join(GPU_FIELDS),'--format=csv,nounits','--loop-ms=250'],stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            yield
        finally:
            if monitor.poll() is None:monitor.terminate()
            monitor.wait(timeout=10)
            metadata['after']=clock_pair()
            (output/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')

def digest(tensors):
    h=hashlib.sha256()
    for name,tensor in sorted(tensors.items()):
        h.update(name.encode());h.update(tensor.detach().cpu().numpy().tobytes())
    return h.hexdigest()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=120)
    p.add_argument('--profile',action='store_true')
    p.add_argument('--stream',action='store_true')
    p.add_argument('--device-cast',action='store_true')
    p.add_argument('--benchmark',action='store_true')
    p.add_argument('--channels-last',action='store_true')
    p.add_argument('--blas-lt',action='store_true')
    p.add_argument('--graphs',action='store_true')
    p.add_argument('--prerender',action='store_true',help='Isolate inference from recurring OpenGL submissions')
    p.add_argument('--ablation',action='store_true',help='Six 40-second within-worker phases; overrides seconds')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    import cv2
    import numpy as np
    import mujoco
    import torch
    if args.benchmark:torch.backends.cudnn.benchmark=True
    if args.blas_lt:torch.backends.cuda.preferred_blas_library('cublaslt')
    from simulation import Simulation
    from learned_perception import LearnedImagePerception
    from precision import GoalRefinedPerception,load_precision_config
    from reference_image import load_reference
    from perception import NATURAL_REFERENCE
    from latency_stress import distribution
    from run_camera_robustness import fingerprint
    cv2.setNumThreads(4)
    baseline=json.loads(gzip.decompress((APP/'results/latency/20260922-sustained/traces/learned-session-000.json.gz').read_bytes()))
    poses=[f['qpos'] for f in baseline['sensor_frames'] if f['capture_active'] and f['generation']==1]
    rows=[];profiles=[]
    with Simulation() as sim:
        sim.set_target_mode('natural')
        base=LearnedImagePerception(cuda_graphs=False);torch=base.torch
        if args.channels_last:base.extractor.to(memory_format=torch.channels_last)
        graphs=[]
        if args.graphs:
            from learned_cuda_graphs import install_block_graphs
            graphs=install_block_graphs(base)
            base.cuda_graph_blocks=graphs
        from types import MethodType
        def legacy_tensor(self,rgb):
            return self.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1))).to(device=self.device,dtype=self.torch.float32)/255.
        original_tensor=MethodType(legacy_tensor,base)
        base._tensor=original_tensor
        def device_tensor(self,rgb):
            return self.torch.from_numpy(np.ascontiguousarray(rgb.transpose(2,0,1))).to(device=self.device).to(dtype=self.torch.float32)/255.
        if args.device_cast:
            base._tensor=MethodType(device_tensor,base)
        rgb,corners=load_reference(NATURAL_REFERENCE,sim.camera_intrinsics(),base.reference_config(sim.config),lambda image:base.observe(image).corners)
        detector=GoalRefinedPerception(base,rgb,corners,load_precision_config())
        base.profile_diagnostics=True;detector.profile_diagnostics=True
        detector.observe(sim.image())
        images=[]
        if args.prerender:
            for pose in poses:
                sim.data.qpos[:]=pose;mujoco.mj_forward(sim.model,sim.data)
                images.append(sim.image())
        def state():
            memory=torch.cuda.memory_stats()
            return dict(model=digest({**{'extractor.'+k:v for k,v in base.extractor.state_dict().items()},**{'matcher.'+k:v for k,v in base.matcher.state_dict().items()}}),
                template=digest(base.template_features),stream=torch.cuda.current_stream().cuda_stream,
                threads=torch.get_num_threads(),training=[base.extractor.training,base.matcher.training],
                memory={k:memory[k] for k in ('allocated_bytes.all.current','reserved_bytes.all.current','active_bytes.all.current','num_alloc_retries','num_ooms')})
        omp=ctypes.CDLL(str(Path(torch.__file__).parent/'lib/libiomp5md.dll'))
        omp.kmp_get_blocktime.restype=ctypes.c_int
        omp.kmp_set_blocktime.argtypes=[ctypes.c_int];omp.kmp_set_blocktime.restype=None
        before=state();before['blocktime_ms']=omp.kmp_get_blocktime()
        start=time.perf_counter();last_finished=start
        phases=['baseline','passive','device_cast','baseline','device_cast','passive']
        previous_phase=-1
        stream=torch.cuda.Stream() if args.stream else torch.cuda.current_stream()
        stream.wait_stream(torch.cuda.current_stream())
        with gpu_record(args.output), torch.cuda.stream(stream):
            while time.perf_counter()-start<(240 if args.ablation else args.seconds):
                i=len(rows)
                phase_index=min(5,int((time.perf_counter()-start)//40)) if args.ablation else 0
                phase=phases[phase_index] if args.ablation else ('device_cast' if args.device_cast else 'baseline')
                if args.ablation and phase_index!=previous_phase:
                    omp.kmp_set_blocktime(0 if phase=='passive' else before['blocktime_ms'])
                    base._tensor=MethodType(device_tensor,base) if phase=='device_cast' else original_tensor
                    previous_phase=phase_index
                sim.data.qpos[:]=poses[i%len(poses)];mujoco.mj_forward(sim.model,sim.data)
                t0=time.perf_counter();frame=images[i%len(images)] if images else sim.image();rendered=time.perf_counter()
                # Explicitly bypass only the identical-pixel cache in this diagnostic.
                base._last_rgb=None;detector._rgb=None
                elapsed=rendered-start
                profile_now=args.profile and (not profiles or elapsed>=60 and len(profiles)==1)
                prof=None
                if profile_now:
                    prof=torch.profiler.profile(activities=[torch.profiler.ProfilerActivity.CPU,torch.profiler.ProfilerActivity.CUDA])
                    prof.__enter__()
                cpu=time.process_time();thread=time.thread_time()
                observation=detector.observe(frame)
                finished=time.perf_counter()
                row=dict(elapsed_s=elapsed,phase_index=phase_index,phase=phase,blocktime_ms=omp.kmp_get_blocktime(),render_ms=1000*(rendered-t0),processing_ms=1000*(finished-rendered),
                    process_cpu_ms=1000*(time.process_time()-cpu),thread_cpu_ms=1000*(time.thread_time()-thread),
                    gap_ms=1000*(t0-last_finished),reason=observation.reason,stage_ms=observation.stage_ms,
                    corners=None if observation.corners is None else observation.corners.tolist(),profiled=profile_now)
                memory=torch.cuda.memory_stats()
                row['memory']={k:memory[k] for k in ('allocated_bytes.all.current','reserved_bytes.all.current','num_alloc_retries','num_ooms')}
                rows.append(row);last_finished=finished
                if prof:
                    prof.__exit__(None,None,None);profiles.append(prof)
                # Repeated active intervals and inter-attempt idle, no keepalive kernels.
                if (i+1)%len(poses)==0:time.sleep(1.1)
                else:time.sleep(max(0,1/30-(time.perf_counter()-t0)))
        after=state();after['blocktime_ms']=omp.kmp_get_blocktime()
        for i,prof in enumerate(profiles):
            prof.export_chrome_trace(str(args.output/f'profile-{i}.json'))
            (args.output/f'profile-{i}.txt').write_text(prof.key_averages().table(sort_by='self_cpu_time_total',row_limit=40),encoding='utf-8')
    result=dict(excluded_from_campaign=True,fingerprint=fingerprint(),tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),environment={k:os.environ.get(k) for k in ('KMP_BLOCKTIME','OMP_WAIT_POLICY','OMP_NUM_THREADS','CUDA_LAUNCH_BLOCKING')},
        before=before,after=after,alternate_stream=args.stream,device_cast=args.device_cast,benchmark=args.benchmark,channels_last=args.channels_last,blas_lt=args.blas_lt,graphs=[dict(replays=g.replays,fallbacks=g.fallbacks) for g in graphs],prerender=args.prerender,ablation=args.ablation,rows=rows,
        cohorts={name:distribution([r['processing_ms'] for r in rows if not r['profiled'] and lo<=r['elapsed_s']<hi]) for name,lo,hi in [('first_30s',0,30),('later',30,float('inf'))]})
    (args.output/'replay.json').write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('before','after','cohorts')},indent=2),flush=True)

if __name__=='__main__':main()
