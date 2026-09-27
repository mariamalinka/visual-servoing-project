"""Opt-in bounded timing diagnostics; no file I/O or forced GPU synchronization."""
from collections import deque
import gc
import threading
import time
import os

if os.name=='nt':
    import ctypes
    from ctypes import wintypes
    _kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    _cycles=ctypes.WINFUNCTYPE(wintypes.BOOL,wintypes.HANDLE,ctypes.POINTER(ctypes.c_ulonglong))(
        ('QueryThreadCycleTime',_kernel))
    _kernel.GetCurrentThread.restype=wintypes.HANDLE
    def thread_cycles():
        value=ctypes.c_ulonglong()
        if not _cycles(_kernel.GetCurrentThread(),ctypes.byref(value)):
            return None
        return value.value
else:
    def thread_cycles():
        return None


class CycleProbe:
    def __init__(self, threshold_ms=10., capacity=2048):
        self.threshold_ms=threshold_ms
        self.rows=deque(maxlen=capacity)
        self.dropped=0
        self.marks=[]
        self.active=False
        self.generation=0
        self.sleep_until=None
        self.previous=None

    def start(self, active, generation):
        self.marks=[('start',time.perf_counter(),time.thread_time(),thread_cycles())]
        self.active=active
        self.generation=generation
        self.sleep_until=None

    def mark(self, name):
        self.marks.append((name,time.perf_counter(),time.thread_time(),thread_cycles()))

    def finish(self):
        if not self.marks:
            return
        self.mark('sleep_or_descheduled')
        first,last=self.marks[0],self.marks[-1]
        wall_ms=1000*(last[1]-first[1])
        if wall_ms>=self.threshold_ms:
            stages=[]
            for a,b in zip(self.marks,self.marks[1:]):
                stages.append(dict(stage=b[0],started_s=a[1],finished_s=b[1],
                    wall_ms=1000*(b[1]-a[1]),thread_cpu_ms=1000*(b[2]-a[2]),
                    cpu_cycles=None if a[3] is None or b[3] is None else b[3]-a[3]))
            row=dict(started_s=first[1],finished_s=last[1],wall_ms=wall_ms,
                thread_cpu_ms=1000*(last[2]-first[2]),
                cpu_cycles=None if first[3] is None or last[3] is None else last[3]-first[3],active=self.active,generation=self.generation,
                wake_lateness_ms=None if self.sleep_until is None else 1000*(last[1]-self.sleep_until),
                thread_id=threading.get_native_id(),stages=stages)
            if len(self.rows)==self.rows.maxlen:
                self.dropped+=1
            self.rows.append(row)
            self.previous=row
        self.marks=[]


class GcProbe:
    def __init__(self, capacity=2048):
        self.rows=deque(maxlen=capacity)
        self.dropped=0
        self.starts={}

    def callback(self, phase, info):
        tid=threading.get_native_id()
        if phase=='start':
            self.starts[tid]=(time.perf_counter(),time.thread_time())
        elif tid in self.starts:
            start,cpu=self.starts.pop(tid)
            if len(self.rows)==self.rows.maxlen:
                self.dropped+=1
            self.rows.append(dict(started_s=start,finished_s=time.perf_counter(),
                thread_cpu_ms=1000*(time.thread_time()-cpu),thread_id=tid,**info))

    def start(self):
        gc.callbacks.append(self.callback)
        return self

    def close(self):
        if self.callback in gc.callbacks:
            gc.callbacks.remove(self.callback)
