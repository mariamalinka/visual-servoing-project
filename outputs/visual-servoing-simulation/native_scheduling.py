"""Scoped Windows scheduling for the owned control thread and sensor process.

HighQoS declares latency-sensitive work to Windows' power/core-selection policy.
It does not select a power plan, pin cores, lock GPU clocks, or issue extra work.
Only the control thread gets a modest priority increase; the sensor keeps its
normal scheduling priority. All previous settings are restored on shutdown.
"""
from __future__ import annotations
import os
import threading


class WindowsScheduling:
    def __init__(self):
        import ctypes
        from ctypes import wintypes
        self.ctypes=ctypes
        self.kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        class PowerState(ctypes.Structure):
            _fields_=[('version',wintypes.ULONG),('control',wintypes.ULONG),('state',wintypes.ULONG)]
        self.PowerState=PowerState
        for kind in ('Thread','Process'):
            current=getattr(self.kernel,'GetCurrent'+kind)
            current.argtypes=[];current.restype=wintypes.HANDLE
            for operation in ('Get','Set'):
                function=getattr(self.kernel,operation+kind+'Information')
                function.argtypes=[wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD]
                function.restype=wintypes.BOOL
        self.kernel.GetThreadPriority.argtypes=[wintypes.HANDLE]
        self.kernel.GetThreadPriority.restype=ctypes.c_int
        self.kernel.SetThreadPriority.argtypes=[wintypes.HANDLE,ctypes.c_int]
        self.kernel.SetThreadPriority.restype=wintypes.BOOL

    def _handle(self,scope):
        return getattr(self.kernel,'GetCurrent'+scope)()

    def _check(self,ok):
        if not ok:raise self.ctypes.WinError(self.ctypes.get_last_error())

    def get_qos(self,scope):
        value=self.PowerState(1,0,0)
        self._check(getattr(self.kernel,'Get'+scope+'Information')(
            self._handle(scope),3 if scope=='Thread' else 4,
            self.ctypes.byref(value),self.ctypes.sizeof(value)))
        return value.control,value.state

    def set_qos(self,scope,masks):
        value=self.PowerState(1,*masks)
        self._check(getattr(self.kernel,'Set'+scope+'Information')(
            self._handle(scope),3 if scope=='Thread' else 4,
            self.ctypes.byref(value),self.ctypes.sizeof(value)))

    def get_priority(self):
        value=self.kernel.GetThreadPriority(self._handle('Thread'))
        self._check(value!=0x7fffffff)
        return value

    def set_priority(self,value):
        self._check(self.kernel.SetThreadPriority(self._handle('Thread'),value))


class SchedulingLease:
    """Acquire and release on the same owned thread; failures stay observable."""
    def __init__(self,role,*,api=None):
        if role not in ('control','sensor'):raise ValueError('Unknown scheduling role')
        self.role=role
        self.scope='Thread' if role=='control' else 'Process'
        self.owner=threading.get_ident()
        self.previous_qos=None;self.previous_priority=None;self.closed=False
        self.metadata=dict(role=role,scope=self.scope.lower(),platform=os.name,
            thread_id=threading.get_native_id(),qos_applied=False,priority_applied=False,errors=[])
        self.api=api
        if api is None:
            if os.name!='nt':return
            try:self.api=WindowsScheduling()
            except (OSError,AttributeError) as exc:
                self._error('initialize',exc);return
        try:
            previous=self.api.get_qos(self.scope)
            desired=(previous[0]|1,previous[1]&~1)  # EXECUTION_SPEED controlled, throttling off.
            self.api.set_qos(self.scope,desired)
            self.previous_qos=previous
            observed=self.api.get_qos(self.scope)
            self.metadata.update(qos_previous=list(previous),qos_requested=list(desired),qos_observed=list(observed),
                qos_applied=bool(observed[0]&1 and not observed[1]&1))
            if not self.metadata['qos_applied']:self._error('verify_qos',RuntimeError('HighQoS was not retained'))
        except (OSError,AttributeError) as exc:self._error('high_qos',exc)
        if role=='control':
            try:
                previous=self.api.get_priority()
                desired=max(previous,1)  # THREAD_PRIORITY_ABOVE_NORMAL, never real-time.
                self.api.set_priority(desired)
                self.previous_priority=previous
                self.metadata.update(priority_previous=previous,priority_requested=desired,
                    priority_observed=self.api.get_priority(),priority_applied=True)
            except (OSError,AttributeError) as exc:self._error('priority',exc)

    def _error(self,stage,error):
        self.metadata['errors'].append(dict(stage=stage,type=type(error).__name__,message=str(error)))

    def close(self):
        if self.closed:return
        if threading.get_ident()!=self.owner:raise RuntimeError('Scheduling lease must close on its owning thread')
        self.closed=True
        if self.previous_priority is not None:
            try:self.api.set_priority(self.previous_priority)
            except OSError as exc:self._error('restore_priority',exc)
        if self.previous_qos is not None:
            try:self.api.set_qos(self.scope,self.previous_qos)
            except OSError as exc:self._error('restore_qos',exc)
