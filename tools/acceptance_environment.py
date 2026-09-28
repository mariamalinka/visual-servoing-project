"""GPU throttle and CPU/thermal telemetry for the acceptance test (diagnostic only).

Nothing here changes production code, drivers, power plans or clocks. The monitors
are separate processes that only read counters:

* nvidia-smi logs clocks, utilisation, temperature, power and the
  clocks_event_reasons bitmask, which this module decodes into per-reason flags.
* PowerShell/CIM logs Windows' ACPI thermal zones (temperature, passive-cooling
  limit, throttle reasons) and processor frequency (% of maximum). These are the
  counters behind Performance Monitor; they need no administrator rights and their
  class names are not localized.

After the run, samples are placed on the same clock as the controller telemetry and
each failed alignment and slow frame is checked against the throttle state.
"""
from __future__ import annotations

import base64
from bisect import bisect_left
import csv
from datetime import datetime, timezone
import os
from pathlib import Path
import subprocess
import time

# NVML clocks_event_reasons bits (nvidia-smi -q calls them "Clocks Event Reasons").
REASONS = {
    0x1: 'gpu_idle', 0x2: 'applications_clocks', 0x4: 'sw_power_cap', 0x8: 'hw_slowdown',
    0x10: 'sync_boost', 0x20: 'sw_thermal_slowdown', 0x40: 'hw_thermal_slowdown',
    0x80: 'hw_power_brake', 0x100: 'display_clocks',
}
# Reasons that lower clocks below what the workload asks for.
LIMITING = ('sw_power_cap', 'hw_slowdown', 'sw_thermal_slowdown', 'hw_thermal_slowdown', 'hw_power_brake')
GPU_FIELDS = ('timestamp,index,pstate,clocks.sm,clocks.mem,utilization.gpu,temperature.gpu,power.draw,'
              'power.limit,enforced.power.limit,memory.used,clocks_event_reasons.active')
GPU_FIELDS_MINIMAL = 'timestamp,index,pstate,clocks.sm,utilization.gpu,temperature.gpu,power.draw,clocks_event_reasons.active'
SLOW_FRAME_MS = 150.0
LOW_GPU_CLOCK_MHZ = 400    # Busy (>= 20% utilisation) below this SM clock = low-clock state.
LOW_CPU_FREQUENCY_PCT = 85  # Processor frequency below this % of maximum = low-frequency state.
MATCH_WINDOW_S = 1.5  # Samples are 1 s apart; a frame/stop is matched to throttle state within this distance.

SAMPLER_MARKER = '# visual-servoing acceptance-test system sampler'
SYSTEM_SCRIPT = SAMPLER_MARKER + r'''
$ErrorActionPreference = 'SilentlyContinue'
while ($true) {
  # Stop when the test that started this sampler is gone (backup to the job object).
  if (-not (Get-Process -Id __PARENT__ -ErrorAction SilentlyContinue)) { exit }
  $t = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
  foreach ($z in Get-CimInstance -ClassName Win32_PerfFormattedData_Counters_ThermalZoneInformation) {
    [Console]::Out.WriteLine("$t,zone,$($z.Name),$($z.Temperature),$($z.HighPrecisionTemperature),$($z.PercentPassiveLimit),$($z.ThrottleReasons)")
  }
  foreach ($p in Get-CimInstance -ClassName Win32_PerfFormattedData_Counters_ProcessorInformation -Filter "Name='_Total'") {
    [Console]::Out.WriteLine("$t,cpu,$($p.Name),$($p.ProcessorFrequency),$($p.PercentofMaximumFrequency),$($p.PercentProcessorPerformance),$($p.PercentProcessorUtility)")
  }
  [Console]::Out.Flush()
  Start-Sleep -Milliseconds __INTERVAL__
}
'''
SYSTEM_HEADER = ('# utc_ms,kind,name,a,b,c,d  zone: a=temperature_K b=high_precision_temperature_dK '
                 'c=passive_limit_pct d=throttle_reasons  cpu: a=frequency_MHz b=pct_of_max_frequency '
                 'c=pct_processor_performance d=pct_processor_utility')
ACPI_PROBE = ('$ErrorActionPreference="Stop"; try { Get-CimInstance -Namespace root/wmi -ClassName MSAcpi_ThermalZoneTemperature | '
              'ForEach-Object { "$($_.InstanceName),$($_.CurrentTemperature)" } } catch { "error," + $_.Exception.Message }')


def _flags():
    return subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0


class _KillOnExitJob:
    """Windows job object: every assigned process is killed when the test process ends,
    however it ends (closed window, Ctrl+Break, crash). Samplers can never outlive a run."""
    _instance = None

    def __init__(self):
        import ctypes
        from ctypes import wintypes
        self.ctypes = ctypes
        self.kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        self.kernel.CreateJobObjectW.restype = wintypes.HANDLE
        self.kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]

        class Basic(ctypes.Structure):
            _fields_ = [('PerProcessUserTimeLimit', ctypes.c_int64), ('PerJobUserTimeLimit', ctypes.c_int64),
                        ('LimitFlags', wintypes.DWORD), ('MinimumWorkingSetSize', ctypes.c_size_t),
                        ('MaximumWorkingSetSize', ctypes.c_size_t), ('ActiveProcessLimit', wintypes.DWORD),
                        ('Affinity', ctypes.c_size_t), ('PriorityClass', wintypes.DWORD), ('SchedulingClass', wintypes.DWORD)]

        class Io(ctypes.Structure):
            _fields_ = [(n, ctypes.c_uint64) for n in ('ReadOps', 'WriteOps', 'OtherOps', 'ReadBytes', 'WriteBytes', 'OtherBytes')]

        class Extended(ctypes.Structure):
            _fields_ = [('BasicLimitInformation', Basic), ('IoInfo', Io), ('ProcessMemoryLimit', ctypes.c_size_t),
                        ('JobMemoryLimit', ctypes.c_size_t), ('PeakProcessMemoryUsed', ctypes.c_size_t),
                        ('PeakJobMemoryUsed', ctypes.c_size_t)]
        self.handle = self.kernel.CreateJobObjectW(None, None)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        info = Extended(); info.BasicLimitInformation.LimitFlags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not self.kernel.SetInformationJobObject(self.handle, 9, ctypes.byref(info), ctypes.sizeof(info)):
            raise ctypes.WinError(ctypes.get_last_error())

    @classmethod
    def assign(cls, process):
        """Returns None on success, else the reason (recorded, never fatal)."""
        if os.name != 'nt':
            return 'not Windows'
        try:
            if cls._instance is None:
                cls._instance = cls()
            job = cls._instance
            if not job.kernel.AssignProcessToJobObject(job.handle, int(process._handle)):
                return repr(job.ctypes.WinError(job.ctypes.get_last_error()))
            return None
        except OSError as exc:
            return repr(exc)


def _powershell(script):
    encoded = base64.b64encode(script.encode('utf-16-le')).decode('ascii')
    return ['powershell.exe', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', encoded]


def uptime_s():
    """Seconds since the computer started (read-only; used to confirm a fresh restart)."""
    try:
        if os.name == 'nt':
            import ctypes
            get = ctypes.windll.kernel32.GetTickCount64
            get.restype = ctypes.c_uint64
            return get() / 1000
        return float(Path('/proc/uptime').read_text().split()[0])
    except (OSError, AttributeError, ValueError):
        return None


def utc_offset_s():
    return datetime.now().astimezone().utcoffset().total_seconds()


class GpuMonitor:
    """nvidia-smi at a fixed interval. Falls back to fewer fields if the driver rejects one."""
    def __init__(self, path, interval_ms, index=None):
        self.path, self.interval_ms, self.index = path, interval_ms, index
        self.process, self.log, self.fields, self.error, self.job_error = None, None, None, None, None

    def _launch(self, fields):
        command = ['nvidia-smi', '--query-gpu=' + fields, '--format=csv,nounits', f'--loop-ms={self.interval_ms}']
        if self.index is not None:
            command += ['-i', str(self.index)]
        self.log = self.path.open('w', encoding='utf-8')
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, creationflags=_flags())
        self.job_error = _KillOnExitJob.assign(self.process)
        self.fields = fields

    def start(self):
        try:
            self._launch(GPU_FIELDS)
            time.sleep(2.5)
            if self.process.poll() is not None:  # A field this driver does not support.
                self.log.close(); self.path.replace(self.path.with_suffix('.rejected.txt'))
                self._launch(GPU_FIELDS_MINIMAL)
        except OSError as exc:
            self.error = repr(exc); self.process = None
            if self.log:
                self.log.write(f'unavailable: {exc!r}\n')
        return self

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if self.log:
            self.log.close()

    def metadata(self):
        return dict(fields=self.fields, interval_ms=self.interval_ms, error=self.error, kill_on_exit_error=self.job_error,
                    pid=None if self.process is None else self.process.pid)


class SystemMonitor:
    """Windows thermal zones and processor frequency through one long-lived PowerShell."""
    def __init__(self, path, interval_ms):
        self.path, self.interval_ms, self.process, self.log, self.error = path, interval_ms, None, None, None
        self.job_error = None

    def start(self):
        if os.name != 'nt':
            self.error = 'not Windows'
            return self
        try:
            self.log = self.path.open('w', encoding='utf-8')
            self.log.write(SYSTEM_HEADER + '\n'); self.log.flush()
            script = (SYSTEM_SCRIPT.replace('__INTERVAL__', str(int(self.interval_ms)))
                      .replace('__PARENT__', str(os.getpid())))
            self.process = subprocess.Popen(_powershell(script), stdout=self.log, stderr=subprocess.STDOUT,
                                            stdin=subprocess.DEVNULL, creationflags=_flags())
            self.job_error = _KillOnExitJob.assign(self.process)
        except OSError as exc:
            self.error = repr(exc); self.process = None
        return self

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        if self.log:
            self.log.close()

    def metadata(self):
        return dict(interval_ms=self.interval_ms, error=self.error, kill_on_exit_error=self.job_error,
                    pid=None if self.process is None else self.process.pid,
                    source='Win32_PerfFormattedData_Counters_ThermalZoneInformation, '
                           'Win32_PerfFormattedData_Counters_ProcessorInformation(_Total)')


# First version of the sampler script (before the marker line), for cleaning up its leftovers.
LEGACY_SCRIPT_PREFIX = "\n$ErrorActionPreference = 'SilentlyContinue'\nwhile ($true) {\n  $t = [DateTimeOffset]::UtcNow"
LEFTOVER_QUERY = r'''
Get-CimInstance -ClassName Win32_Process -Filter "Name='nvidia-smi.exe' OR Name='powershell.exe'" |
  ForEach-Object { "$($_.ProcessId)`t$($_.ParentProcessId)`t$($_.Name)`t$($_.CommandLine)" }
'''
# Windows power-mode overlay GUIDs (Settings > System > Power & battery > Power mode).
POWER_MODES = {
    'ded574b5-45a0-4f42-8737-46345c09c238': 'Best performance',
    '00000000-0000-0000-0000-000000000000': 'Balanced',
    '961cc777-2547-4f9d-8174-7d86181b8a7a': 'Best power efficiency',
    '3af9b8d9-7c97-431d-ad78-34a8bfea439f': 'Better performance',
}


def power_mode_name(record):
    """Name from the recorded GUID (older runs stored a wrong label; the GUID is authoritative)."""
    if not record:
        return None
    guid = (record.get('effective_overlay') or record.get('ActiveOverlayAcPowerScheme') or '').lower()
    return POWER_MODES.get(guid, record.get('mode') if not guid else 'unknown')


def is_our_sampler(name, command_line):
    """Only processes this test starts: nvidia-smi with our query, or PowerShell running our script."""
    command_line = command_line or ''
    if name.lower() == 'nvidia-smi.exe':
        return '--query-gpu=timestamp,index,pstate' in command_line and '--loop-ms=' in command_line
    markers = [base64.b64encode(text.encode('utf-16-le')).decode('ascii')[:60] for text in (SAMPLER_MARKER, LEGACY_SCRIPT_PREFIX)]
    return name.lower() == 'powershell.exe' and any(m in command_line for m in markers)


def stop_leftover_samplers():
    """Find and stop samplers left by an earlier run that was killed (they would perturb timing)."""
    if os.name != 'nt':
        return dict(found=[], stopped=[], error='not Windows')
    try:
        run = subprocess.run(_powershell(LEFTOVER_QUERY), capture_output=True, text=True, timeout=60, creationflags=_flags())
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(found=[], stopped=[], error=repr(exc))
    found, stopped, errors = [], [], []
    for line in run.stdout.splitlines():
        parts = line.split('\t', 3)
        if len(parts) != 4 or not parts[0].isdigit():
            continue
        pid, _, name, command_line = int(parts[0]), parts[1], parts[2], parts[3]
        if pid == os.getpid() or not is_our_sampler(name, command_line):
            continue
        found.append(dict(pid=pid, name=name))
        kill = subprocess.run(['taskkill', '/PID', str(pid), '/F'], capture_output=True, text=True, creationflags=_flags())
        (stopped if kill.returncode == 0 else errors).append(pid)
    return dict(found=found, stopped=stopped, error=None if not errors else f'could not stop {errors}')


def power_mode():
    """Windows power mode (the Settings slider) and active power plan, read-only."""
    if os.name != 'nt':
        return None
    out = {}
    try:
        import ctypes, uuid
        guid = (ctypes.c_byte * 16)()
        result = ctypes.WinDLL('powrprof').PowerGetEffectiveOverlayScheme(ctypes.byref(guid))
        if result == 0:
            value = str(uuid.UUID(bytes_le=bytes(bytearray(guid))))
            out['effective_overlay'] = value; out['mode'] = POWER_MODES.get(value, 'unknown')
        else:
            out['effective_overlay_error'] = result
    except (OSError, AttributeError, ValueError) as exc:
        out['effective_overlay_error'] = repr(exc)
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'SYSTEM\CurrentControlSet\Control\Power\User\PowerSchemes') as key:
            for name in ('ActiveOverlayAcPowerScheme', 'ActivePowerScheme'):
                try:
                    out[name] = winreg.QueryValueEx(key, name)[0]
                except OSError:
                    pass
        if 'mode' not in out and 'ActiveOverlayAcPowerScheme' in out:
            out['mode'] = POWER_MODES.get(out['ActiveOverlayAcPowerScheme'].lower(), 'unknown')
    except OSError as exc:
        out['registry_error'] = repr(exc)
    return out


def snapshot(directory, label):
    """One-off readouts outside the measured sessions (driver limits, ACPI access)."""
    out = {}
    try:
        run = subprocess.run(['nvidia-smi', '-q', '-d', 'PERFORMANCE,POWER,TEMPERATURE,CLOCK'],
                             capture_output=True, text=True, timeout=30, creationflags=_flags())
        (directory / 'traces' / f'nvidia-smi-{label}.txt').write_text(run.stdout + run.stderr, encoding='utf-8')
        out['nvidia_smi_query'] = f'traces/nvidia-smi-{label}.txt'
    except (OSError, subprocess.TimeoutExpired) as exc:
        out['nvidia_smi_query'] = repr(exc)
    if os.name == 'nt':
        try:
            run = subprocess.run(_powershell(ACPI_PROBE), capture_output=True, text=True, timeout=60, creationflags=_flags())
            out['acpi_thermal_zone_temperature'] = run.stdout.strip()[-2000:] or run.stderr.strip()[-2000:]
        except (OSError, subprocess.TimeoutExpired) as exc:
            out['acpi_thermal_zone_temperature'] = repr(exc)
    return out


# ----------------------------------------------------------------------------- parsing

def decode(mask):
    return [name for bit, name in REASONS.items() if mask & bit]


def _number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def read_gpu(path, offset_s=None, created_utc=None):
    """nvidia-smi CSV (local timestamps) -> samples on the UTC epoch clock."""
    path = Path(path)
    if not path.exists():
        return [], None
    rows = list(csv.reader(path.read_text(encoding='utf-8', errors='replace').splitlines()))
    if not rows:
        return [], None
    header = [h.strip().split(' [')[0] for h in rows[0]]
    samples = []
    for row in rows[1:]:
        if len(row) != len(header) or row[0].strip().startswith('timestamp'):
            continue
        r = dict(zip(header, (c.strip() for c in row)))
        try:
            local = datetime.strptime(r['timestamp'], '%Y/%m/%d %H:%M:%S.%f')
        except (KeyError, ValueError):
            continue
        mask = r.get('clocks_event_reasons.active', '')
        try:
            mask = int(mask, 16)
        except ValueError:
            mask = None
        samples.append(dict(local=local, pstate=r.get('pstate'), sm_mhz=_number(r.get('clocks.current.sm') or r.get('clocks.sm')),
                            util=_number(r.get('utilization.gpu')), temp_c=_number(r.get('temperature.gpu')),
                            power_w=_number(r.get('power.draw')), power_limit_w=_number(r.get('enforced.power.limit')),
                            mask=mask, reasons=[] if mask is None else decode(mask)))
    if offset_s is None:
        # Older runs: nvidia-smi starts seconds after the manifest; round to the zone offset.
        if not samples or not created_utc:
            return samples, None
        created = datetime.fromisoformat(created_utc).replace(tzinfo=None)
        offset_s = round((samples[0]['local'] - created).total_seconds() / 900) * 900
    for s in samples:
        s['t'] = s['local'].replace(tzinfo=timezone.utc).timestamp() - offset_s
    return samples, offset_s


def read_system(path):
    zones, cpu = [], []
    path = Path(path)
    if not path.exists():
        return zones, cpu
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        parts = line.strip().split(',')
        if len(parts) != 7 or not parts[0].isdigit():
            continue
        t = int(parts[0]) / 1000
        a, b, c, d = (_number(x) for x in parts[3:])
        if parts[1] == 'zone':
            temp = (b / 10 - 273.15) if b else (a - 273.15 if a else None)
            zones.append(dict(t=t, name=parts[2], temp_c=temp, passive_limit_pct=c, throttle_reasons=d))
        elif parts[1] == 'cpu':
            cpu.append(dict(t=t, frequency_mhz=a, pct_max_frequency=b, pct_performance=c, pct_utility=d))
    return zones, cpu


# ----------------------------------------------------------------------------- correlation

def clock_anchor(ending, first_align_s):
    """Map controller perf_counter seconds to UTC epoch seconds."""
    anchor = ending.get('clock_anchor')
    if anchor:
        return anchor['utc_epoch_s'] - anchor['perf_s']
    if first_align_s is None or not ending.get('started_utc'):
        return None
    # Older runs: auto-armed first alignment coincides with ready; ready = start + (elapsed - duration).
    ready = datetime.fromisoformat(ending['started_utc']).timestamp() + ending['elapsed_s'] - ending['duration_s']
    return ready - first_align_s


def _nearest(samples, times, t):
    i = bisect_left(times, t)
    best = [samples[j] for j in (i - 1, i) if 0 <= j < len(samples)]
    best = [s for s in best if abs(s['t'] - t) <= MATCH_WINDOW_S]
    return best


def limiting(sample):
    return [r for r in sample['reasons'] if r in LIMITING]


def episodes(samples, start, end, origin=None, merge_gap_s=2.5):
    """Runs of clock-limited samples; runs separated by at most merge_gap_s are one episode."""
    origin = start if origin is None else origin
    out, current = [], None
    for s in samples:
        if not start <= s['t'] <= end:
            continue
        reasons = limiting(s)
        if reasons:
            if current is not None and s['t'] - current['end'] > merge_gap_s:
                out.append(current); current = None
            if current is None:
                current = dict(start=s['t'], end=s['t'], reasons=set(reasons), sm=[s['sm_mhz']], util=[s['util']])
            else:
                current['end'] = s['t']; current['reasons'] |= set(reasons)
                current['sm'].append(s['sm_mhz']); current['util'].append(s['util'])
        elif current is not None and s['t'] - current['end'] > merge_gap_s:
            out.append(current); current = None
    if current is not None:
        out.append(current)
    return [dict(start_s=round(e['start'] - origin, 1), duration_s=round(e['end'] - e['start'] + 1, 1),
                 reasons=sorted(e['reasons']), min_sm_mhz=min((x for x in e['sm'] if x is not None), default=None),
                 max_util=max((x for x in e['util'] if x is not None), default=None)) for e in out]


def gpu_low(sample):
    return (sample['util'] or 0) >= 20 and sample['sm_mhz'] is not None and sample['sm_mhz'] < LOW_GPU_CLOCK_MHZ


def cpu_low(sample):
    return sample['pct_max_frequency'] is not None and sample['pct_max_frequency'] < LOW_CPU_FREQUENCY_PCT


def low_state_episodes(gpu, cpu, start, end, origin, window_s=30):
    """Coarse 30 s windows where the majority of GPU-busy samples were low-clock or CPU samples low-frequency."""
    out, current = [], None
    t = start
    while t < end:
        g = [s for s in gpu if t <= s['t'] < t + window_s and (s['util'] or 0) >= 20]
        c = [s for s in cpu if t <= s['t'] < t + window_s]
        state = []
        if len(g) >= 5 and sum(gpu_low(s) for s in g) > len(g) / 2:
            state.append('gpu_low_clock')
        if c and sum(cpu_low(s) for s in c) > len(c) / 2:
            state.append('cpu_low_frequency')
        if state and current and current['end'] == t:
            current['end'] = t + window_s
            current['state'] = sorted(set(current['state']) | set(state))
        elif state:
            current = dict(start=t, end=t + window_s, state=state); out.append(current)
        t += window_s
    return [dict(start_s=round(e['start'] - origin), duration_s=round(e['end'] - e['start']), state=e['state']) for e in out]


def session_environment(session_frames, events, attempts, ending, gpu, zones, cpu):
    """Throttle and thermal state during one session, and its overlap with failures."""
    aligns = sorted(e['at_s'] for e in events if e['kind'] == 'align')
    shift = clock_anchor(ending, aligns[0] if aligns else None)
    if shift is None or not ending.get('started_utc'):
        return None
    start = datetime.fromisoformat(ending['started_utc']).timestamp()
    end = start + ending['elapsed_s']
    ready = end - ending['duration_s']  # First alignment is armed at ready.
    g = [s for s in gpu if start <= s['t'] <= end]
    busy = [s for s in g if (s['util'] or 0) >= 20 and s['sm_mhz'] is not None]
    reason_seconds = {r: sum(r in s['reasons'] for s in g) for r in LIMITING}
    times = [s['t'] for s in gpu]
    stops = {}
    for e in events:
        if e['kind'] == 'stop':
            stops.setdefault(e['generation'], e['at_s'])

    def throttled_at(t_perf):
        near = _nearest(gpu, times, t_perf + shift)
        return None if not near else any(limiting(s) for s in near)

    def gpu_low_at(t_perf):
        near = _nearest(gpu, times, t_perf + shift)
        return None if not near else any(gpu_low(s) for s in near)
    cpu_times = [s['t'] for s in cpu]

    def cpu_low_at(t_perf):
        near = _nearest(cpu, cpu_times, t_perf + shift)
        return None if not near else any(cpu_low(s) for s in near)
    failed = [a for a in attempts if a['outcome'] != 'converged' and a['generation'] in stops]
    fail_state = [throttled_at(stops[a['generation']]) for a in failed]
    fail_gpu_low = [gpu_low_at(stops[a['generation']]) for a in failed]
    fail_cpu_low = [cpu_low_at(stops[a['generation']]) for a in failed]
    slow = [f for f in session_frames if 1000 * (f['finished_s'] - f['rendered_s']) >= SLOW_FRAME_MS]
    slow_state = [throttled_at(f['rendered_s']) for f in slow]
    z = [s for s in zones if start <= s['t'] <= end]
    c = [s for s in cpu if start <= s['t'] <= end]
    hottest = {}
    for s in z:
        if s['temp_c'] is not None:
            hottest[s['name']] = max(hottest.get(s['name'], -1e9), s['temp_c'])
    passive = [s['passive_limit_pct'] for s in z if s['passive_limit_pct'] is not None]
    freq = sorted(s['pct_max_frequency'] for s in c if s['pct_max_frequency'] is not None)
    return dict(
        gpu_samples=len(g),
        gpu_busy_sm_mhz=None if not busy else dict(median=sorted(s['sm_mhz'] for s in busy)[len(busy) // 2],
                                                   min=min(s['sm_mhz'] for s in busy)),
        gpu_max_temp_c=max((s['temp_c'] for s in g if s['temp_c'] is not None), default=None),
        gpu_max_power_w=max((s['power_w'] for s in g if s['power_w'] is not None), default=None),
        limiting_reason_seconds=reason_seconds,
        throttle_episodes=episodes(gpu, start, end, origin=ready),
        failed_alignments=len(failed),
        failed_during_throttle=sum(x is True for x in fail_state),
        failed_without_gpu_sample=sum(x is None for x in fail_state),
        slow_frames=len(slow), slow_frames_during_throttle=sum(x is True for x in slow_state),
        system_samples=len(c),
        thermal_zone_max_c={k: round(v, 1) for k, v in sorted(hottest.items())},
        min_passive_limit_pct=min(passive, default=None),
        cpu_pct_max_frequency=None if not freq else dict(min=freq[0], median=freq[len(freq) // 2]),
        gpu_low_clock_seconds=sum(gpu_low(s) for s in g), gpu_busy_seconds=len(busy),
        cpu_low_frequency_samples=sum(cpu_low(s) for s in c),
        failed_during_gpu_low_clock=sum(x is True for x in fail_gpu_low),
        failed_during_cpu_low_frequency=sum(x is True for x in fail_cpu_low),
        low_state_episodes=low_state_episodes(gpu, cpu, start, end, ready),
    )
