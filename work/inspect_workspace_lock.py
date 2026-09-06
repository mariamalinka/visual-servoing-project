"""Read-only, targeted check of candidate processes' Windows current directory."""
import ctypes as c
import json
import struct

kernel = c.WinDLL("kernel32", use_last_error=True)
ntdll = c.WinDLL("ntdll")
kernel.OpenProcess.argtypes = [c.c_ulong, c.c_int, c.c_ulong]
kernel.OpenProcess.restype = c.c_void_p
kernel.ReadProcessMemory.argtypes = [c.c_void_p, c.c_void_p, c.c_void_p, c.c_size_t, c.c_void_p]
kernel.ReadProcessMemory.restype = c.c_int
kernel.CloseHandle.argtypes = [c.c_void_p]
ntdll.NtQueryInformationProcess.argtypes = [c.c_void_p, c.c_ulong, c.c_void_p, c.c_ulong, c.c_void_p]
ntdll.NtQueryInformationProcess.restype = c.c_long

class BasicInfo(c.Structure):
    _fields_ = [("reserved1", c.c_void_p), ("peb", c.c_void_p),
                ("reserved2", c.c_void_p * 2), ("pid", c.c_size_t), ("reserved3", c.c_void_p)]

root = r"C:\Users\marys\Documents\Codex\2026-09-06\i-want-to-do-that-project".lower()
for pid in [26224, 32532, 53964, 17948, 51384, 52652, 30284, 34532]:
    handle = kernel.OpenProcess(0x410, False, pid)
    if not handle:
        print(json.dumps({"pid": pid, "result": "process inaccessible"}))
        continue
    try:
        def read(address, size):
            buffer = c.create_string_buffer(size)
            if not kernel.ReadProcessMemory(handle, address, buffer, size, None):
                raise OSError(c.get_last_error())
            return buffer.raw
        info = BasicInfo()
        status = ntdll.NtQueryInformationProcess(handle, 0, c.byref(info), c.sizeof(info), None)
        if status:
            raise OSError(status)
        parameters = struct.unpack("<Q", read(info.peb + 0x20, 8))[0]
        length, maximum, padding, address = struct.unpack("<HHIQ", read(parameters + 0x38, 16))
        directory = read(address, length).decode("utf-16-le")
        if directory.lower().rstrip("\\") == root or directory.lower().startswith(root + "\\"):
            print(json.dumps({"pid": pid, "workspace_cwd": directory}))
        else:
            print(json.dumps({"pid": pid, "result": "outside this workspace"}))
    except OSError as exc:
        print(json.dumps({"pid": pid, "result": "current directory inaccessible", "error": str(exc)}))
    finally:
        kernel.CloseHandle(handle)
