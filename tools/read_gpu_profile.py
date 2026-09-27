"""Read the NVIDIA driver profile for one executable; never save or set policy.

ABI: NVIDIA/nvapi nvapi.h, nvapi_interface.h and NvApiDriverSettings.h.
Only documented read operations and temporary session allocation are resolved.
Run outside a timing campaign: even read-only driver initialization adds work.
"""
import argparse
import ctypes as C
import json
import os
from pathlib import Path

U32=C.c_uint32
Handle=C.c_void_p
Wide=C.c_wchar*2048

class Binary(C.Structure):
    _fields_=[('length',U32),('data',C.c_ubyte*4096)]

class Value(C.Union):
    _pack_=4
    _fields_=[('dword',U32),('binary',Binary),('wide',Wide),('qword',C.c_uint64)]

class Setting(C.Structure):
    _pack_=4
    _fields_=[('version',U32),('name',Wide),('id',U32),('type',U32),('location',U32),
              ('current_predefined',U32),('predefined_valid',U32),('predefined',Value),('current',Value)]

class Application(C.Structure):
    _fields_=[('version',U32),('predefined',U32),('name',Wide),('friendly',Wide),
              ('launcher',Wide),('file_in_folder',Wide),('flags',U32),('command_line',Wide)]

class Profile(C.Structure):
    _fields_=[('version',U32),('name',Wide),('gpu_support',U32),('predefined',U32),('apps',U32),('settings',U32)]

def versioned(cls,n=1):
    obj=cls();obj.version=C.sizeof(cls)|(n<<16);return obj

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executable',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    assert os.name=='nt' and C.sizeof(C.c_wchar)==2 and C.sizeof(Setting)==12320
    dll=C.CDLL(str(Path(os.environ['SystemRoot'])/'System32/nvapi64.dll'))
    query=dll.nvapi_QueryInterface;query.argtypes=[U32];query.restype=Handle
    def fn(ident,*types):
        address=query(ident)
        if not address:raise RuntimeError(f'NVAPI interface unavailable: {ident:#x}')
        return C.CFUNCTYPE(C.c_int,*types)(address)
    error=fn(0x6c2d048c,C.c_int,C.c_char_p)
    def message(status):
        buffer=C.create_string_buffer(64);error(status,buffer)
        return buffer.value.decode('ascii',errors='replace')
    def check(status):
        if status:raise RuntimeError(f'NVAPI {status}: {message(status)}')
    init=fn(0x0150e828);unload=fn(0xd22bdd7e)
    create=fn(0x0694d52e,C.POINTER(Handle));destroy=fn(0xdad9cff8,Handle)
    load=fn(0x375dbd6b,Handle)
    base=fn(0xda8466a0,Handle,C.POINTER(Handle))
    global_profile=fn(0x617bff9f,Handle,C.POINTER(Handle))
    find=fn(0xeee566b2,Handle,C.c_wchar_p,C.POINTER(Handle),C.POINTER(Application))
    info=fn(0x61cd6fd6,Handle,Handle,C.POINTER(Profile))
    get=fn(0x73bf8338,Handle,Handle,U32,C.POINTER(Setting))
    check(init());session=Handle()
    result={'read_only':True,'executable':str(args.executable.resolve()),'profiles':[]}
    try:
        check(create(C.byref(session)));check(load(session))
        handles=[]
        for name,call in [('base',base),('current_global',global_profile)]:
            handle=Handle();check(call(session,C.byref(handle)));handles.append((name,handle))
        app=versioned(Application,4);handle=Handle()
        status=find(session,str(args.executable.resolve()),C.byref(handle),C.byref(app))
        result['application_lookup']={'status':status,'message':message(status),'name':app.name}
        if status==0:handles.append(('application',handle))
        elif status!=-166:check(status)
        for label,handle in handles:
            profile=versioned(Profile);check(info(session,handle,C.byref(profile)))
            setting=versioned(Setting);status=get(session,handle,0x1057EB71,C.byref(setting))
            result['profiles'].append({'scope':label,'name':profile.name,
                'power_management':{'status':status,'message':message(status),'name':setting.name,
                 'type':setting.type if status==0 else None,'location':setting.location if status==0 else None,
                 'current':setting.current.dword if status==0 else None,
                 'predefined':setting.predefined.dword if status==0 and setting.predefined_valid else None,
                 'predefined_valid':bool(setting.predefined_valid) if status==0 else None}})
    finally:
        if session.value:check(destroy(session))
        check(unload())
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
