import os
import sys
from pathlib import Path
import threading
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_scheduling import SchedulingLease,WindowsScheduling


class FakeScheduling:
    def __init__(self):
        self.masks=(4,4);self.priority=0;self.priority_calls=[]
    def get_qos(self,scope):return self.masks
    def set_qos(self,scope,masks):self.masks=masks
    def get_priority(self):return self.priority
    def set_priority(self,value):self.priority_calls.append(value);self.priority=value


class SchedulingTests(unittest.TestCase):
    def test_preserves_unrelated_policy_and_restores_on_shutdown(self):
        api=FakeScheduling();lease=SchedulingLease('control',api=api)
        self.assertEqual(api.masks,(5,4))
        self.assertEqual(api.priority,1)
        lease.close();lease.close()
        self.assertEqual(api.masks,(4,4));self.assertEqual(api.priority,0)
        self.assertEqual(api.priority_calls,[1,0])

    def test_sensor_priority_is_not_raised(self):
        api=FakeScheduling();lease=SchedulingLease('sensor',api=api)
        self.assertTrue(lease.metadata['qos_applied'])
        self.assertFalse(api.priority_calls)
        lease.close();self.assertFalse(api.priority_calls)

    def test_denied_policy_stays_visible_and_does_not_abort_safety_owner(self):
        api=FakeScheduling()
        def denied(*args):raise PermissionError('test policy denied')
        api.set_qos=denied
        lease=SchedulingLease('control',api=api)
        self.assertFalse(lease.metadata['qos_applied'])
        self.assertEqual(lease.metadata['errors'][0]['stage'],'high_qos')
        lease.close();self.assertEqual(api.priority,0)

    def test_cannot_restore_another_threads_settings(self):
        api=FakeScheduling();lease=SchedulingLease('control',api=api);errors=[]
        def attempt():
            try:lease.close()
            except RuntimeError as exc:errors.append(str(exc))
        thread=threading.Thread(target=attempt);thread.start();thread.join()
        self.assertEqual(len(errors),1);self.assertFalse(lease.closed)
        lease.close()

    @unittest.skipUnless(os.name=='nt','Windows API integration')
    def test_real_owned_thread_qos_and_priority_round_trip(self):
        result={}
        def run():
            try:
                api=WindowsScheduling();before=(api.get_qos('Thread'),api.get_priority())
                lease=SchedulingLease('control')
                result.update(metadata=lease.metadata,before=before,during=(api.get_qos('Thread'),api.get_priority()))
                lease.close();result['after']=(api.get_qos('Thread'),api.get_priority())
            except BaseException as exc:result['error']=repr(exc)
        thread=threading.Thread(target=run);thread.start();thread.join()
        self.assertNotIn('error',result)
        self.assertEqual(result['metadata']['errors'],[])
        self.assertEqual(result['before'],result['after'])
        self.assertTrue(result['during'][0][0]&1)
        self.assertFalse(result['during'][0][1]&1)
        self.assertGreaterEqual(result['during'][1],1)


if __name__=='__main__':unittest.main()
