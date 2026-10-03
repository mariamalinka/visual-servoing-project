# Reproducer for the defect found by the fault-injection campaign (see docs/FAULT_INJECTION.md):
# a sender killed in the middle of a multiprocessing.Queue message leaves a partial message,
# and the reader's get_nowait() then blocks forever. Run on Linux (cloud workspace);
# Windows pipes may behave differently.
import multiprocessing as mp, time, threading, numpy as np
def child(q):
    big = np.zeros(50_000_000, dtype=np.uint8)  # 50 MB: sending takes a while while parent is not reading
    q.put(big)
    time.sleep(10)
if __name__ == '__main__':
    ctx = mp.get_context('spawn'); q = ctx.Queue(1)
    p = ctx.Process(target=child, args=(q,)); p.start()
    time.sleep(3.0)  # child is blocked mid-write (pipe buffer full)
    p.kill(); p.join()
    result = {}
    def reader():
        try:
            q.get_nowait(); result['r'] = 'got'
        except Exception as e:
            result['r'] = repr(e)
    t = threading.Thread(target=reader, daemon=True); t.start(); t.join(3)
    print('reader alive after 3 s (blocked):', t.is_alive(), result)
