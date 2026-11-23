"""Fan-out from Python through pycvc's net_engine, against N1's C++ numbers
(docs/roadmap/ARIADNE_NET_ENGINE.md 4; net_engine_test's NetEngineFanOut).

    python3 bench_pycvc_net.py [--receivers 6000] [--io-threads 2] [--rounds 21]

Over loopback, as N1's benchmark: N receivers on one engine listener, all in group "all", a
113-byte chat line broadcast to them. It prints, as medians over the rounds:

  * out, burst 1 and 10: the broadcast() call from Python; the I/O threads' busy time a receiver
    (net_stats.threads' busy_ns), the number to compare with N1's (17.6 us at 2 threads on
    prettyhatemachine, where a bare write() is 16.5 us); and the wall time until every receiver
    has the line (the readers here are Python threads, so it is slower than N1's);
  * baseline: the same line written by a Python loop of sock.send() to N other loopback
    sockets, the CPU a receiver of the program's own thread, which is what a hub on one asyncio
    loop pays today (sublevels' soak: about 35 us a connection with asyncio's transports);
  * in: every receiver sends one line; the time until all of them reached on_frames on an asyncio
    loop, and the loop thread's CPU a frame (the waker's batches, memoryviews and the callback).

Not a ctest: it needs room for 4N descriptors and takes about a minute.
"""

import argparse
import asyncio
import resource
import selectors
import socket
import statistics
import threading
import time

import pycvc

ap = argparse.ArgumentParser()
ap.add_argument("--receivers", type=int, default=6000)
ap.add_argument("--io-threads", type=int, default=2)
ap.add_argument("--rounds", type=int, default=21)
ap.add_argument("--readers", type=int, default=2)
args = ap.parse_args()
n = args.receivers
soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
want = 4 * n + 256
if soft < want:
    resource.setrlimit(resource.RLIMIT_NOFILE, (min(want, hard), hard))

MSG = b"<sublevels> " + b"m" * 100 + b"|"
eng = pycvc.net_engine(pycvc.engine_options(io_threads=args.io_threads))
lst = eng.listen(address="127.0.0.1", port=0, frames=pycvc.framing.delimited(b"|"))


def busy_ns():
    return sum(t.busy_ns for t in eng.stats().threads)


# -- connect N receivers, and let the engine accept them -------------------------------------------
socks = []
accepted = []
batch = pycvc.net_batch()
for i in range(n):
    s = socket.create_connection(("127.0.0.1", lst.port))
    s.setblocking(False)
    socks.append(s)
    if i % 256 == 0 and eng.poll(batch):
        accepted += [e.conn for e in batch.events if e.kind == pycvc.net_event.type_accepted]
deadline = time.monotonic() + 60
while len(accepted) < n and time.monotonic() < deadline:
    if eng.wait(batch, 100):
        accepted += [e.conn for e in batch.events if e.kind == pycvc.net_event.type_accepted]
assert len(accepted) == n, f"accepted {len(accepted)} of {n}"
for c in accepted:
    eng.join(c, "all")


# -- readers: Python threads on epoll, each a share of the sockets ----------------------------------
class Readers:
    def __init__(self, sockets, threads):
        self.parts = [sockets[i::threads] for i in range(threads)]
        self.want = 0
        self.go = [threading.Event() for _ in range(threads)]
        self.done = threading.Semaphore(0)
        self.bad = 0
        self.stop = False
        self.threads = [threading.Thread(target=self.run, args=(i,), daemon=True)
                        for i in range(threads)]
        for t in self.threads:
            t.start()

    def run(self, i):
        sel = selectors.DefaultSelector()
        for s in self.parts[i]:
            sel.register(s, selectors.EVENT_READ)
        while True:
            self.go[i].wait()
            self.go[i].clear()
            if self.stop:
                return
            left = {s.fileno(): self.want for s in self.parts[i]}
            while left:
                for key, _ in sel.select(5):
                    fd = key.fd
                    if fd not in left:
                        continue
                    data = key.fileobj.recv(65536)
                    left[fd] -= len(data)
                    if left[fd] <= 0:
                        if left[fd] < 0:
                            self.bad += 1
                        del left[fd]
            self.done.release()

    def arm(self, nbytes):
        self.want = nbytes
        for g in self.go:
            g.set()

    def wait(self):
        for _ in self.threads:
            self.done.acquire()


readers = Readers(socks, args.readers)
med = statistics.median


def measure(burst):
    call_us, per_rx, wall_ms = [], [], []
    for r in range(args.rounds + 1):  # round 0 warms up
        readers.arm(len(MSG) * burst)
        time.sleep(0.005)
        b0 = busy_ns()
        t0 = time.perf_counter()
        for _ in range(burst):
            eng.broadcast("all", MSG)
        t1 = time.perf_counter()
        readers.wait()
        t2 = time.perf_counter()
        time.sleep(0.005)
        if r == 0:
            continue
        busy = (busy_ns() - b0) / 1000.0
        call_us.append((t1 - t0) * 1e6 / burst)
        per_rx.append(busy / n / burst)
        wall_ms.append((t2 - t0) * 1000)
    print(f"out burst={burst:<2}  broadcast() call {med(call_us):6.1f} us   I/O busy "
          f"{med(per_rx):6.2f} us a receiver a message   all received {med(wall_ms):6.1f} ms")
    return med(per_rx)


print(f"receivers={n} io_threads={eng.io_threads()} message={len(MSG)} B rounds={args.rounds} "
      f"python readers={args.readers}")
per1 = measure(1)
measure(10)
assert readers.bad == 0

# -- baseline: a Python loop of send() to N other loopback sockets --------------------------------
srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
srv.listen(4096)
mine, theirs = [], []
for i in range(n):
    c = socket.create_connection(srv.getsockname())
    a, _ = srv.accept()
    a.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    a.setblocking(False)
    c.setblocking(False)
    mine.append(a)
    theirs.append(c)
base_readers = Readers(theirs, args.readers)
loop_us = []
for r in range(args.rounds + 1):
    base_readers.arm(len(MSG))
    time.sleep(0.005)
    c0 = time.thread_time()
    for a in mine:
        a.send(MSG)
    c1 = time.thread_time()
    base_readers.wait()
    if r:
        loop_us.append((c1 - c0) * 1e6 / n)
print(f"baseline  a Python loop of sock.send(): {med(loop_us):6.2f} us of the program's CPU a "
      f"receiver (the engine's broadcast(): {med(loop_us) * n / 1000:.1f} ms -> one call)")
base_readers.stop = True
for g in base_readers.go:
    g.set()
for s in mine + theirs:
    s.close()
srv.close()

# -- in: every receiver sends a line; on_frames on an asyncio loop ---------------------------------


async def inbound(rounds=5):
    loop = asyncio.get_running_loop()
    eng.attach_loop(loop)
    got = [0]
    cpu = [0.0]
    calls = [0]
    all_in = asyncio.Event()

    @lst.on_frames
    def frames(b):
        c0 = time.thread_time()
        calls[0] += 1
        for _, fs in b:
            for f in fs:
                if f[0] == 60:  # b"<"
                    got[0] += 1
        cpu[0] += time.thread_time() - c0
        if got[0] >= n:
            all_in.set()

    wall, per_frame, batches = [], [], []
    line = b"<nick> hello hub|"
    for _ in range(rounds):
        got[0], calls[0] = 0, 0
        all_in.clear()
        t0 = time.perf_counter()
        for s in socks:
            s.send(line)
        c_sent = time.thread_time()  # the sends ran on this thread too: count from here
        await asyncio.wait_for(all_in.wait(), 30)
        t1 = time.perf_counter()
        c_loop = time.thread_time() - c_sent
        wall.append((t1 - t0) * 1000)
        per_frame.append(c_loop * 1e6 / n)
        batches.append(calls[0])
    print(f"in        {n} lines, one a receiver: all in on_frames {med(wall):6.1f} ms after the "
          f"first send; the loop thread's CPU (waker hand-off, memoryviews, callback) "
          f"{med(per_frame):5.2f} us a frame, in {med(batches):.0f} on_frames call(s)")
    eng.detach_loop()


asyncio.run(inbound())
readers.stop = True
for g in readers.go:
    g.set()
print(f"N1 C++ reference (prettyhatemachine, 2 threads): 17.6 us a receiver; bare write() 16.5 us."
      f" Here: {per1:.2f} us.")
for s in socks:
    s.close()
eng.stop()
