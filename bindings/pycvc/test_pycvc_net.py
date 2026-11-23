"""pycvc's cvc::net, the connection engine (docs/roadmap/ARIADNE_NET_ENGINE.md 2.7, N2), over
loopback:
  * the C++ API as pycvc wraps it: the option structs (keywords, values), framing.delimited(b"|"),
    listen() and its OSError, wait()/poll() batches whose frames are memoryviews into the batch's
    buffers (still valid after the batch is cleared), send / send_many / broadcast / join / leave
    / close / set_framing with pause_after / want_writable, the close reasons, stats();
  * the asyncio waker: on_accept (and its refusal, sync and async), on_frames (one call a batch a
    listener, each connection once), on_writable, on_close (reason, and message with a third
    parameter), callbacks on the loop's thread only;
  * pycvc.ariadne: pycvc.ariadne.net is pycvc's own names, and Host.net under plain python is an
    engine of the Host's own, attached to host.loop and stopped (its closes delivered) at shutdown.
"""

import asyncio
import gc
import socket
import sys
import threading
import time

import pycvc

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


if not hasattr(pycvc, "net_engine"):
    print("SKIP test_pycvc_net: this pycvc has no net_engine")
    sys.exit(0)
if not pycvc.net_engine.available():
    print("SKIP test_pycvc_net: this build cannot listen")
    sys.exit(0)

from pycvc import (engine_options, framing, inbound_limits, listener_options,  # noqa: E402
                   net_batch, net_engine, net_event)


def connect(port):
    s = socket.create_connection(("127.0.0.1", port), timeout=5)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    return s


def recv_exactly(s, n):
    out = b""
    while len(out) < n:
        chunk = s.recv(n - len(out))
        if not chunk:
            break
        out += chunk
    return out


def collect(engine, want, timeout=5.0):
    """Events from wait() until want(events) holds; the batches stay alive for their frames."""
    events, batches = [], []
    deadline = time.monotonic() + timeout
    while not want(events) and time.monotonic() < deadline:
        b = net_batch()
        if engine.wait(b, 100):
            events.extend(b.events)
            batches.append(b)
    check(want(events), f"the events came: {[(e.kind, e.conn) for e in events]}")
    return events


def of(events, kind):
    return [e for e in events if e.kind == kind]


# -- the option structs ------------------------------------------------------------------------

f = framing.delimited(b"|")
check(f.kind == framing.mode_delimiter and f.delimiter == ord("|"), "delimited(b'|')")
check(framing.delimited(10).delimiter == 10 and framing.delimited("\n").delimiter == 10,
      "delimited takes an int or a one-character str")
check(framing.length_prefixed(2).kind == framing.mode_length
      and framing.length_prefixed(2).prefix == 2, "length_prefixed")
check(framing.raw(5).raw_bytes == 5 and framing.raw().kind == framing.mode_raw, "raw")
try:
    framing.delimited(b"ab")
    check(False, "a two-byte delimiter is refused")
except TypeError:
    check(True, "a two-byte delimiter is refused")
lo = listener_options(address="127.0.0.1", port=0, name="t", frames=f, max_frame=64,
                      limits=inbound_limits(window=500, first_frame_timeout=0))
check(lo.address == "127.0.0.1" and lo.name == "t" and lo.max_frame == 64, "keyword members")
check(lo.frames.delimiter == ord("|") and lo.limits.window == 500, "nested values")
check(lo.backlog == 1024 and lo.max_queued == 4 * 1024 * 1024 and lo.flush_timeout == 10000,
      "the C++ defaults")
try:
    listener_options(nope=1)
    check(False, "an unknown member is refused")
except TypeError:
    check(True, "an unknown member is refused")
p = framing.delimited(b"|")
p.pause_after = ["$ADCSND"]
check(list(p.pause_after) == ["$ADCSND"], "pause_after takes a list")
check(pycvc.close_reason_name(pycvc.close_reason_slow_reader) == "slow_reader", "reason names")

# -- the C++ API: wait() batches ---------------------------------------------------------------

e = net_engine(engine_options(io_threads=2))
check(e.io_threads() == 2, "io_threads")
l1 = e.listen(lo)
check(int(l1) > 0 and l1.port > 0 and e.port(l1) == l1.port, "listen, port")
try:
    e.listen(address="127.0.0.1", port=l1.port)
    check(False, "a taken port raises")
except OSError as exc:
    check(str(l1.port) in str(exc), f"a taken port raises OSError: {exc}")

c1 = connect(l1.port)
c1.sendall(b"$Key abc|$ValidateNick zed|")
ev = collect(e, lambda evs: len([f for x in of(evs, net_event.type_frames) for f in x.frames]) >= 2)
acc = of(ev, net_event.type_accepted)
check(len(acc) == 1 and acc[0].info.peer_address == "127.0.0.1" and acc[0].info.local_port == l1.port,
      "accepted, with its conn_info")
conn = acc[0].conn
check(acc[0].listener == int(l1) and acc[0].info.id == conn, "the event's listener and id")
frames = [f for x in of(ev, net_event.type_frames) for f in x.frames]
check(all(isinstance(f, memoryview) and f.readonly for f in frames), "frames are memoryviews")
check([bytes(f) for f in frames] == [b"$Key abc|", b"$ValidateNick zed|"],
      "delimited frames include the delimiter")
kept = frames[1]
del ev, acc, frames
gc.collect()
check(bytes(kept) == b"$ValidateNick zed|" and bytes(kept[1:9]) == b"Validate",
      "a kept memoryview (and its slice) outlives its batch")

e.send(conn, b"$Lock x|")
e.send(conn, bytearray(b"$Hello zed|"))
e.send(conn, "$Unicode é|")  # a str goes as UTF-8
e.send(conn, memoryview(b"..|")[0:3])
want = b"$Lock x|$Hello zed|$Unicode \xc3\xa9|..|"
check(recv_exactly(c1, len(want)) == want, "send: bytes, bytearray, str, memoryview, in order")

c2 = connect(l1.port)
ev = collect(e, lambda evs: len(of(evs, net_event.type_accepted)) == 1)
conn2 = of(ev, net_event.type_accepted)[0].conn
check(conn2 != conn, "a new id a connection")
e.join(conn, "all")
e.join(conn2, "all")
e.broadcast("all", b"<hub> hi|")
check(recv_exactly(c1, 9) == b"<hub> hi|" and recv_exactly(c2, 9) == b"<hub> hi|", "broadcast")
e.broadcast("all", b"<hub> not you|", conn)
check(recv_exactly(c2, 14) == b"<hub> not you|", "broadcast with exclude")
e.leave(conn2, "all")
e.broadcast("all", b"<hub> one|")
e.send_many([conn, conn2], b"<both>|")
check(recv_exactly(c1, 17) == b"<hub> one|<both>|", "leave; send_many")
check(recv_exactly(c2, 7) == b"<both>|", "send_many reaches a connection outside the group")

# set_framing with pause_after: a switch to length-prefixed lands right after $Switch.
p = framing.delimited(b"|")
p.pause_after = ["$Switch"]
e.set_framing(conn, p)
c1.sendall(b"$Switch|\x00\x03abc\x00\x01z")
ev = collect(e, lambda evs: len([f for x in of(evs, net_event.type_frames) for f in x.frames]) >= 1)
check([bytes(f) for x in of(ev, net_event.type_frames) for f in x.frames] == [b"$Switch|"],
      "pause_after: nothing split past the frame")
e.set_framing(conn, framing.length_prefixed(2))
ev = collect(e, lambda evs: len([f for x in of(evs, net_event.type_frames) for f in x.frames]) >= 2)
check([bytes(f) for x in of(ev, net_event.type_frames) for f in x.frames] == [b"abc", b"z"],
      "set_framing applies from the first byte not split")

e.want_writable(conn, 0)
ev = collect(e, lambda evs: len(of(evs, net_event.type_writable)) == 1)
check(of(ev, net_event.type_writable)[0].conn == conn, "want_writable")

# Close reasons: the program's (flushed), the peer's, a frame past max_frame.
e.send(conn, b"bye|")
e.close(conn, True)
check(recv_exactly(c1, 4) == b"bye|" and c1.recv(1) == b"", "a flushing close sends, then ends")
c2.close()
c3 = connect(l1.port)
c3.sendall(b"x" * 100)
ev = collect(e, lambda evs: len(of(evs, net_event.type_closed)) == 3)
reasons = {x.conn: pycvc.close_reason_name(x.reason) for x in of(ev, net_event.type_closed)}
check(reasons[conn] == "program" and reasons[conn2] == "peer_closed"
      and "frame_too_long" in reasons.values(), f"close reasons: {reasons}")
c1.close()
c3.close()

st = e.stats()
check(st.engine.accepted == 3 and st.engine.open == 0, "stats: accepted, open")
check(st.engine.closed["program"] == 1 and st.engine.closed["frame_too_long"] == 1
      and st.engine.closed["peer_closed"] == 1, f"stats: closed {st.engine.closed}")
check(st.engine.broadcasts == 3 and st.engine.frames_in >= 5 and st.engine.bytes_out > 0,
      "stats: broadcasts, frames, bytes")
check(len(st.listeners) == 1 and st.listeners[0].name == "t" and st.listeners[0].port == l1.port
      and st.listeners[0].counters.accepted == 3, "stats: the listener's")
check(len(st.threads) == 2 and sum(t.handlers for t in st.threads) > 0, "stats: the threads'")
# Members read as copies: from a temporary net_stats, after other allocations, they hold.
threads = e.stats().threads
listeners = e.stats().listeners
counters = e.stats().engine
gc.collect()
junk = [net_engine(engine_options(io_threads=1)) for _ in range(2)]
for j in junk:
    j.stop()
check(sum(t.handlers for t in threads) <= sum(t.handlers for t in e.stats().threads)
      and all(t.busy_ns < 10**13 for t in threads), "stats().threads outlives its net_stats")
check(listeners[0].name == "t" and listeners[0].counters.accepted == 3 and counters.accepted == 3,
      "stats().listeners and .engine outlive their net_stats")
check(listener_options(frames=framing.delimited(b"|")).frames.delimiter == ord("|"),
      "a nested member read from a temporary")
b = net_batch()
check(e.poll(b) is False and len(b) == 0, "poll with nothing queued")

# set_notify: an I/O thread calls it when the queue goes non-empty.
woke = threading.Event()
e.set_notify(woke.set)
c4 = connect(l1.port)
check(woke.wait(5), "set_notify's callable is called")
e.set_notify(None)
c4.close()
l1.stop()
e.stop()
del e

# -- the asyncio waker -------------------------------------------------------------------------


async def waker_test():
    loop = asyncio.get_running_loop()
    eng = net_engine()
    eng.attach_loop(loop)
    hub = eng.listen(address="127.0.0.1", port=0, frames=framing.delimited(b"|"), name="hub")
    seen = {"accept": [], "frames": [], "close": [], "writable": [], "threads": set()}
    done = asyncio.Event()

    @hub.on_accept
    def accepted(c, info):
        seen["threads"].add(threading.get_ident())
        seen["accept"].append((c, info.peer_port))
        if len(seen["accept"]) == 3:
            return "banned"  # the third is refused
        eng.join(c, "all")
        eng.send(c, b"$Lock x|")
        return None

    @hub.on_frames
    def frames(batch):
        seen["threads"].add(threading.get_ident())
        conns = [c for c, _ in batch]
        check(len(conns) == len(set(conns)), "on_frames: each connection once a batch")
        for c, fs in batch:
            check(all(isinstance(f, memoryview) for f in fs), "on_frames: memoryviews")
            seen["frames"].append((c, [bytes(f) for f in fs]))
        if sum(len(fs) for _, fs in seen["frames"]) >= 4:
            done.set()

    @hub.on_writable
    def writable(c):
        seen["writable"].append(c)

    @hub.on_close
    def closed(c, reason, message):
        seen["threads"].add(threading.get_ident())
        seen["close"].append((c, reason))

    def blocking_client():
        a = connect(hub.port)
        bb = connect(hub.port)
        lock_a = recv_exactly(a, 8)
        lock_b = recv_exactly(bb, 8)
        a.sendall(b"one|two|")
        bb.sendall(b"three|four|")
        return a, bb, lock_a, lock_b

    a, bb, lock_a, lock_b = await asyncio.to_thread(blocking_client)
    check(lock_a == b"$Lock x|" and lock_b == b"$Lock x|", "on_accept's sends")
    await asyncio.wait_for(done.wait(), 5)
    got = {}
    for c, fs in seen["frames"]:
        got.setdefault(c, []).extend(fs)
    check(sorted(got.values()) == [[b"one|", b"two|"], [b"three|", b"four|"]],
          f"on_frames: every frame, in order: {got}")

    # A refused connection is closed at once: its client reads the end.
    def refused_client():
        with connect(hub.port) as r:
            r.sendall(b"ignored|")
            r.settimeout(5)
            return r.recv(1)

    check(await asyncio.to_thread(refused_client) == b"", "on_accept's refusal closes it")
    conns = [c for c, _ in seen["accept"]]

    # Async callbacks run as tasks; an async refusal closes once it returns.
    @hub.on_accept
    async def accepted_later(c, info):
        await asyncio.sleep(0.01)
        return "later"

    check(await asyncio.to_thread(refused_client) == b"", "an async on_accept's refusal")

    eng.want_writable(conns[0], 0)
    eng.broadcast("all", b"<hub> bye|")

    def read_bye():
        return recv_exactly(a, 10), recv_exactly(bb, 10)

    check(await asyncio.to_thread(read_bye) == (b"<hub> bye|", b"<hub> bye|"),
          "broadcast from the loop")
    a.close()
    bb.close()
    deadline = loop.time() + 5
    while len(seen["close"]) < 4 and loop.time() < deadline:
        await asyncio.sleep(0.01)
    reasons = dict(seen["close"])
    check(reasons.get(conns[0]) == "peer_closed" and reasons.get(conns[2]) == "program",
          f"on_close reasons: {seen['close']}")
    check(seen["writable"] == [conns[0]], "on_writable")
    check(seen["threads"] == {threading.get_ident()}, "callbacks run on the loop's thread")
    eng.stop()
    eng.detach_loop()


asyncio.run(waker_test())

# -- pycvc.ariadne: pycvc.ariadne.net and Host.net ---------------------------------------------

if hasattr(pycvc, "ariadne"):
    import pycvc.ariadne.net as ariadne_net
    from pycvc.ariadne import Host

    check(ariadne_net.net_engine is pycvc.net_engine and ariadne_net.framing is pycvc.framing,
          "pycvc.ariadne.net is pycvc's own names")
    from pycvc.ariadne.net import close_reason_peer_closed  # noqa: F401

    host = Host(net_threads=1)
    net = host.net
    check(isinstance(net, pycvc.net_engine) and host.net is net and net.io_threads() == 1,
          "Host.net: the Host's engine")
    lst = net.listen(address="127.0.0.1", port=0, frames=framing.delimited(b"\n"))
    heard, closes = [], []

    @lst.on_frames
    def heard_frames(batch):
        heard.extend(bytes(f) for _, fs in batch for f in fs)
        if heard:
            host.stop(0)

    @lst.on_close
    def heard_close(c, reason):
        closes.append(reason)

    def line_client():
        s = connect(lst.port)
        s.sendall(b"hello\n")
        return s

    s = None

    async def start():
        global s
        s = await asyncio.to_thread(line_client)

    host.loop.run_until_complete(start())
    check(host.run() == 0, "the Host ran until stopped")
    check(heard == [b"hello\n"], "Host.net's callbacks ran on host.loop")
    check(closes == ["engine_stopped"], f"the Host stopped its engine at shutdown: {closes}")
    check(s.recv(1) == b"", "its connections are closed")
    s.close()

print(f"ok test_pycvc_net ({checks} checks)")
