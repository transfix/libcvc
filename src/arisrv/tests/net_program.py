"""host.net in arisrvx (ctest arisrvx_net; ARIADNE_NET_ENGINE.md 2.7, N2): the relay's own
connection engine, through pycvc from this build. A listener on loopback greets each connection
from on_accept and joins it to a group; a client on the same loop sends two lines, which arrive in
on_frames as memoryviews; a broadcast reaches it; its close reaches on_close; then the program
stops the host, which stops the relay with status 0."""

import asyncio

from pycvc.ariadne import Host
from pycvc.ariadne.net import framing

host = Host()
net = host.net
hub = net.listen(address="127.0.0.1", port=0, frames=framing.delimited(b"|"), name="nmdc")
heard, closed = [], asyncio.Event()


@hub.on_accept
def accepted(conn, info):
    net.join(conn, "all")
    net.send(conn, b"$Lock hi|")


@hub.on_frames
def frames(batch):
    for conn, lines in batch:
        heard.extend(bytes(f) for f in lines)
    if len(heard) == 2:
        net.broadcast("all", b"<hub> welcome|")


@hub.on_close
def gone(conn, reason):
    print(f"net_program: closed {reason}", flush=True)
    closed.set()


async def main():
    reader, writer = await asyncio.open_connection("127.0.0.1", hub.port)
    lock = await reader.readuntil(b"|")
    writer.write(b"$Key k|$ValidateNick zed|")
    welcome = await asyncio.wait_for(reader.readuntil(b"|"), 5)
    writer.close()
    await asyncio.wait_for(closed.wait(), 5)
    threads = len(net.stats().threads)
    ok = (lock == b"$Lock hi|" and welcome == b"<hub> welcome|"
          and heard == [b"$Key k|", b"$ValidateNick zed|"])
    print(f"net_program: {'ok' if ok else 'FAILED'}: {lock!r} {heard!r} {welcome!r}, "
          f"{threads} I/O threads", flush=True)
    host.stop(0 if ok else 1)


main_task = host.loop.create_task(main())  # kept: the loop holds a task only weakly
host.run()
