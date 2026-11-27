"""host.net in arisrvx (ctest arisrvx_net; ARIADNE_NET_ENGINE.md 2.7, N2): the relay's own
connection engine, through pycvc from this build. A listener on loopback greets each connection
from on_accept and joins it to a group; a client on the same loop sends two lines, which arrive in
on_frames as memoryviews; a broadcast reaches it; its close reaches on_close; then the program
stops the host, which stops the relay with status 0. Where the engine has TLS (N3), a second
listener takes TLS and plain on one port (tests/tls's certificate, detect_tls): an asyncio TLS
client is greeted over TLS from on_accept, whose conn_info says so."""

import asyncio
import os
import ssl

from pycvc.ariadne import Host
from pycvc.ariadne.net import framing, tls_options

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


tls_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tls")
hubs, tls_infos = None, []
if net.tls_available():
    hubs = net.listen(address="127.0.0.1", port=0, frames=framing.delimited(b"|"), name="nmdcs",
                      detect_tls=300,
                      tls=tls_options(certificate_file=os.path.join(tls_dir, "localhost.pem"),
                                      key_file=os.path.join(tls_dir, "localhost.key")))

    @hubs.on_accept
    def tls_accepted(conn, info):
        tls_infos.append(info)
        net.send(conn, b"$Lock tls|")


async def tls_part():
    if hubs is None:
        print("net_program: tls skipped (no TLS in this build)", flush=True)
        return True
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    reader, writer = await asyncio.open_connection("127.0.0.1", hubs.port, ssl=ctx,
                                                   server_hostname="localhost")
    lock = await asyncio.wait_for(reader.readuntil(b"|"), 5)
    writer.close()
    ok = (lock == b"$Lock tls|" and len(tls_infos) == 1 and tls_infos[0].tls()
          and tls_infos[0].tls_sni == "localhost" and hubs.keyprint.startswith("SHA256/"))
    print(f"net_program: tls {'ok' if ok else 'FAILED'}: {lock!r} "
          f"{tls_infos[0].tls_version if tls_infos else None} {hubs.keyprint}", flush=True)
    return ok


async def main():
    reader, writer = await asyncio.open_connection("127.0.0.1", hub.port)
    lock = await reader.readuntil(b"|")
    writer.write(b"$Key k|$ValidateNick zed|")
    welcome = await asyncio.wait_for(reader.readuntil(b"|"), 5)
    tls_ok = await tls_part()
    writer.close()
    await asyncio.wait_for(closed.wait(), 5)
    threads = len(net.stats().threads)
    ok = (lock == b"$Lock hi|" and welcome == b"<hub> welcome|"
          and heard == [b"$Key k|", b"$ValidateNick zed|"] and tls_ok)
    print(f"net_program: {'ok' if ok else 'FAILED'}: {lock!r} {heard!r} {welcome!r}, "
          f"{threads} I/O threads", flush=True)
    host.stop(0 if ok else 1)


main_task = host.loop.create_task(main())  # kept: the loop holds a task only weakly
host.run()
