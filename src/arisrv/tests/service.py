"""A service for arisrvx's end-to-end ctest (arisrvx_serve, src/arisrv/CMakeLists.txt).

Run as ``arisrvx --python service.py --doc service_server.ari``: a FastAPI app on the script
thread's loop, beside a server document that publishes ``svc``. No client joins; the program drives
itself and exits 0 when every step held:

1. three requests, each writing ``svc.hits``, and the relay applying the last of them;
2. a 40-row ``set_tree`` the server document counts on a room step and writes back;
3. a stop raised in-process as Ctrl-C would be (the relay's handler, not Python's), with a request
   in flight that finishes before the app's shutdown runs.

``arisrvx check service.py`` runs the same file without serving (arisrvx_check).
"""

import asyncio
import contextlib
import json
import signal

from fastapi import FastAPI
from pycvc.ariadne import Host, serve_asgi

host = Host()


@contextlib.asynccontextmanager
async def lifespan(app):
    host.set("svc.status", "up")
    print("service: startup", flush=True)
    yield
    host.set("svc.status", "down")
    print("service: shutdown", flush=True)


app = FastAPI(lifespan=lifespan)
hits = 0


@app.get("/hello/{name}")
async def hello(name: str):
    global hits
    hits += 1
    host.set("svc.hits", hits)
    return {"hello": name, "hits": hits}


@app.get("/slow")
async def slow():
    await asyncio.sleep(0.4)
    print("service: /slow finished", flush=True)
    return {"slow": True}


async def get(port, path):
    """A plain HTTP/1.1 GET on loopback: (status, JSON body)."""
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    writer.write(f"GET {path} HTTP/1.1\r\nHost: test\r\nConnection: close\r\n\r\n".encode())
    await writer.drain()
    data = await reader.read()
    writer.close()
    head, _, body = data.partition(b"\r\n\r\n")
    return int(head.split()[1]), json.loads(body)


async def until(what, pred, timeout=15.0):
    deadline = host.loop.time() + timeout
    while not pred():
        if host.loop.time() > deadline:
            raise TimeoutError(f"waiting for {what}")
        await asyncio.sleep(0.02)


async def selftest():
    try:
        await until("the server to listen", lambda: host.listening())
        port = host.listening()[0][1]
        for i in range(3):
            status, body = await get(port, f"/hello/n{i}")
            assert status == 200 and body["hits"] == i + 1, (status, body)
        print("selftest: 3 requests served", flush=True)
        await until("svc.hits", lambda: host.get("svc.hits") == "3")
        print("selftest: the relay applied svc.hits = 3", flush=True)

        host.set_tree("svc.items", [{"id": i, "name": f"item{i}"} for i in range(40)])
        host.set("svc.selftest", "writes-queued")
        await until("the document's count", lambda: host.get("svc.doc_saw") == "40")
        assert host.get_tree("svc.items")[39] == {"id": "39", "name": "item39"}
        print("selftest: the server document read the 40 items", flush=True)

        asyncio.ensure_future(get(port, "/slow"))
        await asyncio.sleep(0.1)  # the request is in flight
        signal.raise_signal(signal.SIGINT)
    except Exception as e:  # noqa: BLE001 - the test's verdict
        print(f"selftest: FAILED: {e!r}", flush=True)
        host.stop(1)


host.loop.create_task(selftest())
serve_asgi(app, host="127.0.0.1", port=0)
print(f"service: host.run() returned {host.run()}", flush=True)
