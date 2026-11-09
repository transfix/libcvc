"""pycvc.ariadne under plain python: the server program's API with no relay (ARIADNE_SCRIPTING_HOSTS.md
§6.5.2). arisrvx runs the same module built in; here Host keeps a tree of its own, so a service's
code runs and is tested without arisrvx:
  * pycvc imports it as pycvc.ariadne, and current_host() is None until a Host is made;
  * set / set_tree / set_series / get_tree on Host's own tree, set_tree as (state-set-tree) writes;
  * serve_asgi + run(): a FastAPI app served on host.loop, a request, and stop() from the loop,
    with the app's shutdown run; skipped where fastapi or uvicorn is not installed.
"""

import asyncio
import contextlib
import json
import sys

import pycvc

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


if not hasattr(pycvc, "ariadne"):
    print("SKIP test_pycvc_ariadne_server: this pycvc ships no pycvc.ariadne")
    sys.exit(0)

from pycvc.ariadne import Host, current_host, serve_asgi  # noqa: E402

check(sys.modules["pycvc.ariadne"] is pycvc.ariadne, "pycvc.ariadne is pycvc's attribute")
check(current_host() is None, "no Host until the program makes one")

host = Host()
check(current_host() is host, "current_host() is the program's Host")
check(host.mode == "standalone", f"plain python is standalone, not {host.mode!r}")
try:
    Host()
    check(False, "a second Host raises")
except RuntimeError:
    check(True, "a second Host raises")

host.set("svc.status", "up")
host.set("/svc.count", 3)
host.set("svc.flag", True)
check(host.get("svc.status") == "up", "set then get")
check(host.get_int("svc.count") == 3 and host.get("svc.flag") == "1", "numbers and bools as text")
check(host.get("svc.none", "dflt") == "dflt", "a missing key's default")

host.set_tree("svc.jobs", {"a": 1, "b": {"x": 2}, "ring": [10, 20, 30]})
check(host.get_tree("svc.jobs") == {"a": "1", "b": {"x": "2"}, "ring": ["10", "20", "30"]},
      f"set_tree round trip: {host.get_tree('svc.jobs')}")
host.set_tree("svc.jobs", {"b": "done", "ring": [5]})
check(host.get_tree("svc.jobs") == {"b": "done", "ring": ["5"]},
      f"set_tree removes what it does not name: {host.get_tree('svc.jobs')}")
host.set_series("svc.s", [1.5, 2, 3], limit=2)
check(host.get_series("svc.s") == [2.0, 3.0], f"set_series keeps the last: {host.get('svc.s')}")

try:
    import fastapi
    import uvicorn  # noqa: F401
except ImportError as e:
    print(f"  skip: the ASGI part ({e})")
    print(f"ok test_pycvc_ariadne_server ({checks} checks)")
    sys.exit(0)

events = []


@contextlib.asynccontextmanager
async def lifespan(app):
    events.append("startup")
    yield
    events.append("shutdown")
    host.set("svc.status", "down")


app = fastapi.FastAPI(lifespan=lifespan)


@app.get("/ping")
async def ping():
    return {"pong": True}


async def client():
    while not host.listening():
        await asyncio.sleep(0.01)
    port = host.listening()[0][1]
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    writer.write(b"GET /ping HTTP/1.1\r\nHost: t\r\nConnection: close\r\n\r\n")
    await writer.drain()
    head, _, body = (await reader.read()).partition(b"\r\n\r\n")
    writer.close()
    events.append((int(head.split()[1]), json.loads(body)))
    host.stop(7)


server = serve_asgi(app, host="127.0.0.1", port=0)
check(host.servers == [server], "serve_asgi returns its uvicorn server")
host.loop.create_task(client())
status = host.run()
check(status == 7, f"run() returns stop()'s code, not {status}")
check(events == ["startup", (200, {"pong": True}), "shutdown"], f"served and shut down: {events}")
check(host.get("svc.status") == "down", "the app's shutdown wrote the tree")
print(f"ok test_pycvc_ariadne_server ({checks} checks)")
