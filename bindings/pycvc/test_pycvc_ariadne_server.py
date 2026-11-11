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

# The capsules arisrvx's calls ride on (value_capsule / value_from_capsule): a value converts as
# every pycvc value does, so it keeps its type both ways.
sent = {"i": 3, "f": 3.0, "s": "x", "sym": pycvc.symbol("a"), "b": b"\x00\x01", "n": None,
        "t": True, "l": [1, [2.5, "y"]]}
back = pycvc.value_from_capsule(pycvc.value_capsule(sent))
check(back == sent, f"a value round-trips through a capsule: {back!r}")
check(type(back["i"]) is int and type(back["f"]) is float, "an int stays an int, a float a float")
check(isinstance(back["sym"], pycvc.symbol) and type(back["b"]) is bytes, "symbols and bytes stay")
try:
    pycvc.value_capsule(object())
    check(False, "a value no DSL value is raises")
except TypeError:
    check(True, "a value no DSL value is raises")
try:
    pycvc.value_from_capsule("not a capsule")
    check(False, "value_from_capsule wants a capsule")
except (RuntimeError, ValueError, TypeError):
    check(True, "value_from_capsule wants a capsule")

check(sys.modules["pycvc.ariadne"] is pycvc.ariadne, "pycvc.ariadne is pycvc's attribute")
check(current_host() is None, "no Host until the program makes one")

host = Host()
check(current_host() is host, "current_host() is the program's Host")

# host.authenticate (S5) keeps the hook; under plain python nothing calls it, and disconnect does
# nothing.
def hook(cred):
    return {"id": "x"}


check(host.authenticate(hook) is hook and host._authenticate is hook, "authenticate keeps its hook")
host.disconnect("x")
check("bearer=<set>" in repr(pycvc.ariadne.Credential("secret", "grpc", "p")) and
      "secret" not in repr(pycvc.ariadne.Credential("secret", "grpc", "p")),
      "a credential's repr never shows its token")

# The per-principal call limits: calls in flight (--call-concurrency) and a second (--call-rate).
prog = pycvc.ariadne._Program.__new__(pycvc.ariadne._Program)
prog.call_concurrency, prog.call_rate, prog.in_flight, prog.buckets = 2, 0.0, {"alice": 2}, {}
check("in flight" in prog.limited("alice"), "alice has 2 in flight already")
check(prog.limited("bob") == "", "bob has none")
prog.call_concurrency, prog.call_rate = 0, 2.0
check(prog.limited("carol") == "" and prog.limited("carol") == "", "a burst of 2 a second")
check("a second" in prog.limited("carol"), "a third at once is over the rate")
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
