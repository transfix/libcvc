"""pycvc.ariadne.testing.ServerHarness (ARIADNE_SCRIPTING_HOSTS.md §6.5.8): a service's tests under
plain python, with no arisrvx and no network. A small service is defined here (its Host, an
authenticate hook that knows test tokens, exports), and the harness:
  * connects through the hook: a known token's principal; a refused token, one the hook raises on
    and one that runs past auth_timeout, each Refused; without a hook a client is its pid;
  * calls the exports as room-call would: ctx.principal and progress, a role gate (forbidden),
    CallError's code, message and detail, internal for any other exception, deadline_exceeded,
    not_found, values typed as the wire carries them (a tuple arrives as a list, a function does
    not cross), the idempotency key, call_concurrency's rate_limited;
  * reads what the relay would send: the published namespaces, bounded per principal by the
    read_scope hook (a refresh_scope widens it; a hook that raises refuses the connection);
  * closes a principal's clients on host.disconnect (unavailable);
  * with fastapi installed, puts the same service function behind an HTTP route, tested with
    FastAPI's TestClient beside the harness.
The tokens are test values, nothing else's.
"""

import asyncio
import sys

import pycvc

checks = 0


def check(cond, what):
    global checks
    if not cond:
        raise AssertionError(what)
    checks += 1


if not hasattr(pycvc, "ariadne"):
    print("SKIP test_pycvc_ariadne_harness: this pycvc ships no pycvc.ariadne")
    sys.exit(0)

from pycvc.ariadne import CallError, Host, export  # noqa: E402
from pycvc.ariadne.testing import Refused, ServerHarness  # noqa: E402

import pycvc.ariadne.testing as testing  # noqa: E402

check(testing.ServerHarness is ServerHarness and pycvc.ariadne.testing is testing,
      "pycvc.ariadne.testing is a module of its own name")

# -- the service ------------------------------------------------------------------------------------

host = Host()
TOKENS = {"staff-token-0123456789": {"id": "alice", "roles": ["staff"]},
          "user-token-0123456789": {"id": "bob", "roles": []},
          "eve-token-0123456789": {"id": "eve", "roles": []}}
PROJECTS = {"bob": {"acme"}}
host.set("svc.status", "up")
host.set("svc.public.motd", "hello")
host.set("svc.projects.acme.name", "Acme")
host.set("svc.projects.zeta.name", "Zeta")
host.set("private.note", "the server's own")


@host.authenticate
async def auth(cred):
    if cred.bearer == "slow-token-0123456789":
        await asyncio.sleep(5)
    if cred.bearer == "boom-token-0123456789":
        raise RuntimeError("the identity provider is down (on purpose)")
    return TOKENS.get(cred.bearer)


@host.read_scope
def scope(principal):
    if principal["id"] == "eve":
        raise LookupError("no projects for eve (on purpose)")
    if "staff" in principal["roles"]:
        return ["svc"]
    projects = sorted(PROJECTS.get(principal["id"], ()))
    return ["svc.public"] + [f"svc.projects.{p}" for p in projects]


def greeting(who: str) -> str:  # the service layer: the export and the HTTP route both call it
    return f"hello, {who}"


@export(clients=True)
async def whoami(ctx, note: str = ""):
    ctx.progress(0.5, "halfway")
    ctx.progress(1.0, "done")
    return {"id": ctx.principal["id"], "roles": ctx.principal["roles"], "note": note,
            "key": ctx.idempotency_key, "greeting": greeting(ctx.principal["id"])}


@export(clients=True)
def echo(ctx, value):
    return {"value": value, "kind": type(value).__name__}


@export(clients=True, roles=["staff"])
def staff_only(ctx):
    return "staff"


@export(clients=True)
def refuse(ctx):
    raise CallError("not_found", "no such project", detail={"project": "acme"})


@export(clients=True)
def explode(ctx):
    raise ValueError("a bug")


@export(clients=True, timeout=0.2)
async def slow(ctx):
    await asyncio.sleep(2)
    return "late"


@export(clients=True)
async def hold(ctx):
    await asyncio.sleep(0.2)
    return "held"


@export(clients=True)
def join_project(ctx, project: str):
    PROJECTS.setdefault(ctx.principal["id"], set()).add(project)
    host.refresh_scope(ctx.principal["id"])
    return sorted(PROJECTS[ctx.principal["id"]])


@export(clients=True)
def kick(ctx):
    host.disconnect(ctx.principal["id"])
    return "kicked"


@export
def internal():  # not for clients
    return "internal"


# -- the tests ----------------------------------------------------------------------------------------

harness = ServerHarness(host, publish=["svc"], call_concurrency=2, auth_timeout=0.5)

alice = harness.connect("staff-token-0123456789")
check(alice.principal == {"id": "alice", "roles": ["staff"]}, f"the hook's principal: {alice.principal}")
for token, why in (("nope-token-0123456789", "refused this connection"),
                   ("boom-token-0123456789", "could not judge"),
                   ("slow-token-0123456789", "could not judge"),
                   ("eve-token-0123456789", "read_scope")):
    try:
        harness.connect(token)
        check(False, f"{token} is refused")
    except Refused as e:
        check(str(e).startswith("unauthenticated") and why in str(e), f"{token}: {e}")

me = alice.call("whoami", note="n")
check(me["id"] == "alice" and me["roles"] == ["staff"] and me["note"] == "n", f"ctx.principal: {me}")
check(me["greeting"] == "hello, alice", "the service layer's answer")
check(me["key"].startswith(alice.pid), f"the idempotency key is the call id without one: {me['key']}")
check(alice.progress["whoami"] == [(0.5, "halfway"), (1.0, "done")],
      f"ctx.progress: {alice.progress['whoami']}")
check(alice.call_with("whoami", key="k-1")["key"] == "k-1", "the caller's idempotency key")

r = alice.call("echo", (1, 2.0, "s"))
check(r["value"] == [1, 2.0, "s"] and r["kind"] == "list", f"a tuple arrives as a list: {r}")
check(type(r["value"][0]) is int and type(r["value"][1]) is float, "an int stays an int, 2.0 a float")
check(alice.call("echo", b"\x00\xff")["kind"] == "bytes", "bytes stay bytes")
check(alice.call("echo", pycvc.symbol("a"))["kind"] == "symbol", "a symbol stays a symbol")
try:
    alice.call("echo", lambda x: x)
    check(False, "a function does not cross")
except TypeError:
    check(True, "a function does not cross: a TypeError where it is sent")

check(alice.call("staff_only") == "staff", "a role gate passes")
bob = harness.connect("user-token-0123456789")
for name, code in (("staff_only", "forbidden"), ("explode", "internal"), ("slow", "deadline_exceeded"),
                   ("internal", "not_found"), ("nothing_here", "not_found")):
    try:
        bob.call(name)
        check(False, f"{name} raises")
    except CallError as e:
        check(e.code == code, f"{name}: {e.code} {e.message}")
try:
    bob.call("refuse")
    check(False, "refuse raises")
except CallError as e:
    check((e.code, e.message, e.detail) == ("not_found", "no such project", {"project": "acme"}),
          f"CallError reaches the caller whole: {e.code} {e.message} {e.detail}")


async def three():
    return await asyncio.gather(bob.acall("hold"), bob.acall("hold"), bob.acall("hold"),
                                return_exceptions=True)


got = host.loop.run_until_complete(three())
codes = ["ok" if r == "held" else getattr(r, "code", repr(r)) for r in got]
check(codes == ["ok", "ok", "rate_limited"], f"call_concurrency 2 for one principal: {codes}")

check(alice.get("svc.status") == "up" and alice.get("svc.projects.zeta.name") == "Zeta",
      "staff read all of svc")
check(alice.get("private.note") is None and not alice.reads("private.note"), "private is unpublished")
check(host.get("private.note") == "the server's own", "the server's own tree has it")
check(alice.get_tree("private") is None, "nor its subtree")
check(bob.get("svc.public.motd") == "hello" and bob.get("svc.projects.acme.name") == "Acme",
      "bob reads svc.public and his project")
check(bob.get("svc.status") is None and bob.get("svc.projects.zeta.name") is None,
      "and nothing else of svc")
check(bob.get_tree("svc") == {"public": {"motd": "hello"}, "projects": {"acme": {"name": "Acme"}}},
      f"his tree: {bob.get_tree('svc')}")
check(bob.call("join_project", "zeta") == ["acme", "zeta"], "join_project")
check(bob.get("svc.projects.zeta.name") == "Zeta", "host.refresh_scope widens what he reads")

check(bob.call("kick") == "kicked", "kick")
check(not bob.connected and alice.connected, "host.disconnect closes that principal's clients")
try:
    bob.call("whoami")
    check(False, "a closed client's call fails")
except CallError as e:
    check(e.code == "unavailable", f"a closed client's call: {e.code}")

saved, host._authenticate = host._authenticate, None
anon = harness.connect(pid="p-7")
host._authenticate = saved
check(anon.principal == {"id": "p-7", "roles": []}, f"without a hook a client is its pid: {anon.principal}")

try:
    import fastapi  # noqa: F401
    from fastapi.testclient import TestClient
except (ImportError, RuntimeError) as e:  # starlette raises RuntimeError without its HTTP client
    print(f"note: no FastAPI TestClient here ({type(e).__name__}: {e}): the HTTP half is skipped")
else:
    app = fastapi.FastAPI()

    @app.get("/greeting/{who}")
    def http_greeting(who: str):
        return {"greeting": greeting(who)}

    with TestClient(app) as http:
        check(http.get("/greeting/carol").json() == {"greeting": "hello, carol"},
              "the same service function behind an HTTP route")

harness.close()
print(f"ok test_pycvc_ariadne_harness ({checks} checks)")
