"""The program beside the relay for the room-call ctests (arisrvx_room_call_*): the functions
call_client.ari calls (ARIADNE_SCRIPTING_HOSTS.md 6.5.5). Their arguments arrive as the DSL sent
them, through pycvc, so echo reports each value's Python type; bye stops the relay once the client
is done."""

import asyncio
import time

from pycvc.ariadne import CallError, Host, export

host = Host()


@export(clients=True)
def echo(ctx, value, label="plain"):
    return {"value": value, "kind": type(value).__name__, "label": label,
            "caller": ctx.principal["id"], "key": ctx.idempotency_key, "call": ctx.call_id}


@export(clients=True, timeout=10)
async def work(ctx, steps):
    for i in range(steps):
        ctx.progress((i + 1) / steps, f"step {i + 1}")
        await asyncio.sleep(0.02)
    return steps


@export(clients=True, roles=["staff"])
def staff_only(ctx):
    return "never reached"


@export(clients=True)
def refuse(ctx):
    raise CallError("conflict", "refused on purpose", detail={"why": 7})


@export(clients=True)
def explode(ctx):
    raise RuntimeError("a detail the client must not see")


@export(clients=True, timeout=0.2)
async def slow(ctx):
    await asyncio.sleep(5)


@export(clients=True)
def blocking(ctx):
    time.sleep(0.12)  # over --script-slow-ms 50: the relay logs it by name
    return "done"


@export(clients=True)
def big(ctx):
    return "x" * 4000  # over --call-result-max 1024


@export  # the server document's, not the clients'
def internal():
    return 1


@export(clients=True)
def bye(ctx):
    host.loop.call_later(0.5, host.stop, 0)
    return "bye"


host.run()
