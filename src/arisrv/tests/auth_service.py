"""The program beside the relay for the authenticate ctests (arisrvx_auth_*): an authenticate hook
(ARIADNE_SCRIPTING_HOSTS.md 6.5.4) that knows two test tokens, and the functions the clients call
as the principals it names. One token stalls past --auth-timeout and one makes the hook raise; any
other is refused. The tokens are test values, nothing else's."""

import asyncio

from pycvc.ariadne import Host, export

host = Host()

PRINCIPALS = {
    "staff-token-0123456789": {"id": "alice", "name": "Alice", "roles": ["staff"]},
    "user-token-0123456789": {"id": "bob", "roles": []},
}


@host.authenticate
async def auth(cred):
    if cred.bearer == "slow-token-0123456789":
        await asyncio.sleep(10)  # past --auth-timeout: refused
    if cred.bearer == "boom-token-0123456789":
        raise RuntimeError("the identity provider is down (on purpose)")
    return PRINCIPALS.get(cred.bearer)


@export(clients=True)
def whoami(ctx):
    return ctx.principal


@export(clients=True, roles=["staff"])
def staff_only(ctx):
    return "staff"


@export(clients=True)
async def hold(ctx):
    await asyncio.sleep(0.5)
    return "held"


@export(clients=True)
def kick(ctx):
    # After the reply has gone: every connection of the caller's principal closes.
    host.loop.call_later(0.3, host.disconnect, ctx.principal["id"])
    return "kicked"


host.run()
