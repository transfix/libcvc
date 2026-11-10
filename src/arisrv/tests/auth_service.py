"""The program beside the relay for the authenticate ctests (arisrvx_auth_*): an authenticate hook
(ARIADNE_SCRIPTING_HOSTS.md 6.5.4) that knows two test tokens, and the functions the clients call
as the principals it names. One token stalls past --auth-timeout and one makes the hook raise; any
other is refused, and so is a principal once revoked. The tokens are test values, nothing else's."""

import asyncio

from pycvc.ariadne import Host, export

host = Host()

PRINCIPALS = {
    "staff-token-0123456789": {"id": "alice", "name": "Alice", "roles": ["staff"]},
    "user-token-0123456789": {"id": "bob", "roles": []},
}
REVOKED = set()
AUTHENTICATED = {}  # principal id -> the times the hook admitted it (a reconnect is another)


@host.authenticate
async def auth(cred):
    if cred.bearer == "slow-token-0123456789":
        await asyncio.sleep(10)  # past --auth-timeout: refused
    if cred.bearer == "boom-token-0123456789":
        raise RuntimeError("the identity provider is down (on purpose)")
    principal = PRINCIPALS.get(cred.bearer)
    if principal is None or principal["id"] in REVOKED:
        return None
    AUTHENTICATED[principal["id"]] = AUTHENTICATED.get(principal["id"], 0) + 1
    return principal


@export(clients=True)
def whoami(ctx):
    return ctx.principal


@export(clients=True)
def admitted(ctx):
    return AUTHENTICATED.get(ctx.principal["id"], 0)


@export(clients=True, roles=["staff"])
def staff_only(ctx):
    return "staff"


@export(clients=True)
async def hold(ctx):
    await asyncio.sleep(0.5)
    return "held"


@export(clients=True)
def kick(ctx):
    # After the reply has gone: every connection of the caller's principal closes. Its token still
    # admits it, so a client that dials again is back.
    host.loop.call_later(0.3, host.disconnect, ctx.principal["id"])
    return "kicked"


@export(clients=True)
def revoke(ctx):
    # The same, after which the hook refuses the principal: it stays out.
    REVOKED.add(ctx.principal["id"])
    host.loop.call_later(0.3, host.disconnect, ctx.principal["id"])
    return "revoked"


host.run()
