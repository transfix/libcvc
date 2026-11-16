"""The program beside the relay for ctest arisrvx_rcon_admin_role (ARISRV_IDENTITY.md, ID3): an
export only an admin may call. A connection that rcon made an admin has the role admin in
ctx.principal (arisrv's main.cpp adds it to a copy of the principal), so it may; any other
connection, the same client's next one too, may not."""

from pycvc.ariadne import Host, export

host = Host()


@export(clients=True, roles=["admin"])
def admin_only(ctx):
    roles = list(ctx.principal["roles"])
    return {"admin": "admin" in roles, "roles": roles, "id": ctx.principal["id"]}


host.run()
