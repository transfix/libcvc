"""The program beside the relay for ctests arisrvx_guest_password_hook{,_refuses} (ARISRV_IDENTITY.md
2.4, ID4c): with an authenticate hook the hook decides who joins, so a relay that takes OIDC and a
guest password checks the password there, with host.guest_password_ok. The relay compares the
bearer itself; the program never holds the password."""

from pycvc.ariadne import Host, export

host = Host()


@host.authenticate
def auth(cred):
    # An OIDC bearer would be verified first; the guest password is the other way in.
    if host.guest_password_ok(cred):
        return {"id": "guest", "roles": ["guest"]}
    return None


@export(clients=True)
def me(ctx):
    return {"id": ctx.principal["id"], "roles": list(ctx.principal["roles"]),
            "other_bearer": host.guest_password_ok("not the bearer")}


host.run()
