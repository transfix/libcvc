"""The program beside the relay for the read-scope ctests (arisrvx_read_scope*): an authenticate
hook that knows two test tokens, and a read_scope hook (ARIADNE_SCRIPTING_HOSTS.md 6.5.6) under
which staff read all of svc and anyone else reads svc.public and the projects they belong to. Calls
change a caller's projects, which re-scopes its connections (host.refresh_scope), and rename a
project, which the relay sends only to those who read it. The tokens are test values, nothing
else's."""

from pycvc.ariadne import Host, export

host = Host()

PRINCIPALS = {
    "staff-token-0123456789": {"id": "alice", "roles": ["staff"]},
    "user-token-0123456789": {"id": "bob", "roles": []},
}
PROJECTS = {"bob": {"acme"}}

host.set("svc.public.motd", "hello")
host.set("svc.projects.acme.name", "Acme")
host.set("svc.projects.zeta.name", "Zeta")


@host.authenticate
def auth(cred):
    return PRINCIPALS.get(cred.bearer)


@host.read_scope
def scope(principal):
    if "staff" in principal["roles"]:
        return ["svc"]
    projects = sorted(PROJECTS.get(principal["id"], ()))
    return ["svc.public"] + [f"svc.projects.{p}" for p in projects]


@export(clients=True)
def join_project(ctx, project: str):
    PROJECTS.setdefault(ctx.principal["id"], set()).add(project)
    host.refresh_scope(ctx.principal["id"])
    return ",".join(sorted(PROJECTS[ctx.principal["id"]]))


@export(clients=True)
def leave_project(ctx, project: str):
    PROJECTS.setdefault(ctx.principal["id"], set()).discard(project)
    host.refresh_scope(ctx.principal["id"])
    return ",".join(sorted(PROJECTS[ctx.principal["id"]]))


@export(clients=True)
def rename(ctx, project: str, name: str):
    host.set(f"svc.projects.{project}.name", name)
    return name


host.run()
