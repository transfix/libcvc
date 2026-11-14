"""The service of the end-to-end ctest arisrvx_echo_service (ARIADNE_SCRIPTING_HOSTS.md 6.5.8): one
program with each part an application service has. An HTTP route (FastAPI on uvicorn) and an
export clients call, both over the same service function; an authenticate hook that admits one
fixed test token; the echo.* subtree its server document (tests/echo_service.ari) publishes; and a
read_scope hook under which a client reads echo.public but not echo.private. tests/echo_client.ari
joins, calls the export, reads what it wrote, fails to read the private half and fetches the route.
The token is a test value, nothing else's."""

import asyncio

from fastapi import FastAPI

from pycvc.ariadne import Host, export, serve_asgi

host = Host()
TOKEN = "echo-token-0123456789"

host.set("echo.public.status", "up")
host.set("echo.private.secret", "the service's own")


@host.authenticate
def auth(cred):
    return {"id": "tester", "roles": ["tester"]} if cred.bearer == TOKEN else None


@host.read_scope
def scope(principal):
    return ["echo.public"]


def echoed(text: str) -> dict:  # the service layer, under the route and the export alike
    return {"text": text, "length": len(text)}


app = FastAPI()


@app.get("/echo")
def http_echo(text: str = ""):
    return echoed(text)


@export(clients=True, roles=["tester"])
async def echo(ctx, text: str):
    ctx.progress(0.5, "echoing")
    host.set("echo.public.last", text)
    host.set("echo.public.by", ctx.principal["id"])
    await asyncio.sleep(0)
    ctx.progress(1.0, "echoed")
    return {**echoed(text), "by": ctx.principal["id"]}


@export(clients=True)
def http_url(ctx):
    h, p = host.listening()[0]
    return f"http://{h}:{p}"


serve_asgi(app, host="127.0.0.1", port=0)
host.run()
