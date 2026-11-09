"""A program for --http-listen (ctest arisrvx_http_listen): it defines `app` and never calls
serve_asgi, so `arisrvx --python http_listen.py --http-listen 127.0.0.1:0` serves it. It never calls
host.run() either: the host runs the loop. Its selftest requests /ping, then stops the host itself,
which stops the relay with status 0."""

import asyncio
import json

from fastapi import FastAPI
from pycvc.ariadne import current_host

app = FastAPI()


@app.get("/ping")
async def ping():
    return {"pong": True}


async def selftest():
    host = current_host()
    while not host.listening():
        await asyncio.sleep(0.02)
    port = host.listening()[0][1]
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    writer.write(b"GET /ping HTTP/1.1\r\nHost: test\r\nConnection: close\r\n\r\n")
    await writer.drain()
    head, _, body = (await reader.read()).partition(b"\r\n\r\n")
    writer.close()
    status = int(head.split()[1])
    print(f"selftest: GET /ping {status} {json.loads(body)}", flush=True)
    host.stop(0 if status == 200 else 1)


current_host().loop.create_task(selftest())
