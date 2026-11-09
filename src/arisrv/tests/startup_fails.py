"""An app whose lifespan startup fails (ctest arisrvx_startup_fails): arisrvx exits 1 with the
traceback, and the relay never listens."""

import contextlib

from fastapi import FastAPI
from pycvc.ariadne import serve_asgi


@contextlib.asynccontextmanager
async def lifespan(app):
    raise RuntimeError("the database is not there (on purpose)")
    yield


app = FastAPI(lifespan=lifespan)
serve_asgi(app, host="127.0.0.1", port=0)
