"""pycvc.ariadne: Python services beside the Ariadne relay (ARIADNE_SCRIPTING_HOSTS.md 6.5).

``arisrvx --python main.py`` runs a Python program next to the relay, on a script thread of its own
with one asyncio event loop. An ASGI app (FastAPI on uvicorn) is served on that loop, and so, later,
are the functions clients call. The relay never runs Python and never waits on it::

    from pycvc.ariadne import Host, serve_asgi
    from myservice.api import app          # a FastAPI app

    host = Host()                          # the relay's tree, and the script thread's loop
    serve_asgi(app, host="127.0.0.1", port=8780)
    host.run()                             # until SIGTERM; then the ASGI shutdown, then the relay

``Host`` (one per program; ``current_host()`` returns it):

* ``host.loop`` is the script thread's asyncio loop. Under plain ``python`` it is the running loop,
  or a new one that ``run()`` runs.
* ``host.set(key, value)``, ``set_tree`` and ``set_series`` write the relay's tree, as
  ``pycvc_gl.ariadne.Host``'s do. A key is a path from the tree's root (``cyprov.status``; a leading
  ``/`` is allowed too). In arisrvx a write is queued, and the relay applies it on its next room step
  (60 Hz): coalesced per path, at most ``--script-write-budget`` bytes a step, the rest on the
  steps after in order. So ``get`` sees a write a step later. Clients receive the namespaces the
  server document publishes (``server: publish:``).
* ``host.get``, ``get_float``, ``get_int``, ``get_bool``, ``get_series`` and ``get_tree`` read it.
* ``host.run()`` runs the loop until SIGTERM or SIGINT (the host owns signals) or ``host.stop()``.
  Then every ASGI server stops accepting, finishes its requests within ``--shutdown-timeout`` (10 s)
  and runs its app's shutdown, and the loop's other tasks are cancelled. It returns the exit
  status. When the program returns without calling it, the host runs the loop itself.

``export`` makes a function callable: by the server document's programs as ``(name a b ...)``, and,
with ``clients=True``, by clients with ``(room-call "name" ...)`` (ARIADNE_SCRIPTING_HOSTS.md
6.5.5)::

    from pycvc.ariadne import export, CallError

    @export(clients=True, roles=["staff"], timeout=600)
    async def instance_create(ctx, project: str, spec: dict):
        job = await service.create(ctx.principal, project, spec, key=ctx.idempotency_key)
        async for step in job.steps():
            ctx.progress(step.fraction, step.message)   # lands in the caller's tree
        return job.result

* A client's function takes ``ctx`` first: ``principal`` (``{"id": <the caller's pid>, "roles":
  [...]}``; S5's authenticate hook fills in who it is), ``call_id``, ``idempotency_key`` (the
  caller's, or the call id), ``deadline`` (a ``time.time()``) and ``progress(fraction, message)``.
* ``roles=`` is checked before the function runs: a caller with none of them gets ``forbidden``.
* ``raise CallError(code, message, detail=...)`` reaches the caller with those; any other exception
  is ``internal`` to it, its traceback logged here.
* The call is cancelled at ``timeout`` (the export's, or the caller's if shorter; 30 s when neither
  says), and the caller gets ``deadline_exceeded``. A result over ``--call-result-max`` is
  ``too_large``.
* The values are the DSL's own, converted by pycvc (``pycvc.value_from_capsule``): an int arrives as
  an int, a float as a float, and the result goes back the same way. So the functions need a pycvc
  beside arisrvx; without one, calls are answered ``unavailable``.
* An ``async def`` runs on the loop; a plain function runs inline on it, and one that takes longer
  than ``--script-slow-ms`` (50 ms) is logged by name, since it held up every other caller.
  ``is_async=True`` runs a plain function with ``asyncio.to_thread`` instead.
* A server document's ``python:`` block runs on this thread too, as ARIADNE_SCRIPTING_HOSTS.md 4.2
  describes for the client hosts, with this ``export`` predefined in its source.

``serve_asgi(app, host=, port=, **config)`` serves an ASGI app with uvicorn on ``host.loop``; several
are allowed. Their startup (an app's lifespan startup, the bind) runs before the relay accepts
clients, and one that fails stops arisrvx, non-zero, before it listens. ``arisrvx --python main.py
--http-listen ADDR:PORT`` serves ``main.app`` for a program that does not call it.

Under plain ``python`` (no relay) the same program runs: Host keeps a tree of its own (pycvc's),
a write lands at once, and ``run()`` takes SIGINT and SIGTERM itself.

Threads: a sync handler runs on the loop's thread and holds up every other caller, HTTP and the
relay's events alike, so keep it quick and move blocking work to ``await asyncio.to_thread(...)``.

This file is built into arisrvx (src/ariadne_python/ariadne_server.py) and shipped with pycvc as
_pycvc_ariadne.py; pycvc imports it as pycvc.ariadne.
"""

from __future__ import annotations

import asyncio
import contextlib
import inspect
import logging
import math
import os
import signal
import socket
import sys
import threading
import time
import traceback
from typing import Any, Mapping

log = logging.getLogger("pycvc.ariadne")

_current = None  # the program's Host (current_host)
_program = None  # arisrvx's run of this program (_serve_program), else None
_server_class = None  # uvicorn.Server without its signal handlers, made on first use


def _arihost():
    """The server host (arisrvx) this interpreter runs in, once it has lent its relay; else None."""
    try:
        import _arihost
    except ImportError:
        return None
    return _arihost if _arihost.attached() and _arihost.mode() == "server" else None


def current_host():
    """The program's Host. In arisrvx it is made on first use; under plain ``python``, None until
    the program makes one."""
    if _current is None and _arihost() is not None:
        Host()
    return _current


# -- values, as pycvc_gl.ariadne writes them -------------------------------------------------------


def _format(value) -> str:
    """The text a scalar is kept as in the state tree (what a document's widgets read)."""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "1" if value else "0"
    if hasattr(value, "item") and not isinstance(value, (list, tuple, dict)):
        value = value.item()  # a numpy scalar
        if isinstance(value, bool):
            return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value) if math.isfinite(value) else str(value)
    if value is None:
        return ""
    raise TypeError(f"pycvc.ariadne: a {type(value).__name__} is not a state value")


def _node_name(key) -> str:
    k = str(key)
    if not k or "." in k or k != k.strip():
        raise ValueError(f"pycvc.ariadne: the key {k!r} can't name a node (empty, '.', or space at "
                         "an end)")
    return k


def _tree(value, depth=0):
    """``value`` as a tree for the relay: a scalar's text, or (text, [(name, tree), ...]) for a list
    (its text its length, its children 0..n-1) or a dict."""
    if depth > 64:
        raise ValueError("Host.set_tree: nested deeper than 64")
    if hasattr(value, "tolist") and not isinstance(value, (str, bytes)):
        value = value.tolist()
    if isinstance(value, dict):
        return ("", [(_node_name(k), _tree(v, depth + 1)) for k, v in value.items()])
    if isinstance(value, (list, tuple)):
        return (str(len(value)), [(str(i), _tree(v, depth + 1)) for i, v in enumerate(value)])
    return _format(value)


def _exit_code(e: SystemExit) -> int:
    code = e.code
    if code is None:
        return 0
    if isinstance(code, int):
        return code
    print(code, file=sys.stderr)
    return 1


# -- Host ------------------------------------------------------------------------------------------


class _Served:
    """One serve_asgi(): its uvicorn server, the task serving it, and how it ended."""

    def __init__(self, app, server, address):
        self.app = app
        self.server = server
        self.address = address  # (host, port) as asked
        self.task = None
        self.error = None  # why it stopped, if it failed
        self.asked_to_stop = False


class Host:
    """The relay's tree and the script thread's loop, for a service's Python. See the module
    docstring."""

    def __init__(self, *, app=None):
        """Adopt the server host this program runs in (arisrvx), or, under plain ``python``, keep a
        tree of its own (``app``: a pycvc app to use instead of a new one)."""
        global _current
        if _current is not None and not _current._closed:
            raise RuntimeError("Host(): this program already has its Host; use "
                               "pycvc.ariadne.current_host()")
        self._link = _arihost()
        self.mode = "server" if self._link is not None else "standalone"
        self._program = _program if self._link is not None else None
        if self._program is not None:
            self.loop = self._program.loop
            self.shutdown_timeout = self._program.shutdown_timeout
        else:
            try:
                self.loop = asyncio.get_running_loop()
            except RuntimeError:
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
            self.shutdown_timeout = 10.0
        if self._link is None:
            try:
                import pycvc
            except ImportError as exc:
                raise ImportError("pycvc.ariadne.Host under plain python keeps its tree in pycvc: "
                                  "install pycvc-cp312") from exc
            self._pycvc = pycvc
            self.app = app if app is not None else pycvc.make_app()
        self._served: list[_Served] = []
        self._stop_code = None  # stop()'s, once asked
        self._stop_event = None  # made on the loop by main()
        self._exit_code = 0
        self._ran = False
        self._closed = False
        _current = self

    # -- data in -----------------------------------------------------------------------------------

    @staticmethod
    def _path(key: str) -> str:
        path = key[1:] if isinstance(key, str) and key.startswith("/") else key
        if not isinstance(path, str) or not path.strip("."):
            raise ValueError(f"pycvc.ariadne: {key!r} is not a state path")
        return path

    def set(self, key: str, value) -> None:
        """Write ``value`` at ``key``: a number, bool, str or None as its text; a dict or list as a
        subtree (``set_tree``); a 1-D numpy array as a series (``set_series``)."""
        if isinstance(value, (dict, list, tuple)):
            self.set_tree(key, value)
            return
        if hasattr(value, "ndim") and getattr(value, "ndim", 0) >= 1:
            if value.ndim != 1:
                raise TypeError("Host.set: a numpy array with ndim > 1 is not a value")
            self.set_series(key, value)
            return
        path, text = self._path(key), _format(value)
        if self._link is not None:
            self._link.queue_set(path, text)
        else:
            self._pycvc.state_set(self.app, path, text)

    def __setitem__(self, key: str, value) -> None:
        self.set(key, value)

    def set_many(self, values: Mapping[str, Any], prefix: str = "") -> None:
        """``set()`` each ``key: value`` (under ``prefix.``, if given)."""
        for k, v in values.items():
            self.set(f"{prefix}.{k}" if prefix else k, v)

    def set_tree(self, key: str, value) -> None:
        """Replace the subtree at ``key`` with ``value``, as the DSL's ``(state-set-tree)`` does: a
        dict's keys and a list's indices name child nodes, a list's node holds its length, a scalar
        is a node's text, and the children ``value`` doesn't name are removed."""
        path, tree = self._path(key), _tree(value)
        if self._link is not None:
            self._link.queue_tree(path, tree)
        else:
            self._write_tree(path, tree)

    def _write_tree(self, path, tree):
        p = self._pycvc
        text, kids = (tree, []) if isinstance(tree, str) else tree
        if not p.state_has(self.app, path) or p.state_get(self.app, path) != text:
            p.state_set(self.app, path, text)
        names = {name for name, _ in kids}
        for old in list(p.state_children(self.app, path)):
            if old not in names:
                p.state_remove(self.app, f"{path}.{old}")
        for name, child in kids:
            self._write_tree(f"{path}.{name}", child)

    def set_series(self, key: str, values, *, fmt: str = ".6g", limit: int | None = None) -> None:
        """Write numbers at ``key`` as the comma-separated text a ``plot:`` draws (oldest first).
        ``limit`` keeps the last ``limit`` of them."""
        vals = values.tolist() if hasattr(values, "tolist") else list(values)
        if limit is not None:
            vals = vals[-int(limit):]
        self.set(key, ",".join(format(float(v), fmt) for v in vals))

    # -- data out ----------------------------------------------------------------------------------

    def get(self, key: str, default: str | None = None) -> str | None:
        """The text at ``key``, or ``default`` when there is no such node. In arisrvx it is the tree
        as the relay holds it: a write of this program's shows once the relay has applied it."""
        path = self._path(key)
        if self._link is not None:
            v = self._link.get(path)
            return default if v is None else v
        if not self._pycvc.state_has(self.app, path):
            return default
        return self._pycvc.state_get(self.app, path)

    def __getitem__(self, key: str) -> str:
        v = self.get(key)
        if v is None:
            raise KeyError(key)
        return v

    def __contains__(self, key: str) -> bool:
        return self.get(key) is not None

    def get_float(self, key: str, default: float = math.nan) -> float:
        v = self.get(key)
        try:
            return float(v) if v not in (None, "") else default
        except ValueError:
            return default

    def get_int(self, key: str, default: int = 0) -> int:
        v = self.get(key)
        try:
            return int(float(v)) if v not in (None, "") else default
        except ValueError:
            return default

    def get_bool(self, key: str, default: bool = False) -> bool:
        v = (self.get(key) or "").strip().lower()
        if v in ("1", "true", "yes", "on", "y", "t"):
            return True
        if v in ("0", "false", "no", "off", "n", "f"):
            return False
        try:
            return float(v) != 0.0
        except ValueError:
            return default

    def get_series(self, key: str) -> list[float]:
        """The comma-separated numbers at ``key``, as floats."""
        return [float(x) for x in (self.get(key) or "").split(",") if x.strip()]

    def _children(self, path):
        if self._link is not None:
            return self._link.children(path)
        if not self._pycvc.state_has(self.app, path):
            return None
        return list(self._pycvc.state_children(self.app, path))

    def get_tree(self, key: str):
        """The subtree at ``key`` as Python data, the inverse of ``set_tree``: a node whose
        children are 0..n-1 is a list, other parents dicts, and leaves their text (``None`` when
        there is no such node)."""
        return self._read_tree(self._path(key))

    def _read_tree(self, path):
        kids = self._children(path)
        if kids is None:
            return None
        if not kids:
            return self.get("/" + path)
        if all(k.isdigit() for k in kids) and sorted(int(k) for k in kids) == list(range(len(kids))):
            return [self._read_tree(f"{path}.{i}") for i in range(len(kids))]
        return {k: self._read_tree(f"{path}.{k}") for k in kids}

    # -- serving -----------------------------------------------------------------------------------

    @property
    def servers(self) -> list:
        """The uvicorn servers serve_asgi() made, in order."""
        return [s.server for s in self._served]

    def listening(self) -> list[tuple[str, int]]:
        """Where the ASGI servers listen now, as (host, port): a port 0 asked for reads as the one
        bound."""
        out = []
        for s in self._served:
            for srv in getattr(s.server, "servers", None) or []:
                for sock in srv.sockets or []:
                    name = sock.getsockname()
                    out.append((name[0], name[1]))
        return out

    def _add_served(self, served: _Served) -> None:
        self._served.append(served)
        if self.checking:
            return
        coro = self._serve_one(served)
        if self._on_loop_thread() or not self.loop.is_running():
            served.task = self.loop.create_task(coro)
        else:
            def start():
                served.task = self.loop.create_task(coro)

            self.loop.call_soon_threadsafe(start)

    async def _serve_one(self, served: _Served) -> None:
        # uvicorn ends a failed startup (a lifespan startup that raised, a port it could not bind)
        # with sys.exit(3). Out of a task of its own, asyncio re-raises a SystemExit out of the
        # loop and so stops every task on it (risk 16), so it is caught here, inside the task.
        try:
            await served.server.serve()
        except SystemExit as e:
            served.error = f"its startup failed (uvicorn exit {e.code})"
        except asyncio.CancelledError:
            raise
        except Exception as e:  # noqa: BLE001 - logged, and the host stops on it
            served.error = f"{type(e).__name__}: {e}"
            log.exception("serve_asgi: the server on %s:%s failed", *served.address)
        if not served.server.started and not served.asked_to_stop:
            log.error("serve_asgi on %s:%s: %s", *served.address,
                      served.error or "it stopped before it started")
        elif not served.asked_to_stop:
            # A server that ends on its own after it started stops the host too, so a supervisor
            # sees the service go (Restart=on-failure).
            log.warning("serve_asgi: the server on %s:%s stopped by itself%s", *served.address,
                        f" ({served.error})" if served.error else "")
            self.stop(1 if served.error else 0)

    @property
    def checking(self) -> bool:
        """True under ``arisrvx check``: nothing is served, and run() returns at once."""
        return self._program is not None and self._program.check

    def _on_loop_thread(self) -> bool:
        try:
            return asyncio.get_running_loop() is self.loop
        except RuntimeError:
            return False

    def stop(self, code: int = 0) -> None:
        """Stop the host: its loop shuts down as for SIGTERM, and run() returns ``code``. From any
        thread."""
        if self._stop_code is None:
            self._stop_code = int(code)
        if self._stop_event is not None:
            if self._on_loop_thread():
                self._stop_event.set()
            else:
                try:
                    self.loop.call_soon_threadsafe(self._stop_event.set)
                except RuntimeError:  # the loop is closed
                    pass

    def run(self) -> int:
        """Run the loop until the host is stopped (SIGTERM, SIGINT, ``stop()``), then shut down;
        returns the exit status. Under ``arisrvx check`` it returns 0 at once."""
        if self._ran:
            raise RuntimeError("Host.run: the host has already run")
        self._ran = True
        if self.checking:
            return 0
        if self.loop.is_running():
            raise RuntimeError("Host.run: the loop is already running; `await host.main()` instead")
        restore = self._take_signals() if self._link is None else {}
        try:
            self._exit_code = self.loop.run_until_complete(self.main())
        finally:
            for sig, handler in restore.items():
                signal.signal(sig, handler)
        return self._exit_code

    def _take_signals(self) -> dict:
        """Plain python on its main thread: SIGINT and SIGTERM stop the host, as arisrvx's do."""
        if threading.current_thread() is not threading.main_thread():
            return {}
        old = {}
        for sig in (signal.SIGINT, signal.SIGTERM):
            old[sig] = signal.signal(sig, lambda *_: self.loop.call_soon_threadsafe(self.stop, 0))
        return old

    async def main(self) -> int:
        """What run() runs, for a program that runs the loop itself: serve until stopped, then
        shut down. Returns the exit status."""
        self._ran = True
        self._stop_event = asyncio.Event()
        if self._stop_code is not None:
            self._stop_event.set()
        if self._program is not None and self._program.shutdown_asked:
            self.stop(0)
        self._serve_flag_app()
        await self._started()
        await self._stop_event.wait()
        await self._shutdown()
        self._closed = True
        return self._stop_code or 0

    def _serve_flag_app(self) -> None:
        """--http-listen: serve the program's ``app`` there."""
        prog = self._program
        if prog is None or not prog.http_listen or prog.http_served:
            return
        prog.http_served = True
        main = sys.modules.get("__main__")
        app = getattr(main, "app", None) if prog.namespace is None else prog.namespace.get("app")
        if app is None:
            raise RuntimeError(f"--http-listen {prog.http_listen}: {prog.path} defines no `app` "
                               "to serve")
        h, p = _split_address(prog.http_listen)
        serve_asgi(app, host=h, port=p)

    async def _started(self) -> None:
        """Wait for every server to start (or fail, or the host to stop); then tell the relay it
        may listen, or that the program failed."""
        while not self._stop_event.is_set():
            waiting = [s for s in self._served
                       if s.task is not None and not s.server.started and not s.task.done()]
            if not waiting:
                break
            await asyncio.sleep(0.01)
        failed = [s for s in self._served if s.task is not None and s.task.done()
                  and not s.server.started]
        why = ""
        if failed:
            why = "; ".join(f"serve_asgi on {s.address[0]}:{s.address[1]}: "
                            f"{s.error or 'it stopped before it started'}" for s in failed)
            self.stop(1)
        elif self._stop_event.is_set():
            why = "it was stopped before it served"
        if self._link is not None:
            self._link.ready(not why, why)
            self._program.ready_sent = True
        if not why:
            for addr in self.listening():
                log.info("serving ASGI on %s:%d", *addr)

    async def _shutdown(self) -> None:
        """Every server stops accepting and finishes (within shutdown_timeout), its app's shutdown
        included; then the loop's other tasks are cancelled."""
        tasks = []
        for s in self._served:
            s.asked_to_stop = True
            s.server.should_exit = True
            if s.task is not None and not s.task.done():
                tasks.append(s.task)
        if tasks:
            # uvicorn bounds the drain by the timeout, then runs the app's shutdown: 2 s more for
            # that, inside the 5 s arisrvx allows past the timeout before it gives up on the program.
            done, pending = await asyncio.wait(tasks, timeout=self.shutdown_timeout + 2.0)
            if pending:
                log.warning("shutdown: %d ASGI server(s) still running after %.1f s; forcing them",
                            len(pending), self.shutdown_timeout + 2.0)
                for s in self._served:
                    s.server.force_exit = True
                done, pending = await asyncio.wait(pending, timeout=1.0)
                for t in pending:
                    t.cancel()
        me = asyncio.current_task()
        rest = [t for t in asyncio.all_tasks(self.loop) if t is not me and not t.done()]
        for t in rest:
            t.cancel()
        if rest:
            await asyncio.wait(rest, timeout=1.0)


def _split_address(address: str) -> tuple[str, int]:
    host, sep, port = address.rpartition(":")
    if not sep or not port.isdigit():
        raise ValueError(f"{address!r} is not ADDR:PORT")
    return host.strip("[]") or "0.0.0.0", int(port)


def serve_asgi(app, *, host: str = "127.0.0.1", port: int = 8000, **config):
    """Serve the ASGI ``app`` with uvicorn on the program's Host's loop (made when there is none),
    at ``host``:``port``; ``config`` is the rest of ``uvicorn.Config``'s arguments. Returns the
    ``uvicorn.Server``. uvicorn's logging goes to the host's (``log_config=None``), and its graceful
    shutdown is bounded by the host's shutdown timeout unless ``timeout_graceful_shutdown`` says
    otherwise. The server's signal handlers are left out: the host owns signals."""
    h = current_host() or Host()
    config.setdefault("log_config", None)
    config.setdefault("timeout_graceful_shutdown", h.shutdown_timeout)
    import uvicorn

    server = _uvicorn_server()(uvicorn.Config(app, host=host, port=int(port), **config))
    h._add_served(_Served(app, server, (host, int(port))))
    return server


def _uvicorn_server():
    global _server_class
    if _server_class is None:
        import uvicorn

        class Server(uvicorn.Server):
            """uvicorn's Server with no signal handlers of its own: the host stops it."""

            @contextlib.contextmanager
            def capture_signals(self):
                yield

        _server_class = Server
    return _server_class


# -- functions: room-call and the server document's (S2) -------------------------------------------

DEFAULT_TIMEOUT = 30.0  # seconds a client's call may run when neither it nor its export says


class CallError(Exception):
    """An error a function raises for its caller: ``code`` (a short word: ``not_found``,
    ``conflict``, ...), ``message`` and ``detail`` (a DSL value: numbers, strings, lists, dicts).
    The caller's ``try`` catches it with them as fields. Any other exception reaches a client only
    as ``internal``."""

    def __init__(self, code: str, message: str = "", *, detail=None):
        super().__init__(f"{code}: {message}" if message else str(code))
        self.code = str(code)
        self.message = str(message)
        self.detail = detail


class CallContext:
    """``ctx``: what a client's call carries, passed first to a function exported with
    ``clients=True``."""

    def __init__(self, principal, call_id, idempotency_key, deadline, progress=None):
        self.principal = principal  # {"id": ..., "roles": [...]}
        self.call_id = call_id
        self.idempotency_key = idempotency_key  # the caller's, or the call id
        self.deadline = deadline  # the time.time() at which the call is cancelled
        self._progress = progress

    def progress(self, fraction: float, message: str = "") -> None:
        """Tell the caller how far along the call is (0 to 1): it lands in the caller's tree."""
        if self._progress is not None:
            self._progress(float(fraction), str(message))

    @property
    def time_left(self) -> float:
        """Seconds until the deadline."""
        return max(0.0, self.deadline - time.time())


# The server's own calls (the server document's programs): trusted, so no role gate applies.
SERVER_PRINCIPAL = {"id": "server", "roles": []}


class _Export:
    def __init__(self, fn, name, options, origin):
        self.fn = fn
        self.name = name
        self.clients = bool(options.get("clients"))
        roles = options.get("roles") or ()
        self.roles = [roles] if isinstance(roles, str) else [str(r) for r in roles]
        timeout = options.get("timeout")
        self.timeout = float(timeout) if timeout else None
        self.coroutine = inspect.iscoroutinefunction(fn)
        self.threaded = bool(options.get("is_async")) and not self.coroutine
        self.origin = origin  # "program", or "block" (the server document's, replaced on reload)


_exports: dict[str, _Export] = {}
_in_block = False  # the server document's block is loading: its load registers what it exports


def export(fn=None, *, name=None, clients=False, roles=None, timeout=None, is_async=False):
    """Export ``fn`` as ``name`` (its own name by default): the server document's programs call it
    as ``(name a b ...)``, and with ``clients=True`` a client may call it with ``(room-call "name"
    ...)``, ``ctx`` first. ``roles``: the caller needs one of them. ``timeout``: seconds before the
    call is cancelled. ``is_async``: a plain function runs on a thread (``asyncio.to_thread``)."""

    def mark(f):
        if not callable(f):
            raise TypeError(f"export: {f!r} is not callable")
        options = {"name": name or f.__name__, "is_async": is_async, "with_call": False,
                   "done_channel": "", "clients": clients, "roles": roles, "timeout": timeout}
        f.__ari_export__ = options
        if not _in_block:
            _exports[options["name"]] = _Export(f, options["name"], options, "program")
        return f

    return mark(fn) if fn is not None else mark


def exports() -> dict:
    """The functions exported now, by name: ``{"name": {"clients": bool, "roles": [...]}}``."""
    return {n: {"clients": e.clients, "roles": list(e.roles)} for n, e in sorted(_exports.items())}


class _Deadline(Exception):
    """The call ran past its timeout."""


def _codec():
    """pycvc's conversion of DSL values, as (from a capsule, to a capsule), or None without one."""
    try:
        import pycvc

        return pycvc.value_from_capsule, pycvc.value_capsule
    except (ImportError, AttributeError):
        return None


# -- arisrvx's runner ------------------------------------------------------------------------------


class _Program:
    """arisrvx's run of the program: its loop, its options and the relay's events."""

    def __init__(self, link, path, options):
        self.link = link
        self.path = path
        self.loop = asyncio.new_event_loop()
        self.http_listen = options.get("http_listen") or ""
        self.http_served = False
        self.shutdown_timeout = float(options.get("shutdown_timeout", 10.0))
        self.check = bool(options.get("check"))
        self.ports = [tuple(p) for p in options.get("ports", ())]
        self.namespace = None  # the program's globals, once it has run
        self.slow_ms = float(options.get("slow_ms", 50.0))
        self.shutdown_asked = False
        self.check_done = None  # arisrvx check: set when the relay has loaded the document
        self.ready_sent = False
        self.waker = None
        self.codec = None  # pycvc's value conversion (_codec), once the program starts
        self.tasks = set()  # the calls running

    def start_waker(self) -> None:
        """A thread that waits on the relay's events without the GIL and hands each to the loop,
        so the relay never takes the GIL to post one."""

        def wake():
            while True:
                event = self.link.next_event()  # (kind, id, value capsule or None)
                if event is None:
                    return
                try:
                    self.loop.call_soon_threadsafe(self.on_event, *event)
                except RuntimeError:  # the loop is closed: nothing is left to tell
                    return

        self.waker = threading.Thread(target=wake, name="arisrvx-events", daemon=True)
        self.waker.start()

    def on_event(self, kind: str, ident: int = 0, value=None) -> None:
        if kind == "shutdown":
            self.shutdown_asked = True
            if self.check_done is not None:
                self.check_done.set()
            if _current is not None:
                _current.stop(0)
            return
        try:
            if self.codec is None:
                raise RuntimeError("no pycvc to convert the values (install pycvc-cp312)")
            value = self.codec[0](value) if value is not None else None
        except Exception as e:  # noqa: BLE001 - answered, not raised into the loop
            if ident:
                self.link.answer_error(ident, f"{type(e).__name__}: {e}")
            else:
                log.error("room-call: a call's values could not be read: %s", e)
            return
        if kind == "call":
            self.spawn(self.client_call(value))
        elif kind == "load_block":
            self.load_block(ident, value)
        elif kind == "doc_call":
            self.doc_call(ident, value)
        elif ident:
            self.link.answer_error(ident, f"pycvc.ariadne: no event {kind!r}")

    def spawn(self, coro) -> None:
        task = self.loop.create_task(coro)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    # -- running a function --------------------------------------------------------------------

    async def run(self, exp: _Export, args, kwargs, timeout):
        """Run an export: an async def on the loop, a plain one inline (warned past
        --script-slow-ms) or on a thread (is_async). _Deadline past ``timeout`` seconds."""
        if exp.coroutine:
            work = exp.fn(*args, **kwargs)
        elif exp.threaded:
            work = asyncio.to_thread(exp.fn, *args, **kwargs)
        else:
            t0 = time.perf_counter()
            result = exp.fn(*args, **kwargs)
            took = time.perf_counter() - t0
            if self.slow_ms and took * 1000.0 > self.slow_ms:
                log.warning("%s: a sync function ran %.0f ms, over --script-slow-ms %g, and held up "
                            "every other caller: make it async, or move its work to "
                            "asyncio.to_thread", exp.name, took * 1000.0, self.slow_ms)
            if timeout and took > timeout:
                raise _Deadline()
            if not inspect.isawaitable(result):
                return result
            work = result
            timeout = max(0.001, timeout - took) if timeout else None
        scope = None
        try:
            async with asyncio.timeout(timeout) as scope:
                return await work
        except TimeoutError:
            if scope is not None and scope.expired():
                raise _Deadline() from None
            raise

    # -- a client's call -----------------------------------------------------------------------

    def reply(self, pid, call_id, value) -> None:
        try:
            capsule = self.codec[1](value)
        except Exception as e:  # noqa: BLE001
            log.error("room-call %s: the function's value is not a DSL value: %s", call_id, e)
            self.link.reply_error(pid, call_id, "internal", "the function's value is not a DSL "
                                  f"value ({type(e).__name__}: {e})")
            return
        self.link.reply(pid, call_id, capsule)

    def reply_error(self, pid, call_id, code, message, detail=None) -> None:
        capsule = None
        if detail is not None:
            try:
                capsule = self.codec[1](detail)
            except Exception as e:  # noqa: BLE001 - the error goes without it
                log.warning("room-call %s: CallError's detail is not a DSL value: %s", call_id, e)
        self.link.reply_error(pid, call_id, str(code), str(message), capsule)

    async def client_call(self, call: dict) -> None:
        pid = str(call.get("pid") or "")
        call_id = str(call.get("id") or "")
        name = str(call.get("fn") or "")
        exp = _exports.get(name)
        if exp is None or not exp.clients:
            self.reply_error(pid, call_id, "not_found", f"no function {name!r} clients may call")
            return
        principal = call.get("principal") or {"id": pid, "roles": []}
        if exp.roles and not set(exp.roles) & set(principal.get("roles") or ()):
            self.reply_error(pid, call_id, "forbidden",
                             f"{name} needs the role {' or '.join(exp.roles)}")
            return
        asked = float(call.get("timeout") or 0) or None
        limits = [t for t in (exp.timeout, asked) if t]
        timeout = min(limits) if limits else DEFAULT_TIMEOUT
        ctx = CallContext(principal, call_id, str(call.get("key") or call_id), time.time() + timeout,
                          lambda f, m: self.link.progress(pid, call_id, f, m))
        try:
            result = await self.run(exp, [ctx, *(call.get("args") or [])],
                                    dict(call.get("kwargs") or {}), timeout)
        except CallError as e:
            self.reply_error(pid, call_id, e.code, e.message, e.detail)
        except _Deadline:
            self.reply_error(pid, call_id, "deadline_exceeded",
                             f"{name} ran past its {timeout:g} s")
        except asyncio.CancelledError:
            self.reply_error(pid, call_id, "unavailable", "the relay's Python program stopped")
            raise
        except Exception:  # noqa: BLE001 - logged here; the client hears only that it failed
            log.exception("room-call %s from %s raised", name, pid)
            self.reply_error(pid, call_id, "internal",
                             f"{name} failed; its traceback is in the relay's log")
        else:
            self.reply(pid, call_id, result)

    # -- the server document -------------------------------------------------------------------

    def load_block(self, ident: int, value: dict) -> None:
        """The server document loads: run its python: block (if it has one), and answer with what
        it may call, every function exported now, and the block's warnings."""
        global _in_block
        import _ariadne_python as doc

        for name in [n for n, e in _exports.items() if e.origin == "block"]:
            del _exports[name]  # a reload runs the block again, in a fresh module
        warnings = []
        block = value.get("block") if value else None
        if block is not None:
            def register(name, fn, is_async, with_call, done_channel):
                options = dict(getattr(fn, "__ari_export__", None) or {})
                options["is_async"] = is_async
                _exports[name] = _Export(fn, name, options, "block")

            _in_block = True
            try:
                warnings = doc.load_document(block, value.get("document") or "", register,
                                             export_fn=export)
            except BaseException as e:  # noqa: BLE001 - the load fails with it
                self.link.answer_error(ident, doc.describe(e))
                return
            finally:
                _in_block = False
        self.link.answer(ident, self.codec[1]({"exports": sorted(_exports),
                                               "warnings": list(warnings)}))

    def doc_call(self, ident: int, value: dict) -> None:
        name = str(value.get("fn") or "")
        exp = _exports.get(name)
        if exp is None:
            self.link.answer_error(ident, f"no exported function {name!r}")
            return
        args = list(value.get("args") or [])
        if exp.clients:  # it takes ctx first: the server's own call
            call_id = f"server-{ident}"
            args.insert(0, CallContext(dict(SERVER_PRINCIPAL), call_id, call_id,
                                       time.time() + (exp.timeout or DEFAULT_TIMEOUT)))
        self.spawn(self._doc_call(ident, exp, args))

    async def _doc_call(self, ident: int, exp: _Export, args) -> None:
        import _ariadne_python as doc

        try:
            result = await self.run(exp, args, {}, exp.timeout)
            capsule = self.codec[1](result)
        except _Deadline:
            self.link.answer_error(ident, f"{exp.name} ran past its {exp.timeout:g} s")
        except asyncio.CancelledError:
            self.link.answer_error(ident, "the relay's Python program stopped")
            raise
        except BaseException as e:  # noqa: BLE001 - a PythonError in the DSL
            self.link.answer_error(ident, doc.describe(e))
        else:
            self.link.answer(ident, capsule)


def _install_parent():
    """At arisrvx's boot: `import pycvc.ariadne` binds `pycvc`, so without a pycvc installed beside
    the host a stand-in holds this module. With one, pycvc's own import keeps this module."""
    import importlib.util
    import types

    me = sys.modules[__name__]
    if importlib.util.find_spec("pycvc") is None:
        parent = types.ModuleType("pycvc", "A stand-in: pycvc is not installed beside this host, so "
                                  "only pycvc.ariadne (built into arisrvx) is here.")
        parent.ariadne = me
        sys.modules["pycvc"] = parent


def _setup_logging() -> None:
    """Python's logging to stderr in the relay's manner, unless the program configures its own."""
    root = logging.getLogger()
    if not root.handlers:
        handler = logging.StreamHandler(sys.stderr)
        handler.setFormatter(logging.Formatter("[arisrvx] %(name)s: %(levelname)s: %(message)s"))
        root.addHandler(handler)
        root.setLevel(logging.INFO)


def _print_error(path: str) -> None:
    """The exception being handled, with its traceback from the program's own frames on."""
    exc = sys.exc_info()[1]
    tb = exc.__traceback__
    here = os.path.normcase(__file__) if "__file__" in globals() else ""
    while tb is not None and tb.tb_next is not None:
        name = tb.tb_frame.f_code.co_filename
        if name.endswith("runpy.py") or name.startswith("<frozen") or "(built in)" in name or (
                here and os.path.normcase(name) == here):
            tb = tb.tb_next
            continue
        break
    traceback.print_exception(type(exc), exc, tb, file=sys.stderr)


def _close_loop(loop) -> None:
    try:
        rest = [t for t in asyncio.all_tasks(loop) if not t.done()]
        for t in rest:
            t.cancel()
        if rest:
            loop.run_until_complete(asyncio.wait(rest, timeout=1.0))
        loop.run_until_complete(loop.shutdown_asyncgens())
        loop.run_until_complete(loop.shutdown_default_executor(timeout=5.0))
    finally:
        loop.close()


def _serve_program(path, args, options) -> int:
    """Run the program at ``path`` on this thread, arisrvx's script thread (Engine::serve): as
    ``__main__``, with its loop as this thread's current one; then run that loop until the host is
    stopped, unless the program ran it. Returns the exit status."""
    import _arihost
    import runpy

    global _program
    prog = _Program(_arihost, path, options)
    _program = prog
    asyncio.set_event_loop(prog.loop)
    _setup_logging()
    # Before the program runs, so the relay knows from the start whether calls can be posted.
    prog.codec = _codec()
    _arihost.set_converts(prog.codec is not None)
    prog.start_waker()
    sys.argv = [path or "arisrvx"] + list(args)
    if path:
        sys.path.insert(0, os.path.dirname(os.path.abspath(path)))
    try:
        if path:  # else the server document's python: block alone: the loop runs for it
            try:
                prog.namespace = runpy.run_path(path, run_name="__main__")
            except SystemExit as e:
                return _exit_code(e)
            except BaseException:  # noqa: BLE001 - the program's own error, as python reports it
                _print_error(path)
                return 1
        host = current_host()
        if prog.codec is None and any(e.clients for e in _exports.values()):
            log.warning("room-call: %d function(s) are exported for clients, but there is no "
                        "pycvc beside arisrvx to convert their values (install pycvc-cp312): "
                        "their calls are answered unavailable",
                        sum(1 for e in _exports.values() if e.clients))
        if prog.check:
            # The relay loads the server document now (its python: block runs on this loop),
            # then posts shutdown; the report comes after.
            prog.check_done = asyncio.Event()
            if prog.shutdown_asked:
                prog.check_done.set()
            prog.link.ready(True, "")
            prog.loop.run_until_complete(prog.check_done.wait())
            return _check(prog, host)
        if not host._ran:
            return host.run()
        return host._exit_code
    except SystemExit as e:
        return _exit_code(e)
    except BaseException:  # noqa: BLE001
        _print_error(path)
        return 1
    finally:
        _close_loop(prog.loop)
        prog.link.close_events()
        prog.waker.join(timeout=5.0)


# -- arisrvx check ---------------------------------------------------------------------------------


def _port_free(host: str, port: int) -> str:
    """'' when ``host``:``port`` can be bound now, else why not."""
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    with socket.socket(family, socket.SOCK_STREAM) as s:
        if sys.platform == "win32":
            s.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
        except OSError as e:
            return e.strerror or str(e)
    return ""


def _check(prog: _Program, host: Host) -> int:
    """arisrvx check: the program ran without serving; report what it would serve and whether the
    ports are free. 0 when all is well."""
    problems = []
    listeners = list(prog.ports)
    if prog.http_listen:
        main = prog.namespace or {}
        if main.get("app") is None:
            problems.append(f"--http-listen {prog.http_listen}: {prog.path} defines no `app` to "
                            "serve")
        listeners.append(("--http-listen", prog.http_listen))
    for s in host._served:
        listeners.append(("serve_asgi", f"{s.address[0]}:{s.address[1]}"))
    seen = {}
    free = []
    for what, address in listeners:
        try:
            h, p = _split_address(address)
        except ValueError as e:
            problems.append(f"{what}: {e}")
            continue
        if p == 0:
            continue
        if (h, p) in seen:
            problems.append(f"{what} {address}: {seen[(h, p)]} asks for that port too")
            continue
        seen[(h, p)] = what
        why = _port_free(h, p)
        if why:
            problems.append(f"{what} {address}: the port is taken ({why})")
        else:
            free.append(f"{what} {address}")
    for p in problems:
        print(f"arisrvx check: {p}", file=sys.stderr)
    if problems:
        return 1
    n = len(host._served) + (1 if prog.http_listen else 0)
    clients = sorted(name for name, e in _exports.items() if e.clients)
    calls = f"; clients may call {', '.join(clients)}" if clients else ""
    print(f"arisrvx check: {prog.path or 'the server document'}: ok: {n} ASGI app(s) to serve"
          f"{calls}; free: {', '.join(free) if free else 'no fixed ports'}")
    return 0
