"""Run an Ariadne (``.ari``) document from a Python program.

``Host`` opens a document in a cvcGL window, offscreen, or headless with no GL, and wires it to the
Python program around it:

* the document calls Python: ``@host.on("event")`` for a button's or menu item's ``on: event``,
  ``@host.function("name")`` for ``(name a b ...)`` in any of its programs (arguments in, a value
  back), and ``@host.async_function("name")`` for work that runs on the app's background task pool
  while the calling program waits (the UI keeps drawing);
* Python feeds the document: ``host.set(key, value)`` for values, ``host.set_tree`` /
  ``host.set_table`` for nested data and tables (what ``repeat:`` counts), ``host.set_series`` for
  the numbers a ``plot:`` draws, ``host.publish_image`` for the pictures an ``image:`` shows, and
  ``host.open_stream`` for live video a ``stream_view`` or ``type: stream`` node plays;
* Python reads what the document produces: ``host.get`` / ``get_float`` / ``get_tree`` /
  ``get_series``, ``@host.watch(key)`` for a callback when a value changes, the values its
  programs return (``host.run``), ``host.warnings``, and the rendered frame (``host.snapshot``,
  ``host.frame_rgb``).

A key is relative to the document's state prefix, as a ``bind:`` is: ``host.set("form.a", 1)``
writes what ``bind: form.a`` shows. A key starting with ``/`` is app-root absolute.

The loop is yours (``host.step()``) or the host's (``host.run()``)::

    from pycvc_gl.ariadne import Host

    host = Host(mode="window")            # or "offscreen" / "headless"

    @host.on("ping")
    def ping():
        host.set("count", host.get_int("count", 0) + 1)

    @host.function("solve")
    def solve(a, b):                      # (solve 2 3) in the document
        return a * b

    host.load("hello.ari")
    host.run()                            # until the window closes or the document quits
    host.close()

Threads and the GIL. Every handler, function and watch callback runs on the thread that calls
``step()`` / ``run()``, except an async function, which runs on a pool thread. A frame releases the
GIL while C++ runs and each callback takes it back for its call, so an async function, or another
Python thread, runs while the frame does. A callback may read and write state and queue programs
(``host.run``), but it may not run a frame: ``step()`` or ``run()`` from inside one raises
``RuntimeError``, and ``load()`` from inside one is deferred until the frame ends. ``close()``
waits for async functions still running, so give a long one a way to stop (``with_call=True``
passes it the call, whose ``cancelled()`` turns true when its program is gone).

Errors. An exception in an ``on`` handler, an ``on_frame`` hook or a ``watch`` callback is raised
from the ``step()`` / ``run()`` that ran it, once the frame is done. An exception in a
``function`` is the document's: its call raises ``PythonError``, which a ``try`` in the document
can catch. One it does not catch stops that program with a warning, and with ``strict=True``
every warning is raised from ``step()`` as an ``AriadneError`` (the function's exception as its
cause), which is what a test wants.
"""

from __future__ import annotations

import functools
import logging
import math
import os
import threading
import time
from typing import Any, Callable, Iterable, Mapping, Sequence

log = logging.getLogger("pycvc_gl.ariadne")

MODES = ("window", "offscreen", "headless")

# Ariadne caps each action (docs/ARIADNE.md §2.2) at 0.1 s and 5000 steps, sized for a document's own
# code. A Python function's run time counts against the time cap, and an async one's CPU time as
# steps (8000 a CPU millisecond), so a Python host raises them: an async call parks its program and
# the UI keeps drawing, so a long one costs nothing while it runs. Host(limits=...) overrides these;
# limits={} keeps Ariadne's.
DEFAULT_LIMITS = {"action_max_time": 30.0, "action_max_steps": 0}


class AriadneError(RuntimeError):
    """A runtime warning from the document, raised under ``Host(strict=True)``."""


def _modules():
    try:
        import pycvc
    except ImportError as exc:  # pragma: no cover - environment guard
        raise ImportError(
            "pycvc_gl.ariadne needs the pycvc bindings (from libcvc): install libcvc with its "
            "Python bindings into your prefix (e.g. cvcpkg install pycvc-gl-cp312)."
        ) from exc
    if not hasattr(pycvc, "host_function"):  # pragma: no cover - an older pycvc
        raise ImportError("pycvc_gl.ariadne needs a pycvc with host_function (libcvc 3.7.9+)")
    return pycvc


def _format(value) -> str:
    """The text a scalar is kept as in the state tree (what the document's widgets read)."""
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "1" if value else "0"  # what a checkbox writes; "true"/"false" read the same
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
    raise TypeError(f"pycvc_gl.ariadne: a {type(value).__name__} is not a state value")


def _to_dsl(value):
    """A Python function's value as one the DSL takes: numpy scalars and arrays become Python ones."""
    if hasattr(value, "tolist") and not isinstance(value, (str, bytes)):
        return value.tolist()
    if isinstance(value, tuple):
        return [_to_dsl(v) for v in value]
    if isinstance(value, list):
        return [_to_dsl(v) for v in value]
    if isinstance(value, dict):
        return {k: _to_dsl(v) for k, v in value.items()}
    return value


def _node_name(key) -> str:
    """A dict key as a child node's name, checked as (state-set-tree) checks it."""
    k = str(key)
    if not k or "." in k or k != k.strip():
        raise ValueError(f"pycvc_gl.ariadne: the key {k!r} can't name a node (empty, '.', or "
                         "space at an end)")
    return k


class Host:
    """An Ariadne document, run by this Python program. See the module docstring."""

    def __init__(
        self,
        document: str | None = None,
        *,
        mode: str = "window",
        width: int = 1024,
        height: int = 768,
        name: str = "ari",
        app=None,
        args: Sequence[str] | None = None,
        fixed_clock: bool | None = None,
        streaming: bool = False,
        component_paths: Iterable[str] = (),
        background: Sequence[float] | None = (0.09, 0.10, 0.12),
        strict: bool = False,
        limits: Mapping[str, float] | None = None,
    ):
        """Open a host. ``document`` (optional) is loaded at once; otherwise register what it calls
        first, then ``load()`` it (a document's ``requires:`` is checked at load).

        ``mode``: ``"window"`` (a real window), ``"offscreen"`` (GL with no window: captures, and
        CI on a software GL), or ``"headless"`` (no GL at all: the document's programs, state and
        host calls run, nothing is drawn, and a document that requires ``gl.scene`` doesn't load).
        ``name`` is the scene's name and the document's state prefix. ``fixed_clock`` steps the
        scene and UI clocks by each frame's dt instead of wall time (default: on offscreen, for
        reproducible frames). ``streaming`` registers the stream verbs and widgets before the
        document loads (``open_stream`` does too). ``limits`` sets the app's per-action caps,
        ``ariadne.limits.<name>`` (default ``DEFAULT_LIMITS``: a Python function may take its time).
        """
        if mode not in MODES:
            raise ValueError(f"Host: mode must be one of {MODES}, not {mode!r}")
        self._pycvc = pycvc = _modules()
        self.mode = mode
        self.app = app if app is not None else pycvc.make_app()
        for k, v in (DEFAULT_LIMITS if limits is None else limits).items():
            pycvc.state_set(self.app, f"ariadne.limits.{k}", _format(v))
        self.width, self.height = int(width), int(height)
        self.strict = strict
        self.warnings: list[str] = []
        self.images: dict[str, Any] = {}
        self.streams: dict[str, Any] = {}
        self.frame_count = 0
        self.document: str | None = None
        self._handlers: dict[str, Callable] = {}
        self._frame_hooks: list[Callable] = []
        self._watches: list[list] = []  # [key, path, tree, fn, last]
        self._errors: list[BaseException] = []
        self._function_errors: list[BaseException] = []
        self._program_failures: list[str] = []
        self._emitted: list[str] = []
        self._pending_load = None
        self._in_frame = False
        self._closed = False
        self._lab = None
        self._args = list(args) if args is not None else None
        self._thread = threading.get_ident()

        self.scene = self.view = self.camera = self.overlay = None
        if mode == "headless":
            self.runtime = pycvc.AriApp(self.app, name)
        else:
            import pycvc_gl

            self.scene = pycvc_gl.SceneGraph(self.app, name)
            self.view = pycvc_gl.SceneRenderer(self.scene, self.width, self.height,
                                               mode == "offscreen", "main")
            if background is not None:
                self.view.setBackground(*background)
            self.camera = pycvc_gl.CameraController(self.view)
            self.camera.setMode(pycvc_gl.CameraController.Mode_Orbit)  # as arigl's
            self.overlay = pycvc_gl.ImGuiOverlay(self.view)
            self.overlay.attachCamera(self.camera)
            self.overlay.setVisible(True)  # the widgets draw only while the overlay is visible
            self.runtime = pycvc_gl.AriRuntime(self.view, self.camera, self.overlay)
            if fixed_clock is None:
                fixed_clock = mode == "offscreen"
            self.runtime.set_fixed_clock(bool(fixed_clock))
        self.prefix = self.runtime.prefix()
        self.runtime.register_quit()  # on: quit and (quit [code]) close the host
        for d in component_paths:
            self.runtime.add_component_path(str(d))
        self._streaming = False
        if streaming:
            self.enable_streaming()
        if document is not None:
            self.load(document)

    # ── registration: what the document calls ────────────────────────────────────────────────────

    def on(self, event: str, fn: Callable[[], Any] | None = None):
        """Run ``fn()`` when the document raises ``event`` (a button's or menu item's ``on:``).
        Usable as a decorator. An exception is raised from the ``step()`` that ran it."""
        if fn is None:
            return lambda f: self.on(event, f)

        @functools.wraps(fn)
        def handler():
            try:
                fn()
            except BaseException as e:  # noqa: BLE001 - raised from step(), not into C++
                self._errors.append(e)

        self._handlers[event] = handler
        self.runtime.on(event, handler)
        return fn

    def emit(self, event: str) -> None:
        """Raise ``event`` as a button whose ``on:`` names it would: its ``on()`` handler runs in the
        next frame. What a script or a test presses a button with (a program's ``on:`` runs through
        ``run_program`` instead)."""
        if event not in self._handlers:
            raise KeyError(f"Host.emit: no on() handler for {event!r}")
        self._emitted.append(event)

    def function(self, name: str, fn: Callable | None = None):
        """Make ``fn`` the document's function ``(name a b ...)``: it gets the call's arguments as
        Python values (numbers, str, list, dict, None) and its value goes back (numpy scalars and
        arrays become numbers and lists). It runs inline in the frame, so keep it quick; use
        ``async_function`` for slow work. Usable as a decorator."""
        if fn is None:
            return lambda f: self.function(name, f)
        self.runtime.register_host_function(
            self._pycvc.host_function(name, self._guard_function(fn), is_async=False))
        return fn

    def async_function(self, name: str, fn: Callable | None = None, *, with_call: bool = False,
                       done_channel: str = ""):
        """Make ``fn`` the document's async function ``(name a b ...)``: the calling program parks
        while ``fn`` runs on the app's background task pool (the UI keeps drawing), then gets its
        value; ``(name-async ...)`` returns a future instead. With ``with_call``, ``fn`` gets the
        call first (``call.cancelled()``, ``call.post(value)`` for progress to "<name>.progress").
        With ``done_channel``, the value is also posted there. Usable as a decorator."""
        if fn is None:
            return lambda f: self.async_function(name, f, with_call=with_call,
                                                 done_channel=done_channel)
        self.runtime.register_host_function(
            self._pycvc.host_function(name, self._guard_function(fn), is_async=True,
                                      with_call=with_call, done_channel=done_channel))
        return fn

    def on_frame(self, fn: Callable[[float], Any]):
        """Call ``fn(dt)`` every frame, after the document's programs ran and before the draw: where
        a simulation steps and pushes what it computed. Usable as a decorator."""
        self._frame_hooks.append(fn)
        return fn

    def watch(self, key: str, fn: Callable | None = None, *, tree: bool = False):
        """Call ``fn(value)`` after a frame in which the value at ``key`` changed (with ``tree``,
        anything under it; ``value`` is then ``get_tree(key)``). Checked once a frame, on the
        frame's thread. Usable as a decorator."""
        if fn is None:
            return lambda f: self.watch(key, f, tree=tree)
        path = self._path(key)
        self._watches.append([key, path, tree, fn, self._snapshot(path, tree)])
        return fn

    def _guard_function(self, fn):
        @functools.wraps(fn)
        def call(*args):
            try:
                return _to_dsl(fn(*args))
            except BaseException as e:
                self._function_errors.append(e)  # the cause, if strict raises its warning
                raise

        return call

    # ── the document ─────────────────────────────────────────────────────────────────────────────

    def load(self, path: str, args: Sequence[str] | None = None) -> list[str]:
        """Load (or switch to) the document at ``path``; returns its load warnings. ``args`` is the
        document's command line (its ``args:`` block). From inside a frame it is deferred to the
        frame's end (and returns ``[]``). Raises ``RuntimeError`` when the document doesn't load."""
        self._check_open()
        if self._in_frame:
            self._pending_load = (path, args)
            return []
        a = args if args is not None else self._args
        if a is not None:
            self.runtime.set_document_args(list(a), f"python {os.path.basename(path)}", True)
        warns = list(self.runtime.load(str(path)))
        self.document = str(path)
        for w in warns:
            log.warning("%s", w)
        return warns

    def has_function(self, name: str) -> bool:
        """Whether the document's programs can call ``name`` here."""
        return bool(self.runtime.has_function(name))

    def enable_streaming(self):
        """Register the stream verbs, the ``stream_view`` widget and the ``type: stream`` node (GL
        modes; headless has no stream widgets). Call before ``load()``."""
        if not self._streaming and self.mode != "headless":
            self.runtime.enable_streaming()
        self._streaming = True

    # ── data in ──────────────────────────────────────────────────────────────────────────────────

    def _path(self, key: str) -> str:
        if not key:
            raise ValueError("pycvc_gl.ariadne: the key is empty")
        return key[1:] if key.startswith("/") else f"{self.prefix}.{key}"

    def set(self, key: str, value) -> None:
        """Write ``value`` at ``key``: a number, bool, str or None as its text; a dict or list as a
        subtree (``set_tree``); a 1-D numpy array as a series (``set_series``)."""
        if isinstance(value, (dict, list, tuple)):
            self.set_tree(key, value)
        elif hasattr(value, "ndim") and getattr(value, "ndim", 0) >= 1:
            if value.ndim != 1:
                raise TypeError("Host.set: a numpy array with ndim > 1 is not a value; "
                                "publish_image() shows a picture, set_table() a table")
            self.set_series(key, value)
        else:
            self._pycvc.state_set(self.app, self._path(key), _format(value))

    def __setitem__(self, key: str, value) -> None:
        self.set(key, value)

    def set_many(self, values: Mapping[str, Any], prefix: str = "") -> None:
        """``set()`` each ``key: value`` (under ``prefix.``, if given)."""
        for k, v in values.items():
            self.set(f"{prefix}.{k}" if prefix else k, v)

    def set_tree(self, key: str, value) -> None:
        """Replace the subtree at ``key`` with ``value``, as the DSL's ``(state-set-tree)`` does: a
        dict's keys and a list's indices name child nodes, a list's node holds its length (so
        ``repeat: (state-get "key")`` counts it), and a scalar is a node's value. What ``value``
        doesn't name is removed."""
        path = self._path(key)
        self._check_tree(value, 0)
        self._pycvc.state_remove(self.app, path)
        self._write_tree(path, value)

    def _check_tree(self, value, depth):
        if depth > 64:
            raise ValueError("Host.set_tree: nested deeper than 64")
        if isinstance(value, dict):
            for k, v in value.items():
                _node_name(k)
                self._check_tree(v, depth + 1)
        elif isinstance(value, (list, tuple)):
            for v in value:
                self._check_tree(v, depth + 1)
        elif hasattr(value, "tolist") and not isinstance(value, (str, bytes)):
            self._check_tree(value.tolist(), depth)
        else:
            _format(value)

    def _write_tree(self, path, value):
        if hasattr(value, "tolist") and not isinstance(value, (str, bytes)):
            value = value.tolist()
        if isinstance(value, dict):
            if not value:
                self._pycvc.state_set(self.app, path, "")
            for k, v in value.items():
                self._write_tree(f"{path}.{_node_name(k)}", v)
        elif isinstance(value, (list, tuple)):
            self._pycvc.state_set(self.app, path, str(len(value)))
            for i, v in enumerate(value):
                self._write_tree(f"{path}.{i}", v)
        else:
            self._pycvc.state_set(self.app, path, _format(value))

    def set_table(self, key: str, rows, columns: Sequence[str] | None = None) -> None:
        """Write a table at ``key``: ``rows`` is a list of dicts, or (with ``columns``) a list of
        rows or a 2-D array. Row ``i``'s column ``c`` is at ``key.i.c`` and ``key`` holds the row
        count, so a document lists it with ``repeat: (state-get "key")`` and ``bind: key.{i}.c``.
        The column names are also kept at ``key_columns`` (a list)."""
        if columns is not None:
            cols = [_node_name(c) for c in columns]
            table = []
            for r in (rows.tolist() if hasattr(rows, "tolist") else rows):
                r = list(r)
                if len(r) != len(cols):
                    raise ValueError(f"Host.set_table: a row has {len(r)} values for "
                                     f"{len(cols)} columns")
                table.append(dict(zip(cols, r)))
        else:
            table = [dict(r) for r in rows]
            cols = []
            for r in table:
                cols += [c for c in r if c not in cols]
        self.set_tree(key, table)
        self.set_tree(f"{key}_columns", list(cols))

    def set_series(self, key: str, values, *, fmt: str = ".6g", limit: int | None = None) -> None:
        """Write numbers at ``key`` as the comma-separated text a ``plot:`` draws (oldest first).
        ``limit`` keeps the last ``limit`` of them."""
        vals = values.tolist() if hasattr(values, "tolist") else list(values)
        if limit is not None:
            vals = vals[-int(limit):]
        self._pycvc.state_set(self.app, self._path(key),
                              ",".join(format(float(v), fmt) for v in vals))

    def publish_image(self, name: str, image) -> None:
        """Show ``image`` in the document's ``image:`` widgets with ``src: name``: a numpy array of
        shape (H, W), (H, W, 3) or (H, W, 4), uint8, or float in [0, 1]; row 0 is the top. Replaces
        the last one by that name; ``host.images[name]`` keeps it (also headless, where nothing is
        drawn)."""
        import numpy as np

        arr = np.asarray(image)
        if arr.ndim not in (2, 3) or (arr.ndim == 3 and arr.shape[2] not in (1, 3, 4)):
            raise ValueError("Host.publish_image: expected an (H,W), (H,W,3) or (H,W,4) array, "
                             f"not shape {arr.shape}")
        if arr.dtype != np.uint8:
            if arr.dtype.kind != "f":
                raise TypeError(f"Host.publish_image: dtype {arr.dtype} (uint8, or float in "
                                "[0, 1])")
            arr = (np.clip(arr, 0.0, 1.0) * 255.0 + 0.5).astype(np.uint8)
        if arr.ndim == 2:
            arr = arr[:, :, None]
        if arr.shape[2] == 1:
            arr = np.repeat(arr, 3, axis=2)
        if arr.shape[2] == 3:  # the texture upload wants RGBA
            arr = np.concatenate([arr, np.full(arr.shape[:2] + (1,), 255, np.uint8)], axis=2)
        arr = np.ascontiguousarray(arr)
        self.images[name] = arr
        if self.mode != "headless":
            self.runtime.publish_image(name, self._pycvc.image.from_numpy(arr))

    def open_stream(self, stream_id: str, width: int, height: int, *, codec: str = "rgba8"):
        """Open a live video stream the document plays: ``stream: streams.<stream_id>`` on a
        ``stream_view`` widget or a ``type: stream`` scene node. Returns the ``pycvc`` stream;
        ``stream.publish(frame)`` sends a (height, width, 4) uint8 frame (rgba8). Closed by
        ``close()``."""
        self.enable_streaming()
        s = self._pycvc.stream_open(self.app, stream_id, codec, int(width), int(height))
        self.streams[stream_id] = s
        return s

    # ── data out ─────────────────────────────────────────────────────────────────────────────────

    def get(self, key: str, default: str | None = None) -> str | None:
        """The text at ``key``, or ``default`` when there is no such node."""
        path = self._path(key)
        if not self._pycvc.state_has(self.app, path):
            return default
        return self._pycvc.state_get(self.app, path)

    def __getitem__(self, key: str) -> str:
        v = self.get(key)
        if v is None:
            raise KeyError(key)
        return v

    def __contains__(self, key: str) -> bool:
        return self._pycvc.state_has(self.app, self._path(key))

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

    def get_series(self, key: str):
        """The comma-separated numbers at ``key`` as a float numpy array (empty when none)."""
        import numpy as np

        v = self.get(key) or ""
        return np.array([float(x) for x in v.split(",") if x.strip()], dtype=float)

    def get_tree(self, key: str):
        """The subtree at ``key`` as Python data, the inverse of ``set_tree``: a node whose children
        are 0..n-1 is a list, other parents dicts, and leaves their text (``None`` when there is no
        such node)."""
        path = self._path(key)
        if not self._pycvc.state_has(self.app, path):
            return None
        return self._read_tree(path)

    def _read_tree(self, path):
        kids = list(self._pycvc.state_children(self.app, path))
        if not kids:
            return self._pycvc.state_get(self.app, path)
        if all(k.isdigit() for k in kids) and sorted(int(k) for k in kids) == list(
                range(len(kids))):
            return [self._read_tree(f"{path}.{i}") for i in range(len(kids))]
        return {k: self._read_tree(f"{path}.{k}") for k in kids}

    def snapshot(self, path: str) -> str:
        """Draw a frame and write it to ``path`` (PNG). Not headless."""
        self._need_gl("snapshot")
        self.view.writePNG(str(path))
        return str(path)

    def frame_rgb(self):
        """Draw a frame and return it as an (H, W, 3) uint8 numpy array, row 0 at the top."""
        import numpy as np

        self._need_gl("frame_rgb")
        raw = self.view.frameRGB()
        h, w = self.view.frameHeight(), self.view.frameWidth()
        return np.frombuffer(raw, dtype=np.uint8).reshape(h, w, 3)[::-1].copy()

    def run_program(self, script: str, done: Callable[[str], Any] | None = None) -> None:
        """Queue ``script`` (a DSL program) as one of the document's actions: it runs in the next
        frame, chrooted to the document's prefix; ``done(how)`` gets "" when it finished, else why
        not. Without ``done``, a program that did not finish is reported as a warning."""
        self._check_open()

        def finished(how):
            if done is not None:
                try:
                    done(how)
                except BaseException as e:  # noqa: BLE001 - raised from step()
                    self._errors.append(e)
            elif how:
                self._program_failures.append(f"run_program: {how}")

        self.runtime.run_program(script, finished)

    def run(self, fps: float = 60.0, *, frames: int | None = None,
            until: Callable[[], bool] | None = None, realtime: bool | None = None) -> int:
        """Run frames until the window closes or the document quits (or ``frames`` ran, or
        ``until()`` is true). ``realtime`` paces them at ``fps`` (default: in a window). Ctrl-C goes
        to the document's ``on_signal:`` as SIGINT, and closes it when it has none. Returns the
        document's exit code."""
        dt = 1.0 / fps if fps > 0 else 1.0 / 60.0
        if realtime is None:
            realtime = self.mode == "window"
        n = 0
        while not self.should_close():
            if frames is not None and n >= frames:
                break
            if until is not None and until():
                break
            t0 = time.perf_counter()
            try:
                self.step(dt)
            except KeyboardInterrupt:
                if not self.runtime.post_signal("SIGINT", "os"):
                    self.runtime.request_close()
            n += 1
            if realtime:
                rest = dt - (time.perf_counter() - t0)
                if rest > 0:
                    time.sleep(rest)
        return self.exit_code()

    def step(self, dt: float = 1.0 / 60.0) -> None:
        """One frame: the document's queued actions, residents and host calls, then the
        ``on_frame`` hooks and ``watch`` callbacks, then the draw."""
        self._check_open()
        if self._in_frame:
            raise RuntimeError("Host.step: called from inside a frame (a handler, function or "
                               "hook)")
        self._in_frame = True
        try:
            self.runtime.frame(dt)
            emitted, self._emitted = self._emitted, []
            for event in emitted:
                self._handlers[event]()
            self._collect_warnings()
            self._raise_errors()
            for hook in list(self._frame_hooks):
                hook(dt)
            self._poll_watches()
            self.runtime.render()
            self.frame_count += 1
        finally:
            self._in_frame = False
        if self._pending_load is not None:
            path, args = self._pending_load
            self._pending_load = None
            self.load(path, args)

    def settle(self, max_seconds: float = 10.0, dt: float = 1.0 / 60.0) -> bool:
        """Run frames until the document is idle (no async call running, nothing waiting to run)
        for three frames in a row, or ``max_seconds`` pass (as AppRuntime::settle); True when it
        went idle."""
        deadline = time.monotonic() + max_seconds
        quiet = 0
        while time.monotonic() < deadline:
            self.step(dt)
            quiet = quiet + 1 if self.runtime.idle() else 0
            if quiet >= 3:
                return True
            time.sleep(0.002)
        return False

    def _collect_warnings(self):
        new = list(self.runtime.take_warnings()) + self._program_failures
        self._program_failures = []
        for w in new:
            self.warnings.append(w)
            log.warning("%s", w)
        if new and self.strict:
            cause = self._function_errors[-1] if self._function_errors else None
            self._function_errors.clear()
            raise AriadneError("; ".join(new)) from cause
        if not new:
            self._function_errors.clear()

    def _raise_errors(self):
        if self._errors:
            e = self._errors.pop(0)
            self._errors.clear()
            raise e

    def _snapshot(self, path, tree):
        if not self._pycvc.state_has(self.app, path):
            return None
        return self._read_tree(path) if tree else self._pycvc.state_get(self.app, path)

    def _poll_watches(self):
        for w in self._watches:
            key, path, tree, fn, last = w
            now = self._snapshot(path, tree)
            if now != last:
                w[4] = now
                fn(now)

    # ── state of the host ────────────────────────────────────────────────────────────────────────

    def should_close(self) -> bool:
        """True once the window closed, the document quit, or ``close()`` ran."""
        return self._closed or bool(self.runtime.should_close())

    def request_close(self) -> None:
        if not self._closed:
            self.runtime.request_close()

    def exit_code(self) -> int:
        """The code ``(quit code)`` asked for (0 until something asks to close)."""
        if self._closed:
            return self._exit_code
        return int(self.runtime.exit_code())

    @property
    def lab(self):
        """A ``pycvc_gl.lab.Lab`` over the document's scene: add meshes, markers, paths and fields
        next to what its ``scene:`` declares, and move them every frame. Not headless."""
        self._need_gl("lab")
        if self._lab is None:
            from pycvc_gl.lab import Lab

            self._lab = Lab(app=self.app, scene=self.scene)
        return self._lab

    def frame_bounds(self, lo: Sequence[float], hi: Sequence[float], up=(0.0, 0.0, 1.0)) -> None:
        """Aim the camera at the box ``lo``..``hi`` with ``up`` as up (a document's scene frames
        itself at load; call this after adding nodes of your own). A no-op headless."""
        if self.mode == "headless":
            return
        self.camera.setUpAxis(*[float(v) for v in up])
        self.camera.frameBounds(*[float(v) for v in lo], *[float(v) for v in hi])

    def _need_gl(self, what):
        self._check_open()
        if self.mode == "headless":
            raise RuntimeError(f"Host.{what}: there is no GL in headless mode")

    def _check_open(self):
        if self._closed:
            raise RuntimeError("Host: closed")

    def close(self) -> None:
        """Close the document and the window: wait for async functions still running, then take
        everything down in order. Idempotent. Not from inside a frame."""
        if self._closed:
            return
        if self._in_frame:
            raise RuntimeError("Host.close: called from inside a frame; request_close() instead")
        self._closed = True
        rt = self.runtime
        rt.request_close()
        try:
            rt.wait_idle()  # without the GIL: an async function still running finishes
        finally:
            for s in self.streams.values():
                try:
                    s.close()
                except Exception:  # noqa: BLE001 - closing what we can
                    pass
            self.streams.clear()
            self._lab = None
            self._exit_code = int(rt.exit_code())
            del self.runtime  # ~AriRuntime (without the GIL) before the view it draws through
            rt = None
            self.overlay = None
            self.camera = None
            if self.view is not None:
                self.view.close()
                self.view = None
            self.scene = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def main_args(argv: Sequence[str] | None = None, *, frames: int = 0):
    """The command line every example host takes: ``[--offscreen | --headless] [--frames N]
    [--png PATH] [--width W] [--height H]``. Returns an ``argparse.Namespace`` with ``mode``."""
    import argparse

    p = argparse.ArgumentParser(add_help=True)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--offscreen", action="store_true", help="GL with no window")
    g.add_argument("--headless", action="store_true", help="no GL at all; nothing is drawn")
    p.add_argument("--frames", type=int, default=frames, help="stop after N frames (0: run)")
    p.add_argument("--png", default="", help="write the last frame here (implies --offscreen)")
    p.add_argument("--width", type=int, default=1024)
    p.add_argument("--height", type=int, default=768)
    ns, rest = p.parse_known_args(argv)
    ns.rest = rest
    ns.mode = "headless" if ns.headless else ("offscreen" if (ns.offscreen or ns.png) else
                                              "window")
    if ns.mode != "window" and ns.frames <= 0:
        ns.frames = 30
    return ns
