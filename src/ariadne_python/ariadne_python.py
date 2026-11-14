"""The scripting hosts' document side (ARIADNE.md §9.6): a document's ``python:`` block.

The engine (python_engine.cpp) compiles this into the host and imports it as ``_ariadne_python``
once the interpreter is up. For each document with a ``python:`` block, ``load_document`` runs the
block in a module of the document's own and registers the functions it exports as host functions
on the document's runtime (``pycvc.host_function``), before the load checks ``requires:`` and scans
what the programs call, so both see them. ``(name a b ...)`` in any of the document's programs then
calls the Python function with the arguments and returns its value.

The block is the source alone, or a map::

    python:
      path: [py]                   # sys.path entries, relative to the document
      modules: [sim]               # imported at load, before init:
      export: [sim.step_world]     # functions to export by name (or {name:, as:, async:})
      console: true                # a Python console in this namespace (ariadne_console.py)
      source: |
        @export
        def histogram(values, bins=16): ...

The document's directory is always on sys.path. ``export`` (``@export``, ``@export(name=...)``) is
in the source's namespace; a module imports it with ``from _arihost import export``. So is
``display(obj)``: pycvc_gl.ariadne_display's where a GL host has lent its window, else a print.
``console`` is read by the engine after the block runs (``_ariadne_console.consoles_for``).

The host's app keeps whether a block is loading (``_arihost.loading()``), for the code it calls.
matplotlib's default backend, in a host that has lent a window: see _MatplotlibDefault.
"""

import importlib
import importlib.util
import os
import re
import sys
import types

KEYS = ("source", "modules", "path", "export", "console")

MATPLOTLIB_BACKEND = "module://pycvc_gl.ariadne_mpl"


class _MatplotlibDefault:
    """matplotlib's default backend in a host with a window: on the first ``import matplotlib``,
    once the package has run, figures go to Ariadne windows (pycvc_gl.ariadne_mpl) when a GL host
    has lent its window (ariglx; not arix, ariglx --terminal or ``<host> python``) and nothing chose
    a backend (MPLBACKEND, a matplotlibrc). Set in matplotlib's rcParams, never in os.environ, so a
    subprocess keeps its own default. The finder leaves sys.meta_path once matplotlib has loaded."""

    @classmethod
    def find_spec(cls, fullname, path=None, target=None):
        if fullname != "matplotlib":
            return None
        from importlib.machinery import PathFinder

        spec = PathFinder.find_spec(fullname, path)
        loader = getattr(spec, "loader", None)
        if loader is None or not hasattr(loader, "exec_module"):
            return None  # left to the other finders
        run = loader.exec_module

        def exec_module(module):
            run(module)
            if cls in sys.meta_path:
                sys.meta_path.remove(cls)
            _default_backend(module)

        loader.exec_module = exec_module  # this spec's loader only: find_spec makes a new one
        return spec


def _lent_window():
    """Whether a GL host has lent its window (ariglx): not arix, ariglx --terminal or arisrvx,
    nor ``<host> python``, where nothing is attached."""
    try:
        import _arihost

        return bool(_arihost.attached()) and _arihost.gl() is not None
    except Exception:  # noqa: BLE001 - no host to ask
        return False


def _default_backend(matplotlib):
    try:
        if not _lent_window() or not has_module("pycvc_gl"):
            return
        rc = matplotlib.rcParams
        if rc._get_backend_or_none() is None:
            rc["backend"] = MATPLOTLIB_BACKEND
    except Exception:  # an older matplotlib (no _get_backend_or_none): its own default stands
        pass


sys.meta_path.insert(0, _MatplotlibDefault)


def export(fn=None, *, name=None, is_async=False, with_call=False, done_channel=""):
    """Export ``fn`` to the document: its programs call it as ``(name a b ...)``. ``name`` defaults
    to the function's own; ``is_async=True`` runs it on the app's pool while the calling program
    waits (``with_call`` and ``done_channel`` as ``pycvc.host_function``'s)."""

    def mark(f):
        if not callable(f):
            raise TypeError(f"export: {f!r} is not callable")
        f.__ari_export__ = {
            "name": name or f.__name__,
            "is_async": is_async,
            "with_call": with_call,
            "done_channel": done_channel,
        }
        return f

    return mark(fn) if fn is not None else mark


def display(obj, **kwargs):
    """Show ``obj`` in an Ariadne window: its richest form, as IPython's display() would
    (pycvc_gl.ariadne_display.display, imported at the first call). Only where a GL host has lent
    its window, or ``host=`` names a Host: arix, ariglx --terminal and arisrvx have no window to
    show it in (and a server host's runtime is the relay's), so there it prints the text/plain
    form, as IPython's display() does outside a notebook, and returns None."""
    if kwargs.get("host") is None and not _lent_window():
        print(_text_plain(obj))
        return None
    from pycvc_gl.ariadne_display import display as show

    return show(obj, **kwargs)


def _text_plain(obj):
    """``obj``'s text/plain form: its ``_repr_mimebundle_``'s, else ``repr()``."""
    if getattr(type(obj), "_repr_mimebundle_", None) is not None:
        try:
            bundle = obj._repr_mimebundle_(include=None, exclude=None)
            if isinstance(bundle, tuple):
                bundle = bundle[0]
            if isinstance(bundle, dict) and bundle.get("text/plain") is not None:
                return str(bundle["text/plain"])
        except Exception:  # noqa: BLE001 - a broken formatter offers nothing, as in IPython
            pass
    return repr(obj)


def describe(exc):
    """An exception from a document's Python as text, its traceback starting where the document's
    code is: the frames of this module (the engine's) left out."""
    import traceback

    tb = exc.__traceback__
    while tb is not None and tb.tb_frame.f_code.co_filename == "ariadne_python.py":
        tb = tb.tb_next
    return "".join(traceback.format_exception(type(exc), exc, tb)).rstrip()


def has_module(name):
    """Whether ``name`` is importable: the ``python.<name>`` capability token."""
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def _names(value, what):
    if value is None or value == "":
        return []
    if isinstance(value, (str, dict)):
        return [value]
    if isinstance(value, list):
        return value
    raise ValueError(f"python: {what}: is a name or a list of them")


def _flag(value):
    return str(value).lower() in ("true", "1", "yes", "on")


def document_module_name(path):
    """The module the ``python:`` block of the document at ``path`` runs in: ``__ari__.<stem>``."""
    stem = re.sub(r"\W", "_", os.path.splitext(os.path.basename(path))[0]) or "document"
    return f"__ari__.{stem}"


def _under(file, dirs):
    if not file:
        return False
    file = os.path.normcase(os.path.abspath(file))
    return any(file.startswith(os.path.normcase(d) + os.sep) for d in dirs)


def _resolve(target, ns):
    if "." not in target:
        if target not in vars(ns):
            raise NameError(f"python: export: the block defines no '{target}'")
        return vars(ns)[target]
    module, attr = target.rsplit(".", 1)
    obj = getattr(importlib.import_module(module), attr, None)
    if obj is None:
        raise NameError(f"python: export: module '{module}' has no '{attr}'")
    return obj


def _marked(namespace, module_name):
    for obj in list(vars(namespace).values()):
        if (
            callable(obj)
            and hasattr(obj, "__ari_export__")
            and getattr(obj, "__module__", None) == module_name
        ):
            yield obj


def pycvc_register():
    """The ``register`` the engine hands load_document: each export becomes a
    ``pycvc.host_function`` on the document's runtime, which the host lends through ``_arihost``.
    pycvc is imported at the first export, so a block that exports nothing doesn't need it."""
    held = {}

    def register(name, fn, is_async, with_call, done_channel):
        if not held:
            try:
                import pycvc
            except ImportError as e:
                raise ImportError(
                    "python: exporting a function needs pycvc (install pycvc-cp312, or name its "
                    "directory with --python-path)"
                ) from e
            import _arihost

            app = pycvc.app_from_capsule(_arihost.app())
            held["pycvc"] = pycvc
            held["runtime"] = pycvc.ari_app_attach(app, _arihost.runtime())
        pycvc = held["pycvc"]
        held["runtime"].register_host_function(
            pycvc.host_function(name, fn, is_async, with_call, done_channel)
        )

    return register


def load_document(block, path, register, export_fn=None):
    """Run the ``python:`` block of the document at ``path`` and export its functions through
    ``register(name, fn, is_async, with_call, done_channel)``. Returns the load's warnings.
    ``export_fn`` is the ``export`` predefined in the block's source (this module's by default; a
    server host's is pycvc.ariadne.export, whose functions it runs on its script thread)."""
    warnings = []
    if isinstance(block, str):
        block = {"source": block}
    if not isinstance(block, dict):
        raise ValueError("python: is the block's source, or a map of source:, modules:, path:, export:")
    for key in block:
        if key not in KEYS:
            warnings.append(f"python: unknown key '{key}' ignored (it takes {', '.join(KEYS)})")
    here = os.path.dirname(os.path.abspath(path))
    dirs = [here] + [
        os.path.normpath(os.path.join(here, p)) for p in _names(block.get("path"), "path")
    ]
    for d in reversed(dirs):
        if d not in sys.path:
            sys.path.insert(0, d)

    # The document's own module, fresh on every load: a reload runs the block again.
    ns = types.ModuleType(document_module_name(path), f"the python: block of {path}")
    ns.__file__ = os.path.abspath(path)
    ns.export = export_fn or export
    ns.display = display
    sys.modules[ns.__name__] = ns
    source = block.get("source")
    if source:
        exec(compile(source, f"{path} (python: source)", "exec"), vars(ns))

    modules = []
    for name in _names(block.get("modules"), "modules"):
        mod = sys.modules.get(name)
        if mod is not None and _under(getattr(mod, "__file__", None), dirs):
            mod = importlib.reload(mod)  # the document's own module: pick up its edits
        else:
            mod = importlib.import_module(name)
        modules.append(mod)

    exports = {}  # the DSL name -> (fn, options); a later one replaces an earlier one
    for fn in _marked(ns, ns.__name__):
        exports[fn.__ari_export__["name"]] = (fn, fn.__ari_export__)
    for mod in modules:
        for fn in _marked(mod, mod.__name__):
            exports[fn.__ari_export__["name"]] = (fn, fn.__ari_export__)
    for entry in _names(block.get("export"), "export"):
        if isinstance(entry, str):
            entry = {"name": entry}
        if not isinstance(entry, dict) or not entry.get("name"):
            raise ValueError("python: export: an entry is a name or {name:, as:, async:}")
        fn = _resolve(str(entry["name"]), ns)
        if not callable(fn):
            raise TypeError(f"python: export: '{entry['name']}' is not callable")
        options = dict(getattr(fn, "__ari_export__", {}))
        options["name"] = str(
            entry.get("as") or options.get("name") or str(entry["name"]).rsplit(".", 1)[-1]
        )
        if "async" in entry:
            options["is_async"] = _flag(entry["async"])
        exports[options["name"]] = (fn, options)

    for name, (fn, options) in exports.items():
        register(
            name,
            fn,
            bool(options.get("is_async")),
            bool(options.get("with_call")),
            str(options.get("done_channel") or ""),
        )
    return warnings
