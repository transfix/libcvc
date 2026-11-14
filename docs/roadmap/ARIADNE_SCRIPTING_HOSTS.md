# Ariadne scripting hosts: Python inside ari, arigl and arisrv

Status:
- **P1 shipped** in transfix/libcvc#985 (d0bb057d, released in v3.7.11): `arix` and `ariglx`,
  natively, with Linux packages on cvcpkg.org. §3 and §4 describe what shipped and mark what
  didn't. The user-facing reference is [ARIADNE.md](../ARIADNE.md) §9.6.
- **S0 shipped** in transfix/libcvc#1046: `arisrv --doc` and `server: publish:` (§6.1, §6.5.3).
- **S1 shipped** in transfix/libcvc#1081: `arisrvx`, the operator's Python beside the relay on a
  script thread with one asyncio loop, `serve_asgi` and `--http-listen`, lifespan and graceful
  shutdown, `arisrvx check`, and its recipe (§6.2, §6.5.1, §6.5.2, §6.5.9). The reference is
  ARIADNE.md §9.8.
- **S2 shipped** in transfix/libcvc#1122 (`room-call`, relay joins, TLS, a server document's
  `python:` block), and **S5** in transfix/libcvc#1130 (the authenticate hook and principals, with
  the client's `token: oidc` and reconnects).
- **S6 is built:** `host.read_scope`, a per-connection read scope on both planes, and
  `host.refresh_scope` (§6.5.6).
- **P1b's engine interface shipped:** `ScriptEngine` and its lane, with Python moved onto it
  (§3.1, §8.3). It is what G1 and L1 plug into.
- **G1's first part: WebAssembly guests natively on wasmtime,** in `arix` and `ariglx`
  ([`src/ariadne_wasm`](../../src/ariadne_wasm/wasm_engine.h); ARIADNE.md §9.6). §8.3 says what
  it built and what G1 has left.
- **The rest is design** (2026-11-02; §4.6, §5.5, §5.6 and §8.2–8.9 added 2026-11-03;
  §6.5, application services: an ASGI API and live state from one `arisrvx`, and phases
  S5–S7 added 2026-11-05).
- Each phase in §10 becomes one or more PRs, and the reference moves into ARIADNE.md as each one
  lands.

Python can *host* a document: `pycvc_gl.ariadne.Host` (#985) lets `python app.py` open a window,
load a `.ari` document and give it Python functions to call. Since P1 the other direction works
natively. In `arix` and `ariglx` a document can carry Python, and a Python program can be the
host's entry point. arisrv runs DSL server documents (S0), and `arisrvx` runs the operator's Python
beside the relay (S1).

This roadmap adds host variants that embed a CPython interpreter, with pycvc, pycvc_gl, numpy
and the science packages, natively and in one WebAssembly module:

- A document can carry Python (a `python:` block, or modules beside it) and call it from its
  programs.
- A Python script can be the entry point (`ariglx run app.py`). It is the same script that
  runs under `python app.py` with #985 today.
- The same documents and scripts run natively and in the browser. That is what puts the Python
  Ariadne demos and GRL-SNAM's demos in an `arigl` page.
- arisrv can run server-side Python. Clients can ask it to run code without being able to run
  arbitrary code or take the relay down (§6).
- Python output shows up in documents: matplotlib figures as interactive images, and anything
  that renders in Jupyter (§4.6).
- Lua, and plugins compiled to WebAssembly, can come later through the same seams, without
  another set of binaries (§8).

VolRover3 is the precedent: an embedded interpreter, a host module that hands scripts the
running app, a console. Going forward, VolRover should be as much a plain Ariadne app on these
hosts as it can be, with C++ or Python helpers only where it needs them (§7).

## 1. What exists

Every item was checked against master `dded8e56` and cvcpkg.org on 2026-11-02, and the P1 rows
against `f88eb3dd` (v3.7.11) on 2026-11-06.

| Piece | Where | State |
|---|---|---|
| CPython 3.12, static, for wasm | `python312` | wasm and wasm-mt `3.12.10+cvc.24` (2026-11-09). Since cvc.23, `libpython3.12.a` has pyexpat's expat and `_sha2`'s HACL SHA-2 (cy-pca/cvcpkg#565, #567); earlier wasm bundles left them out. cvc.25 (cy-pca/cvcpkg#581) renames two expat internals that clash with VTK's expat, which a link with vtk-python needs (§5.1) |
| numpy for wasm | `numpy-cp312` 2.4.6 | wasm and wasm-mt `+cvc.6` (2026-11-07). Its C extensions are relocatable wasm objects, each in a namespace of its own and listed in `share/python-wasm/numpy-cp312.inittab` (§5.5), linked statically and registered as builtins ([`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh)) |
| vtk-python for wasm | `vtk-python-cp312` 9.5.0 | wasm and wasm-mt `+cvc.8` (2026-11-05, cy-pca/cvcpkg#543 and #544): rendering, IOHDF, IONetCDF |
| pycvc + pycvc_gl for wasm | `pycvc-gl-cp312` 3.7.8 | wasm only (87 MB of static archives). [`pycvc_host.cpp`](../../bindings/pycvc/wasm/pycvc_host.cpp) embeds them with numpy and vtk-python and prints `PYCVC_VTK_BRIDGE_OK`, **under node only**: no browser, no window |
| Python drives a document | [`pymod_gl/ariadne.py`](../../bindings/pycvc/pymod_gl/ariadne.py) (#985, merged) | `Host` (window / offscreen / headless / embedded), functions with arguments and values, the GL-free `AriApp`, and GIL release in both directions |
| The scripting hosts (P1) | `arix`, `ariglx`: `src/ari`, `src/arigl` with [`src/ariadne_python`](../../src/ariadne_python) | Native. `arix` and `ariglx` 3.7.11 are on cvcpkg.org for Linux. No `arisrvx` |
| The browser host (W1) | `ariglx-web`: [`cvcpkg/recipes/ariglx-web`](../../cvcpkg/recipes/ariglx-web/recipe.yaml) | wasm and, since W3, wasm-mt (§5.4); the lean tier, checked in headless Chrome (§5.1). Not on cvcpkg.org yet |
| An app with an embedded interpreter | volrover `volrover3/docs/EMBEDDED_PYTHON.md` | `EmbeddedInterpreter`: one interpreter per process, GIL parked after boot, the live app handed to a host module (`vrhost`) by PyCapsule, a console dock. Qt-bound |
| The hosts | `src/ari`, `src/arigl`, `src/arisrv`; `arigl-web` | `ari` = libcvc + FTXUI/tvision. `arigl` = + cvcGL. `arisrv` = the conference relay, core libcvc only; since S0 it runs a DSL server document (`--doc`), no Python. `arigl-web` = wasm and wasm-mt pages |
| Native pycvc packages | `pycvc-cp312`, `pycvc-gl-cp312` | **Linux only**. No Windows or macOS, so the `arix` and `ariglx` packages are Linux-only too |
| Pure-Python packages (noarch, so usable in wasm) | cvcpkg | `restrictedpython-cp312` 8.4, `jedi`, `sympy`, `networkx`, `imageio`, `trimesh` |
| Science packages not on wasm | cvcpkg | scipy (Fortran), pandas, matplotlib (+ contourpy, kiwisolver), pillow, h5py, torch |
| Lua | `lua` 5.4.9 | every platform, wasm included. `luajit` native only. No libcvc or cvcGL bindings |
| WebAssembly runtimes (checked 2026-11-03) | `wasmtime` 45.0.0, `wamr` 2.4.4, `wasmer` 7.1.0, `wasmedge` 0.17.0 | Linux, macOS and Windows. `wasmtime` and `wasmer` repackage upstream's prebuilt C API, so no Rust build; WAMR and WasmEdge build from source |

**Verdict: feasible.** The parts that could have killed it are proven: CPython, numpy and
vtk-python as static wasm, pycvc_gl in the same module, and a Python host for documents. What's
new is integration work:

- an engine that boots Python inside a host and hands it the running runtime;
- a document schema for Python, and a trust rule for it;
- one combined wasm link, run in a real browser;
- a sandboxed script service on arisrv.

The risks are in §9.

## 2. Names

**`arix`, `ariglx`, `arisrvx`, and the browser package `ariglx-web`** (chosen in review).

- The `x` marks the extended host: the stock runtime plus the scripting engines. Python comes
  first. Lua or another language joins the same binaries (§8), so the name doesn't go stale.
- The base name says which runtime, as `arigl-web` already does: `ariglx` is arigl plus the
  engines, and `ariglx-web` is its browser build.
- The binary name, the cvcpkg package name and the CMake target are the same.
- What forces separate binaries is a pinned CPython: an ABI, tens of MB of stdlib and packages
  natively, and a browser module several times arigl-web's size. The stock hosts must not carry
  any of it.
- There is no `-cp312` in the name. The host is an application that bundles its interpreter, so
  it pins one CPython (3.12 now), as VolRover3 does. Nobody imports it into their own Python.
- Documents never name a binary. They ask for `python` (later `lua`) in `requires:` (§4.1). The
  `arix` and `ariglx` names are published now (v3.7.11).

Considered and dropped:

- `ari-py`, `arigl-py`, `arisrv-py`: they say Python, so a Lua engine would make them wrong.
- `aripy`, `ariglpy`, `arisrvpy`: the same problem, and `ariglpy` is hard to read.
- `pyari`, `pyarigl`: these read as Python packages for ari, which is what `pycvc_gl.ariadne`
  already is.
- `daedalus`, `daedalus-gl`, `daedalus-srv`: a brand of its own, but long and far from `ari`.

`arisrvx` links pycvc core, not pycvc_gl: the relay has no GL.

## 3. Architecture

### 3.1 One engine, compiled into each `x` host

As built in P1 (#985):

- **The engine** is [`src/ariadne_python/`](../../src/ariadne_python): `python_engine.h/.cpp`,
  plus `ariadne_python.py`, compiled in as a source string.
  - `cvc_ariadne_python_host(<target>)` (`ariadne_python.cmake`) compiles it into a host
    executable, links `Python3::Python` (the embed component) and defines `CVC_ARIADNE_PYTHON`.
    There is no separate library.
- **Building.**
  - `-DCVC_BUILD_PYCVC=ON` in-tree, or `-DARI_WITH_PYTHON=ON` / `-DARIGL_WITH_PYTHON=ON` for a
    standalone recipe build, adds the `arix` / `ariglx` targets. `-DARISRV_WITH_PYTHON=ON` adds
    `arisrvx`, in-tree too, since it needs no pycvc (S1).
  - In-tree, a target is skipped when Python's Development.Embed isn't found. With
    `-DARI_WITH_PYTHON=ON` / `-DARIGL_WITH_PYTHON=ON`, a missing Development.Embed is a configure
    error.
  - Nothing is built under Emscripten.
- **`cvc::ariadne::python::Engine`** (`EngineConfig`, `HostHandles`, `Attachment`, `attach()`,
  `run_script()`, `python_main()`) boots CPython once per process:
  - CPython's regular config: `PYTHONPATH` and the other variables apply, `parse_argv` is off,
    and no signal handlers are installed;
  - the home is `PYTHONHOME` if set, else the prefix beside the executable when it holds the
    stdlib (`lib/python3.12/os.py`; `Lib/os.py` on Windows), else, in-tree, the Python it was
    built against;
  - `--python-path DIR` adds directories ahead of `PYTHONPATH`. `<host> python` doesn't take it.

  The lifecycle is VolRover3's `EmbeddedInterpreter`'s: initialize on the main thread, park the
  GIL, finalize once.
- **The `_arihost` builtin module,** registered with `PyImport_AppendInittab`. `pycvc_gl.ariadne`
  reads it (§3.2).
  - `app()`, `runtime()` and `gl()` return capsules. `gl()`'s are the `AriRuntime`,
    `SceneRenderer`, `CameraController` and `ImGuiOverlay`, or None.
  - `attached()`, `host()`, and `mode()` (`window`, `offscreen`, `terminal` or `headless`).
  - `run()`: the host's loop, with the GIL released, once.
  - `add_frame_hook(fn)`, and `export`.
- **What it installs:** Python's `ScriptEngine` ([`script_engine.h`](../../inc/cvc/ariadne/script_engine.h),
  P1b; §8.3 describes the interface).
  - Process-wide, through `register_script_engine`: the `python` tokens (§4.1), answered by the
    engine's `has_module`, and the `python:` block key (§4.2).
  - Per runtime, through `attach()`: the host's handles lent to Python, raised action caps (§4.2),
    and `AppRuntime::use_script_engine`, so that runtime's loads hand the block to the engine's
    `load_block`.
  - Exports as host functions, through `pycvc.host_function` and `register_host_function`, from
    Python. That is the lane's one function registry (`AppRuntime::register_function`).
- **Not built:** the `py-eval` / `py-exec` verbs.

  Before P1b, the seams were a token registry keyed by language (`set_script_engine`) and
  `AppRuntime::add_load_hook` / `add_frame_hook`. Lua (§8.1) and wasm guests (§8.2) plug into
  `ScriptEngine` beside Python.

Each host keeps its one `main.cpp`, and the `x` target compiles it with `CVC_ARIADNE_PYTHON`
defined:

| Host | Is | Links | Runtime packages |
|---|---|---|---|
| `ariglx` | `arigl` | the engine, libpython | `python312`, `numpy-cp312`, `pycvc-cp312`, `pycvc-gl-cp312` |
| `arix` | `ari` | the engine, libpython | `python312`, `numpy-cp312`, `pycvc-cp312`; a Python program also needs `pycvc-gl-cp312`, since `Host` is pycvc_gl's |
| `arisrvx` | `arisrv` | the engine with its server part (`cvc_ariadne_python_host(arisrvx SERVER)`: `pycvc.ariadne` built in, the ServerLink), libpython | `python312`; for its apps, `fastapi-cp312`, `uvicorn-cp312` and the rest of §6.5's stack. No pycvc |

Natively `ariglx` links the prefix's shared `libcvc`, `libcvcGL` and `libpython3.12`, and imports
pycvc and pycvc_gl from the prefix's site-packages; `arix` links `libcvc` and `libpython3.12`. So
the `_pycvc_gl` extension that Python imports in `ariglx` binds to the **same** cvcGL as the
host: one app, one scene graph, one ImGui context. A second copy of cvcGL would mean a second
ImGui context, and raw ImGui calls from the wrong copy crash. In wasm everything is one static
module anyway (§5).

A runtime-loaded Python plugin for the stock binaries was considered for native builds. wasm
has to be a separate link regardless, and separate binaries keep Python out of the stock
packages' install closure, so the variants are separate binaries everywhere.

### 3.2 The host module and `Host`'s embedded mode

`pycvc_gl.ariadne.Host` has a fourth mode, `embedded` (P1):

- **Inside an `x` host,** `Host()` defaults to `embedded`: a Host over the host's own runtime
  (from `_arihost`), not a new window. Its `mode` reads `window` or `offscreen` as the host's
  does, or `headless` where there is no GL (`arix`, `ariglx --terminal`).
- **Other modes.**
  - An explicit `window`, `offscreen` or `headless` mode opens a separate Host beside the host's.
  - `mode="embedded"` outside a scripting host raises.
  - A second embedded `Host()` raises too: use `current_host()`.
- **`pycvc_gl.ariadne.current_host()`** returns the embedded Host. It returns `None` under plain
  `python`, and under `<host> python`, where nothing is attached.
- **The rest of the API is unchanged:** `function`, `async_function`, `on_frame`, `watch`, `set`,
  `set_tree`, `set_series`, `publish_image`, `get_tree`, `run_program`, `host.lab`.
- **`run()`.**
  - A bare `run()` runs the host's loop natively. With `frames=`, `until=` or `realtime=` it
    steps the frames itself.
  - A program may instead load its document and return, and the host runs it.
  - `load()` from inside a frame raises: load before `run()`.
  - In the browser the page will own the loop (§5.3), so `run()` will return at once there, and
    the work goes in `on_frame` and in functions.

So one script runs both ways: `python app.py` (#985) and `ariglx run app.py`. GRL-SNAM's
Ariadne ports already have the right shape: `setup()` builds the scene and `step(dt)` drives
it.

### 3.3 Threads and the GIL

#985's rules hold:

- A frame releases the GIL, and each Python call takes it.
- Async functions run on the app's background pool and take the GIL for the call.
- Single-threaded wasm has no pool thread, so async functions run in the drain, one step per
  frame, as the media cache does (ARIADNE_MEDIA.md §2).
- wasm-mt has real threads: `python312` wasm-mt is built with `-pthread`.

There is one interpreter per process. Sub-interpreters are out because numpy and the SWIG
modules aren't sub-interpreter safe. VolRover3 reached the same conclusion
(EMBEDDED_PYTHON.md §6).

## 4. Documents that carry Python

### 4.1 `requires:` tokens

| Token | Holds when |
|---|---|
| `python` | the host embeds an interpreter |
| `python.<module>` | `<module>` is importable. Checked at load with `importlib.util.find_spec`: a top-level module isn't imported, but a dotted one imports its parent package. For example `python.numpy`, or `python.vtkmodules` for the full browser tier (§5.2) |
| `python.console` | the console widget (§4.5). Not built yet |

- `(has? "python")` works as for any token, and `ari info` lists the tokens under its
  capabilities.
- On a stock host, a document that requires `python` fails with an error naming the scripting
  hosts, arix, ariglx or arisrvx.
- A stock host ignores a `python:` block, with a warning.

### 4.2 The `python:` block

```yaml
requires: [python, python.numpy]
python:
  path: [py]                     # sys.path entries, relative to this document
  modules: [sim]                 # imported at load, before init:
  export: [sim.step_world]       # module functions the document's programs may call
  source: |
    import numpy as np
    from pycvc_gl.ariadne import export

    @export
    def histogram(values, bins=16):
        h, edges = np.histogram(values, bins)
        return {"counts": h.tolist(), "edges": edges.tolist()}
init: (state-set-tree "hist" (histogram (state-get "samples")))
```

- **When.** The block runs on every load, a reload too: after the parse, before `requires:` is
  checked and `init:` runs. It runs in a fresh module of its own, `__ari__.<stem>`.
- **Keys.** `source`, `modules`, `path` and `export`, or a bare source string. Other keys warn.
  The document's own directory is always on `sys.path`.
- **Exports.**
  - `@export` is predefined in `source`. A module imports it with `from _arihost import export`
    or from `pycvc_gl.ariadne`.
  - `@export` and the `export:` entries (a name, or `{name, as, async}`) become host functions.
  - `@export(is_async=True)` makes an async function; `name=`, `with_call=` and `done_channel=`
    are as `pycvc.host_function`'s.
  - Values cross through #985's conversion: numbers, strings, lists, dicts, and numpy arrays as
    lists or series.
  - Exporting needs pycvc in the prefix.
- **Errors.** A raise inside a call is a `PythonError` in the DSL, which a `try` can catch
  ([STATE_EXEC_EXCEPTIONS.md](../STATE_EXEC_EXCEPTIONS.md)). A raise at load fails the load and
  prints the traceback.
- **Caps.** #985 found that the default action caps (`ariadne.limits.action_max_time` 0.1 s,
  `action_max_steps` 5000; ARIADNE.md §2.2) stop any slow Python call, so `Host` raises them.
  An `x` host raises them for every document it runs, Python or not (`action_max_time` 30 s,
  `action_max_steps` 0), unless the tree already sets them (`--set`).
- **`load:`ed modules: planned (P1b).** A module's `python:` block would run in its own
  namespace, with its exports visible only inside that module, like its `functions:` (ARIADNE.md
  §6.6), and granted as `functions:` are. P1 runs only the top document's block, and its exports
  are host functions for the whole runtime.
- **Reload.** A reload runs the block again in a fresh module. It reloads the `modules:` whose
  files are under the document's directory or its `path:` entries. Packages from the prefix are
  not reloaded.

### 4.3 Trust

Natively, Python in a document runs with the user's full authority: it can `import os`. Unlike
the DSL it has no chroot. So:

- **Native, as built (P1): no gate is needed yet.** Python comes only from a local top
  document: natively a document by URL is refused, and a `load:`ed module's block doesn't run. It
  runs with the user's authority, so run a document with Python only when you would run the
  program (ARIADNE.md §9.6).
- **Native, planned with module blocks (P1b).** A document fetched over http(s), or a `load:` of
  one, that has a `python:` block fails to load unless `--python-allow ORIGIN` names its origin,
  as `--net-allow` does for the network. `ari check` says which origin it would need.
- **Browser.** The wasm sandbox is the boundary, as it is for the page's own JavaScript. The
  page's `ariadne.config.json` lists any origins beyond its own in `python_origins`.
- **Untrusted Python** (a snippet a user typed, a shared notebook cell) never goes through
  `python:`. It uses the restricted engine from §6.4.

### 4.4 A script as the entry point

- **`ariglx run app.py [args]`** runs `app.py` as `__main__`.
  - `sys.argv` is `[app.py, args...]`, and the script's directory goes first on `sys.path`.
  - Typically the script does `host = Host()`, registers functions, then `host.load("app.ari")`
    and `host.run()`.
  - `snapshot` and `check` take a script too, and so does `arix`. A program under `arix` needs
    `pycvc-gl-cp312` installed beside it, for `Host`.
  - Not under `ariglx --terminal`.
- **`ariglx run -m package.module [args]`: planned (P1b).** It would do the same for a module.
  Today `ariglx python -m grl_snam.ariadne live austin-freedrive` runs one, but with no host
  attached, so its `Host()` opens a window of its own.
- **`ariglx run main.ari`** loads a document as usual.
- **`ariglx python [args]`** is the embedded interpreter as a plain `python3` (a REPL, `-c`,
  `-m`), so scripts and tests use exactly the host's packages, e.g. `ariglx python -m pytest`.

### 4.5 The console widget

Not built yet (P2).

- `py_console:` is an input line plus a scrollback, bound to the engine: ImGui in a window,
  `text_view` / `message_input` in a terminal.
- `py_editor:` comes later: ImGuiColorTextEdit, with completion from jedi (noarch on cvcpkg, so
  it works in the browser too).

These replace VolRover3's `PyConsoleDock`. A console runs full Python, so only a trusted
document (§4.3) can mount one.

### 4.6 Charts and rich output

**What other environments do:**

- **Jupyter.** An object offers formats through `_repr_png_`, `_repr_svg_`, `_repr_html_`, ...,
  or all at once through `_repr_mimebundle_`. The kernel sends that bundle, and the front end
  shows the richest type it can render. A `display_id` lets later output update it in place.
- **matplotlib inline.** At the end of a cell, every open figure is rendered to PNG, displayed
  and closed.
- **Interactive matplotlib.** ipympl, Pyodide 0.28's default backend and marimo all use
  matplotlib's WebAgg protocol:
  - the Agg canvas goes out as full or diff PNG frames;
  - mouse and key events come back as JSON, which becomes `MouseEvent(...)._process()`;
  - `NavigationToolbar2` does pan and zoom.

**In the `x` hosts:**

**As built (C1, first part), in `ariglx`:** the backend is `module://pycvc_gl.ariadne_mpl`, since
`pycvc_gl.ariadne` is a module, not a package. ariglx makes it the default through a finder on
matplotlib's first import. A program with no document gets the shipped `pycvc_gl/figures.ari`, one
window per figure. A `plt.show()` during a load, a call the document makes or a frame neither
blocks nor loads a document, and the host refuses a loop inside its loop, a load or another
thread (`_arihost.run`; the app keeps that state, `_arihost.loading()`). `bind(fig, name)` gives a
document's `image:` the figure. `display()` (`pycvc_gl.ariadne_display`) draws figures, numpy
pictures, PIL images and PNG/JPEG bundles, and shows other forms as text; with no figures.ari
window (a document of its own, `arix`, `arisrvx`) it prints. A plain `python` Host is given to the
figures (`ariadne_mpl.show_in(host)`, `attach(host)`): no process-wide default host, which pyplot's
process-wide figures would want; that waits on a decision. Not yet: interaction (C2), the
browser, VTK and mesh output.

- **A matplotlib backend, `pycvc_gl.ariadne.mpl`.** It is the default inside an `x` host, or
  selected with `matplotlib.use("module://pycvc_gl.ariadne.mpl")`.
  - It's a `FigureCanvasAgg`, and a draw publishes `buffer_rgba()` to an `image:` widget with
    `Host.publish_image`.
  - Python and the window share a process, so nothing is PNG-encoded: the pixels go straight to
    the widget's texture.
- **Figures.**
  - `plt.show()` opens each figure as an Ariadne window holding that image, so an unmodified
    matplotlib script works.
  - A figure can instead be bound to a named image that the document's own `image:` widget shows.
- **Interaction.**
  - Any widget, `image:` included, already takes `on_click`, `on_hover` and `on_drag*`. The
    handler's `event` has the pointer in the item's own pixels (ARIADNE.md §4.6).
  - The backend turns those into matplotlib mouse events (y flipped, in physical pixels).
    `NavigationToolbar2` then does pan, zoom and home with no toolbar widgets, and Ariadne buttons
    call its `pan()`, `zoom()`, `home()` and `back()`.
  - For parity Ariadne needs four small additions:
    - a per-item wheel event;
    - the item's size in the event, to fit the figure to the widget at the device pixel ratio;
    - keys while the item is hovered;
    - the cursor shape.
- **Frames and threads.**
  - `draw_idle` coalesces into the next frame (`on_frame`), and `new_timer` runs on the same hook.
  - matplotlib isn't thread-safe. All of this runs on the runtime thread, which is where the
    engine runs Python anyway.
- **`display(obj)`.** It takes the object's MIME bundle and shows what Ariadne can render:
  - `image/png` or an RGBA array as an image;
  - `text/markdown` and `text/plain` as text;
  - a VTK object or a mesh in the scene.

  So pandas, PIL and sympy objects that already render in Jupyter show up with no Ariadne code.
  `py_console` (§4.5) shows its results this way, which makes it notebook-like.

**Not planned:**

- **HTML and JavaScript outputs** (Plotly, Bokeh, Vega) need a browser engine. In `ariglx-web` a
  DOM overlay could host them later.
- **Considered:**
  - ImPlot: GPU plots inside ImGui, fast for live data, and a candidate to back the `plot:`
    widget;
  - VTK's charts in the 3D scene (`vtkContextActor`), which lose tooltips and pan/zoom there.
- **A chart shared with a conference room.** `publish_image` is local to its host. Sharing a
  chart means sending frames, which is where WebAgg's diff PNGs pay off.

**In the browser** matplotlib needs its C extensions for wasm (§5.5):

- `_backend_agg`, `ft2font` (with FreeType, already on cvcpkg for wasm), `_image`, `_path`,
  `_tri`, `_qhull` and `_c_internal_utils`;
- kiwisolver, contourpy, Pillow and the fonts.

That is about 12 MB compressed in Pyodide. Ship a prebuilt font list, as Pyodide does, so the
first import doesn't scan for fonts.

## 5. The browser: `ariglx-web`

### 5.1 One module

A recipe `ariglx-web` (`cvcpkg/recipes/ariglx-web/build-wasm.sh`) does:

- arigl-web's closure compile from the repo root, with `-DCVC_ARIADNE_PYTHON=ON`;
- the Python link from [`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh):
  `libpython3.12.a`, `_pycvc.a` and `_pycvc_gl.a` whole-archived, numpy's extension objects with
  the generated inittab registrar, and `_vtkmodules_static.a` in the full tier;
- the cvcGL browser contract `link-host.sh` skipped because it only ran under node:
  `webgl_state_shadow.js`, and mimalloc on wasm-mt ([CVCGL_WASM.md](../CVCGL_WASM.md)).

It ships arigl-web's page files plus `python312.zip`: the stdlib and the pure-Python packages,
fetched before start and imported through zipimport.

**What W1 built.**
- **The target.** `ariglx` is the arigl-web branch's second target in
  [`src/arigl/CMakeLists.txt`](../../src/arigl/CMakeLists.txt). It is built when the bindings are
  on (`CVC_BUILD_PYCVC`), which on Emscripten makes the SWIG modules static archives.
- **The link.** `cvc_ariadne_python_wasm_host` in
  [`ariadne_python.cmake`](../../src/ariadne_python/ariadne_python.cmake) links:
  - libpython3.12.a;
  - pycvc and pycvc_gl, whole-archived;
  - every extension object the deps' `share/python-wasm/*.inittab` manifests list (§5.5). A
    numpy from before the manifests (cvc.5) is localized at configure time.
  It also generates `cvc_ariadne_python_register_builtins()`, which the engine calls before the
  interpreter boots.
- **The zip.** [`make_python_zip.py`](../../src/ariadne_python/make_python_zip.py) writes it,
  stored, not deflated, so zipimport needs no zlib. It holds:
  - the stdlib, without its tests or the GUI and build tools;
  - the prefix's packages, `.py` files and package data only;
  - `pycvc.py` and the `pycvc_gl` package.
  The page writes it to `/py/lib/python312.zip`, which CPython puts on `sys.path`.
- **The page** is arigl-web's. `arigl-web.js` reads `data-module="ariglx"` and
  `data-python="python312.zip"` from its script tag, downloads both files together, and writes
  the zip in `preRun`.
- **Tiers.** The recipe builds the lean tier, or the full tier when vtk-python's wasm build is in
  the deps prefix.
- **The check.** ci.yml's `ariglx-web` job runs arigl-web's browser smoke on the page, with
  arigl-web's cases and four of its own (`--pydocs src/arigl/web/test/python`):
  - a `python:` block with numpy and pycvc_gl;
  - a block that parses XML with `xml.parsers.expat` and takes a SHA-256;
  - a Python program in place of the document;
  - a block that raises.
- **The link takes undefined symbols as errors**, as link-host.sh's does. That needs python312
  3.12.10+cvc.23 or later for wasm. The bundles before it archive pyexpat and `_sha2` without the
  objects they call: the bundled expat (52 `PyExpat_XML_*` symbols) and HACL's SHA-2 (12). Until
  cvc.23 both links allowed undefined symbols, and `xml.parsers.expat` and hashlib's SHA-2 aborted
  when called. cvcpkg's version ranges ignore the `+cvc.N` revision, so the recipe can't ask for
  cvc.23. Instead `cvc_ariadne_python_wasm_host` reads the installed bundle's manifest and stops
  the configure on an older one. There is still no OpenSSL `_hashlib`: hashlib uses the builtin
  `_md5`, `_sha1`, `_sha2`, `_sha3` and `_blake2`.
- **The full tier needs python312 cvc.25 or later.** vtk-python's wrappers bring in VTK's
  bundled expat (`libvtkexpat-9.5.a`). CPython's `pyexpatns.h` renames expat's API to
  `PyExpat_*`, but leaves two expat 2.7 internals bare:
  `_INTERNAL_trim_to_complete_utf8_characters` and `g_reparseDeferralEnabledDefault`. VTK's copy
  leaves the same two, so once cvc.23 archived expat, a link with both failed on duplicate
  symbols. cvc.25 renames them (cy-pca/cvcpkg#581), and the configure asks for it when
  `VTK_BRIDGE` is on. The lean tier never pulls VTK's expat in.

### 5.2 Packages in the browser

- **C extensions are linked in**, so the browser's package set is fixed at build time. The
  exception is plain wasm's runtime installs (§5.6).
  - Each extension gets a namespace of its own.
  - Each package ships an `inittab` manifest, and the host link reads every manifest it finds in
    the deps prefix (§5.5).
  - This is needed before the second C-extension package.
- **Pure-Python packages** (noarch) go into the zip: restrictedpython, jedi, sympy, networkx,
  imageio, trimesh.
- **Next C-extension wasm builds**, in order of demand: pillow, matplotlib (Agg; with contourpy
  and kiwisolver; §4.6), h5py (hdf5 is already published for wasm), pandas.
- **scipy** needs a Fortran answer for wasm, as it does on Windows. Not planned; the demos stay
  scipy-free, which GRL-SNAM already requires of itself.
- **torch: no.** Learned policies run through libcvc's native nav via pycvc, or a numpy forward
  pass (§7).
- **Two tiers.**
  - *Lean:* CPython + numpy + pycvc + pycvc_gl built with `CVC_PYCVCGL_VTK_BRIDGE=OFF`, so no
    vtk-python at all.
  - *Full:* the vtk-python bridge, for code that uses `vtkmodules` directly.

  §9 says what to measure.

### 5.3 The frame loop

arigl's browser loop yields with `emscripten_sleep(0)` under Asyncify (the
`while (!rt.should_close())` loop in [`src/arigl/main.cpp`](../../src/arigl/main.cpp)).

- Python must never be on the stack at that yield. So `Host.run()` in the browser returns
  immediately, and Python code runs inside frames: functions, `on_frame`, `watch`.
- Asyncify instruments every function that might reach a sleep through an indirect call, and
  CPython's eval loop is full of indirect calls. Linking CPython under `-sASYNCIFY=1` may grow
  the module and slow the interpreter a lot.
- Phase W1 measures it. The fallbacks, in order:
  1. `ASYNCIFY_REMOVE` for CPython's eval, which is safe because Python never sleeps;
  2. drive the `ariglx-web` page from `emscripten_set_main_loop` instead of Asyncify;
  3. JSPI, where the browser has it.

  `-sASYNCIFY_IGNORE_INDIRECT` stays a checklist, never a default (CVCGL_WASM.md).

**W1's numbers** (2026-11-07). The lean tier, single-threaded, emsdk 5.0.7, `-O3`. Headless Chrome
on a Windows desktop, served from loopback; the median of three loads.

| | `ariglx.wasm` raw | imports of numpy, pycvc, pycvc_gl | a 200000-step Python loop | the numpy document running |
|---|---|---|---|---|
| everything instrumented (the default, shipped) | 89.2 MB | 1.83 s | 32.5 ms | 3.4 s after navigation |
| libpython's and numpy's 16,000 functions in `ASYNCIFY_REMOVE` (fallback 1) | 82.1 MB | 1.32 s | 17.1 ms | 2.5 s |
| the full tier (vtk-python), everything instrumented | 190.4 MB | 11.1 s | 106 ms | 18 s |

The same loop takes 19.2 ms in native CPython 3.12.10 on the same machine, and stock `arigl.wasm`
from the same tree is 54.2 MB. So Asyncify costs the interpreter about half its speed and 8% of
the module, and fallback 1 removes both. The full tier doubles the module. pycvc_gl with the bridge
imports vtkmodules, which costs ten seconds, and the interpreter runs three times slower in the
larger module. So the lean tier is the package, and the full tier is for pages that need
`vtkmodules` itself. CVCGL_WASM.md has the compressed sizes.

Fallback 1 is not the default because it assumes Python is never on the stack when the module
sleeps. ariglx-web's own loop keeps that true (`host.run()` returns at once). But any C++ that
Python calls and that reaches an Asyncify import (a blocking fetch, a sleep) would unwind through
uninstrumented frames. W1 didn't audit those paths; the smoke's cases pass under both links.
Before fallback 1 becomes the default, those paths need to be found and made non-blocking, or the
list narrowed to CPython's eval loop.

### 5.4 Flavors

- **wasm** (W1): single-threaded, served from any path of an ordinary site.
- **wasm-mt** (W3, the lean tier): the recipe's second matrix leg, `CVC_PLATFORM=wasm-mt` in its
  `build-wasm.sh`. It is linked as arigl-web's wasm-mt is: `-pthread`, mimalloc, and no
  conference client, since libopus has no `-pthread` build.

**What W3 built.**
- **No pycvc-gl-cp312 package was needed.** ariglx-web compiles pycvc and pycvc_gl in its own
  tree (§5.1), so the lean tier needs only python312 and numpy-cp312 for wasm-mt, both on the
  catalog. vtk-python-cp312 wasm-mt is `+cvc.8` too (§1), but the full tier isn't built for wasm-mt
  yet.
- **The page must be cross-origin isolated** (`Cross-Origin-Opener-Policy: same-origin`,
  `Cross-Origin-Embedder-Policy: require-corp`), as any SharedArrayBuffer page must be. The smoke's
  `--isolated` sends both, plus `Cross-Origin-Resource-Policy: same-origin`, on every response,
  and fails a case whose page isn't `crossOriginIsolated`. The CSP stays as it is, since the
  workers are same-origin scripts.
- **Python's threads are real.** `threading.Thread` starts a pthread on a worker from
  Emscripten's pool. `start()` waits for the thread to run, and a new worker can't load while the
  page's main thread waits. A document's Python runs on that main thread (`init:`, an export, a
  tick), so it can have only as many threads at once as the pool has free workers. One more hangs
  the page. By `init:` the shell holds one worker. Linked with arigl's pool of 4, ariglx had room
  for 3 threads (4 hung). Its own pool is 8, which leaves room for 7 (8 hung). The pool starts with the module, and 8
  workers in place of 4 cost no startup that could be measured: a plain document ran after
  4.7 s against 5.3 s, medians of four loads.
- **Each thread gets the 8 MB stack** (`STACK_SIZE`), since Emscripten's default pthread stack
  follows it.
- **An `is_async` export runs on the app's pool,** off the main thread, when it is called from a
  process that can park, such as a tick. Called from `init:`, it runs inline.
- **The check** is `threads.ari` (`src/arigl/web/test/python`). Four threads do numpy linalg and
  random at once and are joined, which a pool of 4 can't do. Then a tick calls an async export
  that reports that it ran off the main thread.

**W3's numbers** (2026-11-12). The lean tier, emsdk 5.0.7, `-O3`, in headless Chrome on W1's
machine, with medians of three loads. The single-threaded row is W1's package, measured in the same
session: the machine ran slower than it did for §5.3's numbers.

| | `ariglx.wasm` raw | gzip -9 / brotli -q 11 | imports of numpy, pycvc, pycvc_gl | a 200000-step Python loop |
|---|---|---|---|---|
| wasm (W1's package) | 89.2 MB | 24.5 / 16.5 MB | 4.46 s | 80.0 ms |
| wasm-mt | 101.3 MB | 27.5 / 18.5 MB | 4.31 s | 88.0 ms |

So threads cost the module 14% and the interpreter about a tenth of its speed, and the imports
take as long as before. The numpy document runs about 8 s after navigation in both flavors.

### 5.5 C extensions: one namespace each

**The problem:**

- **Native separation is lost.** Natively each extension is its own shared library, loaded
  `RTLD_LOCAL`, so its init function and internals are private. Linked into one wasm module, all
  the extensions share one namespace.
- **Init names collide.** `PyInit_<leaf>` is named after the last component only, so `a._common`
  and `b._common` both define `PyInit__common`.
- **Duplicates were hidden, until W2.**
  - numpy's 14 extensions define 799 of the same global names: npymath and the random
    distributions are copied into several of them.
  - So [`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh) linked with
    `--allow-multiple-definition`.
  - That flag also hides real clashes. Two packages with different code under one name would link,
    and one would silently run the other's.
  - **It already broke numpy.** `numpy.random.mtrand` and `_bounded_integers` define seven
    distributions with different signatures (`random_poisson`, `random_zipf`,
    `random_binomial_btpe` and four more; i32 against i64). wasm-ld warned "function signature
    mismatch" and bound one extension's calls to a trapping stub. On 2026-11-06,
    `np.random.RandomState(1).poisson(4.0, 4)` aborted the host with `RuntimeError:
    unreachable`.
- **Binary tools can't fix it.**
  - `llvm-objcopy` refuses every symbol option on wasm objects, in emsdk's LLVM 23 and on LLVM
    main.
  - `wasm-ld` has no version scripts and no `--exclude-libs`.
  - Natively, Buck2 renames with `objcopy --redefine-syms`; kivy-ios rewrites Cython's output.

**The fix, proven on 2026-11-03:**

- **Where it edits.** Each extension is one relocatable object (emcc `-shared` without
  `SIDE_MODULE`). Its symbol table lives in the `linking` custom section that
  [tool-conventions/Linking.md](https://github.com/WebAssembly/tool-conventions/blob/main/Linking.md)
  documents. A rewriter in Python edits that table:
  - `PyInit_<leaf>` becomes `PyInit__x_` plus each part of the dotted name with its length in
    front, as C++ mangling does. `numpy._core._simd` becomes `PyInit__x_5numpy5_core5_simd`, so
    distinct module names give distinct symbols.
  - Every other defined global becomes local, so it's private, as `RTLD_LOCAL` makes it natively.
  - **Weak C++ RTTI is the exception.** Weak `_ZTI*` type_info objects and `_ZTS*` names stay
    global and weak. libc++abi matches a catch clause by comparing type_info pointers, and numpy
    builds `-fno-rtti`, which leaves weak copies of `std::bad_alloc`'s and the other std
    typeinfo in the extension. A private copy stops the extension catching what libc++ throws: in
    review, an out-of-memory in `np.unique` aborted the host instead of raising `MemoryError`.
  - A COMDAT group with a symbol made local gets a per-extension prefix, so C++ inline and
    template code stays per extension. RTTI's groups keep their names, so the linker merges them
    as usual.

  References inside an object go by symbol index, so they survive the rename.
- **The test.** A minimal CPython host linked numpy 2.4.6's 14 extensions with no
  `--allow-multiple-definition` and ran them under node in both flavors:
  - plain wasm: core, random, linalg and fft;
  - wasm-mt: two Python threads doing linalg and random at once.

  The rewrite covered 395 COMDAT groups. The unmodified objects fail that link with
  duplicate-symbol errors.

**The convention (W2):**

- **The tool.** `cvcpkg/recipes/_common/wasm_pyext.py`: stdlib-only Python, run with a build's
  native interpreter. It's vendored from cy-pca/cvcpkg's `recipes/_common/`, where its unit tests
  live (`tests/unit/test_wasm_pyext.py`).
  - `object` and `package` localize extensions. With `--out DIR`, `package` writes the copies
    into a prefix overlay instead of in place.
  - Exported and `NO_STRIP` symbols stay global and are reported.
  - Debug function names get the same per-extension prefix, so a combined `wasm-ld -r` link of
    several objects stays readable.
  - Common symbols (binding 3) are refused; emsdk's clang doesn't emit them for wasm.
- **Recipes.** A wasm C-extension recipe runs `package` on its extensions and ships
  `share/python-wasm/<cvcpkg package>.inittab`. Each line holds the dotted module, the renamed
  init symbol, and the object's path relative to the prefix.
  - numpy-cp312 does this from cvc.6 (wasm and wasm-mt), in cy-pca/cvcpkg#549, which adds the
    tool. cvc.6 has been on cvcpkg.org since 2026-11-07.
  - A link-host older than this one registers `PyInit_<leaf>`, which cvc.6's objects no longer
    define, so it can't link cvc.6.
  - A prefix with an older numpy (cvc.5) still links: the host localizes numpy itself (step 2
    below).
- **The host link** ([`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh)) works in six
  steps:
  1. It reads every manifest in the deps prefix.
  2. It localizes copies of any numpy that ships no manifest, in a scratch directory, never in
     the prefix.
  3. `check` refuses a set that would clash: a module listed twice, an object that isn't
     localized, or a global defined by two extensions. Weak RTTI is exempt, since its copies are
     meant to merge. Any other weak global still counts, because two packages' copies may
     differ.
  4. `registrar` writes the `pycvc_register_pyext_inittab()` that `pycvc_host.cpp` calls.
  5. It links with no `--allow-multiple-definition`.
  6. It takes undefined symbols as errors (§5.1). So it names what a CMake executable gets from
     the targets it links: cvcGL's `libcvcMedia.a`, cvc::net's `-sFETCH=1` and `-lwebsocket.js`,
     the WebGL 2 flags from VTK's FindOpenGL (`-sFULL_ES3=1 -sMIN_WEBGL_VERSION=2
     -sMAX_WEBGL_VERSION=2`), and VTK RenderingUI's js-library. Until python312 cvc.23 it allowed
     undefined symbols. That hid 44 of these beside Python's 64; the node smoke calls none of
     them. It always links the VTK bridge, so it needs python312 cvc.25 or later (§5.1), and it
     checks for it.

  The whole pycvc_gl host (VTK bridge, numpy, the deps' 256 archives) links that way, and its
  smoke prints `PYCVC_WASM_OK`.
- **Limits.** An extension that relies on another extension's global symbols breaks here, as it
  would natively under `RTLD_LOCAL`. Extensions share through capsules instead; numpy's C API is
  one. A package's own C library is linked into each extension that uses it, or shipped as a
  normal archive.
- **Upstream doesn't make this unnecessary.** PEP 793's `PyModExport_<name>` (3.15) still names
  the symbol after the module. 3.15's `PyImport_CreateModuleFromInitfunc` helps custom static
  importers but keeps the same names.

### 5.6 Installing packages at runtime

**What Pyodide does:**

- **Main and side modules.** The interpreter is the main module (`MAIN_MODULE`, everything
  `-fPIC`). Each extension is a side module that exports only its `PyInit_`
  (`pyodide build --exports pyinit`).
- **Install.**
  - micropip fetches a wheel, checks its sha256 and unpacks it into site-packages in MEMFS.
  - It then `dlopen`s every `.so` in it with `_emscripten_dlopen_promise`, before Python imports
    it. Python imports are synchronous, and Chrome won't compile a module over 8 MB synchronously
    on the main thread.
  - Each library is loaded `RTLD_LOCAL` and keeps its symbols in a scope of its own, so §5.5's
    clash doesn't arise.
- **ABI pin.** The ABI is pinned per Python version. Under PEP 783, `pyemscripten_2024_0_wasm32`
  is Python 3.12 with Emscripten 3.1.58, and every package must be built with exactly that
  Emscripten.
- **No threads.** Pyodide, PEP 783 and CPython's own Emscripten build all rule out `-pthread` with
  dynamic loading.
  - Emscripten calls dynamic linking with pthreads experimental.
  - A `dlopen` waits until every thread has caught up inside a futex wait, so a thread that
    busy-waits deadlocks it.
  - Each thread also starts slower.
- **emscripten-forge** (conda packages for Emscripten, behind JupyterLite's xeus kernels) also
  installs C-extension packages at runtime. Its kernel runs in a Worker, where the 8 MB limit
  doesn't apply.

**For `ariglx-web`:**

- **Plain wasm only.** wasm-mt stays static (§5.5): a fixed set of C extensions, plus pure-Python
  packages, which can always be added at runtime.
- **The host.** A variant of `ariglx-web` linked `MAIN_MODULE=2`, rather than Pyodide's
  `MAIN_MODULE=1`. That keeps dead-code elimination, and the C API and libc are exported
  explicitly. CPython's configure puts the cost of a loadable interpreter at about 1.5–2 MB.
- **Packages.** cvcpkg already has a `link` dimension. A `wasm` package with `link: shared` is the
  side-module build, and `link: static` is §5.5's rewritten objects. One recipe makes both.
- **The ABI key.** A side module must match the host's Emscripten version, exception mode and
  Python minor version. cvcpkg's `emsdk` pin plus the Python version is that key, so bumping
  emsdk republishes every side module.
- **Installing.** `host.install("matplotlib")`:
  1. queries the catalog for the `wasm` / `link: shared` build;
  2. fetches the `.tar.zst` and checks the catalog's sha256;
  3. decompresses with libzstd, which is already in the wasm deps;
  4. unpacks into site-packages and caches it in OPFS;
  5. `dlopen`s its extensions with `emscripten_dlopen_promise`;
  6. imports.

  A document that requires `python.matplotlib` gets an install hint when it's missing.
- **cvcpkg.org** answers browser requests only from an allowlist of origins, so the pages'
  origins have to be on it.
- **Natively** the same call is `cvcpkg install` into the host's user prefix and a `sys.path`
  entry. The extensions load as ordinary `.so` / `.pyd` files.
- **Trust.** An install request from a document follows §4.3. Natively it's allowed only from a
  local document or an allowed origin. In the browser, the page's config lists what it may
  install.
- **Pyodide's own wheels** would install only if we matched its Emscripten pin and exception mode
  exactly. That would tie our toolchain to theirs, so cvcpkg builds its own.

## 6. `arisrvx`

### 6.1 Server documents first, in stock arisrv

Before S0, arisrv ran no document: it was a fixed relay. **Shipped in S0**
(transfix/libcvc#1046; the reference is [ARIADNE.md](../ARIADNE.md) §9.7): `arisrv --doc
server.ari` runs a headless `AppRuntime` (no backend, like #985's `AriApp`) on the relay's app,
for room rules, bots and checks written in the DSL
([`server_document.h`](../../inc/cvc/ariadne/server_document.h)).

- **Startup order.** arisrv reads the document's `server:` block before it binds its ports. It
  then applies `--set`, seeds the room, and loads the document, whose `init:` runs then. `--set`
  stays before both the seeds and the document; the worlds plan's WD0 moves it last.
- **Its cadence and clock.** `on_tick:` runs once per room step (60 Hz), on the relay's loop, ahead
  of the avatar holon, under the DSL's caps. Its writes go out at the end of that step.
  - It is the room rules' cadence, not a per-world-tick controller: worlds Q3 keeps DSL programs
    off per-tick control until their cost at 120 Hz is measured.
  - Each step first writes the holon's world clock to the document's `sim.tick` and `sim.time`.
    `sim.tick` is the current world tick (120 Hz, about 2 per step, never backwards), so a later
    activation tick is simply a future `sim.tick`.
- **The world.** This runtime is the one that will run the world headless. A `scene:` is not run
  yet: its parse, physics and bodies included, is kept on the runtime for the world runtime to
  attach to, as the holon's one physics world (ARIADNE_WORLDS.md §14, L0, then WD2). The load
  warns that its physics is not simulated.
- **Its state.** The document binds under `ui.docs.arisrv`, which the relay never sends (worlds
  Q2). It reads the room through a read-only hole, `room` (`conf.rooms.<room>`).
- **One per room, later.** `ServerDocument(app, room, prefix)` builds a runtime for one room under
  its own prefix, so a relay with many rooms (worlds RM1, RM2) can build one each. arisrv
  serves one room today.
- **`requires: [relay]`** holds only in a server document, and elsewhere names `arisrv --doc`. It
  is outside the `server.*` feature tokens and `conference.world`, which can build on it.
- `(quit [code])` stops the relay with that exit status.
- Not in S0: reloading a changed document (the holes are re-planted idempotently, so a reload can
  be added), `--net`, and `on_signal:` for arisrv's own signals.

That belongs in the stock arisrv, because the DSL is already sandboxed: step and time caps, the
chroot, and bound writes.

The largest server document planned is a world: the scene, physics, avatars and controllers every
client builds its simulation from ([`ARIADNE_WORLDS.md`](ARIADNE_WORLDS.md), whose WD1 rides S0).
WD1 is built: a server document whose `scene:` has nodes or lights is the room's world, and the
relay publishes it before `init:` runs.

### 6.2 Python on the server

`arisrvx` adds the engine with pycvc core.

- **Never on the relay loop.** The relay steps rooms in fixed substeps and fans out poses and
  media, so a slow call there is everyone's lag.
- **A script thread owns the interpreter.** It reads and writes the room tree through the same
  paths and gates as everything else (`room_write_gate`, bound writes), and results land on a
  later relay tick.
- **Operator code is trusted**, as the operator's own process: the server document's `python:`
  block, or `arisrvx --python main.py`.

**As built in S1** (transfix/libcvc#1081): `arisrvx --python main.py`. The program runs on a script
thread with one asyncio loop (§6.5.1), and the relay's tree is reached only through a `ServerLink`
([`server_link.h`](../../src/ariadne_python/server_link.h)) that the relay applies on its own
thread. A server document's `python:` block is refused in arisrvx, not run: its functions would
be called from the document's programs, on the relay's loop. It waits for S2, where exports run
on the script thread.

**As built in S2** (transfix/libcvc#1122): the block runs. When the document loads, the relay posts the block
to the script thread and waits for it there. The thread starts for the block alone when there is
no `--python`. The block runs as §4.2 says, through the client hosts' own loader, with
`pycvc.ariadne.export` predefined. Its exports and the program's become the document's functions
(`register_async_work_function`). A call from one of the document's programs posts its arguments
to the script thread and parks the program, and Python's answer settles it. `init:`, which can't
park, waits in place. A raise is a `PythonError`.

Values cross as DSL values, typed, never as JSON. The relay posts `value_t`s in
`cvc.state_exec.value` capsules, and pycvc converts them as it converts every value
(`pycvc.value_from_capsule`, `pycvc.value_capsule`, after `pycvc_value.h`). So an int stays an int
and a symbol a `pycvc.symbol`, both ways. That is why exports need a pycvc beside arisrvx, as they
do in the client hosts. Without one, a server document's `python:` block fails its load, and
clients' calls are answered `unavailable`.

**Guests with scoped access to the host.** A server document's `wasm:` guest gets WASI on
arisrvx (§8.6.1). That means files under the directories the operator grants, named environment
variables, and stdio to the log, each only as far as the operator's `--wasi-*` flags allow. Where
operator Python is trusted with the whole process, a guest gets exactly what it was granted.

### 6.3 Code from clients, three tiers

Clients are untrusted. In order of preference:

1. **Named functions only (the default).** The server document marks the functions clients may
   call:

   ```yaml
   python:
     modules: [scoring]
     export:
       - {name: scoring.score_route, clients: true, limits: {time: 0.5}}
   ```

   A client calls `(room-call "score_route" args...)`. The request goes in the client's own
   bound subtree, the relay checks who is calling, the function runs on the script thread, and
   the answer comes back the same way. Clients send data, never code, so this needs no
   RestrictedPython.
2. **Client DSL programs.** When a client must send logic, it sends a `state_exec` program. The
   server runs it with `run_program` in a reduced environment, chrooted to the client's subtree,
   under step and time caps. It runs in-process, it's cheap, and it's already sandboxed.
3. **Client Python, restricted.** Only when the server document opts in
   (`client_scripts: {engine: restricted-python, ...}`). The source is compiled with
   RestrictedPython and runs in a worker process (§6.4), never in the relay or in the script
   thread's interpreter.

### 6.4 What RestrictedPython guarantees, and what it doesn't

**It does:**

- `compile_restricted` rejects names that start with `_`;
- it routes every attribute access, item access, iteration and write through guard functions
  the host supplies (`_getattr_`, `_getitem_`, `_getiter_`, `_write_`), and collects `print`;
- the script sees only the builtins and globals we hand it. With no `__import__`, or a whitelist
  importer, it can't reach `os`, files, sockets or pycvc.

**It doesn't:**

- limit CPU, memory, recursion or output. `while True: pass`, `"x" * 10**10` or a list of 10^8
  elements go straight through;
- stand on its own. It has had sandbox escapes, e.g. CVE-2023-37271 (through generator frames)
  and CVE-2023-41039 (through `str.format`). Treat it as one layer, not the boundary;
- protect other callers. In one interpreter, a script that holds the GIL stalls every other
  Python call.

**So the guarantee comes from layers:**

- **Policy (RestrictedPython):**
  - `safe_builtins` plus `limited_builtins`;
  - a whitelist importer that hands out facades: `math`, `statistics`, a read-only `room` view,
    and a numpy facade of pure array functions. The facade has no `load`, `save`, `fromfile`,
    `memmap` or `ctypeslib`, and no object arrays.
- **Budgets inside the worker:**
  - a `sys.monitoring` (3.12) counter that raises after N line or instruction events;
  - a watchdog thread that raises with `PyThreadState_SetAsyncExc` at the deadline;
  - `sys.setrecursionlimit`;
  - caps on result and print size;
  - plain values only, in and out.
- **The hard stop, outside the worker:**
  - every job runs in a pool of `arisrvx worker` processes with `RLIMIT_AS` and `RLIMIT_CPU`;
  - the workers have no network (unshare or seccomp on Linux, a Job Object on Windows);
  - a worker is recycled after N jobs;
  - a job that overruns its deadline gets its worker killed, and the client gets an error. That
    includes a C call the in-process budgets can't interrupt, such as one huge numpy operation;
  - the relay never waits on a worker.
- **Per-identity rate and concurrency limits**, keyed on the identity that bound writes already
  establish.

A client can then at worst spend its own quota and one worker's time. It can't reach the host,
the relay's interpreter, or other clients' state.

Lua (§8.1) and WebAssembly guests (§8.5) can both do tier 3 with hard in-process limits, at a
fraction of the cost: Lua through an allocator cap and an instruction hook, guests through gas and
a memory cap. A client's guest runs on the scheduler with the DSL's caps and the access of the
client's program (§8.6). Even client Python can run this way, in a CPython or MicroPython compiled
to WASI. Guests also take any language that compiles to wasm, and they run the same in
the browser. Once either is in, it's the better engine for client code, and RestrictedPython stays
for clients that need Python's libraries.

### 6.5 Application services: an HTTP API and live state from one process

Added 2026-11-05 for cy-pca/cyprov, CyberPC Angel's VPS and colo provisioning service. The
same shape fits any service that needs a REST API for programs and live views for Ariadne
clients.

**The shape.** One `arisrvx` process holds three things:

- the relay, as today;
- the server document (S0);
- the script thread (S1). The thread runs **one asyncio event loop**, which serves both an
  ASGI app (FastAPI on uvicorn) and the `room-call` functions (S2).

Programs and webhooks reach the service over HTTP/JSON. Ariadne clients (`ari`, `arigl`,
`arix`, `ariglx`, the browser) reach it over the state plane: a server-published subtree for
live views, and `room-call` for actions. Both front ends call the same Python service layer.

```
 programs, webhooks ──HTTP/JSON──► uvicorn ─┐
                                           ├─► service layer (Python, asyncio) ──► DB, outbound APIs
 Ariadne clients ──room-call──► relay ─────┘            │
        ▲                                               │ host.set_tree("app.…")
        └──────── replicated "app.*" subtree ◄── relay ◄┘  (applied on the next relay tick)
```

TLS is terminated by a reverse proxy in front of both the HTTP port and the WebSocket port,
the same way cyprty's relay is served today. Native clients that dial gRPC directly need
transport security of their own (see "Transport for native clients" below).

**Packages.** Everything the Python side needs is on cvcpkg for cp312. Checked 2026-11-05:

- `fastapi-cp312` 0.141.1, `starlette-cp312` 1.7.0, `uvicorn-cp312` 0.53.0;
- `pydantic-cp312` 2.13.4, with native `pydantic-core-cp312` 2.46.4;
- `httpx-cp312`, `httpcore-cp312`;
- `sqlalchemy-cp312` 2.0.50, `aiosqlite-cp312`, `alembic-cp312` 1.20.0;
- `pyjwt-cp312`, `cryptography-cp312` 48.0.0;
- `websockets-cp312`, `anyio-cp312`, `h11-cp312`, `python-multipart-cp312`.

Unless noted they are noarch. `cvcpkg install arisrvx fastapi-cp312 uvicorn-cp312 …` puts them
in the host's prefix, and the embedded interpreter finds them there (§3.1). No venv is needed.

#### 6.5.1 The script thread's event loop (part of S1)

- The script thread creates an asyncio loop at start and runs it until shutdown.
  `Host().loop` is that loop. Under plain `python` it is the running loop.
- **Relay to Python.** Room-call requests, connection authentication (6.5.4), signals and
  shutdown are posted to the loop with `call_soon_threadsafe`. The relay never waits on them.
- **Python to relay.**
  - `host.set`, `set_tree` and `set_series` queue mutations. The relay applies them on its next
    tick, coalesced per path, so a burst of writes costs one delta.
  - A per-tick byte cap (`--script-write-budget`, with a default) keeps a runaway writer from
    starving the relay. Writes past the cap wait for the next tick, in order.
- **Sync and async functions.** Async exports are awaited on the loop. Sync exports run on the
  loop thread, so they must be quick. Blocking or CPU-heavy work goes to
  `await asyncio.to_thread(...)`. A sync export that runs longer than `--script-slow-ms`
  (default 50 ms) logs a warning with its name, because it stalls every other caller.
- **One interpreter, one GIL.** This is unchanged from §3.3. The relay's own threads never take
  the GIL.

**As built in S1** (transfix/libcvc#1081):

- **The thread.** arisrvx boots CPython on its main thread (`EngineConfig::server`), then runs the
  program on a `std::thread` of its own through `Engine::serve`, which calls the built-in
  `pycvc.ariadne._serve_program`. That creates the loop, makes it the thread's current one, runs
  `main.py` as `__main__`, and runs the loop if the program didn't.
- **Relay to Python** goes through the ServerLink's event queue. A Python waker thread blocks in
  `_arihost.next_event()` with the GIL released and hands each event to the loop with
  `call_soon_threadsafe`, so the relay only pushes onto a queue and never takes the GIL. In S1 the
  one event is `shutdown`. S2's calls and S5's authentication ride the same queue.
- **Python to relay.** The writes queue on the ServerLink, coalesced per path. A write replaces a
  queued one it makes moot and moves to the back, so writes land in the order of each path's last
  write: a section's fields, then its `rev`. `set_tree` supersedes what is queued under it, and
  is applied as `(state-set-tree)` does it, changing only what differs. The relay applies them
  at the top of each room step (60 Hz), ahead of the server document's programs, which read them
  in the same step. `--script-write-budget` is in bytes a step (paths plus texts), 65536 by
  default (about 3.9 MB/s), and 0 for none. The first write of a step always goes.
- **Reads** (`host.get`, `get_tree`) read the relay's tree in place, pinned (td 4.8), so a
  program's own write shows a step later.
- **Not in S1:** `--script-slow-ms`. A sync export is S2's, and the warning names one, so it comes
  with them.

**As built in S2** (transfix/libcvc#1122): the relay's events carry DSL values (`ServerLink::Event`: a kind, the id
of the answer the relay waits for, a `value_t`). Python's answers come back two ways. Replies to a
client's call (`ServerLink::Reply`) go out on the relay's next room step. Answers to work the relay
waits on (`await_answer` / `answer`) settle a document's call.

`--script-slow-ms MS` (50; 0 = never) times each plain function's run on the loop. One that runs
longer is logged with its name and the time it took, client call or document call alike. An
`async def` is never timed this way, and a plain function exported with `is_async=True` runs in
`asyncio.to_thread`.

#### 6.5.2 Serving an ASGI app (part of S1)

```python
# main.py, run as: arisrvx --python main.py [relay flags]
from pycvc.ariadne import Host, serve_asgi
from myservice.api import app          # a FastAPI app
from myservice import rpc              # room-call exports (6.5.5)

host = Host()                          # the arisrvx's own runtime (§3.2 embedded mode)
serve_asgi(app, host="127.0.0.1", port=8780,
           proxy_headers=True, forwarded_allow_ips="10.10.10.5")
host.run()                             # until SIGTERM; then ASGI shutdown, then the relay
```

- `serve_asgi` builds a `uvicorn.Config` and schedules `uvicorn.Server(config).serve()` as a task
  on `host.loop`. Several apps on several ports are allowed.
- Uvicorn installs signal handlers only on the main thread (to verify per uvicorn version,
  §9). The script thread isn't the main thread, so the host keeps ownership of signals
  (`host_signals.h`).
- **Lifespan.**
  - The app's startup runs before the relay starts accepting clients. A startup failure exits
    non-zero with the traceback.
  - On SIGTERM or SIGINT the host stops accepting, runs uvicorn's graceful shutdown (bounded by
    `--shutdown-timeout`, default 10 s), runs the app's shutdown, cancels the loop's remaining
    tasks, and then stops the relay.
- Under plain `python main.py` (#985's `Host`, no relay), `serve_asgi` and `host.run()` behave the
  same, so the API runs and is tested without arisrvx. Room-call exports then simply have no
  callers.
- `--http-listen ADDR:PORT` is a shorthand for scripts that export one `app` and don't call
  `serve_asgi` themselves: `arisrvx --python main.py --http-listen 127.0.0.1:8780` serves
  `main.app`.

**As built in S1** (transfix/libcvc#1081; the reference is ARIADNE.md §9.8):

- **Where `pycvc.ariadne` lives.** It is [`ariadne_server.py`](../../src/ariadne_python/ariadne_server.py).
  arisrvx builds it in and imports it at boot as `pycvc.ariadne`, so it needs no pycvc; without a
  pycvc beside it, a stand-in `pycvc` module holds it. pycvc installs the same file as
  `_pycvc_ariadne.py`, and its import registers it as `pycvc.ariadne` unless the host's copy is
  already there. Under plain `python`, `Host` keeps its tree in a pycvc app of its own.
- **Signals.** `serve_asgi` uses a `uvicorn.Server` whose `capture_signals` does nothing, so uvicorn
  installs no handler whichever thread it runs on. Under plain `python` on the main thread,
  `run()` takes SIGINT and SIGTERM itself.
- **Startup.** The relay waits for the program before it binds anything. The program reports
  `ready` once every ASGI server has started (lifespan startup, then the bind), or that one failed.
  A failure, or a program that ends before it serves, stops arisrvx with the program's status
  (1 when that is 0), its traceback and uvicorn's log on stderr, and the relay never listens. A
  stop that comes before the program serves stops it there too.
- **A failed startup is caught inside its task.** uvicorn 0.53.0 ends a failed lifespan startup or
  bind with `sys.exit(3)`. Raised in a task of its own, asyncio re-raises a SystemExit out of the
  loop, which would stop every task on it (risk 16), so `serve_asgi`'s task catches it.
- **Stopping.** On SIGTERM or SIGINT the relay posts `shutdown` and keeps stepping while the program
  stops. Each server stops accepting and drains, within `timeout_graceful_shutdown`, which is set
  from `--shutdown-timeout` unless the program sets it, and runs its app's shutdown. The servers
  still running 2 s after the timeout are forced, and the loop's other tasks cancelled. The relay
  applies the program's last writes, publishes them and stops. If the program has not ended by
  the timeout plus 5 s, arisrvx exits without finalizing Python.
- **The exit status.** `host.stop(code)`, or a server that ends by itself after it started (0, or 1
  if it raised), stops the relay with that status, as `(quit code)` does.
- **Logging.** uvicorn's `log_config` defaults to None, so its loggers reach the host's handler:
  `[arisrvx] <logger>: <level>: <message>` on stderr, unless the program configures logging first.
- **`--http-listen`** serves the program's `app` from its globals, both when the program calls
  `host.run()` itself and when the host runs the loop after it returns.

#### 6.5.3 Server-published subtrees (part of S0)

Before S0 arisrv relayed only `conf.*` (`shard.add_interest("conf")`, with enforcement on). A
server publishes its own namespaces in its document:

```yaml
server:
  publish: [cyprov]          # replicated to clients; only the server writes under it
```

**As built in S0** (transfix/libcvc#1046):

- **Names.** A namespace is a dotted path of identifiers. It may not be under `conf`, `ui`,
  `ariadne`, `state_exec` or `room`, no two may overlap, and none may overlap the document's own
  prefix.
- **The document writes through holes.** Each namespace is a writable transparent link of the
  same name in the document's chroot, so `(state-set "cyprov.status" "up")` writes the tree's
  `cyprov.status`. `state-set-tree` now follows a writable link on every hop, as `state-set`
  does, so it can replace (and so prune) a published subtree. `state-delete` still walks the
  physical path.
- **What the relay sends.** Its shard observes the whole app tree, so it now has an outbound scope
  (`state_cluster_shard::add_publish_prefix`): `conf` plus the published namespaces, for live
  writes, backfill and every snapshot. The area of interest subscribes each participant to the
  same prefixes. The server document's own state never leaves the relay.
  - This is where S0 departs from the text above. A namespace is not added to the shard's
    *ingest* interest: the relay never needs to take a client's write there, and bound writes
    refuse one before ingest would.
- **One write path for both kinds of relay-written state.** The holes, and the scope behind them,
  carry (a) the application namespaces outside `conf`, and later (b) the world sections inside
  `conf.rooms.<room>.*`, which are already in the scope. WD1 points the same mechanism at (b).
  Nothing makes "every room's sections are relay-written" an invariant of the write gate:
  private rooms (worlds §13) will have a member's client as the authority.
- **Order.** Writes leave in the order a program makes them. A section's fields, then its `rev`
  last, arrive in that order, and a test pins it. So the revisioned-section pattern works by
  hand in the DSL today. WD1's writer, in core for L0 to share, is sugar over it.
- **First writes replicate.** The sync adapter used to miss the first write to a fresh node (its
  `valueChanged` fired before the adapter attached to it). That is why C++ writers `touch()` first
  and tests "set twice". A DSL program can do neither, so the adapter now journals what it finds
  when it first sees a node through a local write.
- **Writes from clients.** Refused by bound writes, as for any path outside the client's own
  participant. A document that publishes therefore needs `--bind-writes on`, and arisrv refuses to
  start without it.
- **Clients** are all sent the published subtrees. They still apply only `conf` until S2's
  `relay: subscribe:` adds the namespace to their interest.

- **Writes.** Only the server document's programs and the script thread can write a published
  namespace. A client write there is refused, as for any path outside its bound subtree.
- **Reads.** By default every authenticated client receives the whole published subtree. With
  6.5.6, each client receives only what its principal may read.
- **Size.** A published subtree is state, not a database. Keep large or historical data behind
  HTTP, and publish only what a live view shows: status, progress, rings of recent samples.

#### 6.5.4 Identity: an authenticate hook (S5)

Today a client proves only that it holds the shared room token, plus its pinned device key.
Bound writes are keyed by participant. Applications need to know **who** is connecting.

```python
@host.authenticate                     # async; runs on host.loop, with a timeout
async def auth(cred):                  # cred.bearer, cred.transport ("grpc"|"ws"), cred.peer
    user = await verify_oidc(cred.bearer)        # e.g. JWT check with pyjwt, or the IdP's userinfo
    if not user:
        return None                              # refuse the connection
    return {"id": user.sub, "name": user.name,
            "roles": ["staff"] if "cyprov-staff" in user.groups else ["customer"]}
```

- **Where the bearer comes from.** Over gRPC it is the `authorization: Bearer` metadata. Over
  WebSocket it is the `cvc-token.<t>` subprotocol. Both are already parsed for the room token
  (`--room-token-file`). When a server has an `authenticate` hook, the hook decides instead of
  the room token.
- **What a principal is.** A plain dict with a required `id`. The relay keys bound writes by
  `id`, room-call passes it to functions as `ctx.principal`, and rate limits (§6.4) key on it.
- **What stays in Python.** Token validation (pyjwt, cryptography, or an HTTP call to the
  IdP). This keeps JWT and signature crypto out of libcvc, which has no RSA or EC verification in
  every build (ARIADNE.md §10.5).
- **Timeouts and failures.** A hook that doesn't answer within `--auth-timeout` (default 5 s), or
  that raises, refuses the connection. The client gets an `unauthenticated` error, not a hang.
- **Revocation.** `host.disconnect(principal_id)` drops that principal's connections.
  Re-validating long-lived connections is the application's job: it schedules a task that calls
  `disconnect`.

**As built in S5** (transfix/libcvc#1130; the reference is ARIADNE.md §9.8.4):

- **In the transports.** `state_transport::set_authenticator` takes a `state_authenticator`, which
  maps a `state_credential` (bearer, transport, peer) to a principal id, or to "" and why.
  - gRPC calls it in `Channel()`, and WS during the upgrade. Each runs on the connection's own
    thread, before it is admitted, so it may block. With an authenticator, the room token is not
    checked.
  - A refusal is the room token's: gRPC `UNAUTHENTICATED`, or a refused upgrade, with nothing sent.
  - `disconnect_principal` closes a principal's connections.
  - A bearer is `valid_bearer_token`: 1-4096 characters from the room token's set, so a JWT fits.
- **In the gate.** `state_inbound_gate::bind` takes the claimant's principal. A silent holder's id
  is taken over only by a claim of the same principal (`total_principal_refusals`), and
  `principal_of(node_id)` says whose a participant is. That is "bound writes keyed by principal".
  The participant path stays the pid's: a client need not know its principal's id to join.
- **In arisrvx.**
  - `@host.authenticate` sets `ServerLink::set_authenticates`.
  - Once the program serves, the relay installs an authenticator on both planes. It posts an
    `authenticate` event and waits for the verdict within `--auth-timeout` (`ServerLink::request`),
    then keeps each principal's dict by id for its calls' `ctx.principal`.
  - `host.disconnect` queues the id, and the next room step closes those connections.
  - Python limits calls per principal: `--call-concurrency` (8 in flight) and `--call-rate` (a
    token bucket, off by default), then `rate_limited`.
- **Clients.** A refusal fails `relay_session::join` at once, "refused this host
  (unauthenticated…)", instead of waiting out the join timeout. A connection the relay closes
  later ends the waiting calls `unavailable` (transfix/libcvc#1132 then dials it again, §6.5.7).
- **Not in S5:** `token: oidc` on the client (§6.5.7; transfix/libcvc#1132, stacked on this); re-validating
  long-lived connections, which stays the service's (a task that calls `host.disconnect`).

#### 6.5.5 `room-call` for applications (part of S2)

§6.3's tier 1, with what a service needs:

```python
from pycvc.ariadne import export, CallError

@export(clients=True, roles=["staff"], timeout=30)
async def instances_list(ctx, project: str):
    return await service.instances.list(ctx.principal, project)      # same call the HTTP route makes

@export(clients=True, roles=["staff"], timeout=600)
async def instance_create(ctx, project: str, spec: dict):
    job = await service.instances.create(ctx.principal, project, spec,
                                         idempotency_key=ctx.idempotency_key)
    async for step in job.steps():
        ctx.progress(step.fraction, step.message)                     # lands in the caller's reply subtree
    return job.result
```

- **Client side.** `(room-call "instances_list" {project: "acme"})` returns a future, which
  `await` resolves. `(room-call-async …)` returns at once. Progress updates appear under the
  call's reply path, so a `progress:` widget can bind to them.
- **Context.** `ctx` carries:
  - `principal` (6.5.4);
  - `call_id`;
  - `idempotency_key`: the client's, or the `call_id`;
  - `progress(fraction, message)`;
  - `deadline`.
- **Gates.** `roles=` is checked against the principal before the function runs. A missing role is
  a `forbidden` error, and the function never sees the call.
- **Errors.** `raise CallError("not_found", "no such project", detail={...})` reaches the client
  as `{code, message, detail}`, which a DSL `try` catches. Any other exception becomes
  `internal`: the traceback is logged on the server and never sent to the client.
- **Limits.**
  - The call is cancelled at `timeout` and the client gets `deadline_exceeded`.
  - Results are capped (`--call-result-max`, default 1 MiB), and anything bigger belongs behind
    HTTP.
  - Concurrency and rate limits are per principal (§6.4).
- **One service layer.** FastAPI routes and room-call exports are two thin front ends over the
  same async service functions. Both take a principal, so the same authorization applies either
  way.

**As built in S2** (transfix/libcvc#1122; the reference is ARIADNE.md §9.8.1; the wire is
[`room_calls.h`](../../inc/cvc/ariadne/conference/room_calls.h)):

- **The wire.** A call is a message on the caller's own path,
  `conf.rooms.<room>.participants.<pid>.call`, with `ttl_hops` 0: for the relay alone, and
  `room_write_gate` refuses one it would fan out, or one on another participant's path. The replies
  (`progress`, `result`, `error`) go back on the same path to the connection that holds the pid,
  never to another that claims it. That is `state_transport::send_message_to_holder`, on both
  transports. So calls take bound writes; without them the relay answers `unavailable`.
- **The DSL.**
  - `(room-call "fn" arg ...)` parks the program and returns the value, as every async work
    function does. `(room-call-async ...)` is the future.
  - `(room-call-opts (dict "key" K "timeout" S) "fn" ...)` sets the idempotency key and the
    timeout.
  - One dict argument is the keyword arguments.
  - Progress lands in the caller's tree at `calls.<fn>.progress` and `calls.<fn>.message`, and is
    posted to the future.
  - Errors raise with `code` and `detail` as fields: `forbidden` as a `PermissionError`,
    `deadline_exceeded` as a `TimeoutError`, the rest as `RuntimeError`.
- **Values.** Both sides hold DSL values, and between the processes they travel as typed CBOR.
  The codec is [`value_cbor.h`](../../inc/cvc/state/state_exec/value_cbor.h), the WebAssembly
  guests' (§8.4), moved into the core library; a symbol crosses under tag 39 (IANA's
  "identifier"), which the guests' boundary leaves off.
  - Every data value arrives as it was sent: an int an int, `2.0` a float, bytes as bytes, a
    symbol as a symbol.
  - In arisrvx the relay decodes a call and hands Python the values themselves, converted by
    pycvc (§6.2), so no hop is untyped.
  - A function or a host object doesn't cross and fails where it is sent, as a `TypeError`.
- **Python.**
  - `export(clients=True, roles=, timeout=, is_async=)` and `CallError(code, message, detail=)`.
  - `ctx` (`CallContext`): `principal`, `call_id`, `idempotency_key`, `deadline` (a `time.time()`)
    and `progress()`.
  - The principal is `{"id": pid, "roles": []}` until S5.
  - The timeout is the shorter of the export's and the caller's (30 s when neither sets one).
  - The result cap is the relay's, `room_call_relay::set_result_max`, measured as the bytes sent.
  - A function not exported for clients is `not_found`.
- **Not in S2:** per-principal concurrency and rate limits (§6.4, with S5's principals), and
  idempotency itself, which is the service's: it gets the key.

#### 6.5.6 Bound reads: per-principal read scope (S6)

Bound writes keep a client from writing outside its subtree, but today every client still reads
the whole relayed tree. A multi-tenant service needs reads bounded too:

```python
@host.read_scope                       # sync and quick; called at connect and on refresh
def scope(principal):
    if "staff" in principal["roles"]:
        return ["cyprov"]
    return ["cyprov.public"] + [f"cyprov.projects.{p}" for p in projects_of(principal["id"])]
```

- **What's filtered.** The relay sends a connection only the deltas, and the initial snapshot,
  under its prefixes. The connection's own bound subtree and its call replies are always
  included. This is the same per-connection filter the area-of-interest code already applies
  to `conf.*`, extended to path prefixes.
- **Changes.** `host.refresh_scope(principal_id)` re-evaluates the scope. Paths that become
  readable arrive as a snapshot; for paths that stop being readable, the client gets a
  retraction.
- **Without a hook** nothing changes: every authenticated client reads everything published,
  which suits staff-only consoles.

**As built in S6** (the reference is ARIADNE.md §9.8.5):

- **In the transports.** `state_transport::set_read_scope` takes a `state_read_scope`: for the
  connection admitted as a principal that identified as a node id, a `state_delivery_filter` over
  the paths it is sent (a `state_prefix_filter`), or none for all of them.
  - A scoped connection is held for its bootstrap until its Hello, which sets its scope on its own
    thread; before that it is sent no tree state.
  - The scope joins the peer's registry rule (`delivery_view::scope`), so the admit snapshot,
    snapshot replies and the WS broadcasts take it. The live fan-out, the relay's forwards, the
    bridge, targeted pushes and messages check it under the connection's write lock.
  - A delete or reset above a readable path is delivered (`delivers_below`).
  - `refresh_read_scope(principal)` swaps each of the principal's connections' scopes under its
    write lock and sends it a delete of each subtree it can no longer read
    (`read_scope_change`, `read_scope_retraction`), then one snapshot of what it can newly read.
  - A retraction is a `delete_subtree` from the origin `<shard>#scope`, whose sequence is the
    shard clock's time, so a client's seen-set never mistakes it for a journal entry.
  - `state_sync_adapter` applies a `delete_subtree` now (it ignored the op, which nothing sent),
    and its `childChanged` slot no longer makes a removed child again: a node made on the way to
    another came back empty after a delete, a local `(state-delete)` included.
  - Not scoped: media, avatar poses, and blob chunks fetched by digest.
- **In the relay.** `room_read_scope(prefixes, room, pid)` adds the participant's own subtree and
  the room's nodes above it.
- **In arisrvx.**
  - `@host.read_scope` sets `ServerLink::set_read_scopes`.
  - The authenticate event answers each principal's prefixes before its verdict
    (`ServerLink::set_scope`), so the relay has them as it admits the connection.
  - `host.refresh_scope` asks the hook again on the script thread. A change is queued, and the
    relay's next step re-scopes that principal's connections on both planes.
  - A program with `read_scope` but no `authenticate` fails to start, and `arisrvx check` says so.

#### 6.5.7 Clients join a server (part of S2)

- **Declaring the server.** A document can name its server:

  ```yaml
  relay:
    url: wss://example.com/app-ws        # or grpc://, grpcs:// (below)
    token: oidc                          # or env:NAME
    subscribe: [cyprov]                  # published namespaces to mirror into this document's tree
  ```

  `--relay URL` on the command line does the same.
- **Which hosts.** `ari` gains this join mode. Today only arigl joins a relay, in conference
  mode through `CVC_ARIADNE_RELAY`. The `x` hosts inherit it.
- **The token.** `token: oidc` presents the document's OIDC session access token (`oidc_host.h`)
  as the bearer and refreshes it on reconnect. The token never enters the tree, as for
  `http-request`.
- **Mirroring.** Subscribed namespaces appear read-only in the client's tree at the same paths,
  so widgets bind to `cyprov.projects.acme.instances` like any other state.
- **Transport for native clients.** arisrv's WebSocket listener has no TLS, and arisrv never
  turns on the gRPC transport's TLS. Two routes:
  - **wss:// through the proxy.** This needs the native WebSocket client to speak TLS: Beast's
    SSL stream, the "native wss:// state transport" card in the conference work.
  - **`grpcs://` direct.** arisrv(x) takes `--tls-cert` and `--tls-key`, and clients dial
    `grpcs://`, using `state_transport_grpc::tls_config`, which already exists.

  Either is enough for staff consoles. Pick one.

**As built in S2** (transfix/libcvc#1122; the reference is ARIADNE.md §9.8.3;
[`relay_session.h`](../../inc/cvc/ariadne/conference/relay_session.h)):

- **The join.** `relay_session` joins as arigl's conference session does: a shard rooted at `conf`
  that applies only `conf` and the subscriptions, the device id and key, and the room token as the
  bearer. Then it waits until the room's tree arrives. Until then the relay's admission hold drops
  whatever it would send the connection, call replies included. `attach_relay` makes it a load
  hook of the runtime, so the document joins before its `requires:` and `init:`, and a failed
  join fails the load. `ari` (and so `arix`) takes `--relay URL`, `--relay-room` and `--relay-ca`
  over the block. arigl's conference session gains a `room_call_client`.
- **The block.**
  - Keys: `url`, `room` (default standup), `token` (`env:NAME` or `file:PATH`; a literal token in
    a document is refused), `ca` and `subscribe`.
  - `token: oidc` was refused in S2; S5 (the hook) left it out too, and transfix/libcvc#1132 adds it (below).
  - A subscribed namespace is linked read-only (a transparent, non-writable link) at the same path
    under the document's root, so `cyprov.projects` binds as written.
- **Transport security: `grpcs://` direct.** `arisrv --tls-cert --tls-key` serves both gRPC planes
  over TLS. A client dials `grpcs://` and checks the relay's certificate against `ca:` /
  `--relay-ca`, or against gRPC's roots when it names none (`tls_config::client_tls`). `wss://` is
  refused with a pointer to `grpcs://`: the native WebSocket client's TLS stays the conference
  card it was.

**As built, the client's `token: oidc` and reconnects** (transfix/libcvc#1132, stacked on S5; ARIADNE.md
§9.8.3):

- **The token's forms.** `token: credential:NAME`, or `oidc:NAME` for the credential an
  `(oidc-sign-in ...)` keeps, and `--relay-credential NAME`. The token is read from the host's
  credential store (`net_host_access`) at every join, for the relay's origin
  (`relay_credential_origin`: `https://host:port` for `grpcs://` and `wss://`, `http://` for
  `grpc://` and `ws://`).
  - `CredentialStore::token_for` releases a token only to the origins it is bound to, so a policy
    line binds the credential there, and the origin must be allowed.
  - Before a dial, a credential whose renewer says it is expiring, or that the relay refused, is
    renewed (`CredentialRenewer::renew`: an OIDC sign-in's refresh). This is "refreshes it on
    reconnect".
  - A bare `token: oidc` is refused, since a sign-in's credential has no default name.
- **The deferred join.** With a credential, `attach_relay` calls `relay_session::begin`, not
  `join`. The first dial happens from the frames (`pump`) once the credential holds a token, on a
  thread of its own, so a frame never waits on the network or a refresh. Until then `room-call`
  has no client and answers `unavailable`.
- **Reconnects, for every join.** A link that closes is dialed again with backoff (1 s, doubling to
  30 s). It reuses the shard and the participant id, so the gate's same-principal rule lets the
  host take its participant back. A dial counts as joined once the relay's Hello arrives: replies
  write through the admission hold, so it does not wait for the room's tree again.
- **The document sees it.** `relay.status` and `relay.joins` under its root.
- **S1's `arisrvx_program`** waited for `job.state` and could read `steps` half written, since the
  relay writes a subtree leaf by leaf. It waits for the last leaf now.

#### 6.5.8 Testing without a server (S7)

- **`pycvc.ariadne.testing.ServerHarness`.** A fake server Host for plain `python`. It holds an
  in-process tree; `authenticate`, `read_scope` and `room-call` dispatch run as arisrvx would run
  them; and `harness.client(principal)` returns an object with `call(name, **args)` and a view of
  what that principal can read. Together with FastAPI's `TestClient`, a service's tests run under
  `pytest` with no arisrvx and no network.
- **One end-to-end test in libcvc's CI.** `arisrvx --python tests/echo_service.py`, with:
  - an HTTP route;
  - an export;
  - an authenticate hook using a fixed test token;
  - a published `echo.*` subtree.

  An `arix test` document joins it, calls the export, checks the reply and progress, reads the
  published subtree, and fails its read of a path outside its scope.

#### 6.5.9 Operating it

- **Checking before starting.** `arisrvx check main.py` imports the script without serving.
  It reports missing modules (`requires: [python.fastapi, python.uvicorn]` in the server
  document works as in §4.1), ports that are already bound, and flag errors. *As built in S1:*
  it takes the relay's flags after the program, loads the `--doc` (its `requires:` and `init:`),
  runs the program as `__main__` with `serve_asgi` recording and `host.run()` returning at once,
  then binds and releases each fixed port (the relay's, `--http-listen` and every `serve_asgi`).
  It exits 1 on a problem and 2 on a bad flag.
- **Logs.**
  - Python's `logging` goes to stderr in the relay's format, which journald collects under
    systemd.
  - Tracebacks from exports and routes are logged on the server and never sent to clients.
- **Secrets.** Tokens and keys stay in the environment or in files the application reads. The
  tree, the published subtrees and room-call results never carry them. This is the same rule
  `net_host` follows for documents.
- **Process.** One `arisrvx` process per service, under systemd, with `Restart=on-failure`.
  Durable work (jobs that must survive a restart) belongs in the application's database, not in
  loop tasks.
- **Reference service.** cy-pca/cyprov's `docs/plan/PLAN.md` describes the first service built
  on this shape.

## 7. The demos and VolRover

- **GRL-SNAM's Ariadne ports** (`grl_snam/ariadne/`: `live`, `fleet`, ...) run on #985 under
  `python`. Natively they run on `ariglx` unchanged (`ariglx run -m grl_snam.ariadne ...`).
- **In the browser they need:**
  - **no torch at run time.** `austin_freedrive`, `austin_learned` and `austin_planner` import
    torch to load a checkpoint and run inference. Route that through libcvc's native nav (the
    `.cvcnav` export, `grl_snam/tools/coef_export.py`) or a numpy forward pass. Training stays
    on torch;
  - small scene bundles (`scene-austin-south-small`, noarch since cy-pca/graphics-assets#16);
  - no scipy, which is already the rule;
  - the full tier for `austin_planner`, which uses `vtkmodules`.
- **grl-snam-dbg's demos** (`austin_live`, `platoon_hud`, ...) follow the same path after
  GRL-SNAM.
- **VolRover as an Ariadne app:**
  - `share/volrover/main.ari`, built from the shipped components (`volren_controls`,
    `volslice_controls`, `transfer_function`, `mesh_tools`, `scene_controls`);
  - Python helpers for what VolRover does in C++ that libcvc doesn't yet expose to the DSL;
  - `py_console` in place of `PyConsoleDock`.

  The Qt shell, `EmbeddedInterpreter` and `vrhost` then retire. The first step is a gap audit:
  which VolRover features have no Ariadne or pycvc equivalent.

## 8. Other engines: Lua and WebAssembly guests

### 8.1 Lua, later

- **Bindings.** SWIG has a Lua backend, so a `luacvc` can reuse pycvc's `.i` files. The
  Python-specific typemaps (callables, numpy, PyCapsule) need Lua twins. sol2 isn't on cvcpkg.
- **Engine.** A `lua` `script_engine`, with `requires: [lua]`, a `lua:` block, and exports as
  host functions: the same shape as Python.
- **Hosting.** Compiled into `arix`, `ariglx`, `arisrvx` and `ariglx-web` beside Python once
  bound. `lua` 5.4.9 is on cvcpkg for every platform, wasm included. Lua is ~300 KB of ANSI C
  with no ABI to pin, so it could also go into the stock hosts later, for documents that want
  Lua without Python. That is a separate decision.
- **Sandbox:**
  - `lua_newstate` with an allocator that enforces a byte cap;
  - a `lua_sethook` count hook for an instruction budget;
  - a fresh `_ENV` holding only the functions we hand it;
  - text-only `load` (no bytecode);
  - one `lua_State` per client job.

### 8.2 WebAssembly guests

**The goal** is plugins that:

- can be written in any language that compiles to wasm (C, C++, Rust, Zig, Go);
- run on the same scheduler, under the same limits and with the same access as the document's own
  programs, so many can run at once without stalling a frame, natively or in a single-threaded
  browser page (§8.5, §8.6);
- hook the document's events as native code and Python do (§8.7);
- behave the same natively and in the browser;
- can be installed from cvcpkg while the host runs (§8.8).

**Instances, not `dlopen`.**

- A side module loaded with `dlopen` (§5.6) shares the host's memory and C ABI, and it exists only
  inside an Emscripten build.
- A guest is a separate instance with its own memory, and it sees only the functions the host
  hands it. That works the same natively, through an embedded runtime, and in the browser,
  through `WebAssembly.instantiate`.

**An engine, like Python.** Guests come in through the `script_engine` seam (§3.1):
`requires: [wasm]`, a `wasm:` block, and the guest's exports as host functions.

```yaml
requires: [wasm]
wasm:
  module: cvcpkg:route-scorer@^1     # or ./scorer.wasm, beside the document
  export: [score_route]              # functions the document's programs may call
  hooks: {on_tick: tick, on_key: key, on: {replan: replan}}
  imports: [state.read, state.write, call]   # narrows the lane's surface (§8.6); never widens it
  priority: 5                        # as a program's set-priority
  limits: {memory_mb: 32}            # may lower the host's caps (§8.5), never raise them
init: (state-set "score" (score_route (state-get "route")))
```

- **One registry.** The DSL, Python and guests share one function registry. A guest calls Python,
  and Python calls a guest, both through `register_function`.
- **Which hosts.** A guest brings no interpreter, so it doesn't need an `x` host. Which hosts
  carry a runtime is an open question (§9).

### 8.3 Runtimes, behind one interface

**The engine interface: landed in P1b.** Every engine plugs into `cvc::ariadne::ScriptEngine`
([`script_engine.h`](../../inc/cvc/ariadne/script_engine.h)): Python now, Lua (L1) and wasm guests
(G1) later.

- **Registration.**
  - `register_script_engine` is process-wide. The engine's language becomes a block key, and its
    tokens hold (`has_module` answers `<language>.<module>`).
  - `AppRuntime::use_script_engine` is per runtime. Each load of that runtime hands the block to
    `load_block(block, lane, warnings)`, after the parse and before the load hooks, `requires:` and
    `init:`.
  - From the second load on, `unload(lane)` runs first, and the lane's hooks are dropped.
- **What a block hands its engine** (`ScriptBlock`): the language, the block as written, the
  document's path, directory and prefix, and its module (`""` until module blocks run).
- **The lane** (`ScriptLane`, one per runtime and engine) is how the engine reaches the document.
  It is the program lane's own surface:
  - `export_function` and `export_async_function`: the one registry (§8.2);
  - `start(activation, options)`, `stop` and `running`: activations (§8.5);
  - the hooks (§8.7): `on_event`, an action per event; `on_frame`, a resident at the end of each
    drain; `on_watch`, a resident when a value under the prefix changed. The frame and watch hooks
    keep one delivery in flight, so a slow one coalesces.
- **Activations are processes of the document's program lane** (`Runtime::start_process`), built
  like sliced tasks.
  - The process gets the document's owner tag, chroot, channel policy, lock manager and
    `functions:`. It runs under the lane's caps (`ProgramLane::action` or `resident`), which
    `ActivationOptions` can lower and never raise.
  - The process is a DSL loop around `(script-step)`. Each step runs one
    `ScriptActivation::step(ScriptSlice &)` of `ariadne.limits.<language>_slice_steps` (64). The
    slice ends in one of five ways: more work, `finish`, `yield_frame`, `await(future)` or
    `call(function, args)`.
  - `charge(steps)` adds the slice's work to the process as DSL steps, so the lane's step cap is
    the gas cap.
  - The loop makes a `call` itself, as `(apply f args)` inside a `try`. So a guest reaches exactly
    what a DSL program there reaches, and a call that waits parks the process. The value or the
    exception comes back to the next slice.
  - `ActivationOptions::may_call` narrows the calls (§8.6's `imports:`). A refused name comes back
    as a `PermissionError`, and a missing one as a `NameError`.
  - A language's activations share `ariadne.limits.<language>_frame_budget` of each frame
    (0.008 s; frames by `exec_frame`). Once it is spent, the rest yield to the next frame.
  - A throw from a step fails the activation with its class, and a cap or a kill ends it as "did
    not complete". An activation that never finished is cancelled when its process ends, by its
    document's teardown too.
- **Python on it.** Python's engine answers the tokens and runs the block in `load_block`. It
  starts no activations: its calls stay inline natives (async ones on the pool), and `Host`'s
  hooks stay `add_frame_hook`. What a document sees is unchanged.
- **Tested** by `ariadne_script_engine_test` (a stub engine through all of the above), the
  `AriadneRequires` token test (through a stub Python engine), and the `arix_*` / `ariglx_*`
  ctests, which are unchanged.
- **G1's first part, built** ([`src/ariadne_wasm`](../../src/ariadne_wasm/wasm_engine.h)):
  - the runtime interface below (`guest_runtime.h`) with its wasmtime backend: each slice is one
    `wasmtime_call_future_poll` on the guest's own stack, under a fuel yield interval of the
    slice's gas, and an import that waits is an async host function;
  - the `wasm:` block (`module:` a `.wasm` or `.wat` beside the document, `export:`, `hooks:`,
    `imports:`, `limits: {memory_mb}`, `priority:`), an instance per document, `_initialize`;
  - the guest ABI as §8.4 builds it, CBOR in the guest's memory;
  - exports as functions of the document through `ScriptLane::export_activation`: a call is an
    activation the caller parks on (an async function whose work starts itself, so it needs no
    pool thread, and its wait counts against the caller's caps), with `-async` futures;
  - fuel charged as DSL steps (`ariadne.limits.wasm_gas_per_step`, 1000), the action caps, the
    frame budget; one activation per instance (the others wait a frame at a time), and a call
    back into the guest's own export refused;
  - the `call` import through the lane, narrowed by `imports:` (`state.read`, `state.write`,
    `msg`, `net`, `call`), and `yield` and `fail`;
  - the `on_tick`, `on:` and `watch` hooks; `WasmError` (a built-in class beside `PythonError`);
    the `wasm` token; `ariadne.limits.wasm_max_memory` through the store's limiter;
  - in `arix` and `ariglx` where wasmtime is found (`-DARI_WITH_WASM` / `-DARIGL_WITH_WASM` make
    it required), and CI's `package-linux` installs it. Tests: `ariadne_wasm_test`, the
    `arix_wasm_block` and `ari_wasm_needs_arix` ctests.
- **G1's second part, built:** `ScriptLane::run` runs an activation to its end now, pumping
  the app's scheduler a frame at a time (`Runtime::pump`), under `ProgramLane::init`'s caps for
  a load's work. A guest's `ari_init` runs that way at every load, before the document's `init:`.
  An export called from `init:` (which runs on a scheduler of its own and can't park) runs that
  way in the call: the provider binds the export's blocking form where the lane's scheduler isn't
  the app's.
- **G1's third part, built:** host objects by handle, the reading half of §8.4.1.
  - A value holding a `data_object` crosses as a handle under CBOR tag `0x61726968` ("arih"), and
    a handle the guest hands back is the object again.
  - The instance's table pins the object it was handed until `handle_drop`. Each slot has a
    generation, so a dropped handle names nothing.
  - `data_info`, `volume_read` (a box, every 2^lod-th voxel), `volume_sample`, `mesh_read` and
    `heightfield_read` copy only what they're asked for into the guest's memory, at a step per
    `kDataNodeBytes` moved.
  - `imports:` gains `data.read` and `data.write`.
  - A trap with the guest's memory at its limit is a `MemoryError`: a `memory.grow` past it gives
    the guest -1, and the guest gave up.
- **G1's fourth part, built:** a call from a program is that program's own sliced native call
  (§8.5), in place of an activation the program parked on.
  - `export_activation` binds three forms. `(name ...)` from a program on the app's scheduler is
    a `SlicedCall`, a data job (`data_job.h`) that runs a slice of the activation per data step of
    the calling process. `(name-async ...)` is still an activation of its own, with a future.
    `init:` still runs the activation to its end.
  - Each slice's steps are charged to the calling process, so its caps are the guest's, and the
    language's frame budget still bounds the slices.
  - A slice's `call` and `await` are the program's own: a closure around a `try` that the
    evaluator runs in the calling process. So a native, a DSL function the program defined and a
    future all work, and a throw comes back to the next slice instead of unwinding the job.
    `yield_frame` awaits the next frame. `may_call` narrows the names as before.
  - A throw from a step raises at the call with its class. The job dies with the call, and an
    activation that never finished is cancelled.
  - It is the core's, so every engine's exports get it. The wasm engine changed nothing.
- **G1's fifth part, built:** the pump budget counts what the slices charge (§8.5).
  - `async_scheduler`'s run loop counts each pick as the steps it charged its process: its
    evaluator step, plus what the step added (`evaluation_stats::add_steps`).
  - So a slice charged 64 steps costs the drain's 200,000-step budget 64, not one, and an inline
    async call's run time counts too.
  - A plain program charges one step a pick, so nothing else changes. `sync_run_steps()` returns
    the same count.
- **G1's sixth part, built:** waiting for the instance through the lock manager, and
  `RecursionError` (§8.5).
  - `ActivationOptions::lock` is a with-lock key under the document's prefix that an activation
    holds, exclusive, from before its first slice to its end. Its process runs the loop inside
    `(with-lock (script-lock) ...)`. A program's sliced call makes `(with-lock key (run))` its own
    call (`call_function`), with the key written as that program's chroot writes it, and the
    job starts in the body.
  - So the activations with one lock wait for each other as programs wait in `with-lock`:
    - `ariadne.exec.lock_timeout` for an action, and none for a resident;
    - priority inheritance under the priority policies;
    - `DeadlockError` for a cycle;
    - and `lock-query` shows the holder.
  - It is the core's, so any engine's activations can take one.
  - The wasm engine gives every activation of a block `wasm:<stem>`, so the next frame no longer
    re-tries a busy instance.
  - A call back into the guest from inside one of its own host calls is made in the same
    process, so it is granted the lock again, as a process's locks are reentrant. It finds the
    instance busy with that process's own activation, and raises `RecursionError`
    (`RuntimeError`'s, with `running`, the export in flight). The guest gets it back as that
    call's error.
  - This replaces the check by name in `call`, which missed an export renamed by `as:`, and it
    covers a call through a document function too.
  - A call back that goes through another process, a `-async` future the host call awaits,
    waits like any other call: an action's lock timeout ends it.
- **G1's seventh part, built:** the key, pointer, signal and message hooks.
  - **The channels fan out.** `Runtime::set_event_tap` gets every event `post_input` and
    `post_signal` take, as the dict the `on_key:` / `on_pointer:` / `on_signal:` program reads,
    beside that program. It gets them in order, on the drain's thread, before the pump. A tapped
    signal is handled, so the host skips its default. `AppRuntime` taps the kinds its lanes'
    hooks take.
  - **The lane's hooks.** `ScriptLane::on_key`, `on_pointer`, `on_signal` and
    `on_message(channel)` queue their events. One resident takes a hook's queue, an activation per
    event in its one process, and ends once the queue is empty, so a burst is taken in the drain
    it came in and nothing is coalesced. A failed event is that event's outcome (`on_done`, else a
    warning once per message), and the next goes on.
  - **Messages.** A message hook's receiver is a process of the lane that waits in
    `(msg-recv channel)`, under the document's prefix and channel policy, and holds no lock. It
    hands each message, `{channel, value}`, to the hook's queue. One that keeps stopping (a
    channel the policy refuses) is restarted as a DSL resident is, and given up after 8 stops in
    a row.
  - **The `wasm:` block** gains `on_key`, `on_pointer`, `on_signal` and `msg_recv: {channel:
    export}`.
- **G1's eighth part, built:** the state imports, and §8.4.1's writes.
  - **A fast path beside `call`.** `ScriptSlice::call_now` calls a function the lane holds as a
    native, the one `call()` would reach, inside the slice, which goes on. Its deep-data work runs
    to its end there, and what it raises is the reply. A name the lane holds otherwise (a
    document function) is left to `call()`. It is the core's, so any engine can use it.
  - **The state imports** (`state_get`, `state_exists`, `state_children`, `state_data_get`,
    `state_set`, `state_data_set`, `state_delete`, `state_swap`, `state_incr`, `state_cas`) are
    the lane's own state natives through it, as §8.6 has them: the path as text, the values as
    CBOR, a step a call. A guest's state access no longer ends its slice for the loop's
    `(apply ...)`.
  - **`state.write` holds every state writer.** The groups were split on the name, so the atomics
    and the expiry setters were `state.read`'s; now the writers are listed.
  - **The writes** (`volume_write`, `volume_new`) and the `data.write` group they need. A write
    goes to the guest's own copy: a volume anyone else may hold is copied at its first write
    (the voxels copy on write), and the handle names the copy. A volume only the guest holds is
    written in place. A new volume counts against the call's allocation limit.
- **G1's ninth part, built:** wasmtime in the `arix` and `ariglx` cvcpkg recipes.
  - Both name `wasmtime` as a build and a runtime dep, and configure with `-DARI_WITH_WASM=ON` /
    `-DARIGL_WITH_WASM=ON`, so a build that can't find it fails instead of packaging a host
    without guests.
  - `publish-cvcpkg.yml` installs it for their steps, and each host runs
    `src/ariadne_wasm/tests/wasm_block.ari` before it packs: `arix test`, and `ariglx check`.
  - **Linked by name.** Upstream's Linux `libwasmtime.so` has no SONAME, and the package's
    target is `UNKNOWN`, so CMake linked it by its path. The hosts recorded that path, relative to
    the build tree, as the library they need, so they would load it from nowhere else.
    `cvc_ariadne_wasm_host` now links it through a `SHARED` imported target with
    `IMPORTED_NO_SONAME` (`-lwasmtime`). The host needs plain `libwasmtime.so`, found in `../lib`
    beside libcvc.
- **G1's tenth part, built:** the scheduler's policy as a host option, and pure exports on the
  read lane. G1 is done.
  - **`--exec-policy round_robin|priority|priority_rr`**, one of the general flags (`host_cli.h`),
    so `ari`, `arigl` and the `x` hosts take it. `apply_exec_policy` writes `state_exec.policy`
    on the fresh app, before anything builds its scheduler. A guest's `priority:` then picks first
    under `priority` and `priority_rr` (§8.5).
  - **`Runtime::register_read_function`** adds a function to the read lane's environment, beside
    its allowlist. It gets the arguments and the lane's environment, and runs inline in the
    expression's eval. Its reads go through the lane's wrapped readers, so the expression's
    result is kept until one of those paths is written. A later registration replaces it and
    drops the kept results.
  - **`ScriptLane::export_pure`** makes an activation such a function. Its slices run to the end
    inside the eval, charged to the eval's steps, with the eval's deadline checked between them.
    Its `call_now()`s and `call()`s reach only the read lane's environment; anything else is a
    `PermissionError`. A yield goes straight on, and an await is a `RuntimeError`.
    `ScriptSlice::may_wait()` is false there, so an engine refuses what would have to wait: a
    wasm instance busy with another activation raises `RuntimeError` at once.
  - **The `wasm:` block** takes `{name:, as:, pure: true}` in `export:`. The export is the
    programs' function too.

**The wasm runtimes.** A `cvc::ariadne::wasm_runtime` interface, behind G1's engine, covers:

- load, and instantiate with imports;
- start a call, step it one slice, and suspend or resume it (§8.5);
- limits.

proxy-wasm's C++ host is the model: it puts V8, WAMR, WasmEdge and wasmtime behind one interface.

- **wasmtime** on Linux, macOS and Windows.
  - cvcpkg publishes 45 from upstream's prebuilt C API; upstream is at 50.0.1.
  - It has fuel, epoch deadlines, a store limiter for memory, and an async C API that runs a guest
    on its own stack and pauses it at a yield (§8.5), plus WASI preview 1 and 2.
- **WAMR** for the BSDs, and as the small option.
  - Pure C with CMake, and it has a FreeBSD port.
  - Published at 2.4.4, with the interpreter, fast interpreter, AOT and WASI.
  - It can't pause a running guest, so it runs the install-time transform's guests (§8.5). Its
    WASI isn't supported on Windows, where wasmtime runs instead.
- **The browser's own engine,** reached from the Emscripten module through `EM_JS`.
  - The guest's imports call our module's exports, and bytes copy between the two memories
    (`HEAPU8.set`).
  - It runs the transformed guests too, in gas slices on the page's own thread (§8.5). So even an
    untrusted guest can't hang the page.
  - A Web Worker is an optional backstop for untrusted guests on a cross-origin-isolated page.
    Synchronous host calls from a Worker need `SharedArrayBuffer` and `Atomics.wait`.
  - Chrome compiles modules over 8 MB only asynchronously on the main thread.
    `compileStreaming` caches compiled code for modules of 128 kB or more.
- **The rest.**
  - Wasmer (7.1) and WasmEdge (0.17) are on cvcpkg too, but neither adds a platform that
    wasmtime and WAMR don't cover.
  - wasm3 restarted development in 2026. It's a lighter interpreter if WAMR disappoints.

### 8.4 The guest ABI

**As G1's first part builds it** (`wasm_engine.h`): an export takes `(ptr, len)`, the CBOR of its
argument list, and returns an i64 with its result's CBOR pointer in the high 32 bits and length in
the low 32. The guest exports `memory` and `ari_alloc(len)`; the imports are `call(name, args) ->
i64` (the result's length, or minus its error's), `reply(dst)`, `yield()` and `fail(text)`. Named
exports stand in for `ari_dispatch`. The handles came with the third part (§8.4.1), and the state
imports with the eighth: `state_get(path, len)` and the rest take the path as text and their
values as CBOR, and return as `call` does. The rest of this section is the plan.

- **Core wasm, in the style of Extism and proxy-wasm.**
  - Values cross as a pointer and a length, encoded as CBOR that mirrors state-tree values.
  - The guest exports `ari_alloc`, `ari_init` and `ari_dispatch`. `ari_dispatch` takes calls and
    hook events (§8.7).
- **Host imports** (module `ari`). Each is a program-lane native of the DSL, run in the guest
  activation's own environment (§8.6):
  - the state readers and writers: `state_get`, `state_set`, `state_children`, the
    `state_data_*` calls and the atomics;
  - `msg_send`, `msg_recv`, `sleep` and `await`, which park the activation (§8.5);
  - `with_lock`;
  - `call`, for any function the lane can call;
  - `hook`, `set_priority` and `log`;
  - `gas`, the check the install-time transform inserts (§8.5);
  - the calls on host objects' handles (§8.4.1).
- **No pointers into the host.** A guest reaches state only through those imports.
- **Bulk data goes by handle** (§8.4.1). A volume or a mesh stays on the host. The guest copies only
  the part it reads.
- **WIT later.** The interface is also written in WIT, so a component-model adapter can come later.
  Not yet:
  - wasmtime's C API still has gaps in the component model, including the async half
    (bytecodealliance/wasmtime#13705);
  - only wasmtime and jco implement it;
  - the browser needs jco's generated JavaScript.

  WASI 0.3 (async, streams) was ratified in 2026-06.

#### 8.4.1 Host objects: handles, and copies of only what is read

**As G1's third part builds it** (`wasm_engine.h`): the reading half, natively on wasmtime.
- A `data_object` crosses as tag `0x61726968` ("arih") over an unsigned integer, the slot's
  generation in its high 32 bits.
- The instance's table holds at most 65,536 live handles.
- The readers take their region as plain integers, with `h` an i64 and the rest i32s, and return
  the bytes written or, as `call` does, minus the length of an error for `reply()`:
  - `volume_read(h, x, y, z, nx, ny, nz, lod, dst, len)`, in the volume's own voxel type;
  - `volume_sample(h, points, n, dst, len)`, f64 points in, f64 values out;
  - `mesh_read(h, attr_ptr, attr_len, first, count, dst, len)`, the attribute's name in the
    guest's memory: `positions` or `normals` as f64 triples, and `indices`, `quads` or `lines` as
    u32s;
  - `heightfield_read(h, row, col, rows, cols, dst, len)`, in f32s.
- `data_info` knows volumes, meshes and heightfields. Any other object (a `city_bundle`) gives only
  its type name.
- **The writers, as G1's eighth part builds them:**
  - `volume_write(h, x, y, z, nx, ny, nz, src, len)`, the box in the volume's voxel type, into the
    guest's own copy;
  - `volume_new(spec, len)`, a CBOR map as `data_info` gives one (`dims`, `voxel`, `bounds`), which
    returns the new volume's handle. A handle is always positive as an i64, so it can be a
    return value.
- **Not yet:** the browser's path, and writers for meshes and height fields.

A guest gets a host object (a volume, a mesh, a heightfield, a city bundle) as a handle, not as
its bytes, and reads only the parts it asks for. The DSL already works this way. On a node holding
a host object, `(state-data-get path)` returns an opaque `data_object`, a shared pointer to the
payload, and copies nothing. The objects a `data:` block decodes (`mesh`, `heightfield`, `volume`,
`city_bundle`) sit on such nodes. pycvc gets the same objects as SWIG wrappers.

- **Handles in place of values.**
  - A value crossing to the guest that holds a `data_object` crosses as a handle: a CBOR integer
    under a tag of its own.
  - The instance's handle table keeps the object by shared pointer. So a guest pins the version it
    was handed, even if a reload or a writer replaces the node meanwhile. The voxels are
    copy-on-write, so a pinned volume costs no copy.
  - Each handle carries a generation, so a guest can't forge a handle, or use one after it is gone.
  - `handle_drop` lets one go. The table goes with the instance.
  - A handle the guest passes back (to `call`, `state_data_set`, a reply) is the object itself on
    the host side.
- **Metadata costs nothing to read.** `data_info(h)` returns a small CBOR map: the type, the
  dimensions and voxel type, the bounding box, the vertex and face counts. No data moves.
- **Reads copy only the region asked for, straight into the guest's memory.** The bulk goes as raw
  typed arrays, not CBOR:
  - `volume_read(h, box, lod, dst, len)`: a sub-box, a slab or a brick;
  - `volume_sample(h, points, dst)`: values at the given points;
  - `mesh_read(h, attribute, range, dst)`: a range of `positions`, `normals` or `indices`;
  - `heightfield_read(h, rect, dst)`.

  The guest allocates `dst` (`ari_alloc`). The host writes into it in place:
  `wasmtime_memory_data` and a `memcpy` natively, and a typed array's `set` on the guest's
  `memory.buffer` in the browser.
- **Writes and new objects.** `volume_write(h, box, src)` writes a region. `volume_new(dims, type)`
  returns the handle of a new volume, which the guest publishes with `state_data_set(path, h)`.
  Copy-on-write means a write clones only what it touches.
- **Operations on the host.** A guest can pass handles to host functions through `call`, as a DSL
  program passes `data_object`s: an isosurface, a resample, a filter, returning a handle to the
  result. Then the data never enters the guest's memory, and the guest only orchestrates. That is
  most of what a control-logic guest over large data needs.
- **Cost and slices.**
  - Each read or write is one import call, so a large read can be split across slices.
  - It is charged by the byte, at `data_job`'s rate (a node of work per `kDataNodeBytes`, 256
    bytes). So a bulk copy counts against the activation's gas cap and the frame budget like any
    other work.
- **Access.**
  - A guest gets a handle only through `state_get` or `state_data_get` under the lane's chroot, or
    from a function it may call. So a handle grants nothing the document's prefix and `link:` holes
    don't.
  - `imports:` narrows the calls on handles with the `data.read` and `data.write` groups (§8.6).
- **The floor.** Core wasm can't read host memory in place: a module sees only its own linear
  memory. Exposing a host buffer as an imported memory would make it writable by the guest, and
  would mean keeping volumes in wasm memories. So a guest that reads data copies the region it
  reads, and the handle keeps that cost to what it asks for.
- **The other way round.** A guest buffer passed to a host call as a pointer and a length is read
  in place for that call. The host doesn't keep the pointer: `memory.grow` can move the memory.
- **Phases.** The handle table, `data_info` and the readers came with G1's third part, and
  `volume_write` and `volume_new` with the eighth. More object kinds can follow as guests need
  them.

### 8.5 Scheduling and limits

**A guest activation is a process on the app's scheduler.** An activation is one export call, one
hook delivery, or the guest's `ari_init`. It runs on the scheduler that runs the document's actions
and residents (`app.exec_scheduler()`, ARIADNE.md §2.2), the way a sliced task does
(`cvc/core/sliced_task.h`):

- it's a small DSL loop around a native `(guest-step)`;
- it has the document's owner tag, so it's reaped with the document;
- `(ps)` shows it as `wasm/<module>:<export>`;
- the drain pumps it with everything else, in the same `sync_run` slices (200,000 steps or 0.1 s a
  frame).

A guest function called from inside a program runs as that program's sliced native call: one slice
per data step, charged to the calling program, as `collections.map` is (developer guide, "Sliced
work").

**Slices.** Each step of the process runs the guest for one slice of gas, then gives the thread
back.

- `ariadne.limits.wasm_slice_steps` sets the slice in DSL steps (64 by default).
- `ariadne.limits.wasm_gas_per_step` converts steps to gas. It's calibrated as
  `limits.async_steps_per_cpu_ms` is: a step's worth of gas costs about what a DSL step costs.
- A slice is charged to the process as that many steps. The drain's pump budget counts those
  steps, not one per pick, so guests can't stretch a drain. That's a small scheduler change, made
  in G1's fifth part (§8.3).

**Gas.**

- Counters are injected into the guest's bytecode at install (paritytech/wasm-instrument). So the
  count is the same in every runtime, native and browser.
- The `gas` import they call ends the slice when it's spent, and kills the activation when its cap
  is spent.
- wasmtime's own fuel stays on as a backstop.

**How a guest pauses mid-run.** A slice ends at a gas check, or at a host call that has to wait
(`await`, `msg_recv`, `sleep`, an async host function). Then the guest pauses where it stopped,
and the process's next step resumes it. Two mechanisms, one contract:

- **wasmtime natively: its async C API.**
  - `wasmtime_func_call_async` runs the guest on its own stack.
  - Each scheduler step calls `wasmtime_call_future_poll` once. That runs the guest until an async
    host function (`wasmtime_linker_define_async_func`, the `gas` import among them) yields, or the
    call returns.
- **WAMR and the browser: Binaryen's Asyncify.** The guest is transformed at install with
  `wasm-opt --asyncify`, which emsdk already ships. The `gas` import and the waiting imports unwind
  the guest's stack into its own memory and return to the host, and the next step rewinds it to the
  same point.

  Proven on 2026-11-03 under node:
  - three transformed guests, stepped round robin in 8192-unit gas slices on one thread,
    interleaved `ABCABC…`;
  - one was killed at its gas limit mid-run while the other two went on;
  - both survivors' results matched unsliced runs.

  Asyncify doubled that tiny guest (444 → 893 bytes), so measure it on real ones.
- **JSPI** (`WebAssembly.Suspending`) could replace Asyncify in the browser later. It resumes
  through the event loop, so measure it against Asyncify first.

**The DSL's limits, mapped.** The caps are the host's settings, read per activation as the DSL's
are. A document can lower them for its guests, never raise them.

| Limit | DSL program | Guest activation |
|---|---|---|
| steps (`ariadne.limits.action_max_steps`, 5000) | an `on:` action's evaluator steps | its gas, at `wasm_gas_per_step` |
| time (`ariadne.limits.action_max_time`, 0.1 s) | an action's run time | its slices' run time, on the same `se::time_budget` |
| memory (`ariadne.limits.action_max_memory`, 4 MiB) | state writes and held values | what it writes to state and hands back, measured the same way |
| per step (`ariadne.limits.resident_max_step_time`, 0.05 s; 1 s in wasm) | one step of a resident | one slice of a hook's resident (§8.7), a backstop for a slow host call |
| `init:` lane (10 million steps, 5 s) | the document's `init:` | `ari_init`, and calls from `init:` |
| read lane (100,000 steps, 5 ms; 30 ms a frame) | a `visible_when` or computed `bind:` | a `pure` export called there (§8.6) |
| instance memory | n/a | `ariadne.limits.wasm_max_memory`, lowered by `limits: {memory_mb}`, checked at `memory.grow` |

- **Errors.**
  - Running out of gas or time kills the activation, and the drain reports it as it reports a DSL
    action's step limit (ARIADNE.md §2.4).
  - A refused `memory.grow` gives the guest -1. A trap with the memory at its limit (the guest
    gave up) is a `MemoryError`.
  - A trap is a `WasmError`, which a calling program's `try` can catch.

**Many guests at once, without choking a frame.**

- **Slices interleave.** A guest gives the thread back after every slice, so the scheduler
  interleaves it with every other process. One that waits on a host call parks and costs nothing.
- **Quick calls finish in the drain.** An `on:` handler that needs a few slices completes in the
  drain that fired it, as a quick DSL action does.
- **Long runs yield the frame.** Guest activations together get at most
  `ariadne.limits.wasm_frame_budget` of a frame (0.008 s by default). Once it's spent, the rest yield
  to the next frame, as cooperative sliced tasks do. So N long-running guests share the budget
  instead of each taking a share of the frame.
- **One thread, everywhere.** This works the same in a single-threaded native host, a
  single-threaded wasm page and wasm-mt: the scheduler is stepped by one thread in all three.

**Priorities.**

- An activation's priority comes from the `wasm:` block, the hook's declaration, or the
  `set_priority` import, as a program's `set-priority` does. Under the `priority` and `priority_rr`
  policies the scheduler picks the best level first, and the frame budget goes to the better levels
  first.
- The stock hosts run `round_robin`, which stores a priority but picks by pid order (ARIADNE.md
  §2.2). `--exec-policy priority` (or `priority_rr`) changes that: the general flag sets
  `state_exec.policy` before the host builds its scheduler (G1's tenth part).

**One activation per instance.**

- An instance's memory and stack belong to the activation in flight, and wasmtime allows one call
  future per store. So an instance runs one activation at a time.
- The others wait for it as a DSL program waits for a lock: through the app's lock manager, with
  its timeouts (`ariadne.exec.lock_timeout` for an action, none for a resident) and its priority
  inheritance. The lock is the instance's `wasm:<stem>`, under the document's prefix
  (`ActivationOptions::lock`; built in G1's sixth part).
- A call back into the same instance from inside one of its own host calls is refused with a
  `RecursionError`. Such a call runs in the same process, which the reentrant lock lets through,
  and finds that process's own activation on the instance. A call back through another process
  (a `-async` future the host call awaits) waits as any other call does.

**Threads, later.** Native and wasm-mt hosts have threads, but guests run on the scheduler's
thread, because their imports touch state. An export marked `compute: true`, which makes no host
calls between gas checks, could later run its slices on the pool as threaded sliced tasks do, and
post back through the scheduler's ingress.

### 8.6 Access: what a DSL program has, and no more

A guest runs as a program of the document, or of the module, that declares it, in the lane that
started it. It gets exactly what a DSL program there gets.

- **The same natives.** The guest's state, message and lock imports are the program lane's own
  natives, applied in the activation's environment. So all of this holds with no second
  implementation:
  - the chroot to the program's prefix;
  - the `link:` holes (ARIADNE.md §7.4): read through, `ro` or `rw` over the whole subtree, and
    torn down and re-planted at each reconcile;
  - channel scoping (ARIADNE.md §8);
  - the lock manager and the memory accounting.

  A path starting with `/` escapes only where the DSL's would. `state_watch` acts on the physical
  path and doesn't reach through a hole, as `state-watch` doesn't.
- **The same functions.** `call` reaches what that lane's environment holds:
  - the document's `functions:` (ARIADNE.md §6.6);
  - the host functions, Python's exports included;
  - the lane's host intrinsics, with the network verbs under the host's allowlist (ARIADNE.md
    §10.3).

  A function the DSL can't call there, a guest can't either.
- **Modules are the fine-grained grant.**
  - A guest declared in a `load:`ed module sees the module's sub-prefix, its `link:` holes, its
    `channels:` grants and its `functions:` grant. It sees nothing of the parent beyond those.
  - To give a guest a few named parts of a document, mount it in a module and link exactly those.
- **Narrowing.**
  - The block's `imports:` names the groups a guest gets: `state.read`, `state.write`, `msg`,
    `lock`, `call`, `net`, and `data.read` / `data.write` for the calls on host objects' handles
    (§8.4.1).
  - An import outside them is linked to a stub that raises `PermissionError`, so a guest fails
    closed, and the manifest check (§8.8) says which import it lacked.
  - Narrowing never widens: a group the lane doesn't have stays out.
- **The read lane.** An export marked `pure: true` can be called from a reactive slot. It's linked
  with the read lane's allowlist (the state readers, no writes, no messages, no `call` beyond pure
  functions) and runs under that lane's caps. *As built* (`ScriptLane::export_pure`): it runs to
  its end inside the expression, and its reads keep the expression's result as the DSL's reads
  do.
- **Clients on `arisrvx`.** A client's guest runs as that client's program (§6.3, tier 2): chrooted
  to its bound subtree, under the bound-writes gate.

#### 8.6.1 WASI on `arisrvx`: scoped access to the operating system

A guest reaches nothing outside its own memory except what the host imports into it (§8.4). WASI
is the standard set of those imports: files, the environment, stdio, clocks and random numbers.
The toolchains target it: `wasm32-wasip1` for Rust and C, wasi-sdk, and CPython and MicroPython
built for WASI. A server guest on `arisrvx` gets WASI, but each resource only as far as two grants
allow: the operator's for the process, and the document's request for the guest.

- **The operator's grant is the ceiling.** It belongs to arisrvx itself, in flags or the service's
  config, and no document can widen it:

  ```
  arisrvx --doc server.ari \
    --wasi-dir /srv/app/data:ro --wasi-dir /srv/app/cache:rw \
    --wasi-env DATABASE_URL --wasi-env LOG_LEVEL
  ```

  Without a `--wasi-*` flag, a guest gets no directory and no variable.
- **The guest asks in its block.** A `wasm:` block's `wasi:` key names what the guest gets:

  ```yaml
  wasm:
    module: indexer.wasm
    wasi:
      dirs: [{host: /srv/app/data/maps, guest: /maps, mode: ro},
             {host: /srv/app/cache, guest: /cache, mode: rw}]
      env: [DATABASE_URL]
      args: [--quiet]
  ```

  - Each directory must lie under a `--wasi-dir` root, with no wider mode: `ro` under an `ro`
    root. Each variable must be one a `--wasi-env` names, and its value is the process's.
  - A request outside the grant fails the load, naming the request and the grant. A guest
    doesn't run without its data. `arisrvx check` reports the same before serving.
  - The load logs each grant it makes: `wasi: indexer.wasm: /maps (ro) = /srv/app/data/maps`.
- **What each resource is.**
  - **Files:** the preopened directories only, as WASI's capability model has them.
    - A path resolves inside its preopen: wasmtime's `cap-std` lets no `..`, absolute path or
      symlink leave it.
    - `ro` refuses every write.
    - Nothing else of the filesystem exists for the guest.
  - **The environment and args:** only the named variables, and the block's args.
  - **Stdio:** stdout and stderr go to the relay's log, a line at a time, labelled with the module,
    under a byte cap per frame. stdin is empty.
  - **Clocks and random numbers:** always granted, as the DSL has `time-now` and `math.random`.
  - **The network:** not through WASI.
    - Preview 1 has no sockets, and preview 2's `wasi:sockets` needs the component model, whose C
      API isn't ready (§8.4).
    - A guest reaches the network through `call` and the net verbs (`http-*`, `ws-*`), under the
      host's allowlist (ARIADNE.md §10.3), which is already scoped.
    - `wasi:sockets` can come later under the same allowlist (`--wasi-net host:port`).
  - **Processes and signals:** none. `proc_exit` ends the guest's activation, not the server.
- **Stubs everywhere else.** A guest built for WASI imports `wasi_snapshot_preview1` whether or not
  it uses it. Where nothing is granted (a block without `wasi:`, a client host, a client's guest),
  the WASI imports are still linked, to stubs:
  - clocks, random numbers and `proc_exit` work;
  - args and the environment are empty;
  - every file and directory call returns `ENOTCAPABLE`.

  So one module loads everywhere, and gets resources only where they were granted.

  *As built (G6's first part):* `src/ariadne_wasm/wasi_preview1.cpp`, our own stubs on the runtime
  interface, so they stay backend-neutral. Every guest links all 46 functions. Where they differ
  from the list above:
  - argv is the module's file name, then the block's `wasi: args`;
  - `fd_prestat_get` and `fd_prestat_dir_name` return `EBADF`, because wasi-libc's start scans the
    preopens until `EBADF` and exits on any other errno;
  - fds 0-2 are character devices (`fd_fdstat_get`, `fd_filestat_get`): stdout and stderr are
    sinks that take every byte, and stdin is at its end. A call takes at most 1024 iovecs
    (wasi-libc's `IOV_MAX`), which bounds the host's work in it;
  - `poll_oneoff` takes clock subscriptions, and fires each at once, so a sleep returns at once and
    never blocks the loop; a poll on an fd is `ENOTCAPABLE`;
  - `proc_exit` ends the guest: wasi-libc's `exit` has run its teardown, so the instance goes, and
    every later call is a `WasmError` until the document reloads. A setup that exits fails the load.

  A block's `dirs:` or `env:` fails the load, naming the request, until `arisrvx` grants them.
- **Clients get none.** A client's guest (G4, §6.3) runs as that client's program, whatever the
  operator granted the server's own guests: no WASI files, environment or stdio. Its data comes
  through its bound subtree.
- **What it costs.** A WASI call is a host call. It is charged a step, plus a step per
  `kDataNodeBytes` it moves, against the activation's caps.
- **Open question: disk IO on the relay loop** (§9).
  - wasmtime's own WASI (`wasi_config_*`) runs each call in place, on the guest's stack. So a
    server guest's file read blocks the loop that steps the rooms (§6.2). The first cut caps the
    bytes a call moves.
  - One fix: implement preview 1's file calls as our own imports. Each would park the activation
    while `app.exec_pool()` does the IO, as an async native does.
  - The other: WASI 0.3's async calls, once wasmtime's C API has them.
  - Measure against the room step's budget before choosing.
- **The client hosts later.** The same grant shape (`--wasi-dir`, `--wasi-env`, a block's
  `wasi:`) serves `arix`, `ariglx` and §8.8's commands (`<host> run tool.wasm`), where the user is
  the operator.

### 8.7 Hooks

A guest hooks the document's events through the same lanes the DSL, Python and native code use. A
hook is declared in the block's `hooks:`, or registered by `ari_init` through the `hook` import.

| Event | DSL | Python (`Host`) | Native | Guest hook | Runs as |
|---|---|---|---|---|---|
| a button's or menu item's `on: event` | `on:` | `@host.on("event")` | `register_verb` | `on: {event: export}` | an action, under the action caps |
| every frame | `on_tick:` | `@host.on_frame` | `add_frame_hook` | `on_tick` | a resident on the tick channel: one tick per drain, coalesced |
| keys, pointer, signals | `on_key:`, `on_pointer:`, `on_signal:` | — | `post_input`, `post_signal` | `on_key`, `on_pointer`, `on_signal` | a resident per hook, fed the same event dicts beside the DSL's (`Runtime::set_event_tap`), every one in order |
| a value changes | `state-watch` | `@host.watch(key)` | a `cvc::state` change signal | `watch: {path: export}` | a resident woken by the change |
| a call from a program | `functions:` | `@host.function`, `@host.async_function` | `register_function` | `export:` | inside the calling program, as a sliced call |
| a message | `msg-recv` | — | `post_message` | `msg_recv: {channel: export}` | a resident per message; a lockless process waits in `msg-recv` for it |

- **Same events.** Events reach the guest's `ari_dispatch` as the same dicts the DSL reads (as CBOR
  maps), in the same order, after the same coalescing.
- **Same restart rules.** A guest's resident restarts as a DSL resident does: after 1, 2, 4 … 64
  drains while it keeps stopping soon after starting, and never again after 8 such stops in a row
  (ARIADNE.md §2.3). As built, a message hook's receiver does. A key, pointer or signal hook's
  resident ends with its queue, so a failed event ends only that event's activation, and the next
  event starts the next.
- **Python and native code** can call a guest's exports, and a guest can call theirs. The registry
  is one.

### 8.8 Installing and entering programs

- **The platform.** cvcpkg already has a `wasi` platform. A guest package is `platform: wasi`,
  `arch: wasm32`. It's built once and runs on every host.
- **The transform.** The package ships its module already transformed (§8.5): gas counters, plus
  Asyncify for the runtimes that need it. It also ships the original, which wasmtime can run with
  its own async support.
- **Install.** Query the catalog, fetch, and check the sha256. Cache it (in the user prefix
  natively, in OPFS in the browser), compile it, then enter it:
  - **a command** (exports `_start`) runs like a program: `<host> run tool.wasm [args]`. The host
    maps argv, the environment, preopened directories and stdio: `wasi_config_*` in wasmtime,
    browser_wasi_shim in the browser. It runs as an activation like any other, under the `init:`
    lane's caps unless the host gives it more.
  - **a plugin** (a reactor: `_initialize` plus its functions) registers its exports and hooks, as
    a `wasm:` block does.
- **The import list.** A package's manifest lists the host imports it needs, as the CNCF Wasm OCI
  layout lists imports and exports. The host checks them against what the lane grants (§8.6). It
  asks before it runs anything the user didn't install.
- **Prior art:**
  - Wasmer's registry and `@wasmer/sdk`. It runs only WASI / WASIX commands, and it needs
    cross-origin isolation.
  - YoWASP: tools shipped as wasm on PyPI and npm.
  - OCI artifacts through `wkg`.

  The Bytecode Alliance's warg registry was archived in 2025-07.

### 8.9 Building the runtimes from source

- **Today** `wasmtime` and `wasmer` repackage upstream's prebuilt C API archives. WAMR and
  WasmEdge build from source.
- **To close the loop,** cvcpkg already has the pieces:
  - the `rust` 1.90 recipe: the official prebuilt toolchain, including FreeBSD and NetBSD;
  - `tools/make_crate_packages.py`, which turns a `Cargo.lock` into `crate-*` packages checked
    against the lock's hashes;
  - `cargo --offline` builds from those packages, which wezterm already uses.
- **Size.**
  - wasmtime v45's workspace lock lists 507 registry crates, 64 of them already packaged.
  - Wasmer 7.1's lists 768, with 74 packaged.
  - The C API needs only a subset of either.
- **The payoff:** no network at build time, and wasmtime on FreeBSD and NetBSD, which upstream
  doesn't ship.
- **Order.** wasmtime first, and Wasmer only if something needs it. None of this blocks G1–G5
  (§10).

## 9. Risks and open questions

1. **Browser module size.** Measured in W1 (§5.3, CVCGL_WASM.md). The lean `ariglx.wasm` is
   89.2 MB raw and 16.5 MB brotli, plus a 2.4 MB brotli zip, against 54.2 MB raw for arigl from the
   same tree. The full tier is 190.4 MB raw (26.8 MB brotli). The stdlib is already a zip
   fetched with the module, without its tests. Levers left: the Asyncify remove list (−8%), and
   `-Oz` on the Python objects.
2. **Asyncify with CPython** (§5.3). Measured in W1: it halves the interpreter's speed. The
   remove-list fallback restores it, once the paths that could sleep under Python are audited.
3. **Python in a real browser.** Done in W1. ci.yml's `ariglx-web` job runs documents with Python
   in headless Chrome, under a strict CSP, through the CDP driver from the conference work. The
   link takes undefined symbols as errors since python312 3.12.10+cvc.23, so nothing in the module
   is left unresolved, and `xml.parsers.expat` and hashlib's SHA-2 work in the page (§5.1).
4. **Native packages are Linux-only.** `pycvc-cp312` and `pycvc-gl-cp312` are Linux-only on
   cvcpkg.org, so the `arix` and `ariglx` packages are too.
   - In-tree builds work elsewhere with `-DCVC_BUILD_PYCVC=ON`.
   - Windows packages wait on those two (N1); the Windows pycvc rebuild has known blockers.
     python312's DLLs have been in `bin\` since cy-pca/cvcpkg#542.
   - macOS `ariglx` also waits on the VTK and cvcGL macOS bundles.
5. **Startup time.** CPython init, then importing numpy and pycvc_gl, in the browser. W1 measured
   1.3–1.8 s for the imports, and 2.5–3.4 s from navigation to a running document with numpy,
   against 0.8 s for stock arigl's documents (§5.3). Next: precompile `.pyc` into the zip, or
   freeze the hot modules.
6. **Native trust UX** (§4.3). P1 needs no gate: Python comes only from a local top document.
   Before module blocks or remote documents run Python, decide whether `--python-allow` is
   enough or documents need signing.
7. **RestrictedPython 8.4 on wasm.** It's noarch on cvcpkg; confirm it imports in the browser
   build (it matters there only for §4.3's untrusted snippets).
8. **The extension rewriter tracks LLVM's object format** (version 2 of the `linking` section,
   §5.5). Pin it to the `emsdk` recipe, and run it in the recipes' CI.
9. **Runtime installs pin the ABI** (§5.6). Side modules match one Emscripten, so an emsdk bump
   republishes every side-module package.
10. **Dynamic loading with threads** is experimental in Emscripten, so wasm-mt gets no C-extension
    installs. Revisit when Pyodide or CPython supports the combination.
11. **Guest data copies** (§8.4.1). Host objects cross as handles, and a guest copies only the
    regions it reads. What is left to measure is the cost of a read: a volume-sized sweep in
    bricks, from wasmtime and from the browser, against the same sweep in the DSL and in pycvc.
    Measure it before promising guests for data processing.
12. **Where the wasm runtime lives** (§8.2). WAMR is small: cvcpkg's static build is a 355 KB
    archive, so it could go into the stock hosts. wasmtime is far larger and belongs in the `x`
    hosts. One runtime everywhere, or the choice per host?
13. **Asyncify's cost** (§8.5). It doubled a tiny test guest. Measure size and speed on real
    guests, and JSPI against it in the browser.
14. **Gas calibration** (§8.5). `wasm_gas_per_step` has to be measured, as
    `limits.async_steps_per_cpu_ms` was. A wrong rate makes guests too cheap or too dear against
    DSL programs under the same caps.
15. **Priorities need a policy** (§8.5). Under the stock hosts' `round_robin`, a guest's priority
    is stored but changes no pick, until a host sets `state_exec.policy`. Settled for the hosts by
    `--exec-policy`; `round_robin` stays the default.
16. **Uvicorn inside an embedded interpreter** (§6.5.2). Confirm on the pinned uvicorn
    (0.53.0) that `Server.serve()` off the main thread installs no signal handlers. Also confirm
    that `asyncio`'s default loop on the script thread coexists with the relay's own threads.
    Test it in S1 before relying on it. **Tested in S1** (2026-11-08, cvcpkg's python312 3.12.10,
    uvicorn 0.53.0, FastAPI 0.141.1, Starlette 1.7.0, on Windows; Linux in CI):
    - `capture_signals` returns at once off the main thread: SIGINT, SIGTERM and SIGBREAK keep
      their handlers, and the wakeup fd is untouched. The lifespan runs on the script thread, the
      requests are served there, and a graceful stop with a request in flight takes about 0.2 s.
    - **A finding:** a failed startup (a lifespan that raises, a port that is taken) ends with
      `sys.exit(3)`. When `serve()` is a task of its own, asyncio re-raises that SystemExit out of
      `run_until_complete` and every other task on the loop dies with it. `serve_asgi` catches it
      inside the task's own coroutine, and the other tasks run on.
    - arisrvx's tests serve FastAPI beside a live relay (its gRPC and WebSocket threads), and
      `serve_asgi`'s server never installs signal handlers on any thread.
17. **Sync exports stall everyone** (§6.5.1). One interpreter means one slow sync handler delays
    HTTP and room-call alike. The `--script-slow-ms` warning makes this visible, but it is a
    convention, not a guarantee. Services should be async throughout.
18. **Published state is not a database** (§6.5.3). Every byte under a published namespace is
    replicated to every reader. Size rules and the per-tick write budget need measured defaults.
    In S0 a namespace has no size cap, and the DSL's action caps are its only write budget.
19. **Native client transport security** (§6.5.7). Neither TLS route exists in the native
    clients yet. Until one does, staff consoles join through a proxy only from inside the
    network.
20. **WASI's disk IO on the relay loop** (§8.6.1). wasmtime's WASI runs a call in place, so a
    server guest's file read blocks the room step. Cap the bytes a call moves first, then measure
    whether file calls need to run on the pool as parked imports.

## 10. Phases

**The application-services cut** is S0 → S1 → S2 + S5 → S7. It is what cy-pca/cyprov needs to
start. S6 can follow, because cyprov's first consoles are staff-only. S3 and S4 aren't needed
for it.

| Phase | What | Needs | Size |
|---|---|---|---|
| P0 | **Done.** #985 merged (d0bb057d) and released in v3.7.11, so `Host`, functions with values and `AriApp` are on cvcpkg.org | review | S |
| P1 | **Done** (#985, v3.7.11). The engine, `_arihost`, `Host` embedded mode, the top document's `python:` block, the `python` / `python.<module>` tokens, `run app.py` and `<host> python`, `--python-path`; `ariglx` and `arix`, native, with Linux recipes. Tests: the `arix_*` / `ariglx_*` ctests and three `AriadneRequires` cases | P0 | L |
| P1b | The rest of P1. **Done:** the engine interface (§8.3): `ScriptEngine`, `ScriptBlock`, the `ScriptLane` with its function registry, activations and hooks, `Runtime::start_process`, Python's engine on it, and `ariadne_script_engine_test`. **Left:** `run -m`; `load:`ed modules' `python:` blocks, then `--python-allow`; pycvc's tests through the embedded interpreter; GRL-SNAM's ports on `ariglx` | P1 | M |
| P2 | `py_console`, then `py_editor` with jedi | P1 | M |
| W1 | **Done** (§5.1, §5.3). `ariglx-web` wasm: the combined link, a browser smoke in headless Chrome, size and Asyncify numbers, lean vs full. Since python312 3.12.10+cvc.23 (cy-pca/cvcpkg#565, #567) its link and link-host.sh's take undefined symbols as errors, and the configure refuses an older python312: cvc.25 (cy-pca/cvcpkg#581) with the VTK bridge | P1 | L |
| W2 | **Done** (cy-pca/cvcpkg#549, transfix/libcvc#1027; numpy-cp312 2.4.6+cvc.6 on cvcpkg.org for wasm and wasm-mt, 2026-11-07). The wasm C-extension convention in cvcpkg (§5.5): the rewriter and `inittab` manifests, numpy republished with them, and a host link that no longer allows duplicate extension symbols | — | S |
| W3 | **Done for ariglx-web's lean tier** (§5.4). The recipe's wasm-mt leg, a pool of 8 workers for Python's threads, the smoke's `--isolated` (COOP/COEP, and every case must find its page cross-origin isolated), and ci.yml's `ariglx-web` job over wasm and wasm-mt. vtk-python-cp312 wasm-mt was republished as `+cvc.8` (cy-pca/cvcpkg#543, #544), and the lean tier needs no pycvc-gl-cp312. Tests: `threads.ari` (Python threads and an async export on the pool), and arigl-web's wasm-mt package smoked `--isolated` in publish-cvcgl-wasm.yml. **Left:** a wasm-mt leg in publish-pycvc-gl-wasm.yml for the node-only pycvc_host package, the full tier on wasm-mt, and publishing ariglx-web | W1 | M |
| W4 | Runtime package installs, plain wasm (§5.6): a `MAIN_MODULE=2` `ariglx-web` variant, `link: shared` side-module packages, `host.install`, the cvcpkg.org origin allowlist | W1, W2 | L |
| W5 | pillow, then matplotlib (kiwisolver, contourpy, fonts) for wasm | W2 | M each |
| C1 | matplotlib in the `x` hosts (§4.6): the backend, `plt.show()` as windows, `display()`; native first. **First part built:** in `ariglx`, `module://pycvc_gl.ariadne_mpl` (the default), Agg draws published as `figure-<num>`, `plt.show()` as windows of `figures.ari` (a blocking one runs the host's loop), `bind()`, `draw_idle`, timers and `plt.pause()` on the frames, and `display()` (`pycvc_gl.ariadne_display`) for figures, pictures and text (printed in `arix`). Tests: `pycvc_gl_ariadne_mpl`, `ariglx_matplotlib_show`, `_close`, `_block`, `_nested`, `arix_matplotlib_backend`. **Left:** the browser (W5), VTK and mesh `display()` | P1 | M |
| C2 | Interactive figures: per-item wheel, item size, keys on hover, cursor in Ariadne; pan and zoom | C1 | S |
| N1 | `ariglx` on Windows and macOS. **Part 1 done:** Windows columns for `pycvc-cp31X` / `pycvc-gl-cp31X` (cp312 packed and smoke-tested locally; cp311/cp313 edited, not built), `arix`, `ariglx` and `arisrvx` (packed and smoke-tested locally). **Left:** the Windows publish lanes (part 2), cp311/cp313 verification, a wasmtime bundle with `wasmtime.dll` in `bin\` (arix and ariglx both ship that path until then), macOS | pycvc packages for those platforms | M |
| D1 | GRL-SNAM in the browser: torch-free inference, a demo page | W1 | M |
| D2 | VolRover gap audit, then VolRover as an Ariadne app | P2 | L |
| S0 | **Done** (transfix/libcvc#1046). Server documents in stock arisrv (`--doc`), plus the `server: publish:` namespaces (§6.1, §6.5.3): `ServerDocument`, the shard's outbound scope, the area of interest subscribed to it, the read-only `room` hole, the holon's world clock at `sim.tick`, the parsed `scene:` kept for the world runtime (with a warning that it is not simulated), `requires: [relay]`, `(quit)` as arisrv's exit status, `state-set-tree` through writable links on every hop, and first writes to fresh nodes journaled. Tests: `ServerBlockTest`, `ServerDocumentTest`, `StatePublishScopeTest`, `RoomRelayLoopbackTest.ServerDocumentPublishesItsNamespaceOnly`, the rev-before-fields order, the `arisrv_doc_*` ctests, `ari_server_block_ignored` and `ari_server_document_needs_relay` | — | M |
| S1 | **Done** (transfix/libcvc#1081). `arisrvx`: the engine on a script thread with an asyncio loop, operator Python (§6.5.1); `serve_asgi` / `--http-listen`, lifespan and graceful shutdown (§6.5.2); `arisrvx check`; the `arisrvx` recipe. As built: `pycvc.ariadne` built in (and shipped with pycvc for plain `python`), the ServerLink (writes coalesced per path in last-write order, `--script-write-budget`, the relay's events through a waker thread), the relay listening only once the ASGI apps have started, a stop that drains the program before the relay, `arisrvx python`, a server document's `python:` block refused, risk 16 tested. Tests: `ServerLinkTest`, the `arisrvx_*` ctests (serve, program, http_listen, startup_fails, check, check_missing_module, doc_python_refused, python_interpreter), `arisrv_python_needs_arisrvx`, `pycvc_ariadne_server`. `--script-slow-ms` moved to S2 | S0, P1 | M |
| S2 | **Done** (transfix/libcvc#1122). `room-call`: functions clients may call, with `ctx` (principal, progress, idempotency key, deadline), role gates and `CallError` (§6.5.5); the `relay:` block and `--relay` join mode in `ari` and the `x` hosts, and transport security for native clients (§6.5.7); `--script-slow-ms` (§6.5.1); a server document's `python:` block, its exports run on the script thread (§6.2). As built: DSL values typed across every hop (pycvc's conversion through `cvc.state_exec.value` capsules in-process, typed CBOR on the wire, symbols and bytes included), replies only to the connection holding the pid (`send_message_to_holder`), `--call-result-max`, `grpcs://` with `--tls-cert`/`--tls-key` (wss:// left to the conference card), a join that waits out the admission hold, `arisrvx check` without a program. Tests: `ServerLinkTest`, `RoomRelayLoopbackTest.ARoomCallReachesTheRelayAndOnlyItsCaller`, `RoomCallsTest.ParseCall`, `ValueCbor.*`, `arisrvx_doc_python`, `arisrvx_check_doc_python`, `arisrvx_room_call_{grpc,grpcs,tls_needs_its_ca,ws_arix}`, `arisrv_room_call_unavailable`, `pycvc_ariadne_server`. `token: oidc` left for later (see S5) | S1 | M |
| S3 | Client DSL programs | S0 | S |
| S4 | Restricted client Python in worker processes | S2 | L |
| S5 | **Done** (transfix/libcvc#1130). The `authenticate` hook and principals: bound writes, room-call and rate limits keyed by principal; `host.disconnect` (§6.5.4). As built: `state_transport::set_authenticator` on gRPC and WS (the verdict on the connection's own thread, before admission, in place of the room token), the gate's principal (takeover only by the same principal, `principal_of`), `--auth-timeout`, `--call-concurrency`, `--call-rate`, `disconnect_principal`, bearers up to 4096 characters, a refused or dropped join told at once. Tests: `StateTransportGrpcTest.InboundGateKeysANodeIdToItsPrincipal`, `StateTransportGrpcTest.AnAuthenticatorAdmitsAndBindsPrincipals`, `StateTransportWsTest.AnAuthenticatorJudgesTheHandshake`, `ServerLinkTest.ARequestWaitsForItsAnswerOrItsTimeout`, `arisrvx_auth_{staff,user_ws,refuses_wrong,refuses_slow,refuses_raises}`, `pycvc_ariadne_server`. Not in it: the client's `token: oidc` | S1 | S |
| S6 | **Built.** Bound reads (§6.5.6): `host.read_scope` and `host.refresh_scope`. As built: `state_transport::set_read_scope` and `refresh_read_scope` on gRPC and WS (a scoped connection held until its Hello sets its scope; the scope in the peer's `delivery_view` and checked under the write lock on every tree write; deletes for what it can no longer read, a snapshot of what it newly can), `state_prefix_filter`, `room_read_scope` (the participant's own subtree), the prefixes answered with the authenticate verdict (`ServerLink::set_scope`). Tests: `StateSyncAdapterTest.ARemoteDeleteTakesTheSubtreeOut`, `StatePrefixFilterTest`, `StatePeerRegistryTest.AScopedViewDeniesWhatItsScopeDoes`, `StateReadScopeTest.*`, `StateTransportGrpcTest.AReadScopeBoundsWhatAPrincipalReads`, `StateTransportWsTest.AReadScopeBoundsWhatAPrincipalReads`, `RoomCallsTest.AReadScopeKeepsTheParticipantsOwnSubtree`, `arisrvx_read_scope`, `arisrvx_read_scope_ws` | S5 | M |
| S7 | `ServerHarness` for plain `python`, and the end-to-end CI test (§6.5.8) | S2, S5 | S |
| L1 | Lua bindings (SWIG), the Lua engine in the `x` hosts, Lua for client code on `arisrvx` | the engine interface (P1b, done) | L |
| G1 | WebAssembly guests natively on wasmtime (§8.2–8.7), as a `ScriptEngine` on P1b's interface. **First part done** (§8.3): the runtime interface and its wasmtime backend, the `wasm:` block, the ABI with `call`/`reply`/`yield`/`fail`, exports as parked calls (`ScriptLane::export_activation`), fuel as steps under the lane's caps, the frame budget, one activation per instance, `imports:` narrowing, the `on_tick`/`on:`/`watch` hooks, `WasmError`, in `arix` and `ariglx`. **Second part done:** `ari_init` at load and calls from `init:` (`ScriptLane::run`). **Third part done:** host objects by handle, read a region at a time (§8.4.1), and `MemoryError`. **Fourth part done:** a call from a program as its own sliced native call (`SlicedCall`). **Fifth part done:** the pump budget counts what the slices charge. **Sixth part done:** the instance waited for through the lock manager (`ActivationOptions::lock`), and `RecursionError` for a call back into it. **Seventh part done:** the key, pointer, signal and message hooks (`Runtime::set_event_tap`, `ScriptLane::on_key` and the rest). **Eighth part done:** the state imports as fast paths beside `call` (`ScriptSlice::call_now`), and §8.4.1's writes (`volume_write`, `volume_new`, `data.write`). **Ninth part done:** wasmtime in the `arix` and `ariglx` recipes, linked by name. **Tenth part done:** `--exec-policy` and pure exports on the read lane (`ScriptLane::export_pure`). **Done.** | the engine interface (P1b, done) | L |
| G2 | The install-time transform (§8.5: gas injection, built through cvcpkg's Rust crates; Asyncify) and the browser backend: the same slices, limits, access and hooks in a single-threaded page | G1 | L |
| G3 | The WAMR backend, on G2's transform: guests on the BSDs | G2 | S |
| G4 | Client guests on `arisrvx` (§6.3, §8.6), with the optional Worker backstop in the browser | G1, S2 | M |
| G5 | Guest packages on cvcpkg (§8.8): the `wasi` platform, install and enter, the import manifest and consent | G1, G2 | M |
| G6 | WASI for server guests on `arisrvx` (§8.6.1): the operator's `--wasi-*` grant as the ceiling and a block's `wasi:` request within it, preopened directories, named variables, stdio to the log, clocks and random numbers; WASI stubs wherever nothing is granted. **First part done:** the preview-1 stubs every guest links, the block's `wasi:` request (its `dirs:` and `env:` refused until a grant exists). **Left:** arisrvx's `--wasi-*` ceiling and the grant, stdio to the log, the byte cap | G1, S2 | M |
| H1 | wasmtime from source (§8.9): crate packages, `cargo --offline`, 50.x, FreeBSD and NetBSD. Later; blocks nothing | — | M |
