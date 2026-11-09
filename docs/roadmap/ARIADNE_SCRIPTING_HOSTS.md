# Ariadne scripting hosts: Python inside ari, arigl and arisrv

Status:
- **P1 shipped** in transfix/libcvc#985 (d0bb057d, released in v3.7.11): `arix` and `ariglx`,
  natively, with Linux packages on cvcpkg.org. §3 and §4 describe what shipped and mark what
  didn't. The user-facing reference is [ARIADNE.md](../ARIADNE.md) §9.6.
- **S0 shipped** in transfix/libcvc#1046: `arisrv --doc` and `server: publish:` (§6.1, §6.5.3).
- **The rest is design** (2026-11-02; §4.6, §5.5, §5.6 and §8.2–8.9 added 2026-11-03;
  §6.5, application services: an ASGI API and live state from one `arisrvx`, and phases
  S5–S7 added 2026-11-05).
- Each phase in §10 becomes one or more PRs, and the reference moves into ARIADNE.md as each one
  lands.

Python can *host* a document: `pycvc_gl.ariadne.Host` (#985) lets `python app.py` open a window,
load a `.ari` document and give it Python functions to call. Since P1 the other direction works
natively. In `arix` and `ariglx` a document can carry Python, and a Python program can be the
host's entry point. Still missing: Python on arisrv. arisrv runs DSL server documents (S0).

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
| vtk-python for wasm | `vtk-python-cp312` 9.5.0 | wasm `+cvc.8` (rendering, IOHDF, IONetCDF); wasm-mt only `+cvc.2` |
| pycvc + pycvc_gl for wasm | `pycvc-gl-cp312` 3.7.8 | wasm only (87 MB of static archives). [`pycvc_host.cpp`](../../bindings/pycvc/wasm/pycvc_host.cpp) embeds them with numpy and vtk-python and prints `PYCVC_VTK_BRIDGE_OK`, **under node only**: no browser, no window |
| Python drives a document | [`pymod_gl/ariadne.py`](../../bindings/pycvc/pymod_gl/ariadne.py) (#985, merged) | `Host` (window / offscreen / headless / embedded), functions with arguments and values, the GL-free `AriApp`, and GIL release in both directions |
| The scripting hosts (P1) | `arix`, `ariglx`: `src/ari`, `src/arigl` with [`src/ariadne_python`](../../src/ariadne_python) | Native. `arix` and `ariglx` 3.7.11 are on cvcpkg.org for Linux. No `arisrvx` |
| The browser host (W1) | `ariglx-web`: [`cvcpkg/recipes/ariglx-web`](../../cvcpkg/recipes/ariglx-web/recipe.yaml) | Single-threaded wasm, lean tier, checked in headless Chrome (§5.1). Not on cvcpkg.org yet |
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
    standalone recipe build, adds the `arix` / `ariglx` targets.
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
- **What it installs:**
  - the `python` tokens, process-wide, through `set_script_engine("python", ...)` (§4.1);
  - the `python:` block parser, process-wide, through `register_ari_block` (§4.2);
  - per runtime, through `attach()`: the host's handles lent to Python, raised action caps (§4.2),
    and a load hook (`AppRuntime::add_load_hook`) that runs the block;
  - exports as host functions, through `pycvc.host_function` and `register_host_function`, from
    Python.
- **Not built:**
  - the `py-eval` / `py-exec` verbs;
  - a `script_engine` interface (name, tokens, load a block, call a function, budget).

  P1's seams are the token registry (`set_script_engine`, `has_script_engine`) and
  `AppRuntime::add_load_hook` / `add_frame_hook`. Lua (§8.1) and wasm guests (§8.2) need the
  interface (P1b, §10).

Each host keeps its one `main.cpp`, and the `x` target compiles it with `CVC_ARIADNE_PYTHON`
defined:

| Host | Is | Links | Runtime packages |
|---|---|---|---|
| `ariglx` | `arigl` | the engine, libpython | `python312`, `numpy-cp312`, `pycvc-cp312`, `pycvc-gl-cp312` |
| `arix` | `ari` | the engine, libpython | `python312`, `numpy-cp312`, `pycvc-cp312`; a Python program also needs `pycvc-gl-cp312`, since `Host` is pycvc_gl's |
| `arisrvx` | `arisrv` | not built (S1) | |

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
  hosts, arix or ariglx.
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

- **wasm first.** Every dependency is published single-threaded.
- **wasm-mt waits on two things:**
  - vtk-python-cp312 wasm-mt is stuck at `+cvc.2`. The fix that stops it building VTK's
    interpreter executable (cy-pca/cvcpkg#109) needs a republish.
  - pycvc-gl-cp312 has no wasm-mt build. The lean tier needs only this one.

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
  document works as in §4.1), ports that are already bound, and flag errors.
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

A `cvc::ariadne::wasm_runtime` interface covers:

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
  - `gas`, the check the install-time transform inserts (§8.5).
- **No pointers into the host.** A guest reaches state only through those imports.
- **Bulk data.** A volume or a mesh is copied between memories. That's fine for control logic. A
  host-owned blob API that reads in chunks comes when a guest needs one.
- **WIT later.** The interface is also written in WIT, so a component-model adapter can come later.
  Not yet:
  - wasmtime's C API still has gaps in the component model, including the async half
    (bytecodealliance/wasmtime#13705);
  - only wasmtime and jco implement it;
  - the browser needs jco's generated JavaScript.

  WASI 0.3 (async, streams) was ratified in 2026-06.

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
  in G1.

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
  - A refused `memory.grow` is a `MemoryError`.
  - A trap is a `WasmError`, which a calling program's `try` can catch.

**Many guests at once, without choking a frame.**

- **Slices interleave.** A guest gives the thread back after every slice, so the scheduler
  interleaves it with every other process. One that waits on a host call parks and costs nothing.
- **Quick calls finish in the drain.** An `on:` handler that needs a few slices completes in the
  drain that fired it, as a quick DSL action does.
- **Long runs yield the frame.** Guest activations together get at most
  `ariadne.limits.wasm_frame_budget` of a frame (8 ms by default). Once it's spent, the rest yield
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
  §2.2), and they have no option to change it. G1 adds a host option for `state_exec.policy`.

**One activation per instance.**

- An instance's memory and stack belong to the activation in flight, and wasmtime allows one call
  future per store. So an instance runs one activation at a time.
- The others wait for it as a DSL program waits for a lock: through the app's lock manager, with
  its timeouts (`ariadne.exec.lock_timeout` for an action, none for a resident) and its priority
  inheritance.
- A call back into the same instance from inside one of its own host calls is refused with a
  `RecursionError`.

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
    `lock`, `call` and `net`.
  - An import outside them is linked to a stub that raises `PermissionError`, so a guest fails
    closed, and the manifest check (§8.8) says which import it lacked.
  - Narrowing never widens: a group the lane doesn't have stays out.
- **The read lane.** An export marked `pure: true` can be called from a reactive slot. It's linked
  with the read lane's allowlist (the state readers, no writes, no messages, no `call` beyond pure
  functions) and runs under that lane's caps.
- **Clients on `arisrvx`.** A client's guest runs as that client's program (§6.3, tier 2): chrooted
  to its bound subtree, under the bound-writes gate.

### 8.7 Hooks

A guest hooks the document's events through the same lanes the DSL, Python and native code use. A
hook is declared in the block's `hooks:`, or registered by `ari_init` through the `hook` import.

| Event | DSL | Python (`Host`) | Native | Guest hook | Runs as |
|---|---|---|---|---|---|
| a button's or menu item's `on: event` | `on:` | `@host.on("event")` | `register_verb` | `on: {event: export}` | an action, under the action caps |
| every frame | `on_tick:` | `@host.on_frame` | `add_frame_hook` | `on_tick` | a resident on the tick channel: one tick per drain, coalesced |
| keys, pointer, signals | `on_key:`, `on_pointer:`, `on_signal:` | — | `post_input`, `post_signal` | `on_key`, `on_pointer`, `on_signal` | residents on the same channels, with the same event dicts |
| a value changes | `state-watch` | `@host.watch(key)` | a `cvc::state` change signal | `state_watch` | a resident woken by the change |
| a call from a program | `functions:` | `@host.function`, `@host.async_function` | `register_function` | `export:` | inside the calling program, as a sliced call |
| a message | `msg-recv` | — | `post_message` | `msg_recv` | the activation parks until it comes |

- **Same events.** Events reach the guest's `ari_dispatch` as the same dicts the DSL reads (as CBOR
  maps), in the same order, after the same coalescing.
- **Same restart rules.** A guest's resident restarts as a DSL resident does: after 1, 2, 4 … 64
  drains while it keeps stopping soon after starting, and never again after 8 such stops in a row
  (ARIADNE.md §2.3).
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
11. **Guest data copies** (§8.4). Every value crosses between memories. Measure with a volume-sized
    payload before promising guests for data processing.
12. **Where the wasm runtime lives** (§8.2). WAMR is small: cvcpkg's static build is a 355 KB
    archive, so it could go into the stock hosts. wasmtime is far larger and belongs in the `x`
    hosts. One runtime everywhere, or the choice per host?
13. **Asyncify's cost** (§8.5). It doubled a tiny test guest. Measure size and speed on real
    guests, and JSPI against it in the browser.
14. **Gas calibration** (§8.5). `wasm_gas_per_step` has to be measured, as
    `limits.async_steps_per_cpu_ms` was. A wrong rate makes guests too cheap or too dear against
    DSL programs under the same caps.
15. **Priorities need a policy** (§8.5). Under the stock hosts' `round_robin`, a guest's priority
    is stored but changes no pick, until a host sets `state_exec.policy`.
16. **Uvicorn inside an embedded interpreter** (§6.5.2). Confirm on the pinned uvicorn
    (0.53.0) that `Server.serve()` off the main thread installs no signal handlers. Also confirm
    that `asyncio`'s default loop on the script thread coexists with the relay's own threads.
    Test it in S1 before relying on it.
17. **Sync exports stall everyone** (§6.5.1). One interpreter means one slow sync handler delays
    HTTP and room-call alike. The `--script-slow-ms` warning makes this visible, but it is a
    convention, not a guarantee. Services should be async throughout.
18. **Published state is not a database** (§6.5.3). Every byte under a published namespace is
    replicated to every reader. Size rules and the per-tick write budget need measured defaults.
    In S0 a namespace has no size cap, and the DSL's action caps are its only write budget.
19. **Native client transport security** (§6.5.7). Neither TLS route exists in the native
    clients yet. Until one does, staff consoles join through a proxy only from inside the
    network.

## 10. Phases

**The application-services cut** is S0 → S1 → S2 + S5 → S7. It is what cy-pca/cyprov needs to
start. S6 can follow, because cyprov's first consoles are staff-only. S3 and S4 aren't needed
for it.

| Phase | What | Needs | Size |
|---|---|---|---|
| P0 | **Done.** #985 merged (d0bb057d) and released in v3.7.11, so `Host`, functions with values and `AriApp` are on cvcpkg.org | review | S |
| P1 | **Done** (#985, v3.7.11). The engine, `_arihost`, `Host` embedded mode, the top document's `python:` block, the `python` / `python.<module>` tokens, `run app.py` and `<host> python`, `--python-path`; `ariglx` and `arix`, native, with Linux recipes. Tests: the `arix_*` / `ariglx_*` ctests and three `AriadneRequires` cases | P0 | L |
| P1b | The rest of P1: `run -m`; `load:`ed modules' `python:` blocks, then `--python-allow`; the `script_engine` interface; pycvc's tests through the embedded interpreter; GRL-SNAM's ports on `ariglx` | P1 | M |
| P2 | `py_console`, then `py_editor` with jedi | P1 | M |
| W1 | **Done** (§5.1, §5.3). `ariglx-web` wasm: the combined link, a browser smoke in headless Chrome, size and Asyncify numbers, lean vs full. Since python312 3.12.10+cvc.23 (cy-pca/cvcpkg#565, #567) its link and link-host.sh's take undefined symbols as errors, and the configure refuses an older python312: cvc.25 (cy-pca/cvcpkg#581) with the VTK bridge | P1 | L |
| W2 | **Done** (cy-pca/cvcpkg#549, transfix/libcvc#1027; numpy-cp312 2.4.6+cvc.6 on cvcpkg.org for wasm and wasm-mt, 2026-11-07). The wasm C-extension convention in cvcpkg (§5.5): the rewriter and `inittab` manifests, numpy republished with them, and a host link that no longer allows duplicate extension symbols | — | S |
| W3 | wasm-mt: vtk-python wasm-mt republished, pycvc-gl wasm-mt | W1, cy-pca/cvcpkg#109 | M |
| W4 | Runtime package installs, plain wasm (§5.6): a `MAIN_MODULE=2` `ariglx-web` variant, `link: shared` side-module packages, `host.install`, the cvcpkg.org origin allowlist | W1, W2 | L |
| W5 | pillow, then matplotlib (kiwisolver, contourpy, fonts) for wasm | W2 | M each |
| C1 | matplotlib in the `x` hosts (§4.6): the backend, `plt.show()` as windows, `display()`; native first | P1 | M |
| C2 | Interactive figures: per-item wheel, item size, keys on hover, cursor in Ariadne; pan and zoom | C1 | S |
| N1 | `ariglx` on Windows and macOS | pycvc packages for those platforms | M |
| D1 | GRL-SNAM in the browser: torch-free inference, a demo page | W1 | M |
| D2 | VolRover gap audit, then VolRover as an Ariadne app | P2 | L |
| S0 | **Done** (transfix/libcvc#1046). Server documents in stock arisrv (`--doc`), plus the `server: publish:` namespaces (§6.1, §6.5.3): `ServerDocument`, the shard's outbound scope, the area of interest subscribed to it, the read-only `room` hole, the holon's world clock at `sim.tick`, the parsed `scene:` kept for the world runtime (with a warning that it is not simulated), `requires: [relay]`, `(quit)` as arisrv's exit status, `state-set-tree` through writable links on every hop, and first writes to fresh nodes journaled. Tests: `ServerBlockTest`, `ServerDocumentTest`, `StatePublishScopeTest`, `RoomRelayLoopbackTest.ServerDocumentPublishesItsNamespaceOnly`, the rev-before-fields order, the `arisrv_doc_*` ctests, `ari_server_block_ignored` and `ari_server_document_needs_relay` | — | M |
| S1 | `arisrvx`: the engine on a script thread with an asyncio loop, operator Python (§6.5.1); `serve_asgi` / `--http-listen`, lifespan and graceful shutdown (§6.5.2); `arisrvx check`; the `arisrvx` recipe | S0, P1 | M |
| S2 | `room-call`: functions clients may call, with `ctx` (principal, progress, idempotency key, deadline), role gates and `CallError` (§6.5.5); the `relay:` block and `--relay` join mode in `ari` and the `x` hosts, and transport security for native clients (§6.5.7) | S1 | M |
| S3 | Client DSL programs | S0 | S |
| S4 | Restricted client Python in worker processes | S2 | L |
| S5 | The `authenticate` hook and principals: bound writes, room-call and rate limits keyed by principal; `host.disconnect` (§6.5.4) | S1 | S |
| S6 | Bound reads: `read_scope`, the per-connection prefix filter, `refresh_scope` (§6.5.6) | S5 | M |
| S7 | `ServerHarness` for plain `python`, and the end-to-end CI test (§6.5.8) | S2, S5 | S |
| L1 | Lua bindings (SWIG), the Lua engine in the `x` hosts, Lua for client code on `arisrvx` | the `script_engine` interface (P1b) | L |
| G1 | WebAssembly guests natively on wasmtime (§8.2–8.7): the runtime interface; the guest ABI; activations as app-scheduler processes in slices from wasmtime's async C API (its fuel until G2's gas), under the DSL's limits; the frame budget; one activation per instance; the lane's access surface; hooks; the `wasm:` block; a host option for `state_exec.policy`; a pump budget that counts guest slices' steps | the `script_engine` interface (P1b) | L |
| G2 | The install-time transform (§8.5: gas injection, built through cvcpkg's Rust crates; Asyncify) and the browser backend: the same slices, limits, access and hooks in a single-threaded page | G1 | L |
| G3 | The WAMR backend, on G2's transform: guests on the BSDs | G2 | S |
| G4 | Client guests on `arisrvx` (§6.3, §8.6), with the optional Worker backstop in the browser | G1, S2 | M |
| G5 | Guest packages on cvcpkg (§8.8): the `wasi` platform, install and enter, the import manifest and consent | G1, G2 | M |
| H1 | wasmtime from source (§8.9): crate packages, `cargo --offline`, 50.x, FreeBSD and NetBSD. Later; blocks nothing | — | M |
