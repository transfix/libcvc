# Ariadne scripting hosts: Python inside ari, arigl and arisrv

Status: design (2026-11-02). §4.6, §5.5, §5.6 and §8.2–8.7 were added on 2026-11-03. Nothing
here is built yet. Each phase in §10 becomes one or more PRs, and the user-facing reference moves
into [ARIADNE.md](../ARIADNE.md) as each one lands.

Python can already *host* a document. transfix/libcvc#985 (`pycvc_gl.ariadne.Host`) lets
`python app.py` open a window, load a `.ari` document and give it Python functions to call. The
other direction is missing. A document can't bring its own Python, a Python program can't run
in the browser host, and arisrv runs no scripts at all.

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

Every item was checked against master `dded8e56` and cvcpkg.org on 2026-11-02.

| Piece | Where | State |
|---|---|---|
| CPython 3.12, static, for wasm | `python312` | wasm `3.12.10+cvc.13`, wasm-mt `+cvc.11` |
| numpy for wasm | `numpy-cp312` 2.4.6 | wasm and wasm-mt. Its C extensions are relocatable wasm objects, linked statically and registered as builtins ([`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh)) |
| vtk-python for wasm | `vtk-python-cp312` 9.5.0 | wasm `+cvc.6` (rendering, IOHDF, IONetCDF); wasm-mt only `+cvc.2` |
| pycvc + pycvc_gl for wasm | `pycvc-gl-cp312` 3.7.8 | wasm only (87 MB of static archives). [`pycvc_host.cpp`](../../bindings/pycvc/wasm/pycvc_host.cpp) embeds them with numpy and vtk-python and prints `PYCVC_VTK_BRIDGE_OK`, **under node only**: no browser, no window |
| Python drives a document | [`pycvc_ari.i`](../../bindings/pycvc/pycvc_ari.i); #985 (open) | `AriRuntime` from Python today. #985 adds `Host` (window / offscreen / headless), functions with arguments and values, the GL-free `AriApp`, and GIL release in both directions |
| An app with an embedded interpreter | volrover `volrover3/docs/EMBEDDED_PYTHON.md` | `EmbeddedInterpreter`: one interpreter per process, GIL parked after boot, the live app handed to a host module (`vrhost`) by PyCapsule, a console dock. Qt-bound |
| The hosts | `src/ari`, `src/arigl`, `src/arisrv`; `arigl-web` | `ari` = libcvc + FTXUI/tvision. `arigl` = + cvcGL. `arisrv` = the conference relay, core libcvc only, **no document runtime**. `arigl-web` = wasm and wasm-mt pages |
| Native pycvc packages | `pycvc-cp312`, `pycvc-gl-cp312` | **Linux only** (+ wasm for pycvc-gl). No Windows or macOS |
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
- Documents never name a binary. They ask for `python` (later `lua`) in `requires:` (§4.1), so a
  rename costs nothing until the recipes are published.

Considered and dropped:

- `ari-py`, `arigl-py`, `arisrv-py`: they say Python, so a Lua engine would make them wrong.
- `aripy`, `ariglpy`, `arisrvpy`: the same problem, and `ariglpy` is hard to read.
- `pyari`, `pyarigl`: these read as Python packages for ari, which is what `pycvc_gl.ariadne`
  already is.
- `daedalus`, `daedalus-gl`, `daedalus-srv`: a brand of its own, but long and far from `ari`.

`arisrvx` links pycvc core, not pycvc_gl: the relay has no GL.

## 3. Architecture

### 3.1 One engine, linked into each `x` host

A new library, `cvc_ari_python` (`src/cvc/ariadne/python/`), is built when
`CVC_ARIADNE_PYTHON=ON`:

- **`cvc::ariadne::python_engine`** boots CPython once per process from a `PyConfig`:
  - isolated mode;
  - `home` is the install prefix, found from the executable, so the prefix's `python312`
    stdlib and site-packages are what loads (hermetic, no venv);
  - `--python-path DIR` adds directories.

  The lifecycle is a port of VolRover3's `EmbeddedInterpreter`: initialize on the main thread,
  park the GIL, finalize once.
- **The `_arihost` builtin module** is registered with `PyImport_AppendInittab` before
  initialization. It holds capsules for the live `cvc::app` and the `AppRuntime` / `AriRuntime`,
  plus `argv` and the host's name. It has no logic of its own; `pycvc_gl.ariadne` reads it (§3.2).
- **Installing into an `AppRuntime`:**
  - the `python` capability tokens (§4.1);
  - the loader hook for `python:` blocks (§4.2);
  - `py-eval` and `py-exec` verbs, for trusted documents only (§4.3);
  - each exported Python function as a host function, through #985's `register_function` /
    `register_async_function`.
- **A `script_engine` interface** (name, capability tokens, load a block, call a function,
  budget) sits between the hosts and Python, so Lua plugs in the same way (§8).

Each host keeps its one `main.cpp` and gains a second target built with `CVC_ARIADNE_PYTHON`
defined:

| Host | Is | Adds |
|---|---|---|
| `ariglx` | `arigl` | the engine, pycvc, pycvc_gl |
| `arix` | `ari` | the engine, pycvc |
| `arisrvx` | `arisrv` | the engine, pycvc, and the script service (§6) |

Natively they link the prefix's shared `libcvc`, `libcvcGL` and `libpython3.12`. So the
`_pycvc_gl` extension that Python imports binds to the **same** cvcGL as the host: one app, one
scene graph, one ImGui context. A second copy of cvcGL would mean a second ImGui context, and
raw ImGui calls from the wrong copy crash. In wasm everything is one static module anyway (§5).

A runtime-loaded Python plugin for the stock binaries was considered for native builds. wasm
has to be a separate link regardless, and separate binaries keep Python out of the stock
packages' install closure, so the variants are separate binaries everywhere.

### 3.2 The host module and `Host`'s embedded mode

#985's `pycvc_gl.ariadne.Host` gets a fourth mode, `embedded`:

- `Host()` inside an `x` host returns a Host over the host's own runtime (from `_arihost`), not
  a new window. An explicit `mode=` that contradicts it (`"headless"` in a window) raises with a
  message that says why.
- `pycvc_gl.ariadne.current_host()` returns that Host, or `None` under plain `python`.
- The rest of the API is unchanged: `function`, `async_function`, `on_frame`, `watch`, `set`,
  `set_tree`, `set_series`, `publish_image`, `get_tree`, `run_program`, `host.lab`.
- `run()` drives the host's loop natively, where the script is the program's main. In the
  browser the page owns the loop (§5.3), so `run()` returns at once and the work goes in
  `on_frame` and in functions.

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
| `python.numpy`, `python.vtk` | numpy; `vtkmodules` (the full browser tier, §5.2) |
| `python.<module>` | `<module>` is importable. Checked at load with `importlib.util.find_spec`, not imported |
| `python.console` | the console widget (§4.5) |

`(has? "python")` works as for any token, and `ari check` lists them. On a stock host a document
that requires `python` fails with one error that names the `x` host to use.

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

- **When.** The block runs once at load, before `init:`, in a module namespace of its own (one
  per document).
- **Exports.** `@export` and the `export:` names become host functions. Values cross through
  #985's conversion: numbers, strings, lists, dicts, and numpy arrays as lists or series.
  `@export(is_async=True)` makes an async function.
- **Errors.** A raise inside a call is a `PythonError` in the DSL, which a `try` can catch
  ([STATE_EXEC_EXCEPTIONS.md](../STATE_EXEC_EXCEPTIONS.md)). A raise at load fails the load and
  prints the traceback.
- **Caps.** #985 found that the default action caps (`ariadne.limits.action_max_time` 0.1 s,
  `action_max_steps` 5000; ARIADNE.md §2.2) stop any slow Python call, so `Host` raises them.
  An embedded host raises them the same way, and a document that loads Python can lower them
  again.
- **`load:`ed modules.** A module's `python:` block runs in its own namespace, and its exports
  are visible only inside that module, like its `functions:` (ARIADNE.md §6.6). Granting
  follows `functions:`.
- **Reload.** A hot reload runs the block again in a fresh namespace and reloads the modules
  under the document's own `path:` entries. Packages from the prefix are not reloaded.

### 4.3 Trust

Natively, Python in a document runs with the user's full authority: it can `import os`. Unlike
the DSL it has no chroot. So:

- **Native.** A document's Python runs only when the document is a local file. A document
  fetched over http(s), or a `load:` of one, that has a `python:` block fails to load unless
  `--python-allow ORIGIN` names its origin, as `--net-allow` does for the network. `ari check`
  says which origin it would need.
- **Browser.** The wasm sandbox is the boundary, as it is for the page's own JavaScript. The
  page's `ariadne.config.json` lists any origins beyond its own in `python_origins`.
- **Untrusted Python** (a snippet a user typed, a shared notebook cell) never goes through
  `python:`. It uses the restricted engine from §6.4.

### 4.4 A script as the entry point

- `ariglx run app.py [args]` runs `app.py` as `__main__`, with `sys.argv` set to
  `[app.py, args...]`. Typically the script does `host = Host()`, registers functions, then
  `host.load("app.ari")` and `host.run()`.
- `ariglx run -m package.module [args]` does the same for a module, e.g.
  `ariglx run -m grl_snam.ariadne live austin-freedrive`.
- `ariglx run main.ari` loads a document as usual.
- `ariglx python [args]` is the embedded interpreter as a plain `python3` (a REPL, `-c`, `-m`),
  so scripts and tests use exactly the host's packages, e.g. `ariglx python -m pytest`.

### 4.5 The console widget

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
- **Duplicates are hidden today.**
  - numpy's 14 extensions define 799 of the same global names: npymath and the random
    distributions are copied into several of them.
  - So [`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh) links with
    `--allow-multiple-definition`.
  - That flag also hides real clashes. Two packages with different code under one name would link,
    and one would silently run the other's.
- **Binary tools can't fix it.**
  - `llvm-objcopy` refuses every symbol option on wasm objects, in emsdk's LLVM 23 and on LLVM
    main.
  - `wasm-ld` has no version scripts and no `--exclude-libs`.
  - Natively, Buck2 renames with `objcopy --redefine-syms`; kivy-ios rewrites Cython's output.

**The fix, proven on 2026-11-03:**

- **Where it edits.** Each extension is one relocatable object (emcc `-shared` without
  `SIDE_MODULE`). Its symbol table lives in the `linking` custom section that
  [tool-conventions/Linking.md](https://github.com/WebAssembly/tool-conventions/blob/main/Linking.md)
  documents. A rewriter of about 150 lines of Python edits that table:
  - `PyInit_<leaf>` becomes `PyInit__x_<dotted name>`, with `.` written as `__`;
  - every other defined global becomes local, so it's private, as `RTLD_LOCAL` makes it natively;
  - COMDAT groups get a per-extension prefix, so C++ inline and template code stays per extension.

  References inside an object go by symbol index, so they survive the rename.
- **The test.** A minimal CPython host linked numpy 2.4.6's 14 extensions with no
  `--allow-multiple-definition` and ran them under node in both flavors:
  - plain wasm: core, random, linalg and fft;
  - wasm-mt: two Python threads doing linalg and random at once.

  The rewrite covered 395 COMDAT groups. The unmodified objects fail that link with
  duplicate-symbol errors.

**The convention:**

- **Recipes.** A wasm C-extension recipe runs the rewriter on each extension. It ships the objects
  plus `share/python-wasm/<pkg>.inittab`, which maps each dotted module name to its renamed
  `PyInit_` symbol.
- **The host link.** It generates one registrar from every manifest in the deps prefix. A clash
  between two extensions is then a link error.
- **Debug names.** Strip each object's `name` section, or prefix the names in it. After the
  rewrite several objects carry the same debug names, which `llvm-nm` rejects in a combined `-r`
  link. The final link is fine either way.
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

arisrv runs no document today: it is a fixed relay. Step S0 gives it one. `arisrv --doc
server.ari` runs a headless `AppRuntime` (no backend, like #985's `AriApp`) on the relay's app,
for room rules, bots and checks written in the DSL.

That belongs in the stock arisrv, because the DSL is already sandboxed: step and time caps, the
chroot, and bound writes.

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
fraction of the cost: Lua through an allocator cap and an instruction hook, guests through fuel
and a memory cap. Guests also take any language that compiles to wasm, and they run the same in
the browser. Once either is in, it's the better engine for client code, and RestrictedPython stays
for clients that need Python's libraries.

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
- are sandboxed with real CPU and memory limits;
- behave the same natively and in the browser;
- can be installed from cvcpkg while the host runs (§8.6).

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
  export: [score_route]
  limits: {fuel: 50000000, memory_mb: 64, time: 0.5}
init: (state-set "score" (score_route (state-get "route")))
```

- **One registry.** The DSL, Python and guests share one function registry. A guest calls Python,
  and Python calls a guest, both through `register_function`.
- **Which hosts.** A guest brings no interpreter, so it doesn't need an `x` host. Which hosts
  carry a runtime is an open question (§9).

### 8.3 Runtimes, behind one interface

A `cvc::ariadne::wasm_runtime` interface covers load, instantiate with imports, call, and limits.
proxy-wasm's C++ host is the model: it puts V8, WAMR, WasmEdge and wasmtime behind one interface.

- **wasmtime** on Linux, macOS and Windows.
  - cvcpkg publishes 45 from upstream's prebuilt C API; upstream is at 50.0.1.
  - It has fuel, epoch deadlines, and a store limiter for memory, plus WASI preview 1 and 2.
- **WAMR** for the BSDs, and as the small option.
  - Pure C with CMake, and it has a FreeBSD port.
  - Published at 2.4.4, with the interpreter, fast interpreter, AOT and WASI.
  - Its instruction metering works only in the interpreter, and its WASI isn't supported on
    Windows (wasmtime covers Windows).
- **The browser's own engine,** reached from the Emscripten module through `EM_JS`.
  - The guest's imports call our module's exports, and bytes copy between the two memories
    (`HEAPU8.set`).
  - Untrusted guests run in a Web Worker so they can be killed.
  - Chrome compiles modules over 8 MB only asynchronously on the main thread.
    `compileStreaming` caches compiled code for modules of 128 kB or more.
- **The rest.**
  - Wasmer (7.1) and WasmEdge (0.17) are on cvcpkg too, but neither adds a platform that
    wasmtime and WAMR don't cover.
  - wasm3 restarted development in 2026. It's a lighter interpreter if WAMR disappoints.

### 8.4 The guest ABI and the state tree

- **Core wasm, in the style of Extism and proxy-wasm.**
  - Values cross as a pointer and a length, encoded as CBOR that mirrors state-tree values.
  - The guest exports `ari_alloc`, `ari_init` and `ari_dispatch`; `ari_dispatch` takes calls and
    events.
- **Host imports** (module `ari`):
  - `state_get` and `state_set`;
  - `state_watch`, whose events come back through `ari_dispatch`;
  - `call`, for any registered host function;
  - `emit` and `log`.
- **No pointers into the host.** A guest's state access goes through the same gates as a
  client's: bound writes, `room_write_gate`, and the capabilities its manifest asks for. Extism's
  `allowed_hosts` / `allowed_paths` and Zellij's plugin permissions are the same idea.
- **Threads.** Guests are single-threaded and are called on the runtime thread, during the drain.
- **Bulk data.** A volume or a mesh is copied between memories. That's fine for control logic. A
  host-owned blob API that reads in chunks comes when a guest needs one.
- **WIT later.** The interface is also written in WIT, so a component-model adapter can come later.
  Not yet:
  - wasmtime's C API still has gaps in the component model, including the async half
    (bytecodealliance/wasmtime#13705);
  - only wasmtime and jco implement it;
  - the browser needs jco's generated JavaScript.

  WASI 0.3 (async, streams) was ratified in 2026-06.

### 8.5 Limits

- **Natively:** wasmtime's fuel, epoch deadline and memory limiter, and WAMR's interpreter
  metering.
- **The browser has no fuel.**
  - So gas counters are injected into the guest's bytecode when it's installed
    (paritytech/wasm-instrument). The same limit then holds natively and in the browser.
  - The hard stop is `Worker.terminate` in the browser, and dropping the store natively.
- **Client code (§6.3, tier 3).** This gives tier 3 in-process hard limits. A client can send a
  guest, or call one, and spend its fuel and memory and nothing else. Even client Python can run
  this way, in a CPython or MicroPython compiled to WASI, which is cheaper than §6.4's worker
  processes.

### 8.6 Installing and entering programs

- **The platform.** cvcpkg already has a `wasi` platform. A guest package is `platform: wasi`,
  `arch: wasm32`. It's built once and runs on every host.
- **Install.** Query the catalog, fetch, and check the sha256. Cache it (in the user prefix
  natively, in OPFS in the browser), compile it, then enter it:
  - **a command** (exports `_start`) runs like a program: `<host> run tool.wasm [args]`. The host
    maps argv, the environment, preopened directories and stdio: `wasi_config_*` in wasmtime,
    browser_wasi_shim in the browser.
  - **a plugin** (a reactor: `_initialize` plus its functions) registers its exports, as a
    `wasm:` block does.
- **The import list.** A package's manifest lists the host imports it needs, as the CNCF Wasm OCI
  layout lists imports and exports. The host checks them, and asks before it runs anything the
  user didn't install.
- **Prior art:**
  - Wasmer's registry and `@wasmer/sdk`. It runs only WASI / WASIX commands, and it needs
    cross-origin isolation.
  - YoWASP: tools shipped as wasm on PyPI and npm.
  - OCI artifacts through `wkg`.

  The Bytecode Alliance's warg registry was archived in 2025-07.

### 8.7 Building the runtimes from source

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

1. **Browser module size.** arigl.wasm is 14.6 MB gzipped. The September node harness was 158 MB
   uncompressed, with the stdlib embedded in MEMFS and the full vtk-python. Measure lean and full.
   Levers: the stdlib as a lazily fetched zip, no test modules, `-Oz` on the Python objects.
2. **Asyncify with CPython** (§5.3).
3. **Python has never run in a real browser here.** The wasm harness only ran under node,
   headless. W1's first milestone is a browser page with a window, checked in headless Chrome
   with the CDP driver from the conference work.
4. **Native packages are Linux-only.** `pycvc-cp312` and `pycvc-gl-cp312` are Linux-only on
   cvcpkg.org, so `ariglx` on Windows and macOS waits on those packages. The Windows pycvc
   rebuild has known blockers.
5. **Startup time.** CPython init, then importing numpy and pycvc_gl, in the browser. Measure,
   then precompile `.pyc` into the zip or freeze the hot modules.
6. **Native trust UX** (§4.3). Is `--python-allow` enough, or do documents need signing?
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

## 10. Phases

| Phase | What | Needs | Size |
|---|---|---|---|
| P0 | #985 merged and released, so `Host`, functions with values and `AriApp` are on cvcpkg.org | review | S |
| P1 | The engine, `_arihost`, `Host` embedded mode, the `python:` block, the `python` tokens, `--python-allow`, `run app.py` / `run -m` / `python`; `ariglx` and `arix`, native Linux, with recipes. Tests: `ari test` documents that call Python, pycvc's tests through the embedded interpreter, GRL-SNAM's ports on `ariglx` | P0 | L |
| P2 | `py_console`, then `py_editor` with jedi | P1 | M |
| W1 | `ariglx-web` wasm: the combined link, a browser smoke in headless Chrome, size and Asyncify numbers, lean vs full | P1 | L |
| W2 | The wasm C-extension convention in cvcpkg (§5.5): the rewriter and `inittab` manifests, numpy republished with them, and a host link that no longer allows duplicate extension symbols | — | S |
| W3 | wasm-mt: vtk-python wasm-mt republished, pycvc-gl wasm-mt | W1, cy-pca/cvcpkg#109 | M |
| W4 | Runtime package installs, plain wasm (§5.6): a `MAIN_MODULE=2` `ariglx-web` variant, `link: shared` side-module packages, `host.install`, the cvcpkg.org origin allowlist | W1, W2 | L |
| W5 | pillow, then matplotlib (kiwisolver, contourpy, fonts) for wasm | W2 | M each |
| C1 | matplotlib in the `x` hosts (§4.6): the backend, `plt.show()` as windows, `display()`; native first | P1 | M |
| C2 | Interactive figures: per-item wheel, item size, keys on hover, cursor in Ariadne; pan and zoom | C1 | S |
| N1 | `ariglx` on Windows and macOS | pycvc packages for those platforms | M |
| D1 | GRL-SNAM in the browser: torch-free inference, a demo page | W1 | M |
| D2 | VolRover gap audit, then VolRover as an Ariadne app | P2 | L |
| S0 | Server documents in stock arisrv (`--doc`) | — | M |
| S1 | `arisrvx`: the engine on a script thread, operator Python | S0, P1 | M |
| S2 | `room-call`: functions clients may call | S1 | M |
| S3 | Client DSL programs | S0 | S |
| S4 | Restricted client Python in worker processes | S2 | L |
| L1 | Lua bindings (SWIG), the Lua engine in the `x` hosts, Lua for client code on `arisrvx` | the `script_engine` seam from P1 | L |
| G1 | WebAssembly guests natively (§8.2–8.4): the runtime interface, wasmtime, the guest ABI, the `wasm:` block | the `script_engine` seam from P1 | L |
| G2 | Guests in the browser: `EM_JS` instantiation, a Worker for untrusted guests | G1 | M |
| G3 | The WAMR backend: guests on the BSDs | G1 | S |
| G4 | Limits everywhere (§8.5): gas injection at install, fuel and memory caps; client guests on `arisrvx` | G1, S2 | M |
| G5 | Guest packages on cvcpkg (§8.6): the `wasi` platform, install and enter, the import manifest and consent | G1, G2 | M |
| H1 | wasmtime from source (§8.7): crate packages, `cargo --offline`, 50.x, FreeBSD and NetBSD. Later; blocks nothing | — | M |
