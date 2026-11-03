# Ariadne scripting hosts: Python inside ari, arigl and arisrv

Status: design (2026-11-02). Nothing here is built yet. Each phase in §10 becomes one or more
PRs, and the user-facing reference moves into [ARIADNE.md](../ARIADNE.md) as each one lands.

Python can already *host* a document. transfix/libcvc#985 (`pycvc_gl.ariadne.Host`) lets
`python app.py` open a window, load a `.ari` document and give it Python functions to call. The
other direction is missing. A document can't bring its own Python, a Python program can't run
in the browser host, and arisrv runs no scripts at all.

This roadmap adds host variants that embed a CPython interpreter, with pycvc, pycvc_gl, numpy
and the science packages, natively and in one WebAssembly module:

- A document can carry Python (a `python:` block, or modules beside it) and call it from its
  programs.
- A Python script can be the entry point (`arigl-py run app.py`). It is the same script that
  runs under `python app.py` with #985 today.
- The same documents and scripts run natively and in the browser. That is what puts the Python
  Ariadne demos and GRL-SNAM's demos in an `arigl` page.
- arisrv can run server-side Python. Clients can ask it to run code without being able to run
  arbitrary code or take the relay down (§6).
- Lua can come later through the same seams, without another set of binaries (§8).

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

**Verdict: feasible.** The parts that could have killed it are proven: CPython, numpy and
vtk-python as static wasm, pycvc_gl in the same module, and a Python host for documents. What's
new is integration work:

- an engine that boots Python inside a host and hands it the running runtime;
- a document schema for Python, and a trust rule for it;
- one combined wasm link, run in a real browser;
- a sandboxed script service on arisrv.

The risks are in §9.

## 2. Names

**Recommendation: `ari-py`, `arigl-py`, `arisrv-py`, and the browser package `arigl-py-web`.**

- This is the pattern `arigl-web` already set: the base name says which runtime, and the suffix
  says what the variant adds.
- The binary name, the cvcpkg package name and the CMake target are the same.
- The suffix names what forces a separate binary: a pinned CPython. That means an ABI, tens of
  MB of stdlib and packages natively, and a browser module several times arigl-web's size. The
  stock hosts must not carry any of it.
- There is no `-cp312` in the name. The host is an application that bundles its interpreter, so
  it pins one CPython (3.12 now), as VolRover3 does. Nobody imports it into their own Python.
- Documents never name a binary. They ask for `python` in `requires:` (§4.1), so a rename costs
  nothing until the recipes are published.
- Lua doesn't get `-lua` variants (§8). It's ~300 KB of ANSI C with no ABI to pin, so once it has
  bindings it goes into the stock hosts.

Considered and dropped:

- `aripy`, `ariglpy`, `arisrvpy`: `ariglpy` is hard to read.
- `pyari`, `pyarigl`: these read as Python packages for ari, which is what `pycvc_gl.ariadne`
  already is.
- A neutral suffix (`arix`): it says nothing about the one thing that makes the variant
  different.

`arisrv-py` links pycvc core, not pycvc_gl: the relay has no GL.

## 3. Architecture

### 3.1 One engine, linked into each `-py` host

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
| `arigl-py` | `arigl` | the engine, pycvc, pycvc_gl |
| `ari-py` | `ari` | the engine, pycvc |
| `arisrv-py` | `arisrv` | the engine, pycvc, and the script service (§6) |

Natively they link the prefix's shared `libcvc`, `libcvcGL` and `libpython3.12`. So the
`_pycvc_gl` extension that Python imports binds to the **same** cvcGL as the host: one app, one
scene graph, one ImGui context. A second copy of cvcGL would mean a second ImGui context, and
raw ImGui calls from the wrong copy crash. In wasm everything is one static module anyway (§5).

A runtime-loaded Python plugin for the stock binaries was considered for native builds. wasm
has to be a separate link regardless, and separate binaries keep Python out of the stock
packages' install closure, so the variants are separate binaries everywhere.

### 3.2 The host module and `Host`'s embedded mode

#985's `pycvc_gl.ariadne.Host` gets a fourth mode, `embedded`:

- `Host()` inside a `-py` host returns a Host over the host's own runtime (from `_arihost`), not
  a new window. An explicit `mode=` that contradicts it (`"headless"` in a window) raises with a
  message that says why.
- `pycvc_gl.ariadne.current_host()` returns that Host, or `None` under plain `python`.
- The rest of the API is unchanged: `function`, `async_function`, `on_frame`, `watch`, `set`,
  `set_tree`, `set_series`, `publish_image`, `get_tree`, `run_program`, `host.lab`.
- `run()` drives the host's loop natively, where the script is the program's main. In the
  browser the page owns the loop (§5.3), so `run()` returns at once and the work goes in
  `on_frame` and in functions.

So one script runs both ways: `python app.py` (#985) and `arigl-py run app.py`. GRL-SNAM's
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
that requires `python` fails with one error that names the `-py` host to use.

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

- `arigl-py run app.py [args]` runs `app.py` as `__main__`, with `sys.argv` set to
  `[app.py, args...]`. Typically the script does `host = Host()`, registers functions, then
  `host.load("app.ari")` and `host.run()`.
- `arigl-py run -m package.module [args]` does the same for a module, e.g.
  `arigl-py run -m grl_snam.ariadne live austin-freedrive`.
- `arigl-py run main.ari` loads a document as usual.
- `arigl-py python [args]` is the embedded interpreter as a plain `python3` (a REPL, `-c`, `-m`),
  so scripts and tests use exactly the host's packages, e.g. `arigl-py python -m pytest`.

### 4.5 The console widget

- `py_console:` is an input line plus a scrollback, bound to the engine: ImGui in a window,
  `text_view` / `message_input` in a terminal.
- `py_editor:` comes later: ImGuiColorTextEdit, with completion from jedi (noarch on cvcpkg, so
  it works in the browser too).

These replace VolRover3's `PyConsoleDock`. A console runs full Python, so only a trusted
document (§4.3) can mount one.

## 5. The browser: `arigl-py-web`

### 5.1 One module

A recipe `arigl-py-web` (`cvcpkg/recipes/arigl-py-web/build-wasm.sh`) does:

- arigl-web's closure compile from the repo root, with `-DCVC_ARIADNE_PYTHON=ON`;
- the Python link from [`link-host.sh`](../../bindings/pycvc/wasm/link-host.sh):
  `libpython3.12.a`, `_pycvc.a` and `_pycvc_gl.a` whole-archived, numpy's extension objects with
  the generated inittab registrar, and `_vtkmodules_static.a` in the full tier;
- the cvcGL browser contract `link-host.sh` skipped because it only ran under node:
  `webgl_state_shadow.js`, and mimalloc on wasm-mt ([CVCGL_WASM.md](../CVCGL_WASM.md)).

It ships arigl-web's page files plus `python312.zip`: the stdlib and the pure-Python packages,
fetched before start and imported through zipimport.

### 5.2 Packages in the browser

- **C extensions are linked in**, so the browser's package set is fixed at build time.
  Generalize `link-host.sh`'s numpy registrar into a cvcpkg convention: a wasm C-extension
  package ships its relocatable objects plus `share/python-wasm/<pkg>.inittab` (dotted module
  name → `PyInit_` symbol), and the host build links every manifest it finds in the deps prefix.
  This is needed before the second C-extension package.
- **Pure-Python packages** (noarch) go into the zip: restrictedpython, jedi, sympy, networkx,
  imageio, trimesh.
- **Next C-extension wasm builds**, in order of demand: pillow, matplotlib (Agg; with contourpy
  and kiwisolver), h5py (hdf5 is already published for wasm), pandas.
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
  2. drive the `-py` page from `emscripten_set_main_loop` instead of Asyncify;
  3. JSPI, where the browser has it.

  `-sASYNCIFY_IGNORE_INDIRECT` stays a checklist, never a default (CVCGL_WASM.md).

### 5.4 Flavors

- **wasm first.** Every dependency is published single-threaded.
- **wasm-mt waits on two things:**
  - vtk-python-cp312 wasm-mt is stuck at `+cvc.2`. The fix that stops it building VTK's
    interpreter executable (cy-pca/cvcpkg#109) needs a republish.
  - pycvc-gl-cp312 has no wasm-mt build. The lean tier needs only this one.

## 6. `arisrv-py`

### 6.1 Server documents first, in stock arisrv

arisrv runs no document today: it is a fixed relay. Step S0 gives it one. `arisrv --doc
server.ari` runs a headless `AppRuntime` (no backend, like #985's `AriApp`) on the relay's app,
for room rules, bots and checks written in the DSL.

That belongs in the stock arisrv, because the DSL is already sandboxed: step and time caps, the
chroot, and bound writes.

### 6.2 Python on the server

`arisrv-py` adds the engine with pycvc core.

- **Never on the relay loop.** The relay steps rooms in fixed substeps and fans out poses and
  media, so a slow call there is everyone's lag.
- **A script thread owns the interpreter.** It reads and writes the room tree through the same
  paths and gates as everything else (`room_write_gate`, bound writes), and results land on a
  later relay tick.
- **Operator code is trusted**, as the operator's own process: the server document's `python:`
  block, or `arisrv-py --python main.py`.

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
  - every job runs in a pool of `arisrv-py worker` processes with `RLIMIT_AS` and `RLIMIT_CPU`;
  - the workers have no network (unshare or seccomp on Linux, a Job Object on Windows);
  - a worker is recycled after N jobs;
  - a job that overruns its deadline gets its worker killed, and the client gets an error. That
    includes a C call the in-process budgets can't interrupt, such as one huge numpy operation;
  - the relay never waits on a worker.
- **Per-identity rate and concurrency limits**, keyed on the identity that bound writes already
  establish.

A client can then at worst spend its own quota and one worker's time. It can't reach the host,
the relay's interpreter, or other clients' state.

Lua (§8) can do tier 3 with hard in-process limits (an allocator cap, an instruction hook) at a
fraction of the cost. Once Lua has bindings it is the better engine for client code, and
RestrictedPython stays for clients that need Python's libraries.

## 7. The demos and VolRover

- **GRL-SNAM's Ariadne ports** (`grl_snam/ariadne/`: `live`, `fleet`, ...) run on #985 under
  `python`. Natively they run on `arigl-py` unchanged (`arigl-py run -m grl_snam.ariadne ...`).
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

## 8. Lua, later

- **Bindings.** SWIG has a Lua backend, so a `luacvc` can reuse pycvc's `.i` files. The
  Python-specific typemaps (callables, numpy, PyCapsule) need Lua twins. sol2 isn't on cvcpkg.
- **Engine.** A `lua` `script_engine`, with `requires: [lua]`, a `lua:` block, and exports as
  host functions: the same shape as Python.
- **Hosting.** Compiled into the stock ari, arigl, arisrv and arigl-web once bound. `lua` 5.4.9
  is on cvcpkg for every platform, wasm included.
- **Sandbox:**
  - `lua_newstate` with an allocator that enforces a byte cap;
  - a `lua_sethook` count hook for an instruction budget;
  - a fresh `_ENV` holding only the functions we hand it;
  - text-only `load` (no bytecode);
  - one `lua_State` per client job.

## 9. Risks and open questions

1. **Browser module size.** arigl.wasm is 14.6 MB gzipped. The September node harness was 158 MB
   uncompressed, with the stdlib embedded in MEMFS and the full vtk-python. Measure lean and full.
   Levers: the stdlib as a lazily fetched zip, no test modules, `-Oz` on the Python objects.
2. **Asyncify with CPython** (§5.3).
3. **Python has never run in a real browser here.** The wasm harness only ran under node,
   headless. W1's first milestone is a browser page with a window, checked in headless Chrome
   with the CDP driver from the conference work.
4. **Native packages are Linux-only.** `pycvc-cp312` and `pycvc-gl-cp312` are Linux-only on
   cvcpkg.org, so `arigl-py` on Windows and macOS waits on those packages. The Windows pycvc
   rebuild has known blockers.
5. **Startup time.** CPython init, then importing numpy and pycvc_gl, in the browser. Measure,
   then precompile `.pyc` into the zip or freeze the hot modules.
6. **Native trust UX** (§4.3). Is `--python-allow` enough, or do documents need signing?
7. **Names** (§2). Decide before the first recipe is published.
8. **RestrictedPython 8.4 on wasm.** It's noarch on cvcpkg; confirm it imports in the browser
   build (it matters there only for §4.3's untrusted snippets).

## 10. Phases

| Phase | What | Needs | Size |
|---|---|---|---|
| P0 | #985 merged and released, so `Host`, functions with values and `AriApp` are on cvcpkg.org | review | S |
| P1 | The engine, `_arihost`, `Host` embedded mode, the `python:` block, the `python` tokens, `--python-allow`, `run app.py` / `run -m` / `python`; `arigl-py` and `ari-py`, native Linux, with recipes. Tests: `ari test` documents that call Python, pycvc's tests through the embedded interpreter, GRL-SNAM's ports on `arigl-py` | P0 | L |
| P2 | `py_console`, then `py_editor` with jedi | P1 | M |
| W1 | `arigl-py-web` wasm: the combined link, a browser smoke in headless Chrome, size and Asyncify numbers, lean vs full | P1 | L |
| W2 | The wasm C-extension manifest convention in cvcpkg; then pillow and matplotlib for wasm | W1 | M each |
| W3 | wasm-mt: vtk-python wasm-mt republished, pycvc-gl wasm-mt | W1, cy-pca/cvcpkg#109 | M |
| N1 | `arigl-py` on Windows and macOS | pycvc packages for those platforms | M |
| D1 | GRL-SNAM in the browser: torch-free inference, a demo page | W1 | M |
| D2 | VolRover gap audit, then VolRover as an Ariadne app | P2 | L |
| S0 | Server documents in stock arisrv (`--doc`) | — | M |
| S1 | `arisrv-py`: the engine on a script thread, operator Python | S0, P1 | M |
| S2 | `room-call`: functions clients may call | S1 | M |
| S3 | Client DSL programs | S0 | S |
| S4 | Restricted client Python in worker processes | S2 | L |
| L1 | Lua bindings (SWIG), the Lua engine in the stock hosts, Lua for client code on arisrv | the `script_engine` seam from P1 | L |
