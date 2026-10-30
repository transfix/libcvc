# cvcGL in the Browser: the WebAssembly App Contract

*How a cvcGL / Ariadne app built with Emscripten gets the browser-side speedups,
and what it promises in return. Reference for `cvcgl_wasm_app()`
(`src/cvcGL/wasm/cvcGLWasm.cmake`), the WebGL state shim
(`src/cvcGL/wasm/webgl_state_shadow.js`) and `cvc::gl::FrameYield`
(`inc/cvc/gl/FrameYield.h`).*

## Table of Contents

- [Overview](#overview)
- [Quick start](#quick-start)
- [cvcgl_wasm_app()](#cvcgl_wasm_app)
- [The WebGL state shim](#the-webgl-state-shim)
- [FrameYield: who yields to the browser](#frameyield-who-yields-to-the-browser)
- [mimalloc](#mimalloc)
- [-sASYNCIFY_IGNORE_INDIRECT: a checklist, never a default](#-sasyncify_ignore_indirect-a-checklist-never-a-default)
- [Measuring: glsync and serve.py](#measuring-glsync-and-servepy)
- [Testing](#testing)
- [Checking an app in a browser](#checking-an-app-in-a-browser)
- [arigl-web: the stock browser host](#arigl-web-the-stock-browser-host)
- [Related](#related)

## Overview

A cvcGL app in a browser pays for things a native build never sees. Two of those costs
come from cvcGL's own code, so every cvcGL app has them:

- **Synchronous GL state queries.** cvcGL compiles Dear ImGui's `imgui_impl_opengl3`
  into `libcvcGL`, and `ImGuiOverlay` calls `ImGui_ImplOpenGL3_RenderDrawData` once per
  frame. Ariadne's `ImGuiBackend` draws through `ImGuiOverlay`. That backend backs up
  and restores `ACTIVE_TEXTURE`, `VIEWPORT`, `SCISSOR_BOX`, the six `BLEND_*` values,
  five `isEnabled` flags and `isProgram` around every draw. VTK also re-reads
  `READ_BUFFER` and `MAX_DRAW_BUFFERS` several times a frame. In a browser several of
  these block until the GPU process has drained its command queue. Firefox does not cache
  `ACTIVE_TEXTURE`, so there ImGui's first query of each frame waits for all of the
  frame's queued GPU work.
- **A second yield per frame.** VTK 9.5's `vtkWebAssemblyOpenGLRenderWindow::Frame()`
  calls `emscripten_sleep(0)` at the end of every render. An app whose loop already
  yields once per frame then pays a clamped trip through the event loop twice.

A third cost is the allocator. dlmalloc serialises every `malloc`/`free` on one global
lock, and in a threaded (wasm-mt) build the threads contend for it.

The fixes are opt-in, one per cost:

| Fix | How an app gets it | Effect |
|---|---|---|
| WebGL state shim | `cvcgl_wasm_app(<target>)` (CMake) | the shadowed state queries are answered client-side and never wait for the GPU process (check with glsync) |
| `FrameYield::App` | `view.setFrameYield(SceneRenderer::FrameYield::App)` (C++) | one event-loop trip per frame instead of two |
| mimalloc | `cvcgl_wasm_app` on a wasm-mt build | no global allocator lock for the threads to contend on |

The CPU-side frame-cost fixes already in cvcGL need no opt-in: the metadata mirror,
the idle `CameraController::update`, and the cheaper bounds walk. `setCastsShadow`
and `texture_modified_rows` / `_rect` need the app to call them.

## Quick start

```cmake
find_package(cvcGL CONFIG REQUIRED)        # or in-tree: cvcGL's own CMakeLists
add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE cvc::cvcGL)
if(EMSCRIPTEN)
  target_link_options(my_app PRIVATE -sASYNCIFY=1 -sALLOW_MEMORY_GROWTH=1)
endif()
if(COMMAND cvcgl_wasm_app)   # older cvcGL bundles do not ship it
  cvcgl_wasm_app(my_app)     # no-op outside Emscripten: call it unconditionally
endif()
```

```cpp
cvc::gl::SceneRenderer view(sg, w, h, /*offscreen=*/false, "main");
view.setFrameYield(cvc::gl::SceneRenderer::FrameYield::App); // no-op natively
while (!view.windowClosed()) {
  // ... frame work ...
  view.render();
#ifdef __EMSCRIPTEN__
#ifndef __EMSCRIPTEN_PTHREADS__
  sg.publisher().flush();  // no worker thread: drain publishes at frame cadence
#endif
  emscripten_sleep(0);     // the ONE yield per frame -- on every path that renders
#endif
}
```

Every wasm demo in the repo's `examples/` is built this way: the gallery demos
(`_wasm_demos`) and `ariadne_hello` (`_wasm_extra_demos`, not in the gallery), and so
is arigl's browser build, `cyprty` (`src/arigl/CMakeLists.txt`). So is `arigl` itself
in the gallery: it is the gallery's Ariadne app, built with every document in
`examples/ariadne/documents`, the component library and `examples/data` in its file
system. Each `.ari` demo is `arigl/?doc=<name>`, and its Demos menu switches documents in
place (ARIADNE.md §9). A wasm build without the examples makes `arigl` the stock browser
host instead, with no documents of its own ([arigl-web](#arigl-web-the-stock-browser-host)).

## cvcgl_wasm_app()

```cmake
cvcgl_wasm_app(<target>
  [STATE_SHIM ON|OFF]                 # default ON
  [STATE_SHIM_MODE on|verify|norb]    # build-time default mode; default on
  [MIMALLOC AUTO|ON|OFF]              # default AUTO = ON iff cvcGL was built -pthread
  [FRAME_YIELD_LOCKED AUTO|ON|OFF])   # default AUTO = ON iff the app links -sASYNCIFY_IGNORE_INDIRECT
```

Defined in `cvcGLWasm.cmake`. `cvcGLConfig.cmake` includes it for installed consumers,
and `src/cvcGL/CMakeLists.txt` includes it in-tree.

- **Outside Emscripten** it returns at once.
- **STATE_SHIM** adds `target_link_options(PRIVATE "SHELL:--pre-js <dir>/webgl_state_shadow.js")`
  and puts the file in `LINK_DEPENDS`, so editing the shim relinks the app.
- **STATE_SHIM_MODE** other than `on` generates `<build>/<target>_glshim_default.js`,
  which sets `Module.glStateShadowDefault`, and links it as a `--pre-js` ahead of the shim.
- **MIMALLOC** adds `-sMALLOC=mimalloc` (see [mimalloc](#mimalloc) for AUTO).
  - A `-sMALLOC=mimalloc` the target already has counts as ON: nothing is added and
    nothing is said.
  - Any other `-sMALLOC=` on the target, its directory or `CMAKE_EXE_LINKER_FLAGS` is
    kept, and a warning says so.
  - An `-sMALLOC=` the app adds after this call comes later on the link line, and emcc
    keeps the last `-s` value, so it wins too.
- **FRAME_YIELD_LOCKED** links a generated `<build>/<target>_frameyield_lock.js` as a
  `--pre-js`. It sets `Module.cvcglFrameYieldLocked = 1`, which locks every cvcGL window
  of the app in `FrameYield::App` ([the lock](#the-lock-apps-linked-with--sasyncify_ignore_indirect)).
  - `AUTO` locks exactly when the target's final link options carry
    `-sASYNCIFY_IGNORE_INDIRECT=1` (the deferred check below; on CMake older than 3.19, the
    options present at the call).
  - Pass `ON` when the flag comes from somewhere the check cannot see, such as a
    dependency's `INTERFACE_LINK_OPTIONS` or a generator expression.
  - `OFF` never locks. With `IGNORE_INDIRECT` linked, the lint then warns that the app
    traps unless its C++ locks every window itself.
- **Target properties** `CVCGL_WASM_APP`, `CVCGL_WASM_APP_STATE_SHIM`,
  `CVCGL_WASM_APP_STATE_SHIM_MODE`, `CVCGL_WASM_APP_MIMALLOC` and
  `CVCGL_WASM_APP_FRAME_YIELD_LOCKED` record what was applied. The last reads `AUTO`
  until the deferred check resolves it.
- **Deferred check and lint.** With CMake 3.19 or newer, a deferred check runs on the
  target's final link options when the calling directory finishes. It resolves
  `FRAME_YIELD_LOCKED AUTO`, warns on `-sASYNCIFY_IGNORE_INDIRECT=1`, and warns more
  loudly if `-sFETCH=1` is there too. See the [checklist](#-sasyncify_ignore_indirect-a-checklist-never-a-default).

**Feature tests:** `if(COMMAND cvcgl_wasm_app)`, and for later keywords
`if("FRAME_YIELD_LOCKED" IN_LIST CVCGL_WASM_APP_FEATURES)`.

**Why a function and not an INTERFACE option on `cvc::cvcGL`?** An interface
`--pre-js` would reach every static consumer's final link: tests, helper tools, and
pages that never asked for a page-global WebGL patch. It also could not carry
`-sMALLOC`. Dependency link options come after the target's own and emcc keeps the
last `-s` value, so cvcGL would silently override the app's allocator.

**No absolute paths are exported.** The function finds the JS through the GLOBAL
property `CVCGL_WASM_DATA_DIR`:
- in-tree it is `src/cvcGL/wasm`;
- installed it is `@PACKAGE_CVCGL_WASM_DATADIR@`, which
  `configure_package_config_file(... PATH_VARS)` computes from the installed config's
  own location (`PACKAGE_PREFIX_DIR`).

`cvcGLConfig.cmake` reads that path right after `@PACKAGE_INIT@`, before any
`find_dependency`. On CMake 3.29 and older, every dependency config
generated with `@PACKAGE_INIT@` (cvc, SDL3, zstd, libxml2 and others) overwrites
`PACKAGE_PREFIX_DIR` with its own prefix. Reading it later would hand out the
dependency prefix's `share/cvcGL/wasm` whenever cvc or SDL3 lives in a different
prefix than cvcGL, as in publish-cvcgl-wasm's `$INST` / `$WASM_DEPS` split.

So a relocated prefix hands out its own `share/cvcGL/wasm`, and the path only ever
appears in the consumer's build tree. `CVCGL_WASM_PTHREADS` records whether cvcGL
was built `-pthread`. Emscripten forbids mixing pthread and non-pthread objects, so
a consumer of a wasm-mt `libcvcGL.a` is threaded too. A bundle without the JS still
passes `find_package`; the function stops with an error only when it is asked for the
shim and the file is missing.

**Not CMake?** Pass the installed file yourself:
`--pre-js <prefix>/share/cvcGL/wasm/webgl_state_shadow.js`, plus `-sMALLOC=mimalloc` on a
wasm-mt (`-pthread`) build.

**TODO: the pycvc_gl wasm host.** `bindings/pycvc/wasm/link-host.sh` is node-only today,
so it links neither speedup. When it gets a browser page, its link must add the shim
`--pre-js "$INST/share/cvcGL/wasm/webgl_state_shadow.js"` and, on wasm-mt,
`-sMALLOC=mimalloc`. A TODO in the script says the same.

## The WebGL state shim

`webgl_state_shadow.js` wraps every WebGL setter that can change a value it shadows,
and keeps a copy per context. The matching query is then answered from that copy
instead of a synchronous call:
- `getParameter` for `SCISSOR_BOX`, `VIEWPORT`, `BLEND_{SRC,DST}_{RGB,ALPHA}`,
  `BLEND_EQUATION_{RGB,ALPHA}` and `ACTIVE_TEXTURE`;
- on WebGL2, also `MAX_DRAW_BUFFERS`, `MAX_COLOR_ATTACHMENTS`, and `READ_BUFFER`,
  which is tracked per framebuffer;
- `isEnabled` for 9 or 10 capabilities;
- `isProgram` for live programs.

Anything it cannot prove is passed to the real call: a rejected setter, a foreign
framebuffer, a deleted program. The copy is dropped on context loss. In a worker
(OffscreenCanvas / PROXY_TO_PTHREAD) the shim does nothing.

**Scope.** It patches the WebGL *prototypes*, so it serves every WebGL context on the
page, cvcGL's or not. A fuzz test checks the answers against a mock WebGL with real
GL/WebGL semantics, and `?glshim=verify` checks them against a live app (see
[Checking an app in a browser](#checking-an-app-in-a-browser)). A page that embeds a
cvcGL module beside other WebGL code can still opt out with `STATE_SHIM OFF`, or
`?glshim=0` per load.

**Modes and where they come from.** The first of these that is set wins, so a URL can
always A/B any page:

1. the URL: `?glshim=0|off|on|verify|norb`;
2. the page: `Module.glStateShadow`, set by the host page before the module starts;
3. the build: `Module.glStateShadowDefault`, from `cvcgl_wasm_app(... STATE_SHIM_MODE m)`;
4. `on`.

| mode | behaviour |
|---|---|
| `on` | everything above |
| `0` / `off` | nothing installed; every call goes straight to WebGL (the A/B baseline) |
| `norb` | on, except `READ_BUFFER`, which goes to the real call |
| `verify` | answers with the REAL value, compares it to the shadow, and counts mismatches |

`window.__cvcGlShadow.stats` holds:
- `version`: which copy of the shim is active (`cvcGL-1` for this one). The first copy
  installed on a page wins, so a page that may load another copy can tell which one runs.
- `mode` and `modeFrom` (`url` / `page` / `build` / `default`);
- `served`, `verified`, `mismatches` and `mismatchBy` (per value name).

## FrameYield: who yields to the browser

```cpp
namespace cvc::gl { enum class FrameYield { Vtk, App }; }   // cvc/gl/FrameYield.h
// SceneRenderer and ViewportManager:
using FrameYield = cvc::gl::FrameYield;
void setFrameYield(FrameYield);
FrameYield frameYield() const;   // the effective mode
void lockFrameYield();           // App for good (the -sASYNCIFY_IGNORE_INDIRECT lock)
bool frameYieldLocked() const;
#define CVC_GL_HAS_FRAME_YIELD 1
#define CVC_GL_HAS_FRAME_YIELD_LOCK 1
```

- **`Vtk`** (the default) keeps today's behaviour: VTK 9.5 yields inside every
  `render()`.
- **`App`** means the app's loop yields once per frame, so cvcGL calls
  `SetDoubleBuffer(0)` on the render window. In VTK 9.5 that is the switch for the
  in-render sleep.

**The contract.** In `App` mode the loop calls `emscripten_sleep(0)` on **every** path
that renders a frame. A `continue` that skips the frame-end yield breaks it. That is
the bug `nav_fog_ghost`'s paused path had. Without the in-render yield it would never
let the browser paint again.

- **Where App does nothing.** It is a no-op natively, and in a wasm build without
  Asyncify (`emscripten_has_asyncify() == 0`, e.g. an `emscripten_set_main_loop` app).
  There is no in-render yield there to remove, so `DoubleBuffer` is left alone and
  `frameYield()` reads `Vtk`.
- **DoubleBuffer.** cvcGL only changes `DoubleBuffer` back if cvcGL turned it off, and
  then it restores the value it found at that moment rather than forcing 1. An app's own
  `SetDoubleBuffer` survives `Vtk` mode. An app's own `SetDoubleBuffer(0)` survives a
  watchdog trip only if the app made it before cvcGL switched to `App` (before
  `setFrameYield(App)`, or under `?frameyield=app` before the window exists): one made
  after is undone by the trip. An `-sASYNCIFY_IGNORE_INDIRECT` app locks instead.
- **Readback.** `writePNG` and `frameRGB` read the back buffer explicitly (`front=0`,
  `ReadFrontBufferOff`), so readback is unaffected.
- **URL override.** `?frameyield=vtk|app` overrides the app's choice for an A/B without
  a rebuild. It is read once, when the window is created, and logged. A locked window
  refuses `?frameyield=vtk` and logs that instead.
- **Watchdog** (wasm with Asyncify, `App`). An `EM_JS` epoch counter is bumped by a
  microtask. A microtask can only run once wasm has returned to the browser. If one
  window renders 8 times in a single epoch, the loop broke the contract. cvcGL then
  logs once to stderr (the console) and restores the `DoubleBuffer` value it found, so
  the page keeps running, slower, instead of freezing.
  - It counts **every** `vtkRenderWindow::Render()` of the window through a
    `vtkCommand::StartEvent` observer, whichever path made it: `render()`, `writePNG`
    (two renders), `frameRGB` (one), `renderWindow()->Render()`, a node's own fallback
    render, the interactor's resize render.
  - `frameYield()` reads `Vtk` afterwards. Calling `setFrameYield(App)` again re-arms it.
  - Each trip also bumps `globalThis.__cvcglFrameYield.trips`, for automated browser
    checks.
  - The threshold assumes no window renders more than 7 times per browser task.
  - A locked window only reports, as a loud `cvcGL: ERROR:` line. See
    [the lock](#the-lock-apps-linked-with--sasyncify_ignore_indirect).
- **VTK 9.6** removed the in-render yield (`81a272dd1ee`). With VTK 9.6 every wasm loop
  must yield by itself in either mode, so the watchdog there only warns.

**Feature detection** for a consumer that also builds against an older cvcGL:

```cpp
template <class V> void app_yields(V &v) {
  if constexpr (requires { v.setFrameYield(V::FrameYield::App); })
    v.setFrameYield(V::FrameYield::App);
  else {
    // Older cvcGL: by hand, and only in the browser. Natively, DoubleBuffer off makes
    // the window draw to the front buffer or stop presenting.
#ifdef __EMSCRIPTEN__
    if (auto *rw = v.renderWindow())
      rw->SetDoubleBuffer(0);
#endif
  }
}
```

An app linked with `-sASYNCIFY_IGNORE_INDIRECT` locks instead, so nothing can turn the
trapping sleep back on: `if constexpr (requires { v.lockFrameYield(); }) v.lockFrameYield();
else` in front of the first branch, or let `cvcgl_wasm_app` do it (`FRAME_YIELD_LOCKED AUTO`).
Any other app must not lock: that turns off the watchdog's rescue and the `?frameyield` A/B.

**Loop audit of the examples** (why `Vtk` stays the library default):

| App | Its own per-frame yield | App mode? |
|---|---|---|
| lsystem_forest, lsystem_coast, terrain_lab, bunny_shadow | at the loop end | yes |
| volren_bunny, volslice_bunny | at the loop end (the `break`s are native capture) | yes |
| nav_city_swarm, nav_city_drive | at the loop end (the `continue`s are inside the trail lambda's `for`) | yes |
| nav_fog_ghost | at the loop end, and on the paused path (fixed: it used to `continue` past the only yield) | yes |
| ariadne_hello | at the loop end on Emscripten (`sleep_for` natively); its `--png` / `--offscreen` capture loop renders back to back, so it keeps `Vtk` | yes, interactive loop only |
| nav_convoy, nav_compute | `std::this_thread::sleep_for` only: native-only, not wasm apps | n/a |
| pycvc_gl wasm host | Python-driven, no `-sASYNCIFY` | n/a (VTK's sleep never runs) |

### The lock: apps linked with -sASYNCIFY_IGNORE_INDIRECT

Under `-sASYNCIFY_IGNORE_INDIRECT=1`, VTK 9.5's in-render sleep is a trap, not a
fallback, because it is reached through the virtual `Render()`. Two things can bring it
back at runtime on an unlocked window: `?frameyield=vtk`, and a watchdog trip. Such an
app therefore **locks** `App`:

- **Automatically**, from `cvcgl_wasm_app(<target>)`. `FRAME_YIELD_LOCKED AUTO` sees the
  flag in the target's final link options and links `<target>_frameyield_lock.js`, which
  sets `Module.cvcglFrameYieldLocked = 1`. Every window reads it when it is created.
- **By hand**: `view.lockFrameYield()` in C++, `FRAME_YIELD_LOCKED ON`, or a host page
  that sets `Module.cvcglFrameYieldLocked = 1` before the module starts.

A locked window is in `App` from that moment, whatever the app asked before:
- `?frameyield=vtk` and `setFrameYield(Vtk)` are refused, and each refusal is logged;
- the watchdog never turns VTK's yield back on. It reports once, loudly, and the page
  cannot paint until the loop yields;
- there is no unlock, because the reason is a link flag.

Natively the lock is only recorded: `frameYieldLocked()` reads `true`, and
`frameYield()` still reads `Vtk`.

## mimalloc

`MIMALLOC AUTO` turns mimalloc on for wasm-mt builds and leaves single-threaded builds
on dlmalloc. Emscripten's own guidance (`src/settings.js`) recommends mimalloc for
malloc contention, and notes that it is larger and uses more memory. A single-threaded
dlmalloc has no lock to contend on, so it gains nothing there.

As a result:
- the gh-pages gallery (single-threaded) keeps dlmalloc;
- the libcvc-examples-web wasm-mt gallery, like any other wasm-mt app, gets mimalloc.

Under `-fsanitize=address`, `AUTO` resolves to OFF with a status message, because emcc
refuses to combine mimalloc with ASan. An explicit `MIMALLOC ON` under ASan gets a
warning instead.

Because mimalloc uses more memory, check a memory-heavy app at its `MAXIMUM_MEMORY`
cap for growth failures or OOM. The gallery's Austin nav demos (`nav_city_swarm`,
`nav_city_drive`) run at 4 GB.

## -sASYNCIFY_IGNORE_INDIRECT: a checklist, never a default

`-sASYNCIFY_IGNORE_INDIRECT=1` tells Asyncify that no indirect call leads to an
unwind: no virtual call, function pointer or `std::function`. Asyncify then
instruments far fewer functions, which gives a smaller and faster module. It is only
correct while every sleep the app can reach is reached through direct calls. A sleep
reached indirectly **traps** at its unwind. That depends on the app, so it is not a
`cvcgl_wasm_app` option, and `cvcgl_wasm_app`'s lint warns when a target links it.
Before shipping it:

1. **Every async import reachable on the main thread is reached only through direct
   calls.** These cvcGL apps can reach a sleep indirectly:
   - VTK 9.5's `vtkWebAssemblyOpenGLRenderWindow::Frame()` sleep, reached through the
     virtual `Render()`. It needs `FrameYield::App`, **locked**, because on an unlocked
     window `?frameyield=vtk` or a watchdog trip turns it back on.
     `cvcgl_wasm_app`'s `FRAME_YIELD_LOCKED AUTO` locks it when the flag is in the final
     link options; otherwise use `FRAME_YIELD_LOCKED ON` or `lockFrameYield()` (see
     [the lock](#the-lock-apps-linked-with--sasyncify_ignore_indirect)).
   - `cvc::net`'s blocking fetch (`emscripten_sleep` in `http_client_fetch.cpp`),
     reached from Ariadne's http verbs through `std::function` intrinsics.
   - Anything that runs inside `ImGuiOverlay`'s draw callback. That callback is a
     `std::function`, so this covers every Ariadne UI action.
   - ImageMagick's `fd_sync`.
   - Any other blocking `emscripten_*` call.
2. **`-sFETCH=1`.** Every gallery demo links it (`examples/CMakeLists.txt`).
   An app that keeps it must prove fetch is unreachable. Otherwise, do not use
   `IGNORE_INDIRECT`.
3. **Link once with `-sASYNCIFY_ADVISE=1 -sASSERTIONS=1`**, exercise every UI path, and
   confirm there is no Asyncify "unreachable" trap.
4. **Re-run this audit** whenever anything new that can sleep is linked in.

An app passes it when, for example, its only sleeps are direct calls in `main()`, it
links no fetch, and the VTK sleep is locked off. Once such an app calls
`cvcgl_wasm_app`, `FRAME_YIELD_LOCKED AUTO` does that locking.

## Measuring: glsync and serve.py

`src/cvcGL/wasm/devtools/glsync.js` is a Firefox census of synchronous WebGL calls. It
is a developer tool, not installed with the SDK; `devtools/GLSYNC.md` explains how to
read it. Serve a built gallery with it injected:

```
python3 src/cvcGL/wasm/devtools/serve.py -d build-wasm-mt/gallery --glsync 8822
# then  http://localhost:8822/nav_city_drive/?glsync          (shim on)
#       http://localhost:8822/nav_city_drive/?glsync&glshim=0 (shim off: the A/B)
```

`serve.py` takes three flags, all off by default (`libcvc-examples-web` passes none):
- `--glsync[=PATH]` injects the census;
- `--prof-param NAME` makes the census add `?NAME` to the URL, for apps that print
  `PROF n=` lines with it;
- `--js-profiling` sends `Document-Policy: js-profiling` for Chromium's JS
  Self-Profiling API.

## Testing

| What | How |
|---|---|
| state shim (fuzz + precedence) | `cvcgl_webgl_state_shadow` (ctest, label `js`) |
| glsync | `cvcgl_glsync` (ctest, label `js`) |
| serve.py flags | `python3 src/cvcGL/wasm/devtools/tests/test_serve.py` |
| all three, no CMake (CI) | `CVC_EMSDK_DIR=<emsdk> src/cvcGL/wasm/run-js-tests.sh` |
| FrameYield native contract | `cvcgl_frame_yield` (ctest) |

The JS tests run only under the Emscripten SDK's node, never one from `PATH`:
- under `emcmake`, that is `CMAKE_CROSSCOMPILING_EMULATOR`;
- otherwise CMake looks only in `$CVC_EMSDK_DIR/node/*/bin` and
  `/opt/cvc-wasm/emsdk/node/*/bin` (`NO_DEFAULT_PATH`; `-DCVCGL_EMSDK_NODE=<node>` by
  hand). It looks again when `CVC_EMSDK_DIR` changes.

Without an emsdk node, ctest does not register them. CI runs them in the dedicated
`cvcgl-wasm-js` job through `run-js-tests.sh`, which installs emsdk from cvcpkg.

## Checking an app in a browser

Check in both Firefox and Chromium:

1. **Exactness.** `?glshim=verify`: interact with the app (menus, panels, camera,
   resize), then `__cvcGlShadow.stats.mismatches === 0` in the console.
2. **The win.** Firefox `?glsync`. There should be no `getParameter(ACTIVE_TEXTURE)`,
   `SCISSOR_BOX` or `BLEND_*` `[S]` keys; with `&glshim=0` they come back.
3. **Every render path yields.** Pause, resume and use every mode. No `cvcGL:
   FrameYield::App ... rendered 8 times` line may appear, and
   `__cvcglFrameYield.trips` must stay 0. A deliberately non-yielding debug build must
   print it once and keep running. A locked one, with `-sASYNCIFY_IGNORE_INDIRECT=1`,
   must print the `cvcGL: ERROR:` line instead and never trap.
4. **Memory with mimalloc** (wasm-mt). Run for a few minutes at the app's
   `MAXIMUM_MEMORY` and watch for growth failures.

## arigl-web: the stock browser host

`arigl-web` (`cvcpkg/recipes/arigl-web`) is `arigl` built to wasm for running anyone's
`.ari` documents. It carries none of its own. Its page loads a document by URL, and
it can be served from any path of an ordinary site. The single-threaded package also has
`ari`'s terminal page, which runs the same documents in the FTXUI backend in an xterm.js
terminal, loaded the same way (see [The terminal page](#the-terminal-page-arihtml)).

A wasm build of the repo with `CVC_BUILD_EXAMPLES=OFF` builds the shell as the `arigl` target
(`src/arigl/CMakeLists.txt`). With the examples on, the same target is the gallery's arigl
instead, with the documents preloaded.

The package is `share/arigl-web/`:

| File | What |
|---|---|
| `index.html`, `arigl-web.js`, `arigl-web.css` | the page (`src/arigl/web/arigl-web.*`): config, `?src=`, loading and error screens |
| `arigl.js`, `arigl.wasm` | the module; the component library is embedded at `/usr/share/libcvc/ariadne/components` |
| `ari.html`, `ari.js`, `ari.wasm`, `ari-web.js`, `ari-web.css`, `xterm/` | the terminal page (`src/ari/web`), `ari` built to wasm, and xterm.js with its licences; single-threaded package only (plus `ari_tvision.*` with tvision's wasm bundle) |
| `THIRD_PARTY_NOTICES.md`, `LICENSE` | VTK-derived code in cvcGL is BSD-3-Clause |

Serve those files in a directory beside the documents, e.g. for an app at `/apps/nicotina/`:

```
/apps/nicotina/index.html  arigl-web.js  arigl-web.css  arigl.js  arigl.wasm
/apps/nicotina/ari.html    ari.js        ari.wasm  ari-web.js  ari-web.css  xterm/   (the terminal page)
/apps/nicotina/main.ari    lib/feed.ari  ...             (the app's own documents)
/apps/nicotina/ariadne.config.json                         (optional)
```

and open `/apps/nicotina/` (the default document is `main.ari`) or
`/apps/nicotina/?src=main.ari`, and for the terminal client `/apps/nicotina/ari.html`. Any static server works for the single-threaded build,
`python3 -m http.server` included.

**How a document loads.** The page resolves `src` against its own URL and runs
`arigl run <flags> <absolute src URL> <document args>`. Before the load, arigl
([`web_document.h`](../inc/cvc/ariadne/web_document.h), over [`doc_fetch.h`](../inc/cvc/ariadne/doc_fetch.h)) does
the following:

1. It checks that the document is on the page's origin, or on one the config lists.
2. It fetches the document with the page's `fetch()`.
3. It reads the files the document names: its `import:` entries and every `load:` and
   `vinay:`.
4. It fetches each one named by a relative or absolute path, against the URL of the file
   that names it, and then the files those name.
5. It writes every fetched file at its URL path in MEMFS (`https://tx.wtf/apps/nicotina/lib/feed.ari`
   lands at `/apps/nicotina/lib/feed.ari`), changes to the document's directory, and loads
   the local copy.

So a relative reference resolves exactly as it would on disk, and the loader knows nothing
about URLs. References with a scheme are not fetched: `cvc://` is the embedded component
library, and `https://`/`state://` go through the loader's URI handlers, under whatever
network policy the host flags set. A missing import is a warning, as it is natively. A
missing document is an error. A name built from a unit's arguments (`load: "{panel}.ari"`)
cannot be known ahead, so it is a warning: list such files, and data files a scene reads,
in the config's `preload`.

**The config** is `ariadne.config.json` beside the page (a 404 means the defaults), or
`window.ARIADNE_CONFIG` set by a script ahead of `arigl-web.js`. `ari.html` reads the same file:

| Key | Default | Meaning |
|---|---|---|
| `title` | `arigl` | the page title |
| `src` | `main.ari` | the document when the URL names none |
| `src_param` | `true` | `false` ignores `?src=` and always runs `src` |
| `src_origins` | `[]` | other origins `src` may name (`"*"` for any); their server must allow CORS |
| `flags` | `[]` | arigl's host flags, before the document, e.g. the network flags: `["--net"]` lets the document reach the page's origin, `["--net-allow", "https://api.example"]` another; `{origin}` in a flag becomes the page's origin. A `--net-policy` file must be in MEMFS: list it in `preload` and pass its URL path. `--credential-store` is refused in a browser |
| `args` | `[]` | the document's arguments (§5.6 of ARIADNE.md); `{origin}` in one becomes the page's origin, e.g. `["--instance", "{origin}"]` for an app that talks to the server it is served from. The URL adds more with `?arg=`, taken as given |
| `preload` | `[]` | more files to fetch with the document, relative to it |
| `env` | `{}` | the module's environment (e.g. `CVC_ARIADNE_RELAY`) |

Host flags come only from the config, never from the URL, so a link cannot widen what the
page lets a document do. The page's origin is always allowed, and a site that also serves
untrusted files from it (user uploads, say) should pin the document with
`"src_param": false`.

**Errors.** A failed load replaces the loading screen with an error page: one line on what
went wrong, then everything arigl printed (`arigl: could not fetch .../main.ari: HTTP 404`,
`... not this page's origin ...`, a YAML error, an unknown host flag). A document that quits
shows that it has ended. The page reads `Module.ariadneRunning()` and `Module.ariadneExit(status)`,
which arigl calls, and keeps its state in `window.ariadneWeb` (`state` is `loading`,
`running`, `ended` or `failed`, and `log` holds arigl's output) for tests and the console.

**Content-Security-Policy.** Nothing in the page is inline, and nothing in the module
evals: cvcGL's touch probe used `emscripten_run_script_int` and is now an `EM_JS`. The
smallest policy it runs under is
`script-src 'self' 'wasm-unsafe-eval'; style-src 'self'` (and `connect-src` covering the
document's origin).

**wasm-mt.** The recipe's other column, `--platform wasm-mt`, links `-pthread` with
mimalloc and a 4-worker pool. A threaded page needs `SharedArrayBuffer`, which needs cross-origin
isolation. Its `index.html` must be served with `Cross-Origin-Opener-Policy: same-origin`
and `Cross-Origin-Embedder-Policy: require-corp`. Every subresource must then be same-origin
or carry `Cross-Origin-Resource-Policy`/CORS, which also covers a document from another
origin and anything a document fetches. Use the single-threaded build unless a document
needs threads. It needs no headers and can live at an ordinary site path.

**Size.** Measured on 3.7.4, single-threaded, emsdk 5.0.7, `-O3`. The recipe's script prints
the first row on every build.

| Module | raw | gzip -9 | brotli -q 11 |
|---|---|---|---|
| `arigl.wasm` as packaged (the gallery's features) | 49.0 MB | 13.8 MB | 9.5 MB |
| `arigl.wasm`, `CVC_ARIGL_WEB_LEAN=1`: no ImageMagick, assimp, mesher, SDF or libigl | 33.7 MB | 9.0 MB | 6.1 MB |
| `arigl.wasm` as packaged, linked without Asyncify (it cannot run; for scale) | 31.8 MB | 9.9 MB | 7.3 MB |
| `ari.wasm` as packaged | 21.7 MB | 7.1 MB | 5.1 MB |
| `ari.wasm`, `CVC_ARIGL_WEB_LEAN=1` | 7.7 MB | 2.8 MB | 2.1 MB |

`arigl.js` is 325 KB raw, 68 KB gzip and 59 KB brotli. `ari.js` is 109 KB, 26 KB and 23 KB.
The terminal page's own files and `xterm/` (with its licences) come to 411 KB raw. The arigl
page's three files come to under 10 KB. The deployed gallery's `arigl.wasm` (the same program
with the examples) is 48.9 MB, and its `ari.wasm` is 21.6 MB. ari is libcvc alone (no VTK, no cvcGL), and the lean
features take two thirds of it.

Where the code goes: 43.3 MB of function bodies plus 5.5 MB of data segments. The bodies are
attributed by name, from a link with `--profiling-funcs`. A library's templates that another
library instantiates count under libc++.

| Code | with Asyncify | without |
|---|---|---|
| VTK | 14.9 MB (34%) | 8.1 MB (31%) |
| ImageMagick | 5.3 MB | 3.3 MB |
| assimp | 4.8 MB | 3.0 MB |
| image and compression codecs | 2.7 MB | 1.9 MB |
| libc++ | 2.9 MB | 1.6 MB |
| the cvc core outside Ariadne (geometry, volume, LBIE, SDF, ...) | 3.3 MB | 2.4 MB |
| cvcGL | 1.8 MB | 1.1 MB |
| cvc::ariadne | 1.4 MB | 0.9 MB |
| cvc::state and state_exec | 1.2 MB | 0.6 MB |
| libc and the Emscripten runtime | 1.8 MB | 1.1 MB |
| libigl / Eigen | 0.7 MB | 0.6 MB |
| SDL3, Dear ImGui, boost, yaml-cpp, ftxui | 2.1 MB | 1.3 MB |
| total | 43.3 MB | 26.1 MB |

Three things stand out:

- **VTK is the largest single part of the packaged module, and most of the lean one.** In the
  packaged module it is a third of the code. The optional features the gallery needs
  (ImageMagick, assimp, the codecs behind them, the mesher) are about as large again.
  `CVC_ARIGL_WEB_LEAN=1` drops those features, which saves 15.3 MB raw and 3.4 MB brotli.
  Without them, the lean `ari.wasm` (libcvc alone) is 7.7 MB of the lean `arigl.wasm`'s
  33.7 MB, so VTK and cvcGL are about three quarters of the lean shell. The lean builds pass
  the same headless test.
- **Asyncify costs about 40% of the code.** The code is 26.1 MB without it and 43.3 MB with it.
  arigl's loop yields with `emscripten_sleep`, and a document's fetch and the document-set load
  block. Because any indirect call might reach one of those, nearly every function gets
  instrumented.
- **An ImGui-only host would be about a quarter of the lean shell.** Such a host draws
  Ariadne's widgets with Dear ImGui straight on WebGL2, with no VTK render window and no cvcGL
  scene graph. It is the lean `ari` with ImGui in place of FTXUI: cvc::ariadne, state_exec,
  the parts of the cvc core its registries reach, ImGui with its WebGL backend, SDL3 (or
  Emscripten's HTML5 input), yaml-cpp, boost, libc++ and libc. The lean `ari.wasm` measures
  that core at 7.7 MB. ImGui, SDL3 and the widget backend add about 1.5 MB with Asyncify (the
  table's figures). So such a host would be about 9-10 MB raw and ~2.5 MB brotli. That is a
  quarter of the lean shell (33.7 MB, 6.1 MB brotli) and a fifth of the packaged one. It would
  run no `scene:`, and a document that needs one says so with `requires:` (ARIADNE.md §5.7).
  This is an estimate from the two measured builds, not a build. Dropping Asyncify on top
  would cut about another third, as it does the packaged module (49.0 MB to 31.8 MB). The frame
  loop would use `emscripten_set_main_loop`, and the two blocking waits would move to JSPI
  where the browser has it, or to a worker on wasm-mt.

### The terminal page (`ari.html`)

`ari`'s browser page is `ari` built to single-threaded wasm behind an xterm.js terminal
(`src/ari/web/ari.html`). It runs a document in the FTXUI backend, the same backend as
`ari run` in a native terminal. In the gallery it runs its baked document
(`terminal_tour.ari`). Given `?src=`, or a config naming a document, it runs that one
instead, loaded exactly as arigl-web loads one. The page passes `ari run <flags> <src URL>`,
and `ari` fetches the document set into MEMFS through the same libcvc code
(`cvc/ariadne/web_document.h`). It honours the same config (`ariadne.config.json`) and the
same-origin rule. A build with `-DCVC_ARI_WASM_DEFAULT_SRC=main.ari`, which the arigl-web
package uses, makes `main.ari` the default when the URL names none. A failed load prints what
`ari` said into the terminal.

The page has no inline code and loads nothing from another origin. Its loader and style sheet
are `ari-web.js` and `ari-web.css`, and xterm.js is served beside it in `xterm/`, so it runs under
the same strict policy as arigl-web's: `script-src 'self' 'wasm-unsafe-eval'; style-src 'self'`.
The WebGL renderer addon is loaded before the terminal opens, so xterm.js never builds its DOM
renderer, which injects `<style>` elements. A browser without WebGL2 falls back to that renderer,
and there the page also needs `style-src 'unsafe-inline'`.

The build finds xterm.js 5.5.0, addon-fit 0.10.0 and addon-webgl 0.18.0 (with their licences) in
the first of these places:

1. `CVC_XTERM_DIR`, a directory holding `xterm.js`, `xterm.css`, `addon-fit.js` and
   `addon-webgl.js`;
2. the deps prefix: cvcpkg's `xterm-js`, `xterm-js-addon-fit` and `xterm-js-addon-webgl`
   packages install them in `share/xterm.js/`. They are noarch, so they are installed beside a
   wasm prefix's deps rather than listed in its recipes (a wasm resolve is static, and no static
   request matches a noarch bundle yet):
   `cvcpkg install xterm-js xterm-js-addon-fit xterm-js-addon-webgl --platform any --prefix <deps>`;
3. a download of the same npm packages from registry.npmjs.org, pinned by SHA-256, once per
   build tree.

All three give the files the page used to pin by SRI hash on jsDelivr. The gallery copies them
with the page: `build-pages.py` copies what `<page>.page-files` lists.

**The tvision page.** `ari_tvision.html` is the same page on the tvision desktop. It needs
cvcpkg's tvision with its browser console (`tvision` cvc.4, wasm column, on the catalog since
2026-10-30). The arigl-web package depends on it, so its wasm column has the page, and the
headless test runs the document set there too. The gallery adds the page whenever its deps
prefix has tvision. Natively, `ari run --backend tvision` runs the same documents.

**Testing.**

| What | How |
|---|---|
| URL parsing, reference resolution (and that it agrees with the file scheme), the origin rule, the fetch walk | `ariadne_doc_fetch_test` (ctest) |
| a document set run from another directory, by `arigl` and by `ari`; a URL refused natively | `ctest -R '(ari\|arigl)_(docset\|url_document)'` |
| both pages in a real browser: a same-origin set loads and runs (in `ari.html`, its text on the terminal), a cross-origin or missing `src` shows the error; arigl-web's page under a strict CSP, both with no COOP/COEP | `node src/arigl/web/test/arigl_web_smoke.mjs --web <share/arigl-web> --docset src/cvc/tests/data/ariadne_docset --chrome <chrome>` (CI: the `arigl-web` job, which builds with the recipe's script) |

By hand: serve a directory holding the five files, `app/` and `shared/` from
`src/cvc/tests/data/ariadne_docset` (the shell files inside `app/`), with `python3 -m http.server`,
and open `http://localhost:8000/app/`. The *Document set* window shows a line from
`lib/panel.ari` and one from `shared/note.ari`, beside the component library's *Display*.

## Related

This document covers the cvcGL-specific half. The platform-agnostic flow —
organizing a project to build wasm easily, the cvcpkg requirements-file
pattern, the generic CMake boilerplate (`cvcpkg_wasm_app()`), and packaging
any wasm app as a cvcpkg bundle — is in the cvcpkg repo:
`cvcpkg/docs/wasm-packaging.md` + `cvcpkg/cmake/cvcpkg-wasm-app.cmake`.
The complete build + package commands with `/tmp` prefixes are in
`libcvc/docs/FULL_BUILD_CVCPKG.md` → *Building and packaging wasm apps*.
