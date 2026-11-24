# JSPI and wasm exceptions across the browser stack

**Status: plan.** Joe decided it on 2026-11-16: "we should use JSPI and also add `-fwasm-exceptions`
to packages on cvcpkg". It settles ARIADNE_SCRIPTING_HOSTS.md §9, item 13.

Every wasm host moves from Asyncify to JSPI (`-sJSPI`). libcvc and every cvcpkg wasm package move
from Emscripten's JS exceptions and longjmp to wasm's own (`-fwasm-exceptions`). The phases are
JX1 to JX4.

## 1. Why

transfix/libcvc#1272 measured it (ARIADNE_SCRIPTING_HOSTS.md §8.5, "As measured"):

- **Guests lose the Asyncify transform.** That more than doubles a metered guest, and runs
  call-heavy code at half speed (`fib` 30.1 ms against 14.1). A JSPI guest is the metered module as
  it is.
- **A pause costs about 1.0 µs at any depth.** Asyncify costs 0.5 µs at the top of the stack and
  3.7 µs 48 frames down.
- **The host's wait for a guest is 0.6 µs a slice at any depth.** Today's `ariglx` pays 11.4 µs at
  its real 34 frames.
- **The hosts lose their own Asyncify too.** That takes away §5.3's W1 costs: CPython at half speed,
  and 8% of the module.
- **The Asyncify audits go.** No more `ASYNCIFY_STACK_SIZE` (the 4 KiB default aborts at about 128
  frames), `ASYNCIFY_IGNORE_INDIRECT` or `FRAME_YIELD_LOCKED`.
- **Rust.** Since Rust 1.93, Rust's Emscripten target uses wasm exceptions only. With the opt-out
  gone in 1.98, C++ linked with Rust has to use them too. That is cvcdbg's blocker (the cvcdbg and
  cvcpkg TODOs).

## 2. Why every piece moves together

**A JSPI host can't be built with JS exceptions.**
- libcvc builds with `-fexceptions` (`CMakeLists.txt:101-105`). Every call past an object with a
  destructor then goes through a JS `invoke_*`.
- Under `-sJSPI`, Emscripten makes every `invoke_*` a suspending import. V8 throws "SuspendError:
  trying to suspend JS frames" for a nested one, even when nothing waits (#1272).

**Asyncify and wasm exceptions don't compose.** emsdk 5.0.7 warns: "ASYNCIFY=1 is not compatible
with -fwasm-exceptions. Parts of the program that mix ASYNCIFY and exceptions will not compile". So
the move has no Asyncify fallback. A fallback would be a second, JS-exception set of every wasm
package.

**The break runs both ways.** Linked with emsdk 5.0.7: `setjmp`/`longjmp` in C, a throw past a
destructor in C++, and a C++ `main` that catches.

| main | longjmp in C | throw in C++ | links | runs |
|---|---|---|---|---|
| JS EH | JS | JS EH | yes | longjmp, catch and destructor work |
| wasm EH | wasm | wasm EH | yes | longjmp, catch and destructor work |
| wasm EH | **JS** | wasm EH | **no**: `emscripten_longjmp` undefined | |
| wasm EH | wasm | **none** (emcc's default) | yes | catch works; that object's destructors don't run (as today) |
| wasm EH | wasm | **JS EH** | **no**: `__resumeException` undefined | |
| **JS EH** | **wasm** | wasm EH or JS EH | **no**: `__wasm_longjmp`, `__c_longjmp` undefined | |
| wasm EH + `-sJSPI` (also with `-pthread`) | wasm | wasm EH | yes | not run: node 22.16's V8 lacks JSPI |

So the packages that use JS exceptions or JS longjmp move in one step with every consumer that
links them. Packages that use neither link with both sides, so they can be rebuilt on any schedule.

**Which packages use them.** JX1's pack gate checked the latest wasm and wasm-mt bundle of every
component on cvcpkg.org, 126 bundles. 16 components have JS-exception or JS-longjmp objects:
- JS longjmp: boost (its test libraries), freetype, libjpeg-turbo, libpng, lua, qhull and tiff;
- JS exceptions: datachannel-wasm, and mujoco and mujoco3 (as LTO bitcode);
- both: imagemagick, qt6 and vtk;
- libcvc's own: libcvc, cvcgl and pycvc-gl-cp312.

python312, numpy, vtk-python-cp312 and the rest use neither.

**Browsers.** All three engines ship JSPI:
- Chrome since 137 (May 2025);
- Firefox since 153 (21 July 2026);
- Safari since 27 (September 2026).

Every one of them also has wasm exceptions. A browser without JSPI gets a page that says what it
needs (JX4), not a fallback.

## 3. Phases

| Phase | What | Needs | Size |
|---|---|---|---|
| JX1 | cvcpkg: `-fwasm-exceptions` in `_common/env-wasm.sh`, the pack gate, and the 13 recipes that need it rebuilt to the dev catalog (cy-pca/cvcpkg PR, open until JX4) | — | M |
| JX2 | libcvc: `-fwasm-exceptions`, every host on `-sJSPI`, the suspension audit, and tests on a node with JSPI | JX1 on dev | M |
| JX3 | Guests on JSPI: the gas pass without Asyncify, and `browser_runtime.cpp` with a suspending `ari.gas` | JX2 | S |
| JX4 | The switch: JX1 to prod, JX2 and JX3 merged, consumers moved the same day, and the browser gate | JX1-JX3 | S |

### JX1: cvcpkg

- **`recipes/_common/env-wasm.sh` and `env-wasm.ps1`** export `EMCC_CFLAGS=-fwasm-exceptions`, for
  wasm and wasm-mt alike. emcc adds it to every compile and link (emsdk 5.0.7, `emcc.py:194`).
  - For C, it makes `setjmp` use wasm longjmp. For C++, a package can throw and catch, with its
    destructors run.
  - Not `CFLAGS`: several recipes build native host tools from the same environment (CPython's
    build Python, protoc), and gcc rejects the flag. emcc keeps `EMCC_CFLAGS` out of its own system
    libraries (`system_libs.py:89-95`).
  - A package's own `-fexceptions` loses to it: the object comes out with wasm exceptions. Its own
    `-fno-exceptions` wins, which links either way. Its own `-sDISABLE_EXCEPTION_CATCHING=0` is an
    error, which the dev rebuild shows.
  - wasi keeps `env-wasi.sh` as it is.
- **The recipes' own `-fexceptions` goes:** `datachannel-wasm/build-wasm.sh:15`, and
  `pillow-cp312/build-wasm.sh:173,186`.
- **A pack gate** (`cvcpkg.wasm_eh`) refuses a wasm or wasm-mt bundle with a member that imports
  `emscripten_longjmp`, `__resumeException`, `__cxa_find_matching_catch_*` or `invoke_*`.
  - It reads the wasm import sections itself, with no llvm-nm.
  - For LTO bitcode it checks the C++ personality: `__gxx_personality_v0` is JS exception IR. setjmp
    in bitcode is lowered at the consumer's link, so that part doesn't matter.
  - It catches a build script that drops `EMCC_CFLAGS`.
- **Rebuild only the 13 recipes that need it** (section 2's list, less libcvc's own): bump their
  `cvc_revision`.
  - Every other recipe's wasm objects link either way and pick the flag up at their next bump.
  - A bump also opens a gap in every native column, which a default populate would fill. So
    populate with `platforms=wasm,wasm-mt` only.
  - VTK takes longest: over 90 minutes a flavour on the fleet.
- **The PR stays open until JX4.** Once the flag is on master, the next prod populate publishes
  wasm-exception bundles that today's libcvc can't link. And the gate would fail every wasm pack
  of a package that hasn't been rebuilt. Dev rebuilds run from the PR's branch.
- **Rust.** Rust recipes already use wasm exceptions (Rust 1.93 and later), so nothing changes
  there.
- **Encoding.** Keep Emscripten's default `WASM_LEGACY_EXCEPTIONS`. Every browser with JSPI also has
  exnref, so moving later is Binaryen's translation at link.

### JX2: libcvc

- **Flags.**
  - `CMakeLists.txt:101-105`: `-fexceptions` becomes `-fwasm-exceptions`, at compile and link.
  - `bindings/pycvc/wasm/link-host.sh:153`: the same.
  - `-sDISABLE_EXCEPTION_CATCHING=0` is an error under wasm exceptions, so drop it wherever it
    appears (cvcdbg's tests link it).
- **Every Asyncify host moves to `-sJSPI`:**
  - ari: `src/ari/CMakeLists.txt:480`;
  - arigl, ariglx and cyprty: `src/arigl/CMakeLists.txt:241,280,340`;
  - the gallery demos: `examples/CMakeLists.txt:258`;
  - `cvcgl_wasm_app()`.

  `ASYNCIFY_STACK_SIZE`, `ASYNCIFY_IGNORE_INDIRECT` and `FRAME_YIELD_LOCKED` go with Asyncify
  (`src/cvcGL/wasm/cvcGLWasm.cmake`).
- **The suspension audit.**
  - Asyncify could suspend under any export. JSPI suspends only under an export wrapped by
    `WebAssembly.promising`. `main` is one; the rest have to be listed in `JSPI_EXPORTS`, and their
    JS callers must await them.
  - No JS frame may sit between that export and the suspending import. That means an `EM_JS` that
    calls back into wasm, a DOM or SDL event callback, or a call through `dynCall`.
  - The suspension sites today, all in `main`'s run loops as far as is known:
    - `FrameYield`'s `emscripten_sleep(0)` (`ViewportManager.cpp`, and VTK's canvas window in
      `SdlWindow.cpp:348`);
    - the FTXUI run loop (`ftxui_backend.cpp:4267`);
    - `app_runtime.cpp:124`'s settle;
    - `ws_client.cpp:365,380`'s blocking send;
    - `http_client_fetch.cpp:278`;
    - `web_document.cpp`'s `ariadne_fetch_begin`;
    - `browser_runtime.cpp`'s `ASYNC_LOAD` compile and instantiate.

    Prove each one is reached only from `main`, or wrap its entry.
- **Tests.** CI runs the wasm tests under node. emsdk 5.0.7's node 22.16 predates V8's JSPI API, and
  cvcpkg packages only `nodejs22`. So JX2 either adds a newer nodejs recipe or runs the JSPI suites
  in headless Chrome, as #1272's bench does.
- **Packages.** Until JX4, CI builds against JX1's packages on the dev catalog.

### JX3: guests

- **`cvc_ariadne_wasm_transform`** (`src/ariadne_wasm/ariadne_wasm.cmake`) keeps the gas pass and
  drops Asyncify for browser guests. It keeps the Asyncify output only if G3's WAMR still needs it.
- **`browser_runtime.cpp`** drives a guest as #1272's bench `jspiGuest` does:
  - `ari.gas` is a `WebAssembly.Suspending`, whose promise the next poll resolves;
  - the guest's exports go through `WebAssembly.promising`.

  The gas counts are the same as under Asyncify (#1272 checked them at 64,000-, 8,000- and
  1,000-gas slices).
- **ARIADNE_RESIDENT_GUESTS.md's RG6 and RG7 follow** (the follow-up agreed with the G4 session):
  - a module that arrives at run time pauses after the page's own gas pass alone;
  - the Worker backstop uses JSPI.

### JX4: the switch

- **Publish and merge on one day.**
  1. Merge JX1's cvcpkg PR, and populate prod from master with `platforms=wasm,wasm-mt`.
  2. Merge JX2 and JX3 right after. Between the two, libcvc master's wasm CI (JS exceptions against
     wasm-longjmp packages) can't link.
  3. Release libcvc's wasm packages (ari-web, arigl-web, pycvc-gl, libcvc wasm and wasm-mt) on
     Joe's go.
- **Consumers that link these packages with their own JS-exception code move the same day:**
  - cvcdbg: it drops `-sDISABLE_EXCEPTION_CATCHING=0`, and Rust can then link into its wasm build;
  - cyprty;
  - VolRover3.

  cypkg and nicotina run the packaged ari-web, so they move with its release.
- **The browser gate (built).** A host page checks for JSPI (`WebAssembly.Suspending` and
  `WebAssembly.promising` are functions) before it fetches anything of the module. Without it,
  the page says what the browser needs in place of a load that would fail part way.
  - **The pages.** The two stock loaders: `src/arigl/web/arigl-web.js` (arigl-web, and ariglx-web,
    the same page with `data-module="ariglx"`) and `src/ari/web/ari-web.js` (`ari.html`,
    `ari_tvision.html`). And the pages with their own inline script: the gallery's demo page
    (`examples/web/templates/demo.html.in`, whose `?gldiag` report also gets a `jspi` line),
    `src/arigl/web/arigl_term.html` and `src/arigl/web/cyprty.html`. Not the bench
    (`src/ariadne_wasm/bench/jspi_bench.html`), which measures JSPI against Asyncify.
  - **What it says.** "This page needs a newer browser"; that the page runs on WebAssembly with
    JSPI, which this browser does not have; and Chrome 137, Firefox 153 or Safari 27, or later
    (Edge 137 and other browsers built on Chrome 137 too). Every browser on an iPhone or iPad is
    WebKit, so there it asks for iOS 27 (iPadOS 27 on an iPad), whichever browser it is, and gives
    the computer versions after. It tells an iPhone or iPad by its user agent (`iPhone`, `iPad`,
    `iPod`), or by a Mac user agent with a touch screen, which is what iPadOS sends. The browser's
    user agent comes last.
  - **One copy per page.** Each page carries the check and the text itself, not a shared file:
    the arigl-web recipe and cyprty's `stage.sh` ship a fixed list of files, and a page that
    names a file they don't ship would break. So cyprty gets the gate with the arigl-web release
    that has it, and needs no change.
  - **State.** arigl-web and `ari.html` set `window.ariadneWeb.state` to `unsupported`, with the
    text in `ariadneWeb.needs`.
  - **The test.** `src/arigl/web/test/arigl_web_smoke.mjs` (CI's arigl-web and ariglx-web jobs)
    deletes `WebAssembly.Suspending` before the page's own script runs. arigl-web (and
    ariglx-web, through the same page) must name the three browsers and not iOS. As Chrome on an
    iPhone (its user agent), it must ask for iOS 27. `ari.html` must do the same in place of its
    terminal, and ask for iOS 27 as Safari on an iPad (a Mac user agent with touch). None of them
    may fetch the module's `.js`, its `.wasm` or `python312.zip`. Every other case is the normal
    run, through the same gate.

## 4. Risks

- **JSPI is "still experimental" to emsdk 5.0.7** (its link warning), though the three engines ship
  it.
- **The audit is the real work.** A suspension under an unwrapped entry fails at run time with
  SuspendError, not at link. Each host's smoke has to reach every suspension site.
- **The wasm-mt flavour.** JSPI with `-pthread` links (the table), but no threaded host has run on
  it yet.
- **The dev rebuild is the first build of these 13 recipes with the flag** on the fleet's Linux
  hosts. Locally, only libjpeg-turbo was built (Windows host, `build-wasm.ps1`). It came out with
  wasm longjmp and packed, while its published cvc.3 objects are refused.
