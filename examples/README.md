# libcvc examples

Programs and `.ari` documents that show libcvc at work: cvcGL scenes, `cvc::nav` swarms, and
Ariadne apps. They build with cvcGL:

```sh
cmake -B build -DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON
cmake --build build
```

Packaged, they are two cvcpkg packages. `libcvc-examples` has the native programs and the
documents (`share/libcvc-examples/ariadne/`). `libcvc-examples-web` has the browser gallery
that [transfix.github.io/libcvc](https://transfix.github.io/libcvc/) serves.

| directory | what is in it |
|---|---|
| [`cvcgl/`](cvcgl) | scene demos on the cvcGL scene graph: `lsystem_forest` ([LSYSTEM_FOREST.md](cvcgl/LSYSTEM_FOREST.md)), `lsystem_coast`, `terrain_lab`, and the `bunny_shadow` / `volren_bunny` / `volslice_bunny` benches |
| [`nav/`](nav) | the GRL-SNAM navigation demos rendered through cvcGL, and three headless `cvc::nav` CLIs ([NAV_DEMOS.md](nav/NAV_DEMOS.md)) |
| [`ariadne/documents/`](ariadne/documents) | `.ari` documents for a generic host: `arigl doc.ari` (a window), `arigl --terminal doc.ari`, or `ariadne_hello` |
| [`ariadne/terminal/`](ariadne/terminal) | GL-free documents for the terminal runner: `ari run ariadne/terminal/terminal_tour.ari` |
| [`ariadne/hosts/`](ariadne/hosts) | C++ Ariadne hosts. `ariadne_hello` is the generic one, the reference for embedding the runtime ([ARIADNE.md](../docs/ARIADNE.md) §9.1); `ariadne_terminal`, `ariadne_stream` and `ariadne_conference` add the terminal, streaming and conference pieces |
| [`ariadne/nav_convoy/`](ariadne/nav_convoy), [`ariadne/nav_compute/`](ariadne/nav_compute) | hosts that register their own verbs, each beside the document that calls them |
| [`web/`](web) | the browser gallery: `build-wasm-demo.sh` cross-compiles every `wasm-demos` target and `build-pages.py` writes the card index and a page per demo ([CVCGL_WASM.md](../docs/CVCGL_WASM.md)) |

Not here: the runtimes themselves (`src/ari`, `src/arigl`, and arigl's browser build `cyprty`,
`src/arigl/web`), the dev server `serve.py` (`src/cvcGL/wasm/devtools`), and OceanFFT, which
`lsystem_coast` uses and a cvcGL test checks (`src/cvcGL/ocean`).
