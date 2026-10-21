# Building the Full libcvc Project with cvcpkg Only

Everything in this document is driven exclusively by `cvcpkg` commands — no
`apt-get`, no manual `cmake` invocation. The complete dependency closure (the
same pinned set CI builds against) is installed into a prefix, then libcvc and
the `cvc` CLI are built from *this checkout* into that prefix.

This document uses **`/tmp/libcvc`** as the prefix. The commands work with any
prefix; the in-tree docs and CI use `deps` (relative to the repo root) — swap
the prefix anywhere in the blocks below.

## Commands

Run from the libcvc checkout root:

```bash
cd /path/to/libcvc

# 1. Install the full dependency closure of the `cvc` CLI (libcvc + boost + …)
#    into /tmp/libcvc. The closure is read from the in-tree recipe
#    cvcpkg/recipes/cvc-cli → cvcpkg/recipes/libcvc `depends:` block.
cvcpkg install-deps cvcpkg/recipes/cvc-cli --prefix /tmp/libcvc --config release --link shared

# 2. Build libcvc from this checkout into the prefix (--no-deps: reuse step 1).
cvcpkg build libcvc  --recipes-dir cvcpkg/recipes --no-deps --prefix /tmp/libcvc --config release

# 3. Build the `cvc` CLI from this checkout into the same prefix.
cvcpkg build cvc-cli --recipes-dir cvcpkg/recipes --no-deps --prefix /tmp/libcvc --config release
```

Result: `/tmp/libcvc/lib/libcvc*`, `/tmp/libcvc/lib/libxmlrpc*`,
`/tmp/libcvc/bin/cvc`, and the CMake package dir `/tmp/libcvc/lib/cmake/cvc/`
(targets `cvc::cvc`, `cvc::xmlrpc`), plus the entire prebuilt dependency closure
(boost, cgal, gmp, mpfr, hdf5, pocketfft, gsl, imagemagick + codecs, assimp,
levmar, libiimod, openblas, openssl/c-ares/re2/abseil/protobuf/grpc, yaml-cpp, …).

### Why step 1 targets `cvc-cli`, not `libcvc`

`cvc-cli`'s recipe is `source.type: vendored` (it builds the checkout it sits
in) and depends on `libcvc` + `boost`. `install-deps` therefore resolves the
CLI's *entire* closure and lands a **published** libcvc in the prefix. Step 2
then rebuilds libcvc **from this checkout** into the same prefix, overwriting
it — so the CLI (step 3) links against your tree rather than the published
bundle.

If you only want the SDK and not the CLI, use `install-deps
cvcpkg/recipes/libcvc` instead of step 1, and step 2 is the whole build.

### Alternative: one command, everything from source

```bash
cvcpkg build libcvc --with-deps --recipes-dir cvcpkg/recipes --prefix /tmp/libcvc --config release
```

This resolves and builds the entire closure **from source, topologically**
(deps first, then libcvc). Caveat: in this checkout, a few linux-only
recipes (e.g. `nlohmann-json`, `json-schema-validator`, `ftxui`) are not
present under `cvcpkg/recipes/` — they are catalog-published upstream
(cy-pca/cvcpkg). If the resolver can't find them locally, the
`install-deps` (prebuilt) path above is the safe recommendation.

## What "everything enabled" means

Feature flags fall into three buckets.

**Hardcoded by the libcvc recipe's `build.sh`** (not env-overridable):

| Flag | Value | Notes |
| --- | --- | --- |
| `CVC_USING_XMLRPC` | `ON` | builds the XML-RPC transport |
| `CVC_BUILD_CLI` | `OFF` | the CLI ships as the separate `cvc-cli` package (step 3) |
| `CVC_ENABLE_GRPC` | `OFF` | gRPC *dependencies* still land in the prefix, but the gRPC transport is compiled off in this build |
| `CVC_BUILD_PYCVC` | `OFF` | pycvc/cvcgl are separate recipes |

**Env-overridable** (set in front of the `build` command):

| Flag | Default | Effect when set |
| --- | --- | --- |
| `CVC_BUILD_TESTS=ON` | `OFF` | compiles and (via ctest) runs the gtest suite |
| `CVC_ENABLE_CUDA` | `OFF` | ON via the sibling `libcvc-cuda` recipe (`build.matrix.env`) |

**Auto-detected from the prefix:** the builder exports
`CVC_DEPS_PREFIX=<prefix>`, which the recipe's `build.sh` turns into
`-DCMAKE_PREFIX_PATH`. When the full closure from step 1 is present, all
auto-detect CMake features — HDF5, CGAL, GSL, ImageMagick, assimp, levmar,
yaml-cpp, ftxui, nlohmann-json, json-schema-validator, log4cplus — find their
libs and turn **ON**. That is what makes this the "everything enabled" build:
nothing is enabled except by being present in the prefix.

## Host tools

cmake and ninja come from your system `PATH`; `install-deps` installs only
library dependencies. Add `--include-host-tools` to step 1 if you want
cvcpkg to supply them in the prefix too. The compiler always comes from the
system.

## Verification

```bash
cvcpkg list --installed --prefix /tmp/libcvc   # closure + libcvc + cvc-cli present
ls /tmp/libcvc/lib/libcvc* /tmp/libcvc/lib/libxmlrpc* /tmp/libcvc/bin/cvc
ls /tmp/libcvc/lib/cmake/cvc/
```

To exercise the CLI: `LD_LIBRARY_PATH=/tmp/libcvc/lib /tmp/libcvc/bin/cvc --help`.

## Building and packaging wasm apps

cvcpkg treats WebAssembly as a first-class *target platform* (`--platform
wasm` / `wasm-mt`), and the Emscripten toolchain itself is a cvcpkg package
— so a wasm app can be built, packaged, and installed with cvcpkg commands
only. The existing libcvc/cvcGL wasm targets are the worked example:

- **`libcvc-examples-web` recipe** (in-tree `cvcpkg/recipes/libcvc-examples-web`)
  — its `wasm` and `wasm-mt` matrix entries (`build-wasm.sh`) cross-compile the
  browser demos and package the servable gallery under
  `share/libcvc-examples-web/web/`; the native `libcvc-examples` recipe ships the
  `libcvc-examples-web` launcher that serves it. This is the "packaged wasm app".
- **`wasm-demos` / `wasm-extra-demos` CMake targets** (`examples/`)
  — the gallery demos + `ariadne_hello` and arigl's `cyprty`, each a
  `cvcgl_wasm_app()` (see `docs/CVCGL_WASM.md`).
- **`libcvc` + `cvcgl` wasm-mt SDK bundles** — published by the
  `publish-cvcgl-wasm.yml` workflow (in the libcvc repo), which is the
  reference "build + package a wasm app" flow below.

Two platforms, **not interchangeable**: `wasm` is single-threaded,
`wasm-mt` adds `-pthread` + SharedArrayBuffer (served cross-origin
isolated, COOP/COEP). Emscripten forbids mixing `-pthread` and
non-`-pthread` objects, so the catalog keeps them separate and a `wasm`
bundle must never feed a `wasm-mt` consumer. All wasm builds are static
(`recipes/_common/env-wasm.sh` forces `BUILD_SHARED_LIBS=OFF`), which is
why every command below passes `--link static`.

### 1. Run a prebuilt wasm app (one command)

The launcher ships in the *native* `libcvc-examples` bundle (it serves the wasm
gallery out of the prefix's `share/libcvc-examples-web/web/`); the gallery itself
is the `libcvc-examples-web` wasm-mt bundle. So the documented pairing is both
into one prefix:

```bash
cvcpkg install libcvc-examples --prefix /tmp/cvc-wasm
cvcpkg install libcvc-examples-web --platform wasm-mt --arch wasm32 --link static \
    --prefix /tmp/cvc-wasm
/tmp/cvc-wasm/bin/libcvc-examples-web
```

The wasm-mt bundle is a self-contained static `.wasm` + JS gallery (no
runtime deps). The launcher runs `serve.py`, which sends the COOP/COEP
headers the `-pthread` build requires, and opens the browser.

### 2. Build + package your own wasm app on the cvcGL SDK

This is exactly the shape `publish-cvcgl-wasm.yml` uses. The only step not
driven by cvcpkg is the `emcmake` configure of your app's sources.

**a. Toolchain + dependency closure — all from the catalog:**

```bash
cvcpkg install cmake ninja --prefix /tmp/cvc-wasm/tools --config release
export PATH=/tmp/cvc-wasm/tools/bin:$PATH
cvcpkg install emsdk --platform linux --prefix /tmp/cvc-wasm/emsdk
export CVC_EMSDK_DIR=/tmp/cvc-wasm/emsdk

# The wasm-mt static closure your app links (the publish workflow's list):
for p in zlib bzip2 xz zstd libpng libjpeg-turbo tiff freetype libwebp \
         libxml2 lerc assimp imagemagick boost imgui sdl3 vtk; do
  cvcpkg install "$p" --platform wasm-mt --arch wasm32 --config release \
      --link static --prefix /tmp/cvc-wasm/deps
done
```

The `emsdk` recipe declares `cross_toolchain: target_platforms: [wasm,
wasm-mt]` and exports `CVC_EMSDK_DIR=<prefix>` — so when cvcpkg builds a
recipe *targeting* wasm, the toolchain is injected automatically and never
listed as a dependency. `emsdk` is a host tool (runs natively,
cross-compiles to wasm); install it for the *host* platform.

If you would rather *consume* the published cvcGL SDK bundles instead of
building libcvc/cvcGL yourself:

```bash
cvcpkg install cvc/libcvc cvc/cvcgl --platform wasm-mt --arch wasm32 \
    --link static --prefix /tmp/cvc-wasm/deps
```

(`cvc/<name>` is the org-qualified form of the cvc-published bundles;
unqualified names resolve against the default catalog.)

**b. Build + install the app (Emscripten cross):**

```bash
source "$CVC_EMSDK_DIR/emsdk_env.sh"
emcmake cmake -G Ninja -S . -B build-wasm \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/tmp/cvc-wasm/inst \
    -DCMAKE_FIND_ROOT_PATH=/tmp/cvc-wasm/deps \
    -DCVC_ENABLE_CUDA=OFF -DCVC_BUILD_TESTS=OFF -DCVC_BUILD_CLI=OFF \
    -DCVC_ENABLE_OPENMP=OFF -DDISABLE_CGAL=ON -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF -DCVC_ENABLE_IMAGEMAGICK=ON \
    -DCVC_ENABLE_FFTW=OFF -DCVC_FFT_PROVIDER=none -DCVC_ENABLE_ASSIMP=ON \
    -DCVC_ENABLE_MESHER=OFF -DCVC_ENABLE_SDF=ON \
    -DCVC_BUILD_CVCGL=ON -DCVC_WASM_PTHREADS=ON
cmake --build build-wasm --target cvc cvcGL -j
cmake --install build-wasm
```

- `-DCVC_WASM_PTHREADS=ON` makes the Emscripten `-pthread` flags follow and
  is recorded in the installed `cvcGLConfig.cmake` (`CVCGL_WASM_PTHREADS`),
  which is what turns `MIMALLOC AUTO` on for wasm-mt consumers.
- `-DCMAKE_FIND_ROOT_PATH` is what makes the wasm-mt closure under
  `/tmp/cvc-wasm/deps` discoverable under emcmake.
- A cvcGL app's CMakeLists opts into the browser-side fixes (WebGL state
  shim as `--pre-js`, mimalloc on wasm-mt) by calling
  `cvcgl_wasm_app(<target>)` — see `docs/CVCGL_WASM.md`.

**c. Package it with cvcpkg** (this is what makes it a distributable wasm
bundle, e.g. `dist/libcvc-examples-web-<ver>-wasm-mt-release-static.tar.gz`):

```bash
cvcpkg pack cvcpkg/recipes/libcvc-examples-web --from-prefix /tmp/cvc-wasm/inst \
    --platform wasm-mt --config release --link static --local \
    --output-dir dist
```

`--from-prefix` skips the build and archives the *installed* prefix against
the recipe's metadata (name, deps, cmake packages). The in-tree
`build-wasm.sh` shows the full packaging shape for the gallery: it runs
`examples/web/build-wasm-demo.sh`, which builds the demos and then has
`build-pages.py` turn the built `.wasm/.js` pairs into the servable gallery
(per-demo host page + index of cards) in `share/libcvc-examples-web/web/`, with
`THIRD_PARTY_NOTICES.md` and `LICENSE` shipped beside it for binary
redistribution.

**d. Publish (optional)** — then any consumer can run the one-liner from
step 1:

```bash
cvcpkg publish dist/libcvc-examples-web-*-wasm-mt-*.tar.gz \
    --server https://cvcpkg.org --org cvc --token "$CVCPKG_TOKEN"
```

### 3. The in-tree gallery dev loop (`examples/web/`)

`build-wasm-demo.sh` is the same flow outside cvcpkg, for every gallery demo
(`wasm-demos` target) + the single-threaded `ari` page; the
`libcvc-examples-web` recipe and the gh-pages deploy both run it:

```bash
cvcpkg install emsdk --platform linux --prefix <emsdk-dir>
export CVC_EMSDK_DIR=<emsdk-dir>
cvcpkg install boost zstd --platform wasm --arch wasm32 --link static \
    --prefix <deps>
# VTK with the rendering modules: the published compute-only vtk-wasm bundle
# does NOT link the demos — build it from this cvcpkg checkout's recipe:
cd /path/to/cvcpkg   # the checkout root auto-overlays its recipes/ dir
cvcpkg build vtk --platform wasm --local --prefix <deps>
cd /path/to/libcvc
export CVC_WASM_DEPS=<deps>
bash examples/web/build-wasm-demo.sh            # single-threaded
# or
bash examples/web/build-wasm-demo.sh --pthread  # threaded, needs
# a deps closure built ENTIRELY with CVC_WASM_THREADS=1 (wasm-mt), and the
# gallery must be served by serve.py (COOP/COEP), not http.server
```

The script packages the build with `build-pages.py` into
`build-wasm[-mt]/gallery/`. Two caveats it encodes: without `yaml-cpp` in
`CVC_WASM_DEPS` (`cvcpkg install yaml-cpp --platform wasm --arch wasm32 --link
static`) it builds one from source there, since the `.ari` demos' loader needs
it; and the threaded variant needs the whole closure `-pthread` (Emscripten
forbids mixing).

## Where this is already documented

- `libcvc/README.md` → *Quick Start* → **"Dependencies via cvcpkg
  (reproducible — matches CI)"** — the canonical `deps`-prefix two-command flow
  (SDK only).
- `cvc-engagement-docs/modernization/cvc-nav-and-grl-snam-guide.md` →
  **"Two canonical builds worth having in your fingers"** — the full-project
  (libcvc + `cvc` CLI) variant used in this document. (Mirrored in the
  `cvc-engagement-docs-*` worktree copies.)
- `cvcpkg install-deps` CLI help text ships the same example;
  `cvcpkg/docs/getting-started-tutorial.md` documents the generic
  `install-deps` + `build` loop.
- **wasm**: `libcvc/.github/workflows/publish-cvcgl-wasm.yml` — the
  build+package flow for the `libcvc`/`cvcgl`/`libcvc-examples-web`
  wasm-mt bundles (catalog-provisioned emsdk + wasm-mt closure, `emcmake`
  build, `cvcpkg pack --from-prefix`, `cvcpkg publish`); `cvcpkg/docs/wasm-packaging.md`
  + `cvcpkg/cmake/cvcpkg-wasm-app.cmake` — the *general* wasm-app guide for
  cvcpkg users (project organization, requirements files, the generic
  `cvcpkg_wasm_app()` boilerplate, packaging any wasm app as a bundle);
  `libcvc/docs/CVCGL_WASM.md`
  — the `cvcgl_wasm_app()` app contract (state shim, FrameYield, mimalloc,
  glsync, serve.py); `libcvc/examples/web/build-wasm-demo.sh`
  — the in-tree gallery build whose prerequisites all come from
  the cvcpkg catalog. The `libcvc-examples` and `libcvc-examples-web` recipes'
  own comments document the two-package install (native + wasm-mt) and the
  launcher.

What this document adds that the above don't: the `/tmp` prefix variant, the
"everything enabled" flag table, the `--with-deps` from-source alternative,
and the consolidated wasm build/package flow with `/tmp` prefixes.
