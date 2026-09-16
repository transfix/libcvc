#!/usr/bin/env bash
# cvcpkg/recipes/cvcgl-examples/build-wasm.sh — cross-compile the cvc::nav /
# cvcGL demo GALLERY to WebAssembly and package it under
# share/cvcgl-examples/web/ (served by the native bundle's cvcgl-examples-web).
#
# Ships the SAME gallery that deploys to gh-pages — terrain_lab + the nav demos
# (nav_city_swarm / nav_city_drive / nav_fog_ghost), THREADED (wasm-mt) — not
# just lsystem_forest. There is no wasm libcvc bundle: the whole (trimmed) cvc
# core + cvcGL closure is compiled from the repo root with the same option set as
# src/cvcGL/examples/wasm/build-wasm-demo.sh. Deps (boost/zstd/zlib + the
# rendering-enabled vtk >= 9.5.0+cvc.3) come static from CVC_DEPS_PREFIX as their
# wasm-mt variants; emsdk is injected by the builder via CVC_EMSDK_DIR.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../_common/env-wasm.sh"

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

# Preload the real Austin scene + trained policy into the nav_city demos' MEMFS
# when the data packages are staged (declared wasm-mt build deps), so the browser
# gallery shows downtown Austin — matching the gh-pages build. Absent the data,
# the demos fall back to a synthetic city + the default-biased net.
BUNDLE_ARGS=()
_austin="$CVC_DEPS_PREFIX/share/cvc-scenes/austin_south"
_weights="$CVC_DEPS_PREFIX/share/grl-snam-weights/coef_sdf.cvcnav"
[ -d "$_austin" ] && BUNDLE_ARGS+=("-DCVC_WASM_BUNDLE=$_austin")
[ -f "$_weights" ] && BUNDLE_ARGS+=("-DCVC_WASM_NAV_WEIGHTS=$_weights")

emcmake cmake -G Ninja \
    -S "$CVC_SOURCE_DIR" \
    -B "$CVC_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE" \
    -DCMAKE_FIND_ROOT_PATH="$CVC_DEPS_PREFIX" \
    -DCVC_ENABLE_CUDA=OFF \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_BUILD_CLI=OFF \
    -DCVC_ENABLE_OPENMP=OFF \
    -DDISABLE_CGAL=ON \
    -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF \
    -DCVC_ENABLE_IMAGEMAGICK=ON \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_FFT_PROVIDER=none \
    -DCVC_ENABLE_ASSIMP=ON \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=ON \
    -DCVC_STATE_EXEC=OFF \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=ON \
    -DCVC_WASM_PTHREADS=ON \
    ${BUNDLE_ARGS[@]+"${BUNDLE_ARGS[@]}"}

# Build the nav/scene gallery subset — NOT the shared `wasm-demos` target, which
# also carries lsystem_forest / bunny / volren and drives gh-pages. Each target
# still gets its Emscripten link flags from examples/CMakeLists.txt.
cmake --build "$CVC_BUILD_DIR" \
    --target terrain_lab nav_city_swarm nav_city_drive nav_fog_ghost \
    -j "$CVC_JOBS"

# Assemble the servable gallery (card index + per-demo pages) straight into the
# packaged web dir, reusing the same build-pages.py / templates / thumbnails as
# gh-pages. build-pages.py packages whatever .wasm/.js it finds under --bin.
WEB="$CVC_INSTALL_DIR/share/cvcgl-examples/web"
install -d "$WEB"
python3 "$CVC_SOURCE_DIR/src/cvcGL/examples/wasm/build-pages.py" \
    --bin "$CVC_BUILD_DIR/bin" \
    --out "$WEB" \
    --assets "$CVC_SOURCE_DIR/src/cvcGL/examples/wasm/gallery-assets"
