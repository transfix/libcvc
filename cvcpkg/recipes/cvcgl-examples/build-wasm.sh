#!/usr/bin/env bash
# cvcpkg/recipes/cvcgl-examples/build-wasm.sh — cross-compile the cvcGL example
# programs to WebAssembly and package the browser build under
# share/cvcgl-examples/web/ (served by the native bundle's cvcgl-examples-web).
#
# There is no wasm libcvc bundle: the whole (trimmed) cvc core + cvcGL closure
# is compiled from the repo root — the same option set as
# src/cvcGL/examples/wasm/build-wasm-demo.sh. Deps (boost/zstd/zlib/cgal + the
# rendering-enabled vtk >= 9.5.0+cvc.3) come static from CVC_DEPS_PREFIX;
# emsdk is injected by the builder via CVC_EMSDK_DIR (cross_toolchain).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../_common/env-wasm.sh"

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

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
    -DCVC_ENABLE_ASSIMP=ON \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=OFF \
    -DCVC_STATE_EXEC=OFF \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=ON
cmake --build "$CVC_BUILD_DIR" --target lsystem_forest -j "$CVC_JOBS"

WEB="$CVC_INSTALL_DIR/share/cvcgl-examples/web"
install -d "$WEB"
install -m 644 "$CVC_BUILD_DIR/bin/lsystem_forest.js"   "$WEB/"
install -m 644 "$CVC_BUILD_DIR/bin/lsystem_forest.wasm" "$WEB/"
# examples/CMakeLists.txt copies wasm/index.html next to the output POST_BUILD.
install -m 644 "$CVC_BUILD_DIR/bin/index.html"          "$WEB/"
