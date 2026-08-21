#!/usr/bin/env bash
# build-wasm-demo.sh — cross-compile the cvcGL lsystem_forest demo to WebAssembly.
#
# Prerequisites (all via the cvcpkg wasm channel):
#   1. The activated Emscripten SDK bundle:
#        cvcpkg install emsdk --platform linux --prefix <emsdk-dir>
#      and export CVC_EMSDK_DIR=<emsdk-dir>.
#   2. A wasm deps prefix holding boost/zstd/zlib plus a VTK built WITH the
#      rendering modules (recipes/vtk/build-wasm.sh on the feat/vtk-wasm-rendering
#      branch of libcvc-deps — the published compute-only vtk-wasm bundle will
#      NOT link this demo):
#        cvcpkg install boost zstd --platform wasm --arch wasm32 --link static \
#            --prefix <deps>
#        CVC_EMSDK_DIR=<emsdk-dir> cvcpkg build vtk --platform wasm --local \
#            --prefix <deps>          # from the libcvc-deps checkout root
#      and export CVC_WASM_DEPS=<deps>.
#
# Output: build-wasm/bin/{lsystem_forest.js,lsystem_forest.wasm,index.html}
# Serve:  python3 -m http.server -d build-wasm/bin 8811
# (Single-threaded build — no COOP/COEP headers required.)
#
# --pthread builds the threaded variant into build-wasm-mt/ instead. It needs
# a deps prefix whose ENTIRE closure was built with CVC_WASM_THREADS=1
# (Emscripten forbids mixing -pthread and non-pthread objects), and the page
# must be served cross-origin isolated: use wasm/serve.py, not http.server.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"

PTHREAD=OFF
BUILD_DIR="${REPO_ROOT}/build-wasm"
if [[ "${1:-}" == "--pthread" ]]; then
    PTHREAD=ON
    BUILD_DIR="${REPO_ROOT}/build-wasm-mt"
fi

: "${CVC_EMSDK_DIR:?point at the installed emsdk bundle}"
: "${CVC_WASM_DEPS:?point at the wasm deps prefix (boost/zstd/vtk-with-rendering)}"

# shellcheck disable=SC1091
source "${CVC_EMSDK_DIR}/emsdk_env.sh"

emcmake cmake -G Ninja -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_ROOT_PATH="${CVC_WASM_DEPS}" \
    -DCVC_ENABLE_CUDA=OFF \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_BUILD_CLI=OFF \
    -DCVC_ENABLE_OPENMP=OFF \
    -DDISABLE_CGAL=ON \
    -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF \
    -DCVC_ENABLE_IMAGEMAGICK=OFF \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_ENABLE_ASSIMP=OFF \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=OFF \
    -DCVC_STATE_EXEC=OFF \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=ON \
    -DCVC_WASM_PTHREADS=${PTHREAD}

cmake --build "${BUILD_DIR}" \
    --target lsystem_forest nav_city_swarm nav_fog_ghost -j "$(nproc)"

echo
echo "Done: ${BUILD_DIR}/bin/{lsystem_forest,nav_city_swarm,nav_fog_ghost}.{js,wasm}"
if [[ "${PTHREAD}" == "ON" ]]; then
    echo "Threaded build — serve cross-origin isolated:"
    echo "  python3 $(dirname "${BASH_SOURCE[0]}")/serve.py -d ${BUILD_DIR}/bin 8822"
else
    echo "Serve with: python3 -m http.server -d ${BUILD_DIR}/bin 8811"
fi
