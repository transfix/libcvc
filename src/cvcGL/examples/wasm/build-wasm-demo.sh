#!/usr/bin/env bash
# build-wasm-demo.sh — cross-compile the cvcGL browser demos to WebAssembly and
# assemble the servable gallery. Builds the CMake `wasm-demos` target (lsystem_forest
# + the nav demos) then runs build-pages.py; add a demo to _wasm_demos in the examples
# CMakeLists and it flows through here with no edits.
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
# Output: build-wasm[-mt]/gallery/ — index.html (cards) + one <demo>/ subdir each
#         holding its host page and .js/.wasm. Serve with wasm/serve.py.
# (Default build is single-threaded — no COOP/COEP headers required.)
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

# On a shared self-hosted runner host the emsdk's cache dir can be owned by a
# different user than the one running this job; emcc then can't write it and the
# build dies at "Check if compiler accepts -pthread - no" / "Could NOT find
# Threads". If the shared cache is not writable, point emcc at a per-user cache
# via a private EM_CONFIG. This MUST come after emsdk_env.sh, which clears
# EM_CONFIG. No-op when the shared cache is already writable (e.g. the owner).
if [ ! -w "${CVC_EMSDK_DIR}/upstream/emscripten/cache" ]; then
    _em_cfg="${HOME}/.emscripten-cvcgl"
    _em_cache="${HOME}/.emscripten-cache-cvcgl"
    _em_node="$(ls -d "${CVC_EMSDK_DIR}"/node/*/bin/node 2>/dev/null | head -1)"
    cat > "${_em_cfg}" <<EOF_EMCFG
NODE_JS = "${_em_node}"
LLVM_ROOT = "${CVC_EMSDK_DIR}/upstream/bin"
BINARYEN_ROOT = "${CVC_EMSDK_DIR}/upstream"
EMSCRIPTEN_ROOT = "${CVC_EMSDK_DIR}/upstream/emscripten"
CACHE = "${_em_cache}"
EOF_EMCFG
    mkdir -p "${_em_cache}"
    export EM_CONFIG="${_em_cfg}"
    echo "build-wasm-demo: shared emsdk cache not writable by $(id -un); using per-user EM_CONFIG=${_em_cfg}"
fi

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
    -DCVC_ENABLE_IMAGEMAGICK=ON \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_ENABLE_ASSIMP=ON \
    ${CVC_WASM_BUNDLE:+-DCVC_WASM_BUNDLE="${CVC_WASM_BUNDLE}"} \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=OFF \
    -DCVC_STATE_EXEC=OFF \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=ON \
    -DCVC_WASM_PTHREADS=${PTHREAD}

cmake --build "${BUILD_DIR}" --target wasm-demos -j "$(nproc)"

# Turn the built bin/ into a servable gallery: build-pages.py discovers every
# <demo>.js/.wasm pair, generates each demo's host page from templates/demo.html.in
# and the index of cards from templates/gallery.html.in (real thumbnails from
# gallery-assets/, a name-tile placeholder for any demo without one).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GALLERY="${BUILD_DIR}/gallery"
python3 "${HERE}/build-pages.py" \
    --bin "${BUILD_DIR}/bin" --out "${GALLERY}" --assets "${HERE}/gallery-assets"

echo
echo "Done: gallery at ${GALLERY}/ (index.html + one subdir per demo)"
if [[ "${PTHREAD}" == "ON" ]]; then
    echo "Threaded build — serve cross-origin isolated:"
    echo "  python3 ${HERE}/serve.py -d ${GALLERY} 8822"
else
    echo "Serve with: python3 -m http.server -d ${GALLERY} 8811"
fi
