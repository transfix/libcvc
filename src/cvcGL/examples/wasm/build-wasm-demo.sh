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
    # Seed once from the shared cache so the sysroot is internally consistent: a
    # from-scratch cache builds fresh libc++ headers that then clash with the
    # shared install's older <math.h> (isinf macro breaks std::isinf in <complex>).
    # Copying also brings the shared cache's prebuilt libs, so no slow rebuild.
    if [ ! -e "${_em_cache}/.cvc-seeded" ]; then
        rm -rf "${_em_cache}"
        mkdir -p "${_em_cache}"
        cp -a "${CVC_EMSDK_DIR}/upstream/emscripten/cache/." "${_em_cache}/" 2>/dev/null || true
        touch "${_em_cache}/.cvc-seeded"
    fi
    export EM_CONFIG="${_em_cfg}"
    echo "build-wasm-demo: shared emsdk cache not writable by $(id -un); using seeded per-user cache ${_em_cache}"
fi

# yaml-cpp for wasm — the Ariadne .ari loader's only extra dependency (src/cvc/
# CMakeLists.txt: find_package(yaml-cpp) → CVC_ARIADNE_HAVE_YAML). It is NOT in
# the cvcpkg wasm catalog (published linux/macos/win/bsd only), so without this
# the loader silently compiles to a stub and the .ari demos (volren_bunny_ari,
# volslice_bunny_ari) deploy BLANK. Build it from the pinned upstream tarball
# into CVC_WASM_DEPS, STATIC, and pthread-matched to the module so the emscripten
# link doesn't reject mixed -pthread objects. Idempotent: skipped once installed.
if [ ! -f "${CVC_WASM_DEPS}/lib/cmake/yaml-cpp/yaml-cpp-config.cmake" ] \
   && [ ! -f "${CVC_WASM_DEPS}/share/cmake/yaml-cpp/yaml-cpp-config.cmake" ]; then
    echo "build-wasm-demo: yaml-cpp not in ${CVC_WASM_DEPS}; building it for wasm from source"
    _yc_ver=0.8.0
    _yc_sha=fbe74bbdcee21d656715688706da3c8becfd946d92cd44705cc6098bb23b3a16
    _yc_work="${BUILD_DIR}/_yaml-cpp"
    _yc_src="${_yc_work}/yaml-cpp-${_yc_ver}"
    _yc_tar="${_yc_work}/yaml-cpp-${_yc_ver}.tar.gz"
    mkdir -p "${_yc_work}"
    if [ ! -d "${_yc_src}" ]; then
        # Fetch with python3 (always present — build-pages.py needs it) rather than curl: the deploy
        # runner prepends a cvcpkg lib dir that shadows the system libcurl with an older one, so the
        # stock `curl` binary dies at startup ("undefined symbol: curl_easy_header"). urllib uses the
        # system SSL stack, not libcurl, so it is immune. sha256 still pins the payload.
        python3 - "https://github.com/jbeder/yaml-cpp/archive/refs/tags/${_yc_ver}.tar.gz" \
            "${_yc_tar}" <<'PY'
import sys, urllib.request
urllib.request.urlretrieve(sys.argv[1], sys.argv[2])
PY
        echo "${_yc_sha}  ${_yc_tar}" | sha256sum -c -
        tar -xzf "${_yc_tar}" -C "${_yc_work}"
    fi
    _yc_pthread=""
    [[ "${PTHREAD}" == "ON" ]] && _yc_pthread="-pthread"
    emcmake cmake -G Ninja -S "${_yc_src}" -B "${_yc_work}/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="${CVC_WASM_DEPS}" \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF \
        -DYAML_CPP_BUILD_CONTRIB=OFF \
        -DYAML_BUILD_SHARED_LIBS=OFF \
        -DCMAKE_CXX_FLAGS="${_yc_pthread}" \
        -DCMAKE_C_FLAGS="${_yc_pthread}"
    cmake --build "${_yc_work}/build" --target install
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
    -DCVC_FFT_PROVIDER=none \
    -DCVC_ENABLE_ASSIMP=ON \
    ${CVC_WASM_BUNDLE:+-DCVC_WASM_BUNDLE="${CVC_WASM_BUNDLE}"} \
    ${CVC_WASM_NAV_WEIGHTS:+-DCVC_WASM_NAV_WEIGHTS="${CVC_WASM_NAV_WEIGHTS}"} \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=ON \
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
