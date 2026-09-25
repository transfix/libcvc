#!/usr/bin/env bash
# recipes/pycvc-gl-cp312/build-wasm.sh — cross-build pycvc + pycvc_gl (BRIDGE=ON)
# to WebAssembly as STATIC archives and link a CPython-wasm host that embeds them.
#
# Unlike the native build (which compiles only bindings/pycvc against an installed
# libcvc SDK), the wasm build compiles the WHOLE trimmed cvc + cvcGL + pycvc +
# pycvc_gl closure from the repo root under emcc — mirroring cvcgl-examples/
# build-wasm.sh, the proven cvc-on-wasm pattern (there is no shared-lib wasm libcvc
# to link). The SWIG wrappers build as static archives (bindings/pycvc/CMakeLists.txt
# TYPE STATIC on EMSCRIPTEN); the host launcher (bindings/pycvc/wasm/pycvc_host.cpp)
# registers their PyInit__pycvc / PyInit__pycvc_gl via PyImport_AppendInittab.
#
# Flavor: env-wasm.sh sets CVC_WASM_THREADS=1 for the wasm-mt column and prepends
# -pthread; pass it through as CVC_WASM_PTHREADS so the whole closure is one flavor
# (emscripten forbids mixing). Deps (python312, numpy-cp312, vtk, vtk-python-cp312,
# boost/zstd/zlib/assimp/...) come from CVC_DEPS_PREFIX as their matching-flavor
# wasm/wasm-mt static variants.
#
# FIRST CUT — expected to iterate on the fleet (Python3 cross-detection, the
# vtk-python static _load() symbol for the bridge, and the final host link line
# are the likely rough edges). See docs/roadmap/static-single-binary-python.md.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

: "${CVC_SOURCE_DIR:?}"   # libcvc repo root (vendored ../../..)
: "${CVC_BUILD_DIR:?}"
: "${CVC_INSTALL_DIR:?}"
: "${CVC_DEPS_PREFIX:?}"

# ── (1) native host tools (emcc can't run the SWIG/codegen the build invokes) ──
# SWIG runs natively to generate the wrappers; CMake's FindPython3 needs a native
# interpreter to introspect (numpy include dir etc.) even though the link target is
# the wasm libpython3.12.a in CVC_DEPS_PREFIX. Provision a native toolchain like
# the vtk-python wasm recipe does.
HOSTENV="${CVC_BUILD_DIR}/hostenv"
_hp="$(uname -s 2>/dev/null || echo Linux)"; case "${_hp}" in Linux) _hp=linux;; Darwin) _hp=macos;; *) _hp=linux;; esac
_cvc="cvcpkg"; command -v cvcpkg >/dev/null 2>&1 || _cvc="python3 -m cvcpkg"
${_cvc} install python312 swig cmake ninja \
    --platform "${_hp}" --config release --link shared \
    --prefix "${HOSTENV}" --no-fallback-to-source >&2
export PATH="${HOSTENV}/bin:${PATH}"
PY_NATIVE="${HOSTENV}/bin/python3.12"
NUMPY_INC="$("${PY_NATIVE}" -c 'import numpy; print(numpy.get_include())' 2>/dev/null || \
            echo "${CVC_DEPS_PREFIX}/lib/python3.12/site-packages/numpy/_core/include")"

# ── (2) emsdk PATH + -pthread flavor (must come AFTER host tools: sets CC=emcc) ──
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../_common/env-wasm.sh"
: "${CVC_JOBS:=$(nproc 2>/dev/null || echo 4)}"
_PTHREADS=OFF
[[ "${CVC_WASM_THREADS:-0}" == "1" ]] && _PTHREADS=ON

# Point config-mode find_package(Boost) (CMP0167=NEW) straight at the cvcpkg boost
# config dir — avoids the emscripten cross FIND_ROOT_PATH re-rooting trap where a
# prefix search for BoostConfig.cmake resolves to the wrong path. Empty if absent
# (harmless — falls back to the normal search).
_BOOST_DIR="$(find "${CVC_DEPS_PREFIX}/lib/cmake" -maxdepth 1 -type d -name 'Boost-*' 2>/dev/null | head -1)"

# ── (3) configure the trimmed closure from the repo root (static, BRIDGE=ON) ──
# Same OFF set as cvcgl-examples/build-wasm.sh (the wasm-linkable subset) PLUS the
# pycvc bindings: CVC_BUILD_PYCVC=ON builds bindings/pycvc (core + gl) in-tree.
emcmake cmake -G Ninja \
    -S "${CVC_SOURCE_DIR}" \
    -B "${CVC_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DCMAKE_FIND_ROOT_PATH="${CVC_DEPS_PREFIX}" \
    -DBUILD_SHARED_LIBS=OFF \
    `# cmake >=3.30 deprecates the legacy FindBoost module (CMP0167); its module` \
    `# mode fails to locate the cvcpkg boost layout under the cross FIND_ROOT_PATH.` \
    `# NEW = use boost's own BoostConfig.cmake (config mode), which the wasm boost` \
    `# package ships (lib/cmake/Boost-*). Fixes "Could NOT find Boost" at configure.` \
    -DCMAKE_POLICY_DEFAULT_CMP0167=NEW \
    -DBoost_DIR="${_BOOST_DIR}" \
    -DCVC_ENABLE_CUDA=OFF \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_BUILD_CLI=OFF \
    -DCVC_ENABLE_OPENMP=OFF \
    -DDISABLE_CGAL=ON \
    -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF \
    -DCVC_ENABLE_IMAGEMAGICK=OFF \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_FFT_PROVIDER=none \
    -DCVC_ENABLE_ASSIMP=ON \
    -DCVC_ENABLE_MESHER=OFF \
    -DCVC_ENABLE_SDF=ON \
    -DCVC_STATE_EXEC=OFF \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=OFF \
    -DCVC_WASM_PTHREADS="${_PTHREADS}" \
    -DCVC_BUILD_PYCVC=ON \
    -DCVC_BUILD_PYCVC_CORE=ON \
    -DCVC_BUILD_PYCVC_GL=ON \
    -DCVC_PYCVCGL_VTK_BRIDGE=ON \
    -DPython3_EXECUTABLE="${PY_NATIVE}" \
    -DPython3_NumPy_INCLUDE_DIRS="${NUMPY_INC}" \
    -DSWIG_EXECUTABLE="${HOSTENV}/bin/swig"

# ── (4) build + install the static SWIG archives + the cvc/cvcGL closure ──
# Build the default targets (cvc + cvcGL + pycvc + pycvc_gl; examples/tests/cli are
# OFF), then install — the install rules place _pycvc.a / pycvc_gl/_pycvc_gl.a + the
# .py proxies + libcvc.a/libcvcGL.a into CVC_INSTALL_DIR, which the host link reads.
# Keep-going (-- -k 0): cvcGL's CMake adds ~20 test/example executables
# (cvcgl_ocean_fft/renderer/texture/...) UNCONDITIONALLY (not gated by
# CVC_BUILD_TESTS/EXAMPLES), and wasm-opt -O3 crashes linking some of them on
# some hosts (Windows: 0xC0000409). They are NOT part of the pycvc-gl package, so
# tolerate their failure and press on — the libs (libcvc/libcvcGL), the SWIG
# archives (_pycvc.a/_pycvc_gl.a) and the .py proxies all build fine and have no
# dependency on those test exes. The archive check below is the real gate.
cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}" -- -k 0 || true
cmake --install "${CVC_BUILD_DIR}"
_PXA="$(find "${CVC_INSTALL_DIR}" -name '_pycvc.a' 2>/dev/null | head -1)"
_GLA="$(find "${CVC_INSTALL_DIR}" -name '_pycvc_gl.a' 2>/dev/null | head -1)"
[ -n "${_PXA}" ] && [ -n "${_GLA}" ] || {
    echo "pycvc-gl(wasm): FATAL — static archives not installed" >&2
    find "${CVC_INSTALL_DIR}" -name '*pycvc*.a' >&2 2>/dev/null; exit 1; }
echo "pycvc-gl(wasm): installed ${_PXA} + ${_GLA}"

# ── (5) link the CPython-wasm host (embeds pycvc + pycvc_gl + numpy) ──────────
# link-host.sh is the PROVEN link (numpy static-embed registrar + the closure
# group + MEMFS stdlib/proxies → pycvc_host.{wasm,js}). Non-fatal: the archives are
# the primary deliverable, and link-host.sh can also be run standalone.
mkdir -p "${CVC_INSTALL_DIR}/share/pycvc-gl-wasm"
cp "${CVC_SOURCE_DIR}/bindings/pycvc/wasm/pycvc_host.cpp" "${CVC_INSTALL_DIR}/share/pycvc-gl-wasm/"
_NODE="$(command -v node 2>/dev/null || ls "${CVC_EMSDK_DIR}"/node/*/bin/node 2>/dev/null | head -1 || true)"
if DEPS="${CVC_DEPS_PREFIX}" INST="${CVC_INSTALL_DIR}" SRC="${CVC_SOURCE_DIR}" \
     EMSDK="${CVC_EMSDK_DIR}" OUT="${CVC_INSTALL_DIR}/share/pycvc-gl-wasm" NODE="${_NODE}" \
     bash "${CVC_SOURCE_DIR}/bindings/pycvc/wasm/link-host.sh"; then
    echo "pycvc-gl(wasm): host binary pycvc_host.wasm linked"
else
    echo "pycvc-gl(wasm): host link failed — archives installed; link-host.sh is standalone-runnable" >&2
fi
echo "pycvc-gl(wasm) build complete"
