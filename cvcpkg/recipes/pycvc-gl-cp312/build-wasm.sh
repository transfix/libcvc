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

# ── (3) configure the trimmed closure from the repo root (static, BRIDGE=ON) ──
# Same OFF set as cvcgl-examples/build-wasm.sh (the wasm-linkable subset) PLUS the
# pycvc bindings: CVC_BUILD_PYCVC=ON builds bindings/pycvc (core + gl) in-tree.
emcmake cmake -G Ninja \
    -S "${CVC_SOURCE_DIR}" \
    -B "${CVC_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DCMAKE_FIND_ROOT_PATH="${CVC_DEPS_PREFIX}" \
    -DBUILD_SHARED_LIBS=OFF \
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

# ── (4) build the static SWIG archives (+ the cvc/cvcGL closure they need) ──
cmake --build "${CVC_BUILD_DIR}" --target pycvc pycvc_gl -j "${CVC_JOBS}"

_LIBPYCVC="$(find "${CVC_BUILD_DIR}" -name 'libpycvc.a' -o -name 'pycvc.a' 2>/dev/null | head -1)"
_LIBPYCVCGL="$(find "${CVC_BUILD_DIR}" -name 'libpycvc_gl.a' -o -name 'pycvc_gl.a' 2>/dev/null | head -1)"
[ -n "${_LIBPYCVC}" ] && [ -n "${_LIBPYCVCGL}" ] || {
    echo "pycvc-gl(wasm): FATAL — static archives not produced" >&2
    find "${CVC_BUILD_DIR}" -name '*pycvc*.a' >&2 2>/dev/null; exit 1; }
echo "pycvc-gl(wasm): built ${_LIBPYCVC} + ${_LIBPYCVCGL}"

# ── (5) stage the archives, the .py proxies, and the host source ──────────────
# The single-.wasm host LINK (force-load the pycvc archives + libvtkWrappingPythonCore
# + the vtk-python _load()) is the next step; determining the vtk-python static
# _load() symbol needs this build's vtk-python artifact. Stage everything a host
# link needs so it can be driven here or by a downstream `bake`.
DEST="${CVC_INSTALL_DIR}/lib/python3.12/site-packages"
mkdir -p "${DEST}/pycvc_gl" "${CVC_INSTALL_DIR}/lib" "${CVC_INSTALL_DIR}/share/pycvc-gl-wasm"
cp "${_LIBPYCVC}" "${_LIBPYCVCGL}" "${CVC_INSTALL_DIR}/lib/"
cp "${CVC_BUILD_DIR}/bindings/pycvc/pycvc.py" "${DEST}/" 2>/dev/null || \
  find "${CVC_BUILD_DIR}" -name pycvc.py -path '*bindings*' -exec cp {} "${DEST}/" \;
cp "${CVC_BUILD_DIR}/bindings/pycvc/pycvc_gl.py" "${DEST}/pycvc_gl/__init__.py" 2>/dev/null || \
  find "${CVC_BUILD_DIR}" -name pycvc_gl.py -path '*bindings*' -exec cp {} "${DEST}/pycvc_gl/__init__.py" \;
cp -r "${CVC_SOURCE_DIR}/bindings/pycvc/pymod_gl/." "${DEST}/pycvc_gl/" 2>/dev/null || true
cp "${CVC_SOURCE_DIR}/bindings/pycvc/wasm/pycvc_host.cpp" "${CVC_INSTALL_DIR}/share/pycvc-gl-wasm/"
echo "pycvc-gl(wasm) build complete (static archives + proxies + host source staged)"
