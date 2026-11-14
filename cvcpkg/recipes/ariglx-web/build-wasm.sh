#!/usr/bin/env bash
# cvcpkg/recipes/ariglx-web/build-wasm.sh — ariglx-web, arigl-web with an embedded Python
# (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §5), packaged under share/ariglx-web/: index.html
# (arigl-web's page, told to start ariglx), arigl-web.js and arigl-web.css, ariglx.js and
# ariglx.wasm, python312.zip, and the notices.
#
# arigl-web's closure compiles from the repo root (its build-wasm.sh says why), here with the pycvc
# bindings on, and only the ariglx target is built: src/arigl/main.cpp with the engine
# (src/ariadne_python), libpython3.12.a, the pycvc / pycvc_gl SWIG archives and numpy's extension
# objects linked in (src/ariadne_python/ariadne_python.cmake, cvc_ariadne_python_wasm_host). Deps
# come static from CVC_DEPS_PREFIX; emsdk from the builder via CVC_EMSDK_DIR.
#
# The tier follows the prefix, as roadmap §5.2 names them:
#   lean  no vtk-python: pycvc_gl without the VTK-Python bridge (the default)
#   full  vtk-python-cp312 for wasm installed in CVC_DEPS_PREFIX: the bridge, its
#         _vtkmodules_static builtin, and its _vtk.zip merged into python312.zip
# CVC_ARIGLX_WEB_TIER=lean|full says which instead (full fails without vtk-python).
#
#   CVC_PLATFORM=wasm     single-threaded: any static host, no special headers
#   CVC_PLATFORM=wasm-mt  threaded (-pthread), as arigl-web's wasm-mt is (roadmap §5.4): the page
#                         must be cross-origin isolated (COOP/COEP), Python's threads are real,
#                         and there is no conference client
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_DEPS_PREFIX:?CVC_DEPS_PREFIX must point at the wasm deps prefix}"
: "${CVC_EMSDK_DIR:?CVC_EMSDK_DIR must point to the activated emsdk bundle}"
: "${CVC_PLATFORM:?CVC_PLATFORM must be wasm or wasm-mt}"
case "$CVC_PLATFORM" in
  wasm) PTHREADS=OFF ;;
  wasm-mt) PTHREADS=ON ;;
  *) echo "ariglx-web: CVC_PLATFORM is wasm or wasm-mt, not '${CVC_PLATFORM}'" >&2; exit 1 ;;
esac

PYINC="$CVC_DEPS_PREFIX/include/python3.12"
PYLIB="$CVC_DEPS_PREFIX/lib/libpython3.12.a"
NUMPY_INC="$CVC_DEPS_PREFIX/lib/python3.12/site-packages/numpy/_core/include"
for f in "$PYINC/Python.h" "$PYLIB" "$NUMPY_INC/numpy/arrayobject.h"; do
  [[ -f "$f" ]] || { echo "ariglx-web: $f is missing: install python312 and numpy-cp312 for wasm in $CVC_DEPS_PREFIX" >&2; exit 1; }
done

TIER="${CVC_ARIGLX_WEB_TIER:-}"
if [[ -z "$TIER" ]]; then
  TIER=lean
  [[ -f "$CVC_DEPS_PREFIX/lib/_vtkmodules_static.a" ]] && TIER=full
fi
case "$TIER" in
  lean) BRIDGE=OFF ;;
  full) BRIDGE=ON
        [[ -f "$CVC_DEPS_PREFIX/lib/_vtkmodules_static.a" ]] || {
          echo "ariglx-web: the full tier needs vtk-python-cp312 for wasm in $CVC_DEPS_PREFIX" >&2
          exit 1
        } ;;
  *) echo "ariglx-web: CVC_ARIGLX_WEB_TIER is lean or full, not '$TIER'" >&2; exit 1 ;;
esac
echo "ariglx-web: the $TIER tier (VTK-Python bridge $BRIDGE), $CVC_PLATFORM (pthreads $PTHREADS)"

# Native host tools: SWIG generates the wrappers, and a native Python runs it, the extension
# manifests (cvcpkg/recipes/_common/wasm_pyext.py) and the zip, as pycvc-gl-cp312's wasm build
# provisions them. The packaged SWIG reports a stale -swiglib after relocation, so its library
# directory is named outright.
HOSTENV="$CVC_BUILD_DIR/hostenv"
_cvc="cvcpkg"; command -v cvcpkg >/dev/null 2>&1 || _cvc="python3 -m cvcpkg"
${_cvc} install python312 swig --platform linux --config release --link shared \
  --prefix "$HOSTENV" --no-fallback-to-source >&2
PY_NATIVE="$HOSTENV/bin/python3.12"
SWIG_LIBDIR="$(find "$HOSTENV/share/swig" -mindepth 1 -maxdepth 1 -type d 2>/dev/null | head -1)"
[[ -x "$PY_NATIVE" && -n "$SWIG_LIBDIR" ]] || {
  echo "ariglx-web: no native python3.12 / swig library in $HOSTENV" >&2
  exit 1
}
export SWIG_LIB="$SWIG_LIBDIR"

# The WebSocket transport (an Ariadne conference client), as arigl-web builds it: single-threaded
# only (libopus has no -pthread wasm build), when the prefix has protobuf's wasm build, with a
# native protoc of the same version on PATH.
WS=OFF
if [[ "$PTHREADS" == "OFF" ]] &&
   [[ -d "$CVC_DEPS_PREFIX/lib/cmake/protobuf" || -f "$CVC_DEPS_PREFIX/lib/libprotobuf.a" ]]; then
  WS=ON
  command -v protoc >/dev/null || {
    echo "ariglx-web: the WebSocket transport needs a native protoc on PATH (protobuf, the recipe's host tool)" >&2
    exit 1
  }
fi

# shellcheck disable=SC1091
source "${CVC_EMSDK_DIR}/emsdk_env.sh"

# Config-mode Boost (CMP0167=NEW) straight from the prefix, as pycvc-gl-cp312's wasm build finds
# it for the bindings.
BOOST_DIR="$(find "$CVC_DEPS_PREFIX/lib/cmake" -maxdepth 1 -type d -name 'Boost-*' 2>/dev/null | head -1)"

mkdir -p "$CVC_BUILD_DIR"
emcmake cmake -G Ninja -S "$CVC_SOURCE_DIR" -B "$CVC_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_ROOT_PATH="$CVC_DEPS_PREFIX" \
    -DCMAKE_PREFIX_PATH="$CVC_DEPS_PREFIX" \
    -DCMAKE_POLICY_DEFAULT_CMP0167=NEW \
    -DBoost_DIR="$BOOST_DIR" \
    -DCVC_ENABLE_CUDA=OFF \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_BUILD_CLI=OFF \
    -DCVC_BUILD_ARI=OFF \
    -DCVC_BUILD_ARIGL=ON \
    -DCVC_ENABLE_OPENMP=OFF \
    -DDISABLE_CGAL=ON \
    -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF \
    -DCVC_ENABLE_IMAGEMAGICK=ON \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_FFT_PROVIDER=none \
    -DCVC_ENABLE_ASSIMP=ON \
    -DCVC_ENABLE_MESHER=ON \
    -DCVC_ENABLE_SDF=ON \
    -DCVC_ENABLE_LIBIGL=ON \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=OFF \
    -DCVC_WASM_PTHREADS="$PTHREADS" \
    -DCVC_ENABLE_WS="$WS" \
    -DCVC_ENABLE_GRPC=OFF \
    -DCVC_BUILD_PYCVC=ON \
    -DCVC_BUILD_PYCVC_CORE=ON \
    -DCVC_BUILD_PYCVC_GL=ON \
    -DCVC_PYCVCGL_VTK_BRIDGE="$BRIDGE" \
    -DPython3_EXECUTABLE="$PY_NATIVE" \
    -DPython3_INCLUDE_DIR="$PYINC" \
    -DPython3_LIBRARY="$PYLIB" \
    -DPython3_NumPy_INCLUDE_DIR="$NUMPY_INC" \
    -DPython3_NumPy_INCLUDE_DIRS="$NUMPY_INC" \
    -DSWIG_EXECUTABLE="$HOSTENV/bin/swig" \
    -DSWIG_DIR="$SWIG_LIBDIR" \
    | tee "$CVC_BUILD_DIR/configure.log"

# What a configure that missed a dependency would quietly turn off (arigl-web's list), and the
# embedded Python itself.
WANT=("Ariadne: .ari YAML loader ENABLED"
      "cvcGL: Dear ImGui overlay ENABLED"
      "cvcGL: SDL3 input/peripheral seam ENABLED"
      "ariglx: embedded Python 3.12")
if [[ "$WS" == "ON" ]]; then
  WANT+=("cvc::state: WebSocket transport ENABLED"
         "cvc::physics Jolt backend ENABLED"
         "cvc::media: Opus mics with"
         "cvc::state: device keys ENABLED")
fi
for want in "${WANT[@]}"; do
  grep -qF "$want" "$CVC_BUILD_DIR/configure.log" \
    || { echo "ariglx-web: the configure did not report '$want' (deps in $CVC_DEPS_PREFIX)" >&2; exit 1; }
done
grep -q '^CVC_IMAGEMAGICK_ENABLED:INTERNAL=ON' "$CVC_BUILD_DIR/CMakeCache.txt" || {
  echo "ariglx-web: the configure found no ImageMagick in $CVC_DEPS_PREFIX" >&2
  exit 1
}

cmake --build "$CVC_BUILD_DIR" --target ariglx -j "${CVC_JOBS:-$(nproc)}"

BIN="$CVC_BUILD_DIR/bin"
WEB="$CVC_INSTALL_DIR/share/ariglx-web"
install -d "$WEB"
install -m 644 "$BIN/ariglx.js" "$BIN/ariglx.wasm" "$BIN/python312.zip" \
  "$BIN/arigl-web.js" "$BIN/arigl-web.css" "$WEB/"
install -m 644 "$BIN/ariglx.html" "$WEB/index.html"
install -m 644 "$CVC_SOURCE_DIR/THIRD_PARTY_NOTICES.md" "$CVC_SOURCE_DIR/LICENSE" "$WEB/"

# What a page downloads, raw and compressed as a server would send it (arigl-web's measure).
brotli_size() {
  if command -v brotli >/dev/null 2>&1; then
    brotli -q 11 -c "$1" | wc -c
  elif [[ -n "${EMSDK_NODE:-}" ]]; then
    "$EMSDK_NODE" -e 'const z = require("zlib");
      const b = require("fs").readFileSync(process.argv[1]);
      console.log(z.brotliCompressSync(b, {params: {[z.constants.BROTLI_PARAM_QUALITY]: 11,
        [z.constants.BROTLI_PARAM_SIZE_HINT]: b.length}}).length)' "$1"
  else
    echo -
  fi
}
echo "ariglx-web ($TIER, $CVC_PLATFORM) sizes, bytes: raw / gzip -9 / brotli -q 11"
for f in ariglx.wasm ariglx.js python312.zip index.html arigl-web.js arigl-web.css; do
  printf '  %-13s %10s %10s %10s\n' "$f" "$(wc -c < "$WEB/$f")" \
    "$(gzip -9 -c "$WEB/$f" | wc -c)" "$(brotli_size "$WEB/$f")"
done
