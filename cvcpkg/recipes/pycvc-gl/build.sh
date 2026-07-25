#!/usr/bin/env bash
# cvcpkg/recipes/pycvc-gl/build.sh — build the FULL pycvc Python bindings
# (core cvc::geometry/volume numpy facades + the pycvc_gl scriptable scene over
# cvcGL + VTK) standalone against an installed libcvc SDK. CVC_BUILD_PYCVC_GL=ON
# ⇒ core + scene; the lean, VTK-free core-only build is the separate `pycvc`
# recipe (which this package `provides:`).
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"   # libcvc repo root (vendored)
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

CVC_BUILD_TYPE="${CVC_BUILD_TYPE:-Release}"
CVC_JOBS="${CVC_JOBS:-$(nproc 2>/dev/null || echo 4)}"

case "$(echo "$CVC_BUILD_TYPE" | tr '[:upper:]' '[:lower:]')" in
  debug) CMAKE_BUILD_TYPE=Debug ;;
  *) CMAKE_BUILD_TYPE=Release ;;
esac

CMAKE_ARGS=(
  -G Ninja
  -S "$CVC_SOURCE_DIR/bindings/pycvc"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  # Scene module ONLY (cvcGL + VTK). The core pycvc module comes from the
  # pycvc package (a runtime dependency), so this package installs just
  # _pycvc_gl + pycvc_gl.py — pycvc and pycvc-gl live side by side.
  -DCVC_BUILD_PYCVC_CORE=OFF
  -DCVC_BUILD_PYCVC_GL=ON
)

# The deps prefix carries the installed libcvc SDK (find_package(cvc CONFIG)),
# python311, numpy, and swig.
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
  # Pin the interpreter + its numpy headers so find_package(Python3 ... NumPy)
  # doesn't fall through to a system python without numpy — which fails configure
  # with "Could NOT find Python3 (missing: Python3_NumPy_INCLUDE_DIRS NumPy)".
  # Derive the include dir from the interpreter (numpy>=2 -> _core/include,
  # <2 -> core/include) rather than hardcoding it.
  if [[ -x "$CVC_DEPS_PREFIX/bin/python3.11" ]]; then
    CMAKE_ARGS+=("-DPython3_EXECUTABLE=$CVC_DEPS_PREFIX/bin/python3.11")
    _npy_inc=$("$CVC_DEPS_PREFIX/bin/python3.11" -c 'import numpy; print(numpy.get_include())' 2>/dev/null || true)
    [[ -n "$_npy_inc" ]] && CMAKE_ARGS+=("-DPython3_NumPy_INCLUDE_DIRS=$_npy_inc")
  fi
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"
cmake --install "$CVC_BUILD_DIR"
