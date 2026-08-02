#!/usr/bin/env bash
# cvcpkg/recipes/pycvc-cp31X/build.sh — build the core pycvc Python bindings
# (cvc::geometry/volume facades + zero-copy numpy views) standalone against an
# installed libcvc SDK. CVC_BUILD_PYCVC_GL=OFF ⇒ no cvcGL/VTK dependency; the
# scriptable-scene bindings are the separate `pycvc-gl-cp31X` recipes.
#
# One script serves every interpreter column: the cvcpkg builder exports
# CVC_PYTHON_INTERPRETER (e.g. python312) from the recipe's `python:` block,
# and the python3.X paths are derived from it below (3.11 fallback). Each
# pycvc-cp31X recipe dir carries an identical copy of this script.
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"   # libcvc repo root (vendored)
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

CVC_BUILD_TYPE="${CVC_BUILD_TYPE:-Release}"
CVC_JOBS="${CVC_JOBS:-$(nproc 2>/dev/null || echo 4)}"

# Dotted python version for bin/python3.X + lib/python3.X paths, derived from
# the recipe's interpreter column (python311 -> 3.11, python312 -> 3.12, ...).
# A trailing free-threaded `t` (python313t) is stripped defensively — no `t`
# columns exist for pycvc (SWIG modules are not free-threaded-safe).
_pyinterp="${CVC_PYTHON_INTERPRETER:-python311}"
_pydigits="${_pyinterp#python}"
_pydigits="${_pydigits%t}"
PYVER="${_pydigits:0:1}.${_pydigits:1}"

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
  # Core numpy-bindings module only (no VTK). The scene module ships as the
  # separate, side-by-side pycvc-gl-cp31X package.
  -DCVC_BUILD_PYCVC_CORE=ON
  -DCVC_BUILD_PYCVC_GL=OFF
)

# The deps prefix carries the installed libcvc SDK (find_package(cvc CONFIG)),
# this column's python31X interpreter, its numpy, and swig.
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
  # Pin the interpreter + its numpy headers so find_package(Python3 ... NumPy)
  # doesn't fall through to a system python without numpy — which fails configure
  # with "Could NOT find Python3 (missing: Python3_NumPy_INCLUDE_DIRS NumPy)".
  # Derive the include dir from the interpreter (numpy>=2 -> _core/include,
  # <2 -> core/include) rather than hardcoding it.
  if [[ -x "$CVC_DEPS_PREFIX/bin/python$PYVER" ]]; then
    CMAKE_ARGS+=("-DPython3_EXECUTABLE=$CVC_DEPS_PREFIX/bin/python$PYVER")
    _npy_inc=$("$CVC_DEPS_PREFIX/bin/python$PYVER" -c 'import numpy; print(numpy.get_include())' 2>/dev/null || true)
    [[ -n "$_npy_inc" ]] && CMAKE_ARGS+=("-DPython3_NumPy_INCLUDE_DIRS=$_npy_inc")
  fi
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"
cmake --install "$CVC_BUILD_DIR"
