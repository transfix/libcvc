#!/usr/bin/env bash
# cvcpkg/recipes/arisrvx/build.sh — build `arisrvx`, the arisrv relay with an embedded Python,
# standalone against an installed libcvc SDK and the deps prefix's python312. Compiles src/arisrv
# with -DARISRV_WITH_PYTHON=ON (main.cpp plus src/ariadne_python's engine and its server part) and
# links the pre-installed cvc::cvc and libpython, so libcvc is NOT rebuilt. Installs just
# bin/arisrvx: arisrv, configured beside it, is not built.
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
  -S "$CVC_SOURCE_DIR/src/arisrv"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  -DARISRV_WITH_PYTHON=ON
)

# The deps prefix carries the installed libcvc SDK (find_package(cvc CONFIG)) and python312. It may
# be several prefixes, ;-separated. The interpreter is named outright: FindPython3 left to itself
# would take a system python3 first, and arisrvx must embed the one it runs beside.
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
  IFS=';' read -r -a _prefixes <<<"$CVC_DEPS_PREFIX"
  for _p in "${_prefixes[@]}"; do
    if [[ -x "$_p/bin/python3.12" ]]; then
      CMAKE_ARGS+=("-DPython3_EXECUTABLE=$_p/bin/python3.12")
      break
    fi
  done
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS" --target arisrvx
cmake --install "$CVC_BUILD_DIR" --component arisrvx
