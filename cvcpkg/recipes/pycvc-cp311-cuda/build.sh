#!/usr/bin/env bash
# cvcpkg/recipes/pycvc-cp311-cuda/build.sh — build the core pycvc Python
# bindings (cvc::geometry/volume facades + zero-copy numpy views) standalone
# against an installed libcvc SDK (the CUDA one — the deps prefix carries
# libcvc-cuda). CVC_BUILD_PYCVC_GL=OFF ⇒ no cvcGL/VTK dependency; the
# scriptable-scene bindings are the separate `pycvc-gl-cp311-cuda` recipe.
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
  # Core numpy-bindings module only (no VTK). The scene module ships as the
  # separate, side-by-side pycvc-gl-cp311-cuda package.
  -DCVC_BUILD_PYCVC_CORE=ON
  -DCVC_BUILD_PYCVC_GL=OFF
)

# The deps prefix carries the installed libcvc SDK (find_package(cvc CONFIG)),
# python311, numpy, and swig.
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"
cmake --install "$CVC_BUILD_DIR"
