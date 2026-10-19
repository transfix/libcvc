#!/usr/bin/env bash
# cvcpkg/recipes/cvcgl/build.sh — build the cvcGL scene-graph library standalone
# against an installed libcvc SDK + VTK. Installs libcvcGL (shared libcvcGL.so for
# --link shared, the libcvcGL.a archive for --link static), the cvc/gl headers,
# and a cvcGLConfig.cmake so consumers can find_package(cvcGL CONFIG) and link
# cvc::cvcGL (e.g. the pycvc-gl recipe).
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

# cvcGL's library type follows BUILD_SHARED_LIBS (src/cvcGL/CMakeLists.txt), and
# CMake reads that only as a cache variable, so map the bundle's link mode onto it
# here, as _common/env-windows.ps1 does for build.ps1. Without it every bundle,
# --link shared included, carried the static archive.
case "${CVC_LINK:-shared}" in
  static) BUILD_SHARED_LIBS=OFF ;;
  *) BUILD_SHARED_LIBS=ON ;;
esac

CMAKE_ARGS=(
  -G Ninja
  -S "$CVC_SOURCE_DIR/src/cvcGL"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  -DBUILD_SHARED_LIBS="$BUILD_SHARED_LIBS"
)

# The deps prefix carries the installed libcvc SDK (find_package(cvc CONFIG))
# and VTK.
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"
cmake --install "$CVC_BUILD_DIR"
