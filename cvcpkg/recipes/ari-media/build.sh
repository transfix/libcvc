#!/usr/bin/env bash
# cvcpkg/recipes/ari-media/build.sh — build ari-media (ari with cvcMedia's SDL3 audio) standalone
# against an installed libcvc SDK and the sdl3 package. Compiles src/ari and src/cvcMedia (static,
# linked into the binary), so libcvc is NOT rebuilt. Installs just bin/ari-media.
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
  -S "$CVC_SOURCE_DIR/src/ari"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  -DARI_WITH_MEDIA=ON
)

# The deps prefix carries the installed libcvc SDK and SDL3 (find_package(cvc), find_package(SDL3)).
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS" --target ari-media
cmake --install "$CVC_BUILD_DIR" --component ari
# The ari component holds bin/ari too; this package is bin/ari-media alone.
rm -f "$CVC_INSTALL_DIR/bin/ari" "$CVC_INSTALL_DIR/bin/ari.exe"
