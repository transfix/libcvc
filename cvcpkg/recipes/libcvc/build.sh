#!/usr/bin/env bash
# cvcpkg/recipes/libcvc/build.sh - build libcvc from the in-repo source tree.
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

CVC_BUILD_TYPE="${CVC_BUILD_TYPE:-Release}"
CVC_LINK="${CVC_LINK:-shared}"
CVC_JOBS="${CVC_JOBS:-$(nproc 2>/dev/null || echo 4)}"

case "$(echo "$CVC_BUILD_TYPE" | tr '[:upper:]' '[:lower:]')" in
  debug) CMAKE_BUILD_TYPE=Debug ;;
  *) CMAKE_BUILD_TYPE=Release ;;
esac

if [[ "$CVC_LINK" == "static" ]]; then
  BUILD_SHARED_LIBS=OFF
else
  BUILD_SHARED_LIBS=ON
fi

CMAKE_ARGS=(
  -G Ninja
  -S "$CVC_SOURCE_DIR"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  -DBUILD_SHARED_LIBS="$BUILD_SHARED_LIBS"
  -DCVC_BUILD_TESTS=OFF
  # The `cvc` CLI ships as its own package (the cvc-cli / cvc-cli-cuda recipes)
  # so it lands deliberately in a user's PATH instead of riding along with the
  # SDK bundle. Keep it OUT of the libcvc bundle.
  -DCVC_BUILD_CLI=OFF
  # CUDA is opt-in per recipe: the `libcvc` recipe leaves this OFF (portable
  # CPU build); the sibling `libcvc-cuda` recipe sets CVC_ENABLE_CUDA=ON in its
  # build.matrix env so this same script produces the GPU variant.
  -DCVC_ENABLE_CUDA="${CVC_ENABLE_CUDA:-OFF}"
  -DCVC_ENABLE_GRPC=OFF
  # libcvc is a pure C++ SDK: the Python bindings are a SEPARATE `pycvc`
  # recipe that builds bindings/pycvc against the installed libcvc, so a plain
  # libcvc bundle pulls in no Python/numpy/swig/VTK dependency. (Default is
  # already OFF; kept explicit.)
  -DCVC_BUILD_PYCVC=OFF
  # Enable the XMLRPC module so the bundle exports the cvc::xmlrpc target
  # (and ships libxmlrpc). cvc::state's network sharing and downstream
  # consumers such as VolumeRover 2.x link cvc::xmlrpc directly; it is a
  # self-contained STATIC + PIC library with no extra third-party deps.
  -DCVC_USING_XMLRPC=ON
)

if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"
cmake --install "$CVC_BUILD_DIR"
