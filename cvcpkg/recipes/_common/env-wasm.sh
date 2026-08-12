#!/usr/bin/env bash
# recipes/_common/env-wasm.sh — shared environment for wasm cross-compilation.
#
# Sourced by build-wasm.sh scripts.  Loads the host platform env first
# (for the native compiler/cmake), then activates Emscripten and
# overrides the cmake helper to use the Emscripten toolchain file.
set -euo pipefail

# Load the native host environment (compiler, cmake helpers, etc.).
_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_host="${CVC_HOST_PLATFORM:-linux}"
source "${_COMMON_DIR}/env-${_host}.sh"

: "${CVC_EMSDK_DIR:?CVC_EMSDK_DIR must point to the activated emsdk bundle}"

# Activate Emscripten.
# shellcheck disable=SC1091
source "${CVC_EMSDK_DIR}/emsdk_env.sh"

# Wasm builds are always static.
BUILD_SHARED_LIBS=OFF
CVC_LINK=static
export CVC_LINK

# Re-define cvc_cmake_build to inject the Emscripten toolchain file.
cvc_cmake_build() {
    local _find_root_path_args=()
    if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
        _find_root_path_args+=(-DCMAKE_FIND_ROOT_PATH="${CVC_DEPS_PREFIX}")
    fi
    cmake -G Ninja \
        -S "${CVC_SOURCE_DIR}" \
        -B "${CVC_BUILD_DIR}" \
        -DCMAKE_INSTALL_PREFIX="${CVC_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_TOOLCHAIN_FILE="${EMSDK}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" \
        ${_find_root_path_args[@]+"${_find_root_path_args[@]}"} \
        "$@"
    cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}"
    cmake --install "${CVC_BUILD_DIR}"
    cvc_rewrite_install_paths
}

echo "── env-wasm.sh loaded (host=${_host}) ──"
echo "  EMSDK=${EMSDK}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=static  JOBS=${CVC_JOBS}"
