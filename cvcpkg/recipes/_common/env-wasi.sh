#!/usr/bin/env bash
# recipes/_common/env-wasi.sh — shared environment for wasi cross-compilation.
#
# Sourced by build-wasi.sh scripts.  Loads the host platform env first
# (for the native cmake), then configures the wasi-sdk toolchain.
set -euo pipefail

# Load the native host environment (cmake helpers, etc.).
_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_host="${CVC_HOST_PLATFORM:-linux}"
source "${_COMMON_DIR}/env-${_host}.sh"

: "${CVC_WASI_SDK_DIR:?CVC_WASI_SDK_DIR must point to the installed wasi-sdk}"

# WASI builds are always static — no shared library support.
BUILD_SHARED_LIBS=OFF
CVC_LINK=static
export CVC_LINK

# Locate the wasi-sdk toolchain file.
_WASI_TOOLCHAIN="${CVC_WASI_SDK_DIR}/share/cmake/wasi-sdk.cmake"
if [[ ! -f "${_WASI_TOOLCHAIN}" ]]; then
    # Older wasi-sdk versions may not ship a CMake toolchain file.
    # Fall back to manual cross-compilation flags.
    _WASI_TOOLCHAIN=""
fi

_WASI_SYSROOT="${CVC_WASI_SDK_DIR}/share/wasi-sysroot"

# Export compiler environment variables.
export CC="${CVC_WASI_SDK_DIR}/bin/clang"
export CXX="${CVC_WASI_SDK_DIR}/bin/clang++"
export AR="${CVC_WASI_SDK_DIR}/bin/llvm-ar"
export RANLIB="${CVC_WASI_SDK_DIR}/bin/llvm-ranlib"
export NM="${CVC_WASI_SDK_DIR}/bin/llvm-nm"
export STRIP="${CVC_WASI_SDK_DIR}/bin/llvm-strip"

# Re-define cvc_cmake_build to inject the wasi-sdk toolchain.
cvc_cmake_build() {
    local _find_root_path_args=()
    if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
        _find_root_path_args+=(-DCMAKE_FIND_ROOT_PATH="${CVC_DEPS_PREFIX}")
    fi

    local _toolchain_args=()
    if [[ -n "${_WASI_TOOLCHAIN}" ]]; then
        _toolchain_args+=(-DCMAKE_TOOLCHAIN_FILE="${_WASI_TOOLCHAIN}")
    else
        _toolchain_args+=(
            -DCMAKE_SYSTEM_NAME=WASI
            -DCMAKE_SYSTEM_PROCESSOR=wasm32
            -DCMAKE_C_COMPILER="${CC}"
            -DCMAKE_CXX_COMPILER="${CXX}"
            -DCMAKE_AR="${AR}"
            -DCMAKE_RANLIB="${RANLIB}"
            -DCMAKE_SYSROOT="${_WASI_SYSROOT}"
            -DCMAKE_C_COMPILER_TARGET=wasm32-wasip1
            -DCMAKE_CXX_COMPILER_TARGET=wasm32-wasip1
        )
    fi

    cmake -G Ninja \
        -S "${CVC_SOURCE_DIR}" \
        -B "${CVC_BUILD_DIR}" \
        -DCMAKE_INSTALL_PREFIX="${CVC_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        "${_toolchain_args[@]}" \
        ${_find_root_path_args[@]+"${_find_root_path_args[@]}"} \
        "$@"
    cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}"
    cmake --install "${CVC_BUILD_DIR}"
    cvc_rewrite_install_paths
}

echo "── env-wasi.sh loaded (host=${_host}) ──"
echo "  WASI_SDK=${CVC_WASI_SDK_DIR}"
echo "  CC=${CC}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=static  JOBS=${CVC_JOBS}"
