#!/usr/bin/env bash
# recipes/_common/env-cosmo.sh — shared environment for cross-compilation
# to Cosmopolitan (Actually Portable Executables — APE).
#
# Sourced by build-cosmo.sh scripts.  Loads the host platform env first
# (for the native cmake), then points CC/CXX/AR/RANLIB at the cosmocc
# toolchain and overrides cvc_cmake_build to inject a suitable
# CMake toolchain hint.
#
# Downstream consumers link the resulting .a archives with the same
# cosmocc frontend to produce a single binary that runs on
# Linux (2.6.18+), macOS (23.1+), Windows (8+), FreeBSD (13+),
# OpenBSD (7.3+), and NetBSD (9.2+) — both x86_64 and aarch64 hosts.
set -euo pipefail

# Load the native host environment (cmake helpers, etc.).
_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_host="${CVC_HOST_PLATFORM:-linux}"
source "${_COMMON_DIR}/env-${_host}.sh"

: "${CVC_COSMOCC_DIR:?CVC_COSMOCC_DIR must point to the installed cosmocc toolchain}"

# Cosmopolitan is static-only: APE binaries link everything into one file.
BUILD_SHARED_LIBS=OFF
CVC_LINK=static
export CVC_LINK

# Put cosmocc tools on PATH for autotools recipes that shell out to
# things like `ar`, `ranlib`, `strip` unqualified.
export PATH="${CVC_COSMOCC_DIR}/bin:${PATH}"

# Point the standard compiler env vars at the x86_64 cosmocc frontend.
# (The unknown-unknown-cosmo-cc frontend produces fat binaries with
# both x86_64 and aarch64 code — desirable for the final consumer link,
# but for building intermediate .a archives the per-arch frontend is
# what most autotools projects expect.)  aarch64 bundles are a
# straightforward duplicate of this env with the frontend swapped.
export CC="${CVC_COSMOCC_DIR}/bin/x86_64-unknown-cosmo-cc"
export CXX="${CVC_COSMOCC_DIR}/bin/x86_64-unknown-cosmo-c++"
export AR="${CVC_COSMOCC_DIR}/bin/x86_64-linux-cosmo-ar"
export RANLIB="${CVC_COSMOCC_DIR}/bin/x86_64-linux-cosmo-ranlib"
export NM="${CVC_COSMOCC_DIR}/bin/x86_64-linux-cosmo-nm"
export STRIP="${CVC_COSMOCC_DIR}/bin/x86_64-linux-cosmo-strip"

# Locale + timestamp normalisation for reproducible-ish output.
export LC_ALL=C
export SOURCE_DATE_EPOCH=0

# Re-define cvc_cmake_build to drive CMake with the cosmo toolchain.
# Cosmopolitan ships no dedicated CMake toolchain file (yet), so we
# tell CMake the compilers via CC/CXX and disable shared libs.
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
        -DCMAKE_C_COMPILER="${CC}" \
        -DCMAKE_CXX_COMPILER="${CXX}" \
        -DCMAKE_AR="${AR}" \
        -DCMAKE_RANLIB="${RANLIB}" \
        -DCMAKE_SYSTEM_NAME=Linux \
        -DBUILD_SHARED_LIBS=OFF \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        ${_find_root_path_args[@]+"${_find_root_path_args[@]}"} \
        "$@"
    cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}"
    cmake --install "${CVC_BUILD_DIR}"
    cvc_rewrite_install_paths
}

echo "── env-cosmo.sh loaded (host=${_host}) ──"
echo "  COSMOCC=${CVC_COSMOCC_DIR}"
echo "  CC=${CC}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=static  JOBS=${CVC_JOBS}"
