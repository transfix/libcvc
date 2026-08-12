#!/usr/bin/env bash
# recipes/_common/env-netbsd.sh — shared environment for NetBSD recipe builds.
set -euo pipefail

_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_COMMON_DIR}/rewrite-install-paths.sh"

: "${CVC_BUILD_TYPE:=Release}"
: "${CVC_LINK:=shared}"
: "${CVC_JOBS:=$(sysctl -n hw.ncpu 2>/dev/null || echo 4)}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"

# NetBSD base ships GCC; clang is available via pkgsrc.
# Prefer clang if available, fall back to GCC.
if command -v clang >/dev/null 2>&1; then
    export CC="${CC:-clang}"
    export CXX="${CXX:-clang++}"
else
    export CC="${CC:-gcc}"
    export CXX="${CXX:-g++}"
fi

_build_type_lc=$(echo "$CVC_BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
case "$_build_type_lc" in
    release) CMAKE_BUILD_TYPE=Release  ;;
    debug)   CMAKE_BUILD_TYPE=Debug    ;;
    *)       CMAKE_BUILD_TYPE=Release  ;;
esac

if [[ "${CVC_LINK}" == "static" ]]; then
    BUILD_SHARED_LIBS=OFF
else
    BUILD_SHARED_LIBS=ON
fi

if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
    export CMAKE_PREFIX_PATH="${CVC_DEPS_PREFIX}"
fi

# NetBSD pkgsrc installs under /usr/pkg.
export CFLAGS="${CFLAGS:-} -I/usr/pkg/include"
export CXXFLAGS="${CXXFLAGS:-} -I/usr/pkg/include"
export LDFLAGS="${LDFLAGS:-} -L/usr/pkg/lib -Wl,-R/usr/pkg/lib"

# Ensure pkgsrc tools are on PATH, but AFTER our own deps prefix so that
# cvcpkg-built host tools (ninja, cmake, ...) win over pkgsrc versions.
# The pkgsrc ninja on NetBSD has been observed to crash when invoked via
# subprocess with an empty environment, which broke every cmake+ninja recipe.
export PATH="${PATH}:/usr/pkg/bin:/usr/pkg/sbin"

# Prefer our own ninja from the deps prefix when present.
_CVC_NINJA=""
if [[ -n "${CVC_DEPS_PREFIX:-}" && -x "${CVC_DEPS_PREFIX}/bin/ninja" ]]; then
    _CVC_NINJA="${CVC_DEPS_PREFIX}/bin/ninja"
fi

cvc_cmake_build() {
    cmake -G Ninja \
        -S "${CVC_SOURCE_DIR}" \
        -B "${CVC_BUILD_DIR}" \
        -DCMAKE_INSTALL_PREFIX="${CVC_INSTALL_DIR}" \
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
        -DBUILD_SHARED_LIBS="${BUILD_SHARED_LIBS}" \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_CXX_STANDARD=17 \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DCMAKE_INSTALL_RPATH=\$ORIGIN \
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
        ${_CVC_NINJA:+-DCMAKE_MAKE_PROGRAM="${_CVC_NINJA}"} \
        "$@"
    cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}"
    cmake --install "${CVC_BUILD_DIR}"
    cvc_rewrite_install_paths
}

echo "── env-netbsd.sh loaded ──"
echo "  CC=${CC}  CXX=${CXX}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=${CVC_LINK}  JOBS=${CVC_JOBS}"
