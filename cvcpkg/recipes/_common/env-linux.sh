#!/usr/bin/env bash
# recipes/_common/env-linux.sh — shared environment for Linux recipe builds.
#
# Sourced by every build.sh on Linux.  Sets compiler flags, paths,
# and helper functions that all recipes share.
set -euo pipefail

# Relocatability helper: rewrite absolute install-dir paths in .pc/.cmake.
_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_COMMON_DIR}/rewrite-install-paths.sh"

: "${CVC_BUILD_TYPE:=Release}"
: "${CVC_LINK:=shared}"
: "${CVC_JOBS:=$(nproc 2>/dev/null || echo 4)}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"

export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"

# Build-type → CMake flags mapping.
_build_type_lc=$(echo "$CVC_BUILD_TYPE" | tr '[:upper:]' '[:lower:]')
case "$_build_type_lc" in
    release) CMAKE_BUILD_TYPE=Release  ;;
    debug)   CMAKE_BUILD_TYPE=Debug    ;;
    *)       CMAKE_BUILD_TYPE=Release  ;;
esac

# Shared/static → CMake flags.
if [[ "${CVC_LINK}" == "static" ]]; then
    BUILD_SHARED_LIBS=OFF
else
    BUILD_SHARED_LIBS=ON
fi

# Assemble CMAKE_PREFIX_PATH from the two dependency roots:
#   CVC_DEPS_PREFIX  — the runtime closure (install prefix; these ship)
#   CVC_BUILD_PREFIX — the build closure (build prefix; stripped on install)
# Both must be searchable at build time; only the former is part of the
# deliverable.  CVC_BUILD_PREFIX is unset/equal for legacy single-prefix
# layouts, in which case this collapses to the old behaviour.
_cvc_prefix_path=""
for _cvc_root in "${CVC_DEPS_PREFIX:-}" "${CVC_BUILD_PREFIX:-}"; do
    [[ -n "${_cvc_root}" ]] || continue
    case ":${_cvc_prefix_path}:" in
        *":${_cvc_root}:"*) continue ;;   # already present
    esac
    _cvc_prefix_path="${_cvc_prefix_path:+${_cvc_prefix_path};}${_cvc_root}"
done
if [[ -n "${_cvc_prefix_path}" ]]; then
    export CMAKE_PREFIX_PATH="${_cvc_prefix_path}"
fi
unset _cvc_prefix_path _cvc_root

# Helper: run cmake configure + build + install in one call.
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
        -DCMAKE_INSTALL_RPATH="\$ORIGIN;\$ORIGIN/../lib" \
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
        "$@"
    cmake --build "${CVC_BUILD_DIR}" -j "${CVC_JOBS}"
    cmake --install "${CVC_BUILD_DIR}"
    cvc_rewrite_install_paths
}

echo "── env-linux.sh loaded ──"
echo "  CC=${CC}  CXX=${CXX}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=${CVC_LINK}  JOBS=${CVC_JOBS}"
