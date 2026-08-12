#!/usr/bin/env bash
# recipes/_common/env-haiku.sh — shared environment for Haiku recipe builds.
#
# UNVERIFIED-ON-HARDWARE: every line below marked "[unverified]" is written
# from Haiku R1/beta5 documentation and the HaikuPorts layout, but has NOT yet
# been executed on a live Haiku host — the cluster's haiku-build VM is still
# blank (Haiku has no getty and no virtio-console, so a stock ISO install needs
# graphical interaction; the pre-installed image recipe is haiku-image).  The
# unmarked lines mirror env-freebsd.sh / env-netbsd.sh verbatim and carry the
# same confidence as those.  Fix marked lines first when a build misbehaves.
set -euo pipefail

_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${_COMMON_DIR}/rewrite-install-paths.sh"

: "${CVC_BUILD_TYPE:=Release}"
: "${CVC_LINK:=shared}"

# [unverified] Haiku has no sysctl(8), so the one-liner the BSD env files use
# does not port.  `nproc` comes from HaikuPorts coreutils and `sysinfo -cpu` is
# in the base system (one "CPU #<n>" line per core).  This is a function rather
# than an `a || b || echo 4` chain because `grep -c` PRINTS "0" *and* exits 1
# when it matches nothing — in a chain that concatenates "0" and the fallback
# into a two-line CVC_JOBS, which reaches `make -j` as garbage.  The result is
# validated against a positive-integer pattern for the same reason.
_cvc_haiku_ncpu() {
    local n
    n=$(nproc 2>/dev/null) || n=""
    if [[ ! "$n" =~ ^[1-9][0-9]*$ ]]; then
        n=$(sysinfo -cpu 2>/dev/null | grep -c '^CPU #') || n=""
    fi
    [[ "$n" =~ ^[1-9][0-9]*$ ]] || n=4
    printf '%s' "$n"
}
: "${CVC_JOBS:=$(_cvc_haiku_ncpu)}"

: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"

# Haiku's system compiler on x86_64 is GCC (13.3.0 in R1/beta5); clang exists
# in HaikuPorts but is not the platform toolchain and is not guaranteed
# present.  Unlike NetBSD we therefore do NOT prefer clang when it happens to
# be installed: Haiku's own headers and libroot are built and tested against
# the base GCC, and mixing the two produces C++ ABI mismatches against
# HaikuPorts' libstdc++.
export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"

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

# Haiku's filesystem hierarchy is neither FHS nor pkgsrc: there is no
# /usr/local and no /usr/pkg.  HaikuPorts packages are mounted read-only under
# /boot/system by packagefs — headers at develop/headers, link-time libraries
# at develop/lib, run-time libraries at lib.  /boot/system/non-packaged is the
# writable escape hatch for anything installed outside a .hpkg, which is where
# a hand-installed build tool ends up.
_HAIKU_SYSTEM="/boot/system"
_HAIKU_NONPKG="${_HAIKU_SYSTEM}/non-packaged"
export CFLAGS="${CFLAGS:-} -I${_HAIKU_SYSTEM}/develop/headers -I${_HAIKU_NONPKG}/develop/headers"
export CXXFLAGS="${CXXFLAGS:-} -I${_HAIKU_SYSTEM}/develop/headers -I${_HAIKU_NONPKG}/develop/headers"
export LDFLAGS="${LDFLAGS:-} -L${_HAIKU_SYSTEM}/develop/lib -L${_HAIKU_NONPKG}/develop/lib"

# [unverified] Haiku's run-time linker is runtime_loader, and it reads
# LIBRARY_PATH — NOT LD_LIBRARY_PATH, which it ignores outright.  Exporting the
# wrong variable fails silently: nothing warns, the build just dies later with
# an unresolved symbol from a dependency the prefix demonstrably contains.
# Convenient side effect: GCC also treats LIBRARY_PATH as extra link-time
# search directories, so one variable serves both the compile and the run.
# Mirrors cvcpkg.platform.lib_path_var(), which sets the same variable for the
# build subprocess — keep the two in step.
#
# ORDER: cvcpkg's own prefixes FIRST, Haiku's directories LAST.  A cvcpkg build
# must link and run against the cvcpkg closure — that is the whole point of the
# prefix — so a HaikuPorts copy of a library we ship must never win, and this is
# the same "ours first" layering that PATH, PKG_CONFIG_PATH and
# builder._build_env already use on every platform.
#
# The inherited value therefore goes in FRONT of the system dirs, not behind
# them: under haikuhost the runner exports LIBRARY_PATH already ordered
# build-prefix : deps : install : Haiku's run-time defaults (see
# cvcpkg.haikuhost._runner_script), and appending it after
# /boot/system/develop/lib — as this file used to — put the system's link
# libraries ahead of the build prefix and the component's own install dir,
# inverting the two files against each other.
_HAIKU_SYS_LIBS="${_HAIKU_SYSTEM}/develop/lib:${_HAIKU_NONPKG}/develop/lib"
_HAIKU_LIBS="${LIBRARY_PATH:-}"
if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
    # A native (non-delegated) run inherits nothing, so put the deps prefix in
    # ourselves; the case guard keeps it from being duplicated when the runner
    # already named it.
    case ":${_HAIKU_LIBS}:" in
        *":${CVC_DEPS_PREFIX}/lib:"*) ;;
        *) _HAIKU_LIBS="${_HAIKU_LIBS:+${_HAIKU_LIBS}:}${CVC_DEPS_PREFIX}/lib" ;;
    esac
fi
export LIBRARY_PATH="${_HAIKU_LIBS:+${_HAIKU_LIBS}:}${_HAIKU_SYS_LIBS}"

# pkg-config data ships beside the link libraries, in both trees.
export PKG_CONFIG_PATH="${_HAIKU_SYSTEM}/develop/lib/pkgconfig:${_HAIKU_NONPKG}/develop/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

# Non-packaged tools go on PATH AFTER our own deps prefix (already prepended by
# the builder), so a cvcpkg-built cmake/ninja still wins over a hand-installed
# one — same ordering rationale as env-netbsd.sh.
export PATH="${PATH}:${_HAIKU_NONPKG}/bin"

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

# Haiku is ELF and runtime_loader honours DT_RPATH/DT_RUNPATH with $ORIGIN
# expansion, so the -DCMAKE_INSTALL_RPATH=$ORIGIN above and cvcpkg's patchelf
# pass (builder._ELF_RPATH_PLATFORMS) produce relocatable bundles exactly as on
# Linux and the $ORIGIN-honouring BSDs.  HaikuPorts ships patchelf 0.18.0.

# [unverified] Two Haiku link-time gotchas recipes hit before this file can
# help them, recorded here because this is the file everyone reads first:
#   * There is no libpthread.so — POSIX threads live in libroot.so, so an
#     explicit -lpthread does not link.  CMake's FindThreads gets this right
#     (empty CMAKE_THREAD_LIBS_INIT); hand-rolled configure scripts often do
#     not and need -DTHREADS_PREFER_PTHREAD_FLAG=OFF or a patch.
#   * Sockets are in libnetwork.so, not libroot.so — anything doing BSD
#     sockets needs an explicit -lnetwork that no other platform wants.

echo "── env-haiku.sh loaded ──"
echo "  CC=${CC}  CXX=${CXX}"
echo "  BUILD_TYPE=${CMAKE_BUILD_TYPE}  LINK=${CVC_LINK}  JOBS=${CVC_JOBS}"
