#!/usr/bin/env bash
# cvcpkg/recipes/libcvc/build.sh - build libcvc from the in-repo source tree.
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

# macOS: the builder puts the deps prefix's lib on DYLD_LIBRARY_PATH, which dyld searches by leaf
# name BEFORE a library's own install path, so a prefix library shadows the system's of the same
# name for every program this script runs. Since imagemagick cvc.25 pulled cvcpkg's GNU libiconv
# into the prefix, Homebrew's cmake loaded it in place of /usr/lib/libiconv.2.dylib and aborted
# (Symbol not found: _iconv; GNU names it _libiconv). DYLD_FALLBACK_LIBRARY_PATH is searched only
# when a library is not found where it is named, so the prefix still resolves @rpath libraries and
# the system keeps its own. The default fallback dirs are kept after it.
if [[ "$(uname -s)" == Darwin && -n "${DYLD_LIBRARY_PATH:-}" ]]; then
  export DYLD_FALLBACK_LIBRARY_PATH="$DYLD_LIBRARY_PATH:${DYLD_FALLBACK_LIBRARY_PATH:-$HOME/lib:/usr/local/lib:/usr/lib}"
  unset DYLD_LIBRARY_PATH
fi

CVC_BUILD_TYPE="${CVC_BUILD_TYPE:-Release}"
CVC_LINK="${CVC_LINK:-shared}"
# macOS has no nproc: ask sysctl there, so a Mac build is not held to 4 jobs.
CVC_JOBS="${CVC_JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)}"

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
  # Test gate (default OFF): `CVC_BUILD_TESTS=ON cvcpkg build libcvc ...`
  # builds the Google Test suite (into the build tree) and runs it after the
  # build. OFF by default so the packaging/release path stays fast. GoogleTest
  # is the recipe's googletest build dep, which `cvcpkg install-deps` puts in
  # CVC_DEPS_PREFIX for CMake/SetupGTest.cmake to find; only a prefix without
  # it makes the configure fetch GoogleTest from GitHub.
  -DCVC_BUILD_TESTS="${CVC_BUILD_TESTS:-OFF}"
  # The `cvc` CLI and `ari` ship as their own packages (the cvc-cli /
  # cvc-cli-cuda and ari recipes) so they land deliberately in a user's PATH
  # instead of riding along with the SDK bundle. Keep them OUT of the libcvc
  # bundle.
  -DCVC_BUILD_CLI=OFF
  -DCVC_BUILD_ARI=OFF
  # arisrv, the conference relay, is its own package too (the arisrv recipe). It defaults ON and
  # builds whenever CVC_ENABLE_GRPC is on, as it is below, and `cvcpkg pack` takes everything the
  # install stages, so without this the SDK bundle would carry bin/arisrv, the same file the
  # arisrv package installs.
  -DCVC_BUILD_ARISRV=OFF
  # CUDA is opt-in per recipe: the `libcvc` recipe leaves this OFF (portable
  # CPU build); the sibling `libcvc-cuda` recipe's build.sh runs this same
  # script with CVC_ENABLE_CUDA=ON to produce the GPU variant.
  -DCVC_ENABLE_CUDA="${CVC_ENABLE_CUDA:-OFF}"
  # The conference transports: cvc::state over gRPC and WebSocket, which arigl and the arisrv
  # relay need to join a room. The recipe's runtime deps already carry grpc, protobuf, abseil and
  # re2 for every platform this builds; with these OFF the SDK shipped without them, so an arigl or
  # arisrv built against it compiled the conference out.
  -DCVC_ENABLE_GRPC=ON
  -DCVC_ENABLE_WS=ON
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
  # Isosurfacing/meshing and signed distance fields ship in every bundle. Both default ON; set
  # here so the SDK's whole feature set is written down in this one script, which
  # publish-cvcpkg.yml runs too.
  -DCVC_ENABLE_MESHER=ON
  -DCVC_ENABLE_SDF=ON
)

if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

# macOS: nothing from Homebrew. GitHub's macOS images ship it (/usr/local on Intel, /opt/homebrew
# on arm64), and a CMake search there once compiled libcvc against Homebrew's turbojpeg.h 3.2
# instead of the deps prefix's 3.1, so every Intel row of 3.7.1 failed to link
# (_tj3InitVersion). cvcpkg is the only source of libcvc's dependencies.
if [[ "$(uname -s)" == Darwin ]]; then
  CMAKE_ARGS+=("-DCMAKE_IGNORE_PREFIX_PATH=/usr/local;/opt/homebrew")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"

# Build-and-run tests, never install them: no test target has an install()
# rule and package.files selects only lib/ and include/, so nothing test-
# related reaches CVC_INSTALL_DIR or the published bundle. Running before the
# install step keeps a failed suite from staging a bundle at all.
# CVC_CTEST_EXCLUDE: a ctest -E regex of tests to leave out (CI's long-running suites).
# CVC_CTEST_REPEAT: a ctest --repeat mode (CI's until-pass:3, as its package jobs run).
case "$(printf %s "${CVC_BUILD_TESTS:-OFF}" | tr "[:upper:]" "[:lower:]")" in
  on|1|true|yes)
    ctest --test-dir "$CVC_BUILD_DIR" --output-on-failure -j "$CVC_JOBS" \
      ${CVC_CTEST_EXCLUDE:+-E "$CVC_CTEST_EXCLUDE"} \
      ${CVC_CTEST_REPEAT:+--repeat "$CVC_CTEST_REPEAT"} ;;
esac

cmake --install "$CVC_BUILD_DIR"
