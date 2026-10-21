#!/usr/bin/env bash
# cvcpkg/recipes/libcvc-examples/build.sh — build libcvc's example programs as
# commands. src/cvcGL is configured standalone against the installed libcvc SDK
# + VTK (the cvcgl recipe pattern) with CVC_BUILD_EXAMPLES=ON, which adds the
# repo's examples/; every example binary (the cvcGL scene demos, the nav demos and
# the headless nav CLIs) and the .ari documents land in the prefix via the
# libcvc-examples install component, and the libcvc-examples-web launcher +
# serve.py are staged so the browser gallery (the libcvc-examples-web package,
# share/libcvc-examples-web/web/) is one command away.
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
  -S "$CVC_SOURCE_DIR/src/cvcGL"
  -B "$CVC_BUILD_DIR"
  -DCMAKE_INSTALL_PREFIX="$CVC_INSTALL_DIR"
  -DCMAKE_BUILD_TYPE="$CMAKE_BUILD_TYPE"
  # cvcGL is built STATIC into the example binaries — no soname coupling to the
  # cvcgl SDK bundle; libcvc/VTK/Boost stay shared from the deps prefix.
  -DBUILD_SHARED_LIBS=OFF
  -DCVC_BUILD_EXAMPLES=ON
  # find_package(cvc) pulls CGAL in transitively, and CGAL_TweakFindBoost.cmake
  # sets Boost_USE_STATIC_LIBS from the CGAL_Boost_USE_STATIC_LIBS cache option
  # -- ON in the cvcpkg CGAL bundle.  cvcpkg ships Boost shared-only, so leaving
  # that in place makes the examples' find_package(Boost COMPONENTS
  # program_options) reject the very bundle installed beside it:
  #   Could not find a configuration file for package "boost_program_options"
  #   that exactly matches requested version "1.86.0"
  #   ... boost_program_options-1.86.0-shared/...  version: 1.86.0 (shared)
  # Setting the CGAL option (not Boost_USE_STATIC_LIBS, which CGAL overwrites as
  # a normal variable) asks for the variant the toolchain actually ships.
  -DCGAL_Boost_USE_STATIC_LIBS=OFF
  # Installed examples get RUNPATH $ORIGIN/../lib only. Every runtime dep is
  # installed into the same prefix as the bundle, and the link directories would
  # be this build's prefixes, which cvcpkg's pack-time relocation gate rejects
  # when publish-cvcpkg.yml packs the result with --from-prefix
  # (examples/CMakeLists.txt).
  -DLIBCVC_EXAMPLES_RPATH_USE_LINK_PATH=OFF
)

if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"

# UI guard (linux). cvcGL's imgui block degrades SILENTLY: when imgui is not
# found, ImGuiOverlay compiles to inert stubs and the demos still build, run and
# publish -- with every control panel gone and nothing in the log saying so. That
# is exactly how the published binaries shipped with no UI at all. imgui is a
# declared dependency of this recipe on linux, so a miss here is a broken closure,
# not a soft option: fail the build instead of quietly shipping a UI-less demo.
# cvcGL finds imgui through its CMake config (imgui::imgui) or, for a bundle with
# none, by header and library (CVC_IMGUI_LIBRARY), so the check is the result both
# share: cvcGL compiles with CVC_ENABLE_IMGUI. (It used to read CVC_IMGUI_LIBRARY
# alone, which the config path never sets.)
if [[ "${CVC_PLATFORM:-}" == "linux" ]]; then
  if ! grep -q -- '-DCVC_ENABLE_IMGUI' "$CVC_BUILD_DIR/build.ninja"; then
    echo "ERROR: cvcGL configured WITHOUT imgui -- the demos would ship with no UI." >&2
    echo "       imgui is a build dependency of libcvc-examples on linux; ensure its" >&2
    echo "       prefix is on CVC_DEPS_PREFIX ('cvcpkg install imgui --prefix ...')." >&2
    exit 1
  fi
  _imgui_dir=$(sed -n 's/^imgui_DIR:PATH=//p' "$CVC_BUILD_DIR/CMakeCache.txt")
  _imgui_lib=$(sed -n 's/^CVC_IMGUI_LIBRARY:FILEPATH=//p' "$CVC_BUILD_DIR/CMakeCache.txt")
  echo "libcvc-examples: imgui overlay ENABLED (${_imgui_lib:-${_imgui_dir}})"
fi

# Build every example and install just the libcvc-examples component: the bin/
# executables, RUNPATH $ORIGIN/../lib only, and the .ari documents
# (examples/CMakeLists.txt; NOT the cvcGL SDK, which shares this build tree).
cmake --build "$CVC_BUILD_DIR" -j "$CVC_JOBS"

# ABI guard: the demos link the SHARED libcvc from the deps prefix but were
# compiled against the in-tree cvc/nav headers. Run the headless nav_abi_smoke
# against that libcvc BEFORE installing, so a stale/lagging libcvc (its
# sim_world layout out of sync with these headers) fails HERE with a clear
# message instead of shipping demos that segfault at launch. `set -e` turns a
# non-zero exit into a recipe failure. Every native build makes it, so a missing
# binary is an error too: it used to be skipped, and a moved binary would have
# skipped the check without a word.
smoke="$CVC_BUILD_DIR/examples/nav_abi_smoke"
if [[ ! -x "$smoke" ]]; then
  echo "ERROR: the libcvc ABI guard $smoke was not built" >&2
  exit 1
fi
echo "libcvc-examples: running nav_abi_smoke (libcvc ABI guard)"
# CVC_DEPS_PREFIX may be a CMake list (publish-cvcpkg.yml passes four prefixes,
# ';'-separated): each prefix's lib/ goes on the loader path.
_libdirs=""
IFS=';' read -ra _prefixes <<< "${CVC_DEPS_PREFIX:-}"
for _p in ${_prefixes[@]+"${_prefixes[@]}"}; do
  [[ -n "$_p" ]] && _libdirs="${_libdirs:+${_libdirs}:}${_p}/lib"
done
LD_LIBRARY_PATH="${_libdirs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
DYLD_LIBRARY_PATH="${_libdirs}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}" \
  "$smoke"

cmake --install "$CVC_BUILD_DIR" --component libcvc-examples

# The web launcher: serve.py sends the COOP/COEP headers a -pthread wasm build
# needs (harmless for the single-threaded one); the bin/ command finds the web
# payload relative to its own prefix. /usr/bin/env shebangs pass through
# cvcpkg's shebang-rewrite untouched.
install -d "$CVC_INSTALL_DIR/share/libcvc-examples"
install -m 755 "$CVC_SOURCE_DIR/src/cvcGL/wasm/devtools/serve.py" \
  "$CVC_INSTALL_DIR/share/libcvc-examples/serve.py"
# The Firefox WebGL sync census (a dev tool, off unless serve.py --glsync): serve.py finds it in
# devtools/ next to itself. The launcher below does not pass --glsync, so pages are unchanged.
install -d "$CVC_INSTALL_DIR/share/libcvc-examples/devtools"
install -m 644 "$CVC_SOURCE_DIR/src/cvcGL/wasm/devtools/glsync.js" \
  "$CVC_SOURCE_DIR/src/cvcGL/wasm/devtools/GLSYNC.md" \
  "$CVC_INSTALL_DIR/share/libcvc-examples/devtools/"

cat > "$CVC_INSTALL_DIR/bin/libcvc-examples-web" <<'LAUNCHER'
#!/usr/bin/env bash
# Serve the libcvc examples' WebAssembly gallery and open it in a browser.
# The gallery is the libcvc-examples-web package (threaded, or --platform wasm
# for the single-threaded build with the ari page):
#   cvcpkg install libcvc-examples-web --platform wasm-mt --arch wasm32 --link static
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0" 2>/dev/null || echo "$0")")" && pwd)"
PREFIX="$(dirname "$HERE")"
WEB="$PREFIX/share/libcvc-examples-web/web"
PORT="${1:-8811}"
if [[ ! -f "$WEB/index.html" ]]; then
  echo "libcvc-examples-web: $WEB/index.html not found." >&2
  echo "Install the gallery into this prefix first:" >&2
  echo "  cvcpkg install libcvc-examples-web --platform wasm-mt --arch wasm32 --link static --prefix $PREFIX" >&2
  exit 1
fi
URL="http://localhost:$PORT/"
( sleep 1; xdg-open "$URL" 2>/dev/null || open "$URL" 2>/dev/null \
    || echo "Open $URL in a browser." ) &
exec python3 "$PREFIX/share/libcvc-examples/serve.py" -d "$WEB" "$PORT"
LAUNCHER
chmod +x "$CVC_INSTALL_DIR/bin/libcvc-examples-web"
