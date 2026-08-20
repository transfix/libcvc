#!/usr/bin/env bash
# cvcpkg/recipes/cvcgl-examples/build.sh — build the cvcGL example programs as
# commands. src/cvcGL is configured standalone against the installed libcvc SDK
# + VTK (the cvcgl recipe pattern) with CVC_BUILD_EXAMPLES=ON; the example
# binaries land in bin/, and the cvcgl-examples-web launcher + serve.py are
# staged so the wasm bundle (share/cvcgl-examples/web/) is one command away.
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
)

if [[ -n "${CVC_DEPS_PREFIX:-}" ]]; then
  CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CVC_DEPS_PREFIX")
fi

cmake "${CMAKE_ARGS[@]}"
cmake --build "$CVC_BUILD_DIR" --target lsystem_forest -j "$CVC_JOBS"

# examples/CMakeLists.txt has no install() rule — stage the binary by hand.
install -d "$CVC_INSTALL_DIR/bin"
install -m 755 "$CVC_BUILD_DIR/bin/lsystem_forest" "$CVC_INSTALL_DIR/bin/lsystem_forest"

# The web launcher: serve.py sends the COOP/COEP headers a -pthread wasm build
# needs (harmless for the single-threaded one); the bin/ command finds the web
# payload relative to its own prefix. /usr/bin/env shebangs pass through
# cvcpkg's shebang-rewrite untouched.
install -d "$CVC_INSTALL_DIR/share/cvcgl-examples"
install -m 755 "$CVC_SOURCE_DIR/src/cvcGL/examples/wasm/serve.py" \
  "$CVC_INSTALL_DIR/share/cvcgl-examples/serve.py"

cat > "$CVC_INSTALL_DIR/bin/cvcgl-examples-web" <<'LAUNCHER'
#!/usr/bin/env bash
# Serve the cvcgl-examples WebAssembly demos and open them in a browser.
# The web payload comes from the wasm variant of this package:
#   cvcpkg install cvcgl-examples --platform wasm --arch wasm32 --link static
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0" 2>/dev/null || echo "$0")")" && pwd)"
PREFIX="$(dirname "$HERE")"
WEB="$PREFIX/share/cvcgl-examples/web"
PORT="${1:-8811}"
if [[ ! -f "$WEB/index.html" ]]; then
  echo "cvcgl-examples-web: $WEB/index.html not found." >&2
  echo "Install the wasm variant into this prefix first:" >&2
  echo "  cvcpkg install cvcgl-examples --platform wasm --arch wasm32 --link static --prefix $PREFIX" >&2
  exit 1
fi
URL="http://localhost:$PORT/"
( sleep 1; xdg-open "$URL" 2>/dev/null || open "$URL" 2>/dev/null \
    || echo "Open $URL in a browser." ) &
exec python3 "$PREFIX/share/cvcgl-examples/serve.py" -d "$WEB" "$PORT"
LAUNCHER
chmod +x "$CVC_INSTALL_DIR/bin/cvcgl-examples-web"
