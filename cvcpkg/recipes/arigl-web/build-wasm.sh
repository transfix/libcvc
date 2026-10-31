#!/usr/bin/env bash
# cvcpkg/recipes/arigl-web/build-wasm.sh — arigl-web, the stock browser host for Ariadne (.ari)
# documents, packaged under share/arigl-web/: index.html (src/arigl/web/arigl-web.html) with its
# arigl-web.js and arigl-web.css, arigl.js and arigl.wasm, and the third-party notices the
# module's VTK-derived code needs. The single-threaded package also has the terminal runner's page,
# ari.html with ari.js and ari.wasm, its ari-web.js and ari-web.css, and xterm/ (src/ari/web): the
# same documents in the FTXUI terminal backend, loaded the same way (?src=, default main.ari, the
# same page config). With tvision's browser console in the deps prefix it adds ari_tvision.html,
# the tvision desktop.
#
# arigl built to wasm with no documents of its own, only the component library embedded: a page
# serving these files opens one by URL (?src=), and arigl fetches it and the files it imports
# (cvc/ariadne/web_document.h). There is no single-threaded wasm libcvc bundle to link, so the
# trimmed cvc core + cvcGL closure compiles from the repo root, as libcvc-examples-web's does,
# with the examples off: that is what makes src/arigl build the shell rather than the gallery's
# arigl. Deps come static from CVC_DEPS_PREFIX; emsdk from the builder via CVC_EMSDK_DIR.
#
#   CVC_PLATFORM=wasm     single-threaded: any static host, no special headers
#   CVC_PLATFORM=wasm-mt  threaded (-pthread): the page must be cross-origin isolated
#                         (Cross-Origin-Opener-Policy: same-origin and
#                         Cross-Origin-Embedder-Policy: require-corp on index.html)
#
# The package has what the gallery's arigl has. CVC_ARIGL_WEB_LEAN=1 builds a smaller one for
# documents that are mostly widgets: no ImageMagick (PNG and JPEG still read through stb), no
# assimp (glTF and the other mesh formats it reads), no LBIE mesher, SDF or libigl. Measured on
# 3.7.4 (docs/CVCGL_WASM.md): arigl.wasm 49.0 MB -> 33.7 MB raw, 9.5 MB -> 6.1 MB brotli.
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_DEPS_PREFIX:?CVC_DEPS_PREFIX must point at the wasm deps prefix}"
: "${CVC_EMSDK_DIR:?CVC_EMSDK_DIR must point to the activated emsdk bundle}"
: "${CVC_PLATFORM:?CVC_PLATFORM must be wasm or wasm-mt}"

# ari's pages are single-threaded only, as in the gallery.
case "$CVC_PLATFORM" in
  wasm) PTHREADS=OFF; ARI=ON ;;
  wasm-mt) PTHREADS=ON; ARI=OFF ;;
  *) echo "arigl-web: CVC_PLATFORM is wasm or wasm-mt, not '${CVC_PLATFORM}'" >&2; exit 1 ;;
esac
FULL=ON
[[ "${CVC_ARIGL_WEB_LEAN:-0}" == "1" ]] && FULL=OFF

# shellcheck disable=SC1091
source "${CVC_EMSDK_DIR}/emsdk_env.sh"

mkdir -p "$CVC_BUILD_DIR"
emcmake cmake -G Ninja -S "$CVC_SOURCE_DIR" -B "$CVC_BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_ROOT_PATH="$CVC_DEPS_PREFIX" \
    -DCMAKE_PREFIX_PATH="$CVC_DEPS_PREFIX" \
    -DCVC_ENABLE_CUDA=OFF \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_BUILD_CLI=OFF \
    -DCVC_BUILD_ARI="$ARI" \
    -DCVC_ARI_WASM_DEFAULT_SRC=main.ari \
    -DCVC_BUILD_ARIGL=ON \
    -DCVC_ENABLE_OPENMP=OFF \
    -DDISABLE_CGAL=ON \
    -DCVC_USING_HDF5=OFF \
    -DCVC_USING_IMOD_MRC=OFF \
    -DCVC_ENABLE_IMAGEMAGICK="$FULL" \
    -DCVC_ENABLE_FFTW=OFF \
    -DCVC_FFT_PROVIDER=none \
    -DCVC_ENABLE_ASSIMP="$FULL" \
    -DCVC_ENABLE_MESHER="$FULL" \
    -DCVC_ENABLE_SDF="$FULL" \
    -DCVC_ENABLE_LIBIGL="$FULL" \
    -DCVC_BUILD_CVCGL=ON \
    -DCVC_BUILD_EXAMPLES=OFF \
    -DCVC_WASM_PTHREADS="$PTHREADS" \
    | tee "$CVC_BUILD_DIR/configure.log"

# A configure that misses one of these turns it off and carries on, and the shell would ship
# unable to load a document (yaml-cpp), with inert widgets (imgui) or without keys (sdl3).
# Nor without the terminal backend, for ari's page.
for want in "Ariadne: .ari YAML loader ENABLED" \
            "Ariadne: FTXUI terminal backend ENABLED" \
            "cvcGL: Dear ImGui overlay ENABLED" \
            "cvcGL: SDL3 input/peripheral seam ENABLED"; do
  grep -qF "$want" "$CVC_BUILD_DIR/configure.log" \
    || { echo "arigl-web: the configure did not report '$want' (deps in $CVC_DEPS_PREFIX)" >&2; exit 1; }
done
if [[ "$FULL" == "ON" ]] \
   && ! grep -q '^CVC_IMAGEMAGICK_ENABLED:INTERNAL=ON' "$CVC_BUILD_DIR/CMakeCache.txt"; then
  echo "arigl-web: the configure found no ImageMagick in $CVC_DEPS_PREFIX" >&2
  exit 1
fi

TARGETS=(arigl)
ARI_PAGES=()
if [[ "$ARI" == "ON" ]]; then
  ARI_PAGES=(ari)
  # tvision with its browser console (cvcpkg tvision cvc.4 on, the wasm column; a declared dep)
  # adds the tvision desktop's page; a deps prefix without it builds the package without the page.
  if grep -q '^CVC_ARIADNE_TVISION_ENABLED:INTERNAL=ON' "$CVC_BUILD_DIR/CMakeCache.txt"; then
    ARI_PAGES+=(ari_tvision)
  else
    echo "arigl-web: no tvision with a browser console in $CVC_DEPS_PREFIX; no ari_tvision page"
  fi
  TARGETS+=("${ARI_PAGES[@]}")
fi
cmake --build "$CVC_BUILD_DIR" --target "${TARGETS[@]}" -j "${CVC_JOBS:-$(nproc)}"

BIN="$CVC_BUILD_DIR/bin"
WEB="$CVC_INSTALL_DIR/share/arigl-web"
install -d "$WEB"
install -m 644 "$BIN/arigl.js" "$BIN/arigl.wasm" "$BIN/arigl-web.js" "$BIN/arigl-web.css" "$WEB/"
install -m 644 "$BIN/arigl-web.html" "$WEB/index.html"
for p in ${ARI_PAGES[@]+"${ARI_PAGES[@]}"}; do
  install -m 644 "$BIN/$p.html" "$BIN/$p.js" "$BIN/$p.wasm" "$WEB/"
done
if [[ ${#ARI_PAGES[@]} -gt 0 ]]; then
  # The pages' loader and style sheet, and xterm.js with its addons and licenses (no CDN).
  install -m 644 "$BIN/ari-web.js" "$BIN/ari-web.css" "$WEB/"
  install -d "$WEB/xterm"
  install -m 644 "$BIN"/xterm/* "$WEB/xterm/"
fi
# The module embeds third-party code (VTK-derived code in cvcGL, BSD-3-Clause; stb, sdf / mtxlib
# and the DTU .vtk reader in the cvc core) whose notices a binary redistribution must reproduce.
install -m 644 "$CVC_SOURCE_DIR/THIRD_PARTY_NOTICES.md" "$CVC_SOURCE_DIR/LICENSE" "$WEB/"

# What a page downloads, raw and compressed as a server would send it. Brotli through the brotli
# CLI, else the emsdk's node (its zlib has it).
brotli_size() {
  if command -v brotli >/dev/null 2>&1; then
    brotli -q 11 -c "$1" | wc -c
  elif [[ -n "${EMSDK_NODE:-}" ]]; then
    "$EMSDK_NODE" -e 'const z = require("zlib");
      const b = require("fs").readFileSync(process.argv[1]);
      console.log(z.brotliCompressSync(b, {params: {[z.constants.BROTLI_PARAM_QUALITY]: 11,
        [z.constants.BROTLI_PARAM_SIZE_HINT]: b.length}}).length)' "$1"
  else
    echo -
  fi
}
echo "arigl-web ($CVC_PLATFORM) sizes, bytes: raw / gzip -9 / brotli -q 11"
FILES=(arigl.wasm arigl.js index.html arigl-web.js arigl-web.css)
for p in ${ARI_PAGES[@]+"${ARI_PAGES[@]}"}; do FILES+=("$p.wasm" "$p.js" "$p.html"); done
for f in "${FILES[@]}"; do
  printf '  %-13s %10s %10s %10s\n' "$f" "$(wc -c < "$WEB/$f")" \
    "$(gzip -9 -c "$WEB/$f" | wc -c)" "$(brotli_size "$WEB/$f")"
done
