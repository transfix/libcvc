#!/usr/bin/env bash
# cvcpkg/recipes/libcvc-examples-web/build-wasm.sh — the examples' browser gallery,
# packaged under share/libcvc-examples-web/web/ (served by libcvc-examples' launcher).
#
# It runs examples/web/build-wasm-demo.sh, the same script the gh-pages deploy runs
# (deploy-pages.yml calls this file), so the package and the site are one build: the
# whole `wasm-demos` gallery — the cvcGL scene demos, the nav demos with the Austin
# scene and trained policy preloaded, and the .ari demos — plus, single-threaded, the
# `ari` terminal page. There is no wasm libcvc to link: the trimmed cvc core + cvcGL
# closure is compiled from the repo root. Deps (vtk with rendering, boost, zstd, zlib,
# pegtl, yaml-cpp, assimp, imgui, and ftxui for the ari page) come static from
# CVC_DEPS_PREFIX; emsdk comes from the builder via CVC_EMSDK_DIR.
#
#   CVC_PLATFORM=wasm     single-threaded, no COOP/COEP needed (what gh-pages serves)
#   CVC_PLATFORM=wasm-mt  threaded (-pthread); serve it with serve.py
set -euo pipefail

: "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"
: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
: "${CVC_DEPS_PREFIX:?CVC_DEPS_PREFIX must point at the wasm deps prefix}"
: "${CVC_EMSDK_DIR:?CVC_EMSDK_DIR must point to the activated emsdk bundle}"

: "${CVC_PLATFORM:?CVC_PLATFORM must be wasm or wasm-mt}"

THREADS=()
case "$CVC_PLATFORM" in
  wasm-mt) THREADS=(--pthread) ;;
  wasm) ;;
  *) echo "libcvc-examples-web: CVC_PLATFORM is wasm or wasm-mt, not '${CVC_PLATFORM}'" >&2; exit 1 ;;
esac

# Preload the real Austin scene + trained policy into the nav_city demos' MEMFS
# when the data packages are staged (declared build deps), so the gallery shows
# downtown Austin. Absent the data, the demos fall back to a synthetic city + the
# default-biased net. A caller's own CVC_WASM_BUNDLE / CVC_WASM_NAV_WEIGHTS win.
_austin="$CVC_DEPS_PREFIX/share/cvc-scenes/austin_south"
_weights="$CVC_DEPS_PREFIX/share/grl-snam-weights/coef_sdf.cvcnav"
if [[ -z "${CVC_WASM_BUNDLE:-}" && -d "$_austin" ]]; then export CVC_WASM_BUNDLE="$_austin"; fi
if [[ -z "${CVC_WASM_NAV_WEIGHTS:-}" && -f "$_weights" ]]; then
  export CVC_WASM_NAV_WEIGHTS="$_weights"
fi

# IMAGEMAGICK is OFF on purpose. cvc's zero-dependency stb image handler
# (src/cvc/image/stb_io.cpp) is registered AHEAD of Magick for png/jpg/bmp/tga/gif
# reads, so the demos' textures (the Austin satellite.png, glb base-color maps)
# load without it; ImageMagick only backs volume_ops image I/O, which no gallery
# demo uses (the bunny volumes are generated). Linking it in would drag
# libMagickCore's bz2 / libxml2 / lzma delegate symbols into the executable link,
# and wasm-ld fails on them (they are not propagated to the demo link line).
WEB="$CVC_INSTALL_DIR/share/libcvc-examples-web/web"
install -d "$WEB"
CVC_WASM_DEPS="$CVC_DEPS_PREFIX" \
CVC_WASM_BUILD_DIR="$CVC_BUILD_DIR" \
CVC_WASM_GALLERY_DIR="$WEB" \
CVC_WASM_IMAGEMAGICK=OFF \
  bash "$CVC_SOURCE_DIR/examples/web/build-wasm-demo.sh" ${THREADS[@]+"${THREADS[@]}"}

# The .wasm files embed third-party code (VTK-derived code in cvcGL, BSD-3-Clause;
# lsystem_coast's OceanFFT, MIT; stb, sdf / mtxlib and the DTU .vtk reader in the cvc
# core) whose notices a binary redistribution must reproduce: ship them, and the
# LICENSE they link to, beside the gallery.
install -m 644 "$CVC_SOURCE_DIR/THIRD_PARTY_NOTICES.md" "$CVC_SOURCE_DIR/LICENSE" "$WEB/"
