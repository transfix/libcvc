#!/usr/bin/env bash
# bindings/pycvc/wasm/link-host.sh — link the CPython-wasm host that embeds
# pycvc + pycvc_gl + numpy into one .wasm, and (optionally) run it under node.
#
# This is the PROVEN recipe (validated end-to-end on 2026-09-23: numpy 2.4.6
# computes, `import pycvc`/`import pycvc_gl` succeed, PYCVC_WASM_OK). Run it after
# `cvcpkg build pycvc-gl-cp312 --platform wasm` has produced the static archives.
#
# Env:
#   DEPS   = wasm deps prefix (python312 + numpy + vtk + vtk-python + boost/...).
#   INST   = the pycvc-gl install prefix (_pycvc.a, pycvc_gl/_pycvc_gl.a, libcvc.a,
#            libcvcGL.a, the .py proxies). May equal DEPS (one-prefix layout).
#   SRC    = libcvc repo root (for bindings/pycvc/wasm/pycvc_host.cpp).
#   EMSDK  = activated emsdk (emcc.bat/em++ on it); OUT = output dir.
#   NODE   = optional node to smoke-run the result. When set, the script's exit
#            status is the smoke's (pycvc_host.cpp's default script exits non-zero
#            when an import, or Exec(app) running a with-lock, fails).
#   PYTHON = optional native Python 3 for the extension steps (default: python3).
#
# Key facts it encodes:
#   * Python C extensions for wasm (numpy's) are RELOCATABLE objects (not side
#     modules), so they link statically; each is inittab-registered under its
#     dotted name.
#   * Each extension object has a namespace of its own (its PyInit renamed after
#     its dotted module, its other globals local; cvcpkg/recipes/_common/
#     wasm_pyext.py), listed in a share/python-wasm/*.inittab manifest. numpy's
#     extensions each bundle npymath, which once needed --allow-multiple-definition;
#     localized, they don't, and a real clash between extensions fails the link.
#   * pycvc core is a flat module (_pycvc); pycvc_gl is a package (pycvc_gl._pycvc_gl).
#   * Undefined symbols are errors. So the link names what a CMake executable gets from the
#     targets it links: cvcGL's cvcMedia, cvc::net's -sFETCH=1 and -lwebsocket.js, the WebGL 2
#     flags VTK's FindOpenGL adds, and VTK RenderingUI's js-library. And libpython3.12.a must be
#     python312 3.12.10+cvc.23 or later: earlier wasm bundles archive pyexpat and _sha2 without
#     their expat and HACL SHA-2 objects (cy-pca/cvcpkg#565). With the VTK bridge, cvc.25 or
#     later: cvc.23 and cvc.24 clash with VTK's expat on two unrenamed internals
#     (cy-pca/cvcpkg#581).
#   * cvc, cvcGL and the wrappers compile with -fexceptions (the top-level CMakeLists.txt),
#     so this link must say it too. Without it emscripten links with exception catching
#     off and every throw aborts ("Aborted(undefined)" in ___cxa_throw), even one a catch
#     would take: cvc::state::instance() on a new app catches its first lookup's bad_any_cast.
set -euo pipefail
: "${DEPS:?}"; : "${INST:?}"; : "${SRC:?}"; : "${EMSDK:?}"; : "${OUT:?}"
EMCC="${EMSDK}/upstream/emscripten/emcc.bat"; [ -x "$EMCC" ] || EMCC="${EMSDK}/upstream/emscripten/emcc"
mkdir -p "$OUT"

# site-packages can be lib/ (POSIX) or Lib/ (Windows) — find whichever holds numpy.
SP_DEPS="$(find "$DEPS" -maxdepth 3 -type d -name site-packages -path '*python3.12*' 2>/dev/null | head -1)"
PYINC="$(find "$DEPS" -maxdepth 3 -type d -name python3.12 -path '*include*' 2>/dev/null | head -1)"
PYSTD="$(dirname "$SP_DEPS")"                       # .../lib/python3.12 (stdlib root)
PXA="$(find "$INST" -name '_pycvc.a' 2>/dev/null | head -1)"
GLA="$(find "$INST" -name '_pycvc_gl.a' 2>/dev/null | head -1)"
[ -n "$PXA" ] && [ -n "$GLA" ] || { echo "link-host: pycvc archives not found under $INST" >&2; exit 1; }
# pycvc.ari, the Host that pycvc_gl.ariadne is: pure Python, installed beside pycvc.py.
ARI_PY="$(dirname "$PXA")/_pycvc_ari.py"
[ -f "$ARI_PY" ] || { echo "link-host: $ARI_PY not found (pycvc.ari, which pycvc_gl.ariadne is)" >&2; exit 1; }

# The Python C extensions, from the deps prefix's share/python-wasm/*.inittab
# manifests: each lists a package's extensions (dotted module, renamed init symbol,
# object). A package installed before it shipped one (numpy-cp312 before cvc.6) is
# localized here instead, into copies under a scratch dir, so the deps prefix is
# never modified. Then `check` refuses a set that would clash (one module listed
# twice, a global defined by two extensions, an object not localized), and
# `registrar` writes pycvc_register_pyext_inittab() and the list of objects.
PYTHON="${PYTHON:-python3}"
PYEXT="$SRC/cvcpkg/recipes/_common/wasm_pyext.py"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/link-host.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
MANIFESTS=()
for m in "$DEPS"/share/python-wasm/*.inittab; do [ -f "$m" ] && MANIFESTS+=("$m"); done
lists_numpy() { [ ${#MANIFESTS[@]} -gt 0 ] && grep -qsE '^numpy[.]' "${MANIFESTS[@]}"; }
if [ -d "$SP_DEPS/numpy" ] && ! lists_numpy; then
    echo "link-host: numpy ships no inittab manifest -> localizing copies of its extensions"
    "$PYTHON" "$PYEXT" package "$DEPS" numpy --site-packages "$SP_DEPS" --out "$WORK/pyext" >/dev/null
    MANIFESTS+=("$WORK/pyext/share/python-wasm/numpy.inittab")
fi
GEN="$WORK/pyext_inittab.cpp"
if [ ${#MANIFESTS[@]} -gt 0 ]; then
    "$PYTHON" "$PYEXT" check "${MANIFESTS[@]}"
    "$PYTHON" "$PYEXT" registrar --function pycvc_register_pyext_inittab \
        --cpp "$GEN" --objects "$WORK/pyext_objects.txt" "${MANIFESTS[@]}"
else
    echo "link-host: no Python C extensions in DEPS"
    printf '#include <Python.h>\nextern "C" int pycvc_register_pyext_inittab(void) { return 0; }\n' > "$GEN"
    : > "$WORK/pyext_objects.txt"
fi
# strip CR: on a Windows host the generator's python may write CRLF, which would
# leave a trailing \r on each path and break emcc ("No such file").
mapfile -t PYEXT_OBJS < <(tr -d '\r' < "$WORK/pyext_objects.txt")

# vtk-python cvc.6: its static build aggregates ALL wrapped modules into ONE
# builtin archive (_vtkmodules_static.a exporting PyInit__vtkmodules_static) plus
# pure-python stubs in _vtk.zip. Whole-archive the static lib (so the builtin + all
# wrapped code survive), embed the zip + put it on PYTHONPATH, and define
# -DPYCVC_EMBED_VTKPYTHON so the host registers the builtin. The vtk C++ libs +
# libvtkWrappingPythonCore come in via the $DEPS/lib/*.a group. Absent (a
# numpy-only build) => cleanly skipped (host still runs; no VTK bridge).
VTKA="$(find "$DEPS" -name '_vtkmodules_static.a' 2>/dev/null | head -1)"
VTKZIP="$(find "$DEPS" -name '_vtk.zip' 2>/dev/null | head -1)"
VTK_DEF=(); VTK_WHOLE=(); VTK_EMBED=(); VTK_PYPATH=""
if [ -n "$VTKA" ] && [ -n "$VTKZIP" ]; then
    VTK_DEF=(-DPYCVC_EMBED_VTKPYTHON)
    VTK_WHOLE=(-Wl,--whole-archive "$VTKA" -Wl,--no-whole-archive)
    VTK_EMBED=(--embed-file "${VTKZIP}@/py/lib/python3.12/site-packages/_vtk.zip")
    VTK_PYPATH="ENV.PYTHONPATH='/py/lib/python3.12/site-packages/_vtk.zip';"
    echo "link-host: vtk-python bridge ON -> ${VTKA}"
else
    echo "link-host: vtk-python not in DEPS -> numpy-only host (no VTK bridge)"
fi

# The python312 revision, from the bundle's manifest, as src/ariadne_python/ariadne_python.cmake
# reads it (cvcpkg's version ranges ignore +cvc.N, so the recipe can't ask for it): cvc.23 or
# later, and with the VTK bridge cvc.25 or later, since vtk-python brings in VTK's expat and cvc.23
# and cvc.24 leave two expat internals unrenamed that VTK's copy defines too. A Python that cvcpkg
# didn't install has no manifest, and is left to the link.
PYMAN="$DEPS/share/libcvc-deps/python312/manifest.yaml"
if [ -f "$PYMAN" ]; then
    PYUP="$(sed -n 's/^  upstream_version: *//p' "$PYMAN" | head -1)"
    PYREV="$(sed -n 's/^  cvc_revision: *//p' "$PYMAN" | head -1)"
    PYMIN=23; PYWHY="lacks pyexpat's expat and _sha2's HACL objects"
    if [ -n "$VTKA" ] && [ -n "$VTKZIP" ]; then
        PYMIN=25; PYWHY="$PYWHY, or (cvc.23, cvc.24) clashes with VTK's expat"
    fi
    case "$PYUP" in
        3.12.[0-9]) PYOLD=1 ;;
        3.12.10) [ "${PYREV:-0}" -ge "$PYMIN" ] || PYOLD=1 ;;
    esac
    if [ -n "${PYOLD:-}" ]; then
        echo "link-host: $DEPS has python312 $PYUP+cvc.$PYREV, whose libpython3.12.a $PYWHY:" \
             "install 3.12.10+cvc.$PYMIN or later for wasm there" >&2
        exit 1
    fi
fi

printf "Module.preRun=Module.preRun||[];Module.preRun.push(function(){ENV.PYTHONHOME='/py';%sENV.PYTHONDONTWRITEBYTECODE='1';});\n" "$VTK_PYPATH" > "$OUT/pre.js"

# Every deps archive goes into the group. This needs vtk 9.5.0+cvc.4 or later, whose wasm
# builds use cvcpkg's libxml2, libtiff, liblzma and freetype instead of VTK's bundled
# copies. Older vtk wasm bundles ship libvtklibxml2-9.5.a, which renames only libxml2's
# public API. Its plain-named internals (__xmlRaiseError, xmlBuf*, ...) collide with the
# libxml2 that ImageMagick brings: a duplicate-symbol error at this link (while it allowed
# multiple definitions, a module that did not compile: "expected 0 elements on the stack
# for fallthru").
DEP_LIBS=("$DEPS"/lib/*.a)

# TODO(cvcGL wasm app contract, docs/CVCGL_WASM.md): this host is node-only today, so it links
# neither of cvcGL's browser-side speedups. When it gets a browser page, that link must add
#   --pre-js "$INST/share/cvcGL/wasm/webgl_state_shadow.js"   (the WebGL state shim), and
#   -sMALLOC=mimalloc                                         (on a wasm-mt / -pthread build only)
# -- what cvcgl_wasm_app() does for a CMake app. It links no -sASYNCIFY, so FrameYield and
# -sASYNCIFY_IGNORE_INDIRECT do not apply until it does.

"$EMCC" "$SRC/bindings/pycvc/wasm/pycvc_host.cpp" "$GEN" \
    -std=c++17 -O1 -fexceptions -DPYCVC_EMBED_PYEXT ${VTK_DEF[@]+"${VTK_DEF[@]}"} -I "$PYINC" \
    -Wl,--whole-archive "$PXA" "$GLA" -Wl,--no-whole-archive \
    ${VTK_WHOLE[@]+"${VTK_WHOLE[@]}"} \
    ${PYEXT_OBJS[@]+"${PYEXT_OBJS[@]}"} \
    -Wl,--start-group "$INST"/lib/libcvc.a "$INST"/lib/libcvcGL.a "$INST"/lib/libcvcMedia.a \
        "${DEP_LIBS[@]}" -Wl,--end-group \
    -sALLOW_MEMORY_GROWTH=1 -sEXIT_RUNTIME=1 -sSTACK_SIZE=8mb \
    -sFETCH=1 -lwebsocket.js -sFULL_ES3=1 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 \
    --js-library "$DEPS/lib/vtkWebAssemblyRenderWindowInteractor.js" \
    --embed-file "${PYSTD}@/py/lib/python3.12" \
    --embed-file "$(dirname "$PXA")/pycvc.py@/py/lib/python3.12/site-packages/pycvc.py" \
    --embed-file "${ARI_PY}@/py/lib/python3.12/site-packages/_pycvc_ari.py" \
    --embed-file "$(dirname "$GLA")@/py/lib/python3.12/site-packages/pycvc_gl" \
    ${VTK_EMBED[@]+"${VTK_EMBED[@]}"} \
    --pre-js "$OUT/pre.js" \
    -o "$OUT/pycvc_host.js"
echo "link-host: built $OUT/pycvc_host.wasm"
# The smoke, only with NODE set; under set -e a failing smoke fails the script. build-wasm.sh
# keeps the link (and so this) non-fatal for packaging; the release workflow runs the installed
# host itself and fails on it.
if [ -n "${NODE:-}" ]; then
    "$NODE" "$OUT/pycvc_host.js"
fi
