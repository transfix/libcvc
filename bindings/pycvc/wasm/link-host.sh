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
#   NODE   = optional node to smoke-run the result.
#
# Key facts it encodes:
#   * numpy's wasm C-extensions are RELOCATABLE objects (not side modules), so
#     they link statically; each is inittab-registered under its dotted name.
#   * each numpy ext statically bundles npymath -> --allow-multiple-definition.
#   * pycvc core is a flat module (_pycvc); pycvc_gl is a package (pycvc_gl._pycvc_gl).
#   * libpython's _sha2 has an un-archived HACL gap -> ERROR_ON_UNDEFINED_SYMBOLS=0
#     (harmless: the missing symbols are only reached if hashlib.sha256 is called).
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

# Generate the numpy static-extension inittab registrar from the actual .so set.
GEN="$OUT/numpy_embed_gen.cpp"
python3 - "$SP_DEPS" "$GEN" "$OUT/numpy_so_list.txt" <<'PY'
import os, glob, re, sys
sp, gen, lst = sys.argv[1], sys.argv[2], sys.argv[3]
E=[]
for so in glob.glob(os.path.join(sp,"numpy","**","*.so"), recursive=True):
    rel=os.path.relpath(so,sp).replace("\\","/"); base=os.path.basename(so)
    mod=re.sub(r"\.so$","",re.sub(r"\.cpython-.*$","",base))
    if mod.endswith("_tests"): continue
    E.append((os.path.dirname(rel).replace("/",".")+"."+mod,"PyInit_"+mod,so.replace("\\","/")))
E.sort()
with open(gen,"w") as f:
    f.write('#include <Python.h>\nextern "C" {\n')
    for d,pi,_ in E: f.write(f"PyObject *{pi}(void);\n")
    f.write("int pycvc_register_numpy_inittab(void){\n")
    for d,pi,_ in E: f.write(f'  if(PyImport_AppendInittab("{d}",{pi})) return 1;\n')
    f.write("  return 0;\n}\n}\n")
open(lst,"w").write("\n".join(e[2] for e in E))
print(f"link-host: registered {len(E)} numpy extensions")
PY
mapfile -t NPYOBJS < "$OUT/numpy_so_list.txt"

printf "Module.preRun=Module.preRun||[];Module.preRun.push(function(){ENV.PYTHONHOME='/py';ENV.PYTHONDONTWRITEBYTECODE='1';});\n" > "$OUT/pre.js"

"$EMCC" "$SRC/bindings/pycvc/wasm/pycvc_host.cpp" "$GEN" \
    -std=c++17 -O1 -DPYCVC_EMBED_NUMPY -I "$PYINC" \
    -Wl,--allow-multiple-definition \
    -Wl,--whole-archive "$PXA" "$GLA" -Wl,--no-whole-archive \
    "${NPYOBJS[@]}" \
    -Wl,--start-group "$INST"/lib/libcvc.a "$INST"/lib/libcvcGL.a "$DEPS"/lib/*.a -Wl,--end-group \
    -sALLOW_MEMORY_GROWTH=1 -sEXIT_RUNTIME=1 -sSTACK_SIZE=4mb -sERROR_ON_UNDEFINED_SYMBOLS=0 \
    --embed-file "${PYSTD}@/py/lib/python3.12" \
    --embed-file "$(dirname "$PXA")/pycvc.py@/py/lib/python3.12/site-packages/pycvc.py" \
    --embed-file "$(dirname "$GLA")@/py/lib/python3.12/site-packages/pycvc_gl" \
    --pre-js "$OUT/pre.js" \
    -o "$OUT/pycvc_host.js"
echo "link-host: built $OUT/pycvc_host.wasm"
[ -n "${NODE:-}" ] && [ -f "$OUT/pycvc_host.wasm" ] && "$NODE" "$OUT/pycvc_host.js" || true
