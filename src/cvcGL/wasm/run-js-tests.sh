#!/usr/bin/env bash
# run-js-tests.sh — run cvcGL's browser-side tests without CMake (what CI runs):
#   - the WebGL state shim's node test   (test/webgl_state_shadow_test.js)
#   - the glsync census's node test       (wasm/devtools/tests/test_glsync.js)
#   - serve.py's flag/injection tests     (wasm/devtools/tests/test_serve.py, python3)
#
# node comes from the Emscripten SDK (the cvcpkg emsdk bundle ships it), never a system one:
#   CVC_EMSDK_DIR=<emsdk prefix> src/cvcGL/wasm/run-js-tests.sh
# NODE=<path> overrides; /opt/cvc-wasm/emsdk (the fleet's) is the fallback. PYTHON=<python3>
# overrides the interpreter for test_serve.py (default: python3 on PATH).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" # src/cvcGL/wasm
CVCGL="$(dirname "$HERE")"                           # src/cvcGL

if [[ -z "${NODE:-}" ]]; then
    for d in "${CVC_EMSDK_DIR:-}" /opt/cvc-wasm/emsdk; do
        [[ -n "$d" ]] || continue
        for n in "$d"/node/*/bin/node; do
            if [[ -x "$n" ]]; then NODE="$n"; break 2; fi
        done
    done
fi
if [[ -z "${NODE:-}" || ! -x "$NODE" ]]; then
    echo "run-js-tests: no emsdk node (set CVC_EMSDK_DIR to the cvcpkg emsdk bundle, or NODE=)" >&2
    exit 2
fi
PYTHON="${PYTHON:-python3}"
echo "run-js-tests: node $("$NODE" --version) ($NODE); $("$PYTHON" --version 2>&1)"

fail=0
run() {
    local name="$1"; shift
    echo "=== $name"
    if "$@"; then echo "--- $name: ok"; else echo "--- $name: FAILED"; fail=1; fi
}
run webgl_state_shadow "$NODE" "$CVCGL/test/webgl_state_shadow_test.js" "$HERE/webgl_state_shadow.js"
run glsync "$NODE" "$HERE/devtools/tests/test_glsync.js"
run serve.py "$PYTHON" "$HERE/devtools/tests/test_serve.py"

if [[ "$fail" -ne 0 ]]; then
    echo "run-js-tests: FAILED" >&2
    exit 1
fi
echo "run-js-tests: all passed"
