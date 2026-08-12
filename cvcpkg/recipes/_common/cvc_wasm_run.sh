#!/usr/bin/env bash
# recipes/_common/cvc_wasm_run.sh — helpers for running wasm/wasi binaries
# in smoke tests.
#
# Source this file from test.sh scripts to get:
#   cvc_wasm_cc      — cross-compile a C source file to a wasm binary
#   cvc_wasm_run     — execute a wasm binary with the appropriate runtime
#   CVC_WASM_RUNNER  — "node" (emscripten) or "wasmtime" (wasi)
#
# Expects CVC_PLATFORM to be set ("wasm" or "wasi").
# For wasm: CVC_EMSDK_DIR must point to the activated emsdk prefix.
# For wasi: CVC_WASI_SDK_DIR must point to the wasi-sdk prefix.

set -euo pipefail

CVC_WASM_RUNNER=""

if [[ "${CVC_PLATFORM:-}" == "wasm" ]]; then
    # ── Emscripten (wasm) ──────────────────────────────────────────
    : "${CVC_EMSDK_DIR:?CVC_EMSDK_DIR must be set for wasm tests}"

    # Activate emscripten environment (adds emcc, node to PATH).
    # shellcheck disable=SC1091
    source "${CVC_EMSDK_DIR}/emsdk_env.sh" 2>/dev/null

    if ! command -v emcc >/dev/null 2>&1; then
        echo "WARN: emcc not found after sourcing emsdk_env.sh, skipping wasm test" >&2
        CVC_WASM_RUNNER="skip"
    elif ! command -v node >/dev/null 2>&1; then
        echo "WARN: node not found in emsdk, skipping wasm test" >&2
        CVC_WASM_RUNNER="skip"
    else
        CVC_WASM_RUNNER="node"
    fi

    # Cross-compile a C file using emcc.
    # Usage: cvc_wasm_cc output.js input.c [extra flags...]
    cvc_wasm_cc() {
        local out="$1"; shift
        local src="$1"; shift
        emcc -o "$out" "$src" \
            -I"${CVC_INSTALL_DIR}/include" \
            -L"${CVC_INSTALL_DIR}/lib" \
            "$@"
    }

    # Run a wasm program compiled with emscripten.
    # Usage: cvc_wasm_run program.js [args...]
    cvc_wasm_run() {
        node "$@"
    }

elif [[ "${CVC_PLATFORM:-}" == "wasi" ]]; then
    # ── WASI ───────────────────────────────────────────────────────
    : "${CVC_WASI_SDK_DIR:?CVC_WASI_SDK_DIR must be set for wasi tests}"

    _WASI_CC="${CVC_WASI_SDK_DIR}/bin/clang"
    _WASI_SYSROOT="${CVC_WASI_SDK_DIR}/share/wasi-sysroot"

    # Look for a wasm runtime: wasmtime, wasmer, or wasm3.
    if command -v wasmtime >/dev/null 2>&1; then
        CVC_WASM_RUNNER="wasmtime"
    elif [[ -x "${CVC_DEPS_PREFIX:-}/bin/wasmtime" ]]; then
        CVC_WASM_RUNNER="wasmtime"
        export PATH="${CVC_DEPS_PREFIX}/bin:${PATH}"
    elif command -v wasmer >/dev/null 2>&1; then
        CVC_WASM_RUNNER="wasmer"
    elif command -v wasm3 >/dev/null 2>&1; then
        CVC_WASM_RUNNER="wasm3"
    else
        echo "WARN: no wasi runtime found (wasmtime/wasmer/wasm3), skipping wasi test" >&2
        CVC_WASM_RUNNER="skip"
    fi

    # Cross-compile a C file using wasi-sdk clang.
    # Usage: cvc_wasm_cc output.wasm input.c [extra flags...]
    cvc_wasm_cc() {
        local out="$1"; shift
        local src="$1"; shift
        "${_WASI_CC}" --target=wasm32-wasip1 \
            --sysroot="${_WASI_SYSROOT}" \
            -o "$out" "$src" \
            -I"${CVC_INSTALL_DIR}/include" \
            -L"${CVC_INSTALL_DIR}/lib" \
            "$@"
    }

    # Run a wasi binary with the discovered runtime.
    # Usage: cvc_wasm_run program.wasm [args...]
    cvc_wasm_run() {
        case "${CVC_WASM_RUNNER}" in
            wasmtime) wasmtime run "$@" ;;
            wasmer)   wasmer run "$@" ;;
            wasm3)    wasm3 "$@" ;;
            *)        echo "ERROR: no wasi runner"; return 1 ;;
        esac
    }
else
    # Not a wasm/wasi platform — provide stubs so sourcing this file
    # is harmless in native test scripts.
    cvc_wasm_cc()  { echo "cvc_wasm_cc: not a wasm platform" >&2; return 1; }
    cvc_wasm_run() { echo "cvc_wasm_run: not a wasm platform" >&2; return 1; }
fi
