#!/usr/bin/env bash
# The browser gallery build moved to examples/web/build-wasm-demo.sh with the rest of the
# examples. This forwarder keeps the old path working for one release, so callers outside this
# repo (cy-pca/cyberpcangel-cvcgl's nightly) can move over. Its output directories
# (build-wasm/, build-wasm-mt/) are unchanged.
set -euo pipefail
echo "note: src/cvcGL/examples/wasm/build-wasm-demo.sh moved to examples/web/build-wasm-demo.sh" >&2
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)/examples/web/build-wasm-demo.sh" "$@"
