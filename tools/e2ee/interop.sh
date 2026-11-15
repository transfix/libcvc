#!/usr/bin/env bash
# tools/e2ee/interop.sh -- a live round trip between cvc::net's E2eeStore and TX.WTF's own client:
# C++ seals three messages, e2ee.js opens them and replies twice, C++ opens the replies.
#
#   tools/e2ee/interop.sh <build dir with bin/e2ee_test> <txwtf checkout>
#
# Needs node, and libsodium-wrappers-sumo 0.7.15 (the build the TX.WTF page loads) where node finds
# it: next to these scripts (npm install --prefix tools/e2ee libsodium-wrappers-sumo@0.7.15) or on
# NODE_PATH. The vectors the unit tests replay come from txwtf_vectors.js; this is the same
# exchange with the system's randomness, end to end.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
build=${1:?usage: interop.sh BUILD_DIR TXWTF_CHECKOUT}
txwtf=${2:?usage: interop.sh BUILD_DIR TXWTF_CHECKOUT}
test_bin=$build/bin/e2ee_test
[ -x "$test_bin" ] || test_bin=$build/bin/e2ee_test.exe
dir=$(mktemp -d "${TMPDIR:-/tmp}/cvc-e2ee-interop.XXXXXX")
trap 'rm -rf "$dir"' EXIT
export CVC_E2EE_INTEROP_DIR=$dir
echo "== 1. C++ seals for jill"
"$test_bin" --gtest_filter=E2eeInterop.* | grep -E '^(wrote|\[  (PASSED|FAILED)|\[       OK)'
echo "== 2. e2ee.js opens and replies"
node "$here/txwtf_interop.js" "$txwtf/txwtf/static/js" "$dir"
echo "== 3. C++ opens e2ee.js's replies"
"$test_bin" --gtest_filter=E2eeInterop.* | grep -E '^(opened|\[  (PASSED|FAILED)|\[       OK)'
