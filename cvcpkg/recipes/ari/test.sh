#!/usr/bin/env bash
# cvcpkg/recipes/ari/test.sh -- the built ari must load and run a document: the loader reads it,
# init: runs in the DSL with a command-line argument, and the terminal backend draws the result.
# `cvcpkg build` runs it, so the cvcpkg.org fleet's columns (the BSDs, Haiku) are checked before they
# pack; publish-cvcpkg.yml's own lanes pack from a prefix and render terminal_tour.ari instead.
set -euo pipefail

: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

if [[ -n "${CVC_WINHOST:-}" ]]; then
    # A Windows host's test shell has no console for the terminal backend.
    test -f "${CVC_INSTALL_DIR}/bin/ari.exe" || { echo "FAIL: bin/ari.exe not installed"; exit 1; }
    echo "ari test: OK (files only on a Windows host)"
    exit 0
fi

ari="${CVC_INSTALL_DIR}/bin/ari"
test -x "${ari}" || { echo "FAIL: bin/ari not installed"; exit 1; }

tmp=$(mktemp -d)
trap 'rm -rf "${tmp}"' EXIT
cat > "${tmp}/smoke.ari" <<'ARI'
meta:
  name: cvcpkg ari smoke
  min_libcvc: 4.0.0

requires:
  - ui.windows

args:
  who: {type: string, default: fleet}

init: |
  (state-set "smoke.greeting" (str-concat "hello, " (state-get "args.who")))

windows:
  - window: Smoke
    id: smoke
    pos: [0, 0]
    children:
      - text: "greeting"
        bind: smoke.greeting
ARI

"${ari}" snapshot --width 60 --height 12 "${tmp}/smoke.ari" --who cvcpkg | tee "${tmp}/out.txt"
grep -qF "greeting: hello, cvcpkg" "${tmp}/out.txt" \
    || { echo "FAIL: ari did not run the document's init: and draw its result"; exit 1; }
echo "ari test: OK"
