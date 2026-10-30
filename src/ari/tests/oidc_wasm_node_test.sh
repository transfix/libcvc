#!/usr/bin/env bash
# Sign-in in the browser build (docs/ARIADNE.md §10.4), the wasm ari under node: the page shim
# (page-shim.cjs) stands in for the page's location, sessionStorage and history, and xhr-shim.cjs
# for the browser's XMLHttpRequest. Against stub_oidc_provider.py:
#   1. the redirect flow: the page leaves for the authorization endpoint (the PKCE record in
#      sessionStorage); the provider's redirect is followed by hand; the page loads again at the
#      reply's URL and completes the sign-in, takes code and state out of the address bar, and
#      calls the API with the credential;
#   2. the device flow (flow device): the code is shown.
# No token, code or device code the provider issued may reach the output.
#
#   oidc_wasm_node_test.sh NODE ARI_JS   (ari built with CVC_ARI_WASM_TEST_DOCS=ON)
set -u
NODE=$1
ARI_JS=$2
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
python3 "$HERE/stub_oidc_provider.py" --secrets "$WORK/secrets" > "$WORK/port" &
STUB=$!
trap 'kill $STUB; rm -rf "$WORK"' EXIT
for _ in $(seq 50); do [ -s "$WORK/port" ] && break; sleep 0.1; done
ISS="http://127.0.0.1:$(cat "$WORK/port")"
PAGE="https://app.example/apps/demo/ari.html?src=main.ari"
failed=0
fail() { echo "::error::$*"; failed=1; }
ari() { # ari ARGS... with the page at $ARI_PAGE_URL
  ARI_PAGE_STATE="$WORK/page.json" timeout 120 "$NODE" --require "$HERE/xhr-shim.cjs" \
    --require "$HERE/page-shim.cjs" "$ARI_JS" snapshot --oidc-issuer "$ISS" --width 120 \
    --height 14 /tests/oidc_probe.ari --issuer "$ISS" --api "$ISS/api/me" "$@" 2>&1
}
page() { python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print($1)" "$WORK/page.json"; }

ARI_PAGE_URL="$PAGE" ari > "$WORK/leave.txt"
cat "$WORK/leave.txt"
grep -qF "phase: redirecting" "$WORK/leave.txt" || fail "the page did not leave for the provider"
[ "$(page "len(d['navigated'])")" = 1 ] || fail "the page did not navigate once"
[ "$(page "list(d['session'])")" = "['cvc.oidc.me']" ] || fail "no sign-in record in sessionStorage"
authorize=$(page "d['navigated'][0]")
case "$authorize" in
  "$ISS/authorize?"*code_challenge_method=S256*) ;;
  *) fail "the page went to '$authorize'" ;;
esac
back=$(python3 - "$authorize" <<'PY'
import http.client, sys, urllib.parse
u = urllib.parse.urlsplit(sys.argv[1])
c = http.client.HTTPConnection(u.hostname, u.port)
c.request("GET", u.path + "?" + u.query)
print(c.getresponse().getheader("Location", ""))
PY
)
case "$back" in
  "$PAGE&code="*) ;;
  *) fail "the provider sent the page to '$back'" ;;
esac

ARI_PAGE_URL="$back" ari > "$WORK/back.txt"
cat "$WORK/back.txt"
for want in "phase: signed_in" "flow: redirect" "name: Ada Lovelace" "done: signed_in" "api: 200"; do
  grep -qF "$want" "$WORK/back.txt" || fail "the page that came back is missing '$want'"
done
[ "$(page "d['replaced'][-1] if d['replaced'] else ''")" = "$PAGE" ] \
  || fail "code and state were not taken out of the address bar"
[ "$(page "len(d['session'])")" = 0 ] || fail "the sign-in record stayed in sessionStorage"

rm -f "$WORK/page.json"
ARI_PAGE_URL="$PAGE" ari --flow device > "$WORK/device.txt"
cat "$WORK/device.txt"
grep -qF "phase: device_code_shown" "$WORK/device.txt" || fail "no device code in the browser build"
grep -qF "code: WDJB-" "$WORK/device.txt" || fail "the device code is not shown"

n=0
while IFS= read -r secret; do
  [ -z "$secret" ] && continue
  n=$((n + 1))
  if grep -qF -- "$secret" "$WORK/leave.txt" "$WORK/back.txt" "$WORK/device.txt"; then
    fail "a secret (${secret:0:10}...) reached the wasm ari's output"
  fi
done < "$WORK/secrets"
echo "checked $n secrets"
exit $failed
