#!/usr/bin/env bash
# Sign-in through the hosts as a person runs them (docs/ARIADNE.md §10.5): ari run (FTXUI and
# tvision) and arigl run --terminal in tmux, and arigl's window snapshot, against
# stub_oidc_provider.py. Each host signs in twice with tests/oidc_probe.ari: with a device code
# (--no-browser), approved through the stub as a person would on another device, and through the
# loopback redirect with fake_browser.py as $BROWSER. Every screen, stderr and stdout is then
# checked for the tokens, codes and device codes the provider issued: none may appear.
#
#   oidc_hosts_test.sh ARI [ARIGL]
#
# Exits 77 (skipped) without tmux or python3.
set -u
ARI=$1
ARIGL=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
command -v tmux >/dev/null 2>&1 || { echo "no tmux: skipped"; exit 77; }
command -v python3 >/dev/null 2>&1 || { echo "no python3: skipped"; exit 77; }

WORK=$(mktemp -d)
SOCK="oidc-test-$$"
STUB=""
cleanup() {
  tmux -L "$SOCK" kill-server >/dev/null 2>&1
  [ -n "$STUB" ] && kill "$STUB" >/dev/null 2>&1
  rm -rf "$WORK"
}
trap cleanup EXIT

python3 "$HERE/stub_oidc_provider.py" --secrets "$WORK/secrets" > "$WORK/port" &
STUB=$!
for _ in $(seq 50); do [ -s "$WORK/port" ] && break; sleep 0.1; done
ISS="http://127.0.0.1:$(cat "$WORK/port")"
PROBE="$HERE/oidc_probe.ari --issuer $ISS --api $ISS/api/me"
failed=0
fail() { echo "FAIL: $*"; failed=1; }

# Run `cmd` in a fresh tmux window; wait until the screen shows `want` (at most `secs`).
screen() { tmux -L "$SOCK" capture-pane -p -t oidc 2>/dev/null; }
wait_for() {
  local want=$1 secs=$2
  for _ in $(seq $((secs * 10))); do
    screen | grep -qF "$want" && return 0
    sleep 0.1
  done
  return 1
}

# run_host NAME FLOW CMD...: one sign-in in tmux, its screen and stderr kept in $WORK/NAME.FLOW.*
run_host() {
  local name=$1 flow=$2
  shift 2
  local out="$WORK/$name.$flow"
  local env="BROWSER="
  local flags="--no-browser"
  if [ "$flow" = loopback ]; then
    env="BROWSER='python3 $HERE/fake_browser.py' FAKE_BROWSER_LOG=$out.browser"
    flags=""
  fi
  tmux -L "$SOCK" kill-server >/dev/null 2>&1
  tmux -L "$SOCK" new-session -d -s oidc -x 120 -y 30 \
    "env $env $* $flags --oidc-issuer $ISS $PROBE 2>$out.stderr; sleep 30"
  if [ "$flow" = device ]; then
    if wait_for "code: WDJB-" 20; then
      code=$(screen | grep -o "WDJB-[0-9]*" | head -1)
      python3 -c "import urllib.request,sys; urllib.request.urlopen(sys.argv[1]).read()" \
        "$ISS/approve?user_code=$code" || fail "$name: approving $code"
    else
      fail "$name $flow: no device code shown"
    fi
  fi
  wait_for "api: 200" 30 || fail "$name $flow: the sign-in did not reach the API"
  screen > "$out.screen"
  screen | grep -qF "name: Ada Lovelace" || fail "$name $flow: no claims shown"
  screen | grep -qF "flow: $flow" || fail "$name $flow: the flow was not $flow"
  tmux -L "$SOCK" send-keys -t oidc C-c C-c C-c
  sleep 0.5
  screen >> "$out.screen"
  tmux -L "$SOCK" kill-server >/dev/null 2>&1
  echo "ok: $name $flow"
}

# The hosts this build has: tvision is optional, and arigl --terminal needs a GL it can open.
tvision=0
"$ARI" info | grep -q "desktop backend (tvision)  *yes" && tvision=1
arigl_term=0
if [ -n "$ARIGL" ] && [ -x "$ARIGL" ] && "$ARIGL" info --terminal >/dev/null 2>&1; then
  arigl_term=1
fi
for flow in device loopback; do
  run_host ari-ftxui "$flow" "$ARI" run
  [ $tvision = 1 ] && run_host ari-tvision "$flow" "$ARI" run --backend tvision
  [ $arigl_term = 1 ] && run_host arigl-terminal "$flow" "$ARIGL" run --terminal
done

# arigl in a window (a snapshot, the PNG written after the sign-in), where it has a display.
if [ -n "$ARIGL" ] && [ -x "$ARIGL" ] && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
  env BROWSER="python3 $HERE/fake_browser.py" "$ARIGL" snapshot --png "$WORK/window.png" \
    --oidc-issuer "$ISS" $PROBE > "$WORK/arigl-window.stdout" 2> "$WORK/arigl-window.stderr" \
    || fail "arigl window snapshot exited $?"
  [ -s "$WORK/window.png" ] || fail "arigl window: no PNG"
  echo "ok: arigl window"
fi

# No token, code or device code the provider issued may be in any output.
n=0
while IFS= read -r secret; do
  [ -z "$secret" ] && continue
  n=$((n + 1))
  if grep -rlF -- "$secret" "$WORK" --include='*.screen' --include='*.stderr' \
       --include='*.stdout' --include='*.browser' >/dev/null 2>&1; then
    fail "a secret (${secret:0:10}...) reached a host's output: $(grep -rlF -- "$secret" "$WORK" --include='*.screen' --include='*.stderr' --include='*.stdout')"
  fi
done < "$WORK/secrets"
echo "checked $n secrets against every screen, stdout and stderr"
[ "$n" -gt 0 ] || fail "the provider issued nothing"
exit $failed
