#!/usr/bin/env bash
# shell_tmux_test.sh ARI SHELL CATALOG COMPONENTS BACKEND
#
# The Ariadne shell (cvc://components/shell.ari, docs/roadmap/ARIADNE_SHELL.md SH1) on a real
# terminal: `ari run --backend BACKEND` in a 160x40 tmux pane, on CATALOG (shell_apps.json) with
# `--open counter,json_feed`, the component library at COMPONENTS.
#   1. The shell's own Apps menu is in the bar, and so are the two applications it opened at start:
#      Counter's own menu bar folded into one menu named Counter, and JSON feed, which has no bar,
#      as a menu of its own (ARIADNE.md §6.8, §5.25). Tray demo, in the catalog but not opened,
#      has none.
#   2. The JSON feed's window draws its title (the windows overlap, so the Counter's may be under
#      it).
# The AppRuntime stand-in (ariadne_runtime_test's AriadneComponents.ShellOpensItsCatalogOnDemand)
# covers the same document with Close and Open again. Exits 77 (skipped) without tmux or without
# the backend in this ari.
set -euo pipefail
ari=$1 shell=$2 catalog=$3 components=$4 backend=$5

command -v tmux >/dev/null || { echo "no tmux: skipped"; exit 77; }
info=$("$ari" info 2>&1 || true)
case $backend in
  ftxui) want="terminal backend (FTXUI) *yes" ;;
  tvision) want="desktop backend (tvision) *yes" ;;
  *) echo "backend is ftxui or tvision, not $backend"; exit 2 ;;
esac
grep -q "$want" <<<"$info" || { echo "ari has no $backend backend: skipped"; exit 77; }

export LC_ALL=C.UTF-8 LANG=C.UTF-8
sock="ari-test-$$" # a tmux server of its own, never the user's
trap 'tmux -L "$sock" kill-server 2>/dev/null || true' EXIT
tmux -L "$sock" -f /dev/null new-session -d -s ari -x 160 -y 40 \
  "TERM=xterm-256color exec '$ari' run --backend $backend --component-path '$components' '$shell' --catalog '$catalog' --open counter,json_feed"

screen() { tmux -L "$sock" capture-pane -p -t ari; }
# Wait (up to 15 s) until the screen shows TEXT.
wait_for() {
  for _ in $(seq 1 150); do
    if grep -qF -- "$1" <<<"$(screen)"; then
      sleep 0.3
      return 0
    fi
    sleep 0.1
  done
  return 1
}
fail() {
  echo "FAIL: $1"
  echo "--- the $backend screen:"
  screen
  exit 1
}

wait_for "Apps" || fail "the shell's Apps menu never showed"
wait_for "Counter" || fail "the Counter application's menu never joined the bar"
wait_for "JSON feed" || fail "the JSON feed application never got a menu"
bar=$(screen | head -n 1)
grep -qF "Apps" <<<"$bar" || fail "Apps is not in the menu bar: $bar"
grep -qF "Counter" <<<"$bar" || fail "Counter is not in the menu bar: $bar"
grep -qF "JSON feed" <<<"$bar" || fail "JSON feed is not in the menu bar: $bar"
grep -qF "tray demo" <<<"$bar" && fail "Tray demo has a menu, though nothing opened it"
echo "ari run --backend $backend: the shell's bar holds Apps, Counter and JSON feed"

# The feed's window (the windows overlap: the Counter's may be under it, so its text is not
# looked for).
wait_for "tx.wtf/public" || fail "the JSON feed's window never drew its title"
echo "ari run --backend $backend: the JSON feed's window drew"
