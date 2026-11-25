#!/usr/bin/env bash
# context_menu_test.sh ARI DOC BACKEND
#
# A right-click menu (ARIADNE.md §5.30) on a real terminal: DOC (context_menu.ari) runs with
# `ari run --backend BACKEND` (ftxui or tvision) in a 100x30 tmux pane, driven by its keys.
#   1. Tab focuses the checkbox in the card; Shift+F10 (tmux's S-F10, which the terminal sends as
#      CSI 21;2~) opens the card's menu: both its items show. Enter runs the first, which writes
#      what the menu's event says into the line at the top: "opened group card key". The menu
#      closes.
#   2. Shift+F10 again, Down, Enter: the second item runs.
#   3. Shift+F10, Esc: the menu closes and nothing runs.
# Exits 77 (skipped) without tmux or without the backend in this ari.
set -euo pipefail
ari=$1 doc=$2 backend=$3

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
tmux -L "$sock" -f /dev/null new-session -d -s ari -x 100 -y 30 \
  "TERM=xterm-256color exec '$ari' run --backend $backend '$doc'"

screen() { tmux -L "$sock" capture-pane -p -t ari; }
# Wait (up to 10 s) until the screen shows TEXT (or, with "!" first, until it does not).
wait_for() {
  local not=0 text=$1
  if [ "$text" = "!" ]; then
    not=1
    text=$2
  fi
  for _ in $(seq 1 100); do
    if grep -qF -- "$text" <<<"$(screen)"; then
      [ $not = 0 ] && { sleep 0.3; return 0; }
    else
      [ $not = 1 ] && { sleep 0.3; return 0; }
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
keys() { tmux -L "$sock" send-keys -t ari "$@"; }

wait_for "log: nothing yet" || fail "the document never showed its window"
wait_for "Pinned" || fail "the card never showed"
grep -qF "Open card" <<<"$(screen)" && fail "the card's menu shows before it is opened"

# 1. Tab to the checkbox, Shift+F10: the card's menu. Enter runs its first item.
keys Tab
sleep 0.3
keys S-F10
wait_for "Open card" || fail "Shift+F10 did not open the card's menu"
wait_for "Close card" || fail "the menu does not show its second item"
keys Enter
wait_for "opened group card key" || fail "Enter did not run the first item with the menu's event"
wait_for ! "Open card" || fail "the menu did not close after its item ran"
echo "ari run --backend $backend: Shift+F10 opened the card's menu and Enter ran its first item"

# 2. Again, Down, Enter: the second item.
keys S-F10
wait_for "Open card" || fail "Shift+F10 did not open the menu again"
keys Down
sleep 0.3
keys Enter
wait_for "closed card" || fail "Down then Enter did not run the second item"
echo "ari run --backend $backend: Down then Enter ran the second item"

# 3. Esc closes it with nothing run.
keys S-F10
wait_for "Open card" || fail "Shift+F10 did not open the menu a third time"
keys Escape
wait_for ! "Open card" || fail "Esc did not close the menu"
grep -qF "closed card" <<<"$(screen)" || fail "Esc ran an item"
echo "ari run --backend $backend: Esc closed the menu, nothing ran"
