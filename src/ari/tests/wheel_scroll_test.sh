#!/usr/bin/env bash
# wheel_scroll_test.sh ARI DOC BACKEND
#
# The mouse wheel on a real terminal: DOC (wheel_scroll.ari) runs with `ari run --backend BACKEND`
# (ftxui or tvision) in a 100x30 tmux pane, and the test types the wheel's own reports (SGR mouse,
# buttons 64 and 65) at a cell, as a terminal does for a turn of the wheel there.
#   1. tvision only: "Feed", 20 rows in a window with room for 6, has a scroll bar on its frame.
#   2. Down over Feed, which is not the window in front: Feed scrolls 3 rows. (tvision hands the
#      wheel to every window, front to back, and the window in front used to take it from
#      anywhere: Feed stood still.)
#   3. Down over "Front", which fits: Feed stays where it is.
#   4. Up over Feed: back at its first row.
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
fail() {
  echo "FAIL: $1"
  echo "--- the $backend screen:"
  screen
  exit 1
}
# The first of Feed's rows that shows ("row 3."), or nothing.
top_row() { screen | grep -oE 'row [0-9]+\.' | head -n 1 || true; }
# Wait (up to 10 s) until Feed's first row showing is ROW.
wait_top() {
  for _ in $(seq 1 100); do
    [ "$(top_row)" = "$1" ] && return 0
    sleep 0.1
  done
  return 1
}
# The 1-based column and row of the first cell showing TEXT. Counted in characters by bash (in a
# UTF-8 locale), not bytes: the desktop's ░ and the borders before it are 3 bytes each.
cell_of() {
  local n=0 line prefix
  while IFS= read -r line; do
    n=$((n + 1))
    if [[ $line == *"$1"* ]]; then
      prefix=${line%%"$1"*}
      echo "$((${#prefix} + 1)) $n"
      return 0
    fi
  done < <(screen)
}
# A turn of the wheel at the cell showing TEXT: BUTTON 64 up, 65 down. The report's bytes, typed
# into the pane as the terminal would send them.
wheel() {
  local at col row
  at=$(cell_of "$1")
  [ -n "$at" ] || fail "nothing on the screen shows '$1'"
  read -r col row <<<"$at"
  # shellcheck disable=SC2046 # one argument per byte
  tmux -L "$sock" send-keys -t ari -H $(printf '\033[<%d;%d;%dM' "$2" "$col" "$row" | od -An -tx1)
  sleep 0.3
}

wait_top "row 0." || fail "Feed never showed its first row"
grep -q "in front" <<<"$(screen)" || fail "the Front window never opened"

if [ "$backend" = tvision ]; then
  # tvision's scroll bar glyphs: its arrows at the ends of the frame's right column.
  screen | grep -q '▲' && screen | grep -q '▼' ||
    fail "Feed overflows but has no scroll bar on its frame"
fi

wheel "row 1." 65
wait_top "row 3." || fail "the wheel down over Feed did not scroll it (showing '$(top_row)')"
wheel "in front" 65
sleep 0.3
[ "$(top_row)" = "row 3." ] || fail "the wheel over Front moved Feed (showing '$(top_row)')"
wheel "row 4." 64
wait_top "row 0." || fail "the wheel up over Feed did not scroll it back (showing '$(top_row)')"
echo "ari run --backend $backend: the wheel scrolls the window under it"
