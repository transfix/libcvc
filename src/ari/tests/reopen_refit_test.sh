#!/usr/bin/env bash
# reopen_refit_test.sh ARI DOC BACKEND
#
# Windows that close and open again in one run, on a real terminal: DOC (reopen_refit.ari) runs
# with `ari run --backend BACKEND` (ftxui or tvision) in a 100x30 tmux pane, driven by its keys.
#   1. o signs out: "Sign in" and "Notes" close, and each gains a row while closed. i signs in
#      again: both must show that row, so each window fits what it shows now, not what it showed
#      when it first opened (ARIADNE.md §5.8).
#   2. tvision only: Ctrl-F5 and Shift-arrows resize "Sign in" from the keyboard, as a user
#      would. After o and i it must come back at the user's size, not refit.
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
# The rows inside window TITLE's borders, each ending in '|' where its right border is: the rows
# below its top border (the one with the title and a corner) down to its bottom border. The
# windows of DOC stand in rows of their own, so a row holds one window. Borders: │ ┃ for FTXUI,
# ║ │ for tvision (the active window's and the others').
body() {
  awk -v t=" $1 " '
    !on && index($0, t) && ($0 ~ /╭|┏|╔|┌/) { on = 1; next }
    on && $0 ~ /^[░ ]*(│|║|┃)/ { print; next }
    on { exit }
  ' | sed 's/^[░ ]*\(│\|║\|┃\)\(.*\)\(│\|║\|┃\)[░ ]*$/\2|/'
}
# Wait (up to 10 s) until window TITLE's rows include ROW (or, with "!", until no window TITLE).
wait_for() {
  local title=$1 row=$2
  for _ in $(seq 1 100); do
    local b
    b=$(screen | body "$title")
    if [ "$row" = "!" ]; then
      [ -z "$b" ] && return 0
    elif grep -qF -- "$row" <<<"$b"; then
      sleep 0.3 # let the frame settle
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
expect_body() { # TITLE EXPECTED-ROWS
  local got
  got=$(screen | body "$1")
  if [ "$got" != "$2" ]; then
    diff <(echo "$2") <(echo "$got") || true
    fail "window '$1' (expected, then what it shows)"
  fi
}

wait_for "Sign in" "Password: ********" || fail "the sign-in window never opened"
wait_for "Notes" "Nothing new." || fail "the notes window never opened"

# 1. Sign out, then in: the rows that appeared while the windows were closed show.
tmux -L "$sock" send-keys -t ari o
wait_for "Sign in" "!" || fail "o did not close the sign-in window"
tmux -L "$sock" send-keys -t ari i
wait_for "Sign in" "Signed out." || fail "the reopened sign-in window does not show its new row"
expect_body "Sign in" "Username: grace             |
Password: ********          |
Signed out.                 |"
expect_body "Notes" "Welcome.                      |
Nothing new.                  |
You signed out; sign in again.|"
echo "ari run --backend $backend: the reopened windows fit their new rows"

[ "$backend" = tvision ] || exit 0

# 2. The user resizes "Sign in" (selected: tvision selects the window it opened last), from the
# keyboard: Ctrl-F5, Shift-arrows grow it, Return keeps it.
fitted=$(screen | body "Sign in")
tmux -L "$sock" send-keys -t ari C-F5
sleep 0.3
tmux -L "$sock" send-keys -t ari S-Right S-Right S-Right S-Right S-Down
sleep 0.3
tmux -L "$sock" send-keys -t ari Enter
sleep 0.5
resized=$(screen | body "Sign in")
[ "$(wc -l <<<"$resized")" -gt "$(wc -l <<<"$fitted")" ] || fail "Ctrl-F5 / Shift-Down did not resize the window"
[ "$(head -n 1 <<<"$resized" | wc -m)" -gt "$(head -n 1 <<<"$fitted" | wc -m)" ] ||
  fail "Ctrl-F5 / Shift-Right did not resize the window"

tmux -L "$sock" send-keys -t ari o
wait_for "Sign in" "!" || fail "o did not close the resized sign-in window"
tmux -L "$sock" send-keys -t ari i
wait_for "Sign in" "Signed out." || fail "the resized sign-in window did not open again"
expect_body "Sign in" "$resized"
expect_body "Notes" "Welcome.                      |
Nothing new.                  |
You signed out; sign in again.|"
echo "ari run --backend $backend: the window the user resized opened again at their size"
