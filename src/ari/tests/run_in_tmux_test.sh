#!/usr/bin/env bash
# run_in_tmux_test.sh ARI DOC BACKEND ROWS
#
# Run DOC with `ari run --backend BACKEND` (ftxui or tvision) in a 100x30 tmux pane, the way a
# person would, and compare the text inside its window's borders with ROWS: one row per line,
# each ending in '|' where the right border is, separator rows left out. `ari snapshot` always
# lays out with FTXUI; this checks what each backend really puts on a terminal, wide glyphs (CJK,
# emoji) included. Exits 77 (skipped) without tmux or without the backend in this ari.
set -euo pipefail
ari=$1 doc=$2 backend=$3 rows=$4

command -v tmux >/dev/null || { echo "no tmux: skipped"; exit 77; }
info=$("$ari" info 2>&1 || true)
case $backend in
  ftxui) want="terminal backend (FTXUI) *yes" ;;
  tvision) want="desktop backend (tvision) *yes" ;;
  *) echo "backend is ftxui or tvision, not $backend"; exit 2 ;;
esac
grep -q "$want" <<<"$info" || { echo "ari has no $backend backend: skipped"; exit 77; }

export LC_ALL=C.UTF-8 LANG=C.UTF-8 # a wide glyph is two columns only in a UTF-8 locale
sock="ari-test-$$"                 # a tmux server of its own, never the user's
trap 'tmux -L "$sock" kill-server 2>/dev/null || true' EXIT
tmux -L "$sock" -f /dev/null new-session -d -s ari -x 100 -y 30 \
  "TERM=xterm-256color exec '$ari' run --backend $backend '$doc'"

# The rows inside the borders (│ for FTXUI, ║ for tvision), separators left out.
inside() {
  sed -n 's/^ *\(│\|║\)\(.*\)\(│\|║\).*$/\2|/p' | grep -v '^─*|$' || true
}
last=$(tail -n 1 "$rows")
screen=
for _ in $(seq 1 100); do
  screen=$(tmux -L "$sock" capture-pane -p -t ari 2>/dev/null || true)
  if inside <<<"$screen" | grep -qxF -- "$last"; then
    break
  fi
  sleep 0.1
done
sleep 0.3 # let the frame settle, then take it
screen=$(tmux -L "$sock" capture-pane -p -t ari)
if ! diff <(inside <<<"$screen") "$rows"; then
  echo "--- the $backend screen:"
  echo "$screen"
  exit 1
fi
echo "ari run --backend $backend: $(wc -l <"$rows") rows as expected"
