#!/usr/bin/env bash
# media_in_tmux_test.sh ARI DOC BACKEND
#
# Run DOC (media_images.ari) with `ari run --backend BACKEND` (ftxui or tvision) in a 100x30 tmux
# pane and wait for its status line to say every picture by URI went as expected (two ready, one
# failed), then check the pictures really are on the screen: half-block cells inside the window,
# and the window's background beside its last line of text.
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
sock="ari-media-$$"
trap 'tmux -L "$sock" kill-server 2>/dev/null || true' EXIT
tmux -L "$sock" -f /dev/null new-session -d -s ari -x 100 -y 30 \
  "TERM=xterm-256color exec '$ari' run --backend $backend --graphics color '$doc'"

want_line="banner ready avatar ready gone failed"
screen=
for _ in $(seq 1 100); do
  screen=$(tmux -L "$sock" capture-pane -p -t ari 2>/dev/null || true)
  grep -qF -- "$want_line" <<<"$screen" && break
  sleep 0.1
done
if ! grep -qF -- "$want_line" <<<"$screen"; then
  echo "--- no '$want_line' on the $backend screen:"
  echo "$screen"
  exit 1
fi
sleep 0.3
screen=$(tmux -L "$sock" capture-pane -p -t ari)
# The banner alone is 40 x 3 cells of half-blocks; the avatar's corners are transparent, so some
# of its cells are the one-half ▀ / ▄ form.
blocks=$(grep -o '[▀▄]' <<<"$screen" | wc -l)
if [ "$blocks" -lt 130 ]; then
  echo "--- only $blocks half-block cells on the $backend screen:"
  echo "$screen"
  exit 1
fi
if ! grep -q 'background ready▀' <<<"$screen"; then
  echo "--- no background beside the text on the $backend screen:"
  echo "$screen"
  exit 1
fi
echo "ari run --backend $backend: three pictures and a background by URI, $blocks half-block cells"
