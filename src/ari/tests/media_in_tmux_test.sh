#!/usr/bin/env bash
# media_in_tmux_test.sh ARI DOC BACKEND WANT MIN_BLOCKS [BESIDE]
#
# Run DOC with `ari run --backend BACKEND` (ftxui or tvision) in a 100x30 tmux pane, wait until
# the screen shows the line WANT (a document's line saying how its pictures went), then check the
# pictures really are on the screen: at least MIN_BLOCKS half-block cells, and, when BESIDE is
# given, that text on the screen (a background's picture cells beside a line of text: "line▀").
# ARI_FLAGS (environment) goes before the document (--component-path, say). Exits 77 (skipped)
# without tmux or without the backend in this ari.
set -euo pipefail
ari=$1 doc=$2 backend=$3 want_line=$4 min_blocks=$5 beside=${6:-}

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
  "TERM=xterm-256color exec '$ari' run --backend $backend --graphics color ${ARI_FLAGS:-} '$doc'"

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
blocks=$(grep -o '[▀▄]' <<<"$screen" | wc -l)
if [ "$blocks" -lt "$min_blocks" ]; then
  echo "--- only $blocks half-block cells (want $min_blocks) on the $backend screen:"
  echo "$screen"
  exit 1
fi
if [ -n "$beside" ] && ! grep -qF -- "$beside" <<<"$screen"; then
  echo "--- no '$beside' on the $backend screen:"
  echo "$screen"
  exit 1
fi
echo "ari run --backend $backend: '$want_line' with $blocks half-block cells"
