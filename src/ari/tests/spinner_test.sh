#!/usr/bin/env bash
# spinner_test.sh ARI DOC
#
# A spinner turns by the host's frame clock while the document's program is parked (DOC is
# tests/spinner.ari): `ari snapshot` frame 1 against frame 3 (a step apart at 20 fps), then, where
# tmux is, `ari run` under each terminal backend this ari has, its screen taken twice 0.4 s apart.
set -euo pipefail
ari=$1 doc=$2
export LC_ALL=C.UTF-8 LANG=C.UTF-8 TERM=xterm-256color

# The glyph before LABEL on the screen read from stdin.
glyph() { sed -n "s/.*\\(.\\) $1.*/\\1/p" | head -n 1; }

one=$("$ari" snapshot --width 40 --height 10 --frames 1 "$doc")
three=$("$ari" snapshot --width 40 --height 10 --frames 3 "$doc")
for label in Loading Plain; do
  a=$(glyph "$label" <<<"$one") b=$(glyph "$label" <<<"$three")
  if [ -z "$a" ] || [ "$a" = "$b" ]; then
    echo "snapshot: the $label spinner did not step ('$a' then '$b')"; echo "$one"; echo "$three"; exit 1
  fi
  echo "snapshot: $label $a -> $b"
done
grep -qF "| Plain" <<<"$one" || { echo "snapshot: glyphs: ascii starts at |"; echo "$one"; exit 1; }
grep -qF "parked: yes" <<<"$three" || { echo "snapshot: the program is not parked"; echo "$three"; exit 1; }
! grep -qF "Hidden" <<<"$one" || { echo "snapshot: visible_when did not hide one"; exit 1; }

command -v tmux >/dev/null || { echo "no tmux: ari run not checked"; exit 0; }
info=$("$ari" info 2>&1 || true)
for backend in ftxui tvision; do
  case $backend in
    ftxui) want="terminal backend (FTXUI) *yes" ;;
    tvision) want="desktop backend (tvision) *yes" ;;
  esac
  grep -q "$want" <<<"$info" || { echo "ari has no $backend backend: not checked"; continue; }
  sock="ari-spinner-$$-$backend" # a tmux server of its own, never the user's
  trap 'tmux -L "$sock" kill-server 2>/dev/null || true' EXIT
  tmux -L "$sock" -f /dev/null new-session -d -s ari -x 80 -y 20 \
    "TERM=xterm-256color exec '$ari' run --backend $backend '$doc'"
  first=
  for _ in $(seq 1 100); do
    first=$(tmux -L "$sock" capture-pane -p -t ari 2>/dev/null | glyph Loading || true)
    [ -n "$first" ] && break
    sleep 0.1
  done
  sleep 0.4
  screen=$(tmux -L "$sock" capture-pane -p -t ari)
  second=$(glyph Loading <<<"$screen")
  tmux -L "$sock" kill-server 2>/dev/null || true
  if [ -z "$first" ] || [ "$first" = "$second" ]; then
    echo "ari run --backend $backend: the spinner did not turn ('$first' then '$second')"
    echo "$screen"; exit 1
  fi
  grep -qF "parked: yes" <<<"$screen" || { echo "$backend: the program is not parked"; echo "$screen"; exit 1; }
  echo "ari run --backend $backend: $first -> $second"
done
