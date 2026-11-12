#!/usr/bin/env bash
# file_pick_upload_test.sh ARI DOC BACKEND
#
# Pick a file and send it, on a real terminal: DOC (file_pick_upload.ari) runs with `ari run
# --backend BACKEND` (ftxui or tvision) in a 100x30 tmux pane, as a person would run it, against
# upload_probe_server.py. Enter presses Attach; the Runtime's "Open file" window opens (a tvision
# window on the tvision desktop) and lists the folder; the file's name typed and Enter pick it;
# the file then goes in 1 KiB chunks. The window must say the server confirmed every byte, and the
# server must hold the file: its size and sha256. Exits 77 (skipped) without tmux, python3 or the
# backend in this ari.
set -euo pipefail
ari=$1 doc=$2 backend=$3
here=$(cd "$(dirname "$0")" && pwd)

command -v tmux >/dev/null || { echo "no tmux: skipped"; exit 77; }
command -v python3 >/dev/null || { echo "no python3: skipped"; exit 77; }
info=$("$ari" info 2>&1 || true)
case $backend in
  ftxui) want="terminal backend (FTXUI) *yes" ;;
  tvision) want="desktop backend (tvision) *yes" ;;
  *) echo "backend is ftxui or tvision, not $backend"; exit 2 ;;
esac
grep -q "$want" <<<"$info" || { echo "ari has no $backend backend: skipped"; exit 77; }

export LC_ALL=C.UTF-8 LANG=C.UTF-8
work=$(mktemp -d)
sock="ari-test-$$" # a tmux server of its own, never the user's
srv=
cleanup() {
  tmux -L "$sock" kill-server 2>/dev/null || true
  [ -n "$srv" ] && kill "$srv" 2>/dev/null || true
  rm -rf "$work"
}
trap cleanup EXIT

head -c 5000 /dev/urandom >"$work/photo.bin" # five chunks, the last one short
python3 "$here/upload_probe_server.py" >"$work/port" &
srv=$!
for _ in $(seq 1 100); do
  [ -s "$work/port" ] && break
  sleep 0.05
done
port=$(head -n 1 "$work/port")
[ -n "$port" ] || { echo "FAIL: the probe server did not start"; exit 1; }
base="http://127.0.0.1:$port"

# From the file's folder, so the Open file window starts there.
tmux -L "$sock" -f /dev/null new-session -d -s ari -x 100 -y 30 \
  "cd '$work' && TERM=xterm-256color exec '$ari' run --backend $backend --net-allow $base '$doc' --url $base/up"

screen() { tmux -L "$sock" capture-pane -p -t ari; }
# Wait (up to 10 s) until the screen shows TEXT.
wait_for() {
  for _ in $(seq 1 100); do
    if screen | grep -qF -- "$1"; then
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

wait_for "Nothing picked." || fail "the document never showed"
tmux -L "$sock" send-keys -t ari Enter
wait_for "Attach a file" || fail "Attach did not open the Open file window"
wait_for "photo.bin" || fail "the Open file window does not list the folder"
tmux -L "$sock" send-keys -t ari -l "photo.bin"
tmux -L "$sock" send-keys -t ari Enter
wait_for "Sent 5000 of 5000 bytes." || fail "the upload did not finish"

got=$(python3 -c 'import sys, urllib.request; print(urllib.request.urlopen(sys.argv[1]).read().decode())' "$base/up")
want="5000 $(sha256sum "$work/photo.bin" | cut -d' ' -f1)"
[ "$got" = "$want" ] || fail "the server holds '$got', not '$want'"
echo "ari run --backend $backend: picked photo.bin and sent it in chunks ($got)"
