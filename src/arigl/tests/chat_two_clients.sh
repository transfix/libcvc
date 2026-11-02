#!/usr/bin/env bash
# chat_two_clients.sh — room chat and direct messages end to end through a local relay: arisrv,
# then scripted arigl clients (chat_driver.ari) joined to it over gRPC, each offscreen with the
# shipped Chat and Users windows:
#
#   alice  posts CHAT_TEST_POSTS room messages, then messages bob directly and waits for his reply
#   bob    joins after alice has posted them all, pages back through every one, answers alice's
#          direct message and posts "done"
#   carol  joins with bob and, once she sees "done", checks she was pushed no direct message
#
#   ARISRV=build/bin/arisrv ARIGL=build/bin/arigl src/arigl/tests/chat_two_clients.sh
#
# Exits 0 when all three report OK. Each client's log and last frame (a PNG) are left in
# CHAT_TEST_OUT (a fresh temporary directory by default), with the relay's log.
set -u
here=$(cd "$(dirname "$0")" && pwd)
src=$(cd "$here/../.." && pwd) # the repository's src/
arisrv=${ARISRV:-arisrv}
arigl=${ARIGL:-arigl}
port=${CHAT_TEST_PORT:-27760}
posts=${CHAT_TEST_POSTS:-40}
out=${CHAT_TEST_OUT:-$(mktemp -d)}
mkdir -p "$out"
relay="grpc://127.0.0.1:$port"

CVC_ARISRV_ROOM_DEBUG=1 "$arisrv" --room chatroom --state-listen "127.0.0.1:$port" \
  --media-listen "127.0.0.1:$((port + 1))" --ws-listen "127.0.0.1:$((port + 2))" \
  >"$out/arisrv.log" 2>&1 &
srv=$!
trap 'kill $srv 2>/dev/null' EXIT
for _ in $(seq 50); do
  grep -q "relay up" "$out/arisrv.log" 2>/dev/null && break
  sleep 0.2
done

client() { # NAME ROLE PEER
  CVC_ARIADNE_RELAY=$relay CVC_ARIADNE_ROOM=chatroom CVC_ARIADNE_NAME=$1 CVC_ARIADNE_SYNTHETIC_CAM=1 \
    "$arigl" snapshot --component-path "$src/cvc/ariadne" --png "$out/$1.png" --frames 9000 \
    "$here/chat_driver.ari" --role "$2" --peer "$3" --posts "$posts" >"$out/$1.log" 2>&1
}

client alice poster bob &
a=$!
# bob and carol join once the relay has taken all of alice's posts (its room-debug chat lines count
# them every ~5 s; a client's own prints reach its log only when it exits): bob is a late joiner.
for _ in $(seq 240); do
  taken=$(grep -h '^\[arisrv\] chat:' "$out/arisrv.log" 2>/dev/null | awk '{n += $3} END {print n + 0}')
  [ "$taken" -ge "$posts" ] && break
  kill -0 $a 2>/dev/null || break
  sleep 0.5
done
client carol outsider bob &
c=$!
client bob reader alice &
b=$!
wait $a
ra=$?
wait $b
rb=$?
wait $c
rc=$?
grep -h "DRIVER" "$out/alice.log" "$out/bob.log" "$out/carol.log"
grep -h "chat:" "$out/arisrv.log" | tail -4
echo "alice $ra, bob $rb, carol $rc (logs and frames in $out)"
[ $ra -eq 0 ] && [ $rb -eq 0 ] && [ $rc -eq 0 ]
