#!/usr/bin/env bash
# cvcpkg/recipes/arisrv/test.sh -- the built relay must open a room on loopback with its gRPC and
# WebSocket planes, chat, Jolt avatars and the libjpeg-turbo video rungs (a stub would log none of
# them), and stop cleanly on SIGTERM. `cvcpkg build` runs it, so the cvcpkg.org fleet's BSD columns
# are checked before they pack; publish-cvcpkg.yml's lanes run the same checks on their own builds.
set -euo pipefail

: "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

if [[ -n "${CVC_WINHOST:-}" ]]; then
    test -f "${CVC_INSTALL_DIR}/bin/arisrv.exe" || { echo "FAIL: bin/arisrv.exe not installed"; exit 1; }
    echo "arisrv test: OK (files only on a Windows host)"
    exit 0
fi

arisrv="${CVC_INSTALL_DIR}/bin/arisrv"
test -x "${arisrv}" || { echo "FAIL: bin/arisrv not installed"; exit 1; }

tmp=$(mktemp -d)
trap 'rm -rf "${tmp}"' EXIT
# Ports of this job's own, so two builds on one builder don't collide.
base=$((20000 + $$ % 20000))
state=$((base)) media=$((base + 1)) ws=$((base + 2))

"${arisrv}" --room smoke --state-listen "127.0.0.1:${state}" --media-listen "127.0.0.1:${media}" \
    --ws-listen "127.0.0.1:${ws}" > "${tmp}/arisrv.log" 2>&1 &
pid=$!
# Up to 30 s for the relay to say it is up, then let it run a moment and stop it.
for ((_i = 0; _i < 60; _i++)); do
    grep -q "relay up" "${tmp}/arisrv.log" 2>/dev/null && break
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.5
done
sleep 2
kill -TERM "${pid}" 2>/dev/null || true
rc=0
wait "${pid}" || rc=$?
cat "${tmp}/arisrv.log"
[[ "${rc}" == 0 ]] || { echo "FAIL: arisrv exited ${rc} on SIGTERM"; exit 1; }
grep -qE "relay up .*state=[^ ]*:${state} " "${tmp}/arisrv.log" \
    || { echo "FAIL: arisrv did not open its gRPC room on 127.0.0.1:${state}"; exit 1; }
grep -qE "websocket \(browser\) clients: ws=[^ ]*:${ws}" "${tmp}/arisrv.log" \
    || { echo "FAIL: arisrv did not open its WebSocket on 127.0.0.1:${ws}"; exit 1; }
for want in "chat: room" "avatar physics: Jolt rigid bodies" "transcode libjpeg-turbo"; do
    grep -qF "${want}" "${tmp}/arisrv.log" || { echo "FAIL: arisrv did not log '${want}'"; exit 1; }
done
echo "arisrv test: OK"
