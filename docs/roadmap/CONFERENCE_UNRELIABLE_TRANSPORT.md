# Conference fast lane — an unreliable, unordered transport for poses and media

**Goal.** The conference should feel like QuakeWorld: a lost packet is simply superseded by the
next one, and nothing ever waits behind it. That should hold while it also streams camera video
and microphone audio.

**The problem today.** Everything rides TCP.
- Native clients use the gRPC `Channel` stream (port 7700).
- Browsers use one WebSocket (`/cyprty-ws` → 7702).
- Each carries the `Frame` oneof: tree mutations, out-of-band messages, `MediaFrame`s.

A lost TCP segment holds up every byte behind it, and that includes a 6 KB camera frame sitting in
front of the next pose. The tail of pose latency, and with it the stutter of every peer's avatar,
is set by TCP loss recovery.

**Decision.** Each client gets a **fast lane** beside its reliable connection: WebRTC data
channels to the relay, unordered and never waited for. It is in fact **two lanes**, each its own
peer connection on the relay's one UDP port:
- a **pose lane** (`maxRetransmits: 0`) that carries only pose snapshots and their acks. The
  snapshots are the conference's own pose stream (`pose_stream.h`, the `PoseSnapshot` the reliable
  transports carry), in an *acked* mode: each differs from the last one the viewer acknowledged;
- a **media lane** (lifetime 100 ms) that carries camera and mic frames. It is off by default:
  `arisrv --lane-media on` turns it on.

On one SCTP association the poses would share the video's congestion window, and under loss they
queue behind it at the sender (§7.3).

- **Libraries.** Native clients use **libdatachannel 0.24.6** (libjuice ICE, usrsctp, OpenSSL DTLS).
  The browser uses **datachannel-wasm** (the same `rtc::` API over the page's own
  `RTCPeerConnection`). Both are cvcpkg recipes.
- **What moves to the lane.** Pose snapshots (`room_pose_relay`'s per-viewer stream of the relay
  holon's authoritative poses), and the camera/mic frames when the media lane is on.
- **What stays on the reliable connection.** The room tree: roster, scene, intents, chat, link
  stats, AOI parameters, and the lanes' own signaling.
- **Fallback.** A client whose lane never opens (UDP blocked, an old relay, a build without
  WebRTC) stays entirely on its reliable connection, as today.

**What the spike measured (§7).**
- **Poses.** The pose lane does what was asked. These are the conference's 60 Hz pose snapshots,
  for 2 clients with camera and mic, under 1 % loss, 3 % loss with jitter, and bursty loss, over
  3 runs each:
  - a pose reaches its viewer in one-way delay plus jitter at p99: **21–25 ms**, against 71–139 ms
    over WebSocket, and 74–98 ms over gRPC at random loss (in bursts gRPC can stall for seconds
    whatever carries the poses, §7.4);
  - the newest pose a viewer holds is at most **51–62 ms** old at p99, against 84–141 ms over TCP
    at random loss, and up to 668 ms (WebSocket) or 7.8 s (gRPC) in bursts;
  - a client draws a walking peer 75–77 ms behind the server at p50, instead of 79–111 ms;
  - it costs about 13 kb/s per viewer for this room.
- **Corrections** of a client's own avatar are the same as TCP's with gRPC beside the lane (2.6–3.5 m
  over 30 s, against gRPC's 2.6–5.6). With WebSocket beside it they are 1.5–2 m higher on average
  than WebSocket alone (4.8–8.6 m against 2.7–5.8). The intents still ride the reliable connection
  (§7.4).
- **Media.** Over SCTP, media turned out worse than over TCP under loss (the first spike, §7.3).
  - Camera p95 is 121–210 ms on the media lane, against 94–124 ms in the hybrid (media on the
    connection, poses on the lane).
  - The lane also delivers fewer frames (92–99 %).
  - The cause is usrsctp's congestion control holding video longer than Linux TCP's (CUBIC,
    RACK). It is not head-of-line blocking: that is gone either way, since the poses no longer
    share the connection.
- **Hence the default.** Poses on the lane, media on the connection, with the media lane one flag
  away until a real-time congestion control is evaluated for it (§9).

This document compares the candidate stacks (§1), describes the design (§2–§6), and gives the
spike's measured comparison against the TCP paths (§7). Where this differs from
ARIADNE_CONFERENCE_ROADMAP.md §6, see §8.

---

## 1. The stack: WebRTC data channels vs WebTransport vs raw UDP

| | **WebRTC data channels** (picked) | **WebTransport** (QUIC datagrams) | **Raw UDP** (native only) |
|---|---|---|---|
| **Browser support** | Every engine, for years. Unordered, `maxRetransmits` and `maxPacketLifeTime` are supported everywhere, **iOS Safari since 11**. | Chrome/Edge 97+, Firefox 114+. **Safari only since 26.4 (iOS/macOS 26.4, 2026-03-24)**; WKWebView and in-app browsers are unconfirmed. | None. A browser cannot open a UDP socket. |
| **iOS Safari** (a real client: a phone camera) | Works on every iOS a phone is likely to run. | Only on iOS ≥ 26.4. Older phones fall back to TCP. | Not possible. |
| **Server library (C++)** | libdatachannel (MPL-2.0), CMake, small deps. ICE, DTLS, SCTP and partial reliability are all done. | An HTTP/3 + WebTransport server: msquic + libwtf, or ngtcp2 + nghttp3 plus our own WebTransport framing. Less mature, and much more to own. | Nothing to integrate, but we would write auth, encryption, keep-alive, and fragmentation of video frames. |
| **Browser library** | datachannel-wasm (MIT): about 600 lines of C++ plus a JS library over `RTCPeerConnection`. | Hand-written JS glue over the `WebTransport` object. No C++ wrapper exists. | n/a |
| **NAT / reachability** | ICE. With a public relay, the relay's host candidate is enough: no STUN, no TURN (§3). | Plain UDP to the server's port. Needs a CA-signed certificate for the hostname, or `serverCertificateHashes` with ≤ 14-day ECDSA certificates, and the relay must terminate QUIC itself: Apache cannot proxy HTTP/3. | Plain UDP, plus our own NAT keep-alive. |
| **Message size** | Up to 64 KiB here (SCTP fragments it; a message is delivered whole or not at all). | One datagram ≤ ~1200 B: a video frame must be fragmented and reassembled by us. | Same as WebTransport. |
| **Congestion control** | SCTP's (usrsctp: Reno by default, RTCC available, §5). Media gets no GCC, which is RTP-only. | QUIC's (Cubic/BBR, per implementation). | None, unless we write it. |
| **Security** | DTLS 1.2, with the fingerprint bound by signaling over our authenticated connection. | TLS 1.3. | Ours to design (an AEAD keyed over the reliable channel). |
| **License fit** (LGPL-2.1-only libcvc) | MPL-2.0 and MIT, linked as a separate library: compatible (§6). | msquic MIT, ngtcp2 MIT: compatible. | n/a |

**Why data channels.**
- **They work in every browser we ship to, including the iOS Safari on phones today.**
  WebTransport would leave every iPhone below iOS 26.4 on TCP.
- **They are one protocol** for native and browser alike: one relay endpoint, and one
  implementation of acks and shaping.
- **They need no new certificate handling**, and no listener in front of Apache.
- **SCTP partial reliability gives what we want per message**, with fragmentation built in:
  unordered delivery, and an abandoned message never holds up the next one.

**What it costs.**
- **SCTP congestion control.** A data channel is unreliable but still congestion-controlled, and
  usrsctp's congestion control is plain RFC 2581.
  - Poses barely notice: about 13 kb/s never fills a window.
  - Video does: after a loss, frames wait for the window to regrow, and they wait longer than on
    Linux TCP (§5, §7).
  - Hence two associations, and media on the connection by default.
- Media on a data channel gets none of the browser's RTP machinery: no GCC/TWCC bandwidth
  estimation, no jitter buffer, no hardware codec path. Our relay's per-viewer rung controller
  stays the bandwidth estimator, and our own playout buffers stay in charge (holon `display_pose`
  for poses; the arigl media buffers for A/V).

**WebTransport stays the obvious second step** once iOS ≥ 26.4 is the floor. Its datagrams
carry the same lane envelope (§2), so `fast_lane_hub` gets a QUIC implementation and nothing
above it changes. **Raw UDP** buys nothing a native libdatachannel lane does not already give,
and it would be a second protocol to secure.

**RTP media tracks** (libdatachannel `rtc::Track`, simulcast H.264/VP8) are deliberately not part
of this step.
- They would give the browser's GCC and hardware decode.
- But the relay could then no longer re-encode a viewer's camera at its rung (`media_rungs.cpp`
  transcodes JPEG). AOI thumbnails would need simulcast layers.
- And the browser would decode into a `<video>` element that has to be copied into a GL texture.

That is a separate decision about codecs, not about transport. ARIADNE_CONFERENCE_ROADMAP.md §6e
keeps it open.

---

## 2. The lanes' wire

Every lane datagram is one data-channel message. Its first byte is its kind (`cvc/net/fast_lane.h`),
and every field is little-endian. Kinds 1 and 2 travel the pose lane; kind 3 travels the media
lane; pings and pongs travel both.

| kind | direction | body |
|---|---|---|
| 1 `pose` | relay → client | a serialized `cvc.transport.v1.PoseSnapshot`, the message the reliable transports carry as `Frame` field 13: `server_tick`, `seq`, `base_seq`, the entries (slot, the pid the first time, position mm, velocity mm/s, orientation ×32767, each a difference from the base) and the slots `gone` |
| 2 `pose_ack` | client → relay | `u32 seq`: the snapshot it decoded |
| 3 `media` | both ways | origin, stream and codec (each a length byte and its bytes), `u32 w`, `u32 h`, `i64 seq`, `f64 pts`, `u32 rate`, `u32 channels`, then the frame |
| 4 `ping`, 5 `pong` | both ways | `u64 seq`, `i64 sent_ns` (a pong echoes the ping; RTT on the prober's own clock) |

**The pose payload is the pose stream** (`pose_stream.h`), unchanged on the wire. What the lane
changes is when the encoder moves on.

- **On a reliable connection** the encoder commits on write: TCP will deliver the snapshot, so the
  next one differs from it (`base_seq` = the last `seq`). The decoder applies a snapshot only
  against the one before it, and a missing base means the stream is out of step.
- **On the pose lane** the encoder runs *acked* (`pose_stream_encoder::set_acked`):
  - Nothing commits on write. Every snapshot gets a `seq` of its own, and its state is kept (the
    last 64) until the viewer acknowledges a `seq`.
  - The acknowledged snapshot becomes the base. Every snapshot until the next ack differs from it,
    so several are in flight on one base and any of them decodes on its own.
  - A lost snapshot costs only its own latency: the next one still carries every difference from
    what the viewer has. An ack older than the base is ignored.
  - The client decodes with **several bases** (`pose_stream_decoder::set_bases(32)`): it keeps the
    last 32 decoded states by `seq`, applies a snapshot against the one its `base_seq` names, and
    acknowledges it. A snapshot whose base it no longer has is dropped and counted as a gap, as on
    the reliable path; the next one, or the keyframe, carries it.
  - Snapshots arrive in any order. One whose tick is not newer than the newest decoded is still
    kept as a base and acknowledged, but gives no samples.
- **What stays the same:** the per-viewer rate (60/30/20 Hz by RTT, `pose_send_interval_steps`,
  fed the lane's own ping RTT once the lane is open), the area of interest's cadence for non-inner
  subjects (`push_interval_ticks`), and the keyframe every 2 s (a snapshot with base 0, which also
  restarts the acked window).
- **A keyframe keeps the bases.** Until the keyframe's ack returns, every snapshot is a keyframe
  too (base 0), and any of their acks may become the relay's base. So the client keeps every one of
  them. It forgets its bases only when a keyframe's `seq` goes back, which means a new encoder (the
  relay forgot the viewer and met it again).
  - The first cut cleared the bases at every base-0 snapshot. That cost about 68 gaps per run even
    without loss. When the ack of the newest keyframe was lost, the stream stalled until the next
    keyframe: pose age p99 up to 814 ms under loss. With the fix, every run had 0 gaps and a p99 of
    51–62 ms (`PoseStreamAckedTest.AKeyframeKeepsTheBasesItsAcksMayName`).
- **Switching path** restarts the stream: when a viewer's pose lane opens, its next snapshot is a
  keyframe on the lane; when the lane closes, a keyframe on the connection.

---

## 3. Signaling and NAT

Each lane is offered and answered over the client's reliable connection. Both messages are
`state_message`s at the participant's own path, `<participant>.lane.pose` or
`<participant>.lane.media` (`cvc/ariadne/conference/room_lane.h`).

**The offer.**
- It goes up with `ttl_hops = 0`. A relay now delivers a ttl-0 message to its own shards only, and
  never forwards it to other clients or across the WS↔gRPC bridge.
- Bound writes (`room_write_gate`) already let only the participant's own connection send at its
  path. So an offer speaks for exactly the pid its connection is bound to, and the lane is bound to
  that pid. A lane's media is refused if it claims any other origin.

**The answer.**
- It comes back through the new `state_transport::send_message_to(pid, msg)`, to that
  participant's connections alone.
- It is written even while the admission hold still has the connection. It is not tree state, so
  it cannot get ahead of the bootstrap. Without this, a client that offers at once loses its answer
  to the hold.

**DTLS.** The SDP carries each end's certificate fingerprint over the authenticated connection, so
the DTLS handshake proves both ends.

**NAT.** The relay has a public address (38.57.161.19), so the client is always the ICE side that
reaches out.
- The relay binds **one** UDP port for every lane (libjuice's UDP mux, `--lane-listen
  0.0.0.0:7703`).
- Its answer carries that port as host candidates.
- `--lane-advertise 38.57.161.19` adds the public address in front of a NAT that forwards the port.
- The client's own candidates are irrelevant. Browsers obfuscate them as mDNS names anyway, and the
  relay learns the client's real address from the client's connectivity checks (a peer-reflexive
  candidate).

**No STUN or TURN.**
- **STUN** would only tell a client its public address, which the relay never needs.
- **TURN** would only help a network that blocks UDP outright. There it would carry the lane over
  TCP or TLS on 443, and that brings back exactly the head-of-line blocking the lane exists to
  avoid. Such a client already has the right fallback: its reliable connection.

**Deployment** (not done; it needs the user's go-ahead):
- forward UDP 7703 on 38.57.161.19 to paladin;
- run arisrv with `--lane-listen 0.0.0.0:7703 --lane-advertise 38.57.161.19`;
- serve cyprty built with `CVC_ENABLE_RTC=ON`; the page opts in with `?lane=1` for now.

The Apache WebSocket proxy stays as it is.

---

## 4. The relay stays the hub: AOI, rungs and the pose stream

The relay is still an SFU: every lane runs to the relay, never a mesh. It runs two
`fast_lane_server`s (`cvc/net/fast_lane_rtc.h`) on the same UDP port, one per lane: libjuice's
mux tells their ICE sessions apart.

**The media lanes' server is a `media_endpoint` of the relay's `media_router`**, beside the WS and
gRPC endpoints. It exists only with `--lane-media on`; by default media stays on the reliable
endpoints and only the pose lanes run.
- A frame from any endpoint fans out to the lanes. A frame from a lane fans out to everyone.
- `room_aoi_relay::add_media_endpoint` gives it the same **area-of-interest gate** as the others:
  outer peers dropped, middle ones thumbnailed.
- arisrv gives it the same **rung shaper** (`media_rung_shaper`): a viewer's camera at full, half,
  quarter or thumb.
- The reliable endpoints' shaper drops media for a viewer whose lane is open, so each viewer gets
  each frame once.
- A frame is never sent back to its origin pid.
- **The rung controller** (`room_rung_controller`) reads the lane's link report ahead of the
  reliable one. That report holds the lane-ping RTT, and the share of media the lane dropped at a
  full send buffer. So a viewer's rung follows the link that actually carries its video.

**The pose lanes' server is `room_pose_relay`'s lane** (`room_lane_relay`, set by
`room_pose_relay::set_lane`).
- *Routing.* Each room step, for each viewer that is due, `room_pose_relay` asks whether its pose
  lane is open. If it is, the snapshot goes there from the acked stream; if not, it goes to the
  viewer's connection from the committed stream, as before. Nothing else about the snapshot
  changes: the same area of interest, the same rate, the same keyframes.
- *Acks.* The client acknowledges each snapshot it decodes. `room_lane_relay` hands the ack to
  `room_pose_relay::acknowledge`, which applies it at the next room step (acks arrive on the lane's
  threads).
- *RTT.* arisrv feeds each lane's ping RTT to `room_pose_relay::set_rtt` after the reliable links'
  reports, so a lane viewer's rate follows the path that carries its poses.
- *Applying.* The client applies the decoded samples with `apply_pose_samples` (`holon_sync.h`),
  the same call `apply_pose_snapshot` makes for a snapshot from the connection. `read_pose`,
  `client_pump` and the holon's playout timeline therefore read lane poses unchanged.

This is QuakeWorld's delta from the last acknowledged state, at field granularity (the pose
stream's differences in mm and quaternion units).

---

## 5. Congestion and queueing

- **SCTP congestion control is per association.** A data channel with `maxRetransmits: 0` is
  never retransmitted, but it is still congestion-controlled: its sender holds messages until
  SCTP's window lets them go.
  - libdatachannel gives usrsctp a 1 MiB send buffer, RFC 2581 congestion control, and a 20 ms
    delayed SACK. After a loss halves the window, a 5-packet camera frame waits for SACKs to regrow
    it.
  - **On one association with the video, the poses wait too.** At 3 % loss their p95 arrival rose
    to 97 ms, worse than gRPC's 80. With media off it stayed at 25 (§7.3).
  - **This is why there are two lanes.** The pose association carries a few kb/s and its window
    never binds.
- **Tuning the media association did not make it TCP's equal.** Each was measured at 1–3 % loss
  on one association (`fast_lane_sctp`, `arisrv_aoi_bench --lane-*`):
  - usrsctp's RTCC module (`cc_module 3`) helped at 2 % loss, not at 1 %.
  - A 1 ms delayed SACK, and H-TCP, made it worse (H-TCP badly: camera p50 318 ms).
  - A 64 KiB SCTP send buffer instead of 1 MiB changed nothing, so the queue is the window, not the
    buffer.

  None of them is the default.
- **Newest wins at the sender.** Each lane refuses a datagram once its own buffered amount passes
  `send_buffer_max` (64 KiB by default). The datagram is dropped, and the next frame or snapshot
  supersedes it. That counts as a media drop in the link report, so the viewer's rung steps down.
- **Timed partial reliability** (`set_lifetime`, `maxPacketLifeTime`) is the other knob. SCTP may
  retransmit a lost fragment while the message is younger than the lifetime, and abandons it then,
  even if it is still queued.
  - At 100 ms (the media lane's default, `kMediaLaneLifetime`) camera delivery rises from 92 % to
    99 % at 1 % loss, and from 74 % to 92 % at 3 %. The cost is the p95: 81 → 121 ms at 1 % loss
    (§7).
  - The pose lane never retransmits: the acked stream already resends, in every snapshot,
    whatever differs from the acknowledged base, at its newest state.
- **The browser's side.** Chrome and Safari run their own SCTP (dcsctp), so a browser's uplink is
  the browser's congestion control. The client checks `bufferedAmount` before each send and drops
  past the limit.

---

## 6. License fit

libcvc is LGPL-2.1-only. The lane adds only libraries we link as separate components, unmodified,
from cvcpkg:

| Component | License | How it is linked |
|---|---|---|
| libdatachannel | MPL-2.0 | shared (static on wasm is not needed: the browser side is datachannel-wasm) |
| libjuice | MPL-2.0 | inside libdatachannel |
| usrsctp, libSRTP | BSD-3-Clause | inside libdatachannel |
| plog | MIT | header-only, inside libdatachannel |
| OpenSSL | Apache-2.0 | shared; libcvc already links it for device keys |
| datachannel-wasm | MIT | static, into the wasm client |

**MPL-2.0 is file-level copyleft.** It requires the MPL-covered files to stay available under the
MPL; it places no condition on the code they are linked with (§3.3, "Larger Work"). The cvcpkg
recipe ships upstream's unmodified source and its LICENSE.

Nothing here constrains how libcvc is licensed, and the LGPL relinking guarantee is unaffected.

---

## 7. Measured comparison

### 7.1 How it was measured

**Harness.** `arisrv_aoi_bench` runs a relay assembled as arisrv assembles it, and synthetic clients
assembled as arigl assembles them, all in one process, so every probe reads one steady clock.
- Each run is a fresh unprivileged network namespace on catx-03 (`unshare -rn`, Ubuntu 22.04,
  kernel 5.15).
- Its loopback carries a netem qdisc. Every packet between a client and the relay, TCP or UDP,
  crosses it once, so both directions see the same impairment.
- The scripts are `lane_run.sh` and `lane_matrix*.sh`, under `/mnt/ramdisk/tfx/rtc` on catx-03.

**Load.** 2 clients walking, for 30 s after a 4 s warm-up.
- A 15 fps camera of 5.4 KB frames: prod's 320×240 q60 publisher is about 650 kb/s.
- A 50 Hz Opus mic of 80 B packets.

**Two series.**
- **The pose snapshots (§7.2)**, on `feat/conf-pose-snapshots` (#945 included, so the WebSocket
  has TCP_NODELAY):
  - the relay steps at 60 Hz and sends each viewer its `PoseSnapshot`s;
  - the clients run 60 Hz frames (`--poses stream --client-hz 60`);
  - each lossy row ran 3 times.
- **The first spike (§7.3)**, on master before #945:
  - the relay stepped at 20 Hz, with poses as tree mutations;
  - the lane carried its own float pose format, with a subject-level book;
  - this series compared the media lane's options.

**Probes**, as p50 / p95 / p99 in ms:
- **pose arrival:** from the relay writing a tick to the first moment a receiver holds that tick for
  a peer;
- **pose age:** at each client frame, how old the newest tick held for each walking peer is. It
  includes the phase of the room step: up to 17 ms at 60 Hz, up to 50 ms at the first spike's 20 Hz;
- **camera / mic:** each frame's trip from its publisher's send to its arrival at the other
  client;
- **delivered:** frames that arrived over frames sent (capped at 100 %; the window's edges add a
  few);
- **corrections:** the reconcile corrections of each client's own avatar over the run, in metres
  (`probes.correction_sum_m`);
- **peer lag:** how far behind the server a client displays its peers (`probes.peer_lag_ms`).

**Transports compared.**
- WebSocket (with #945's TCP_NODELAY) and gRPC.
- In §7.2, the pose lane with everything else on the connection (arisrv's default): behind a
  WebSocket, as the browser runs it, and behind gRPC, as arigl does.
- In §7.3, the lane in three shapes:
  - one association for poses and media;
  - two associations;
  - two with the media lane's lifetime 100 ms, plus the default: the pose lane with media on the
    WebSocket.

**Caveats.**
- netem's correlated-loss mode (`loss 3% 50%`) dropped nothing on this kernel, so the bursty
  condition uses its Gilbert-Elliott model instead.
- Jitter is applied with `rate 1gbit`, so packets are delayed but not reordered. Per-packet netem
  jitter reorders freely, which no real path does, and SCTP takes reordering for loss.
- The browser's own SCTP (dcsctp in Chrome and Safari) was not under netem here: §7.5.

### 7.2 Results: the pose snapshots

**Loopback, no impairment**

| transport | pose arrival | pose age | camera | camera delivered | mic | corrections, m | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket | 0 / 0 / 1 | 1 / 18 / 18 | 0 / 0 / 0 | 100.0 % | 0 / 0 / 0 | 11.9 | 751 |
| gRPC | 0 / 0 / 1 | 1 / 18 / 19 | 0 / 1 / 1 | 100.0 % | 0 / 1 / 1 | 18.3 | 745 |
| **pose lane** + WebSocket (the browser) | 0 / 0 / 1 | 1 / 3 / 26 | 0 / 0 / 0 | 100.0 % | 0 / 0 / 0 | 7.9 | 756 + 11 lane |
| **pose lane** + gRPC (arigl) | 0 / 1 / 3 | 1 / 3 / 23 | 0 / 1 / 1 | 100.0 % | 0 / 1 / 1 | 11.6 | 743 + 12 lane |

**20 ms each way (RTT 40 ms)**

| transport | pose arrival | pose age | camera | camera delivered | mic | corrections, m | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket | 20 / 20 / 20 | 34 / 35 / 37 | 40 / 40 / 40 | 100.0 % | 40 / 40 / 40 | 30.5 | 751 |
| gRPC | 20 / 20 / 21 | 34 / 36 / 37 | 40 / 41 / 41 | 100.0 % | 40 / 41 / 41 | 2.3 | 743 |
| **pose lane** + WebSocket (the browser) | 20 / 20 / 21 | 34 / 36 / 45 | 40 / 40 / 40 | 100.0 % | 40 / 40 / 40 | 4.4 | 756 + 13 lane |
| **pose lane** + gRPC (arigl) | 20 / 21 / 22 | 34 / 36 / 50 | 40 / 41 / 41 | 100.0 % | 40 / 41 / 41 | 2.9 | 743 + 13 lane |

**20 ms each way, 1 % loss**

| transport | pose arrival | pose age | camera | camera delivered | mic | corrections, m | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket | 20 / 43 / 72 | 34 / 51 / 84 | 40 / 79 / 99 | 100.0 % | 40 / 78 / 98 | 3.5 | 752 |
| gRPC | 20 / 50 / 75 | 34 / 63 / 84 | 41 / 84 / 96 | 100.0 % | 41 / 84 / 101 | 2.8 | 745 |
| **pose lane** + WebSocket (the browser) | 20 / 21 / 21 | 34 / 36 / 51 | 40 / 91 / 104 | 100.0 % | 40 / 79 / 100 | 6.0 | 756 + 13 lane |
| **pose lane** + gRPC (arigl) | 20 / 21 / 22 | 34 / 36 / 51 | 41 / 81 / 99 | 100.0 % | 40 / 82 / 102 | 3.0 | 746 + 13 lane |

**20 ms ± 5 ms each way, 3 % loss (jitter without reordering)**

| transport | pose arrival | pose age | camera | camera delivered | mic | corrections, m | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket | 26 / 75 / 94 | 35 / 84 / 102 | 58 / 111 / 147 | 100.0 % | 52 / 108 / 140 | 3.3 | 754 |
| gRPC | 28 / 72 / 94 | 35 / 84 / 101 | 61 / 114 / 148 | 100.0 % | 60 / 112 / 139 | 2.9 | 745 |
| **pose lane** + WebSocket (the browser) | 22 / 25 / 25 | 34 / 44 / 54 | 56 / 117 / 165 | 100.0 % | 49 / 113 / 153 | 8.6 | 756 + 13 lane |
| **pose lane** + gRPC (arigl) | 22 / 25 / 25 | 34 / 45 / 52 | 59 / 120 / 146 | 100.0 % | 57 / 117 / 153 | 3.1 | 748 + 13 lane |

**20 ms each way, bursty loss (Gilbert-Elliott: ~3 %, bursts of ~3 packets)**

| transport | pose arrival | pose age | camera | camera delivered | mic | corrections, m | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket | 20 / 69 / 109 | 34 / 85 / 668 | 48 / 112 / 835 | 96.5 % | 41 / 181 / 1310 | 10.0 | 727 |
| gRPC | 24 / 844 / 3665 | 51 / 3301 / 3884 | 63 / 7578 / 8841 | 100.0 % | 60 / 7579 / 8818 | 34.7 | 740 |
| **pose lane** + WebSocket (the browser) | 20 / 21 / 21 | 34 / 42 / 52 | 44 / 120 / 184 | 100.0 % | 40 / 122 / 176 | 4.9 | 756 + 13 lane |
| **pose lane** + gRPC (arigl) | 20 / 21 / 22 | 34 / 44 / 53 | 45 / 118 / 202 | 100.0 % | 41 / 123 / 202 | 3.5 | 748 + 13 lane |

**Media off, 20 ms ± 5 ms each way, 3 % loss** (poses alone on each transport)

| transport | pose arrival | pose age | corrections, m |
|---|---|---|---|
| WebSocket | 23 / 76 / 105 | 34 / 84 / 117 | 3.3 |
| gRPC | 25 / 75 / 96 | 35 / 84 / 117 | 3.3 |
| **pose lane** + WebSocket (the browser) | 22 / 25 / 25 | 34 / 42 / 52 | 5.5 |
| **pose lane** + gRPC (arigl) | 22 / 25 / 25 | 34 / 47 / 54 | 3.4 |

**Spread over three runs of each lossy row** (p99 and p95 in ms; a range where the runs differ):

| path | transport | runs | pose arrival p99 | pose age p95 | pose age p99 | camera p95 | corrections, m (each run) |
|---|---|---|---|---|---|---|---|
| 1 % loss | WebSocket | 3 | 71–74 | 51–52 | 84 | 79–84 | 3.5, 5.8, 3.0 |
| 1 % loss | gRPC | 3 | 74–76 | 59–67 | 84 | 80–91 | 2.8, 2.6, 5.6 |
| 1 % loss | pose lane + WebSocket | 3 | 21 | 36 | 51–52 | 91–93 | 6.0, 4.8, 6.3 |
| 1 % loss | pose lane + gRPC | 3 | 22 | 36 | 51 | 81 | 3.0, 2.8, 2.6 |
| 3 % loss ± 5 ms | WebSocket | 3 | 94–133 | 84–85 | 102–141 | 111–124 | 3.3, 3.8, 3.3 |
| 3 % loss ± 5 ms | gRPC | 3 | 90–98 | 84 | 101–102 | 114–118 | 2.9, 2.8, 2.8 |
| 3 % loss ± 5 ms | pose lane + WebSocket | 3 | 25 | 42–44 | 52–54 | 117–121 | 8.6, 5.1, 5.1 |
| 3 % loss ± 5 ms | pose lane + gRPC | 3 | 25 | 41–47 | 52–62 | 120–123 | 3.1, 3.1, 3.3 |
| bursty | WebSocket | 3 | 93–139 | 84–85 | 101–668 | 110–116 | 10.0, 2.7, 4.0 |
| bursty | gRPC | 3 | 314–7792 | 139–6672 | 585–7789 | 154–8535 | 34.7, 62.3, 4.5 |
| bursty | pose lane + WebSocket | 3 | 21 | 42–50 | 52 | 116–129 | 4.9, 5.8, 5.2 |
| bursty | pose lane + gRPC | 3 | 22 | 44–48 | 52–53 | 112–137 | 3.5, 2.9, 3.2 |

The lane's own counters, in every run of this series: 0 gaps. An ack moved the base for about
92 % of the snapshots under 3 % loss, and 98 % at 40 ms RTT without loss (an ack that arrives after
a newer one moves nothing).

### 7.3 Results: the first spike (tree poses at 20 Hz, and the media lane)

The pose rows here are the lane's first payload: whole float poses, with a book of acked subjects.
§7.2 supersedes them; the media rows still stand.

**Loopback, no impairment**

| transport | pose arrival | pose age | camera | camera delivered | mic | mic delivered | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket (+TCP_NODELAY of #945) | 0 / 1 / 2 | 11 / 31 / 41 | 0 / 0 / 1 | 100.0 % | 0 / 0 / 0 | 100.0 % | 823 |
| gRPC | 1 / 2 / 5 | 25 / 44 / 54 | 0 / 1 / 2 | 100.0 % | 0 / 1 / 2 | 100.0 % | 816 |
| one lane (poses + media on one association) | 1 / 1 / 1 | 17 / 41 / 44 | 1 / 1 / 2 | 100.0 % | 0 / 1 / 1 | 100.0 % | 32 + 737 lane |
| **two lanes** (pose / media) | 1 / 1 / 2 | 14 / 40 / 50 | 1 / 3 / 8 | 100.0 % | 0 / 1 / 3 | 100.0 % | 42 + 730 lane |
| two lanes, media lifetime 100 ms (`--lane-media on`) | 0 / 1 / 1 | 23 / 44 / 48 | 1 / 1 / 2 | 100.0 % | 0 / 1 / 1 | 100.0 % | 41 + 736 lane |

**20 ms each way (RTT 40 ms, about prod's 41 ms)**

| transport | pose arrival | pose age | camera | camera delivered | mic | mic delivered | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket (+TCP_NODELAY of #945) | 20 / 21 / 21 | 51 / 64 / 67 | 40 / 41 / 47 | 100.0 % | 40 / 40 / 43 | 100.0 % | 824 |
| gRPC | 21 / 22 / 27 | 52 / 63 / 77 | 41 / 41 / 49 | 100.0 % | 41 / 42 / 48 | 99.9 % | 816 |
| one lane (poses + media on one association) | 21 / 21 / 25 | 51 / 69 / 79 | 41 / 44 / 51 | 100.0 % | 40 / 42 / 48 | 100.0 % | 32 + 734 lane |
| **two lanes** (pose / media) | 20 / 21 / 21 | 44 / 68 / 73 | 41 / 41 / 44 | 100.0 % | 40 / 41 / 43 | 100.0 % | 41 + 736 lane |
| two lanes, media lifetime 100 ms (`--lane-media on`) | 20 / 21 / 21 | 44 / 70 / 73 | 41 / 41 / 43 | 100.0 % | 40 / 41 / 42 | 100.0 % | 40 + 734 lane |

**20 ms each way, 1 % loss**

| transport | pose arrival | pose age | camera | camera delivered | mic | mic delivered | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket (+TCP_NODELAY of #945) | 20 / 58 / 74 | 53 / 80 / 111 | 40 / 84 / 100 | 100.0 % | 40 / 80 / 100 | 100.0 % | 826 |
| gRPC | 24 / 61 / 82 | 59 / 94 / 111 | 43 / 86 / 107 | 100.0 % | 41 / 85 / 103 | 100.0 % | 817 |
| one lane (poses + media on one association) | 21 / 39 / 58 | 52 / 74 / 101 | 41 / 90 / 121 | 90.8 % | 40 / 77 / 97 | 97.8 % | 32 + 703 lane |
| **two lanes** (pose / media) | 20 / 21 / 21 | 51 / 69 / 77 | 41 / 81 / 106 | 91.9 % | 40 / 71 / 89 | 98.1 % | 40 + 705 lane |
| two lanes, media lifetime 100 ms (`--lane-media on`) | 21 / 21 / 21 | 50 / 70 / 80 | 43 / 121 / 139 | 99.1 % | 40 / 81 / 116 | 99.7 % | 41 + 732 lane |
| **pose lane, media on the connection** (the default) | 21 / 22 / 28 | 51 / 71 / 84 | 41 / 94 / 106 | 100.0 % | 40 / 80 / 104 | 100.0 % | 742 + 21 lane |

**20 ms ± 5 ms each way, 3 % loss (jitter without reordering)**

| transport | pose arrival | pose age | camera | camera delivered | mic | mic delivered | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket (+TCP_NODELAY of #945) | 29 / 84 / 125 | 61 / 113 / 155 | 64 / 132 / 180 | 99.8 % | 59 / 126 / 169 | 100.0 % | 827 |
| gRPC | 35 / 80 / 101 | 61 / 110 / 130 | 61 / 115 / 142 | 100.0 % | 59 / 114 / 141 | 99.8 % | 818 |
| one lane (poses + media on one association) | 25 / 97 / 139 | 60 / 128 / 176 | 95 / 188 / 231 | 73.6 % | 71 / 161 / 207 | 93.9 % | 33 + 635 lane |
| **two lanes** (pose / media) | 23 / 25 / 26 | 52 / 78 / 117 | 89 / 186 / 252 | 74.0 % | 61 / 166 / 226 | 94.6 % | 44 + 629 lane |
| two lanes, media lifetime 100 ms (`--lane-media on`) | 24 / 26 / 28 | 52 / 79 / 106 | 98 / 180 / 207 | 91.8 % | 63 / 144 / 186 | 98.6 % | 44 + 699 lane |
| **pose lane, media on the connection** (the default) | 23 / 26 / 26 | 53 / 74 / 110 | 55 / 114 / 146 | 100.0 % | 49 / 114 / 145 | 100.0 % | 743 + 21 lane |

**20 ms each way, bursty loss (Gilbert-Elliott: ~3 %, bursts of ~3 packets)**

| transport | pose arrival | pose age | camera | camera delivered | mic | mic delivered | relay → each client, kb/s |
|---|---|---|---|---|---|---|---|
| WebSocket (+TCP_NODELAY of #945) | 21 / 76 / 112 | 56 / 109 / 152 | 51 / 119 / 190 | 100.0 % | 46 / 113 / 179 | 99.9 % | 825 |
| gRPC | 32 / 83 / 152 | 61 / 112 / 168 | 53 / 124 / 208 | 100.0 % | 50 / 123 / 201 | 99.9 % | 816 |
| **two lanes** (pose / media) | 21 / 22 / 27 | 51 / 81 / 122 | 63 / 166 / 273 | 84.2 % | 42 / 161 / 265 | 94.5 % | 44 + 665 lane |
| two lanes, media lifetime 100 ms (`--lane-media on`) | 21 / 22 / 27 | 52 / 83 / 116 | 73 / 210 / 427 | 93.3 % | 43 / 199 / 408 | 97.6 % | 43 + 703 lane |
| **pose lane, media on the connection** (the default) | 21 / 22 / 26 | 51 / 73 / 103 | 44 / 124 / 214 | 100.0 % | 41 / 132 / 217 | 99.9 % | 742 + 21 lane |

**Media off, 20 ms ± 5 ms each way, 3 % loss** (poses alone on each transport)

| transport | pose arrival | pose age |
|---|---|---|
| WebSocket (+TCP_NODELAY) | 25 / 82 / 115 | 60 / 113 / 151 |
| gRPC | 50 / 103 / 201 | 64 / 127 / 229 |
| lane | 22 / 25 / 26 | 42 / 73 / 106 |

### 7.4 What the numbers say

1. **On a clean or merely distant path, every transport is the same.** All of them sit at one-way
   delay (20 ms for poses, 40 ms for media relayed through the hub). TCP has no loss to recover
   from, so there is nothing to block behind.
2. **Under loss, TCP's pose tail is loss recovery, and the pose lane removes it.**
   - With media off (§7.2's last table), WebSocket's and gRPC's pose p99 at 3 % loss is 96–105 ms:
     that is TCP alone, with nothing in front of the poses. The lane's is 25 ms.
   - With video on, the lane's arrival p99 is 21–25 ms in every lossy run, against 71–139 ms over
     WebSocket and 74–98 ms over gRPC.
   - The newest pose a viewer holds stays younger: p99 51–62 ms against 84–141 ms. That is about
     three 60 Hz steps at worst, where TCP leaves five to eight.
   - Peers are drawn closer to the server: peer lag p50 75–77 ms on the lane in every lossy run,
     against 79–83 ms over WebSocket at 1 % loss and 96–111 ms at 3 % and in bursts.
   - A lost snapshot costs one room step: the next one still differs from the acknowledged base.
     It never costs a retransmission timeout.
3. **gRPC can stall for seconds under bursty loss, whatever carries the poses.**
   - In 2 of 3 runs here, with the snapshots on gRPC, everything on the stream queued for seconds:
     pose p99 up to 7.8 s, camera p95 up to 8.5 s, and the clients' clocks re-aligned again and
     again (25 times each in one run).
   - The snapshot session reran it with the same build and conditions (transfix/libcvc#951), with
     3 runs per cell:

     | poses on gRPC | 60 Hz client frames | 30 Hz client frames |
     |---|---|---|
     | tree poses | 3 of 3 stalled, camera p95 4.1–31 s | 1 of 3 stalled, camera p95 6.7 s |
     | snapshots | 0 of 3 stalled, camera p95 116–520 ms (p99 up to 1.6 s) | 2 of 3 stalled, camera p95 2.4–3.0 s |

     So the stall is random in both pose modes and at both client rates. It predates both
     changes, and neither the snapshots nor the 60 Hz client rate cause it.
   - gRPC accepts every write and queues it while TCP waits out a burst, so the media arrives
     seconds late instead of being dropped. The transport's media stall drop (`--media-stall-ms`)
     never fired in any of the 15 bursty gRPC runs (`stalled 0`), presumably because the library
     queues the writes out of the transport's sight.
   - With the poses on the lane, gRPC carried the media through the same bursts without a stall in
     3 of 3 runs (camera p95 112–137 ms). Snapshots on gRPC stalled in 4 of 9 runs across both
     batches and rates, so 3 clean runs is a hint, not a proof.
   - The first spike's one bursty gRPC run (20 Hz room steps, 30 Hz client frames) did not stall
     either (§7.3). With odds like these, one run says nothing.
4. **One association was not enough.** With poses and video on one SCTP association, the poses
   waited for the video's congestion window at 3 % loss: p95 97 ms, worse than gRPC's 80 (§7.3).
   Splitting them is what makes the lane work.
5. **Media is the open question** (§7.3).
   - On the media lane, under loss, camera frames come later at the tail than over TCP: p95 81–210
     against 84–132 ms. The lane also loses some:
     - with `maxRetransmits 0`, 92 % delivered at 1 % loss, 74 % at 3 %, 84 % in bursts (a 5-packet
       frame must survive two hops);
     - with a 100 ms lifetime, 99 %, 92 % and 93 %, but a longer tail.
   - Nothing waits behind a lost media frame any more. The cost is usrsctp's RFC 2581 congestion
     control, which regrows its window more slowly than Linux TCP's CUBIC with RACK.
   - The default keeps TCP's media numbers (100 % delivered, camera p95 81–137 ms in §7.2) and the
     lane's pose numbers.
   - On prod's real path (41 ms RTT, loss well under 1 % most of the time) the three media options
     are within a few ms of one another.
6. **Cost.**
   - The pose lane carries about 13 kb/s to each viewer in this room: 60 Hz snapshots of two
     walking participants, plus the lane's probes.
   - The reliable connection to a viewer whose media also rides the lane drops from about 820 kb/s
     to about 40 kb/s (§7.3).
   - Signaling is two out-of-band messages each way per lane. A lane opens in about 250 ms on
     loopback.
7. **Reconcile corrections** of a client's own avatar (`probes.correction_sum_m`, over 30 s):
   - **pose lane + gRPC: 2.6–3.5 m**, the same as gRPC alone (2.6–5.6 m when it did not stall;
     34.7 and 62.3 m in the two stalls);
   - **pose lane + WebSocket: 4.8–8.6 m**, 1.5–2 m more on average than WebSocket alone
     (2.7–5.8 m, and one run of 10.0).
   - Only the poses moved. The client's intents still ride the reliable connection, and they are
     stamped with a server tick that the client now learns from the lane's quicker poses. A
     WebSocket uplink that is waiting out a loss then delivers intents for ticks the relay has
     already stepped, and the client corrects. gRPC's clients do not show it. This is an
     inference, not yet measured directly. Moving the intents onto the lane (§9) is the test.
   - Without loss the numbers scatter more than they differ (2.3–30.5 m on one 40 ms run each).

### 7.5 The browser

cyprty (`CVC_ENABLE_RTC=ON`, datachannel-wasm) opened both lanes to a local arisrv from the
desktop app's Chromium browser pane (synthetic camera, `?lane=1`, relay with
`--lane-media on`). This was the first spike's build, before the pose snapshots; the PoseSnapshot
build of cyprty compiles and links, but has not been run in a browser yet.
- Both lanes opened. Over about a minute the relay took 853 media frames (1.1 MB) from the browser
  on the media lane. On the pose lane it sent 3.2 KB of snapshots and probes and got 0.8 KB back:
  acks, which the browser sends only after decoding and applying a snapshot, plus probe answers.
- The lane's round trip settled at about 11 ms, its minimum at 1 ms. The early pings answer slowly
  while the single-threaded page is still loading.
- **It took a recipe fix to get here.** Upstream datachannel-wasm read `Module['HEAPU8']`, which
  emscripten 5 leaves undefined. The lanes connected but every message threw, so the relay closed
  them as silent: the idle timeout doing its job. The recipe patches it (cy-pca/cvcpkg#539).
- **Known glitch.** In one session of three, the page logged `Uncaught (in promise) {excPtr}`: a
  C++ exception escaping the Asyncify main loop. About two minutes later the page stopped and left
  the room. That session had seen its first lanes time out while the page was still loading, and
  re-offered them. A fresh session then ran three minutes with no error:
  - 1123 media frames (0.5 MB) in on the media lane;
  - 25 KB of pose snapshots and probes out on the pose lane, 4 KB of acks and probes back;
  - lane RTT 4–23 ms.

  Not yet explained. Reproduce with a debug wasm build (`-sASSERTIONS`,
  `-sEXPORT_EXCEPTION_HANDLING_HELPERS`) before `?lane=1` goes to users. The lane is opt-in, so
  nothing changes for anyone else.

The browser's SCTP is the browser's own (dcsctp in Chrome and Safari, usrsctp in Firefox), so its
uplink congestion behaviour is not what §7.2 and §7.3 measured for the native client. An impaired
browser run needs netem in front of a real browser (a Linux host with Chromium in the netns), which
this spike did not have.

---

## 8. What changed against ARIADNE_CONFERENCE_ROADMAP.md §6

- **§6d picked libdatachannel with mbed TLS.** The recipe uses **OpenSSL** instead, for three
  reasons:
  - cvcpkg ships it on every platform, wasm included;
  - libcvc already links it for device keys;
  - libdatachannel's default build uses it.
- **§6b/6e assumed WebRTC media tracks with H.264/Opus.** This step carries our existing MJPEG and
  Opus frames on a **data channel** instead, which keeps the relay's per-viewer rungs and AOI
  thumbnails (§1). RTP tracks remain the codec question it was.
- **§6g holds as written:** no STUN, no TURN, and the relay advertises its own address.

---

## 9. Next

1. **Deploy** (needs the go-ahead):
   - forward UDP 7703 on 38.57.161.19 to paladin;
   - restart arisrv with `--lane-listen 0.0.0.0:7703 --lane-advertise 38.57.161.19`;
   - serve cyprty built with `CVC_ENABLE_RTC=ON` (datachannel-wasm from cy-pca/cvcpkg#539);
   - try `?lane=1` from a phone and a desktop.

   Once it holds, offer lanes by default (arigl and the page) instead of on request.
2. **Move the client's intents onto the pose lane**, QuakeWorld-style. Each client datagram carries
   its last few intents (redundant, so one that is lost arrives in the next), and the relay takes
   the newest by stamp. That takes the last latency-critical stream off the reliable connection,
   where it can still wait behind media, and it is the test of §7.4 item 7.
3. **gRPC under bursty loss** (§7.4 item 3): drop stale media at the gRPC writer instead of
   letting the library queue it for seconds. This holds whatever carries the poses. Done: the
   gRPC delay drop (`--media-delay-ms`, ARIADNE_CONFERENCE_ROADMAP §5h.1) dates the queue with
   in-band probes and drops video past 100 ms of it, poses past 200 ms and audio past 400 ms.
4. **A real-time congestion control for media** before the media lane becomes the default:
   - QUIC datagrams with BBR (WebTransport, once iOS ≥ 26.4 is the floor; the lane envelope
     carries over unchanged);
   - or RTP tracks with GCC for the camera (a codec decision, §1).

   Then measure a real browser under netem (Chromium in the netns on a Linux host).
5. **Run the PoseSnapshot cyprty in a browser** with `?lane=1`, and chase the Asyncify exception
   of §7.5 with a debug wasm build.
6. **libcvc's own cvcpkg recipe:** add libdatachannel / datachannel-wasm and turn
   `CVC_ENABLE_RTC` on, once #539's packages publish.

---

## 10. Files

- `inc/cvc/net/fast_lane.h`, `src/cvc/net/fast_lane.cpp`: the datagram envelope, the media and
  probe codecs, and `fast_lane_hub`.
- `inc/cvc/net/fast_lane_rtc.h`, `src/cvc/net/fast_lane_rtc.cpp`: `fast_lane_server` (relay) and
  `fast_lane_client` over libdatachannel or datachannel-wasm.
  - Inert stubs without `CVC_ENABLE_RTC`, so callers need no `#if`.
  - `CVC_LANE_TRACE=1` logs signaling and state changes.
- `inc/cvc/ariadne/conference/room_lane.h`, `src/cvc/ariadne/conference/room_lane.cpp`:
  - the lane's pose datagram (the kind byte, then the `PoseSnapshot` message) and its ack;
  - the offer/answer messages;
  - `room_lane_relay` (the relay's glue, and `room_pose_relay`'s lane) and `room_lane_session` (a
    client's: it decodes with 32 bases and acknowledges).
- `pose_stream.h`: the encoder's acked mode (`set_acked`, `ack`) and the decoder's bases
  (`set_bases`).
- `room_pose_relay`: `set_lane`, `acknowledge`, and the per-viewer switch between the lane and the
  connection.
- `holon_sync.h`'s `apply_pose_samples`, which `apply_pose_snapshot` now calls.
- `state_transport::send_message_to`, plus the ttl-0 no-forward rule (WS and gRPC).
- arisrv: `--lane-listen`, `--lane-advertise`, `--lane-media on|off` (default off).
- arigl: `CVC_ARIADNE_LANE=1`. cyprty: `?lane=1`.
- `arisrv_aoi_bench`: `--lane rtc`, `--lane-media`, the SCTP and lifetime knobs
  (`--lane-lifetime-ms`, `--lane-cc`, `--lane-sack-ms`, `--lane-sctp-kb`, `--lane-buffer-kb`), and
  the latency probes (§7).
- `src/cvc/tests/fast_lane_test.cpp`.
- cvcpkg (cy-pca/cvcpkg#539): `recipes/libdatachannel` (linux, macos, windows) and
  `recipes/datachannel-wasm` (wasm). The wasm recipe carries `heap-views.patch`: upstream's JS read
  `Module['HEAPU8']`, which emscripten 5 no longer exports, so every send and receive threw.
