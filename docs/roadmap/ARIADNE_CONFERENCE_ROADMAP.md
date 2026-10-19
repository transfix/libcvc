# Ariadne Conference — flagship multi-party video conferencing app

**North star.** A multi-party video conferencing application, written as an **Ariadne `.ari` document** —
and, through building it, an **end-to-end `.ari` SDK** that end users can use to build their *own* apps,
to run on our hosted surface **or** self-host. The client is hostable by the **`ariadne_hello`** shell on
the desktop **and in the browser (wasm)**; the **server is itself as `.ari`-scriptable as possible** (a
thin native media core driven by a server-side `.ari` room-controller). Camera + mic go out, every other
participant's video + audio comes in, and the whole call — tiles, roster, mute / camera / leave — is
expressed in the DSL, not in C++.

**MVP.** A user opens **cyberpcangel.com**, the page loads the wasm client, and they join a **default
room** and video-conference with whoever else is there, backend (native server/SFU) running. **Target 8
participants** to start; architecture aimed at **~200** (SFU with simulcast / active-speaker selection).

Two connection topologies are **first-class**, chosen per room:

* **Central server / SFU** — clients connect *out* to a relay that forwards each publisher to the others.
  Required for browser clients and for anything past a handful of people — **the MVP path**.
* **Peer-to-peer mesh** — clients connect directly (WebRTC peer connections / native `serve`+`pull`). For
  small calls with no server in the path. Get it working first; **authorization hardens later** (§5, §11).

> **Status: SPEC / iterating.** The realtime substrate (capture, audio, local streaming, the gRPC data
> plane, the bridge, most of the UI DSL) is merged (§1); nothing in §4 onward is built. §5 (access) and
> §6 (transport) are grounded against the live code. Decisions baked in this iteration: **WebRTC** is the
> production browser media plane (WebSocket is bring-up/debug + signaling); **raw and H.264/Opus** codecs;
> **WASI server** is a committed goal (fill cvcpkg gaps); mesh works first, auth hardens later; identity
> (OIDC/JWT) is a later version.

---

## 1. What already exists (the foundation)

Verified merged on `master` (the cross-node streaming effort, #599–#617, plus the earlier stream +
UI DSL work). The realtime hard part is **done**; the gaps are glue.

| Layer | State | Where |
|---|---|---|
| Camera capture (SDL3 → `frame_source`, rgba8) | ✅ merged (SDL-gated, cvcGL) | `inc/cvc/gl/capture/camera_source.h` |
| Audio device capture + composable graph (device / tone / gain / mix) | ✅ merged | `inc/cvc/gl/capture/audio_source.h`, `inc/cvc/ariadne/stream/audio_sources.h` |
| Audio **playback** sink (f32 → SDL speakers) | ✅ merged (bare interface, not auto-pumped) | `inc/cvc/gl/playback/audio_output.h` |
| Local stream (pool, channel, zero-copy frames, `start_producer`) | ✅ merged | `inc/cvc/ariadne/stream/stream.h` |
| Video-on-quad (`type: stream`), video window (`type: stream_view`) | ✅ merged | `inc/cvc/gl/ariadne/stream_node.h`, `stream_view_widget.cpp` |
| Cross-node data plane — gRPC `StreamTransport`, chunking, TLS | ✅ merged | `inc/cvc/state/state_stream_transport_grpc.h` |
| Bridge — **both** push directions (`serve`/`receive_into`, `serve_port`/`pull_from`), auto-negotiation | ✅ merged | `inc/cvc/ariadne/stream/stream_bridge.h` |
| UI DSL — grid + `repeat` tiles, buttons → pure-`.ari` actions, labels, reactive `visible_when`/`enabled_when` | ✅ merged | `inc/cvc/ariadne/widget.h`, `src/cvc/ariadne/loader.cpp` |

**The four gaps this roadmap closes:**

1. **No DSL verbs** for capture, playback, or anything cross-node — all the above is C++-only API.
2. **`ariadne_hello` wires none of it** (no stream/A-V/net verbs, plain `Runtime` not `AriRuntime`,
   `CVC_ENABLE_GRPC` OFF).
3. **No room / roster / signaling layer**, no SFU relay class, and **no client-safe authz** — only
   pairwise offer/answer over trust-all-peers replication with self-asserted writer identity (§5c).
4. **No browser media plane** — native gRPC can't run in a browser; the production path is **WebRTC**
   (none in-tree today), with a WebSocket fallback for bring-up/signaling. Plus two UI holes: no
   `input_text` widget (chat) and the audio sink isn't auto-pumped.

---

## 2. Architecture

```
        ┌─────────── SERVER (native now; WASI target) — as .ari-scriptable as possible ───────────┐
        │  room authority: owns conf/rooms/<room> tree + roster + admission   (server-side .ari)    │
        │  SFU: ingests each publisher ONCE, forwards per-subscriber; simulcast/active-speaker @ scale│
        │  media legs:  WebRTC (browsers + native)   ·   gRPC StreamTransport (native)              │
        │  signaling/control:  cvc::state over a WebSocket transport_kind (+ native gRPC/ipc)       │
        └──────▲──────────────────────────▲───────────────────────────────▲────────────────────────┘
      WebRTC+WS │                  gRPC+ctl│                        WebRTC+WS│      ← MVP clients served
        ┌───────┴──────────┐     ┌─────────┴────────┐          ┌────────────┴──────┐   from cyberpcangel.com
        │ BROWSER client   │ ... │ NATIVE client    │   ...    │ BROWSER client    │
        │ariadne_hello.wasm│     │ ariadne_hello    │          │ariadne_hello.wasm │
        │ .ari conference  │     │ .ari conference  │          │ .ari conference   │
        └──────────────────┘     └──────────────────┘          └───────────────────┘
                   P2P mesh (small calls): peers connect directly — WebRTC / native serve+pull
```

* **Two planes.** *Control / signaling* = `cvc::state` (room tree + message bus: join/leave, **SDP/ICE**
  offer/answer, roster, flags, chat) — low-bandwidth, fine over WebSocket to the browser. *Media* =
  audio/video over **WebRTC** (browser: hardware H.264/Opus, congestion-controlled) or the gRPC
  `StreamTransport` (native).
* **SFU, not fan-out-at-source.** The server receives each publisher **once** and forwards to
  subscribers. MVP = full fan-out for 8. Toward ~200 the SFU selects what each subscriber gets (simulcast
  layer / active-speaker / paused off-screen tiles) — never 200 streams to everyone. The forwarding is
  native; the *policy* (who sees whom) is server-side `.ari`.
* **Server is an `.ari` app too.** A thin native core (WebRTC/RTP + gRPC legs + frame fabric) driven by a
  server-side `.ari` room-controller using the DSL verbs (§4) — same language both ends. That symmetry
  *is* the SDK story.
* **Browser = SFU-only** (can't host a server); native can do SFU or mesh.

---

## 3. The flagship `.ari` (what the app looks like in the DSL)

The whole app is one document (plus `cvc://` component imports). Sketch:

```scheme
; ── join: server endpoint + room id; returns a room handle; mode comes from the server ──
(set room (room-join "conf.example.org:7000" "standup" (dict "name" "Joe")))

; ── publish my camera + mic into the room ──
(set cam (stream-open (dict "id" "self/cam" "codec" "rgba8" "w" 1280 "h" 720)))
(stream-capture-camera (get-attr cam "token") 0)              ; §4 capture verb
(room-publish room (dict "kind" "camera" "token" (get-attr cam "token")))

(set mic (stream-open (dict "id" "self/mic" "codec" "f32" "rate" 48000 "ch" 1)))
(stream-capture-audio (get-attr mic "token") (dict "device" 0))
(room-publish room (dict "kind" "audio" "token" (get-attr mic "token")))

; ── UI: a self tile + one tile per remote participant (roster-driven) ──
; scene / widget tree uses `repeat` over (room-roster room); each tile binds a
; (room-subscribe room PID "camera") proxy token into a `stream_view`, and
; (stream-play-audio (room-subscribe room PID "audio")) plays their voice.

; ── controls: pure-.ari actions, no C++ ──
; button "Mute"   on: (room-set-flag room (dict "mic" (not (room-flag room "self" "mic"))))
; button "Camera" on: (room-set-flag room (dict "cam" ...))
; button "Leave"  on: (room-leave room)
```

Everything above runs in `ariadne_hello`. The only C++ in the picture is the registration bundle (§8)
and the server.

---

## 4. DSL verb layer

Document-scoped ownership throughout — a transport/bridge/room handle is owned by the opening
document's `se::document_scope` and torn down with it (mirror the existing per-document stream table in
`stream_intrinsics.cpp`). The bridge ctor needs `app&` + `transport&`, which only the host supplies
today; these verbs are what expose it.

### 4a. Media verbs (local; wrap already-merged C++)

| Verb | Wraps | Notes |
|---|---|---|
| `(stream-capture-camera TOKEN [device w h hz])` | `open_camera` + `stream::start_producer` | attach a camera producer to an open stream |
| `(stream-capture-audio TOKEN SPEC)` | `open_audio_capture` / audio-source graph + `start_producer` | `SPEC` = device id, or a composable graph (mix/gain/tone) |
| `(stream-play-audio TOKEN)` | `audio_output` + an auto-pump | subscribe the stream and push frames to the speaker sink (new auto-pump, §10) |
| `(stream-audio-graph SPEC) → token` | `mix_source`/`gain_source`/`tone_source` | optional: build a pure audio graph as a source |

### 4b. Room verbs (the primary high-level surface)

These hide the transport/bridge/relay behind a room abstraction; a conferencing `.ari` uses only these.

| Verb | Effect |
|---|---|
| `(room-join SERVER-ENDPOINT ROOM-ID [profile]) → room` | join the room's state cluster, create the local bridge/transport, publish my participant subtree, get the roster + chat channels. Mode (sfu/mesh) comes from the server. |
| `(room-leave room)` | stop my servings/subscriptions, remove my participant subtree, drop the bridge. |
| `(room-publish room {kind token})` | advertise an open local stream (camera/audio) into the room; in SFU mode `serve`/`pull` it to the relay, in mesh mode make it available for peers to pull. |
| `(room-unpublish room kind)` | stop advertising it. |
| `(room-subscribe room PID KIND) → proxy-token` | open a proxy for participant `PID`'s stream; wire the inbound frames (from relay in SFU, from the peer in mesh). Feed the token to `stream_view` / `stream-play-audio`. |
| `(room-roster room) → [ {pid name flags…} … ]` | current participants + their published kinds + flags; backs the tile grid via `repeat`. |
| `(room-flag room PID KEY)` / `(room-set-flag room {mic cam hand …})` | read any participant's flag; write only my own (server-validated). |
| `(room-chat-send room TEXT)` / `(room-chat-stream room) → sub` | the room's ordered **chat stream** — send a line, or subscribe (backlog + live). Rendered in the UI; composing needs the `input_text` widget (§9). |

### 4c. Low-level transport verbs (escape hatch / mesh / tooling)

For a server to be scriptable, for P2P without a server, and for tests. Thin wrappers over the merged
bridge/transport:

* `(stream-transport-open [port] [tls]) → {handle endpoint}` — wrap `stream_transport_grpc::start()`.
* `(stream-serve TOKEN ENDPOINT)` / `(stream-serve-port TOKEN)` — wrap `serve` / `serve_port`.
* `(stream-receive STREAM-ID PROXY)` / `(stream-pull ENDPOINT PARAMS) → proxy-token` — wrap
  `receive_into` / `pull_from`.
* `(stream-auto-serve TOKEN)` / `(stream-request-remote PARAMS) → proxy-token` — wrap `auto_serve` /
  `request_remote`.
* `(stream-list ROOT) → [id…]` — enumerate `<root>.streams` descriptors (discovery helper).

---

## 5. Server-side state tree + access control

The room lives in the replicated `cvc::state` tree. The **target** is: the server owns and validates
it; clients write **only their own participant subtree** and read the rest. **This target is not
enforceable on today's cross-node security model** — §5c is the honest reality, and §5d is the design
decision that closes the gap. (This is the hardest part of the project and the one most worth getting
right.)

### 5a. Tree layout

```
conf/
  rooms/
    <room-id>/
      meta/            { mode: sfu|mesh, created, title, relay-endpoint }   ← server-owned
      roster/          ← server-owned index: <pid> → { name, joined, present }
      participants/
        <pid>/         ← owned by participant <pid> (and the server)
          profile/     { name, avatar }
          flags/       { mic, cam, hand, screenshare }
          streams/
            <kind>/    descriptor: { token, format_desc, endpoint-or-relay-slot }
          presence/    { last-heartbeat }
      chat/            append-only log channel (messaging, not tree)
      control/         ← server-only (admission, kicks, mode changes)
```

### 5b. Target ownership model

* **Server authority.** `meta/`, `roster/`, and `control/` are written only by the server. A joiner is
  *admitted* by the server (allocated a `<pid>`, granted a scope), not by self-insertion into the
  roster. The server reaps a participant's subtree on leave/timeout.
* **Participant-owned subtree.** A client may affect only `participants/<self-pid>/` — its profile,
  flags, and stream descriptors. Everything else is read-only to it.
* **Read access.** Every participant reads the whole room (roster + others' descriptors + flags) to
  build the tiles. Chat and offer/answer are messaging channels scoped to the room.

### 5c. The reality today (two systems, not wired together)

The code probe found **two independent access systems that are not connected**, and the cross-node one
is permissive by default:

* **In-process DSL grants (`apply_chroot` + `link:` / `channels:`)** sandbox and alias nodes *inside a
  single app's tree*, enforced locally by link resolution + a read-only flag (a `ro` transparent link
  reads the target but routes writes to the caller's own node). This is exactly the "read a subtree you
  don't own" primitive — **but it is in-process only.** It does **not** extend to, or constrain, what a
  peer may do across the replication boundary.
* **Cross-node replication (`state_cluster_shard` + transports)** is, by default, **trust-all-peers**:
  authority/write-policy/conflict enforcement are all **off** (`_enforce_* = false`), so any peer's
  mutation to any path is applied (only loop/seen detection runs). Worse, the writer identity
  (`origin_node_id`) is **self-asserted** by the sender and trusted by the receiver, with **no binding**
  to the transport credential. The gRPC bearer token / mTLS authenticate the *connection to the whole
  cluster*, not a node, path, or scope. `request_snapshot` serves **any** subtree in full to **any**
  same-cluster peer.
* A receiver-side **`state_write_policy`** (prefix → allowed writer `node_id`s) **does exist**, but it
  is **off by default, enforced only on inbound remote mutations** (local writes are ungated), and
  **not distributed** — each node must hand-install the same table (unlike delegations, which journal
  cluster-wide). Because `origin_node_id` is forgeable, it's only as strong as node-id honesty.
* There is **no roster primitive**: `children()` is a recursive deep walk (no immediate-children
  accessor), there's no participant-id allocation and no presence reaping (though an `expireAt` TTL
  primitive exists to build on).

**Consequence:** "clients write only their own subtree; server owns the roster" **cannot be enforced
against an untrusted (esp. browser) client today.** The in-process grant model doesn't cross the wire,
and the replication layer trusts peers.

### 5d. Design decision — server-mediated writes for authority state

Rather than try to harden trust-all gossip into a client-safe mesh, make the **server the sole writer
of shared/authority state**:

* **Clients never replicate into authority paths.** `meta/`, `roster/`, and even
  `participants/<pid>/{flags,profile,streams}` are written **only by the server**. A client changes its
  state by **submitting an intent** over a request channel (the message bus) — e.g. `set-flag mic=off`,
  `publish camera <descriptor>` — which the server **validates** (is this the right `pid` for this
  connection? is the value legal?) and then writes. Clients receive the room tree **read-only** via
  replication (server → client fan-out).
* This gives a single interception point (the server) for validation, needs **no** forgeable-identity
  fix for v1, and matches the central-server topology. It also works for **browser clients** unchanged
  (they already only read + send intents).
* **Binding to connection identity** is what makes "the right `pid` for this connection" meaningful: on
  admission the server mints a per-participant credential and maps connection → `pid`, so an intent's
  authority comes from *which connection it arrived on*, not a self-declared field.
* The existing primitives then harden the **trusted native / mesh** case as a later increment:
  `write_policy` (now meaningful once identity is bound), HLC last-writer-wins conflict resolution
  (`set_resolve_conflicts`, also off today), and the delegation control-plane as the model for
  distributing policy.

### 5e. What exists vs. what must be built

**Exists (building blocks):** `write_policy` trie + `ingest_remote` enforcement point; HLC-LWW conflict
resolution with `(origin_node_id, sequence)` tie-break; per-connection gRPC bearer token + optional
mTLS; the delegation control-plane op pattern (journaled cluster-wide) as a template; `expireAt` /
`sweepExpired` TTL; `data<T>()` + per-shard codecs for typed descriptors; in-process `link:`/`channels:`
grants for the *client-local* document sandbox.

**Must be built (the room's new security work):**
1. **Connection↔identity binding** — admission maps a connection to a `pid`; intents/writes derive
   authority from the connection, not a self-asserted `origin_node_id`.
2. **Admission hook** — at join/snapshot: allocate `pid`, grant scope, check capacity/ban/role, and
   gate what subtree a peer may pull (snapshot currently serves everything).
3. **Server-mediated write path** — the intent channel + validator + sole-writer discipline (§5d).
4. **Roster primitive** — immediate-children enumeration, `pid` allocation, presence/heartbeat reaping
   of participant subtrees (drive `expireAt`).
5. **(Later) distributed write-policy / bridged grants** — to make the *mesh* / trusted-native case
   safe without a central writer.

### 5f. Identity / admission

Join carries a credential. v1: a room bearer token the server checks at admission, from which it mints
the per-connection `pid` binding (§5d). Richer identity (OIDC via the existing SSO) and roles
(host/moderator/guest, capacity, bans) are admission-time hooks layered on top.

---

## 6. Data plane & transport

### 6a. Media transports
* **Native ↔ native / native ↔ server:** the merged gRPC `StreamTransport` (serve/receive,
  serve_port/pull_from), TLS + chunking already done. **No new work.**
* **Browser:** **WebRTC is the production media plane** (§6b). The browser captures via `getUserMedia`
  (hardware **H.264/Opus**, echo-cancel / noise-suppress / AGC built in) and the SFU forwards its RTP. A
  **WebSocket** path — raw frames over our existing chunked envelope — is kept as a bring-up/debug
  loopback and is the natural **signaling** carrier.

### 6b. WebSocket vs WebRTC — why WebRTC (answering the obvious question)
WebSocket is tempting: it reuses our `StreamTransport` envelope verbatim (trivial server bridge), needs no
ICE/SDP/SRTP, and raw frames are easy to inspect — genuinely useful for **bring-up and debugging**. But it
runs over **TCP** (head-of-line blocking collapses video under loss), has **no congestion control**, and
forces raw or wasm-encoded frames. **WebRTC** gives UDP/SRTP, congestion control, jitter buffering, **NAT
traversal** (ICE/STUN/TURN), and **hardware H.264/Opus + audio processing** from the browser for free, and
is the only realistic browser path to 8→200 via an SFU. **Decision: WebRTC for production media; WebSocket
for bring-up/debug + signaling.**

### 6c. The wasm client already has the engine — it needs transport + a WebRTC leg
Verified: the `cvc::state` **engine** and the **stream core** are **unconditionally compiled** (not
gRPC-gated), so a wasm build already contains them. The browser gaps are purely transport:

* **Signaling/control:** a new **WebSocket `state_transport` / `transport_kind`** carrying the existing
  `Frame`/`Mutation`/`Message`/`Snapshot` schema (the control transport is already an abstract, pluggable
  interface; `{inproc, ipc, grpc}` are all non-browser). The message bus rides it (join/leave, SDP/ICE,
  roster, chat). WebSocket is fine here — control is low-bandwidth.
* **Media:** a **WebRTC leg** in the client ↔ the SFU (§6d). `cvc::net` is request/response-only and
  cannot carry realtime, so it is not the carrier.

### 6d. WebRTC stack — integrate, don't build (dependencies + WASI fit)
SRTP/ICE/DTLS/RTP is far too much to write; integrate a C++ WebRTC library through **cvcpkg** (hermetic).

| Library | Lang / build | Core dependencies | Hermetic + WASI fit |
|---|---|---|---|
| **libdatachannel** (recommended) | C++17, **CMake**, MPL-2.0 | **libjuice** (ad-hoc ICE, BSD, *no GLib*) or libnice; **usrsctp** (userspace SCTP); **libsrtp** (media); TLS = **mbed TLS** / OpenSSL / GnuTLS; plog | **Best.** Small, CMake → drops into cvcpkg; deps are small C/C++ with a swappable, portable TLS (mbed TLS ports to wasm/WASI cleanly); no GLib. |
| libwebrtc (Google) | C++, **GN/Ninja** | BoringSSL, abseil, libyuv, libvpx, opus, (ffmpeg) | **Worst.** Non-CMake build is painful to make hermetic; huge; assumes full POSIX. |
| mediasoup | C++ worker + **Node.js** controller | libuv, openssl, libsrtp, usrsctp, abseil | C++ worker is fine, but the Node orchestration layer is off our `.ari`-server / WASI path. |
| Pion | **Go**, MIT | pure-Go (no C deps) | Go has native `wasip1`, but it's off our C++/cvcpkg path. |

**Decision (was the top open item): libdatachannel** with **libjuice + usrsctp + libsrtp + mbed TLS**,
packaged in cvcpkg, our SFU forwarder built on top and wired to the room tree. It is the only option that
is C++/CMake/hermetic-friendly, small, and whose dependencies are the most portable toward WASI (§6f). The
**browser** side uses the native `RTCPeerConnection` API via an Emscripten/JS seam (no wasm RTC library).
NAT traversal does **not** use STUN/TURN — see §6g.

### 6e. Media codecs
* **Raw (rgba8 / f32 PCM)** — debugging + the WS loopback; trivially inspectable; what the merged stream
  core already carries.
* **H.264 / Opus** — production. In the browser: free via WebRTC/`getUserMedia`. On native: encode/decode
  via the **ffmpeg** path (existing `ffmpeg-lgpl` recipe / process-pipe plan) so the native leg and the
  SFU interoperate with browser codecs. `format_desc.codec` already tags each stream, so raw and encoded
  coexist per stream.

### 6f. Server runtime & WASI
Native is the first target; **WASI is a committed goal** (fill the cvcpkg WASI cross-toolchain gaps —
currently broken). Better news than last pass: **WASI 0.2 (Preview 2) defines UDP *and* TCP sockets and
wasmtime implements both** — UDP is no longer the hard blocker it was in Preview 1. The remaining gaps for
a C++ WebRTC SFU under WASI are *practical, not fundamental*: the **wasi-libc POSIX-socket shim over
`wasi:sockets`** is still maturing, **threads** (`wasi-threads`) that WebRTC stacks assume, and building
libjuice / usrsctp / libsrtp / mbed TLS with **wasi-sdk** (mbed TLS already ports cleanly; the rest are
small C). Pragmatic path: ship the **native** server first; bring up the WASI build of the **control +
SFU-policy core** early (no tight UDP there); the **media/UDP leg follows** as the socket shim +
`wasi-threads` mature. libdatachannel's small, portable deps (§6d) are the most likely to make that port.
The server stays a native media core + a server-side `.ari` controller (§8).

### 6g. NAT traversal, relay & end-to-end encryption (no STUN/TURN)
We deliberately **avoid STUN/TURN**. The central server/cluster is publicly reachable, so:

* **Relay through the server by default.** A client — including anyone behind NAT — connects *out* to the
  server/cluster it is already attached to, and the **SFU relays**. browser↔server WebRTC needs no
  STUN/TURN: the server advertises its own reachable address and the client dials it.
* **P2P without STUN/TURN — reverse-connect.** For the mesh/P2P path, coordinate connection *direction*
  over the signaling bus with a **DC++-style `ConnectToMe` / reverse-connect** handshake: when one peer
  is behind NAT, the server tells *that* peer to initiate the outbound connection (active/passive roles,
  exactly as in our **eiskaltdcpp / verlihub** DC++ client+hub). No ICE/STUN hole-punching is needed when
  at least one side is reachable; the server is the rendezvous.
* **End-to-end encrypted relay.** When media relays through the server it is **encrypted so the server
  cannot read it** — only the clients hold keys (reusing our encrypted-relay implementation from
  eiskaltdcpp/verlihub). In WebRTC terms this is **SFrame / Insertable-Streams E2EE**: the SFU still sees
  RTP **headers** (enough to route and to pick simulcast layers) but the **payload is opaque**. Trade-off
  to note: a fully-E2EE SFU cannot transcode or do content-based processing — header-based routing,
  simulcast selection, and active-speaker (via marked headers) still work; anything needing *decoded*
  media does not.

---

## 7. The SFU (server media forwarder, new C++)

Two ingress/egress legs over one internal frame fabric:

* **gRPC leg (native):** reuse the merged bridge — a proxy per publisher (`receive_into`/`pull_from`)
  re-exposes a normal `channel()` that is `serve`/`serve_port`'d to each subscriber (proven in
  `stream_bridge_test`). No relay class exists yet; package the proven proxy→serve pattern as a helper
  (size pools for N via `subscriber_depth`/`expected_subscribers`; unique `<pid>/<kind>` ids so the
  `(root_path,id)` token never collides; drop-at-source so a slow subscriber never back-pressures).
* **WebRTC leg (browser + native):** a forwarder built on the WebRTC stack (§6d) — terminate each peer's
  RTP and forward to subscribers; the browser's simulcast layers let the SFU choose a resolution per
  subscriber.
* **Internal fabric:** both legs normalize to the same `frame` / `format_desc`, so a native publisher can
  be seen by a browser subscriber and vice-versa.

**Two scale regimes (both wanted):**

* **Interactive conference** — everyone can publish. MVP = full fan-out for 8; scale to **~200** via
  **simulcast/SVC** layer selection per subscriber, **active-speaker** switching, and pausing
  off-screen/backgrounded tiles. The selection *policy* is server-side `.ari`; the packet forwarding is
  native.
* **Broadcast / single-streamer** — one (or a few) publishers to **thousands** of passive viewers
  (webinar / livestream). This is a fan-out *tree*, not full-mesh forwarding: the viewer count is served
  by **cascaded / edge SFUs** (a relay hierarchy), closer to a CDN than to one node. The MVP targets the
  interactive regime; broadcast-to-thousands is its own scaling workstream (§11).

**Chat stream.** Each room carries a lightweight **chat stream** — an ordered message channel on the room
tree/bus — subscribed like any other room resource (backlog + live) and rendered in the UI (§9). It rides
the control plane, not the media plane.

The forwarding primitives are exposed to the DSL (§4c + room-control verbs) so the room controller can be
scripted.

---

## 8. Host: extend `ariadne_hello`

A reusable registration bundle in cvcGL, called by `ariadne_hello` (native) and by the server target:

```cpp
// register_conference(app, runtime): stream intrinsics + gl stream verbs + stream node
// + stream_view widget + capture/playback/room verbs. Guarded by CVC_ENABLE_SDL (capture/
// playback) and CVC_ENABLE_GRPC (transport/room); graceful no-op when a capability is absent.
```

* `ariadne_hello` calls the bundle and switches to `AriRuntime` (or registers the extra verbs on its
  `Runtime`); **enable `CVC_ENABLE_GRPC`** on its native conference build. Keep the generic/core/wasm
  builds green via the guards (no-op when a capability is missing) so `ariadne_hello` stays the generic
  `.ari` host it is today.
* **wasm build:** the bundle registers the room verbs over the **WebSocket control transport** + a
  **WebRTC media leg**; capture/playback use the browser's `getUserMedia`/WebAudio + `RTCPeerConnection`
  seam (new, §10) rather than SDL. Served from **cyberpcangel.com** the same way as
  **cyberpcangel.com/cvcdbg** — the Emscripten bundle with **`-pthread`**, behind the **COOP/COEP**
  headers `SharedArrayBuffer` requires.
* **Server target:** a new `ariadne_conference_server` = a **thin native media core** (the SFU's WebRTC +
  gRPC legs, §7) driven by a **server-side `.ari` room-controller** that runs admission, roster, and
  forwarding policy through the DSL verbs (§4c + room-control verbs). Same headless host pattern as
  `ariadne_hello`; **WASI build** is a goal (§6f). "The server is also an `.ari` app" is the SDK proof —
  users self-host by running the server `.ari` on our surface or their own.

---

## 9. UI (all in the DSL, mostly already possible)

From the UI probe, the conferencing UI is ~90% expressible today:

* **Tile grid** — `Grid` layout + `repeat` over `(room-roster)`, each cell a `type: stream_view`
  window (or `type: stream` quad) bound to a `(room-subscribe …)` token. Self-view tile from the local
  camera stream. ✅ primitives exist.
* **Controls** — `button` with `on: (…)` pure-`.ari` `state_exec` programs calling `room-set-flag` /
  `room-leave`; reactive `enabled_when`/`visible_when` off flags. ✅ exists.
* **Participant list / name labels** — `repeat` + `Text` (bound, colorable for speaking/mute state). ✅
  exists.
* **Chat** — a chat-stream feed (scrollable log via `repeat`) + an **`input_text`** box to compose.
  `input_text` is the one real widget gap (only the key-event path landed) — a Phase 0 prerequisite (§10).

Registration of `type: stream` / `type: stream_view` is host opt-in — included in the §8 bundle.

---

## 10. Build phases (each testable; the arc converges on the browser MVP)

Ordered to de-risk inner-to-outer: prove the room + SFU with native clients first, then WebRTC off the
browser, then the browser itself, then the MVP.

* **Phase 0 — prerequisites (headless).** `input_text` widget; stream→`audio_output` auto-pump
  (`stream-play-audio`); the **gRPC SFU relay helper** (§7) with an N×M fan-out unit test (synthetic
  sources).
* **Phase 1 — media DSL verbs (§4a).** `stream-capture-camera/-audio`, `stream-play-audio`,
  `stream-audio-graph`, document-scoped. Headless with synthetic sources (device capture gated/skipped
  on CI).
* **Phase 2 — room control plane (§5).** `conf/rooms/<room>` schema; admission (connection↔`pid`); the
  **intent channel + server validator** (sole writer of authority state); read-only replication; roster
  primitive (immediate-children + presence reaping). Inproc two-app test **+ adversarial test** (an
  intent targeting another `pid` is rejected).
* **Phase 3 — room verbs + gRPC SFU; native proof (§4b).** `room-join/publish/subscribe/roster/flags/
  leave` over gRPC. Two **native** clients conference end-to-end from one `.ari`. Proves the room model
  + SFU before any browser work.
* **Phase 4 — WebRTC leg, native first (§6d/§6g).** Package libdatachannel (+ libjuice/usrsctp/libsrtp/
  mbed TLS) in **cvcpkg**; signaling over the state message bus; a native peer ↔ server SFU over WebRTC,
  **relayed through the server (no STUN/TURN)** with **E2E-encrypted (SFrame) payloads** the relay can't
  read. De-risks WebRTC away from the browser.
* **Phase 5 — browser bring-up: control + WS loopback (§6c).** New **WebSocket `transport_kind`**; the
  wasm client joins the room **control plane** and renders a tile over a raw-frame WS loopback. Browser
  without WebRTC yet.
* **Phase 6 — browser WebRTC + MVP (§6a).** `getUserMedia` + `RTCPeerConnection` ↔ SFU, **H.264/Opus**;
  render remote tiles; **chat stream** in the UI. **★ MVP:** deploy the server, serve the wasm client
  from **cyberpcangel.com** (the **cvcdbg `-pthread`/COOP-COEP** pattern), **default room, 8-way** browser
  video conference.
* **Phase 7 — scale & robustness (§7).** Interactive toward ~200 (simulcast/active-speaker selection,
  off-screen pause, pool/thread/bandwidth budgets); the **broadcast→thousands** regime (cascaded/edge
  SFUs); reconnection/presence/heartbeat; TLS end-to-end.
* **Phase 8 — mesh + auth hardening + WASI.** `mode: mesh` (WebRTC peer / native serve+pull) with the
  **DC++-style reverse-connect** for NAT (§6g); connection-bound identity + distributed `write_policy`
  for mesh authz; identity (OIDC via SSO / revocable txwtf-style JWT); the **WASI server** build (fill
  cvcpkg gaps, §6f).

---

## 11. Open questions / decisions

**Decided:** WebRTC = production media, WebSocket = bring-up/debug + signaling (§6b); **WebRTC stack =
libdatachannel + libjuice + usrsctp + libsrtp + mbed TLS via cvcpkg**, our SFU built on it (§6d); raw
**and** H.264/Opus (§6e); **no STUN/TURN — relay through the reachable server/cluster, P2P via a
DC++-style reverse-connect, E2E-encrypted (SFrame) relay** (§6g); **both** an interactive conference
(8 → ~200) **and** a broadcast/single-streamer (→ thousands) regime, with a per-room **chat stream** (§7);
mesh works first, auth hardens later (§5, Phase 8); **WASI server committed** (§6f); server as
`.ari`-scriptable as possible (§8); serve the wasm client from **cyberpcangel.com** via the **cvcdbg
`-pthread`/COOP-COEP** setup; MVP = 8-way browser conference in a default room.

**Still open / workstreams:**
1. **Scaling both regimes** — simulcast vs SVC + the active-speaker/tile-size selection policy for ~200;
   the **cascaded / edge-SFU** hierarchy for broadcast-to-thousands; where thread/pool/bandwidth ceilings
   land, and whether broadcast reuses the conference SFU or a separate fan-out tree.
2. **WASI media leg** — WASI 0.2 has UDP, but the wasi-libc socket shim + `wasi-threads` + wasi-sdk
   builds of libjuice/usrsctp/libsrtp are the gaps; sequence the control/policy core first (§6f).
3. **E2EE vs SFU features** — SFrame keeps header-based routing + simulcast working but blocks
   server-side transcoding; confirm that's acceptable (it is for a relay-only SFU).
4. **Identity / authn (point 5 — to flesh out)** — OIDC-via-SSO vs revocable txwtf-style JWT; roles
   (host/moderator/guest), capacity, bans in the roster; mesh authz lands with it (Phase 8).

---

## 12. Where this plugs into existing roadmaps

* Realtime substrate: [`STATE_BINARY_STREAMING.md`](STATE_BINARY_STREAMING.md), developer guide
  [`../STREAMING.md`](../STREAMING.md) (§6 cross-node).
* Distributed state / cluster: [`DISTRIBUTED_STATE_ROADMAP.md`](DISTRIBUTED_STATE_ROADMAP.md),
  [`STATE_LIFETIME_AND_ATOMICITY.md`](STATE_LIFETIME_AND_ATOMICITY.md).
* UI DSL (widgets, `input_text` gap): [`CVCGL-UI-DSL-ROADMAP.md`](CVCGL-UI-DSL-ROADMAP.md).
