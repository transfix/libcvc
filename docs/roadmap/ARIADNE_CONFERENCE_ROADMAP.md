# Ariadne Conference — flagship multi-party video conferencing app

**North star.** A multi-party video conferencing application, written as an **Ariadne `.ari` document**,
hostable by the **`ariadne_hello`** shell on the desktop **and** in the browser (wasm), talking to a
**native (optionally WASI) server** that acts as the room authority and media relay. Camera + mic go
out, every other participant's video + audio comes in, and the whole call — tiles, roster, mute /
camera / leave controls — is expressed in the DSL, not in C++.

Two connection topologies are **first-class** and chosen at runtime per room:

* **Central server / SFU** — clients connect *out* to a relay that fans each publisher's frames to the
  other participants. The default for browser clients and for calls of more than a few people.
* **Peer-to-peer mesh** — native clients connect directly to each other. For small native-only calls
  with no server in the path.

> **Status: SPEC / not started.** This document scopes the work. The realtime substrate it builds on
> (capture, audio, local streaming, the gRPC data plane, the bridge, and most of the UI DSL) is
> already merged — see §1. Nothing in §4–§10 is implemented yet. Sections §5 (server state tree +
> access control) and §6 (transport) are being cross-checked against the live grant/cluster model and
> the wasm networking posture; specifics there may tighten.

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
4. **No browser data plane** (native gRPC can't run in a browser), and two UI holes: no `input_text`
   widget (chat) and the audio sink isn't auto-pumped.

---

## 2. Architecture

```
            ┌──────────────────────────── SERVER (native, maybe WASI) ───────────────────────────┐
            │  room authority: owns conf/rooms/<room> state tree + roster + admission             │
            │  SFU relay: ingests each publisher's stream, fans out to subscribers                │
            │  transport: native gRPC StreamTransport  +  browser-leg terminator (WS/WebTransport) │
            └───────▲───────────────────────▲──────────────────────────────▲─────────────────────┘
       control+data │ gRPC          control+data │ gRPC           browser leg │ (WS/WebTransport)
          ┌─────────┴─────────┐     ┌───────────┴────────┐          ┌─────────┴──────────┐
          │ NATIVE client     │     │ NATIVE client      │          │ BROWSER client     │
          │ ariadne_hello     │ ... │ ariadne_hello      │          │ ariadne_hello.wasm │
          │ .ari conference   │     │ .ari conference    │          │ .ari conference    │
          └───────────────────┘     └────────────────────┘          └────────────────────┘
                     │  P2P mesh mode (native only): clients also serve/pull each other directly
                     └───────────────────────────────────────────────────────────────────────
```

* **Two planes, as today.** *Control* = `cvc::state` replication of the room tree + the message bus
  (join/leave, offer/answer, roster, flags, chat). *Data* = frame bytes over `StreamTransport`.
* **Roles.** A *client* publishes its camera/mic and subscribes to others. A *server* is the room
  authority (owns the roster, admits participants) and, in SFU mode, the media relay. A room runs in
  **SFU** or **mesh** mode; the server advertises the mode, and the same client verb layer drives both
  (in mesh mode, "subscribe participant P" resolves to a direct `pull`/`serve` with P instead of with
  the relay).
* **Browser clients are SFU-only** — they cannot host a server, so they always connect out to the
  relay over the browser transport leg (§6). Native clients can do either.

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
| `(room-chat-send room TEXT)` / chat channel subscription | text chat (needs the `input_text` widget, §10). |

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

### 6a. Native ↔ native / native ↔ server
Use the merged gRPC `StreamTransport` directly: SFU = clients `serve`/`pull` to the relay; mesh =
clients `serve_port`/`pull_from` each other. TLS and chunking already exist. **No new transport work.**
(For same-host native multi-process, `cvc::state`'s `ipc`/UNIX-socket transport is also available, but
that's not relevant to the browser.)

### 6b. What a wasm client already has — and the one thing it lacks
**The good news (verified):** the `cvc::state` replication **engine** (shards, journal, subscription
router, replica, authority map, message bus, snapshot) **and** the in-process **stream core** (frame
pool, channel, registry, producer, codecs, sources) are **unconditionally compiled — not gRPC-gated**,
so they are *already present in a wasm build*. A browser client does **not** need the stack ported; it
needs a **browser-capable transport** plus a server to terminate it. Two legs:

* **Control plane.** `cvc::state` replicates over an **abstract `state_transport`** with pluggable
  `transport_kind ∈ {inproc, ipc, grpc}` — none browser-reachable (`inproc` = same process, `ipc` =
  same-host UNIX sockets, `grpc` = HTTP/2 the browser sandbox can't open). So the browser control leg
  is a **new `state_transport` implementation / `transport_kind`** (WebSocket) carrying the existing
  `Frame`/`Mutation`/`Message`/`Snapshot` schema. The message bus rides the same transport, so this one
  leg also carries join/leave/offer/answer/chat.
* **Data plane.** The frame transport (`stream_transport_grpc`) is **concrete** (not abstracted like
  the control transport) and the bridge is bound to it. So the browser data leg is, for v1, a
  **server-side WS↔gRPC bridge**: the browser speaks WS to the server; the server re-injects frames
  into the native `StreamTransport` + SFU. (Later option: abstract the stream transport the way the
  control transport already is, and add a WS impl client-side.)
* **`cvc::net` can't carry realtime.** It is **request/response-only** (one-shot buffered `send()`),
  native *and* wasm (emscripten-fetch) — it cannot carry a bidirectional frame stream, and its wasm
  HTTP DSL verbs aren't even wired yet. Realtime needs a real bidirectional carrier.

Browser carrier options (used by both legs over one connection):

| Option | Pros | Cons |
|---|---|---|
| **WebSocket** (recommended v1) | universal, trivial client, easy server bridge, reuses our chunking | TCP head-of-line, no congestion control |
| **WebTransport** (HTTP/3 / QUIC) | datagrams + streams, low latency | newer server stack; browser support maturing |
| **WebRTC** | NAT traversal + congestion control, true media | heavy; ICE/STUN/TURN; large surface |

**Plan:** v1 = one **WebSocket carrier terminated at the server**, multiplexing the control leg (new
`transport_kind`) and the data leg (WS↔gRPC bridge), reusing the existing chunking + `format_desc`.
WebTransport/WebRTC are later upgrades behind the same room verbs — the `.ari` doesn't change.

### 6c. Server runtime
Native is the committed target. **WASI** is exploratory (the WASI cross-toolchain is currently noted as
broken; revisit). The server is a new target (§8) built from the low-level transport verbs + an SFU
relay helper (§7) — it can be a C++ host or itself scripted in `.ari` (open question, §11).

---

## 7. SFU relay helper (new C++)

No relay/SFU class exists; the forwarding mechanism does (a proxy filled by `receive_into`/`pull_from`
re-exposes a normal `channel()` that can be `serve`d onward — proven in `stream_bridge_test`). The
helper packages that pattern:

* Hold one **ingest proxy per publisher** (sized for N subscribers via `subscriber_depth` /
  `expected_subscribers`).
* On each subscriber, `serve`/`serve_port` the matching proxy to it (one pusher per downstream).
* Manage **globally-unique ids** (`<pid>/<kind>`) so the token = `(root_path,id)` scheme never collides
  (the mesh gotcha from the topology analysis).
* Drive add/remove from the roster (join publishes → relay ingests; subscribe → relay serves).
* Drop-at-source throughout (inherited from the channel), so a slow subscriber never back-pressures a
  publisher.

Exposed to the DSL via the low-level verbs so the relay node can also be scripted.

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
* **wasm build:** the bundle registers the room verbs over the **browser transport leg** instead of
  native gRPC; capture/playback use the browser's getUserMedia/WebAudio seam (new, §10) rather than SDL.
* **Server target:** a new native `ariadne_conference_server` (room authority + SFU relay + WS
  terminator). Reuses the low-level transport verbs and the relay helper.

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
* **Chat** — ❌ needs the `input_text` widget (only the keyboard *event* path landed). Small UI
  prerequisite (§10).

Registration of `type: stream` / `type: stream_view` is host opt-in — included in the §8 bundle.

---

## 10. Build phases (each a PR / small stack, each testable)

Ordered so something runs end-to-end as early as possible and each phase is independently verifiable.

* **Phase 0 — prerequisites.** (a) `input_text`/`input_int` widget; (b) stream→`audio_output` auto-pump
  (`stream-play-audio` core); (c) **SFU relay helper** (§7) as a C++ class with a unit test (N
  publishers × M subscribers, synthetic sources). *Headless.*
* **Phase 1 — media DSL verbs.** `stream-capture-camera`, `stream-capture-audio`, `stream-play-audio`,
  `stream-audio-graph` (§4a), document-scoped. Tested headless with synthetic sources (camera/mic gated
  + device-skipped on CI).
* **Phase 2 — room state tree + server-mediated access (§5).** The `conf/rooms/<room>` schema;
  admission (connection↔`pid` binding + token check); the **intent channel + server validator** as the
  sole writer of authority state; read-only replication to clients; a roster primitive
  (immediate-children + presence reaping). Tested with the in-process state transport (two apps, one
  room) **plus an adversarial test**: a client intent targeting another `pid` is rejected.
* **Phase 3 — room DSL verbs (§4b) over native gRPC + SFU relay.** `room-join/leave/publish/subscribe/
  roster/flags`. Two-native-client integration test: A publishes, B subscribes via the relay, frames
  cross; roster reactive.
* **Phase 4 — host + server (§8).** Extend `ariadne_hello` (bundle + gRPC on) and ship
  `ariadne_conference_server`. A runnable **native** multi-party call from a single `.ari` (synthetic or
  camera) end-to-end.
* **Phase 5 — browser/wasm client (§6b).** WebSocket browser leg terminated at the server; wasm
  `ariadne_hello` joins a room, sends + receives frames, renders tiles. getUserMedia/WebAudio capture
  seam.
* **Phase 6 — P2P mesh mode (§2).** `mode: mesh` rooms: `room-subscribe` resolves to direct peer
  `pull`/`serve`; unique-id scheme; native-only.
* **Phase 7 — polish.** TLS end-to-end, reconnection/presence/heartbeat, chat, bandwidth/drop tuning,
  encoded media (H.264/Opus via the ffmpeg process-pipe path) to replace raw rgba8/pcm.

---

## 11. Open questions / decisions

1. **Browser transport** — WebSocket for v1 (recommended) → WebTransport/WebRTC later? WebRTC would
   give real NAT traversal + congestion control but is a large surface.
2. **Mesh authorization.** Confirmed: replication is trust-all-peers with self-asserted
   `origin_node_id`. The server-mediated model (§5d) secures the *central-server* case without a
   forgeable-identity fix. The **mesh** case still rides trust-all gossip — do we invest in
   connection-bound identity + distributed `write_policy` to make peer-to-peer rooms safe against a
   dishonest peer, or restrict mesh to trusted/native deployments?
3. **WASI server** — worth pursuing given the WASI toolchain is currently broken, or stay native?
4. **Media codecs** — raw rgba8/pcm for v1 (simple, bandwidth-heavy) vs encoded (H.264/Opus) from the
   start. Raw v1, encode in Phase 7 via the existing ffmpeg-process-pipe plan.
5. **Server scriptability** — is `ariadne_conference_server` a C++ host, or itself an `.ari` document
   driving the low-level transport verbs + relay helper?
6. **Identity/auth** — room password/bearer for v1; OIDC (existing SSO) later. Roles
   (host/moderator/guest) in the roster?
7. **Participant ceiling** — target N for the SFU (tile layout, pool sizing, pusher-thread count all
   scale with N).

---

## 12. Where this plugs into existing roadmaps

* Realtime substrate: [`STATE_BINARY_STREAMING.md`](STATE_BINARY_STREAMING.md), developer guide
  [`../STREAMING.md`](../STREAMING.md) (§6 cross-node).
* Distributed state / cluster: [`DISTRIBUTED_STATE_ROADMAP.md`](DISTRIBUTED_STATE_ROADMAP.md),
  [`STATE_LIFETIME_AND_ATOMICITY.md`](STATE_LIFETIME_AND_ATOMICITY.md).
* UI DSL (widgets, `input_text` gap): [`CVCGL-UI-DSL-ROADMAP.md`](CVCGL-UI-DSL-ROADMAP.md).
