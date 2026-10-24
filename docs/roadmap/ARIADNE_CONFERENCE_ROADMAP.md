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
   (none in-tree today), with a WebSocket fallback for bring-up/signaling. The two earlier UI
   holes — the `input_text` widget (chat) and audio auto-pump — have since landed (#636, #634).

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
      scene/           ← server-owned: the SHARED 3D world every client renders (§5g).
        env/           { background, ground, lights }
        objects/       <id> → { source(uri), transform, material }
      participants/
        <pid>/         ← owned by participant <pid> (and the server)
          profile/     { name, avatar }
          pose/        { pos, orient }   ← participant-owned: avatar position/orientation in scene/ (§5g)
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
   gate what subtree a peer may pull (snapshot currently serves everything). *S7 (§5h.1) shapes the gRPC
   snapshot reply and the WebSocket broadcast by the requester's registry filter, but that is delivery
   policy, not access control — a peer with no filter still gets everything — and the admit snapshot,
   sent before Hello, is still unfiltered.*
3. **Server-mediated write path** — the intent channel + validator + sole-writer discipline (§5d).
4. **Roster primitive** — immediate-children enumeration, `pid` allocation, presence/heartbeat reaping
   of participant subtrees (drive `expireAt`).
5. **(Later) distributed write-policy / bridged grants** — to make the *mesh* / trusted-native case
   safe without a central writer.

### 5f. Identity / admission

Join carries a credential. v1: a room bearer token the server checks at admission, from which it mints
the per-connection `pid` binding (§5d). Richer identity (OIDC via the existing SSO) and roles
(host/moderator/guest, capacity, bans) are admission-time hooks layered on top.

### 5g. Shared 3D scene + avatars (NEXT WORK — user-requested)

The conference is not just floating video windows: it is a **shared 3D space** that every client renders
**from its own camera viewpoint** (orbit/fly locally; the camera is never replicated). The scene itself is
**server-authoritative** — the room owns it as replicated state and all clients draw the same world.

* **Scene tree.** `conf.rooms.<room>.scene.*` (server-owned, replicates on the existing `"conf"` shard
  scope). The server seeds `scene.env` (background, ground, lights) and `scene.objects.<id>`
  (`source` URI + `transform` + `material`). Clients render it via a `scene:` block in the conference
  `.ari` fed from that subtree — likely a small custom scene node type (the `agents`/`count:` pattern in
  §3 and the example scenes), since the live tree, not the document, is the source of truth.
* **Avatars.** Each participant writes its own `participants.<pid>.pose` (`pos`, `orient`) — e.g. derived
  from its camera — which replicates; a custom "avatars" scene node reads the roster + each pose and draws
  a marker/model per participant, so **users see each other moving around the scene** (the `agents` pattern
  again, keyed by `pid`). Pose is participant-owned (§5b); the scene env is server-owned.
* **Why this matters for correctness, not just looks.** It was the broad (accidental) replication of each
  client's per-document `graphics.root` that gave the earlier build a de-facto shared scene; scoping
  replication to the room tree (the name-collision fix) correctly removed that. This section makes the
  shared scene a **first-class, server-authoritative** feature instead of a replication side effect.
* **Sequencing.** MVP first = a minimal server-seeded environment (ground + grid + a reference object) to
  prove the shared-scene render path end-to-end; then layer avatars (pose publish + the avatars node).
  Schema impact (a `scene:` driven from state, a custom scene-node type) is coordinated with the host-caps
  / `requires:` work (transfix/libcvc#754) so the terminal hosts gate it cleanly (`requires: [gl.scene]`).

---

### 5h. The holon — the shared scene formalized (authority, prediction, LOD, composition)

§5g's shared scene is the first rung of a larger unit, the **holon**: a bundle of
`{ cvc::world_clock + cvc::physics::world (Jolt) + scene/pose state }` that is **server-authoritative,
client-replicable, and composable**. The clock lives *inside* the holon (per-holon, not an app global), so
a node interacting with several holons runs several clocks. The DSL, media and render *observe* a holon;
only its deterministic core (clock + physics) replicates and reconciles. All three primitives exist today
independently (`inc/cvc/core/world_clock.h`, `inc/cvc/physics/*`, the cvcGL retained scene graph) — nothing
bundles them yet.

* **Driver.** The frame loop is the only driver (as in WORLD-CLOCK-SYNC): one `clock.advance(wall_dt)` →
  `{steps, alpha, first_tick, dropped_steps}`; an `on_tick` pass runs each due fixed quantum (deterministic
  physics prediction); an `on_frame` pass runs once (authoritative interpolation, DSL, media pump, render
  tween via `alpha`). `fixed_dt` is **constant** (1/120 for a navigator-free room); CPU load varies the
  *step count*, never dt — which is what makes the client's prediction bit-reproducible against the server
  (`physics/det_math.h`). `clock_follower` already gives per-follower `ticks_per_step`/`steps_due`/`alpha`.
* **Authority & reconcile.** Server holon = truth; client holon = a replica that predicts locally and
  reconciles on each authoritative snapshot (soft-blend while tracking, `seek_tick` + replay when it
  diverged or `dropped_steps` fired). **Pose is server-authoritative with client proposal + veto +
  override**: the client writes a proposal to `participants.<pid>.intent.*` (replicated), the server
  validates it as an input to that avatar's physics body and writes the authoritative `…pose/body` —
  accept, veto (clamp/reject), or override at will (teleport, re-mass, re-parent). "Respect the veto" =
  authoritative wins on reconcile. This is the server-mediated-write path (§5d) applied per avatar.
  *(No client-prediction/reconcile/rollback loop exists today — it is new; distributed state resolves
  last-writer with vector clocks, not predict/rollback.)*
* **LOD (realtime for large scenes).** A replica is a *scoped, decimated* view, not a full copy: **spatial
  interest** picks *what*; **geometry LOD** (landed, PR #249+: `lod/select.h` rung/budget solver,
  `lod/pyramid.h`, QEM `geometry/simplify.h`, `lod/tiles.h`, HDF5 `lod/store.h`, `LodGraphicsNode`) picks
  *how much*; reconcile fixes what was skipped. **H1's spatial interest is built (S7, §5h.1)** and it is
  *not* the `scene.<cell>`-near-the-camera filter this bullet first proposed: it is a **relay-authoritative,
  avatar-centric** area of interest over the *participants* subtree (the camera is never replicated, §5g,
  so the relay centres interest on the authoritative avatar pose it already holds), enforced outbound per
  receiver by the relay. Cell interest over `scene.*` moves to H3 (§5h.1 says why). **Physics LOD is new**
  (zero hits today) — a sim-detail ladder (collision-hull decimation, substep/rate reduction,
  freeze-to-kinematic for distant/subordinate bodies) + re-hydrate when a body re-enters interest.
* **Composition / holarchy.** A server may **subdelegate a sub-volume** `V` to a subordinate server running
  a child holon (its own clients/streams/clock); the parent becomes a **client of the child that keeps
  ultimate authority**. Two axes: **scene-graph** — mount the child's replicated scene as a frame node at
  `V`'s transform (the `physics-world-frame = scene node` + render-origin-offset mechanism already exists
  for city-scale), presented at the parent's chosen copy/LOD; **physics** — authority **handoff** at `∂V`
  first, then cross-boundary kinematic proxies (research-grade). Subdelegation rides
  `state_delegation_manager` (leases: delegate/renew/**revoke**, longest-prefix routing) with a
  **spatial-volume → path-prefix** mapping (`state_brick_manifest::bricks_in_region` is the spatial index)
  and a **new parent override/veto primitive** (leases give revoke, not per-write override; build on
  `write_policy`/`state_authority_map`).
* **Cross-node clock sync.** `world_clock_binding` already publishes `sim.tick/time/fixed_dt/realized_speed/
  dropped` to the tree; replicate them and the replica tracks the remote authoritative tick — `seek_tick`
  on hard correction, `set_scale` nudge toward the remote `realized_scale()` for drift (time dilation),
  server-set `sim.mode/speed` replicated for **shared pause/slowmo** (local pause stays local).
* **Hard prerequisite — LANDED (this bullet was stale).** It read: "`data()` object replication is a known
  DISTRIBUTED_STATE gap (§52: only scalar value/child changes journal; `set_data` is a no-op pending a
  codec)." That path has landed on all four transports — codec-encoded `set_data` (inline or a
  content-addressed `blob_ref`), snapshot capture + replay, and an automatically driven
  `state_data_hydrator` (DISTRIBUTED_STATE_ROADMAP.md Phase 10, status corrected 2026-10-22). Its residual
  items (WebSocket late-joiner blob hydration, the async hydrator install guard, oversized `value()`
  offload) are tracked there. It no longer gates the phasing; H1 replicates scalar leaves only.

**Phasing.**
- **H0 — substrate:** close `data()` object replication (DISTRIBUTED_STATE §52 — **LANDED**, see the
  prerequisite bullet above); spatial-interest filter (**LANDED in S7** as the per-peer delivery
  predicate on `state_peer_registry`, §5h.1); confirm the deterministic physics path (`det_math.h`).
- **H1 — single holon** (the §5g scene + avatars, formalized): bundle `{clock,physics,scene}`;
  server-authoritative; client replica with prediction + reconcile; pose propose→veto→override; geometry
  LOD + spatial interest (**S7, §5h.1**: avatar-centric AOI on the relay + per-frame mesh LOD in
  `room_view`). *(Corrected: this item used to say H1 uses a "per-camera `RenderView` +
  `VisibilityMask`, both already exist". They exist, but they drive the separate `cvc::gl::Node` graph
  (`nodes.h` / `traversal.h`), which neither `SceneGraph` nor `room_view` uses; arigl has one view and
  one camera, and LOD selects from that renderer's camera through `SceneGraph::selectLOD`. Per-camera
  views wait for a PiP or minimap — CVCGL-UI-DSL-ROADMAP.md §9.5, stage C.)*
- **H2 — physics/sim LOD + re-hydrate; cross-node clock tracking** (seek/scale, shared pause).
- **H3 — subdelegation / holarchy:** spatial→path delegation on the lease manager; parent-as-authority-
  client + override primitive; live-remote-holon scene mount at copy/LOD; residency/paging driver
  (VISIBILITY-AND-LOD HLOD §4.2 — designed, not landed). **Scene spatial interest lands here** with the
  paging driver (`scene.cells.<i>_<j>`, per-cell revs, per-cell realize/teardown, the cell index shared
  with `lod/tiles.h`'s partition), as does client- or camera-declared interest built on S7's registry
  filter (§5h.1).
- **H4 — physics boundary composition:** authority handoff at `∂V`, then cross-boundary kinematic proxies
  *(research-grade; last)*.

#### 5h.1 S7 — room spatial interest + geometry LOD (built)

H1's interest and LOD slice. `room_relay_integration_test` (suite `RoomRelayLoopbackTest`, 10 cases:
a relay built as arisrv builds it, real clients on loopback, WS and gRPC mixed in one room) pins the
behaviour end to end, and `arisrv_aoi_bench` measured it (**Measured**, below).

**What it is.** A **relay-authoritative, avatar-centric area of interest**. Each room step the relay
places every participant from the server holon's authoritative pose and decides, per ordered pair
(receiver *r*, subject *q*) and from their XY distance, how much of *q* the client *r* gets. **The camera
is never replicated** (§5g), so interest follows the avatar, not where a client looks, and no client can
widen or narrow it. All filtering is relay-side, outbound and per receiver; client shards keep
`add_interest("conf")` + enforcement as defense in depth. Code: `room_aoi.h` (pure tiers, immutable
snapshot, filter, media decisions, push schedule, presets, audio curve) and `room_aoi_relay.h` (the
driver + `relay_room_step`), under `inc/cvc/ariadne/conference/`.

| Tier | Subject's pose | Camera | Mic |
|---|---|---|---|
| **inner** | live, every room step (~20 Hz) | full rate | yes |
| **middle** | relay **pushes**: 5 Hz while it moves, a 5 s refresh while still | **MJPEG thumbnails at 2 fps** | yes |
| **outer** | relay pushes, as for middle | none | none |

Tiers have hysteresis: 2 m enter/exit bands; an upgrade is immediate (outer→inner in one step); a
downgrade moves one tier, and only after the distance has stayed past the exit radius for 1 s.

**Presets** (`aoi_params`; a default-constructed one *is* the conference preset):

| Preset | inner enter/exit | middle enter/exit | Client audio | Default for |
|---|---|---|---|---|
| `conference` | 30 / 32 m | 40 / 44 m | gain 1 (the fade at 38–40 m is never reached) | the seeded 20 m room |
| `hall` | 10 / 12 m | 16 / 18 m | `min(1, 10/d)`, fading to 0 over 14–16 m | `arisrv --scene large` |

Shared defaults: 1 s downgrade dwell; pushes every 24 ticks at the 120 Hz holon quantum (5 Hz; with one
6-tick room step of jitter that fits `holon_config::max_peer_span_ticks` = 30, so a pushed peer never
stutters); a 600-tick (5 s) refresh; 2 fps thumbnails; `video_full_max` 0 (every inner camera at full
rate); a 2 m audio fade. The conference radii exist so the 20 m room is provably unchanged: the plane is
[-10,10]², its diagonal 28.3 m, so every pair is inner at first sight and stays inner (pinned by
`RoomAoiTest.ConferencePresetCoversDefaultRoom`). The relay publishes the active parameters once under
`conf.rooms.<room>.aoi.*`; clients read them about once a second for their audio gain. An absent
subtree (an older relay) means gain 1; with `--aoi off` or `--s7 off` the relay writes them disabled
(`aoi.enabled` 0), also gain 1 — so a client that lived through an earlier relay's AOI and kept its
leaves follows the restarted relay. The fade ends at `middle_enter`, the *upgrade*
radius, so the relay's cut (outer) and its resume (back to middle) both happen in silence.

**State: a deny-only predicate that fails open.** Per receiver, the relay installs an
`aoi_state_filter` on both transports' peer registries. Under `conf.rooms.<room>.participants.<q>`
it denies only: another participant's `intent` (only the relay and its owner read it); *q*'s whole
`pose` subtree, `server_tick` included, unless the pair is inner; and a departed *q*'s `pose` and
`intent` (the soft-left ghost). Everything else delivers — the receiver's own subtree,
presence/profile/net/streams/voice of every participant (the roster stays room-wide, §5b; a departed
one's `net` and `voice` too, so an in-process reconnect's writes made before its rejoin lands are never
withheld), `scene`, `server`, `aoi`, unknown pids and unknown leaf families — so a new family is never
dropped silently. The relay remembers the most recent 1024 departures; an older ghost falls back to the
unknown-pid rule. A pid that is no C identifier (the room tree takes any child name; a registry entry
cannot) is filtered as a subject but never registered as a receiver — no connection can carry it.

**Pose pushes.** A non-inner subject's pose reaches the receiver as a targeted snapshot
(`state_transport::push_snapshot_to`) carrying its whole pose subtree, with `server_tick` re-stamped to
the relay holon's *current* tick and written last: the tree's heartbeat tick for a still subject can be
~120 ticks old, and a client re-aligns its whole clock (and rubber-bands) on more than 24 ticks of skew.
A pair entering inner, and a new pair that starts middle or outer, is pushed at once. `relay_room_step`
sends every push **before** the step's `drain_local` + publish, so on each connection a push precedes
that tick's live writes.

**Media gate — MJPEG-only thumbnails.** The driver installs `media_endpoint::set_media_gate` on both
relay transports; the router's fan-out asks it once per receiver per frame. Outer drops everything. Only
stream `camera` in codec `mjpeg` is ever thumbnailed (MJPEG is intra-only, so any subset of frames
decodes): always in middle, and in inner only under the opt-in `--aoi-video-max K` cap (nearest *K*,
swapped only past a 1 m margin). Every other stream and codec — the mic, H.264, an MJPEG screen-share
under another stream id — is delivered in inner and middle (fail open). The first frame after a tier
change goes out at once. A node's own uplink (`publish_media_frame`) is never gated, and an unidentified
receiver always gets the frame. The nearest-*K* cap is off by default because `conference_roster.ari`
tiles `cam_<i>` in roster order, not by distance, so a cap would put visible tiles at 2 fps.

**gRPC media stall drop.** `--media-stall-ms 40`: the gRPC relay skips a receiver whose in-flight media
write started more than 40 ms ago (a full flow-control window), and one whose stream a frame cannot get
within 40 ms, rather than queue the publisher's reader thread behind it — the counterpart of the
WebSocket leg's 32 KB backlog drop. It covers every gRPC media write, the relay's own uplink included,
and the media gate is asked only once the stream is held, so a dropped frame spends no thumbnail. (A
write that starts and then blocks still holds its own thread until the 3 s watchdog; only an
asynchronous write path would avoid that.)

**Transport and journal changes** (detail in DISTRIBUTED_STATE_ROADMAP.md, "Landed for the conference
relay"): a deny-only per-peer delivery predicate on `state_peer_registry`; write-once connection
identity (fixes a `remote_node_id` data race; a second Hello is ignored); the registry honoured on every
room-state send (relay forward loops, the WS ~1 Hz broadcast per connection, the gRPC snapshot
responder); `push_snapshot_to`; the **relay interest gate** (when on, a forwarded frame that no local
same-cluster shard `admits()` is withheld from both the relay fan and the bridge; delegation ops still
pass); and the
**journal trim** (relay backfill off, `trim_published()` after each drain, `replay_after` by binary
search), because the admit snapshot already carries every relay-local leaf.

**Client changes.**
* **Shard rooted at `"conf"`** (`state_cluster_shard(app, "conf", node, "conf")`): `ui.docs.*` —
  marker matrices at 30 Hz while avatars move, camera/HUD state, ping leaves — is no longer journaled,
  drained or sent. That ends the `ui.docs` echo through the relay and the client journal that grew from
  it; unsolicited-snapshot ingest now also honours enforced interest on both transports.
* **`pose_known` (no origin ghost).** `client_pump` registers a peer that has no replicated pose with
  `holon::add_participant(…, pose_known = false)`: no Jolt body (it used to be an invisible obstacle at
  the origin, inside the reference box) and no marker or label until its first sample, which creates a
  kinematic body there. One's own avatar appears at its first pose with no glide from the origin.
  `holon::clock_realigns()` counts clock seeks, and arigl logs each one.
* **Pose leaves journal once per change** (`write_pose`: touch only for a new, empty or forced leaf),
  halving pose egress, backfill and journal growth; `write_intent` and the existing `room_*` leaves keep
  always-touch (their re-send semantics rely on it).
* **Audio.** Per-peer gain is `aoi_audio_gain` over the XY distance (once both poses are known). A peer's
  audio clock is followed only while its audio is fresh (0.5 s), so a gated or muted mic no longer
  freezes that peer's video. Each client writes a changed-only `participants.<pid>.voice.talking` leaf
  (one mutation per change), which lights the name of a peer whose audio is stale or gated; no `.ari`
  change.
* **Mesh LOD in `room_view`.** It owns a `LodPyramidCache` that outlives whole-scene re-realizes,
  realizes with `RealizeOptions{&cache}`, and runs `SceneGraph::selectLOD` from its one camera every
  frame (`balanced` natively, `aggressive` under `__EMSCRIPTEN__`). Eligible: a file/URI mesh of ≥ 4096
  triangles with no texture, shader, clip, body or children. Physics never sees a rung (`room_statics`
  keeps primitives only). Builds run on one `async_lane` (deferred to one per frame on single-threaded
  wasm), one per distinct mesh, capped at 2M triangles threaded and 60k single-threaded — so the deployed
  single-threaded cyprty keeps the 69,473-triangle bunny at rung 0 and logs the refusal once.
* **`--scene large`**: a 100 m plane, the reference box, and 24 `x.bunny` meshes on a 5×5 lattice 20 m
  apart (the centre cell, which holds the box and the spawn ring, is skipped), each fitted ~1.7 m tall
  by new `source.fit.{height, up_y}` scene leaves (a fit, not a transform scale, so LOD errors are in
  metres); one pyramid serves all 24.

**Switches.** In the library every relay mechanism is **opt-in and off by default** (no filter or gate
until one is installed, `set_relay_requires_interest(false)`, backfill on and no trim,
`set_media_stall_drop(0)`, `RealizeOptions{}` makes no LOD node, and `relay_room_step(…, nullptr, …,
false)` is the old drain-and-publish exactly); arisrv and arigl turn them on.

| Switch | Default | Effect |
|---|---|---|
| `arisrv --s7 on\|off` / `CVC_ARISRV_S7=0\|1\|on\|off` | on | **master kill switch**: off forces `--aoi off`, gate off, journal keep, stall drop 0 — the pre-S7 relay (plus the disabled `aoi.*` leaves) |
| `--aoi off\|conference\|hall` / `CVC_ARISRV_AOI` | conference (hall with `--scene large`) | off: no filters, pushes or media gate; `aoi.enabled` 0 |
| `--aoi-video-max K` | 0 (all) | opt-in nearest-*K* full-rate cameras; the rest of the inner peers as thumbnails |
| `--relay-gate on\|off` | on | off: forward every client frame |
| `--relay-journal trim\|keep` | trim | keep: journal backfill on connect, no trim |
| `--media-stall-ms N` | 40 | gRPC stall drop; 0 = off |
| `--scene default\|large` | default | the seeded scene; not an S7 switch (honoured under `--s7 off`) |
| `CVC_ARISRV_ROOM_DEBUG=1` | off | adds an `[arisrv] aoi` stats line every ~5 s |
| `CVC_ARIGL_REPLICATE_ROOT` (arigl) | unset = `conf` | empty or `*`: replicate the whole tree (the S6 A/B) |
| `CVC_ARIGL_LOD=0` (arigl) | LOD on | every room mesh at full detail (the LOD A/B) |

An invalid arisrv value is a usage error (exit 2); arisrv prints the effective configuration at start.
Always on, with no switch: `pose_known`, single pose journaling, the write-once identity, the registry
checks (a no-op without a filter), interest-checked snapshot ingest (a no-op without enforcement), the
client's audio-clock staleness and its `voice.talking` leaf. The A/B "before" is `--s7 off` with
`CVC_ARIGL_REPLICATE_ROOT=*`, which is S6 apart from those.

**The default 20 m room.** It *will* show: no origin ghost or invisible obstacle at join, and one's own
avatar at its first pose (arigl frames the camera there); less control-plane traffic while walking (the
`ui.docs` echo gone, poses single-journaled, others' intents and departed ghosts' poses withheld);
joins that no longer replay the relay's whole journal (admit snapshot, no backfill), with relay journal
and drain cost flat — a join still grows with the soft-left ghosts' subtrees the room tree keeps until
Phase-2 reaping, as in S6; and a peer's video that keeps running when its mic is cut. It will **not** show tiering, pose
pushes, thumbnails, audio rolloff or LOD — every pair is inner, and the plane (2 triangles) and box (12)
are far below the 4096-triangle LOD minimum. Media is unchanged, apart from the gRPC stall drop for a
receiver whose writes are stuck. `arisrv --scene large` (hall) is the demo for all of those.

**Compatibility.** A pre-S7 client on an S7 relay keeps uplinking the `ui.docs` echo (the gate withholds
it from everyone else), plays every peer at gain 1, draws the large scene's bunnies unfitted (~15 cm,
on their side) and pose-less peers at the origin. An S7 client on a pre-S7 relay finds no `aoi.*`
subtree and plays at gain 1.

**Known limits.**
* The admit snapshot a connection gets before its Hello is unfiltered (there is no identity to filter
  by yet), so a late joiner sees every participant's *current* intent once, and the non-inner poses
  and ghost leftovers the live filters would withhold. Live traffic after that is filtered.
* A process that restarts under the node id of a live session has its writes ignored until the relay
  restarts (replication drops an `(origin, sequence)` it has seen), which is why arigl takes a fresh id
  per launch. An in-process reconnect keeps its id and its sequence, and works: the relay withholds the
  departed subject's pose meanwhile, and a client takes the pose subtree a peer left behind for no pose
  until a newer sample (`holon::left_tick`).
* The WebSocket leg's ~1 Hz full-tree re-broadcast is filtered per connection but otherwise unchanged,
  and in the hall it dominates a browser's state traffic (**Measured**).

**Measured** (`arisrv_aoi_bench` on catx-03, 24 cores: the relay plus N synthetic clients in one
process on loopback, WS and gRPC alternating, 10 s windows; `--s7 off` is the S6 relay and clients).
Media off, so the state plane alone:

| | S6 | S7 |
|---|---|---|
| 20 m room, 8 walking — relay egress | 6.4 Mb/s | 3.3 Mb/s |
| … others' intents, per receiver | 40.6 kb/s | 0 |
| … a late joiner's bootstrap | 309 kB | 35 kB |
| hall, 12 in clusters (48 of 66 pairs outer) — per gRPC receiver | 242 kb/s | 162 kb/s |
| … a late joiner's bootstrap | 491 kB | 215 kB |
| hall, 12 standing still — per WS receiver (the 1 Hz re-broadcast) | 847 kb/s | 768 kb/s |

With media on: in the 20 m room the gate denied no frame (4 and 8 clients: every pair inner); in the
hall it withheld 2,688 of 3,695 camera and mic frames (73%) for 12 in clusters, and sent 157 MJPEG
thumbnails for 16 walking across the tiers. No run showed an origin ghost; clock re-aligns stayed at
0–3 per run in both modes. Caveats: the bench is one thread, CPU-bound at these sizes (room-step p99
35–130 ms), so its media rates follow the loop rate and are compared by counts, not rates; and its
reconcile-correction sum varies two-fold between identical runs, so it says nothing either way.

**Deferred, with destinations.**
* Scene spatial interest (`scene.cells.<i>_<j>`, per-cell revs and realize) → **H3**. The scene is a
  handful of objects written once (2 in the default room, 26 in `--scene large`), so there is no
  bandwidth to win, and a client's physics statics must stay complete for its prediction to match the
  server's.
* Client- or camera-declared interest (a fly camera far from the avatar; a client want-list plus a relay
  clamp) and a grid-hash AOI broad phase for N > ~200 → **H3**, on S7's registry filter.
* Admission-scoped bootstrap (the admit snapshot goes out before Hello, unfiltered) → **§5e item 2**.
  Bootstrap ghost trim and hard reaping of departed subtrees → **Phase 2 / §5e item 4**.
* A video subscription matching visible cam tiles, simulcast/resolution tiers, inter codecs, stereo
  panning, an "out of range" tile placeholder; peer-staleness hiding (reconciled with the 5 s outer
  refresh) → **Phase 7**.
* Interest as access control, spoofed media `origin_node_id`, forwarding of authority-rejected writes →
  **§5e items 1–2 / Phase 8**.
* Avatar frustum/screen-size culling, label nearest-*K* caps and the `cvc::vis` ViewParams adapter →
  **VISIBILITY-AND-LOD §4.1** follow-up (inert at room scale). Per-camera `RenderView`/`VisibilityMask`
  → **CVCGL-UI-DSL stage C**.
* The LOD budget solver over `LodGraphicsNode`s, relay-baked `cvch5` pyramids and networked LOD
  streaming (wasm has no HDF5) → **H3**; physics/sim LOD and re-hydrate on interest entry → **H2**;
  per-object diff realize in `room_view` → with per-cell revs (**H3**); an `.ari` `lod:` key and LOD for
  `AriRuntime` documents → **§5i**.
* `LodGraphicsNode::select` compares mesh-unit errors with world distances, so a transform-scaled LOD
  node coarsens at the wrong distance (the large scene avoids it with `fit`) → LOD follow-up.
* Client journal trim, WebSocket `request_snapshot` and a gRPC periodic snapshot →
  DISTRIBUTED_STATE_ROADMAP.md (known gaps); a WS `send_backlog` on Windows-hosted relays (`SIOCOUTQ` is
  Linux-only) → transport follow-up; an incremental (revision-gated) WS re-broadcast in place of the
  ~1 Hz full tree, which the hall bench shows at ~0.8 Mb/s per browser → WS_STATE_TRANSPORT.md Phase 2;
  per-connection broadcast encode caching → only if the N = 64 bench shows the ~1 Hz broadcast costing
  > 20 ms/s; merging the visor into the body mesh (2N → N avatar
  actors) → perf follow-up.

---

### 5i. DSL as a first-class holon citizen (programmability & modding)

The DSL is a **first-class read *and write* participant** in the holon — a mutation/event source peer to
physics — **authority-scoped and capability-sandboxed**. It is not part of the deterministic core, so its
*authoritative* writes **replicate** (clients receive + reconcile them) rather than being predicted. Three
roles, each already separable by existing machinery:

* **Client mods — local customize + propose.** A sandboxed client DSL over the replica: local-only effects
  (UI, LOD prefs, cosmetic overrides, reactions) write the **non-replicated** `ui.docs.*` scope;
  holon-affecting changes write the **replicated intent** scope as **proposals** the server accepts/vetoes —
  the exact interest-filter split the name-collision fix relied on (`conf` replicates, `ui.docs.ari` does
  not).
* **Server / admin DSLs — authoritative.** Run on the authority: their writes are holon truth (spawn,
  enforce rules, react to events), role-gated by `state_authority_map`/`write_policy`.
* **Cross-injection — inject a DSL to run inside the counterpart** under a **capability grant**. Every
  constraint the grant names is an existing *enforced* primitive: *intrinsics* → reduced-env default-deny
  allowlist + `requires:` preflight (§7.6); *state tree* → `apply_chroot` to a sub-prefix (+ authority /
  interest); *stream + message channels* → chroot'd channel scoping (#463–467) + stream-token scoping;
  *resource* → step/time/memory budgets (§7.3/§7.4, `memory_tracker`). The **new** work is the *grant
  object*, the *injection transport* (ship DSL + grant over a channel), *host-side sandboxed instantiation*
  (build an env from a grant, run under it), and the *accept policy* (who may inject into whom; auto vs.
  prompted). **Security-critical:** a client→server injection is untrusted code on the authority (the grant
  + reduced-env + chroot + bounded budgets are the whole defense — no fs, no transport beyond granted
  channels, deterministic caps, no escalation); server→client is the mirror. **Write the threat model
  first.**

**Phasing.**
- **D1 — DSL first-class writer** (rides H1): authority-scoped writes (client propose via `intent` → veto;
  server authoritative) on `write_policy` + the replicated/non-replicated scope split.
- **D2 — Client mods:** sandboxed client DSL — local (`ui.docs`) customization + replicated proposals.
- **D3 — Server/admin DSLs:** authoritative holon scripting + event reactions, role-gated.
- **D4 — Cross-injection + capability grants:** grant object + injection transport + host-side
  instantiation + accept policy + a **threat-model doc** (built on reduced-env / `apply_chroot` /
  channel-scoping #463–467 / §7.3–7.6 limits + `requires:`).

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
  `input_text` **landed (#636)** — the compose box uses it directly.

Registration of `type: stream` / `type: stream_view` is host opt-in — included in the §8 bundle.

---

## 10. Build phases (each testable; the arc converges on the browser MVP)

Ordered to de-risk inner-to-outer: prove the room + SFU with native clients first, then WebRTC off the
browser, then the browser itself, then the MVP.

* **Phase 0 — prerequisites (headless). ✅ DONE (#636/#634/#633).** `input_text` widget; stream→`audio_output` auto-pump
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
  SFUs); reconnection/presence/heartbeat; TLS end-to-end. *S7's per-receiver media gate (§5h.1) is the
  first selection hook on the relay: distance tiers, MJPEG thumbnails, a gRPC stall drop.*
* **Phase 8 — mesh + auth hardening + WASI.** `mode: mesh` (WebRTC peer / native serve+pull) with the
  **DC++-style reverse-connect** for NAT (§6g); connection-bound identity + distributed `write_policy`
  for mesh authz; identity (OIDC via SSO / revocable txwtf-style JWT); the **WASI server** build (fill
  cvcpkg gaps, §6f).

> **Baseline shipped (2026-10-21).** The shared-scene + roster conference is **live at
> cyberpcangel.com/cyprty** on the **WebSocket-relay** path — Phases 2–3 + 5 realized (room control plane,
> room verbs, native proof, browser bring-up over a WS loopback); the gRPC/WebRTC legs (Phases 4/6) and
> scale/mesh (7/8) are still pending. The next arc is the **holon + DSL model (§5h/§5i)**; its H0–H4 / D1–D4
> phasing lives there, parallel to the media/transport phases above (the shared 3D scene + avatars is H1).

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
* UI DSL (status refreshed in #764; `input_text`/widgets landed): [`CVCGL-UI-DSL-ROADMAP.md`](CVCGL-UI-DSL-ROADMAP.md).
* World clock / sim pacing (holon driver, §5h): [`WORLD-CLOCK-SYNC-ROADMAP.md`](WORLD-CLOCK-SYNC-ROADMAP.md).
* LOD — geometry (holon §5h; **doc stale — landed PR #249+**; what the room uses since S7 is in its
  §4.1 status note): [`VISIBILITY-AND-LOD-ROADMAP.md`](VISIBILITY-AND-LOD-ROADMAP.md).
* Physics (holon §5h): [`NAV-VEHICLE-PHYSICS-ROADMAP.md`](NAV-VEHICLE-PHYSICS-ROADMAP.md).

---

## 13. Status of the Ariadne (cvcGL UI & scene DSL) roadmap — verified against code

The Ariadne UI/scene DSL is **essentially complete in code, and its roadmap docs are badly stale** — a
*doc* audit mis-reports finished work as open (this file's §9/§10 called `input_text` a gap until this pass,
though it shipped in **#636** — now corrected). The statuses below are **verified against the tree**, not read off the doc.

**Landed (several still listed open/planned in the docs — refresh needed):**
* `input_text` / `input_int` widgets (§3.5/§6) — `src/cvc/ariadne/loader.cpp:650` (#636).
* Multi-view **`RenderView` / `VisibilityMask`** (§9.5) — `src/cvcGL/RenderView.cpp`. They drive the
  `cvc::gl::Node` graph, not `SceneGraph`, so the conference's `room_view` does not use them (§5h, H1).
* **Light** scene node (§9.1) — `src/cvcGL/LightNode.cpp` + loader `"light"`.
* **Clipboard** (§17.8) — `src/cvcGL/Clipboard*.{cpp,mm}`.
* `state_exec` **async-scheduler parity** (§7.1/§7.7) — `receive_message`/`sleep`/`yield_frame` in
  `async_scheduler.cpp`.
* **JSON-Schema validator integration** (§15) — `nlohmann_json_schema_validator` wired in CMake
  (dep-conditional).
* **s-expr in binds/scripts** (part of §18) — homoiconic read-lane in `ariadne.cpp` / `loader.cpp`.
* **Geometry LOD** (PR #249+), **tvision** terminal backend (§16.4), **EGL golden-image harness** (§16.6).
* The core: engine, widget/layout, reactive + action lanes, composition/URI, scene-graph core,
  streamed-source zero-copy + clipping, Unicode, backend split + AppRuntime.

**Genuinely remaining — VERIFY each in code before scheduling (the doc can't be trusted); all retained:**
* Transfer-function editor + `data()`-channel authoring (§10) — *no code found.*
* `state_exec` program modularity `(include/load "uri")` (§19) — *no code found.*
* pycvc URI-scheme handler override + bytes branch (§14.2) — *not found; verify.*
* Full YAML-free s-expr `.ari` **document** dialect (§18) — exprs land in binds/scripts; the whole-document
  front-end is the remainder.
* Formal draft-2020-12 JSON Schema + the `ari validate` command (§15) — the validator *integration* exists;
  the formal schema + CLI may be the gap.
* HTTP response-**cache LRU** (§13.9) — per-request `max_bytes` cap exists (`http_client_fetch.cpp`); the
  response-cache LRU + the write side (§13.10) are unverified.

**Action + takeaway:** refresh the stale statuses in `CVCGL-UI-DSL-ROADMAP.md`,
`VISIBILITY-AND-LOD-ROADMAP.md`, and this file's own §9/§10. Net: the DSL's UI/scene substrate is **done** —
the conference's forward work is the **holon (§5h) + DSL-programmability (§5i)**, genuinely new capability,
**not** DSL catch-up.
