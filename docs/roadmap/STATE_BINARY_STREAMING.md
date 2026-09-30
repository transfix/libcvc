# Binary Messages & Real-Time Streaming for cvc::state

*Design for review — no code yet. Two layers: (a) discrete binary message payloads over
the existing bus, and (b) a real-time high-bandwidth stream transport (video/camera,
audio/mic, peripherals, sensors). Produced by a design workflow + adversarial critique;
every load-bearing claim is grounded in `file:line` against master.*

Status: **DRAFT for review.** The Layer-(b) mechanisms below already incorporate the
adversarial review's corrections (see §7); the open questions in §6 need your sign-off
before implementation.

## Table of Contents
- [1. The two distinct needs](#1-the-two-distinct-needs)
- [2. Layer (a) — binary messages (small, non-breaking)](#2-layer-a--binary-messages-small-non-breaking)
- [3. Layer (b) — real-time stream transport](#3-layer-b--real-time-stream-transport)
- [4. Integration with cvc::state](#4-integration-with-cvcstate)
- [5. Phased plan + validation](#5-phased-plan--validation)
- [6. Open questions for sign-off](#6-open-questions-for-sign-off)
- [7. Corrections from the adversarial review](#7-corrections-from-the-adversarial-review)

---

## 1. The two distinct needs

The user wants messaging to carry **binary** payloads and to support **fast,
high-bandwidth, latency-sensitive real-time streams**. These are two problems with two
different correct answers:

- **Need (1) — discrete binary MESSAGES.** Bounded-size, delivery-oriented bytes (a
  captured still, a serialized command, a compressed tile, a telemetry blob). Wants
  reliability + ordering; tolerates copies and pump latency.
- **Need (2) — real-time STREAMS.** Continuous, high-throughput, latency-sensitive feeds.
  Wants freshness: bounded memory, never stall the producer, drop stale frames, zero
  per-frame copies, off the single-threaded scheduler pump.

**Central thesis: these must be two layers, because one mechanism cannot satisfy both.**
A guaranteed-delivery ordered queue, under sustained overload, must either buffer
(unbounded latency) or block the producer (a camera can't be back-pressured — it drops or
overruns at the device). A real-time transport must instead drop the oldest data to stay
fresh. Delivery-guarantee and freshness are mutually exclusive policies at the same queue.
So Layer (a) rides the existing bus; Layer (b) is a new off-pump transport that only
*borrows* cvc::state for control/metadata.

| Property | Layer (a) message bus (exists) | Layer (b) stream transport (new) |
|---|---|---|
| Goal | delivery + ordering | freshness + throughput |
| Overload policy | buffer / FIFO drop-**newest** | bounded, drop-**oldest** (latest-wins) |
| Producer under overload | grows queue / can stall | never blocks; drops stale |
| Copies per unit | 2–3 deep copies of bare bytes | zero **in-process** (shared_ptr borrow); ≥2 cross-node/wasm (§7 H4) |
| Threading | single pump, one step/drain | dedicated threads / lock-free ring |
| Guarantee | best-effort reliable, dedup, per-peer order | best-effort, lossy-by-design |

---

## 2. Layer (a) — binary messages (small, non-breaking)

Both existing buses are already byte-*capable* at the storage/wire layer; the only gap is
the two front doors that coerce to string.

- **Scheduler bus** (in-process, DSL): `async_scheduler::deliver_to_receivers` /
  `pop_pending_message` / per-channel `pending_messages_` FIFO / cross-thread
  `post_message` carry a `value_t` verbatim, and `value_t` already has a first-class
  `bytes` alternative (`inc/cvc/core/state_exec/types.h:69-72,93,180`). Binary HTTP bodies
  already flow this way to `msg-recv`.
- **Cluster bus** (out-of-band, cross-node): `state_message` already carries
  `std::vector<unsigned char> bytes` + `content_type` + `make_bytes`/`make_typed`
  (`inc/cvc/core/state_message.h:62-97`), and all three transports serialize the `bytes`
  field (`state_transport_ipc.cpp:217`, `state_transport_grpc.cpp:161,173`, proto
  `bytes_payload = 8`).

The gaps: the DSL `msg-send` intrinsic coerces its payload with `as_string`
(`intrinsics.cpp:729`), and `state::sendMessage` sets only `string_value`
(`state.cpp:1314`).

**Change 1 — `msg-send` accepts `bytes` (pure widening, non-breaking).** Replace
`as_string(args[1])` with a small `string_or_bytes` helper returning the byte buffer from
either a `string` or a `bytes_value`, and default `content_type` to
`application/octet-stream` when the arg is `bytes` and none was given. Existing callers are
unaffected (input only widened).

**Change 2 — what `msg-recv` receives (needs sign-off; see Q1).** Today `msg-send` does two
independent things: pushes the raw payload to the state-side bus (`node->sendMessage`),
and delivers a `{status, path}` **dict — not the payload** — to scheduler receivers
(`intrinsics.cpp:752-767`), returning that dict to the sender. The canonical `msg-recv` is
paired with `post_message` (which delivers the raw `value_t`), not `msg-send` (no in-tree
`.ari` pairs them). **Recommendation:** deliver an **envelope** `{status, path,
content_type, payload}` to receivers and keep the sender's return value `{status, path}`
unchanged — backward-compatible (the delivered dict only *gains* fields; the return is
untouched, so `state_exec_intrinsics_test.cpp:668-680` stays green), cheaper (the extra
`dict_ptr` fields are refcount bumps, not buffer copies), and it carries `content_type`.

**Change 3 — `state::sendMessage` bytes overload.** Add an overload that sets `m.bytes`
(not `string_value`); binary must ride the proto `bytes_payload` field, not `string_value`
(`state_transport.proto:71-72`). The struct + all three codecs already round-trip `bytes`.

**Change 4 — transport size limits (real bugs, §7 H5/H6).** (a) gRPC sets **no**
max-message-size (`state_transport_grpc.cpp` server builder / `CreateChannel`) → the ~4 MiB
default tears the stream on a >4 MiB binary; raise it explicitly. (b) IPC has **no
send-side guard** (`state_transport_ipc.cpp:515-532`) → an oversize send silently
disconnects the peer at the 64 MiB receive cap; add a clean send-side error.

**Change 5 — pycvc `bytes` overload** for the binding surface.

**Caveat (§7 H7):** a multi-MB binary `msg-send` on the DSL path stalls the single pump
(the cluster bus fans out synchronously under a `recursive_mutex`,
`state_message_bus.cpp:61,94`). Layer (a) is for *discrete, bounded* binaries — never a
stream. This reinforces the two-layer split.

---

## 3. Layer (b) — real-time stream transport

A new `cvc::stream` lane (namespace TBD, Q9). It **never** touches
`deliver_to_receivers`, `ingress_`, `pending_messages_`, or `state_message_bus::admit`.

### 3.1 Zero-copy frame (fixes the `bytes_value` inline-copy blocker)

`bytes_value` is an inline non-COW `std::string` (`types.h:69`) — *the* zero-copy blocker.
The stream lane uses a shared, ref-counted, frozen-after-publish buffer:

```cpp
namespace cvc::stream {
struct format_desc { /* kind, codec, w/h/stride, sample_rate/channels, extra */ };
struct frame {
  const uint8_t*        data;        // borrowed; owned by keepalive
  size_t                size;
  int64_t               seq;         // monotonic per stream
  double                pts_seconds; // producer clock
  format_desc           format;
  std::shared_ptr<void> keepalive;   // owns data: pool slab, image storage, SAB, ...
};
using frame_ptr = std::shared_ptr<const frame>; // borrow = refcount bump, never a copy
}
```

`keepalive` lets a frame alias foreign storage (a `cvc::image` buffer, a pool slab, a wasm
`SharedArrayBuffer`) with no memcpy. **Frame lifetime is UAF-safe by construction** (the
review confirmed this): a consumer's `frame_ptr` copy keeps the slab's refcount > 0 while
it borrows, so a concurrent drop/recycle can't free it under the consumer — the same
owning-pin discipline as the Phase-1 state work.

### 3.2 Frame pool (zero steady-state allocation) — with a required sizing invariant

Producer `acquire()`s a fixed slab, fills it, `publish()`es → a `frame_ptr` whose
`keepalive` returns the slab to the pool when the **last** consumer drops it. Steady state:
zero malloc, zero copy on borrow.

**Invariant (§7 H3 — load-bearing, must be enforced):** the pool must be provisioned to
**Σ over subscribers of (queue depth + frames in-flight) + producer working set**. A fixed
pool that is too small re-couples producer and consumers: a slow consumer that pins slabs
starves `acquire()`, so the producer stalls at the source — defeating the whole point of
the split. The pool must either be sized to that sum or grow/borrow-fail with an explicit
drop-at-source policy (never a hidden stall). The "producer never blocks" and
"slow-consumer isolation" guarantees hold *only* under this invariant.

### 3.3 Stream channel — bounded, drop-oldest, per-subscriber

Built on the existing `state_bounded_queue<frame_ptr>` with
`overflow_policy::drop_oldest` (`inc/cvc/core/state_bounded_queue.h:66-107,90-93` — exactly
latest-wins, with drop counters `:196-211`). Because the element is a `shared_ptr`,
push/drop/pop are refcount ops, never buffer copies.

- Each **subscriber gets its own bounded queue** (slow-consumer isolation, subject to the
  §3.2 pool invariant), mirroring the inproc per-peer outbox model.
- **Video display:** `latest()` (freshest-wins, depth 1–3); a specialization can collapse
  to a lock-free single-slot triple-buffer.
- **Audio/sensor:** `pop()` on a deeper drop-oldest ring; on underrun the consumer holds
  last / inserts silence.

### 3.4 Off-pump threading (§7 H1 — corrected)

- **Producer runs on a DEDICATED `std::thread`** (a device/capture thread, or a long-lived
  worker started via `startThreadPooled(..., wait=false)`), calling `publish()` directly.
  **It must NOT be a `computePool()`/`parallel_for` loop** — that pool runs one job at a
  time and holds `_post_mtx` until the job drains, so a never-returning producer would
  deadlock the entire compute subsystem (every `launch_pool_task`, `compute_async`,
  http/nav async). This was the review's most serious finding.
- **Consumer** runs on its own cadence — for cvcGL, `AriRuntime::frame()` once per rendered
  frame calls `subscription::latest()`.
- **Only lightweight EVENTS** cross onto the pump via `post_message` — "stream live",
  "format changed", "closed", or a frame-available *tick* carrying a bare `seq` int —
  **never the frame bytes.**

### 3.5 Cross-node (§7 H4/H5 — scoped honestly)

- Control plane (offer/answer, format, endpoint, stats) rides cvc::state + Layer (a). Frame
  data does **not** ride `state_message_bus::admit`.
- **Phase-4 (best-effort LAN):** a dedicated gRPC bidi `StreamService` on its **own
  connection** (not multiplexed with control traffic), own reader thread, own per-peer
  drop-oldest queue — requires Change 4(a) size limits, and per-frame chunking (seq/offset)
  for frames over one message. Cross-node is **≥2 copies/frame** (serialize/deserialize) —
  the zero-copy claim is native-in-process only.
- **Phase-6 (true real-time WAN):** a pluggable RTP/WebRTC data-channel transport with
  congestion control, NACK, jitter buffering. gRPC-over-TCP has head-of-line blocking under
  loss — fine for LAN, not real-time WAN.
- **Do not** content-address live frames (`state_blob_store` SHA-256s every put — pure cost,
  no dedup for unique frames) and **do not** reuse the pull-based `fetch_chunk` path
  (request/response, not a live feed).

### 3.6 wasm

Default wasm is single-threaded (ASYNCIFY; `thread_pool` yields 0 workers). Two modes:
- **wasm-mt (pthreads + SharedArrayBuffer):** `frame_pool` slabs backed by a SAB; worker
  producer + main-thread consumer = the native model. **Gate (§7 H8):** SAB needs
  COOP/COEP cross-origin isolation on the served page; without it, fall back. (wasm-mt is
  tracked separately, deps #448.)
- **default single-thread wasm:** the browser event loop is the producer — a JS callback
  (WebCodecs `VideoFrame`, `<video>`, `getUserMedia`, `requestAnimationFrame`) copies into
  a wasm-heap buffer and calls `publish()` inline; the consumer reads `latest()` in the
  same rAF-driven `frame()`. One copy per frame here (H4).

### 3.7 How the layers use it

**DSL (control-plane only — never pulls frame bytes):**
```lisp
(define s (stream-open {:kind "video_raw" :codec "rgba8" :w 1920 :h 1080}))
(stream-info s)            ; descriptor + stats dict from the state tree
(gl-bind-stream node s)    ; HOST action wires the C++ consumer to a GeometryNode texture
(msg-recv "stream.evt")    ; lifecycle events (small dicts via post_message)
```

**pycvc (zero-copy producer/consumer, native):**
```python
s = cvc.stream.open(fmt)
s.publish(numpy_frame)     # aliases the ndarray buffer via a keepalive capsule
f = s.latest(); arr = f.numpy()   # zero-copy view; base capsule holds the shared_ptr
```
Reuses the pycvc ArrayView zero-copy machinery (numpy base = capsule owning a `shared_ptr`).

**cvcGL video sink (needs new `image` plumbing — §7 H2, NOT free reuse):**
The headline camera→texture zero-copy requires a **new `image` adopt-constructor** (today
`image(w,h,f,dt,const void*)` *copies*, `image.h:35-38`) **plus a keepalive bridge** between
the frame's `std::shared_ptr<void>` and `image`/`GeometryNode`'s
`boost::shared_array<unsigned char>` (`image.h:57`, `GeometryNode.h:214`). Only then does
`GeometryNode`'s zero-copy texture path (`GeometryNode.h:147-161`) apply. Treat this as new
work, not reuse. Audio sink: an SDL3 audio callback `pop()`ing an audio channel (SDL3 is the
sanctioned peripheral seam; capture is marked "later", so it's new work — §7 H9).

---

## 4. Integration with cvc::state

- **Discovery:** a stream is a state node `/streams/<id>` carrying the `format_desc`, a
  lifecycle field (`negotiating|live|paused|closed`), stats (fps/drops/bitrate from the
  queue counters), producer identity, and a stable **handle token** that resolves in a
  process-local `stream_registry` to the live `stream_channel`. The descriptor node
  replicates; the channel object does not.
- **Lifecycle:** creating the node = announce (a `state-watch` fires); a consumer resolves
  the token → `subscribe()`. Deleting / `close()` = teardown (watch notifies, `close()`
  unblocks parked consumers).
- **Negotiation (cross-node):** a remote consumer watching `/streams/<id>` learns
  format+endpoint from the replicated descriptor and opens the dedicated stream RPC;
  offer/answer flow as Layer-(a) messages.
- **Boundary rule:** REUSE cvc::state for descriptors/lifecycle/stats/discovery/negotiation
  (small, slow, benefits from watch+replication); NEVER for frame bytes or anything at
  frame rate.

---

## 5. Phased plan + validation

- **Phase 0 — Layer (a) binary messages (small, buildable now):** Changes 1–5. Tests:
  `msg-send` bytes round-trip through both buses; IPC + gRPC wire round-trip of
  `bytes_payload` (octet-safety incl. NUL/high bytes); backward-compat (return dict
  unchanged; delivered envelope additive); pycvc bytes.
- **Phase 1 — `cvc::stream` core (native, in-process):** `frame`/`frame_ptr`, `frame_pool`
  (with the §3.2 sizing invariant), `stream_channel` on `state_bounded_queue<frame_ptr>`,
  `stream_registry`, event hookup via `post_message`, **dedicated producer thread** (not the
  compute pool). Unit + microbench.
- **Phase 2 — consumer sinks:** the **new `image` adopt-ctor + keepalive bridge** (H2), then
  cvcGL `StreamTextureBinding`; pycvc producer/consumer; a synthetic test-pattern producer
  (end-to-end with no hardware).
- **Phase 3 — producers:** SDL3 camera/mic capture (new device-capture work) + a
  decoded-file producer.
- **Phase 4 — cross-node:** dedicated gRPC `StreamService` bidi (own connection, size
  limits, per-frame chunking) + state-tree negotiation.
- **Phase 5 — wasm:** wasm-mt SAB frame pool (COOP/COEP gate) + single-thread main-loop
  fallback.
- **Phase 6 (long-term):** RTP/WebRTC transport; A/V sync + congestion control; codecs.

**Perf/validation (Layer b):** zero-copy assertion (consumer `frame->data` == producer slab
pointer, native path); flat allocation counter after warm-up; sustain 1080p60 raw (~180
MB/s) and 4K, with the bus deep-copy cost absent from the stream path; drop-oldest under a
slow consumer (producer never blocks, `total_dropped_oldest` climbs, newest always
delivered) **and** the pool-starvation case (H3); multi-consumer isolation; TSan/ASan on
ring + pool (TSan needs `setarch -R`); wasm-mt SAB test + single-thread fallback smoke;
glass-to-glass camera→texture latency.

---

## 5a. Decisions (signed off 2026-09-30)

- **Q1 msg-recv contract → ENVELOPE.** Deliver `{status, path, content_type, payload}` to receivers;
  keep the sender's return value `{status, path}` unchanged (backward-compatible).
- **Q2 codec scope → raw-first + ffmpeg PROCESS-PIPE (+ optional in-process LGPL).** Raw frames first.
  Then a license-clean **ffmpeg subprocess pipe** in BOTH directions — accept a decoded stream *from* an
  external ffmpeg process, and forward our frames *to* an ffmpeg process for encoding — as the primary
  full-codec path (a separate ffmpeg *process* is not linked, so even a GPL ffmpeg build, incl.
  libx264/x265, does not taint us). PLUS optional **in-process** codecs strictly via the existing
  **LGPL** ffmpeg (`ffmpeg-lgpl`, PR #110 — dynamic-link-safe) for low-latency, limited to non-GPL
  encoders: hardware (VideoToolbox/NVENC), **openh264** (BSD), or royalty-free **VP9/AV1/Opus**. NEVER
  link libx264/libx265 or build ffmpeg `--enable-gpl` in-process — that copylefts the whole project.
- **Q3 cross-node → gRPC-LAN first (Phase 4), THEN RTP/WebRTC (Phase 6).** RTP/WebRTC is required for
  future engagements needing real-time A/V, so it is committed, not optional — just sequenced second.

Remaining open (Q4 A/V master clock, Q5 stream scoping/ACLs, Q6 frame mutability, Q7 pool sizing
defaults, Q8 snapshot binary gap, Q9 namespace) still need sign-off before the phases they gate.

## 6. Open questions for sign-off

*(Q1–Q3 decided — see §5a.)*

1. **`msg-recv` contract:** deliver the **envelope** `{status,path,content_type,payload}`
   (recommended — backward-compatible, cheaper, carries content_type) or the **raw payload**
   (parity with `post_message`, but a silent contract change + loses content_type)?
2. **Codec scope:** raw frames only, or in-transport encode/decode (H.264/HEVC/Opus)? That
   pulls in WebCodecs (wasm) / ffmpeg (native) — a dependency + hermetic-recipe decision.
3. **Cross-node priority:** is gRPC-bidi acceptable for the Phase-4 LAN cut, or is
   RTP/WebRTC needed sooner?
4. **A/V sync:** master clock owner; resampling + A/V alignment policy; is the `cvc::world`
   clock the intended time base?
5. **Security/scoping:** do streams inherit the §12 channel-scoping/chroot model? Per-stream
   ACLs for cross-node consumers?
6. **Frame mutability:** frames are frozen after `publish`; the existing cvcGL zero-copy path
   supports in-place pixel edits (`texture_modified()`). Need a mutable-aliased mode, or is
   publish-immutable + pool-recycle sufficient?
7. **Pool sizing defaults / budget:** default `slab_bytes`/`slab_count`? (4K RGBA ≈ 33
   MB/frame; depth-3 ≈ 100 MB/stream.) This ties directly to the §3.2 isolation invariant.
8. **Snapshot binary gap (separate scope):** extend the initial-sync snapshot path to carry
   a bytes field (`SnapshotEntry` / IPC snapshot serializer carry `string_value` only) so
   replicated binary node values survive a full-tree resync? Independent of streaming.
9. **Namespace/library:** `cvc::stream` as a new small library, or under `cvc::media`?
   (CMake/recipe surface.)

---

## 7. Corrections from the adversarial review

The design above already reflects these; listed for the record (verdict: **sound-with-fixes**;
Layer (a) buildable/non-breaking as-is).

- **H1 (SERIOUS, fixed in §3.4):** the producer must be a dedicated thread — a
  compute-pool/`parallel_for` loop deadlocks the whole compute subsystem (`_post_mtx` held
  until a job drains; a never-returning producer never drains).
- **H2 (fixed in §3.7/Phase 2):** end-to-end zero-copy to the VTK texture needs a new
  `image` adopt-constructor + a `shared_ptr`↔`boost::shared_array` keepalive bridge; today's
  `const void*` image ctor copies. Not free reuse.
- **H3 (fixed in §3.2):** a fixed frame pool + a slow consumer pinning slabs starves
  `acquire()` → producer stalls; the isolation guarantee requires the stated pool-sizing
  invariant.
- **H4 (scoped in §1 table / §3.5-3.6):** "zero copies" is native-in-process only;
  cross-node (serialize/deserialize) and wasm (copyTo) are ≥1–2 copies/frame.
- **H5 (Change 4a / §3.5):** Phase-4 gRPC needs explicit max-message-size and is best-effort
  LAN, not real-time WAN.
- **H6 (Change 4b):** IPC needs a send-side size guard (oversize currently kills the peer).
- **H7 (§2 caveat):** a multi-MB binary `msg-send` stalls the single pump — Layer (a) is for
  discrete binaries only.
- **H8 (§3.6):** wasm-mt SAB requires COOP/COEP cross-origin isolation.
- **H9 (§3.7/Phase 3):** SDL3 capture + an audio-output sink are new work, not reuse.

Confirmed sound by the review: the two-layer thesis; frame lifetime under drop-oldest
(UAF-safe by refcount); Change 1 (pure widening); Change 2 envelope (backward-compatible);
`state_bounded_queue` drop-oldest reuse; "don't content-address live frames / don't reuse
pull-based chunking for live."
