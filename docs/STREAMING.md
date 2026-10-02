# Real-time streaming — `cvc::ariadne::stream`

A real-time, high-bandwidth **frame transport** for libcvc: video (camera / rendered / decoded),
audio (any capture device or a mixed/filtered graph), and sensor feeds, carried from a *producer* to
one or more *consumers* with bounded memory, zero copies on the hot path, and a never-stall
drop-at-source policy. It is deliberately separate from the `cvc::state` message bus — that bus is a
control/command path pumped on a single scheduler thread, which is the wrong shape for a 1080p60
camera or a 48 kHz microphone. Streams borrow `cvc::state` only for *control and metadata* (a
descriptor node, lifecycle events), never for the frame payloads themselves.

Design/roadmap rationale lives in [`roadmap/STATE_BINARY_STREAMING.md`](roadmap/STATE_BINARY_STREAMING.md);
this guide is the developer's view: how it works, how it plugs into a host, and how peripherals feed
it, with examples.

> **Status.** Phases 1–2 (core, sinks, DSL verbs, pycvc, gl-bind) and the Phase-3 producer
> *foundation* (`frame_source`, `synthetic_source`, `stream::start_producer`) are implemented. The
> Phase-3 **device** sources (SDL3 camera, the composable audio-source family) and the decoded-file
> source are in progress; they are marked 🔜 below and the interface they implement is final.

---

## 1. The shape of a stream

```
                         (owner / render thread)                 (any thread: producer)
  cvc::state tree  <----  stream  ---- owns ---->  stream_channel  <---- publish() ----  producer_thread
   /streams/<id>         (descriptor,              (per-subscriber                         drives a
   descriptor node        seq/evt channels)         delivery)                              frame_source
   + stats child                |                      |                                      |
                                |                   subscribe()                            fill() one
                                v                      v                                    frame/tick
                          lifecycle + seq          frame_pool  <-- acquire() a slab -----------+
                          events on the pump       (fixed slabs, non-blocking, drop-at-source)
```

A **`stream`** is the owner object. `open()` sizes a frame pool, builds the channel, installs a
registry token, and publishes a `/streams/<id>` descriptor node into `cvc::state`. A **producer**
(optional) fills frames into pooled slabs and `publish()`es them; **consumers** `subscribe()` to the
channel and pull the latest (or a short ring of) frames. Only lightweight **events** — a throttled
sequence heartbeat and open/closed lifecycle — ever cross onto the `cvc::state` scheduler pump.

### Key types (`inc/cvc/ariadne/stream/…`)

| Type | Role |
|---|---|
| `frame` / `frame_ptr` (`= shared_ptr<const frame>`) | An **immutable** frame: borrowed bytes (`data`,`size`), `seq`, `pts_seconds`, `format`, and a `keepalive` that owns the backing storage. |
| `frame_pool` | Fixed-count fixed-size **slabs**. `acquire()` is **non-blocking** (returns `nullopt` when full — the caller drops, never stalls). A published frame's `keepalive` returns its slab to the free list when the last `frame_ptr` drops. |
| `stream_channel` | Per-subscriber delivery. `deliver_mode::latest` = a lock-free single-slot register (newest wins); `deliver_mode::ring` = a drop-oldest bounded queue. Fan-out snapshots subscribers under a lock, delivers off-lock. |
| `stream_registry` | Per-app, non-owning `token → stream_channel*` side table (`for_app(app).lookup(token)`), first-writer-wins. |
| `stream` | The owner: `open()`, `channel()`, `token()`, `close()`, and (Phase 3) `start_producer()`. |
| `producer_thread` | The one sanctioned host for a long-running `publish()` loop — a dedicated `std::thread` that joins deterministically; never a compute-pool job. |
| `frame_source` | The pull-source abstraction a producer drives: `fill()` one frame per tick. Camera / audio / file / synthetic all implement it. |

### The pool-sizing invariant (why memory stays bounded)

`open()` sizes the pool to **Σ over subscribers of (depth + in-flight) + the producer working set**.
A pool that is too small re-couples producer and consumers (a slow consumer that pins slabs would
starve `acquire()`), so the Phase-1 policy is **refuse-rather-than-under-provision**: `subscribe()`
returns null instead of handing out a subscriber the pool can't cover, and a producer that can't
`acquire()` **drops at the source** — never a hidden stall. All of this is tunable at open time via
`stream_params` (`subscriber_depth`, `expected_subscribers`, `producer_working_set`,
`in_flight_per_subscriber`).

---

## 2. Host interface — opening and owning a stream

### From C++

```cpp
#include <cvc/ariadne/stream/stream.h>
namespace st = cvc::ariadne::stream;

st::stream_params p;
p.id = "cam0";                              // unique within the scope
p.format.kind   = st::frame_kind::video_raw;
p.format.codec  = "rgba8";
p.format.w = 1280; p.format.h = 720;
p.format.stride = 1280 * 4;                 // dense
p.expected_subscribers = 2;                 // sizes the pool
std::unique_ptr<st::stream> s = st::stream::open(app, p);   // null on bad params / dup id
// s->token()  == the canonical key consumers look up (see §4.1 scoping)
// s->channel() is where producers publish and consumers subscribe
```

`open()` publishes a descriptor node at the stream's canonical path (see scoping below), holding the
format, a `live`/`closed` lifecycle value, and a `stats` child (published / dropped / slabs-in-use /
subscriber count, refreshed on the owner thread). `close()` (and `~stream`) stops any producer,
closes the channel (unblocking parked consumers), uninstalls the token, and marks the descriptor
`closed`. The descriptor node persists in the state tree, marked closed, so a late reader sees the
stream existed.

### From the Ariadne DSL

A document opts in to the stream verbs (a host/test calls
`cvc::ariadne::register_stream_intrinsics(app)` once). Then, from a program `on:`/resident action
lane:

```scheme
; open a video stream scoped to THIS document; returns a handle dict
(set cam (stream-open (dict "id" "cam0" "codec" "rgba8" "w" 1280 "h" 720)))
(get-attr cam "ok")            ; => #t
(get-attr cam "token")         ; => the canonical token to hand a sink
(get-attr cam "seq_channel")   ; => a channel that ticks the latest frame seq

(stream-info cam)              ; => { ok live id token seq_channel evt_channel published subscribers }
(stream-close cam)             ; => { ok #t }  (also closed at document teardown)
```

Streams opened this way are **owned by the calling document** (its `se::document_scope`) and closed
automatically when the document tears down — you never leak a stream whose owner went away. The verbs
run on the action/resident lane; opening from the load-time `init:` lane is rejected cleanly (no
document to own the stream).

### §4.1 scoping — how a stream is addressed and shared

The stream's identity is one canonical key: the descriptor node's full path under the document's
chroot, `normalize_path("<root_path>.streams.<id>")`. This single string is **both** the
`stream_registry` key **and** the descriptor node path, so:

* a same-document consumer resolves it from `(root_path, id)` — e.g. `StreamTextureBinding` looks the
  channel up by exactly the token `(stream-open)` returned;
* a different document reaches it only through the existing Ariadne grants — a `link:` grant to the
  `/streams/<id>` descriptor (state) or a `channels:` grant to the seq/evt channel (messaging) — and
  a followed grant resolves to the *same* canonical key. Isolation falls out: different document
  roots produce different paths.

The seq/evt channels are chroot-scoped the same way (`<root>.channels.streams.<id>.seq`/`.evt`), so a
document's stream lifecycle is private unless explicitly granted.

---

## 3. Consuming a stream (sinks)

### Render video onto geometry

Two paths, both built on **`StreamTextureBinding`** (`cvc/gl/ariadne/stream_texture_binding.h`),
which on each render-thread `tick()` pulls the latest frame, aliases its bytes into a `vtkTexture`
**zero-copy** (no pixel copy), and applies it to a `GeometryNode`.

**Declarative** — a `"stream"` scene node (a host calls
`cvc::gl::ariadne::register_stream_node_type()` once):

```json
{ "id": "screen", "type": "stream", "stream": "<token>", "width": 1.6, "height": 0.9 }
```

The realizer builds a UV'd quad, subscribes the stream named by the `stream`/`token` attribute, and
drives a per-frame binding tick — all on the render thread, torn down with the scene.

**Imperative** — the `(gl-bind-stream NODE-ID TOKEN)` verb binds a stream onto an *existing*
`GeometryNode` at runtime from an action:

```scheme
(gl-bind-stream "screen" (get-attr cam "token"))   ; => { ok #t node token }
```

### Produce / consume frames in Python (pycvc)

The pycvc stream API is opener-centric — open a `Stream`, then subscribe and/or publish on it:

```python
import pycvc
s   = pycvc.stream_open(app, "cam0", "rgba8", 1280, 720)  # a Stream
sub = s.subscribe("latest", 3)                            # or "ring", <depth>
f   = sub.latest()                                        # a StreamFrame; f.valid() False until a frame
if f.valid():
    arr = f.numpy()          # zero-copy (H, W, 4) uint8 view that PINS this frame (and its slab)
# producer side — aliases the ndarray zero-copy (the frame pins it for its lifetime):
seq = s.publish(arr_uint8, pts_seconds)
```

See `bindings/pycvc/pycvc_stream.i`.

---

## 4. Producing frames — peripherals and sources

A stream with no producer is fed externally (any thread calling `channel().publish(...)`, or
`publish_external(...)` to alias caller-owned bytes with no pool slab). To drive a stream from a
**continuous source** — a camera, a microphone, a decoded file, a generated pattern — attach a
**`frame_source`** and let the stream own a producer thread.

### The `frame_source` contract (`cvc/ariadne/stream/frame_source.h`)

```cpp
struct produced_frame { std::size_t bytes = 0; double pts_seconds = 0.0; bool stop = false; };

class frame_source {
public:
  virtual ~frame_source() = default;
  virtual std::size_t frame_bytes() const = 0;                       // one frame's size (validated vs the slab)
  virtual produced_frame fill(std::uint8_t *buf, std::size_t cap) = 0; // write one frame; runs on the producer thread
};
```

`fill()` runs on the **dedicated producer thread**: it writes one frame into a pool slab and returns.
It is self-contained — no DSL, no `cvc::state`, no channel calls — the stream's producer tick does
the `acquire` / `publish` / seq-post around it. It **never blocks or back-pressures**: when the pool
is full the tick simply doesn't call `fill()` and drops that frame (a camera can't be
back-pressured). A source reports `stop = true` when it is spent (end of file).

### Attaching a producer

```cpp
#include <cvc/ariadne/stream/synthetic_source.h>
s->start_producer(std::make_unique<st::synthetic_source>(1280, 720), /*hz*/ 30.0);
```

`stream::start_producer(source, hz)` creates a `producer_thread` whose tick acquires a slab,
`fill()`s it, publishes (stamping the seq), and posts the throttled seq heartbeat. It **rejects** a
source whose `frame_bytes()` exceed the pool slab (a format mismatch fails loudly, not as a silent
dead stream), and it replaces any prior producer (stop + join first).

**Lifetime (load-bearing):** `close()`/`~stream` **stop and join the producer before** the channel
and pool it publishes into are torn down — so no producer callback can ever race teardown. This is
the one hard rule for anything that owns a producer. The whole teardown runs on the owner thread.

`synthetic_source` (a moving rgba8 test pattern) is the reference `frame_source` and lets a stream
run end-to-end with no hardware.

### Peripheral capture — SDL3 (the host peripheral seam)

Device capture lives in **cvcGL**, which is where libcvc links SDL3 (its keyboard/mouse/gamepad/audio
"peripheral seam", built when `CVC_ENABLE_SDL` is on). A device source is just a `frame_source` whose
`fill()` pulls from SDL and converts into the stream's codec; the stream and the sinks don't know or
care that the frames came from a device.

#### 🔜 Camera — `camera_source` (Phase 3b)

Wraps SDL3's camera API: `SDL_GetCameras()` enumerates devices, `SDL_OpenCamera(id, spec)` opens one,
and each tick `SDL_AcquireCameraFrame()` yields an `SDL_Surface` in the camera's native format that
`fill()` converts to the stream's `rgba8` (releasing it with `SDL_ReleaseCameraFrame`). Capture
requires a camera device and, on some platforms, a user permission grant
(`SDL_GetCameraPermissionState`), so its tests are device-gated (they skip on a headless builder).

```cpp
// (interface, Phase 3b)
auto cam = cvc::gl::capture::open_camera(/*device*/ 0, /*w*/1280, /*h*/720);  // -> unique_ptr<frame_source>
s->start_producer(std::move(cam), /*hz*/ 30.0);
```

#### 🔜 Audio — a composable audio-source family (Phase 3b)

Audio is **not** a single `microphone_source`. The source is a general, composable `frame_source`
family producing `audio_pcm` frames, so it is broadly compatible with every audio source — real
devices *and* virtual ones that mix or filter others:

* **device audio** over *any* SDL3 capture device — microphone, line-in, a loopback/monitor of
  system output, a virtual device — not microphone-specific;
* **mix** — sums N audio sources into one;
* **filter / gain** — transforms one audio source (gain, a simple EQ, …);

all composable into an audio graph (e.g. a mix of a filtered microphone and a synthetic tone). They
share the `frame_source` pull model: a mix source pulls one chunk from each input and sums them; a
filter wraps one input. Composed inputs must agree on sample rate / channels / format (resampling is
part of this layer). The pure sources (mix, filter, a synthetic tone) need no SDL and are fully
testable; only the device source needs a capture device.

```cpp
// (interface, Phase 3b — illustrative)
auto mic  = cvc::gl::capture::open_audio_device(dev, fmt);         // frame_source (audio_pcm)
auto bed  = std::make_unique<st::tone_source>(fmt, /*hz*/440.0);   // a virtual source
auto mix  = cvc::audio::mix({ cvc::audio::gain(std::move(mic), 0.8), std::move(bed) });
audio_stream->start_producer(std::move(mix), /*hz*/ 100.0);        // 100 chunks/sec
```

#### 🔜 Decoded file — `file_source` (Phase 3c)

Decodes a media file to frames via an ffmpeg **process pipe** (a separate process, not linked — so
even GPL decoders don't taint the library), feeding video and/or audio streams.

---

## 5. Threading & lifetime, in one place

* **Owner / render thread** — `open`, `close`, `start_producer`, descriptor writes, and every sink
  `tick()` (the GL sinks ride the scene's per-frame `tick_scene`). Single-threaded per document.
* **Producer thread** — a dedicated `std::thread` (or a device/file callback); only `fill()` +
  `publish()` + `post_message` of events. Never the compute pool.
* **Consumers** — any thread may `subscribe`/`latest`/`pop`; delivery is refcount-only.
* **Zero-copy** — a `frame_ptr` aliases pool (or foreign) bytes via its `keepalive`; the slab returns
  to the pool only when the last `frame_ptr` drops. A GL texture or a numpy view pins the frame it
  shows.
* **Join-before-free** — a stream that owns a producer stops and joins it **before** its channel and
  pool are destroyed; destroy the owning document/scene on the render thread.

---

## 6. End-to-end examples

### A document that shows a camera on a quad (DSL)

```scheme
; init: or a Start button's on: program
(set cam (stream-open (dict "id" "cam0" "codec" "rgba8" "w" 1280 "h" 720)))
; (Phase 3b) begin capture from the default camera into the stream:
(stream-capture-camera (get-attr cam "token") 0)
; bind it onto the scene's "screen" node:
(gl-bind-stream "screen" (get-attr cam "token"))
```

### A C++ host feeding a synthetic stream to a texture

```cpp
auto s = st::stream::open(app, params);                       // rgba8 WxH
s->start_producer(std::make_unique<st::synthetic_source>(W, H), 30.0);
auto sub  = s->channel().subscribe(st::deliver_mode::latest);
cvc::gl::ariadne::StreamTextureBinding bind(app, s->token(), weak_node, sub);
// each render frame:
bind.tick();                                                  // newest frame -> node texture, zero-copy
```

### A pycvc producer + consumer

```python
import pycvc, numpy as np
s   = pycvc.stream_open(app, "cam0", "rgba8", W, H)
sub = s.subscribe("latest")
s.publish(np.zeros((H, W, 4), np.uint8), 0.0)   # produce a frame (zero-copy alias)
f = sub.latest()
if f.valid():
    arr = f.numpy()                              # (H, W, 4) uint8, zero-copy
```

---

## 7. Where things live

| Area | Header(s) | Target |
|---|---|---|
| Core transport | `cvc/ariadne/stream/{frame,frame_pool,stream_channel,stream_registry,stream}.h` | `cvc` |
| Producers (pull sources) | `cvc/ariadne/stream/{frame_source,synthetic_source}.h` | `cvc` |
| Image bridge | `cvc/ariadne/stream/to_image.h` | `cvc` |
| DSL verbs (`stream-open/-info/-close`) | `cvc/ariadne/stream_intrinsics.h` | `cvc` |
| GL sinks (`StreamTextureBinding`, `"stream"` node, `gl-bind-stream`) | `cvc/gl/ariadne/{stream_texture_binding,stream_node,stream_verbs}.h` | `cvcGL` |
| 🔜 Device capture (camera, audio devices) | `cvc/gl/capture/…` | `cvcGL` (SDL-gated) |
| pycvc bindings | `bindings/pycvc/pycvc_stream.i` | `pycvc` |
