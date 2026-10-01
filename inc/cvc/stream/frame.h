/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STREAM_FRAME_H__
#define __CVC_STREAM_FRAME_H__

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

// cvc::stream — Layer-(b) real-time in-process frame transport
// (roadmap docs/roadmap/STATE_BINARY_STREAMING.md, Phase 1).
//
// A `frame` is a single unit of a stream (a video frame, an audio buffer, a
// sensor sample). It is IMMUTABLE after publish and borrows its bytes from a
// keepalive owner (a frame_pool slab, an image buffer, a wasm SharedArrayBuffer,
// ...), so passing a frame between threads is a shared_ptr refcount bump and
// NEVER a byte copy. A consumer that holds a frame_ptr keeps the underlying
// storage alive (refcount > 0), so a concurrent producer can never recycle a
// slab out from under it.

namespace cvc {
namespace stream {

enum class frame_kind { unknown, video_raw, audio_pcm, sensor };

// Raster row order for video_raw frames. top_left matches cvc::image (and the
// zero-copy adopt path); bottom_left (e.g. a GL framebuffer/camera capture)
// cannot be aliased into a top-left image without a flip, so the zero-copy
// bridge rejects it.
enum class frame_origin { top_left, bottom_left };

// Describes the wire/pixel layout of every frame on a stream. Round-trips into
// the /streams/<id> descriptor node so a remote consumer can negotiate.
struct format_desc {
  frame_kind kind = frame_kind::unknown;
  std::string codec; // e.g. "rgba8", "s16le"
  int w = 0, h = 0, stride = 0;
  frame_origin origin = frame_origin::top_left; // raster row order (video_raw)
  int sample_rate = 0, channels = 0;
  // Explicit per-frame byte size. Required for non-video kinds (audio/sensor)
  // whose size is not stride*h; for video it may stay 0 and be derived.
  std::size_t bytes = 0;
  std::string extra; // opaque; carried verbatim into the descriptor node

  // The number of bytes one frame occupies. Prefer the explicit `bytes`; fall
  // back to stride*h for raster video. Returns 0 when neither is set (an
  // unsized format — stream::open rejects it rather than build a 0-byte pool).
  std::size_t frame_bytes() const noexcept {
    if (bytes != 0)
      return bytes;
    if (stride > 0 && h > 0)
      return static_cast<std::size_t>(stride) * static_cast<std::size_t>(h);
    return 0;
  }
};

// One published frame. All fields are const-once-published; the transport never
// mutates a frame after publish() returns it.
struct frame {
  const std::uint8_t *data = nullptr; // borrowed; owned by `keepalive`
  std::size_t size = 0;               // valid bytes at `data`
  std::int64_t seq = 0;               // monotonic per stream
  double pts_seconds = 0.0;           // producer clock (Q4: cvc::world time base, Phase 6)
  format_desc format;
  std::shared_ptr<void> keepalive; // owns `data`: a pool slab, image storage, SAB, ...
};

// A borrow of a frame. Copying it bumps a refcount; it never copies bytes.
using frame_ptr = std::shared_ptr<const frame>;

} // namespace stream
} // namespace cvc

#endif // __CVC_STREAM_FRAME_H__
