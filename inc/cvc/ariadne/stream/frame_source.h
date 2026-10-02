/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_FRAME_SOURCE_H__
#define __CVC_ARIADNE_STREAM_FRAME_SOURCE_H__

#include <cstddef>
#include <cstdint>

// frame_source — a PULL source of frame payloads that a stream's producer thread drives, one frame
// per tick (roadmap STATE_BINARY_STREAMING.md §"Phase 3 — producers"). It is the abstraction every
// real producer implements: a synthetic test pattern (synthetic_source.h), an SDL3 camera/mic
// capture (Phase 3b), a decoded video file (Phase 3c). The stream owns the source and the
// producer_thread that drives it; see stream::start_producer().
//
// THREADING: fill() runs on the dedicated producer thread (never the pump / compute pool). It must
// be self-contained — it fills a raw slab buffer and returns; it MUST NOT touch the DSL evaluator,
// the cvc::state tree, or the stream_channel directly (the stream's producer tick does the
// acquire / publish / seq-post around it). The stream stops (joins) the producer before destroying
// the source, so whatever the source captures outlives each fill().
//
// BACKPRESSURE: the stream only calls fill() once it has a free pool slab; when the pool is
// exhausted (every slab pinned by a slow consumer) the producer DROPS at the source and does not
// call fill() — a source never blocks or back-pressures (a camera can't be back-pressured; §3.2).

namespace cvc {
namespace ariadne {
namespace stream {

// What a single fill() produced. `bytes == 0` with `stop == false` means "no frame this tick"
// (the slab is discarded, the producer keeps running); `stop == true` means the source is spent
// (e.g. end-of-file) and the producer loop exits.
struct produced_frame {
  std::size_t bytes = 0;    // payload bytes written into the slab (0 = skip this tick)
  double pts_seconds = 0.0; // presentation timestamp on the producer clock
  bool stop = false;        // true => stop the producer (source exhausted)
};

class frame_source {
public:
  virtual ~frame_source() = default;

  // Fill ONE frame into `buf` (capacity `cap` bytes, the stream's slab size) and report what was
  // written. The source knows its own format (width/height/codec), fixed when it was created to
  // match the stream it feeds, so it needs no format argument. Called on the producer thread.
  virtual produced_frame fill(std::uint8_t *buf, std::size_t cap) = 0;
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_FRAME_SOURCE_H__
