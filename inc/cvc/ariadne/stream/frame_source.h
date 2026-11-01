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

  // The exact byte size of one frame this source produces (e.g. w*h*4 for dense rgba8). The stream
  // checks this against its pool slab size at start_producer() and refuses a source whose frames
  // would not fit — so a width/height/codec mismatch is a loud rejection, not an invisible dead
  // stream that discards every frame. Must match what fill() writes.
  virtual std::size_t frame_bytes() const = 0;

  // Fill ONE frame into `buf` (capacity `cap` bytes, the stream's slab size) and report what was
  // written. The source knows its own format (width/height/codec), fixed when it was created to
  // match the stream it feeds, so it needs no format argument. Called on the producer thread.
  virtual produced_frame fill(std::uint8_t *buf, std::size_t cap) = 0;

  // The ACTUAL pixel dimensions of the frame the last fill() produced, for a video source whose
  // content size can differ from its slab budget — e.g. a camera that preserves the device's NATIVE
  // ASPECT RATIO within a max budget (a portrait phone camera stays portrait instead of being
  // stretched). 0 (the default) means "fixed — use the size the stream/source was created with". A
  // self-describing-codec wrapper (jpeg_encode_source) reads these to encode at the real size, so
  // the decoded frame carries the true aspect ratio. Read right after fill() returns bytes > 0.
  virtual int cur_width() const { return 0; }
  virtual int cur_height() const { return 0; }
};

// The size a SRC_W x SRC_H frame takes within a BUDGET_W x BUDGET_H budget: its aspect ratio kept,
// never upscaled. The budget turns with the frame, its long side bounding the frame's long side, so
// a portrait phone camera in a 320x240 budget comes out 240x320 (a 9:16 one 180x320), not a
// postage stamp 180x240; either way it fits the budget's pixel count, so the slab is unchanged. A
// degenerate source or budget gives the budget.
inline void fit_frame(int src_w, int src_h, int budget_w, int budget_h, int &out_w, int &out_h) {
  out_w = budget_w;
  out_h = budget_h;
  if (src_w <= 0 || src_h <= 0 || budget_w <= 0 || budget_h <= 0)
    return;
  int bw = budget_w, bh = budget_h;
  if ((src_h > src_w) != (bh > bw)) { // orient the budget like the frame
    bw = budget_h;
    bh = budget_w;
  }
  double s = static_cast<double>(bw) / src_w;
  if (static_cast<double>(bh) / src_h < s)
    s = static_cast<double>(bh) / src_h;
  if (s > 1.0)
    s = 1.0;
  const int w = static_cast<int>(src_w * s + 0.5), h = static_cast<int>(src_h * s + 0.5);
  out_w = w < 1 ? 1 : (w > bw ? bw : w);
  out_h = h < 1 ? 1 : (h > bh ? bh : h);
}

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_FRAME_SOURCE_H__
