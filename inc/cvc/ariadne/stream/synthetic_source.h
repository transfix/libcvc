/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_STREAM_SYNTHETIC_SOURCE_H__
#define __CVC_ARIADNE_STREAM_SYNTHETIC_SOURCE_H__

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame_source.h>

// synthetic_source — a hardware-free frame_source that fills a MOVING rgba8 test pattern, so a
// stream can be exercised end to end (producer -> pool -> channel -> sink) with no camera or file
// (roadmap STATE_BINARY_STREAMING.md §"Phase 3 — producers": "a synthetic test-pattern producer").
// It is the reference frame_source: PR 3b/3c (SDL3 camera, decoded file) implement the same
// interface. The pattern shifts every frame so consumers observe real motion and the seq advances.

namespace cvc {
namespace ariadne {
namespace stream {

class synthetic_source : public frame_source {
public:
  // `w` x `h` rgba8 frames (4 bytes/pixel = the stream's dense rgba8 slab). `fps` only stamps the
  // produced pts (the producer_thread's own rate governs real cadence); defaults to 30.
  synthetic_source(int w, int h, double fps = 30.0);

  std::size_t frame_bytes() const override {
    return static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_) * 4u; // dense rgba8
  }

  // Writes a w*h*4 rgba8 frame whose pattern is offset by the frame counter (motion), returns
  // w*h*4 bytes and a pts advanced by 1/fps. Skips (bytes = 0) only if the slab is too small for
  // one frame. Never stops on its own (an endless generator).
  produced_frame fill(std::uint8_t *buf, std::size_t cap) override;

private:
  int w_;
  int h_;
  double frame_dt_;           // 1/fps, added to pts each frame
  std::uint64_t counter_ = 0; // frames produced so far (drives both the motion and the pts)
};

} // namespace stream
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_STREAM_SYNTHETIC_SOURCE_H__
