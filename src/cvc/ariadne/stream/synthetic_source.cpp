/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/ariadne/stream/synthetic_source.h>

namespace cvc {
namespace ariadne {
namespace stream {

synthetic_source::synthetic_source(int w, int h, double fps)
    : w_(std::max(w, 0)), h_(std::max(h, 0)), frame_dt_(fps > 0.0 ? 1.0 / fps : 0.0) {}

produced_frame synthetic_source::fill(std::uint8_t *buf, std::size_t cap) {
  produced_frame out;
  const std::size_t need = static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_) * 4u;
  if (w_ <= 0 || h_ <= 0 || need == 0 || cap < need || buf == nullptr)
    return out; // bytes = 0: nothing to fill (the producer discards the slab and keeps running)

  // A diagonal gradient offset by the frame counter — every frame differs, so a consumer sees
  // motion and the stream's seq strictly advances. Cheap per-pixel math (no allocation, no lib).
  const std::uint8_t t = static_cast<std::uint8_t>(counter_);
  std::uint8_t *p = buf;
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) {
      p[0] = static_cast<std::uint8_t>(x + t); // R sweeps horizontally over time
      p[1] = static_cast<std::uint8_t>(y + t); // G sweeps vertically over time
      p[2] = static_cast<std::uint8_t>(x + y); // B: a static diagonal
      p[3] = 0xFF;                             // A opaque
      p += 4;
    }
  }

  out.bytes = need;
  out.pts_seconds = static_cast<double>(counter_) * frame_dt_;
  ++counter_;
  return out;
}

} // namespace stream
} // namespace ariadne
} // namespace cvc
