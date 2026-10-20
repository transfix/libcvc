/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/capture/jpeg_encode_source.h>
#include <cvc/gl/jpeg_codec.h>
#include <utility>

namespace cvc {
namespace gl {
namespace capture {

namespace st = cvc::ariadne::stream;

jpeg_encode_source::jpeg_encode_source(std::unique_ptr<st::frame_source> inner, int w, int h,
                                       int quality)
    : inner_(std::move(inner)), w_(w), h_(h), quality_(quality), slab_(jpeg_stream_slab(w, h)),
      rgba_(static_cast<std::size_t>(w > 0 ? w : 1) * static_cast<std::size_t>(h > 0 ? h : 1) *
            4u) {}

st::produced_frame jpeg_encode_source::fill(std::uint8_t *buf, std::size_t cap) {
  st::produced_frame out; // bytes = 0: "no frame this tick"
  if (!inner_ || !buf || cap == 0 || rgba_.empty())
    return out;
  // Pull one rgba8 frame from the wrapped source into our scratch buffer.
  const st::produced_frame in = inner_->fill(rgba_.data(), rgba_.size());
  if (in.bytes == 0)
    return out; // the inner source had nothing this tick (warming up / between frames)
  // Encode at the inner source's ACTUAL frame size when it reports one — a camera that preserves
  // the device's native aspect ratio within our w_ x h_ budget produces a frame smaller than (and a
  // different shape from) w_ x h_, and the JPEG must carry those real dims so the decoded video
  // keeps the true ratio. Fall back to the fixed w_ x h_ (and if the reported size would overrun
  // the rgba scratch, which it never should). A frame whose JPEG would exceed the slab is dropped.
  int ew = inner_->cur_width(), eh = inner_->cur_height();
  if (ew <= 0 || eh <= 0 ||
      static_cast<std::size_t>(ew) * static_cast<std::size_t>(eh) * 4u > rgba_.size()) {
    ew = w_;
    eh = h_;
  }
  const std::size_t jbytes = encode_rgba_to_jpeg(rgba_.data(), ew, eh, quality_, buf, cap);
  if (jbytes == 0)
    return out;
  out.bytes = jbytes;
  out.pts_seconds = in.pts_seconds;
  return out;
}

} // namespace capture
} // namespace gl
} // namespace cvc
