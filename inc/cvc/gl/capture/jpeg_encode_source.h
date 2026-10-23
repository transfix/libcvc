/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_CAPTURE_JPEG_ENCODE_SOURCE_H__
#define __CVC_GL_CAPTURE_JPEG_ENCODE_SOURCE_H__

// A frame_source that wraps an rgba8 source (a camera or a synthetic pattern) and emits JPEG frames
// instead — the publisher side of conference video compression. The wrapped stream is opened with
// codec "mjpeg" and a generous byte slab (JPEG is variable-size, one frame per fill); the relay
// forwards the opaque bytes and the subscriber's stream_view decodes them. See jpeg_codec.h.

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/stream/frame_source.h>
#include <memory>
#include <vector>

namespace cvc {
namespace gl {
namespace capture {

// The byte slab a "mjpeg" stream of w*h video should open with. A quality-capped JPEG is well under
// one byte per pixel; w*h is a safe ceiling (a frame whose JPEG would exceed it is simply dropped).
inline std::size_t jpeg_stream_slab(int w, int h) {
  return static_cast<std::size_t>(w > 0 ? w : 1) * static_cast<std::size_t>(h > 0 ? h : 1);
}

class jpeg_encode_source : public cvc::ariadne::stream::frame_source {
public:
  // `inner` produces packed rgba8 of exactly w*h; this source encodes each to JPEG at `quality`.
  jpeg_encode_source(std::unique_ptr<cvc::ariadne::stream::frame_source> inner, int w, int h,
                     int quality = 70);

  std::size_t frame_bytes() const override { return slab_; }
  cvc::ariadne::stream::produced_frame fill(std::uint8_t *buf, std::size_t cap) override;

private:
  std::unique_ptr<cvc::ariadne::stream::frame_source> inner_;
  int w_, h_, quality_;
  std::size_t slab_;
  std::vector<std::uint8_t> rgba_; // scratch: the inner source's rgba8 frame
};

} // namespace capture
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAPTURE_JPEG_ENCODE_SOURCE_H__
