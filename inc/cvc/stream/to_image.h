/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STREAM_TO_IMAGE_H__
#define __CVC_STREAM_TO_IMAGE_H__

#include <cvc/image/image.h>
#include <cvc/stream/frame.h>

// Bridge a video stream frame to a cvc::image with NO copy (roadmap
// STATE_BINARY_STREAMING.md Phase 2 / §7 H2). The returned image ALIASES the
// frame's bytes and co-owns the frame's keepalive, so the underlying slab
// survives for the image's lifetime even if the frame_ptr is dropped first.
//
// This lives in cvc::stream (not cvc::image) so cvc::image keeps zero dependency
// on cvc::stream: the dependency points stream -> image.

namespace cvc {
namespace stream {

// Alias a video_raw frame as a cvc::image (zero copy). Throws std::runtime_error
// when the frame cannot be aliased losslessly: a null/non-video frame, an unknown
// or channel-swapped codec (only codecs whose in-memory channel order matches
// cvc::image are accepted — never a silent reinterpret), a padded row stride
// (cvc::image has no row padding), a bottom-left origin (would display flipped),
// or a buffer smaller than w*h*bytes_per_pixel. A stride of 0 is treated as
// densely packed.
cvc::image to_image(const frame_ptr &fp);

} // namespace stream
} // namespace cvc

#endif // __CVC_STREAM_TO_IMAGE_H__
