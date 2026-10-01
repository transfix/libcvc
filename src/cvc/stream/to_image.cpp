/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/stream/to_image.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace stream {

namespace {

// Map a stream codec to a cvc::image (pixel_format, data_type) ONLY when the
// in-memory interleaved channel order matches cvc::image's. A byte-swapped order
// (e.g. "bgra8") or an unknown codec returns false -> the caller throws, rather
// than aliasing bytes under a mismatched interpretation (silent color/corruption).
bool codec_to_image_format(const std::string &codec, cvc::image::pixel_format &pf,
                           cvc::image::data_type &dt) {
  if (codec == "rgba8") {
    pf = cvc::image::pixel_format::RGBA;
    dt = cvc::image::data_type::u8;
    return true;
  }
  if (codec == "rgb8") {
    pf = cvc::image::pixel_format::RGB;
    dt = cvc::image::data_type::u8;
    return true;
  }
  if (codec == "graya8") {
    pf = cvc::image::pixel_format::GRAY_ALPHA;
    dt = cvc::image::data_type::u8;
    return true;
  }
  if (codec == "gray8") {
    pf = cvc::image::pixel_format::GRAY;
    dt = cvc::image::data_type::u8;
    return true;
  }
  return false;
}

} // namespace

cvc::image to_image(const frame_ptr &fp) {
  if (!fp)
    throw std::runtime_error("cvc::stream::to_image: null frame");
  if (!fp->data)
    throw std::runtime_error("cvc::stream::to_image: frame has null data");
  if (fp->format.kind != frame_kind::video_raw)
    throw std::runtime_error("cvc::stream::to_image: frame is not video_raw");

  cvc::image::pixel_format pf;
  cvc::image::data_type dt;
  if (!codec_to_image_format(fp->format.codec, pf, dt))
    throw std::runtime_error("cvc::stream::to_image: unsupported or channel-swapped codec '" +
                             fp->format.codec + "'");

  if (fp->format.origin != frame_origin::top_left)
    throw std::runtime_error(
        "cvc::stream::to_image: only top-left origin can be aliased zero-copy");

  const int w = fp->format.w;
  const int h = fp->format.h;
  if (w <= 0 || h <= 0)
    throw std::runtime_error("cvc::stream::to_image: non-positive dimensions");

  // Build the aliasing image first (cheap, no copy), then validate against its
  // own derived layout. On a validation failure the image is discarded and its
  // keepalive released, so no data is ever read through an invalid alias.
  cvc::image img(w, h, pf, dt, fp->data, fp->keepalive);
  const std::size_t dense_stride = img.row_stride_bytes();

  // stride == 0 means "densely packed" (frame_bytes() derived the size); a
  // positive stride must equal the dense stride (cvc::image has no row padding).
  if (fp->format.stride != 0 && static_cast<std::size_t>(fp->format.stride) != dense_stride)
    throw std::runtime_error("cvc::stream::to_image: padded row stride cannot be aliased");

  if (fp->size < img.size_bytes())
    throw std::runtime_error("cvc::stream::to_image: frame buffer smaller than w*h*bpp");

  return img;
}

} // namespace stream
} // namespace cvc
