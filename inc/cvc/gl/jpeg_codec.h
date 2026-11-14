/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_JPEG_CODEC_H__
#define __CVC_GL_JPEG_CODEC_H__

// Minimal rgba8 <-> JPEG codec (libjpeg-turbo, tj3 API). Used to compress conference video for the
// network: a publisher encodes rgba8 camera/synthetic frames to JPEG (codec "mjpeg"), the relay
// forwards the opaque bytes, and the subscriber's stream_view decodes them for display. JPEG is
// ~10-50x smaller than raw rgba8, which is what makes video over a home-internet uplink feasible.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cvc {
namespace gl {

// A TurboJPEG compressor, kept by whoever encodes (tj3Init is not free, so its handle is reused
// from frame to frame): a producer's own. One thread at a time.
class jpeg_encoder {
public:
  jpeg_encoder() = default;
  ~jpeg_encoder();
  jpeg_encoder(const jpeg_encoder &) = delete;
  jpeg_encoder &operator=(const jpeg_encoder &) = delete;

  // Encode w*h packed rgba8 (top-left origin) into a JPEG written INTO `dst` (capacity `dst_cap`);
  // on success returns the JPEG byte count, else 0 (encode error, or the JPEG did not fit
  // `dst_cap`). `quality` is 1..100.
  std::size_t encode(const std::uint8_t *rgba, int w, int h, int quality, std::uint8_t *dst,
                     std::size_t dst_cap);

private:
  void *handle_ = nullptr; // the tjhandle, made on the first encode
};

// A TurboJPEG decompressor, kept by whoever decodes (a stream view's decoder). One thread at a
// time.
class jpeg_decoder {
public:
  jpeg_decoder() = default;
  ~jpeg_decoder();
  jpeg_decoder(const jpeg_decoder &) = delete;
  jpeg_decoder &operator=(const jpeg_decoder &) = delete;

  // Decode a JPEG (`data`[`size`]) to packed rgba8 into `out` (resized to w*h*4), filling `out_w` /
  // `out_h`. Returns true on success.
  bool decode(const std::uint8_t *data, std::size_t size, std::vector<std::uint8_t> &out,
              int &out_w, int &out_h);

private:
  void *handle_ = nullptr; // the tjhandle, made on the first decode
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_JPEG_CODEC_H__
