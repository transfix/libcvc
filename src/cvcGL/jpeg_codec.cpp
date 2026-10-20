/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/gl/jpeg_codec.h>

#ifdef CVC_HAVE_TURBOJPEG
#include <turbojpeg.h>

namespace cvc {
namespace gl {

namespace {
// One TurboJPEG handle per thread per direction, reused across frames (tj3Init is not free).
// Producer fill() and the render-thread decode run on their own threads; thread_local keeps each a
// private handle with no locking.
tjhandle compressor() {
  thread_local tjhandle h = tj3Init(TJINIT_COMPRESS);
  return h;
}
tjhandle decompressor() {
  thread_local tjhandle h = tj3Init(TJINIT_DECOMPRESS);
  return h;
}
} // namespace

std::size_t encode_rgba_to_jpeg(const std::uint8_t *rgba, int w, int h, int quality,
                                std::uint8_t *dst, std::size_t dst_cap) {
  tjhandle tj = compressor();
  if (!tj || !rgba || !dst || w <= 0 || h <= 0 || dst_cap == 0)
    return 0;
  if (quality < 1)
    quality = 1;
  else if (quality > 100)
    quality = 100;
  // NOREALLOC: write straight into the caller's slab (the stream frame buffer). If the JPEG would
  // exceed dst_cap, tj3Compress8 fails rather than allocating — we then drop the frame.
  tj3Set(tj, TJPARAM_NOREALLOC, 1);
  tj3Set(tj, TJPARAM_QUALITY, quality);
  tj3Set(tj, TJPARAM_SUBSAMP, TJSAMP_420);
  unsigned char *jpeg = dst;
  std::size_t jpeg_size = dst_cap; // in: buffer capacity; out: actual JPEG size
  if (tj3Compress8(tj, rgba, w, /*pitch=*/0, h, TJPF_RGBA, &jpeg, &jpeg_size) != 0)
    return 0;
  return (jpeg == dst) ? jpeg_size : 0; // sanity: it must have used our buffer
}

bool decode_jpeg_to_rgba(const std::uint8_t *data, std::size_t size, std::vector<std::uint8_t> &out,
                         int &out_w, int &out_h) {
  tjhandle tj = decompressor();
  if (!tj || !data || size == 0)
    return false;
  if (tj3DecompressHeader(tj, data, size) != 0)
    return false;
  const int w = tj3Get(tj, TJPARAM_JPEGWIDTH);
  const int h = tj3Get(tj, TJPARAM_JPEGHEIGHT);
  if (w <= 0 || h <= 0)
    return false;
  out.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u);
  if (tj3Decompress8(tj, data, size, out.data(), /*pitch=*/0, TJPF_RGBA) != 0)
    return false;
  out_w = w;
  out_h = h;
  return true;
}

} // namespace gl
} // namespace cvc

#else // !CVC_HAVE_TURBOJPEG — no JPEG codec in this build; the "mjpeg" path yields no video.

namespace cvc {
namespace gl {
std::size_t encode_rgba_to_jpeg(const std::uint8_t *, int, int, int, std::uint8_t *, std::size_t) {
  return 0;
}
bool decode_jpeg_to_rgba(const std::uint8_t *, std::size_t, std::vector<std::uint8_t> &, int &,
                         int &) {
  return false;
}
} // namespace gl
} // namespace cvc

#endif // CVC_HAVE_TURBOJPEG
