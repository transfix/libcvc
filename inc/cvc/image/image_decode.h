/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_IMAGE_DECODE_H__
#define __CVC_IMAGE_DECODE_H__

// cvc::decode_image — decode an encoded image held in memory (a reply body, a blob), with no file.
// The format is sniffed from the bytes, not named: PNG, JPEG, GIF (its first frame) and BMP through
// stb_image; WebP through libwebp where the build has it (an animated WebP gives its first frame);
// TIFF, AVIF and HEIC through ImageMagick where the build has that, the coder named from the magic
// bytes so the input never picks one of its own. The result is RGBA u8.
//
// Bounded, because the bytes usually come from the network: an image whose header claims more
// than `limits.max_side` pixels a side, or more than `limits.max_pixels` in all, is refused
// before any pixel is decoded, and so is an input longer than `limits.max_bytes`.

#include <cstddef>
#include <cvc/image/image.h>
#include <string>
#include <vector>

namespace cvc {

struct image_decode_limits {
  int max_side = 8192;                          // either dimension, in pixels
  std::size_t max_pixels = 40u * 1000u * 1000u; // width * height
  std::size_t max_bytes = 64u * 1024u * 1024u;  // the encoded input
};

// Decode `size` bytes at `data`. Returns an empty image and sets `*why` (when given) on failure:
// an unrecognized or unsupported format, a truncated or corrupt file, or a limit exceeded. Never
// throws. Thread-safe.
image decode_image(const void *data, std::size_t size, const image_decode_limits &limits,
                   std::string *why = nullptr);
inline image decode_image(const void *data, std::size_t size, std::string *why = nullptr) {
  return decode_image(data, size, image_decode_limits{}, why);
}

// An animated picture's frames (GIF, animated WebP), each the canvas as it shows at that point,
// with how long it shows. A still picture is one frame.
struct image_animation {
  std::vector<image> frames;
  std::vector<int> delays_ms; // one per frame
  int total_ms() const;
};

// decode_image, keeping every frame of an animated GIF or WebP (APNG and the rest: the first
// frame). Bounded too: past `max_frames` frames, or past `max_decoded_bytes` of frames in all, the
// picture is taken as still (its first frame). Empty, with `*why`, on failure.
image_animation decode_image_animation(const void *data, std::size_t size,
                                       const image_decode_limits &limits,
                                       std::string *why = nullptr, int max_frames = 512,
                                       std::size_t max_decoded_bytes = 128u * 1024u * 1024u);

// The format decode_image would take the bytes as: "png", "jpeg", "gif", "bmp", "webp", or ""
// when they are none of those. Reads at most the first 16 bytes.
std::string sniff_image_format(const void *data, std::size_t size);

// Whether this build decodes WebP (libwebp was found).
bool have_webp_decoder();

} // namespace cvc

#endif // __CVC_IMAGE_DECODE_H__
