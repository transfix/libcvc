/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

// cvc::decode_image — see inc/cvc/image/image_decode.h.

#include <cstring>
#include <cvc/image/image_decode.h>
#include <exception>
#include <string>

#ifdef CVC_HAVE_WEBP
#include <webp/decode.h>
#ifdef CVC_HAVE_WEBP_DEMUX
#include <webp/demux.h>
#endif
#endif

namespace cvc {

namespace detail {
// stb_io.cpp, where stb's implementation lives.
bool stb_info_memory(const void *data, std::size_t size, int &w, int &h);
image stb_decode_memory(const void *data, std::size_t size, std::string *why);
#ifdef CVC_ENABLE_IMAGEMAGICK
// magick_io.cpp.
image magick_decode_memory(const void *data, std::size_t size, const std::string &format,
                           std::string *why);
#endif
} // namespace detail

namespace {

bool starts_with(const unsigned char *p, std::size_t n, const char *magic, std::size_t at = 0) {
  const std::size_t m = std::strlen(magic);
  return n >= at + m && std::memcmp(p + at, magic, m) == 0;
}

// The ISO base media formats ImageMagick may decode for us: the `ftyp` box's major brand.
std::string ftyp_brand(const unsigned char *p, std::size_t n) {
  if (!starts_with(p, n, "ftyp", 4) || n < 12)
    return std::string();
  return std::string(reinterpret_cast<const char *>(p + 8), 4);
}

void set_why(std::string *why, const std::string &message) {
  if (why)
    *why = message;
}

bool within(const image_decode_limits &limits, int w, int h, std::string *why) {
  if (w <= 0 || h <= 0) {
    set_why(why, "the image has no pixels");
    return false;
  }
  if (w > limits.max_side || h > limits.max_side ||
      static_cast<std::size_t>(w) * static_cast<std::size_t>(h) > limits.max_pixels) {
    set_why(why, "the image is too large (" + std::to_string(w) + "x" + std::to_string(h) +
                     ", at most " + std::to_string(limits.max_side) + " a side)");
    return false;
  }
  return true;
}

#ifdef CVC_HAVE_WEBP
image webp_decode(const unsigned char *p, std::size_t n, const image_decode_limits &limits,
                  std::string *why) {
  WebPBitstreamFeatures f;
  if (WebPGetFeatures(p, n, &f) != VP8_STATUS_OK) {
    set_why(why, "webp: corrupt or truncated");
    return image();
  }
  if (!within(limits, f.width, f.height, why))
    return image();
  const unsigned char *frame = p;
  std::size_t frame_size = n;
#ifdef CVC_HAVE_WEBP_DEMUX
  // An animated WebP's frames are in ANMF chunks the plain decoder doesn't read: take the first.
  WebPDemuxer *demux = nullptr;
  WebPIterator it;
  bool have_frame = false;
  if (f.has_animation) {
    WebPData data{p, n};
    demux = WebPDemux(&data);
    if (demux && WebPDemuxGetFrame(demux, 1, &it)) {
      frame = it.fragment.bytes;
      frame_size = it.fragment.size;
      have_frame = true;
    }
  }
#else
  if (f.has_animation) {
    set_why(why, "webp: an animated WebP needs libwebpdemux, which this build lacks");
    return image();
  }
#endif
  int w = 0, h = 0;
  uint8_t *px = WebPDecodeRGBA(frame, frame_size, &w, &h);
#ifdef CVC_HAVE_WEBP_DEMUX
  if (have_frame)
    WebPDemuxReleaseIterator(&it);
  if (demux)
    WebPDemuxDelete(demux);
#endif
  if (!px) {
    set_why(why, "webp: corrupt or truncated");
    return image();
  }
  image out;
  if (within(limits, w, h, why))
    out = image(w, h, image::pixel_format::RGBA, image::data_type::u8, px);
  WebPFree(px);
  return out;
}
#endif

} // namespace

std::string sniff_image_format(const void *data, std::size_t size) {
  const auto *p = static_cast<const unsigned char *>(data);
  if (!p)
    return std::string();
  if (starts_with(p, size, "\x89PNG\r\n\x1a\n"))
    return "png";
  if (size >= 3 && p[0] == 0xFF && p[1] == 0xD8 && p[2] == 0xFF)
    return "jpeg";
  if (starts_with(p, size, "GIF87a") || starts_with(p, size, "GIF89a"))
    return "gif";
  if (starts_with(p, size, "BM"))
    return "bmp";
  if (starts_with(p, size, "RIFF") && starts_with(p, size, "WEBP", 8))
    return "webp";
  return std::string();
}

bool have_webp_decoder() {
#ifdef CVC_HAVE_WEBP
  return true;
#else
  return false;
#endif
}

image decode_image(const void *data, std::size_t size, const image_decode_limits &limits,
                   std::string *why) {
  try {
    const auto *p = static_cast<const unsigned char *>(data);
    if (!p || size == 0) {
      set_why(why, "no image data");
      return image();
    }
    if (size > limits.max_bytes) {
      set_why(why, "the image file is too large (" + std::to_string(size) + " bytes)");
      return image();
    }
    const std::string fmt = sniff_image_format(p, size);
    if (fmt == "webp") {
#ifdef CVC_HAVE_WEBP
      return webp_decode(p, size, limits, why);
#elif defined(CVC_ENABLE_IMAGEMAGICK)
      image img = detail::magick_decode_memory(p, size, "WEBP", why);
      return img.empty() || within(limits, img.width(), img.height(), why) ? img : image();
#else
      set_why(why, "webp: this build has no WebP decoder");
      return image();
#endif
    }
    if (!fmt.empty()) {
      int w = 0, h = 0;
      if (!detail::stb_info_memory(p, size, w, h)) {
        set_why(why, fmt + ": corrupt or truncated");
        return image();
      }
      if (!within(limits, w, h, why))
        return image();
      std::string reason;
      image img = detail::stb_decode_memory(p, size, &reason);
      if (img.empty())
        set_why(why, fmt + ": " + reason);
      return img;
    }
    // Formats ImageMagick may decode, named from their magic so the bytes never choose a coder.
    std::string magick;
    if (size >= 4 && ((p[0] == 'I' && p[1] == 'I' && p[2] == 42 && p[3] == 0) ||
                      (p[0] == 'M' && p[1] == 'M' && p[2] == 0 && p[3] == 42)))
      magick = "TIFF";
    else if (const std::string brand = ftyp_brand(p, size); brand == "avif" || brand == "avis")
      magick = "AVIF";
    else if (brand == "heic" || brand == "heix" || brand == "mif1")
      magick = "HEIC";
    if (magick.empty()) {
      set_why(why, "not an image this build decodes (PNG, JPEG, GIF, BMP, WebP)");
      return image();
    }
#ifdef CVC_ENABLE_IMAGEMAGICK
    image img = detail::magick_decode_memory(p, size, magick, why);
    return img.empty() || within(limits, img.width(), img.height(), why) ? img : image();
#else
    set_why(why, "this build has no decoder for " + magick);
    return image();
#endif
  } catch (const std::exception &e) {
    set_why(why, e.what());
  } catch (...) {
    set_why(why, "decode failed");
  }
  return image();
}

} // namespace cvc
