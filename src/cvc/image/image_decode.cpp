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
bool stb_decode_gif_memory(const void *data, std::size_t size, std::vector<image> &frames,
                           std::vector<int> &delays, std::string *why);
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

// A GIF's frame count, from its block structure (no pixel decoded), or -1 when it does not parse:
// so an animation too large to keep is known before stb decodes every frame at once.
int gif_frame_count(const unsigned char *p, std::size_t n) {
  if (n < 13)
    return -1;
  std::size_t at = 13;
  if (p[10] & 0x80)
    at += 3u << ((p[10] & 7) + 1); // the global colour table
  const auto sub_blocks = [&]() {
    while (at < n && p[at] != 0)
      at += 1u + p[at];
    ++at; // the terminator
  };
  int frames = 0;
  while (at < n) {
    const unsigned char b = p[at++];
    if (b == 0x3B)
      return frames; // the trailer
    if (b == 0x21) { // an extension: its label, then data sub-blocks
      ++at;
      sub_blocks();
    } else if (b == 0x2C) { // an image: its descriptor, local colour table, LZW size, data
      if (at + 9 > n)
        return -1;
      const unsigned char flags = p[at + 8];
      at += 9;
      if (flags & 0x80)
        at += 3u << ((flags & 7) + 1);
      ++at;
      sub_blocks();
      ++frames;
    } else {
      return -1;
    }
  }
  return frames; // no trailer: what was there
}

} // namespace

int image_animation::total_ms() const {
  int t = 0;
  for (int d : delays_ms)
    t += d;
  return t;
}

image_animation decode_image_animation(const void *data, std::size_t size,
                                       const image_decode_limits &limits, std::string *why,
                                       int max_frames, std::size_t max_decoded_bytes) {
  image_animation out;
  const auto still = [&]() {
    out.frames.clear();
    out.delays_ms.clear();
    image img = decode_image(data, size, limits, why);
    if (!img.empty()) {
      out.frames.push_back(img);
      out.delays_ms.push_back(0);
    }
    return out;
  };
  try {
    const auto *p = static_cast<const unsigned char *>(data);
    if (!p || size == 0 || size > limits.max_bytes)
      return still();
    const std::string fmt = sniff_image_format(p, size);
    if (fmt == "gif") {
      int w = 0, h = 0;
      const int count = gif_frame_count(p, size);
      if (count <= 1 || count > max_frames || !detail::stb_info_memory(p, size, w, h) ||
          !within(limits, w, h, nullptr) ||
          static_cast<std::size_t>(w) * h * 4 * static_cast<std::size_t>(count) > max_decoded_bytes)
        return still();
      std::string reason;
      if (!detail::stb_decode_gif_memory(p, size, out.frames, out.delays_ms, &reason) ||
          out.frames.empty())
        return still();
      for (int &d : out.delays_ms)
        d = d <= 10 ? 100 : d; // browsers show a 0 or 10 ms GIF frame for 100 ms
      return out;
    }
#if defined(CVC_HAVE_WEBP) && defined(CVC_HAVE_WEBP_DEMUX)
    if (fmt == "webp") {
      WebPBitstreamFeatures f;
      if (WebPGetFeatures(p, size, &f) != VP8_STATUS_OK || !f.has_animation)
        return still();
      WebPAnimDecoderOptions opt;
      if (!WebPAnimDecoderOptionsInit(&opt))
        return still();
      opt.color_mode = MODE_RGBA;
      opt.use_threads = 0;
      WebPData wd{p, size};
      WebPAnimDecoder *dec = WebPAnimDecoderNew(&wd, &opt);
      if (!dec)
        return still();
      WebPAnimInfo info;
      const bool ok =
          WebPAnimDecoderGetInfo(dec, &info) &&
          within(limits, static_cast<int>(info.canvas_width), static_cast<int>(info.canvas_height),
                 nullptr) &&
          info.frame_count >= 1 && static_cast<int>(info.frame_count) <= max_frames &&
          static_cast<std::size_t>(info.canvas_width) * info.canvas_height * 4 * info.frame_count <=
              max_decoded_bytes;
      if (!ok) {
        WebPAnimDecoderDelete(dec);
        return still();
      }
      int prev = 0;
      while (WebPAnimDecoderHasMoreFrames(dec)) {
        uint8_t *buf = nullptr;
        int ts = 0;
        if (!WebPAnimDecoderGetNext(dec, &buf, &ts))
          break;
        out.frames.emplace_back(static_cast<int>(info.canvas_width),
                                static_cast<int>(info.canvas_height), image::pixel_format::RGBA,
                                image::data_type::u8, buf);
        const int d = ts - prev;
        out.delays_ms.push_back(d <= 10 ? 100 : d);
        prev = ts;
      }
      WebPAnimDecoderDelete(dec);
      if (out.frames.empty())
        return still();
      return out;
    }
#endif
  } catch (const std::exception &e) {
    set_why(why, e.what());
    return image_animation{};
  }
  return still();
}

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
