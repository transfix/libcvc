/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cvc/image/image.h>
#include <mutex>
#include <stdexcept>

namespace cvc {

// ── cvc::image value type ────────────────────────────────────────────────────

image::image() {}

image::image(int w, int h, pixel_format f, data_type dt) : _w(w), _h(h), _fmt(f), _dt(dt) {
  if (w > 0 && h > 0) {
    std::size_t n = size_bytes();
    _data.reset(new unsigned char[n]);
    std::memset(_data.get(), 0, n);
  }
}

image::image(int w, int h, pixel_format f, data_type dt, const void *src)
    : _w(w), _h(h), _fmt(f), _dt(dt) {
  if (w > 0 && h > 0) {
    std::size_t n = size_bytes();
    _data.reset(new unsigned char[n]);
    if (src)
      std::memcpy(_data.get(), src, n);
    else
      std::memset(_data.get(), 0, n);
  }
}

int image::channels() const {
  switch (_fmt) {
  case pixel_format::GRAY:
    return 1;
  case pixel_format::GRAY_ALPHA:
    return 2;
  case pixel_format::RGB:
    return 3;
  case pixel_format::RGBA:
    return 4;
  }
  return 4;
}

std::size_t image::bytes_per_channel() const {
  switch (_dt) {
  case data_type::u8:
    return 1;
  case data_type::u16:
    return 2;
  case data_type::f32:
    return 4;
  }
  return 1;
}

void image::detach() {
  // Copy-on-write: if the buffer is shared, give this image its own copy.
  if (_data && _data.use_count() > 1) {
    std::size_t n = size_bytes();
    boost::shared_array<unsigned char> copy(new unsigned char[n]);
    std::memcpy(copy.get(), _data.get(), n);
    _data = copy;
  }
}

unsigned char *image::data() {
  detach();
  return _data.get();
}

image image::flipped_vertical() const {
  if (empty())
    return *this;
  image out(_w, _h, _fmt, _dt);
  std::size_t stride = row_stride_bytes();
  const unsigned char *src = _data.get();
  unsigned char *dst = out._data.get();
  for (int y = 0; y < _h; ++y)
    std::memcpy(dst + std::size_t(y) * stride, src + std::size_t(_h - 1 - y) * stride, stride);
  return out;
}

image image::resized(int w, int h) const {
  if (empty() || w <= 0 || h <= 0)
    return image(w, h, _fmt, _dt);
  // Nearest-neighbour resample (works for any format/type; a higher-quality
  // filter is a follow-up, and the Magick handler can resample too).
  image out(w, h, _fmt, _dt);
  std::size_t bpp = bytes_per_pixel();
  const unsigned char *src = _data.get();
  unsigned char *dst = out._data.get();
  for (int y = 0; y < h; ++y) {
    int sy = static_cast<int>((static_cast<long long>(y) * _h) / h);
    if (sy >= _h)
      sy = _h - 1;
    for (int x = 0; x < w; ++x) {
      int sx = static_cast<int>((static_cast<long long>(x) * _w) / w);
      if (sx >= _w)
        sx = _w - 1;
      std::memcpy(dst + (std::size_t(y) * w + x) * bpp, src + (std::size_t(sy) * _w + sx) * bpp,
                  bpp);
    }
  }
  return out;
}

image image::converted(pixel_format f, data_type dt) const {
  if (f == _fmt && dt == _dt)
    return *this;
  if (empty())
    return image(_w, _h, f, dt);
  // Channel conversion is implemented for the common 8-bit texture path; other
  // per-channel types (u16/f32) round-trip only when the format is unchanged.
  if (dt != _dt)
    throw std::runtime_error("cvc::image::converted: data_type conversion not yet implemented");
  if (_dt != data_type::u8)
    throw std::runtime_error("cvc::image::converted: channel conversion implemented for u8 only");

  auto lum = [](int r, int g, int b) -> unsigned char {
    return static_cast<unsigned char>((r * 77 + g * 150 + b * 29) >> 8); // Rec.601-ish
  };
  image out(_w, _h, f, data_type::u8);
  const int sc = channels();
  const int dc = out.channels();
  const unsigned char *s = _data.get();
  unsigned char *d = out._data.get();
  std::size_t px = static_cast<std::size_t>(_w) * _h;
  for (std::size_t i = 0; i < px; ++i, s += sc, d += dc) {
    // read source as RGBA (fill sensible defaults for missing channels)
    unsigned char r, g, b, a = 255;
    if (sc == 1) {
      r = g = b = s[0];
    } else if (sc == 2) {
      r = g = b = s[0];
      a = s[1];
    } else { // 3 or 4
      r = s[0];
      g = s[1];
      b = s[2];
      if (sc == 4)
        a = s[3];
    }
    // write destination
    if (dc == 1) {
      d[0] = lum(r, g, b);
    } else if (dc == 2) {
      d[0] = lum(r, g, b);
      d[1] = a;
    } else { // 3 or 4
      d[0] = r;
      d[1] = g;
      d[2] = b;
      if (dc == 4)
        d[3] = a;
    }
  }
  return out;
}

// ── file extension helper ────────────────────────────────────────────────────

std::string image_file_extension(const std::string &path) {
  std::string::size_type dot = path.find_last_of('.');
  std::string::size_type slash = path.find_last_of("/\\");
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
    return "";
  std::string ext = path.substr(dot + 1);
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return ext;
}

// ── image_file_io registry ──────────────────────────────────────────────────

namespace {
std::vector<image_file_io::ptr> &registry() {
  static std::vector<image_file_io::ptr> r;
  return r;
}
std::mutex &registry_mutex() {
  static std::mutex m;
  return m;
}
} // namespace

// Defined in the handler translation units; register the built-in handlers.
void register_default_image_handlers();

namespace {
void ensure_defaults() {
  static std::once_flag once;
  std::call_once(once, []() { register_default_image_handlers(); });
}
} // namespace

void image_file_io::add(const ptr &handler) {
  if (!handler)
    return;
  std::lock_guard<std::mutex> lk(registry_mutex());
  registry().push_back(handler);
}

image_file_io::ptr image_file_io::for_read(const std::string &path) {
  ensure_defaults();
  std::lock_guard<std::mutex> lk(registry_mutex());
  for (const auto &h : registry())
    if (h->can_read(path))
      return h;
  return ptr();
}

image_file_io::ptr image_file_io::for_write_ext(const std::string &ext) {
  ensure_defaults();
  std::string e = ext;
  std::transform(e.begin(), e.end(), e.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  std::lock_guard<std::mutex> lk(registry_mutex());
  // Last registered wins (a fast-path handler registered after the catch-all
  // overrides it for the extensions it declares).
  ptr found;
  for (const auto &h : registry())
    for (const auto &he : h->extensions())
      if (he == e) {
        found = h;
        break;
      }
  return found;
}

std::vector<std::string> image_file_io::known_extensions() {
  ensure_defaults();
  std::lock_guard<std::mutex> lk(registry_mutex());
  std::vector<std::string> out;
  for (const auto &h : registry())
    for (const auto &e : h->extensions())
      if (std::find(out.begin(), out.end(), e) == out.end())
        out.push_back(e);
  return out;
}

// ── free-function I/O + static load/save ─────────────────────────────────────

image read_image(const std::string &path) {
  image_file_io::ptr h = image_file_io::for_read(path);
  if (!h)
    throw std::runtime_error("cvc::read_image: no handler can read '" + path + "'");
  return h->read(path);
}

void write_image(const image &img, const std::string &path, int quality) {
  ensure_defaults();
  const std::string ext = image_file_extension(path);
  // Candidates in write-priority order: last registered first (the same
  // last-wins rule as for_write_ext), earlier registrations as fallbacks. A
  // handler signals "can't encode this" by throwing — e.g. magick_image_io on
  // an ImageMagick built without the target format's delegate — and the next
  // candidate gets its chance (the stb encoder for png/jpg/bmp/tga), so a
  // crippled preferred writer degrades to another encoder instead of an error.
  // Collected under the lock, called outside it (a handler may re-enter I/O).
  std::vector<image_file_io::ptr> candidates;
  {
    std::lock_guard<std::mutex> lk(registry_mutex());
    const auto &r = registry();
    for (auto it = r.rbegin(); it != r.rend(); ++it)
      for (const auto &he : (*it)->extensions())
        if (he == ext) {
          candidates.push_back(*it);
          break;
        }
  }
  if (candidates.empty())
    throw std::runtime_error("cvc::write_image: no handler writes '." + ext + "'");
  std::string errors;
  for (const auto &h : candidates) {
    try {
      h->write(img, path, quality);
      return;
    } catch (const std::exception &e) {
      if (!errors.empty())
        errors += "; ";
      errors += h->id() + ": " + e.what();
    }
  }
  throw std::runtime_error("cvc::write_image: every handler for '." + ext + "' failed (" + errors +
                           ")");
}

image image::load(const std::string &path) { return read_image(path); }

void image::save(const std::string &path) const { write_image(*this, path, 90); }

void image::save(const std::string &path, int quality) const { write_image(*this, path, quality); }

} // namespace cvc
