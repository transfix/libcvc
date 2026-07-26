/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

#ifndef __CVC_IMAGE_H__
#define __CVC_IMAGE_H__

// cvc::image — a small, VTK-free raster image value type + a pluggable file-I/O
// registry, part of the Phase-1 image surface. Images decode BELOW the VTK line
// (in libcvc core, via ImageMagick Magick++ / stb) so cvcGL, pycvc, and the mesh
// loaders consume already-decoded pixels. The value type mirrors cvc::geometry:
// a reference-counted byte buffer shared on copy, copy-on-write on mutation.

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <cstddef>
#include <list>
#include <string>
#include <vector>

namespace cvc {

class image {
public:
  // Channel layout (interleaved) and per-channel storage type.
  enum class pixel_format { GRAY, GRAY_ALPHA, RGB, RGBA };
  enum class data_type { u8, u16, f32 };

  image();
  image(int w, int h, pixel_format f = pixel_format::RGBA, data_type dt = data_type::u8);
  // Copy `interleaved_src` (w*h*bpp bytes, row-major, top-left origin) into a new image.
  image(int w, int h, pixel_format f, data_type dt, const void *interleaved_src);

  // Decode from a file (dispatched by extension/magic through the registry).
  static image load(const std::string &path);

  // -- queries --
  bool empty() const { return _w <= 0 || _h <= 0 || !_data; }
  int width() const { return _w; }
  int height() const { return _h; }
  pixel_format format() const { return _fmt; }
  data_type type() const { return _dt; }
  int channels() const;               // 1..4, from format
  std::size_t bytes_per_channel() const; // 1/2/4, from data_type
  std::size_t bytes_per_pixel() const { return static_cast<std::size_t>(channels()) * bytes_per_channel(); }
  std::size_t row_stride_bytes() const { return static_cast<std::size_t>(_w) * bytes_per_pixel(); }
  std::size_t size_bytes() const { return row_stride_bytes() * static_cast<std::size_t>(_h < 0 ? 0 : _h); }

  // -- save (encode by extension) --
  void save(const std::string &path) const;
  void save(const std::string &path, int quality) const; // jpeg/webp quality 0..100

  // -- manipulate (non-mutating; return a new image) --
  image resized(int w, int h) const;                   // resample (nearest for now)
  image converted(pixel_format f, data_type dt) const; // channel/type convert
  image flipped_vertical() const;                      // GL/VTK bottom-left origin helper

  // -- raw pixel access --
  unsigned char *data();                     // detaches (copy-on-write), interleaved
  const unsigned char *data() const { return _data.get(); }
  // The owning buffer, for zero-copy pinning (e.g. a numpy view or a
  // vtkUnsignedCharArray::SetArray that must keep the storage alive).
  boost::shared_array<unsigned char> storage() const { return _data; }

private:
  void detach(); // copy-on-write: give this image a private copy of the buffer
  int _w = 0, _h = 0;
  pixel_format _fmt = pixel_format::RGBA;
  data_type _dt = data_type::u8;
  boost::shared_array<unsigned char> _data;
};

// Pluggable per-format handler, mirroring geometry_file_io / volume_file_io. A
// handler declares the extensions it writes and sniffs whether it can read a
// given path; the registry tries handlers in registration order (first that can
// read wins), so a zero-dependency fast-path handler can sit ahead of a
// catch-all.
class image_file_io {
public:
  typedef boost::shared_ptr<image_file_io> ptr;
  virtual ~image_file_io() {}
  virtual std::string id() const = 0;
  virtual const std::list<std::string> &extensions() const = 0; // e.g. {"png","jpg"}
  virtual bool can_read(const std::string &path) const = 0;
  virtual image read(const std::string &path) const = 0;
  virtual void write(const image &img, const std::string &path, int quality) const = 0;

  // Registry. add() appends (registration order = priority).
  static void add(const ptr &handler);
  static ptr for_read(const std::string &path);      // first can_read() wins
  static ptr for_write_ext(const std::string &ext);  // last-registered wins per ext
  static std::vector<std::string> known_extensions();
};

// Free-function I/O (mirrors read/writeVolumeFile). Default handlers register
// lazily on first call.
image read_image(const std::string &path);
void write_image(const image &img, const std::string &path, int quality = 90);

// lower-cased file extension without the dot ("" if none).
std::string image_file_extension(const std::string &path);

} // namespace cvc

#endif // __CVC_IMAGE_H__
