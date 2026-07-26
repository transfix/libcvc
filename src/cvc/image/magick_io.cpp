/*
  Copyright 2024 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License version 2.1 as published by the
  Free Software Foundation.
*/

// magick_image_io — the catch-all cvc::image handler backed by ImageMagick
// Magick++ (already linked PUBLIC into libcvc core; VTK-free). Loads to / saves
// from 8-bit interleaved RGBA; Magick's CharPixel storage down-converts the
// Q16-HDRI internal quantum, invisible to callers. Registered as the default
// (and, for now, only) image handler; a zero-dependency stb fast-path handler
// can be registered AHEAD of it later for png/jpg/bmp/tga.

#include <cvc/image/image.h>

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h>
#include <mutex>
#include <stdexcept>

namespace cvc {
namespace {

void init_magick_once() {
  static std::once_flag once;
  std::call_once(once, []() { Magick::InitializeMagick(nullptr); });
}

class magick_image_io : public image_file_io {
public:
  magick_image_io() { _exts = {"png", "jpg", "jpeg", "tif", "tiff", "bmp", "tga", "gif", "webp"}; }

  std::string id() const override { return "magick_image_io"; }
  const std::list<std::string> &extensions() const override { return _exts; }

  bool can_read(const std::string &path) const override {
    std::string e = image_file_extension(path);
    for (const auto &x : _exts)
      if (x == e)
        return true;
    return false;
  }

  image read(const std::string &path) const override {
    init_magick_once();
    try {
      Magick::Image mi(path);
      // Normalize non-RGB inputs to sRGB before exporting via the "RGBA" map.
      // ExportImagePixels (what write(...,"RGBA",...) calls) does NOT convert
      // colorspace — it dumps the raw channel quantums. So a CMYK/Lab/etc. image
      // (CMYK JPEG/TIFF are common in print pipelines and are advertised
      // extensions) would otherwise copy C,M,Y,K straight into R,G,B,A = garbage.
      // colorSpace(sRGB) calls TransformImageColorspace (a no-op if already that
      // space). RGB (linear) is left untouched to avoid an unwanted gamma shift.
      if (mi.colorSpace() != Magick::sRGBColorspace && mi.colorSpace() != Magick::RGBColorspace)
        mi.colorSpace(Magick::sRGBColorspace);
      int w = static_cast<int>(mi.columns());
      int h = static_cast<int>(mi.rows());
      image out(w, h, image::pixel_format::RGBA, image::data_type::u8);
      if (w > 0 && h > 0)
        mi.write(0, 0, w, h, "RGBA", Magick::CharPixel, out.data());
      return out;
    } catch (const Magick::Exception &e) {
      throw std::runtime_error(std::string("cvc::image magick read '") + path + "': " + e.what());
    }
  }

  void write(const image &img, const std::string &path, int quality) const override {
    init_magick_once();
    // Magick's CharPixel constructor expects 8-bit interleaved RGBA.
    image src = (img.format() == image::pixel_format::RGBA && img.type() == image::data_type::u8)
                    ? img
                    : img.converted(image::pixel_format::RGBA, image::data_type::u8);
    try {
      Magick::Image mi(static_cast<size_t>(src.width()), static_cast<size_t>(src.height()), "RGBA",
                       Magick::CharPixel, src.data());
      if (quality >= 0 && quality <= 100)
        mi.quality(static_cast<size_t>(quality));
      mi.write(path); // format inferred from the path extension
    } catch (const Magick::Exception &e) {
      throw std::runtime_error(std::string("cvc::image magick write '") + path + "': " + e.what());
    }
  }

private:
  std::list<std::string> _exts;
};

} // namespace
} // namespace cvc
#endif // CVC_ENABLE_IMAGEMAGICK

namespace cvc {
// Register the built-in image handlers. Called once (lazily) from image.cpp.
// When ImageMagick is disabled this registers nothing — read/write_image then
// raise "no handler" until another handler (e.g. stb) is added.
void register_default_image_handlers() {
#ifdef CVC_ENABLE_IMAGEMAGICK
  image_file_io::add(image_file_io::ptr(new magick_image_io()));
#endif
}
} // namespace cvc
