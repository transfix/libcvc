/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// image_magick_roundtrip_test: ImageMagick at run time in a wasm build (ci.yml's ari-wasm job
// runs it under node).
//
// A wasm build links a static ImageMagick and registers its coders itself (init_magick_once in
// src/cvc/image/magick_io.cpp). Registering them before MagickCore had created its coder
// registry left the registry empty: every Magick read and write failed at run time while the
// link and every native test stayed green. This checks the coders behind each extension
// magick_image_io advertises, then round-trips an image through TIFF and WEBP, which the stb
// handler can neither read nor write, so both directions go through Magick and its codec
// libraries (libtiff, libwebp).
//
// A plain main(), not googletest: googletest has no wasm build. Natively, image_test covers the
// same round trips. Exit codes: 0 when every check passes, 1 on a failure, 77 (ctest's skip)
// in a build without ImageMagick.

#include <cstdio>
#include <cstring>
#include <cvc/image/image.h>
#include <exception>
#include <string>

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h>
#endif

#ifdef CVC_ENABLE_IMAGEMAGICK
namespace {

int failures = 0;

void fail(const std::string &what) {
  std::printf("FAIL: %s\n", what.c_str());
  ++failures;
}

// A w x h RGBA u8 image whose every byte differs from its neighbours, so a channel swap, a
// row flip or a lost alpha all show up as a byte mismatch.
cvc::image pattern(int w, int h) {
  cvc::image img(w, h, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = img.data();
  for (int i = 0; i < w * h * 4; ++i)
    p[i] = static_cast<unsigned char>((i * 37 + 11) & 0xff);
  return img;
}

// Write src to path and read it back as RGBA u8. exact: the bytes must match (lossless formats).
void roundtrip(const cvc::image &src, const std::string &path, bool exact) {
  try {
    cvc::write_image(src, path);
    cvc::image back =
        cvc::read_image(path).converted(cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    if (back.width() != src.width() || back.height() != src.height()) {
      fail(path + ": read back " + std::to_string(back.width()) + "x" +
           std::to_string(back.height()));
      return;
    }
    if (exact && std::memcmp(back.data(), src.data(),
                             static_cast<size_t>(src.width()) * src.height() * 4) != 0) {
      fail(path + ": pixels differ after the round trip");
      return;
    }
    std::printf("ok: %s round trip (%dx%d RGBA%s)\n", path.c_str(), src.width(), src.height(),
                exact ? ", byte-identical" : "");
  } catch (const std::exception &e) {
    fail(path + ": " + e.what());
  }
}

} // namespace
#endif

int main() {
#ifndef CVC_ENABLE_IMAGEMAGICK
  std::printf("SKIP: this build has no ImageMagick\n");
  return 77;
#else
  cvc::init_imagemagick();

  // Every extension magick_image_io claims needs a coder that both decodes and encodes.
  for (const char *fmt : {"PNG", "JPG", "JPEG", "TIF", "TIFF", "BMP", "TGA", "GIF", "WEBP"}) {
    try {
      Magick::CoderInfo info(fmt);
      if (!info.isReadable() || !info.isWritable())
        fail(std::string("coder ") + fmt + (info.isReadable() ? "" : " cannot decode") +
             (info.isWritable() ? "" : " cannot encode"));
    } catch (const std::exception &e) {
      fail(std::string("coder ") + fmt + " is not registered (" + e.what() + ")");
    }
  }

  const cvc::image src = pattern(7, 5);
  roundtrip(src, "/tmp/cvc_magick_roundtrip.tif", true);
  roundtrip(src, "/tmp/cvc_magick_roundtrip.webp", false); // lossy by default: size only

  if (failures) {
    std::printf("image_magick_roundtrip_test: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("image_magick_roundtrip_test: ImageMagick works\n");
  return 0;
#endif
}
