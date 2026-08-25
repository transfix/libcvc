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

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <cstdlib>
#include <string>
#include <vector>
#include <windows.h>
#endif

namespace cvc {
namespace {

#ifdef _WIN32
// The cvcpkg ImageMagick bundle is EXTRACTED from the official installer, not
// installed, so its module + config paths are absent from the registry and
// ImageMagick reports "no decode delegate" for every read. Point it at our layout
// relative to the loaded libcvc DLL (which sits in the same <prefix>\bin as the
// ImageMagick DLLs): <prefix>\bin holds the CORE_RL_* libs AND the coder/filter
// modules FLAT — a coder resolves its own CORE_RL_* deps in its directory, so the
// modules must sit beside them — and <prefix>\share\imagemagick holds the .xml
// config. Only set what the environment has not already overridden (a user wins).
void set_magick_paths_win() {
  HMODULE h = nullptr;
  GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                         GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                     reinterpret_cast<LPCSTR>(&set_magick_paths_win), &h);
  if (!h)
    return;
  char buf[MAX_PATH];
  const DWORD n = GetModuleFileNameA(h, buf, MAX_PATH);
  if (n == 0 || n >= MAX_PATH)
    return;
  std::string p(buf, n);
  const auto slash = p.find_last_of("\\/");
  if (slash == std::string::npos)
    return;
  // Search order: DLL's own directory (normal case: <prefix>\bin), then
  // <prefix>\bin from the DLL's parent (in case cvc.dll got copied to
  // build\bin\Release\ for CTest while the real coders live in deps\bin), then
  // any CVC_DEPS_PREFIX\bin the environment names. Pick the first that contains
  // at least one IM_MOD_RL_*.dll — an empty dir here becomes a MIFF-only Magick.
  const std::string dllDir = p.substr(0, slash);
  auto has_coder = [](const std::string &dir) -> bool {
    WIN32_FIND_DATAA d;
    HANDLE h = FindFirstFileA((dir + "\\IM_MOD_RL_*.dll").c_str(), &d);
    if (h == INVALID_HANDLE_VALUE)
      return false;
    FindClose(h);
    return true;
  };
  std::vector<std::string> cand{dllDir};
  // If dllDir is <something>\build\bin\Release, walk up to try <something>\bin.
  cand.push_back(dllDir + "\\..\\..\\..\\..\\deps\\bin");
  if (const char *dp = std::getenv("CVC_DEPS_PREFIX"))
    cand.push_back(std::string(dp) + "\\bin");
  std::string bindir = dllDir; // fallback
  for (const auto &c : cand)
    if (has_coder(c)) {
      bindir = c;
      break;
    }
  const std::string share = bindir + "\\..\\share\\imagemagick";
  auto set_if_absent = [](const char *k, const std::string &v) {
    if (!std::getenv(k))
      _putenv_s(k, v.c_str());
  };
  set_if_absent("MAGICK_CONFIGURE_PATH", share);
  set_if_absent("MAGICK_CODER_MODULE_PATH", bindir);
  set_if_absent("MAGICK_FILTER_MODULE_PATH", bindir);
}
#endif

void init_magick_once() {
  static std::once_flag once;
  std::call_once(once, []() {
#ifdef _WIN32
    set_magick_paths_win();
#endif
    Magick::InitializeMagick(nullptr);
  });
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
