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
// Q16-HDRI internal quantum, invisible to callers. The zero-dependency stb
// handler (stb_io.cpp) sits AHEAD of it for png/jpg/bmp/tga/gif reads and
// BEHIND it as the write fallback when this Magick lacks the target encoder.

#include <cvc/image/image.h>

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <list>
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
// ImageMagick reports "no decode delegate" for every read, after a
// RegistryKeyLookupFailed `CoderModulesPath'. Point it at the bundle's layout:
// <prefix>\bin holds the CORE_RL_* libs AND the coder/filter modules FLAT — a
// coder resolves its own CORE_RL_* deps in its directory, so the modules must
// sit beside them — and <prefix>\share\imagemagick holds the .xml config. Only
// set what the environment has not already overridden (a user wins).
//
// Setting the variables from inside the process is enough: MagickCore reads
// them with GetEnvironmentVariableW, from the OS environment block, each time it
// opens a module, and _putenv_s writes that block as well as the CRT's copy.
// What failed before was Magick being reached without this running at all, or
// this running without finding the bundle; magick_bootstrap_test checks the
// lookup with none of the variables set.

// The directory a loaded module's file is in; empty if it cannot be found.
std::string module_dir(HMODULE h) {
  if (!h)
    return std::string();
  char buf[MAX_PATH];
  const DWORD n = GetModuleFileNameA(h, buf, MAX_PATH);
  if (n == 0 || n >= MAX_PATH)
    return std::string();
  std::string p(buf, n);
  const auto slash = p.find_last_of("\\/");
  return slash == std::string::npos ? std::string() : p.substr(0, slash);
}

// The module holding `addr`.
HMODULE module_at(const void *addr) {
  HMODULE h = nullptr;
  GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                         GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                     reinterpret_cast<LPCSTR>(addr), &h);
  return h;
}

// The directory `dir` is in; empty above a drive root.
std::string parent_dir(const std::string &dir) {
  const auto slash = dir.find_last_of("\\/");
  return slash == std::string::npos ? std::string() : dir.substr(0, slash);
}

// Whether `dir` holds a file matching `pattern`.
bool has_file(const std::string &dir, const char *pattern) {
  WIN32_FIND_DATAA d;
  HANDLE h = FindFirstFileA((dir + "\\" + pattern).c_str(), &d);
  if (h == INVALID_HANDLE_VALUE)
    return false;
  FindClose(h);
  return true;
}

void set_magick_paths_win() {
  const std::string dllDir =
      module_dir(module_at(reinterpret_cast<const void *>(&set_magick_paths_win)));
  // Where the coders may be, in order; the first directory holding at least one
  // IM_MOD_RL_*.dll wins.
  std::vector<std::string> cand;
  // 1. Beside libcvc: an installed prefix keeps cvc.dll in <prefix>\bin too.
  if (!dllDir.empty())
    cand.push_back(dllDir);
  // 2. Beside the ImageMagick DLL actually loaded, whose coders they must be: a
  //    dev build's cvc.dll is in build\bin while ImageMagick comes from the deps
  //    prefix on PATH. By its name in the cvcpkg bundle (the official Windows
  //    build's), then by the address of one of its functions, for other builds.
  //    When that address is an import thunk in cvc.dll it names dllDir again.
  for (const std::string &d :
       {module_dir(GetModuleHandleA("CORE_RL_MagickCore_.dll")),
        module_dir(module_at(reinterpret_cast<const void *>(&MagickCore::GetMagickVersion)))})
    if (!d.empty())
      cand.push_back(d);
  // 3. A deps prefix the environment names.
  for (const char *var : {"CVC_DEPS_PREFIX", "LIBCVC_DEPS_PREFIX", "CVCPKG_PREFIX"}) {
    const char *dp = std::getenv(var);
    if (dp && *dp)
      cand.push_back(std::string(dp) + "\\bin");
  }
  // 4. A build tree beside its deps prefix: <root>\deps next to
  //    <root>\build-X\bin\cvc.dll, or <root>\build-X\bin\Release\cvc.dll.
  std::string up = parent_dir(dllDir);
  for (int i = 0; i < 4 && !up.empty(); ++i, up = parent_dir(up))
    cand.push_back(up + "\\deps\\bin");
  std::string bindir;
  for (const auto &c : cand)
    if (has_file(c, "IM_MOD_RL_*.dll")) {
      bindir = c;
      break;
    }
  // None: leave the environment alone, so an ImageMagick that IS installed still
  // finds its modules through the registry.
  if (bindir.empty())
    return;
  // Test the OS environment block, the one MagickCore reads.
  auto set_if_absent = [](const char *k, const std::string &v) {
    if (GetEnvironmentVariableA(k, nullptr, 0) == 0)
      _putenv_s(k, v.c_str());
  };
  set_if_absent("MAGICK_CODER_MODULE_PATH", bindir);
  set_if_absent("MAGICK_FILTER_MODULE_PATH", bindir);
  // The config: <prefix>\share\imagemagick, for <prefix>\bin and <prefix>\debug\bin.
  const std::string prefix = parent_dir(bindir);
  for (const std::string &share :
       {prefix + "\\share\\imagemagick", parent_dir(prefix) + "\\share\\imagemagick"})
    if (has_file(share, "*.xml")) {
      set_if_absent("MAGICK_CONFIGURE_PATH", share);
      break;
    }
}
#endif

// ── wasm-only coder bootstrap ────────────────────────────────────────────
//
// Emscripten links a STATIC ImageMagick whose coders are .o members of
// libMagickCore. RegisterStaticModules() is only reached from inside
// MagickCore/magick.c's GetMagickInfo("*", ...) path — Magick::InitializeMagick
// alone doesn't force it — so a caller asking specifically for "png" hits an
// empty coder registry and dies with "NoDecodeDelegateForThisImageFormat PNG".
// Every embedded glTF texture load on the wasm demo tripped this.
//
// This block is __EMSCRIPTEN__-only ON PURPOSE. It was originally applied to
// every platform (#262 for RegisterStaticModules, #263 for the per-coder
// entrypoints) and broke all three native platforms:
//
//   * linux/macos link a DYNAMIC-MODULES ImageMagick, where the coders are
//     already registered by Genesis. Calling the Register* entrypoints again
//     duplicates those registry entries, and IM then resolves neither the
//     extension nor its own MIFF fallback — ImageTest.PngRoundTrip and
//     JpegRoundTripDimensions failed with
//     "NoEncodeDelegateForThisImageFormat `MIFF'". (#268's commit message
//     already fingered "duplicate stubs" as the root cause.)
//   * windows links against the IM DLL's import library, which does not export
//     the per-coder Register*Image symbols at all, so cvc.dll failed to link
//     with 7x LNK2001 + LNK1120.
//
// Guarding on __EMSCRIPTEN__ keeps the wasm fix — the only build that needs
// it — and leaves every native build on the coder registry Genesis populates.
#ifdef __EMSCRIPTEN__
extern "C" void RegisterStaticModules(void);
// Also declare the individual coder Register* entrypoints — under
// -Wl,--gc-sections + wasm-ld's whole-program DCE, calling only
// RegisterStaticModules() sometimes still lets the linker drop png.o
// (its call site inside static.o is behind a big function-pointer table
// that DCE misreads as unreachable). Referencing each Register* here
// via a volatile pointer forces the linker to keep the coder .o for
// each format the demos need.
extern "C" std::size_t RegisterPNGImage(void);
extern "C" std::size_t RegisterJPEGImage(void);
extern "C" std::size_t RegisterTIFFImage(void);
extern "C" std::size_t RegisterWEBPImage(void);
extern "C" std::size_t RegisterBMPImage(void);
extern "C" std::size_t RegisterGIFImage(void);
extern "C" std::size_t RegisterMIFFImage(void);
#endif // __EMSCRIPTEN__

void init_magick_once() {
  static std::once_flag once;
  std::call_once(once, []() {
#ifdef _WIN32
    set_magick_paths_win();
#endif
    Magick::InitializeMagick(nullptr);
#ifdef __EMSCRIPTEN__
    // wasm only — see the block above. On a native build Genesis has already
    // populated the coder registry, and re-registering duplicates its entries.
    //
    // Create the registry first. MagickCore builds it lazily, on the first
    // GetMagickInfo, and until then RegisterMagickInfo silently drops every entry,
    // while RegisterStaticModules still marks each module registered, so the lazy
    // path never retries. Registering straight after InitializeMagick therefore left
    // the registry empty for good: every Magick read and write failed with "no
    // encoder"/NoDecodeDelegate. GetMagickInfo("*") instantiates it and registers
    // every static coder into it; the calls below then only replace those entries.
    {
      MagickCore::ExceptionInfo *ex = MagickCore::AcquireExceptionInfo();
      (void)MagickCore::GetMagickInfo("*", ex);
      ex = MagickCore::DestroyExceptionInfo(ex);
    }
    RegisterStaticModules();
    // Force-observe each Register* return value through a volatile sink so
    // wasm-ld / wasm-opt cannot elide the call (plain "(void)RegisterPNGImage()"
    // was being DCEd in the wasm build).
    using RegFn = std::size_t (*)(void);
    static volatile RegFn keep[] = {
        &RegisterPNGImage, &RegisterJPEGImage, &RegisterTIFFImage, &RegisterWEBPImage,
        &RegisterBMPImage, &RegisterGIFImage,  &RegisterMIFFImage,
    };
    volatile std::size_t magick_reg_sink = 0;
    for (auto &fn : keep)
      magick_reg_sink += fn();
    (void)magick_reg_sink;
#endif // __EMSCRIPTEN__
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
    // Resolve the target coder from the extension UP FRONT and refuse when this
    // ImageMagick cannot encode it. Image::write(filename) does NOT fail on a
    // missing encoder (seen with a deps-prefix ImageMagick built without the
    // png/jpeg delegates): the format stays unresolved and WriteImage silently
    // falls back to Magick's native MIFF, so save("out.png") wrote a MIFF blob
    // ("id=ImageMagick...") under the .png name. Throwing here instead lets
    // write_image fall back to the stb encoder for png/jpg/bmp/tga.
    std::string fmt = image_file_extension(path);
    std::transform(fmt.begin(), fmt.end(), fmt.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    bool writable = false;
    try {
      // CoderInfo throws on a coder Magick does not know at all (never built,
      // or its module failed to load); isWritable() is false for decode-only
      // coders. Either way the encoder is unusable.
      writable = !fmt.empty() && Magick::CoderInfo(fmt).isWritable();
    } catch (const Magick::Exception &) {
      writable = false;
    }
    if (!writable)
      throw std::runtime_error("cvc::image magick write '" + path + "': this ImageMagick has no '" +
                               fmt + "' encoder (missing delegate?)");
    // Magick's CharPixel constructor expects 8-bit interleaved RGBA.
    image src = (img.format() == image::pixel_format::RGBA && img.type() == image::data_type::u8)
                    ? img
                    : img.converted(image::pixel_format::RGBA, image::data_type::u8);
    try {
      Magick::Image mi(static_cast<size_t>(src.width()), static_cast<size_t>(src.height()), "RGBA",
                       Magick::CharPixel, src.data());
      if (quality >= 0 && quality <= 100)
        mi.quality(static_cast<size_t>(quality));
      mi.magick(fmt); // explicit format — never let write() re-infer (and miss)
      mi.write(path);
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
void init_imagemagick() {
#ifdef CVC_ENABLE_IMAGEMAGICK
  init_magick_once();
#endif
}

void register_stb_image_handler(); // stb_io.cpp — no dependencies at all.

// Register the built-in image handlers. Called once (lazily) from image.cpp.
// Order matters:
//   for_read walks the registry and takes the FIRST can_read match, so stb
//     must be added FIRST to beat magick on png/jpg/bmp/tga/gif reads (the
//     zero-dependency fast path — see src/cvc/image/stb_io.cpp).
//   Writes prefer the LAST extension match (for_write_ext and write_image's
//     first candidate), so magick must be added LAST — it covers every
//     extension and honors quality/format options stb does not. write_image
//     falls back to stb's png/jpg/bmp/tga encoders when magick throws (e.g.
//     an ImageMagick built without the target format's delegate).
void register_default_image_handlers() {
  register_stb_image_handler();
#ifdef CVC_ENABLE_IMAGEMAGICK
  image_file_io::add(image_file_io::ptr(new magick_image_io()));
#endif
}
} // namespace cvc
