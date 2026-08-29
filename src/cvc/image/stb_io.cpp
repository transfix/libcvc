// stb_image_io — zero-dependency PNG/JPG/BMP/TGA/GIF fast-path handler,
// registered AHEAD of magick_image_io in register_default_image_handlers.
//
// Motivation: on wasm builds ImageMagick's libtool build ships libMagickCore.a
// with two independent compilations of magick.c/cache.c/tree.c/etc. and
// wasm-ld's satisfy-and-DCE model splits its static state — the coder
// registry the reader consults is not the one Register*Image() writes to, and
// every PNG/JPG texture load throws NoDecodeDelegateForThisImageFormat.
// stb_image is a single self-contained header; it has none of that machinery
// and just decodes bytes -> pixels.
//
// image.cpp registers handlers "last wins" (the reader iterates the registry
// and takes the LAST matching handler), so registering stb AFTER magick makes
// stb win for its declared extensions (png/jpg/jpeg/bmp/tga/gif). Formats stb
// does not cover (tif/webp/etc.) still fall through to magick_image_io.

#include <cvc/image/image.h>

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_NO_PIC
#define STBI_NO_PNM
#define STBI_NO_PSD
#include "third_party/stb_image.h"

namespace cvc {

namespace {

class stb_image_io : public image_file_io {
public:
  stb_image_io() : _exts{"png", "jpg", "jpeg", "bmp", "tga", "gif"} {}

  std::string id() const override { return "stb_image_io"; }
  const std::list<std::string> &extensions() const override { return _exts; }

  bool can_read(const std::string &path) const override {
    std::string e = image_file_extension(path);
    for (const auto &x : _exts)
      if (x == e)
        return true;
    return false;
  }

  image read(const std::string &path) const override {
    int w = 0, h = 0, chan = 0;
    // Always demand 4 channels — libcvc's image layout is interleaved RGBA
    // u8, which is what magick_image_io produces too, so downstream code
    // does not need to branch on the handler.
    unsigned char *px = stbi_load(path.c_str(), &w, &h, &chan, 4);
    if (!px)
      throw std::runtime_error(std::string("cvc::image stb read '") + path +
                               "': " + (stbi_failure_reason() ? stbi_failure_reason() : "unknown"));
    image out(w, h, image::pixel_format::RGBA, image::data_type::u8);
    if (w > 0 && h > 0)
      std::memcpy(out.data(), px, static_cast<std::size_t>(w) * h * 4);
    stbi_image_free(px);
    return out;
  }

  void write(const image & /*img*/, const std::string &path, int /*quality*/) const override {
    // stb_image is read-only in this handler; refuse writes so callers fall
    // back to whatever write handler image.cpp finds (magick_image_io on
    // native, nothing on wasm — the wasm demos never write images).
    throw std::runtime_error(std::string("cvc::image stb write '") + path +
                             "': not supported (read-only fast path)");
  }

private:
  std::list<std::string> _exts;
};

} // namespace

// Called from register_default_image_handlers (defined in magick_io.cpp) —
// registered AFTER magick_image_io so image_file_io::for_read (last-wins)
// picks stb for its declared extensions. Missing formats fall through.
void register_stb_image_handler() {
  image_file_io::add(image_file_io::ptr(new stb_image_io()));
}

} // namespace cvc
