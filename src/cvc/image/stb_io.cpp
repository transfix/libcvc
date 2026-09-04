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
// Dispatch (see image.cpp): for_read takes the FIRST can_read match, so stb —
// registered first — wins reads for its declared extensions
// (png/jpg/jpeg/bmp/tga/gif); tif/webp/etc. fall through to magick_image_io.
// Writes prefer the LAST extension match, so magick stays the primary writer
// and write_image falls back to the stb_image_write encoders below when it
// throws — e.g. an ImageMagick built without the png/jpeg delegates, whose
// magick_image_io write path detects the missing encoder instead of silently
// emitting a MIFF blob under the requested name (see magick_io.cpp).

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

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include "third_party/stb_image_write.h"

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

  void write(const image &img, const std::string &path, int quality) const override {
    const std::string e = image_file_extension(path);
    // GIF is decode-only in stb; throw so write_image can hand it to another
    // handler (or fail loudly) rather than emit a mis-formatted file.
    if (e != "png" && e != "jpg" && e != "jpeg" && e != "bmp" && e != "tga")
      throw std::runtime_error(std::string("cvc::image stb write '") + path +
                               "': no stb encoder for '." + e + "'");
    // stb_image_write encodes interleaved u8; convert like magick_image_io
    // does so both writers accept any cvc::image. `const` so data() takes the
    // no-detach overload (no needless buffer copy when img is already RGBA u8).
    const image src =
        (img.format() == image::pixel_format::RGBA && img.type() == image::data_type::u8)
            ? img
            : img.converted(image::pixel_format::RGBA, image::data_type::u8);
    const int w = src.width(), h = src.height();
    int ok = 0;
    if (e == "png")
      ok = stbi_write_png(path.c_str(), w, h, 4, src.data(), w * 4);
    else if (e == "jpg" || e == "jpeg") // alpha ignored by the jpeg encoder
      ok = stbi_write_jpg(path.c_str(), w, h, 4, src.data(),
                          (quality >= 1 && quality <= 100) ? quality : 90);
    else if (e == "bmp")
      ok = stbi_write_bmp(path.c_str(), w, h, 4, src.data());
    else
      ok = stbi_write_tga(path.c_str(), w, h, 4, src.data());
    if (!ok)
      throw std::runtime_error(std::string("cvc::image stb write '") + path + "': encode failed");
  }

private:
  std::list<std::string> _exts;
};

} // namespace

// Called from register_default_image_handlers (defined in magick_io.cpp) —
// registered BEFORE magick_image_io so image_file_io::for_read (first-wins)
// picks stb for its declared extensions. Missing formats fall through.
void register_stb_image_handler() {
  image_file_io::add(image_file_io::ptr(new stb_image_io()));
}

} // namespace cvc
