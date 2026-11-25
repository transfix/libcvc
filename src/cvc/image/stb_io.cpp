// stb_image_io — zero-dependency PNG/JPG/BMP/TGA/GIF fast-path handler,
// registered AHEAD of magick_image_io in register_default_image_handlers.
//
// Motivation: on wasm every PNG/JPG texture load through ImageMagick used to throw
// NoDecodeDelegateForThisImageFormat. That was blamed on duplicated objects in the
// static libMagickCore.a; the real cause was magick_io.cpp registering the static
// coders before MagickCore had created its registry, which left it empty (fixed
// there). stb_image is still the fast path for these formats: a single
// self-contained header that just decodes bytes -> pixels.
//
// Dispatch (see image.cpp): for_read takes the FIRST can_read match, so stb —
// registered first — wins reads for its declared extensions
// (png/jpg/jpeg/bmp/tga/gif); tif/webp/etc. fall through to magick_image_io.
// Writes prefer the LAST extension match, so magick stays the primary writer
// and write_image falls back to the stb_image_write encoders below when it
// throws — e.g. an ImageMagick built without the png/jpeg delegates, whose
// magick_image_io write path detects the missing encoder instead of silently
// emitting a MIFF blob under the requested name (see magick_io.cpp).

#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <cvc/image/image_encode.h>
#include <cvc/utility/archive.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

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

// For decode_image (image_decode.cpp): stb's implementation is in this translation unit only.
namespace detail {

bool stb_info_memory(const void *data, std::size_t size, int &w, int &h) {
  int chan = 0;
  return size <= static_cast<std::size_t>(INT_MAX) &&
         stbi_info_from_memory(static_cast<const stbi_uc *>(data), static_cast<int>(size), &w, &h,
                               &chan) != 0;
}

image stb_decode_memory(const void *data, std::size_t size, std::string *why) {
  int w = 0, h = 0, chan = 0;
  unsigned char *px = size <= static_cast<std::size_t>(INT_MAX)
                          ? stbi_load_from_memory(static_cast<const stbi_uc *>(data),
                                                  static_cast<int>(size), &w, &h, &chan, 4)
                          : nullptr;
  if (!px) {
    if (why)
      *why = stbi_failure_reason() ? stbi_failure_reason() : "corrupt image";
    return image();
  }
  image out(w, h, image::pixel_format::RGBA, image::data_type::u8);
  if (w > 0 && h > 0)
    std::memcpy(out.data(), px, static_cast<std::size_t>(w) * h * 4);
  stbi_image_free(px);
  return out;
}

// Every frame of a GIF (stb composes each onto the canvas), with its delay in ms.
bool stb_decode_gif_memory(const void *data, std::size_t size, std::vector<image> &frames,
                           std::vector<int> &delays, std::string *why) {
  int w = 0, h = 0, z = 0, comp = 0;
  int *ds = nullptr;
  unsigned char *px =
      size <= static_cast<std::size_t>(INT_MAX)
          ? stbi_load_gif_from_memory(static_cast<const stbi_uc *>(data), static_cast<int>(size),
                                      &ds, &w, &h, &z, &comp, 4)
          : nullptr;
  if (!px) {
    if (why)
      *why = stbi_failure_reason() ? stbi_failure_reason() : "corrupt GIF";
    return false;
  }
  const std::size_t one = static_cast<std::size_t>(w) * h * 4;
  for (int i = 0; i < z; ++i) {
    frames.emplace_back(w, h, image::pixel_format::RGBA, image::data_type::u8, px + one * i);
    delays.push_back(ds ? ds[i] : 100);
  }
  stbi_image_free(px);
  STBI_FREE(ds);
  return true;
}

} // namespace detail

namespace detail {
// For cvc::archive::deflate (archive.cpp): `bytes` as a zlib stream, by stb's compressor.
std::string stb_zlib_compress(std::string_view bytes, int quality) {
  int n = 0;
  unsigned char *z =
      stbi_zlib_compress(reinterpret_cast<unsigned char *>(const_cast<char *>(bytes.data())),
                         static_cast<int>(bytes.size()), &n, quality);
  if (!z)
    return std::string();
  std::string out(reinterpret_cast<const char *>(z), static_cast<std::size_t>(n));
  STBIW_FREE(z);
  return out;
}
} // namespace detail

namespace {

// A 16-bit grey PNG, which stb does not write: each row filtered Sub (a height map's neighbours are
// close, so the differences pack small), its samples big-endian, the rows one zlib stream.
std::string encode_png_gray16(const image &img) {
  const int w = img.width(), h = img.height();
  const std::size_t row = static_cast<std::size_t>(w) * 2;
  std::string raw;
  raw.reserve((row + 1) * static_cast<std::size_t>(h));
  for (int y = 0; y < h; ++y) {
    const auto *src = reinterpret_cast<const std::uint16_t *>(
        static_cast<const unsigned char *>(img.data()) +
        static_cast<std::size_t>(y) * img.row_stride_bytes());
    raw += '\x01'; // Sub
    std::uint16_t prev = 0;
    for (int x = 0; x < w; ++x) {
      // Sub works byte by byte against the byte 2 back: the high byte, then the low, each less
      // the previous sample's, with no carry between them.
      raw += static_cast<char>(((src[x] >> 8) - (prev >> 8)) & 0xFF);
      raw += static_cast<char>((src[x] - prev) & 0xFF);
      prev = src[x];
    }
  }
  const std::string z = detail::stb_zlib_compress(raw, 8);
  if (z.empty())
    throw std::runtime_error("cvc::encode_png: the encode failed");
  std::string out("\x89PNG\r\n\x1A\n", 8);
  const auto be32 = [](std::string &s, std::uint32_t v) {
    for (int k = 3; k >= 0; --k)
      s += static_cast<char>((v >> (8 * k)) & 0xFF);
  };
  const auto chunk = [&](const char *type, const std::string &data) {
    be32(out, static_cast<std::uint32_t>(data.size()));
    std::string body(type, 4);
    body += data;
    out += body;
    be32(out, cvc::archive::crc32(body));
  };
  std::string ihdr;
  be32(ihdr, static_cast<std::uint32_t>(w));
  be32(ihdr, static_cast<std::uint32_t>(h));
  ihdr += std::string("\x10\x00\x00\x00\x00", 5); // 16 bits, grey, deflate, adaptive, no interlace
  chunk("IHDR", ihdr);
  chunk("IDAT", z);
  chunk("IEND", std::string());
  return out;
}

} // namespace

std::string encode_png(const image &img) {
  if (img.width() <= 0 || img.height() <= 0)
    throw std::runtime_error("cvc::encode_png: the image is empty");
  if (img.format() == image::pixel_format::GRAY && img.type() == image::data_type::u16)
    return encode_png_gray16(img);
  const bool keep =
      img.type() == image::data_type::u8 && img.channels() >= 1 && img.channels() <= 4;
  const image src = keep ? img : img.converted(image::pixel_format::RGBA, image::data_type::u8);
  std::string out;
  auto put = [](void *ctx, void *data, int size) {
    static_cast<std::string *>(ctx)->append(static_cast<const char *>(data),
                                            static_cast<std::size_t>(size));
  };
  const int ok = stbi_write_png_to_func(put, &out, src.width(), src.height(), src.channels(),
                                        src.data(), static_cast<int>(src.row_stride_bytes()));
  if (!ok)
    throw std::runtime_error("cvc::encode_png: the encode failed");
  return out;
}

// Called from register_default_image_handlers (defined in magick_io.cpp) —
// registered BEFORE magick_image_io so image_file_io::for_read (first-wins)
// picks stb for its declared extensions. Missing formats fall through.
void register_stb_image_handler(app &ctx) {
  ctx.image_handlers().insert(image_file_io::ptr(new stb_image_io()));
}

} // namespace cvc
