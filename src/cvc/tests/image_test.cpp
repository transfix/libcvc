// Tests for cvc::image (Phase-1 image surface): the value type + copy-on-write,
// manipulation (flip/convert/resize), the image_file_io registry, and the
// ImageMagick round-trip (guarded on CVC_ENABLE_IMAGEMAGICK).

#include <cstdint>
#include <cstdio>
#include <cvc/image/image.h>
#include <cvc/image/image.h> // include-twice: header must be idempotent
#include <fstream>
#include <gtest/gtest.h>
#include <list>
#include <stdexcept>
#include <string>

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h> // for the CMYK-input regression test (synthesizes a CMYK file)
#endif

using cvc::image;

namespace {

image make_rgba(int w, int h) {
  image im(w, h, image::pixel_format::RGBA, image::data_type::u8);
  unsigned char *p = im.data();
  for (int i = 0; i < w * h; ++i) {
    p[i * 4 + 0] = static_cast<unsigned char>((i * 7) & 0xff);
    p[i * 4 + 1] = static_cast<unsigned char>((i * 13) & 0xff);
    p[i * 4 + 2] = static_cast<unsigned char>((i * 29) & 0xff);
    p[i * 4 + 3] = 255;
  }
  return im;
}

} // namespace

TEST(ImageTest, DefaultIsEmpty) {
  image im;
  EXPECT_TRUE(im.empty());
  EXPECT_EQ(im.width(), 0);
  EXPECT_EQ(im.height(), 0);
  EXPECT_EQ(im.size_bytes(), 0u);
}

TEST(ImageTest, DimensionsAndLayout) {
  image im(8, 4, image::pixel_format::RGB, image::data_type::u8);
  EXPECT_FALSE(im.empty());
  EXPECT_EQ(im.width(), 8);
  EXPECT_EQ(im.height(), 4);
  EXPECT_EQ(im.channels(), 3);
  EXPECT_EQ(im.bytes_per_channel(), 1u);
  EXPECT_EQ(im.bytes_per_pixel(), 3u);
  EXPECT_EQ(im.row_stride_bytes(), 8u * 3u);
  EXPECT_EQ(im.size_bytes(), 8u * 4u * 3u);
}

TEST(ImageTest, ChannelsPerFormat) {
  EXPECT_EQ(image(1, 1, image::pixel_format::GRAY).channels(), 1);
  EXPECT_EQ(image(1, 1, image::pixel_format::GRAY_ALPHA).channels(), 2);
  EXPECT_EQ(image(1, 1, image::pixel_format::RGB).channels(), 3);
  EXPECT_EQ(image(1, 1, image::pixel_format::RGBA).channels(), 4);
}

TEST(ImageTest, BytesPerChannelPerType) {
  EXPECT_EQ(image(1, 1, image::pixel_format::GRAY, image::data_type::u8).bytes_per_channel(), 1u);
  EXPECT_EQ(image(1, 1, image::pixel_format::GRAY, image::data_type::u16).bytes_per_channel(), 2u);
  EXPECT_EQ(image(1, 1, image::pixel_format::GRAY, image::data_type::f32).bytes_per_channel(), 4u);
}

TEST(ImageTest, ConstructZeroInitialized) {
  image im(2, 2, image::pixel_format::RGBA, image::data_type::u8);
  const unsigned char *p = im.data();
  for (std::size_t i = 0; i < im.size_bytes(); ++i)
    EXPECT_EQ(p[i], 0) << "byte " << i;
}

TEST(ImageTest, ConstructFromSourceCopies) {
  unsigned char src[2 * 1 * 4] = {1, 2, 3, 4, 5, 6, 7, 8};
  image im(2, 1, image::pixel_format::RGBA, image::data_type::u8, src);
  const unsigned char *p = im.data();
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(p[i], src[i]);
  src[0] = 99; // mutating the source must not touch the image (it copied)
  EXPECT_EQ(im.data()[0], 1);
}

TEST(ImageTest, CopyOnWrite) {
  image a = make_rgba(4, 4);
  image b = a; // shares the buffer
  // The copy really does share: check via storage(), which does NOT detach.
  // (Checking with data() cannot work — the first data() call detaches, so it
  // would destroy the very aliasing it is trying to observe.)
  EXPECT_EQ(a.storage().get(), b.storage().get());
  unsigned char before = a.data()[0];
  // mutate b through data() -> detach; a must be untouched
  b.data()[0] = static_cast<unsigned char>(before ^ 0xff);
  EXPECT_EQ(a.data()[0], before);
  EXPECT_NE(b.data()[0], before);

  // storage() shares without detaching — but this MUST be sequenced, not
  // written as EXPECT_EQ(a.storage().get(), a.data()).
  //
  // storage() returns the shared_array BY VALUE, so the returned temporary
  // holds a reference and pushes use_count() to 2; data() -> detach() copies
  // whenever use_count() > 1. Argument evaluation order is UNSPECIFIED in C++,
  // so in one full-expression the two calls race:
  //   data() first    -> no extra ref, no detach       -> pointers equal
  //   storage() first -> its temporary is still alive  -> data() DETACHES
  // gcc happened to pick the first order and clang the second, so this passed
  // on linux and failed on macOS for as long as macOS got far enough to run
  // tests. Reproduced locally: same expression, g++ PASS / clang++ FAIL.
  //
  // Detaching while a storage() handle is outstanding is CORRECT — that handle
  // may be a live zero-copy view (pycvc numpy, cvcGL vtkTexture), and a
  // mutable data() must not write through it. The implementation is fine; only
  // the test's sequencing was wrong.
  unsigned char *p = a.data(); // detach (if still shared) happens here
  EXPECT_EQ(a.storage().get(), p);
}

// Binding contract for the zero-copy pycvc image.numpy() view + the cvcGL
// zero-copy setTexture: storage() shares the buffer WITHOUT a copy-on-write
// detach, so two copies (or a numpy view + a vtkTexture) alias the same bytes;
// an edit through storage() is visible to all, until someone data()-detaches.
TEST(ImageTest, StorageAliasesAcrossCopyUntilDetach) {
  image a = make_rgba(4, 4);
  image b = a; // shares the buffer
  // storage() does not detach: a and b alias the SAME block.
  EXPECT_EQ(a.storage().get(), b.storage().get());
  ASSERT_NE(a.storage().get(), nullptr);
  // A raw write through storage() (no COW) is seen by both — the zero-copy
  // live-edit path (numpy view edit reaches the aliased vtkTexture buffer).
  a.storage().get()[0] = 123;
  EXPECT_EQ(b.storage().get()[0], 123);
  // data() detaches the shared image to a private buffer; b keeps the old one.
  unsigned char *pa = a.data();
  EXPECT_NE(a.storage().get(), b.storage().get());
  pa[0] = 200;
  EXPECT_EQ(a.storage().get()[0], 200);
  EXPECT_EQ(b.storage().get()[0], 123); // b decoupled, unaffected
}

TEST(ImageTest, FlippedVertical) {
  image a = make_rgba(2, 3);
  image f = a.flipped_vertical();
  ASSERT_EQ(f.width(), 2);
  ASSERT_EQ(f.height(), 3);
  const std::size_t stride = a.row_stride_bytes();
  const unsigned char *pa = a.data();
  const unsigned char *pf = f.data();
  for (int y = 0; y < 3; ++y)
    for (std::size_t b = 0; b < stride; ++b)
      EXPECT_EQ(pf[y * stride + b], pa[(2 - y) * stride + b]);
  // flipping twice is the identity
  image ff = f.flipped_vertical();
  for (std::size_t b = 0; b < a.size_bytes(); ++b)
    EXPECT_EQ(ff.data()[b], pa[b]);
}

TEST(ImageTest, ConvertRgbaToRgbDropsAlpha) {
  image a = make_rgba(3, 2);
  image rgb = a.converted(image::pixel_format::RGB, image::data_type::u8);
  ASSERT_EQ(rgb.channels(), 3);
  ASSERT_EQ(rgb.width(), 3);
  ASSERT_EQ(rgb.height(), 2);
  const unsigned char *pa = a.data();
  const unsigned char *pr = rgb.data();
  for (int i = 0; i < 3 * 2; ++i) {
    EXPECT_EQ(pr[i * 3 + 0], pa[i * 4 + 0]);
    EXPECT_EQ(pr[i * 3 + 1], pa[i * 4 + 1]);
    EXPECT_EQ(pr[i * 3 + 2], pa[i * 4 + 2]);
  }
}

TEST(ImageTest, ConvertToGrayLuminance) {
  unsigned char src[4] = {255, 0, 0, 255}; // one red RGBA pixel
  image a(1, 1, image::pixel_format::RGBA, image::data_type::u8, src);
  image g = a.converted(image::pixel_format::GRAY, image::data_type::u8);
  ASSERT_EQ(g.channels(), 1);
  // Rec.601-ish red weight ~0.30 -> ~76
  EXPECT_GT(g.data()[0], 60);
  EXPECT_LT(g.data()[0], 90);
}

TEST(ImageTest, ConvertSameIsCopy) {
  image a = make_rgba(2, 2);
  image b = a.converted(image::pixel_format::RGBA, image::data_type::u8);
  EXPECT_EQ(b.channels(), 4);
  for (std::size_t i = 0; i < a.size_bytes(); ++i)
    EXPECT_EQ(b.data()[i], a.data()[i]);
}

TEST(ImageTest, Resized) {
  image a = make_rgba(4, 2);
  image up = a.resized(8, 4);
  EXPECT_EQ(up.width(), 8);
  EXPECT_EQ(up.height(), 4);
  EXPECT_EQ(up.channels(), 4);
  image down = a.resized(2, 1);
  EXPECT_EQ(down.width(), 2);
  EXPECT_EQ(down.height(), 1);
}

TEST(ImageTest, ExtensionHelper) {
  EXPECT_EQ(cvc::image_file_extension("/a/b/c.PNG"), "png");
  EXPECT_EQ(cvc::image_file_extension("foo.tar.jpg"), "jpg");
  EXPECT_EQ(cvc::image_file_extension("noext"), "");
  EXPECT_EQ(cvc::image_file_extension("/a.dir/file"), "");
}

// ── write dispatch: fallback on handler failure ──────────────────────────────
//
// The real-world trigger is untestable here (an ImageMagick built without the
// png/jpeg delegates makes magick_image_io's write throw its missing-encoder
// error), so these pin the dispatch contract with synthetic handlers on
// made-up extensions: write_image must try the preferred (last-registered)
// handler first, fall back to earlier registrations when it throws, and
// aggregate every failure when none succeeds. Registrations are process-global
// and append-only, hence the unique extensions per test.

namespace {

class fake_write_io : public cvc::image_file_io {
public:
  fake_write_io(std::string id, std::string ext, bool fail)
      : _id(std::move(id)), _exts{std::move(ext)}, _fail(fail) {}
  std::string id() const override { return _id; }
  const std::list<std::string> &extensions() const override { return _exts; }
  bool can_read(const std::string &) const override { return false; }
  image read(const std::string &) const override { throw std::runtime_error(_id + ": no read"); }
  void write(const image &, const std::string &, int) const override {
    ++writes;
    if (_fail)
      throw std::runtime_error(_id + ": simulated missing encoder");
  }
  mutable int writes = 0;

private:
  std::string _id;
  std::list<std::string> _exts;
  bool _fail;
};

} // namespace

TEST(ImageTest, WriteDispatchFallsBackWhenPreferredHandlerThrows) {
  boost::shared_ptr<fake_write_io> fallback(new fake_write_io("fallback_io", "cvcfbk1", false));
  boost::shared_ptr<fake_write_io> preferred(new fake_write_io("preferred_io", "cvcfbk1", true));
  cvc::image_file_io::add(fallback);
  cvc::image_file_io::add(preferred); // registered last -> tried first
  // for_write_ext still reports only the last-registered handler...
  auto h = cvc::image_file_io::for_write_ext("cvcfbk1");
  ASSERT_TRUE(h);
  EXPECT_EQ(h->id(), "preferred_io");
  // ...but when that one throws, write_image falls back to the earlier one.
  cvc::write_image(make_rgba(2, 2), "ignored.cvcfbk1");
  EXPECT_EQ(preferred->writes, 1);
  EXPECT_EQ(fallback->writes, 1);
}

TEST(ImageTest, WriteDispatchReportsEveryFailure) {
  boost::shared_ptr<fake_write_io> a(new fake_write_io("fail_a_io", "cvcfbk2", true));
  boost::shared_ptr<fake_write_io> b(new fake_write_io("fail_b_io", "cvcfbk2", true));
  cvc::image_file_io::add(a);
  cvc::image_file_io::add(b);
  try {
    cvc::write_image(make_rgba(2, 2), "ignored.cvcfbk2");
    FAIL() << "expected write_image to throw when every handler fails";
  } catch (const std::exception &e) {
    const std::string msg = e.what();
    EXPECT_NE(msg.find("fail_a_io"), std::string::npos) << msg;
    EXPECT_NE(msg.find("fail_b_io"), std::string::npos) << msg;
  }
  EXPECT_EQ(a->writes, 1);
  EXPECT_EQ(b->writes, 1);
}

TEST(ImageTest, WriteDispatchUnknownExtensionThrows) {
  EXPECT_ANY_THROW(cvc::write_image(make_rgba(2, 2), "x.nosuchext"));
}

// ── stb encoders (the write fallback) ────────────────────────────────────────
//
// stb is the handler write_image falls back to when magick refuses, so its
// encoders must round-trip on their own. for_read is first-match and stb
// registers first, so this also pins the read-dispatch order.

TEST(ImageTest, StbHandlerEncodesPngRoundTrip) {
  auto h = cvc::image_file_io::for_read("x.png");
  ASSERT_TRUE(h);
  ASSERT_EQ(h->id(), "stb_image_io");
  image a = make_rgba(5, 3);
  const std::string path = std::string(::testing::TempDir()) + "/cvc_image_stb.png";
  h->write(a, path, 90);
  image b = h->read(path);
  ASSERT_EQ(b.width(), 5);
  ASSERT_EQ(b.height(), 3);
  ASSERT_EQ(b.channels(), 4);
  for (std::size_t i = 0; i < a.size_bytes(); ++i)
    EXPECT_EQ(b.data()[i], a.data()[i]) << "byte " << i;
}

TEST(ImageTest, StbHandlerEncodesJpegDimensions) {
  auto h = cvc::image_file_io::for_read("x.jpg");
  ASSERT_TRUE(h);
  ASSERT_EQ(h->id(), "stb_image_io");
  image a = make_rgba(6, 4);
  const std::string path = std::string(::testing::TempDir()) + "/cvc_image_stb.jpg";
  h->write(a, path, 90);
  image b = h->read(path);
  EXPECT_EQ(b.width(), 6);
  EXPECT_EQ(b.height(), 4);
}

// bmp and tga are the other two stb encoders the fallback can land on; both are
// lossless for RGBA u8 (stb gives a 4-channel bmp the V4 header with an alpha
// mask, and tga 32bpp carries alpha), so assert exact round-trips.
TEST(ImageTest, StbHandlerEncodesLosslessBmpAndTga) {
  const image a = make_rgba(5, 3);
  for (const char *ext : {"bmp", "tga"}) {
    const std::string path =
        std::string(::testing::TempDir()) + "/cvc_image_stb_lossless." + std::string(ext);
    auto h = cvc::image_file_io::for_read(std::string("x.") + ext);
    ASSERT_TRUE(h) << ext;
    ASSERT_EQ(h->id(), "stb_image_io") << ext;
    h->write(a, path, 90);
    image b = h->read(path);
    ASSERT_EQ(b.width(), 5) << ext;
    ASSERT_EQ(b.height(), 3) << ext;
    for (std::size_t i = 0; i < a.size_bytes(); ++i)
      ASSERT_EQ(b.data()[i], a.data()[i]) << ext << " byte " << i;
  }
}

TEST(ImageTest, StbHandlerRefusesGifWrite) {
  auto h = cvc::image_file_io::for_read("x.gif");
  ASSERT_TRUE(h);
  ASSERT_EQ(h->id(), "stb_image_io");
  EXPECT_ANY_THROW(
      h->write(make_rgba(2, 2), std::string(::testing::TempDir()) + "/cvc_image_stb.gif", 90));
}

// Regression for the silent-MIFF corruption: with an ImageMagick built without
// the png delegate, Magick::Image::write("out.png") did not throw — it fell
// back to its native MIFF format and wrote "id=ImageMagick..." bytes under the
// .png name. Whichever handler ends up encoding (magick, or stb after the
// fallback), the bytes on disk must be real PNG.
TEST(ImageTest, SavedPngHasPngMagicNotMiff) {
  image a = make_rgba(4, 4);
  const std::string path = std::string(::testing::TempDir()) + "/cvc_image_magic.png";
  a.save(path);
  std::ifstream f(path, std::ios::binary);
  ASSERT_TRUE(f.good());
  unsigned char sig[8] = {0};
  f.read(reinterpret_cast<char *>(sig), 8);
  ASSERT_EQ(f.gcount(), 8);
  const unsigned char png_sig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(sig[i], png_sig[i]) << "byte " << i << " — MIFF starts with \"id=ImageMagick\"";
}

// Generalizes SavedPngHasPngMagicNotMiff across every registered extension —
// including tif/webp, which have no stb fallback, and the TIF/JPG coder
// aliases. For each one, save() must either succeed with a file that is NOT a
// MIFF blob, or throw. Silently writing MIFF under another name is the bug.
// Guards the extension list too: adding an extension whose uppercase form is
// not a coder Magick knows would surface here rather than in a user's file.
TEST(ImageTest, NoRegisteredExtensionSilentlyWritesMiff) {
  const image a = make_rgba(4, 4);
  for (const std::string &ext : cvc::image_file_io::known_extensions()) {
    const std::string path = std::string(::testing::TempDir()) + "/cvc_image_notmiff." + ext;
    std::remove(path.c_str());
    try {
      a.save(path);
    } catch (const std::exception &) {
      continue; // no encoder for this format in this build — the honest outcome
    }
    std::ifstream f(path, std::ios::binary);
    ASSERT_TRUE(f.good()) << ext << ": save() reported success but wrote no file";
    char head[14] = {0};
    f.read(head, sizeof(head));
    EXPECT_NE(std::string(head, static_cast<std::size_t>(f.gcount())).find("id=ImageMagick"), 0u)
        << ext << ": wrote a MIFF blob under ." << ext;
  }
}

#ifdef CVC_ENABLE_IMAGEMAGICK
TEST(ImageTest, RegistryHasHandlers) {
  auto exts = cvc::image_file_io::known_extensions();
  EXPECT_FALSE(exts.empty());
  bool has_png = false;
  for (const auto &e : exts)
    if (e == "png")
      has_png = true;
  EXPECT_TRUE(has_png);
}

TEST(ImageTest, PngRoundTrip) {
  image a = make_rgba(5, 3);
  std::string path = std::string(::testing::TempDir()) + "/cvc_image_test.png";
  a.save(path);
  image b = image::load(path);
  ASSERT_EQ(b.width(), 5);
  ASSERT_EQ(b.height(), 3);
  ASSERT_EQ(b.channels(), 4);
  // PNG is lossless -> exact pixel match
  for (std::size_t i = 0; i < a.size_bytes(); ++i)
    EXPECT_EQ(b.data()[i], a.data()[i]) << "byte " << i;
}

TEST(ImageTest, JpegRoundTripDimensions) {
  image a = make_rgba(6, 4);
  std::string path = std::string(::testing::TempDir()) + "/cvc_image_test.jpg";
  a.save(path, 90); // lossy: only check it decodes back to the same shape
  image b = image::load(path);
  EXPECT_EQ(b.width(), 6);
  EXPECT_EQ(b.height(), 4);
  EXPECT_FALSE(b.empty());
}

TEST(ImageTest, ReadMissingFileThrows) { EXPECT_ANY_THROW(image::load("/no/such/dir/nope.png")); }

// Regression: a CMYK-encoded input (common in print/Adobe pipelines; jpg/tif are
// advertised extensions) must be colorspace-converted to RGB on read. Magick's
// "RGBA" pixel export does NOT convert colorspace, so without an explicit
// normalization the raw C,M,Y,K quantums land in R,G,B,A — a pure-cyan CMYK pixel
// came back bright RED. Synthesize a genuine CMYK file and assert read() -> cyan.
TEST(ImageTest, ReadsCmykInputAsRgb) {
  Magick::InitializeMagick(nullptr); // idempotent; the test uses Magick++ directly
  // A 4x4 pure-cyan image transformed into the CMYK colorspace, so the file on
  // disk is genuinely CMYK (not an RGB image merely tagged CMYK).
  Magick::Image cyan(Magick::Geometry(4, 4), Magick::ColorRGB(0.0, 1.0, 1.0));
  cyan.colorSpace(Magick::sRGBColorspace);
  cyan.colorSpace(Magick::CMYKColorspace); // transforms the pixels to CMYK
  ASSERT_EQ(cyan.colorSpace(), Magick::CMYKColorspace);

  const std::string path = std::string(::testing::TempDir()) + "/cvc_image_cmyk.tif";
  try {
    cyan.write(path); // TIFF stores CMYK losslessly, no Adobe-inversion games
  } catch (const Magick::Exception &e) {
    GTEST_SKIP() << "CMYK/TIFF delegate unavailable in this ImageMagick build: " << e.what();
  }

  image b = image::load(path);
  ASSERT_EQ(b.width(), 4);
  ASSERT_EQ(b.height(), 4);
  ASSERT_EQ(b.channels(), 4);
  const unsigned char *p = b.data();
  // Cyan is sRGB (0, 255, 255). Loose bounds cleanly separate the correct cyan
  // from the pre-fix garbage (a raw CMYK->RGBA dump rendered this ~ (255, 0, 0)).
  EXPECT_LT(p[0], 80) << "R too high — CMYK was dumped straight into RGBA (the bug)";
  EXPECT_GT(p[1], 180) << "G too low for cyan";
  EXPECT_GT(p[2], 180) << "B too low for cyan";
  EXPECT_EQ(p[3], 255) << "alpha should be opaque";
}
#endif // CVC_ENABLE_IMAGEMAGICK
