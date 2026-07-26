// Tests for cvc::image (Phase-1 image surface): the value type + copy-on-write,
// manipulation (flip/convert/resize), the image_file_io registry, and the
// ImageMagick round-trip (guarded on CVC_ENABLE_IMAGEMAGICK).

#include <cstdint>
#include <cvc/image/image.h>
#include <cvc/image/image.h> // include-twice: header must be idempotent
#include <gtest/gtest.h>
#include <string>

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
  EXPECT_EQ(a.data(), a.data());
  unsigned char before = a.data()[0];
  // mutate b through data() -> detach; a must be untouched
  b.data()[0] = static_cast<unsigned char>(before ^ 0xff);
  EXPECT_EQ(a.data()[0], before);
  EXPECT_NE(b.data()[0], before);
  // storage() shares without detaching
  EXPECT_EQ(a.storage().get(), a.data());
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

TEST(ImageTest, ReadMissingFileThrows) {
  EXPECT_ANY_THROW(image::load("/no/such/dir/nope.png"));
}
#endif // CVC_ENABLE_IMAGEMAGICK
