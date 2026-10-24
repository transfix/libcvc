// cvc::init_imagemagick() must give ImageMagick its coders and config with no
// MAGICK_* variables in the environment. On Windows the cvcpkg ImageMagick is
// extracted, not installed, so it has no registry entry and libcvc points it at
// the bundle itself (magick_io.cpp). CI sets the MAGICK_* variables for its
// whole Windows ctest step, which hides a broken lookup from every other test,
// so this executable removes them before anything initializes Magick. It is an
// executable of its own because that lookup runs once per process: in a process
// where Magick is already up, removing the variables proves nothing.

#include <cvc/image/image.h>
#include <gtest/gtest.h>

#ifdef CVC_ENABLE_IMAGEMAGICK
#include <Magick++.h>
#include <cstdio>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

// Whether Magick was already initialized when the variables were removed, which
// would make every check below meaningless.
bool magick_up_before_scrub = false;

// Runs during static initialization: before main, so before any test, in any
// order, can initialize Magick.
const bool magick_env_scrubbed = [] {
  magick_up_before_scrub = MagickCore::IsMagickCoreInstantiated() != MagickCore::MagickFalse;
  for (const char *k :
       {"MAGICK_CODER_MODULE_PATH", "MAGICK_FILTER_MODULE_PATH", "MAGICK_CONFIGURE_PATH"}) {
#ifdef _WIN32
    _putenv_s(k, ""); // an empty value removes it, from the CRT's copy and the OS's
#else
    unsetenv(k);
#endif
  }
  return true;
}();

void init_without_magick_env() {
  ASSERT_TRUE(magick_env_scrubbed);
  ASSERT_FALSE(magick_up_before_scrub)
      << "Magick was initialized before the MAGICK_* variables were removed";
  cvc::init_imagemagick();
}

} // namespace

TEST(MagickBootstrapTest, ReadsXcWithoutMagickEnvironment) {
  ASSERT_NO_FATAL_FAILURE(init_without_magick_env());
  // xc: is a coder module like any other: a Magick without its coders cannot
  // even construct a solid-color image.
  Magick::Image cyan(Magick::Geometry(4, 4), Magick::ColorRGB(0.0, 1.0, 1.0));
  EXPECT_EQ(cyan.columns(), 4u);
  EXPECT_EQ(cyan.rows(), 4u);
}

#ifdef _WIN32
TEST(MagickBootstrapTest, PointsMagickAtTheCoderModules) {
  ASSERT_NO_FATAL_FAILURE(init_without_magick_env());
  const char *coders = std::getenv("MAGICK_CODER_MODULE_PATH");
  ASSERT_NE(coders, nullptr) << "init_imagemagick() found no ImageMagick coder modules";
  WIN32_FIND_DATAA d;
  HANDLE h = FindFirstFileA((std::string(coders) + "\\IM_MOD_RL_png_.dll").c_str(), &d);
  EXPECT_NE(h, INVALID_HANDLE_VALUE) << "no PNG coder in MAGICK_CODER_MODULE_PATH=" << coders;
  if (h != INVALID_HANDLE_VALUE)
    FindClose(h);
  const char *config = std::getenv("MAGICK_CONFIGURE_PATH");
  ASSERT_NE(config, nullptr) << "init_imagemagick() found no ImageMagick config";
  h = FindFirstFileA((std::string(config) + "\\*.xml").c_str(), &d);
  EXPECT_NE(h, INVALID_HANDLE_VALUE) << "no .xml config in MAGICK_CONFIGURE_PATH=" << config;
  if (h != INVALID_HANDLE_VALUE)
    FindClose(h);
}

// PNG and JPEG through Magick itself: cvc::image reads both with stb first, so
// only a direct Magick++ read shows that Magick's own decoders load.
TEST(MagickBootstrapTest, RoundTripsPngAndJpegWithoutMagickEnvironment) {
  ASSERT_NO_FATAL_FAILURE(init_without_magick_env());
  for (const char *fmt : {"PNG", "JPEG"}) {
    Magick::Image cyan(Magick::Geometry(4, 4), Magick::ColorRGB(0.0, 1.0, 1.0));
    cyan.magick(fmt);
    const std::string path = std::string(::testing::TempDir()) + "/cvc_magick_bootstrap." + fmt;
    cyan.write(path);
    Magick::Image back(path);
    EXPECT_EQ(back.magick(), fmt);
    ASSERT_EQ(back.columns(), 4u) << fmt;
    ASSERT_EQ(back.rows(), 4u) << fmt;
    const Magick::ColorRGB px = back.pixelColor(1, 1);
    EXPECT_LT(px.red(), 0.2) << fmt;
    EXPECT_GT(px.green(), 0.8) << fmt;
    EXPECT_GT(px.blue(), 0.8) << fmt;
    std::remove(path.c_str());
  }
}

// TIFF through cvc::image, which has no stb reader or writer for it, so both
// directions take libcvc's own Magick handler.
TEST(MagickBootstrapTest, CvcImageRoundTripsTiffWithoutMagickEnvironment) {
  ASSERT_NO_FATAL_FAILURE(init_without_magick_env());
  cvc::image a(3, 2, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  unsigned char *p = a.data();
  for (int i = 0; i < 3 * 2 * 4; ++i)
    p[i] = static_cast<unsigned char>(i % 4 == 3 ? 255 : i * 9); // opaque: no alpha games
  const std::string path = std::string(::testing::TempDir()) + "/cvc_magick_bootstrap.tif";
  a.save(path);
  cvc::image b = cvc::image::load(path);
  ASSERT_EQ(b.width(), 3);
  ASSERT_EQ(b.height(), 2);
  for (int i = 0; i < 3 * 2 * 4; ++i)
    EXPECT_EQ(b.data()[i], p[i]) << "byte " << i;
  std::remove(path.c_str());
}
#endif // _WIN32

#else
TEST(MagickBootstrapTest, SkippedWithoutImageMagick) {
  GTEST_SKIP() << "libcvc was built without ImageMagick";
}
#endif // CVC_ENABLE_IMAGEMAGICK
