/*
  Copyright 2007-2011 The University of Texas at Austin

  Round-trip volume I/O tests across all supported file formats.

  Targets coverage gaps in:
    mrc_io.cpp, rawiv_io.cpp, rawv_io.cpp, spider_io.cpp, vtk_io.cpp,
    cvcraw_io.cpp, volume_file_info.cpp, volume_file_io.cpp
*/

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/volume.h>
#include <cvc/volume_file_info.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <sstream>
#include <thread>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() _getpid()
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#endif

using namespace CVC_NAMESPACE;

namespace {

// Build a small synthetic Float volume with a recognizable gradient pattern.
// Float is the universally-supported voxel type across MRC/RAWIV/RAWV/Spider/VTK.
volume make_test_volume(app &ctx, unsigned int xdim = 8, unsigned int ydim = 8,
                        unsigned int zdim = 8) {
  volume v(ctx, dimension(xdim, ydim, zdim), Float,
           bounding_box(0.0, 0.0, 0.0, double(xdim - 1), double(ydim - 1), double(zdim - 1)));
  for (unsigned int k = 0; k < zdim; ++k)
    for (unsigned int j = 0; j < ydim; ++j)
      for (unsigned int i = 0; i < xdim; ++i) {
        double val = double(i) + 10.0 * double(j) + 100.0 * double(k);
        v(i, j, k, val);
      }
  v.desc("io-test");
  return v;
}

void expect_volume_equal(const volume &a, const volume &b, double tol = 1e-3) {
  ASSERT_EQ(a.XDim(), b.XDim());
  ASSERT_EQ(a.YDim(), b.YDim());
  ASSERT_EQ(a.ZDim(), b.ZDim());
  for (unsigned int k = 0; k < a.ZDim(); ++k)
    for (unsigned int j = 0; j < a.YDim(); ++j)
      for (unsigned int i = 0; i < a.XDim(); ++i)
        EXPECT_NEAR(a(i, j, k), b(i, j, k), tol)
            << "mismatch at (" << i << "," << j << "," << k << ")";
}

} // namespace

class VolumeIOTest : public ::testing::Test {
protected:
  app ctx;
  std::string test_dir;

  void SetUp() override {
    namespace fs = std::filesystem;
    static std::atomic<unsigned> counter{0};
    std::ostringstream oss;
    oss << "volio_test_" << CVC_GETPID() << "_" << std::this_thread::get_id() << "_"
        << counter.fetch_add(1, std::memory_order_relaxed) << "_"
        << std::chrono::steady_clock::now().time_since_epoch().count();
    fs::path base = std::getenv("CMAKE_CURRENT_BINARY_DIR")
                        ? fs::path(std::getenv("CMAKE_CURRENT_BINARY_DIR"))
                        : fs::current_path();
    fs::path dir = base / oss.str();
    fs::create_directories(dir);
    test_dir = dir.string();
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(test_dir, ec);
  }

  std::string path(const std::string &name) const { return test_dir + "/" + name; }
};

// ============================================================================
// RAWIV (.rawiv) - UT Austin volume format, supports UChar/UShort/Float
// ============================================================================

TEST_F(VolumeIOTest, RawivRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.rawiv");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in);
}

TEST_F(VolumeIOTest, RawivFileInfo) {
  volume out = make_test_volume(ctx, 4, 6, 8);
  std::string p = path("info.rawiv");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 4u);
  EXPECT_EQ(info.YDim(), 6u);
  EXPECT_EQ(info.ZDim(), 8u);
  EXPECT_EQ(info.numVariables(), 1u);
  EXPECT_EQ(info.numTimesteps(), 1u);
  EXPECT_EQ(info.voxelType(), Float);
}

// ============================================================================
// RAWV (.rawv) - Multivariate raw format
// ============================================================================

TEST_F(VolumeIOTest, RawvRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.rawv");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in);
}

TEST_F(VolumeIOTest, RawvFileInfo) {
  volume out = make_test_volume(ctx);
  std::string p = path("info.rawv");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 8u);
  EXPECT_EQ(info.numVariables(), 1u);
}

// ============================================================================
// MRC (.mrc / .map) - Cryo-EM standard
// ============================================================================

TEST_F(VolumeIOTest, MrcRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.mrc");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in);
}

TEST_F(VolumeIOTest, MrcMapExtension) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.map");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in);
}

TEST_F(VolumeIOTest, MrcFileInfo) {
  volume out = make_test_volume(ctx, 8, 10, 12);
  std::string p = path("info.mrc");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 8u);
  EXPECT_EQ(info.YDim(), 10u);
  EXPECT_EQ(info.ZDim(), 12u);
}

// ============================================================================
// Spider (.spi / .vol / .xmp) - Single-particle EM
// ============================================================================

TEST_F(VolumeIOTest, SpiderRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.spi");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in, 1e-2);
}

TEST_F(VolumeIOTest, SpiderVolExtension) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.vol");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in, 1e-2);
}

TEST_F(VolumeIOTest, SpiderFileInfo) {
  volume out = make_test_volume(ctx);
  std::string p = path("info.spi");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 8u);
}

// ============================================================================
// VTK (.vtk) - Visualization Toolkit legacy structured points
// ============================================================================

TEST_F(VolumeIOTest, VtkRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.vtk");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in, 1e-2);
}

TEST_F(VolumeIOTest, VtkFileInfo) {
  volume out = make_test_volume(ctx);
  std::string p = path("info.vtk");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_TRUE(info.isSet());
  EXPECT_EQ(info.XDim(), 8u);
}

// ============================================================================
// CVC Raw (.cvc / .raw) - In-house simple format
// ============================================================================

TEST_F(VolumeIOTest, CvcRawRoundTrip) {
  volume out = make_test_volume(ctx);
  std::string p = path("test.cvc");
  ASSERT_NO_THROW(out.write(p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  expect_volume_equal(out, in);
}

// ============================================================================
// Cross-format conversion
// ============================================================================

TEST_F(VolumeIOTest, RawivToMrcConversion) {
  volume original = make_test_volume(ctx);
  std::string rawiv_path = path("conv.rawiv");
  std::string mrc_path = path("conv.mrc");

  original.write(rawiv_path);
  volume mid(ctx);
  mid.read(rawiv_path);
  mid.write(mrc_path);

  volume final_(ctx);
  final_.read(mrc_path);
  expect_volume_equal(original, final_);
}

TEST_F(VolumeIOTest, MrcToRawivConversion) {
  volume original = make_test_volume(ctx);
  std::string mrc_path = path("conv2.mrc");
  std::string rawiv_path = path("conv2.rawiv");

  original.write(mrc_path);
  volume mid(ctx);
  mid.read(mrc_path);
  mid.write(rawiv_path);

  volume final_(ctx);
  final_.read(rawiv_path);
  expect_volume_equal(original, final_);
}

// ============================================================================
// Different voxel types
// ============================================================================

TEST_F(VolumeIOTest, RawivUCharRoundTrip) {
  volume out(ctx, dimension(6, 6, 6), UChar);
  for (unsigned int k = 0; k < 6; ++k)
    for (unsigned int j = 0; j < 6; ++j)
      for (unsigned int i = 0; i < 6; ++i)
        out(i, j, k, double(i + j + k));

  std::string p = path("uchar.rawiv");
  ASSERT_NO_THROW(out.write(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  EXPECT_EQ(in.voxelType(), UChar);
  expect_volume_equal(out, in, 0.5);
}

TEST_F(VolumeIOTest, RawivUShortRoundTrip) {
  volume out(ctx, dimension(6, 6, 6), UShort);
  for (unsigned int k = 0; k < 6; ++k)
    for (unsigned int j = 0; j < 6; ++j)
      for (unsigned int i = 0; i < 6; ++i)
        out(i, j, k, double(i * 100 + j * 10 + k));

  std::string p = path("ushort.rawiv");
  ASSERT_NO_THROW(out.write(p));

  volume in(ctx);
  ASSERT_NO_THROW(in.read(p));
  EXPECT_EQ(in.voxelType(), UShort);
  expect_volume_equal(out, in, 0.5);
}

// ============================================================================
// Error handling
// ============================================================================

TEST_F(VolumeIOTest, ReadNonExistentThrows) {
  volume in(ctx);
  EXPECT_ANY_THROW(in.read(path("does_not_exist.rawiv")));
}

TEST_F(VolumeIOTest, UnknownExtensionThrows) {
  volume out = make_test_volume(ctx);
  EXPECT_ANY_THROW(out.write(path("test.xyz_unknown")));
}

TEST_F(VolumeIOTest, FileInfoOnNonExistentThrows) {
  EXPECT_ANY_THROW(volume_file_info(ctx, path("does_not_exist.mrc")));
}

// ============================================================================
// volume_file_info copy/assign and min/max
// ============================================================================

TEST_F(VolumeIOTest, FileInfoMinMax) {
  volume out = make_test_volume(ctx);
  std::string p = path("minmax.rawiv");
  out.write(p);

  volume_file_info info(ctx, p);
  // make_test_volume goes from 0 (at 0,0,0) to 7+70+700 = 777 (at 7,7,7)
  EXPECT_NEAR(info.min(), 0.0, 1.0);
  EXPECT_NEAR(info.max(), 777.0, 1.0);
}

TEST_F(VolumeIOTest, FileInfoCopy) {
  volume out = make_test_volume(ctx);
  std::string p = path("copy.rawiv");
  out.write(p);

  volume_file_info info1(ctx, p);
  volume_file_info info2(info1);
  EXPECT_EQ(info1.XDim(), info2.XDim());
  EXPECT_EQ(info1.filename(), info2.filename());
}

TEST_F(VolumeIOTest, FileInfoBoundingBox) {
  volume out = make_test_volume(ctx);
  std::string p = path("bbox.rawiv");
  out.write(p);

  volume_file_info info(ctx, p);
  EXPECT_DOUBLE_EQ(info.XMin(), 0.0);
  EXPECT_DOUBLE_EQ(info.YMin(), 0.0);
  EXPECT_DOUBLE_EQ(info.ZMin(), 0.0);
  EXPECT_DOUBLE_EQ(info.XMax(), 7.0);
  EXPECT_DOUBLE_EQ(info.YMax(), 7.0);
  EXPECT_DOUBLE_EQ(info.ZMax(), 7.0);
}
