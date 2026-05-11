/*
  Copyright 2007-2011 The University of Texas at Austin

  Unit tests for cvc/utility.h free functions.

  Targets coverage gaps in:
    utility.cpp (calcGradient, sub, volconvert, json, get_xmlrpc_host_and_port,
                 is_geometry/volume/_file_info, is_geometry/volume_filename,
                 load/save, get_local_ip_address, upToPowerOfTwo, ends_with, getTriVal)
*/

#include <atomic>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <thread>
#include <vector>

#include <cvc/app.h>
#include <cvc/geometry.h>
#include <cvc/utility.h>
#include <cvc/volume.h>
#include <cvc/volume_file_info.h>

#include <boost/any.hpp>
#include <boost/property_tree/ptree.hpp>

#include <gtest/gtest.h>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() _getpid()
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#endif

using namespace CVC_NAMESPACE;

class UtilityTest : public ::testing::Test {
protected:
  app ctx;
  std::string test_dir;

  void SetUp() override {
    namespace fs = std::filesystem;
    static std::atomic<unsigned> counter{0};
    std::ostringstream oss;
    oss << "util_test_" << CVC_GETPID() << "_" << std::this_thread::get_id() << "_"
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

// ---------------------------------------------------------------------------
// Header-inline helpers
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, UpToPowerOfTwo) {
  EXPECT_EQ(cvc::upToPowerOfTwo(1u), 1u);
  EXPECT_EQ(cvc::upToPowerOfTwo(2u), 2u);
  EXPECT_EQ(cvc::upToPowerOfTwo(3u), 4u);
  EXPECT_EQ(cvc::upToPowerOfTwo(5u), 8u);
  EXPECT_EQ(cvc::upToPowerOfTwo(17u), 32u);
  EXPECT_EQ(cvc::upToPowerOfTwo(1000u), 1024u);
}

TEST_F(UtilityTest, EndsWith) {
  EXPECT_TRUE(cvc::ends_with("foo.mrc", ".mrc"));
  EXPECT_TRUE(cvc::ends_with("foo.rawiv", "rawiv"));
  EXPECT_FALSE(cvc::ends_with("foo.mrc", ".rawiv"));
  EXPECT_FALSE(cvc::ends_with("foo", "foobar"));
  EXPECT_TRUE(cvc::ends_with("", ""));
}

TEST_F(UtilityTest, MinMaxTemplates) {
  EXPECT_EQ(cvc::MIN(3, 5), 3);
  EXPECT_EQ(cvc::MAX(3, 5), 5);
  EXPECT_DOUBLE_EQ(cvc::MIN(1.5, 2.5), 1.5);
  EXPECT_DOUBLE_EQ(cvc::MAX(1.5, 2.5), 2.5);
}

TEST_F(UtilityTest, GetTriValAtCorners) {
  // 8 corner values
  double v[8] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};
  // Sampling at the (0,0,0) corner should equal v[0]
  double s = cvc::getTriVal(v, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  EXPECT_NEAR(s, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// JSON property-tree round trip
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, JsonRoundTrip) {
  boost::property_tree::ptree pt;
  pt.put("hello", "world");
  pt.put("answer", 42);

  std::string s = cvc::json(pt);
  EXPECT_NE(s.find("hello"), std::string::npos);
  EXPECT_NE(s.find("world"), std::string::npos);
  EXPECT_NE(s.find("42"), std::string::npos);

  boost::property_tree::ptree pt2 = cvc::json(s);
  EXPECT_EQ(pt2.get<std::string>("hello"), "world");
  EXPECT_EQ(pt2.get<int>("answer"), 42);
}

// ---------------------------------------------------------------------------
// XML-RPC host/port parsing
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, XmlRpcHostPortDefault) {
  boost::tuple<std::string, int> hp = cvc::get_xmlrpc_host_and_port("");
  EXPECT_EQ(boost::get<0>(hp), "");
  EXPECT_EQ(boost::get<1>(hp), 23196);
}

TEST_F(UtilityTest, XmlRpcHostPortHostOnly) {
  boost::tuple<std::string, int> hp = cvc::get_xmlrpc_host_and_port("example.com");
  EXPECT_EQ(boost::get<0>(hp), "example.com");
  EXPECT_EQ(boost::get<1>(hp), 23196);
}

TEST_F(UtilityTest, XmlRpcHostPortBoth) {
  boost::tuple<std::string, int> hp = cvc::get_xmlrpc_host_and_port("server.local:9000");
  EXPECT_EQ(boost::get<0>(hp), "server.local");
  EXPECT_EQ(boost::get<1>(hp), 9000);
}

// ---------------------------------------------------------------------------
// is_geometry / is_volume / is_*_filename
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, IsVolumeFilename) {
  EXPECT_TRUE(cvc::is_volume_filename("foo.rawiv"));
  EXPECT_TRUE(cvc::is_volume_filename("a/b/c.mrc"));
  EXPECT_FALSE(cvc::is_volume_filename("foo.txt"));
  EXPECT_FALSE(cvc::is_volume_filename("nope.bin"));
}

TEST_F(UtilityTest, IsGeometryFilename) {
  EXPECT_TRUE(cvc::is_geometry_filename("mesh.off"));
  EXPECT_FALSE(cvc::is_geometry_filename("foo.rawiv"));
}

TEST_F(UtilityTest, IsVolumeAny) {
  volume v(ctx, dimension(4, 4, 4), Float);
  boost::any a = v;
  EXPECT_TRUE(cvc::is_volume(a));
  EXPECT_FALSE(cvc::is_geometry(a));
  EXPECT_FALSE(cvc::is_volume_file_info(a));
}

TEST_F(UtilityTest, IsGeometryAny) {
  geometry g;
  boost::any a = g;
  EXPECT_TRUE(cvc::is_geometry(a));
  EXPECT_FALSE(cvc::is_volume(a));
}

TEST_F(UtilityTest, IsVolumeFileInfoAny) {
  volume v(ctx, dimension(4, 4, 4), Float);
  v.write(path("vfi.rawiv"));
  volume_file_info vfi(ctx, path("vfi.rawiv"));
  boost::any a = vfi;
  EXPECT_TRUE(cvc::is_volume_file_info(a));
  EXPECT_FALSE(cvc::is_volume(a));
}

// ---------------------------------------------------------------------------
// load / save dispatch
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, SaveLoadVolume) {
  volume out(ctx, dimension(5, 5, 5), Float);
  out.fill(7.5);
  std::string p = path("save_load.rawiv");

  boost::any data = out;
  cvc::save(ctx, data, p);
  ASSERT_TRUE(std::filesystem::exists(p));

  boost::any loaded = cvc::load(ctx, p);
  ASSERT_TRUE(cvc::is_volume(loaded));
  volume in = boost::any_cast<volume>(loaded);
  EXPECT_EQ(in.XDim(), 5u);
  EXPECT_NEAR(in(2, 2, 2), 7.5, 1e-3);
}

TEST_F(UtilityTest, LoadUnknownExtensionThrows) {
  EXPECT_ANY_THROW(cvc::load(ctx, path("foo.bogus_ext_xyz")));
}

// ---------------------------------------------------------------------------
// sub() - extract subvolume
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, SubVolumeExtract) {
  volume src(ctx, dimension(8, 8, 8), Float,
             bounding_box(0.0, 0.0, 0.0, 7.0, 7.0, 7.0));
  for (unsigned int k = 0; k < 8; ++k)
    for (unsigned int j = 0; j < 8; ++j)
      for (unsigned int i = 0; i < 8; ++i)
        src(i, j, k, double(i + j * 10 + k * 100));

  volume dest(ctx);
  cvc::sub(ctx, dest, src, 2, 2, 2, dimension(4, 4, 4));

  EXPECT_EQ(dest.XDim(), 4u);
  EXPECT_EQ(dest.YDim(), 4u);
  EXPECT_EQ(dest.ZDim(), 4u);
  EXPECT_NEAR(dest(0, 0, 0), src(2, 2, 2), 1e-6);
  EXPECT_NEAR(dest(3, 3, 3), src(5, 5, 5), 1e-6);
}

TEST_F(UtilityTest, SubVolumeOutOfBoundsThrows) {
  volume src(ctx, dimension(4, 4, 4), Float);
  volume dest(ctx);
  EXPECT_ANY_THROW(cvc::sub(ctx, dest, src, 0, 0, 0, dimension(5, 5, 5)));
}

// ---------------------------------------------------------------------------
// calcGradient
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, CalcGradient) {
  // Build a simple linear ramp along X so gradient X should be nonzero, Y/Z zero.
  volume v(ctx, dimension(8, 8, 8), Float,
           bounding_box(0.0, 0.0, 0.0, 7.0, 7.0, 7.0));
  for (unsigned int k = 0; k < 8; ++k)
    for (unsigned int j = 0; j < 8; ++j)
      for (unsigned int i = 0; i < 8; ++i)
        v(i, j, k, double(i));

  std::vector<volume> grad;
  ASSERT_NO_THROW(cvc::calcGradient(ctx, grad, v, Float));
  ASSERT_EQ(grad.size(), 3u);
  // Gradient X interior should be positive; Y/Z interior should be near zero.
  EXPECT_GT(grad[0](4, 4, 4), 0.0);
  EXPECT_NEAR(grad[1](4, 4, 4), 0.0, 1e-3);
  EXPECT_NEAR(grad[2](4, 4, 4), 0.0, 1e-3);
}

// ---------------------------------------------------------------------------
// volconvert - out-of-core file format conversion
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, VolConvertRawivToMrc) {
  volume src(ctx, dimension(4, 4, 4), Float);
  for (unsigned int k = 0; k < 4; ++k)
    for (unsigned int j = 0; j < 4; ++j)
      for (unsigned int i = 0; i < 4; ++i)
        src(i, j, k, double(i + 4 * j + 16 * k));
  std::string in_p = path("convert_in.rawiv");
  std::string out_p = path("convert_out.mrc");
  src.write(in_p);

  ASSERT_NO_THROW(cvc::volconvert(ctx, in_p, out_p));
  ASSERT_TRUE(std::filesystem::exists(out_p));

  volume converted(ctx);
  converted.read(out_p);
  EXPECT_EQ(converted.XDim(), 4u);
}

// ---------------------------------------------------------------------------
// createVolumeFile - shortcut overloads
// ---------------------------------------------------------------------------

TEST_F(UtilityTest, CreateVolumeFileFromVolume) {
  volume v(ctx, dimension(4, 4, 4), Float);
  v.fill(9.0);
  std::string p = path("created.rawiv");

  ASSERT_NO_THROW(cvc::createVolumeFile(ctx, v, p));
  ASSERT_TRUE(std::filesystem::exists(p));

  volume readback(ctx);
  readback.read(p);
  EXPECT_NEAR(readback(2, 2, 2), 9.0, 1e-3);
}
