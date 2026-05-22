/*
  Copyright 2026 The University of Texas at Austin
  Tests for cvc::volume and cvc::geometry codec registration.
*/

#include <boost/any.hpp>
#include <cvc/app.h>
#include <cvc/geometry.h>
#include <cvc/state_codec_registry.h>
#include <cvc/state_volume_codec.h>
#include <cvc/volume.h>
#include <gtest/gtest.h>

using namespace CVC_NAMESPACE;

class VolumeCodecTest : public ::testing::Test {
protected:
  void SetUp() override { register_volume_geometry_codecs(reg); }

  state_codec_registry reg;
  app ctx;
};

// ---- volume codec ----

TEST_F(VolumeCodecTest, HasVolumeCodec) { EXPECT_TRUE(reg.has("cvc::volume")); }

TEST_F(VolumeCodecTest, HasGeometryCodec) { EXPECT_TRUE(reg.has("cvc::geometry")); }

TEST_F(VolumeCodecTest, VolumeCodecId) {
  EXPECT_EQ(reg.codec_id_for("cvc::volume"), "cvc.volume.v1");
}

TEST_F(VolumeCodecTest, GeometryCodecId) {
  EXPECT_EQ(reg.codec_id_for("cvc::geometry"), "cvc.geometry.v1");
}

TEST_F(VolumeCodecTest, VolumeRoundTrip_UChar) {
  dimension d;
  d.xdim = 4;
  d.ydim = 4;
  d.zdim = 4;
  bounding_box bb(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
  volume vol(ctx, d, UChar, bb);
  vol.desc("test_volume");

  // Fill with known pattern.
  for (uint64 z = 0; z < d.zdim; ++z)
    for (uint64 y = 0; y < d.ydim; ++y)
      for (uint64 x = 0; x < d.xdim; ++x)
        vol(x, y, z, static_cast<double>((x + y * 4 + z * 16) % 256));

  auto bytes = reg.encode("cvc::volume", boost::any(vol));
  EXPECT_GT(bytes.size(), 0u);

  auto decoded_any = reg.decode("cvc::volume", bytes);
  ASSERT_TRUE(!decoded_any.empty());

  auto decoded = boost::any_cast<volume>(decoded_any);
  EXPECT_EQ(decoded.XDim(), 4u);
  EXPECT_EQ(decoded.YDim(), 4u);
  EXPECT_EQ(decoded.ZDim(), 4u);
  EXPECT_EQ(decoded.voxelType(), UChar);
  EXPECT_DOUBLE_EQ(decoded.XMin(), -1.0);
  EXPECT_DOUBLE_EQ(decoded.XMax(), 1.0);
  EXPECT_EQ(decoded.desc(), "test_volume");

  for (uint64 z = 0; z < d.zdim; ++z)
    for (uint64 y = 0; y < d.ydim; ++y)
      for (uint64 x = 0; x < d.xdim; ++x)
        EXPECT_DOUBLE_EQ(decoded(x, y, z), vol(x, y, z));
}

TEST_F(VolumeCodecTest, VolumeRoundTrip_Float) {
  dimension d;
  d.xdim = 8;
  d.ydim = 8;
  d.zdim = 8;
  bounding_box bb(0, 0, 0, 10, 10, 10);
  volume vol(ctx, d, Float, bb);
  vol.desc("float_vol");

  for (uint64 z = 0; z < d.zdim; ++z)
    for (uint64 y = 0; y < d.ydim; ++y)
      for (uint64 x = 0; x < d.xdim; ++x)
        vol(x, y, z, x * 1.5 + y * 0.5 + z * 0.1);

  auto bytes = reg.encode("cvc::volume", boost::any(vol));
  auto decoded = boost::any_cast<volume>(reg.decode("cvc::volume", bytes));

  EXPECT_EQ(decoded.XDim(), 8u);
  EXPECT_EQ(decoded.voxelType(), Float);
  EXPECT_DOUBLE_EQ(decoded.XMax(), 10.0);
  EXPECT_EQ(decoded.desc(), "float_vol");

  for (uint64 z = 0; z < d.zdim; ++z)
    for (uint64 y = 0; y < d.ydim; ++y)
      for (uint64 x = 0; x < d.xdim; ++x)
        EXPECT_NEAR(decoded(x, y, z), vol(x, y, z), 1e-5);
}

TEST_F(VolumeCodecTest, VolumeEncodedSize) {
  dimension d;
  d.xdim = 4;
  d.ydim = 4;
  d.zdim = 4;
  volume vol(ctx, d, UChar);
  auto bytes = reg.encode("cvc::volume", boost::any(vol));

  // Header: 4(magic) + 3*8(dims) + 4(vtype) + 6*8(bbox) + 4+7(desc="No Name") = 91
  // Body: 4*4*4*1 = 64
  // Total: 155
  EXPECT_EQ(bytes.size(), 155u);
}

// ---- geometry codec ----

TEST_F(VolumeCodecTest, GeometryRoundTrip_TriSurface) {
  geometry geo(ctx);
  geo.set_geometry_type(geometry::SURFACE_TRI);

  auto &pts = geo.points();
  pts.push_back({{0.0, 0.0, 0.0}});
  pts.push_back({{1.0, 0.0, 0.0}});
  pts.push_back({{0.0, 1.0, 0.0}});

  auto &nrm = geo.normals();
  nrm.push_back({{0.0, 0.0, 1.0}});
  nrm.push_back({{0.0, 0.0, 1.0}});
  nrm.push_back({{0.0, 0.0, 1.0}});

  auto &clr = geo.colors();
  clr.push_back({{1.0, 0.0, 0.0}});
  clr.push_back({{0.0, 1.0, 0.0}});
  clr.push_back({{0.0, 0.0, 1.0}});

  auto &tri = geo.tris();
  tri.push_back({{0, 1, 2}});

  auto bytes = reg.encode("cvc::geometry", boost::any(geo));
  EXPECT_GT(bytes.size(), 0u);

  auto decoded = boost::any_cast<geometry>(reg.decode("cvc::geometry", bytes));
  EXPECT_EQ(decoded.get_geometry_type(), geometry::SURFACE_TRI);
  EXPECT_EQ(decoded.num_points(), 3u);
  EXPECT_EQ(decoded.const_normals().size(), 3u);
  EXPECT_EQ(decoded.const_colors().size(), 3u);
  EXPECT_EQ(decoded.num_tris(), 1u);

  EXPECT_DOUBLE_EQ(decoded.const_points()[1][0], 1.0);
  EXPECT_DOUBLE_EQ(decoded.const_normals()[0][2], 1.0);
  EXPECT_DOUBLE_EQ(decoded.const_colors()[2][2], 1.0);
  EXPECT_EQ(decoded.const_tris()[0][2], 2u);
}

TEST_F(VolumeCodecTest, GeometryRoundTrip_TetVolume) {
  geometry geo(ctx);
  geo.set_geometry_type(geometry::VOLUME_TET);

  auto &pts = geo.points();
  pts.push_back({{0, 0, 0}});
  pts.push_back({{1, 0, 0}});
  pts.push_back({{0, 1, 0}});
  pts.push_back({{0, 0, 1}});

  auto &tet = geo.tets();
  tet.push_back({{0, 1, 2, 3}});

  auto bytes = reg.encode("cvc::geometry", boost::any(geo));
  auto decoded = boost::any_cast<geometry>(reg.decode("cvc::geometry", bytes));

  EXPECT_EQ(decoded.get_geometry_type(), geometry::VOLUME_TET);
  EXPECT_EQ(decoded.num_points(), 4u);
  EXPECT_EQ(decoded.num_tets(), 1u);
  EXPECT_EQ(decoded.const_tets()[0][3], 3u);
}

TEST_F(VolumeCodecTest, GeometryRoundTrip_EmptyGeometry) {
  geometry geo(ctx);
  auto bytes = reg.encode("cvc::geometry", boost::any(geo));
  auto decoded = boost::any_cast<geometry>(reg.decode("cvc::geometry", bytes));
  EXPECT_EQ(decoded.num_points(), 0u);
  EXPECT_EQ(decoded.num_tris(), 0u);
}

TEST_F(VolumeCodecTest, GeometryRoundTrip_Lines) {
  geometry geo(ctx);
  auto &pts = geo.points();
  pts.push_back({{0, 0, 0}});
  pts.push_back({{1, 1, 1}});

  auto &lin = geo.lines();
  lin.push_back({{0, 1}});

  auto bytes = reg.encode("cvc::geometry", boost::any(geo));
  auto decoded = boost::any_cast<geometry>(reg.decode("cvc::geometry", bytes));
  EXPECT_EQ(decoded.const_lines().size(), 1u);
  EXPECT_EQ(decoded.const_lines()[0][1], 1u);
}

TEST_F(VolumeCodecTest, RegistrationIdempotent) {
  register_volume_geometry_codecs(reg);
  register_volume_geometry_codecs(reg);
  EXPECT_TRUE(reg.has("cvc::volume"));
  EXPECT_TRUE(reg.has("cvc::geometry"));
}
