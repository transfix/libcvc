// lod_store_test -- scene.cvch5 round-trip for LOD pyramids.
//
// Build mesh + image pyramids, write them to an HDF5 file, read them back, and
// assert the rungs (positions, triangles, pixels, world-error ladder) survive
// bit-for-bit; then that read_lod_index() enumerates both assets and has_pyramid()
// honours the content hash. HDF5-only (gated in CMake by CVC_USING_HDF5).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/store.h>
#include <gtest/gtest.h>
#include <string>

using cvc::geometry;
using cvc::image;

namespace {

geometry bumpy_grid(cvc::app &ctx, int n) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  geometry::uvs_t &UV = g.uvs();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      P.push_back({double(i), double(j), 1.2 * std::sin(0.5 * i) * std::cos(0.7 * j)});
      UV.push_back({double(i) / n, double(j) / n});
    }
  geometry::tris_t &T = g.tris();
  auto id = [n](int i, int j) { return std::uint64_t(i * n + j); };
  for (int i = 0; i < n - 1; ++i)
    for (int j = 0; j < n - 1; ++j) {
      T.push_back({id(i, j), id(i + 1, j), id(i, j + 1)});
      T.push_back({id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)});
    }
  g.set_geometry_type(geometry::SURFACE_TRI);
  return g;
}

std::string tmp_h5() {
  return std::string(::testing::TempDir()) + "/lod_store_test_" +
         std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + ".cvch5";
}

} // namespace

TEST(LodStore, MeshPyramidRoundTrips) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 28);
  cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, {});
  ASSERT_GE(pyr.rungs.size(), 2u);
  const std::string hash = cvc::lod::mesh_content_hash(src);
  const std::string file = tmp_h5();
  std::remove(file.c_str());

  cvc::lod::write_mesh_pyramid(ctx, file, "buildings", pyr, hash);
  cvc::lod::mesh_pyramid rd = cvc::lod::read_mesh_pyramid(ctx, file, "buildings");

  ASSERT_EQ(rd.rungs.size(), pyr.rungs.size());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    EXPECT_DOUBLE_EQ(rd.world_error_m[k], pyr.world_error_m[k]) << "rung " << k;
    ASSERT_EQ(rd.rungs[k].num_points(), pyr.rungs[k].num_points()) << "rung " << k;
    ASSERT_EQ(rd.rungs[k].num_tris(), pyr.rungs[k].num_tris()) << "rung " << k;
    const auto &pa = pyr.rungs[k].const_points();
    const auto &pb = rd.rungs[k].const_points();
    for (std::size_t i = 0; i < pa.size(); ++i)
      for (int c = 0; c < 3; ++c)
        EXPECT_DOUBLE_EQ(pa[i][c], pb[i][c]);
    const auto &ta = pyr.rungs[k].const_tris();
    const auto &tb = rd.rungs[k].const_tris();
    for (std::size_t i = 0; i < ta.size(); ++i)
      for (int c = 0; c < 3; ++c)
        EXPECT_EQ(ta[i][c], tb[i][c]);
    // uv carried through
    EXPECT_EQ(rd.rungs[k].const_uvs().size(), pyr.rungs[k].const_uvs().size());
  }
  std::remove(file.c_str());
}

TEST(LodStore, ImagePyramidRoundTripsAndIndex) {
  cvc::app ctx;
  image src(16, 16, image::pixel_format::RGB, image::data_type::u8);
  unsigned char *d = src.data();
  for (std::size_t i = 0; i < src.size_bytes(); ++i)
    d[i] = static_cast<unsigned char>((i * 37) & 0xff);
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, {});
  ASSERT_GE(pyr.rungs.size(), 2u);
  const std::string ihash = cvc::lod::image_content_hash(src);
  const std::string file = tmp_h5();
  std::remove(file.c_str());

  // write a mesh too, so the index has two kinds
  geometry m = bumpy_grid(ctx, 16);
  cvc::lod::write_mesh_pyramid(ctx, file, "terrain", cvc::lod::build_mesh_pyramid(m, {}),
                               cvc::lod::mesh_content_hash(m));
  cvc::lod::write_image_pyramid(ctx, file, "satellite", pyr, ihash);

  cvc::lod::image_pyramid rd = cvc::lod::read_image_pyramid(ctx, file, "satellite");
  ASSERT_EQ(rd.rungs.size(), pyr.rungs.size());
  for (std::size_t k = 0; k < pyr.rungs.size(); ++k) {
    ASSERT_EQ(rd.rungs[k].width(), pyr.rungs[k].width());
    ASSERT_EQ(rd.rungs[k].height(), pyr.rungs[k].height());
    ASSERT_EQ(rd.rungs[k].size_bytes(), pyr.rungs[k].size_bytes());
    EXPECT_EQ(0, std::memcmp(rd.rungs[k].data(), pyr.rungs[k].data(), pyr.rungs[k].size_bytes()));
  }

  auto idx = cvc::lod::read_lod_index(ctx, file);
  EXPECT_EQ(idx.size(), 2u);
  bool sawImg = false, sawMesh = false;
  for (const auto &e : idx) {
    if (e.name == "satellite") {
      sawImg = true;
      EXPECT_EQ(e.kind, 'I');
      EXPECT_EQ(e.source_hash, ihash);
      EXPECT_EQ(int(e.world_error_m.size()), e.nrungs);
    }
    if (e.name == "terrain") {
      sawMesh = true;
      EXPECT_EQ(e.kind, 'M');
    }
  }
  EXPECT_TRUE(sawImg && sawMesh);

  EXPECT_TRUE(cvc::lod::has_pyramid(ctx, file, "satellite", ihash));
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "satellite", "deadbeef")); // stale hash
  EXPECT_FALSE(cvc::lod::has_pyramid(ctx, file, "missing", ihash));
  std::remove(file.c_str());
}

TEST(LodStore, InMemoryBlobRoundTrips) {
  cvc::app ctx;
  geometry m = bumpy_grid(ctx, 22);
  cvc::lod::mesh_pyramid mp = cvc::lod::build_mesh_pyramid(m, {});
  image src(12, 12, image::pixel_format::RGBA, image::data_type::u8);
  {
    unsigned char *d = src.data();
    for (std::size_t i = 0; i < src.size_bytes(); ++i)
      d[i] = static_cast<unsigned char>((i * 53) & 0xff);
  }
  cvc::lod::image_pyramid ip = cvc::lod::build_image_pyramid(src, {});

  // Bake to a blob ENTIRELY in memory -- no file ever touched.
  std::vector<unsigned char> blob;
  {
    cvc::lod::scene_writer w(ctx);
    w.write_mesh_pyramid("buildings", mp, cvc::lod::mesh_content_hash(m));
    w.write_image_pyramid("satellite", ip, cvc::lod::image_content_hash(src));
    blob = w.to_blob();
  }
  ASSERT_GT(blob.size(), 8u);
  // The blob is a real HDF5 image: it opens with the HDF5 signature.
  EXPECT_EQ(blob[0], 0x89);
  EXPECT_EQ(blob[1], 'H');
  EXPECT_EQ(blob[2], 'D');
  EXPECT_EQ(blob[3], 'F');

  // Read it back from the blob as if it had been fetched over the network.
  cvc::lod::scene_reader r(ctx, blob.data(), blob.size());
  cvc::lod::mesh_pyramid rm = r.read_mesh_pyramid("buildings");
  ASSERT_EQ(rm.rungs.size(), mp.rungs.size());
  for (std::size_t k = 0; k < mp.rungs.size(); ++k) {
    EXPECT_EQ(rm.rungs[k].num_tris(), mp.rungs[k].num_tris());
    EXPECT_EQ(rm.rungs[k].num_points(), mp.rungs[k].num_points());
    EXPECT_DOUBLE_EQ(rm.world_error_m[k], mp.world_error_m[k]);
  }
  cvc::lod::image_pyramid ri = r.read_image_pyramid("satellite");
  ASSERT_EQ(ri.rungs.size(), ip.rungs.size());
  for (std::size_t k = 0; k < ip.rungs.size(); ++k) {
    ASSERT_EQ(ri.rungs[k].size_bytes(), ip.rungs[k].size_bytes());
    EXPECT_EQ(0, std::memcmp(ri.rungs[k].data(), ip.rungs[k].data(), ip.rungs[k].size_bytes()));
  }
  EXPECT_EQ(r.index().size(), 2u);
  EXPECT_TRUE(r.has("buildings", cvc::lod::mesh_content_hash(m)));
  EXPECT_FALSE(r.has("buildings", "nope"));
}
