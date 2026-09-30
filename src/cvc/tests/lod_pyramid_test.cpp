// lod_pyramid_test -- the per-asset LOD ladder builders (cvc::lod::build_*_pyramid).
//
// Covers: a mesh ladder that strictly coarsens with a monotone world-error ladder
// and a verbatim rung 0; pooled == serial; a volume ladder that halves each axis
// with a growing voxel-size error; and an image mip ladder whose box filter
// actually area-averages (a 2x2 checker becomes mid-grey) rather than point-samples.

#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/thread_pool.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/lod/pyramid.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>

using cvc::geometry;
using cvc::image;
using cvc::volume;

namespace {

geometry bumpy_grid(cvc::app &ctx, int n) {
  geometry g(ctx);
  geometry::points_t &P = g.points();
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      P.push_back({double(i), double(j), 1.4 * std::sin(0.6 * i) * std::cos(0.8 * j)});
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

// A ladder's error column must be monotone non-decreasing and start at 0.
void expect_monotone(const std::vector<double> &e) {
  ASSERT_FALSE(e.empty());
  EXPECT_EQ(e.front(), 0.0);
  for (std::size_t k = 1; k < e.size(); ++k)
    EXPECT_GE(e[k], e[k - 1]) << "rung " << k;
}

} // namespace

TEST(LodPyramid, MeshLadderCoarsens) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 30);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  cvc::lod::mesh_pyramid pyr = cvc::lod::build_mesh_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs.size(), pyr.world_error_m.size());
  EXPECT_EQ(pyr.rungs[0].num_tris(), src.num_tris()); // rung 0 is the source, verbatim
  expect_monotone(pyr.world_error_m);
  // Triangle count strictly decreases down the ladder.
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    EXPECT_LT(pyr.rungs[k].num_tris(), pyr.rungs[k - 1].num_tris()) << "rung " << k;
}

TEST(LodPyramid, MeshLadderPoolMatchesSerial) {
  cvc::app ctx;
  geometry src = bumpy_grid(ctx, 44);
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 4;
  cvc::lod::mesh_pyramid s = cvc::lod::build_mesh_pyramid(src, pp, nullptr);
  cvc::lod::mesh_pyramid p = cvc::lod::build_mesh_pyramid(src, pp, &ctx.computePool());
  ASSERT_EQ(s.rungs.size(), p.rungs.size());
  for (std::size_t k = 0; k < s.rungs.size(); ++k) {
    EXPECT_EQ(s.rungs[k].num_tris(), p.rungs[k].num_tris()) << "rung " << k;
    EXPECT_DOUBLE_EQ(s.world_error_m[k], p.world_error_m[k]) << "rung " << k;
  }
}

TEST(LodPyramid, VolumeLadderHalves) {
  cvc::app ctx;
  volume src(ctx, cvc::dimension(64, 64, 64), cvc::Float, cvc::bounding_box(0, 0, 0, 1, 1, 1));
  cvc::lod::pyramid_params pp;
  pp.max_rungs = 3;
  pp.vol_min_dim = 4;
  cvc::lod::volume_pyramid pyr = cvc::lod::build_volume_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs[0].XDim(), 64u); // rung 0 is the source
  expect_monotone(pyr.world_error_m);
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k) {
    EXPECT_LT(pyr.rungs[k].XDim(), pyr.rungs[k - 1].XDim()) << "rung " << k;
    EXPECT_GT(pyr.world_error_m[k], 0.0);
  }
}

TEST(LodPyramid, ImageMipBoxAverages) {
  cvc::app ctx;
  // 4x4 GRAY checker of 0/255 in 2x2 super-cells; a box downscale to 2x2 must
  // average each 2x2 block to ~128 (nearest would keep 0/255).
  image src(4, 4, image::pixel_format::GRAY, image::data_type::u8);
  unsigned char *d = src.data();
  for (int y = 0; y < 4; ++y)
    for (int x = 0; x < 4; ++x)
      d[y * 4 + x] = ((x / 1 + y / 1) % 2) ? 255 : 0; // fine checker

  cvc::lod::pyramid_params pp;
  pp.max_rungs = 2;
  pp.img_min_dim = 1;
  cvc::lod::image_pyramid pyr = cvc::lod::build_image_pyramid(src, pp);

  ASSERT_GE(pyr.rungs.size(), 2u);
  EXPECT_EQ(pyr.rungs[0].width(), 4); // rung 0 = source
  expect_monotone(pyr.world_error_m);
  // rung 1 is 2x2; each pixel is a 2x2 box average of the checker => ~128.
  const image &r1 = pyr.rungs[1];
  ASSERT_EQ(r1.width(), 2);
  ASSERT_EQ(r1.height(), 2);
  const unsigned char *p = r1.data();
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(int(p[i]), 128, 2) << "box average at " << i;
  for (std::size_t k = 1; k < pyr.rungs.size(); ++k)
    EXPECT_LT(pyr.rungs[k].width(), pyr.rungs[k - 1].width());
}
