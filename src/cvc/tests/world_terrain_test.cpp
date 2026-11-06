/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  Licensed under the GNU LGPL v2.1 (see cvc/world/heightfield.h for the full header).
*/

// The terrain_source seam: the gridded relief, rasterizing any source, the .npy reader and a DEM.

#include <cmath>
#include <cstdio>
#include <cvc/world/grid.h>
#include <cvc/world/heightfield.h>
#include <cvc/world/npy.h>
#include <cvc/world/raster.h>
#include <cvc/world/terrain_source.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::world;

namespace {

// A 3 x 4 ramp over [0, 30] x [0, 20]: h = x + 2 y (exactly bilinear).
raster_heightfield ramp() {
  std::vector<float> h;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c)
      h.push_back(float(c * 10.0 + 2.0 * r * 10.0));
  return raster_heightfield(3, 4, 0.0, 0.0, 30.0, 20.0, std::move(h));
}

std::string temp_path(const char *name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

} // namespace

TEST(WorldTerrain, RasterIsBilinearAndClampedToItsEdge) {
  const raster_heightfield t = ramp();
  EXPECT_DOUBLE_EQ(t.sample(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(t.sample(30.0, 20.0), 70.0);
  EXPECT_NEAR(t.sample(12.5, 7.0), 12.5 + 14.0, 1e-9); // between samples: the plane
  EXPECT_DOUBLE_EQ(t.sample(-50.0, 0.0), 0.0);         // outside: the edge
  EXPECT_DOUBLE_EQ(t.sample(99.0, 99.0), 70.0);
  const terrain_bounds b = t.bounds();
  EXPECT_FALSE(b.unbounded);
  EXPECT_DOUBLE_EQ(b.max_x, 30.0);
  EXPECT_DOUBLE_EQ(b.max_y, 20.0);
  // The datum: no relief given, so the samples' range.
  EXPECT_DOUBLE_EQ(t.relief_m(), 70.0);
  EXPECT_DOUBLE_EQ(t.sea_level_m(), 0.0);
  // Slope of h = x + 2 y: atan(sqrt(5)), taken over at least a cell.
  EXPECT_NEAR(t.slope_deg(15.0, 10.0), std::atan(std::sqrt(5.0)) * 180.0 / M_PI, 1e-9);
  EXPECT_THROW(raster_heightfield(1, 4, 0, 0, 1, 1, std::vector<float>(4)), std::invalid_argument);
  EXPECT_THROW(raster_heightfield(2, 2, 0, 0, 1, 1, std::vector<float>(3)), std::invalid_argument);
}

TEST(WorldTerrain, MasksAreOptionalPlanes) {
  raster_heightfield t = ramp();
  EXPECT_FALSE(t.has_mask(terrain_mask::channel));
  EXPECT_DOUBLE_EQ(t.mask(terrain_mask::channel, 5.0, 5.0), 0.0);
  std::vector<float> m(12, 0.0f);
  m[3] = m[7] = m[11] = 2.0f; // the x = 30 column, clamped to 1
  t.set_mask(terrain_mask::channel, m);
  EXPECT_TRUE(t.has_mask(terrain_mask::channel));
  EXPECT_FALSE(t.has_mask(terrain_mask::cliff));
  EXPECT_DOUBLE_EQ(t.mask(terrain_mask::channel, 30.0, 10.0), 1.0);
  EXPECT_DOUBLE_EQ(t.mask(terrain_mask::channel, 25.0, 10.0), 0.5);
  EXPECT_DOUBLE_EQ(t.mask(terrain_mask::channel, 0.0, 10.0), 0.0);
  EXPECT_THROW(t.set_mask(terrain_mask::cliff, std::vector<float>(5)), std::invalid_argument);
}

// Any source onto a grid: the analytic relief sampled exactly at the grid's points, its datum kept.
TEST(WorldTerrain, SampleTerrainRasterizesAnySource) {
  heightfield_params p;
  p.seed = 5;
  p.amp_m = 12.0;
  p.sea_level_m = 1.5;
  const heightfield hf(p);
  const grid_spec g = grid_spec::window(0.0, 0.0, 40.0, 2.0);
  const raster_heightfield r = sample_terrain(hf, g);
  EXPECT_EQ(r.rows(), g.rows);
  EXPECT_DOUBLE_EQ(r.sea_level_m(), 1.5);
  EXPECT_DOUBLE_EQ(r.relief_m(), 12.0);
  for (int i = 0; i < g.rows; i += 7)
    for (int j = 0; j < g.cols; j += 5)
      EXPECT_NEAR(r.sample(g.world_x(j), g.world_y(i)), hf.sample(g.world_x(j), g.world_y(i)),
                  1e-4);
  // Between the points it is close to the relief (a 2 m grid on metre-scale fBm).
  double worst = 0.0;
  for (double x = -35.0; x < 35.0; x += 3.3)
    worst = std::max(worst, std::fabs(r.sample(x, 0.7 * x) - hf.sample(x, 0.7 * x)));
  EXPECT_LT(worst, 1.0);
}

TEST(WorldTerrain, NpyRoundTripsAndRejectsBadFiles) {
  const std::string f = temp_path("cvc_world_terrain_test_f4.npy");
  const std::vector<float> v = {0.5f, -1.25f, 3.0f, 7.0f, 11.0f, 13.5f};
  write_npy(f, v, 2, 3);
  int rows = 0, cols = 0;
  EXPECT_EQ(read_npy(f, rows, cols), v);
  EXPECT_EQ(rows, 2);
  EXPECT_EQ(cols, 3);
  const std::string u = temp_path("cvc_world_terrain_test_u2.npy");
  write_npy(u, std::vector<std::uint16_t>{1, 2, 65535, 4}, 2, 2);
  const std::vector<float> w = read_npy(u, rows, cols);
  ASSERT_EQ(w.size(), 4u);
  EXPECT_FLOAT_EQ(w[2], 65535.0f);
  EXPECT_THROW(read_npy(temp_path("cvc_world_terrain_test_missing.npy"), rows, cols),
               std::runtime_error);
  const std::string bad = temp_path("cvc_world_terrain_test_bad.npy");
  if (std::FILE *fp = std::fopen(bad.c_str(), "wb")) {
    std::fputs("not numpy at all", fp);
    std::fclose(fp);
  }
  EXPECT_THROW(read_npy(bad, rows, cols), std::runtime_error);
  std::remove(f.c_str());
  std::remove(u.c_str());
  std::remove(bad.c_str());
}

// A DEM is laid over the world, its floor anchored at 0 and its relief stretched to the one asked.
TEST(WorldTerrain, DemIsAnchoredAndStretched) {
  const std::string f = temp_path("cvc_world_terrain_test_dem.npy");
  write_npy(f, std::vector<float>{100.0f, 110.0f, 120.0f, 140.0f}, 2, 2);
  const raster_heightfield a = load_dem(f, 200.0, 80.0);
  EXPECT_DOUBLE_EQ(a.sample(-200.0, -200.0), 0.0);
  EXPECT_DOUBLE_EQ(a.sample(200.0, 200.0), 80.0);
  EXPECT_DOUBLE_EQ(a.sample(200.0, -200.0), 20.0);
  EXPECT_DOUBLE_EQ(a.relief_m(), 80.0);
  const raster_heightfield b = load_dem(f, 200.0, 0.0); // the file's metres, only shifted
  EXPECT_DOUBLE_EQ(b.sample(200.0, 200.0), 40.0);
  EXPECT_DOUBLE_EQ(b.relief_m(), 40.0);
  std::remove(f.c_str());
  EXPECT_THROW(load_dem(f, 200.0, 80.0), std::runtime_error);
}

// world_model and raster() read the source they are given, not a heightfield of their own.
TEST(WorldTerrain, WorldModelTakesAnySource) {
  world_params wp;
  wp.min_x = wp.min_y = 0.0;
  wp.max_x = 30.0;
  wp.max_y = 20.0;
  wp.sc.tree_count = wp.sc.rock_count = wp.sc.building_count = 0;
  wp.terrain = std::make_shared<const raster_heightfield>(ramp());
  const world_model wm = world_model::generate(wp);
  EXPECT_EQ(&wm.terrain(), wp.terrain.get());
  grid_spec g;
  g.rows = 5;
  g.cols = 7;
  g.min_x = g.min_y = 0.0;
  g.max_x = 30.0;
  g.max_y = 20.0;
  raster_out o;
  raster(wm, g, o);
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c)
      EXPECT_NEAR(o.height[std::size_t(r) * g.cols + c], g.world_x(c) + 2.0 * g.world_y(r), 1e-4);
}
