/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::world::generate_world — terrain_lab's world, headless: deterministic, the ground mesh is the
// relief, props and buildings stand on dry land, the water fills what is below water_level, and
// the world_geometry carries the terrain and the buildings for physics and nav.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/world/generated_world.h>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <vector>

using namespace cvc::world;

namespace {

// A small box-massing world (no building library), quick to generate.
generated_world_params small_world() {
  generated_world_params p;
  p.seed = 11;
  p.half_m = 200.0;
  p.trees = 40;
  p.rocks = 10;
  p.buildings = 16;
  p.tree_gen = 3;
  p.amp_m = 40.0;
  p.water_level_m = 14.0;
  return p;
}

} // namespace

TEST(WorldGenerated, DeterministicAndTerrainIsTheRelief) {
  const generated_world a = generate_world(small_world()), b = generate_world(small_world());
  ASSERT_TRUE(a.terrain);
  EXPECT_EQ(a.terrain->rows(), 129); // 400 m at 5 m cells is 81 samples, held at the 129 floor
  EXPECT_EQ(a.terrain->const_points().size(), b.terrain->const_points().size());
  EXPECT_EQ(a.props.const_points().size(), b.props.const_points().size());
  EXPECT_EQ(a.buildings.const_points().size(), b.buildings.const_points().size());
  EXPECT_EQ(a.building_count, b.building_count);
  for (std::size_t i = 0; i < a.terrain->const_points().size(); i += 97)
    EXPECT_EQ(a.terrain->const_points()[i][2], b.terrain->const_points()[i][2]);
  // The mesh samples are the model's relief, and height() is the mesh.
  const heightfield &hf = a.model->hf();
  for (double x : {-150.0, -37.5, 0.0, 62.5, 150.0})
    for (double y : {-125.0, 0.0, 100.0}) {
      const double step = 400.0 / 128.0;
      const double sx = std::round((x + 200.0) / step) * step - 200.0;
      const double sy = std::round((y + 200.0) / step) * step - 200.0;
      EXPECT_NEAR(a.height(sx, sy), hf.sample(sx, sy), 1e-3) << sx << "," << sy;
    }
}

TEST(WorldGenerated, BoxMassingCityOnDryLand) {
  const generated_world w = generate_world(small_world());
  EXPECT_FALSE(w.archetypes);
  EXPECT_TRUE(w.roads.const_points().empty()); // box massing paints its streets, extrudes none
  EXPECT_GT(w.building_count, 0u);
  EXPECT_EQ(w.aprons.size(), w.building_count);
  EXPECT_FALSE(w.props.const_points().empty());
  const double dry = small_world().water_level_m + 1.0;
  for (const placed_prop &pp : w.model->props())
    EXPECT_GE(pp.z, dry - 1e-6) << pp.recipe << " at " << pp.x << "," << pp.y;
  for (const generated_world::street &s : w.streets)
    EXPECT_GE(w.model->hf().sample(0.5 * (s.x0 + s.x1), 0.5 * (s.y0 + s.y1)),
              small_world().water_level_m);
}

TEST(WorldGenerated, WaterFillsBelowTheLevel) {
  const generated_world w = generate_world(small_world());
  const int n = water_resolution(w), nz = 22;
  EXPECT_EQ(n, 80); // 400 m at ~8 m a sample, held at the 80 floor
  const std::vector<float> f = water_field(w, 0.0, n, nz);
  ASSERT_EQ(f.size(), std::size_t(n) * n * nz);
  const double half = w.params.half_m, z0 = water_floor(w), z1 = water_top(w);
  int wet = 0, dryChecked = 0;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double x = -half + 2.0 * half * i / (n - 1), y = -half + 2.0 * half * j / (n - 1);
      const double g = w.model->hf().sample(x, y);
      double column = 0.0;
      for (int k = 0; k < nz; ++k)
        column += f[std::size_t(k) * n * n + std::size_t(j) * n + i];
      if (g > w.params.water_level_m + 1.0) { // dry land: no water anywhere above it
        EXPECT_EQ(column, 0.0) << x << "," << y;
        ++dryChecked;
      } else if (g < w.params.water_level_m - 4.0) {
        EXPECT_GT(column, 0.0) << x << "," << y;
        ++wet;
      }
    }
  EXPECT_GT(wet, 0);
  EXPECT_GT(dryChecked, 0);
  EXPECT_LT(z0, w.params.water_level_m);
  EXPECT_GT(z1, w.params.water_level_m);
}

TEST(WorldGenerated, SurfaceTextureAndGeometry) {
  const generated_world w = generate_world(small_world());
  const std::vector<unsigned char> img = paint_surface(w, surface_color::material, 256);
  EXPECT_EQ(img.size(), 256u * 256u * 3u);
  const std::vector<unsigned char> occ = paint_surface(w, surface_color::occupancy, 64);
  EXPECT_EQ(occ.size(), 64u * 64u * 3u);

  const world_geometry g = w.geometry();
  ASSERT_NE(g.terrain(), nullptr);
  EXPECT_EQ(g.terrain()->rows(), w.terrain->rows());
  const geometry_item *b = g.find("buildings");
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(b->has(role_collide));
  const geometry_item *p = g.find("props");
  ASSERT_NE(p, nullptr);
  EXPECT_FALSE(p->has(role_collide));
  EXPECT_EQ(g.find("roads"), nullptr); // box massing: no road surface
}

TEST(WorldGenerated, PresetsQuickSetCounts) {
  generated_world_params p;
  p.preset = world_preset_from_name("forest");
  EXPECT_EQ(p.preset, int(world_preset::forest));
  apply_world_preset(p);
  EXPECT_EQ(p.trees, 1600);
  EXPECT_EQ(p.buildings, 8);
  EXPECT_STREQ(world_preset_name(p.preset), "forest");
  EXPECT_EQ(world_preset_from_name("nonsense"), 0);
  p.preset = int(world_preset::island);
  EXPECT_TRUE(to_world_params(p).hf.island);
}

// Trees are tree-sized in metres (the island-scaled pine once stood 20..100 m over one- and
// two-storey buildings): nothing in the props or needles rises 35 m over the ground under it.
TEST(WorldGenerated, TreesAreTreeSized) {
  generated_world_params p = small_world();
  p.trees = 120;
  const generated_world w = generate_world(p);
  double tallest = 0.0;
  for (const cvc::geometry *g : {&w.props, &w.needles, &w.foliage})
    for (const auto &q : g->const_points())
      tallest = std::max(tallest, q[2] - w.model->hf().sample(q[0], q[1]));
  EXPECT_GT(tallest, 5.0);
  EXPECT_LT(tallest, 35.0);
}

// The raster is coarse (here ~3 m a cell under a 256-texel texture of 1.6 m texels), and the
// texture must not paint its cells as squares: risk, one value per cell, is interpolated (many
// more colours than cells' values), and in material mode a texel's colour does not hold constant
// across a cell (the cells blend at a noise-warped position). Painting is deterministic.
TEST(WorldGenerated, SurfaceIsInterpolatedNotCells) {
  const generated_world w = generate_world(small_world());
  std::set<float> risks(w.raster.risk_raw.begin(), w.raster.risk_raw.end());
  const std::vector<unsigned char> risk = paint_surface(w, surface_color::risk, 256);
  std::set<std::uint32_t> colours;
  for (std::size_t i = 0; i + 2 < risk.size(); i += 3)
    colours.insert(std::uint32_t(risk[i]) << 16 | std::uint32_t(risk[i + 1]) << 8 | risk[i + 2]);
  EXPECT_GT(colours.size(), 4 * risks.size());

  // Material: of the texels falling in one raster cell, how often do all share one colour? With
  // nearest-cell painting a non-grass cell was one flat colour; blended, almost none are.
  const int T = 256;
  const std::vector<unsigned char> mat = paint_surface(w, surface_color::material, T);
  EXPECT_EQ(mat, paint_surface(w, surface_color::material, T));
  const int R = w.grid.rows, C = w.grid.cols;
  std::map<std::size_t, std::set<std::uint32_t>> perCell;
  for (int ty = 0; ty < T; ++ty)
    for (int tx = 0; tx < T; ++tx) {
      const int r = int(std::lround(double(ty) / (T - 1) * (R - 1)));
      const int c = int(std::lround(double(tx) / (T - 1) * (C - 1)));
      const std::size_t o = (std::size_t(ty) * T + tx) * 3;
      perCell[std::size_t(r) * C + c].insert(std::uint32_t(mat[o]) << 16 |
                                             std::uint32_t(mat[o + 1]) << 8 | mat[o + 2]);
    }
  // Only the cells on a boundary between classes (not grass, whose dirt breakup varies anyway):
  // those were one flat colour, and now blend into their neighbours.
  auto klass = [&](int r, int c) { return w.raster.klass[std::size_t(r) * C + c]; };
  auto grass = [](std::uint16_t k) { return k == 6 || k == 7 || k == 8; };
  std::size_t flat = 0, multi = 0;
  for (int r = 1; r + 1 < R; ++r)
    for (int c = 1; c + 1 < C; ++c) {
      const std::uint16_t k = klass(r, c);
      if (grass(k) || (klass(r - 1, c) == k && klass(r + 1, c) == k && klass(r, c - 1) == k &&
                       klass(r, c + 1) == k))
        continue;
      (perCell[std::size_t(r) * C + c].size() == 1 ? flat : multi) += 1;
    }
  EXPECT_GT(multi, 0u);
  EXPECT_GT(multi, 4 * flat);
}
