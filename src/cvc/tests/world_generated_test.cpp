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
#include <cstdio>
#include <cvc/world/generated_world.h>
#include <cvc/world/npy.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::world;

namespace {

// A small world (no building library: procedural buildings), quick to generate.
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
  const terrain_source &hf = a.model->terrain();
  for (double x : {-150.0, -37.5, 0.0, 62.5, 150.0})
    for (double y : {-125.0, 0.0, 100.0}) {
      const double step = 400.0 / 128.0;
      const double sx = std::round((x + 200.0) / step) * step - 200.0;
      const double sy = std::round((y + 200.0) / step) * step - 200.0;
      EXPECT_NEAR(a.height(sx, sy), hf.sample(sx, sy), 1e-3) << sx << "," << sy;
    }
}

// The city is roads first with or without a building library: district cores, an extruded street
// network, procedural buildings along it (no more than asked for), and props on dry land, none of
// them on a road or in a building.
TEST(WorldGenerated, RoadsFirstCityWithoutALibrary) {
  const generated_world w = generate_world(small_world());
  EXPECT_FALSE(w.archetypes);
  EXPECT_FALSE(w.roads.const_points().empty());
  EXPECT_GE(w.districts, 1u);
  EXPECT_GT(w.road_length, 200.0);
  EXPECT_GT(w.building_count, 0u);
  EXPECT_LE(w.building_count, std::size_t(small_world().buildings));
  EXPECT_FALSE(w.props.const_points().empty());
  const double dry = small_world().water_level_m + 1.0;
  for (const placed_prop &pp : w.model->props())
    EXPECT_GE(pp.z, dry - 1e-6) << pp.recipe << " at " << pp.x << "," << pp.y;

  generated_world_params none = small_world();
  none.buildings = 0;
  const generated_world bare = generate_world(none);
  EXPECT_EQ(bare.building_count, 0u);
  EXPECT_TRUE(bare.buildings.const_points().empty());
  EXPECT_EQ(bare.road_length, w.road_length); // the roads do not depend on the buildings
  // The trees the city displaced: the bare world keeps more of them.
  EXPECT_GE(bare.props.const_points().size(), w.props.const_points().size());
}

// Buildings are building-sized: town houses and low blocks (a few storeys, the odd tower in a
// core), standing on the ground (their base at or below the ground under them), and none deep in
// the water.
TEST(WorldGenerated, BuildingsAreBuildingSized) {
  generated_world_params p; // the lab's default world (fewer trees)
  p.trees = 60;
  p.tree_gen = 3;
  p.buildings = 400;
  const generated_world w = generate_world(p);
  ASSERT_GT(w.building_count, 40u);
  double tallest = 0.0, lowest = 1e30;
  std::vector<double> tops; // the walls' and roofs' upper corners, over the ground
  for (const auto &q : w.buildings.const_points()) {
    const double above = q[2] - w.model->terrain().sample(q[0], q[1]);
    tallest = std::max(tallest, above);
    lowest = std::min(lowest, double(q[2]));
    if (above > 1.0)
      tops.push_back(above);
  }
  EXPECT_LE(tallest, 95.0);
  EXPECT_GT(tallest, 6.0);
  EXPECT_GE(lowest, p.water_level_m + 0.5 - 1e-6); // the base, half a metre into dry ground
  // Most roofs are low: the median top over the ground is a few storeys.
  ASSERT_FALSE(tops.empty());
  std::nth_element(tops.begin(), tops.begin() + tops.size() / 2, tops.end());
  const double median = tops[tops.size() / 2];
  EXPECT_GT(median, 3.0);
  EXPECT_LT(median, 20.0);
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
      const double g = w.model->terrain().sample(x, y);
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
  ASSERT_NE(g.find("roads"), nullptr);
  EXPECT_FALSE(g.find("roads")->has(role_collide));
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
      tallest = std::max(tallest, q[2] - w.model->terrain().sample(q[0], q[1]));
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

// The seed shapes the relief. (The fBm seed used to be XORed with the world seed, which was the
// same number, so every seed stood on the same ground.)
TEST(WorldGenerated, TheSeedChangesTheTerrain) {
  generated_world_params p = small_world();
  const std::shared_ptr<const terrain_source> a = make_terrain(p);
  p.seed = 12;
  const std::shared_ptr<const terrain_source> b = make_terrain(p);
  double diff = 0.0;
  for (double x = -180.0; x <= 180.0; x += 30.0)
    for (double y = -180.0; y <= 180.0; y += 30.0)
      diff = std::max(diff, std::fabs(a->sample(x, y) - b->sample(x, y)));
  EXPECT_GT(diff, 5.0);
}

// The generators are chosen by name: the built-ins, a host's own, and an unknown name refused.
TEST(WorldGenerated, GeneratorsByName) {
  const std::vector<std::string> t = terrain_generator_names(), c = city_generator_names();
  ASSERT_GE(t.size(), 2u);
  EXPECT_EQ(t[0], "fbm");
  EXPECT_EQ(t[1], "dem");
  ASSERT_GE(c.size(), 2u);
  EXPECT_EQ(c[0], "districts");
  EXPECT_EQ(c[1], "none");

  generated_world_params p = small_world();
  p.terrain = "no_such_terrain";
  EXPECT_THROW(generate_world(p), std::invalid_argument);
  p.terrain = "fbm";
  p.city = "no_such_city";
  EXPECT_THROW(generate_world(p), std::invalid_argument);
  p.terrain = "dem"; // without a file
  p.city = "districts";
  EXPECT_THROW(generate_world(p), std::invalid_argument);

  // No city: no streets, no buildings, the props still scattered.
  p = small_world();
  p.city = "none";
  const generated_world bare = generate_world(p);
  EXPECT_EQ(bare.building_count, 0u);
  EXPECT_EQ(bare.road_length, 0.0);
  EXPECT_EQ(bare.districts, 0u);
  EXPECT_FALSE(bare.props.const_points().empty());

  // A host's terrain: a tilted plane, flat enough to build on, under the water at its low edge.
  register_terrain_generator("test_tilt", [](const generated_world_params &q) {
    const int n = 33;
    std::vector<float> h;
    for (int r = 0; r < n; ++r)
      for (int col = 0; col < n; ++col)
        h.push_back(float(10.0 + 20.0 * col / (n - 1)));
    return std::make_shared<const raster_heightfield>(n, n, -q.half_m, -q.half_m, q.half_m,
                                                      q.half_m, std::move(h), 0.0, 30.0);
  });
  p = small_world();
  p.terrain = "test_tilt";
  const generated_world tilt = generate_world(p);
  EXPECT_NEAR(tilt.height(-200.0, 0.0), 10.0, 1e-3);
  EXPECT_NEAR(tilt.height(200.0, 0.0), 30.0, 1e-3);
  EXPECT_GT(tilt.road_length, 100.0);
  EXPECT_GT(tilt.building_count, 0u);
}

// A DEM file through the "dem" generator: the world's ground is the file's relief, anchored at 0
// and stretched to amp_m.
TEST(WorldGenerated, DemTerrainFromAFile) {
  const std::string f =
      (std::filesystem::temp_directory_path() / "cvc_world_generated_test_dem.npy").string();
  const int n = 65;
  std::vector<float> h;
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h.push_back(float(500.0 + 0.5 * c + 0.25 * r)); // a gentle slope, 500 m above the datum
  write_npy(f, h, n, n);
  generated_world_params p = small_world();
  p.terrain = "dem";
  p.terrain_file = f;
  p.amp_m = 24.0;
  p.water_level_m = 4.0;
  const generated_world w = generate_world(p);
  std::remove(f.c_str());
  EXPECT_NEAR(w.height(-200.0, -200.0), 0.0, 1e-3);
  EXPECT_NEAR(w.height(200.0, 200.0), 24.0, 1e-3);
  EXPECT_NEAR(w.height(200.0, -200.0), 16.0, 1e-3);
  EXPECT_GT(w.road_length, 100.0);
}

// A generator's channel mask keeps the streets and lots off it, though the ground there is flat
// and dry.
TEST(WorldGenerated, ChannelMaskKeepsTheCityOff) {
  register_terrain_generator("test_channel", [](const generated_world_params &q) {
    const int n = 41;
    std::vector<float> h(std::size_t(n) * n, 20.0f), m(std::size_t(n) * n, 0.0f);
    for (int r = 0; r < n; ++r)
      for (int c = n / 2; c < n; ++c)
        m[std::size_t(r) * n + c] = 1.0f; // the whole east half (x > 0)
    auto t = std::make_shared<raster_heightfield>(n, n, -q.half_m, -q.half_m, q.half_m, q.half_m,
                                                  std::move(h), 0.0, 40.0);
    t->set_mask(terrain_mask::channel, std::move(m));
    return std::shared_ptr<const terrain_source>(t);
  });
  generated_world_params p = small_world();
  p.terrain = "test_channel";
  p.buildings = 200;
  const generated_world w = generate_world(p);
  EXPECT_GT(w.building_count, 0u);
  // Every building vertex is west of the mask's 50 % line (x = -5 m: the mask ramps over a cell),
  // give or take a footprint's half-width.
  double east = -1e9;
  for (const auto &q : w.buildings.const_points())
    east = std::max(east, double(q[0]));
  EXPECT_LT(east, 25.0);
  for (const auto &q : w.roads.const_points())
    EXPECT_LT(q[0], 15.0);
}

// The city's buildings are in the raster: occupied, hard, concrete-wall cells, at least one per
// building (they used to be left out, so the occupancy raster showed open ground under them).
TEST(WorldGenerated, BuildingsAreInTheRaster) {
  generated_world_params p = small_world();
  p.buildings = 40;
  const generated_world w = generate_world(p);
  ASSERT_GT(w.building_count, 0u);
  const std::uint16_t wall = w.model->registry().class_for_role(cvc::lsys::role::wall_concrete);
  std::size_t walls = 0;
  for (std::size_t i = 0; i < w.raster.klass.size(); ++i)
    if (w.raster.klass[i] == wall) {
      ++walls;
      EXPECT_EQ(w.raster.occupancy[i], 1);
      EXPECT_EQ(w.raster.hard[i], w.model->registry()[wall].hard ? 1 : 0);
    }
  EXPECT_GE(walls, w.building_count);
  p.city = "none";
  EXPECT_LT(generate_world(p).raster.occupied_count(), w.raster.occupied_count());
}

// world_builder steps generate_world a stage at a time: stepped in small budgets it makes the same
// world, through every stage, its progress rising to 1; surface_painter paints the same texture a
// few rows a step; and the ground under the water is cached with the world.
TEST(WorldGenerated, SteppedBuildIsTheSameWorld) {
  const generated_world_params p = small_world();
  const generated_world a = generate_world(p);
  world_builder b(p);
  std::set<std::string> phases;
  double last = -1.0;
  int steps = 0;
  while (!b.done()) {
    phases.insert(b.phase());
    b.step(2000);
    EXPECT_GE(b.progress(), last);
    last = b.progress();
    ASSERT_LT(++steps, 100000);
  }
  EXPECT_DOUBLE_EQ(b.progress(), 1.0);
  EXPECT_GT(steps, 20); // the row stages took many steps
  for (const char *s : {"terrain", "scatter", "raster", "mesh", "materials", "streets", "city",
                        "roads", "props", "water"})
    EXPECT_EQ(phases.count(s), 1u) << s;
  const generated_world w = b.take();
  ASSERT_TRUE(w.terrain);
  ASSERT_EQ(w.terrain->const_points().size(), a.terrain->const_points().size());
  for (std::size_t i = 0; i < a.terrain->const_points().size(); ++i)
    ASSERT_EQ(w.terrain->const_points()[i][2], a.terrain->const_points()[i][2]) << i;
  EXPECT_EQ(w.raster.klass, a.raster.klass);
  EXPECT_EQ(w.raster.height, a.raster.height);
  EXPECT_EQ(w.building_count, a.building_count);
  EXPECT_EQ(w.buildings.const_points().size(), a.buildings.const_points().size());
  EXPECT_EQ(w.roads.const_points().size(), a.roads.const_points().size());
  EXPECT_EQ(w.props.const_points().size(), a.props.const_points().size());
  EXPECT_EQ(w.water_ground, a.water_ground);
  const int n = water_resolution(w);
  ASSERT_EQ(w.water_ground.size(), std::size_t(n) * n);
  EXPECT_NEAR(w.water_ground[std::size_t(n) * n - 1], w.model->terrain().sample(200.0, 200.0),
              1e-3);

  const std::vector<unsigned char> whole = paint_surface(w, surface_color::material, 256);
  surface_painter sp(w, surface_color::material, 256);
  int rows = 0;
  while (!sp.step(256 * 3)) // three rows a step
    ASSERT_LT(++rows, 1000);
  EXPECT_GT(rows, 50);
  EXPECT_EQ(sp.take(), whole);
}

// A stepped build fails where generate_world would, and keeps failing.
TEST(WorldGenerated, SteppedBuildRethrows) {
  generated_world_params p = small_world();
  p.terrain = "no_such";
  world_builder b(p);
  EXPECT_THROW(b.step(1000), std::invalid_argument);
  EXPECT_THROW(b.step(1000), std::invalid_argument);
  EXPECT_FALSE(b.done());
}
