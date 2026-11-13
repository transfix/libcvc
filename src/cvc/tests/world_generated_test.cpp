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
#include <cvc/world/road_grade.h>
#include <cvc/world/road_raster.h>
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
  p.water_level_m = 0.0; // sea level
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

// Sea level is 0 m whatever the relief, so no preset drowns its world or drains it: the urban
// preset's 10 m relief under the document's old 34 m water level was a world of sea. The fbm
// land's floor sits world_sea_share of the relief under the sea; the island stands in it.
TEST(WorldGenerated, EveryPresetHasLandAndWater) {
  for (int preset = 0; preset < 5; ++preset) {
    generated_world_params p = small_world();
    p.preset = preset;
    apply_world_preset(p);
    EXPECT_EQ(p.water_level_m, 0.0) << world_preset_name(preset);
    const generated_world w = generate_world(p);
    std::size_t dry = 0;
    const auto &h = w.terrain->heights();
    for (float z : h)
      dry += z > p.water_level_m;
    const double share = double(dry) / double(h.size());
    const bool island = preset == int(world_preset::island);
    EXPECT_GT(share, island ? 0.05 : 0.6) << world_preset_name(preset) << ": " << share << " dry";
    EXPECT_LT(share, island ? 0.5 : 0.995) << world_preset_name(preset) << ": " << share << " dry";
    if (!island) // the same land under every relief: its floor the same share of it below the sea
      EXPECT_NEAR(*std::min_element(h.begin(), h.end()) / p.amp_m, 0.15 - world_sea_share, 0.05);
    EXPECT_GT(w.building_count + w.prop_cells.size(), 0u) << world_preset_name(preset);
  }
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
  // The ground is the generator's, wherever no road grades it.
  const auto *graded = dynamic_cast<const graded_terrain *>(&tilt.model->terrain());
  ASSERT_NE(graded, nullptr);
  int checked = 0;
  for (double y = -190.0; y <= 190.0; y += 20.0)
    for (double x : {-195.0, 195.0})
      if (!graded->graded_at(x, y)) {
        EXPECT_NEAR(tilt.height(x, y),
                    x < 0 ? 10.0 + 20.0 * 5.0 / 400.0 : 30.0 - 20.0 * 5.0 / 400.0, 1e-3)
            << x << "," << y;
        ++checked;
      }
  EXPECT_GT(checked, 4);
  EXPECT_GT(tilt.road_length, 100.0);
  EXPECT_GT(tilt.building_count, 0u);
}

// A DEM file through the "dem" generator: the world's ground is the file's relief, stretched to
// amp_m and anchored as every generator's is, world_sea_share of it below sea level.
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
  const generated_world w = generate_world(p);
  std::remove(f.c_str());
  const double floor = -world_sea_share * 24.0;
  EXPECT_NEAR(w.height(-200.0, -200.0), floor, 1e-3);
  EXPECT_NEAR(w.height(200.0, 200.0), floor + 24.0, 1e-3);
  EXPECT_NEAR(w.height(200.0, -200.0), floor + 16.0, 1e-3);
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
  // Without a city there are no wall cells, and the rest of the ground is occupied as it was: the
  // walls are what the city adds. (Comparing the whole occupied count does not say this: the
  // roads' banks lift some deep water, which is occupied, to dry gravel, which is not.)
  p.city = "none";
  const generated_world bare = generate_world(p);
  std::size_t bareWalls = 0, wallsOccupied = 0;
  for (std::size_t i = 0; i < bare.raster.klass.size(); ++i)
    bareWalls += bare.raster.klass[i] == wall;
  for (std::size_t i = 0; i < w.raster.klass.size(); ++i)
    wallsOccupied += w.raster.klass[i] == wall && w.raster.occupancy[i] == 1;
  EXPECT_EQ(bareWalls, 0u);
  EXPECT_EQ(wallsOccupied, walls);
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
  EXPECT_EQ(w.props_r1.const_points().size(), a.props_r1.const_points().size());
  EXPECT_EQ(w.props_r2.const_points().size(), a.props_r2.const_points().size());
  EXPECT_EQ(w.prop_cells.size(), a.prop_cells.size());
  ASSERT_EQ(w.terrain_chunks.size(), a.terrain_chunks.size());
  for (std::size_t i = 0; i < w.terrain_chunks.size(); ++i)
    EXPECT_EQ(w.terrain_chunks[i].error_m, a.terrain_chunks[i].error_m) << i;
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

// The prop cells cut rung 0 into contiguous runs that cover every mesh exactly, in order; each
// cell's rung-0 mesh is inside its bounds, its coarser rungs stand where its full trees do (no
// higher, no lower, nearly as tall) at a fraction of the triangles, and they carry no lines.
TEST(WorldGenerated, PropCellsAreTheFullTreesCoarsened) {
  generated_world_params p = small_world();
  p.trees = 120;
  const generated_world w = generate_world(p);
  ASSERT_FALSE(w.prop_cells.empty());
  EXPECT_GE(w.prop_cell_m, 128.0);
  EXPECT_TRUE(w.props_r1.const_lines().empty());
  EXPECT_LT(w.props_r1.const_tris().size(), w.props.const_tris().size() / 2);
  EXPECT_LT(w.props_r2.const_tris().size(), w.props.const_tris().size() / 20);
  auto covers = [&](const cvc::geometry &g, auto span, bool lines) {
    std::size_t p = 0, c = 0;
    for (const generated_world::prop_cell &pc : w.prop_cells) {
      const generated_world::prop_span &s = span(pc);
      EXPECT_EQ(s.p0, p);
      EXPECT_EQ(s.c0, c);
      p = s.p1;
      c = s.c1;
    }
    EXPECT_EQ(p, g.const_points().size());
    EXPECT_EQ(c, lines ? g.const_lines().size() : g.const_tris().size());
  };
  covers(w.props, [](const auto &c) -> const auto & { return c.props; }, false);
  covers(w.needles, [](const auto &c) -> const auto & { return c.needles; }, true);
  covers(w.props_r1, [](const auto &c) -> const auto & { return c.props_r1; }, false);
  covers(w.props_r2, [](const auto &c) -> const auto & { return c.props_r2; }, false);
  for (const generated_world::prop_cell &pc : w.prop_cells) {
    const cvc::geometry r0 = prop_span_mesh(w.props, pc.props);
    for (const auto &q : r0.const_points())
      for (int a = 0; a < 3; ++a) {
        EXPECT_GE(q[a], pc.min[a]);
        EXPECT_LE(q[a], pc.max[a]);
      }
    for (const auto &t : r0.const_tris())
      for (int k = 0; k < 3; ++k)
        ASSERT_LT(t[k], r0.const_points().size());
    const double h = pc.max[2] - pc.min[2];
    for (const generated_world::prop_span *s : {&pc.props_r1, &pc.props_r2}) {
      const cvc::geometry r = prop_span_mesh(s == &pc.props_r1 ? w.props_r1 : w.props_r2, *s);
      ASSERT_FALSE(r.const_points().empty());
      double zmin = 1e30, zmax = -1e30;
      for (const auto &q : r.const_points()) {
        zmin = std::min(zmin, q[2]);
        zmax = std::max(zmax, q[2]);
      }
      EXPECT_GE(zmin, pc.min[2] - 0.5);
      EXPECT_LE(zmax, pc.max[2] + 0.5);
      EXPECT_GT(zmax, pc.max[2] - 0.15 * h) << "the coarse trees are as tall as the full ones";
    }
  }
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

namespace {

// The height of mesh g's triangle under (x, y), among its first `top` vertices' triangles (the
// surface, not the skirts); NaN where none covers it.
double meshHeight(const cvc::geometry &g, std::size_t top, double x, double y) {
  const auto &P = g.const_points();
  for (const auto &t : g.const_tris()) {
    if (t[0] >= top || t[1] >= top || t[2] >= top)
      continue;
    const auto &a = P[t[0]], &b = P[t[1]], &c = P[t[2]];
    const double d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    if (std::abs(d) < 1e-12)
      continue;
    const double l0 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d;
    const double l1 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d;
    const double l2 = 1.0 - l0 - l1;
    if (l0 >= -1e-9 && l1 >= -1e-9 && l2 >= -1e-9)
      return l0 * a[2] + l1 * b[2] + l2 * c[2];
  }
  return std::nan("");
}

// A chunk's rung k samples down and across (its last row and column always).
std::size_t stridedCount(int a, int b, int k) {
  return std::size_t((b - a + (1 << k) - 1) >> k) + 1;
}

} // namespace

// The ground's LOD chunks tile the terrain: rung 0 is the terrain's own mesh, cut up, and every
// rung's vertices are the terrain's own samples (point, normal, UV). A rung's error_m bounds how
// far it is from every sample it skips, it grows with the stride, and only edges a neighbour
// shares carry a skirt, hung skirt_m straight down.
TEST(WorldGenerated, TerrainChunksAreTheGroundAtCoarserStrides) {
  const generated_world w = generate_world(small_world());
  const cvc::height_field &hf = *w.terrain;
  const terrain_chunk_layout &L = w.terrain_layout;
  ASSERT_EQ(L.cells, 64);
  ASSERT_EQ(L.rungs, 5); // strides 1..16: down to 4 cells a side
  ASSERT_EQ(w.terrain_chunks.size(), std::size_t(L.chunk_rows) * L.chunk_cols);
  ASSERT_EQ(L.chunk_rows, 2);
  std::size_t tris0 = 0;
  for (const terrain_chunk &ch : w.terrain_chunks) {
    ASSERT_EQ(int(ch.rungs.size()), L.rungs);
    ASSERT_EQ(ch.error_m.size(), ch.rungs.size());
    EXPECT_EQ(ch.error_m[0], 0.0);
    for (int k = 1; k < L.rungs; ++k) {
      EXPECT_GE(ch.error_m[k], ch.error_m[k - 1]);
      EXPECT_LT(ch.rungs[k].const_tris().size(), ch.rungs[k - 1].const_tris().size());
    }
    EXPECT_GT(ch.error_m.back(), 0.1); // 40 m of relief is not flat at 50 m a sample
    for (int k = 0; k < L.rungs; ++k) {
      const cvc::geometry &g = ch.rungs[std::size_t(k)];
      const std::size_t nr = stridedCount(ch.r0, ch.r1, k), nc = stridedCount(ch.c0, ch.c1, k);
      const std::size_t top = nr * nc;
      ASSERT_GE(g.const_points().size(), top);
      ASSERT_EQ(g.const_normals().size(), g.const_points().size());
      ASSERT_EQ(g.const_uvs().size(), g.const_points().size());
      // Every vertex is a terrain sample's, the skirts' lowered by skirt_m.
      for (std::size_t v = 0; v < g.const_points().size(); ++v) {
        const auto &p = g.const_points()[v];
        const int c = int(std::lround((p[0] - hf.x0()) / hf.dx()));
        const int r = int(std::lround((p[1] - hf.y0()) / hf.dy()));
        const auto src = std::size_t(hf.vertex_of(r, c));
        const auto &q = hf.const_points()[src];
        EXPECT_EQ(p[0], q[0]);
        EXPECT_EQ(p[1], q[1]);
        EXPECT_NEAR(p[2], q[2] - (v < top ? 0.0 : ch.skirt_m[std::size_t(k)]), 1e-9);
        EXPECT_EQ(g.const_normals()[v], hf.const_normals()[src]);
        EXPECT_EQ(g.const_uvs()[v], hf.const_uvs()[src]);
      }
      // The skirts: one edge's samples again for each edge a neighbour shares.
      std::size_t skirt = 0;
      skirt += ch.row > 0 ? nc : 0;
      skirt += ch.row + 1 < L.chunk_rows ? nc : 0;
      skirt += ch.col > 0 ? nr : 0;
      skirt += ch.col + 1 < L.chunk_cols ? nr : 0;
      EXPECT_EQ(g.const_points().size() - top, skirt);
      EXPECT_GT(ch.skirt_m[std::size_t(k)], 0.0); // every chunk of a 2 x 2 ground has neighbours
      if (k == 0) {
        std::size_t surface = 0;
        for (const auto &t : g.const_tris())
          surface += t[0] < top && t[1] < top && t[2] < top;
        EXPECT_EQ(surface, std::size_t(2 * (ch.r1 - ch.r0) * (ch.c1 - ch.c0)));
        tris0 += surface;
        continue;
      }
      // No skipped sample is further from the rung than its error.
      double worst = 0.0;
      for (int r = ch.r0; r <= ch.r1; ++r)
        for (int c = ch.c0; c <= ch.c1; ++c) {
          const auto &q = hf.const_points()[std::size_t(hf.vertex_of(r, c))];
          const double z = meshHeight(g, top, q[0], q[1]);
          ASSERT_FALSE(std::isnan(z)) << "rung " << k << " misses sample " << r << "," << c;
          worst = std::max(worst, std::abs(z - q[2]));
        }
      EXPECT_LE(worst, ch.error_m[std::size_t(k)] + 1e-6) << "rung " << k;
    }
  }
  EXPECT_EQ(tris0, hf.const_tris().size());
}

// Where two chunks meet a rung apart, the gap between their surfaces along the shared edge is
// never deeper than either one's skirt: no crack shows.
TEST(WorldGenerated, TerrainSkirtsCoverTheSeams) {
  const generated_world w = generate_world(small_world());
  const terrain_chunk_layout &L = w.terrain_layout;
  ASSERT_GE(L.chunk_cols, 2);
  const terrain_chunk &a = w.terrain_chunks[0], &b = w.terrain_chunks[1]; // b east of a
  const double x = w.terrain->x0() + a.c1 * w.terrain->dx();
  // The edge x = const of a rung as a polyline in y.
  auto edge = [x](const terrain_chunk &ch, int k) {
    const cvc::geometry &g = ch.rungs[std::size_t(k)];
    const std::size_t nr = stridedCount(ch.r0, ch.r1, k), nc = stridedCount(ch.c0, ch.c1, k);
    std::vector<std::pair<double, double>> e;
    for (std::size_t v = 0; v < nr * nc; ++v)
      if (std::abs(g.const_points()[v][0] - x) < 1e-9)
        e.emplace_back(g.const_points()[v][1], g.const_points()[v][2]);
    std::sort(e.begin(), e.end());
    return e;
  };
  auto at = [](const std::vector<std::pair<double, double>> &e, double y) {
    for (std::size_t i = 0; i + 1 < e.size(); ++i)
      if (y >= e[i].first - 1e-9 && y <= e[i + 1].first + 1e-9)
        return e[i].second +
               (e[i + 1].second - e[i].second) * (y - e[i].first) / (e[i + 1].first - e[i].first);
    return std::nan("");
  };
  for (int ka = 0; ka < L.rungs; ++ka)
    for (int kb = std::max(0, ka - 1); kb <= std::min(L.rungs - 1, ka + 1); ++kb) {
      const auto ea = edge(a, ka), eb = edge(b, kb);
      ASSERT_GE(ea.size(), 2u);
      ASSERT_EQ(ea.front().first, eb.front().first);
      ASSERT_EQ(ea.back().first, eb.back().first);
      for (int r = a.r0; r <= a.r1; ++r) {
        const double y = w.terrain->y0() + r * w.terrain->dy();
        const double gap = std::abs(at(ea, y) - at(eb, y));
        EXPECT_LT(gap, a.skirt_m[std::size_t(ka)]) << ka << " beside " << kb << " at " << r;
        EXPECT_LT(gap, b.skirt_m[std::size_t(kb)]) << ka << " beside " << kb << " at " << r;
      }
    }
}

// A ground that does not divide into whole chunks ends in narrower ones, each still ending on the
// terrain's last sample; a ground of one chunk has no skirts.
TEST(WorldGenerated, TerrainChunksFitAnyGround) {
  std::vector<float> h(101 * 101);
  for (int r = 0; r < 101; ++r)
    for (int c = 0; c < 101; ++c)
      h[std::size_t(r) * 101 + c] = float(3.0 * std::sin(0.2 * r) * std::cos(0.13 * c));
  const cvc::height_field hf(101, 101, 0.0, 0.0, 2.0, 2.0, h);
  const terrain_chunk_layout L = plan_terrain_chunks(hf, 16, 32);
  EXPECT_EQ(L.cells, 32);
  EXPECT_EQ(L.chunk_rows, 4); // 32 + 32 + 32 + 4 cells
  EXPECT_EQ(L.rungs, 4);      // 32 / 8 = 4 cells a side at the coarsest
  const terrain_chunk last = make_terrain_chunk(hf, L, 3, 3);
  EXPECT_EQ(last.r1, 100);
  EXPECT_EQ(last.c1, 100);
  for (const cvc::geometry &g : last.rungs) // 4 cells: every rung keeps the corner samples
    EXPECT_FALSE(g.const_tris().empty());
  const terrain_chunk_layout one = plan_terrain_chunks(hf, 1, 32);
  EXPECT_EQ(one.cells, 128);
  ASSERT_EQ(one.chunk_rows * one.chunk_cols, 1);
  const terrain_chunk whole = make_terrain_chunk(hf, one, 0, 0);
  for (double s : whole.skirt_m)
    EXPECT_EQ(s, 0.0);
  EXPECT_EQ(whole.rungs[0].const_tris().size(), hf.const_tris().size());
}

// No chunk ends up more than one rung off a neighbour, and none is made coarser than it chose.
TEST(WorldGenerated, TerrainRungsStayWithinOneOfTheirNeighbours) {
  std::vector<int> r = {4, 4, 4, //
                        4, 0, 4, //
                        4, 4, 4};
  constrain_terrain_rungs(r, 3, 3);
  EXPECT_EQ(r, (std::vector<int>{2, 1, 2, 1, 0, 1, 2, 1, 2}));
  std::vector<int> line = {0, 5, 5, 5, 5, 5};
  constrain_terrain_rungs(line, 1, 6);
  EXPECT_EQ(line, (std::vector<int>{0, 1, 2, 3, 4, 5}));
  std::vector<int> fine = {1, 0, 2, 1};
  const std::vector<int> before = fine;
  constrain_terrain_rungs(fine, 2, 2);
  EXPECT_EQ(fine, before);
}

// The streets are driveable on the ground physics drives on (the terrain mesh): each fits two
// Humvees side by side, is near level across, climbs gently and never dips into the water, though
// the ground the network was traced over tilts and bumps under it.
TEST(WorldGenerated, RoadsAreDriveable) {
  const generated_world w = generate_world(small_world());
  const auto *graded = dynamic_cast<const graded_terrain *>(&w.model->terrain());
  ASSERT_NE(graded, nullptr);
  const road_network &net = graded->roads();
  ASSERT_GT(net.segments.size(), 20u);
  std::vector<double> tilt, tiltBefore, grade;
  bool wet = false;
  for (const road_segment &s : net.segments) {
    EXPECT_GE(s.width, 2.0 * world_lane_m - 1e-9);
    EXPECT_GE(s.width, 2.0 * world_vehicle_width_m + 0.8);
    const road_node &a = net.nodes[s.a], &b = net.nodes[s.b];
    const double L = std::hypot(b.x - a.x, b.y - a.y);
    if (L < 2.0)
      continue;
    const double ux = (b.x - a.x) / L, uy = (b.y - a.y) / L, hw = 0.5 * s.width;
    const int n = int(L / 2.0);
    double prev = 0.0;
    for (int i = 0; i <= n; ++i) {
      const double x = a.x + ux * L * i / n, y = a.y + uy * L * i / n;
      const double l = w.height(x - uy * hw, y + ux * hw), r = w.height(x + uy * hw, y - ux * hw);
      const double c = w.height(x, y);
      tilt.push_back(std::abs(l - r) / s.width);
      tiltBefore.push_back(std::abs(graded->ground().sample(x - uy * hw, y + ux * hw) -
                                    graded->ground().sample(x + uy * hw, y - ux * hw)) /
                           s.width);
      wet = wet || std::min({l, r, c}) < w.params.water_level_m;
      if (i > 0)
        grade.push_back(std::abs(c - prev) / (L / n));
      prev = c;
    }
  }
  auto p95 = [](std::vector<double> v) {
    std::sort(v.begin(), v.end());
    return v[std::size_t(0.95 * (v.size() - 1))];
  };
  EXPECT_LT(p95(tilt), 0.08);
  EXPECT_GT(p95(tiltBefore), 4.0 * p95(tilt)) << "the grading levelled the roads";
  EXPECT_LT(p95(grade), 0.16);
  EXPECT_FALSE(wet);
}

// Water seen from above: the bed's sandy tint in the shallows, absorbed red first, so it greens and
// then blues with depth.
TEST(WorldGenerated, WaterColourDeepensWithDepth) {
  const auto shallow = water_colour(0.05), mid = water_colour(2.0), deep = water_colour(12.0);
  EXPECT_GT(shallow[0], shallow[2]) << "a sandy bed shows through the shallows";
  EXPECT_LT(deep[0], deep[2]) << "deep water is blue";
  for (int k = 0; k < 3; ++k)
    EXPECT_GE(shallow[std::size_t(k)], deep[std::size_t(k)]);
  EXPECT_GT(shallow[0], mid[0]);
  EXPECT_GT(mid[0], deep[0]);
}

// A chunk's water: level at the water line over every cell of a rung with a corner under it, each
// vertex coloured by the water above the bed there; none for a dry chunk.
TEST(WorldGenerated, WaterRungsCoverTheWetCells) {
  std::vector<float> h(65 * 65);
  for (int r = 0; r < 65; ++r)
    for (int c = 0; c < 65; ++c)
      h[std::size_t(r) * 65 + c] = float(std::hypot(r - 32.0, c - 32.0) * 0.4 - 5.0); // a pit
  const cvc::height_field hf(65, 65, 0.0, 0.0, 2.0, 2.0, h);
  const terrain_chunk_layout L = plan_terrain_chunks(hf, 16, 64);
  ASSERT_EQ(L.chunk_rows * L.chunk_cols, 1);
  const terrain_chunk ch = make_terrain_chunk(hf, L, 0, 0);
  const std::vector<cvc::geometry> water = make_water_rungs(hf, L, ch, 0.0);
  ASSERT_EQ(int(water.size()), L.rungs);
  std::size_t wetCells = 0;
  for (int r = 0; r < 64; ++r)
    for (int c = 0; c < 64; ++c)
      wetCells += std::min({hf.height_at(r, c), hf.height_at(r, c + 1), hf.height_at(r + 1, c),
                            hf.height_at(r + 1, c + 1)}) < 0.0f;
  EXPECT_EQ(water[0].const_tris().size(), 2 * wetCells);
  for (std::size_t i = 0; i < water[0].const_points().size(); ++i) {
    const auto &p = water[0].const_points()[i];
    EXPECT_EQ(p[2], 0.0);
    const int c = int(std::lround(p[0] / 2.0)), r = int(std::lround(p[1] / 2.0));
    EXPECT_EQ(water[0].const_colors()[i], water_colour(-double(hf.height_at(r, c))));
    EXPECT_EQ(water[0].const_normals()[i][2], 1.0);
  }
  for (int k = 1; k < L.rungs; ++k)
    EXPECT_LT(water[std::size_t(k)].const_tris().size(),
              water[std::size_t(k - 1)].const_tris().size());
  for (const cvc::geometry &g : make_water_rungs(hf, L, ch, -10.0)) // below the pit's floor: dry
    EXPECT_TRUE(g.const_tris().empty());
}

// The water's texture follows the terrain mesh texel by texel: opaque and the column's colour where
// the mesh is under the water, clear where it is dry. A coarse rung's water cells, drawn whole,
// reach over dry ground wherever their corners straddle the shore (the lake over grass on a large
// world); their texture coordinates land on clear texels there, which the renderer discards, so
// the water ends at the mesh's own shore whatever the rung.
TEST(WorldGenerated, TheWaterTextureEndsTheWaterAtTheMeshShore) {
  const generated_world w = generate_world(small_world());
  const cvc::height_field &hf = *w.terrain;
  const double sea = w.params.water_level_m;
  const int T = 257;
  const std::vector<unsigned char> img = paint_water(w, T);
  ASSERT_EQ(img.size(), std::size_t(T) * T * 4);
  const grid_spec &g = w.grid;
  auto texelX = [&](int tx) { return g.min_x + double(tx) / (T - 1) * (g.max_x - g.min_x); };
  auto texelY = [&](int ty) { return g.min_y + double(ty) / (T - 1) * (g.max_y - g.min_y); };
  int wetTexels = 0, dryTexels = 0;
  for (int ty = 0; ty < T; ++ty)
    for (int tx = 0; tx < T; ++tx) {
      const double h = hf.height(texelX(tx), texelY(ty));
      const unsigned char *px = &img[(std::size_t(ty) * T + tx) * 4];
      if (h < sea) {
        ++wetTexels;
        ASSERT_EQ(px[3], 255) << tx << "," << ty;
        const cvc::geometry::color_t c = water_colour(sea - h);
        for (int q = 0; q < 3; ++q)
          EXPECT_NEAR(px[q], c[std::size_t(q)] * 255.0, 1.0);
      } else {
        ++dryTexels;
        ASSERT_EQ(px[3], 0) << tx << "," << ty;
      }
    }
  EXPECT_GT(wetTexels, 500);
  EXPECT_GT(dryTexels, 500);

  // The coarsest rung's water, sampled across each triangle: where its point's texel lies over dry
  // mesh the texture is clear there (and there are such points: the whole cells spill), where it
  // lies over wet mesh, opaque. The texel found from the point's (u, v) is the point's own.
  const double texel = (g.max_x - g.min_x) / (T - 1);
  int spill = 0, inside = 0;
  for (const terrain_chunk &ch : w.terrain_chunks) {
    if (ch.water.empty() || ch.water.back().const_tris().empty())
      continue;
    const cvc::geometry &wg = ch.water.back();
    const auto &P = wg.const_points();
    const auto &UV = wg.const_uvs();
    ASSERT_EQ(UV.size(), P.size());
    for (const auto &t : wg.const_tris())
      for (int a = 0; a <= 6; ++a)
        for (int b = 0; a + b <= 6; ++b) {
          const double l1 = a / 6.0, l2 = b / 6.0, l0 = 1.0 - l1 - l2;
          const double x = l0 * P[t[0]][0] + l1 * P[t[1]][0] + l2 * P[t[2]][0];
          const double y = l0 * P[t[0]][1] + l1 * P[t[1]][1] + l2 * P[t[2]][1];
          const double u = l0 * UV[t[0]][0] + l1 * UV[t[1]][0] + l2 * UV[t[2]][0];
          const double v = l0 * UV[t[0]][1] + l1 * UV[t[1]][1] + l2 * UV[t[2]][1];
          const int tx = int(std::lround(u * (T - 1))), ty = int(std::lround(v * (T - 1)));
          ASSERT_LE(std::abs(texelX(tx) - x), 0.5 * texel + 1e-6);
          ASSERT_LE(std::abs(texelY(ty) - y), 0.5 * texel + 1e-6);
          const bool wet = hf.height(texelX(tx), texelY(ty)) < sea;
          (wet ? inside : spill) += 1;
          EXPECT_EQ(img[(std::size_t(ty) * T + tx) * 4 + 3], wet ? 255 : 0);
        }
  }
  EXPECT_GT(spill, 100) << "the coarse rung's whole cells reach over dry ground";
  EXPECT_GT(inside, 100);
}

// The ground's texture paints the water by the terrain mesh, not the raster: the raster is a fixed
// number of cells a side (26 m cells on a 10 km world), so classed from it the painted shore lay
// tens of metres off the mesh's, a band of water colour on the dry bank. Under the water a texel
// takes the water's own colour for the column (what a coarse rung's ground shows where it rises
// over the lake); a dry texel is never painted as water.
TEST(WorldGenerated, ThePaintedShoreFollowsTheMesh) {
  generated_world_params p = small_world();
  p.raster_n = 33; // 12.5 m raster cells over the mesh's 3.1 m: a large world's ratio
  const generated_world w = generate_world(p);
  const cvc::height_field &hf = *w.terrain;
  const double sea = w.params.water_level_m;
  const int T = 257;
  const std::vector<unsigned char> img = paint_surface(w, surface_color::material, T);
  const grid_spec &g = w.grid;
  int wet = 0, dryNearWater = 0;
  for (int ty = 0; ty < T; ++ty)
    for (int tx = 0; tx < T; ++tx) {
      const double x = g.min_x + double(tx) / (T - 1) * (g.max_x - g.min_x);
      const double y = g.min_y + double(ty) / (T - 1) * (g.max_y - g.min_y);
      const double h = hf.height(x, y);
      const unsigned char *px = &img[(std::size_t(ty) * T + tx) * 3];
      if (h < sea) {
        ++wet;
        const cvc::geometry::color_t c = water_colour(sea - h);
        for (int q = 0; q < 3; ++q)
          ASSERT_NEAR(px[q], c[std::size_t(q)] * 255.0, 1.5) << tx << "," << ty;
      } else {
        // No water's blue-green on dry ground: every land colour has blue at most a little over
        // red; the old shore paint was 0.36, 0.52, 0.50.
        dryNearWater += h < sea + 3.0;
        ASSERT_LT(int(px[2]) - int(px[0]), 16) << tx << "," << ty << " h " << h;
      }
    }
  EXPECT_GT(wet, 500);
  EXPECT_GT(dryNearWater, 200);
}

// Every point of the generated world under the water has a water surface over it.
TEST(WorldGenerated, TheWaterIsDrawnWhereverItIs) {
  const generated_world w = generate_world(small_world());
  const cvc::height_field &hf = *w.terrain;
  const int cr = hf.cell_rows(), cc = hf.cell_cols();
  std::vector<char> lid(std::size_t(cr) * cc, 0);
  for (const terrain_chunk &ch : w.terrain_chunks) {
    if (ch.water.empty() || ch.water[0].const_tris().empty())
      continue;
    const auto &P = ch.water[0].const_points();
    for (const auto &t : ch.water[0].const_tris()) {
      const double x = (P[t[0]][0] + P[t[1]][0] + P[t[2]][0]) / 3.0;
      const double y = (P[t[0]][1] + P[t[1]][1] + P[t[2]][1]) / 3.0;
      lid[std::size_t(int((y - hf.y0()) / hf.dy())) * cc + int((x - hf.x0()) / hf.dx())] = 1;
    }
  }
  int wet = 0;
  for (int r = 0; r < cr; ++r)
    for (int c = 0; c < cc; ++c) {
      const double x = hf.x0() + (c + 0.5) * hf.dx(), y = hf.y0() + (r + 0.5) * hf.dy();
      if (w.height(x, y) < w.params.water_level_m) {
        ++wet;
        EXPECT_TRUE(lid[std::size_t(r) * cc + c]) << x << "," << y;
      }
    }
  EXPECT_GT(wet, 100);
}

namespace {

// A flat, all-grass world `half` metres each way on the generator's 385-cell material raster,
// with `net` stamped into the raster as asphalt and kept as its streets.
generated_world grass_world(double half, const road_network &net) {
  generated_world w;
  w.grid.rows = w.grid.cols = 385; // generated_world_params::raster_n
  w.grid.min_x = w.grid.min_y = -half;
  w.grid.max_x = w.grid.max_y = half;
  const surface_registry &reg = surface_registry::builtin();
  std::uint16_t grass = 0;
  EXPECT_TRUE(reg.find("grass", grass));
  const std::size_t n = w.grid.count();
  w.raster.rows = w.grid.rows;
  w.raster.cols = w.grid.cols;
  w.raster.klass.assign(n, grass);
  w.raster.risk_raw.assign(n, reg[grass].rho);
  w.raster.hard.assign(n, reg[grass].hard ? 1 : 0);
  w.raster.occupancy.assign(n, reg[grass].hard ? 1 : 0);
  w.raster.height.assign(n, 10.0f); // dry: above the water level
  w.params.water_level_m = 0.0;
  rasterize_roads(net, w.grid, reg, w.raster, 1.0); // as the world builder stamps them
  w.streets = net;
  return w;
}

} // namespace

// A road in the surface texture is its own width on a 10 km world. The raster's cells are 26 m
// there, and the texture used to paint every texel touching an asphalt cell, blended over the four
// cells round it: a dark band tens of metres wide along every road, which the road mesh (6.5 m)
// could not cover. Now past the road and its verge the texture is the bare ground's, texel for
// texel, and the road's own texels are dark.
TEST(WorldGenerated, StreetsPaintAtTheirWidthOnALargeWorld) {
  const double half = 5005.0; // terrain_lab's 10 km world
  road_network net;
  // On a row of raster cells (26.07 m apart), so the stamp marks them: off one, the 4.25 m reach
  // can miss every centre, which is why the old band came and went along a road.
  const double y = 131.0;
  net.nodes = {{-3000.0, y, 1}, {3000.0, y, 1}};
  road_segment s;
  s.a = 0;
  s.b = 1;
  s.width = 6.5;
  net.segments = {s};
  const generated_world road = grass_world(half, net);
  const generated_world bare = grass_world(half, road_network{});
  std::uint16_t asphalt = 0;
  ASSERT_TRUE(surface_registry::builtin().find("asphalt", asphalt));
  ASSERT_GT(std::count(road.raster.klass.begin(), road.raster.klass.end(), asphalt), 100);

  const int T = 2048; // terrain_lab's texture
  const std::vector<unsigned char> a = paint_surface(road, surface_color::material, T);
  const std::vector<unsigned char> b = paint_surface(bare, surface_color::material, T);
  ASSERT_EQ(a.size(), b.size());
  const double texel = 2.0 * half / (T - 1);
  const double reach = 0.5 * s.width + street_verge_max_m + 1.0 + texel;
  int away = 0, awayDiffer = 0, onRoad = 0, darkOnRoad = 0;
  for (int ty = 0; ty < T; ++ty) {
    const double d = std::abs(-half + ty * texel - y);
    for (int tx = 0; tx < T; ++tx) {
      const double x = -half + tx * texel;
      if (x < -2900.0 || x > 2900.0 || d > 80.0)
        continue; // along the road, not round its ends
      const std::size_t i = (std::size_t(ty) * T + tx) * 3;
      const int la = a[i] + a[i + 1] + a[i + 2], lb = b[i] + b[i + 1] + b[i + 2];
      if (d > reach) {
        ++away;
        int most = 0;
        for (int q = 0; q < 3; ++q)
          most = std::max(most, std::abs(int(a[i + q]) - int(b[i + q])));
        awayDiffer += most > 0;
      } else if (d < 0.5 * texel) {
        ++onRoad;
        darkOnRoad += la < lb - 30; // below the grass
      }
    }
  }
  ASSERT_GT(away, 10000);
  EXPECT_EQ(awayDiffer, 0) << "the road darkened the ground past its own width and verge";
  ASSERT_GT(onRoad, 500);
  EXPECT_GT(darkOnRoad, onRoad * 9 / 10) << "the road is not in the texture";
}

// The verge: none past its widest, a frayed edge, and on some stretches of a long road only.
TEST(WorldGenerated, StreetVergesLineSomeStretches) {
  int with = 0, total = 0;
  for (double x = -5000.0; x <= 5000.0; x += 5.0) {
    EXPECT_EQ(street_verge(x, 37.0, street_verge_max_m + 1.5, 1.0), 0.0) << x;
    const double v = street_verge(x, 37.0, 0.6, 0.1);
    EXPECT_GE(v, 0.0);
    EXPECT_LE(v, 1.0);
    ++total;
    with += v > 0.25;
  }
  EXPECT_GT(with, total / 5) << "too few roads have a verge";
  EXPECT_LT(with, total * 4 / 5) << "every road has a verge";
  // Where there is one it fades out over a metre or so, not at one fixed width.
  std::set<int> edges;
  for (double x = -5000.0; x <= 5000.0; x += 5.0) {
    if (street_verge(x, 37.0, 0.6, 0.1) < 0.5)
      continue;
    double e = 0.6;
    while (e < street_verge_max_m + 2.0 && street_verge(x, 37.0, e, 0.1) > 0.0)
      e += 0.1;
    edges.insert(int(std::lround(e * 2.0)));
  }
  EXPECT_GT(edges.size(), 5u) << "the verge's edge does not wander";
}

// A terrain cell's water is the mesh's: a cell whose ground (its corners' mean) is more than half
// a metre under the sea is deep water, hard, which the navigator does not drive into and the
// physics gives deep water's grip; one just under it is shallow water; a dry one is not water.
// Each cell used to take one raster sample's class, a corner of the cell where the raster and the
// mesh share samples, so on a steep shore a cell metres deep could be dry ground or drivable
// shallow water (world_gym.ari's generated world drove a Humvee down one and off the world).
// Cells the roads graded keep the class they had before (the banks lift them out of the water).
TEST(WorldGenerated, ACellsWaterIsTheMeshsDepth) {
  const generated_world w = generate_world(small_world());
  const cvc::height_field &hf = *w.terrain;
  const auto *graded = dynamic_cast<const graded_terrain *>(&w.model->terrain());
  ASSERT_NE(graded, nullptr);
  const surface_registry &reg = w.model->registry();
  std::uint16_t shallow = 0, deep = 0;
  ASSERT_TRUE(reg.find("water_shallow", shallow));
  ASSERT_TRUE(reg.find("water_deep", deep));
  ASSERT_TRUE(reg[deep].hard);
  const double sea = w.params.water_level_m;
  const int N = hf.rows();
  const std::vector<float> &hs = hf.heights();
  int deepCells = 0, shallowCells = 0, dryCells = 0;
  for (int r = 0; r + 1 < N; ++r)
    for (int c = 0; c + 1 < N; ++c) {
      const double x = hf.x0() + c * hf.dx(), y = hf.y0() + r * hf.dy();
      if (graded->graded_at(x, y) || graded->graded_at(x + hf.dx(), y) ||
          graded->graded_at(x, y + hf.dy()) || graded->graded_at(x + hf.dx(), y + hf.dy()))
        continue;
      const std::size_t s = std::size_t(r) * N + c;
      const double h = 0.25 * (double(hs[s]) + hs[s + 1] + hs[s + N] + hs[s + N + 1]);
      const std::uint16_t k = hf.cell_material(r, c);
      if (h < sea - 0.5) {
        ++deepCells;
        EXPECT_TRUE(reg[k].hard) << "a cell " << sea - h << " m deep at (" << x << ", " << y
                                 << ") is " << reg[k].name;
      } else if (h < sea) {
        ++shallowCells;
        EXPECT_TRUE(k == shallow || reg[k].hard) << "(" << x << ", " << y << ") " << reg[k].name;
      } else {
        ++dryCells;
        EXPECT_TRUE(k != shallow && k != deep) << "dry (" << x << ", " << y << ") is water";
      }
    }
  EXPECT_GT(deepCells, 100);
  EXPECT_GT(shallowCells, 10);
  EXPECT_GT(dryCells, 1000);
}

// The world's edge: walls by default, or a shore: a band at the rim becomes coast (the sea floor
// at the edge, rising to the waterline, then a gentle beach, then the relief again), the sea is
// drawn on past the world, and there are no walls. Inland the ground is the walled world's own.
TEST(WorldGenerated, AShoreEdgeLinesTheWorldWithABeachAndTheSea) {
  EXPECT_EQ(world_edge_from_name("wall"), int(world_edge::wall));
  EXPECT_EQ(world_edge_from_name("shore"), int(world_edge::shore));
  EXPECT_EQ(world_edge_from_name("none"), int(world_edge::none));
  EXPECT_EQ(world_edge_from_name("moat"), -1);
  EXPECT_STREQ(world_edge_name(int(world_edge::shore)), "shore");

  generated_world_params p = small_world();
  const generated_world walled = generate_world(p);
  p.edge = int(world_edge::none);
  EXPECT_FALSE(generate_world(p).geometry().edge_walls());
  p.edge = int(world_edge::shore);
  const generated_world shore = generate_world(p);
  EXPECT_TRUE(walled.geometry().edge_walls());
  EXPECT_FALSE(shore.geometry().edge_walls());
  EXPECT_TRUE(outer_sea(walled).const_points().empty());

  const double half = p.half_m, band = world_shore_band_m(p), floor = world_shore_floor_m(p);
  EXPECT_DOUBLE_EQ(band, 50.0); // 20 % of 200 m is 40, held at 50
  EXPECT_DOUBLE_EQ(floor, -12.0);
  const cvc::height_field &hf = *shore.terrain;
  ASSERT_EQ(hf.rows(), walled.terrain->rows());
  // On the edge the ground is the sea floor (the mesh, sampled from it, deep); past the band the
  // ground is the walled world's (its terrain source's: the roads grade the mesh, and the city
  // builds elsewhere off the shore).
  generated_world_params wp = p;
  wp.edge = int(world_edge::wall);
  const auto shore_ground = make_terrain(p), wall_ground = make_terrain(wp);
  int edge = 0, inland = 0;
  for (int r = 0; r < hf.rows(); ++r)
    for (int c = 0; c < hf.cols(); ++c) {
      const double x = hf.x0() + c * hf.dx(), y = hf.y0() + r * hf.dy();
      const double d = half - std::max(std::fabs(x), std::fabs(y));
      if (d < 1e-6) {
        EXPECT_LT(hf.height_at(r, c), p.water_level_m - 5.0) << x << ", " << y;
        EXPECT_LE(shore_ground->sample(x, y), floor + 1e-9) << x << ", " << y;
        ++edge;
      } else if (d > band + 1e-6) {
        EXPECT_DOUBLE_EQ(shore_ground->sample(x, y), wall_ground->sample(x, y)) << x << ", " << y;
        ++inland;
      }
    }
  EXPECT_EQ(edge, 4 * (hf.rows() - 1));
  EXPECT_GT(inland, hf.rows() * hf.cols() / 2);
  // The coast's cells: deep water at the edge, a beach of sand further in, all round.
  const surface_registry &reg = shore.model->registry();
  std::uint16_t sand = 0;
  ASSERT_TRUE(reg.find("sand", sand));
  int rim = 0, deep = 0, beach = 0;
  for (int r = 0; r < hf.cell_rows(); ++r)
    for (int c = 0; c < hf.cell_cols(); ++c) {
      const double x = hf.x0() + (c + 0.5) * hf.dx(), y = hf.y0() + (r + 0.5) * hf.dy();
      const double d = half - std::max(std::fabs(x), std::fabs(y));
      const std::uint16_t k = hf.cell_material(r, c);
      if (d < 0.5 * hf.dx()) {
        ++rim;
        deep += reg[k].hard ? 1 : 0;
      } else if (d > 0.4 * band && d < 0.5 * band) { // the beach, wherever the coast wanders
        beach += k == sand ? 1 : 0;
      }
    }
  EXPECT_EQ(deep, rim) << "the rim is deep water all round";
  EXPECT_GT(beach, 2 * hf.cell_rows()) << "a beach all round";

  // The sea past the world: a ring at the waterline from the edge out, facing up, in the deep
  // water's colour.
  const cvc::geometry sea = outer_sea(shore, 1000.0);
  ASSERT_EQ(sea.const_points().size(), 8u);
  ASSERT_EQ(sea.const_tris().size(), 8u);
  double lo = 0.0, hi = 0.0;
  for (const auto &v : sea.const_points()) {
    EXPECT_DOUBLE_EQ(v[2], p.water_level_m);
    lo = std::min(lo, double(v[0]));
    hi = std::max(hi, double(v[0]));
  }
  EXPECT_DOUBLE_EQ(lo, -1200.0);
  EXPECT_DOUBLE_EQ(hi, 1200.0);
  for (const auto &t : sea.const_tris()) {
    const auto &a = sea.const_points()[t[0]], &b = sea.const_points()[t[1]],
               &c = sea.const_points()[t[2]];
    const double nz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    EXPECT_GT(nz, 0.0) << "a triangle faces down";
  }
  EXPECT_EQ(sea.const_colors()[0], water_colour(p.water_level_m - floor));
}
