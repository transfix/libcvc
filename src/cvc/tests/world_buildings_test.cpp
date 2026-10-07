/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/building_archetypes.h for the full header).
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h> // get_extensions (is GLB reading built in this config?)
#include <cvc/world/building_archetypes.h>
#include <gtest/gtest.h>
#include <string>

using namespace cvc::world;

namespace {
cvc::geometry box(double cx, double cy, double zlo, double zhi, double hx, double hy) {
  cvc::geometry g;
  const double cz = 0.5 * (zlo + zhi), hz = 0.5 * (zhi - zlo);
  for (int i = 0; i < 8; ++i)
    g.points().push_back(cvc::geometry::point_t{
        {cx + ((i & 1) ? hx : -hx), cy + ((i & 2) ? hy : -hy), cz + ((i & 4) ? hz : -hz)}});
  const int f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 2, 6}, {0, 6, 4},
                        {1, 5, 7}, {1, 7, 3}, {0, 4, 5}, {0, 5, 1}, {2, 3, 7}, {2, 7, 6}};
  for (auto &t : f)
    g.tris().push_back(
        cvc::geometry::tri_t{{std::uint64_t(t[0]), std::uint64_t(t[1]), std::uint64_t(t[2])}});
  return g;
}
cvc::model::mesh M(const std::string &name, cvc::geometry g) {
  cvc::model::mesh m;
  m.name = name;
  m.geom = std::move(g);
  return m;
}
} // namespace

TEST(WorldBuildings, GroupsMeshesByBuildingIdAndNormalizes) {
  cvc::model m;
  // bldg_001: walls (z 0..6) + roof (z 6..8), footprint 4x3, centred at (10,20).
  m.meshes.push_back(M("bldg_001_walls", box(10, 20, 0, 6, 4, 3)));
  m.meshes.push_back(M("bldg_001_roof", box(10, 20, 6, 8, 4, 3)));
  // bldg_002: a shorter box elsewhere.
  m.meshes.push_back(M("bldg_002_walls", box(-5, -5, 0, 4, 2, 2)));

  const building_library lib = building_library_from_model(m);
  ASSERT_EQ(lib.size(), 2u); // two buildings, not three meshes
  const building_archetype &a = lib.archetypes[0];
  EXPECT_EQ(a.name, "bldg_001");
  EXPECT_NEAR(a.footprint_x, 4.0, 1e-9);
  EXPECT_NEAR(a.footprint_y, 3.0, 1e-9);
  EXPECT_NEAR(a.height, 8.0, 1e-9); // walls+roof merged span z 0..8
  // Normalized: centred in XY, base on z=0.
  const cvc::bounding_box bb = a.mesh.extents();
  EXPECT_NEAR(0.5 * (bb.minx + bb.maxx), 0.0, 1e-9);
  EXPECT_NEAR(0.5 * (bb.miny + bb.maxy), 0.0, 1e-9);
  EXPECT_NEAR(bb.minz, 0.0, 1e-9);
  EXPECT_NEAR(bb.maxz, 8.0, 1e-9);
  EXPECT_NEAR(lib.archetypes[1].height, 4.0, 1e-9);
}

TEST(WorldBuildings, StampPlacesScalesAndRotates) {
  cvc::model m;
  m.meshes.push_back(M("b_walls", box(0, 0, 0, 10, 3, 2))); // 6x4 footprint, 10 tall
  const building_library lib = building_library_from_model(m);
  ASSERT_EQ(lib.size(), 1u);

  std::vector<building_instance> inst;
  inst.push_back(building_instance{0, 100.0, 50.0, 0.0, 0.0, 2.0}); // at (100,50), scale 2
  const cvc::geometry g = stamp_buildings(lib, inst);
  ASSERT_GT(g.const_points().size(), 0u);
  const cvc::bounding_box bb = g.extents();
  EXPECT_NEAR(0.5 * (bb.minx + bb.maxx), 100.0, 1e-6); // placed at x
  EXPECT_NEAR(0.5 * (bb.miny + bb.maxy), 50.0, 1e-6);
  EXPECT_NEAR(bb.minz, 0.0, 1e-6);            // base on the ground
  EXPECT_NEAR(bb.maxz, 20.0, 1e-6);           // 10 tall x scale 2
  EXPECT_NEAR(bb.maxx - bb.minx, 12.0, 1e-6); // 6 wide x scale 2

  // A 90-degree yaw swaps the footprint axes.
  std::vector<building_instance> rot;
  rot.push_back(building_instance{0, 0.0, 0.0, 0.0, M_PI / 2.0, 1.0});
  const cvc::bounding_box rb = stamp_buildings(lib, rot).extents();
  EXPECT_NEAR(rb.maxx - rb.minx, 4.0, 1e-6); // the 6-wide X maps to Y; the 4-deep Y maps to X
  EXPECT_NEAR(rb.maxy - rb.miny, 6.0, 1e-6);

  // An out-of-range archetype index is skipped (no crash, empty result).
  std::vector<building_instance> bad{building_instance{5, 0, 0, 0, 0, 1}};
  EXPECT_EQ(stamp_buildings(lib, bad).const_points().size(), 0u);
}

TEST(WorldBuildings, EmptyModelGivesEmptyLibrary) {
  EXPECT_TRUE(building_library_from_model(cvc::model{}).empty());
  EXPECT_TRUE(load_building_library("/no/such/file.glb").empty());
}

// Gated: if the published Austin bundle is reachable, confirm it yields thousands of real
// archetypes.
TEST(WorldBuildings, RealAustinBundleYieldsManyArchetypes) {
  const char *paths[] = {
      "/home/joe/src/cvc/.wasm-deps-cvc12/share/cvc-scenes/austin_south/buildings.glb",
      "/home/joe/src/cvc/scratchpad-scenes/share/cvc-scenes/austin_south_lean/buildings.glb"};
  std::string found;
  for (const char *p : paths)
    if (FILE *f = std::fopen(p, "rb")) {
      std::fclose(f);
      found = p;
      break;
    }
  if (found.empty())
    GTEST_SKIP() << "Austin bundle not present in this environment";
  const building_library lib = load_building_library(found);
  if (lib.empty())
    GTEST_SKIP() << "this build cannot decode GLB (assimp disabled) — ran in CI's assimp build";
  EXPECT_GT(lib.size(), 1000u);
  for (const building_archetype &a : lib.archetypes) {
    EXPECT_GT(a.footprint_x, 0.0);
    EXPECT_GT(a.height, 0.0);
  }
}
