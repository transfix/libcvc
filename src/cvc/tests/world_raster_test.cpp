/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/raster.h for the full header).
*/

// The raster seam invariants (roadmap §7.3): every plane the same size and from
// one grid; risk_raw == registry[klass].rho and hard == registry[klass].hard
// EXACTLY; hard ⊆ occupancy; deterministic; row 0 == min_y.

#include <cvc/world/raster.h>
#include <gtest/gtest.h>
#include <string>

using namespace cvc::world;

namespace {
world_model make_world(std::uint64_t seed) {
  world_params wp;
  wp.seed = seed;
  wp.min_x = wp.min_y = -120;
  wp.max_x = wp.max_y = 120;
  return world_model::generate(wp);
}
grid_spec small() { return grid_spec::window(0, 0, 120, 3.0); } // 81x81, fast
} // namespace

TEST(WorldRaster, PlaneSizesAndContract) {
  world_model wm = make_world(7);
  raster_out o;
  raster(wm, small(), o);
  const std::size_t n = std::size_t(o.rows) * o.cols;
  ASSERT_EQ(o.klass.size(), n);
  ASSERT_EQ(o.risk_raw.size(), n);
  ASSERT_EQ(o.hard.size(), n);
  ASSERT_EQ(o.occupancy.size(), n);
  ASSERT_EQ(o.height.size(), n);
  const surface_registry &reg = wm.registry();
  for (std::size_t i = 0; i < n; ++i) {
    const surface_class &c = reg[o.klass[i]];
    EXPECT_EQ(o.risk_raw[i], c.rho);
    EXPECT_EQ(o.hard[i] != 0, c.hard);
    if (o.hard[i])
      EXPECT_EQ(o.occupancy[i], 1); // hard ⊆ occupancy
  }
}

TEST(WorldRaster, Deterministic) {
  world_model wm = make_world(11);
  raster_out a, b;
  raster(wm, small(), a);
  raster(wm, small(), b);
  EXPECT_EQ(a.klass, b.klass);
  EXPECT_EQ(a.occupancy, b.occupancy);
  EXPECT_EQ(a.risk_raw, b.risk_raw);
  EXPECT_EQ(a.height, b.height);
}

TEST(WorldRaster, RowZeroIsMinY) {
  // height at (row 0) must equal the heightfield sampled at y == min_y.
  world_model wm = make_world(4);
  grid_spec g = small();
  raster_out o;
  raster(wm, g, o);
  const double y0 = g.min_y;
  for (int c = 0; c < g.cols; ++c) {
    double x = g.world_x(c);
    EXPECT_NEAR(o.height[std::size_t(0) * g.cols + c], float(wm.hf().sample(x, y0)), 1e-3);
  }
}

TEST(WorldRaster, OccupancyIsSparseButPresent) {
  world_model wm = make_world(7);
  raster_out o;
  raster(wm, small(), o);
  double frac = double(o.occupied_count()) / o.klass.size();
  EXPECT_GT(frac, 0.0);  // props produce obstacles
  EXPECT_LT(frac, 0.35); // a solvable scene, not a wall of water
}

TEST(WorldRaster, BuildingsTileIntoManyOccupiedCells) {
  // The physics caveat: a building footprint must become MANY occupied cells (so
  // downstream it tiles into many small discs, not one bounding circle).
  world_params wp;
  wp.seed = 2;
  wp.min_x = wp.min_y = -120;
  wp.max_x = wp.max_y = 120;
  wp.sc = scatter_params::defaults();
  wp.sc.tree_count = 0;
  wp.sc.rock_count = 0;
  wp.sc.building_count = 4;
  world_model wm = world_model::generate(wp);
  raster_out o;
  raster(wm, grid_spec::window(0, 0, 120, 0.5), o); // real 0.5 m cell
  // At least one reinforced_concrete / brick building => tens of occupied cells.
  const surface_registry &reg = wm.registry();
  std::size_t concrete = 0, brick = 0;
  for (std::size_t i = 0; i < o.klass.size(); ++i) {
    const char *rf = reg[o.klass[i]].rf_material;
    if (o.occupancy[i]) {
      if (std::string(rf) == "reinforced_concrete")
        ++concrete;
      else if (std::string(rf) == "brick")
        ++brick;
    }
  }
  EXPECT_GT(concrete + brick, std::size_t(100)); // a 14x11 m block at 0.5 m is ~600 cells
}
