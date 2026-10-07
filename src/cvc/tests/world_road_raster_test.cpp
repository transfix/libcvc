/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/road_raster.h for the full header).
*/

#include <cmath>
#include <cstdint>
#include <cvc/world/grid.h>
#include <cvc/world/raster.h>
#include <cvc/world/road_raster.h>
#include <cvc/world/roads.h>
#include <cvc/world/surface.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::world;

namespace {

road_network straight_road(double width = 11.0) {
  road_network net;
  net.half = 128.0;
  net.nodes.push_back(road_node{-100.0, 0.0, 1});
  net.nodes.push_back(road_node{100.0, 0.0, 1});
  road_segment s;
  s.a = 0;
  s.b = 1;
  s.width = width;
  s.arterial = true;
  net.segments.push_back(s);
  return net;
}

// A raster_out of all-`klass0` cells, derived values consistent with the registry.
raster_out blank_raster(const grid_spec &g, const surface_registry &reg, std::uint16_t klass0 = 0) {
  raster_out out;
  const std::size_t n = g.count();
  const surface_class &sc = reg[klass0];
  out.rows = g.rows;
  out.cols = g.cols;
  out.klass.assign(n, klass0);
  out.risk_raw.assign(n, sc.rho);
  out.hard.assign(n, sc.hard ? 1 : 0);
  out.occupancy.assign(n, sc.hard ? 1 : 0);
  out.height.assign(n, 0.0f);
  out.layer_owner.assign(n, 0);
  return out;
}

// Verify the raster invariants the module promises to keep.
void expect_invariants(const raster_out &out, const surface_registry &reg) {
  for (std::size_t i = 0; i < out.klass.size(); ++i) {
    const surface_class &sc = reg[out.klass[i]];
    EXPECT_FLOAT_EQ(out.risk_raw[i], sc.rho);
    EXPECT_EQ(out.hard[i], sc.hard ? 1 : 0);
    EXPECT_LE(out.hard[i], out.occupancy[i]); // hard ⊆ occupancy
  }
}

} // namespace

TEST(WorldRoadRaster, StampsAsphaltUnderTheRoad) {
  const surface_registry &reg = surface_registry::builtin();
  std::uint16_t asphalt = 0;
  ASSERT_TRUE(reg.find("asphalt", asphalt));
  const grid_spec g = grid_spec::window(0.0, 0.0, 120.0, 2.0); // 121^2, centred on origin
  raster_out out = blank_raster(g, reg);

  const std::size_t stamped = rasterize_roads(straight_road(), g, reg, out);
  EXPECT_GT(stamped, 0u);

  // The centre cell sits on the road at the origin -> asphalt, drivable, authored layer.
  const std::size_t mid = std::size_t(g.rows / 2) * g.cols + (g.cols / 2);
  EXPECT_EQ(out.klass[mid], asphalt);
  EXPECT_FLOAT_EQ(out.risk_raw[mid], reg[asphalt].rho);
  EXPECT_EQ(out.hard[mid], 0);      // asphalt is not a hard hazard
  EXPECT_EQ(out.occupancy[mid], 0); // ... so it is drivable, not occupied
  EXPECT_EQ(out.layer_owner[mid], 1);

  // A corner far from the road is untouched.
  EXPECT_EQ(out.klass[0], 0u);
  expect_invariants(out, reg);
}

TEST(WorldRoadRaster, OnlyNearRoadCellsChange) {
  const surface_registry &reg = surface_registry::builtin();
  const grid_spec g = grid_spec::window(0.0, 0.0, 120.0, 2.0);
  raster_out out = blank_raster(g, reg);
  rasterize_roads(straight_road(11.0), g, reg, out);
  std::uint16_t asphalt = 0;
  reg.find("asphalt", asphalt);
  // Every stamped cell must actually be within the carriageway of the single y=0 road.
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c) {
      const std::size_t i = std::size_t(r) * g.cols + c;
      if (out.klass[i] == asphalt) {
        // Every stamped cell is within the carriageway (half-width 5.5), treating the road as a
        // capsule — so cells just past the endpoints, within the rounded cap, count too.
        const double d = straight_road().distance_to_road(g.world_x(c), g.world_y(r));
        EXPECT_LE(d, 5.5 + 1e-6);
      }
    }
}

TEST(WorldRoadRaster, MarginWidensTheStamp) {
  const surface_registry &reg = surface_registry::builtin();
  const grid_spec g = grid_spec::window(0.0, 0.0, 120.0, 2.0);
  raster_out a = blank_raster(g, reg), b = blank_raster(g, reg);
  const std::size_t n0 = rasterize_roads(straight_road(), g, reg, a, 0.0);
  const std::size_t n1 = rasterize_roads(straight_road(), g, reg, b, 6.0);
  EXPECT_GT(n1, n0);
}

TEST(WorldRoadRaster, UnknownClassIsANoOp) {
  const surface_registry &reg = surface_registry::builtin();
  const grid_spec g = grid_spec::window(0.0, 0.0, 60.0, 2.0);
  raster_out out = blank_raster(g, reg);
  const raster_out before = out;
  const std::size_t stamped = rasterize_roads(straight_road(), g, reg, out, 0.0, "no_such_class");
  EXPECT_EQ(stamped, 0u);
  EXPECT_EQ(out.klass, before.klass); // untouched
}

TEST(WorldRoadRaster, SizeMismatchIsANoOp) {
  const surface_registry &reg = surface_registry::builtin();
  const grid_spec g = grid_spec::window(0.0, 0.0, 60.0, 2.0);
  raster_out out; // empty arrays != g.count()
  EXPECT_EQ(rasterize_roads(straight_road(), g, reg, out), 0u);
}

TEST(WorldRoadRaster, RealNetworkKeepsInvariants) {
  const surface_registry &reg = surface_registry::builtin();
  road_params rp;
  rp.half = 120.0;
  rp.seed = 5;
  rp.branch_chance = 0.15;
  const road_network net = generate_roads(rp);
  const grid_spec g = grid_spec::window(0.0, 0.0, 120.0, 2.0);
  raster_out out = blank_raster(g, reg);
  EXPECT_GT(rasterize_roads(net, g, reg, out, 1.0), 0u);
  expect_invariants(out, reg);
}
