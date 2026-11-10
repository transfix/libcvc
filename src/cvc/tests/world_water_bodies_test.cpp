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

// cvc::world::water_bodies — the bodies of water and the shore grid
// (OCEAN-AND-WEATHER-ROADMAP.md section 3): an island is one sea, a hollow one lake, two hollows
// two lakes, a hollow joined to the sea by a channel part of the sea; small bodies are ponds; the
// shore distance is the exact Euclidean distance, its normal points offshore; and a generated
// world carries its water bodies, drawn in the water colour mode.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/world/generated_world.h>
#include <cvc/world/water_bodies.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <stdexcept>
#include <vector>

using namespace cvc::world;

namespace {

// A square grid of half-width `half` metres at `cell` metres, its ground from f(x, y).
water_bodies build(double half, double cell, const std::function<double(double, double)> &f,
                   water_bodies_params p = {}) {
  const grid_spec g = grid_spec::window(0.0, 0.0, half, cell);
  std::vector<float> ground(g.count());
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c)
      ground[std::size_t(r) * g.cols + c] = float(f(g.world_x(c), g.world_y(r)));
  return build_water_bodies(std::move(ground), g, p);
}

// A bowl of radius rad at (cx, cy), d metres deep at its middle; 1e9 outside it (take the min with
// the land's height).
double bowl(double x, double y, double cx, double cy, double rad, double d) {
  const double q = ((x - cx) * (x - cx) + (y - cy) * (y - cy)) / (rad * rad);
  return q < 1.0 ? -d * (1.0 - q) : 1.0e9;
}

double wb_at(const water_bodies &wb, const std::vector<float> &plane, double x, double y) {
  return wb.sample(plane, x, y);
}

} // namespace

// ── labelling (the roadmap's four cases) ─────────────────────────────────────────────────────────

TEST(WorldWaterBodies, AnIslandIsOneSea) {
  // A dome 10 m high at the centre, falling 0.2 m a metre: the land is a disc of radius 50 m.
  const water_bodies wb =
      build(100.0, 1.0, [](double x, double y) { return 10.0 - 0.2 * std::hypot(x, y); });
  ASSERT_EQ(wb.bodies.size(), 1u);
  EXPECT_EQ(wb.bodies[0].kind, water_kind::sea);
  EXPECT_EQ(wb.count(water_kind::sea), 1u);
  EXPECT_EQ(wb.body_at(0.0, 0.0), -1);   // the island
  EXPECT_EQ(wb.body_at(90.0, 90.0), 0);  // the sea round it
  EXPECT_EQ(wb.body_at(-75.0, 10.0), 0); // the same sea on the far side
  EXPECT_EQ(water_bodies_summary(wb), "1 sea");
  // The shore: inland negative, offshore positive, about 50 m out from the centre.
  EXPECT_NEAR(wb_at(wb, wb.shore_m, 0.0, 0.0), -50.0, 1.0);
  EXPECT_NEAR(wb_at(wb, wb.shore_m, 80.0, 0.0), 30.0, 1.0);
  // Its normal points offshore: away from the island.
  EXPECT_NEAR(wb_at(wb, wb.normal_x, 70.0, 0.0), 1.0, 1e-3);
  EXPECT_NEAR(wb_at(wb, wb.normal_y, 0.0, -70.0), -1.0, 1e-3);
  EXPECT_NEAR(wb_at(wb, wb.normal_x, 30.0, 0.0), 1.0, 1e-3); // inland it points to the sea too
}

TEST(WorldWaterBodies, AHollowBelowTheLevelIsOneLake) {
  // A bowl of radius 70 m (15 400 m^2, over a hectare) in ground 5 m high.
  const water_bodies wb = build(100.0, 1.0, [](double x, double y) {
    return std::min(5.0, bowl(x, y, 0.0, 0.0, 70.0, 8.0));
  });
  ASSERT_EQ(wb.bodies.size(), 1u);
  const water_body &b = wb.bodies[0];
  EXPECT_EQ(b.kind, water_kind::lake);
  EXPECT_NEAR(b.area_m2, 3.14159265 * 70.0 * 70.0, 0.02 * 3.14159265 * 70.0 * 70.0);
  EXPECT_NEAR(b.max_depth_m, 8.0, 1e-3);
  EXPECT_NEAR(b.min_x, -70.0, 1.0);
  EXPECT_NEAR(b.max_y, 70.0, 1.0);
  EXPECT_DOUBLE_EQ(b.level_m, 0.0);
  EXPECT_EQ(wb.body_at(0.0, 0.0), 0);
  EXPECT_EQ(wb.body_at(95.0, 0.0), -1);
  // The lake's own shore: offshore is toward its middle.
  EXPECT_NEAR(wb_at(wb, wb.normal_x, 40.0, 0.0), -1.0, 1e-3);
  EXPECT_NEAR(wb_at(wb, wb.shore_m, 0.0, 0.0), 70.0, 1.0);
  // Depth is the level less the ground: 8 m at the middle, negative on the bank.
  EXPECT_NEAR(wb_at(wb, wb.depth, 0.0, 0.0), 8.0, 1e-3);
  EXPECT_NEAR(wb_at(wb, wb.depth, 90.0, 0.0), -5.0, 1e-3);
}

TEST(WorldWaterBodies, TwoSeparateHollowsAreTwoLakes) {
  water_bodies_params p;
  p.pond_max_area_m2 = 1000.0; // each bowl is about 5000 m^2
  const water_bodies wb = build(
      100.0, 1.0,
      [](double x, double y) {
        return std::min({5.0, bowl(x, y, -50.0, 0.0, 40.0, 6.0), bowl(x, y, 50.0, 0.0, 40.0, 3.0)});
      },
      p);
  ASSERT_EQ(wb.bodies.size(), 2u);
  EXPECT_EQ(wb.count(water_kind::lake), 2u);
  EXPECT_EQ(wb.count(water_kind::sea), 0u);
  // Numbered by their first sample (row 0 first, x fastest): both start on the same row, so the
  // west one (smaller x) is first.
  EXPECT_EQ(wb.body_at(-50.0, 0.0), 0);
  EXPECT_EQ(wb.body_at(50.0, 0.0), 1);
  EXPECT_NEAR(wb.bodies[0].max_depth_m, 6.0, 1e-3);
  EXPECT_NEAR(wb.bodies[1].max_depth_m, 3.0, 1e-3);
  EXPECT_EQ(water_bodies_summary(wb), "2 lakes");
}

TEST(WorldWaterBodies, AChannelToTheSeaMakesTheHollowPartOfTheSea) {
  // The sea west of x = -80; land 5 m high; a bowl at the middle; a channel 6 m wide and 2 m deep
  // from the sea to the bowl.
  auto ground = [](double x, double y, bool channel) {
    if (x < -80.0)
      return -5.0;
    double h = std::min(5.0, bowl(x, y, 0.0, 0.0, 40.0, 6.0));
    if (channel && std::fabs(y) < 3.0 && x < 0.0)
      h = std::min(h, -2.0);
    return h;
  };
  const water_bodies closed =
      build(100.0, 1.0, [&](double x, double y) { return ground(x, y, false); });
  ASSERT_EQ(closed.bodies.size(), 2u); // the sea, and the bowl on its own
  EXPECT_NE(closed.body_at(0.0, 0.0), closed.body_at(-90.0, 0.0));

  const water_bodies joined =
      build(100.0, 1.0, [&](double x, double y) { return ground(x, y, true); });
  ASSERT_EQ(joined.bodies.size(), 1u);
  EXPECT_EQ(joined.bodies[0].kind, water_kind::sea);
  EXPECT_EQ(joined.body_at(0.0, 0.0), 0);
  EXPECT_EQ(joined.body_at(-90.0, 0.0), 0);
  EXPECT_EQ(joined.body_at(-40.0, 0.0), 0); // the channel
}

TEST(WorldWaterBodies, InletsAtTheEdgeAreOneSea) {
  // Water along the west and the east edges, dry land between, and a hollow in the middle: the
  // two inlets are one sea (they meet past the world), the hollow a body of its own.
  const water_bodies wb = build(100.0, 1.0, [](double x, double y) {
    if (std::fabs(x) > 80.0)
      return -3.0;
    return std::min(4.0, bowl(x, y, 0.0, 0.0, 30.0, 2.0));
  });
  ASSERT_EQ(wb.bodies.size(), 2u);
  EXPECT_EQ(wb.count(water_kind::sea), 1u);
  EXPECT_EQ(wb.body_at(-90.0, 0.0), wb.body_at(90.0, 0.0));
  EXPECT_EQ(wb.bodies[std::size_t(wb.body_at(90.0, 0.0))].kind, water_kind::sea);
  EXPECT_EQ(wb.bodies[std::size_t(wb.body_at(0.0, 0.0))].kind, water_kind::pond);
  EXPECT_EQ(water_bodies_summary(wb), "1 sea, 1 pond");
  // The sea counts both inlets' samples.
  EXPECT_EQ(wb.bodies[std::size_t(wb.body_at(90.0, 0.0))].cells, std::size_t(2 * 20 * 201));
}

TEST(WorldWaterBodies, ADiagonalTouchJoinsTwoRunsOfWater) {
  // Two wet samples meeting only at a corner are one body (8-connected).
  const grid_spec g = grid_spec::window(0.0, 0.0, 2.0, 1.0); // 5 x 5
  std::vector<float> ground(g.count(), 5.0f);
  ground[std::size_t(1) * 5 + 1] = -1.0f;
  ground[std::size_t(2) * 5 + 2] = -1.0f;
  const water_bodies wb = build_water_bodies(ground, g);
  ASSERT_EQ(wb.bodies.size(), 1u);
  EXPECT_EQ(wb.bodies[0].cells, 2u);
}

TEST(WorldWaterBodies, ASmallEnclosedBodyIsAPond) {
  // A bowl of radius 30 m: about 2800 m^2, under a hectare.
  auto f = [](double x, double y) { return std::min(5.0, bowl(x, y, 0.0, 0.0, 30.0, 2.0)); };
  const water_bodies pond = build(60.0, 1.0, f);
  ASSERT_EQ(pond.bodies.size(), 1u);
  EXPECT_EQ(pond.bodies[0].kind, water_kind::pond);
  EXPECT_EQ(water_bodies_summary(pond), "1 pond");
  water_bodies_params p;
  p.pond_max_area_m2 = 1000.0;
  const water_bodies lake = build(60.0, 1.0, f, p);
  EXPECT_EQ(lake.bodies[0].kind, water_kind::lake);
}

TEST(WorldWaterBodies, DryAndFloodedWorlds) {
  const water_bodies dry = build(20.0, 1.0, [](double, double) { return 3.0; });
  EXPECT_TRUE(dry.bodies.empty());
  EXPECT_EQ(water_bodies_summary(dry), "no water");
  const double diag = std::hypot(40.0, 40.0);
  EXPECT_FLOAT_EQ(dry.shore_m[0], float(-diag)); // no shore anywhere
  EXPECT_EQ(dry.normal_x[0], 0.0f);

  const water_bodies sea = build(20.0, 1.0, [](double, double) { return -3.0; });
  ASSERT_EQ(sea.bodies.size(), 1u);
  EXPECT_EQ(sea.bodies[0].kind, water_kind::sea);
  EXPECT_FLOAT_EQ(sea.shore_m[0], float(diag));
}

// ── the shore grid ───────────────────────────────────────────────────────────────────────────────

TEST(WorldWaterBodies, AStraightShoreHasExactDistanceNormalAndSlope) {
  // A plane rising 1 in 10 toward +x, its waterline at x = 0.5 (halfway between two samples).
  const water_bodies wb = build(50.0, 1.0, [](double x, double) { return 0.1 * (x - 0.5); });
  ASSERT_EQ(wb.bodies.size(), 1u);
  EXPECT_EQ(wb.bodies[0].kind, water_kind::sea);
  for (int r = 0; r < wb.grid.rows; r += 7)
    for (int c = 0; c < wb.grid.cols; ++c) {
      const std::size_t i = wb.index(r, c);
      const double x = wb.grid.world_x(c);
      EXPECT_NEAR(wb.shore_m[i], 0.5 - x, 1e-4) << "x " << x;
      EXPECT_NEAR(wb.slope[i], 0.1, 1e-5);
      EXPECT_NEAR(wb.depth[i], -0.1 * (x - 0.5), 1e-5);
      EXPECT_EQ(wb.body[i] >= 0, x < 0.5);
      EXPECT_NEAR(wb.normal_x[i], -1.0, 1e-6); // offshore is -x, on either side of the shore
      EXPECT_NEAR(wb.normal_y[i], 0.0, 1e-6);
    }
}

TEST(WorldWaterBodies, TheShoreDistanceIsTheExactEuclideanOne) {
  // A scatter of wet and dry samples: each one's |shore_m| + half a cell is its distance to the
  // nearest sample of the other kind, by brute force.
  const grid_spec g = grid_spec::window(0.0, 0.0, 15.0, 1.0); // 31 x 31
  std::vector<float> ground(g.count());
  std::uint32_t s = 12345u;
  for (float &h : ground) {
    s = s * 1664525u + 1013904223u;
    h = (s >> 8) % 100 < 12 ? -1.0f : 1.0f; // about one sample in eight wet
  }
  const water_bodies wb = build_water_bodies(ground, g);
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c) {
      const std::size_t i = wb.index(r, c);
      const bool wet = ground[i] < 0.0f;
      double best = std::numeric_limits<double>::infinity();
      for (int rr = 0; rr < g.rows; ++rr)
        for (int cc = 0; cc < g.cols; ++cc)
          if ((ground[wb.index(rr, cc)] < 0.0f) != wet)
            best = std::min(best, std::hypot(double(rr - r), double(cc - c)));
      EXPECT_NEAR(std::fabs(wb.shore_m[i]) + 0.5, best, 1e-5) << r << "," << c;
      EXPECT_EQ(wb.shore_m[i] > 0.0f, wet);
    }
}

TEST(WorldWaterBodies, SmoothingKeepsAStraightShoresNormal) {
  water_bodies_params p;
  p.normal_smooth_m = 8.0;
  const water_bodies wb = build(40.0, 1.0, [](double, double y) { return 0.2 * (y - 0.5); }, p);
  for (std::size_t i = 0; i < wb.normal_y.size(); i += 13) {
    EXPECT_NEAR(wb.normal_x[i], 0.0, 1e-6);
    EXPECT_NEAR(wb.normal_y[i], -1.0, 1e-6);
  }
}

TEST(WorldWaterBodies, SmoothingStraightensANormalAcrossTheMedialAxis) {
  // A quadrant of sea (x < 0, y < 0) with a square corner of land. Offshore the raw normal points
  // away from the nearer of the two straight shores, so it snaps through 90 degrees across the
  // diagonal (the distance field's medial axis); smoothed over 10 m it turns gently there. Every
  // normal is unit or zero.
  auto f = [](double x, double y) { return (x < 0.0 && y < 0.0) ? -2.0 : 2.0; };
  const water_bodies raw = build(40.0, 1.0, f);
  water_bodies_params p;
  p.normal_smooth_m = 10.0;
  const water_bodies smooth = build(40.0, 1.0, f, p);
  for (std::size_t i = 0; i < smooth.normal_x.size(); ++i) {
    const double len = std::hypot(smooth.normal_x[i], smooth.normal_y[i]);
    EXPECT_TRUE(len == 0.0 || std::fabs(len - 1.0) < 1e-5);
  }
  auto turn = [](const water_bodies &w, double x0, double y0, double x1, double y1) {
    const double ax = w.sample(w.normal_x, x0, y0), ay = w.sample(w.normal_y, x0, y0);
    const double bx = w.sample(w.normal_x, x1, y1), by = w.sample(w.normal_y, x1, y1);
    return std::acos(std::clamp(
        (ax * bx + ay * by) / (std::hypot(ax, ay) * std::hypot(bx, by) + 1e-12), -1.0, 1.0));
  };
  // Either side of the diagonal, a sample apart: (-10, -9) is nearer the y = 0 shore, (-9, -10)
  // the x = 0 one.
  EXPECT_NEAR(raw.sample(raw.normal_y, -10.0, -9.0), -1.0, 1e-6);
  EXPECT_NEAR(raw.sample(raw.normal_x, -9.0, -10.0), -1.0, 1e-6);
  const double rawTurn = turn(raw, -10.0, -9.0, -9.0, -10.0);
  EXPECT_NEAR(rawTurn, 1.5707963, 1e-4);
  EXPECT_LT(turn(smooth, -10.0, -9.0, -9.0, -10.0), 0.5 * rawTurn);
}

TEST(WorldWaterBodies, FetchIsTheLongAxis) {
  // A rectangular lake 20 m across and 100 m long, then the same turned 90 degrees.
  auto rect = [](double x, double y) {
    return (std::fabs(x) < 50.0 && std::fabs(y) < 10.0) ? -1.0 : 1.0;
  };
  const water_bodies a = build(80.0, 1.0, rect);
  ASSERT_EQ(a.bodies.size(), 1u);
  EXPECT_GT(a.bodies[0].fetch_m, 100.0);
  EXPECT_LT(a.bodies[0].fetch_m, 120.0); // 4 sqrt(var) = 1.15 L for a rectangle
  const water_bodies b = build(80.0, 1.0, [&](double x, double y) { return rect(y, x); });
  EXPECT_NEAR(b.bodies[0].fetch_m, a.bodies[0].fetch_m, 1e-9);
  // A single wet sample's fetch is a sample.
  const grid_spec g = grid_spec::window(0.0, 0.0, 3.0, 0.5);
  std::vector<float> ground(g.count(), 1.0f);
  ground[g.count() / 2] = -1.0f;
  EXPECT_DOUBLE_EQ(build_water_bodies(ground, g).bodies[0].fetch_m, 0.5);
}

TEST(WorldWaterBodies, TheLevelSetsTheWater) {
  // The same ground under a higher level: more water, and the body's level is the level.
  auto f = [](double x, double y) { return 0.05 * std::hypot(x, y); }; // a bowl to the edges
  water_bodies_params p;
  p.level_m = 2.0;
  const water_bodies wb = build(60.0, 1.0, f, p);
  ASSERT_EQ(wb.bodies.size(), 1u);
  EXPECT_EQ(wb.bodies[0].kind, water_kind::pond); // about 5000 m^2
  EXPECT_DOUBLE_EQ(wb.bodies[0].level_m, 2.0);
  EXPECT_NEAR(wb.bodies[0].min_x, -40.0, 1.0); // wet to radius 40 m
  EXPECT_NEAR(wb_at(wb, wb.level, 55.0, 0.0), 2.0, 1e-9);
  EXPECT_NEAR(wb_at(wb, wb.depth, 0.0, 0.0), 2.0, 1e-6);
}

TEST(WorldWaterBodies, NoGroundIsDry) {
  const grid_spec g = grid_spec::window(0.0, 0.0, 2.0, 1.0);
  std::vector<float> ground(g.count(), -1.0f);
  ground[12] = std::numeric_limits<float>::quiet_NaN();
  const water_bodies wb = build_water_bodies(ground, g);
  EXPECT_EQ(wb.body[12], -1);
  EXPECT_TRUE(std::isnan(wb.depth[12]));
  EXPECT_LT(wb.shore_m[12], 0.0f);
  EXPECT_EQ(wb.bodies[0].cells, g.count() - 1);
}

TEST(WorldWaterBodies, IsDeterministic) {
  auto f = [](double x, double y) {
    return 4.0 * std::sin(0.07 * x) * std::cos(0.05 * y) + 0.02 * x - 0.5;
  };
  water_bodies_params p;
  p.normal_smooth_m = 6.0;
  const water_bodies a = build(120.0, 2.0, f, p), b = build(120.0, 2.0, f, p);
  ASSERT_GT(a.bodies.size(), 1u);
  EXPECT_EQ(a.body, b.body);
  auto same = [](const std::vector<float> &x, const std::vector<float> &y) {
    return x.size() == y.size() && std::memcmp(x.data(), y.data(), x.size() * sizeof(float)) == 0;
  };
  EXPECT_TRUE(same(a.shore_m, b.shore_m));
  EXPECT_TRUE(same(a.normal_x, b.normal_x));
  EXPECT_TRUE(same(a.normal_y, b.normal_y));
  EXPECT_TRUE(same(a.slope, b.slope));
  for (std::size_t k = 0; k < a.bodies.size(); ++k)
    EXPECT_EQ(a.bodies[k].fetch_m, b.bodies[k].fetch_m);
}

TEST(WorldWaterBodies, RejectsAMismatchedGrid) {
  const grid_spec g = grid_spec::window(0.0, 0.0, 2.0, 1.0);
  EXPECT_THROW(build_water_bodies(std::vector<float>(3, 0.0f), g), std::invalid_argument);
  grid_spec one = g;
  one.rows = 1;
  EXPECT_THROW(build_water_bodies(std::vector<float>(std::size_t(one.cols), 0.0f), one),
               std::invalid_argument);
  grid_spec oblong = g; // 1 m cells across, 2 m along
  oblong.max_y = oblong.min_y + 2.0 * (oblong.max_y - oblong.min_y);
  EXPECT_THROW(build_water_bodies(std::vector<float>(oblong.count(), 0.0f), oblong),
               std::invalid_argument);
}

TEST(WorldWaterBodies, QueriesOffTheGrid) {
  const water_bodies wb = build(10.0, 1.0, [](double, double) { return -1.0; });
  EXPECT_EQ(wb.body_at(0.0, 0.0), 0);
  EXPECT_EQ(wb.body_at(10.4, 0.0), 0);  // within half a sample of the edge
  EXPECT_EQ(wb.body_at(10.6, 0.0), -1); // past it
  EXPECT_EQ(wb.body_at(0.0, -11.0), -1);
  EXPECT_NEAR(wb.sample(wb.depth, 50.0, 50.0), 1.0, 1e-9); // clamped
  const water_bodies none;
  EXPECT_EQ(none.body_at(0.0, 0.0), -1);
  EXPECT_TRUE(std::isnan(none.sample(none.depth, 0.0, 0.0)));
}

// ── a generated world's water ────────────────────────────────────────────────────────────────────

namespace {
generated_world_params small_world() {
  generated_world_params p;
  p.seed = 11;
  p.half_m = 200.0;
  p.trees = 40;
  p.rocks = 10;
  p.buildings = 16;
  p.tree_gen = 3;
  p.amp_m = 40.0;
  return p;
}

// An island of the small world's size: ringed by open sea.
generated_world_params island_world() {
  generated_world_params p = small_world();
  p.preset = int(world_preset::island);
  apply_world_preset(p);
  p.half_m = 200.0;
  p.trees = 20;
  p.buildings = 0;
  return p;
}
} // namespace

TEST(WorldWaterBodies, AGeneratedWorldCarriesItsWaterOnTheMeshsSamples) {
  const generated_world w = generate_world(small_world());
  const water_bodies &wb = w.water;
  ASSERT_FALSE(wb.empty());
  const cvc::height_field &t = *w.terrain;
  const int s = water_bodies_stride(t.rows());
  EXPECT_EQ(wb.grid.rows, (t.rows() - 1) / s + 1);
  EXPECT_DOUBLE_EQ(wb.grid.min_x, t.x0());
  EXPECT_DOUBLE_EQ(wb.params.level_m, w.params.water_level_m);
  // Every sample is wet exactly where the mesh's ground is under the water.
  for (int r = 0; r < wb.grid.rows; r += 3)
    for (int c = 0; c < wb.grid.cols; c += 3) {
      const float h = t.height_at(r * s, c * s);
      EXPECT_EQ(wb.ground[wb.index(r, c)], h);
      EXPECT_EQ(wb.body[wb.index(r, c)] >= 0, double(h) < w.params.water_level_m);
    }
  EXPECT_GT(wb.wet_cells(), 0u); // the default relief lays a share of itself under the sea
  EXPECT_NE(water_bodies_summary(wb), "no water");
}

TEST(WorldWaterBodies, AnIslandWorldIsRingedBySea) {
  const generated_world_params p = island_world();
  const generated_world w = generate_world(p);
  ASSERT_GE(w.water.count(water_kind::sea), 1u);
  // The corners are open sea.
  const double e = 0.95 * p.half_m;
  const int id = w.water.body_at(e, e);
  ASSERT_GE(id, 0);
  EXPECT_EQ(w.water.bodies[std::size_t(id)].kind, water_kind::sea);
}

TEST(WorldWaterBodies, LargeWorldsKeepTheGridTo1025) {
  EXPECT_EQ(water_bodies_stride(241), 1);
  EXPECT_EQ(water_bodies_stride(1025), 1);
  EXPECT_EQ(water_bodies_stride(1026), 2);
  EXPECT_EQ(water_bodies_stride(2001), 2);
  EXPECT_EQ(water_bodies_stride(2049), 2);
  for (int n : {129, 1025, 1500, 2001, 2049, 4097})
    EXPECT_LE((n - 1) / water_bodies_stride(n) + 1, 1025) << n;
}

TEST(WorldWaterBodies, TheWaterColourModeDrawsTheBodies) {
  const generated_world w = generate_world(island_world());
  const int T = 128;
  const std::vector<unsigned char> water = paint_surface(w, surface_color::water, T);
  ASSERT_EQ(water.size(), std::size_t(T) * T * 3);
  EXPECT_EQ(water, paint_surface(w, surface_color::water, T)); // deterministic
  EXPECT_NE(water, paint_surface(w, surface_color::material, T));
  // A texel over a body is blue-dominant (sea) or green-dominant (lake, pond); one over dry land
  // is grey.
  const double half = w.params.half_m;
  int wetSeen = 0, drySeen = 0;
  for (int ty = 0; ty < T; ty += 5)
    for (int tx = 0; tx < T; tx += 5) {
      const double x = -half + 2.0 * half * tx / (T - 1), y = -half + 2.0 * half * ty / (T - 1);
      const double sh = w.water.sample(w.water.shore_m, x, y);
      if (std::fabs(sh) < 3.0) // keep off the waterline
        continue;
      const unsigned char *px = &water[(std::size_t(ty) * T + tx) * 3];
      if (sh > 0.0) {
        EXPECT_LT(px[0], std::max(px[1], px[2])) << tx << "," << ty;
        ++wetSeen;
      } else {
        EXPECT_EQ(px[0], px[1]);
        EXPECT_EQ(px[1], px[2]);
        ++drySeen;
      }
    }
  EXPECT_GT(wetSeen, 0);
  EXPECT_GT(drySeen, 0);

  // The water surfaces' texture takes the same colours where it is wet, and keeps its coverage.
  surface_painter sp(w, surface_color::water, T);
  sp.step(0);
  sp.take();
  const std::vector<unsigned char> bodies = sp.take_water(), plain = paint_water(w, T);
  ASSERT_EQ(bodies.size(), plain.size());
  int recoloured = 0;
  for (std::size_t i = 0; i < plain.size(); i += 4) {
    EXPECT_EQ(bodies[i + 3], plain[i + 3]); // the same alpha: wet where the mesh is
    if (plain[i + 3] == 0) {
      EXPECT_EQ(std::memcmp(&bodies[i], &plain[i], 3), 0); // dry texels untouched
      continue;
    }
    const std::size_t t = i / 4;
    const double x = -half + 2.0 * half * double(t % T) / (T - 1),
                 y = -half + 2.0 * half * double(t / T) / (T - 1);
    unsigned char want[3];
    water_bodies_debug_rgb(w.water, x, y, 25.0, want);
    EXPECT_EQ(std::memcmp(&bodies[i], want, 3), 0);
    recoloured += std::memcmp(&bodies[i], &plain[i], 3) != 0;
  }
  EXPECT_GT(recoloured, 0);
}
