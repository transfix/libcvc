/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::plan_vehicle_route (vehicle_route.h): routes a vehicle with a turning circle, a body
// and a reverse gear can drive. The vehicle is a Humvee as the physics plant sizes it (P9): a
// 6.71 m turning radius at the rear axle (3.30 m wheelbase, 0.457 rad lock) and four 1.42 m
// footprint discs from the rear axle to the nose. The ground is 1 m cells.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/vehicle_route.h>
#include <gtest/gtest.h>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A W x H metre world of 1 m cells, walled round its edge.
struct ground {
  int cols, rows;
  std::vector<std::uint8_t> occ;
  ground(int w, int h)
      : cols(w + 1), rows(h + 1), occ(static_cast<std::size_t>(w + 1) * (h + 1), 0) {
    box(0, 0, w, 0);
    box(0, h, w, h);
    box(0, 0, 0, h);
    box(w, 0, w, h);
  }
  void box(int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; ++y)
      for (int x = x0; x <= x1; ++x)
        if (x >= 0 && y >= 0 && x < cols && y < rows)
          occ[static_cast<std::size_t>(y) * cols + x] = 1;
  }
  vehicle_route plan(const route_vehicle &v, route_pose s, double gx, double gy,
                     route_options o = {}) const {
    return plan_vehicle_route(occ.data(), rows, cols, 0.0, 0.0, cols - 1.0, rows - 1.0, v, s, gx,
                              gy, o);
  }
  double clearance(const route_vehicle &v, const route_pose &p) const {
    return route_clearance(occ.data(), rows, cols, 0.0, 0.0, cols - 1.0, rows - 1.0, v, p);
  }
};

route_vehicle humvee() {
  route_vehicle v;
  v.min_radius = 3.30 / std::tan(0.457);
  v.disc_offsets = {-0.01, 1.10, 2.20, 3.31};
  v.disc_radius = 1.42;
  return v;
}

route_vehicle bicycle() { // nav's kinematic bicycle at the trained metric: L 0.7 m, lock 0.6
  route_vehicle v;
  v.min_radius = 0.7 / std::tan(0.6);
  v.disc_radius = 1.0;
  return v;
}

double wrap_pi(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a < -kPi)
    a += 2.0 * kPi;
  return a;
}

// Every move turns no tighter than the vehicle can, and every pose (past `skip`) keeps its body
// clear of the walls.
void expect_drivable(const ground &g, const route_vehicle &v, const vehicle_route &r, double step,
                     std::size_t skip = 0) {
  for (std::size_t k = 1; k < r.poses.size(); ++k) {
    const double turn = std::fabs(wrap_pi(r.poses[k].heading - r.poses[k - 1].heading));
    EXPECT_LE(turn, step / v.min_radius + 1e-9) << "move " << k << " turns too tight";
  }
  for (std::size_t k = skip; k < r.poses.size(); ++k)
    EXPECT_GE(g.clearance(v, r.poses[k]), -1e-9) << "pose " << k << " is in a wall";
}

} // namespace

TEST(VehicleRoute, DrivesStraightAcrossOpenGround) {
  const ground g(100, 60);
  const vehicle_route r = g.plan(humvee(), {10, 30, 0}, 80, 30);
  ASSERT_TRUE(r.reached);
  EXPECT_EQ(r.cusps, 0);
  EXPECT_EQ(r.reverse_length, 0.0);
  EXPECT_NEAR(r.length, 70.0, 3.0);
  expect_drivable(g, humvee(), r, 1.5);
}

// A soft keep-out (route_options::avoid): a corridor another vehicle means to drive, laid along
// the straight line, bends the route round it where the ground allows; at no cost it changes
// nothing.
TEST(VehicleRoute, KeepsOffACorridorOthersMeanToDrive) {
  const ground g(100, 60);
  const route_pose s{10, 30, 0};
  route_options o;
  route_corridor c;
  c.xy = {30.0, 30.0, 60.0, 30.0};
  c.radius = 8.0;
  c.cost = 3.0;
  o.avoid = {c};
  const vehicle_route plain = g.plan(humvee(), s, 80, 30);
  const vehicle_route kept = g.plan(humvee(), s, 80, 30, o);
  ASSERT_TRUE(kept.reached);
  double off_plain = 0.0, off_kept = 0.0;
  for (const route_pose &q : plain.poses)
    if (q.x > 35 && q.x < 55)
      off_plain = std::max(off_plain, std::fabs(q.y - 30.0));
  for (const route_pose &q : kept.poses)
    if (q.x > 35 && q.x < 55)
      off_kept = std::max(off_kept, std::fabs(q.y - 30.0));
  std::printf("  a corridor along the line: the route stays %.1f m off it (%.1f m without), "
              "%.1f m long (%.1f m)\n",
              off_kept, off_plain, kept.length, plain.length);
  EXPECT_LT(off_plain, 1.0);
  EXPECT_GT(off_kept, 5.0) << "the route should keep off the corridor";
  expect_drivable(g, humvee(), kept, 1.5);
  o.avoid[0].cost = 0.0;
  const vehicle_route free_ = g.plan(humvee(), s, 80, 30, o);
  EXPECT_EQ(free_.poses.size(), plain.poses.size());
  EXPECT_DOUBLE_EQ(free_.length, plain.length);
}

// The ground (route_options::terrain): a band across the straight line (x 30..60, y 22..38) whose
// slope runs north (dz/dy 0.4, 22 deg), so a route straight through it has 0.4 across its heading.
// With a side limit of 0.3 no pose in the band has more than that across it (the route climbs it
// head on, or goes round) unless it starts there; with a cost alone it keeps off the band; a ramp
// along the heading (dz/dx 0.4) is taken straight on, since only the slope across is limited.
TEST(VehicleRoute, PlansOffSlopeAcrossItsHeading) {
  const ground g(100, 60);
  const route_pose s{10, 30, 0};
  std::vector<float> slope(2 * static_cast<std::size_t>(g.rows) * g.cols, 0.0f);
  const std::size_t plane = static_cast<std::size_t>(g.rows) * g.cols;
  for (int r = 22; r <= 38; ++r)
    for (int c = 30; c <= 60; ++c)
      slope[plane + static_cast<std::size_t>(r) * g.cols + c] = 0.4f; // dz/dy
  route_options o;
  o.terrain.slope = slope.data();
  o.terrain.rows = g.rows;
  o.terrain.cols = g.cols;
  o.terrain.max_x = g.cols - 1.0;
  o.terrain.max_y = g.rows - 1.0;
  o.terrain.side_max = 0.3;
  const auto inside = [](const vehicle_route &r) {
    int n = 0;
    for (const route_pose &q : r.poses)
      n += q.x > 31 && q.x < 59 && q.y > 23 && q.y < 37 ? 1 : 0;
    return n;
  };
  const vehicle_route plain = g.plan(humvee(), s, 80, 30);
  const vehicle_route hard = g.plan(humvee(), s, 80, 30, o);
  route_options soft = o;
  soft.terrain.side_max = 0.0;
  soft.terrain.side_cost = 4.0;
  const vehicle_route costed = g.plan(humvee(), s, 80, 30, soft);
  std::printf(
      "  side slope across the line: poses in the band %d (plain), %d (limit 0.3), %d (cost "
      "4); lengths %.1f, %.1f, %.1f m\n",
      inside(plain), inside(hard), inside(costed), plain.length, hard.length, costed.length);
  ASSERT_TRUE(hard.reached);
  EXPECT_GT(inside(plain), 0);
  double worst = 0.0; // the most slope across its heading of any pose in the band
  for (const route_pose &q : hard.poses)
    if (q.x > 31 && q.x < 59 && q.y > 23 && q.y < 37)
      worst = std::max(worst, std::fabs(std::cos(q.heading)) * 0.4);
  EXPECT_LE(worst, 0.3 + 1e-6) << "past the side limit is infeasible";
  EXPECT_LT(inside(costed), inside(plain)) << "the cost should keep it mostly off the slope";
  expect_drivable(g, humvee(), hard, 1.5);
  // Standing across the slope in the band: it may still drive off it.
  EXPECT_TRUE(g.plan(humvee(), {45, 30, 0}, 80, 30, o).reached) << "a steep start drives off";
  // The same slope running east (along the heading): taken straight on.
  for (std::size_t k = 0; k < plane; ++k) {
    slope[k] = slope[plane + k];
    slope[plane + k] = 0.0f;
  }
  const vehicle_route along = g.plan(humvee(), s, 80, 30, o);
  ASSERT_TRUE(along.reached);
  EXPECT_GT(inside(along), 0) << "a ramp taken straight on is not limited";
  // Off (no limits, no costs): the plain route.
  route_options off;
  off.terrain.slope = slope.data();
  off.terrain.rows = g.rows;
  off.terrain.cols = g.cols;
  off.terrain.max_x = g.cols - 1.0;
  off.terrain.max_y = g.rows - 1.0;
  EXPECT_DOUBLE_EQ(g.plan(humvee(), s, 80, 30, off).length, plain.length);
}

// A goal behind the vehicle on open ground: it turns round within its circle.
TEST(VehicleRoute, TurnsRoundWithinItsCircle) {
  const ground g(100, 60);
  const vehicle_route r = g.plan(humvee(), {50, 30, 0}, 20, 30);
  ASSERT_TRUE(r.reached);
  std::printf("  turning round in the open: %.1f m (%.1f backing up), %d cusps, %d expansions\n",
              r.length, r.reverse_length, r.cusps, r.expansions);
  expect_drivable(g, humvee(), r, 1.5);
}

// An 8 m street closed at its end, the Humvee facing the end and the goal behind it outside. A
// U-turn needs a 13.4 m circle plus its width, so it backs out; nav's bicycle turns in the street.
TEST(VehicleRoute, BacksOutOfADeadEndTheBicycleTurnsIn) {
  ground g(100, 60);
  g.box(40, 0, 70, 25);  // the street's south side
  g.box(40, 35, 70, 60); // its north side: 9 cells of street (26..34)
  g.box(71, 0, 100, 60); // its end
  const vehicle_route hv = g.plan(humvee(), {58, 30, 0}, 15, 30);
  ASSERT_TRUE(hv.reached);
  std::printf(
      "  the Humvee out of the dead end: %.1f m, %.1f backing up, %d cusps, %d expansions\n",
      hv.length, hv.reverse_length, hv.cusps, hv.expansions);
  EXPECT_GT(hv.reverse_length, 10.0) << "it backed out";
  expect_drivable(g, humvee(), hv, 1.5);
  const vehicle_route bi = g.plan(bicycle(), {58, 30, 0}, 15, 30);
  ASSERT_TRUE(bi.reached);
  std::printf("  the bicycle: %.1f m, %.1f backing up, %d cusps\n", bi.length, bi.reverse_length,
              bi.cusps);
  EXPECT_LT(bi.reverse_length, 5.0) << "the bicycle turns round in the street, not backing out";
}

// A right-angle corner between two 9 m streets: drivable, its turns within the circle and its
// body clear of the walls.
TEST(VehicleRoute, TakesANarrowCorner) {
  ground g(100, 100);
  g.box(0, 0, 100, 25);    // south of the east-west street (26..34)
  g.box(0, 35, 60, 100);   // north of it, west of the north-south street (61..69)
  g.box(70, 35, 100, 100); // east of the north-south street
  const vehicle_route r = g.plan(humvee(), {15, 30, 0}, 65, 90);
  ASSERT_TRUE(r.reached);
  std::printf("  round the corner: %.1f m, %.1f backing up, %d cusps, %d expansions\n", r.length,
              r.reverse_length, r.cusps, r.expansions);
  expect_drivable(g, humvee(), r, 1.5);
}

// The Humvee wedged with its nose on the dead end's wall: the route still leads out, backing up.
TEST(VehicleRoute, LeadsAWedgedVehicleOut) {
  ground g(100, 60);
  g.box(40, 0, 70, 25);
  g.box(40, 35, 70, 60);
  g.box(71, 0, 100, 60);
  const route_pose wedged{70.5 - 3.31 - 1.0, 30, 0}; // the front disc 0.4 m into the wall
  ASSERT_LT(g.clearance(humvee(), wedged), 0.0);
  const vehicle_route r = g.plan(humvee(), wedged, 15, 30);
  ASSERT_TRUE(r.reached);
  ASSERT_GT(r.poses.size(), 2u);
  EXPECT_TRUE(r.poses[1].reverse) << "its first move backs away from the wall";
  // Once out of the escape it keeps clear.
  expect_drivable(g, humvee(), r, 1.5, 6);
}

// A goal walled in: no route, within the expansion budget, ending at the nearest it got.
TEST(VehicleRoute, ReportsAGoalItCannotReach) {
  ground g(100, 60);
  g.box(70, 20, 90, 20);
  g.box(70, 40, 90, 40);
  g.box(70, 20, 70, 40);
  g.box(90, 20, 90, 40);
  route_options o;
  o.max_expansions = 20000;
  const vehicle_route r = g.plan(humvee(), {20, 30, 0}, 80, 30, o);
  EXPECT_FALSE(r.reached);
  EXPECT_LE(r.expansions, o.max_expansions);
  ASSERT_FALSE(r.poses.empty());
  EXPECT_EQ(r.poses.front().x, 20.0);
}

// A city of 40 m blocks and 12 m streets, 384 x 384 cells of 1 m: the weighted route is found in
// interactive time.
TEST(VehicleRoute, PlansAcrossACity) {
  ground g(383, 383);
  for (int bx = 0; bx < 383; bx += 52)
    for (int by = 0; by < 383; by += 52)
      g.box(bx + 12, by + 12, bx + 51, by + 51);
  double exact_length = 0.0;
  for (const double w : {1.0, 1.5, 2.0}) {
    route_options o;
    o.heuristic_weight = w;
    const auto t0 = std::chrono::steady_clock::now();
    const vehicle_route r = g.plan(humvee(), {6, 6, 0}, 370, 370, o);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ASSERT_TRUE(r.reached);
    std::printf("  across the city, heuristic x%.1f: %.0f m, %d cusps, %d expansions, %.1f ms\n", w,
                r.length, r.cusps, r.expansions, ms);
    expect_drivable(g, humvee(), r, 1.5);
    if (w == 1.0) {
      exact_length = r.length;
    } else {
      EXPECT_LE(r.length, w * exact_length + 1e-9) << "weighted A*'s bound";
      // The weighted search is what makes it interactive. Gated on expansions, not time: the
      // exact search takes 0.6 s in a Release build and 6 s in a Debug one.
      EXPECT_LT(r.expansions, 5000);
    }
  }
}
