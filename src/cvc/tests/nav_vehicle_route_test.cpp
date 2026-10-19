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
  EXPECT_EQ(bi.reverse_length, 0.0) << "the bicycle turns round in the street";
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

// A city of 40 m blocks and 12 m streets, 384 x 384 cells of 1 m: the route is found in
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
    if (w == 1.0)
      exact_length = r.length;
    else
      EXPECT_LE(r.length, w * exact_length + 1e-9) << "weighted A*'s bound";
    EXPECT_LT(ms, 5000.0);
  }
}
