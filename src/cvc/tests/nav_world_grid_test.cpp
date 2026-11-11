/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::world_grid: a world as the navigator sees it (world_grid.h). Its occupancy walled at
// the edge, a crop round the terrain's centre, the signed distance, free points drawn the same way
// on every platform, and a navigator's config filled from it.

#include <cmath>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/world_grid.h>
#include <cvc/world/world_geometry.h>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>

using namespace cvc::nav;
using cvc::world::world_geometry;

namespace {

void add_box(cvc::geometry &g, double x0, double y0, double x1, double y1, double z0, double h) {
  const auto base = static_cast<cvc::index_t>(g.num_points());
  for (int k = 0; k < 8; ++k)
    g.points().push_back({{(k & 1) ? x1 : x0, (k & 2) ? y1 : y0, (k & 4) ? z0 + h : z0}});
  const int idx[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                       2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  for (int t = 0; t < 12; ++t)
    g.tris().push_back({{base + idx[3 * t], base + idx[3 * t + 1], base + idx[3 * t + 2]}});
}

// 21 x 21 samples over [0, 20]^2, rising 0.1 m per metre east, and a building over [8, 12]^2.
world_geometry town() {
  world_geometry w;
  std::vector<float> h(21 * 21);
  for (int r = 0; r < 21; ++r)
    for (int c = 0; c < 21; ++c)
      h[r * 21 + c] = 0.1f * c;
  w.add({"terrain", std::make_shared<cvc::height_field>(21, 21, 0.0, 0.0, 1.0, 1.0, h)});
  auto city = std::make_shared<cvc::geometry>();
  add_box(*city, 8.0, 8.0, 12.0, 12.0, 0.0, 10.0);
  w.add({"buildings", city, w.surfaces().by_name("building_concrete")});
  return w;
}

std::uint8_t at(const world_grid &g, int r, int c) {
  return g.occupancy[static_cast<std::size_t>(r) * g.cols + c];
}

} // namespace

TEST(NavWorldGrid, AWorldBecomesAWalledGrid) {
  const world_grid g = make_grid(town(), 1.0);
  EXPECT_EQ(g.rows, 21);
  EXPECT_EQ(g.cols, 21);
  EXPECT_DOUBLE_EQ(g.cell(), 1.0);
  EXPECT_DOUBLE_EQ(g.center_x(), 10.0);
  // The wall: every edge sample, so no agent leaves the grid.
  for (int i = 0; i < 21; ++i) {
    EXPECT_EQ(at(g, 0, i), 1);
    EXPECT_EQ(at(g, 20, i), 1);
    EXPECT_EQ(at(g, i, 0), 1);
    EXPECT_EQ(at(g, i, 20), 1);
  }
  // The building blocks its footprint; the open ground between is free.
  EXPECT_EQ(at(g, 10, 10), 1);
  EXPECT_EQ(at(g, 5, 5), 0);
  EXPECT_TRUE(g.occupied(10.2, 9.8));
  EXPECT_FALSE(g.occupied(4.0, 15.0));
  EXPECT_TRUE(g.occupied(-1.0, 5.0)); // off the grid
  // Heights are the terrain's.
  EXPECT_NEAR(g.height[5 * 21 + 7], 0.7f, 1e-5f);
  // 80 wall samples and the building's 5 x 5.
  EXPECT_EQ(g.occupied_count(), 80u + 25u);
}

TEST(NavWorldGrid, HalfCropsRoundTheTerrainsCentreAndTheCellCutsTheExtent) {
  const world_grid g = make_grid(town(), 1.0, 5.0);
  EXPECT_DOUBLE_EQ(g.min_x, 5.0);
  EXPECT_DOUBLE_EQ(g.max_y, 15.0);
  EXPECT_EQ(g.rows, 11);
  EXPECT_EQ(at(g, 5, 5), 1); // the building, now in the middle
  // 3 m cells over 20 m: 7 samples, the extent cut to 18 m from its minimum corner.
  const world_grid h = make_grid(town(), 0.0, 0.0, 20.0, 20.0, 3.0);
  EXPECT_EQ(h.cols, 7);
  EXPECT_DOUBLE_EQ(h.max_x, 18.0);
  EXPECT_DOUBLE_EQ(h.cell(), 3.0);
}

TEST(NavWorldGrid, AFlatSquareIsOpenInsideItsWall) {
  const world_grid g = flat_grid(10.0, 1.0);
  EXPECT_EQ(g.rows, 21);
  EXPECT_DOUBLE_EQ(g.min_x, -10.0);
  EXPECT_EQ(g.occupied_count(), 80u);
  EXPECT_FALSE(g.occupied(0.0, 0.0));
  EXPECT_FLOAT_EQ(g.height[200], 0.0f);
}

TEST(NavWorldGrid, TheSignedDistanceIsPositiveInTheFreeAndNegativeInTheBlocked) {
  const world_grid g = flat_grid(10.0, 2.0); // 11 x 11, 2 m cells
  const std::vector<float> d = signed_distance(g);
  ASSERT_EQ(d.size(), 121u);
  EXPECT_FLOAT_EQ(d[5 * 11 + 5], 10.0f);          // the centre: 5 cells from the wall
  EXPECT_FLOAT_EQ(d[1 * 11 + 5], 2.0f);           // beside the wall
  EXPECT_FLOAT_EQ(d[0 * 11 + 5], -2.0f);          // in the wall: a cell from the free ground
  EXPECT_FLOAT_EQ(d[0], -2.0f * std::sqrt(2.0f)); // the corner, diagonally
}

TEST(NavWorldGrid, FreePointsAreClearApartAndTheSameEverywhere) {
  const world_grid g = make_grid(town(), 1.0);
  const std::vector<float> d = signed_distance(g);
  const auto a = free_points(g, d, 7, 4, 2.0, 3.0);
  ASSERT_EQ(a.size(), 4u);
  for (std::size_t i = 0; i < a.size(); ++i) {
    int r = 0, c = 0;
    g.sample_of(a[i][0], a[i][1], r, c);
    EXPECT_GE(d[static_cast<std::size_t>(r) * g.cols + c], 2.0f);
    for (std::size_t j = 0; j < i; ++j)
      EXPECT_GE(std::hypot(a[i][0] - a[j][0], a[i][1] - a[j][1]), 3.0);
  }
  // The same seed draws the same points; another seed draws others.
  EXPECT_EQ(free_points(g, d, 7, 4, 2.0, 3.0), a);
  EXPECT_NE(free_points(g, d, 8, 4, 2.0, 3.0), a);
  // splitmix64, not a std distribution: pinned, so every platform draws these.
  EXPECT_EQ(a[0], (std::array<double, 2>{4.0, 7.0}));
  // No sample that clear: any free one. Nothing free: refused.
  const auto loose = free_points(g, d, 7, 2, 50.0, 0.0);
  EXPECT_FALSE(g.occupied(loose[0][0], loose[0][1]));
  world_grid shut = g;
  std::fill(shut.occupancy.begin(), shut.occupancy.end(), 1);
  EXPECT_THROW(free_points(shut, signed_distance(shut), 7, 1, 1.0, 0.0), std::runtime_error);
  EXPECT_THROW(free_points(g, {}, 7, 1, 1.0, 0.0), std::invalid_argument);
}

TEST(NavWorldGrid, ConfigureFillsTheNavigatorsGrid) {
  const world_grid g = make_grid(town(), 1.0, 5.0);
  sim_world::config cfg;
  cfg.range_m = 33.0;
  configure(cfg, g, 0.05);
  EXPECT_EQ(cfg.rows, 11);
  EXPECT_EQ(cfg.cols, 11);
  EXPECT_DOUBLE_EQ(cfg.min_x, 5.0);
  EXPECT_DOUBLE_EQ(cfg.max_y, 15.0);
  EXPECT_DOUBLE_EQ(cfg.cx, 10.0);
  EXPECT_DOUBLE_EQ(cfg.cy, 10.0);
  EXPECT_DOUBLE_EQ(cfg.scale, 0.05);
  EXPECT_DOUBLE_EQ(cfg.range_m, 33.0); // the rest left alone
}

TEST(NavWorldGrid, RefusesWhatItCannotGrid) {
  EXPECT_THROW(make_grid(town(), 0.0), std::invalid_argument);
  EXPECT_THROW(make_grid(town(), 1.0, 0.0, 0.0, 1.5, 1.5, 1.0), std::invalid_argument);
  EXPECT_THROW(make_grid(world_geometry{}, 1.0), std::invalid_argument);
  EXPECT_THROW(flat_grid(0.0, 1.0), std::invalid_argument);
}
