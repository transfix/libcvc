/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/forest.h for the full header).
*/

#include <cmath>
#include <cvc/geometry/geometry.h>
#include <cvc/lsys/forest.h>
#include <gtest/gtest.h>

using namespace cvc::lsys;

namespace {
// A flat terrain well above the waterline, so every in-disc attempt is accepted — lets the tests
// reason about which ATTEMPT produced which tree.
double flat100(double, double) { return 100.0; }

// A cheap order-sensitive checksum of a mesh's points.
double point_sum(const cvc::geometry &g) {
  double s = 0;
  for (const auto &p : g.const_points())
    s += p[0] * 1.0 + p[1] * 3.0 + p[2] * 7.0;
  return s;
}
} // namespace

TEST(LsysForest, GrowForestIsDeterministic) {
  forest_params p;
  p.seed = 4242;
  p.count = 24;
  p.span = 60.0;
  cvc::geometry wa, na, wb, nb;
  const forest_result ra = grow_forest(p, flat100, wa, na);
  const forest_result rb = grow_forest(p, flat100, wb, nb);
  EXPECT_EQ(ra.planted, rb.planted);
  EXPECT_EQ(ra.pines, rb.pines);
  ASSERT_EQ(wa.const_points().size(), wb.const_points().size());
  ASSERT_EQ(na.const_points().size(), nb.const_points().size());
  EXPECT_DOUBLE_EQ(point_sum(wa), point_sum(wb)); // byte-identical re-generation
  EXPECT_DOUBLE_EQ(point_sum(na), point_sum(nb));
}

TEST(LsysForest, PlantsTheRequestedCountOnOpenGround) {
  forest_params p;
  p.count = 30;
  p.span = 80.0;
  cvc::geometry w, n;
  const forest_result r = grow_forest(p, flat100, w, n);
  EXPECT_EQ(r.planted, 30); // the disc is large + all-dry, so the cap is never the binding limit
  EXPECT_GT(w.const_tris().size(), 0u);
}

TEST(LsysForest, InsertionStabilityGrowingTheCountKeepsThePrefix) {
  // Hashed-by-attempt placement: asking for MORE trees must not move the ones already there.
  forest_params a;
  a.seed = 7;
  a.count = 10;
  a.span = 70.0;
  forest_params b = a;
  b.count = 40;
  cvc::geometry wa, na, wb, nb;
  grow_forest(a, flat100, wa, na);
  grow_forest(b, flat100, wb, nb);
  ASSERT_LE(wa.const_points().size(), wb.const_points().size());
  // The first count=10 forest is a prefix of the count=40 forest (same attempts 0..k).
  for (std::size_t i = 0; i < wa.const_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(wa.const_points()[i][0], wb.const_points()[i][0]);
    EXPECT_DOUBLE_EQ(wa.const_points()[i][1], wb.const_points()[i][1]);
    EXPECT_DOUBLE_EQ(wa.const_points()[i][2], wb.const_points()[i][2]);
  }
}

TEST(LsysForest, SpeciesSelectorsControlTheMix) {
  forest_params pine;
  pine.species = species_mix::pine;
  pine.count = 16;
  pine.span = 60.0;
  cvc::geometry pw, pn;
  const forest_result pr = grow_forest(pine, flat100, pw, pn);
  EXPECT_EQ(pr.pines, pr.planted);        // all conifers
  EXPECT_GT(pn.const_lines().size(), 0u); // pines have needle LINES

  forest_params broad;
  broad.species = species_mix::branchy;
  broad.count = 16;
  broad.span = 60.0;
  cvc::geometry bw, bn;
  const forest_result br = grow_forest(broad, flat100, bw, bn);
  EXPECT_EQ(br.pines, 0);                  // no conifers
  EXPECT_EQ(bn.const_points().size(), 0u); // branchy has no needle mesh
  EXPECT_GT(bw.const_tris().size(), 0u);
}

TEST(LsysForest, SeaLevelGateRejectsBelowWaterline) {
  // Terrain entirely below the waterline -> nothing is planted.
  forest_params p;
  p.count = 20;
  p.span = 50.0;
  p.sea_level = 10.0;
  cvc::geometry w, n;
  const forest_result r = grow_forest(p, [](double, double) { return 0.0; }, w, n);
  EXPECT_EQ(r.planted, 0);
  EXPECT_EQ(w.const_points().size(), 0u);
}

TEST(LsysForest, SinglePineHasWoodNeedlesAndWindRecords) {
  cvc::geometry wood, needle;
  forest_wind::tree tr;
  grow_pine(wood, needle, 0.0, 0.0, 0.0, 1.2, /*maturity=*/3, &tr, /*phase=*/0.5, /*sway=*/0.03);
  EXPECT_GT(wood.const_tris().size(), 0u);
  EXPECT_GT(needle.const_lines().size(), 0u);
  EXPECT_GT(tr.mods.size(), 0u);
  EXPECT_DOUBLE_EQ(tr.phase, 0.5);
  EXPECT_DOUBLE_EQ(tr.sway, 0.03);
}

TEST(LsysForest, WindReposeMovesSwayerVerticesOverTime) {
  forest_params p;
  p.species = species_mix::pine;
  p.count = 6;
  p.span = 40.0;
  cvc::geometry wood, needle;
  forest_wind wind;
  grow_forest(p, flat100, wood, needle, &wind);
  ASSERT_GT(wind.trees.size(), 0u);

  // Seed the flat buffers from the (static) mesh, then re-pose at two different times.
  auto flatten = [](const cvc::geometry &g) {
    std::vector<double> b(g.const_points().size() * 3);
    for (std::size_t v = 0; v < g.const_points().size(); ++v) {
      b[v * 3] = g.const_points()[v][0];
      b[v * 3 + 1] = g.const_points()[v][1];
      b[v * 3 + 2] = g.const_points()[v][2];
    }
    return b;
  };
  std::vector<double> w1 = flatten(wood), n1 = flatten(needle);
  std::vector<double> w2 = w1, n2 = n1;
  repose_forest(wind, /*t=*/0.0, /*wind_scale=*/1.0, w1, n1);
  repose_forest(wind, /*t=*/1.0, /*wind_scale=*/1.0, w2, n2);
  // Some wood vertex differs between the two poses (the sway cascade actually moved something).
  bool moved = false;
  for (std::size_t i = 0; i < w1.size(); ++i)
    if (std::fabs(w1[i] - w2[i]) > 1e-9) {
      moved = true;
      break;
    }
  EXPECT_TRUE(moved);
}
