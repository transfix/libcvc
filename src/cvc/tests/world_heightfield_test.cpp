/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/world/heightfield.h for the full header).
*/

#include <cmath>
#include <cvc/world/heightfield.h>
#include <gtest/gtest.h>

using namespace cvc::world;

TEST(WorldHeightfield, RollingLandStaysAboveSeaLevel) {
  heightfield_params p;
  p.seed = 3;
  p.amp_m = 8.0;
  p.sea_level_m = 0.0;
  heightfield hf(p);
  double lo = 1e300, hi = -1e300;
  for (int i = 0; i < 4000; ++i) {
    double x = -120 + 0.06 * i, y = 40 - 0.03 * i;
    double h = hf.sample(x, y);
    lo = std::min(lo, h);
    hi = std::max(hi, h);
  }
  EXPECT_GE(lo, 0.0);            // never below sea -> no spurious water obstacles
  EXPECT_LE(hi, p.amp_m + 1e-6); // bounded by amplitude
  EXPECT_GT(hi - lo, 0.5);       // actually varies
}

TEST(WorldHeightfield, Deterministic) {
  heightfield_params p;
  p.seed = 11;
  heightfield a(p), b(p);
  for (int i = 0; i < 100; ++i) {
    double x = i * 1.7 - 50, y = i * -0.9 + 20;
    EXPECT_DOUBLE_EQ(a.sample(x, y), b.sample(x, y));
  }
}

TEST(WorldHeightfield, SeedChangesTheField) {
  heightfield_params p;
  p.seed = 1;
  heightfield a(p);
  p.seed = 2;
  heightfield b(p);
  int diff = 0;
  for (int i = 0; i < 200; ++i) {
    double x = i * 1.3, y = i * 0.7;
    if (std::fabs(a.sample(x, y) - b.sample(x, y)) > 1e-6)
      ++diff;
  }
  EXPECT_GT(diff, 150);
}

TEST(WorldHeightfield, IslandHasWaterAtEdges) {
  heightfield_params p;
  p.island = true;
  p.island_peak_m = 40;
  p.island_radius_m = 100;
  heightfield hf(p);
  EXPECT_GT(hf.sample(0, 0), 20.0);    // peak in the middle
  EXPECT_LT(hf.sample(115, 115), 0.0); // sea at the corner
}

TEST(WorldHeightfield, SlopeIsFiniteAndFlatWhereExpected) {
  heightfield_params p;
  p.seed = 5;
  p.amp_m = 0.0; // perfectly flat
  heightfield hf(p);
  EXPECT_NEAR(hf.slope_deg(3, 4), 0.0, 1e-6);
}
