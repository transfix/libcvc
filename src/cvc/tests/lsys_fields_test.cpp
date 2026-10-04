/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/cloud.h for the full header).
*/

#include <cmath>
#include <cvc/lsys/cloud.h>
#include <cvc/lsys/water.h>
#include <gtest/gtest.h>

using namespace cvc::lsys;

namespace {
bool in_unit(const std::vector<float> &f) {
  for (float v : f)
    if (v < -1e-6f || v > 1.0f + 1e-6f)
      return false;
  return true;
}
double sum(const std::vector<float> &f) {
  double s = 0;
  for (float v : f)
    s += v;
  return s;
}
} // namespace

// ── cloud ─────────────────────────────────────────────────────────────────────────────────────
TEST(LsysCloud, FieldIsDeterministicAndSized) {
  cloud_params p;
  p.seed = 99;
  p.n = 24;
  p.nz = 12; // smaller grid keeps the test fast
  const std::vector<float> a = cloud_field(p);
  const std::vector<float> b = cloud_field(p);
  ASSERT_EQ(a.size(), static_cast<std::size_t>(p.n) * p.n * p.nz);
  EXPECT_EQ(a, b); // byte-identical re-generation
  EXPECT_TRUE(in_unit(a));
  EXPECT_GT(sum(a), 0.0); // some cloud exists
}

TEST(LsysCloud, VariantsDecorrelate) {
  cloud_params p;
  p.seed = 7;
  p.n = 24;
  p.nz = 12;
  cloud_params q = p;
  q.variant = 1;
  const std::vector<float> a = cloud_field(p);
  const std::vector<float> b = cloud_field(q);
  ASSERT_EQ(a.size(), b.size());
  EXPECT_NE(a, b); // a different variant grows a different cloud
}

TEST(LsysCloud, NonPositiveDimsGiveEmpty) {
  cloud_params p;
  p.n = 0;
  EXPECT_TRUE(cloud_field(p).empty());
}

// ── water ─────────────────────────────────────────────────────────────────────────────────────
TEST(LsysWater, SeaSurfaceVariesInTime) {
  const double a = sea_surface(3.0, -2.0, 0.0, 0.0, 2.4);
  const double b = sea_surface(3.0, -2.0, 1.5, 0.0, 2.4);
  EXPECT_GT(std::fabs(a - b), 1e-6); // the wave rolls
}

TEST(LsysWater, FieldIsDeterministicInUnitAndSized) {
  sea_params p;
  p.n = 20;
  p.nz = 10;
  const std::vector<float> terrain(static_cast<std::size_t>(p.n) * p.n, -5.0f); // seabed below
  std::vector<float> a, b;
  sea_field(p, terrain, 0.3, a);
  sea_field(p, terrain, 0.3, b);
  ASSERT_EQ(a.size(), static_cast<std::size_t>(p.n) * p.n * p.nz);
  EXPECT_EQ(a, b);
  EXPECT_TRUE(in_unit(a));
  EXPECT_GT(sum(a), 0.0); // there is water above a deep seabed
}

TEST(LsysWater, TerrainAboveSurfaceLeavesNoWater) {
  sea_params p;
  p.n = 16;
  p.nz = 8;
  p.sea_level = 0.0;
  // Terrain well above the sea top -> every column is dry land, no water voxels.
  const std::vector<float> terrain(static_cast<std::size_t>(p.n) * p.n,
                                   static_cast<float>(sea_top(p) + 5.0));
  std::vector<float> f;
  sea_field(p, terrain, 0.0, f);
  EXPECT_DOUBLE_EQ(sum(f), 0.0);
}
