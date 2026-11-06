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

// cvc::world::cloud_layer — the world-scale cloud deck: its period covers the world, it holds the
// clouds asked for, it is deterministic, its morphs are continuous, and its shadow lies where the
// sun puts it.

#include <algorithm>
#include <cmath>
#include <cvc/lsys/cloud.h>
#include <cvc/world/cloud_layer.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::world::cloud_cell;
using cvc::world::cloud_layer;
using cvc::world::cloud_layer_params;

namespace {
// A small deck that builds fast: coarse maps, three variants.
cloud_layer_params small(double area = 300.0) {
  cloud_layer_params p;
  p.n = 24;
  p.nz = 12;
  p.variants = 3;
  p.extent = 150.0;
  p.base = 74.0;
  p.top = 122.0;
  p.area = area;
  p.shadow_texels = 32;
  p.far_texels = 8;
  p.far_layers = 4;
  return p;
}
} // namespace

TEST(WorldCloudLayer, PeriodCoversTheWorldAndHoldsTheCoverage) {
  cloud_layer_params p = small(1000.0);
  const cloud_layer L(p);
  EXPECT_DOUBLE_EQ(L.spacing(), 2.5 * p.extent);
  EXPECT_GE(L.period_m(), 2.0 * p.area);
  EXPECT_EQ(L.period_cells(), 6);
  EXPECT_EQ(L.cells().size(), 36u);
  EXPECT_EQ(L.occupied(), 22); // round(0.6 * 36)
  EXPECT_EQ(L.shadow_size(), 6 * 32);
  EXPECT_EQ(L.far_size(), 6 * 8);
  // A small world still gets a period of two cells and at least one cloud.
  cloud_layer_params q = small(10.0);
  q.coverage = 0.01;
  const cloud_layer S(q);
  EXPECT_EQ(S.period_cells(), 2);
  EXPECT_EQ(S.occupied(), 1);
  // A world grown live re-lays the cells over the new period; a cell keeps its place.
  cloud_layer G(small(300.0));
  const double cx = G.cells()[0].cx, cy = G.cells()[0].cy;
  G.set_area(2000.0);
  EXPECT_GE(G.period_m(), 4000.0);
  EXPECT_EQ(G.cells().size(), std::size_t(G.period_cells()) * G.period_cells());
  EXPECT_DOUBLE_EQ(G.cells()[0].cx, cx);
  EXPECT_DOUBLE_EQ(G.cells()[0].cy, cy);
  std::vector<unsigned char> mask;
  G.shadow_mask(0.0, 0.13, 0.3, mask);
  EXPECT_EQ(mask.size(), std::size_t(G.shadow_size()) * G.shadow_size());
}

TEST(WorldCloudLayer, IsDeterministic) {
  const cloud_layer a(small()), b(small());
  std::vector<unsigned char> ma, mb, fa, fb;
  a.shadow_mask(0.7, 0.13, 0.3, ma);
  b.shadow_mask(0.7, 0.13, 0.3, mb);
  EXPECT_EQ(ma, mb);
  a.far_density(0.7, {}, fa);
  b.far_density(0.7, {}, fb);
  EXPECT_EQ(fa, fb);
  cloud_layer_params q = small();
  q.seed = 21;
  const cloud_layer c(q);
  std::vector<unsigned char> mc;
  c.shadow_mask(0.7, 0.13, 0.3, mc);
  EXPECT_NE(ma, mc); // another seed, another sky
}

TEST(WorldCloudLayer, ClearSkyCastsNoShadow) {
  cloud_layer_params p = small();
  p.coverage = 0.0;
  const cloud_layer L(p);
  EXPECT_EQ(L.occupied(), 0);
  std::vector<unsigned char> mask, far;
  L.shadow_mask(0.0, 0.13, 0.3, mask);
  ASSERT_EQ(mask.size(), std::size_t(L.shadow_size()) * L.shadow_size());
  EXPECT_TRUE(std::all_of(mask.begin(), mask.end(), [](unsigned char v) { return v == 255; }));
  L.far_density(0.0, {}, far);
  EXPECT_TRUE(std::all_of(far.begin(), far.end(), [](unsigned char v) { return v == 0; }));
}

TEST(WorldCloudLayer, CloudsShadeTheGround) {
  const cloud_layer L(small());
  std::vector<unsigned char> mask;
  L.shadow_mask(0.0, 0.13, 0.3, mask);
  const auto mn = *std::min_element(mask.begin(), mask.end());
  EXPECT_LT(mn, 160);                                          // somewhere is well shaded
  EXPECT_GE(mn, 76);                                           // never below the floor (0.3 * 255)
  EXPECT_EQ(*std::max_element(mask.begin(), mask.end()), 255); // and somewhere is in full sun
}

TEST(WorldCloudLayer, VariantsNeverRepeatInARowAndMorphsAreContinuous) {
  const cloud_layer L(small());
  for (const cloud_cell &c : L.cells())
    for (long k = 0; k < 12; ++k)
      EXPECT_NE(L.cell_variant(c, k), L.cell_variant(c, k + 1));
  // Across a variant change the field and the shadow are continuous: the end of one crossfade is
  // the start of the next.
  const cloud_cell &c = L.cells()[0];
  const double step = std::ceil(c.phase) - c.phase + 3.0; // an integer morph step for this cell
  std::vector<float> f0, f1;
  L.cell_field(c, step - 1e-6, f0);
  L.cell_field(c, step + 1e-6, f1);
  ASSERT_EQ(f0.size(), f1.size());
  float worst = 0.0f;
  for (std::size_t k = 0; k < f0.size(); ++k)
    worst = std::max(worst, std::fabs(f0[k] - f1[k]));
  EXPECT_LT(worst, 1e-4f);
  // At a step a cell shows exactly one variant's map.
  const cloud_layer::morph_state s = L.cell_morph(c, step);
  EXPECT_NEAR(s.u, 0.0, 1e-9);
  L.cell_field(c, step, f0);
  EXPECT_EQ(f0, L.variant_map(s.a));
}

TEST(WorldCloudLayer, TheShadowLiesAwayFromTheSun) {
  // One cloud; its shadow on the base plane is its mass moved down the sun's rays: the
  // tau-weighted centre of the mask sits (zc - base) / tan(el) from the cloud's mass centre,
  // away from the sun.
  cloud_layer_params p = small(10.0);
  p.coverage = 0.01;
  p.sun_az = -60.0;
  p.sun_el = 40.0;
  const cloud_layer L(p);
  const cloud_cell *one = nullptr;
  for (const cloud_cell &c : L.cells())
    if (c.occupied)
      one = &c;
  ASSERT_NE(one, nullptr);
  const double m = 0.25;
  std::vector<float> f;
  L.cell_field(*one, m, f);
  const int n = p.n, nz = p.nz;
  double W = 0, X = 0, Y = 0, Z = 0;
  for (int z = 0; z < nz; ++z)
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x) {
        const double v = f[cvc::lsys::cloud_index(n, x, y, z)];
        W += v;
        X += v * (one->cx - p.extent + 2.0 * p.extent * x / (n - 1));
        Y += v * (one->cy - p.extent + 2.0 * p.extent * y / (n - 1));
        Z += v * (p.base + (p.top - p.base) * z / (nz - 1));
      }
  ASSERT_GT(W, 0.0);
  X /= W;
  Y /= W;
  Z /= W;
  double sx, sy, sz;
  L.sun_dir(sx, sy, sz);
  const double ex = X - (Z - p.base) * sx / sz, ey = Y - (Z - p.base) * sy / sz;

  std::vector<float> tau;
  L.shadow_tau(m, tau);
  const int N = L.shadow_size();
  const double P = L.period_m(), ts = P / N;
  auto wrapd = [P](double d) { return d - P * std::floor(d / P + 0.5); };
  double T = 0, dx = 0, dy = 0;
  for (int ty = 0; ty < N; ++ty)
    for (int tx = 0; tx < N; ++tx) {
      const double t = tau[std::size_t(ty) * N + tx];
      T += t;
      dx += t * wrapd((tx + 0.5) * ts - ex);
      dy += t * wrapd((ty + 0.5) * ts - ey);
    }
  ASSERT_GT(T, 0.0);
  EXPECT_NEAR(dx / T, 0.0, 2.0 * ts);
  EXPECT_NEAR(dy / T, 0.0, 2.0 * ts);
  // base_point is the same projection: a receiver at the base is its own base point; one lower
  // looks further toward the sun.
  double bx, by;
  L.base_point(10.0, 20.0, p.base, bx, by);
  EXPECT_NEAR(bx, 10.0, 1e-9);
  EXPECT_NEAR(by, 20.0, 1e-9);
  L.base_point(10.0, 20.0, p.base - 50.0, bx, by);
  EXPECT_NEAR(bx, 10.0 + 50.0 * sx / sz, 1e-9);
  EXPECT_NEAR(by, 20.0 + 50.0 * sy / sz, 1e-9);
}

TEST(WorldCloudLayer, FarDensityHonoursTheCellWeights) {
  const cloud_layer L(small());
  const int N = L.far_size(), layers = L.params().far_layers;
  std::vector<unsigned char> all, none;
  L.far_density(0.4, {}, all);
  ASSERT_EQ(all.size(), std::size_t(N) * N * layers);
  EXPECT_GT(*std::max_element(all.begin(), all.end()), 0);
  L.far_density(0.4, std::vector<float>(L.cells().size(), 0.0f), none);
  EXPECT_TRUE(std::all_of(none.begin(), none.end(), [](unsigned char v) { return v == 0; }));
}

TEST(WorldCloudLayer, ACompositeInSharesIsTheWhole) {
  // A renderer spreads the composites over frames: the cells a share at a time, then the rows.
  const cloud_layer L(small(600.0));
  const double m = 1.3;
  std::vector<unsigned char> whole, parts;
  L.shadow_mask(m, 0.13, 0.25, whole);
  std::vector<float> tau;
  for (std::size_t a = 0; a < L.cells().size(); a += 3)
    L.add_shadow(m, a, 3, tau);
  for (int row = 0; row < L.shadow_size(); row += 7)
    L.shade_rows(tau, 0.13, 0.25, row, 7, parts);
  EXPECT_EQ(whole, parts);
  std::vector<unsigned char> farWhole, farParts;
  L.far_density(m, {}, farWhole);
  std::vector<float> acc;
  for (std::size_t a = 0; a < L.cells().size(); a += 5)
    L.add_far(m, a, 5, {}, acc);
  for (int k = 0; k < L.params().far_layers; k += 3)
    L.far_bytes(acc, k, 3, farParts);
  EXPECT_EQ(farWhole, farParts);
  // Every variant is a whole cloud, however faint the grammar grew it, and a round one: nothing
  // reaches the footprint's rim, so no cloud is cut off by its volume's box.
  const int n = L.params().n, nz = L.params().nz;
  for (int v = 0; v < L.params().variants; ++v) {
    const std::vector<float> &f = L.variant_map(v);
    EXPECT_FLOAT_EQ(*std::max_element(f.begin(), f.end()), 1.0f);
    float rim = 0.0f;
    for (int z = 0; z < nz; ++z)
      for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
          const double u = 2.0 * x / (n - 1) - 1.0, w = 2.0 * y / (n - 1) - 1.0;
          if (u * u + w * w >= 1.0)
            rim = std::max(rim, f[cvc::lsys::cloud_index(n, x, y, z)]);
        }
    EXPECT_EQ(rim, 0.0f);
  }
}
