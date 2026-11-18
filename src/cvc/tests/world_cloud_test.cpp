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
// sun puts it. Its coverage evolves (a cellular automaton), deterministically and whatever the
// frame rate, its clouds fade in and out with their shade, and it gathers on windward flanks.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/lsys/cloud.h>
#include <cvc/world/cloud_layer.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
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

// F1, the live sky: the coverage and the sun change on a built deck. A new coverage ranks the
// cells' targets again at once (36 cells at 0.3: 11 of them); a new sun relights the deck, and it
// then shades and draws exactly as a deck built under that sun, its cells and automaton kept.
TEST(WorldCloudLayer, CoverageAndSunChangeLive) {
  cloud_layer L(small(1000.0));
  EXPECT_EQ(L.occupied(), 22);
  L.set_coverage(0.3);
  EXPECT_EQ(L.occupied(), 11);
  L.set_coverage(1.5); // clamped
  EXPECT_EQ(L.occupied(), 36);
  L.set_coverage(0.6);
  EXPECT_EQ(L.occupied(), 22);

  cloud_layer A(small());
  cloud_layer_params q = small();
  q.sun_az = 40.0;
  q.sun_el = 60.0;
  const cloud_layer B(q);
  std::vector<unsigned char> ma, mb, fa, fb;
  A.shadow_mask(0.7, 0.13, 0.3, ma);
  B.shadow_mask(0.7, 0.13, 0.3, mb);
  EXPECT_NE(ma, mb); // another sun, another shadow
  A.set_sun(40.0, 60.0);
  EXPECT_DOUBLE_EQ(A.params().sun_az, 40.0);
  EXPECT_DOUBLE_EQ(A.params().sun_el, 60.0);
  A.shadow_mask(0.7, 0.13, 0.3, ma);
  EXPECT_EQ(ma, mb);
  A.far_density(0.7, {}, fa);
  B.far_density(0.7, {}, fb);
  EXPECT_EQ(fa, fb);
  A.set_sun(40.0, 95.0); // elevation clamped to 89, as at construction
  EXPECT_DOUBLE_EQ(A.params().sun_el, 89.0);
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

TEST(WorldCloudLayer, ACloudIsShadedByItselfAwayFromTheSun) {
  // A cell's optical depth toward the sun: nothing on the top layer or up-sun of the cloud, more
  // with depth into it, more on its down-sun side than its sunward one; and a morph's is the
  // crossfade of its two variants', as its density is.
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
  std::vector<float> f, tau;
  L.cell_field(*one, m, f);
  L.cell_sun_tau(*one, m, tau);
  const int n = p.n, nz = p.nz;
  ASSERT_EQ(tau.size(), f.size());
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x)
      EXPECT_EQ(tau[cvc::lsys::cloud_index(n, x, y, nz - 1)], 0.0f);
  for (float t : tau)
    EXPECT_GE(t, 0.0f);
  // Deeper layers lie under more of the cloud: the layers' mean depth grows downward.
  std::vector<double> mean(nz, 0.0);
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x)
        mean[z] += tau[cvc::lsys::cloud_index(n, x, y, z)];
    mean[z] /= double(n) * n;
  }
  for (int z = 0; z + 1 < nz; ++z)
    EXPECT_GE(mean[z] + 1e-6, mean[z + 1]) << "layer " << z;
  EXPECT_GT(mean[0], 0.0);
  // Inside the cloud, the half facing the sun is lit better than the half away from it.
  double sx, sy, sz;
  L.sun_dir(sx, sy, sz);
  double W = 0, X = 0, Y = 0;
  for (int z = 0; z < nz; ++z)
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x) {
        const double v = f[cvc::lsys::cloud_index(n, x, y, z)];
        W += v;
        X += v * x;
        Y += v * y;
      }
  ASSERT_GT(W, 0.0);
  X /= W;
  Y /= W;
  double up = 0, upW = 0, down = 0, downW = 0;
  for (int z = 0; z < nz; ++z)
    for (int y = 0; y < n; ++y)
      for (int x = 0; x < n; ++x) {
        const std::size_t i = cvc::lsys::cloud_index(n, x, y, z);
        if (f[i] < 0.3f)
          continue;
        const double toward = (x - X) * sx + (y - Y) * sy;
        (toward > 0 ? up : down) += tau[i];
        (toward > 0 ? upW : downW) += 1.0;
      }
  ASSERT_GT(upW, 0.0);
  ASSERT_GT(downW, 0.0);
  EXPECT_LT(up / upW, down / downW);
  // The morph: the crossfade of the variants' depths, by the morph's weight.
  const cloud_layer::morph_state s = L.cell_morph(*one, m);
  const std::vector<float> &A = L.variant_sun_tau(s.a), &B = L.variant_sun_tau(s.b);
  for (std::size_t k = 0; k < tau.size(); k += 97)
    EXPECT_NEAR(tau[k], (1.0 - s.u) * A[k] + s.u * B[k], 1e-4);
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

// ─────────────────────── the evolving coverage

namespace {
// A 21 x 21 deck (a period of 21 cells of 375 m): the size of the 11.2 km world's, stepping
// every 3 s.
cloud_layer_params wide() {
  cloud_layer_params p = small(3900.0);
  p.evolve_s = 3.0;
  return p;
}

std::vector<bool> held(const cloud_layer &L) {
  std::vector<bool> h;
  for (const cloud_cell &c : L.cells())
    h.push_back(c.occupied);
  return h;
}

constexpr double kPi = 3.14159265358979323846;

// Ridges across the wind over one period [0, P)^2: h = A sin(2 pi x / lambda), three to a period,
// steepest slope `slope`.
void ridges(cloud_layer &L, double slope) {
  const double P = L.period_m(), lambda = P / 3.0, A = slope * lambda / (2.0 * kPi);
  const double d = 25.0;
  const int n = static_cast<int>(P / d) + 1;
  std::vector<float> h(std::size_t(n) * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h[std::size_t(r) * n + c] = static_cast<float>(A * std::sin(2.0 * kPi * c * d / lambda));
  L.set_ground(h, n, n, 0.0, 0.0, d, d);
}
} // namespace

TEST(WorldCloudLayer, CoverageEvolvesAndHoldsItsCount) {
  cloud_layer L(wide());
  ASSERT_EQ(L.period_cells(), 21);
  const int want = static_cast<int>(std::lround(0.6 * 21 * 21));
  EXPECT_EQ(L.occupied(), want);
  // A new deck has settled: every cloud it holds is fully there, every other cell clear.
  for (const cloud_cell &c : L.cells())
    EXPECT_EQ(c.presence, c.occupied ? 1.0 : 0.0);
  const std::vector<bool> h0 = held(L);
  for (int s = 0; s < 90; ++s) { // 90 s: 30 steps of 3 s
    L.evolve(1.0);
    ASSERT_EQ(L.occupied(), want);
    for (const cloud_cell &c : L.cells()) {
      ASSERT_GE(c.presence, 0.0);
      ASSERT_LE(c.presence, 1.0);
    }
  }
  EXPECT_EQ(L.generation(), cloud_layer::WARM_STEPS + 30);
  const std::vector<bool> h1 = held(L);
  int changed = 0;
  for (std::size_t k = 0; k < h0.size(); ++k)
    changed += h0[k] != h1[k] ? 1 : 0;
  std::printf("%d of %zu cells changed in 30 steps\n", changed, h0.size());
  EXPECT_GT(changed, 10); // clouds came and went
  // but the deck did not start over (a reshuffle changes 2 * 0.6 * 0.4 of the cells)
  EXPECT_LT(changed, int(0.3 * h0.size()));
}

TEST(WorldCloudLayer, EvolveIsDeterministicAndDoesNotDependOnTheFrames) {
  cloud_layer a(wide()), b(wide());
  const long g0 = a.generation();
  a.evolve(37.0);
  for (int k = 0; k < 148; ++k)
    b.evolve(0.25);
  EXPECT_EQ(a.generation(), g0 + 12);
  EXPECT_EQ(b.generation(), g0 + 12);
  ASSERT_EQ(a.cells().size(), b.cells().size());
  for (std::size_t k = 0; k < a.cells().size(); ++k) {
    EXPECT_EQ(a.cells()[k].occupied, b.cells()[k].occupied);
    EXPECT_NEAR(a.cells()[k].presence, b.cells()[k].presence, 1e-9);
  }
  cloud_layer_params q = wide();
  q.seed = 21;
  cloud_layer c(q);
  c.evolve(37.0);
  EXPECT_NE(held(a), held(c)); // another seed, another weather
}

TEST(WorldCloudLayer, ACloudFadesInAndItsShadeAndSlicesFollow) {
  cloud_layer L(wide());
  // Run until a cloud is part-way in.
  int k = -1;
  for (int s = 0; s < 400 && k < 0; ++s) {
    L.evolve(1.0);
    for (std::size_t i = 0; i < L.cells().size(); ++i)
      if (L.cells()[i].occupied && L.cells()[i].presence > 0.2 && L.cells()[i].presence < 0.8)
        k = static_cast<int>(i);
  }
  ASSERT_GE(k, 0);
  // Its presence climbs dt / fade_s a second while it is held.
  cloud_layer M = L;
  const double p0 = M.cells()[k].presence;
  M.evolve(0.5, 0.0); // rate 0: no step, so it stays held
  EXPECT_NEAR(M.cells()[k].presence, std::min(1.0, p0 + 0.5 / M.params().fade_s), 1e-12);
  EXPECT_EQ(M.generation(), L.generation());
  // Its shade and far opacity are its full cloud's times its weight.
  const double w = L.cells()[k].weight();
  EXPECT_GT(w, 0.0);
  EXPECT_LT(w, 1.0);
  cloud_layer full = L;
  full.snap();
  ASSERT_EQ(full.cells()[k].presence, 1.0);
  std::vector<float> tw, t1, fw, f1;
  L.add_shadow(0.4, k, 1, tw);
  full.add_shadow(0.4, k, 1, t1);
  L.add_far(0.4, k, 1, {}, fw);
  full.add_far(0.4, k, 1, {}, f1);
  double sw = 0, s1 = 0, gw = 0, g1 = 0;
  for (std::size_t i = 0; i < tw.size(); ++i) {
    sw += tw[i];
    s1 += t1[i];
  }
  for (std::size_t i = 0; i < fw.size(); ++i) {
    gw += fw[i];
    g1 += f1[i];
  }
  ASSERT_GT(s1, 0.0);
  ASSERT_GT(g1, 0.0);
  EXPECT_NEAR(sw / s1, w, 1e-4);
  EXPECT_NEAR(gw / g1, w, 1e-4);
}

TEST(WorldCloudLayer, RateZeroFreezesTheWeatherButFinishesTheFades) {
  cloud_layer L(wide());
  L.evolve(4.5); // a step and a half: some fades under way
  const long g = L.generation();
  const std::vector<bool> h = held(L);
  for (int s = 0; s < 25; ++s)
    L.evolve(1.0, 0.0);
  EXPECT_EQ(L.generation(), g);
  EXPECT_EQ(held(L), h);
  for (const cloud_cell &c : L.cells())
    EXPECT_EQ(c.presence, c.occupied ? 1.0 : 0.0); // 25 s > fade_s: every fade is done
}

TEST(WorldCloudLayer, TheForcingIsTheSlopeAlongTheWind) {
  cloud_layer L(small(1000.0));
  EXPECT_FALSE(L.has_ground());
  EXPECT_EQ(L.forcing(500.0, 500.0), 0.0);
  // A plane rising 5 m in 100 toward +x, 4 km square.
  const int n = 161;
  const double d = 25.0;
  std::vector<float> h(std::size_t(n) * n);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      h[std::size_t(r) * n + c] = static_cast<float>(0.05 * c * d);
  L.set_ground(h, n, n, 0.0, 0.0, d, d);
  ASSERT_TRUE(L.has_ground());
  EXPECT_NEAR(L.forcing(2000.0, 2000.0), 0.5, 1e-3); // a wind blowing up it (az 90: +x)
  L.set_wind(270.0);
  EXPECT_NEAR(L.forcing(2000.0, 2000.0), -0.5, 1e-3); // blowing down it: the lee
  L.set_wind(0.0);
  EXPECT_NEAR(L.forcing(2000.0, 2000.0), 0.0, 1e-3); // across it
  L.set_wind(90.0);
  EXPECT_EQ(L.forcing(20000.0, 2000.0), 0.0); // off the ground
  // A steep flank saturates.
  for (float &v : h)
    v *= 10.0f;
  L.set_ground(h, n, n, 0.0, 0.0, d, d);
  EXPECT_EQ(L.forcing(2000.0, 2000.0), 1.0);
  // The cells read the ground where the drift has taken them.
  L.set_drift(130.0, -40.0);
  double x, y;
  L.cell_world(L.cells()[0], x, y);
  const double P = L.period_m();
  EXPECT_NEAR(std::remainder(x - L.cells()[0].cx - 130.0, P), 0.0, 1e-6);
  EXPECT_NEAR(std::remainder(y - L.cells()[0].cy + 40.0, P), 0.0, 1e-6);
  EXPECT_LE(std::fabs(x - 2000.0), 0.5 * P); // the copy nearest the ground's centre
  EXPECT_LE(std::fabs(y - 2000.0), 0.5 * P);
}

TEST(WorldCloudLayer, WindwardFlanksHoldTheCloud) {
  // Over ridges across the wind, the cells on the windward flanks hold a cloud far more of the
  // time than those in the lee; without the coupling they hold one as often.
  auto shares = [](double strength, double &windward, double &lee) {
    cloud_layer_params p = wide();
    p.orographic = strength;
    cloud_layer L(p);
    ridges(L, 0.3);
    L.warm(cloud_layer::WARM_STEPS);
    std::vector<int> on(L.cells().size(), 0);
    const int steps = 300;
    for (int s = 0; s < steps; ++s) {
      L.step();
      for (std::size_t k = 0; k < L.cells().size(); ++k)
        on[k] += L.cells()[k].occupied ? 1 : 0;
    }
    double w = 0, l = 0;
    int nw = 0, nl = 0;
    for (std::size_t k = 0; k < L.cells().size(); ++k) {
      double x, y;
      L.cell_world(L.cells()[k], x, y);
      const double f = L.forcing(x, y);
      if (f > 0.5) {
        w += on[k];
        ++nw;
      } else if (f < -0.5) {
        l += on[k];
        ++nl;
      }
    }
    EXPECT_GT(nw, 20);
    EXPECT_GT(nl, 20);
    windward = nw ? w / nw / steps : 0.0;
    lee = nl ? l / nl / steps : 0.0;
  };
  double w1, l1, w0, l0;
  shares(1.0, w1, l1);
  shares(0.0, w0, l0);
  std::printf("held: windward %.2f lee %.2f (coupled), %.2f / %.2f (not)\n", w1, l1, w0, l0);
  EXPECT_GT(w1, 0.85);
  EXPECT_LT(l1, 0.45);
  EXPECT_LT(std::fabs(w0 - l0), 0.25);
}

TEST(WorldCloudLayer, AStepIsCheap) {
  // A 64 x 64 deck (4096 cells): a step is bit operations and a few draws a cell.
  cloud_layer_params p = small(64 * 375.0 / 2.0);
  p.shadow_texels = 2;
  p.far_texels = 2;
  cloud_layer L(p);
  ASSERT_EQ(L.period_cells(), 64);
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s < 50; ++s)
    L.step();
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / 50;
  std::printf("cloud automaton: %.3f ms a step over %zu cells\n", ms, L.cells().size());
  // Well under a millisecond in a release build. The Debug package lanes measured 25-32 ms a step
  // on a loaded runner, so unoptimized builds get 100. By the optimizer, not NDEBUG, which a Debug
  // build can define.
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  EXPECT_LT(ms, 20.0);
#else
  EXPECT_LT(ms, 100.0);
#endif
}

// ─────────────────────── storms and storm decks (F2)

namespace {
double mean(const std::vector<float> &v) {
  double s = 0.0;
  for (float x : v)
    s += x;
  return v.empty() ? 0.0 : s / v.size();
}
} // namespace

// A storm over the deck: the cells under it gather cloud and are held besides the fair weather's
// coverage, their shade and far opacity deepen, and the deck goes back to its fair coverage once
// the storm has passed.
TEST(WorldCloudLayer, AStormClosesTheDeckUnderIt) {
  cloud_layer_params p = small(1000.0); // 6 x 6 cells of 375 m
  p.coverage = 0.3;
  cloud_layer L(p);
  ASSERT_EQ(L.occupied(), 11);
  // A full storm 4 km across over the pattern's middle: its core (2 km) covers several cells.
  const double mid = 0.5 * L.period_m();
  L.set_storms({{mid, mid, 2000.0, 1.0}});
  EXPECT_DOUBLE_EQ(L.storm_at(mid, mid), 1.0);
  EXPECT_DOUBLE_EQ(L.storm_at(mid + 2050.0, mid), 0.0);
  int under = 0, held = 0;
  double stormy = 0.0;
  for (const cloud_cell &c : L.cells()) {
    stormy += c.storm;
    if (c.storm > 0.9) {
      ++under;
      held += c.occupied ? 1 : 0;
    }
  }
  std::printf("storm: %d cells under it, %d held; %d occupied of %zu (fair 11)\n", under, held,
              L.occupied(), L.cells().size());
  ASSERT_GE(under, 4);
  EXPECT_EQ(held, under); // every cell under the storm holds a cloud
  EXPECT_EQ(L.occupied(), static_cast<int>(std::lround(0.3 * 36 + 0.7 * stormy)));
  // The automaton feeds them too: after a while they hold far more cloud bits than the rest.
  for (int s = 0; s < 60; ++s)
    L.step();
  double in = 0.0, out = 0.0;
  int nin = 0, nout = 0;
  for (const cloud_cell &c : L.cells())
    if (c.storm > 0.9) {
      in += c.cloud;
      ++nin;
    } else if (c.storm < 0.1) {
      out += c.cloud;
      ++nout;
    }
  ASSERT_GT(nin, 0);
  ASSERT_GT(nout, 0);
  std::printf("cloud bits: %.2f under the storm, %.2f in fair weather\n", in / nin, out / nout);
  EXPECT_GT(in / nin, out / nout + 0.2);
  // Its shade and its far opacity deepen against the same deck without the storm: more clouds
  // held, each denser.
  cloud_layer fair = L;
  fair.set_storms({});
  EXPECT_EQ(fair.occupied(), 11); // the storm passes: the fair coverage again
  for (const cloud_cell &c : fair.cells())
    EXPECT_EQ(c.storm, 0.0);
  L.snap();
  fair.snap();
  std::vector<float> ts, tf, fs, ff;
  L.shadow_tau(0.4, ts);
  fair.shadow_tau(0.4, tf);
  L.add_far(0.4, 0, L.cells().size(), {}, fs);
  fair.add_far(0.4, 0, fair.cells().size(), {}, ff);
  std::printf("shade %.3f (storm) %.3f (fair), far %.4f / %.4f\n", mean(ts), mean(tf), mean(fs),
              mean(ff));
  EXPECT_GT(mean(ts), 1.5 * mean(tf));
  EXPECT_GT(mean(fs), 1.5 * mean(ff));
}

// The deck's heights change live (a storm deck rising out of the cumulus and settling back): the
// deck then shades and draws exactly as one built with those heights, layers and slices, built
// here or on another thread; a job made before the sun moved is refused.
TEST(WorldCloudLayer, ADeckChangedLiveIsOneBuiltSo) {
  cloud_layer_params fair = small();
  fair.cumulus = fair.top - fair.base; // 48 m of cumulus
  cloud_layer A(fair);
  EXPECT_FALSE(A.deep());
  cloud_layer_params tall = fair;
  tall.top = 900.0;
  tall.nz = 30;
  tall.far_layers = 9;
  const cloud_layer B(tall);
  EXPECT_TRUE(B.deep());
  auto same = [](const cloud_layer &a, const cloud_layer &b) {
    std::vector<unsigned char> ma, mb, fa, fb;
    a.shadow_mask(0.7, 0.13, 0.3, ma);
    b.shadow_mask(0.7, 0.13, 0.3, mb);
    a.far_density(0.7, {}, fa);
    b.far_density(0.7, {}, fb);
    std::vector<float> ca, cb, sa, sb;
    a.cell_field(a.cells()[3], 0.7, ca);
    b.cell_field(b.cells()[3], 0.7, cb);
    a.cell_sun_tau(a.cells()[3], 0.7, sa);
    b.cell_sun_tau(b.cells()[3], 0.7, sb);
    return ma == mb && fa == fb && ca == cb && sa == sb;
  };
  EXPECT_FALSE(same(A, B));
  A.set_deck(tall.base, tall.top, tall.nz, tall.far_layers);
  EXPECT_TRUE(A.deep());
  EXPECT_EQ(A.params().nz, 30);
  EXPECT_EQ(A.params().far_layers, 9);
  EXPECT_TRUE(same(A, B));
  // ...and back to the cumulus: as it was built.
  const cloud_layer C(fair);
  A.set_deck(fair.base, fair.top, fair.nz, fair.far_layers);
  EXPECT_FALSE(A.deep());
  EXPECT_TRUE(same(A, C));
  // Built on another thread, adopted here.
  std::shared_ptr<cloud_layer::deck_job> job =
      A.prepare_deck(tall.base, tall.top, tall.nz, tall.far_layers);
  std::thread worker([job] { cloud_layer::run(*job); });
  worker.join();
  EXPECT_TRUE(A.adopt_deck(*job));
  EXPECT_TRUE(same(A, B));
  // A job for the deck the layer has costs nothing; one made before the sun moved is refused.
  std::shared_ptr<cloud_layer::deck_job> again =
      A.prepare_deck(tall.base, tall.top, tall.nz, tall.far_layers);
  EXPECT_TRUE(A.adopt_deck(*again));
  std::shared_ptr<cloud_layer::deck_job> stale = A.prepare_deck(fair.base, fair.top, 0, 0);
  cloud_layer::run(*stale);
  A.set_sun(10.0, 50.0);
  EXPECT_FALSE(A.adopt_deck(*stale));
  EXPECT_TRUE(A.deep());
}

// A storm deck's maps are cumulonimbus: cloud up the tower (where the cumulus deck has none above
// its top), and a base deep in the tower's shade: the base's optical depth toward the sun is 2.5 to
// 6 times a cumulus base's (measured), which turns it dark grey through the renderer's ramp.
TEST(WorldCloudLayer, AStormDeckTowersAndDarkensItsBase) {
  cloud_layer_params fair = small();
  fair.n = 32;
  fair.nz = 16;
  fair.extent = 250.0;
  fair.base = 300.0;
  fair.top = 520.0;
  fair.cumulus = fair.top - fair.base;
  fair.sun_el = 28.0;
  fair.opacity = {0.0, 0.0, 0.22, 0.0, 0.55, 0.26, 1.0, 0.54}; // cloud_sky's table
  cloud_layer_params tall = fair;
  tall.top = 3500.0;
  tall.nz = 48;
  const cloud_layer F(fair), T(tall);
  const int n = fair.n;
  const std::size_t layer = std::size_t(n) * n;
  int towered = 0;
  for (int v = 0; v < fair.variants; ++v) {
    const std::vector<float> &m = T.variant_map(v), &s = T.variant_sun_tau(v);
    const std::vector<float> &fm = F.variant_map(v), &fs = F.variant_sun_tau(v);
    // Cloud in the upper half of the storm deck: the tower and its anvil.
    double up = 0.0;
    for (int z = tall.nz / 2; z < tall.nz; ++z)
      for (std::size_t i = 0; i < layer; ++i)
        up += m[z * layer + i];
    towered += up > 1.0 ? 1 : 0;
    // The base: the optical depth toward the sun at each column's lowest voxel that holds cloud.
    auto base_tau = [&](const std::vector<float> &map, const std::vector<float> &tau) {
      const int nz = static_cast<int>(map.size() / layer);
      double acc = 0.0;
      int k = 0;
      for (std::size_t i = 0; i < layer; ++i)
        for (int z = 0; z < nz; ++z)
          if (map[z * layer + i] > 0.2) {
            acc += tau[z * layer + i];
            ++k;
            break;
          }
      return k ? acc / k : 0.0;
    };
    const double tf = base_tau(fm, fs), tt = base_tau(m, s);
    // ...and the cloud above it, which hides the sky from it (cell_up_tau): many times more.
    const double uf = base_tau(fm, F.variant_up_tau(v)), ut = base_tau(m, T.variant_up_tau(v));
    std::printf("variant %d: base optical depth toward the sun %.1f (cumulus) %.1f (storm deck), "
                "up %.1f / %.1f, upper cloud %.1f\n",
                v, tf, tt, uf, ut, up);
    EXPECT_GT(tt, 2.5 * tf);
    EXPECT_GT(ut, 5.0 * uf);
    // The top layer has nothing above it.
    const std::vector<float> &u = T.variant_up_tau(v);
    for (std::size_t i = 0; i < layer; ++i)
      ASSERT_EQ(u[(tall.nz - 1) * layer + i], 0.0f);
  }
  EXPECT_EQ(towered, fair.variants);
}

// A storm deck built a unit at a time (as a renderer with no thread to give it steps it, a share a
// frame: single-threaded wasm) is the deck built at once. Its phases come in order and its progress
// climbs to 1.
TEST(WorldCloudLayer, ADeckBuiltInStepsIsTheDeckBuiltAtOnce) {
  cloud_layer_params fair = small();
  fair.cumulus = fair.top - fair.base;
  cloud_layer A(fair), B(fair);
  std::shared_ptr<cloud_layer::deck_job> ja = A.prepare_deck(fair.base, 900.0, 30, 9);
  cloud_layer::run(*ja);
  ASSERT_TRUE(A.adopt_deck(*ja));
  std::shared_ptr<cloud_layer::deck_job> jb = B.prepare_deck(fair.base, 900.0, 30, 9);
  EXPECT_EQ(std::string(cloud_layer::phase(*jb)), "grow");
  std::vector<std::string> phases;
  double last = -1.0;
  int steps = 0;
  while (!cloud_layer::step(*jb, 1)) {
    ++steps;
    const double pr = cloud_layer::progress(*jb);
    EXPECT_GE(pr, last);
    EXPECT_LT(pr, 1.0);
    last = pr;
    const std::string ph = cloud_layer::phase(*jb);
    if (phases.empty() || phases.back() != ph)
      phases.push_back(ph);
  }
  std::printf("a storm deck in %d steps of one unit\n", steps + 1);
  EXPECT_GT(steps, 50);
  EXPECT_EQ(cloud_layer::progress(*jb), 1.0);
  EXPECT_EQ(std::string(cloud_layer::phase(*jb)), "built");
  EXPECT_EQ(phases, (std::vector<std::string>{"grow", "self-shade", "sky", "shade", "far"}));
  EXPECT_TRUE(cloud_layer::step(*jb, 1)); // built: nothing more to do
  ASSERT_TRUE(B.adopt_deck(*jb));
  std::vector<unsigned char> ma, mb, fa, fb;
  A.shadow_mask(0.7, 0.13, 0.3, ma);
  B.shadow_mask(0.7, 0.13, 0.3, mb);
  EXPECT_EQ(ma, mb);
  A.far_density(0.7, {}, fa);
  B.far_density(0.7, {}, fb);
  EXPECT_EQ(fa, fb);
  for (int v = 0; v < fair.variants; ++v) {
    EXPECT_EQ(A.variant_map(v), B.variant_map(v));
    EXPECT_EQ(A.variant_sun_tau(v), B.variant_sun_tau(v));
    EXPECT_EQ(A.variant_up_tau(v), B.variant_up_tau(v));
  }
}

// What a storm deck costs to build (terrain_lab's deck: 60 x 60 maps of 250 m half-width, the
// cumulus 300 to 520 m, the storm deck to 3000 m on 64 layers with 24 far slices). The cloud_sky
// builds it on a thread of its own, so this is how long a storm's deck takes to come in, not a
// frame's hitch. The cumulus the maps grow from is kept, so neither way grows it again (growing it
// is the seconds the deck's construction takes).
TEST(WorldCloudLayer, AStormDecksBuildIsTimed) {
  cloud_layer_params p;
  p.extent = 250.0;
  p.base = 300.0;
  p.top = 520.0;
  p.cumulus = p.top - p.base;
  p.area = 1264.0;
  p.sun_az = -60.0;
  p.sun_el = 28.0;
  p.opacity = {0.0, 0.0, 0.22, 0.0, 0.55, 0.26, 1.0, 0.54};
  cloud_layer L(p);
  auto timed = [&L](double top, int nz, int layers) {
    const auto t0 = std::chrono::steady_clock::now();
    std::shared_ptr<cloud_layer::deck_job> job = L.prepare_deck(300.0, top, nz, layers);
    cloud_layer::run(*job);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    EXPECT_TRUE(L.adopt_deck(*job));
    return ms;
  };
  const double up = timed(3000.0, 64, 24), down = timed(520.0, 28, 10);
  std::printf("terrain_lab's storm deck: %.0f ms to build, %.0f ms back to the cumulus\n", up,
              down);
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  EXPECT_LT(up, 3000.0); // 153 ms measured (Windows, Release, a loaded machine)
  EXPECT_LT(down, 3000.0);
#endif
}
