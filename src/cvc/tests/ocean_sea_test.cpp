/*
  Copyright 2026 The University of Texas at Austin

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

// cvc::ocean::fft_plan and sea_surface — the sea a floating body feels
// (OCEAN-AND-WEATHER-ROADMAP.md B2-B4): the FFT is the discrete Fourier transform; the physics
// band is the sum of its modes, packed and unpacked without crosstalk, and its velocities are the
// time derivatives of its fields; the height over a point is its source's (B3); the cascades fade
// at the shore and the sideways move stays offshore; the shore layer's velocity is its surface's
// motion, and breakers carry the water shoreward; the swash runs up and drains the beach; the
// digest follows every input; and the whole surface is the same on every platform.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ocean/fft.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace cvc::ocean;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A deterministic stream of numbers in [-1, 1).
struct lcg {
  std::uint64_t s;
  explicit lcg(std::uint64_t seed) : s(seed) {}
  double next() {
    s = s * 6364136223846793005ull + 1442695040888963407ull;
    return double(s >> 11) / 4503599627370496.0 - 1.0;
  }
};

// The definition: out[p] = sum over m of in[m] exp(sign 2 pi i m p / n).
std::vector<double> dft(const std::vector<double> &in, int n, int sign) {
  std::vector<double> out(std::size_t(n) * 2, 0.0);
  for (int p = 0; p < n; ++p)
    for (int m = 0; m < n; ++m) {
      const double a = sign * 2.0 * kPi * double((long long)m * p % n) / n;
      out[p * 2] += in[m * 2] * std::cos(a) - in[m * 2 + 1] * std::sin(a);
      out[p * 2 + 1] += in[m * 2] * std::sin(a) + in[m * 2 + 1] * std::cos(a);
    }
  return out;
}

sea_state windSea() {
  sea_state s;
  s.wind.speed_mps = 9.0;
  s.wind.dir_rad = 0.4;
  return s;
}

// One cascade holding the whole spectrum on a 100 m tile.
sea_cascade wholeCascade(int n = 64, double fade = 0.0, double decay = 0.0) {
  cascade c = make_cascades({100.0}, 0x5EAull)[0];
  return {build_h0(n, c, windSea()), fade, decay};
}

// lsystem_coast's three cascades (251, 83 and 29 m tiles), n x n each.
std::vector<sea_cascade> coastCascades(int n) {
  std::vector<sea_cascade> out;
  for (const cascade &c : make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull))
    out.push_back({build_h0(n, c, windSea()), 3.0, 0.0});
  return out;
}

double maxAbs(const std::vector<double> &v) {
  double m = 0.0;
  for (double x : v)
    m = std::max(m, std::fabs(x));
  return m;
}

// A grid of rows x cols at `cell` metres from (0, 0): depth from f(x, y), shore distance from s.
std::shared_ptr<shore_grid> makeGrid(int rows, int cols, double cell,
                                     const std::function<double(double, double)> &f,
                                     const std::function<double(double, double)> &s,
                                     double nx = -1.0, double ny = 0.0) {
  auto g = std::make_shared<shore_grid>();
  g->rows = rows;
  g->cols = cols;
  g->cell = cell;
  g->depth.resize(g->count());
  g->shore_m.resize(g->count());
  g->normal_x.assign(g->count(), float(nx));
  g->normal_y.assign(g->count(), float(ny));
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const std::size_t i = std::size_t(r) * cols + c;
      g->depth[i] = float(f(g->x(c), g->y(r)));
      g->shore_m[i] = float(s(g->x(c), g->y(r)));
    }
  return g;
}

// A 1:10 beach, its waterline at x = 150, a 1 m, 10 s swell straight in (ocean_shore_test's).
constexpr double kSlope = 0.1, kWaterline = 150.0;
std::shared_ptr<shore_grid> beachGrid() {
  return makeGrid(
      10, 200, 1.0, [](double x, double) { return kSlope * (kWaterline - x); },
      [](double x, double) { return kWaterline - x; });
}
std::shared_ptr<shore_layer> beachLayer(const shore_grid &g) {
  shore_swell sw;
  sw.hs_m = 1.0;
  sw.tp_s = 10.0;
  return std::make_shared<shore_layer>(shore_layer::build(g, sw));
}

} // namespace

// ── the FFT ──────────────────────────────────────────────────────────────────────────────────────

TEST(OceanFft, IsTheDiscreteFourierTransform) {
  lcg r(7);
  for (int n = 1; n <= 64; n *= 2) {
    const fft_plan plan(n);
    for (int sign : {+1, -1}) {
      std::vector<double> a(std::size_t(n) * 2);
      for (double &v : a)
        v = r.next();
      const std::vector<double> want = dft(a, n, sign);
      plan.run(a.data(), 1, sign);
      for (int i = 0; i < n * 2; ++i)
        EXPECT_NEAR(a[i], want[i], 1e-12 * n) << "n " << n << " sign " << sign << " at " << i;
    }
  }
}

TEST(OceanFft, TwoDimensionsAndBackAgain) {
  const int n = 16;
  const fft_plan plan(n);
  lcg r(11);
  std::vector<double> a(std::size_t(n) * n * 2);
  for (double &v : a)
    v = r.next();
  // Rows then columns is the 2-D transform: check one output against the double sum.
  std::vector<double> b = a;
  plan.run_2d(b.data(), +1);
  for (int p : {0, 5, 37, 255}) {
    const int px = p % n, py = p / n;
    double re = 0.0, im = 0.0;
    for (int my = 0; my < n; ++my)
      for (int mx = 0; mx < n; ++mx) {
        const double ang = 2.0 * kPi * double((mx * px + my * py) % n) / n;
        const double ar = a[(my * n + mx) * 2], ai = a[(my * n + mx) * 2 + 1];
        re += ar * std::cos(ang) - ai * std::sin(ang);
        im += ar * std::sin(ang) + ai * std::cos(ang);
      }
    EXPECT_NEAR(b[p * 2], re, 1e-11);
    EXPECT_NEAR(b[p * 2 + 1], im, 1e-11);
  }
  // Forward then back is n^2 times the input.
  plan.run_2d(b.data(), -1);
  for (std::size_t i = 0; i < a.size(); ++i)
    EXPECT_NEAR(b[i], a[i] * n * n, 1e-10);
  EXPECT_THROW(fft_plan(0), std::invalid_argument);
  EXPECT_THROW(fft_plan(12), std::invalid_argument);
}

// ── the physics band (B2) ────────────────────────────────────────────────────────────────────────

// The band's fields are the sums of its modes, computed here from build_h0's texels directly: the
// height and the vertical velocity, the sideways moves i k/|k| h and their time derivatives, each
// unpacked from its transform without the other's crosstalk.
TEST(OceanSea, TheBandIsTheSumOfItsModes) {
  const int n = 64, M = 16, G = 32;
  sea_params p;
  p.band.modes = M;
  p.band.grid = G;
  const sea_cascade c = wholeCascade(n);
  const sea_surface s({c}, p);
  const double t = 3.7, tile = c.h0.tile_m;
  const sea_surface::band_frame f = s.band(0, t);
  ASSERT_EQ(f.grid, G);
  struct m_t {
    double kx, ky, hr, hi, dr, di;
  };
  std::vector<m_t> modes;
  for (int r = 0; r < n; ++r)
    for (int col = 0; col < n; ++col) {
      const int mx = col - n / 2, my = r - n / 2;
      if (std::abs(mx) >= M / 2 || std::abs(my) >= M / 2)
        continue;
      const double kx = texel_k(col, n, tile), ky = texel_k(r, n, tile), k = std::hypot(kx, ky);
      if (k < 1e-6)
        continue;
      const double w = std::sqrt(p.gravity * k * std::tanh(std::min(k * p.depth_m, 20.0)));
      const float *tx = c.h0.texel(col, r);
      const double cs = std::cos(w * t), sn = std::sin(w * t);
      const double ar = tx[0] * cs - tx[1] * sn, ai = tx[0] * sn + tx[1] * cs;
      const double br = tx[2] * cs + tx[3] * sn, bi = tx[3] * cs - tx[2] * sn;
      modes.push_back({kx, ky, ar + br, ai + bi, -w * (ai - bi), w * (ar - br)});
    }
  ASSERT_GT(modes.size(), 50u);
  const double scale = maxAbs(f.eta) + maxAbs(f.w) + 1e-30;
  for (int gy = 0; gy < G; gy += 3)
    for (int gx = 0; gx < G; gx += 5) {
      const double x = gx * tile / G, y = gy * tile / G;
      double eta = 0, dx = 0, dy = 0, u = 0, v = 0, w = 0;
      for (const m_t &m : modes) {
        const double k = std::hypot(m.kx, m.ky), ph = m.kx * x + m.ky * y;
        const double cs = std::cos(ph), sn = std::sin(ph);
        eta += m.hr * cs - m.hi * sn;
        w += m.dr * cs - m.di * sn;
        // Re(i a e^(i ph)) = -a_i cos - a_r sin
        dx += (-m.hi * cs - m.hr * sn) * m.kx / k;
        dy += (-m.hi * cs - m.hr * sn) * m.ky / k;
        u += (-m.di * cs - m.dr * sn) * m.kx / k;
        v += (-m.di * cs - m.dr * sn) * m.ky / k;
      }
      const std::size_t i = std::size_t(gy) * G + gx;
      EXPECT_NEAR(f.eta[i], eta, 1e-9 * scale);
      EXPECT_NEAR(f.dx[i], dx, 1e-9 * scale);
      EXPECT_NEAR(f.dy[i], dy, 1e-9 * scale);
      EXPECT_NEAR(f.u[i], u, 1e-9 * scale);
      EXPECT_NEAR(f.v[i], v, 1e-9 * scale);
      EXPECT_NEAR(f.w[i], w, 1e-9 * scale);
    }
}

// lsystem_coast's long and mid cascades are band-limited to |m| <= 12, so 32 modes keep all of
// them: 64 modes give the same bits. Only the short cascade loses waves to the band.
TEST(OceanSea, TheLongCascadesFitInTheBand) {
  std::vector<sea_cascade> cs = coastCascades(256);
  sea_params a, b;
  a.band = {32, 64};
  b.band = {64, 64};
  const sea_surface sa(cs, a), sb(cs, b);
  for (std::size_t c = 0; c < 2; ++c) {
    const sea_surface::band_frame fa = sa.band(c, 12.5), fb = sb.band(c, 12.5);
    EXPECT_EQ(fa.eta, fb.eta) << "cascade " << c;
    EXPECT_EQ(fa.dx, fb.dx) << "cascade " << c;
    EXPECT_GT(maxAbs(fa.eta), 0.05) << "cascade " << c << " has waves";
  }
  EXPECT_NE(sa.band(2, 12.5).eta, sb.band(2, 12.5).eta) << "the short cascade is truncated";
}

TEST(OceanSea, TheVelocitiesAreTheTimeDerivatives) {
  sea_params p;
  p.band = {32, 32};
  const sea_surface s({wholeCascade()}, p);
  const double t = 5.0, e = 1e-4;
  const auto f0 = s.band(0, t), fm = s.band(0, t - e), fp = s.band(0, t + e);
  const double scale = maxAbs(f0.w) + maxAbs(f0.u);
  ASSERT_GT(scale, 0.01);
  for (std::size_t i = 0; i < f0.eta.size(); i += 7) {
    EXPECT_NEAR((fp.eta[i] - fm.eta[i]) / (2 * e), f0.w[i], 1e-5 * scale);
    EXPECT_NEAR((fp.dx[i] - fm.dx[i]) / (2 * e), f0.u[i], 1e-5 * scale);
    EXPECT_NEAR((fp.dy[i] - fm.dy[i]) / (2 * e), f0.v[i], 1e-5 * scale);
  }
}

// ── height and velocity at a point (B3, B4) ─────────────────────────────────────────────────────

TEST(OceanSea, TheHeightIsTheSourcesHeight) {
  sea_params p;
  p.band = {32, 32};
  p.level = 2.0;
  p.wave_amp = 1.3;
  p.chop = 0.0; // no sideways move: the source is the point
  const sea_cascade c = wholeCascade();
  const double t = 7.25, tile = c.h0.tile_m;
  {
    const sea_surface s({c}, p);
    const auto f = s.band(0, t);
    // On a lattice point the spline is the lattice value.
    for (int i : {0, 17, 300, 1023}) {
      const double x = (i % 32) * tile / 32, y = (i / 32) * tile / 32;
      EXPECT_NEAR(s.height(x, y, t), 2.0 + 1.3 * f.eta[std::size_t(i)], 1e-12);
      EXPECT_NEAR(s.height(x + 3 * tile, y - 2 * tile, t), s.height(x, y, t), 1e-9)
          << "the tile repeats";
    }
  }
  // With a sideways move, the source x0 is moved onto the point: x0 + D(x0) = x.
  p.chop = 1.5;
  for (int iters : {1, 4, 6, 8}) {
    p.point_iterations = iters;
    const sea_surface s({c}, p);
    double worst = 0.0;
    for (int i = 0; i < 40; ++i) {
      const double x = 3.1 * i, y = -1.7 * i + 5.0;
      const auto x0 = s.source(x, y, t);
      const auto D = s.displacement(x0[0], x0[1], t);
      worst = std::max(worst, std::hypot(x0[0] + D[0] - x, x0[1] + D[1] - y));
    }
    std::printf("[ocean-b3] %d iterations: the worst miss is %.3g m\n", iters, worst);
    if (iters == 4)
      EXPECT_LT(worst, 0.05) << "4 steps land within 5 cm on this steep, choppy sea";
    if (iters == 8)
      EXPECT_LT(worst, 1e-6);
  }
}

// Below the still level the cascades' velocity decays as e^(k z); above it, in a crest, it is
// the level's.
TEST(OceanSea, TheVelocityDecaysWithDepth) {
  sea_params p;
  p.band = {32, 32};
  const double k = 0.08;
  const sea_surface s({wholeCascade(64, 0.0, k)}, p);
  const double t = 2.0, x = 13.0, y = 41.0;
  const auto v0 = s.velocity(x, y, 0.0, t), v1 = s.velocity(x, y, -5.0, t),
             up = s.velocity(x, y, 0.5, t);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(v1[i], v0[i] * std::exp(-5.0 * k), 1e-12);
    EXPECT_EQ(up[i], v0[i]);
  }
  EXPECT_GT(std::hypot(v0[0], v0[1]), 0.0);
}

// ── the shore ───────────────────────────────────────────────────────────────────────────────────

TEST(OceanSea, TheCascadesFadeAtTheShoreAndTheMoveStaysOffshore) {
  sea_params p;
  p.band = {32, 32};
  p.chop = 3.0; // a big sideways move, so the limit has work to do
  std::vector<sea_cascade> cs = {wholeCascade(64, 3.0)};
  const auto grid = beachGrid(); // waterline at x = 150, 1:10
  const sea_surface s(cs, p, grid);
  const double t = 4.0;
  for (double y : {2.0, 5.5}) {
    // 3 cm of water: the cascades have faded to almost nothing.
    EXPECT_NEAR(s.height(kWaterline - 0.3, y, t), 0.0, 0.01);
    for (double x = 100.0; x < kWaterline; x += 1.7) {
      const auto D = s.displacement(x, y, t);
      EXPECT_LE(std::hypot(D[0], D[1]), 0.8 * (kWaterline - x) + 1e-6) << "x " << x;
    }
  }
  // Dry ground without a swash: the still level, below the ground.
  EXPECT_EQ(s.height(kWaterline + 5.0, 3.0, t), 0.0);
  EXPECT_EQ(s.velocity(kWaterline + 5.0, 3.0, 0.2, t)[0], 0.0);
  EXPECT_NEAR(s.depth(140.0, 3.0), 1.0, 1e-6);
}

// The shore layer's velocity is its surface's motion: at the surface, its vertical velocity is
// d eta / dt; it slows with depth as cosh / sinh say, and the bottom does not move up or down.
TEST(OceanSea, TheShoreLayersVelocityIsItsSurfacesMotion) {
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  for (double x : {40.0, 100.0, 130.0}) {
    const double y = 4.0, d = kSlope * (kWaterline - x), e = 1e-4;
    double topSq = 0.0, midSq = 0.0;
    for (int i = 0; i < 64; ++i) {
      const double t = 0.37 * i;
      const double deta =
          (layer->evaluate(x, y, t + e).eta - layer->evaluate(x, y, t - e).eta) / (2 * e);
      const shore_layer::flow top = layer->velocity(x, y, 0.0, d, t, false);
      EXPECT_NEAR(top.w, deta, 1e-6) << "x " << x << " t " << t;
      EXPECT_NEAR(layer->velocity(x, y, -d, d, t, false).w, 0.0, 1e-12) << "the bottom";
      const shore_layer::flow mid = layer->velocity(x, y, -0.5 * d, d, t, false);
      topSq += top.u * top.u;
      midSq += mid.u * mid.u;
    }
    EXPECT_GT(topSq, 0.0);
    EXPECT_LT(midSq, topSq) << "the water moves less below, x " << x;
  }
}

// Linear waves carry nothing over a period; with bores, breaking crests carry the water shoreward.
TEST(OceanSea, BreakersCarryTheWaterShoreward) {
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  // The cell that breaks hardest along a line up the beach.
  double xb = -1.0, most = 0.0;
  for (double x = 100.0; x < kWaterline; x += 0.5) {
    const double b = layer->sample_field(layer->breaking, x, 4.0);
    if (b > most) {
      most = b;
      xb = x;
    }
  }
  ASSERT_GT(most, 0.0) << "the 1 m swell breaks on a 1:10 beach";
  const double d = kSlope * (kWaterline - xb);
  // Over a long window (the trains beat over T / 0.06), linear motion averages out.
  double plain = 0.0, bores = 0.0;
  const int steps = 6000;
  for (int i = 0; i < steps; ++i) {
    const double t = 0.29 * i;
    plain += layer->velocity(xb, 4.0, 0.0, d, t, false).u / steps;
    bores += layer->velocity(xb, 4.0, 0.0, d, t, true).u / steps;
  }
  EXPECT_GT(bores, 0.02) << "shoreward (+x) on average, breaking " << most;
  EXPECT_LT(std::fabs(plain), 0.2 * bores) << "linear waves alone carry little";
}

TEST(OceanSea, TheSwashRunsUpAndDrainsTheBeach) {
  sea_params p;
  p.band = {32, 32};
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  const sea_surface s({wholeCascade(64, 3.0)}, p, grid, layer);
  const double x = kWaterline + 3.0, y = 4.0; // 0.3 m above the still water
  const double R = layer->sample_field(layer->runup, x, y);
  ASSERT_GT(R, 0.3) << "the swash reaches 0.3 m";
  int up = 0, down = 0, dry = 0;
  for (int i = 0; i < 200; ++i) {
    const double t = 0.05 * i;
    const shore_layer::swash_sample sw = layer->swash(x, y, t, 0.3, 0.0);
    EXPECT_NEAR(s.height(x, y, t), sw.front, 1e-12);
    const auto v = s.velocity(x, y, 0.32, t);
    if (sw.front > 0.3) {
      if (sw.rate > 0.0) {
        EXPECT_GT(v[0], 0.0) << "up the beach (+x) while the wave runs up, t " << t;
        ++up;
      } else if (sw.rate < 0.0) {
        EXPECT_LT(v[0], 0.0) << "down while it drains, t " << t;
        ++down;
      }
      EXPECT_LE(std::hypot(v[0], v[1]), std::sqrt(2 * 9.80665 * R) + 1e-9);
    } else {
      EXPECT_EQ(v[0], 0.0);
      ++dry;
    }
  }
  EXPECT_GT(up, 0);
  EXPECT_GT(down, 0);
  EXPECT_GT(dry, 0);
}

// ── state ───────────────────────────────────────────────────────────────────────────────────────

TEST(OceanSea, TheDigestFollowsEveryInput) {
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  sea_params p;
  p.band = {32, 32};
  const sea_cascade c = wholeCascade(64, 3.0, 0.05);
  const auto make = [&](const sea_params &q, const sea_cascade &cc,
                        std::shared_ptr<const shore_grid> g, std::shared_ptr<const shore_layer> l) {
    return sea_surface({cc}, q, std::move(g), std::move(l)).digest();
  };
  const std::uint64_t base = make(p, c, grid, layer);
  EXPECT_EQ(base, make(p, c, grid, layer));
  auto changed = [&](auto edit) {
    sea_params q = p;
    sea_cascade cc = c;
    auto g = std::make_shared<shore_grid>(*grid);
    std::shared_ptr<const shore_layer> l = layer;
    edit(q, cc, *g, l);
    return make(q, cc, g, l) != base;
  };
  EXPECT_TRUE(changed([](sea_params &q, sea_cascade &, shore_grid &, auto &) { q.level = 0.1; }));
  EXPECT_TRUE(
      changed([](sea_params &q, sea_cascade &, shore_grid &, auto &) { q.wave_amp = 1.1; }));
  EXPECT_TRUE(changed([](sea_params &q, sea_cascade &, shore_grid &, auto &) { q.chop = 1.2; }));
  EXPECT_TRUE(
      changed([](sea_params &q, sea_cascade &, shore_grid &, auto &) { q.band.grid = 64; }));
  EXPECT_TRUE(changed([](sea_params &q, sea_cascade &, shore_grid &, auto &) { q.swash = false; }));
  EXPECT_TRUE(
      changed([](sea_params &, sea_cascade &cc, shore_grid &, auto &) { cc.fade_m = 2.0; }));
  EXPECT_TRUE(
      changed([](sea_params &, sea_cascade &cc, shore_grid &, auto &) { cc.decay_k = 0.06; }));
  EXPECT_TRUE(changed([](sea_params &, sea_cascade &cc, shore_grid &, auto &) {
    cc.h0.texels[4 * (33 * 64 + 31)] += 1e-3f;
  }));
  EXPECT_TRUE(
      changed([](sea_params &, sea_cascade &, shore_grid &g, auto &) { g.depth[500] += 0.01f; }));
  EXPECT_TRUE(changed([&](sea_params &, sea_cascade &, shore_grid &, auto &l) {
    shore_swell sw;
    sw.hs_m = 1.1;
    sw.tp_s = 10.0;
    l = std::make_shared<shore_layer>(shore_layer::build(*grid, sw));
  }));
  EXPECT_TRUE(changed([](sea_params &, sea_cascade &, shore_grid &, auto &l) { l = nullptr; }));
}

TEST(OceanSea, RefusesWhatItCannotUse) {
  const sea_cascade c = wholeCascade(32);
  sea_params p;
  p.band = {64, 64};
  EXPECT_THROW(sea_surface({c}, p), std::invalid_argument) << "more modes than h0 has";
  p.band = {16, 8};
  EXPECT_THROW(sea_surface({c}, p), std::invalid_argument) << "a grid smaller than the band";
  p.band = {12, 32};
  EXPECT_THROW(sea_surface({c}, p), std::invalid_argument) << "not a power of two";
  p.band = {16, 32};
  p.chop = -1.0;
  EXPECT_THROW(sea_surface({c}, p), std::invalid_argument);
  p.chop = 1.0;
  p.shore_limit = 1.5;
  EXPECT_THROW(sea_surface({c}, p), std::invalid_argument);
  p.shore_limit = 0.8;
  auto bad = std::make_shared<shore_grid>(*beachGrid());
  bad->shore_m.pop_back();
  EXPECT_THROW(sea_surface({c}, p, bad), std::invalid_argument);
  EXPECT_THROW(sea_surface({c}, p).band(1, 0.0), std::out_of_range);
}

// Readers on several threads at several times get the values one thread gets.
TEST(OceanSea, ThreadsReadTheSameSea) {
  sea_params p;
  p.band = {32, 64};
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  const sea_surface s(coastCascades(64), p, grid, layer);
  auto sweep = [&](int seed) {
    std::vector<double> out;
    for (int i = 0; i < 300; ++i) {
      const double t = 0.01 * ((i * 7 + seed) % 13), x = 20.0 + (i % 130), y = 1.0 + (i % 8);
      out.push_back(s.height(x, y, t));
      const auto v = s.velocity(x, y, -0.2, t);
      out.insert(out.end(), v.begin(), v.end());
    }
    return out;
  };
  const std::vector<double> want = sweep(0);
  std::vector<std::vector<double>> got(4);
  std::vector<std::thread> th;
  for (int k = 0; k < 4; ++k)
    th.emplace_back([&, k] { got[k] = sweep(0); });
  for (auto &t : th)
    t.join();
  for (const auto &g : got)
    EXPECT_EQ(g, want);
}

// The surface over lsystem_coast's three cascades and a beach with its shore layer: heights,
// velocities and a band, digested. Everything is det:: math and IEEE arithmetic without FMA
// contraction, so the digest is the same on every platform; it was measured on Windows (MSVC),
// and CI checks it on Linux and macOS.
TEST(OceanSea, IsTheSameOnEveryPlatform) {
  sea_params p;
  p.band = {32, 64};
  p.wave_amp = 1.2;
  p.chop = 1.15;
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  const sea_surface s(coastCascades(256), p, grid, layer);
  std::uint64_t h = 0xcbf29ce484222325ull;
  const auto fnv = [&h](const void *ptr, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(ptr);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 0x100000001b3ull;
  };
  const std::uint64_t dg = s.digest();
  fnv(&dg, sizeof dg);
  for (double t : {0.0, 2.5, 61.25}) {
    for (int i = 0; i < 60; ++i) {
      const double x = 5.0 + 2.6 * i, y = 0.5 + 0.13 * i; // offshore, surf and the beach
      const double ht = s.height(x, y, t);
      fnv(&ht, sizeof ht);
      const auto v = s.velocity(x, y, ht - 0.3, t);
      fnv(v.data(), sizeof(double) * 3);
    }
    const auto f = s.band(2, t);
    fnv(f.eta.data(), f.eta.size() * sizeof(double));
    fnv(f.u.data(), f.u.size() * sizeof(double));
  }
  EXPECT_EQ(h, 0x21bcce29d45653a8ull) << std::hex << "sea digest 0x" << h;
}

// What one physics step's sea costs (printed, not checked): the band of three cascades at time t.
TEST(OceanSea, ReportsWhatABandCosts) {
  const std::vector<sea_cascade> cs = coastCascades(256);
  for (const band_params b : {band_params{32, 64}, band_params{64, 64}, band_params{64, 128}}) {
    sea_params p;
    p.band = b;
    const sea_surface s(cs, p);
    const int reps = 20;
    const auto t0 = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (int i = 0; i < reps; ++i)
      sink += s.height(1.0, 2.0, 0.01 * i); // a new time: a new band each call
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
        reps;
    std::printf("[ocean-band] modes %d grid %d: %.3f ms per step for 3 cascades (%g)\n", b.modes,
                b.grid, ms, sink);
  }
}

// B3 on a lattice (source_cell_m): the reads interpolate the move between corners found exactly
// and take one Newton step from there. Against the source converged (16 steps), offshore, in the
// surf and up the beach of a steep sea, they miss by no more than the default exact reads (4
// steps) do.
TEST(OceanSea, TheSourceLatticeKeepsTheSurface) {
  sea_params p;
  p.wave_amp = 1.2;
  p.chop = 1.15;
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  const sea_surface exact(coastCascades(256), p, grid, layer);
  p.source_cell_m = 0.5;
  const sea_surface lattice(coastCascades(256), p, grid, layer);
  EXPECT_NE(lattice.digest(), exact.digest()) << "the lattice is in the digest";
  p.source_cell_m = 0.0;
  EXPECT_EQ(sea_surface(coastCascades(256), p, grid, layer).digest(), exact.digest());
  p.point_iterations = 16;
  const sea_surface converged(coastCascades(256), p, grid, layer);

  struct misses {
    double sum_h = 0.0, max_h = 0.0, sum_v = 0.0, max_v = 0.0, sum_r = 0.0, max_r = 0.0;
    int n = 0;
  } m[2];
  const sea_surface *const reads[2] = {&exact, &lattice};
  for (double t : {2.5, 17.75, 41.0}) {
    for (int i = 0; i < 400; ++i) {
      const double x = 5.0 + 0.389 * i, y = 0.5 + 0.0197 * i; // offshore, the surf, the beach
      const double hc = converged.height(x, y, t);
      const auto vc = converged.velocity(x, y, hc - 0.3, t);
      for (int k = 0; k < 2; ++k) {
        const sea_surface &s = *reads[k];
        const double dh = std::fabs(s.height(x, y, t) - hc);
        const auto v = s.velocity(x, y, hc - 0.3, t);
        const double dv = std::hypot(std::hypot(v[0] - vc[0], v[1] - vc[1]), v[2] - vc[2]);
        const auto x0 = s.source(x, y, t);
        const auto d = converged.displacement(x0[0], x0[1], t);
        const double r = std::hypot(x0[0] + d[0] - x, x0[1] + d[1] - y);
        m[k].sum_h += dh * dh;
        m[k].max_h = std::max(m[k].max_h, dh);
        m[k].sum_v += dv * dv;
        m[k].max_v = std::max(m[k].max_v, dv);
        m[k].sum_r += r * r;
        m[k].max_r = std::max(m[k].max_r, r);
        ++m[k].n;
      }
    }
  }
  for (int k = 0; k < 2; ++k)
    std::printf(
        "[ocean-lattice] %s: the source misses by %.1f mm RMS (%.0f mm at most), the height "
        "by %.1f mm (%.0f mm), the velocity by %.1f mm/s (%.0f mm/s)\n",
        k ? "0.5 m lattice" : "exact, 4 steps", 1e3 * std::sqrt(m[k].sum_r / m[k].n),
        1e3 * m[k].max_r, 1e3 * std::sqrt(m[k].sum_h / m[k].n), 1e3 * m[k].max_h,
        1e3 * std::sqrt(m[k].sum_v / m[k].n), 1e3 * m[k].max_v);
}

TEST(OceanSea, TheSourceLatticeIsTheSameOnEveryPlatform) {
  sea_params p;
  p.wave_amp = 1.2;
  p.chop = 1.15;
  p.source_cell_m = 0.5;
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  const sea_surface s(coastCascades(256), p, grid, layer);
  std::uint64_t h = 0xcbf29ce484222325ull;
  const auto fnv = [&h](const void *ptr, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(ptr);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 0x100000001b3ull;
  };
  const std::uint64_t dg = s.digest();
  fnv(&dg, sizeof dg);
  for (double t : {0.0, 2.5, 61.25}) {
    for (int i = 0; i < 60; ++i) {
      const double x = 5.0 + 2.6 * i, y = 0.5 + 0.13 * i;
      const double ht = s.height(x, y, t);
      fnv(&ht, sizeof ht);
      const auto v = s.velocity(x, y, ht - 0.3, t);
      fnv(v.data(), sizeof(double) * 3);
    }
  }
  EXPECT_EQ(h, 0x08f956c6889de4d2ull) << std::hex << "sea digest 0x" << h;
}

// What one point costs (printed, not checked): a height and a velocity over a time's band, read
// exactly and on the 0.5 m lattice, the lattice filled as a body's field cells fill it.
TEST(OceanSea, ReportsWhatAPointCosts) {
  sea_params p;
  p.wave_amp = 1.2;
  p.chop = 1.15;
  const auto grid = beachGrid();
  const auto layer = beachLayer(*grid);
  for (double cell : {0.0, 0.5}) {
    p.source_cell_m = cell;
    const sea_surface s(coastCascades(256), p, grid, layer);
    s.height(0.0, 0.0, 3.0); // the band for t = 3
    const int reps = 20000;
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < reps; ++i) { // a metre-wide body's cells, ten of them, over the surf
      const double x = 100.0 + 5.0 * (i / 2000) + 0.0005 * (i % 2000),
                   y = 4.0 + 0.0003 * (i % 2000);
      const double ht = s.height(x, y, 3.0);
      sink += ht + s.velocity(x, y, ht - 0.3, 3.0)[0];
    }
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
        reps;
    std::printf("[ocean-point] %s: %.2f us for a height and a velocity (%g)\n",
                cell > 0.0 ? "on the 0.5 m lattice" : "exact", us, sink);
  }
}
