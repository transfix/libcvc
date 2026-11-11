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

// cvc::ocean::shore_layer — the swell at the coast (OCEAN-AND-WEATHER-ROADMAP.md A3 and its
// section 4 tests): the wavelength follows the dispersion relation; a plane wave over flat water
// stays one; crests turn toward the shore as Snell's law says; the amplitude shoals as Green's law
// says and never passes the breaking cap; a cliff reflects and a gentle beach does not; the lee of
// an island is calm and its windward shore is not; a lake the swell cannot reach is still.

#include <cmath>
#include <cstring>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace cvc::ocean;

namespace {
constexpr double kPi = 3.14159265358979323846;

// A grid of rows x cols at `cell` metres from (0, 0), its depth from f(x, y), and its shore
// distance and normal from g(x, y) -> {distance, nx, ny} (by default: no shore anywhere).
shore_grid
makeGrid(int rows, int cols, double cell, const std::function<double(double, double)> &f,
         const std::function<void(double, double, double &, double &, double &)> &shore = nullptr) {
  shore_grid g;
  g.rows = rows;
  g.cols = cols;
  g.cell = cell;
  const std::size_t n = g.count();
  g.depth.resize(n);
  g.shore_m.assign(n, 1e6f);
  g.normal_x.assign(n, 0.0f);
  g.normal_y.assign(n, 0.0f);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const std::size_t i = std::size_t(r) * cols + c;
      const double x = g.x(c), y = g.y(r);
      g.depth[i] = float(f(x, y));
      if (shore) {
        double s, nx, ny;
        shore(x, y, s, nx, ny);
        g.shore_m[i] = float(s);
        g.normal_x[i] = float(nx);
        g.normal_y[i] = float(ny);
      }
    }
  return g;
}
} // namespace

// ── the pieces ───────────────────────────────────────────────────────────────────────────────────

TEST(OceanShore, TheWavenumberSolvesTheDispersionRelation) {
  for (double T : {5.0, 10.0, 16.0})
    for (double d : {0.3, 2.0, 10.0, 60.0, 2000.0}) {
      const double w = 2.0 * kPi / T, k = shore_wavenumber(w, d);
      EXPECT_NEAR(dispersion(k, d), w, 1e-10 * w) << "T " << T << " d " << d;
    }
  const double w = 2.0 * kPi / 8.0;
  EXPECT_DOUBLE_EQ(shore_wavenumber(w, 5000.0), w * w / gravity); // deep water
}

TEST(OceanShore, ShoalingFollowsGreensLawInShallowWater) {
  const double w = 2.0 * kPi / 14.0;
  EXPECT_NEAR(shoaling(w, 5000.0), 1.0, 1e-6);
  // Shallow (k d well under 1): A ~ d^(-1/4).
  EXPECT_NEAR(shoaling(w, 0.2) / shoaling(w, 0.8), std::pow(0.8 / 0.2, 0.25), 0.03);
  // Shoaling first dips below 1 at intermediate depth, then grows without bound.
  EXPECT_LT(shoaling(w, 60.0), 1.0);
  EXPECT_GT(shoaling(w, 0.5), 1.5);
}

TEST(OceanShore, CliffsReflectAndBeachesDoNot) {
  // A 1 m, 10 s swell: L0 = 156 m.
  EXPECT_DOUBLE_EQ(reflection_coefficient(iribarren(10.0, 1.0, 10.0)), 1.0); // a cliff
  EXPECT_LT(reflection_coefficient(iribarren(1.0 / 50.0, 1.0, 10.0)), 0.05); // 1:50 beach
  EXPECT_NEAR(iribarren(0.1, 1.0, 10.0), 0.1 / std::sqrt(1.0 / (gravity * 100.0 / (2 * kPi))),
              1e-12);
  EXPECT_NEAR(runup(1.5, 2.0), 3.0, 1e-15);
}

// ── the layer ────────────────────────────────────────────────────────────────────────────────────

TEST(OceanShore, OverFlatWaterThePhaseIsAPlaneWave) {
  // Open water 20 m deep, no shore; a 9 s swell travelling 30 degrees off +x.
  const shore_grid g = makeGrid(80, 100, 2.0, [](double, double) { return 20.0; });
  shore_swell s;
  s.tp_s = 9.0;
  s.dir_rad = kPi / 6.0;
  const shore_layer L = shore_layer::build(g, s);
  const double k = shore_wavenumber(L.omega, 20.0), ux = std::cos(s.dir_rad),
               uy = std::sin(s.dir_rad);
  double maxErr = 0.0, minProj = 1e300;
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c)
      minProj = std::min(minProj, g.x(c) * ux + g.y(r) * uy);
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c) {
      const std::size_t i = L.index(r, c);
      maxErr = std::max(maxErr, std::fabs(L.phase[i] - k * (g.x(c) * ux + g.y(r) * uy - minProj)));
      EXPECT_NEAR(L.dir_x[i], ux, 1e-4);
      EXPECT_NEAR(L.dir_y[i], uy, 1e-4);
      EXPECT_NEAR(L.wavenumber[i], k, 1e-6);
    }
  EXPECT_LT(maxErr, 1e-3); // first-order fast marching is exact for a plane wave
  // The wavelength the phase holds is the dispersion relation's.
  EXPECT_NEAR(2.0 * kPi / k, 2.0 * kPi / shore_wavenumber(2.0 * kPi / 9.0, 20.0), 1e-9);
  // No shore: the swell's full deep-water amplitude, shoaled to 20 m.
  EXPECT_NEAR(L.amplitude[L.index(40, 50)], L.deep_amplitude * shoaling(L.omega, 20.0), 1e-6);
  EXPECT_NEAR(L.deep_amplitude, 1.0 / (2.0 * std::sqrt(2.0)), 1e-15);
  EXPECT_EQ(L.breaking[L.index(40, 50)], 0.0f);
}

TEST(OceanShore, CrestsTurnTowardTheShoreAsSnellSays) {
  // A shelf shoaling toward +x, 12 m to 1.5 m; a 10 s swell 35 degrees off the onshore direction.
  // Along the shore the wavenumber's component k sin(theta) is conserved, and theta shrinks.
  const shore_grid g = makeGrid(150, 150, 2.0, [](double x, double) { return 12.0 - 0.035 * x; });
  shore_swell s;
  s.tp_s = 10.0;
  s.dir_rad = 35.0 * kPi / 180.0;
  const shore_layer L = shore_layer::build(g, s);
  auto along = [&](int c) {
    const std::size_t i = L.index(75, c);
    return L.wavenumber[i] * L.dir_y[i];
  };
  auto angle = [&](int c) {
    const std::size_t i = L.index(75, c);
    return std::atan2(L.dir_y[i], L.dir_x[i]);
  };
  EXPECT_NEAR(along(130) / along(20), 1.0, 0.02);
  EXPECT_LT(angle(130), angle(20) - 0.1); // turned at least 6 degrees toward the shore normal
  // And the wave shortens as it shoals: the wavelength is the local dispersion relation's.
  EXPECT_NEAR(L.wavenumber[L.index(75, 130)], shore_wavenumber(L.omega, 12.0 - 0.035 * g.x(130)),
              1e-6);
  EXPECT_GT(L.wavenumber[L.index(75, 130)], 1.5 * L.wavenumber[L.index(75, 20)]);
}

TEST(OceanShore, TheAmplitudeShoalsAndBreaksAtTheCap) {
  // A 1:40 beach running to the waterline; a 1.5 m, 11 s swell straight in.
  auto depth = [](double x, double) { return 8.0 - x / 40.0; }; // dry past x = 320
  auto shore = [](double x, double, double &s, double &nx, double &ny) {
    s = 320.0 - x;
    nx = -1.0;
    ny = 0.0;
  };
  const shore_grid g = makeGrid(20, 200, 2.0, depth, shore);
  shore_swell sw;
  sw.hs_m = 1.5;
  sw.tp_s = 11.0;
  shore_layer_params p;
  const shore_layer L = shore_layer::build(g, sw, p);
  const double rootN = std::sqrt(double(p.trains));
  bool broke = false;
  for (int c = 0; c < g.cols; ++c) {
    const std::size_t i = L.index(10, c);
    const double d = g.depth[i];
    if (d <= 0.0) {
      EXPECT_EQ(L.amplitude[i], 0.0f) << "dry at x " << g.x(c);
      continue;
    }
    EXPECT_LE(2.0 * rootN * L.amplitude[i], p.breaker_index * d + 1e-5) << "x " << g.x(c);
    broke = broke || L.breaking[i] > 0.0f;
  }
  EXPECT_TRUE(broke);
  EXPECT_EQ(L.breaking[L.index(10, 2)], 0.0f); // not out where it is 8 m deep
  // Before it breaks (x = 160, 4 m deep), it grows as it shoals.
  EXPECT_EQ(L.breaking[L.index(10, 80)], 0.0f);
  EXPECT_GT(L.amplitude[L.index(10, 80)], L.amplitude[L.index(10, 10)]);
  EXPECT_NEAR(L.amplitude[L.index(10, 80)] / L.amplitude[L.index(10, 10)],
              shoaling(L.omega, 8.0 - 160.0 / 40.0) / shoaling(L.omega, 8.0 - 20.0 / 40.0), 1e-4);
  // A gentle beach: next to nothing comes back.
  EXPECT_LT(L.reflection[L.index(10, 150)], 0.05f);
}

TEST(OceanShore, ACliffThrowsTheSwellBack) {
  // 15 m of water up to a vertical cliff at x = 200.
  auto depth = [](double x, double) { return x < 200.0 ? 15.0 : -5.0; };
  auto shore = [](double x, double, double &s, double &nx, double &ny) {
    s = 199.0 - x;
    nx = -1.0;
    ny = 0.0;
  };
  const shore_grid g = makeGrid(10, 110, 2.0, depth, shore);
  shore_swell sw;
  sw.tp_s = 8.0;
  const shore_layer L = shore_layer::build(g, sw);
  const std::size_t near = L.index(5, 98); // 3 m off the cliff
  EXPECT_GT(L.reflection[near], 0.99f);
  EXPECT_GT(L.reflected[near], 0.9f * L.amplitude[near]);
  // Fading offshore over a couple of wavelengths.
  EXPECT_LT(L.reflected[L.index(5, 10)], 0.25f * L.reflected[near]);
  // In front of the cliff the incident and reflected trains stand: over a wavelength of points, the
  // time-averaged variance of the surface is (A^2 + A_r^2) / 2, well above the incident train's
  // A^2 / 2 alone.
  double var = 0.0, incident = 0.0;
  for (int c = 55; c <= 98; ++c) {
    double s2 = 0.0;
    int m = 0;
    for (double t = 0.0; t < 30.0 * sw.tp_s; t += 0.25, ++m) {
      const double e = L.evaluate(g.x(c), g.y(5), t).eta;
      s2 += e * e;
    }
    var += s2 / m;
    incident += 0.5 * double(L.amplitude[L.index(5, c)]) * L.amplitude[L.index(5, c)];
  }
  EXPECT_GT(var / incident, 1.3);
}

TEST(OceanShore, AnIslandsLeeIsCalmAndItsWindwardShoreIsNot) {
  // An island of radius 30 m in 10 m of water; a swell travelling +x.
  const double cx = 200.0, cy = 200.0, rad = 30.0;
  auto depth = [&](double x, double y) { return std::hypot(x - cx, y - cy) < rad ? -2.0 : 10.0; };
  auto shore = [&](double x, double y, double &s, double &nx, double &ny) {
    const double r = std::max(1e-9, std::hypot(x - cx, y - cy));
    s = r - rad;
    nx = (x - cx) / r;
    ny = (y - cy) / r;
  };
  const shore_grid g = makeGrid(200, 200, 2.0, depth, shore);
  shore_swell sw;
  sw.tp_s = 9.0;
  const shore_layer L = shore_layer::build(g, sw);
  const std::size_t windward = L.index(100, 83); // x = 166, 4 m off the west shore
  const std::size_t lee = L.index(100, 117);     // x = 234, 4 m off the east shore
  EXPECT_GT(L.amplitude[windward], 0.8f * float(L.deep_amplitude) * float(shoaling(L.omega, 10.0)));
  EXPECT_LT(L.amplitude[lee], 0.1f * L.amplitude[windward]);
  // The phase wraps round the island: every wet cell is reached, behind it too.
  for (std::size_t i = 0; i < g.count(); ++i)
    if (g.depth[i] > 0.0f) {
      ASSERT_TRUE(std::isfinite(L.phase[i]));
      ASSERT_GT(L.wavenumber[i], 0.0f);
    }
  EXPECT_GT(L.phase[lee], L.phase[windward]); // the swell gets behind it later
}

TEST(OceanShore, ALakeTheSwellCannotReachIsStill) {
  // Dry land with a lake in it, but the sea at the left edge where the swell enters.
  auto depth = [](double x, double y) {
    if (x < 20.0)
      return 5.0;                                               // the sea
    return std::hypot(x - 120.0, y - 60.0) < 25.0 ? 3.0 : -1.0; // a lake inland
  };
  const shore_grid g = makeGrid(60, 100, 2.0, depth);
  const shore_layer L = shore_layer::build(g, shore_swell{});
  EXPECT_GT(L.amplitude[L.index(30, 5)], 0.0f);
  EXPECT_EQ(L.amplitude[L.index(30, 60)], 0.0f); // the lake's middle
  EXPECT_EQ(L.evaluate(120.0, 60.0, 3.0).eta, 0.0);
}

TEST(OceanShore, TheEvaluatorIsTheTrainsSum) {
  const shore_grid g = makeGrid(30, 30, 3.0, [](double, double) { return 12.0; });
  shore_swell sw;
  sw.tp_s = 7.0;
  sw.dir_rad = 0.4;
  shore_layer_params p;
  p.seed = 77;
  const shore_layer L = shore_layer::build(g, sw, p);
  ASSERT_EQ(L.trains.size(), 3u);
  EXPECT_DOUBLE_EQ(L.trains[0].ratio, 1.0);
  EXPECT_NEAR(L.trains[1].ratio, 1.0 / 1.06, 1e-15);
  EXPECT_NEAR(L.trains[2].ratio, 1.0 / 0.94, 1e-15);
  // At a sample, by hand.
  const int r = 12, c = 17;
  const std::size_t i = L.index(r, c);
  const double t = 4.2;
  double eta = 0.0, dx = 0.0;
  for (const shore_train &tr : L.trains) {
    const double th = L.omega * tr.ratio * t - tr.ratio * L.phase[i] + tr.phase;
    eta += tr.amplitude * L.amplitude[i] * std::cos(th);
    dx += tr.amplitude * L.steepness[i] * L.amplitude[i] * std::sin(th) * L.dir_x[i];
  }
  const shore_layer::sample s = L.evaluate(g.x(c), g.y(r), t);
  EXPECT_NEAR(s.eta, eta, 1e-6); // no reflection on open water
  EXPECT_NEAR(s.dx, dx, 1e-6);
  // One train repeats every period.
  shore_layer_params one = p;
  one.trains = 1;
  const shore_layer L1 = shore_layer::build(g, sw, one);
  EXPECT_NEAR(L1.evaluate(20.0, 30.0, 1.0).eta, L1.evaluate(20.0, 30.0, 1.0 + sw.tp_s).eta, 1e-9);
  // Steepness keeps the Gerstner surface from looping.
  for (std::size_t j = 0; j < g.count(); ++j)
    EXPECT_LT(L.steepness[j] * std::sqrt(3.0) * L.amplitude[j] * L.wavenumber[j], 1.0f);
  // The texels pack the fields.
  const std::vector<float> wt = L.wave_texels(), dt = L.dir_texels();
  EXPECT_EQ(wt[i * 4], L.phase[i]);
  EXPECT_EQ(wt[i * 4 + 1], L.amplitude[i]);
  EXPECT_EQ(dt[i * 4 + 2], L.runup[i]);
  // Deterministic.
  const shore_layer again = shore_layer::build(g, sw, p);
  EXPECT_EQ(std::memcmp(again.phase.data(), L.phase.data(), L.phase.size() * sizeof(float)), 0);
}

TEST(OceanShore, NoSwellNoWaves) {
  const shore_grid g = makeGrid(10, 10, 1.0, [](double, double) { return 4.0; });
  shore_swell none;
  none.hs_m = 0.0;
  const shore_layer L = shore_layer::build(g, none);
  for (float a : L.amplitude)
    EXPECT_EQ(a, 0.0f);
  EXPECT_EQ(L.evaluate(5.0, 5.0, 2.0).eta, 0.0);
}

// ── the swash ────────────────────────────────────────────────────────────────────────────────────

namespace {
// A 1:10 beach, its waterline at x = 150, a 1 m, 10 s swell straight in.
const double kBeachSlope = 0.1, kWaterline = 150.0;
shore_layer beachLayer(double slope = kBeachSlope) {
  auto depth = [slope](double x, double) { return slope * (kWaterline - x); };
  auto shore = [](double x, double, double &s, double &nx, double &ny) {
    s = kWaterline - x;
    nx = -1.0;
    ny = 0.0;
  };
  const shore_grid g = makeGrid(10, 200, 1.0, depth, shore);
  shore_swell sw;
  sw.hs_m = 1.0;
  sw.tp_s = 10.0;
  return shore_layer::build(g, sw);
}
} // namespace

TEST(OceanShore, TheRunupIsHuntsAndRunsUpTheBeach) {
  const shore_layer L = beachLayer();
  // At the last wet cell (x = 149: 0.1 m deep, 1 m out) the surf zone's slope is the beach's, so
  // xi = 0.1 / sqrt(H0 / L0) and R = xi H0.
  const double xi = iribarren(kBeachSlope, 1.0, 10.0);
  EXPECT_NEAR(L.runup[L.index(5, 149)], xi * 1.0, 1e-3 * xi);
  // Carried up the dry beach, swash_reach_m (30 m) of it, no further.
  EXPECT_NEAR(L.runup[L.index(5, 160)], L.runup[L.index(5, 149)], 0.05 * xi);
  EXPECT_EQ(L.runup[L.index(5, 199)], 0.0f); // 49 m inland
  EXPECT_NEAR(L.phase[L.index(5, 160)], L.phase[L.index(5, 149)], 0.5);
  // A steep shore surges no higher than Hunt's 2.5 H0.
  const shore_layer steep = beachLayer(1.0);
  EXPECT_NEAR(steep.runup[steep.index(5, 149)], 2.5, 1e-6);
}

TEST(OceanShore, TheLeeHasNoRunup) {
  const double cx = 200.0, cy = 200.0, rad = 30.0;
  auto depth = [&](double x, double y) { return std::hypot(x - cx, y - cy) < rad ? -2.0 : 10.0; };
  auto shore = [&](double x, double y, double &s, double &nx, double &ny) {
    const double r = std::max(1e-9, std::hypot(x - cx, y - cy));
    s = r - rad;
    nx = (x - cx) / r;
    ny = (y - cy) / r;
  };
  const shore_grid g = makeGrid(200, 200, 2.0, depth, shore);
  const shore_layer L = shore_layer::build(g, shore_swell{});
  EXPECT_GT(L.runup[L.index(100, 84)], 0.5f);   // just off the windward (west) shore
  EXPECT_LT(L.runup[L.index(100, 116)], 0.01f); // just off the lee (east) shore
}

TEST(OceanShore, TheSwashWetsTheBeachAndItDries) {
  const shore_layer L = beachLayer();
  const double T = 2.0 * kPi / L.omega, R = L.sample_field(L.runup, 153.0, 5.0);
  const double z = kBeachSlope * 3.0; // the ground 3 m up the beach, 0.3 m above the water
  ASSERT_GT(R, z);
  // Under the film for the share of each wave that the front stands above the ground.
  const double alpha = std::acos(2.0 * z / R - 1.0);
  int wetSteps = 0, steps = 0;
  double lastAge = -1.0;
  bool decayed = false;
  for (double t = 0.0; t < 4.0 * T; t += T / 2000.0, ++steps) {
    const shore_layer::swash_sample s = L.swash(153.0, 5.0, t, z);
    EXPECT_NEAR(s.wet, std::exp(-s.age / L.dry_time_s), 1e-12);
    if (s.film > 0.0) {
      ++wetSteps;
      EXPECT_EQ(s.age, 0.0);
      EXPECT_GE(s.front, z - 1e-9);
    } else if (lastAge >= 0.0 && s.age > lastAge) {
      decayed = true; // drying: the age grows while the front is below
    }
    lastAge = s.film > 0.0 ? -1.0 : s.age;
  }
  EXPECT_NEAR(double(wetSteps) / steps, alpha / kPi, 0.01);
  EXPECT_TRUE(decayed);
  // The front comes round each period.
  EXPECT_NEAR(L.swash(153.0, 5.0, 3.3, z).front, L.swash(153.0, 5.0, 3.3 + T, z).front, 1e-9);
  // Above the runup the sand stays dry; below the still water it is the sea's.
  const shore_layer::swash_sample high = L.swash(153.0, 5.0, 1.0, R + 0.5);
  EXPECT_EQ(high.film, 0.0);
  EXPECT_EQ(high.wet, 0.0);
  EXPECT_TRUE(std::isinf(high.age));
  EXPECT_EQ(L.swash(148.0, 5.0, 1.0, -0.2).wet, 1.0);
  // Lower ground is wet longer: right by the water the front covers it most of the time.
  int lowWet = 0;
  for (double t = 0.0; t < T; t += T / 500.0)
    lowWet += L.swash(150.5, 5.0, t, 0.05).film > 0.0 ? 1 : 0;
  EXPECT_GT(lowWet, 400);
}

TEST(OceanShore, RejectsABadGrid) {
  shore_grid g = makeGrid(8, 8, 1.0, [](double, double) { return 3.0; });
  g.depth.pop_back();
  EXPECT_THROW(shore_layer::build(g, shore_swell{}), std::invalid_argument);
  shore_grid h = makeGrid(8, 8, 1.0, [](double, double) { return 3.0; });
  h.cell = 0.0;
  EXPECT_THROW(shore_layer::build(h, shore_swell{}), std::invalid_argument);
}
