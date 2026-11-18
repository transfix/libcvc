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

// The wind on the sea (OCEAN-AND-WEATHER-ROADMAP.md E2-E4, E6, O13): the fetch behind an island
// is near zero and grows as its root; a lake is its own fetch; the sea's wind lags with its time
// constant and makes at most one keyframe per crossfade (the debounce the fetch map is rebuilt
// by); a wind change crossfades the sea with no step in height; the gusts and the current are
// deterministic (golden hashes); the current is divergence-free and its speed is its RMS; and the
// physics sea carries the lee, the gusts and the current.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/algo/wind_field.h>
#include <cvc/ocean/current.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_lag.h>
#include <cvc/ocean/sea_surface.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

using namespace cvc::ocean;

namespace {

constexpr double kPi = 3.14159265358979323846;

// FNV-1a over doubles' bytes.
struct fnv {
  std::uint64_t h = 0xcbf29ce484222325ull;
  void add(double v) {
    unsigned char b[sizeof v];
    std::memcpy(b, &v, sizeof v);
    for (unsigned char c : b) {
      h ^= c;
      h *= 0x100000001b3ull;
    }
  }
};

// A square grid `n` samples a side of `cell` metres from (0, 0), the water `depth` m deep except
// where `land` says.
std::shared_ptr<shore_grid>
makeGrid(int n, double cell, const std::function<bool(double, double)> &land, double depth = 20.0) {
  auto g = std::make_shared<shore_grid>();
  g->rows = g->cols = n;
  g->cell = cell;
  g->depth.resize(g->count());
  g->shore_m.assign(g->count(), 1.0e4f);
  g->normal_x.assign(g->count(), 0.0f);
  g->normal_y.assign(g->count(), 0.0f);
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      g->depth[std::size_t(r) * n + c] = land(g->x(c), g->y(r)) ? -1.0f : float(depth);
  return g;
}

// A round island 150 m in radius at (1000, 1000), on a 2 km grid of 5 m samples.
constexpr double kIslandX = 1000.0, kIslandY = 1000.0, kIslandR = 150.0;
std::shared_ptr<shore_grid> islandGrid() {
  return makeGrid(401, 5.0, [](double x, double y) {
    return std::hypot(x - kIslandX, y - kIslandY) < kIslandR;
  });
}

sea_state windSea(double speed = 9.0, double dir = 0.0) {
  sea_state s;
  s.wind.speed_mps = speed;
  s.wind.dir_rad = dir;
  return s;
}

// lsystem_coast's three cascades (251, 83 and 29 m tiles) at n texels for sea state s.
std::vector<sea_cascade> coastCascades(int n, const sea_state &s) {
  std::vector<sea_cascade> out;
  for (const cascade &c : make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull))
    out.push_back({build_h0(n, c, s), 0.0, 0.0});
  return out;
}

} // namespace

// ── fetch and lee (E2) ───────────────────────────────────────────────────────────────────────────

TEST(OceanFetch, TheLeeOfAnIslandIsCalmAndGrowsAsTheRootOfTheFetch) {
  const std::shared_ptr<shore_grid> g = islandGrid();
  const double open = 120000.0;
  const fetch_field f = build_fetch(*g, 0.0, open); // the wind blows toward +x
  ASSERT_EQ(f.rows, 401);
  const double full = band_full_fetch(9.0, 0.0, spectrum_fetch(9.0, open)); // the long cascade's
  EXPECT_DOUBLE_EQ(full, open);
  const double edge = kIslandX + kIslandR; // the lee shore, on the island's axis
  std::printf("[fetch] downwind of the island:");
  double prev = -1.0;
  for (double d : {10.0, 50.0, 100.0, 200.0, 400.0, 800.0}) {
    const double F = f.sample(edge + d, kIslandY);
    std::printf(" %.0f m -> %.1f m (scale %.3f);", d, F, fetch_scale(F, full));
    EXPECT_NEAR(F, d, 0.05 * d + 6.0) << "the fetch behind the island is the distance from it";
    EXPECT_GT(F, prev);
    prev = F;
  }
  std::printf("\n");
  EXPECT_LT(fetch_scale(f.sample(edge + 10.0, kIslandY), full), 0.02) << "the lee is calm";
  // The scale grows as the root of the fetch: four times as far, twice the waves.
  const double s100 = fetch_scale(f.sample(edge + 100.0, kIslandY), full);
  const double s400 = fetch_scale(f.sample(edge + 400.0, kIslandY), full);
  EXPECT_NEAR(s400 / s100, 2.0, 0.12);
  // Upwind of the island, and beside its lee, the water is open.
  EXPECT_DOUBLE_EQ(f.sample(kIslandX - 400.0, kIslandY), open);
  EXPECT_DOUBLE_EQ(f.sample(kIslandX + 400.0, kIslandY + 400.0), open);
  EXPECT_DOUBLE_EQ(fetch_scale(f.sample(kIslandX - 400.0, kIslandY), full), 1.0);
  // On the island itself, nothing.
  EXPECT_EQ(f.sample(kIslandX, kIslandY), 0.0);

  // A wind toward +y and toward 45 degrees put the lee there instead.
  const fetch_field fy = build_fetch(*g, 0.5 * kPi, open);
  EXPECT_NEAR(fy.sample(kIslandX, kIslandY + kIslandR + 100.0), 100.0, 11.0);
  EXPECT_DOUBLE_EQ(fy.sample(kIslandX + kIslandR + 100.0, kIslandY), open);
  const fetch_field fd = build_fetch(*g, 0.25 * kPi, open);
  const double r = kIslandR + 100.0;
  EXPECT_NEAR(fd.sample(kIslandX + r / std::sqrt(2.0), kIslandY + r / std::sqrt(2.0)), 100.0, 15.0);

  // A coarse map (every fourth sample, 20 m) keeps the lee: the renderer's level of detail.
  const fetch_field fc = build_fetch(*g, 0.0, open, nullptr, 4);
  EXPECT_EQ(fc.rows, 101);
  EXPECT_NEAR(fc.sample(edge + 200.0, kIslandY), 200.0, 25.0);
}

TEST(OceanFetch, ALakeIsItsOwnFetchCappedByItsExtent) {
  // A 400 m lake (x 300..700) in land, the wind toward +x.
  const std::shared_ptr<shore_grid> g = makeGrid(201, 5.0, [](double x, double y) {
    return !(x > 300.0 && x < 700.0 && y > 300.0 && y < 700.0);
  });
  const fetch_field f = build_fetch(*g, 0.0, 120000.0);
  EXPECT_LT(f.sample(305.0, 500.0), 10.0) << "the upwind shore is calm";
  EXPECT_NEAR(f.sample(695.0, 500.0), 395.0, 10.0) << "the downwind shore has the lake's width";
  // With its body's cap (its open extent, here 250 m), never more.
  std::vector<float> cap(g->count(), 250.0f);
  const fetch_field fc = build_fetch(*g, 0.0, 120000.0, &cap);
  EXPECT_FLOAT_EQ(float(fc.sample(695.0, 500.0)), 250.0f);
  // A 400 m lake's waves at 9 m/s: the long cascade at a twentieth of the sea's, the ripples much
  // nearer full (their band is grown within a few kilometres).
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 1);
  const double F = spectrum_fetch(9.0, 120000.0);
  const double f0 = band_full_fetch(9.0, cs[0].k_min, F), f1 = band_full_fetch(9.0, cs[1].k_min, F),
               f2 = band_full_fetch(9.0, cs[2].k_min, F);
  std::printf(
      "[fetch] at 9 m/s the cascades are fully grown at %.0f, %.0f and %.0f m; a 400 m lake "
      "scales them %.3f, %.3f, %.3f\n",
      f0, f1, f2, fetch_scale(400.0, f0), fetch_scale(400.0, f1), fetch_scale(400.0, f2));
  EXPECT_DOUBLE_EQ(f0, F);
  EXPECT_LT(f2, f1);
  EXPECT_LT(f1, f0);
  EXPECT_NEAR(f2, 4600.0, 300.0) << "the 29 m tile's band (waves under 7.3 m) at 9 m/s";
  EXPECT_LT(fetch_scale(400.0, f0), 0.07);
  EXPECT_GT(fetch_scale(400.0, f2), 0.25);
}

TEST(OceanFetch, IsDeterministic) {
  const std::shared_ptr<shore_grid> g = islandGrid();
  const fetch_field a = build_fetch(*g, 0.7, 50000.0, nullptr, 2);
  const fetch_field b = build_fetch(*g, 0.7, 50000.0, nullptr, 2);
  EXPECT_EQ(a.digest(), b.digest());
  EXPECT_NE(a.digest(), build_fetch(*g, 0.75, 50000.0, nullptr, 2).digest());
  EXPECT_EQ(a.digest(), 0xa78c5113f5d5d8eeull) << std::hex << "fetch digest 0x" << a.digest();
}

// ── the sea lags the wind (E4) ───────────────────────────────────────────────────────────────────

TEST(OceanSeaLag, TheSeasWindFollowsWithItsTimeConstant) {
  sea_lag_params p;
  p.lag_s = 20.0;
  p.crossfade_s = 4.0;
  sea_lag lag(p);
  EXPECT_TRUE(lag.advance(0.0, {9.0, 0.0, 120000.0})) << "the first wind makes the first sea";
  EXPECT_EQ(lag.weight(0.0), 1.0);
  std::vector<double> starts;
  double t = 0.0;
  for (int i = 1; i <= 60 * 120; ++i) {
    t = i / 60.0;
    if (lag.advance(t, {15.0, 0.0, 120000.0}))
      starts.push_back(t);
    if (i == 60 * 20)
      EXPECT_NEAR(lag.lagged().speed_mps, 15.0 - 6.0 * std::exp(-1.0), 0.01) << "after one tau";
  }
  ASSERT_GE(starts.size(), 3u);
  for (std::size_t i = 1; i < starts.size(); ++i)
    EXPECT_GE(starts[i] - starts[i - 1], p.crossfade_s - 1e-9) << "one keyframe per crossfade";
  std::printf("[lag] 9 -> 15 m/s with a 20 s lag: %zu keyframes over 120 s, the last at %.1f s\n",
              starts.size(), starts.back());
  EXPECT_EQ(lag.to().speed_mps, 15.0) << "the sea lands on the wind";
  EXPECT_EQ(lag.weight(t), 1.0);

  // The direction follows the shorter arc: from 170 to -170 degrees through 180.
  sea_lag turn(p);
  turn.advance(0.0, {9.0, 170.0 * kPi / 180.0, 1.0e5});
  for (int i = 1; i <= 60 * 10; ++i)
    turn.advance(i / 60.0, {9.0, -170.0 * kPi / 180.0, 1.0e5});
  const double d = std::remainder(turn.lagged().dir_rad, 2.0 * kPi) * 180.0 / kPi;
  EXPECT_GT(std::fabs(d), 170.0) << "it turned through 180, not through 0: " << d;
}

TEST(OceanSeaLag, MakesAtMostOneKeyframePerCrossfade) {
  // A slider dragged back and forth every frame for 10 s, no lag: a keyframe (an h0 and, with the
  // direction moved, a fetch map) at most every crossfade, not every frame.
  sea_lag_params p;
  p.lag_s = 0.0;
  p.crossfade_s = 4.0;
  sea_lag lag(p);
  lag.advance(0.0, {9.0, 0.0, 1.0e5});
  int keys = 0;
  for (int i = 1; i <= 600; ++i)
    if (lag.advance(i / 60.0, {9.0, (i % 2) ? 0.6 : 0.0, 1.0e5}))
      ++keys;
  EXPECT_LE(keys, 3) << "10 s of jitter, a crossfade of 4 s";
  EXPECT_GE(keys, 2);
  // A change under a step (0.2 m/s, 1 degree) with a lag: one keyframe, onto the wind itself.
  p.lag_s = 20.0;
  sea_lag small(p);
  small.advance(0.0, {9.0, 0.0, 1.0e5});
  int smallKeys = 0;
  for (int i = 1; i <= 600; ++i)
    if (small.advance(i / 60.0, {9.2, 1.0 * kPi / 180.0, 1.0e5}))
      ++smallKeys;
  EXPECT_EQ(smallKeys, 1);
  EXPECT_EQ(small.to().speed_mps, 9.2);
  // No change, no keyframe.
  sea_lag still(p);
  still.advance(0.0, {9.0, 0.0, 1.0e5});
  for (int i = 1; i <= 600; ++i)
    EXPECT_FALSE(still.advance(i / 60.0, {9.0, 0.0, 1.0e5}));
}

TEST(OceanSeaLag, TheCrossfadeIsContinuousNoStepInHeight) {
  // The sea grown for 9 m/s toward 0; at t = 30 s the wind turns to 15 m/s toward 30 degrees.
  // Instantly rebuilt (before O13), the surface jumps; crossfaded, the height moves no more in a
  // frame than the waves themselves move.
  const sea_state a = windSea(9.0, 0.0), b = windSea(15.0, 30.0 * kPi / 180.0);
  sea_params p;
  p.point_iterations = 4;
  const std::vector<sea_cascade> ca = coastCascades(64, a), cb = coastCascades(64, b);
  auto forcing = std::make_shared<sea_forcing>();
  const sea_surface before(ca, p, nullptr, nullptr, forcing);
  const double tc = 30.0, fade = 4.0;
  std::vector<sea_cascade> cx = cb;
  for (std::size_t i = 0; i < cx.size(); ++i)
    cx[i].h0_from = ca[i].h0;
  auto fx = std::make_shared<sea_forcing>();
  fx->t0 = tc;
  fx->t1 = tc + fade;
  const sea_surface crossfade(cx, p, nullptr, nullptr, fx);
  const sea_surface instant(cb, p, nullptr, nullptr, forcing);
  const double pts[4][2] = {{3.0, 7.0}, {41.0, -12.0}, {-88.0, 130.0}, {200.5, 50.25}};
  double stepAtChange = 0.0, popAtChange = 0.0, worstFrame = 0.0, natural = 0.0;
  const double dt = 1.0 / 60.0;
  for (const auto &q : pts) {
    // At the change the crossfaded sea is the old one, bit for bit.
    EXPECT_EQ(crossfade.height(q[0], q[1], tc), before.height(q[0], q[1], tc));
    popAtChange = std::max(
        popAtChange, std::fabs(instant.height(q[0], q[1], tc) - before.height(q[0], q[1], tc)));
  }
  for (const auto &q : pts)
    stepAtChange = std::max(stepAtChange, std::fabs(crossfade.height(q[0], q[1], tc + dt) -
                                                    before.height(q[0], q[1], tc)));
  // Frame by frame through the fade, every point at each time (each sea keeps one time's fields).
  std::array<double, 4> hc{}, hb{}, hi{};
  bool first = true;
  for (double t = tc - 1.0; t < tc + fade + 1.0; t += dt, first = false) {
    for (int k = 0; k < 4; ++k) {
      const double c = crossfade.height(pts[k][0], pts[k][1], t);
      if (!first)
        worstFrame = std::max(worstFrame, std::fabs(c - hc[k]));
      hc[k] = c;
    }
    for (int k = 0; k < 4; ++k) {
      const double b0 = before.height(pts[k][0], pts[k][1], t);
      if (!first)
        natural = std::max(natural, std::fabs(b0 - hb[k]));
      hb[k] = b0;
    }
    for (int k = 0; k < 4; ++k) {
      const double i0 = instant.height(pts[k][0], pts[k][1], t);
      if (!first)
        natural = std::max(natural, std::fabs(i0 - hi[k]));
      hi[k] = i0;
    }
  }
  std::printf(
      "[crossfade] at the wind change: an instant rebuild jumps %.3f m; crossfaded the first "
      "frame moves %.4f m; the worst frame in the fade %.4f m, the waves' own worst %.4f m\n",
      popAtChange, stepAtChange, worstFrame, natural);
  EXPECT_GT(popAtChange, 0.3) << "the instant rebuild pops";
  EXPECT_LT(worstFrame, 1.25 * natural) << "no frame steps more than the waves move";
  EXPECT_LT(stepAtChange, 0.25 * popAtChange);
  // After the fade it is the new sea.
  EXPECT_EQ(crossfade.height(3.0, 7.0, tc + fade + 0.5), instant.height(3.0, 7.0, tc + fade + 0.5));
}

// ── gusts (E3) and currents (E6) ────────────────────────────────────────────────────────────────

TEST(OceanWind, TheGustsAreDeterministicAndRideTheWind) {
  const cvc::algo::wind_state w{9.0, 0.6, 0.3};
  fnv h;
  double lo = 1e9, hi = -1e9, sum = 0.0;
  int n = 0;
  for (int k = 0; k < 8; ++k)
    for (int j = 0; j < 32; ++j)
      for (int i = 0; i < 32; ++i) {
        const double g = cvc::algo::wind_gust(w, -500.0 + 31.7 * i, 220.0 + 29.3 * j, 13.0 * k);
        h.add(g);
        lo = std::min(lo, g);
        hi = std::max(hi, g);
        sum += g;
        ++n;
      }
  std::printf("[gusts] gustiness 0.3: %.3f to %.3f, mean %.3f\n", lo, hi, sum / n);
  EXPECT_GT(hi - lo, 0.3);
  EXPECT_NEAR(sum / n, 1.0, 0.08);
  EXPECT_GE(lo, 0.0);
  EXPECT_EQ(h.h, 0xa60b6449e9bd9e94ull) << std::hex << "gust digest 0x" << h.h;
  // The pattern rides downwind: the gust at x at t is the one upwind of it a moment before (the
  // large scale at the wind's speed; the cat's paws at 0.8 of it, so compare the large alone).
  const cvc::algo::wind_state steady{9.0, 0.0, 0.3};
  const double t = 5.0, dx = 9.0 * t;
  const cvc::algo::gust_frame f0 = cvc::algo::gust_frame_at(steady, 0.0);
  const cvc::algo::gust_frame f1 = cvc::algo::gust_frame_at(steady, t);
  for (double x : {10.0, 77.0, 300.0}) {
    const double a =
        cvc::algo::value_noise(x / cvc::algo::kGustCell - f0.drift[0], 0.0, cvc::algo::kGustSeed).v;
    const double b = cvc::algo::value_noise((x + dx) / cvc::algo::kGustCell - f1.drift[0], 0.0,
                                            cvc::algo::kGustSeed)
                         .v;
    EXPECT_NEAR(a, b, 1e-9);
  }
  // No gustiness: a steady wind.
  EXPECT_EQ(cvc::algo::wind_gust(cvc::algo::wind_state{9.0, 0.0, 0.0}, 12.0, 34.0, 56.0), 1.0);
  // The fine scale fades without moving the mean.
  double fine = 0.0, coarse = 0.0;
  for (int i = 0; i < 4000; ++i) {
    fine += cvc::algo::wind_gust(w, 13.1 * i, 7.7 * i, 3.0, 1.0);
    coarse += cvc::algo::wind_gust(w, 13.1 * i, 7.7 * i, 3.0, 0.0);
  }
  EXPECT_NEAR(fine / 4000.0, coarse / 4000.0, 0.02);
}

TEST(OceanCurrent, IsDivergenceFreeAndItsSpeedIsTheRMS) {
  current_params c;
  c.speed_mps = 0.4;
  const current_frame f = current_frame_at(c, 1234.5);
  // Its divergence by central differences, against its gradient's size.
  const double h = 0.25;
  double worst = 0.0, scale = 0.0;
  for (int i = 0; i < 400; ++i) {
    const double x = -3000.0 + 15.1 * i, y = 2000.0 - 9.7 * i;
    const auto px = current_velocity(f, x + h, y), mx = current_velocity(f, x - h, y);
    const auto py = current_velocity(f, x, y + h), my = current_velocity(f, x, y - h);
    const double dudx = (px[0] - mx[0]) / (2 * h), dvdy = (py[1] - my[1]) / (2 * h);
    worst = std::max(worst, std::fabs(dudx + dvdy));
    scale = std::max(scale, std::max(std::fabs(dudx), std::fabs(dvdy)));
  }
  std::printf("[current] divergence at most %.2e /s against gradients up to %.2e /s\n", worst,
              scale);
  EXPECT_LT(worst, 1e-4 * scale) << "the curl of a stream function has no divergence";
  // It is the stream function's curl.
  const auto v = current_velocity(f, 100.0, 200.0);
  const double dpsidy =
      (current_stream(f, 100.0, 200.0 + h) - current_stream(f, 100.0, 200.0 - h)) / (2 * h);
  EXPECT_NEAR(v[0], dpsidy, 1e-4);
  // Its RMS speed is speed_mps.
  double s2 = 0.0;
  const int n = 200000;
  for (int i = 0; i < n; ++i) {
    const auto u = current_velocity(f, 37.3 * (i % 997), 41.9 * (i / 997));
    s2 += u[0] * u[0] + u[1] * u[1];
  }
  const double rms = std::sqrt(s2 / n);
  std::printf("[current] RMS %.4f m/s for speed_mps %.2f (scale %.4f)\n", rms, c.speed_mps,
              rms / c.speed_mps);
  EXPECT_NEAR(rms, c.speed_mps, 0.05 * c.speed_mps);
}

TEST(OceanCurrent, IsDeterministicAndItsFlowMapBlends) {
  current_params c;
  c.speed_mps = 0.3;
  fnv h;
  for (int k = 0; k < 6; ++k) {
    const current_frame f = current_frame_at(c, 7.3 * k + 1000.0);
    h.add(f.tau[0]);
    h.add(f.tau[1]);
    h.add(f.weight[0]);
    h.add(f.weight[1]);
    h.add(f.offset[0][0]);
    h.add(f.offset[1][1]);
    for (int i = 0; i < 64; ++i) {
      const auto v = current_velocity(f, -700.0 + 23.0 * i, 300.0 + 17.0 * (i % 9));
      h.add(v[0]);
      h.add(v[1]);
    }
  }
  EXPECT_EQ(h.h, 0xace1c7be4f9942d6ull) << std::hex << "current digest 0x" << h.h;
  // The two phases' weights keep the variance: w0^2 + w1^2 = 1, each 0 at its restart.
  for (double t = 0.0; t < 30.0; t += 0.37) {
    const current_frame f = current_frame_at(c, t);
    EXPECT_NEAR(f.weight[0] * f.weight[0] + f.weight[1] * f.weight[1], 1.0, 1e-12);
    EXPECT_LE(std::fabs(f.tau[0]), 0.5 * c.flow_period_s + 1e-12);
  }
  const current_frame atRestart = current_frame_at(c, 2.0 * c.flow_period_s);
  EXPECT_NEAR(atRestart.weight[0], 0.0, 1e-12) << "phase 0 restarts with no weight";
}

// ── the physics sea carries them ────────────────────────────────────────────────────────────────

TEST(OceanSea, ThePhysicsSeaIsCalmInTheLeeRuffledByGustsAndCarriedByTheCurrent) {
  const std::shared_ptr<shore_grid> g = islandGrid();
  const sea_state s = windSea(9.0, 0.0);
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  const double F = spectrum_fetch(9.0, s.wind.fetch_m);
  auto forcing = std::make_shared<sea_forcing>();
  forcing->fetch = std::make_shared<fetch_field>(build_fetch(*g, 0.0, s.wind.fetch_m, nullptr, 2));
  std::vector<sea_cascade> sc;
  for (std::size_t i = 0; i < cs.size(); ++i) {
    sea_cascade c{build_h0(64, cs[i], s), 0.0, 0.0};
    c.full_fetch_m = band_full_fetch(9.0, cs[i].k_min, F);
    sc.push_back(std::move(c));
  }
  sea_params p;
  const sea_surface lee(sc, p, g, nullptr, forcing);
  const sea_surface open(sc, p, g, nullptr, nullptr);
  auto rms = [](const sea_surface &sea, double x, double y) {
    double s2 = 0.0;
    int k = 0;
    for (double t = 0.0; t < 30.0; t += 0.25, ++k) {
      const double z = sea.height(x, y, t);
      s2 += z * z;
    }
    return std::sqrt(s2 / k);
  };
  const double behind = rms(lee, kIslandX + kIslandR + 30.0, kIslandY);
  const double ahead = rms(lee, kIslandX - kIslandR - 300.0, kIslandY);
  const double noLee = rms(open, kIslandX + kIslandR + 30.0, kIslandY);
  const double aheadOpen = rms(open, kIslandX - kIslandR - 300.0, kIslandY);
  std::printf("[sea] RMS 30 m behind the island %.3f m (%.3f m without the lee), 300 m ahead of it "
              "%.3f m (%.3f m without)\n",
              behind, noLee, ahead, aheadOpen);
  EXPECT_LT(behind, 0.1 * noLee) << "the lee is calm";
  EXPECT_LT(behind, 0.25 * ahead);
  EXPECT_DOUBLE_EQ(ahead, aheadOpen) << "open water keeps its sea";
  EXPECT_NE(lee.digest(), open.digest());

  // The current: with no cascade carried, the velocity is the waves' plus the current exactly.
  auto withCurrent = std::make_shared<sea_forcing>(*forcing);
  withCurrent->current.speed_mps = 0.5;
  const sea_surface carried(sc, p, g, nullptr, withCurrent);
  const double x = kIslandX - 500.0, y = kIslandY + 300.0, t = 17.0;
  const std::array<double, 3> v0 = lee.velocity(x, y, -2.0, t),
                              v1 = carried.velocity(x, y, -2.0, t);
  const std::array<double, 2> x0 = carried.source(x, y, t);
  const std::array<double, 2> cv = current_velocity(withCurrent->current, x0[0], x0[1], t);
  EXPECT_NEAR(v1[0] - v0[0], cv[0], 1e-12);
  EXPECT_NEAR(v1[1] - v0[1], cv[1], 1e-12);
  EXPECT_NE(cv[0], 0.0);

  // Carried small cascades: the sea is still a sea (the flow map keeps its height), and moves.
  std::vector<sea_cascade> adv = sc;
  adv[1].advect = adv[2].advect = true;
  const sea_surface advected(adv, p, g, nullptr, withCurrent);
  const double ra = rms(advected, x, y), r0 = rms(carried, x, y);
  std::printf(
      "[sea] the small cascades carried by a 0.5 m/s current: RMS %.3f m (not carried %.3f)\n", ra,
      r0);
  EXPECT_NEAR(ra, r0, 0.15 * r0);

  // Gusts move the ripples' weight only: with the ripple cascade gust-weighted, the sea differs.
  std::vector<sea_cascade> gusty = sc;
  gusty[2].gust = 1.0;
  auto windy = std::make_shared<sea_forcing>(*forcing);
  windy->wind = {9.0, 0.0, 0.5};
  const sea_surface ruffled(gusty, p, g, nullptr, windy);
  EXPECT_NE(ruffled.height(x, y, t), lee.height(x, y, t));
}

// The wind's paths through the physics step's reads (sea_surface::at, the per-time tasks): a sea
// mid-crossfade, scaled by two fetch maps, its ripples gusting and carried by the current, reads
// the same bits through a held time whose fields were computed task by task in another order as
// through the plain, cached reads.
TEST(OceanSea, TheWindsSeaAtATimeReadsTheSameValues) {
  const std::shared_ptr<shore_grid> g = islandGrid();
  const sea_state a = windSea(9.0, 0.0), b = windSea(13.0, 0.5);
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  const double F = spectrum_fetch(13.0, b.wind.fetch_m);
  auto forcing = std::make_shared<sea_forcing>();
  forcing->t0 = 15.0;
  forcing->t1 = 19.0;
  forcing->fetch = std::make_shared<fetch_field>(build_fetch(*g, 0.5, b.wind.fetch_m, nullptr, 2));
  forcing->fetch_from =
      std::make_shared<fetch_field>(build_fetch(*g, 0.0, a.wind.fetch_m, nullptr, 2));
  forcing->wind = {13.0, 0.5, 0.5};
  forcing->current.speed_mps = 0.5;
  std::vector<sea_cascade> sc;
  for (std::size_t i = 0; i < cs.size(); ++i) {
    sea_cascade c{build_h0(64, cs[i], b), 0.0, 0.0};
    c.h0_from = build_h0(64, cs[i], a);
    c.full_fetch_m = band_full_fetch(13.0, cs[i].k_min, F);
    c.full_fetch_from_m = band_full_fetch(9.0, cs[i].k_min, F);
    sc.push_back(std::move(c));
  }
  sc[0].decay_k = 0.025;
  sc[2].gust = 1.0;
  sc[1].advect = sc[2].advect = true;
  // Every task, last first: the order they run in must change nothing.
  const sea_surface::parallel_run backwards = [](std::size_t n,
                                                 const std::function<void(std::size_t)> &fn) {
    for (std::size_t i = n; i-- > 0;)
      fn(i);
  };
  for (double cell : {0.0, 0.5}) {
    sea_params p;
    p.band = {32, 64};
    p.chop = 1.15;
    p.source_cell_m = cell;
    const sea_surface plain(sc, p, g, nullptr, forcing), tasked(sc, p, g, nullptr, forcing);
    const double t = 17.25; // mid-crossfade
    const sea_surface::at_time held = tasked.at(t, backwards);
    for (int i = 0; i < 300; ++i) { // across the island's lee and the open water upwind
      const double x = kIslandX - 400.0 + 2.9 * i, y = kIslandY - 60.0 + (i % 13) * 9.5;
      const double z = -0.2 - 0.03 * (i % 40);
      sea_surface::source_memo m1, m2;
      const double h1 = plain.height(x, y, t, m1), h2 = held.height(x, y, m2);
      EXPECT_EQ(std::memcmp(&h1, &h2, sizeof h1), 0) << "height at " << x << ", " << y;
      const std::array<double, 3> v1 = plain.velocity(x, y, z, t, m1),
                                  v2 = held.velocity(x, y, z, m2);
      EXPECT_EQ(std::memcmp(v1.data(), v2.data(), sizeof v1), 0)
          << "velocity at " << x << ", " << y;
    }
    // The band the residual check reads is the same fields as well.
    const sea_surface::band_frame b1 = plain.band(2, t), b2 = tasked.band(2, t);
    EXPECT_EQ(std::memcmp(b1.eta.data(), b2.eta.data(), b1.eta.size() * sizeof(double)), 0);
  }
}
