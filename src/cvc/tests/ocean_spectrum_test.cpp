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

// cvc::ocean — the sea state's spectrum and the cascades' h0 (OCEAN-AND-WEATHER-ROADMAP.md B1,
// E1): dispersion, the JONSWAP peak and fetch law, the swell's height from Hs and Tp, the spread's
// normalisation and rotation with the wind, band-limited cascades that partition the wavenumbers,
// each with its own noise, and h0 that is Hermitian, bit-identical across builds and carries the
// spectrum's energy.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/spectrum.h>
#include <cvc/ocean/whitecaps.h>
#include <gtest/gtest.h>
#include <set>
#include <stdexcept>
#include <vector>

using namespace cvc::ocean;

namespace {
constexpr double kPi = 3.14159265358979323846;

// The spectrum's variance over the k-plane, by a fine sum.
double planeVariance(const sea_state &s, double kmax, int n) {
  const double dk = 2.0 * kmax / n;
  double m = 0.0;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      m += spectrum(-kmax + (i + 0.5) * dk, -kmax + (j + 0.5) * dk, s) * dk * dk;
  return m;
}
} // namespace

TEST(OceanSpectrum, Dispersion) {
  // Deep water: omega^2 = g k. Shallow: omega = k sqrt(g d).
  EXPECT_NEAR(dispersion(0.1, 1000.0), std::sqrt(gravity * 0.1), 1e-12);
  EXPECT_NEAR(dispersion(0.01, 1.0), 0.01 * std::sqrt(gravity * 1.0), 1e-6);
  // d omega / dk against a central difference, deep, intermediate and shallow.
  for (double d : {1000.0, 20.0, 2.0})
    for (double k : {0.02, 0.2, 2.0}) {
      const double h = 1e-6 * k;
      const double fd = (dispersion(k + h, d) - dispersion(k - h, d)) / (2.0 * h);
      EXPECT_NEAR(dispersion_dk(k, d), fd, 1e-6 * std::max(1.0, fd)) << "k " << k << " d " << d;
    }
}

TEST(OceanSpectrum, WindSeaFollowsTheFetchLaw) {
  wind_sea_params w;
  w.speed_mps = 9.0;
  w.fetch_m = 120000.0;
  const jonswap_params p = wind_sea_jonswap(w);
  EXPECT_NEAR(p.alpha, 0.076 * std::pow(81.0 / (120000.0 * gravity), 0.22), 1e-15);
  EXPECT_NEAR(p.peak_omega, 22.0 * std::cbrt(gravity * gravity / (9.0 * 120000.0)), 1e-12);
  // Past the fully developed sea, more fetch changes nothing.
  wind_sea_params far = w, farther = w;
  far.fetch_m = 1e7;
  farther.fetch_m = 1e9;
  EXPECT_EQ(wind_sea_jonswap(far).alpha, wind_sea_jonswap(farther).alpha);
  EXPECT_EQ(wind_sea_jonswap(far).peak_omega, wind_sea_jonswap(farther).peak_omega);
  // A longer fetch grows the sea: a lower peak frequency and a higher Hs.
  sea_state a, b;
  a.wind.fetch_m = 5000.0;
  b.wind.fetch_m = 50000.0;
  EXPECT_GT(wind_sea_jonswap(a.wind).peak_omega, wind_sea_jonswap(b.wind).peak_omega);
  EXPECT_LT(wind_sea_hs(a), wind_sea_hs(b));
}

TEST(OceanSpectrum, JonswapPeaksAtItsPeakFrequency) {
  jonswap_params p;
  p.alpha = 0.01;
  p.peak_omega = 0.9;
  p.gamma = 3.3;
  double best = 0.0, at = 0.0;
  for (double w = 0.3; w < 3.0; w += 0.0005) {
    const double s = jonswap(w, p, 4000.0);
    if (s > best) {
      best = s;
      at = w;
    }
  }
  EXPECT_NEAR(at, 0.9, 0.01);
}

TEST(OceanSpectrum, ShallowWaterDampsTheLongWaves) {
  jonswap_params p;
  p.alpha = 0.01;
  p.peak_omega = 0.6;
  EXPECT_LT(jonswap(0.4, p, 3.0), 0.5 * jonswap(0.4, p, 1000.0)); // TMA
  EXPECT_DOUBLE_EQ(jonswap(3.0, p, 1000.0), jonswap(3.0, p, 1001.0));
}

TEST(OceanSpectrum, TheSwellHasItsHeightAndPeriod) {
  sea_state s;
  s.swell.hs_m = 2.0;
  s.swell.tp_s = 12.0;
  const jonswap_params p = swell_jonswap(s.swell);
  EXPECT_NEAR(p.peak_omega, 2.0 * kPi / 12.0, 1e-15);
  EXPECT_NEAR(p.alpha, 5.061 * 4.0 / std::pow(12.0, 4) * (1.0 - 0.287 * std::log(6.0)), 1e-15);
  EXPECT_NEAR(swell_hs(s), 2.0, 0.2); // DNV-RP-C205's alpha gives back about Hs
  // No swell, no swell spectrum: the spectrum is the wind sea's to the bit.
  sea_state none;
  EXPECT_EQ(swell_spectrum(0.05, 0.01, none), 0.0);
  EXPECT_EQ(spectrum(0.05, 0.01, none), wind_sea_spectrum(0.05, 0.01, none));
  // The swell's energy is at its own wavenumber, travelling its own way.
  s.wind.speed_mps = 0.5; // a calm wind sea
  s.swell.dir_rad = 0.5 * kPi;
  const double kp = p.peak_omega * p.peak_omega / gravity; // deep water
  EXPECT_GT(swell_spectrum(0.0, kp, s), 100.0 * swell_spectrum(0.0, -kp, s));
  EXPECT_GT(swell_spectrum(0.0, kp, s), 100.0 * swell_spectrum(0.0, 4.0 * kp, s)); // cut off
  EXPECT_GT(significant_wave_height(s), 1.8);
}

TEST(OceanSpectrum, TheSpreadIsNormalised) {
  // Donelan-Banner alone integrates to about 1 over a turn; cos^2 x 2 / pi, over the downwind half
  // only, to 1.
  for (double omega : {0.5, 0.9, 2.0}) {
    double db = 0.0, c2 = 0.0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
      const double th = -kPi + (i + 0.5) * 2.0 * kPi / n;
      db += directional(th, omega, 0.9, 1.0, 0.0) * 2.0 * kPi / n;
      c2 += directional(th, omega, 0.9, 0.0, 0.0) * 2.0 * kPi / n;
    }
    EXPECT_NEAR(db, 1.0, 0.05) << omega;
    EXPECT_NEAR(c2, 1.0, 1e-6) << omega;
    // ABYSSAL's cos^2 spans the turn, so its upwind lobe doubled that part's energy and sent it
    // against the wind.
    EXPECT_EQ(directional(0.6 * kPi, omega, 0.9, 0.0, 0.0), 0.0) << omega;
    EXPECT_EQ(directional(-0.9 * kPi, omega, 0.9, 0.0, 0.0), 0.0) << omega;
  }
  // A swell's narrowing makes it narrower: less of it 60 degrees off.
  EXPECT_LT(directional(kPi / 3.0, 0.9, 0.9, 1.0, 1.0), directional(kPi / 3.0, 0.9, 0.9, 1.0, 0.0));
}

TEST(OceanSpectrum, TheWindSeaTravelsDownwindAndTurnsWithTheWind) {
  sea_state s;
  const double k = 0.1;
  const double along = spectrum(k, 0.0, s), across = spectrum(0.0, k, s),
               against = spectrum(-k, 0.0, s);
  EXPECT_GT(along, across);
  EXPECT_GT(along, against);
  // Turned 90 degrees, all of it turns (ABYSSAL's cos^2 term did not; D10).
  sea_state t = s;
  t.wind.dir_rad = 0.5 * kPi;
  EXPECT_NEAR(spectrum(0.0, k, t), along, 1e-12 * along);
  EXPECT_NEAR(spectrum(-k, 0.0, t), across, 1e-12 * along);
  // More wind, more sea.
  sea_state gale = s;
  gale.wind.speed_mps = 20.0;
  EXPECT_GT(significant_wave_height(gale), 1.8 * significant_wave_height(s)); // about U^0.9
  // amplitude scales the variance.
  sea_state half = s;
  half.amplitude = 0.5;
  EXPECT_NEAR(spectrum(k, 0.0, half), 0.5 * along, 1e-12 * along);
}

TEST(OceanSpectrum, TheVarianceMatchesTheHeight) {
  // The spread integrates to 1 over the turn, so the spectrum's variance over the plane is the 1-D
  // spectrum's m0 (deep water, no fades), whatever the blend.
  for (double spread : {0.0, 0.72, 1.0}) {
    sea_state s;
    s.wind.spread = spread;
    const double hs = wind_sea_hs(s), m0 = hs * hs / 16.0;
    EXPECT_NEAR(planeVariance(s, 2.0, 800) / m0, 1.0, 0.06) << spread;
  }
}

// ── cascades and h0 ──────────────────────────────────────────────────────────────────────────────

TEST(OceanH0, CascadesPartitionTheWavenumbers) {
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 42);
  ASSERT_EQ(cs.size(), 3u);
  EXPECT_EQ(cs[0].k_min, 0.0);
  EXPECT_DOUBLE_EQ(cs[0].k_max, 8.0 * kPi / 83.0);
  EXPECT_EQ(cs[1].k_min, cs[0].k_max);
  EXPECT_DOUBLE_EQ(cs[1].k_max, 8.0 * kPi / 29.0);
  EXPECT_EQ(cs[2].k_min, cs[1].k_max);
  EXPECT_GT(cs[2].k_max, 1e100);
  std::set<std::uint64_t> seeds{cs[0].seed, cs[1].seed, cs[2].seed};
  EXPECT_EQ(seeds.size(), 3u); // each its own noise
  EXPECT_EQ(make_cascades({251.0, 83.0, 29.0}, 42)[1].seed, cs[1].seed);
  EXPECT_NE(make_cascades({251.0, 83.0, 29.0}, 43)[1].seed, cs[1].seed);
}

TEST(OceanH0, ZeroOutsideTheBandAndHermitian) {
  const int n = 64;
  std::vector<cascade> cs = make_cascades({200.0, 50.0}, 9);
  sea_state s;
  s.swell.hs_m = 1.5;
  for (const cascade &c : cs) {
    const h0_field h = build_h0(n, c, s);
    ASSERT_EQ(h.texels.size(), std::size_t(n) * n * 4);
    EXPECT_EQ(h.tile_m, c.tile_m);
    int inBand = 0;
    for (int r = 0; r < n; ++r)
      for (int col = 0; col < n; ++col) {
        const float *t = h.texel(col, r);
        const double k = std::hypot(texel_k(col, n, c.tile_m), texel_k(r, n, c.tile_m));
        if (k <= 1e-6 || k < c.k_min || k >= c.k_max) {
          EXPECT_EQ(t[0], 0.0f);
          EXPECT_EQ(t[1], 0.0f);
        } else {
          inBand += t[0] != 0.0f || t[1] != 0.0f;
        }
        // zw is conj h0(-k): the xy of the mirrored texel, its imaginary part negated.
        const float *m = h.texel((n - col) % n, (n - r) % n);
        EXPECT_EQ(t[2], m[0]);
        EXPECT_EQ(t[3], -m[1]);
      }
    EXPECT_GT(inBand, 0);
  }
}

// A sea keyframe built a few rows at a time (off the frame, F4) is build_h0's field to the bit,
// however the rows are split.
TEST(OceanH0, ABuilderStepByStepIsBuildH0) {
  const int n = 64;
  sea_state s;
  s.wind.speed_mps = 17.5;
  s.wind.dir_rad = 0.7;
  s.wind.fetch_m = 80000.0;
  for (const cascade &c : make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull)) {
    const h0_field whole = build_h0(n, c, s);
    for (int rows : {1, 7, 64}) {
      h0_builder b(n, c, s);
      int steps = 0;
      while (!b.step(rows))
        ++steps;
      EXPECT_EQ(b.rows_done(), n);
      EXPECT_EQ(steps, (n + rows - 1) / rows - 1);
      const h0_field parts = b.take();
      EXPECT_EQ(parts.n, whole.n);
      EXPECT_EQ(parts.tile_m, whole.tile_m);
      EXPECT_EQ(parts.texels, whole.texels) << "rows " << rows;
    }
  }
  EXPECT_THROW(h0_builder(48, cascade{}, s), std::invalid_argument);
}

TEST(OceanH0, IsBitIdenticalAcrossBuildsAndSeededPerCascade) {
  const int n = 128;
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  sea_state s;
  s.swell.hs_m = 1.0;
  for (const cascade &c : cs) {
    const h0_field a = build_h0(n, c, s), b = build_h0(n, c, s);
    ASSERT_EQ(a.texels.size(), b.texels.size());
    EXPECT_EQ(std::memcmp(a.texels.data(), b.texels.data(), a.texels.size() * sizeof(float)), 0);
  }
  // The same band under another seed: a different sea, of the same spectrum.
  cascade other = cs[2];
  other.seed ^= 1;
  const h0_field a = build_h0(n, cs[2], s), b = build_h0(n, other, s);
  EXPECT_NE(std::memcmp(a.texels.data(), b.texels.data(), a.texels.size() * sizeof(float)), 0);
}

// The physics band folds h0 into its state_hash, which a native run and a wasm run must agree on,
// so h0 is the same on every platform: the spectrum and the Gaussians call cvc/algo/det_math.h,
// not libm. The digests were measured on Windows (MSVC, x86_64); a platform that disagrees has
// found a determinism leak, to be fixed, never re-baselined.
TEST(OceanH0, IsTheSameOnEveryPlatform) {
  const auto fnv = [](std::uint64_t h, const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 0x100000001b3ull;
    return h;
  };
  sea_state s;
  s.wind.speed_mps = 9.0;
  s.wind.dir_rad = 0.6;
  s.swell.hs_m = 1.2;
  s.swell.tp_s = 11.0;
  s.swell.dir_rad = -0.4;
  s.depth_m = 30.0;
  std::uint64_t spec = 0xcbf29ce484222325ull;
  for (int j = -8; j <= 8; ++j)
    for (int i = -8; i <= 8; ++i) {
      const double v = spectrum(0.037 * i, 0.041 * j, s);
      spec = fnv(spec, &v, sizeof v);
    }
  std::uint64_t h0 = 0xcbf29ce484222325ull;
  for (const cascade &c : make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull)) {
    const h0_field f = build_h0(64, c, s);
    h0 = fnv(h0, f.texels.data(), f.texels.size() * sizeof(float));
  }
  EXPECT_EQ(spec, 0x712605589a89a44eull) << std::hex << "spectrum digest 0x" << spec;
  EXPECT_EQ(h0, 0x512116f1d733aa51ull) << std::hex << "h0 digest 0x" << h0;
}

TEST(OceanH0, CarriesTheSpectrumsEnergy) {
  // E|h0(k)|^2 = S(k) dk^2 amplitude^2 / 2 (a unit complex Gaussian pair has E|g|^2 = 2, times
  // 1/4): over a cascade's band the sum of |h0|^2 is, statistically, half its spectrum's variance,
  // and the sum of |h0(k)|^2 + |h0(-k)|^2, the surface's, all of it. The long band holds a few
  // hundred texels, its energy in a few dozen near the peak, so each cascade is averaged over
  // eight seeds.
  const int n = 256;
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 5);
  sea_state s;
  for (std::size_t ci = 0; ci < cs.size(); ++ci) {
    double sum = 0.0, want = 0.0;
    for (std::uint64_t seed = 0; seed < 8; ++seed) {
      cascade c = cs[ci];
      c.seed += seed;
      c.amplitude = ci == 1 ? 0.5 : 1.0;
      const h0_field h = build_h0(n, c, s);
      const double dk = 2.0 * kPi / c.tile_m;
      for (int r = 0; r < n; ++r)
        for (int col = 0; col < n; ++col) {
          const float *t = h.texel(col, r);
          sum += double(t[0]) * t[0] + double(t[1]) * t[1];
          const double kx = texel_k(col, n, c.tile_m), ky = texel_k(r, n, c.tile_m);
          const double k = std::hypot(kx, ky);
          if (k > 1e-6 && k >= c.k_min && k < c.k_max)
            want += 0.5 * spectrum(kx, ky, s) * dk * dk * c.amplitude * c.amplitude;
        }
    }
    EXPECT_NEAR(sum / want, 1.0, 0.15) << "cascade " << ci;
  }
}

// The cascades together hold the wind sea's variance, so the sea they make has JONSWAP's
// significant height for its wind (4 sqrt(m0)), the height a HUD reports: the bands partition the
// wavenumbers and the spread integrates to 1. Each texel carries h0(k) and conj h0(-k), and the
// surface's variance is the sum over texels of E|h0(k)|^2 + E|h0(-k)|^2 = S(k) dk^2. On
// lsystem_coast's tiles at 128 texels the short tile's last wave is 0.45 m long, and what lies past
// it (ripples over 11.7 rad/s) is a few millionths of the variance.
TEST(OceanH0, TheCascadesHoldTheWindSeasHeight) {
  const int n = 128;
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  for (double u : {4.0, 9.0, 15.0, 25.0}) {
    sea_state s;
    s.wind.speed_mps = u;
    s.wind.dir_rad = 0.4;
    double m = 0.0;
    for (const cascade &c : cs) {
      const double dk = 2.0 * kPi / c.tile_m;
      for (int r = 0; r < n; ++r)
        for (int col = 0; col < n; ++col) {
          const double kx = texel_k(col, n, c.tile_m), ky = texel_k(r, n, c.tile_m);
          const double k = std::hypot(kx, ky);
          if (k > 1e-6 && k >= c.k_min && k < c.k_max)
            m += spectrum(kx, ky, s) * dk * dk;
        }
    }
    std::printf("[sea-height] U %4.1f m/s: the cascades' Hs %.3f m, JONSWAP's %.3f m\n", u,
                4.0 * std::sqrt(m), wind_sea_hs(s));
    EXPECT_NEAR(4.0 * std::sqrt(m) / wind_sea_hs(s), 1.0, 0.03) << u;
  }
}

// The foam field's Jacobian is OceanFFT's: on a single wave, h(k) = h(-k) = a (the surface 2a
// cos k.x), J = 1 + l (Dxx + Dyy) + l^2 (Dxx Dyy - Dxy^2) = 1 - 2 a l k cos k.x (the cross terms
// cancel on one wave), whatever its direction. So its least is 1 - 2alk, its mean 1, and a third
// of the sea lies below 1 - alk (cos > 1/2). A wrong sign on the centred wavenumbers' (-1)^(x+y)
// would turn the wave into a checkerboard, which the sampling's bilinear filter averages away.
TEST(OceanWhitecaps, TheFoamFieldsJacobianIsOceanFFTs) {
  for (int dir = 0; dir < 2; ++dir) {
    h0_field h;
    h.n = 64;
    h.tile_m = 64.0;
    h.texels.assign(std::size_t(h.n) * h.n * 4, 0.0f);
    const int dc = 4, dr = dir == 0 ? 0 : 3; // along x, then oblique
    const double a = 0.5;
    h.texels[(std::size_t(32 + dr) * h.n + (32 + dc)) * 4 + 0] = float(a); // h0(k)
    h.texels[(std::size_t(32 - dr) * h.n + (32 - dc)) * 4 + 2] = float(a); // conj h0(k) at -k
    foam_params p;
    p.times_s = {0.0};
    const foam_field f({h}, p);
    const double k = 2.0 * kPi / h.tile_m * std::hypot(dc, dr);
    const double amp = 2.0 * a * p.chop * k;
    std::printf("[whitecaps] one wave (%d, %d): least J %.4f (1 - 2alk %.4f), mean %.4f, "
                "below 1 - alk %.4f\n",
                dc, dr, f.least(), 1.0 - amp, f.mean(), f.below(1.0 - 0.5 * amp));
    EXPECT_NEAR(f.least(), 1.0 - amp, 0.01 * amp);
    EXPECT_NEAR(f.mean(), 1.0, 1e-3);
    EXPECT_NEAR(f.below(1.0 - 0.5 * amp), 1.0 / 3.0, 0.01);
  }
}

// Whitecaps cover Monahan's share for the wind: W = 3.84e-6 U^3.41, 0.7 % at 9 m/s, at most 16 %.
TEST(OceanWhitecaps, MonahansCoverage) {
  EXPECT_DOUBLE_EQ(monahan_coverage(0.0), 0.0);
  EXPECT_DOUBLE_EQ(monahan_coverage(-3.0), 0.0);
  EXPECT_NEAR(monahan_coverage(9.0), 0.0069, 0.0001);
  EXPECT_NEAR(monahan_coverage(15.0), 3.84e-6 * std::pow(15.0, 3.41), 1e-12);
  EXPECT_DOUBLE_EQ(monahan_coverage(40.0), 0.16);
}

// lsystem_coast's sea (its three cascades at 256 texels, chop 1.15, the wind sea alone, drawn on
// its 224^2 grid over 440 m, a vertex every 1.97 m). On it the least J is 0.5-0.6 at every wind, so
// the foam it drew with (bias 0.48, ramp 0.85) never showed. The bias for Monahan's share now
// rises from about 0.75 at 4 m/s to 0.86 at 25 m/s. The ramp is 0.1 rather than 0.85 (or
// ABYSSAL's 0.3, set on a sea 2.5 times higher): the least J sits only about 0.35 below its mean,
// and a wide ramp meets Monahan's share with a faint veil (at 9 m/s, any foam on 51 % of the sea
// with ramp 0.85, 14 % with 0.3, and none of it half white), where at 0.1 most of the foam is
// white. lsystem_coast also frays the edges (fray 0.12), which the bias takes into account and
// which widens the soft edge: the share at least half white is 0.68-0.95 of Monahan's (within
// 30 % without the fray).
TEST(OceanWhitecaps, TheBiasGivesMonahansShareOnTheCoastsSea) {
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  for (double u : {4.0, 9.0, 15.0, 25.0}) {
    sea_state s;
    s.wind.speed_mps = u;
    s.wind.fetch_m = 120000.0;
    s.wind.dir_rad = std::atan2(0.55, 1.0);
    std::vector<h0_field> hs;
    for (const cascade &c : cs)
      hs.push_back(build_h0(256, c, s));
    foam_params old;
    old.ramp = 0.85;
    old.mesh_m = 2.0 * 220.0 / 223.0;
    foam_params p = old;
    p.ramp = 0.1;
    p.fray = 0.12;
    const auto t0 = std::chrono::steady_clock::now();
    const foam_field f(hs, p);
    const double w = monahan_coverage(u);
    const double b = f.bias_for(w);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const double before = foam_field(hs, old).coverage(0.48);
    const double white = f.white(b), any = f.white(b, 1e-6);
    std::printf("[whitecaps] U %4.1f m/s: least J %.3f, mean %.3f; bias 0.48 / ramp 0.85 covered "
                "%.4f %%; Monahan %.4f %% at bias %.4f (ramp 0.1, fray 0.12): half white %.4f %%, "
                "any foam %.3f %% (%.0f ms)\n",
                u, f.least(), f.mean(), 100.0 * before, 100.0 * w, b, 100.0 * white, 100.0 * any,
                ms);
    EXPECT_NEAR(f.coverage(b) / w, 1.0, 0.01) << u;
    EXPECT_GT(white / w, 0.6) << u;
    EXPECT_LT(white / w, 1.3) << u;
  }
}

// ── persistent foam (section 10.2, O5) ──────────────────────────────────────────────────────────

// Foam builds while a texel folds and decays at its rate after: after the fold is gone it is
// exp(-t / decay_s) of what it was, whatever the frame rate. Bubbles build at bubble_gain of the
// rate and decay over bubble_decay_s; the crest is the injection; spray latches it and decays in a
// sixth of a second. A stall's long step integrates at most max_dt_s.
TEST(OceanFoam, BuildsWhileItFoldsAndDecaysAtItsRate) {
  foam_persistence p; // decay 2.5 s, build 4 / s, bubbles 6 s at 0.55
  foam_state s;
  for (int i = 0; i < 6; ++i) // 0.1 s of a full fold: 0.4 of foam, 0.22 of bubbles
    foam_step(s, 1.0, 1.0 / 60.0, p);
  EXPECT_NEAR(s.foam, 0.4, 0.01);
  EXPECT_NEAR(s.bubbles, 0.22, 0.01);
  EXPECT_DOUBLE_EQ(s.crest, 1.0);
  EXPECT_DOUBLE_EQ(s.spray, 1.0);
  for (int i = 0; i < 30; ++i) // 0.5 s more: white (clamped at 1)
    foam_step(s, 1.0, 1.0 / 60.0, p);
  EXPECT_DOUBLE_EQ(s.foam, 1.0);
  // The fold is gone: 2.5 s later the foam is e^-1 of 1, at 60 or at 25 frames a second.
  foam_state a = s, b = s;
  for (int i = 0; i < 150; ++i)
    foam_step(a, 0.0, 1.0 / 60.0, p);
  for (int i = 0; i < 62; ++i)
    foam_step(b, 0.0, 0.04, p);
  foam_step(b, 0.0, 0.02, p);
  std::printf("[foam] after the fold: foam %.6f / %.6f (e^-1 %.6f), bubbles %.4f, spray %.2e\n",
              a.foam, b.foam, std::exp(-1.0), a.bubbles, a.spray);
  EXPECT_NEAR(a.foam, std::exp(-1.0), 1e-9);
  EXPECT_NEAR(b.foam, std::exp(-1.0), 1e-9);
  EXPECT_NEAR(a.bubbles, s.bubbles * std::exp(-2.5 / 6.0), 1e-9);
  EXPECT_DOUBLE_EQ(a.crest, 0.0);
  EXPECT_NEAR(a.spray, std::exp(-2.5 * 6.0), 1e-9);
  // A 3 s stall integrates 0.25 s.
  foam_state c = s;
  foam_step(c, 0.0, 3.0, p);
  EXPECT_NEAR(c.foam, std::exp(-0.25 / 2.5), 1e-12);
  // A half-folded texel injects smoothstep: J = bias - ramp / 2 gives 0.5.
  EXPECT_NEAR(foam_inject(0.75, 0.8, 0.1), 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(foam_inject(0.85, 0.8, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(foam_inject(0.65, 0.8, 0.1), 1.0);
}

// Aging foam thins into windrow streaks: fresh foam is white over most of the windrow noise, half
// of it only where the noise is high, and it is gone below about 0.2 whatever the noise.
TEST(OceanFoam, AgingFoamThinsIntoStreaks) {
  int fresh = 0, half = 0;
  for (int i = 0; i <= 100; ++i) {
    const double w = i / 100.0;
    fresh += foam_look(1.0, w) > 0.9 ? 1 : 0;
    half += foam_look(0.5, w) > 0.9 ? 1 : 0;
    EXPECT_DOUBLE_EQ(foam_look(0.2, w), 0.0) << w;
    EXPECT_LE(foam_look(0.4, w), foam_look(0.6, w) + 1e-12) << w;
  }
  std::printf("[foam] white over %d %% of the windrow noise when fresh, %d %% at half\n", fresh,
              half);
  EXPECT_GT(fresh, 60);
  EXPECT_LT(half, 50);
  EXPECT_GT(half, 5);
}

namespace {
std::vector<h0_field> coast_sea(double u) {
  const std::vector<cascade> cs = make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  sea_state s;
  s.wind.speed_mps = u;
  s.wind.fetch_m = 120000.0;
  s.wind.dir_rad = std::atan2(0.55, 1.0);
  std::vector<h0_field> hs;
  for (const cascade &c : cs)
    hs.push_back(build_h0(256, c, s));
  return hs;
}
foam_params coast_foam() {
  foam_params p;
  p.ramp = 0.1;
  p.fray = 0.12;
  p.mesh_m = 2.0 * 220.0 / 223.0;
  return p;
}
} // namespace

// The history is deterministic: the same sea and parameters give the same bits, however its
// Jacobians are stepped (all at once, or one at a time as a sliced task would).
TEST(OceanFoam, TheHistoryIsDeterministic) {
  const std::vector<h0_field> hs = coast_sea(12.0);
  foam_params p = coast_foam();
  p.mesh_m = 6.0; // a coarse lattice: this is about the bits, not the share
  foam_persistence q;
  foam_history a(hs, p, q), b(hs, p, q);
  EXPECT_THROW(a.coverage({}), std::logic_error);
  EXPECT_FALSE(a.ready());
  a.step(0);
  std::size_t calls = 0;
  while (!b.step(1))
    ++calls;
  EXPECT_EQ(calls + 1, b.units());
  ASSERT_TRUE(a.ready() && b.ready());
  for (double bias : {0.7, 0.8, 0.9}) {
    const foam_biases fb{bias, bias + 0.1};
    const double ca = a.coverage(fb), cb = b.coverage(fb);
    EXPECT_EQ(std::memcmp(&ca, &cb, sizeof ca), 0) << bias;
    EXPECT_GE(ca + 1e-12, a.active(fb.whitecaps)) << bias;
    EXPECT_GE(ca + 1e-12, a.persistent(fb.persistent)) << bias;
  }
  EXPECT_LT(a.persistent(0.8), a.persistent(0.9));
  EXPECT_LT(a.active(0.7), a.active(0.8));
  foam_persistence bad;
  bad.decay_s = 0.0;
  EXPECT_THROW(foam_history(hs, p, bad), std::invalid_argument);
  bad = q;
  bad.share = 1.0;
  EXPECT_THROW(foam_history(hs, p, bad), std::invalid_argument);
}

// With persistent foam the drawn foam still covers Monahan's share for the wind on lsystem_coast's
// sea: the aging foam alone draws its share of it (0.6), and the whitecaps' bias, lower than for
// the whitecaps alone, makes up the rest.
TEST(OceanFoam, TheBiasesGiveMonahansShareWithPersistentFoam) {
  for (double u : {4.0, 9.0, 15.0, 25.0}) {
    const std::vector<h0_field> hs = coast_sea(u);
    const foam_params p = coast_foam();
    const foam_persistence q;
    const auto t0 = std::chrono::steady_clock::now();
    foam_history h(hs, p, q);
    h.step(0);
    const auto t1 = std::chrono::steady_clock::now();
    const double w = monahan_coverage(u);
    const foam_biases b = h.biases_for(w);
    const auto t2 = std::chrono::steady_clock::now();
    const double inst = foam_field(hs, p).bias_for(w);
    const double ms1 = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double ms2 = std::chrono::duration<double, std::milli>(t2 - t1).count();
    const double cov = h.coverage(b), pers = h.persistent(b.persistent),
                 act = h.active(b.whitecaps);
    std::printf("[foam] U %4.1f m/s: Monahan %.4f %%: whitecaps' bias %.4f (alone %.4f), the "
                "aging foam's %.4f; drawn %.4f %%, the aging foam %.4f %%, the whitecaps %.4f %% "
                "(%zu Jacobians %.0f ms, the biases %.0f ms)\n",
                u, 100.0 * w, b.whitecaps, inst, b.persistent, 100.0 * cov, 100.0 * pers,
                100.0 * act, h.units(), ms1, ms2);
    EXPECT_NEAR(cov / w, 1.0, 0.02) << u;
    EXPECT_NEAR(pers / (q.share * w), 1.0, 0.03) << u;
    EXPECT_LT(b.whitecaps, inst) << u;
  }
}

// The history's time step: the long cascades' Jacobians every 0.4 s and 0.23 s, J a spline between,
// give nearly the aging foam a history four times finer (and so near the GPU's frame-by-frame
// integration) gives.
TEST(OceanFoam, TheHistorysStepIsFineEnough) {
  const std::vector<h0_field> hs = coast_sea(9.0);
  const foam_params p = coast_foam();
  const foam_persistence q;
  foam_history coarse(hs, p, q);
  foam_history_params fine_p;
  fine_p.step_s = 0.1;
  fine_p.dt_s = 1.0 / 60.0;
  foam_history fine(hs, p, q, fine_p);
  coarse.step(0);
  fine.step(0);
  const foam_biases b = coarse.biases_for(monahan_coverage(9.0));
  const double c = coarse.persistent(b.persistent), f = fine.persistent(b.persistent);
  const double cc = coarse.coverage(b), fc = fine.coverage(b);
  std::printf("[foam] at the 9 m/s biases the 0.4 s history's aging foam covers %.4f %% (drawn "
              "%.4f %%), a 0.1 s one's %.4f %% (%.4f %%); %zu and %zu Jacobians\n",
              100.0 * c, 100.0 * cc, 100.0 * f, 100.0 * fc, coarse.units(), fine.units());
  EXPECT_NEAR(c / f, 1.0, 0.1);
  EXPECT_NEAR(cc / fc, 1.0, 0.1);
}

TEST(OceanH0, RejectsABadTile) {
  cascade c;
  sea_state s;
  EXPECT_THROW(build_h0(100, c, s), std::invalid_argument); // not a power of two
  EXPECT_THROW(build_h0(1, c, s), std::invalid_argument);
  c.tile_m = 0.0;
  EXPECT_THROW(build_h0(64, c, s), std::invalid_argument);
}
