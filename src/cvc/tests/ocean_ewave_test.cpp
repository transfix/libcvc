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

// cvc::ocean::ewave — the waves bodies make (OCEAN-AND-WEATHER-ROADMAP.md C1, C3, C5): a single
// mode oscillates at omega(k) exactly for any dt; without damping, mask or sponge the energy is
// conserved; a drop makes a round ring whose envelope moves at the group velocity; a wall reflects
// it, and a wall band with water behind it lets only a few percent through; the sponge absorbs
// it; a beach absorbs the same share per second at any step size; shallow water carries a long
// wave at sqrt(g d); a moving source leaves a Kelvin wake at arcsin(1/3); the same steps give the
// same digest, the same on every platform; and bad parameters are refused.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ocean/ewave.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>

using namespace cvc::ocean;

namespace {

constexpr double kPi = 3.14159265358979323846;

ewave_params bare(int n, double dx, double depth = 1000.0) {
  ewave_params p;
  p.n = n;
  p.dx = dx;
  p.depth = depth;
  p.sponge_cells = 0.0;
  return p;
}

// A Gaussian hump of height a and width sigma at (x0, y0), as a whole field.
std::vector<double> gaussian(const ewave &w, double x0, double y0, double a, double sigma) {
  const int n = w.size();
  std::vector<double> h(std::size_t(n) * std::size_t(n));
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double dx = w.x_of(i) - x0, dy = w.y_of(j) - y0;
      h[std::size_t(j) * std::size_t(n) + std::size_t(i)] =
          a * std::exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
    }
  return h;
}

double group_velocity(const ewave &w, double k) {
  const double e = 1e-6 * k;
  return (w.omega(k + e) - w.omega(k - e)) / (2.0 * e);
}

// The radius (m) of the envelope's peak around (x0, y0): h^2 averaged over the circle, then over
// a band of one wavelength (to drop the carrier).
double ring_radius(const ewave &w, double x0, double y0, double rmax, double band) {
  const double dr = 0.05;
  const int nr = int(rmax / dr);
  std::vector<double> e(std::size_t(nr), 0.0);
  for (int r = 0; r < nr; ++r) {
    const double rr = r * dr;
    double s = 0.0;
    const int na = 256;
    for (int a = 0; a < na; ++a) {
      const double t = 2.0 * kPi * a / na;
      const double v = w.at(x0 + rr * std::cos(t), y0 + rr * std::sin(t));
      s += v * v;
    }
    e[std::size_t(r)] = s / na;
  }
  const int hb = std::max(1, int(0.5 * band / dr));
  double best = -1.0, at = 0.0;
  for (int r = hb; r + hb < nr; ++r) {
    double s = 0.0;
    for (int q = r - hb; q <= r + hb; ++q)
      s += e[std::size_t(q)];
    if (s > best) {
      best = s;
      at = r * dr;
    }
  }
  return at;
}

} // namespace

// One standing mode, cos(k . x), in a periodic window: h(t) = A cos(omega t) cos(k . x) and
// phi(t) = -(g / omega) A sin(omega t) cos(k . x), at a dt far beyond any CFL limit, over many
// steps, in deep and in shallow water.
TEST(Ewave, StandingModeOscillatesAtOmegaExactly) {
  struct mode_case {
    int mx, my;
    double depth, dt;
  };
  for (const mode_case mc :
       {mode_case{3, 0, 1000.0, 0.7}, mode_case{2, 5, 1.5, 1.3}, mode_case{32, 0, 1000.0, 0.21}}) {
    ewave w(bare(64, 0.5, mc.depth));
    const double L = 64 * 0.5, A = 0.3;
    const double kx = 2.0 * kPi * mc.mx / L, ky = 2.0 * kPi * mc.my / L;
    std::vector<double> h0(64 * 64), phi0(64 * 64, 0.0);
    for (int j = 0; j < 64; ++j)
      for (int i = 0; i < 64; ++i)
        h0[std::size_t(j) * 64 + std::size_t(i)] = A * std::cos(kx * w.x_of(i) + ky * w.y_of(j));
    w.set_state(h0, phi0);
    // The dispersion relation, computed here rather than taken from the library.
    const double k = std::sqrt(kx * kx + ky * ky),
                 om = std::sqrt(9.80665 * k * std::tanh(k * mc.depth));
    EXPECT_NEAR(w.omega(k), om, 1e-12 * om);
    const int steps = 400;
    double worst_h = 0.0, worst_phi = 0.0;
    for (int s = 1; s <= steps; ++s) {
      w.step(mc.dt);
      if (s % 50 != 0)
        continue;
      const double t = s * mc.dt, ch = std::cos(om * t), sp = -(9.80665 / om) * std::sin(om * t);
      for (std::size_t c = 0; c < h0.size(); ++c) {
        worst_h = std::max(worst_h, std::abs(w.height()[c] - ch * h0[c]));
        worst_phi = std::max(worst_phi, std::abs(w.phi()[c] - sp * h0[c]));
      }
    }
    std::printf("mode (%d, %d), depth %g, dt %g s (omega dt %.2f rad), %d steps: worst |dh| %.2e "
                "m, |dphi| %.2e m^2/s\n",
                mc.mx, mc.my, mc.depth, mc.dt, om * mc.dt, steps, worst_h, worst_phi);
    EXPECT_LT(worst_h, 1e-10 * A);
    EXPECT_LT(worst_phi, 1e-10 * A * 9.80665 / om);
  }
}

// Without damping, mask or sponge, each mode's g |H|^2 + kappa |PHI|^2 is conserved, so the
// energy is, to round-off, while it moves between potential and kinetic.
TEST(Ewave, EnergyIsConservedWithoutDampingMaskOrSponge) {
  for (double depth : {1000.0, 2.0}) {
    ewave w(bare(64, 0.5, depth));
    w.add_height(6.0, 9.0, 0.2, 2.0);
    w.add_height(20.0, 14.0, -0.1, 3.5);
    w.add_height(25.0, 25.0, 0.05, 1.0);
    const double e0 = w.energy();
    double worst = 0.0, min_pot = 1.0;
    for (int s = 0; s < 2000; ++s) {
      w.step(0.05);
      worst = std::max(worst, std::abs(w.energy() - e0) / e0);
      min_pot = std::min(min_pot, w.potential_energy() / w.energy());
    }
    std::printf("depth %g: energy %.6g J, worst relative drift over 2000 steps %.2e (potential "
                "share down to %.2f)\n",
                depth, e0, worst, min_pot);
    EXPECT_LT(worst, 1e-11);
    EXPECT_LT(min_pot, 0.9); // it did move into kinetic energy
  }
}

// A Gaussian drop of width sigma in deep water: the ring is round, and its envelope's peak moves
// out at the group velocity of the dominant wavenumber. By stationary phase the ring's amplitude
// at time t goes as exp(-k^2 sigma^2 / 2) k^(3/2) in the local wavenumber k = g t^2 / (4 r^2),
// which peaks at k = sqrt(1.5) / sigma.
TEST(Ewave, DropMakesARoundRingAtTheGroupVelocity) {
  ewave_params p = bare(256, 0.25);
  p.origin_x = p.origin_y = -32.0;
  p.sponge_cells = 16.0;
  ewave w(p);
  const double sigma = 0.5;
  w.set_state(gaussian(w, 0.0, 0.0, 0.1, sigma), std::vector<double>(256 * 256, 0.0));
  const double kd = std::sqrt(1.5) / sigma, cg = group_velocity(w, kd), lam = 2.0 * kPi / kd;
  const double dt = 0.1, t1 = 6.0, t2 = 14.0;
  double r1 = 0.0, r2 = 0.0;
  for (int s = 1; s <= int(t2 / dt + 0.5); ++s) {
    w.step(dt);
    if (s == int(t1 / dt + 0.5))
      r1 = ring_radius(w, 0.0, 0.0, 26.0, lam);
  }
  r2 = ring_radius(w, 0.0, 0.0, 26.0, lam);
  // Isotropy: the height along 16 rays, at radii around r2.
  double lo = 1e9, hi = 0.0, worst = 0.0;
  for (int a = 0; a < 16; ++a) {
    const double t = 2.0 * kPi * a / 16.0;
    double m = 0.0;
    for (double r = r2 - lam; r <= r2 + lam; r += 0.02)
      m = std::max(m, std::abs(w.at(r * std::cos(t), r * std::sin(t))));
    lo = std::min(lo, m);
    hi = std::max(hi, m);
  }
  // And point by point against the ray at angle 0, across the ring. (Nearer the centre linger
  // the shortest waves, which the square lattice cuts off at pi / dx along the axes but
  // sqrt(2) pi / dx along the diagonals.) at() is bilinear, and the dominant wavelength is about
  // 10 cells, so the sampling alone misses a crest by up to 1 - cos(k dx / 2), about 5 %.
  for (double r = r2 - 2.0 * lam; r <= r2 + 2.0 * lam; r += 0.05) {
    const double ref = w.at(r, 0.0);
    for (int a = 1; a < 16; ++a) {
      const double t = 2.0 * kPi * a / 16.0;
      worst = std::max(worst, std::abs(w.at(r * std::cos(t), r * std::sin(t)) - ref));
    }
  }
  const double speed = (r2 - r1) / (t2 - t1);
  std::printf("drop sigma %.2f m: dominant wavelength %.2f m, group velocity %.3f m/s; ring at "
              "%.2f m (t %.0f s) and %.2f m (t %.0f s): %.3f m/s (%.1f %%)\n",
              sigma, lam, cg, r1, t1, r2, t2, speed, 100.0 * (speed / cg - 1.0));
  std::printf("  isotropy: crest height over 16 rays %.4g..%.4g m (spread %.1f %%), worst "
              "difference from the ray at 0 %.2e m\n",
              lo, hi, 100.0 * (hi - lo) / hi, worst);
  EXPECT_NEAR(speed / cg, 1.0, 0.1);
  EXPECT_LT((hi - lo) / hi, 0.05);
  EXPECT_LT(worst, 0.1 * hi);
}

// A straight wall (O8's exit): mask 0 over the half-plane x >= 0. The ring from a drop at x = -6 m
// reflects off it, nothing appears beyond it, and the reflection comes back. h = phi = 0 on the
// wall makes the reflection the antisymmetric image, the free solution h(x) minus its mirror; the
// image fits best about x = dx / 2, half a cell into the wall.
TEST(Ewave, ARingReflectsOffAStraightWall) {
  ewave_params p = bare(256, 0.25);
  p.origin_x = p.origin_y = -32.0;
  p.sponge_cells = 16.0;
  ewave wall(p), free_(p);
  std::vector<double> mask(256 * 256, 1.0);
  for (int j = 0; j < 256; ++j)
    for (int i = 0; i < 256; ++i)
      if (wall.x_of(i) >= 0.0)
        mask[std::size_t(j) * 256 + std::size_t(i)] = 0.0;
  wall.set_mask(mask);
  const std::vector<double> drop = gaussian(wall, -6.0, 0.0, 0.1, 0.5);
  wall.set_state(drop, std::vector<double>(256 * 256, 0.0));
  free_.set_state(drop, std::vector<double>(256 * 256, 0.0));
  double beyond = 0.0, incident = 0.0;
  const double dt = 0.1;
  for (int s = 1; s <= 130; ++s) {
    wall.step(dt);
    free_.step(dt);
    for (int j = 0; j < 256; ++j)
      for (int i = 0; i < 256; ++i) {
        const double x = wall.x_of(i);
        const std::size_t c = std::size_t(j) * 256 + std::size_t(i);
        if (x >= 0.0)
          beyond = std::max(beyond, std::abs(wall.height()[c]));
        if (x >= -0.5 && x < 0.0)
          incident = std::max(incident, std::abs(free_.height()[c]));
      }
  }
  // At t = 13 s the ring's peak (1 m/s) has gone 13 m: reflected, it is 7 m back from the wall.
  // Compare x < 0 with the image, and the reflection's part of it for scale.
  double err = 0.0, norm = 0.0, refl = 0.0;
  for (int j = 16; j < 240; ++j)
    for (int i = 16; i < 128; ++i) {
      const std::size_t c = std::size_t(j) * 256 + std::size_t(i),
                        m = std::size_t(j) * 256 + std::size_t(257 - i); // x -> dx - x
      const double img = free_.height()[c] - free_.height()[m];
      err += (wall.height()[c] - img) * (wall.height()[c] - img);
      norm += img * img;
      refl += free_.height()[m] * free_.height()[m];
    }
  const double rel = std::sqrt(err / norm), share = std::sqrt(refl / norm);
  std::printf("wall: the incident ring reached %.3g m at the wall; beyond it at most %.1e m; x < 0 "
              "against the antisymmetric image: %.1f %% RMS (the reflection is %.0f %% of that "
              "field's RMS)\n",
              incident, beyond, 100.0 * rel, 100.0 * share);
  EXPECT_EQ(beyond, 0.0); // by construction: the mask zeroes the whole half-plane every step
  EXPECT_GT(share, 0.5);  // the reflection is a large part of the field there
  EXPECT_LT(rel, 0.25);
}

namespace {

// A wall band of mask 0 over 0 <= x < thick with open water behind it, hit by the ring of a drop
// at x = -6 m. Returns the largest |h| behind the band (x >= thick, out of the sponge) over the
// run, over the free run's largest |h| just before the wall (x in [-0.5, 0)).
double wall_leak(double thick, double sponge_cells, double sponge_rate, int n = 256) {
  ewave_params p = bare(n, 0.25);
  p.origin_x = p.origin_y = -0.125 * n;
  p.sponge_cells = sponge_cells;
  p.sponge_rate = sponge_rate;
  ewave wall(p), free_(p);
  std::vector<double> mask(std::size_t(n) * std::size_t(n), 1.0);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      if (wall.x_of(i) >= 0.0 && wall.x_of(i) < thick)
        mask[std::size_t(j) * std::size_t(n) + std::size_t(i)] = 0.0;
  wall.set_mask(mask);
  const std::vector<double> drop = gaussian(wall, -6.0, 0.0, 0.1, 0.5);
  wall.set_state(drop, std::vector<double>(std::size_t(n) * std::size_t(n), 0.0));
  free_.set_state(drop, std::vector<double>(std::size_t(n) * std::size_t(n), 0.0));
  const int edge = int(sponge_cells + 0.5);
  double behind = 0.0, incident = 0.0;
  for (int s = 1; s <= 200; ++s) {
    wall.step(0.1);
    free_.step(0.1);
    for (int j = edge; j < n - edge; ++j)
      for (int i = edge; i < n - edge; ++i) {
        const double x = wall.x_of(i);
        const std::size_t c = std::size_t(j) * std::size_t(n) + std::size_t(i);
        if (x >= thick)
          behind = std::max(behind, std::abs(wall.height()[c]));
        if (x >= -0.5 && x < 0.0)
          incident = std::max(incident, std::abs(free_.height()[c]));
      }
  }
  return behind / incident;
}

} // namespace

// A wall of finite thickness with water behind it leaks a little: the mask is a lid on the
// surface with water under it, and the spectral operator couples across it. A 12-cell (3 m) band
// lets through a few percent of the incident amplitude, a 6 m band less. Behind a thick band most
// of what is left came round the periodic window through the sponge: in a window twice as wide
// (n 512) a 6 m band reads 1.4 % instead of 2.8 %, and a 12 m band 1.0 % instead of 3.1 %.
TEST(Ewave, AThinWallLeaksLittle) {
  const double leak3 = wall_leak(3.0, 16.0, 2.0), leak6 = wall_leak(6.0, 16.0, 2.0);
  std::printf("wall band 3 m (12 cells): %.2f %% of the incident amplitude behind it; 6 m: %.2f "
              "%%\n",
              100.0 * leak3, 100.0 * leak6);
  EXPECT_LT(leak3, 0.06);
  EXPECT_LT(leak6, leak3);
}

// The sponge absorbs: once the ring has passed the window's edge, more than 90 % of the energy is
// gone; without the sponge it is all still there (wrapped around).
TEST(Ewave, TheSpongeAbsorbsTheRing) {
  ewave_params p = bare(128, 0.25);
  p.origin_x = p.origin_y = -16.0;
  p.sponge_cells = 16.0;
  ewave_params q = p;
  q.sponge_cells = 0.0;
  ewave w(p), open(q);
  const std::vector<double> drop = gaussian(w, 0.0, 0.0, 0.1, 0.5);
  w.set_state(drop, std::vector<double>(128 * 128, 0.0));
  open.set_state(drop, std::vector<double>(128 * 128, 0.0));
  const double e0 = w.energy();
  for (int s = 0; s < 300; ++s) {
    w.step(0.1);
    open.step(0.1);
  }
  const double left = w.energy() / e0, kept = open.energy() / e0;
  std::printf("sponge (%g cells, %g /s): %.2f %% of the energy left after 30 s; without it "
              "%.4f %%\n",
              p.sponge_cells, p.sponge_rate, 100.0 * left, 100.0 * kept);
  EXPECT_LT(left, 0.1);
  EXPECT_NEAR(kept, 1.0, 1e-9);
}

namespace {

// The energy left (a fraction) after 10 s when a ring crosses a beach band: the mask ramps from 1
// down to 0.2 and back over 4 <= x < 12 m, open water on both sides, no sponge, stepped at dt.
double beach_left(double dt) {
  ewave_params p = bare(128, 0.25);
  p.origin_x = p.origin_y = -16.0;
  ewave w(p);
  std::vector<double> mask(128 * 128, 1.0);
  for (int j = 0; j < 128; ++j)
    for (int i = 0; i < 128; ++i) {
      const double x = w.x_of(i);
      if (x >= 4.0 && x < 12.0)
        mask[std::size_t(j) * 128 + std::size_t(i)] = 1.0 - 0.8 * (1.0 - std::abs(x - 8.0) / 4.0);
    }
  w.set_mask(mask);
  w.set_state(gaussian(w, 0.0, 0.0, 0.1, 0.5), std::vector<double>(128 * 128, 0.0));
  const double e0 = w.energy();
  const int steps = int(10.0 / dt + 0.5);
  for (int s = 0; s < steps; ++s)
    w.step(dt);
  return w.energy() / e0;
}

} // namespace

// A beach's absorption is a rate (beach_rate (1 - m), applied as exp(-rate dt)), so it takes the
// same share of a wave per second whatever the step size: 30 Hz and 60 Hz agree.
TEST(Ewave, ABeachAbsorbsTheSameAtAnyStepSize) {
  const double a = beach_left(1.0 / 30.0), b = beach_left(1.0 / 60.0), c = beach_left(0.1);
  std::printf(
      "beach: energy left after 10 s %.2f %% at 30 Hz, %.2f %% at 60 Hz, %.2f %% at 10 Hz\n",
      100.0 * a, 100.0 * b, 100.0 * c);
  EXPECT_LT(a, 0.8); // the beach absorbs
  EXPECT_NEAR((1.0 - a) / (1.0 - b), 1.0, 0.03);
  EXPECT_NEAR((1.0 - c) / (1.0 - b), 1.0, 0.1);
}

// Finite depth: in water 1 m deep a long plane wave (kd about 0.1) splits into two pulses that
// travel at about sqrt(g d).
TEST(Ewave, AShallowLongWaveTravelsAtSqrtGD) {
  ewave_params p = bare(256, 1.0, 1.0);
  p.origin_x = p.origin_y = -128.0;
  ewave w(p);
  std::vector<double> h(256 * 256);
  for (int j = 0; j < 256; ++j)
    for (int i = 0; i < 256; ++i)
      h[std::size_t(j) * 256 + std::size_t(i)] = 0.05 * std::exp(-w.x_of(i) * w.x_of(i) / 128.0);
  w.set_state(h, std::vector<double>(256 * 256, 0.0));
  const double t = 20.0;
  for (int s = 0; s < 40; ++s)
    w.step(0.5);
  // The right pulse's peak on row 128, refined by a parabola.
  const double *row = w.height().data() + std::size_t(128) * 256;
  int best = 129;
  for (int i = 129; i < 255; ++i)
    if (row[i] > row[best])
      best = i;
  const double a = row[best - 1], b = row[best], c = row[best + 1];
  const double x = w.x_of(best) + 0.5 * (a - c) / (a - 2.0 * b + c);
  const double c0 = std::sqrt(9.80665 * 1.0), speed = x / t;
  std::printf("shallow (d 1 m): pulse at %.2f m after %.0f s: %.3f m/s against sqrt(g d) %.3f m/s "
              "(%.2f %%), height %.4f m of 0.025\n",
              x, t, speed, c0, 100.0 * (speed / c0 - 1.0), b);
  EXPECT_NEAR(speed / c0, 1.0, 0.03);
  EXPECT_NEAR(b, 0.025, 0.003);
}

namespace {

// The Kelvin wake of a Gaussian footprint (depth 5 cm, width sigma 1 m) moving at u in deep water,
// applied as C2's source -(zeta(t) - zeta(t - dt)). The source travels `travel` metres; rows
// across the track 16-44 m behind it are then read. For each threshold frac, the envelope's edge
// on a row is the outermost lateral distance where |h| exceeds frac of the row's maximum, and its
// slope against the distance behind the source, fitted, gives a half-angle (degrees); `mean` is
// the mean of atan(y / s) for the same edge. `peak` is the fitted angle of the row maxima.
struct kelvin_reading {
  std::vector<double> fitted, mean;
  double peak = 0.0;
};

double fit_angle(const std::vector<double> &ss, const std::vector<double> &ys, double *mean) {
  double sx = 0, sy = 0, sxx = 0, sxy = 0, ma = 0;
  const double k = double(ss.size());
  for (std::size_t q = 0; q < ss.size(); ++q) {
    sx += ss[q];
    sy += ys[q];
    sxx += ss[q] * ss[q];
    sxy += ss[q] * ys[q];
    ma += std::atan(ys[q] / ss[q]);
  }
  if (mean)
    *mean = ma / k * 180.0 / kPi;
  return std::atan((k * sxy - sx * sy) / (k * sxx - sx * sx)) * 180.0 / kPi;
}

kelvin_reading kelvin_wake(double u, const std::vector<double> &fracs, int n, double travel,
                           double dt, double ramp, double sigma = 1.0) {
  ewave_params p = bare(n, 0.5);
  p.origin_x = p.origin_y = -0.25 * n;
  p.sponge_cells = 12.0;
  ewave w(p);
  const double depth = 0.05, x0 = p.origin_x + 10.0;
  std::vector<double> zeta_old(std::size_t(n) * n, 0.0), zeta(std::size_t(n) * n, 0.0),
      dh(std::size_t(n) * n, 0.0);
  const int steps = int(travel / u / dt + 0.5);
  double xs = x0;
  for (int s = 1; s <= steps; ++s) {
    const double t = s * dt;
    xs = x0 + u * t;
    const double tr = std::min(t / ramp, 1.0), d = depth * tr * tr * (3.0 - 2.0 * tr);
    std::fill(dh.begin(), dh.end(), 0.0);
    std::fill(zeta.begin(), zeta.end(), 0.0);
    const int i0 = std::max(0, int((xs - 6.0 * sigma - p.origin_x) / p.dx)),
              i1 = std::min(n - 1, int((xs + 6.0 * sigma - p.origin_x) / p.dx) + 1);
    for (int j = 0; j < n; ++j) {
      const double y = w.y_of(j);
      if (std::abs(y) > 6.0 * sigma)
        continue;
      for (int i = i0; i <= i1; ++i) {
        const double rx = w.x_of(i) - xs;
        zeta[std::size_t(j) * n + std::size_t(i)] =
            d * std::exp(-(rx * rx + y * y) / (2.0 * sigma * sigma));
      }
    }
    for (std::size_t c = 0; c < dh.size(); ++c)
      dh[c] = -(zeta[c] - zeta_old[c]);
    w.add_heights(dh);
    std::swap(zeta, zeta_old);
    w.step(dt);
  }
  // Rows across the track at distances s behind the source.
  kelvin_reading out;
  std::vector<double> ss, peak;
  std::vector<std::vector<double>> edge(fracs.size());
  for (double s = 16.0; s <= 44.0; s += 2.0) {
    const int i = int((xs - s - p.origin_x) / p.dx + 0.5);
    const double *col = w.height().data();
    double m = 0.0, ym = 0.0;
    for (int j = 16; j < n - 16; ++j) {
      const double v = std::abs(col[std::size_t(j) * n + std::size_t(i)]);
      if (v > m) {
        m = v;
        ym = std::abs(w.y_of(j));
      }
    }
    for (std::size_t f = 0; f < fracs.size(); ++f) {
      double y = 0.0;
      for (int j = 16; j < n - 16; ++j)
        if (std::abs(col[std::size_t(j) * n + std::size_t(i)]) > fracs[f] * m)
          y = std::max(y, std::abs(w.y_of(j)));
      edge[f].push_back(y);
    }
    ss.push_back(xs - w.x_of(i));
    peak.push_back(ym);
  }
  for (std::size_t f = 0; f < fracs.size(); ++f) {
    double mean = 0.0;
    out.fitted.push_back(fit_angle(ss, edge[f], &mean));
    out.mean.push_back(mean);
  }
  out.peak = fit_angle(ss, peak, nullptr);
  return out;
}

} // namespace

// The Kelvin wake (Phase C's exit): its half-angle is arcsin(1/3) = 19.47 degrees at any speed.
// The source travels 140 m in a 256 m window (512 cells of 0.5 m) and its depth ramps in over
// 8 s, so the rows read, 16-44 m behind it, are steady wake, well clear of the ring the start
// leaves. The edge is read at three thresholds, 30, 40 and 50 % of the row's maximum, and each
// must give the angle. Lower thresholds reach into the Airy tail beyond the cusp and read wider
// (21-24 degrees). The mean of atan(y / s) reads wider than the fitted slope, because the edge
// starts a footprint's width off the track. The row maxima are printed, not checked: for this
// Gaussian (sigma 1 m) the speeds are past the Rabaud-Moisy transition, where the brightest waves
// move inside the cusp as the speed goes up (at 4.5 m/s the 30-50 % edge itself narrows to
// 17.7-18.9 degrees). A wider footprint, for a lower Froude number, leaves cusp waves too weak
// to measure.
TEST(Ewave, AMovingSourceLeavesAKelvinWake) {
  const double kelvin = std::asin(1.0 / 3.0) * 180.0 / kPi;
  const std::vector<double> fracs = {0.3, 0.4, 0.5};
  for (double u : {2.5, 3.0}) {
    const auto t0 = std::chrono::steady_clock::now();
    const kelvin_reading r = kelvin_wake(u, fracs, 512, 140.0, 0.2, 8.0);
    const double secs =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    for (std::size_t f = 0; f < fracs.size(); ++f) {
      std::printf("U %.1f m/s, edge at %.0f %% of the row maximum: half-angle %.2f deg fitted, "
                  "%.2f deg mean of atan(y / s) (Kelvin %.2f)\n",
                  u, 100.0 * fracs[f], r.fitted[f], r.mean[f], kelvin);
      EXPECT_NEAR(r.fitted[f], kelvin, 1.5);
      EXPECT_NEAR(r.mean[f], kelvin, 3.0);
    }
    std::printf("U %.1f m/s: the row maxima at %.2f deg [%.1f s]\n", u, r.peak, secs);
  }
}

// The same steps give the same bits, and a fixed scenario gives the same digest on every platform:
// everything is det:: math and IEEE arithmetic without FMA contraction, and every input here is
// exact or computed by the library. Measured on Windows (MSVC); CI checks Linux and macOS.
TEST(Ewave, DigestIsDeterministicAndTheSameOnEveryPlatform) {
  auto scenario = [](double extra) {
    ewave_params p;
    p.n = 64;
    p.dx = 0.5;
    p.origin_x = -3.0;
    p.origin_y = 2.5;
    p.depth = 4.0;
    p.damping_k2 = 1e-3;
    p.damping = 0.01;
    p.sponge_cells = 8.0;
    p.sponge_rate = 1.5;
    ewave w(p);
    std::vector<double> mask(64 * 64, 1.0);
    for (int j = 0; j < 64; ++j)
      for (int i = 40; i < 64; ++i)
        mask[std::size_t(j) * 64 + std::size_t(i)] = i < 44 ? 0.25 * double(44 - i) - 0.25 : 0.0;
    w.set_mask(mask);
    w.add_height(5.3, 17.1, 0.2, 2.0);
    w.add_height(12.0, 20.0, -0.1 + extra, 1.5);
    for (int s = 0; s < 40; ++s)
      w.step(1.0 / 30.0);
    w.add_height(3.0, 30.0, 0.07, 1.0);
    for (int s = 0; s < 20; ++s)
      w.step(0.05);
    return w.digest();
  };
  const std::uint64_t a = scenario(0.0), b = scenario(0.0), c = scenario(1e-12);
  std::printf("ewave digest 0x%016llx\n", (unsigned long long)a);
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  EXPECT_EQ(a, 0xa4933455f6946ecaull) << std::hex << "ewave digest 0x" << a;
}

TEST(Ewave, RefusesBadInput) {
  auto make = [](auto edit) {
    ewave_params p = bare(64, 0.5);
    edit(p);
    return ewave(p);
  };
  EXPECT_THROW(make([](ewave_params &p) { p.n = 48; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.n = 2; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.n = 8192; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.dx = 0.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.dx = std::nan(""); }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.depth = 0.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.depth = -1.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.gravity = 0.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.damping = -0.1; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.beach_rate = -1.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.sponge_cells = 33.0; }), std::invalid_argument);
  EXPECT_NO_THROW(make([](ewave_params &p) { p.depth = INFINITY; }));

  ewave w(bare(64, 0.5));
  EXPECT_THROW(w.set_mask(std::vector<double>(63 * 64, 1.0)), std::invalid_argument);
  std::vector<double> m(64 * 64, 1.0);
  m[5] = 1.5;
  EXPECT_THROW(w.set_mask(m), std::invalid_argument);
  m[5] = -0.1;
  EXPECT_THROW(w.set_mask(m), std::invalid_argument);
  EXPECT_THROW(w.add_heights(std::vector<double>(10, 0.0)), std::invalid_argument);
  EXPECT_THROW(w.add_height(1.0, 1.0, 0.1, 0.0), std::invalid_argument);
  EXPECT_THROW(w.set_state(std::vector<double>(64 * 64, 0.0), std::vector<double>(5, 0.0)),
               std::invalid_argument);
  EXPECT_THROW(w.step(0.0), std::invalid_argument);
  EXPECT_THROW(w.step(-0.1), std::invalid_argument);
  EXPECT_THROW(w.step(INFINITY), std::invalid_argument);
}

// The cost of one step (two complex FFTs, the mode update, the real-space pass).
TEST(Ewave, StepCost) {
  for (int n : {64, 128, 256}) {
    ewave_params p;
    p.n = n;
    ewave w(p);
    w.add_height(0.25 * n * p.dx, 0.5 * n * p.dx, 0.1, 2.0);
    w.step(1.0 / 30.0);
    const int reps = n == 256 ? 60 : n == 128 ? 200 : 1000;
    const auto t0 = std::chrono::steady_clock::now();
    for (int s = 0; s < reps; ++s)
      w.step(1.0 / 30.0);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
        reps;
    std::printf("ewave step at N = %d: %.2f ms (one thread)\n", n, ms);
    EXPECT_GT(ms, 0.0);
  }
}
