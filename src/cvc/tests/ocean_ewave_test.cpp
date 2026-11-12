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
// it, crest for crest under no-flux walls (O8c, opt in) and inverted under zero walls (O8a, the
// default); a no-flux channel's own mode is exact, and a ring bouncing about no-flux basins keeps
// its energy for minutes; zero walls stay bounded in every basin, no-flux walls only in straight
// ones (or with a little damping), and their growth elsewhere is measured; a wall band with
// water behind it lets only a few percent through (more under no-flux walls); the sponge absorbs
// it; a beach absorbs the same share per second at any step size; shallow water carries a long
// wave at sqrt(g d); a moving source leaves a Kelvin wake at arcsin(1/3); the same steps give the
// same digest, the same on every platform; and bad parameters are refused.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ocean/ewave.h>
#include <cvc/ocean/fft.h>
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

namespace {

// The wall scenario (O8's exit): mask 0 over the half-plane x >= 0 (cells i >= 128), a Gaussian
// (sigma 0.5 m, 0.1 m high) at x = -6 m, the sponge, no damping, to t = 13 s. A wall returns the
// image of the free field u: a no-flux wall the even image u(x) + u(-dx - x), about the shore
// half a cell before the first wall cell (cell i mirrors onto 255 - i), a zero wall the odd
// image u(x) - u(dx - x), which O8a found fits best half a cell into the wall (257 - i).
ewave_params wall_params(ewave_walls walls, int ghost_cells) {
  ewave_params p = bare(256, 0.25);
  p.origin_x = p.origin_y = -32.0;
  p.sponge_cells = 16.0;
  p.walls = walls;
  p.ghost_cells = ghost_cells;
  return p;
}

constexpr int kShore = 127; // the last water column before the wall

// The free run, stepped at dt: its last field and, every step, the shore column.
struct free_run {
  std::vector<double> last;
  std::vector<std::vector<double>> shore;
};

std::vector<double> column(const ewave &w, int i) {
  const int n = w.size();
  std::vector<double> c(std::size_t(n), 0.0);
  for (int j = 0; j < n; ++j)
    c[std::size_t(j)] = w.height()[std::size_t(j) * std::size_t(n) + std::size_t(i)];
  return c;
}

free_run run_free(double dt) {
  ewave w(wall_params(ewave_walls::zero, 0));
  w.set_state(gaussian(w, -6.0, 0.0, 0.1, 0.5), std::vector<double>(256 * 256, 0.0));
  free_run out;
  for (int s = 1; s <= int(13.0 / dt + 0.5); ++s) {
    w.step(dt);
    out.shore.push_back(column(w, kShore));
  }
  out.last = w.height();
  return out;
}

struct wall_reading {
  double beyond = 0.0; // the largest |h| on wall cells without a ghost, over the run
  double odd = 0.0;    // x < 0: RMS(h - odd image) / RMS(odd image)
  double even = 0.0;   // x < 0: RMS(h - even image) / RMS(even image)
  double share = 0.0;  // the mirrored free field's RMS over the RMS of the walls' own image
  double gain = 0.0;   // the shore column over the run: sum h u / sum u^2 (even 2, odd about 0)
  double peak = 0.0;   // the shore column: max |h| / max |u| over the run
  double sign = 0.0;   // the reflection h - u against the mirrored u(-dx - x): fitted slope
  std::size_t ghosts = 0;
};

wall_reading run_wall(ewave_walls walls, int ghost_cells, double dt, const free_run &fr) {
  ewave w(wall_params(walls, ghost_cells));
  std::vector<double> mask(256 * 256, 1.0);
  for (int j = 0; j < 256; ++j)
    for (int i = 128; i < 256; ++i)
      mask[std::size_t(j) * 256 + std::size_t(i)] = 0.0;
  w.set_mask(mask);
  std::vector<char> ghost(mask.size(), 0);
  for (const ewave_ghost &g : w.ghosts())
    ghost[g.cell] = 1;
  w.set_state(gaussian(w, -6.0, 0.0, 0.1, 0.5), std::vector<double>(256 * 256, 0.0));
  wall_reading r;
  r.ghosts = w.ghosts().size();
  double hu = 0.0, uu = 0.0, hmax = 0.0, umax = 0.0;
  for (std::size_t s = 0; s < fr.shore.size(); ++s) {
    w.step(dt);
    for (std::size_t c = 0; c < mask.size(); ++c)
      if (mask[c] == 0.0 && !ghost[c])
        r.beyond = std::max(r.beyond, std::abs(w.height()[c]));
    const std::vector<double> h = column(w, kShore);
    for (std::size_t j = 0; j < h.size(); ++j) {
      const double u = fr.shore[s][j];
      hu += h[j] * u;
      uu += u * u;
      hmax = std::max(hmax, std::abs(h[j]));
      umax = std::max(umax, std::abs(u));
    }
  }
  r.gain = hu / uu;
  r.peak = hmax / umax;
  double eo = 0.0, no = 0.0, ee = 0.0, ne = 0.0, mo = 0.0, me = 0.0, rm = 0.0;
  for (int j = 16; j < 240; ++j)
    for (int i = 16; i < 128; ++i) {
      const std::size_t c = std::size_t(j) * 256 + std::size_t(i),
                        mod = std::size_t(j) * 256 + std::size_t(257 - i),
                        mev = std::size_t(j) * 256 + std::size_t(255 - i);
      const double h = w.height()[c], u = fr.last[c];
      const double odd = u - fr.last[mod], even = u + fr.last[mev];
      eo += (h - odd) * (h - odd);
      no += odd * odd;
      ee += (h - even) * (h - even);
      ne += even * even;
      mo += fr.last[mod] * fr.last[mod];
      me += fr.last[mev] * fr.last[mev];
      rm += (h - u) * fr.last[mev];
    }
  r.odd = std::sqrt(eo / no);
  r.even = std::sqrt(ee / ne);
  r.share = walls == ewave_walls::zero ? std::sqrt(mo / no) : std::sqrt(me / ne);
  r.sign = rm / me;
  return r;
}

} // namespace

// A straight wall (O8's exit): the ring from a drop at x = -6 m reflects off the wall at x >= 0.
// Under no_flux walls (the default) a crest comes back a crest: the field before the wall is the
// even image about the shore, the shore column swings to about twice the incident wave, and the
// reflection has the mirrored wave's sign. Under zero walls (O8a) it comes back inverted, as the
// odd image. Either way nothing appears on the wall cells beyond the ghosts.
//
// The spectral operator is not local (kappa = |k| couples cells about as 1 / r^3, so a missing
// half-plane of image beyond r shows as about 1 / r), so the image is cut where the ghost band
// ends: the band sets the error. With the image over the whole wall it is exact (0.01 %).
TEST(Ewave, ARingReflectsOffAStraightWall) {
  const free_run fr = run_free(0.1);
  const int band = ewave_params().ghost_cells;
  const wall_reading z = run_wall(ewave_walls::zero, band, 0.1, fr);
  const wall_reading nf = run_wall(ewave_walls::no_flux, band, 0.1, fr);
  const wall_reading all = run_wall(ewave_walls::no_flux, 256, 0.1, fr);
  for (const wall_reading *r : {&z, &nf, &all})
    std::printf("wall (%s, %zu ghosts): beyond the ghosts at most %.1e m; x < 0 against the even "
                "image %.2f %% RMS, the odd image %.1f %% (the reflection is %.0f %% of the "
                "image's RMS); shore column: gain %.3f, peak %.3f of the incident; the reflection "
                "against the mirrored wave: slope %+.3f\n",
                r == &z ? "zero" : "no_flux", r->ghosts, r->beyond, 100.0 * r->even, 100.0 * r->odd,
                100.0 * r->share, r->gain, r->peak, r->sign);
  // Zero walls: as O8a, the odd image, inverted.
  EXPECT_EQ(z.ghosts, 0u);
  EXPECT_EQ(z.beyond, 0.0);
  EXPECT_GT(z.share, 0.5);
  EXPECT_LT(z.odd, 0.25);
  EXPECT_LT(z.sign, -0.5);
  // No-flux walls: the even image, crest for crest, the shore an antinode.
  EXPECT_EQ(nf.ghosts, std::size_t(band) * 256u);
  EXPECT_EQ(nf.beyond, 0.0);
  EXPECT_GT(nf.share, 0.5);
  EXPECT_LT(nf.even, 0.08);
  EXPECT_NEAR(nf.gain, 2.0, 0.15);
  EXPECT_NEAR(nf.peak, 2.0, 0.25);
  EXPECT_NEAR(nf.sign, 1.0, 0.05);
  // The image over the whole wall (every wall cell mirrors into the window): exact.
  EXPECT_EQ(all.ghosts, 128u * 256u);
  EXPECT_LT(all.even, 1e-3);
  // The ghost band, and a larger step.
  for (int b : {1, 2, 4, 8, 32, 64}) {
    const wall_reading r = run_wall(ewave_walls::no_flux, b, 0.1, fr);
    std::printf("  no_flux, ghost band %2d cells, dt 0.1 s: even image %.2f %% RMS, gain %.3f\n", b,
                100.0 * r.even, r.gain);
  }
  const wall_reading r25 = run_wall(ewave_walls::no_flux, band, 0.25, run_free(0.25));
  std::printf("  no_flux, ghost band %d cells, dt 0.25 s: even image %.2f %% RMS, gain %.3f\n",
              band, 100.0 * r25.even, r25.gain);
  EXPECT_LT(r25.even, 0.08);
}

// A no-flux channel's own mode: water on cells 0..M - 1 and wall on M..2M - 1 along x, uniform
// along y; the window's wrap closes the channel. With every wall cell a ghost (ghost_cells = M)
// the field is the channel's even extension, and the Neumann mode cos(pi m (x + dx / 2) / L),
// L = M dx, is a mode of the periodic window: it must oscillate at omega(pi m / L) to round-off,
// at a dt far beyond any CFL limit, in deep and in shallow water.
TEST(Ewave, ANoFluxChannelModeOscillatesAtOmegaExactly) {
  struct mode_case {
    int m;
    double depth, dt;
  };
  const int n = 64, M = 32;
  const double dx = 0.5, L = M * dx, A = 0.3;
  for (const mode_case mc : {mode_case{1, 1000.0, 0.9}, mode_case{3, 1.5, 1.3},
                             mode_case{8, 1000.0, 0.37}, mode_case{5, 2.0, 2.1}}) {
    ewave_params p = bare(n, dx, mc.depth);
    p.walls = ewave_walls::no_flux;
    p.ghost_cells = M;
    ewave w(p);
    std::vector<double> mask(std::size_t(n) * n, 1.0);
    for (int j = 0; j < n; ++j)
      for (int i = M; i < n; ++i)
        mask[std::size_t(j) * n + std::size_t(i)] = 0.0;
    w.set_mask(mask);
    // Every wall cell mirrors exactly onto its image cell: wall cell M + q onto M - 1 - q.
    ASSERT_EQ(w.ghosts().size(), std::size_t(M) * n);
    bool exact = true;
    for (const ewave_ghost &g : w.ghosts()) {
      const std::size_t i = g.cell % n, j = g.cell / n;
      const std::size_t image = j * n + (2 * M - 1 - i);
      const std::size_t corners[4] = {g.corner, g.corner + 1, g.corner + n, g.corner + n + 1};
      double on = 0.0, off = 0.0;
      for (int k = 0; k < 4; ++k)
        (corners[k] == image ? on : off) += g.w[k];
      exact = exact && on == 1.0 && off == 0.0;
    }
    EXPECT_TRUE(exact) << "a straight wall's ghosts are its exact mirror cells";
    const double k = kPi * mc.m / L, om = std::sqrt(9.80665 * k * std::tanh(k * mc.depth));
    std::vector<double> f(std::size_t(n) * n), zero(f.size(), 0.0);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        f[std::size_t(j) * n + std::size_t(i)] = std::cos(k * (w.x_of(i) + 0.5 * dx));
    std::vector<double> h0(f.size());
    for (std::size_t c = 0; c < f.size(); ++c)
      h0[c] = A * f[c];
    w.set_state(h0, zero);
    double worst_h = 0.0, worst_phi = 0.0;
    const int steps = 400;
    for (int s = 1; s <= steps; ++s) {
      w.step(mc.dt);
      if (s % 50 != 0)
        continue;
      const double t = s * mc.dt, ch = A * std::cos(om * t),
                   sp = -(9.80665 / om) * A * std::sin(om * t);
      for (std::size_t c = 0; c < f.size(); ++c) {
        worst_h = std::max(worst_h, std::abs(w.height()[c] - ch * f[c]));
        worst_phi = std::max(worst_phi, std::abs(w.phi()[c] - sp * f[c]));
      }
    }
    std::printf("no-flux channel mode m = %d, depth %g, dt %g s (omega dt %.2f rad), %d steps: "
                "worst |dh| %.2e m, |dphi| %.2e m^2/s\n",
                mc.m, mc.depth, mc.dt, om * mc.dt, steps, worst_h, worst_phi);
    EXPECT_LT(worst_h, 1e-10 * A);
    EXPECT_LT(worst_phi, 1e-10 * A * 9.80665 / om);
  }
}

namespace {

// The energy of the water alone (mask above 0): 0.5 rho g sum h^2 dA plus 0.5 rho sum phi (K phi)
// dA, K the operator kappa(k) (phi_z at the surface), applied to the whole window with its ghosts.
// For a field even about a wall that is half the window's energy.
double water_energy(const ewave &w, const fft_plan &plan) {
  const ewave_params &p = w.params();
  const int n = p.n;
  const std::size_t cells = std::size_t(n) * std::size_t(n);
  std::vector<double> z(cells * 2, 0.0);
  for (std::size_t c = 0; c < cells; ++c)
    z[c * 2] = w.phi()[c];
  plan.run_2d(z.data(), -1);
  const double dk = 2.0 * kPi / (n * p.dx), inv = 1.0 / (double(n) * double(n));
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double kx = dk * (i < n / 2 ? i : i - n), ky = dk * (j < n / 2 ? j : j - n);
      const double k = std::sqrt(kx * kx + ky * ky);
      const double kap = k * std::tanh(std::min(k * p.depth, 20.0)) * inv;
      const std::size_t c = std::size_t(j) * std::size_t(n) + std::size_t(i);
      z[c * 2] *= kap;
      z[c * 2 + 1] *= kap;
    }
  plan.run_2d(z.data(), +1);
  double pe = 0.0, ke = 0.0;
  for (std::size_t c = 0; c < cells; ++c)
    if (w.mask()[c] > 0.0) {
      pe += w.height()[c] * w.height()[c];
      ke += w.phi()[c] * z[c * 2];
    }
  return 0.5 * p.density * (p.gravity * pe + ke) * p.dx * p.dx;
}

} // namespace

// Nearly lossless walls: with no_flux walls but no damping, beach or sponge, a smooth ring
// bouncing about a closed basin for 300 s (dozens of reflections) keeps its energy, for straight
// walls (with the default band and with the whole wall mirrored), 45-degree staircase walls and a
// disc: staircase and curved walls are first order and lose a few percent. A smooth ring over
// 300 s does not show no-flux walls' slow growth (WallsStayBounded does): this measures the
// loss, not stability. energy() (the water's, ghosts left out) is checked against water_energy.
TEST(Ewave, NoFluxWallsAreNearlyLossless) {
  struct basin {
    const char *name;
    bool (*wall)(int i, int j);
    double x, y; // the drop (m)
    int ghost_cells;
  };
  const int band = ewave_params().ghost_cells;
  const basin basins[] = {
      {"straight channel", [](int i, int) { return i < 24 || i >= 104; }, 12.0, 16.0, band},
      {"straight channel, whole wall mirrored", [](int i, int) { return i < 24 || i >= 104; }, 12.0,
       16.0, 128},
      {"45-degree channel",
       [](int i, int j) {
         const int d = (i - j + 128) % 128;
         return d < 24 || d >= 104;
       },
       20.0, 4.0, band},
      {"disc",
       [](int i, int j) { return (i - 63.5) * (i - 63.5) + (j - 63.5) * (j - 63.5) > 48.0 * 48.0; },
       12.5, 17.5, band},
  };
  for (const basin &b : basins) {
    ewave_params p = bare(128, 0.25);
    p.walls = ewave_walls::no_flux;
    p.ghost_cells = b.ghost_cells;
    ewave w(p);
    const fft_plan plan(128);
    std::vector<double> mask(128 * 128, 1.0);
    for (int j = 0; j < 128; ++j)
      for (int i = 0; i < 128; ++i)
        if (b.wall(i, j))
          mask[std::size_t(j) * 128 + std::size_t(i)] = 0.0;
    w.set_mask(mask);
    w.add_height(b.x, b.y, 0.1, 1.5);
    const double e0 = water_energy(w, plan);
    double lo = 1.0, hi = 1.0, agree = std::abs(w.energy() / e0 - 1.0);
    const int steps = 3000;
    for (int s = 1; s <= steps; ++s) {
      w.step(0.1);
      if (s % 20 != 0)
        continue;
      const double we = water_energy(w, plan), e = we / e0;
      agree = std::max(agree, std::abs(w.energy() - we) / e0);
      lo = std::min(lo, e);
      hi = std::max(hi, e);
    }
    const double last = water_energy(w, plan) / e0;
    std::printf("nearly lossless, %s (%zu ghosts): the water's energy over %d steps of 0.1 s "
                "stays in %.4f..%.4f of its start, ends at %.4f; energy() agrees to %.1e\n",
                b.name, w.ghosts().size(), steps, lo, hi, last, agree);
    EXPECT_GT(lo, 0.85); // staircase and curved walls are first order: a few percent over 300 s
    EXPECT_LT(hi, 1.0 + 1e-3);
    EXPECT_LT(last, 1.0 + 1e-3);
    EXPECT_LT(agree, 1e-9);
  }
}

namespace {

// Broadband noise, uniform in [-a, a] from a fixed xorshift (the same on every platform), on the
// cells whose mask is above 0.
std::vector<double> noise(const std::vector<double> &mask, double a) {
  std::uint64_t x = 0x9e3779b97f4a7c15ull;
  std::vector<double> h(mask.size(), 0.0);
  for (std::size_t c = 0; c < mask.size(); ++c) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    if (mask[c] > 0.0)
      h[c] = a * (2.0 * double(x >> 11) / 9007199254740992.0 - 1.0);
  }
  return h;
}

struct bounded {
  double hi = 1.0, last = 1.0; // the water's energy over its start: the largest seen, the last
};

// A closed basin on a 64^2 window of 0.25 m cells, no sponge, from broadband noise in h: the
// water's energy (energy(), every 100 steps) over its start.
bounded run_basin(bool (*wall)(int i, int j), ewave_walls walls, double dt, int steps,
                  double damping) {
  ewave_params p = bare(64, 0.25);
  p.walls = walls;
  p.damping = damping;
  ewave w(p);
  std::vector<double> mask(64 * 64, 1.0);
  for (int j = 0; j < 64; ++j)
    for (int i = 0; i < 64; ++i)
      if (wall(i, j))
        mask[std::size_t(j) * 64 + std::size_t(i)] = 0.0;
  w.set_mask(mask);
  w.set_state(noise(mask, 0.01), std::vector<double>(mask.size(), 0.0));
  w.add_height(0.0, 0.0, 0.0, 1.0); // refills the ghosts from the water
  const double e0 = w.energy();
  bounded b;
  for (int s = 1; s <= steps; ++s) {
    w.step(dt);
    if (s % 100 == 0) {
      b.last = w.energy() / e0;
      if (!(b.last <= 1e30)) // overflowed or not a number: stop counting
        b.last = 1e30;
      b.hi = std::max(b.hi, b.last);
      if (b.last >= 1e30)
        break;
    }
  }
  return b;
}

bool disc(int i, int j) { return (i - 31.5) * (i - 31.5) + (j - 31.5) * (j - 31.5) > 24.0 * 24.0; }

} // namespace

// Stability (the review of O8c): broadband noise in closed basins, no sponge, the reviewer's
// shapes at 64^2 (a disc of water 12 m across, the disc with an island, the disc with a
// breakwater four cells thick, a straight channel 5 m wide whose walls are thick enough that the
// two faces' images do not meet, and a wall two cells thick across the whole window), at dt 0.1 s
// and 0.25 s (omega_max dt 1.3 and 3.3, omega at the Nyquist wavenumber along a diagonal) and at
// 1/30 s (0.44).
//   - Zero walls (the default) never let the water's energy rise above its start.
//   - No-flux walls do: overwriting the ghosts with the image is an oblique projection, and the
//     water's update it gives is not symmetric (where the image is not an exact mirror, or is
//     cut off at the band). The growth is measured and printed, not asserted: fastest at large
//     omega_max dt, in curved basins and behind thin walls. At 1/30 s the straight channel and
//     the thin wall stay level, which is asserted.
//   - A small global damping, 0.004 /s, holds every basin at 1/30 s under no-flux walls (about
//     twice the fastest growth measured there).
// O8c's investigation (OCEAN-AND-WEATHER-ROADMAP.md) tried making the fill non-amplifying -
// tapering the image, capping each water cell's total weight, no ghosts near a medial axis,
// k^2 damping, and projecting orthogonally onto the image (its transpose folded back) - and none
// held every basin without destroying the straight wall's reflection, so no-flux is opt in.
TEST(Ewave, WallsStayBounded) {
  struct basin {
    const char *name;
    bool (*wall)(int i, int j);
    bool level_at_frame_rate; // asserted level under no_flux, undamped, at 1/30 s
  };
  const basin basins[] = {
      {"disc", disc, false},
      {"disc with an island",
       [](int i, int j) {
         return disc(i, j) || (i - 31.5) * (i - 31.5) + (j - 20.0) * (j - 20.0) < 9.0;
       },
       false},
      {"disc with a breakwater",
       [](int i, int j) { return disc(i, j) || (i >= 30 && i < 34 && j >= 14 && j < 48); }, false},
      {"straight channel", [](int i, int) { return i < 22 || i >= 42; }, true},
      {"thin wall", [](int i, int) { return i >= 31 && i < 33; }, true},
  };
  struct stepping {
    double dt;
    int steps;
  };
  const stepping steppings[] = {{0.1, 8000}, {0.25, 4000}};
  for (const basin &b : basins)
    for (const stepping &st : steppings) {
      const bounded z = run_basin(b.wall, ewave_walls::zero, st.dt, st.steps, 0.0);
      const bounded nf = run_basin(b.wall, ewave_walls::no_flux, st.dt, st.steps, 0.0);
      std::printf("bounded, %s, dt %.2f s, %.0f s: zero walls at most %.4f of the start (ends "
                  "%.4f); no_flux at most %.4g (ends %.4g)\n",
                  b.name, st.dt, st.dt * st.steps, z.hi, z.last, nf.hi, nf.last);
      EXPECT_LE(z.hi, 1.0 + 1e-6) << b.name << " under zero walls at dt " << st.dt;
    }
  for (const basin &b : basins) {
    const bounded nf = run_basin(b.wall, ewave_walls::no_flux, 1.0 / 30.0, 12000, 0.0);
    const bounded nfd = run_basin(b.wall, ewave_walls::no_flux, 1.0 / 30.0, 12000, 0.004);
    std::printf("bounded, %s, dt 1/30 s, %.0f s, no_flux: undamped at most %.4g (ends %.4g); "
                "damping 0.004 /s at most %.4f (ends %.4f)\n",
                b.name, 12000 / 30.0, nf.hi, nf.last, nfd.hi, nfd.last);
    EXPECT_LE(nfd.hi, 1.0 + 1e-6) << b.name << " under no_flux walls with damping 0.004";
    if (b.level_at_frame_rate)
      EXPECT_LT(nf.hi, 1.01) << b.name << " under no_flux walls, undamped, at 1/30 s";
  }
}

namespace {

// A wall band of mask 0 over 0 <= x < thick with open water behind it, hit by the ring of a drop
// at x = -6 m. Returns the largest |h| behind the band (x >= thick, out of the sponge) over the
// run, over the free run's largest |h| just before the wall (x in [-0.5, 0)).
double wall_leak(double thick, double sponge_cells, double sponge_rate, ewave_walls walls,
                 int n = 256) {
  ewave_params p = bare(n, 0.25);
  p.origin_x = p.origin_y = -0.125 * n;
  p.sponge_cells = sponge_cells;
  p.sponge_rate = sponge_rate;
  p.walls = walls;
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
// surface with water under it, and the spectral operator couples across it. Under zero walls a
// 12-cell (3 m) band lets through a few percent of the incident amplitude, a 6 m band less. Behind
// a thick band most of what is left came round the periodic window through the sponge: in a
// window twice as wide (n 512) a 6 m band reads 1.4 % instead of 2.8 %, and a 12 m band 1.0 %
// instead of 3.1 %. Under no_flux walls each face mirrors its own water.
TEST(Ewave, AThinWallLeaksLittle) {
  const double leak3 = wall_leak(3.0, 16.0, 2.0, ewave_walls::zero),
               leak6 = wall_leak(6.0, 16.0, 2.0, ewave_walls::zero);
  std::printf("zero walls: band 3 m (12 cells): %.2f %% of the incident amplitude behind it; 6 m: "
              "%.2f %%\n",
              100.0 * leak3, 100.0 * leak6);
  EXPECT_LT(leak3, 0.06);
  EXPECT_LT(leak6, leak3);
  const double nf3 = wall_leak(3.0, 16.0, 2.0, ewave_walls::no_flux),
               nf6 = wall_leak(6.0, 16.0, 2.0, ewave_walls::no_flux),
               nf12 = wall_leak(12.0, 16.0, 2.0, ewave_walls::no_flux);
  std::printf("no_flux walls: band 3 m: %.2f %%; 6 m: %.2f %%; 12 m: %.2f %%\n", 100.0 * nf3,
              100.0 * nf6, 100.0 * nf12);
  EXPECT_LT(nf3, 0.2);
  EXPECT_LT(nf6, 0.06);
  EXPECT_LT(nf12, 0.06);
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
  auto scenario = [](double extra, ewave_walls walls) {
    ewave_params p;
    p.walls = walls;
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
  // Zero walls: O8a's scenario and pin, bit for bit.
  const std::uint64_t a = scenario(0.0, ewave_walls::zero), b = scenario(0.0, ewave_walls::zero),
                      c = scenario(1e-12, ewave_walls::zero);
  std::printf("ewave digest (zero walls) 0x%016llx\n", (unsigned long long)a);
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
  EXPECT_EQ(a, 0xa4933455f6946ecaull) << std::hex << "ewave digest 0x" << a;
  // No-flux walls: the land at i >= 44 holds ghosts.
  const std::uint64_t d = scenario(0.0, ewave_walls::no_flux),
                      e = scenario(0.0, ewave_walls::no_flux),
                      f = scenario(1e-12, ewave_walls::no_flux);
  std::printf("ewave digest (no_flux walls) 0x%016llx\n", (unsigned long long)d);
  EXPECT_EQ(d, e);
  EXPECT_NE(d, f);
  EXPECT_NE(d, a);
  EXPECT_EQ(d, 0xfc278a415ad67d3full) << std::hex << "ewave digest 0x" << d;
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
  EXPECT_THROW(make([](ewave_params &p) { p.ghost_cells = -1; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.ghost_cells = 65; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.walls = ewave_walls(7); }), std::invalid_argument);
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
  // set_mask with no-flux walls finds each wall cell's nearest shore within the ghost band: a
  // 256^2 window, its right half land behind a wavy coast, for the default band and the whole
  // wall. Then a step with the ghosts.
  for (int band : {ewave_params().ghost_cells, 256}) {
    ewave_params p;
    p.walls = ewave_walls::no_flux;
    p.ghost_cells = band;
    ewave w(p);
    std::vector<double> mask(256 * 256, 1.0);
    for (int j = 0; j < 256; ++j)
      for (int i = 0; i < 256; ++i)
        if (i >= 128 + int(12.0 * std::sin(j * 0.05)))
          mask[std::size_t(j) * 256 + std::size_t(i)] = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    w.set_mask(mask);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    w.add_height(40.0, 64.0, 0.1, 2.0);
    const auto t1 = std::chrono::steady_clock::now();
    for (int s = 0; s < 20; ++s)
      w.step(1.0 / 30.0);
    const double step =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() /
        20.0;
    std::printf("no-flux set_mask at N = 256, ghost band %d: %.1f ms for %zu ghosts; a step %.2f "
                "ms\n",
                band, ms, w.ghosts().size(), step);
    EXPECT_GT(w.ghosts().size(), 0u);
  }
}
