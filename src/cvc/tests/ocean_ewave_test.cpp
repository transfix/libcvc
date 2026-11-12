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
// conserved; a drop makes a round ring whose envelope moves at the group velocity; no-flux walls
// (O8c, the default: the symmetric step) are the exact Neumann operator on a straight channel
// and keep a mode's frequency exactly; a ring reflects off a wall crest for crest under no-flux
// walls and inverted under zero walls (O8a); broadband noise in closed basins cannot grow under
// either (the symmetric step's modified energy is conserved to round-off), nor under a dt that
// changes every step (the substep is fixed per window); no water cell is read by more than
// ewave_max_readers ghosts; a ring bouncing about no-flux basins loses nothing; a wall band with
// water behind it lets a few percent through, and one a cell thick no more under no-flux walls
// than under zero walls; the sponge absorbs it; a beach
// absorbs the same share per second at any step size; shallow water carries a long wave at
// sqrt(g d); a moving source leaves a Kelvin wake at arcsin(1/3); the same steps give the same
// digest, the same on every platform; and bad parameters are refused.

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

// The symmetric step's operators, built here from the window's public ghost table and mass (the
// test's own FFT and its own kappa, from std::tanh), to check the library against.
struct sym_ops {
  const ewave &w;
  fft_plan plan;
  std::vector<double> kappa, kappa_t; // per mode: kappa, and kappa~ for the substep dts
  double dts = 0.0;

  explicit sym_ops(const ewave &w_) : w(w_), plan(w_.size()) {
    const ewave_params &p = w.params();
    const int n = p.n;
    dts = w.substep();
    const double dk = 2.0 * kPi / (n * p.dx);
    kappa.resize(std::size_t(n) * n);
    kappa_t.resize(kappa.size());
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        const double kx = dk * (i < n / 2 ? i : i - n), ky = dk * (j < n / 2 ? j : j - n);
        const double k = std::sqrt(kx * kx + ky * ky);
        const double kap = k > 0.0 ? k * std::tanh(std::min(k * p.depth, 20.0)) : 0.0;
        const double sn = std::sin(0.5 * std::sqrt(p.gravity * kap) * dts);
        const std::size_t c = std::size_t(j) * n + std::size_t(i);
        kappa[c] = kap;
        kappa_t[c] = 4.0 * sn * sn / (p.gravity * dts * dts);
      }
  }

  // R f: f on the water, each ghost its weighted corners, 0 on other walls.
  std::vector<double> extend(const std::vector<double> &f) const {
    const std::size_t n = std::size_t(w.size());
    std::vector<double> r(f.size(), 0.0);
    for (std::size_t c = 0; c < f.size(); ++c)
      if (w.mask()[c] > 0.0)
        r[c] = f[c];
    for (const ewave_ghost &g : w.ghosts()) {
      const std::size_t q[4] = {g.corner, g.corner + 1, g.corner + n, g.corner + n + 1};
      double v = 0.0;
      for (int k = 0; k < 4; ++k)
        v += g.w[k] * (w.mask()[q[k]] > 0.0 ? f[q[k]] : 0.0);
      r[g.cell] = v;
    }
    return r;
  }
  // R^T G R f, G the multiplier mult.
  std::vector<double> s(const std::vector<double> &mult, const std::vector<double> &f) const {
    const std::size_t n = std::size_t(w.size()), cells = f.size();
    const std::vector<double> r = extend(f);
    std::vector<double> z(cells * 2, 0.0);
    for (std::size_t c = 0; c < cells; ++c)
      z[c * 2] = r[c];
    plan.run_2d(z.data(), -1);
    for (std::size_t c = 0; c < cells; ++c) {
      z[c * 2] *= mult[c] / double(cells);
      z[c * 2 + 1] *= mult[c] / double(cells);
    }
    plan.run_2d(z.data(), +1);
    std::vector<double> out(cells, 0.0);
    for (std::size_t c = 0; c < cells; ++c)
      if (w.mask()[c] > 0.0)
        out[c] = z[c * 2];
    for (const ewave_ghost &g : w.ghosts()) {
      const std::size_t q[4] = {g.corner, g.corner + 1, g.corner + n, g.corner + n + 1};
      for (int k = 0; k < 4; ++k)
        if (g.w[k] != 0.0)
          out[q[k]] += g.w[k] * z[std::size_t(g.cell) * 2];
    }
    return out;
  }
  double dot(const std::vector<double> &a, const std::vector<double> &b) const {
    double s = 0.0;
    for (std::size_t c = 0; c < a.size(); ++c)
      if (w.mask()[c] > 0.0)
        s += a[c] * b[c];
    return s;
  }
  double hmh() const {
    double s = 0.0;
    for (std::size_t c = 0; c < w.height().size(); ++c)
      if (w.mask()[c] > 0.0)
        s += w.mass()[c] * w.height()[c] * w.height()[c];
    return s;
  }
  // H = 1/2 (g h.M.h + phi.S.phi) and the modified energy the step conserves,
  // 1/2 (g h.M.h - (g dts / 2)^2 h.S~.h + phi.S~.phi), in m^4/s^2 units (no rho dA).
  double energy() const {
    return 0.5 * (w.params().gravity * hmh() + dot(w.phi(), s(kappa, w.phi())));
  }
  double modified() const {
    const double g = w.params().gravity, a = 0.5 * g * dts;
    return 0.5 * (g * hmh() - a * a * dot(w.height(), s(kappa_t, w.height())) +
                  dot(w.phi(), s(kappa_t, w.phi())));
  }
};

std::vector<double> mask_of(int n, bool (*wall)(int i, int j)) {
  std::vector<double> mask(std::size_t(n) * n, 1.0);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      if (wall(i, j))
        mask[std::size_t(j) * n + std::size_t(i)] = 0.0;
  return mask;
}

} // namespace

// The symmetric step's operator is the exact Neumann operator where a straight wall is mirrored
// cell for cell. A channel: a window of 2M = 64 cells along x, water on 0..M - 1, wall on
// M..2M - 1 (the window's wrap closes it), every wall cell a ghost (ghost_cells = M). Each ghost
// is its exact mirror cell with weight 1, so M = 2 on the water and M^-1 R^T G R is the even
// extension's operator: applied to the Neumann mode f = cos(pi m (x + dx / 2) / L) cos(2 pi q y /
// (n dx)), L = M dx, it gives kappa(k) f to round-off (built here from ghosts() and mass()).
//
// The step itself is Stormer-Verlet on that operator with kappa~ = 4 sin^2(omega dt_s / 2) /
// (g dt_s^2) in place of kappa, whose discrete solution from h = A f, phi = 0 is exactly
//   h_n = A cos(omega t_n) f,  phi_n = -A (g dt_s / 2) cot(omega dt_s / 2) sin(omega t_n) f
// (cos(omega~ dt_s) = 1 - g kappa~ dt_s^2 / 2 = cos(omega dt_s): the mode keeps omega's
// frequency at any substep). The library's steps must match it to round-off, over 400 steps of
// one to six substeps (steps of a whole number of substeps), deep and shallow.
TEST(Ewave, ANoFluxChannelIsTheExactNeumannOperator) {
  struct mode_case {
    int m, q;
    double depth, dts;
    int per_step;
  };
  const int n = 64, M = 32;
  const double dx = 0.5, L = M * dx, A = 0.3, g = 9.80665;
  for (const mode_case mc :
       {mode_case{1, 0, 1000.0, 0.18, 5}, mode_case{3, 0, 1.5, 0.15, 1},
        mode_case{8, 2, 1000.0, 0.185, 2}, mode_case{5, 1, 2.0, 1.1 / 6.0, 6}}) {
    ewave_params p = bare(n, dx, mc.depth);
    p.ghost_cells = M;
    p.substep = mc.dts;
    ewave w(p);
    ASSERT_EQ(w.substep(), mc.dts) << "below the stable limit, the substep is the one asked for";
    const double dt = mc.dts * mc.per_step;
    std::vector<double> mask(std::size_t(n) * n, 1.0);
    for (int j = 0; j < n; ++j)
      for (int i = M; i < n; ++i)
        mask[std::size_t(j) * n + std::size_t(i)] = 0.0;
    w.set_mask(mask);
    ASSERT_EQ(w.ghosts().size(), std::size_t(M) * n);
    bool exact = true, two = true;
    for (const ewave_ghost &gh : w.ghosts()) {
      const std::size_t i = gh.cell % n, j = gh.cell / n;
      const std::size_t image = j * n + (2 * M - 1 - i);
      const std::size_t corners[4] = {gh.corner, gh.corner + 1, gh.corner + n, gh.corner + n + 1};
      double on = 0.0, off = 0.0;
      for (int k = 0; k < 4; ++k)
        (corners[k] == image ? on : off) += gh.w[k];
      exact = exact && on == 1.0 && off == 0.0;
    }
    for (std::size_t c = 0; c < mask.size(); ++c)
      two = two && w.mass()[c] == (mask[c] > 0.0 ? 2.0 : 0.0);
    EXPECT_TRUE(exact) << "a straight wall's ghosts are its exact mirror cells";
    EXPECT_TRUE(two) << "M = 2 on the water of a channel mirrored cell for cell";
    const double kx = kPi * mc.m / L, ky = 2.0 * kPi * mc.q / (n * dx);
    const double k = std::sqrt(kx * kx + ky * ky), kap = k * std::tanh(k * mc.depth),
                 om = std::sqrt(g * kap);
    std::vector<double> f(std::size_t(n) * n, 0.0), zero(f.size(), 0.0), h0(f.size(), 0.0);
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < M; ++i) {
        const std::size_t c = std::size_t(j) * n + std::size_t(i);
        f[c] = std::cos(kx * (w.x_of(i) + 0.5 * dx)) * std::cos(ky * w.y_of(j));
        h0[c] = A * f[c];
      }
    // The operator.
    const sym_ops ops(w);
    const std::vector<double> sf = ops.s(ops.kappa, f);
    double worst_op = 0.0;
    for (std::size_t c = 0; c < f.size(); ++c)
      if (mask[c] > 0.0)
        worst_op = std::max(worst_op, std::abs(sf[c] / w.mass()[c] - kap * f[c]));
    // The steps: each a whole number of substeps, nothing carried.
    w.set_state(h0, zero);
    const int sub = w.substeps(dt);
    EXPECT_EQ(sub, mc.per_step);
    const double dts = mc.dts, amp = A * (0.5 * g * dts) / std::tan(0.5 * om * dts);
    double worst_h = 0.0, worst_phi = 0.0, worst_pending = 0.0;
    long long run = 0;
    const int steps = 400;
    for (int s = 1; s <= steps; ++s) {
      run += w.substeps(dt);
      w.step(dt);
      worst_pending = std::max(worst_pending, w.pending());
      if (s % 50 != 0)
        continue;
      const double t = double(run) * dts, ch = A * std::cos(om * t), sp = -amp * std::sin(om * t);
      for (std::size_t c = 0; c < f.size(); ++c) {
        worst_h = std::max(worst_h, std::abs(w.height()[c] - ch * f[c]));
        worst_phi = std::max(worst_phi, std::abs(w.phi()[c] - sp * f[c]));
      }
    }
    std::printf("no-flux channel, mode (%d, %d), depth %g: |M^-1 S f - kappa f| at most %.1e "
                "(kappa %.3f /m); dt %g s in %d substeps (omega dt_s %.2f rad, phi's amplitude "
                "%.4f of the exact), %d steps: worst |dh| %.2e m, |dphi| %.2e m^2/s\n",
                mc.m, mc.q, mc.depth, worst_op, kap, dt, sub, om * dts, amp / (A * g / om), steps,
                worst_h, worst_phi);
    EXPECT_EQ(run, (long long)steps * mc.per_step);
    EXPECT_LT(worst_pending, 1e-9 * dts);
    EXPECT_LT(worst_op, 1e-12 * kap);
    EXPECT_LT(worst_h, 1e-10 * A);
    EXPECT_LT(worst_phi, 1e-10 * A * g / om);
  }
}

namespace {

// The wall scenario (O8's exit): mask 0 over the half-plane x >= 0 (cells i >= 128), a Gaussian
// (sigma 0.5 m, 0.1 m high) at x = -6 m, the sponge, no damping, to t = 13 s. A wall returns the
// image of the free field u: a no-flux wall the even image u(x) + u(-dx - x), about the shore
// half a cell before the first wall cell (cell i mirrors onto 255 - i), a zero wall the odd
// image u(x) - u(dx - x), which O8a found fits best half a cell into the wall (257 - i).
ewave_params wall_params(ewave_walls walls, int ghost_cells, double sponge = 16.0) {
  ewave_params p = bare(256, 0.25);
  p.origin_x = p.origin_y = -32.0;
  p.sponge_cells = sponge;
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

free_run run_free(double dt, double sponge = 16.0) {
  ewave w(wall_params(ewave_walls::zero, 0, sponge));
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
  double beyond = 0.0; // the largest |h| on wall cells over the run
  double odd = 0.0;    // x < 0: RMS(h - odd image) / RMS(odd image)
  double even = 0.0;   // x < 0: RMS(h - even image) / RMS(even image)
  double share = 0.0;  // the mirrored free field's RMS over the RMS of the walls' own image
  double gain = 0.0;   // the shore column over the run: sum h u / sum u^2 (even 2, odd about 0)
  double peak = 0.0;   // the shore column: max |h| / max |u| over the run
  double sign = 0.0;   // the reflection h - u against the mirrored u(-dx - x): fitted slope
  std::size_t ghosts = 0;
  int substeps = 1;
};

// The substep: dt in the fewest equal parts of at most 0.125 s (1 at dt 0.1 s, 2 at 0.25 s, 4 at
// 0.5 s), so that every step runs whole substeps and the run ends at the free run's time.
wall_reading run_wall(ewave_walls walls, int ghost_cells, double dt, const free_run &fr,
                      double sponge = 16.0) {
  ewave_params p = wall_params(walls, ghost_cells, sponge);
  p.substep = dt / std::ceil(dt / 0.125);
  ewave w(p);
  std::vector<double> mask(256 * 256, 1.0);
  for (int j = 0; j < 256; ++j)
    for (int i = 128; i < 256; ++i)
      mask[std::size_t(j) * 256 + std::size_t(i)] = 0.0;
  w.set_mask(mask);
  w.set_state(gaussian(w, -6.0, 0.0, 0.1, 0.5), std::vector<double>(256 * 256, 0.0));
  wall_reading r;
  r.ghosts = w.ghosts().size();
  r.substeps = w.substeps(dt);
  double hu = 0.0, uu = 0.0, hmax = 0.0, umax = 0.0;
  for (std::size_t s = 0; s < fr.shore.size(); ++s) {
    w.step(dt);
    for (std::size_t c = 0; c < mask.size(); ++c)
      if (mask[c] == 0.0)
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
// odd image. Either way the wall cells hold nothing.
//
// With the default band every wall cell mirrors into the water (the wall is 128 cells deep and its
// image fits), and the symmetric step is the even extension's exact step: without the sponge the
// field is the even image to round-off. With it, the 2 % left is the sponge's form: on the
// symmetric step it damps h alone at twice the rate. A band that cuts the image inside the wall
// costs more: the spectral operator is not local, so the waves longer than the band see the cut
// (an extension stepping down to 0) more than the wall, and the symmetric fold carries G's
// response to that step into the water at the band's mirror distance. The taper (ewave.h) softens
// it; the band sweep below measures what is left.
TEST(Ewave, ARingReflectsOffAStraightWall) {
  const free_run fr = run_free(0.1);
  const int band = ewave_params().ghost_cells;
  const wall_reading z = run_wall(ewave_walls::zero, band, 0.1, fr);
  const wall_reading nf = run_wall(ewave_walls::no_flux, band, 0.1, fr);
  const wall_reading bare_ = run_wall(ewave_walls::no_flux, band, 0.1, run_free(0.1, 0.0), 0.0);
  for (const wall_reading *r : {&z, &nf, &bare_})
    std::printf("wall (%s%s, %zu ghosts): on the wall at most %.1e m; x < 0 against the even "
                "image %.2e RMS, the odd image %.1f %% (the reflection is %.0f %% of the "
                "image's RMS); shore column: gain %.3f, peak %.3f of the incident; the reflection "
                "against the mirrored wave: slope %+.3f\n",
                r == &z ? "zero" : "no_flux", r == &bare_ ? ", no sponge" : "", r->ghosts,
                r->beyond, r->even, 100.0 * r->odd, 100.0 * r->share, r->gain, r->peak, r->sign);
  // Zero walls: as O8a, the odd image, inverted.
  EXPECT_EQ(z.ghosts, 0u);
  EXPECT_EQ(z.beyond, 0.0);
  EXPECT_GT(z.share, 0.5);
  EXPECT_LT(z.odd, 0.25);
  EXPECT_LT(z.sign, -0.5);
  // No-flux walls: the even image, crest for crest, the shore an antinode.
  EXPECT_EQ(nf.ghosts, 128u * 256u);
  EXPECT_EQ(nf.beyond, 0.0);
  EXPECT_GT(nf.share, 0.5);
  EXPECT_LT(nf.even, 0.03);
  EXPECT_NEAR(nf.gain, 2.0, 0.15);
  EXPECT_NEAR(nf.peak, 2.0, 0.25);
  EXPECT_NEAR(nf.sign, 1.0, 0.05);
  // Without the sponge: the even extension's exact step.
  EXPECT_LT(bare_.even, 1e-9);
  // A band that cuts the image inside the wall, and larger steps.
  for (int b : {16, 32, 64, 96}) {
    const wall_reading r = run_wall(ewave_walls::no_flux, b, 0.1, fr);
    std::printf("  no_flux, ghost band %2d cells, dt 0.1 s: even image %.2f %% RMS, gain %.3f, "
                "slope %+.3f\n",
                b, 100.0 * r.even, r.gain, r.sign);
  }
  for (double dt : {0.25, 0.5}) {
    const wall_reading r = run_wall(ewave_walls::no_flux, band, dt, run_free(dt));
    std::printf("  no_flux, dt %.2f s (%d substeps): even image %.2f %% RMS, gain %.3f, slope "
                "%+.3f\n",
                dt, r.substeps, 100.0 * r.even, r.gain, r.sign);
    EXPECT_LT(r.even, 0.03);
    EXPECT_NEAR(r.sign, 1.0, 0.05);
  }
}

namespace {

bool disc(int i, int j) { return (i - 31.5) * (i - 31.5) + (j - 31.5) * (j - 31.5) > 24.0 * 24.0; }
bool disc_island(int i, int j) {
  return disc(i, j) || (i - 31.5) * (i - 31.5) + (j - 20.0) * (j - 20.0) < 9.0;
}
bool disc_breakwater(int i, int j) {
  return disc(i, j) || (i >= 30 && i < 34 && j >= 14 && j < 48);
}
bool straight_channel(int i, int) { return i < 22 || i >= 42; }
bool diagonal_channel(int i, int j) {
  const int d = (i - j + 64) % 64;
  return d < 12 || d >= 52;
}
bool thin_wall(int i, int) { return i >= 31 && i < 33; }
bool thin_segment(int i, int j) { return i >= 31 && i < 33 && j >= 16 && j < 48; }

struct basin {
  const char *name;
  bool (*wall)(int i, int j);
};
const basin kBasins[] = {
    {"disc", disc},
    {"disc with an island", disc_island},
    {"disc with a breakwater", disc_breakwater},
    {"straight channel", straight_channel},
    {"45-degree channel", diagonal_channel},
    {"2-cell wall across the window", thin_wall},
    {"2-cell wall in open water", thin_segment},
};

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
  double hi = 1.0, lo = 1.0, last = 1.0; // energy() over its start: largest, smallest, last
  double early = 0.0, late = 0.0;        // its mean over the first and the last quarter
  double modified = 0.0;                 // the modified energy's largest relative change
  long long substeps = 0;                // substeps run
  double pending = 0.0;                  // the largest pending() over the substep
};

// How a basin run steps: cell size, the window's substep, and the dt of each step: lo always
// (lo == hi), lo and hi in turn (alternate), or uniform in [lo, hi] from a fixed xorshift.
struct stepping {
  const char *name;
  double dx, substep, lo, hi;
  bool alternate = false;
};

// omega at the corner wavenumber of a window of dx cells (deep water): the fastest mode.
double omega_max(double dx) { return std::sqrt(9.80665 * std::sqrt(2.0) * kPi / dx); }

// A closed basin on a 64^2 window, from broadband noise in h: energy() every `every` steps over
// its start, and (with walls) the modified energy. Without `damped`, no damping, beach or sponge;
// with it, damping_k2 0.05 m^2/s, damping 0.01 /s, the sponge (8 cells, 2 /s) and a beach (mask
// 0.6) on every water cell beside a wall.
bounded run_basin(bool (*wall)(int i, int j), ewave_walls walls, const stepping &st, int steps,
                  int every = 100, bool damped = false) {
  ewave_params p = bare(64, st.dx);
  p.walls = walls;
  p.substep = st.substep;
  if (damped) {
    p.damping_k2 = 0.05;
    p.damping = 0.01;
    p.sponge_cells = 8.0;
  }
  ewave w(p);
  std::vector<double> mask = mask_of(64, wall);
  if (damped) {
    const std::vector<double> m0 = mask;
    for (int j = 0; j < 64; ++j)
      for (int i = 0; i < 64; ++i) {
        const std::size_t c = std::size_t(j) * 64 + std::size_t(i);
        if (m0[c] == 0.0)
          continue;
        const int ni[4] = {i - 1, i + 1, i, i}, nj[4] = {j, j, j - 1, j + 1};
        for (int e = 0; e < 4; ++e)
          if (ni[e] >= 0 && nj[e] >= 0 && ni[e] < 64 && nj[e] < 64 &&
              m0[std::size_t(nj[e]) * 64 + std::size_t(ni[e])] == 0.0)
            mask[c] = 0.6;
      }
  }
  w.set_mask(mask);
  w.set_state(noise(mask, 0.01), std::vector<double>(mask.size(), 0.0));
  const bool sym = !w.ghosts().empty();
  const sym_ops ops(w);
  const double e0 = w.energy(), m0 = sym ? ops.modified() : 0.0;
  bounded b;
  int ne = 0, nl = 0;
  std::uint64_t x = 0x2545f4914f6cdd1dull;
  for (int s = 1; s <= steps; ++s) {
    double dt = st.lo;
    if (st.alternate) {
      dt = s % 2 ? st.lo : st.hi;
    } else if (st.hi != st.lo) {
      x ^= x << 13;
      x ^= x >> 7;
      x ^= x << 17;
      dt = st.lo + (st.hi - st.lo) * (double(x >> 11) / 9007199254740992.0);
    }
    b.substeps += w.substeps(dt);
    w.step(dt);
    b.pending = std::max(b.pending, w.pending());
    if (s % every != 0)
      continue;
    b.last = w.energy() / e0;
    if (!(b.last <= 1e30)) // overflowed or not a number: stop counting
      b.last = 1e30;
    b.hi = std::max(b.hi, b.last);
    b.lo = std::min(b.lo, b.last);
    if (s <= steps / 4) {
      b.early += b.last;
      ++ne;
    }
    if (s > steps - steps / 4) {
      b.late += b.last;
      ++nl;
    }
    if (sym && !damped && s % (every * 10) == 0)
      b.modified = std::max(b.modified, std::abs(ops.modified() / m0 - 1.0));
    if (b.last >= 1e30)
      break;
  }
  b.early /= std::max(ne, 1);
  b.late /= std::max(nl, 1);
  return b;
}

} // namespace

// Stability (the review of O8c, as a test): broadband noise in h on the water of closed basins,
// no damping, beach or sponge, at 64^2 with 0.25 m cells (a disc of water 12 m across, the disc
// with an island, the disc with a breakwater four cells thick, a straight channel 5 m wide, a
// 45-degree channel, a wall two cells thick across the window, and a 2-cell wall 8 m long in open
// water), each for 20000 steps at three substeps: omega_max dt_s = 1.8 (the limit, one substep a
// step), 0.125 s (steps of 0.25 s, two substeps each) and 1/30 s (omega_max dt_s 0.44).
//   - The symmetric step conserves its modified energy exactly (a quadratic invariant of the
//     linear map, positive definite at omega dt_s <= 1.8): its drift is round-off, so nothing can
//     grow.
//   - energy(), H itself, wobbles about it: from an h-only start H can only fall below its start
//     by the modified energy's share in the shortest waves (all of their energy is potential at
//     the start, where H is largest against the invariant). It never rises above the start, and
//     its mean over the last quarter is within its fluctuation (0.005) of the first quarter's.
//   - Zero walls (the exact step) never rose above their start either (asserted at fewer steps).
TEST(Ewave, WallsAreStable) {
  const double limit = (1.8 / omega_max(0.25)) * (1.0 - 1e-9);
  const stepping steppings[] = {{"omega_max dt_s 1.8", 0.25, limit, limit, limit},
                                {"dt 0.25 s, dt_s 0.125 s", 0.25, 0.125, 0.25, 0.25},
                                {"dt_s 1/30 s", 0.25, 1.0 / 30.0, 1.0 / 30.0, 1.0 / 30.0}};
  double worst_mod = 0.0, worst_hi = 0.0, lowest = 1.0;
  for (const basin &b : kBasins) {
    for (const stepping &st : steppings) {
      const bounded nf = run_basin(b.wall, ewave_walls::no_flux, st, 20000);
      std::printf("stable, %s, %s (%lld substeps): energy in %.4f..%.4f of the start, ends %.4f "
                  "(mean over the first quarter %.4f, the last %.4f); the modified energy within "
                  "%.1e\n",
                  b.name, st.name, nf.substeps, nf.lo, nf.hi, nf.last, nf.early, nf.late,
                  nf.modified);
      EXPECT_LT(nf.modified, 1e-9) << b.name << ", " << st.name;
      EXPECT_LE(nf.hi, 1.0 + 1e-9) << b.name << ", " << st.name;
      EXPECT_LE(nf.late, nf.early + 0.005) << b.name << ", " << st.name;
      worst_mod = std::max(worst_mod, nf.modified);
      worst_hi = std::max(worst_hi, nf.hi);
      lowest = std::min(lowest, nf.lo);
    }
    const bounded z =
        run_basin(b.wall, ewave_walls::zero, stepping{"dt 0.1 s", 0.25, 0.1, 0.1, 0.1}, 4000);
    std::printf("stable, %s, zero walls, dt 0.1 s, 400 s: at most %.4f of the start (ends %.4f)\n",
                b.name, z.hi, z.last);
    EXPECT_LE(z.hi, 1.0 + 1e-6) << b.name << " under zero walls";
  }
  std::printf("stable: over every basin and step, the modified energy within %.1e, energy() at "
              "most %.6f and at least %.4f of its start\n",
              worst_mod, worst_hi, lowest);
}

// Stability when dt changes from step to step (the second review of O8c): a game loop's frame dt
// jitters. Stormer-Verlet's kappa~ and its invariant both depend on the substep, so substeps that
// changed with dt would each keep a different ellipse, and switching between them pumps energy in
// by parametric resonance (the review measured a disc grow 1e103-fold in 4000 steps of 0.136 and
// 0.101 s in turn, and 26-fold in 60000 steps of a frame dt in [1/60, 1/30] s at 0.05 m cells).
// So the substep is fixed per window (ewave_params::substep, capped at the stable limit) and
// step(dt) runs whole substeps, carrying the rest: every substep is the same linear map whatever
// the dts, and its modified energy is conserved to round-off. Checked in the disc, the straight
// channel and the 2-cell wall in open water, from broadband noise:
//   - dt 0.136 and 0.101 s in turn, 2000 steps, at the default substep (1/60 s, 0.25 m cells);
//   - dt uniform in [0.6, 1.0] of the stable limit, 20000 steps, the substep at the limit (a step
//     runs 0 or 1 substep);
//   - a frame dt uniform in [1/60, 1/30] s, 15000 steps, at 0.05 m cells and the default substep;
//   - the same frame jitter, 6000 steps, with the damping, the sponge and a beach on: the energy
//     never rises above its start and is gone by the end.
TEST(Ewave, VariableStepsAreStable) {
  const double limit = (1.8 / omega_max(0.25)) * (1.0 - 1e-9);
  stepping alternate{"dt 0.136 / 0.101 s in turn", 0.25, 1.0 / 60.0, 0.136, 0.101};
  alternate.alternate = true;
  const stepping steppings[] = {
      alternate,
      {"dt in [0.6, 1.0] of the limit, dt_s at the limit", 0.25, limit, 0.6 * limit, limit},
      {"frame dt in [1/60, 1/30] s, 0.05 m cells", 0.05, 1.0 / 60.0, 1.0 / 60.0, 1.0 / 30.0}};
  const int counts[] = {2000, 20000, 15000};
  const basin basins[] = {kBasins[0], kBasins[3], kBasins[6]};
  for (const basin &b : basins) {
    for (int k = 0; k < 3; ++k) {
      const stepping &st = steppings[k];
      const bounded nf = run_basin(b.wall, ewave_walls::no_flux, st, counts[k]);
      std::printf("variable dt, %s, %s: %d steps, %lld substeps; energy in %.4f..%.4f of the "
                  "start, ends %.4f (first quarter %.4f, last %.4f); the modified energy within "
                  "%.1e; pending at most %.3f of a substep\n",
                  b.name, st.name, counts[k], nf.substeps, nf.lo, nf.hi, nf.last, nf.early, nf.late,
                  nf.modified, nf.pending / std::min(st.substep, 1.8 / omega_max(st.dx)));
      EXPECT_LT(nf.modified, 1e-9) << b.name << ", " << st.name;
      EXPECT_LE(nf.hi, 1.0 + 1e-9) << b.name << ", " << st.name;
      EXPECT_LE(nf.late, nf.early + 0.005) << b.name << ", " << st.name;
    }
    const bounded d = run_basin(b.wall, ewave_walls::no_flux, steppings[2], 6000, 100, true);
    std::printf("variable dt, %s, damped (damping_k2, damping, sponge, beach), %s: energy at most "
                "%.6f of the start, ends %.3e\n",
                b.name, steppings[2].name, d.hi, d.last);
    EXPECT_LE(d.hi, 1.0 + 1e-9) << b.name << ", damped";
    EXPECT_LT(d.last, 0.5) << b.name << ", damped";
  }
}

// The readers' limit: no water cell is read by more than ewave_max_readers ghosts, and where more
// would, whole ghosts go, farthest from the shore first. The 64^2 disc's coast compresses its
// image (a ghost d cells out mirrors onto a circle d cells in), so it needs the limit; a straight
// wall does not.
TEST(Ewave, TheReadersLimitDropsWholeGhosts) {
  struct counted {
    std::size_t ghosts = 0;
    int most = 0;
  };
  auto count = [](const std::vector<ewave_ghost> &gs, int n) {
    counted c;
    c.ghosts = gs.size();
    std::vector<int> r(std::size_t(n) * n, 0);
    for (const ewave_ghost &g : gs) {
      const std::size_t q[4] = {g.corner, g.corner + 1, g.corner + std::size_t(n),
                                g.corner + std::size_t(n) + 1};
      for (int k = 0; k < 4; ++k)
        if (g.w[k] != 0.0)
          c.most = std::max(c.most, ++r[q[k]]);
    }
    return c;
  };
  const int band = ewave_params().ghost_cells;
  for (const basin &b : kBasins) {
    const std::vector<double> mask = mask_of(64, b.wall);
    const counted all = count(ewave_wall_ghosts(64, mask, band, 0), 64),
                  lim = count(ewave_wall_ghosts(64, mask, band), 64);
    std::printf("readers, %s: %zu ghosts, up to %d on a water cell; with the limit (%d) %zu "
                "ghosts, up to %d\n",
                b.name, all.ghosts, all.most, ewave_max_readers, lim.ghosts, lim.most);
    EXPECT_LE(lim.most, ewave_max_readers) << b.name;
    if (all.most <= ewave_max_readers)
      EXPECT_EQ(lim.ghosts, all.ghosts) << b.name;
  }
  // A 256^2 coast like StepCost's, and the straight wall: nothing dropped on the straight wall.
  std::vector<double> mask(256 * 256, 1.0);
  for (int j = 0; j < 256; ++j)
    for (int i = 128; i < 256; ++i)
      mask[std::size_t(j) * 256 + std::size_t(i)] = 0.0;
  const counted straight = count(ewave_wall_ghosts(256, mask, band), 256);
  EXPECT_EQ(straight.ghosts, 128u * 256u);
  EXPECT_EQ(straight.most, 1);
}

// Lossless walls: a smooth ring (sigma 1.5 m) bouncing about a closed basin for 300 s (3000 steps
// of 0.1 s, dozens of reflections) with no damping, beach or sponge, in a straight channel, a
// 45-degree staircase channel, a disc and a pond with an island. Staircase and curved walls are
// first order in where they reflect, but they lose nothing: the modified energy is conserved to
// round-off, and energy() (H) only wobbles below its start, by phi's Stormer-Verlet amplitude
// (ewave.h; from an h-only start H is at its largest). energy() is checked against the test's H.
TEST(Ewave, NoFluxWallsAreLossless) {
  struct pond {
    const char *name;
    bool (*wall)(int i, int j);
    double x, y; // the drop (m)
  };
  const pond ponds[] = {
      {"straight channel", [](int i, int) { return i < 24 || i >= 104; }, 12.0, 16.0},
      {"45-degree channel",
       [](int i, int j) {
         const int d = (i - j + 128) % 128;
         return d < 24 || d >= 104;
       },
       20.0, 4.0},
      {"disc",
       [](int i, int j) { return (i - 63.5) * (i - 63.5) + (j - 63.5) * (j - 63.5) > 48.0 * 48.0; },
       12.5, 17.5},
      {"pond with an island",
       [](int i, int j) {
         return (i - 63.5) * (i - 63.5) + (j - 63.5) * (j - 63.5) > 48.0 * 48.0 ||
                (i - 70.0) * (i - 70.0) + (j - 58.0) * (j - 58.0) < 10.0 * 10.0;
       },
       8.0, 12.0},
  };
  for (const pond &b : ponds) {
    ewave_params p = bare(128, 0.25);
    p.substep = 0.1;
    ewave w(p);
    w.set_mask(mask_of(128, b.wall));
    w.add_height(b.x, b.y, 0.1, 1.5);
    const sym_ops ops(w);
    const double e0 = w.energy(), m0 = ops.modified(), scale = p.density * p.dx * p.dx;
    double lo = 1.0, hi = 1.0, agree = std::abs(ops.energy() * scale / e0 - 1.0), mod = 0.0;
    const int steps = 3000;
    for (int s = 1; s <= steps; ++s) {
      w.step(0.1);
      if (s % 20 != 0)
        continue;
      const double e = w.energy() / e0;
      if (s % 200 == 0) {
        agree = std::max(agree, std::abs(ops.energy() * scale - w.energy()) / e0);
        mod = std::max(mod, std::abs(ops.modified() / m0 - 1.0));
      }
      lo = std::min(lo, e);
      hi = std::max(hi, e);
    }
    const double last = w.energy() / e0;
    std::printf("lossless, %s (%zu ghosts): over %d steps of 0.1 s the modified energy within "
                "%.1e of its start; energy() in %.4f..%.4f, ends at %.4f; agrees with the test's "
                "H to %.1e\n",
                b.name, w.ghosts().size(), steps, mod, lo, hi, last, agree);
    EXPECT_LT(mod, 1e-9);
    EXPECT_GT(lo, 0.97);
    EXPECT_LT(hi, 1.0 + 1e-3);
    EXPECT_LT(agree, 1e-9);
  }
}

// damping_k2 under no-flux walls takes what it takes under zero walls (lsystem_coast's crater
// lake, whose ring faded faster with no-flux banks than with zero ones). A disc of water 36 m
// across on a 128^2 window of 0.5 m cells, 3.5 m deep (the lake's size), a dent dropped off its
// centre, damping_k2 2e-3 m^2/s (lsystem_coast's): the share of the energy at +0.5 s left at
// +10 s is the same with either walls, to 2 %. The filter's first form, h <- M^-1 R^T F R h,
// kept 49 % where the zero walls keep 75 %: M^-1 R^T R is not the identity where a ghost's
// mirror point falls between cells, but averages each water cell the ghost reads with the
// corners round that point, and a disc's coast compresses its image over most of the water. The
// filter h <- h - M^-1 R^T (1 - F) R h is the identity as F goes to 1: at damping_k2 1e-15 the
// run matches the undamped one to round-off (the first form's was 99 % off).
TEST(Ewave, DampingK2TakesTheSameShareWhateverTheWalls) {
  auto run = [](ewave_walls walls, double gamma, std::vector<double> *h) {
    ewave_params p = bare(128, 0.5, 3.5);
    p.walls = walls;
    p.damping_k2 = gamma;
    ewave w(p);
    std::vector<double> mask(128 * 128, 0.0);
    for (int j = 0; j < 128; ++j)
      for (int i = 0; i < 128; ++i) {
        const double x = (i - 64) * 0.5, y = (j - 64) * 0.5;
        if (x * x + y * y < 18.0 * 18.0)
          mask[std::size_t(j) * 128 + std::size_t(i)] = 1.0;
      }
    w.set_mask(mask);
    w.add_height(35.0, 33.2, -0.2, 1.0);
    double e0 = 0.0;
    for (int f = 1; f <= 300; ++f) {
      w.step(1.0 / 30.0);
      if (f == 15)
        e0 = w.energy();
    }
    if (h)
      *h = w.height();
    return w.energy() / e0;
  };
  const double zero = run(ewave_walls::zero, 2e-3, nullptr),
               no_flux = run(ewave_walls::no_flux, 2e-3, nullptr);
  std::vector<double> ha, hb;
  run(ewave_walls::no_flux, 1e-15, &ha);
  run(ewave_walls::no_flux, 0.0, &hb);
  double diff = 0.0, top = 0.0;
  for (std::size_t c = 0; c < ha.size(); ++c) {
    diff = std::max(diff, std::abs(ha[c] - hb[c]));
    top = std::max(top, std::abs(hb[c]));
  }
  std::printf("damping_k2 2e-3 in a disc 36 m across, the energy left after 10 s: zero walls "
              "%.3f, no-flux walls %.3f; at damping_k2 1e-15 the heights within %.1e of the "
              "undamped run's\n",
              zero, no_flux, diff / top);
  EXPECT_GT(zero, 0.6);
  EXPECT_LT(zero, 0.9);
  EXPECT_NEAR(no_flux, zero, 0.02);
  EXPECT_LT(diff / top, 1e-9);
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
  p.substep = 0.1;
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

// A wall of finite thickness with water behind it leaks: the mask is a lid on the surface with
// water under it, and the spectral operator couples across it. Under zero walls a 12-cell (3 m)
// band lets through a few percent of the incident amplitude, a 6 m band less. Behind a thick band
// most of what is left came round the periodic window through the sponge: in a window twice as
// wide (n 512) a 6 m band reads 1.4 % instead of 2.8 %, and a 12 m band 1.0 % instead of 3.1 %.
//
// Under no_flux walls each face mirrors its own water, and the image of the front face (twice the
// incident wave) sits inside the wall, nearer the water behind, so a wall a few metres thick
// leaks about twice what a zero wall does. A wall one cell thick has both faces equally near
// every one of its cells: they are seams (ewave_wall_ghosts), their image tapered to almost 0, so
// it holds no copy of either side's water and leaks as a zero wall does (the first build of the
// symmetric step mirrored the front water into it and let 88 % through). Walls two and four
// cells thick are mostly seam too, and leak within 15 % of a zero wall.
TEST(Ewave, AThinWallLeaksLittle) {
  struct leak {
    double thick, zero, no_flux;
  };
  std::vector<leak> leaks;
  for (double t : {0.25, 0.5, 1.0, 3.0, 6.0, 12.0})
    leaks.push_back(leak{t, wall_leak(t, 16.0, 2.0, ewave_walls::zero),
                         wall_leak(t, 16.0, 2.0, ewave_walls::no_flux)});
  for (const leak &l : leaks)
    std::printf("wall band %.2f m (%d cells): behind it %.2f %% of the incident amplitude under "
                "zero walls, %.2f %% under no_flux walls\n",
                l.thick, int(l.thick / 0.25 + 0.5), 100.0 * l.zero, 100.0 * l.no_flux);
  // Zero walls, as O8a.
  EXPECT_LT(leaks[3].zero, 0.06);
  EXPECT_LT(leaks[4].zero, leaks[3].zero);
  // No-flux walls: one cell thick no worse than a zero wall, two and four cells within 15 % of
  // one, and thick walls a few percent.
  EXPECT_LE(leaks[0].no_flux, leaks[0].zero * 1.02);
  EXPECT_LE(leaks[1].no_flux, leaks[1].zero * 1.15);
  EXPECT_LE(leaks[2].no_flux, leaks[2].zero * 1.15);
  EXPECT_LT(leaks[3].no_flux, 0.12);
  EXPECT_LT(leaks[4].no_flux, 0.07);
  EXPECT_LT(leaks[5].no_flux, 0.04);
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
  // No-flux walls (the default): the land at i >= 44 holds ghosts, and the symmetric step runs
  // in substeps of 1/60 s (two a step of 1/30 s, three of 0.05 s), each with damping_k2's filter,
  // the beach and the sponge.
  const std::uint64_t d = scenario(0.0, ewave_walls::no_flux),
                      e = scenario(0.0, ewave_walls::no_flux),
                      f = scenario(1e-12, ewave_walls::no_flux);
  std::printf("ewave digest (no_flux walls) 0x%016llx\n", (unsigned long long)d);
  EXPECT_EQ(d, e);
  EXPECT_NE(d, f);
  EXPECT_NE(d, a);
  EXPECT_EQ(d, 0xaefff2ae097ed743ull) << std::hex << "ewave digest 0x" << d;
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
  EXPECT_THROW(make([](ewave_params &p) { p.ghost_cells = 4097; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.walls = ewave_walls(7); }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.substep = 0.0; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.substep = 1.5; }), std::invalid_argument);
  EXPECT_THROW(make([](ewave_params &p) { p.substep = std::nan(""); }), std::invalid_argument);
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
  // set_mask with no-flux walls finds each wall cell's nearest shore (a distance transform, O(n^2)
  // whatever the band), the readers' limit and the taper: a 256^2 window, its right half land
  // behind a wavy coast, for a band of 16 cells and the default (the whole wall). Then steps of
  // 1/30 s with the ghosts, two substeps each at the default substep (1/60 s). Then set_mask alone
  // on the same coast scaled to 512^2 and 1024^2.
  for (int band : {16, ewave_params().ghost_cells}) {
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
    const int sub = w.substeps(1.0 / 30.0);
    const auto t1 = std::chrono::steady_clock::now();
    for (int s = 0; s < 20; ++s)
      w.step(1.0 / 30.0);
    const double step =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count() /
        20.0;
    std::printf(
        "no-flux set_mask at N = 256, ghost band %d: %.1f ms for %zu ghosts; a step of 1/30 "
        "s (%d substeps) %.2f ms\n",
        band, ms, w.ghosts().size(), sub, step);
    EXPECT_GT(w.ghosts().size(), 0u);
  }
  for (int n : {512, 1024}) {
    ewave_params p;
    p.n = n;
    ewave w(p);
    std::vector<double> mask(std::size_t(n) * std::size_t(n), 1.0);
    const double r = n / 256.0;
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        if (i >= int(r * (128.0 + 12.0 * std::sin(j / r * 0.05))))
          mask[std::size_t(j) * std::size_t(n) + std::size_t(i)] = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    w.set_mask(mask);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("no-flux set_mask at N = %d, the default band: %.1f ms for %zu ghosts\n", n, ms,
                w.ghosts().size());
    EXPECT_GT(w.ghosts().size(), 0u);
  }
}
