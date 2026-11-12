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

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/ewave.h>
#include <numeric>
#include <stdexcept>
#include <string>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

namespace {

constexpr double kTwoPi = 6.283185307179586;

bool finite(double v) { return std::isfinite(v); }

ewave_params checked(const ewave_params &p) {
  if (p.n < 4 || p.n > 4096 || (p.n & (p.n - 1)) != 0)
    throw std::invalid_argument("ewave: n must be a power of two in 4..4096, got " +
                                std::to_string(p.n));
  if (!(p.dx > 0.0) || !finite(p.dx) || !finite(p.origin_x) || !finite(p.origin_y))
    throw std::invalid_argument("ewave: dx must be finite and above 0, the origin finite");
  if (!(p.depth > 0.0) || !(p.gravity > 0.0) || !finite(p.gravity) || !(p.density > 0.0) ||
      !finite(p.density))
    throw std::invalid_argument("ewave: depth, gravity and density must be above 0 (gravity and "
                                "density finite)");
  if (!(p.damping_k2 >= 0.0) || !finite(p.damping_k2) || !(p.damping >= 0.0) ||
      !finite(p.damping) || !(p.sponge_rate >= 0.0) || !finite(p.sponge_rate) ||
      !(p.beach_rate >= 0.0) || !finite(p.beach_rate))
    throw std::invalid_argument("ewave: damping_k2, damping, sponge_rate and beach_rate must be "
                                "finite and >= 0");
  if (!(p.sponge_cells >= 0.0) || !(p.sponge_cells <= 0.5 * p.n))
    throw std::invalid_argument("ewave: sponge_cells must be in [0, n / 2]");
  if (p.walls != ewave_walls::no_flux && p.walls != ewave_walls::zero)
    throw std::invalid_argument("ewave: walls must be no_flux or zero");
  if (p.ghost_cells < 0 || p.ghost_cells > 4096)
    throw std::invalid_argument("ewave: ghost_cells must be in [0, 4096]");
  return p;
}

// FNV-1a over the bytes of every value the window holds.
struct digester {
  std::uint64_t h = 0xcbf29ce484222325ull;
  void bytes(const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= b[i];
      h *= 0x100000001b3ull;
    }
  }
  void add(double v) { bytes(&v, sizeof v); }
  void add(std::int64_t v) { bytes(&v, sizeof v); }
  void add(const std::vector<double> &v) {
    add(std::int64_t(v.size()));
    if (!v.empty())
      bytes(v.data(), v.size() * sizeof(double));
  }
};

} // namespace

std::vector<ewave_ghost> ewave_wall_ghosts(int n, const std::vector<double> &mask, int ghost_cells,
                                           int max_readers) {
  std::vector<ewave_ghost> out;
  if (n < 2 || mask.size() != std::size_t(n) * std::size_t(n))
    throw std::invalid_argument("ewave_wall_ghosts: the mask must have n x n values, n >= 2");
  if (ghost_cells <= 0)
    return out;
  const int band = std::min(ghost_cells, n);
  const std::size_t un = std::size_t(n);
  auto wall = [&](std::size_t c) { return !(mask[c] > 0.0); };
  // Per cell, the distance along its row to the nearest non-wall cell on the left (left) and on
  // the right (right), capped at far: no wrap.
  const int far = band + 1;
  std::vector<int> left(mask.size()), right(mask.size());
  for (int j = 0; j < n; ++j) {
    const std::size_t row = std::size_t(j) * un;
    int run = far;
    for (int i = 0; i < n; ++i) {
      run = !wall(row + std::size_t(i)) ? 0 : std::min(run + 1, far);
      left[row + std::size_t(i)] = run;
    }
    run = far;
    for (int i = n - 1; i >= 0; --i) {
      run = !wall(row + std::size_t(i)) ? 0 : std::min(run + 1, far);
      right[row + std::size_t(i)] = run;
    }
  }
  // |q - p| - 1/2 < band, in integers: d^2 <= band^2 + band.
  const long long limit = (long long)band * band + band;
  struct offset {
    int di, dj;
  };
  std::vector<offset> nearest;
  std::vector<long long> dist2;           // per ghost in out: |q - p|^2, for the readers' limit
  std::vector<double> mirror_x, mirror_y; // and its mirror point p'
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::size_t c = std::size_t(j) * un + std::size_t(i);
      if (!wall(c))
        continue;
      // The nearest non-wall cells: in each row within the band, the nearest on either side.
      long long best = limit + 1;
      nearest.clear();
      for (int dj = -band; dj <= band; ++dj) {
        const int jj = j + dj;
        if (jj < 0 || jj >= n)
          continue;
        const std::size_t r = std::size_t(jj) * un + std::size_t(i);
        for (int side = 0; side < 2; ++side) {
          const int d = side == 0 ? left[r] : right[r];
          if (d > band || (side == 1 && d == 0))
            continue; // too far, or the cell itself (found on the left already)
          const long long d2 = (long long)dj * dj + (long long)d * d;
          if (d2 > best)
            continue;
          if (d2 < best) {
            best = d2;
            nearest.clear();
          }
          nearest.push_back(offset{side == 0 ? -d : d, dj});
        }
      }
      if (best > limit)
        continue;
      // p' = p + 2 s n = p + (q - p) (2 |q - p| - 1) / |q - p|, the product first so that a
      // straight wall's mirror is exact; the mean over the nearest cells that face the first.
      const double dist = std::sqrt(double(best)), two_s = 2.0 * dist - 1.0;
      const offset ref = nearest.front();
      double sx = 0.0, sy = 0.0;
      int count = 0;
      for (const offset &o : nearest) {
        if ((long long)o.di * ref.di + (long long)o.dj * ref.dj < 0)
          continue;
        sx += (double(o.di) * two_s) / dist;
        sy += (double(o.dj) * two_s) / dist;
        ++count;
      }
      const double px = double(i) + sx / double(count), py = double(j) + sy / double(count);
      if (!(px >= 0.0) || !(py >= 0.0) || !(px <= n - 1.0) || !(py <= n - 1.0))
        continue;
      const int i0 = std::min(int(std::floor(px)), n - 2),
                j0 = std::min(int(std::floor(py)), n - 2);
      const double tx = px - double(i0), ty = py - double(j0);
      const std::size_t c0 = std::size_t(j0) * un + std::size_t(i0);
      const std::size_t corners[4] = {c0, c0 + 1, c0 + un, c0 + un + 1};
      double w[4] = {(1.0 - tx) * (1.0 - ty), tx * (1.0 - ty), (1.0 - tx) * ty, tx * ty};
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        if (wall(corners[k]))
          w[k] = 0.0;
        sum += w[k];
      }
      if (!(sum > 0.0))
        continue;
      ewave_ghost g;
      g.cell = std::uint32_t(c);
      g.corner = std::uint32_t(c0);
      for (int k = 0; k < 4; ++k)
        g.w[k] = w[k] / sum;
      out.push_back(g);
      dist2.push_back(best);
      mirror_x.push_back(px);
      mirror_y.push_back(py);
    }
  if (max_readers > 0 && !out.empty()) {
    // The readers' limit: whole ghosts dropped, farthest from the shore first, while a cell they
    // read is over it.
    const std::size_t offs[4] = {0, 1, un, un + 1};
    std::vector<int> readers(mask.size(), 0);
    int most = 0;
    for (const ewave_ghost &g : out)
      for (int k = 0; k < 4; ++k)
        if (g.w[k] != 0.0)
          most = std::max(most, ++readers[g.corner + offs[k]]);
    if (most > max_readers) {
      std::vector<std::size_t> order(out.size());
      std::iota(order.begin(), order.end(), std::size_t(0));
      std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return dist2[a] != dist2[b] ? dist2[a] > dist2[b] : out[a].cell > out[b].cell;
      });
      std::vector<char> keep(out.size(), 1);
      for (std::size_t q : order) {
        const ewave_ghost &g = out[q];
        bool over = false;
        for (int k = 0; k < 4; ++k)
          over = over || (g.w[k] != 0.0 && readers[g.corner + offs[k]] > max_readers);
        if (!over)
          continue;
        keep[q] = 0;
        for (int k = 0; k < 4; ++k)
          if (g.w[k] != 0.0)
            --readers[g.corner + offs[k]];
      }
      std::size_t m = 0;
      for (std::size_t q = 0; q < out.size(); ++q)
        if (keep[q]) {
          out[m] = out[q];
          mirror_x[m] = mirror_x[q];
          mirror_y[m] = mirror_y[q];
          ++m;
        }
      out.resize(m);
      mirror_x.resize(m);
      mirror_y.resize(m);
    }
  }
  // The taper. The extended field is cut where a wall cell has no ghost (past the band, a mirror
  // point out of the window or among walls, a ghost dropped above) and where neighbouring ghosts
  // mirror far apart (a seam: the two faces of a thin wall, a medial axis), and the fold would
  // carry G's response to that step (or to the other face's image) into the water. So a ghost
  // within `taper` cells of either has its weights scaled by sin^2(pi/2 D / taper), D the
  // distance to the nearest such cell centre (plus half a cell for a seam, which lies between
  // two cells), and the extended field falls to 0 smoothly instead.
  const int taper = ewave_taper_cells, tseam = ewave_taper_cells;
  if ((taper > 0 || tseam > 0) && !out.empty()) {
    std::vector<int> index(mask.size(), -1);
    for (std::size_t q = 0; q < out.size(); ++q)
      index[out[q].cell] = int(q);
    // Seams: a ghost whose mirror point is more than 2.5 cells from a neighbour's (a water
    // neighbour's being itself).
    std::vector<char> seam(mask.size(), 0);
    for (std::size_t q = 0; q < out.size(); ++q) {
      const int i = int(out[q].cell % un), j = int(out[q].cell / un);
      const int ni[4] = {i - 1, i + 1, i, i}, nj[4] = {j, j, j - 1, j + 1};
      for (int e = 0; e < 4; ++e) {
        if (ni[e] < 0 || nj[e] < 0 || ni[e] >= n || nj[e] >= n)
          continue;
        const std::size_t c = std::size_t(nj[e]) * un + std::size_t(ni[e]);
        double qx, qy;
        if (!wall(c)) {
          qx = double(ni[e]);
          qy = double(nj[e]);
        } else if (index[c] >= 0) {
          qx = mirror_x[std::size_t(index[c])];
          qy = mirror_y[std::size_t(index[c])];
        } else {
          continue;
        }
        const double ex = mirror_x[q] - qx, ey = mirror_y[q] - qy;
        if (ex * ex + ey * ey > 6.25)
          seam[out[q].cell] = 1;
      }
    }
    // The squared distance from each ghost to the nearest cell of a set, capped at t^2.
    auto nearest2 = [&](auto in_set, int t, std::vector<long long> &d2) {
      d2.assign(out.size(), (long long)t * t);
      if (t <= 0)
        return;
      const int tfar = t + 1;
      for (int j = 0; j < n; ++j) {
        const std::size_t row = std::size_t(j) * un;
        int run = tfar;
        for (int i = 0; i < n; ++i) {
          run = in_set(row + std::size_t(i)) ? 0 : std::min(run + 1, tfar);
          left[row + std::size_t(i)] = run;
        }
        run = tfar;
        for (int i = n - 1; i >= 0; --i) {
          run = in_set(row + std::size_t(i)) ? 0 : std::min(run + 1, tfar);
          right[row + std::size_t(i)] = run;
        }
      }
      for (std::size_t q = 0; q < out.size(); ++q) {
        const int i = int(out[q].cell % un), j = int(out[q].cell / un);
        for (int dj = -t; dj <= t; ++dj) {
          const int jj = j + dj;
          if (jj < 0 || jj >= n)
            continue;
          const std::size_t r = std::size_t(jj) * un + std::size_t(i);
          const int d = std::min(left[r], right[r]);
          if (d <= t)
            d2[q] = std::min(d2[q], (long long)dj * dj + (long long)d * d);
        }
      }
    };
    std::vector<long long> deep2, seam2;
    nearest2([&](std::size_t c) { return wall(c) && index[c] < 0; }, taper, deep2);
    nearest2([&](std::size_t c) { return seam[c] != 0; }, tseam, seam2);
    for (std::size_t q = 0; q < out.size(); ++q) {
      double f = 1.0;
      if (deep2[q] < (long long)taper * taper) {
        const double s = det::sin(0.25 * kTwoPi * (std::sqrt(double(deep2[q])) / double(taper)));
        f = s * s;
      }
      if (seam2[q] < (long long)tseam * tseam) {
        const double d = std::min(std::sqrt(double(seam2[q])) + 0.5, double(tseam));
        const double s = det::sin(0.25 * kTwoPi * (d / double(tseam)));
        f = std::min(f, s * s);
      }
      if (f < 1.0)
        for (int k = 0; k < 4; ++k)
          out[q].w[k] *= f;
    }
  }
  return out;
}

std::vector<double> ewave_lumped_mass(int n, const std::vector<double> &mask,
                                      const std::vector<ewave_ghost> &ghosts) {
  if (n < 2 || mask.size() != std::size_t(n) * std::size_t(n))
    throw std::invalid_argument("ewave_lumped_mass: the mask must have n x n values, n >= 2");
  // The row sums of R^T R: 1 + for each ghost that reads a water cell its weight on it times its
  // total weight (1 but in the taper).
  const std::size_t un = std::size_t(n);
  const std::size_t offs[4] = {0, 1, un, un + 1};
  std::vector<double> mass(mask.size(), 0.0);
  for (std::size_t c = 0; c < mask.size(); ++c)
    if (mask[c] > 0.0)
      mass[c] = 1.0;
  for (const ewave_ghost &g : ghosts) {
    const double total = (g.w[0] + g.w[1]) + (g.w[2] + g.w[3]);
    for (int k = 0; k < 4; ++k)
      if (g.w[k] != 0.0)
        mass[g.corner + offs[k]] += g.w[k] * total;
  }
  return mass;
}

int ewave_substeps(const ewave_params &p, double dt) {
  if (!(dt > 0.0) || !finite(dt))
    throw std::invalid_argument("ewave_substeps: dt must be above 0 and finite");
  // omega at the corner wavenumber, |k| = sqrt(2) pi / dx: the largest on the window.
  const double k = (0.5 * kTwoPi) * std::sqrt(2.0) / p.dx;
  const double om = std::sqrt(p.gravity * k * det::tanh(std::min(k * p.depth, 20.0)));
  const double x = (om * dt) / ewave_max_phase;
  if (!(x > 1.0))
    return 1;
  if (!(x <= 1e6))
    throw std::invalid_argument("ewave: a step of " + std::to_string(dt) +
                                " s needs more than 10^6 substeps");
  return int(std::ceil(x));
}

ewave::ewave(const ewave_params &p) : _p(checked(p)), _fft(p.n) {
  const int n = _p.n;
  const std::size_t cells = std::size_t(n) * std::size_t(n);
  _h.assign(cells, 0.0);
  _phi.assign(cells, 0.0);
  _mask.assign(cells, 1.0);
  _z.assign(cells * 2, 0.0);
  _k2.resize(cells);
  _kappa.resize(cells);
  // Index p is wavenumber q = p (p < n / 2) or p - n, times 2 pi / (n dx); the Nyquist index n / 2
  // pairs with itself, and |k| is the same either way.
  const double dk = kTwoPi / (double(n) * _p.dx);
  for (int j = 0; j < n; ++j) {
    const double ky = dk * double(j < n / 2 ? j : j - n);
    for (int i = 0; i < n; ++i) {
      const double kx = dk * double(i < n / 2 ? i : i - n);
      const std::size_t c = std::size_t(j) * std::size_t(n) + std::size_t(i);
      _k2[c] = kx * kx + ky * ky;
      const double k = std::sqrt(_k2[c]);
      _kappa[c] = k > 0.0 ? k * det::tanh(std::min(k * _p.depth, 20.0)) : 0.0;
    }
  }
}

double ewave::omega(double k) const {
  if (!(k > 0.0))
    return 0.0;
  return std::sqrt(_p.gravity * k * det::tanh(std::min(k * _p.depth, 20.0)));
}

void ewave::set_mask(const std::vector<double> &mask) {
  if (mask.size() != _mask.size())
    throw std::invalid_argument("ewave: the mask must have n x n values");
  bool below = false;
  for (double m : mask) {
    if (!(m >= 0.0) || !(m <= 1.0))
      throw std::invalid_argument("ewave: mask values must be in [0, 1]");
    below = below || m < 1.0;
  }
  _mask = mask;
  _masked = below;
  _real_stale = true;
  _ghosts.clear();
  _mass.clear();
  _inv_mass.clear();
  if (_p.walls == ewave_walls::no_flux)
    _ghosts = ewave_wall_ghosts(_p.n, _mask, _p.ghost_cells);
  if (_ghosts.empty())
    return;
  _mass = ewave_lumped_mass(_p.n, _mask, _ghosts);
  _inv_mass.assign(_mask.size(), 0.0);
  for (std::size_t c = 0; c < _mask.size(); ++c)
    if (_mask[c] > 0.0)
      _inv_mass[c] = 1.0 / _mass[c];
  _acc.assign(_mask.size(), 0.0);
  _acc2.assign(_mask.size(), 0.0);
}

void ewave::fill(double *z, const double *a, const double *b) const {
  // Every weight with a wall corner is 0, and a and b are 0 on walls.
  const std::size_t un = std::size_t(_p.n);
  for (const ewave_ghost &g : _ghosts) {
    const std::size_t c = g.corner;
    z[std::size_t(g.cell) * 2] =
        g.w[0] * a[c] + g.w[1] * a[c + 1] + g.w[2] * a[c + un] + g.w[3] * a[c + un + 1];
    if (b)
      z[std::size_t(g.cell) * 2 + 1] =
          g.w[0] * b[c] + g.w[1] * b[c + 1] + g.w[2] * b[c + un] + g.w[3] * b[c + un + 1];
  }
}

void ewave::fold(const double *z, double *acc, double *acc2) const {
  const std::size_t un = std::size_t(_p.n), cells = _h.size();
  const std::size_t offs[4] = {0, 1, un, un + 1};
  for (std::size_t c = 0; c < cells; ++c) {
    acc[c] = z[c * 2];
    if (acc2)
      acc2[c] = z[c * 2 + 1];
  }
  // Each ghost adds its weight times its value to the cells it reads, in the table's order.
  for (const ewave_ghost &g : _ghosts) {
    const double vr = z[std::size_t(g.cell) * 2], vi = z[std::size_t(g.cell) * 2 + 1];
    for (int k = 0; k < 4; ++k) {
      if (g.w[k] == 0.0)
        continue;
      acc[g.corner + offs[k]] += g.w[k] * vr;
      if (acc2)
        acc2[g.corner + offs[k]] += g.w[k] * vi;
    }
  }
}

void ewave::set_state(const std::vector<double> &h, const std::vector<double> &phi) {
  if (h.size() != _h.size() || phi.size() != _phi.size())
    throw std::invalid_argument("ewave: the state must have n x n heights and potentials");
  for (std::size_t c = 0; c < h.size(); ++c)
    if (!finite(h[c]) || !finite(phi[c]))
      throw std::invalid_argument("ewave: the state must be finite");
  _h = h;
  _phi = phi;
}

void ewave::add_height(double x, double y, double amount, double radius) {
  if (!finite(x) || !finite(y) || !finite(amount) || !(radius > 0.0) || !finite(radius))
    throw std::invalid_argument("ewave: add_height needs a finite point and amount and a radius "
                                "above 0");
  const int n = _p.n;
  const double fx = (x - _p.origin_x) / _p.dx, fy = (y - _p.origin_y) / _p.dx, fr = radius / _p.dx;
  // The cells within the radius, clipped to the window (sources do not wrap).
  const double lo_x = std::max(std::ceil(fx - fr), 0.0),
               hi_x = std::min(std::floor(fx + fr), n - 1.0);
  const double lo_y = std::max(std::ceil(fy - fr), 0.0),
               hi_y = std::min(std::floor(fy + fr), n - 1.0);
  if (lo_x > hi_x || lo_y > hi_y)
    return;
  const double r2 = radius * radius;
  for (int j = int(lo_y); j <= int(hi_y); ++j)
    for (int i = int(lo_x); i <= int(hi_x); ++i) {
      const double ddx = x_of(i) - x, ddy = y_of(j) - y;
      const double w = 1.0 - (ddx * ddx + ddy * ddy) / r2;
      if (w <= 0.0)
        continue;
      const std::size_t c = std::size_t(j) * std::size_t(n) + std::size_t(i);
      _h[c] += amount * (w * w * w) * _mask[c];
    }
}

void ewave::add_heights(const std::vector<double> &dh) {
  if (dh.size() != _h.size())
    throw std::invalid_argument("ewave: add_heights needs n x n values");
  for (double v : dh)
    if (!finite(v))
      throw std::invalid_argument("ewave: add_heights needs finite values");
  for (std::size_t c = 0; c < dh.size(); ++c)
    _h[c] += dh[c] * _mask[c];
}

void ewave::prepare(double dt) {
  const int n = _p.n;
  const std::size_t cells = _h.size();
  // The inverse FFT is unnormalized: fold its 1 / n^2 (a power of two, so exact) into the
  // coefficients.
  const double inv = 1.0 / (double(n) * double(n));
  _cf.resize(cells * 4);
  for (std::size_t c = 0; c < cells; ++c) {
    double *a = _cf.data() + c * 4;
    const double damp = det::exp(-(_p.damping_k2 * _k2[c] + _p.damping) * dt) * inv;
    const double kap = _kappa[c];
    if (!(kap > 0.0)) {
      // k = 0: the mean height stays; the mean potential is held at 0 (the header says why).
      a[0] = damp;
      a[1] = a[2] = a[3] = 0.0;
      continue;
    }
    const double w = std::sqrt(_p.gravity * kap), ph = w * dt;
    const double cs = det::cos(ph), sn = det::sin(ph);
    a[0] = cs * damp;
    a[1] = (kap / w) * sn * damp;
    a[2] = -(_p.gravity / w) * sn * damp;
    a[3] = cs * damp;
  }
  _dt = dt;
}

void ewave::prepare_symmetric(double dt) {
  const int n = _p.n;
  const std::size_t cells = _h.size();
  const double inv = 1.0 / (double(n) * double(n));
  _sym_n = ewave_substeps(_p, dt);
  _sym_dts = dt / double(_sym_n);
  const double g = _p.gravity, dts = _sym_dts, scale = (4.0 / (g * (dts * dts))) * inv;
  _gk.resize(cells);
  for (std::size_t c = 0; c < cells; ++c) {
    const double kap = _kappa[c];
    if (!(kap > 0.0)) {
      _gk[c] = 0.0;
      continue;
    }
    // kappa~ = 4 sin^2(omega dt_s / 2) / (g dt_s^2): Stormer-Verlet then has omega's frequency.
    const double sn = det::sin(0.5 * (std::sqrt(g * kap) * dts));
    _gk[c] = (sn * sn) * scale;
  }
  // damping_k2 acts on h at twice its rate (see step()).
  if (_p.damping_k2 > 0.0) {
    _fk.resize(cells);
    for (std::size_t c = 0; c < cells; ++c)
      _fk[c] = det::exp(-2.0 * _p.damping_k2 * _k2[c] * dt) * inv;
  } else {
    _fk.clear();
  }
  _sym_damp = _p.damping > 0.0 ? det::exp(-_p.damping * dt) : 1.0;
  _sym_dt = dt;
}

void ewave::update_real(double dt) {
  if (!_real_stale && _real_dt == dt)
    return;
  const int n = _p.n;
  const std::size_t un = std::size_t(n);
  _real.resize(_h.size());
  _real_ones = true;
  const double w = _p.sponge_cells;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::size_t c = std::size_t(j) * un + std::size_t(i);
      // A rate per cell, applied as exp(-rate dt): the same damping per second at any dt.
      const double m = _masked ? _mask[c] : 1.0;
      double rate = m < 1.0 ? _p.beach_rate * (1.0 - m) : 0.0;
      const double d = double(std::min(std::min(i, n - 1 - i), std::min(j, n - 1 - j)));
      if (w > 0.0 && d < w) {
        const double r = (w - d) / w;
        rate += _p.sponge_rate * (r * r);
      }
      const double f = m <= 0.0 ? 0.0 : rate > 0.0 ? det::exp(-rate * dt) : 1.0;
      _real[c] = f;
      _real_ones = _real_ones && f == 1.0;
    }
  _real_stale = false;
  _real_dt = dt;
}

int ewave::substeps(double dt) const { return _ghosts.empty() ? 1 : ewave_substeps(_p, dt); }

void ewave::step(double dt) {
  if (!(dt > 0.0) || !finite(dt))
    throw std::invalid_argument("ewave: step needs a dt above 0 and finite");
  update_real(dt);
  if (_ghosts.empty())
    exact_step(dt);
  else
    symmetric_step(dt);
  _time += dt;
}

void ewave::exact_step(double dt) {
  if (dt != _dt)
    prepare(dt);
  const int n = _p.n;
  const std::size_t un = std::size_t(n), cells = _h.size();

  double *z = _z.data();
  for (std::size_t c = 0; c < cells; ++c) {
    z[c * 2] = _h[c];
    z[c * 2 + 1] = _phi[c];
  }
  _fft.run_2d(z, -1);

  // Each k with its mirror -k: Z(k) = H(k) + i PHI(k) and Z(-k) = conj H(k) + i conj PHI(k), as h
  // and phi are real.
  for (int j = 0; j < n; ++j) {
    const int mj = (n - j) & (n - 1);
    for (int i = 0; i < n; ++i) {
      const int mi = (n - i) & (n - 1);
      const std::size_t c = std::size_t(j) * un + std::size_t(i),
                        m = std::size_t(mj) * un + std::size_t(mi);
      if (m < c)
        continue; // done with its mirror
      const double zr = z[c * 2], zi = z[c * 2 + 1], wr = z[m * 2], wi = z[m * 2 + 1];
      const double hr = 0.5 * (zr + wr), hi = 0.5 * (zi - wi);
      const double pr = 0.5 * (zi + wi), pi = 0.5 * (wr - zr);
      const double *a = _cf.data() + c * 4;
      const double hr2 = a[0] * hr + a[1] * pr, hi2 = a[0] * hi + a[1] * pi;
      const double pr2 = a[2] * hr + a[3] * pr, pi2 = a[2] * hi + a[3] * pi;
      z[c * 2] = hr2 - pi2;
      z[c * 2 + 1] = hi2 + pr2;
      if (m != c) {
        z[m * 2] = hr2 + pi2;
        z[m * 2 + 1] = pr2 - hi2;
      }
    }
  }
  _fft.run_2d(z, +1);

  if (_real_ones) {
    for (std::size_t c = 0; c < cells; ++c) {
      _h[c] = z[c * 2];
      _phi[c] = z[c * 2 + 1];
    }
  } else {
    for (std::size_t c = 0; c < cells; ++c) {
      _h[c] = z[c * 2] * _real[c];
      _phi[c] = z[c * 2 + 1] * _real[c];
    }
  }
}

void ewave::symmetric_step(double dt) {
  if (dt != _sym_dt)
    prepare_symmetric(dt);
  const std::size_t cells = _h.size();
  double *z = _z.data(), *h = _h.data(), *phi = _phi.data();
  const double *im = _inv_mass.data();
  // The walls hold no state.
  for (std::size_t c = 0; c < cells; ++c)
    if (!(_mask[c] > 0.0))
      h[c] = phi[c] = 0.0;

  const double dts = _sym_dts, half = 0.5 * (_p.gravity * dts);
  for (int s = 0; s < _sym_n; ++s) {
    // phi_half, in place (0 on walls), extended by R.
    for (std::size_t c = 0; c < cells; ++c) {
      phi[c] -= half * h[c];
      z[c * 2] = phi[c];
      z[c * 2 + 1] = 0.0;
    }
    fill(z, phi, nullptr);
    // G~ R phi_half.
    _fft.run_2d(z, -1);
    for (std::size_t c = 0; c < cells; ++c) {
      z[c * 2] *= _gk[c];
      z[c * 2 + 1] *= _gk[c];
    }
    _fft.run_2d(z, +1);
    // S phi_half = R^T G~ R phi_half; then h and phi.
    fold(z, _acc.data(), nullptr);
    for (std::size_t c = 0; c < cells; ++c) {
      if (!(im[c] > 0.0))
        continue;
      h[c] += dts * (im[c] * _acc[c]);
      phi[c] -= half * h[c];
    }
  }

  // damping_k2: h <- M^-1 R^T F R h, F = exp(-2 gamma |k|^2 dt): self-adjoint under M with its
  // eigenvalues in [0, 1], so it only takes potential energy. Twice the rate, on h alone, takes a
  // mode's energy at the exact step's mean rate (half of it is potential).
  if (!_fk.empty()) {
    for (std::size_t c = 0; c < cells; ++c) {
      z[c * 2] = h[c];
      z[c * 2 + 1] = 0.0;
    }
    fill(z, h, nullptr);
    _fft.run_2d(z, -1);
    for (std::size_t c = 0; c < cells; ++c) {
      z[c * 2] *= _fk[c];
      z[c * 2 + 1] *= _fk[c];
    }
    _fft.run_2d(z, +1);
    fold(z, _acc.data(), nullptr);
    for (std::size_t c = 0; c < cells; ++c)
      h[c] = im[c] * _acc[c];
  }

  // The real-space pass: the beach and the sponge on h, at twice their rate (a diagonal factor
  // under the diagonal M only takes potential energy; one on phi need not, as S is not
  // diagonal), the global damping on both (a uniform factor scales H).
  if (_real_ones && _sym_damp == 1.0)
    return;
  const double gd = _sym_damp;
  for (std::size_t c = 0; c < cells; ++c) {
    const double f = _real[c];
    h[c] *= (f * f) * gd;
    phi[c] = f > 0.0 ? phi[c] * gd : 0.0;
  }
}

double ewave::at(double x, double y) const {
  const int n = _p.n;
  const double fx = (x - _p.origin_x) / _p.dx, fy = (y - _p.origin_y) / _p.dx;
  if (!(fx >= 0.0) || !(fy >= 0.0) || !(fx <= n - 1.0) || !(fy <= n - 1.0))
    return 0.0;
  const int i = std::min(int(fx), n - 2), j = std::min(int(fy), n - 2);
  const double tx = fx - i, ty = fy - j;
  const std::size_t un = std::size_t(n), c = std::size_t(j) * un + std::size_t(i);
  const double h00 = _h[c], h10 = _h[c + 1], h01 = _h[c + un], h11 = _h[c + un + 1];
  return (h00 * (1.0 - tx) + h10 * tx) * (1.0 - ty) + (h01 * (1.0 - tx) + h11 * tx) * ty;
}

double ewave::potential_energy() const {
  double s = 0.0;
  if (!_ghosts.empty()) {
    for (std::size_t c = 0; c < _h.size(); ++c)
      if (_mask[c] > 0.0)
        s += _mass[c] * (_h[c] * _h[c]);
  } else {
    for (std::size_t c = 0; c < _h.size(); ++c)
      if (_mask[c] > 0.0) // the water's: no walls
        s += _h[c] * _h[c];
  }
  return 0.5 * _p.density * _p.gravity * s * (_p.dx * _p.dx);
}

double ewave::kinetic_energy() const {
  const int n = _p.n;
  const std::size_t un = std::size_t(n), cells = _h.size();
  if (!_ghosts.empty()) {
    // 0.5 rho phi.S.phi dA = 0.5 rho (R phi).G.(R phi) dA, G built on kappa, over the water and
    // the ghosts.
    std::vector<double> z(cells * 2, 0.0), p(cells, 0.0);
    for (std::size_t c = 0; c < cells; ++c)
      if (_mask[c] > 0.0)
        p[c] = _phi[c];
    for (std::size_t c = 0; c < cells; ++c)
      z[c * 2] = p[c];
    fill(z.data(), p.data(), nullptr);
    std::vector<double> r(cells);
    for (std::size_t c = 0; c < cells; ++c)
      r[c] = z[c * 2];
    _fft.run_2d(z.data(), -1);
    const double inv = 1.0 / (double(n) * double(n));
    for (std::size_t c = 0; c < cells; ++c) {
      z[c * 2] *= _kappa[c] * inv;
      z[c * 2 + 1] *= _kappa[c] * inv;
    }
    _fft.run_2d(z.data(), +1);
    double s = 0.0;
    for (std::size_t c = 0; c < cells; ++c)
      s += r[c] * z[c * 2];
    return 0.5 * _p.density * s * (_p.dx * _p.dx);
  }
  std::vector<double> z(cells * 2);
  for (std::size_t c = 0; c < cells; ++c) {
    z[c * 2] = _h[c];
    z[c * 2 + 1] = _phi[c];
  }
  _fft.run_2d(z.data(), -1);
  double s = 0.0;
  for (int j = 0; j < n; ++j) {
    const int mj = (n - j) & (n - 1);
    for (int i = 0; i < n; ++i) {
      const int mi = (n - i) & (n - 1);
      const std::size_t c = std::size_t(j) * un + std::size_t(i),
                        m = std::size_t(mj) * un + std::size_t(mi);
      const double pr = 0.5 * (z[c * 2 + 1] + z[m * 2 + 1]), pi = 0.5 * (z[m * 2] - z[c * 2]);
      s += _kappa[c] * (pr * pr + pi * pi);
    }
  }
  return 0.5 * _p.density * s * (_p.dx * _p.dx) / (double(n) * double(n));
}

std::uint64_t ewave::digest() const {
  digester d;
  d.add(std::int64_t(_p.n));
  for (double v : {_p.dx, _p.origin_x, _p.origin_y, _p.depth, _p.gravity, _p.density, _p.damping_k2,
                   _p.damping, _p.sponge_cells, _p.sponge_rate, _p.beach_rate, _time})
    d.add(v);
  if (_p.walls != ewave_walls::zero) { // zero walls hash as O8a did
    d.add(std::int64_t(_p.walls));
    d.add(std::int64_t(_p.ghost_cells));
  }
  d.add(_mask);
  d.add(_h);
  d.add(_phi);
  return d.h;
}

} // namespace ocean
} // namespace cvc
