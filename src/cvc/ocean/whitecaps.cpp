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

// whitecaps.cpp — the open sea's foam coverage and Monahan's threshold (see whitecaps.h).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/ocean/fft.h>
#include <cvc/ocean/whitecaps.h>
#include <stdexcept>

namespace cvc {
namespace ocean {

double monahan_coverage(double u10_mps) {
  if (!(u10_mps > 0.0))
    return 0.0;
  return std::min(3.84e-6 * std::pow(u10_mps, 3.41), 0.16);
}

namespace {

// One cascade's Jacobian at time t, n x n row-major (row y, column x), as OceanFFT computes it:
// the time evolution's DxDx, DzDz and DxDz (its buffer 1), transformed with the sign OceanFFT's
// butterflies run, the centred wavenumbers' (-1)^(x + y), then (1 + l Dxx)(1 + l Dzz) - (l Dxz)^2.
std::vector<float> cascade_jacobian(const h0_field &h, const fft_plan &plan, const foam_params &p,
                                    double t) {
  const int n = h.n;
  const std::size_t nn = std::size_t(n) * std::size_t(n);
  // Two transforms: Dxx + i Dzz (both real fields, packed as OceanFFT packs them) and Dxz.
  std::vector<double> a(2 * nn, 0.0), b(2 * nn, 0.0);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      // The Nyquist row and column have no partner -k, and k = 0 moves nothing (OceanFFT's).
      if (c == 0 || r == 0)
        continue;
      const double kx = texel_k(c, n, h.tile_m), ky = texel_k(r, n, h.tile_m);
      const double k = std::hypot(kx, ky);
      if (k < 1e-6)
        continue;
      const float *q = h.texel(c, r);
      const double w = std::sqrt(p.gravity * k * std::tanh(std::min(k * p.depth_m, 20.0)));
      const double cs = std::cos(w * t), sn = std::sin(w * t);
      // h = h0(k) e^(i w t) + conj h0(-k) e^(-i w t)
      const double hr = q[0] * cs - q[1] * sn + q[2] * cs + q[3] * sn;
      const double hi = q[0] * sn + q[1] * cs - q[2] * sn + q[3] * cs;
      const double sxx = -kx * kx / k, szz = -ky * ky / k, sxz = -kx * ky / k;
      const std::size_t i = 2 * (std::size_t(r) * std::size_t(n) + std::size_t(c));
      // (sxx h) + i (szz h)
      a[i] = sxx * hr - szz * hi;
      a[i + 1] = sxx * hi + szz * hr;
      b[i] = sxz * hr;
      b[i + 1] = sxz * hi;
    }
  }
  plan.run_2d(a.data(), +1);
  plan.run_2d(b.data(), +1);
  std::vector<float> j(nn);
  const double l = p.chop;
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x) {
      const std::size_t i = std::size_t(y) * std::size_t(n) + std::size_t(x);
      const double s = ((x + y) & 1) ? -1.0 : 1.0;
      const double dxx = s * a[2 * i], dzz = s * a[2 * i + 1], dxz = s * b[2 * i];
      j[i] = float((1.0 + l * dxx) * (1.0 + l * dzz) - (l * dxz) * (l * dxz));
    }
  return j;
}

// A field sampled as the vertex shader samples it (oDisp, linear filtering, repeat): at world
// (x, y) the texel coordinate is (x, y) n / tile, texel centres on whole numbers.
double sample(const std::vector<float> &f, int n, double tile_m, double x, double y) {
  const double u = x * double(n) / tile_m, v = y * double(n) / tile_m;
  const double fu = std::floor(u), fv = std::floor(v);
  const double tu = u - fu, tv = v - fv;
  auto wrap = [n](double i) {
    long long m = static_cast<long long>(i) % n;
    return int(m < 0 ? m + n : m);
  };
  const int x0 = wrap(fu), x1 = (x0 + 1) % n, y0 = wrap(fv), y1 = (y0 + 1) % n;
  auto at = [&](int xi, int yi) { return double(f[std::size_t(yi) * std::size_t(n) + xi]); };
  return (1.0 - tv) * ((1.0 - tu) * at(x0, y0) + tu * at(x1, y0)) +
         tv * ((1.0 - tu) * at(x0, y1) + tu * at(x1, y1));
}

double smooth01(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

// n - 0.5 at 16 quantiles (the midpoints of 16 equal shares) of the fray noise's distribution,
// n = 0.65 v1 + 0.35 v2, each v a smoothstep-weighted bilinear mix of four independent uniforms
// at an independent uniform point in its cell: 4096 draws from a fixed xorshift.
std::vector<double> fray_quantiles() {
  std::uint64_t x = 0x9e3779b97f4a7c15ull;
  auto uni = [&x] {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    return double(x >> 11) / 9007199254740992.0;
  };
  auto value_noise = [&] {
    const double fx = smooth01(uni()), fy = smooth01(uni());
    const double a = uni(), b = uni(), c = uni(), d = uni();
    const double lo = a + (b - a) * fx, hi = c + (d - c) * fx;
    return lo + (hi - lo) * fy;
  };
  std::vector<double> n(4096);
  for (double &v : n)
    v = 0.65 * value_noise() + 0.35 * value_noise();
  std::sort(n.begin(), n.end());
  std::vector<double> q(16);
  for (std::size_t i = 0; i < q.size(); ++i)
    q[i] = n[(2 * i + 1) * n.size() / (2 * q.size())] - 0.5;
  return q;
}

} // namespace

foam_field::foam_field(const std::vector<h0_field> &cascades, const foam_params &p)
    : _ramp(p.ramp), _fray(p.fray) {
  if (_fray > 0.0)
    _fray_q = fray_quantiles();
  if (cascades.empty())
    throw std::invalid_argument("foam_field: no cascades");
  if (!(p.spacing_m > 0.0) || !(p.ramp > 0.0) || p.times_s.empty())
    throw std::invalid_argument("foam_field: spacing_m and ramp must be > 0, times_s not empty");
  double side = 0.0;
  for (const h0_field &h : cascades) {
    if (h.n < 2 || !(h.tile_m > 0.0) || h.texels.size() != std::size_t(h.n) * h.n * 4)
      throw std::invalid_argument("foam_field: an empty or malformed cascade");
    side = std::max(side, h.tile_m);
  }
  if (p.mesh_m < 0.0 || p.mesh_samples < 1)
    throw std::invalid_argument("foam_field: mesh_m must be >= 0 and mesh_samples >= 1");
  // The lattice spans the largest tile exactly, so it holds whole periods of its waves.
  const double step = p.mesh_m > 0.0 ? p.mesh_m : p.spacing_m;
  const int m = std::max(1, int(std::lround(side / step)));
  const double spacing = p.mesh_m > 0.0 ? p.mesh_m : side / double(m);
  const int q = p.mesh_m > 0.0 ? p.mesh_samples : 1;
  _j.reserve(std::size_t(m) * std::size_t(m) * std::size_t(q) * q * p.times_s.size());
  for (double t : p.times_s) {
    std::vector<std::vector<float>> fields;
    for (const h0_field &h : cascades)
      fields.push_back(cascade_jacobian(h, fft_plan(h.n), p, t));
    auto least_at = [&](double x, double y) {
      double least = 1e300;
      for (std::size_t c = 0; c < cascades.size(); ++c)
        least = std::min(least, sample(fields[c], cascades[c].n, cascades[c].tile_m, x, y));
      return least;
    };
    if (p.mesh_m <= 0.0) {
      for (int iy = 0; iy < m; ++iy)
        for (int ix = 0; ix < m; ++ix)
          _j.push_back(float(least_at(ix * spacing, iy * spacing)));
      continue;
    }
    // The mesh: J at its vertices, each cell two triangles split as lsystem_coast's ocean grid is
    // (v, v + 1, v + row) and (v + 1, v + row + 1, v + row), J linear over each, q x q points.
    std::vector<double> v(std::size_t(m + 1) * std::size_t(m + 1));
    for (int iy = 0; iy <= m; ++iy)
      for (int ix = 0; ix <= m; ++ix)
        v[std::size_t(iy) * (m + 1) + ix] = least_at(ix * spacing, iy * spacing);
    for (int iy = 0; iy < m; ++iy)
      for (int ix = 0; ix < m; ++ix) {
        const std::size_t i00 = std::size_t(iy) * (m + 1) + ix;
        const double j00 = v[i00], j10 = v[i00 + 1], j01 = v[i00 + m + 1], j11 = v[i00 + m + 2];
        for (int sy = 0; sy < q; ++sy)
          for (int sx = 0; sx < q; ++sx) {
            const double u = (sx + 0.5) / q, w = (sy + 0.5) / q;
            const double j = u + w <= 1.0 ? j00 + u * (j10 - j00) + w * (j01 - j00)
                                          : j11 + (1.0 - u) * (j01 - j11) + (1.0 - w) * (j10 - j11);
            _j.push_back(float(j));
          }
      }
  }
  double sum = 0.0;
  for (float j : _j)
    sum += j;
  _mean = sum / double(_j.size());
  std::sort(_j.begin(), _j.end());
  _area.assign(_j.size() + 1, 0.0);
  for (std::size_t i = 0; i < _j.size(); ++i)
    _area[i + 1] = _area[i] + std::abs(double(_j[i]));
}

double foam_field::coverage(double bias) const {
  if (_fray_q.empty())
    return coverage_at(bias);
  // The fray shifts each fragment's J by fray (n - 0.5): the same as shifting the bias.
  double sum = 0.0;
  for (double q : _fray_q)
    sum += coverage_at(bias + _fray * q);
  return sum / double(_fray_q.size());
}

double foam_field::white(double bias, double level) const {
  if (_area.back() <= 0.0)
    return 0.0;
  level = std::clamp(level, 1e-9, 1.0);
  // smoothstep(x) = level at x = 0.5 - sin(asin(1 - 2 level) / 3), so foam >= level where
  // J <= bias - ramp x.
  const double x = 0.5 - std::sin(std::asin(1.0 - 2.0 * level) / 3.0);
  const std::vector<double> none{0.0};
  const std::vector<double> &qs = _fray_q.empty() ? none : _fray_q;
  double sum = 0.0;
  for (double q : qs) {
    const float t = float(bias + _fray * q - _ramp * x);
    sum += _area[std::size_t(std::upper_bound(_j.begin(), _j.end(), t) - _j.begin())];
  }
  return sum / (double(qs.size()) * _area.back());
}

double foam_field::coverage_at(double bias) const {
  // Each sample weighs its share of the drawn surface, |J| (the choppy map's area ratio). Those
  // at or below bias - ramp are white; only the ramp's window is summed one by one.
  if (_area.back() <= 0.0)
    return 0.0;
  const std::size_t i0 =
      std::size_t(std::upper_bound(_j.begin(), _j.end(), float(bias - _ramp)) - _j.begin());
  double sum = _area[i0];
  for (std::size_t i = i0; i < _j.size() && double(_j[i]) < bias; ++i)
    sum += smooth01((bias - double(_j[i])) / _ramp) * std::abs(double(_j[i]));
  return sum / _area.back();
}

double foam_field::bias_for(double share) const {
  share = std::clamp(share, 0.0, 1.0);
  double lo = least() - 0.5 * _fray, hi = double(_j.back()) + _ramp + 0.5 * _fray;
  if (share <= 0.0)
    return lo;
  for (int i = 0; i < 100 && hi - lo > 1e-9; ++i) {
    const double mid = 0.5 * (lo + hi);
    (coverage(mid) < share ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

double foam_field::below(double j) const {
  const auto it = std::lower_bound(_j.begin(), _j.end(), float(j));
  return double(it - _j.begin()) / double(_j.size());
}

} // namespace ocean
} // namespace cvc
