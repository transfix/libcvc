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
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/ocean/fft.h>
#include <cvc/ocean/whitecaps.h>
#include <initializer_list>
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

// `count` quantiles (the midpoints of equal shares) of the distribution of n = sum of w_i v_i, each
// v a smoothstep-weighted bilinear mix of four independent uniforms at an independent uniform
// point in its cell: 4096 draws from a fixed xorshift started at `seed`.
std::vector<double> noise_quantiles(std::uint64_t seed, std::initializer_list<double> w,
                                    std::size_t count) {
  std::uint64_t x = seed;
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
  for (double &v : n) {
    v = 0.0;
    for (double wi : w) // in order: the draws are the fray's as they were with two weights
      v += wi * value_noise();
  }
  std::sort(n.begin(), n.end());
  std::vector<double> q(count);
  for (std::size_t i = 0; i < q.size(); ++i)
    q[i] = n[(2 * i + 1) * n.size() / (2 * q.size())];
  return q;
}

// n - 0.5 at 16 quantiles of the fray noise's distribution, n = 0.65 v1 + 0.35 v2.
std::vector<double> fray_quantiles() {
  std::vector<double> q = noise_quantiles(0x9e3779b97f4a7c15ull, {0.65, 0.35}, 16);
  for (double &v : q)
    v -= 0.5;
  return q;
}

// The windrow noise's distribution (foam_look's w = 0.45 v1 + 0.3 v2 + 0.25 v3) at 64 quantiles.
std::vector<double> windrow_quantiles() {
  return noise_quantiles(0x5851f42d4c957f2dull, {0.45, 0.30, 0.25}, 64);
}

// A texel of an n-texel tile at world coordinate x, as the shaders pick it: the nearest, texel i
// holding the field at i tile / n.
int tile_texel(double x, int n, double tile_m) {
  const long long i = static_cast<long long>(std::floor(x * double(n) / tile_m + 0.5)) % n;
  return int(i < 0 ? i + n : i);
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

// ── persistent foam ─────────────────────────────────────────────────────────────────────────────

double foam_inject(double j, double bias, double ramp) { return smooth01((bias - j) / ramp); }

void foam_step(foam_state &s, double inject, double dt_s, const foam_persistence &p) {
  const double dt = std::clamp(dt_s, 0.0, std::max(p.max_dt_s, 0.0));
  inject = std::clamp(inject, 0.0, 1.0);
  auto keep = [dt](double tau) { return tau > 0.0 ? std::exp(-dt / tau) : 0.0; };
  const double gain = inject * std::max(p.build_per_s, 0.0) * dt;
  s.foam = std::clamp(s.foam * keep(p.decay_s) + gain, 0.0, 1.0);
  s.bubbles = std::clamp(s.bubbles * keep(p.bubble_decay_s) + gain * p.bubble_gain, 0.0, 1.0);
  s.crest = inject;
  const double x = std::clamp((inject - 0.45) / 0.5, 0.0, 1.0);
  s.spray = std::max(s.spray * keep(p.spray_decay_s), x * x * (3.0 - 2.0 * x));
}

double foam_look(double foam, double windrow) {
  return smooth01((foam * (0.1 + 1.6 * windrow) - 0.35) / 0.3);
}

foam_history::foam_history(const std::vector<h0_field> &cascades, const foam_params &p,
                           const foam_persistence &q, const foam_history_params &h)
    : _h(cascades), _p(p), _q(q), _hp(h) {
  if (cascades.empty())
    throw std::invalid_argument("foam_history: no cascades");
  if (!(p.spacing_m > 0.0) || !(p.ramp > 0.0) || p.mesh_m < 0.0 || p.mesh_samples < 1)
    throw std::invalid_argument("foam_history: spacing_m and ramp must be > 0, mesh_m >= 0");
  if (!(q.decay_s > 0.0) || !(q.build_per_s >= 0.0) || q.cascades < 1 || !(q.share >= 0.0) ||
      !(q.share < 1.0))
    throw std::invalid_argument(
        "foam_history: decay_s > 0, build_per_s >= 0, cascades >= 1 and share in [0, 1)");
  if (!(h.step_s > 0.0) || !(h.dt_s > 0.0) || h.samples_s.empty())
    throw std::invalid_argument("foam_history: step_s and dt_s must be > 0, samples_s not empty");
  for (double s : h.samples_s)
    if (!(s >= 0.0))
      throw std::invalid_argument("foam_history: a sample before the window's end");
  for (const h0_field &c : cascades) {
    if (c.n < 2 || !(c.tile_m > 0.0) || c.texels.size() != std::size_t(c.n) * c.n * 4)
      throw std::invalid_argument("foam_history: an empty or malformed cascade");
    _side = std::max(_side, c.tile_m);
  }
  _np = std::min(cascades.size(), std::size_t(q.cascades));
  _window = h.window_s > 0.0 ? h.window_s : 2.0 * q.decay_s;
  _end = _window + *std::max_element(h.samples_s.begin(), h.samples_s.end());
  _j.resize(_np);
  _jmin.resize(_np);
  for (std::size_t c = 0; c < _np; ++c) {
    _step.push_back(h.step_s * std::sqrt(cascades[c].tile_m / _side));
    const std::size_t evals = std::size_t(std::ceil(_end / _step.back())) + 1;
    _j[c].resize(evals);
    for (std::size_t e = 0; e < evals; ++e)
      _units.push_back({c, e, false});
  }
  _js.assign(h.samples_s.size(), std::vector<std::vector<float>>(cascades.size()));
  for (std::size_t s = 0; s < h.samples_s.size(); ++s)
    for (std::size_t c = 0; c < cascades.size(); ++c)
      _units.push_back({c, s, true});
  if (p.fray > 0.0)
    _fray_q = fray_quantiles();
  const std::vector<double> w = windrow_quantiles();
  _look.resize(257);
  for (std::size_t i = 0; i < _look.size(); ++i) {
    double sum = 0.0;
    for (double v : w)
      sum += foam_look(double(i) / 256.0, v);
    _look[i] = sum / double(w.size());
  }
}

bool foam_history::step(std::size_t n) {
  std::size_t todo = units() - _done;
  if (n > 0)
    todo = std::min(todo, n);
  for (; todo > 0; --todo, ++_done) {
    const unit u = _units[_done];
    const h0_field &h = _h[u.cascade];
    if (u.sample) {
      const double t = _hp.t0_s + _window + _hp.samples_s[u.index];
      _js[u.index][u.cascade] = cascade_jacobian(h, fft_plan(h.n), _p, t);
    } else {
      const double t = _hp.t0_s + double(u.index) * _step[u.cascade];
      std::vector<std::vector<float>> &js = _j[u.cascade];
      js[u.index] = cascade_jacobian(h, fft_plan(h.n), _p, t);
      if (u.index + 1 == js.size()) { // the history is whole: each texel's least J in it
        std::vector<float> &m = _jmin[u.cascade];
        m = js[0];
        for (std::size_t k = 1; k < js.size(); ++k)
          for (std::size_t i = 0; i < m.size(); ++i)
            m[i] = std::min(m[i], js[k][i]);
      }
    }
    if (_done + 1 == units())
      lattice();
  }
  return ready();
}

void foam_history::lattice() {
  _dt = std::min(_hp.dt_s, std::max(_q.max_dt_s, 1e-6));
  const std::size_t ns = _hp.samples_s.size(), nc = _h.size();
  _at.resize(ns);
  for (std::size_t s = 0; s < ns; ++s)
    _at[s] = std::lround((_window + _hp.samples_s[s]) / _dt);
  // foam_field's lattice: over the largest tile (whole periods of its waves), at its spacing, or
  // the mesh's vertices with q x q points a cell.
  const bool mesh = _p.mesh_m > 0.0;
  const int m = std::max(1, int(std::lround(_side / (mesh ? _p.mesh_m : _p.spacing_m))));
  const double sp = _side / double(m);
  const int q = mesh ? _p.mesh_samples : 1;
  std::vector<std::array<double, 4>> pts; // x, y and, with the mesh, the place in its cell
  for (int iy = 0; iy < m; ++iy)
    for (int ix = 0; ix < m; ++ix)
      for (int sy = 0; sy < q; ++sy)
        for (int sx = 0; sx < q; ++sx) {
          const double u = mesh ? (sx + 0.5) / q : 0.0, w = mesh ? (sy + 0.5) / q : 0.0;
          pts.push_back({(ix + u) * sp, (iy + w) * sp, u, w});
        }
  auto texel_of = [&](std::size_t c, double x, double y) {
    const int n = _h[c].n;
    return std::size_t(tile_texel(y, n, _h[c].tile_m)) * std::size_t(n) +
           std::size_t(tile_texel(x, n, _h[c].tile_m));
  };
  _texel.assign(_np, std::vector<std::uint32_t>(pts.size()));
  for (std::size_t c = 0; c < _np; ++c)
    for (std::size_t i = 0; i < pts.size(); ++i)
      _texel[c][i] = std::uint32_t(texel_of(c, pts[i][0], pts[i][1]));
  _pj.assign(ns * pts.size(), 1.0f);
  _area.assign(ns * pts.size(), 1.0f);
  _area_sum = 0.0;
  std::vector<double> v(std::size_t(m + 1) * std::size_t(m + 1));
  for (std::size_t s = 0; s < ns; ++s) {
    auto least_at = [&](double x, double y) {
      double least = 1e300;
      for (std::size_t c = 0; c < nc; ++c)
        least = std::min(least, double(_js[s][c][texel_of(c, x, y)]));
      return least;
    };
    float *out = &_pj[s * pts.size()];
    if (mesh) {
      for (int iy = 0; iy <= m; ++iy)
        for (int ix = 0; ix <= m; ++ix)
          v[std::size_t(iy) * std::size_t(m + 1) + std::size_t(ix)] = least_at(ix * sp, iy * sp);
    }
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (!mesh) {
        out[i] = float(least_at(pts[i][0], pts[i][1]));
      } else {
        // J linear over each cell's two triangles, split as the ocean grids are:
        // (v, v + 1, v + row) and (v + 1, v + row + 1, v + row).
        const std::size_t cell = i / std::size_t(q * q);
        const std::size_t i00 =
            (cell / std::size_t(m)) * std::size_t(m + 1) + cell % std::size_t(m);
        const double j00 = v[i00], j10 = v[i00 + 1], j01 = v[i00 + std::size_t(m) + 1],
                     j11 = v[i00 + std::size_t(m) + 2];
        const double u = pts[i][2], w = pts[i][3];
        out[i] = float(u + w <= 1.0 ? j00 + u * (j10 - j00) + w * (j01 - j00)
                                    : j11 + (1.0 - u) * (j01 - j11) + (1.0 - w) * (j10 - j11));
      }
      _area[s * pts.size() + i] = std::abs(out[i]);
      _area_sum += std::abs(double(out[i]));
    }
  }
  _js.clear(); // the lattice holds what the samples are needed for
  _js.shrink_to_fit();
}

std::vector<float> foam_history::looks(double bias) const {
  if (!ready())
    throw std::logic_error("foam_history: the Jacobians are not all evaluated (step)");
  const std::size_t ns = _hp.samples_s.size();
  const std::size_t np = _texel.empty() ? 0 : _texel[0].size();
  const long steps = *std::max_element(_at.begin(), _at.end());
  const double keep = std::exp(-_dt / _q.decay_s), gain = std::max(_q.build_per_s, 0.0) * _dt;
  std::vector<float> fz(ns * np, 0.0f); // the stronger long cascade's foam at each point
  for (std::size_t c = 0; c < _np; ++c) {
    const std::size_t nn = std::size_t(_h[c].n) * std::size_t(_h[c].n);
    std::vector<float> foam(ns * nn, 0.0f);
    // Where step k reads the cascade's Jacobians: a Catmull-Rom spline through evaluations
    // e - 1 .. e + 2 (clamped at the ends). Linear interpolation flattens each fold's dip: with
    // ten evaluations to a wave's period it missed a quarter of the aging foam.
    const std::size_t steps1 = std::size_t(steps) + 1, last = _j[c].size() - 1;
    std::vector<std::array<std::size_t, 4>> e(steps1);
    std::vector<std::array<double, 4>> w(steps1);
    for (std::size_t k = 0; k < steps1; ++k) {
      const double u = double(k) * _dt / _step[c];
      const std::size_t e0 = std::min(std::size_t(u), last - 1);
      const double t = std::min(1.0, u - double(e0)), t2 = t * t, t3 = t2 * t;
      e[k] = {e0 > 0 ? e0 - 1 : 0, e0, e0 + 1, std::min(e0 + 2, last)};
      w[k] = {0.5 * (-t3 + 2.0 * t2 - t), 0.5 * (3.0 * t3 - 5.0 * t2 + 2.0),
              0.5 * (-3.0 * t3 + 4.0 * t2 + t), 0.5 * (t3 - t2)};
    }
    // foam_step's foam channel, texel by texel. A texel takes foam only near the evaluations
    // where its J comes within 0.05 of the bias (the spline may dip a little under them): it is
    // integrated from two evaluations before the first such to two after the last, and decays
    // exactly after that; a texel with none never takes any.
    const std::vector<std::vector<float>> &js = _j[c];
    const double per = _step[c] / _dt; // integration steps an evaluation
    std::vector<double> decay(steps1 + 1, 1.0);
    for (std::size_t k = 1; k < decay.size(); ++k)
      decay[k] = decay[k - 1] * keep;
    for (std::size_t i = 0; i < nn; ++i) {
      if (!(double(_jmin[c][i]) - 0.05 < bias))
        continue;
      std::size_t first = last + 1, final = 0;
      for (std::size_t k = 0; k <= last; ++k)
        if (double(js[k][i]) < bias + 0.05) {
          first = std::min(first, k);
          final = k;
        }
      if (first > last)
        continue;
      const std::size_t k0 = std::size_t(std::max(0.0, std::floor((double(first) - 2.0) * per)));
      const std::size_t k1 =
          std::min(steps1 - 1, std::size_t(std::ceil((double(final) + 2.0) * per)));
      double v = 0.0;
      for (std::size_t k = k0; k <= k1; ++k) {
        const std::array<std::size_t, 4> &ek = e[k];
        const std::array<double, 4> &wk = w[k];
        const double j = wk[0] * double(js[ek[0]][i]) + wk[1] * double(js[ek[1]][i]) +
                         wk[2] * double(js[ek[2]][i]) + wk[3] * double(js[ek[3]][i]);
        v = std::clamp(v * keep + foam_inject(j, bias, _p.ramp) * gain, 0.0, 1.0);
        for (std::size_t r = 0; r < ns; ++r)
          if (_at[r] == long(k))
            foam[r * nn + i] = float(v);
      }
      for (std::size_t r = 0; r < ns; ++r)
        if (_at[r] > long(k1))
          foam[r * nn + i] = float(v * decay[std::size_t(_at[r]) - k1]);
    }
    for (std::size_t s = 0; s < ns; ++s)
      for (std::size_t i = 0; i < np; ++i)
        fz[s * np + i] = std::max(fz[s * np + i], foam[s * nn + _texel[c][i]]);
  }
  for (float &v : fz) {
    const double u = std::clamp(double(v), 0.0, 1.0) * 256.0;
    const std::size_t i = std::min(std::size_t(u), std::size_t(255));
    v = float(_look[i] + (_look[i + 1] - _look[i]) * (u - double(i)));
  }
  return fz;
}

double foam_history::drawn(double whitecaps, const std::vector<float> *looks) const {
  if (!ready())
    throw std::logic_error("foam_history: the Jacobians are not all evaluated (step)");
  double sum = 0.0;
  for (std::size_t i = 0; i < _pj.size(); ++i) {
    const double fr = _fray_q.empty() ? 0.0 : _p.fray * _fray_q[i % _fray_q.size()];
    double a = smooth01((whitecaps + fr - double(_pj[i])) / _p.ramp);
    if (looks)
      a = std::max(a, double((*looks)[i]));
    sum += a * double(_area[i]);
  }
  return _area_sum > 0.0 ? sum / _area_sum : 0.0;
}

double foam_history::coverage(const foam_biases &b) const {
  const std::vector<float> l = looks(b.persistent);
  return drawn(b.whitecaps, &l);
}

double foam_history::persistent(double bias) const {
  const std::vector<float> l = looks(bias);
  double sum = 0.0;
  for (std::size_t i = 0; i < l.size(); ++i)
    sum += double(l[i]) * double(_area[i]);
  return _area_sum > 0.0 ? sum / _area_sum : 0.0;
}

double foam_history::active(double bias) const { return drawn(bias, nullptr); }

foam_biases foam_history::biases_for(double share) const {
  if (!ready())
    throw std::logic_error("foam_history: the Jacobians are not all evaluated (step)");
  share = std::clamp(share, 0.0, 1.0);
  double least_p = 1e300, least = 1e300;
  for (const std::vector<float> &m : _jmin)
    for (float j : m)
      least_p = std::min(least_p, double(j));
  for (float j : _pj)
    least = std::min(least, double(j));
  least = std::min(least, least_p);
  const double fray = _fray_q.empty() ? 0.0 : _p.fray;
  foam_biases b;
  b.persistent = least_p;
  b.whitecaps = least - 0.5 * fray;
  if (share <= 0.0)
    return b;
  // The aging foam's bias: walk up from the least J until its share is reached, then bisect. (The
  // integration grows with the texels that fold, so the walk keeps the biases tried low.)
  const double target = share * _q.share;
  std::vector<float> l;
  if (target > 0.0) {
    double lo = least_p - 0.05, hi = least_p;
    while (persistent(hi) < target && hi < 2.0) {
      lo = hi;
      hi += 0.05;
    }
    for (int i = 0; i < 9; ++i) {
      const double mid = 0.5 * (lo + hi);
      (persistent(mid) < target ? lo : hi) = mid;
    }
    b.persistent = 0.5 * (lo + hi);
    l = looks(b.persistent);
  }
  // The whitecaps' bias for the rest (the aging foam's looks fixed: cheap).
  double lo = least - 0.5 * fray, hi = 1.5 + _p.ramp + 0.5 * fray;
  for (int i = 0; i < 40 && hi - lo > 1e-6; ++i) {
    const double mid = 0.5 * (lo + hi);
    (drawn(mid, l.empty() ? nullptr : &l) < share ? lo : hi) = mid;
  }
  b.whitecaps = 0.5 * (lo + hi);
  return b;
}

} // namespace ocean
} // namespace cvc
