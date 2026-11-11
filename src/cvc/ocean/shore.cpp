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

// shore.cpp — the shore layer (see shore.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/spectrum.h>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>

namespace cvc {
namespace ocean {

namespace det = ::cvc::algo::det; // the same bits on every platform (det_math.h)

namespace {

constexpr double kTau = 6.283185307179586;
constexpr double kInf = std::numeric_limits<double>::infinity();

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ull;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
  return x ^ (x >> 31);
}

double smoothstep(double e0, double e1, double x) {
  if (!(e1 > e0))
    return x >= e1 ? 1.0 : 0.0;
  const double t = std::clamp((x - e0) / (e1 - e0), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

// One cell of first-order fast marching: the arrival time from the smaller known neighbour along
// each axis (a along x, b along y; infinite where there is none) at slowness f over spacing h.
double eikonalUpdate(double a, double b, double f, double h) {
  const double fh = f * h;
  if (!std::isfinite(a) || !std::isfinite(b) || std::fabs(a - b) >= fh)
    return std::min(a, b) + fh;
  return 0.5 * (a + b + std::sqrt(2.0 * fh * fh - (a - b) * (a - b)));
}

} // namespace

double shore_wavenumber(double omega, double depth_m) {
  const double k0 = omega * omega / gravity;
  if (!(depth_m > 0.0) || !(omega > 0.0))
    return k0;
  const double x = k0 * depth_m;
  if (x > 20.0)
    return k0;                             // deep water to double precision
  double k = k0 / std::sqrt(det::tanh(x)); // Eckart (1952), within about 5 %
  for (int i = 0; i < 6; ++i) {            // then Newton on g k tanh(k d) - omega^2
    const double kd = k * depth_m, th = det::tanh(kd);
    const double f = gravity * k * th - omega * omega;
    const double df = gravity * th + gravity * kd * (1.0 - th * th);
    k -= f / df;
  }
  return k;
}

double shoaling(double omega, double depth_m) {
  if (!(depth_m > 0.0) || !(omega > 0.0))
    return 1.0;
  const double cg0 = 0.5 * gravity / omega;
  const double cg = dispersion_dk(shore_wavenumber(omega, depth_m), depth_m);
  return cg > 0.0 ? std::sqrt(cg0 / cg) : 1.0;
}

double iribarren(double tan_beta, double h0_m, double period_s) {
  const double L0 = gravity * period_s * period_s / kTau;
  return h0_m > 0.0 ? tan_beta / std::sqrt(h0_m / L0) : kInf;
}

double reflection_coefficient(double xi) { return std::min(1.0, 0.1 * xi * xi); }

double runup(double xi, double h0_m) { return xi * h0_m; }

shore_layer shore_layer::build(const shore_grid &g, const shore_swell &s,
                               const shore_layer_params &p) {
  const std::size_t n = g.count();
  if (g.rows < 2 || g.cols < 2 || !(g.cell > 0.0))
    throw std::invalid_argument(
        "shore_layer: the grid must be at least 2 x 2 with a positive cell");
  if (g.depth.size() != n || g.shore_m.size() != n || g.normal_x.size() != n ||
      g.normal_y.size() != n)
    throw std::invalid_argument(
        "shore_layer: every plane must hold rows x cols = " + std::to_string(n) + " values");
  shore_layer L;
  L.rows = g.rows;
  L.cols = g.cols;
  L.min_x = g.min_x;
  L.min_y = g.min_y;
  L.cell = g.cell;
  const double T = std::max(s.tp_s, 2.0);
  L.omega = kTau / T;
  const int N = std::max(1, p.trains);
  const double rootN = std::sqrt(double(N));
  L.deep_amplitude = std::max(0.0, s.hs_m) / (2.0 * std::sqrt(2.0)); // H_rms / 2
  // The trains: periods spread about T, the amplitude shared so their variance is the swell's.
  L.trains.resize(std::size_t(N));
  for (int i = 0; i < N; ++i) {
    shore_train &tr = L.trains[std::size_t(i)];
    const int step = (i + 1) / 2, sign = i % 2 == 1 ? 1 : -1;
    tr.ratio = 1.0 / (1.0 + sign * step * p.period_spread); // omega ratio from the period's
    tr.amplitude = 1.0 / rootN;
    const std::uint64_t h = splitmix64(p.seed ^ splitmix64(std::uint64_t(i) + 1));
    tr.phase = kTau * double(std::uint32_t(h)) / 4294967296.0;
    tr.phase_reflected = kTau * double(std::uint32_t(h >> 32)) / 4294967296.0;
  }

  // The local wavenumber of every wet cell.
  const int R = g.rows, C = g.cols;
  std::vector<std::uint8_t> wet(n);
  L.wavenumber.assign(n, 0.0f);
  std::vector<double> k(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const float d = g.depth[i];
    wet[i] = std::isfinite(d) && d > 0.0f ? 1 : 0;
    if (wet[i]) {
      k[i] = shore_wavenumber(L.omega, d);
      L.wavenumber[i] = float(k[i]);
    }
  }

  // The phase: fast marching from the wet edge cells the swell enters by, each seeded with the
  // plane wave's phase k (x . swell - min). Dry cells are walls.
  const double ux = det::cos(s.dir_rad), uy = det::sin(s.dir_rad);
  std::vector<double> P(n, kInf);
  std::vector<std::uint8_t> known(n, 0);
  using item = std::pair<double, std::size_t>;
  std::priority_queue<item, std::vector<item>, std::greater<item>> heap;
  {
    std::vector<std::size_t> seeds;
    double minProj = kInf;
    for (int r = 0; r < R; ++r)
      for (int c = 0; c < C; ++c) {
        if (r != 0 && r != R - 1 && c != 0 && c != C - 1)
          continue;
        const std::size_t i = std::size_t(r) * C + c;
        if (!wet[i])
          continue;
        // An edge the swell crosses inward: its outward normal points against the swell.
        const bool enters = (c == 0 && ux > 1e-9) || (c == C - 1 && ux < -1e-9) ||
                            (r == 0 && uy > 1e-9) || (r == R - 1 && uy < -1e-9);
        if (!enters)
          continue;
        seeds.push_back(i);
        minProj = std::min(minProj, g.x(c) * ux + g.y(r) * uy);
      }
    for (std::size_t i : seeds) {
      const int r = int(i / std::size_t(C)), c = int(i % std::size_t(C));
      P[i] = k[i] * (g.x(c) * ux + g.y(r) * uy - minProj);
      heap.emplace(P[i], i);
    }
  }
  while (!heap.empty()) {
    const auto [v, i] = heap.top();
    heap.pop();
    if (known[i] || v > P[i])
      continue;
    known[i] = 1;
    const int r = int(i / std::size_t(C)), c = int(i % std::size_t(C));
    const int nb[4][2] = {{r, c - 1}, {r, c + 1}, {r - 1, c}, {r + 1, c}};
    for (const auto &q : nb) {
      const int rr = q[0], cc = q[1];
      if (rr < 0 || cc < 0 || rr >= R || cc >= C)
        continue;
      const std::size_t j = std::size_t(rr) * C + cc;
      if (!wet[j] || known[j])
        continue;
      auto knownAt = [&](int r2, int c2) {
        if (r2 < 0 || c2 < 0 || r2 >= R || c2 >= C)
          return kInf;
        const std::size_t m = std::size_t(r2) * C + c2;
        return known[m] ? P[m] : kInf;
      };
      const double a = std::min(knownAt(rr, cc - 1), knownAt(rr, cc + 1));
      const double b = std::min(knownAt(rr - 1, cc), knownAt(rr + 1, cc));
      const double t = eikonalUpdate(a, b, k[j], g.cell);
      if (t < P[j]) {
        P[j] = t;
        heap.emplace(t, j);
      }
    }
  }

  // The direction of travel, grad P over the reached cells (one-sided where a neighbour is not).
  L.dir_x.assign(n, float(ux));
  L.dir_y.assign(n, float(uy));
  for (int r = 0; r < R; ++r)
    for (int c = 0; c < C; ++c) {
      const std::size_t i = std::size_t(r) * C + c;
      if (!known[i])
        continue;
      auto d1 = [&](int r0, int c0, int r1, int c1, double &out) {
        const bool ok0 = r0 >= 0 && c0 >= 0 && r0 < R && c0 < C && known[std::size_t(r0) * C + c0];
        const bool ok1 = r1 >= 0 && c1 >= 0 && r1 < R && c1 < C && known[std::size_t(r1) * C + c1];
        if (ok0 && ok1)
          out = (P[std::size_t(r1) * C + c1] - P[std::size_t(r0) * C + c0]) / (2.0 * g.cell);
        else if (ok1)
          out = (P[std::size_t(r1) * C + c1] - P[i]) / g.cell;
        else if (ok0)
          out = (P[i] - P[std::size_t(r0) * C + c0]) / g.cell;
        else
          return false;
        return true;
      };
      double gx = 0.0, gy = 0.0;
      const bool hx = d1(r, c - 1, r, c + 1, gx), hy = d1(r - 1, c, r + 1, c, gy);
      const double len = det::hypot(gx, gy);
      if ((hx || hy) && len > 1e-9) {
        L.dir_x[i] = float(gx / len);
        L.dir_y[i] = float(gy / len);
      }
    }

  L.phase.assign(n, 0.0f);
  for (std::size_t i = 0; i < n; ++i)
    if (known[i])
      L.phase[i] = float(P[i]);

  // The amplitude: shoaled, shadowed near the shore, capped where the wave breaks; the surf zone's
  // reflection and runup; the Gerstner steepness.
  L.dry_time_s = p.dry_time_s;
  L.amplitude.assign(n, 0.0f);
  L.reflected.assign(n, 0.0f);
  L.steepness.assign(n, float(p.steepness));
  L.breaking.assign(n, 0.0f);
  L.reflection.assign(n, 0.0f);
  L.runup.assign(n, 0.0f);
  const double gamma = p.breaker_index;
  for (std::size_t i = 0; i < n; ++i) {
    if (!known[i] || L.deep_amplitude <= 0.0)
      continue;
    const double d = g.depth[i], kk = k[i], lambda = kTau / kk;
    const double shore = std::isfinite(g.shore_m[i]) ? std::max(0.0, double(g.shore_m[i])) : 0.0;
    double e = 1.0;
    const double nx = g.normal_x[i], ny = g.normal_y[i];
    if (nx != 0.0 || ny != 0.0)
      e = det::pow(std::max(0.0, -(nx * ux + ny * uy)), p.exposure_power);
    e += (1.0 - e) * smoothstep(0.0, p.shadow_wavelengths * lambda, shore);
    double A = L.deep_amplitude * shoaling(L.omega, d) * e;
    // A set's crest is the trains' amplitudes summed: sqrt(N) A. Its height may not pass gamma d.
    const double cap = gamma * d / (2.0 * rootN);
    if (A > cap) {
      L.breaking[i] = float(std::min(1.0, (A - cap) / cap));
      A = cap;
    }
    L.amplitude[i] = float(A);
    // The surf zone's mean slope, from the waterline to here, and its reflection.
    const double tanBeta = d / std::max(shore, 0.5 * g.cell);
    const double xi = iribarren(tanBeta, s.hs_m, T);
    const double Kr = reflection_coefficient(xi);
    L.reflection[i] = float(Kr);
    L.reflected[i] = float(Kr * A * det::exp(-shore / (p.reflect_wavelengths * lambda)));
    // Hunt's runup from the swell's deep-water height, as shadowed as the swell is here.
    L.runup[i] = float(std::min(xi, p.runup_xi_max) * s.hs_m * e);
    // Steeper toward breaking, but never so steep the surface loops (Q sqrt(N) A k < 1).
    const double ratio = std::min(1.0, A / std::max(cap, 1e-12));
    double Q = p.steepness + (1.0 - p.steepness) * ratio;
    if (A * kk > 0.0)
      Q = std::min(Q, 0.9 / (rootN * A * kk));
    L.steepness[i] = float(Q);
  }

  // The phase and runup where the swell does not reach, carried over from the reached cells beside
  // them, ring by ring, swash_reach_m up the beach (at least a few rings): the swash runs up the
  // dry sand on them, and a bilinear sample at the waterline mixes in no stray value.
  {
    const int rings = std::max(3, int(std::ceil(std::max(0.0, p.swash_reach_m) / g.cell)));
    std::vector<std::uint8_t> has(known);
    for (int ring = 0; ring < rings; ++ring) {
      std::vector<std::uint8_t> next(has);
      std::vector<float> ph(L.phase), ru(L.runup);
      bool grew = false;
      for (int r = 0; r < R; ++r)
        for (int c = 0; c < C; ++c) {
          const std::size_t i = std::size_t(r) * C + c;
          if (has[i])
            continue;
          double sp = 0.0, sr = 0.0;
          int m = 0;
          const int nb[4][2] = {{r, c - 1}, {r, c + 1}, {r - 1, c}, {r + 1, c}};
          for (const auto &q : nb)
            if (q[0] >= 0 && q[1] >= 0 && q[0] < R && q[1] < C &&
                has[std::size_t(q[0]) * C + q[1]]) {
              sp += L.phase[std::size_t(q[0]) * C + q[1]];
              sr += L.runup[std::size_t(q[0]) * C + q[1]];
              ++m;
            }
          if (m > 0) {
            ph[i] = float(sp / m);
            ru[i] = float(sr / m);
            next[i] = 1;
            grew = true;
          }
        }
      L.phase.swap(ph);
      L.runup.swap(ru);
      has.swap(next);
      if (!grew)
        break;
    }
  }
  return L;
}

shore_layer::swash_sample shore_layer::swash(double x, double y, double t, double ground_z,
                                             double level) const {
  swash_sample out;
  out.age = kInf;
  if (empty() || trains.empty())
    return out;
  const double z = ground_z - level;
  const double R = sample_field(runup, x, y);
  if (z <= 0.0) { // under the still water: wet, the sea's own surface covers it
    out.wet = 1.0;
    out.age = 0.0;
    return out;
  }
  if (!(R > 0.0) || z >= R) // above the runup: the swash never reaches it
    return out;
  const double P = sample_field(phase, x, y);
  const shore_train &tr = trains.front();
  const double th = omega * tr.ratio * t - tr.ratio * P + tr.phase;
  out.front = R * (0.5 + 0.5 * det::cos(th));
  // The front is above z while |theta| <= alpha (mod 2 pi); it last left z at theta = alpha.
  const double alpha = det::acos(std::clamp(2.0 * z / R - 1.0, -1.0, 1.0));
  const double w = std::remainder(th, kTau); // theta wrapped to [-pi, pi]
  if (std::fabs(w) <= alpha) {
    out.age = 0.0;
    out.film = std::clamp((out.front - z) / std::max(0.15 * R, 0.05), 0.0, 1.0);
    out.lace = 1.0 - smoothstep(0.0, std::max(0.12 * R, 0.03), out.front - z);
  } else {
    double since = w - alpha;
    if (since < 0.0)
      since += kTau;
    out.age = since / (omega * tr.ratio);
  }
  out.wet = det::exp(-out.age / std::max(dry_time_s, 1e-6));
  return out;
}

double shore_layer::sample_field(const std::vector<float> &f, double x, double y) const {
  if (f.size() != std::size_t(rows) * std::size_t(cols) || f.empty())
    return 0.0;
  const double fc = std::clamp((x - min_x) / cell, 0.0, double(cols - 1));
  const double fr = std::clamp((y - min_y) / cell, 0.0, double(rows - 1));
  const int c0 = std::min(cols - 2, int(std::floor(fc))),
            r0 = std::min(rows - 2, int(std::floor(fr)));
  const double u = fc - c0, v = fr - r0;
  const double a = f[index(r0, c0)], b = f[index(r0, c0 + 1)], c = f[index(r0 + 1, c0)],
               d = f[index(r0 + 1, c0 + 1)];
  return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}

shore_layer::sample shore_layer::evaluate(double x, double y, double t) const {
  sample out;
  if (empty())
    return out;
  const double A = sample_field(amplitude, x, y), Ar = sample_field(reflected, x, y);
  out.breaking = sample_field(breaking, x, y);
  if (!(A > 0.0) && !(Ar > 0.0))
    return out;
  const double P = sample_field(phase, x, y), Q = sample_field(steepness, x, y);
  double ux = sample_field(dir_x, x, y), uy = sample_field(dir_y, x, y);
  const double len = det::hypot(ux, uy);
  if (len > 1e-6) {
    ux /= len;
    uy /= len;
  }
  for (const shore_train &tr : trains) {
    const double th = omega * tr.ratio * t - tr.ratio * P + tr.phase;
    const double thr = omega * tr.ratio * t + tr.ratio * P + tr.phase_reflected;
    out.eta += tr.amplitude * (A * det::cos(th) + Ar * det::cos(thr));
    const double h = tr.amplitude * Q * A * det::sin(th);
    out.dx += h * ux;
    out.dy += h * uy;
  }
  return out;
}

std::vector<float> shore_layer::wave_texels() const {
  std::vector<float> t(phase.size() * 4);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    t[i * 4] = phase[i];
    t[i * 4 + 1] = amplitude[i];
    t[i * 4 + 2] = reflected[i];
    t[i * 4 + 3] = steepness[i];
  }
  return t;
}

std::vector<float> shore_layer::dir_texels() const {
  std::vector<float> t(phase.size() * 4);
  for (std::size_t i = 0; i < phase.size(); ++i) {
    t[i * 4] = dir_x[i];
    t[i * 4 + 1] = dir_y[i];
    t[i * 4 + 2] = runup[i];
    t[i * 4 + 3] = breaking[i];
  }
  return t;
}

} // namespace ocean
} // namespace cvc
