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

  ---------------------------------------------------------------------------
  gust_pulse is ABYSSAL's (github.com/Token-Gremlin/natural-disasters,
  src/weather/Weather.js:107-110 at 849ff7f4), MIT licensed, Copyright (c) 2026 Davi
  (Token-Gremlin). MIT notice retained; see THIRD_PARTY_NOTICES.md.
*/

// wind_field.cpp — the scene's wind and its gusts (see wind_field.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/algo/wind_field.h>

namespace cvc {
namespace algo {

std::uint32_t lattice_hash(std::int32_t i, std::int32_t j, std::uint32_t seed) {
  // The indices modulo the period (two's complement, so negative ones wrap too), then a mix of
  // multiplies and shifts (the GLSL's, in uint arithmetic).
  const std::uint32_t mask = std::uint32_t(kLatticePeriod) - 1u;
  std::uint32_t h = (std::uint32_t(i) & mask) * 0x8da6b343u;
  h ^= (std::uint32_t(j) & mask) * 0xd8163841u;
  h ^= seed * 0xcb1ab31fu;
  h ^= h >> 16;
  h *= 0x7feb352du;
  h ^= h >> 15;
  h *= 0x846ca68bu;
  h ^= h >> 16;
  return h;
}

namespace {

double corner(std::int32_t i, std::int32_t j, std::uint32_t seed) {
  return double(lattice_hash(i, j, seed) >> 8) * (1.0 / 16777216.0); // 24 bits: exact in a float
}

} // namespace

noise2 value_noise(double x, double y, std::uint32_t seed) {
  const double fx = std::floor(x), fy = std::floor(y);
  const double u = x - fx, v = y - fy;
  // The cell, its indices reduced modulo the period first so any finite coordinate fits.
  const double P = double(kLatticePeriod);
  const auto i = static_cast<std::int32_t>(fx - P * std::floor(fx / P));
  const auto j = static_cast<std::int32_t>(fy - P * std::floor(fy / P));
  const double a = corner(i, j, seed), b = corner(i + 1, j, seed);
  const double c = corner(i, j + 1, seed), d = corner(i + 1, j + 1, seed);
  const double su = u * u * u * (u * (u * 6.0 - 15.0) + 10.0);
  const double sv = v * v * v * (v * (v * 6.0 - 15.0) + 10.0);
  const double du = 30.0 * u * u * (u - 1.0) * (u - 1.0);
  const double dv = 30.0 * v * v * (v - 1.0) * (v - 1.0);
  const double ab = a + (b - a) * su, cd = c + (d - c) * su;
  noise2 n;
  n.v = ab + (cd - ab) * sv;
  n.dx = du * ((b - a) + ((d - c) - (b - a)) * sv);
  n.dy = dv * (cd - ab);
  return n;
}

double gust_pulse(double t) {
  const double phi = 0.21 * t;
  return 0.5 * det::sin(phi) + 0.3 * det::sin(phi * 2.37 + 1.1) + 0.2 * det::sin(phi * 5.1);
}

std::array<double, 4> wind_drift(const wind_state &w, double t) {
  const double P = double(kLatticePeriod);
  const double cx = det::cos(w.dir_rad), cy = det::sin(w.dir_rad);
  const double s = std::max(w.speed_mps, 0.0) * t;
  auto wrap = [P](double v) { return v - P * std::floor(v / P); };
  // The large gusts ride at the wind's speed, the cat's paws at 0.8 of it.
  return {wrap(s * cx / kGustCell), wrap(s * cy / kGustCell), wrap(0.8 * s * cx / kGustCellFine),
          wrap(0.8 * s * cy / kGustCellFine)};
}

gust_frame gust_frame_at(const wind_state &w, double t) {
  gust_frame f;
  f.gustiness = std::max(w.gustiness, 0.0);
  if (f.gustiness > 0.0) {
    f.drift = wind_drift(w, t);
    f.pulse = gust_pulse(t);
  }
  return f;
}

double wind_gust(const gust_frame &g, double x, double y, double fine) {
  if (!(g.gustiness > 0.0))
    return 1.0;
  const std::array<double, 4> &o = g.drift;
  const double f = std::clamp(fine, 0.0, 1.0);
  const double n1 = value_noise(x / kGustCell - o[0], y / kGustCell - o[1], kGustSeed).v;
  const double n2 =
      f > 0.0 ? value_noise(x / kGustCellFine - o[2], y / kGustCellFine - o[3], kGustSeedFine).v
              : 0.5;
  const double n = 0.65 * n1 + 0.35 * (0.5 + f * (n2 - 0.5));
  return std::max(0.0, 1.0 + g.gustiness * (2.4 * (n - 0.5) + 0.5 * g.pulse));
}

double wind_gust(const wind_state &w, double x, double y, double t, double fine) {
  return wind_gust(gust_frame_at(w, t), x, y, fine);
}

std::array<double, 2> wind_velocity(const wind_state &w, double x, double y, double t) {
  const double s = std::max(w.speed_mps, 0.0) * wind_gust(w, x, y, t);
  return {s * det::cos(w.dir_rad), s * det::sin(w.dir_rad)};
}

double wind_gust_mean(const gust_frame &f) {
  if (!(f.gustiness > 0.0))
    return 1.0;
  return std::max(0.0, 1.0 + f.gustiness * 0.5 * f.pulse);
}

double wind_profile(double h) {
  if (!(h > 0.0))
    return 0.0;
  constexpr double kAlpha = 1.0 / 7.0;
  constexpr double kFloor = 0.1; // m: linear below, up to the law's value there
  if (h < kFloor)
    return h / kFloor * det::pow(kFloor / 10.0, kAlpha);
  return det::pow(std::min(h, 500.0) / 10.0, kAlpha);
}

} // namespace algo
} // namespace cvc
