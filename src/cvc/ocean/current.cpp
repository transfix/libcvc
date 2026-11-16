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
  After ABYSSAL's currentAt (src/ocean/OceanSampleGLSL.js:31-41 at 849ff7f4), MIT licensed,
  Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained; see THIRD_PARTY_NOTICES.md.
*/

// current.cpp — currents (see current.h).

#include <algorithm>
#include <cmath>
#include <cvc/algo/wind_field.h>
#include <cvc/ocean/current.h>

namespace cvc {
namespace ocean {

namespace {

constexpr double kSmall = kCurrentSmall;
constexpr double kWeight = kCurrentWeight;
// The RMS of (d psi / dy, -d psi / dx) for unit noise gradients at these two scales, measured over
// 2 x 10^5 points (OceanCurrent.IsDivergenceFreeAndItsSpeedIsTheRMS): speed_mps is its RMS.
constexpr double kNorm = 0.6878;
// How fast each scale's pattern drifts (m/s), so the current changes over tens of minutes.
constexpr double kDriftLarge[2] = {0.10, 0.035}, kDriftSmall[2] = {-0.10, 0.25};

double wrap(double v) {
  const double P = double(cvc::algo::kLatticePeriod);
  return v - P * std::floor(v / P);
}

} // namespace

current_frame current_frame_at(const current_params &p, double t) {
  current_frame f;
  f.p = p;
  f.gain = std::max(p.speed_mps, 0.0) / kNorm;
  const double L = std::max(p.scale_m, 1e-3), Ls = L * kSmall;
  f.drift = {wrap(kDriftLarge[0] * t / L), wrap(kDriftLarge[1] * t / L),
             wrap(kDriftSmall[0] * t / Ls), wrap(kDriftSmall[1] * t / Ls)};
  const double T = p.flow_period_s;
  if (!(T > 0.0)) {
    f.tau[0] = f.tau[1] = 0.0;
    f.weight[0] = 1.0;
    f.weight[1] = 0.0;
    return f;
  }
  double w[2];
  for (int i = 0; i < 2; ++i) {
    const double s = t / T + 0.5 * i;
    const double cyc = std::floor(s), fr = s - cyc;
    f.tau[i] = (fr - 0.5) * T;
    w[i] = 1.0 - std::fabs(2.0 * fr - 1.0);
    // The cycle's offset: a hash of the cycle (modulo 2^32) and the phase.
    const double c32 = cyc - 4294967296.0 * std::floor(cyc / 4294967296.0);
    const auto cycle = static_cast<std::uint32_t>(c32);
    const std::uint32_t hx = cvc::algo::lattice_hash(std::int32_t(cycle & 0xffffu),
                                                     std::int32_t(cycle >> 16), p.seed + 2u * i);
    const std::uint32_t hy = cvc::algo::lattice_hash(
        std::int32_t(cycle & 0xffffu), std::int32_t(cycle >> 16), p.seed + 2u * i + 1u);
    f.offset[i][0] = double(hx >> 8) * (1.0 / 16777216.0);
    f.offset[i][1] = double(hy >> 8) * (1.0 / 16777216.0);
  }
  const double n = std::sqrt(w[0] * w[0] + w[1] * w[1]); // at least 1/sqrt(2)
  f.weight[0] = w[0] / n;
  f.weight[1] = w[1] / n;
  return f;
}

std::array<double, 2> current_velocity(const current_frame &f, double x, double y) {
  if (!(f.p.speed_mps > 0.0))
    return {0.0, 0.0};
  const double L = std::max(f.p.scale_m, 1e-3), Ls = L * kSmall;
  const cvc::algo::noise2 a =
      cvc::algo::value_noise(x / L - f.drift[0], y / L - f.drift[1], f.p.seed);
  const cvc::algo::noise2 b =
      cvc::algo::value_noise(x / Ls - f.drift[2], y / Ls - f.drift[3], f.p.seed + 17u);
  // psi = s L (n_a + kWeight kSmall n_b) / kNorm, so d psi / dx = s (n_a.dx + kWeight n_b.dx) /
  // kNorm.
  const double s = f.gain;
  return {s * (a.dy + kWeight * b.dy), -s * (a.dx + kWeight * b.dx)};
}

std::array<double, 2> current_velocity(const current_params &p, double x, double y, double t) {
  return current_velocity(current_frame_at(p, t), x, y);
}

double current_stream(const current_frame &f, double x, double y) {
  const double L = std::max(f.p.scale_m, 1e-3), Ls = L * kSmall;
  const double na = cvc::algo::value_noise(x / L - f.drift[0], y / L - f.drift[1], f.p.seed).v;
  const double nb =
      cvc::algo::value_noise(x / Ls - f.drift[2], y / Ls - f.drift[3], f.p.seed + 17u).v;
  return f.p.speed_mps / kNorm * L * (na + kWeight * kSmall * nb);
}

} // namespace ocean
} // namespace cvc
