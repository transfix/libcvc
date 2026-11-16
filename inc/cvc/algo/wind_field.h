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
  The gust pulse over time is ABYSSAL's (github.com/Token-Gremlin/natural-disasters,
  src/weather/Weather.js:107-110 at 849ff7f4), MIT licensed, Copyright (c) 2026 Davi
  (Token-Gremlin). MIT notice retained; see THIRD_PARTY_NOTICES.md.
*/

// wind_field.h — the scene's wind at a point and a time: one mean wind (its speed, the way it
// blows and its gustiness) and the gusts riding on it (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md
// E3, E5 and section 10.4).
//
//   cvc::algo::wind_state w{9.0, 0.52, 0.25};                // 9 m/s toward 30 degrees, gusty
//   double g = cvc::algo::wind_gust(w, x, y, t);             // the factor on the mean wind there
//   std::array<double, 2> v = cvc::algo::wind_velocity(w, x, y, t);  // m/s
//
// One function of (x, y, t) that everything the wind moves samples alike: the water's ripples and
// roughness (cat's paws), and later the clouds' drift, the trees and the air physics pushes bodies
// with (E5). It lives in cvc::algo because every one of those libraries may depend on it, and it
// on none of them.
//
// The gusts are a pattern of two scales carried downwind: the large gusts (kGustCell) at the
// wind's speed, the cat's paws inside them (kGustCellFine) at 0.8 of it, so the pattern changes as
// it goes rather than sliding past whole. Over them the whole wind pulses with ABYSSAL's three
// sines. The factor is
//   g = max(0, 1 + gustiness (2.4 (n - 0.5) + 0.5 pulse(t))),  n = 0.65 n_large + 0.35 n_fine,
// each n value noise in [0, 1). A renderer may fade the fine scale out with distance (`fine`):
// its mean stays, so the factor's mean does not move.
//
// Deterministic and the same on every platform: integer hashes, quintic interpolation and
// det_math.h's sine and cosine, built without FMA contraction. The lattice repeats every
// kLatticePeriod cells, so the downwind drift can be reduced modulo the period in double precision
// (wind_drift) and a GPU, which has single precision, reads the same pattern after hours of drift.
// The GLSL in cvc/gl/WindGLSL.h is this arithmetic in floats.
//
// Standard library only, like the rest of cvc::algo.

#ifndef CVC_ALGO_WIND_FIELD_H
#define CVC_ALGO_WIND_FIELD_H

#include <array>
#include <cstdint>

namespace cvc {
namespace algo {

struct wind_state {
  double speed_mps = 0.0; // the mean wind (U10)
  double dir_rad = 0.0;   // the way it blows, from +x toward +y
  double gustiness = 0.0; // 0: steady; ABYSSAL's default is 0.25
};

// The noise lattice repeats every kLatticePeriod cells along each axis.
constexpr std::int32_t kLatticePeriod = 4096;

// A 32-bit hash of lattice point (i, j), its indices taken modulo kLatticePeriod. The GLSL does the
// same in uint arithmetic.
std::uint32_t lattice_hash(std::int32_t i, std::int32_t j, std::uint32_t seed);

// Value noise at (x, y) in lattice units: bilinear between the hashed corners, eased by the quintic
// 6u^5 - 15u^4 + 10u^3 (so the noise is twice continuously differentiable). In [0, 1), with its
// gradient per lattice unit.
struct noise2 {
  double v = 0.0, dx = 0.0, dy = 0.0;
};
noise2 value_noise(double x, double y, std::uint32_t seed);

// ABYSSAL's gust over time: 0.5 sin(phi) + 0.3 sin(2.37 phi + 1.1) + 0.2 sin(5.1 phi), phi = 0.21
// t. In [-1, 1].
double gust_pulse(double t);

constexpr double kGustCell = 120.0;    // m: the large gusts
constexpr double kGustCellFine = 37.0; // m: the cat's paws inside them
constexpr std::uint32_t kGustSeed = 0x6057u, kGustSeedFine = 0x6058u;

// How far each scale's pattern has drifted downwind by time t, in its lattice units, reduced modulo
// kLatticePeriod: (large x, large y, fine x, fine y). The pattern at (x, y) is the noise at
// (x / cell - drift). A renderer computes these once a frame in double precision.
std::array<double, 4> wind_drift(const wind_state &w, double t);

// The gust at (x, y) at time t: the factor the mean wind is multiplied by there, at least 0 and
// about 1 on average. `fine` (0..1) scales the cat's paws' share, for a renderer's distance fade.
double wind_gust(const wind_state &w, double x, double y, double t, double fine = 1.0);

// The gusts at one time, for many reads: the drift (wind_drift) and the pulse (gust_pulse), and
// the gustiness. gust(frame, x, y) is wind_gust(w, x, y, t).
struct gust_frame {
  std::array<double, 4> drift{};
  double pulse = 0.0;
  double gustiness = 0.0;
};
gust_frame gust_frame_at(const wind_state &w, double t);
double wind_gust(const gust_frame &f, double x, double y, double fine = 1.0);

// The wind's velocity at (x, y) at time t (m/s, along x and y): the mean wind times the gust.
std::array<double, 2> wind_velocity(const wind_state &w, double x, double y, double t);

} // namespace algo
} // namespace cvc

#endif // CVC_ALGO_WIND_FIELD_H
