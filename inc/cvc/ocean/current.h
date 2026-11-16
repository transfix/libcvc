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
  The current's shape (the curl of noise at two scales, the second at 0.45 of the first's weight,
  drifting slowly) follows ABYSSAL's currentAt (github.com/Token-Gremlin/natural-disasters,
  src/ocean/OceanSampleGLSL.js:31-41 at 849ff7f4, and src/gfx/ProceduralTextures.js's curl bake),
  MIT licensed, Copyright (c) 2026 Davi (Token-Gremlin). MIT notice retained; see
  THIRD_PARTY_NOTICES.md.
*/

// current.h — currents: a slowly varying flow over the water that carries the small waves and
// floating bodies alike (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E6, section 10.3).
//
//   cvc::ocean::current_params c;  c.speed_mps = 0.3;
//   cvc::ocean::current_frame f = cvc::ocean::current_frame_at(c, t);
//   std::array<double, 2> v = cvc::ocean::current_velocity(f, x, y);   // m/s
//
// The flow is the curl of a stream function psi, value noise at two scales (scale_m and 0.41 of
// it, the second at 0.45 of the first's weight) drifting slowly: v = (d psi / dy, -d psi / dx). So
// it is divergence-free exactly (the noise is twice continuously differentiable), and it is a
// physical speed: speed_mps is its RMS. ABYSSAL normalises its curl to a direction and uses it as a
// 26 m domain warp; a normalised curl is not divergence-free, and a warp is not a velocity
// (section 10.3), so here the one field is both what the drawn waves are carried by and what a
// floating body drifts with.
//
// The small cascades are carried by it with a two-phase flow map: each cascade is read at
//   p - v(p) (tau_i - T / 2) + o_i tile,  i = 0, 1,
// tau_i the time since phase i last restarted (the phases half a period T apart), o_i a random
// offset in the tile drawn afresh at each restart (it hides the restart, where the phase's weight
// is 0). The phases' weights are triangles summing to 1, divided by sqrt(w0^2 + w1^2) so the sum
// of two uncorrelated reads keeps the sea's variance through the blend (no pulsing in height).
// Neighbouring tiles see different currents, so the same tile is read at different offsets across
// the sea: that breaks up the small cascades' tiling, as ABYSSAL's warp does, while the waves move
// with the water as the physics feels it. The waves on a current are Doppler-shifted, which the
// advection gives.
//
// Standard library only; deterministic (cvc::algo's lattice noise, no libm transcendentals).

#ifndef CVC_OCEAN_CURRENT_H
#define CVC_OCEAN_CURRENT_H

#include <array>
#include <cstdint>

namespace cvc {
namespace ocean {

// The small eddies' size as a share of the large, and their weight in the stream function.
constexpr double kCurrentSmall = 0.41, kCurrentWeight = 0.45;

struct current_params {
  double speed_mps = 0.0;       // the RMS speed (0: no current)
  double scale_m = 600.0;       // the large eddies' size (the noise's cell)
  double flow_period_s = 12.0;  // the flow map's period T
  std::uint32_t seed = 0xC022u; // the pattern
};

// The current at one time: the stream function's drift (each scale's, in its lattice units,
// reduced modulo the lattice's period) and the flow map's phases.
struct current_frame {
  current_params p;
  double gain = 0.0;             // speed_mps over the noise gradients' RMS: v = gain curl(n)
  std::array<double, 4> drift{}; // large x, y; small x, y
  double tau[2] = {0.0, 0.0};    // each phase's advection time, tau_i - T / 2 (s)
  double weight[2] = {1.0, 0.0}; // each phase's weight, the blend's normalisation in it
  double offset[2][2] = {{0.0, 0.0}, {0.0, 0.0}}; // each phase's offset, a fraction of the tile
};

current_frame current_frame_at(const current_params &p, double t);

// The current's velocity at (x, y) (m/s along x and y).
std::array<double, 2> current_velocity(const current_frame &f, double x, double y);
std::array<double, 2> current_velocity(const current_params &p, double x, double y, double t);

// The stream function psi at (x, y) (m^2/s): the current is (d psi / dy, -d psi / dx).
double current_stream(const current_frame &f, double x, double y);

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_CURRENT_H
