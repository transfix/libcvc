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

// whitecaps.h — how much of the open sea the drawn wave foam covers, and the foam threshold that
// makes it cover Monahan's share for the wind (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, O5 and
// section 10.2).
//
//   cvc::ocean::foam_field f({h0_long, h0_mid, h0_short});      // build_h0's fields
//   double bias = f.bias_for(cvc::ocean::monahan_coverage(9.0)); // lsystem_coast's uFoamBias
//
// The drawn sea's wave foam is a threshold on the Jacobian of its choppy map. Each cascade's
// J = (1 + l Dxx)(1 + l Dyy) - (l Dxy)^2 (OceanFFT's assemble pass, l its chop); the vertex shader
// keeps the least of the cascades' (a faded cascade folds less; on the open sea none is faded),
// and the fragment shader whitens the water by foam = smoothstep(0, 1, (bias - J) / ramp).
//
// A fixed bias gives a fixed share of foam on one sea only: on a calmer sea J seldom dips that
// low, on a rougher one it often does. Whitecaps cover a share of the sea that grows with the
// wind, W = 3.84e-6 U^3.41 (Monahan and O'Muircheartaigh 1980, U the wind at 10 m; capped at
// 0.16, as ABYSSAL caps it). So the bias follows the sea: foam_field evolves the cascades' h0 on
// the CPU as OceanFFT does, samples the least J over a lattice of the open sea at a few times,
// and finds the bias whose mean foam, the share of the surface the foam whitens, is W.
//
// The share is the mean of the foam value, since the shader mixes the water toward white by it:
// a fully white patch counts whole and a half-white one half. It is measured over the open sea's
// least J at lattice points; the drawn surface interpolates J between its vertices, so a coarse
// mesh shows a little less foam than this says.

#ifndef CVC_OCEAN_WHITECAPS_H
#define CVC_OCEAN_WHITECAPS_H

#include <cstddef>
#include <cvc/ocean/h0.h>
#include <vector>

namespace cvc {
namespace ocean {

// The share of the sea under whitecaps at a 10 m wind of u10 m/s: min(3.84e-6 u10^3.41, 0.16)
// (Monahan and O'Muircheartaigh 1980); 0 for u10 <= 0.
double monahan_coverage(double u10_mps);

struct foam_params {
  double chop = 1.15; // the Jacobian's l: OceanFFT::chop (lsystem_coast's coast.water.chop)
  double ramp = 0.1;  // the shader's foam ramp, in J: foam reaches 1 at J = bias - ramp
  // The shader's fray: each fragment's J shifted by fray x (n - 0.5), n = 0.65 v1 + 0.35 v2 with
  // v1, v2 value noise (a smoothstep-weighted bilinear mix of four uniform hashes) at two scales,
  // so a patch's edge is ragged rather than the straight line J makes across a triangle. Modelled
  // by 16 quantiles of n's distribution (from a fixed draw); 0: no fray.
  double fray = 0.0;
  double depth_m = 1000.0;  // the depth the cascades disperse in (OceanFFT::depth_m)
  double gravity = 9.80665; // m/s^2 (OceanFFT's)
  double spacing_m = 0.37;  // the open-sea lattice's spacing (the side is the largest tile)
  // The drawn mesh's vertex spacing: J taken at its vertices and linear over its triangles, as
  // the fragments see it, mesh_samples x mesh_samples points a cell. 0: J at the lattice points.
  double mesh_m = 0.0;
  int mesh_samples = 4;
  std::vector<double> times_s = {0.0, 3.7}; // the times the sea is sampled at
};

// The open sea's least Jacobian over a lattice, at a few times, and the foam it makes.
class foam_field {
public:
  // `cascades` are build_h0's fields (any n, any tiles). Throws std::invalid_argument when there
  // are none, a field is empty, spacing_m <= 0, ramp <= 0 or times_s is empty.
  explicit foam_field(const std::vector<h0_field> &cascades, const foam_params &p = {});

  // The mean foam over the drawn surface at this bias (each sample weighed by |J|, its share of
  // the surface; the fray averaged over): the share of the open sea whitened.
  double coverage(double bias) const;
  // The bias whose coverage is `share` (clamped to [0, 1]); coverage rises with the bias, so
  // this is a bisection. At 0 it is below the least sampled J (no foam).
  double bias_for(double share) const;
  // The share of the drawn surface at least `level` white (0 < level <= 1) at this bias.
  double white(double bias, double level = 0.5) const;

  std::size_t samples() const noexcept { return _j.size(); }
  double least() const noexcept { return _j.empty() ? 1.0 : double(_j.front()); }
  double mean() const noexcept { return _mean; }
  // The share of the samples (unweighted) with J < j (the fold's share when j is 0).
  double below(double j) const;

private:
  double coverage_at(double bias) const; // without the fray
  std::vector<float> _j;                 // the samples' least J, ascending
  double _ramp = 0.1;
  double _fray = 0.0;
  std::vector<double> _fray_q; // n - 0.5 at 16 quantiles of the fray noise
  double _mean = 1.0;
  std::vector<double> _area; // the sums of |J| (the drawn surface) of the first i samples
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_WHITECAPS_H
