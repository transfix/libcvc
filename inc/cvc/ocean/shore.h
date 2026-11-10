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

// shore.h — the shore layer: the swell as it refracts, shoals, breaks and reflects on a coast
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, A3).
//
//   cvc::ocean::shore_grid g;            // depth, shore distance and normal on a square grid
//   cvc::ocean::shore_layer s = cvc::ocean::shore_layer::build(g, swell, params);
//   cvc::ocean::shore_layer::sample w = s.evaluate(x, y, t);  // height, sideways move, breaking
//   s.wave_texels(), s.dir_texels()      // the same fields for the GPU, which does the same sum
//
// The swell (a period, a deep-water height and a direction) is a few regular trains. Their phase
// P(x) solves the eikonal equation |grad P| = k(x), k the local wavenumber of the finite-depth
// dispersion relation, by fast marching from the grid's edge where the swell enters, as a plane
// wave. Far from land the crests stay straight; where the water shoals they slow and turn toward
// the shore (refraction, from the physics rather than painted on), and they wrap round an island
// to meet behind it. The amplitude at each wet cell is the deep-water one times
//   * the shoaling coefficient sqrt(c_g0 / c_g), Green's law d^(-1/4) in shallow water;
//   * the exposure max(0, -n . swell)^p near the shore, n the offshore normal, so a shore facing
//     away from the swell lies in its shadow (fading back to the open swell a few wavelengths out);
// and is capped where the wave would break, H = 2A <= gamma d (McCowan's 0.78), the excess
// recorded as the breaking intensity that feeds foam. The surf zone's Iribarren number
// xi = tan(beta) / sqrt(H0 / L0) (tan beta the mean slope from the waterline) sorts shores: a beach
// absorbs, a cliff reflects with Battjes' K_r = min(1, 0.1 xi^2), and the reflected train runs back
// offshore, fading over a couple of wavelengths; with the incident one it stands in front of a
// cliff. The profile is Gerstner's, its steepness rising toward breaking.
//
// It is analytic: evaluate() is the sum the GPU draws, from the same fields, so physics (the
// roadmap's B5) reads what is drawn. Standard library only; deterministic. A shore_grid is plain
// planes (cvc::world::water_bodies holds them; cvc::ocean depends on no other library).

#ifndef CVC_OCEAN_SHORE_H
#define CVC_OCEAN_SHORE_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cvc {
namespace ocean {

// The coast on one square grid, row-major, row 0 = min y, x fastest.
struct shore_grid {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, cell = 1.0; // the sample spacing (m)
  std::vector<float> depth;                    // still-water depth (m): > 0 wet, <= 0 (or NaN) dry
  std::vector<float> shore_m;                  // signed distance to the shore (m): + offshore
  std::vector<float> normal_x, normal_y;       // the unit shore normal, offshore (0, 0 undefined)

  std::size_t count() const noexcept { return std::size_t(rows) * std::size_t(cols); }
  double x(int c) const noexcept { return min_x + c * cell; }
  double y(int r) const noexcept { return min_y + r * cell; }
};

struct shore_swell {
  double hs_m = 1.0;    // deep-water significant height (0: no swell)
  double tp_s = 10.0;   // period (at least 2 s)
  double dir_rad = 0.0; // the direction it travels, from +x toward +y
};

struct shore_layer_params {
  int trains = 3;                   // regular trains summed, so waves come in sets
  double period_spread = 0.06;      // their periods: T, T (1 + s), T (1 - s), T (1 + 2s), ...
  double breaker_index = 0.78;      // H <= gamma d (McCowan)
  double exposure_power = 1.0;      // max(0, -n . swell)^p near the shore
  double shadow_wavelengths = 2.0;  // the shadow fades to the open swell this far from shore
  double reflect_wavelengths = 2.0; // a reflected train fades over this far offshore
  double steepness = 0.5;           // Gerstner steepness in deep water, rising to 1 at breaking
  std::uint64_t seed = 0;           // the trains' phases
};

// One regular train: its frequency and wavenumber as ratios of the swell's, its share of the
// amplitude, and its phases (incident and reflected).
struct shore_train {
  double ratio = 1.0;
  double amplitude = 1.0;
  double phase = 0.0, phase_reflected = 0.0;
};

class shore_layer {
public:
  // The layer over grid g for swell s. Throws std::invalid_argument on an empty or inconsistent
  // grid (a plane of the wrong size, a cell that is not positive).
  static shore_layer build(const shore_grid &g, const shore_swell &s,
                           const shore_layer_params &p = {});

  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, cell = 1.0;
  double omega = 0.0;          // the swell's angular frequency (rad/s)
  double deep_amplitude = 0.0; // the trains' amplitude in deep water, before shoaling (m)
  std::vector<shore_train> trains;

  // One value per cell, the grid's layout. Dry cells and wet ones the swell cannot reach (an
  // enclosed lake) have no waves (amplitude 0); their phase is carried over from the wet cells
  // beside them so a bilinear sample at the waterline is smooth.
  std::vector<float> phase;        // P (rad); a crest travels toward increasing P
  std::vector<float> amplitude;    // the incident amplitude (m): shoaled, shadowed, capped
  std::vector<float> reflected;    // the reflected train's amplitude (m)
  std::vector<float> steepness;    // Gerstner steepness Q, kept so the surface does not loop
  std::vector<float> dir_x, dir_y; // the unit direction of travel, grad P / |grad P|
  std::vector<float> wavenumber;   // k (rad/m), |grad P|
  std::vector<float> breaking;     // 0..1: how far past the breaking cap the wave was
  std::vector<float> reflection;   // Battjes' K_r of the surf zone here

  bool empty() const noexcept { return phase.empty(); }
  std::size_t index(int r, int c) const noexcept { return std::size_t(r) * cols + c; }

  // The surface at (x, y) at time t (s): the trains' sum, bilinear in the fields (clamped to the
  // grid) — the arithmetic the GPU does.
  struct sample {
    double eta = 0.0;          // height above the still water (m)
    double dx = 0.0, dy = 0.0; // the sideways (Gerstner) move (m)
    double breaking = 0.0;     // the breaking intensity there
  };
  sample evaluate(double x, double y, double t) const;
  // A field above, bilinear at (x, y), clamped to the grid.
  double sample_field(const std::vector<float> &f, double x, double y) const;

  // The fields packed for the GPU, cell by cell: (phase, amplitude, reflected, steepness) and
  // (dir_x, dir_y, wavenumber, breaking).
  std::vector<float> wave_texels() const;
  std::vector<float> dir_texels() const;
};

// The wavenumber (rad/m) of angular frequency omega in water depth d: the finite-depth dispersion
// relation omega^2 = g k tanh(k d), solved by Newton's method from Eckart's approximation.
double shore_wavenumber(double omega, double depth_m);
// The shoaling coefficient sqrt(c_g0 / c_g) of a wave of omega at depth d (1 in deep water).
double shoaling(double omega, double depth_m);
// The Iribarren number of a shore of slope tan_beta for deep-water height H0 and period T.
double iribarren(double tan_beta, double h0_m, double period_s);
// Battjes' reflection coefficient min(1, 0.1 xi^2).
double reflection_coefficient(double xi);
// Hunt's runup R = xi H (valid for xi up to about 2.5), for the swash.
double runup(double xi, double h0_m);

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_SHORE_H
