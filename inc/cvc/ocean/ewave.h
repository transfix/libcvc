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

// ewave.h — waves that bodies make, on a square window of the sea
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, C1, C3 and C5): Tessendorf's eWave on the CPU.
//
//   cvc::ocean::ewave_params p;   // n = 256 cells of dx = 0.5 m, deep water
//   cvc::ocean::ewave w(p);
//   w.set_mask(mask);             // 1 open water, 0 land and obstacles, between: absorbing beach
//   w.add_height(x, y, -0.2, 1.0); // a dropped crate's dent
//   w.step(1.0 / 30.0);
//   w.at(x, y);                   // the wave height there (m, above the still level)
//
// The state is the surface height h and the surface velocity potential phi on n x n cells (n a
// power of two) of side dx, cell (i, j) at origin + (i dx, j dx), stored row-major (row j). One
// step of dt:
//   - h and phi pack into one complex field z = h + i phi, which one forward FFT takes to
//     wavenumbers (fft.h, sign -1);
//   - each wavenumber k and its mirror -k are separated into H(k) and PHI(k) through Hermitian
//     symmetry and moved on exactly, by linearized potential flow in one depth d:
//       kappa = |k| tanh(|k| d), omega = sqrt(g kappa)
//       H'   = H cos(omega dt) + (kappa / omega) PHI sin(omega dt)
//       PHI' = PHI cos(omega dt) - (g / omega) H sin(omega dt)
//     then damped by exp(-(damping_k2 |k|^2 + damping) dt); the inverse FFT (sign +1) brings them
//     back. The update is exact for any dt: there is no CFL limit. The per-mode coefficients are
//     computed for a dt and kept while dt is unchanged.
//   - In real space, h and phi are multiplied by the mask (applied once a step, so a beach value m
//     takes 1 - m of the wave each step: a beach's absorption depends on the step rate) and by the
//     sponge: a damping rate that ramps up as r^2 over the last sponge_cells cells toward the
//     window's edge (r from 0 to 1), to sponge_rate at the edge. The sponge absorbs outgoing waves
//     and hides the transform's wrap-around: the window is periodic.
//   - Sources (add_height, add_heights) go straight into h between steps, times the mask, so
//     water never appears on land. add_heights takes C2's whole-field -(zeta(t) - zeta(t - dt)).
//
// k = 0: the mean height has kappa = 0 and stays (but for the global damping). Exactly, the mean
// potential would drift by -g mean(h) dt a step, but a uniform potential moves no water, and the
// mask would turn that drift into a step of phi at every shore, so the mean potential is held at
// 0 instead (a choice of gauge: the heights are those of the exact update).
//
// Energy: 0.5 rho g sum h^2 dA (potential) plus 0.5 rho sum over k of kappa |PHI(k)|^2 dA / n^2
// (kinetic, Parseval). Without damping, mask or sponge, each mode's g |H|^2 + kappa |PHI|^2 is
// conserved by the update, so the energy is conserved to round-off.
//
// Determinism: the coefficients and the sponge use cvc::algo::det math, the FFT is fft.h's, and
// the module is built without FMA contraction, so a window gives the same bits on every platform
// for the same inputs and steps; digest() hashes the parameters, the mask and the state. A window
// may be read from several threads while nothing writes to it.
//
// Standard library only, like the rest of cvc::ocean.

#ifndef CVC_OCEAN_EWAVE_H
#define CVC_OCEAN_EWAVE_H

#include <cstdint>
#include <cvc/ocean/fft.h>
#include <vector>

namespace cvc {
namespace ocean {

struct ewave_params {
  int n = 256;                // cells a side (a power of two, 4..4096)
  double dx = 0.5;            // cell size (m)
  double origin_x = 0.0;      // cell (0, 0)'s position (m, world x and y)
  double origin_y = 0.0;      //
  double depth = 1000.0;      // the window's one water depth (m; +infinity is deep water)
  double gravity = 9.80665;   // m/s^2
  double density = 1025.0;    // kg/m^3, for energy()
  double damping_k2 = 0.0;    // gamma: modes decay as exp(-gamma |k|^2 dt) (m^2/s)
  double damping = 0.0;       // and all of them as exp(-damping dt) (1/s)
  double sponge_cells = 16.0; // the edge band's width (cells, at most n / 2; 0: no sponge)
  double sponge_rate = 2.0;   // its damping rate at the edge (1/s)
};

class ewave {
public:
  // Throws std::invalid_argument for parameters out of range.
  explicit ewave(const ewave_params &p);

  const ewave_params &params() const noexcept { return _p; }
  int size() const noexcept { return _p.n; }

  // The mask, n x n values in [0, 1], row-major. Throws std::invalid_argument for a wrong size or
  // a value out of range. It applies from the next step (and to sources added after it).
  void set_mask(const std::vector<double> &mask);
  const std::vector<double> &mask() const noexcept { return _mask; }

  // Replaces the state: n x n heights (m) and potentials (m^2/s), row-major. Not masked.
  void set_state(const std::vector<double> &h, const std::vector<double> &phi);
  // Adds amount (m) at (x, y) in world coordinates, spread over a smooth bump of the given radius
  // (m): amount (1 - r^2 / radius^2)^3 within it. Throws std::invalid_argument for a radius not
  // above 0 or a value that is not finite.
  void add_height(double x, double y, double amount, double radius);
  // Adds n x n heights (m), row-major, times the mask.
  void add_heights(const std::vector<double> &dh);

  // Moves the waves on by dt (s, above 0 and finite).
  void step(double dt);

  const std::vector<double> &height() const noexcept { return _h; }
  const std::vector<double> &phi() const noexcept { return _phi; }
  // The height at (x, y) in world coordinates, bilinear between the cells; 0 outside the window.
  double at(double x, double y) const;
  // Cell (i, j)'s position.
  double x_of(int i) const noexcept { return _p.origin_x + double(i) * _p.dx; }
  double y_of(int j) const noexcept { return _p.origin_y + double(j) * _p.dx; }
  // The time stepped so far (s).
  double time() const noexcept { return _time; }

  // The waves' energy (J): potential and kinetic, as in the opening comment.
  double potential_energy() const;
  double kinetic_energy() const;
  double energy() const { return potential_energy() + kinetic_energy(); }

  // A 64-bit FNV-1a digest of the parameters, the mask and the state.
  std::uint64_t digest() const;

  // omega(k) for this window's depth and gravity (rad/s, k in rad/m).
  double omega(double k) const;

private:
  void prepare(double dt);

  ewave_params _p;
  fft_plan _fft;
  std::vector<double> _h, _phi, _mask;
  bool _masked = false;            // some mask value is below 1
  std::vector<double> _k2, _kappa; // per mode: |k|^2 and kappa
  std::vector<double> _z;          // the complex work field, (re, im) interleaved
  double _dt = 0.0;                // the dt the coefficients below were computed for (0: none)
  std::vector<double> _cf;         // per mode: H' = cf[0] H + cf[1] PHI, PHI' = cf[2] H + cf[3] PHI
  std::vector<double> _real;       // per cell: mask times the sponge's factor for _dt
  bool _real_ones = true;          // every _real value is 1: the real-space pass is skipped
  bool _real_stale = true;         // the mask or dt changed since _real was computed
  double _time = 0.0;
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_EWAVE_H
