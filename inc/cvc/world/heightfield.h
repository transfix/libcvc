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

// heightfield.h — analytic, resolution-independent terrain, in METRES.
//
// A pure function of world (x,y): value-noise fBm at metre wavelengths, an
// optional domain warp, and an optional Wyvill island dome. Unlike the
// lsystem_forest predecessor (whose relief frequencies were absolute while its
// dome scaled with HALF, so the two halves scaled differently and aliased as
// the world grew), every wavelength here is a metre length, so the field is
// scale-consistent. The export always samples the full analytic surface — LOD
// band-limiting is a render concern and lives elsewhere.

#ifndef CVC_WORLD_HEIGHTFIELD_H
#define CVC_WORLD_HEIGHTFIELD_H

#include <cstdint>

namespace cvc {
namespace world {

struct heightfield_params {
  std::uint64_t seed = 0;
  double sea_level_m = 0.0;
  double amp_m = 8.0;              // relief amplitude (metres)
  double base_wavelength_m = 70.0; // largest relief feature wavelength
  int octaves = 5;
  double lacunarity = 2.0;
  double gain = 0.5;
  double warp_m = 6.0; // domain-warp distance (0 = off)
  double warp_wavelength_m = 130.0;
  // Optional island dome (peaks in the middle, sea at the edges).
  bool island = false;
  double island_peak_m = 30.0;
  double island_radius_m = 100.0; // Wyvill falloff radius (metres)
  double shelf_m = -9.0;          // depth outside the island mask
};

class heightfield {
public:
  explicit heightfield(const heightfield_params &p) : p_(p) {}

  double sample(double x, double y) const;

  // Slope in degrees, from a centred metre-scale finite difference.
  double slope_deg(double x, double y, double h_m = 1.0) const;

  const heightfield_params &params() const noexcept { return p_; }

private:
  heightfield_params p_;
};

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_HEIGHTFIELD_H
