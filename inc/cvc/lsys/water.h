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

// water.h — the demo's travelling-wave sea as a time-varying depth FIELD (promoted from the demo).
//
// Not an L-system — a closed-form sum of crested travelling waves — but it lives beside cloud.h
// because together they are the forest demo's two volumetric environment layers, and it shares the
// family's determinism contract (it needs no RNG at all, so it is trivially reproducible). The
// output is a water-depth density field for a GPU ray-cast VolumeNode, re-filled each frame at the
// animation time to make the sea roll. Ported in shape from
// src/cvcGL/examples/lsystem_forest.cpp.

#ifndef CVC_LSYS_WATER_H
#define CVC_LSYS_WATER_H

#include <vector>

namespace cvc {
namespace lsys {

// The sea-surface height at world (x, y) and animation time t (seconds): four crested travelling
// waves at incommensurate speeds/headings (pinched peaks, broad troughs), about sea_level with
// vertical scale wave_amp.
double sea_surface(double x, double y, double t, double sea_level, double wave_amp);

struct sea_params {
  int n = 56;            // XY resolution (the field is n x n x nz)
  int nz = 18;           // Z resolution
  double half = 120.0;   // world half-extent in XY (metres)
  double sea_level = 0.0;
  double wave_amp = 2.40;
};

// The Z band the field spans (metres): [sea_level - 20, sea_level + 5].
inline double sea_floor(const sea_params &p) { return p.sea_level - 20.0; }
inline double sea_top(const sea_params &p) { return p.sea_level + 5.0; }

// Fill `out` (n*n*nz floats, Z-major index (z*n + y)*n + x) with the water depth under the wave
// surface and above the terrain at animation time t, normalised to [0, 1] (full at ~6 m deep).
// `terrain` holds n*n ground heights sampled on the SAME XY grid (row-major j*n + i); an empty
// `terrain` is treated as a flat floor at sea_floor(p). `out` is resized as needed.
void sea_field(const sea_params &p, const std::vector<float> &terrain, double t,
               std::vector<float> &out);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_WATER_H
