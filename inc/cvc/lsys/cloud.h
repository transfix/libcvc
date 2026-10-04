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

// cloud.h — a volumetric cloud DENSITY FIELD from a 3-D L-system (promoted from the demo).
//
// This is the first VOLUME output in cvc::lsys: the turtle alphabet so far only drew surface
// primitives (segments/leaves/boxes/paints, interp.h). A cloud is different in kind — a 3-D turtle
// walks a cumulus silhouette and deposits Gaussian puffs into a scalar grid, then fractal (value-
// noise fBm) detail frays the edges and a cheap single-scatter march toward the sun bakes a top
// light. The output is a density field for a GPU ray-cast VolumeNode, not a mesh.
//
// Ported in shape from src/cvcGL/examples/lsystem_forest.cpp so the cloud looks as it does in the
// demo. The one library-mandated change (roadmap §5.2): the handful of initial placement/heading
// draws that the predecessor took from a std::mt19937 now go through the hashed RNG on
// stream::cloud, so a given (seed, variant) always grows the same cloud and distinct variants
// decorrelate. The fBm itself is already a pure integer-hash value noise and is kept verbatim.
//
// COST: O(steps + N*N*NZ * (octaves + light_steps)). Heavy enough to generate off the render
// thread (a pool task) and cache — a few maps are crossfaded/scrolled at runtime by the consumer,
// which this header does not do (that animation is cheap and belongs in the renderer).

#ifndef CVC_LSYS_CLOUD_H
#define CVC_LSYS_CLOUD_H

#include <cstdint>
#include <vector>

namespace cvc {
namespace lsys {

struct cloud_params {
  std::uint64_t seed = 1;  // grammar + placement seed
  std::uint32_t variant = 0; // which decorrelated cloud map (element id into stream::cloud)
  int n = 60;              // XY resolution (the field is n x n x nz)
  int nz = 28;             // Z resolution
  double half = 150.0;     // world half-extent in XY (metres), for the turtle's z-aspect scaling
  double base = 74.0;      // cloud slab base height (metres)
  double top = 122.0;      // cloud slab top height (metres)
  double sun_az = -52.0;   // sun azimuth (deg) for the baked top-light
  double sun_el = 34.0;    // sun elevation (deg)
  int depth = 6;           // grammar recursion depth cap
};

// Generate one cloud density field: n*n*nz floats in [0, 1], indexed
// (z*n + y)*n + x (Z-major), with the baked top-light already folded in. Deterministic in
// (seed, variant). Empty vector only if the dims are non-positive.
std::vector<float> cloud_field(const cloud_params &p);

// The flat index convention for a cloud field (Z-major): (z*n + y)*n + x.
inline std::size_t cloud_index(int n, int x, int y, int z) {
  return (static_cast<std::size_t>(z) * n + y) * n + x;
}

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_CLOUD_H
