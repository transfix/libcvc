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

// sites.h — road-side building lots: where a procedural city puts its buildings.
//
// Given the road network (cvc::world::roads) this lays out building LOTS along both sides of every
// street: step along each segment in fixed frontage slots, offset perpendicular by the road's
// half-width + a setback, and emit a lot centre that FACES the road. Lots near a junction are
// dropped (so corners stay open and intersections read correctly), lots whose footprint would sit
// on any road are dropped (so there is always room between the building and the kerb), and lots
// that would overlap an already-placed lot are dropped (a deterministic spatial check). The result
// is a plain list of oriented lots — reusable for buildings OR road-aware prop scatter.
//
// fit_buildings() then draws from a building_library (the real Austin archetypes), picks one per
// lot deterministically, and scales + orients it to FIT the lot (longer footprint axis along the
// frontage, front toward the road), producing building_instances ready for stamp_buildings().
//
// GL-free (plain geometry + math). Deterministic: every random choice is a hashed function of a
// stable lot id (its quantized position), never a loop counter — adding or removing a road cannot
// reshuffle the buildings on an untouched street [see cvc/lsys/rng.h].

#ifndef CVC_WORLD_SITES_H
#define CVC_WORLD_SITES_H

#include <cstdint>
#include <cvc/world/building_archetypes.h> // building_library, building_instance
#include <cvc/world/roads.h>               // road_network, road_height_fn
#include <vector>

namespace cvc {
namespace world {

// One building lot: a footprint-centre site on the setback line, oriented so the building's front
// faces the road it fronts. `frontage` runs along the road, `depth` runs away from it.
struct lot {
  double x = 0.0, y = 0.0; // footprint-centre (metres), on the setback line
  double yaw = 0.0;        // radians about +Z; the building front (local +Y) points at the road
  double frontage = 0.0;   // usable width ALONG the road (metres)
  double depth = 0.0;      // usable depth AWAY from the road (metres)
  int seg = -1;            // index of the road (traced polyline) this lot fronts
  bool arterial = false;   // fronts a major road
};

struct site_params {
  std::uint64_t seed = 1;
  double setback = 6.0;         // road EDGE (half-width) to the building front (metres)
  double frontage = 14.0;       // nominal lot width along the road; the stepping slot (metres)
  double frontage_jitter = 0.0; // +/- fraction added to frontage per lot (0 = uniform slots)
  double spacing = 4.0;         // gap between adjacent lots along the road (metres)
  double lot_depth = 20.0;      // how far back a lot extends from the setback line (metres)
  double junction_clear = 12.0; // keep lots this far (centre) from any junction node (metres)
  double road_margin = 1.5;     // a lot footprint must clear every road by this much (metres)
  bool both_sides = true;       // place on both sides of each road (false = +normal side only)
  double min_segment = 10.0;    // skip a road (traced polyline) shorter than this (metres)
};

// Lay out building lots along the roads. Deterministic and order-stable in the network. Streets are
// traced into polylines (through degree-2 nodes, breaking at junctions), so lots flow continuously
// along a curved road and junctions stay open. NOTE: this needs streets that run BETWEEN junctions
// — a road network generated with a very high road_params::branch_chance is a dense every-segment
// crosshatch with no block interiors, and almost no lots will fit; feed a network with real blocks.
std::vector<lot> layout_lots(const road_network &net, const site_params &sp);

struct fit_params {
  std::uint64_t seed = 1;
  double margin = 1.0;    // leave this gap inside the lot on each side (metres)
  double min_scale = 0.3; // clamp the fit scale so a huge archetype is not shrunk to nothing
  double max_scale = 2.5; // ... and a tiny one is not blown up past this
};

// Pick + fit one archetype per lot and return the placed instances (ground height from `height`,
// which may be empty → z = 0). Each archetype is scaled so its footprint fits the lot (longer axis
// along the frontage) and yawed to the lot. Deterministic in fp.seed. Empty if the library is
// empty.
std::vector<building_instance> fit_buildings(const building_library &lib,
                                             const std::vector<lot> &lots, const fit_params &fp,
                                             const road_height_fn &height = {});

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_SITES_H
