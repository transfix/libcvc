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
// Optional, all off by default (the lots are then exactly the classic fixed slots):
//
//   * BLOCK DEPTH: a lot reaches back only to the middle of its block (a ray cast to the next road
//     behind it), so the lots of two parallel streets back onto each other instead of one street's
//     overlap test starving the other.
//   * A GROUND CHECK (with a height sampler): a lot whose footprint drops more than max_relief, or
//     dips below min_ground (the water), is left empty; a kept lot records its ground range, so a
//     building can stand on the low side with its base hidden in the slope.
//   * DISTRICTS (with an urbanity field in [0, 1]): a core narrows the setbacks and side gaps (down
//     to shared walls) and builds out every street side, while sparse ground leaves whole street
//     sides empty (sparse_fill), so a town reads as streets that are built up or not rather than
//     an even sprinkle. frontage_jitter varies the lot widths.
//
// GL-free (plain geometry + math). Deterministic: every random choice is a hashed function of a
// stable lot id (its quantized position), never a loop counter — adding or removing a road cannot
// reshuffle the buildings on an untouched street [see cvc/lsys/rng.h].

#ifndef CVC_WORLD_SITES_H
#define CVC_WORLD_SITES_H

#include <cstdint>
#include <cvc/world/building_archetypes.h> // building_library, building_instance
#include <cvc/world/roads.h>               // road_network, road_height_fn
#include <functional>
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
  int side = 1;            // which side of that road (+1 the left of its direction, -1 the right)
  double urbanity = 1.0;   // the district field at the lot (1 without one)
  bool has_ground = false; // ground_min/max were sampled (layout_lots had a height sampler)
  double ground_min = 0.0, ground_max = 0.0; // the ground's range over the footprint (metres)
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

  // Block depth (see the header): lot_depth caps it, and a lot shallower than min_depth is
  // dropped. The ray looks up to max_block metres behind the setback line.
  bool block_depth = false;
  double min_depth = 8.0;
  double max_block = 120.0;
  // The ground check (needs a height sampler; 0 / -inf = off).
  double max_relief = 0.0;
  double min_ground = -1e30;
  // Districts (need an urbanity field): at urbanity 1 the setback and spacing shrink by these
  // fractions; a street side is built with probability sparse_fill at urbanity 0, rising to 1.
  double core_setback = 0.0;
  double core_spacing = 0.0;
  double sparse_fill = 1.0;
};

// The district field layout_lots reads: urbanity at world (x, y), in [0, 1].
using lot_density_fn = std::function<double(double x, double y)>;

// Lay out building lots along the roads. Deterministic and order-stable in the network. Streets are
// traced into polylines (through degree-2 nodes, breaking at junctions), so lots flow continuously
// along a curved road and junctions stay open. NOTE: this needs streets that run BETWEEN junctions
// — a road network generated with a very high road_params::branch_chance is a dense every-segment
// crosshatch with no block interiors, and almost no lots will fit; feed a network with real blocks.
// `height` (the ground check) and `density` (districts) are optional.
std::vector<lot> layout_lots(const road_network &net, const site_params &sp,
                             const road_height_fn &height = {}, const lot_density_fn &density = {});

struct fit_params {
  std::uint64_t seed = 1;
  double margin = 1.0;    // leave this gap inside the lot on each side (metres)
  double min_scale = 0.3; // clamp the fit scale so a huge archetype is not shrunk to nothing
  double max_scale = 2.5; // ... and a tiny one is not blown up past this
  // true_scale: keep the buildings their real size. A uniform fit scales the height with the
  // footprint, so a 30 m-wide archetype squeezed into a 10 m lot stands a third of its height,
  // shorter than the trees. Instead draw up to `candidates` archetypes for the lot and take the
  // first that fits at a scale of at least min_true (never more than 1); with none, the lot is
  // left empty rather than holding a miniature.
  bool true_scale = false;
  double min_true = 0.85;
  int candidates = 32;
};

// Pick + fit one archetype per lot and return the placed instances (ground height from `height`,
// which may be empty → z = 0; a lot with a sampled ground range stands at its ground_min, so no
// corner floats above a slope). Each archetype is scaled so its footprint fits the lot (longer axis
// along the frontage) and yawed to the lot; with true_scale, an archetype that fits near its own
// size is chosen instead (see fit_params). Deterministic in fp.seed. Empty if the library is
// empty.
std::vector<building_instance> fit_buildings(const building_library &lib,
                                             const std::vector<lot> &lots, const fit_params &fp,
                                             const road_height_fn &height = {});

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_SITES_H
