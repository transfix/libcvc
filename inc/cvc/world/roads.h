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

// roads.h — a tensor-field street network for cvc::world.
//
// The road graph the procedural city is laid out FROM (buildings line the roads, not the reverse).
// It follows the Parish & Müller [SIGGRAPH 2001] framework — a priority queue of candidate
// segments, each accepted only after local constraints snap it into the existing network — with the
// Chen et al. [2008] tensor-field twist for direction: at each point a 2-D tensor field gives a
// major and a minor (perpendicular) road direction, blended from
//
//   * a GRID field (a fixed base orientation — regular blocks), and
//   * a TERRAIN field (perpendicular to the height gradient — roads follow the contours / low
//   slope),
//
// weighted by the local slope, so the city is a clean grid on the flats that bends organically over
// hills. Roads are traced as short straight segments stepped along the field (Euler streamlines,
// of the major or the minor direction, whichever the road is on), so a road curves as a chain of
// segments and an intersection is simply a graph node of degree > 2.
//
// The LOCAL CONSTRAINTS are what make the intersections correct: a candidate segment's free end is
// snapped to a nearby node (junction), or to a nearby segment which is split to form a T-junction,
// or a crossing is cut to form a 4-way; near-misses are extended to join rather than leaving
// dangling stubs. A uniform spatial hash gives the O(1) proximity queries.
//
// Deterministic (hashed RNG, stream::road; no <random>, per the §5.2 world rule) and GL-free. The
// output is a plain graph (nodes + width-tagged segments) plus a nearest-road query; a consumer
// extrudes it to geometry, stamps it into the surface raster, and places buildings along its edges.
//
// Optional, all off by default (the network is then exactly the classic one):
//
//   * A HIERARCHY: with collector_width set, the branches off an arterial are collectors spaced a
//     block apart, and only the collectors (and the locals) sprout local streets.
//   * A GRADE LIMIT: a segment steeper than max_grade is steered toward the contour (turned in
//     growing steps either way) and dropped if no turn fits, so hill roads wind instead of climbing
//     the fall line.
//   * A NO-GO MASK (generate_roads' `blocked`, e.g. water): a segment that would cross it is
//     steered the same way, so streets stop at the shore instead of being cut off afterwards.
//   * DISTRICTS: several centres (each seeds an arterial cross), and an urbanity field in [0, 1]
//     that packs the local streets tighter and branches more often in a district's core.
//   * PRUNE_ROADS afterwards trims short dead-end stubs and recomputes the node degrees.

#ifndef CVC_WORLD_ROADS_H
#define CVC_WORLD_ROADS_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace world {

// Ground height at world (x, y) — the terrain field reads its gradient from this. A null/empty
// sampler (always 0) degenerates the terrain field to flat, giving a pure grid.
using road_height_fn = std::function<double(double x, double y)>;
// True where no road may go (water, a cliff). Empty = nowhere blocked.
using road_mask_fn = std::function<bool(double x, double y)>;
// A district's urbanity at world (x, y), in [0, 1] (1 = a core). Empty = uniform.
using road_density_fn = std::function<double(double x, double y)>;

// A district centre: the network grows an arterial cross out of each.
struct road_centre {
  double x = 0.0, y = 0.0;
};

struct road_params {
  std::uint64_t seed = 1;
  double half = 128.0; // the city spans the square [-half, half]^2 (metres, centred on origin)

  // Field blend.
  double grid_angle_deg = 0.0; // base grid orientation (degrees)
  double terrain_weight = 0.6; // 0 = pure grid; 1 = roads fully follow terrain contours. Scaled by
                               // the local slope, so flats stay grid-like regardless.
  double terrain_slope_ref_deg = 8.0; // slope at which the terrain field reaches full weight

  // Network shape.
  double segment_len = 14.0;    // length of one traced segment (metres) — the streamline step
  double block_size = 90.0;     // target spacing between parallel major roads (metres)
  double local_block = 34.0;    // spacing between minor (local) roads
  int max_segments = 4000;      // hard cap on the network size
  double snap_dist = 10.0;      // junction snap / near-miss-extend radius (metres)
  double min_segment = 5.0;     // drop a candidate shorter than this after snapping
  double arterial_width = 11.0; // major-road width (metres)
  double local_width = 6.5;     // minor-road width
  double core_radius = 70.0;    // roads within this radius of the centre are traced first (denser)
  double branch_chance = 0.9;   // probability a road branches (perpendicular, both sides) at a
                                // step once it has run a block since its last branch

  // The hierarchy: > 0 makes an arterial's branches collectors of this width (see the header).
  double collector_width = 0.0;
  // The grade limit (rise over run; 0 = none) and the steering tried before a segment is dropped:
  // turns of steer_step_deg, 2 x, ... up to steer_max_deg, alternating sides.
  double max_grade = 0.0;
  double steer_step_deg = 15.0;
  double steer_max_deg = 75.0;
  // District centres (empty = one, at the origin). The urbanity field (generate_roads' `density`)
  // scales the local spacing from local_block / sparse_spacing (urbanity 0) down to local_block
  // (urbanity 1), and the branch chance from branch_chance x sparse_branch up to branch_chance.
  std::vector<road_centre> centres;
  double sparse_spacing = 1.0;
  double sparse_branch = 1.0;
  // A local street is not continued, or branched, where the urbanity is below this (0 = never
  // stops): the local grids fade out at the districts' edges while the arterials and collectors
  // run on between them.
  double local_min_urbanity = 0.0;
  // A collector or local street stops rather than run alongside another road (within 25 degrees
  // of parallel) closer than this fraction of the local spacing there, so two streets branched
  // from different roads cannot end up a few metres apart (0 = off).
  double parallel_gap = 0.0;
};

// A road's class, widest first.
enum class road_class : int { local = 0, collector = 1, arterial = 2 };

// An intersection or road endpoint.
struct road_node {
  double x = 0.0, y = 0.0;
  int degree = 0; // number of segments meeting here (>2 => a real junction)
};

// A straight road piece between two nodes (a chain of these makes a curved road).
struct road_segment {
  int a = -1, b = -1;    // endpoint node indices into road_network::nodes
  double width = 6.5;    // metres
  bool arterial = false; // major road (vs local street)
  road_class kind = road_class::local;
};

struct road_network {
  std::vector<road_node> nodes;
  std::vector<road_segment> segments;
  double half = 128.0; // the region the network was generated over (for the SVG emitter / queries)

  bool empty() const { return segments.empty(); }

  // Distance (metres) from world (x, y) to the nearest road CENTERLINE, and — via out-params — the
  // index of that segment and the parameter t in [0,1] of the closest point along it. Returns a
  // large value for an empty network. Backed by a lazily-built uniform-grid index (rebuilt when the
  // segment COUNT changes, e.g. after pruning), so repeated per-site queries are ~O(1) instead of
  // O(segments) — the result is identical to a brute-force scan (same minimum, same lowest-index
  // tie-break).
  double distance_to_road(double x, double y, int *seg = nullptr, double *t = nullptr) const;

  // distance_to_road when the nearest centerline is within r metres; otherwise a large value and
  // *seg = -1, found without looking further than r (a query far inside a big block stays cheap).
  double distance_within(double x, double y, double r, int *seg = nullptr,
                         double *t = nullptr) const;

  // True when (x, y) is within `margin` metres of any road surface (its half-width + margin) — the
  // test a road-aware scatter uses to keep trees/props off the pavement.
  bool on_or_near_road(double x, double y, double margin = 0.0) const;

private:
  // Cached proximity index (opaque; defined in roads.cpp). Mutable so the const query methods can
  // build it on demand. Invalidated by a change in segment count. If a caller mutates segment
  // CONTENTS in place without changing the count, it should reset this by reassigning the network.
  mutable std::shared_ptr<void> idx_cache_;
  mutable std::size_t idx_nseg_ = static_cast<std::size_t>(-1);
};

// Generate the street network. Deterministic in p.seed. GL-free. `blocked` and `density` are
// optional (see road_params' grade limit and districts).
road_network generate_roads(const road_params &p, const road_height_fn &height = {},
                            const road_mask_fn &blocked = {}, const road_density_fn &density = {});

// Trim dead ends: repeatedly drop a dead-end chain (from a degree-1 node back to the first
// junction) shorter than min_stub metres, then recompute every node's degree. Segments are
// removed, nodes are kept (an orphan has degree 0). Returns how many segments went.
std::size_t prune_roads(road_network &net, double min_stub);

// The tensor field's MAJOR road direction (unit vector) at world (x, y), for the given params +
// terrain — exposed for tests and for a consumer that wants to orient something to the local grid.
void road_field_dir(const road_params &p, const road_height_fn &height, double x, double y,
                    double &dx, double &dy);

// Emit the network as a standalone SVG (centrelines, width-scaled strokes, junction dots) for
// review and golden tests — the same "a PR you can open and look at" discipline cvc::lsys uses.
std::string roads_to_svg(const road_network &net);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_ROADS_H
