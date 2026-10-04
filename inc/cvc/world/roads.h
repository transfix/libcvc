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
// hills. Roads are traced as short straight segments stepped along the major field (Euler
// streamlines), so a road curves as a chain of segments and an intersection is simply a graph node
// of degree > 2.
//
// The LOCAL CONSTRAINTS are what make the intersections correct: a candidate segment's free end is
// snapped to a nearby node (junction), or to a nearby segment which is split to form a T-junction,
// or a crossing is cut to form a 4-way; near-misses are extended to join rather than leaving
// dangling stubs. A uniform spatial hash gives the O(1) proximity queries.
//
// Deterministic (hashed RNG, stream::road; no <random>, per the §5.2 world rule) and GL-free. The
// output is a plain graph (nodes + width-tagged segments) plus a nearest-road query; a consumer
// extrudes it to geometry, stamps it into the surface raster, and places buildings along its edges.

#ifndef CVC_WORLD_ROADS_H
#define CVC_WORLD_ROADS_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace world {

// Ground height at world (x, y) — the terrain field reads its gradient from this. A null/empty
// sampler (always 0) degenerates the terrain field to flat, giving a pure grid.
using road_height_fn = std::function<double(double x, double y)>;

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
  double branch_chance = 0.9;   // probability a segment spawns a perpendicular branch
};

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
};

struct road_network {
  std::vector<road_node> nodes;
  std::vector<road_segment> segments;
  double half = 128.0; // the region the network was generated over (for the SVG emitter / queries)

  bool empty() const { return segments.empty(); }

  // Distance (metres) from world (x, y) to the nearest road CENTERLINE, and — via out-params — the
  // index of that segment and the parameter t in [0,1] of the closest point along it. A brute-force
  // scan (fine for the consumer's per-site queries); returns a large value for an empty network.
  double distance_to_road(double x, double y, int *seg = nullptr, double *t = nullptr) const;

  // True when (x, y) is within `margin` metres of any road surface (its half-width + margin) — the
  // test a road-aware scatter uses to keep trees/props off the pavement.
  bool on_or_near_road(double x, double y, double margin = 0.0) const;
};

// Generate the street network. Deterministic in p.seed. GL-free.
road_network generate_roads(const road_params &p, const road_height_fn &height = {});

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
