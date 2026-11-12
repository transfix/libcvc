/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar -- exact planar geometry on an integer grid, the layer the ContourTiler port sits on
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3, phase P3a). It replaces the CGAL packages
// ContourTiler used, all of them GPL (§1.2), with permissive libraries:
//
//   CGAL (GPL-3+)                                 cvc::planar
//   exact kernels (Gmpq, Lazy_exact_nt)           points on an integer grid (grid, point) and
//                                                   Shewchuk's predicates (public domain)
//   Polygon_set_2 Booleans                        unite / intersect / subtract / exclusive_or
//                                                   (Clipper2, Boost Software Licence)
//   straight-skeleton offsets                     wavefront_offset (Clipper2, stepped through
//                                                   the skeleton's events) and offset
//   minkowski_sum_2                               minkowski_sum (Clipper2)
//   Constrained_Delaunay_triangulation_2,         delaunay / constrained_delaunay / triangulate
//     Delaunay_triangulation_2                      (CDT, MPL-2.0)
//   approx_convex_partition_2 (Hertel-Mehlhorn)   convex_partition (Hertel-Mehlhorn on CDT)
//   natural_neighbor_coordinates_2,               sibson_coordinates / sibson_interpolate
//     linear_interpolation
//   Straight_skeleton_2 (as a medial axis)        medial_axis (Boost.Polygon's segment Voronoi
//                                                   diagram, Boost Software Licence)
//
// Conventions shared by every function:
//  * Coordinates are integers on a grid (`grid`): a power-of-two unit and an origin chosen from a
//    bounding box, so every input vertex snaps once and every grid point has an exact world
//    position. Every operation takes coordinates within +-coord_max (Boost.Polygon's int32 range,
//    which the medial axis needs; Shewchuk's predicates see them exactly as doubles), and throws
//    planar_error on one outside it, or on an offset or sum that would leave it.
//  * A ring is a closed polygonal chain: its last vertex joins its first, which is not repeated.
//    A polygon is an outer ring and its holes. Functions that return polygons give outer rings
//    counterclockwise and holes clockwise ("normalised"); functions that take them accept either
//    orientation and normalise a copy first.
//  * Predicates are exact. Constructions (an intersection point, an offset vertex) round to the
//    grid, so an operation's output may differ from the exact result by up to a grid unit at each
//    new vertex.
//  * Functions are pure: no state outside their arguments, safe to call from any thread
//    (§1.3). Errors are planar_error, which says which function and what was wrong.
//
// Availability. The functions are defined only when libcvc is built with Clipper2, CDT and
// Shewchuk's predicates (the cvcpkg packages clipper2, cdt and shewchuk-predicates); cvc then
// exports the PUBLIC compile definition CVC_WITH_PLANAR. Boost.Polygon is header-only and part of
// Boost. Without them (macOS, until those packages have macOS bundles) the header still compiles,
// and calling a function fails to link.

#ifndef __CVC_GEOMETRY_PLANAR_H__
#define __CVC_GEOMETRY_PLANAR_H__

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace planar {

// What every function throws: bad input (a coordinate off the grid's range, a non-simple ring
// where a simple one is needed) or a result that cannot be represented.
class planar_error : public std::runtime_error {
public:
  explicit planar_error(const std::string &what) : std::runtime_error(what) {}
};

// ============================================================================================
// The grid

using coord = std::int64_t;

// Every operation takes coordinates within +-coord_max: INT32_MAX, the range of Boost.Polygon's
// Voronoi input.
constexpr coord coord_max = 2147483647;

// grid::fit puts a bounding box within +-fit_extent (2^30), leaving the other half of the range for
// offsets and sums that grow past the box.
constexpr coord fit_extent = coord(1) << 30;

// A point on the grid.
struct point {
  coord x = 0, y = 0;
  point() = default;
  point(coord x_, coord y_) : x(x_), y(y_) {}
  friend bool operator==(const point &a, const point &b) { return a.x == b.x && a.y == b.y; }
  friend bool operator!=(const point &a, const point &b) { return !(a == b); }
  friend bool operator<(const point &a, const point &b) {
    return a.x < b.x || (a.x == b.x && a.y < b.y);
  }
};

// A point off the grid: a world position, or a position in grid units that is not a grid point
// (a medial-axis vertex, a query point).
struct pointd {
  double x = 0, y = 0;
  pointd() = default;
  pointd(double x_, double y_) : x(x_), y(y_) {}
};

// The snapping grid: world = origin + unit * grid. `unit` is a power of two and the origin a
// multiple of it, so a grid point's world position is exact (to 2^53 units).
struct grid {
  double origin_x = 0, origin_y = 0;
  double unit = 1;

  // The grid for everything within [xmin, xmax] x [ymin, ymax]: the origin at the box's centre
  // (rounded to a unit), and the smallest power-of-two unit that puts the box within
  // +-fit_extent. A box of one point gets unit 1. Throws planar_error on a non-finite or empty
  // (min > max) box.
  static grid fit(double xmin, double ymin, double xmax, double ymax);

  // The nearest grid point (halves away from zero). Throws planar_error if it is outside
  // +-coord_max, or on a non-finite coordinate.
  point snap(double x, double y) const;
  point snap(const pointd &p) const { return snap(p.x, p.y); }
  std::vector<point> snap(const std::vector<pointd> &ps) const;

  // World positions: exact for a grid point, rounded for a pointd in grid units.
  pointd to_world(const point &p) const;
  pointd to_world(const pointd &p) const;
  // Grid units, unrounded.
  pointd to_grid(const pointd &world) const;
  // World length of `n` grid units, and grid units of a world length.
  double to_world(double n) const { return n * unit; }
  double to_grid(double length) const { return length / unit; }
};

// ============================================================================================
// Exact predicates (Shewchuk's orient2d and incircle)

// +1 if a, b, c turn counterclockwise (c is left of a->b), -1 clockwise, 0 if collinear.
int orientation(const point &a, const point &b, const point &c);
int orientation(const pointd &a, const pointd &b, const pointd &c);

// +1 if d is inside the circle through a, b, c, -1 outside, 0 on it, when a, b, c turn
// counterclockwise; the sign flips when they turn clockwise, and is 0 when they are collinear
// (CGAL's side_of_oriented_circle).
int in_circle(const point &a, const point &b, const point &c, const point &d);
int in_circle(const pointd &a, const pointd &b, const pointd &c, const pointd &d);

// Do the closed segments [a, b] and [c, d] share a point? Touching counts. A zero-length segment is
// a point.
bool segments_intersect(const point &a, const point &b, const point &c, const point &d);

// Is p on the closed segment [a, b]?
bool on_segment(const point &p, const point &a, const point &b);

// ============================================================================================
// Rings and polygons

// A closed chain: the last vertex joins the first, which is not repeated.
using ring = std::vector<point>;

// An outer ring and its holes. Normalised: the outer ring counterclockwise, the holes clockwise.
struct polygon {
  ring outer;
  std::vector<ring> holes;
  polygon() = default;
  explicit polygon(ring outer_, std::vector<ring> holes_ = {})
      : outer(std::move(outer_)), holes(std::move(holes_)) {}
};

// Polygons whose interiors do not overlap.
using polygon_set = std::vector<polygon>;

// Twice the signed area would overflow int64 near the range's edge, so areas are doubles: the
// shoelace sum, positive for a counterclockwise ring. A polygon's area is its outer ring's minus
// its holes'; a set's is the sum.
double signed_area(const ring &r);
double area(const polygon &p);
double area(const polygon_set &s);

// The length of the boundary, in grid units.
double perimeter(const ring &r);
double perimeter(const polygon &p);

// +1 counterclockwise, -1 clockwise, 0 for a ring with no area. Exact for a simple ring (the
// orientation at its lowest, then leftmost, vertex).
int orientation(const ring &r);

// Is the ring simple: at least 3 vertices, no repeated vertex, and no two edges meeting except
// consecutive ones at their shared vertex? Exact.
bool is_simple(const ring &r);
// Is every ring simple, and do no two of them share a point? (Holes inside the outer ring and
// outside each other are not checked: see nest() for that.)
bool is_simple(const polygon &p);

// Where a point lies, exactly.
enum class location { outside, boundary, inside };
location locate(const point &p, const ring &r);     // r simple
location locate(const point &p, const polygon &p_); // inside the outer ring and outside the holes
location locate(const point &p, const polygon_set &s);

// The ring without repeated consecutive vertices and without collinear vertices (spikes, where the
// chain turns back on itself, included). The result can have fewer than 3 vertices.
ring clean(const ring &r);

// A copy with every ring cleaned, the outer ring counterclockwise and the holes clockwise; holes
// left with fewer than 3 vertices are dropped (an outer ring so left gives an empty polygon).
polygon normalize(const polygon &p);

// Rings that do not cross or touch, arranged into polygons with holes by nesting depth: a ring
// inside an even number of the others is an outer ring, and one inside an odd number is a hole of
// the smallest ring around it (ContourTiler's arrange_polygons). Orientations are ignored. Rings
// left with fewer than 3 vertices by clean() are dropped. Throws planar_error if two rings share a
// point or one is not simple.
polygon_set nest(const std::vector<ring> &rings);

// ============================================================================================
// Booleans (Clipper2)

// How the winding number of a set of rings decides what is inside.
enum class fill_rule { even_odd, non_zero, positive, negative };

// Each operand is normalised first, so a set is the union of its polygons (outer minus holes).
polygon_set unite(const polygon_set &a, const polygon_set &b);
polygon_set intersect(const polygon_set &a, const polygon_set &b);
polygon_set subtract(const polygon_set &a, const polygon_set &b);
polygon_set exclusive_or(const polygon_set &a, const polygon_set &b);

// The region rings enclose under a fill rule, as non-overlapping polygons: resolves
// self-intersections, overlaps and nesting in one go.
polygon_set resolve(const std::vector<ring> &rings, fill_rule rule = fill_rule::non_zero);

// ============================================================================================
// Offsets and Minkowski sums (Clipper2)

enum class join_type {
  miter,  // the offset edges extended to meet, up to miter_limit
  round,  // an arc, within arc_tolerance
  square, // squared off at the offset distance
  bevel   // the offset edges' ends joined directly
};

struct offset_params {
  join_type join = join_type::miter;
  // How far (a multiple of the offset distance) a mitre may reach before it is squared off.
  double miter_limit = 2.0;
  // How far (grid units) a rounded join may stray from the true arc.
  double arc_tolerance = 0.25;
};

// The set grown (delta > 0) or shrunk (delta < 0) by |delta| grid units, in one pass: every edge
// moved along its normal, the joins as `params` says, and the loops that leaves resolved. With
// round joins it is the exact Minkowski sum or difference with a disc (to arc_tolerance and
// rounding).
//
// A mitred offset in one pass is not always the straight skeleton's offset once an edge has shrunk
// to nothing on the way (an "edge event"). Where the collapse leaves a convex join the two agree;
// where it leaves a reflex join the skeleton mitres the vanished edge's neighbours, while a single
// pass still bounds the region by the vanished edge's line. tools/cgal_ab found that in 2 of 50
// cases (deep inward offsets of stars, 0.4% and 7.4% of the area). Use wavefront_offset for the
// skeleton's offset.
polygon_set offset(const polygon_set &s, double delta, const offset_params &params = {});

// The straight skeleton's offset (CGAL's create_{interior,exterior}_skeleton_and_offset_polygons,
// which ContourTiler's offset_polygon used): the wavefront of every edge moving at unit speed,
// mitred without limit. It runs as mitred offsets in steps that each end at the wavefront's next
// event (an edge shrinking to nothing, or a vertex reaching another edge), resolving the event
// before going on. The steps run on a grid 2^20 times finer, so the result rounds to the caller's
// grid once; a step is at least 1/16 of a unit. Equal to offset(s, delta, {miter, unlimited}) until
// the first edge event. tools/cgal_ab: it agrees with CGAL's straight skeleton to within rounding
// in all 50 cases, about 27x faster.
polygon_set wavefront_offset(const polygon_set &s, double delta);

// The Minkowski sum of two filled rings: {p + q : p in a, q in b}.
polygon_set minkowski_sum(const ring &a, const ring &b);

// ============================================================================================
// Triangulations (CDT)

// A triangulation of grid points.
struct triangulation {
  // The input points, in input order (duplicates kept: triangles use each point's first copy).
  std::vector<point> vertices;
  // Counterclockwise vertex indices.
  std::vector<std::array<int, 3>> triangles;
  // neighbors[t][i]: the triangle across the edge opposite triangles[t][i], or -1.
  std::vector<std::array<int, 3>> neighbors;
};

// The Delaunay triangulation of the points' convex hull. Points all on one line give no triangles.
// Cocircular points have several Delaunay triangulations; CDT picks one, the same on every run.
triangulation delaunay(const std::vector<point> &points);

// The constrained Delaunay triangulation of the points' convex hull: every edge (a pair of point
// indices) is an edge of the result, and no other point lies inside a triangle's circumcircle
// unless a constraint hides it. A constraint through another point is split there. Throws
// planar_error if two constraints cross, or an index names no point.
triangulation constrained_delaunay(const std::vector<point> &points,
                                   const std::vector<std::array<int, 2>> &edges);

// The constrained Delaunay triangulation of a polygon's inside (outer ring minus holes), every ring
// edge a constraint. Vertices are the outer ring's, then each hole's, as given (collinear vertices
// kept). A set's polygons are triangulated one by one and their vertices concatenated. Throws
// planar_error if the polygon is not simple (is_simple()).
triangulation triangulate(const polygon &p);
triangulation triangulate(const polygon_set &s);

// A simple polygon (holes allowed) as convex pieces, counterclockwise: Hertel-Mehlhorn on
// triangulate()'s triangles (each diagonal dropped when both its ends stay convex), as CGAL's
// approx_convex_partition_2 is on CGAL's triangulation. At most 4 times the fewest pieces possible,
// and for a polygon without holes at most 2r + 1 for r reflex vertices. Pieces keep the input's
// vertices, collinear ones included.
std::vector<ring> convex_partition(const polygon &p);

// ============================================================================================
// Sibson (natural-neighbour) interpolation

// A natural neighbour of a query point and its Sibson coordinate.
struct natural_neighbor {
  int vertex = -1;   // an index into the triangulation's vertices
  double weight = 0; // the share of the query's Voronoi cell taken from vertex's cell
};

// Sibson's natural-neighbour coordinates of q (grid units) in a Delaunay triangulation
// (delaunay()): positive weights summing to 1 that reproduce q (sum weight * vertex = q, to
// rounding). q at a vertex gives that vertex alone; q on the hull's boundary gives the two ends of
// its edge, linearly (the cell is unbounded there); q outside the hull gives none. q is found by a
// visibility walk from the first triangle (about sqrt(triangles) steps on spread-out points), and
// the rest costs the conflict region's size. ContourTiler's interp.cpp used CGAL's
// natural_neighbor_coordinates_2; tools/cgal_ab finds the weights equal to within 1e-14.
std::vector<natural_neighbor> sibson_coordinates(const triangulation &dt, const pointd &q);

// Sibson's linear interpolation of one value per vertex at q: sum weight * value over q's
// natural neighbours (CGAL's linear_interpolation). NaN outside the hull. Throws planar_error
// unless there is one value per vertex.
double sibson_interpolate(const triangulation &dt, const std::vector<double> &values,
                          const pointd &q);

// ============================================================================================
// The medial axis (Boost.Polygon's segment Voronoi diagram)

struct medial_axis_params {
  // How far (grid units) a sampled parabolic arc may stray from the true arc.
  double tolerance = 0.5;
};

// A polygon's medial axis: the points inside it with two or more nearest boundary points, as a
// graph of straight pieces (its parabolic arcs sampled within the tolerance).
struct medial_axis_graph {
  std::vector<pointd> vertices; // grid units
  std::vector<double> radius;   // each vertex's distance to the boundary (grid units)
  std::vector<std::array<int, 2>> edges;
  // The graph as polylines between vertices whose degree is not 2 (an end at a convex corner, or a
  // branch point), through the degree-2 vertices between. A loop of degree-2 vertices (around a
  // hole) is a polyline that ends where it starts.
  std::vector<std::vector<int>> polylines;
};

// The medial axis of a simple polygon (holes allowed): the edges of the Voronoi diagram of its
// edges that lie inside it, without those between an edge and its own endpoints (which run along
// the boundary's normals at reflex vertices). It reaches the boundary only at convex corners. It is
// the true medial axis, with parabolic arcs where a reflex vertex faces an edge, so it differs from
// the straight skeleton ContourTiler had there (the two agree on convex polygons). Collinear
// vertices are dropped first. Throws planar_error if the polygon is not simple or the tolerance is
// not positive.
medial_axis_graph medial_axis(const polygon &p, const medial_axis_params &params = {});

} // namespace planar
} // namespace cvc

#endif // __CVC_GEOMETRY_PLANAR_H__
