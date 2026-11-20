/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#include "skeleton.h"

#include "Vertices.h"
#include "common.h"
#include "interp.h"
#include "planar_bridge.h"
#include "print_utils.h"
#include "sweep_line_visitors.h"
#include "theorems.h"
#include "triangle_utils.h"
#include "xy_pred.h"

#include <boost/foreach.hpp>
#include <float.h>
#include <limits>
#include <time.h>

// Ported: the straight-skeleton fill (triangulate_to_medial_axis and its helpers, on CGAL's
// Straight_skeleton_2) was never called, as P3a found, and is gone. The live fill,
// medial_axis_stable(), keeps its shape: CGAL's approx_convex_partition_2 (and Greene's as the
// fallback) became cvc::planar::convex_partition, Hertel-Mehlhorn on CDT's triangulation, whose
// pieces are fewer and different (the roadmap's §3 and §9 item 2: untiled regions' interior
// vertices differ from the legacy's), and interpolate() is natural-neighbour interpolation on
// cvc::planar (interp.cpp).

namespace cvc::tiler {

Point_3 centroid(const Polygon_2 &P, Number_type z) {
  Number_type A = P.area();
  Number_type xsum = 0, ysum = 0;
  Polygon_2::Vertex_circulator start = P.vertices_circulator();
  Polygon_2::Vertex_circulator ci = start;
  Polygon_2::Vertex_circulator ci_next = start;
  ++ci_next;
  do {
    const Point_2 &cur = *ci;
    const Point_2 &next = *ci_next;
    xsum += (cur.x() + next.x()) * (cur.x() * next.y() - next.x() * cur.y());
    ysum += (cur.y() + next.y()) * (cur.x() * next.y() - next.x() * cur.y());
    ++ci;
    ++ci_next;
  } while (ci != start);

  Number_type x = xsum / (6 * A);
  Number_type y = ysum / (6 * A);

  //   typedef CGAL::Delaunay_triangulation_2<Kernel> DT;
  //   typedef std::vector<std::pair<Point_2, Number_type> > Point_coordinate_vector;

  //   DT cdt;
  //   cdt.insert(P.vertices_begin(), P.vertices_end());

  //   Point_coordinate_vector coords;
  //   CGAL::Triple<
  //     std::back_insert_iterator<Point_coordinate_vector>,
  //     Number_type, bool> result =
  //     CGAL::natural_neighbor_coordinates_2(cdt, Point_2(x, y),
  // 					 std::back_inserter(coords));
  //   if(!result.third){
  //     std::cout << "The coordinate computation was not successful."
  // 	      << std::endl;
  //     std::cout << "The point (" << pp(Point_2(x,y)) << ") lies outside the convex hull."
  // 	      << std::endl;
  //   }
  //   Number_type norm = result.second;

  //   Triangle tri(coords[0].first, coords[1].first, coords[2].first);
  //   z = get_z(tri, Point_2(x, y));

  return Point_3(x, y, z);
}

Untiled_region::const_iterator next(Untiled_region::const_iterator it,
                                    const Untiled_region &region) {
  ++it;
  if (it == region.end())
    it = region.begin();
  return it;
}

Untiled_region::const_iterator prev(Untiled_region::const_iterator it,
                                    const Untiled_region &region) {
  if (it == region.begin())
    it = region.end();
  --it;
  return it;
}

Polygon_2 to_polygon(const Untiled_region &region, map<Point_2, Point_2> &old2new) {

  Polygon_2 polygon;
  for (Untiled_region::const_iterator it = region.begin(); it != region.end(); ++it) {
    Point_2 p = *it;
    Point_2 n = *(next(it, region));
    if (!xy_equal(p, n)) {
      TILER_TRACE(logger, "Adding to polygon: " << pp(p));
      polygon.push_back(p);
    }
  }
  return polygon;
}

//------------------------------------------------------------------------------
// point_intersection_2
//
/// Returns true if segment a intersects with line b at a point in two
/// dimensions.  The intersection must occur on the projection onto the z plane.
/// Returns false if the two segments either don't intersect or if they
/// intersect on a segment.
//------------------------------------------------------------------------------
// THIS FUNCTION IS NOT GENERAL!
bool mid_int(const Segment_2 &a, const Segment_2 &b, Point_2 &ia) {
  Number_type numa = (b.target().x() - b.source().x()) * (a.source().y() - b.source().y()) -
                     (b.target().y() - b.source().y()) * (a.source().x() - b.source().x());
  Number_type numb = (a.target().x() - a.source().x()) * (a.source().y() - b.source().y()) -
                     (a.target().y() - a.source().y()) * (a.source().x() - b.source().x());
  Number_type den = (b.target().y() - b.source().y()) * (a.target().x() - a.source().x()) -
                    (b.target().x() - b.source().x()) * (a.target().y() - a.source().y());

  if (numa == 0 && numb == 0 && den == 0)
    // Intersection is a segment
    return false;

  if (den == 0)
    return false;
  Number_type ua = numa / den;
  Number_type ub = numb / den;
  // SEE?  NOT GENERAL!
  //   bool intersects = ua > 0 && ua < 1;
  bool intersects = ua > 0.1 && ua < 0.9;

  if (intersects) {
    ia = Point_2(a.source().x() + ua * (a.target().x() - a.source().x()),
                 a.source().y() + ua * (a.target().y() - a.source().y()));
  }

  return intersects;
}

Point_3 midpoint(const Segment_2_undirected &s, const Segment_2 &cross, Number_type z) {
  //   TILER_TRACE(logger, "Midpoint: " << pp(s.lesser()) << " " << pp(s.greater()));
  Point_2 ia;
  if (mid_int(s.segment(), cross, ia))
    return Point_3(ia.x(), ia.y(), z);
  return Point_3((s.lesser().x() + s.greater().x()) / 2.0, (s.lesser().y() + s.greater().y()) / 2.0,
                 z);
}

// typedef boost::unordered_map<Point_2, list<Segment_2_undirected> > Vertex2extras;
typedef boost::unordered_map<Point_2, list<Point_2>> Vertex2extras;

bool valid_as_start(Untiled_region::const_iterator it, const Untiled_region &region,
                    const Vertex2extras &vertex2extras, map<Point_2, Point_2> &old2new) {
  Point_2 p = *it;
  Point_2 n = *(next(it, region));
  Segment_2 edge(p, n);
  const Point_2 &source = old2new[edge.source()];
  const Point_2 &target = old2new[edge.target()];

  return (!xy_equal(source, target) && vertex2extras.find(source) == vertex2extras.end());
}

// Segment_2 as_source(const Segment_2_undirected& segment, const Point_2& p)
// {
//   Segment_2 s = segment.segment();
//   if (p == s.source())
//     return s;
//   if (p != s.target())
//     throw logic_error("p not equal to either endpoint of segment");
//   return s.opposite();
// }

class Angle_functor {
public:
  Angle_functor(const Segment_2 &segment) : _segment(segment) {}
  ~Angle_functor() {}

  //   bool operator()(const Segment_2_undirected& a, const Segment_2_undirected& b)
  //   {
  //     typedef Direction_2 Direction_2;
  //     Direction_2 pd(as_source(a, _segment.source()).direction());
  //     Direction_2 qd(as_source(b, _segment.source()).direction());
  //     Direction_2 d(_segment.direction());
  //     return pd.counterclockwise_in_between(qd, d);
  //   }

  bool operator()(const Point_2 &a, const Point_2 &b) {
    static const Number_type EPSILON = 0.00000000001;
    if (abs(a.x() - b.x()) < EPSILON && abs(a.y() - b.y()) < EPSILON) {
      return a.id() < b.id();
    }

    Direction_2 pd(Segment_2(_segment.source(), a).direction());
    Direction_2 qd(Segment_2(_segment.source(), b).direction());
    Direction_2 d(_segment.direction());
    return pd.counterclockwise_in_between(qd, d);
  }

private:
  Segment_2 _segment;
};

Triangle make_triangle(Point_3 a, Point_3 b, Point_3 c, bool reverse,
                       map<Point_2, Point_2> &new2old) {

  if (new2old.find(a) != new2old.end()) {
    TILER_TRACE(logger, "changing " << pp(a) << " to " << pp(new2old[a]));
    a = new2old[a];
  }
  if (new2old.find(b) != new2old.end())
    b = new2old[b];
  if (new2old.find(c) != new2old.end())
    c = new2old[c];

  if (reverse)
    return Triangle(c, b, a);
  return Triangle(a, b, c);
}

template <typename Out_iter> Number_type partition(const Polygon_2 &P, Out_iter out) {
  //   if (P.is_convex()) {
  //     *out++ = P;
  //     return;
  //   }
  //   TILER_TRACE(logger, "Not convex: " << pp(P));

  TILER_TRACE(logger, "partition(): " << pp(P));

  if (!P.is_simple()) {
    throw logic_error("Polygon given to partition is not simple: " + pp(P));
  }

  // Split the polygon into convex polygons. The pieces keep the polygon's vertices, so each piece
  // vertex maps back to the original point (with its z and id).
  typedef std::list<Polygon_2> Partition_polygons;
  Partition_polygons partition_polygons;
  try {
    const planar::grid g = fit_grid(P.bbox());
    std::map<std::pair<planar::coord, planar::coord>, Point_2> back;
    planar::ring ring;
    for (Polygon_2::Vertex_const_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
      const planar::point q = to_planar(*it, g);
      back[std::make_pair(q.x, q.y)] = *it;
      ring.push_back(q);
    }
    for (const planar::ring &piece : planar::convex_partition(planar::polygon(ring))) {
      // The pieces are convex on the grid. A vertex the tiler constructed (an augmenting point)
      // sits between grid points, so where three vertices are nearly collinear the turn the
      // original coordinates make can differ from the grid's: the check is on the grid.
      for (std::size_t i = 0; i < piece.size(); ++i)
        if (planar::orientation(piece[i], piece[(i + 1) % piece.size()],
                                piece[(i + 2) % piece.size()]) < 0)
          throw logic_error("planar::convex_partition gave a piece that is not convex");
      Polygon_2 p;
      for (const planar::point &q : piece)
        p.push_back(back.at(std::make_pair(q.x, q.y)));
      partition_polygons.push_back(p);
    }
  } catch (const std::exception &e) {
    throw logic_error("Convex partition failed: " + std::string(e.what()) + ": " + pp(P));
  }

  list<Polygon_2> polygons;
  list<Segment_2> segments;
  Number_type max_area = 0;
  for (Partition_polygons::iterator it = partition_polygons.begin(); it != partition_polygons.end();
       ++it) {
    Polygon_2 p(it->vertices_begin(), it->vertices_end());
    *out++ = p;
    if (p.area() > max_area) {
      max_area = p.area();
    }
    //     partition(p, out);

    //     if (!p.is_convex()) {
    //       TILER_ERROR(logger, "Somehow a non-convex polygon made it here: " << pp(p) << "
    //       Original polygon: " << pp(P)); throw logic_error("Somehow a non-convex polygon made it
    //       here");
    //     }
    //     polygons.push_back(p);
    //     TILER_TRACE(logger, "Convex polygon: " << pp(p));

    //     segments.insert(segments.end(), p.edges_begin(), p.edges_end());
  }
  return max_area;
}

string pp(const Triangle &T) {
  stringstream ss;
  for (int i = 0; i < 3; ++i) {
    ss << i << ": " << pp(T[i]) << " ";
  }
  return ss.str();
}

template <typename Tile_iterator, typename ID_factory>
void medial_axis_stable(const Untiled_region &r, Number_type zmid, Tile_iterator tiles,
                        ID_factory &id_factory, Tiler_workspace &tw) {
  run_to_end(medial_axis_stable_steps(r, zmid, tiles, id_factory, tw));
}

template <typename Tile_iterator, typename ID_factory>
step_task medial_axis_stable_steps(const Untiled_region &r, Number_type zmid, Tile_iterator tiles,
                                   ID_factory &id_factory, Tiler_workspace &tw) {

  Logger logger = tw.logger("tiler.medial_axis_stable");
  Untiled_region region(r);
  map<Point_2, Point_2> old2new, new2old;

  // Simplify the region into a polygon with no degenerate points.
  Polygon_2 P = to_polygon(region, old2new);

  TILER_TRACE(logger, "medial_axis_stable called with: " << pp(P));

  // Basic error checking
  if (!P.is_simple()) {
    TILER_WARN(logger, "Polygon being given to the medial axis splitter is not simple: " << pp(P));
    co_return;
  }

  if (P.size() == 0) {
    TILER_WARN(logger, "Polygon being given to the medial axis splitter is empty");
    co_return;
  }

  bool planar = true;
  Number_type planar_z = P[0].z();
  for (auto v_it = P.vertices_begin(); v_it != P.vertices_end(); ++v_it) {
    const Point_2 &v = *v_it;
    planar = (v.z() == planar_z);
  }
  TILER_TRACE(logger, "planar = " << planar);

  bool propagated = true;
  try {
    tw.propagate_z_home(P.vertices_begin(), P.vertices_end());
  } catch (logic_error &e) {
    TILER_ERROR(logger, "Failed to propagate z_home - aborting medial axis tile: " << pp(P));
    propagated = false;
  }
  if (!propagated)
    co_return;

  Number_type z_home;
  for (auto v_it = P.vertices_begin(); v_it != P.vertices_end(); ++v_it) {
    const Point_2 &v = *v_it;
    if (tw.has_z_home(v)) {
      z_home = tw.z_home(v);
      break;
    }
  }

  // Ensure validity (ccw)
  bool reverse = P.is_clockwise_oriented();
  if (reverse) {
    TILER_TRACE(logger, "Reversing untiled region polygon");
    region.reverse_orientation();
    old2new.clear();
    P = to_polygon(region, old2new);
  }

  // P = remove_duplicate_vertices(P);

  // Adjust points that are close to another edge, making the possibility of
  // nonsimple polygons.
  Polygon_2 old_P = P;
  P = adjust_nonsimple_polygon(P, 0.00001, old2new);
  for (auto p_it = old_P.vertices_begin(); p_it != old_P.vertices_end(); ++p_it) {
    const Point_2 &p = *p_it;
    if (old2new.find(p) == old2new.end()) {
      old2new[p] = p;
    } else {
      TILER_TRACE(logger, "old = " << pp(p) << " new = " << pp(old2new[p]));
    }
    new2old[old2new[p]] = p;
  }

  TILER_TRACE(logger, "Splitting polygon: " << pp(P));

  // Split the polygon into convex polygons
  list<Polygon_2> polygons;
  list<Segment_2> segments;
  bool partitioned = true;
  try {
    Number_type max_area = partition(P, std::back_inserter(polygons));
  } catch (logic_error &e) {
    TILER_ERROR(logger, "Failed to partition polygon - aborting medial axis tile: "
                            << e.what() << ": " << pp(P));
    TILER_ERROR(logger, "Original: " << pp(old_P));
    partitioned = false;
  }
  if (!partitioned)
    co_return;
  for (const auto &p : polygons) {
    segments.insert(segments.end(), p.edges_begin(), p.edges_end());
    TILER_TRACE(logger, "convex polygon area = " << p.area());
  }

  const Number_type zdiff = tw.zmax() - tw.zmin();

  // * anchor
  // + polygon center
  // x midpnt
  //
  //       *------------------------*
  //        \                       |
  //         \                      |
  //          \                     |
  //           \          +         |
  //            \         |         |
  //             \        |         |
  //              \       |         |
  //               *------x---------*
  //              /       |         |
  //             /        |         |
  //            /         |         |
  //           /          +         |
  //          /                     |
  //         /                      |
  //        *-----------------------*
  //
  //   edge2center maps *--*  ---->  +
  // vertex2extras maps   *   ---->  + x

  list<Point_2> centers;
  list<Point_2> anchors(P.vertices_begin(), P.vertices_end());

  // Create a map of edges and their associated polygon center points.  The center
  // points are the center of the convex polygon the edge belongs to.
  boost::unordered_map<Segment_2, Point_3> edge2center;
  Number_type largest_area = std::numeric_limits<Number_type>::min();
  Point_2 largest_centroid;
  for (const auto &p : polygons) {
    Number_type area = p.area();
    Point_3 center = centroid(p, zmid);
    const size_t id = id_factory.unique_id();

    if (!p.has_on_bounded_side(center)) {
      TILER_TRACE(logger, "Not simple.  Polygon: " << pp(p));
      TILER_TRACE(logger, "Not simple.  Area: " << p.area());
      Bbox_2 bb = p.bbox();
      center = Point_3((bb.xmin() + bb.xmax()) / 2.0, (bb.ymin() + bb.ymax()) / 2.0, zmid);
    }
    center.id() = id;
    tw.set_z_home(center, z_home);
    TILER_TRACE(logger, "center: " << pp(center) << " " << pp(p));

    for (Polygon_2::Edge_const_iterator e_it = p.edges_begin(); e_it != p.edges_end(); ++e_it) {
      edge2center[*e_it] = center;
    }

    // If we're tiling an end contour (only one contour tiling to the z midpoint)
    // then we're going to interpolate using the polygon and the centroid of the
    // largest sub-polygon, which will be at the midpoint.
    if (planar) {
      if (p.area() > largest_area) {
        if (largest_area > std::numeric_limits<Number_type>::min()) {
          centers.push_back(largest_centroid);
        }
        largest_centroid = center;
        largest_area = p.area();
      } else {
        centers.push_back(center);
      }
    } else {
      centers.push_back(center);
    }
  }

  boost::unordered_map<Point_2, Number_type> point2z;
  if (planar) {
    anchors.push_back(largest_centroid);
    point2z[largest_centroid] = largest_centroid.z();
  }

  // Find shared edges (cuts) between convex polygons
  list<SL_intersection> intersections;
  get_intersections(segments.begin(), segments.end(), back_inserter(intersections), true, true);

  Vertex2extras vertex2extras;
  for (const auto &i : intersections) {
    const list<Segment_2> &ends = i.ends();
    if (ends.size() == 2) {
      Segment_2_undirected a(*ends.begin());
      Segment_2_undirected b(*ends.rbegin());
      if (a == b) {
        const Segment_2 &edge1 = *ends.begin();
        const Segment_2 &edge2 = *ends.rbegin();
        const Point_3 &center1 = edge2center[edge1];
        const Point_3 &center2 = edge2center[edge2];
        // 	Point_3 midpnt = midpoint(a, Segment_2(center1, center2), zmid);
        Point_3 midpnt =
            midpoint(a, Segment_2(center1, center2), (center1.z() + center2.z()) / 2.0);
        midpnt.id() = id_factory.unique_id();
        TILER_TRACE(logger, "midpnt: " << pp(midpnt) << " " << pp(Segment_2(center1, center2)));
        tw.set_z_home(midpnt, z_home);
        centers.push_back(midpnt);

        vertex2extras[a.lesser()].push_back(midpnt);
        vertex2extras[a.lesser()].push_back(center1);
        vertex2extras[a.lesser()].push_back(center2);

        vertex2extras[a.greater()].push_back(midpnt);
        vertex2extras[a.greater()].push_back(center1);
        vertex2extras[a.greater()].push_back(center2);

        TILER_TRACE(logger, "Inserting common edge: " << pp(a.orig_segment()));
      }
    }
  }

  Number_type anchor_min = std::numeric_limits<Number_type>::max();
  Number_type anchor_max = std::numeric_limits<Number_type>::min();
  for (list<Point_2>::iterator it = anchors.begin(); it != anchors.end(); ++it) {
    anchor_min = min(anchor_min, it->z());
    anchor_max = max(anchor_max, it->z());
  }
  const Number_type anchor_mid = (anchor_min + anchor_max) / 2.0;
  // Draw points slightly in toward center to avoid numerical problem causing
  // center points on slices.
  for (list<Point_2>::iterator it = anchors.begin(); it != anchors.end(); ++it) {
    it->z() = it->z() * 0.99 + anchor_mid * 0.01;
  }

  SK_RUN(interpolate_steps(anchors.begin(), anchors.end(), centers.begin(), centers.end(), zmid));

  for (const auto &c : centers) {
    if (c.z() > anchor_max || c.z() < anchor_min) {
      TILER_ERROR(logger, "Bad interpolation: failing.  z = " << c.z() << ", min = " << anchor_min
                                                              << ", max = " << anchor_max
                                                              << "  Anchors:");
      for (const auto &a : anchors) {
        TILER_DEBUG(logger, "  Anchors = " << pp(a));
      }
      for (const auto &p : polygons) {
        TILER_DEBUG(logger, "  Polygon = " << pp(p));
      }
      TILER_DEBUG(logger, "  Original polygon = " << pp(P));
      co_return;
      // TILER_TRACE(logger, "Bad interpolation: failing.  z = " << c.z()
      //   	      << ", min = " << anchor_min << ", max = " << anchor_max << "  Anchors:");
      // for (const auto& a : anchors) {
      //   TILER_TRACE(logger, "  " << pp(a));
      // }
      // for (const auto& p : polygons) {
      //   TILER_TRACE(logger, "  " << pp(p));
      // }
      // point2z[c] = anchor_mid;
    } else {
      point2z[c] = c.z();
    }
  }

  // Sort the cuts in ccw order
  typedef Untiled_region::const_iterator Iter;
  for (Iter it = region.begin(); it != region.end(); ++it) {
    Point_2 vertex = old2new[*it];
    if (vertex2extras.find(vertex) != vertex2extras.end()) {
      Point_2 previous = *prev(it, region);
      Segment_2 segment(vertex, previous);
      list<Point_2> &extras = vertex2extras[vertex];
      extras.sort(Angle_functor(segment));
      extras.unique();

      // debug
      if (vertex.id() == 1397 && previous.id() == 1398) {
        for (const auto &p : extras) {
          TILER_TRACE(logger, "Extra: " << pp(p));
        }
      }
      // /debug

      // The first point will be taken care of by standard adding
      extras.pop_front();
    }
  }

  // Create the tiles by iterating over the original untiled region.
  Iter begin = region.begin();
  while (!valid_as_start(begin, region, vertex2extras, old2new)) {
    TILER_TRACE(logger, "Not valid as start: " << pp(*begin));
    ++begin;
  }

  Iter it = begin;
  Point_3 cur;
  do {
    Point_2 p = old2new[*it];
    Point_2 n = old2new[*(next(it, region))];
    Segment_2 edge(p, n);
    Segment_2_undirected edge_u(edge);
    const Point_2 &source = edge.source();
    const Point_2 &target = edge.target();

    TILER_TRACE(logger, "Tiling edge: " << pp(edge));

    // If the source and target are equal, it's a double: tile
    // to the current point (centroid of the polygon)
    if (xy_equal(source, target)) {
      try {
        Triangle T = make_triangle(source, target, cur, reverse, new2old);
        if (!T.is_degenerate()) {
          *tiles++ = T;
        } else {
          TILER_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
        }
        TILER_TRACE(logger, "Made triangle: " << pp(T));
      } catch (std::logic_error &e) {
        TILER_WARN(logger, "Failed to make triangle: " << pp(source) << " " << pp(target) << " "
                                                       << pp(cur) << " " << e.what());
      }
    }

    // Otherwise:
    //   1) tile from the current edge to the centroid
    //   2) tile from the target to extra to centroid
    else {
      cur = edge2center[edge];
      cur = Point_3(cur.x(), cur.y(), point2z[cur], cur.id());
      try {
        Triangle T = make_triangle(source, target, cur, reverse, new2old);
        if (!T.is_degenerate()) {
          *tiles++ = T;
        } else {
          TILER_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
        }
        TILER_TRACE(logger, "Made triangle: " << pp(T));
      } catch (std::logic_error &e) {
        TILER_WARN(logger, "Failed to make triangle: " << pp(source) << " " << pp(target) << " "
                                                       << pp(cur) << " " << e.what());
      }

      if (vertex2extras.find(target) != vertex2extras.end()) {
        // const list<Point_2>& extras = vertex2extras[target];
        // for (list<Point_2>::const_iterator e_it = extras.begin(); e_it != extras.end(); ++e_it) {
        //   Point_3 next = *e_it;
        const vector<Point_2> extras(vertex2extras[target].begin(), vertex2extras[target].end());
        for (int i = 0; i < extras.size(); ++i) {
          Point_3 next = extras[i];
          next = Point_3(next.x(), next.y(), point2z[next], next.id());
          // 	  next.z() = point2z[next];

          try {
            Triangle T = make_triangle(target, next, cur, reverse, new2old);
            if (!T.is_degenerate()) {
              *tiles++ = T;
            } else {
              TILER_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
            }
            TILER_TRACE(logger, "Made triangle: " << pp(T));
          } catch (std::logic_error &e) {
            TILER_WARN(logger, "Failed to make triangle: " << pp(target) << " " << pp(next) << " "
                                                           << pp(cur) << " " << e.what());
          }
          cur = next;
        }
      }
    }
    it = next(it, region);
  } while (it != begin);
}

template void medial_axis_stable(const Untiled_region &region, Number_type zmid,
                                 back_insert_iterator<list<Triangle>> tiles, Vertices &id_factory,
                                 Tiler_workspace &tw);

template void medial_axis_stable(const Untiled_region &region, Number_type zmid,
                                 back_insert_iterator<vector<Triangle>> tiles, Vertices &id_factory,
                                 Tiler_workspace &tw);

template step_task medial_axis_stable_steps(const Untiled_region &region, Number_type zmid,
                                            back_insert_iterator<list<Triangle>> tiles,
                                            Vertices &id_factory, Tiler_workspace &tw);

template step_task medial_axis_stable_steps(const Untiled_region &region, Number_type zmid,
                                            back_insert_iterator<vector<Triangle>> tiles,
                                            Vertices &id_factory, Tiler_workspace &tw);

// void to_simple(const Untiled_region& region)
// {
//   log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.to_simple");

//   TILER_TRACE(logger, "Beginning polygon");

//   Polygon_2 P = to_polygon(region);
//   typedef Polygon_2::iterator Iter;
//   for (Iter it = P.vertices_begin(); it != P.vertices_end(); ++it) {
//     Iter next = it;
//     ++next;
//     if (find_if(next, P.vertices_end(), xy_pred(*it)) != P.vertices_end())
//       TILER_TRACE(logger, pp(*it));
//   }
// }

} // namespace cvc::tiler
