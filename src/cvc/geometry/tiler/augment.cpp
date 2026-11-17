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

#include "augment.h"

#include "Distance_functor.h"
#include "Segment_3_undirected.h"
#include "print_utils.h"
#include "sweep_line_visitors.h"

namespace cvc::tiler {

template <typename V1, typename V2> class Fork_visitor {
public:
  Fork_visitor(V1 v1, V2 v2) : _v1(v1), _v2(v2) {}
  ~Fork_visitor() {}

  bool end_end(const Point_2 &p, const Segment_2 &a, size_t a_id, const Segment_2 &b, size_t b_id) {
    return (_v1.end_end(p, a, a_id, b, b_id) && _v2.end_end(p, a, a_id, b, b_id));
  }
  bool end_int(const Point_2 &p, const Segment_2 &a, size_t a_id, const Segment_2 &b, size_t b_id) {
    return (_v1.end_int(p, a, a_id, b, b_id) && _v2.end_int(p, a, a_id, b, b_id));
  }
  bool int_int(const Point_2 &p, const Segment_2 &a, size_t a_id, const Segment_2 &b, size_t b_id) {
    return (_v1.int_int(p, a, a_id, b, b_id) && _v2.int_int(p, a, a_id, b, b_id));
  }

private:
  V1 _v1;
  V2 _v2;
};

template <typename V1, typename V2> Fork_visitor<V1, V2> fork_visitor(V1 v1, V2 v2) {
  return Fork_visitor<V1, V2>(v1, v2);
}

void insert_if(list<Point_2> &points, const Point_2 &p, bool force) {

  if (p.y() == 0) {
    TILER_DEBUG(logger, "Bad point! " << pp(p));
  }

  if (points.empty())
    points.push_back(p);
  else {
    Point_2 front = points.front();
    Point_2 back = points.back();
    if (xy_equal(p, back)) {
      TILER_DEBUG(logger, "Equal point (back)! " << pp(p));
    } else if (xy_equal(p, front)) {
      TILER_DEBUG(logger, "Equal point (front)! " << pp(p));
    }
    // if (compare_squared_distance(p, back, 0.0000000001) == SMALLER) {
    //   set_pp_precision(15);
    //   TILER_ERROR(logger, "Really close points: " << pp(p) << " " << pp(back));
    //   restore_pp_precision();
    // }
    //     if (compare_squared_distance(p, back, 0.00000001) == SMALLER)
    //     {
    //       if (force)
    //       {
    // 	TILER_TRACE(logger, "Erased point that was too close: " << pp(back));
    // 	points.pop_back();
    // 	points.push_back(p);
    //       }
    //       else
    //       {
    // 	// Don't insert the point
    // 	TILER_TRACE(logger, "Failing to insert point: " << pp(p));
    //       }
    //     }
    // //     else if (xy_equal(p, front))
    //     else if (compare_squared_distance(p, front, 0.00000001) == SMALLER)
    //     {
    //       if (force)
    //       {
    // 	TILER_TRACE(logger, "Erased point that was too close: " << pp(front));
    // 	points.pop_front();
    // 	points.push_back(p);
    //       }
    //       else
    //       {
    // 	// Don't insert the point
    // 	TILER_TRACE(logger, "Failing to insert point: " << pp(p));
    //       }
    //     }
    else {
      points.push_back(p);
      TILER_TRACE(logger, "Inserted point: " << pp(p));
    }
  }
}

template <typename Points_iter>
void insert_if(list<Point_2> &points, Points_iter begin, Points_iter end, bool force) {
  for (Points_iter it = begin; it != end; ++it)
    insert_if(points, *it, force);
}

pair<Polygon_2, Polygon_2> augment1(const Polygon_2 &P, const Polygon_2 &Q) {

  TILER_TRACE(logger, "Augmenting poly1 = " << pp(P));
  TILER_TRACE(logger, "Augmenting poly2 = " << pp(Q));

  // Get all segment intersections and put them in map,
  // that maps segments to their interior intersection points.
  std::list<Segment_2> s0, s1;
  s0.insert(s0.end(), P.edges_begin(), P.edges_end());
  s1.insert(s1.end(), Q.edges_begin(), Q.edges_end());

  list<SL_intersection> ints;
  //   get_intersections(s0.begin(), s0.end(),
  // 		    s1.begin(), s1.end(),
  // 		    back_inserter(ints),
  // 		    true, false);
  sweep_line(s0.begin(), s0.end(), s1.begin(), s1.end(),
             // 	     filter_visitor(fork_visitor(intersection_visitor(back_inserter(ints)),
             // 					 Debug_visitor()),
             filter_visitor(intersection_visitor(back_inserter(ints)), true, false));

  typedef boost::unordered_map<Segment_2, list<Point_2>> Map;
  Map map;
  get_interior_only(ints.begin(), ints.end(), map);

  bool has_intersections = false;
  for (Map::iterator it = map.begin(); it != map.end(); ++it) {
    const Segment_2 &segment = it->first;
    list<Point_2> &points = it->second;
    points.sort(dist_functor(segment.source()));

    has_intersections = has_intersections || !points.empty();
    for (list<Point_2>::iterator p_it = points.begin(); p_it != points.end(); ++p_it) {
      TILER_TRACE(logger, "Augmented point: " << pp(*p_it) << "(" << pp(segment) << ")");
    }
  }

  if (has_intersections) {
    TILER_TRACE(logger, "Augmenting:");
    TILER_TRACE(logger, "  P = " << pp(P));
    TILER_TRACE(logger, "  Q = " << pp(Q));
  }

  Number_type Pz = P[0].z();
  Number_type Qz = Q[0].z();

  // P
  list<Point_2> points;
  for (Polygon_2::Edge_const_iterator it = P.edges_begin(); it != P.edges_end(); ++it) {
    insert_if(points, it->source(), false);
    insert_if(points, map[*it].begin(), map[*it].end(), true);
  }
  for (list<Point_2>::iterator it = points.begin(); it != points.end(); ++it)
    it->z() = Pz;
  points.unique();
  Polygon_2 Pn(points.begin(), points.end());

  // Q
  points.clear();
  for (Polygon_2::Edge_const_iterator it = Q.edges_begin(); it != Q.edges_end(); ++it) {
    insert_if(points, it->source(), false);
    insert_if(points, map[*it].begin(), map[*it].end(), true);
  }
  for (list<Point_2>::iterator it = points.begin(); it != points.end(); ++it)
    it->z() = Qz;
  points.unique();
  Polygon_2 Qn(points.begin(), points.end());

  Polygon_2::Vertex_circulator begin = Pn.vertices_circulator();
  Polygon_2::Vertex_circulator c = begin;
  do {
    Polygon_2::Vertex_circulator p = c;
    Polygon_2::Vertex_circulator n = c;
    p--;
    n++;
    if (collinear(Point_2(*p), Point_2(*c), Point_2(*n))) {
      TILER_TRACE(logger, "Collinear point: " << pp(*c));
    }
    c++;
  } while (c != begin);

  begin = Qn.vertices_circulator();
  c = begin;
  do {
    Polygon_2::Vertex_circulator p = c;
    Polygon_2::Vertex_circulator n = c;
    p--;
    n++;
    if (collinear(Point_2(*p), Point_2(*c), Point_2(*n))) {
      TILER_TRACE(logger, "Collinear: " << pp(*c));
    }
    c++;
  } while (c != begin);

  TILER_TRACE(logger, "Augmented:");
  TILER_TRACE(logger, "  P = " << pp(Pn));
  TILER_TRACE(logger, "  Q = " << pp(Qn));

  return make_pair(Pn, Qn);
}

} // namespace cvc::tiler
