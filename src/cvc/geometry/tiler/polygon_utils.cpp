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

#include "polygon_utils.h"

#include "print_utils.h"
#include "set_utils.h"
#include "sweep_line_visitors.h"
#include "triangle_utils.h"

#include <boost/foreach.hpp>

namespace cvc::tiler {

// /// Returns true if there are any polygon boundary intersections.
// /// Precondition: InputIterator iterates over CGAL::Polygon_2 objects.
// template <typename PolyIterator, typename OutputIterator>
// void get_boundary_intersections(PolyIterator begin, PolyIterator end, OutputIterator
// intersecting_points, bool report_endpoints)
// {
//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("boundary_intersections");

//   std::list<Segment_2> segments;
//   for (PolyIterator it = begin; it != end; ++it)
//     segments.insert(segments.end(), it->edges_begin(), it->edges_end());

//   list<SL_intersection> ints;
//   get_intersections(segments.begin(), segments.end(), back_inserter(ints), report_endpoints,
//   report_endpoints);

//   boost::unordered_set<Point_2> points;
//   for (list<SL_intersection>::iterator it = ints.begin(); it != ints.end(); ++it)
//     points.insert(it->point());
//   copy(points.begin(), points.end(), intersecting_points);
// }

/// Returns true if there are any polygon boundary intersections.
/// Precondition: InputIterator iterates over CGAL::Polygon_2 objects.
boost::unordered_set<Point_2> get_boundary_intersections(const std::list<Segment_2> &segments,
                                                         bool report_endpoints) {

  list<SL_intersection> ints;
  get_intersections(segments.begin(), segments.end(), back_inserter(ints), report_endpoints,
                    report_endpoints);

  boost::unordered_set<Point_2> points;
  for (list<SL_intersection>::iterator it = ints.begin(); it != ints.end(); ++it)
    points.insert(it->point());
  return points;
}

bool boundaries_intersect(const Polygon_2 &P, const Polygon_2 &Q, bool proper) {
  std::list<Segment_2> segments0, segments1;
  segments0.insert(segments0.end(), P.edges_begin(), P.edges_end());
  segments1.insert(segments1.end(), Q.edges_begin(), Q.edges_end());

  bool intersects = has_intersection(segments0.begin(), segments0.end(), segments1.begin(),
                                     segments1.end(), true, true);
  if (!intersects || !proper)
    return intersects;

  if (has_intersection(segments0.begin(), segments0.end(), segments1.begin(), segments1.end(),
                       false, false))
    return true;

  typedef Polygon_2::Vertex_const_iterator Iter;
  for (Iter it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    if (Q.has_on_bounded_side(*it))
      return true;
  }
  for (Iter it = Q.vertices_begin(); it != Q.vertices_end(); ++it) {
    if (P.has_on_bounded_side(*it))
      return true;
  }
  return false;
}

bool intersects_boundary(const Polygon_2 &P, const Segment_2 &s, bool end_internal, bool end_end) {
  std::list<Segment_2> segments0, segments1;
  segments0.insert(segments0.end(), P.edges_begin(), P.edges_end());
  segments1.push_back(s);

  return has_intersection(segments0.begin(), segments0.end(), segments1.begin(), segments1.end(),
                          end_internal, end_end);
}

bool intersects_boundary(const Point_2 &point, const Polygon_2 &P) {
  Polygon_2::Edge_const_iterator it = P.edges_begin();
  for (; it != P.edges_end(); ++it) {
    if (do_intersect(point, *it))
      return true;
  }
  return false;
}

bool do_intersect(const Polygon_2 &P, const Polygon_2 &Q) {
  //   return do_intersect(P, Q);
  if (boundaries_intersect(P, Q, true))
    return true;
  return P.has_on_positive_side(Q[0]) || Q.has_on_positive_side(P[0]);
}

bool collinear(const Polygon_2 &P) {
  return false;
  // Need to find the root of the problem.
  // return CGAL::orientation_2(P.vertices_begin(), P.vertices_end()) == COLLINEAR;
}

Polygon_relation relation(const Polygon_2 &subject, const Polygon_2 &relative, bool proper) {
  typedef Polygon_with_holes_2 Polygon_with_holes;

  Polygon_relation ret = Polygon_relation::BOUNDARY_INTERSECT;

  const Polygon_2 &P = subject;
  const Polygon_2 &Q = relative;
  // Yes, !proper is weird -- see comments to relation() in polygon_utils.h.
  if (boundaries_intersect(P, Q, !proper))
    ret = Polygon_relation::BOUNDARY_INTERSECT;
  else if (!collinear(P) && P.has_on_positive_side(Q[0]))
    ret = Polygon_relation::PARENT;
  else if (!collinear(Q) && Q.has_on_positive_side(P[0]))
    ret = Polygon_relation::CHILD;
  else
    ret = Polygon_relation::SIBLING;

  //   // Do a first check to see if they intersect
  //   if (do_intersect(subject, relative))
  //   {
  //     std::list<Polygon_with_holes> result;
  //     intersection(subject, relative, std::back_inserter(result));
  //     Polygon_with_holes& intersection = result.front();
  //     if (result.size() > 1 || intersection.holes_begin() != intersection.holes_end() ||
  //     boundaries_intersect(subject, relative))
  //     {
  //       // Intersection is multiple polygons or polygon with holes
  //       ret = Polygon_relation::BOUNDARY_INTERSECT;
  //     }
  //     else if (intersection.outer_boundary() == subject)
  //     {
  //       ret = Polygon_relation::CHILD;
  //     }
  //     else if (intersection.outer_boundary() == relative)
  //     {
  //       ret = Polygon_relation::PARENT;
  //     }
  //     else
  //     {
  //       // Intersection is a proper subset of both polygons
  //       ret = Polygon_relation::BOUNDARY_INTERSECT;
  //     }
  //   }
  //   else
  //   {
  //     ret = Polygon_relation::SIBLING;
  //   }

  return ret;
}

Polygon_2 remove_collinear(const Polygon_2 &p, Number_type epsilon) {

  TILER_TRACE(logger, pp(p));

  if (!p.is_simple()) {
    stringstream ss;
    ss << "Polygon is not simple in remove_collinear: " << pp(p);
    throw logic_error(ss.str());
  }

  Polygon_2::Vertex_circulator start = p.vertices_circulator();
  Polygon_2::Vertex_circulator c = start;
  Polygon_2::Vertex_circulator n = c;
  Polygon_2::Vertex_circulator prev = c;
  ++n;
  --prev;
  Polygon_2 newp;
  do {
    Triangle_2 t(*prev, *c, *n);
    Number_type a = abs(t.area());
    if (a > epsilon)
      //     if (!collinear(*prev, *c, *n))
      newp.push_back(*c);
    else
      TILER_TRACE(logger, "Removing collinearity at " << pp(*c) << " area = " << a);
    ++prev;
    ++c;
    ++n;
  } while (c != start);

  return newp;
}

class Cross_checker {
public:
  Cross_checker() : _side(ON_BOUNDARY), _crosses(false) {}

  bool crosses() { return _crosses; }
  void add(Bounded_side side) {
    // We already know that we cross -- no need to check more
    if (_crosses)
      return;
    // Something on the boundary gives us no information
    if (side == ON_BOUNDARY)
      return;

    if (_side == ON_BOUNDARY)
      _side = side;
    else
      _crosses = (_side != side);
  }

private:
  Bounded_side _side;
  bool _crosses;
};

/// Returns true if the segment crosses a boundary line of the polygon.
/// It is not enough for the segment to simply intersect the boundary
/// line -- it must cross it such that the segment extends to both inside
/// and outside of the boundary of the polygon.
bool intersects_proper(const Segment_2 &segment, const Polygon_2 &polygon) {

  Point_2 seg_pts[] = {segment.source(), segment.target()};

  // Get intersection points between segment and polygon
  TILER_TRACE(logger, "Doing a line sweep: " << pp(segment));
  list<Segment_2> segments(polygon.edges_begin(), polygon.edges_end());
  segments.push_back(segment);
  list<Point_2> points;
  get_intersection_points(segments.begin(), segments.end(), back_inserter(points), true, false);

  Bounded_side side1 = polygon.bounded_side(seg_pts[0]);
  Bounded_side side2 = polygon.bounded_side(seg_pts[1]);

  TILER_TRACE(logger, "Checking with cross checker");
  if (points.size() == 0)
    return false;
  Cross_checker checker;
  checker.add(side1);
  checker.add(side2);
  if (checker.crosses())
    return true;
  if (points.size() == 1)
    return false;

  points.push_back(seg_pts[0]);
  points.push_back(seg_pts[1]);
  points.sort();
  list<Point_2>::iterator it0 = points.begin();
  list<Point_2>::iterator it1 = it0;
  ++it1;
  while (it1 != points.end()) {
    const Point_2 &p0 = *it0;
    const Point_2 &p1 = *it1;

    // find an intermediate point and test for where it is
    Point_2 midpoint((p0.x() + p1.x()) / 2.0, (p0.y() + p1.y()) / 2.0);
    checker.add(polygon.bounded_side(midpoint));
    if (checker.crosses())
      return true;

    ++it0;
    ++it1;
  }
  return false;
}

//------------------------------------------------------------------------------
// prev
//
/// Returns the previous vertex in the polygon, crossing the end iterator
/// boundary if necessary
//------------------------------------------------------------------------------
Polygon_2::Vertex_const_iterator prev(Polygon_2::Vertex_const_iterator iter, const Polygon_2 &p) {
  if (iter == p.vertices_begin())
    return p.vertices_end() - 1;
  return iter - 1;
}

//------------------------------------------------------------------------------
// next
//
/// Returns the next vertex in the polygon, crossing the end iterator
/// boundary if necessary
//------------------------------------------------------------------------------
Polygon_2::Vertex_const_iterator next(Polygon_2::Vertex_const_iterator iter, const Polygon_2 &p) {
  if (iter + 1 == p.vertices_end())
    return p.vertices_begin();
  return iter + 1;
}

Point_3 yz_swap_pos(const Point_3 &p) { return Point_3(p.x(), -p.z(), p.y()); }

Point_3 yz_swap_neg(const Point_3 &p) { return Point_3(p.x(), p.z(), -p.y()); }

Polygon_2 yz_swap_pos(const Polygon_2 &p) {
  Polygon_2 ret;
  for (Polygon_2::Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
    ret.push_back(yz_swap_pos(*it));
  return ret;
}

Polygon_2 yz_swap_neg(const Polygon_2 &p) {
  Polygon_2 ret;
  for (Polygon_2::Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
    ret.push_back(yz_swap_neg(*it));
  return ret;
}

Polyline_2 yz_swap_neg(const Polyline_2 &p) {
  Polyline_2 ret;
  for (Polyline_2::const_iterator it = p.begin(); it != p.end(); ++it)
    ret.push_back(yz_swap_neg(*it));
  return ret;
}

Point_3 xz_swap_pos(const Point_3 &p) { return Point_3(-p.z(), p.y(), p.x()); }

Point_3 xz_swap_neg(const Point_3 &p) { return Point_3(p.z(), p.y(), -p.x()); }

Polygon_2 xz_swap_pos(const Polygon_2 &p) {
  Polygon_2 ret;
  for (Polygon_2::Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
    ret.push_back(xz_swap_pos(*it));
  return ret;
}

Polygon_2 xz_swap_neg(const Polygon_2 &p) {
  Polygon_2 ret;
  for (Polygon_2::Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
    ret.push_back(xz_swap_neg(*it));
  return ret;
}

Polyline_2 xz_swap_neg(const Polyline_2 &p) {
  Polyline_2 ret;
  for (Polyline_2::const_iterator it = p.begin(); it != p.end(); ++it)
    ret.push_back(xz_swap_neg(*it));
  return ret;
}

// Point_3 rotate_x(const Point_3& p, Number_type q)
// {
//   return Point_3(p.x(),
// p.y() * cos(q) - p.z() * sin(q),
// p.y() * sin(q) + p.z() * cos(q));
// }

// Polygon_2 rotate_x(const Polygon_2& p, Number_type q)
// {
//   Polygon_2 ret;
//   for (Polygon_2::Vertex_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
//     ret.push_back(rotate_x(*it, q));
//   return ret;
// }

bool is_vertical(const Polygon_2 &p) {
  for (int i = 0; i < p.size() - 1; ++i)
    for (int j = i + 1; j < p.size(); ++j)
      if (xy_equal(p[i], p[j]))
        return true;
  //   for (int i = 0; i < p.size() - 2; ++i)
  //     if (!collinear(p[i], p[i+1], p[i+2]))
  //       return false;
  //   return true;
  return false;
}

// Makes sure that no three points all on an original tile edge
// are triangulated.
bool is_legal(
    const Point_3 &a, const Point_3 &b, const Point_3 &c,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {

  TILER_TRACE(logger, "is_legal: " << pp(a) << " " << pp(b) << " " << pp(c));

  if (point2edges.find(a) == point2edges.end())
    return true;
  if (point2edges.find(b) == point2edges.end())
    return true;
  if (point2edges.find(c) == point2edges.end())
    return true;

  const boost::unordered_set<Segment_3_undirected> &edgesa = point2edges.find(a)->second;
  const boost::unordered_set<Segment_3_undirected> &edgesb = point2edges.find(b)->second;
  const boost::unordered_set<Segment_3_undirected> &edgesc = point2edges.find(c)->second;

  boost::unordered_set<Segment_3_undirected> edges(
      set_intersection(edgesa, set_intersection(edgesb, edgesc)));
  return edges.empty();
}

// template <typename Polygon, typename Triangle_iter>
// void triangulate_convex(const Polygon& polygon, Triangle_iter triangles,
// const boost::unordered_map<Point_3, list<Segment_3_undirected> >& point2edges)
// {
//   if (!polygon.is_convex())
//     throw logic_error("Polygon is not convex");

//   typename Polygon::Vertex_circulator start = polygon.vertices_circulator();
//   typename Polygon::Vertex_circulator c = start;
//   Point_2 p(*c);
//   ++c;
//   typename Polygon::Vertex_circulator n = c;
//   ++n;
//   do
//   {
//     Triangle t(p, *c, *n);
//     *triangles++ = t;
//     ++c;
//     ++n;
//   } while (n != start);
// }

bool is_strictly_convex(
    const Polygon_2 &polygon,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {
  Polygon_2::Vertex_circulator start = polygon.vertices_circulator();
  Polygon_2::Vertex_circulator c = start;
  Polygon_2::Vertex_circulator p = c;
  Polygon_2::Vertex_circulator n = c;
  --p;
  ++n;
  do {
    if (!left_turn(*p, *c, *n) || !is_legal(*p, *c, *n, point2edges))
      return false;

    ++p;
    ++c;
    ++n;
  } while (c != start);
  return true;
}

bool can_cut(
    const Polygon_2 &polygon, Polygon_2::Vertex_circulator p, Polygon_2::Vertex_circulator c,
    Polygon_2::Vertex_circulator n,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {
  Segment_2 seg(*p, *n);
  Point_2 mid((p->x() + n->x()) / 2.0, (p->y() + n->y()) / 2.0);
  return (left_turn(*p, *c, *n) && is_legal(*p, *c, *n, point2edges) &&
          !intersects_boundary(polygon, seg, true, false) && polygon.has_on_bounded_side(mid));
}

template <typename Triangle_iter>
Polygon_2 cut_ear(
    const Polygon_2 &polygon, Triangle_iter triangles,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {

  TILER_TRACE(logger, "Cutting ear of " << pp(polygon));
  ;

  Polygon_2 ret(polygon);
  Polygon_2::Vertex_circulator start = ret.vertices_circulator();
  Polygon_2::Vertex_circulator c = start;
  Polygon_2::Vertex_circulator p = c - 1;
  Polygon_2::Vertex_circulator n = c + 1;
  // Do a run preferring to get rid of collinear points
  do {
    Polygon_2::Vertex_circulator pp = p - 1;
    if (!is_legal(*pp, *p, *c, point2edges) && can_cut(polygon, p, c, n, point2edges))
    //     if (!is_legal(*pp, *p, *c, point2edges) &&
    // is_legal(*p, *c, *n, point2edges))
    {
      *triangles++ = Triangle(*p, *c, *n);
      ret.erase(c);
      return ret;
    }
    Polygon_2::Vertex_circulator ppp = pp - 1;
    if (!is_legal(*pp, *p, *c, point2edges) && can_cut(polygon, ppp, pp, p, point2edges))
    //     if (!is_legal(*pp, *p, *c, point2edges) &&
    // is_legal(*p, *c, *n, point2edges))
    {
      *triangles++ = Triangle(*ppp, *pp, *p);
      ret.erase(pp);
      return ret;
    }
    ++p;
    ++c;
    ++n;
  } while (c != start);

  // Okay, just take any cut
  do {
    if (can_cut(polygon, p, c, n, point2edges)) {
      *triangles++ = Triangle(*p, *c, *n);
      ret.erase(c);
      return ret;
    }
    ++p;
    ++c;
    ++n;
  } while (c != start);

  // Okay, really just take any cut
  do {
    //     if (can_cut(polygon, p, c, n, point2edges))
    if (is_legal(*p, *c, *n, point2edges)) {
      *triangles++ = Triangle(*p, *c, *n);
      ret.erase(c);
      return ret;
    }
    ++p;
    ++c;
    ++n;
  } while (c != start);

  TILER_DEBUG(logger, "Polygon is not strictly convex");
  TILER_DEBUG(logger, "  Original: " << pp(polygon));
  TILER_DEBUG(logger, "   Current: " << pp(ret));
  throw logic_error("Polygon is not strictly convex");
}

template <typename Triangle_iter>
void triangulate_convex(
    const Polygon_2 &P, Triangle_iter triangles,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {

  if (!is_strictly_convex(P, point2edges)) {
    TILER_TRACE(logger, "Attempting to triangulate non-convex polygon: " << pp(P));
    //     throw logic_error("Polygon is not strictly convex");
  }

  Polygon_2 polygon(P);
  while (polygon.size() > 3)
    polygon = cut_ear(polygon, triangles, point2edges);

  *triangles++ = Triangle(polygon[0], polygon[1], polygon[2]);
}

template <typename Triangle_iter>
void triangulate_safe(
    const Polygon_2 &P, Triangle_iter triangles,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges) {

  Polygon_2 polygon(P);
  TILER_TRACE(tlogger, "Triangulating " << pp(polygon));
  if (polygon.size() < 3)
    return;

  bool vertical_yz = is_vertical(polygon);
  bool vertical_xz = false;
  if (vertical_yz) {
    TILER_TRACE(tlogger, "Polygon is vertical.  Rotating.");
    Polygon_2 temp = yz_swap_neg(polygon);
    vertical_xz = is_vertical(temp);
    if (vertical_xz) {
      temp = xz_swap_neg(polygon);
      vertical_yz = false;
    }
    polygon = temp;
  }

  if (!polygon.is_simple()) {
    // The legacy logged this and returned; both callers catch the error and warn, so the polygon is
    // skipped as before and the warning reaches the caller's log.
    throw logic_error("Attempting (and failing) to triangulate non-simple polygon " + pp(polygon));
  }

  bool reverse = !polygon.is_counterclockwise_oriented();
  if (reverse)
    polygon.reverse_orientation();
  if (!polygon.is_counterclockwise_oriented())
    TILER_TRACE(tlogger, "Unexpected polygon orientation: " << polygon.orientation());

  list<Triangle> tris;

  triangulate_convex(polygon, back_inserter(tris), point2edges);

  //   // Split the polygon into convex polygons
  //   typedef std::list<PT::Polygon_2> Partition_polygons;
  //   Partition_polygons partition_polygons;
  //   CGAL::approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(),
  // std::back_inserter(partition_polygons));

  //   for (Partition_polygons::iterator it = partition_polygons.begin(); it !=
  //   partition_polygons.end(); ++it)
  //   {
  //     Polygon_2 p(it->vertices_begin(), it->vertices_end());
  //     triangulate_convex(p, back_inserter(tris), point2edges);
  //   }

  for (list<Triangle>::iterator it = tris.begin(); it != tris.end(); ++it) {
    Triangle t;
    for (int i = 0; i < 3; ++i) {
      int idx = reverse ? 2 - i : i;
      if (vertical_yz)
        t[idx] = yz_swap_pos((*it)[i]);
      else if (vertical_xz)
        t[idx] = xz_swap_pos((*it)[i]);
      else
        t[idx] = (*it)[i];
    }
    TILER_TRACE(tlogger, "Adding tile: " << pp_tri(t));
    *triangles++ = t;
  }
}

int distance(const Point_2 &p, const Point_2 &q, const Polygon_2 &P) {
  typedef Polygon_2::Vertex_const_iterator Iter;
  typedef iterator_traits<Iter>::difference_type Diff;

  Iter pit = find(P.vertices_begin(), P.vertices_end(), p);
  Iter qit = find(P.vertices_begin(), P.vertices_end(), q);
  if (pit == P.vertices_end() || qit == P.vertices_end()) {
    return -1;
  }

  Diff pbeg = std::distance(P.vertices_begin(), pit);
  Diff qbeg = std::distance(P.vertices_begin(), qit);
  if (qbeg < pbeg) {
    std::swap(pit, qit);
    std::swap(pbeg, qbeg);
  }
  Diff pq = std::distance(pit, qit);
  Diff qend = std::distance(qit, P.vertices_end());

  return (int)min(pbeg + qend, pq);
}

// template <typename Out_iter>
// void convex_partition(const Polygon& P, Out_iter out)
// {
//   typedef CGAL::Partition_traits_2<Kernel> PT;
//   typedef std::list<PT::Polygon_2> Partition_polygons;
//   Partition_polygons partition_polygons;
//   CGAL::approx_convex_partition_2(P.vertices_begin(), P.vertices_end(),
// std::back_inserter(partition_polygons));

//   list<Polygon_2> polygons;
//   list<Segment_2> segments;
//   for (Partition_polygons::iterator it = partition_polygons.begin(); it !=
//   partition_polygons.end(); ++it)
//   {
//     Polygon_2 p(it->vertices_begin(), it->vertices_end());
//     polygons.push_back(p);
//     TILER_TRACE(logger, "Convex polygon: " << pp(p));

//     segments.insert(segments.end(), p.edges_begin(), p.edges_end());
//   }
// }

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *** Implementations ***
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

// template
// void convex_partition(const Polygon& P, back_insert_iterator<vector<Polygon_2> > out);

// template
// void convex_partition(const Polygon& P, back_insert_iterator<list<Polygon_2> > out);

// template
// void get_boundary_intersections(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator
// end, back_insert_iterator<vector<Point_2> > intersecting_points, bool report_endpoints);

// template
// void get_boundary_intersections(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator
// end, back_insert_iterator<list<Point_2> > intersecting_points, bool report_endpoints);

// template
// void get_boundary_intersections(list<Polygon_2>::iterator begin, list<Polygon_2>::iterator end,
// back_insert_iterator<list<Point_2> > intersecting_points, bool
// report_endpoints);

// template
// void get_boundary_intersections(vector<Polygon_2>::const_iterator begin,
// vector<Polygon_2>::const_iterator end, back_insert_iterator<vector<Point_2> >
// intersecting_points, bool report_endpoints);

// template
// void get_boundary_intersections(list<Polygon_2>::const_iterator begin,
// list<Polygon_2>::const_iterator end, back_insert_iterator<list<Point_2> > intersecting_points,
// bool report_endpoints);

template void triangulate_safe(
    const Polygon_2 &P, back_insert_iterator<list<Triangle>> triangles,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges);

template void triangulate_safe(
    const Polygon_2 &P, back_insert_iterator<vector<Triangle>> triangles,
    const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected>> &point2edges);

//------------------------------------------------------------
// split_nonsimple
//------------------------------------------------------------

bool operator<(const Segment_2 &a, const Segment_2 &b) {
  if (a.source() == b.source()) {
    return a.target() < b.target();
  }
  return a.source() < b.source();
}

Point_2 find_start(const std::map<Point_2, set<Segment_2>> &source2sub) {
  typedef pair<Point_2, set<Segment_2>> Entry;
  for (const auto &entry : source2sub) {
    if (entry.second.size() > 1)
      return entry.first;
  }
  return source2sub.begin()->first;
}

Segment_2 pop_outgoing(const Point_2 &p, std::map<Point_2, set<Segment_2>> &source2sub) {
  const Segment_2 s = *source2sub[p].begin();
  source2sub[p].erase(s);
  if (source2sub[p].empty()) {
    source2sub.erase(p);
  }
  return s;
}

//------------------------------------------------------------
// compute_subcurves
//
// In place of CGAL::compute_subcurves: the segments cut at every point where they meet another,
// so that no two pieces meet except at their ends. Each piece keeps its segment's direction; a
// piece that two collinear segments share is kept once.
//------------------------------------------------------------
template <typename Seg_iter, typename Out_iter>
void compute_subcurves(Seg_iter begin, Seg_iter end, Out_iter out) {
  const std::vector<Segment_2> segs(begin, end);
  std::set<Segment_2> seen;
  for (std::size_t i = 0; i < segs.size(); ++i) {
    const Segment_2 &s = segs[i];
    if (s.is_degenerate())
      continue;
    // Where the others meet s, ordered from its source.
    std::vector<Point_2> cuts;
    for (std::size_t j = 0; j < segs.size(); ++j) {
      if (j == i)
        continue;
      Object o = intersection(s, segs[j]);
      Point_2 p;
      Segment_2 overlap;
      if (assign(p, o)) {
        cuts.push_back(p);
      } else if (assign(overlap, o)) {
        cuts.push_back(overlap.source());
        cuts.push_back(overlap.target());
      }
    }
    const Vector_2 d = s.to_vector();
    std::sort(cuts.begin(), cuts.end(), [&](const Point_2 &a, const Point_2 &b) {
      return (a - s.source()) * d < (b - s.source()) * d;
    });
    Point_2 from = s.source();
    cuts.push_back(s.target());
    for (const Point_2 &c : cuts) {
      if (xy_equal(c, from) || !s.has_on(c))
        continue;
      const Segment_2 piece(from, c);
      if (seen.find(piece) == seen.end() && seen.find(piece.opposite()) == seen.end()) {
        seen.insert(piece);
        *out++ = piece;
      }
      from = c;
    }
  }
}

// Takes a non-simple polygon and splits it into multiple simple polygons
template <typename Out_iter> void split_nonsimple(const Polygon_2 &P, Out_iter out) {

  std::set<Segment_2> segments;
  segments.insert(P.edges_begin(), P.edges_end());

  // Compute the subsegments, such that there are no interior pairwise
  // intersections between subsegments.
  std::list<Segment_2> subsegments_list;
  compute_subcurves(segments.begin(), segments.end(), back_inserter(subsegments_list));

  // Index the subsegments by their source and target points.
  std::map<Point_2, set<Segment_2>> source2sub, target2sub;
  std::set<Segment_2> subsegments;
  for (const auto &s : subsegments_list) {
    // if (segments.find(s.opposite()) != segments.end()) {
    //   s = s.opposite();
    // }
    subsegments.insert(s);

    source2sub[s.source()].insert(s);
    target2sub[s.target()].insert(s);
  }

  while (!source2sub.empty()) {
    Polygon_2 Q;
    Point_2 p = find_start(source2sub);
    Segment_2 e = pop_outgoing(p, source2sub);
    Q.push_back(e.target());
    const Point_2 beg = e.source();
    while (e.target() != beg) {
      p = e.target();
      e = pop_outgoing(p, source2sub);
      Q.push_back(e.target());
    }
    if (!Q.is_counterclockwise_oriented()) {
      Q.reverse_orientation();
    }
    *out++ = Q;
    TILER_TRACE(logger, "Q = " << pp(Q));
  }
}

template void split_nonsimple(const Polygon_2 &P, back_insert_iterator<vector<Polygon_2>> out);
template void split_nonsimple(const Polygon_2 &P, back_insert_iterator<list<Polygon_2>> out);

//    x ___...__ b
//      \      /
//       \    /
//        \  /
//  z _____\/ y
//    \    /
//    ... /
//      \/
//      a
//
// Finds which side of a--b that y should be moved to.
Oriented_side find_side(const Polygon_2 &P, const Number_type delta, const int yi, const int ai,
                        const int bi) {
  const int n = P.size();
  const Point_2 &y = P[yi];
  const Line_2 l(P[ai], P[bi]);
  const Segment_2 s(P[ai], P[bi]);
  const Number_type delta2 = delta * delta;

  int zi = -1;
  for (int i = (yi + 1) % n; zi == -1 && i != ai; i = (i + 1) % n) {
    // if (squared_distance(P[i], l) > delta2 && squared_distance(P[i], s) > delta2) {
    if (squared_distance(P[i], s) > delta2) {
      zi = i;
    }
  }
  for (int i = (yi + n - 1) % n; zi == -1 && i != bi; i = (i + n - 1) % n) {
    // if (squared_distance(P[i], l) > delta2 && squared_distance(P[i], s) > delta2) {
    if (squared_distance(P[i], s) > delta2) {
      zi = i;
    }
  }
  if (zi == -1) {
    // choose arbitrarily
    return ON_NEGATIVE_SIDE;
  }
  return l.oriented_side(P[zi]);
}

// Polygon_2 separate_close_vertices(const Polygon_2& P, const Number_type delta, map<Point_2,
// Point_2>& old2new)
// {
// }

//    x ___...__ b
//      \      /
//       \    /
//        \  /
//  z _____\/ y
//    \    /
//    ... /
//      \/
//      a
//
// Takes a polygon in the form above where point y is within
// delta of ab and moves y in the normal direction of ab.
//
//    x ___...__ b
//      \      /
//       \    /
//        \  /
//  z __---*/  *=y
//    \    /
//    ... /
//      \/
//      a
//

Polygon_2 adjust_nonsimple_polygon(const Polygon_2 &P, const Number_type delta,
                                   map<Point_2, Point_2> &old2new) {

  const Number_type delta2 = delta * delta;
  const int n = P.size();
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (abs(i - j) > 0 && abs(i - ((j + 1) % n)) > 0) {
        const Point_2 &y = P[i];
        const Point_2 &a = P[j];
        const Point_2 &b = P[(j + 1) % n];
        const Line_2 l(a, b);
        const Segment_2 s(a, b);
        if (squared_distance(y, s) < delta2) {
          TILER_TRACE(logger, "close: " << pp(y));

          // Get the side of l that y should be on
          const Oriented_side side = find_side(P, delta, i, j, (j + 1) % n);
          if (side == ON_ORIENTED_BOUNDARY) {
            throw logic_error("Didn't expect z to be on boundary.  P = " + pp(P));
          }

          const Vector_2 v = l.to_vector();
          Vector_2 n = v.perpendicular(COUNTERCLOCKWISE);
          if (side == ON_NEGATIVE_SIDE) {
            n = v.perpendicular(CLOCKWISE);
          }
          // normalize
          n = n / sqrt(n.squared_length());

          // Find intersection of line in direction of n passing
          // through y with l.
          const Object result = intersection(Line_2(y, n), l);
          Point_2 y_int;
          if (const Point_2 *ipoint = object_cast<Point_2>(&result)) {
            y_int = *ipoint;
          } else {
            throw logic_error("Unexpected non-intersection between n and l");
          }

          // Now move y in the direction of the normal by delta
          Point_2 y_new = y_int + delta * n;
          y_new.z() = y.z();
          old2new[y] = y_new;
          TILER_TRACE(logger, "Old = " << pp(y));
          TILER_TRACE(logger, "a-b = " << pp(s));
          TILER_TRACE(logger, "New = " << pp(y_new));
        }
      }
    }
  }

  if (!old2new.empty()) {
    typedef pair<Point_2, Point_2> Pair;
    for (const auto &oldnew : old2new) {
      TILER_TRACE(logger, "old->new: " << pp(oldnew.first) << " " << pp(oldnew.second));
    }

    Polygon_2 new_P;
    for (auto p_it = P.vertices_begin(); p_it != P.vertices_end(); ++p_it) {
      const Point_2 &p = *p_it;
      if (old2new.find(p) != old2new.end()) {
        new_P.push_back(old2new[p]);
      } else {
        new_P.push_back(p);
      }
    }
    return new_P;
  }
  return P;
}

// Removes duplicate vertices with equality based on their
// (x,y) value
Polygon_2 remove_duplicate_vertices(const Polygon_2 &P) {
  Polygon_2 Q;
  const int n = P.size();
  for (int i = 0; i < n; ++i) {
    if (!xy_equal(P[i], P[(i + 1) % n])) {
      Q.push_back(P[i]);
    }
  }
  return Q;
}

} // namespace cvc::tiler
