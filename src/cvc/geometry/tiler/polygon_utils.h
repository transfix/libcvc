#ifndef __POLYGON_UTILS_H__
#define __POLYGON_UTILS_H__

#include <list>

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/Arr_polyline_traits_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Sweep_line_2_algorithms.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Right_enum.h>
#include <ContourTiler/Statistics.h>
#include <ContourTiler/Segment_3_undirected.h>

CONTOURTILER_BEGIN_NAMESPACE

struct Polygon_relation_def
{
  enum type { PARENT, CHILD, SIBLING, BOUNDARY_INTERSECT };
};

typedef Right_enum<Polygon_relation_def> Polygon_relation;

/// STRICT - throws on construction if orientations don't match hierarchy
/// FORCE - forces all orientations to match hierarchy
/// FORCE_CCW - forces all orientation to be CCW.  This is the most efficient policy
///   for later operations.
/// NATURAL - leaves orientations unchanged
struct Hierarchy_policy_def
{
  enum type { STRICT, FORCE, FORCE_CCW, NATURAL };
};

typedef Right_enum<Hierarchy_policy_def> Hierarchy_policy;

struct Vertex_sign_def
{
  enum type { POSITIVE, NEGATIVE, OVERLAPPING };
};

typedef Right_enum<Vertex_sign_def> Vertex_sign;

/// Returns true if there are any polygon boundary intersections.
/// Precondition: InputIterator iterates over CGAL::Polygon_2 objects.
// template <typename PolyIterator, typename OutputIterator>
// void get_boundary_intersections(PolyIterator begin, PolyIterator end, OutputIterator intersecting_points, bool report_endpoints = false);

boost::unordered_set<Point_2> get_boundary_intersections(const std::list<Segment_2>& segments, bool report_endpoints);

/// Returns true if there are any polygon boundary intersections.
/// Precondition: InputIterator iterates over CGAL::Polygon_2 objects.
template <typename PolyIterator, typename OutputIterator>
void get_boundary_intersections(PolyIterator begin, PolyIterator end, OutputIterator intersecting_points, bool report_endpoints)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("boundary_intersections");

  std::list<Segment_2> segments;
  for (PolyIterator it = begin; it != end; ++it)
    segments.insert(segments.end(), it->edges_begin(), it->edges_end());
  
  boost::unordered_set<Point_2> points = get_boundary_intersections(segments, report_endpoints);

  // list<SL_intersection> ints;
  // get_intersections(segments.begin(), segments.end(), back_inserter(ints), report_endpoints, report_endpoints);

  // boost::unordered_set<Point_2> points;
  // for (list<SL_intersection>::iterator it = ints.begin(); it != ints.end(); ++it)
  //   points.insert(it->point());
  copy(points.begin(), points.end(), intersecting_points);
}


/// Returns true if there are any polygon boundary intersections.
bool boundaries_intersect(const Polygon_2& P, const Polygon_2& Q, bool proper);

/// Returns true if the point intersects the polygon's boundary
bool intersects_boundary(const Point_2& point, const Polygon_2& P);

/// Returns true if the intersection of two polygons is non-empty
bool do_intersect(const Polygon_2& P, const Polygon_2& Q);

/// Returns true if the segment crosses a boundary line of the polygon.
/// It is not enough for the segment to simply intersect the boundary
/// line -- it must cross it such that the segment extends to both inside
/// and outside of the boundary of the polygon.
bool intersects_proper(const Segment_2& segment, const Polygon_2& polygon);

/// Returns:
///  PARENT if the relative is completely enclosed by the subject
///  CHILD if the subject is completely enclosed by the relative
///  SIBLING if the subject and relative do not intersect
///  BOUNDARY_INTERSECT if the subject and relative's boundaries intersect
/// If proper is false, then the boundaries are open when checking for
/// PARENT, CHILD and SIBLING, and the boundaries are closed when checking
/// for BOUNDARY_INTERSECT.
Polygon_relation relation(const Polygon_2& subject, const Polygon_2& relative, bool proper);

/// Returns a polygon that is a spatial proper superset of all input polygons.  This is the only
/// guarantee -- nothing is said about the shape or size of the returned
/// polygon other than this.
template <typename PolyIterator>
Polygon_2 super_polygon(PolyIterator start, PolyIterator end)
{
//   typedef typename std::iterator_traits<PolyIterator>::value_type Polygon_2;
//   typedef typename Polygon_2::Traits K;
//   typedef typename K::Point_2 Point_2;
//   typedef CGAL::Arr_polyline_traits_2<CGAL::Arr_segment_traits_2<K> > Polyline;
//   typedef typename Polyline::Curve_2 Curve;

//   if (start == end)
//     return Polygon_2();

//   // First, find the bounding box of all polygons and make a virtual contour
//   CGAL::Bbox_2 bb = start->bbox();
//   for (PolyIterator it = start; it != end; ++it)
//   {
//     bb = bb + it->bbox();
//   }
//   // make the bounding box just a little bigger
//   bb = bb + CGAL::Bbox_2(bb.xmin() - 0.1, bb.ymin() - 0.1, bb.xmax() + 0.1, bb.ymax() + 0.1);
//   Point_2 points[] = { Point_2(bb.xmin(), bb.ymin(), 0), 
// 		       Point_2(bb.xmax(), bb.ymin(), 0), 
// 		       Point_2(bb.xmax(), bb.ymax(), 0), 
// 		       Point_2(bb.xmin(), bb.ymax(), 0) };

  // Divide by 4 to make absolutely sure that the width, height and area of the polygon
  // can fit in the number type.
  Number_type min_val = -numeric_limits<Number_type>::max()/8;
  Number_type max_val = numeric_limits<Number_type>::max()/8;
  CGAL::Bbox_2 bb(min_val, min_val, max_val, max_val);
  Point_2 points[] = { Point_2(bb.xmin(), bb.ymin(), 0), 
		       Point_2(bb.xmax(), bb.ymin(), 0), 
		       Point_2(bb.xmax(), bb.ymax(), 0), 
		       Point_2(bb.xmin(), bb.ymax(), 0) };

  return Polygon_2(points, points + 4);
}

Polygon_2::Vertex_const_iterator prev(Polygon_2::Vertex_const_iterator iter, const Polygon_2& p);
Polygon_2::Vertex_const_iterator next(Polygon_2::Vertex_const_iterator iter, const Polygon_2& p);

// Point_3 rotate_x(const Point_3& p, Number_type q);
// Polygon_2 rotate_x(const Polygon_2& p, Number_type q);

template <typename Polygon, typename Triangle_iter>
void triangulate(const Polygon& polygon, Triangle_iter triangles,
		 const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> >& point2edges);

template <typename Triangle_iter>
void triangulate_safe(const Polygon_2& P, Triangle_iter triangles,
		      const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> >& point2edges);

template <typename Cut_iter, typename Out_iter>
void triangulate(const Polygon_2& polygon, 
		 Cut_iter cuts_begin, Cut_iter cuts_end,
		 const boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> >& point2edges,
		 Out_iter triangles);

/// If the triangle formed by three consecutive points has an area
/// less than or equal to epsilon then the points are considered
/// collinear and the center point is discarded.
Polygon_2 remove_collinear(const Polygon_2& p, Number_type epsilon);

/// Returns the topological distance between p and q, that is, the
/// number of segments separating p and q on the polygon P.
/// If one or both of the points are not on P, then -1 is returned.
int distance(const Point_2& p, const Point_2& q, const Polygon_2& P);

/// Partitions polygon P into convex parts and puts resultant polygons
/// into out.
// template <typename Out_iter>
// void convex_partition(const Polygon& P, Out_iter out);

// Takes a non-simple polygon and splits it into multiple simple polygons
template <typename Out_iter>
void split_nonsimple(const Polygon_2& P, Out_iter out);

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
Polygon_2 adjust_nonsimple_polygon(const Polygon_2& P, const Number_type delta, std::map<Point_2, Point_2>& old2new);

Polygon_2 remove_duplicate_vertices(const Polygon_2& P);

CONTOURTILER_END_NAMESPACE

#endif
