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

#ifndef __SWEEP_LINE_H__
#define __SWEEP_LINE_H__

#include <boost/unordered_set.hpp>

#include "common.h"
#include "print_utils.h"
#include "Distance_functor.h"

namespace cvc::tiler {

inline bool point_intersection_2(const Segment_2& a, const Segment_2& b, Point_2& ia)
{
  Number_type numa = 
    (b.target().x() - b.source().x()) * (a.source().y() - b.source().y()) -
    (b.target().y() - b.source().y()) * (a.source().x() - b.source().x());
  Number_type numb = 
    (a.target().x() - a.source().x()) * (a.source().y() - b.source().y()) -
    (a.target().y() - a.source().y()) * (a.source().x() - b.source().x());
  Number_type den = 
    (b.target().y() - b.source().y()) * (a.target().x() - a.source().x()) -
    (b.target().x() - b.source().x()) * (a.target().y() - a.source().y());

  if (numa == 0 && numb == 0 && den == 0)
    // Intersection is a segment
    return false;

  if (den == 0)
    return false;
  Number_type ua = numa / den;
  Number_type ub = numb / den;
  bool intersects = ua > 0 && ua < 1;

  if (intersects)
  {
    ia = Point_2(a.source().x() + ua * (a.target().x() - a.source().x()),
		 a.source().y() + ua * (a.target().y() - a.source().y()));
  }

  return intersects;
}

//------------------------------------------------------------------------------
// SL_point
//
/// Internal utility class
//------------------------------------------------------------------------------
class SL_point
{
public:
  SL_point() : _id(0) {}
  SL_point(const Point_3& point, size_t index, Segment_2_undirected& segment) 
    : _point(point), _index(index), _id(0)
  {
    _lesser = xy_equal(point, segment.lesser());
  }
  SL_point(const Point_3& point, size_t index, Segment_2_undirected& segment, size_t component_id) 
    : _point(point), _index(index), _id(component_id)
  {
    _lesser = xy_equal(point, segment.lesser());
  }
  ~SL_point() {}

  const Point_3& point() const
  { return _point; }

  Point_3& point()
  { return _point; }

  size_t index() const
  { return _index; }

  size_t component_id() const
  { return _id; }

  bool operator<(const SL_point& p) const
  { 
    Comparison_result r = compare_xy(_point, p.point());
    if (r != EQUAL)
      return r == SMALLER; 
    return _lesser && !p._lesser;
  }

  friend bool operator==(const SL_point& a, const SL_point& b)
  { 
    return a._point == b._point && a._lesser == b._lesser;
  }

private:
  Point_3 _point;
  size_t _index;
  bool _lesser;
  size_t _id;
};

template <typename Segment, typename Point>
bool has_on(const Segment& s, const Point& p)
{
// #ifdef CONTOUR_EXACT_ARITHMETIC
  return s.has_on(p);
// #else
//   return squared_distance(s, p) == 0;//< std::numeric_limits<double>::min();
// #endif
}

inline bool in(const Point_2& p, const boost::unordered_set<Point_2> set) {
  return set.find(p) != set.end();
}

//------------------------------------------------------------------------------
// do_intersection
//
/// Used by sweep_line().  Returns false if sweep should stop.
//------------------------------------------------------------------------------
template <typename Visitor>
bool do_intersection(const Segment_2& segment, size_t id, 
		     const Segment_2& test_segment, size_t test_id, 
		     Visitor& v)
{

  if (!do_intersect(segment, test_segment))
    return true;

//   bool intersects = false;
  boost::unordered_set<Point_2> intersects;

  // Check endpoints
  if (xy_equal(segment.source(), test_segment.source())) {
    intersects.insert(segment.source());
    intersects.insert(test_segment.source());
    if (!v.end_end(segment.source(), segment, id, test_segment, test_id))
      return false;
  }
  else if (xy_equal(segment.source(), test_segment.target())) {
    intersects.insert(segment.source());
    intersects.insert(test_segment.target());
    if (!v.end_end(segment.source(), segment, id, test_segment, test_id))
      return false;
  }
  if (xy_equal(segment.target(), test_segment.source())) {
    intersects.insert(segment.target());
    intersects.insert(test_segment.source());
    if (!v.end_end(segment.target(), segment, id, test_segment, test_id))
      return false;
  }
  else if (xy_equal(segment.target(), test_segment.target()))
  {
    intersects.insert(segment.target());
    intersects.insert(test_segment.target());
    if (!v.end_end(segment.target(), segment, id, test_segment, test_id))
      return false;
  }

  // Check endpoint-interior
  if (!in(segment.source(), intersects) && has_on(test_segment, segment.source()))
  {
    intersects.insert(segment.source());
    if (!v.end_int(segment.source(), segment, id, test_segment, test_id))
      return false;
  }
  if (!in(segment.target(), intersects) && has_on(test_segment, segment.target()))
  {
    intersects.insert(segment.target());
    if (!v.end_int(segment.target(), segment, id, test_segment, test_id))
      return false;
  }
  if (!in(test_segment.source(), intersects) && has_on(segment, test_segment.source()))
  {
    intersects.insert(test_segment.source());
    if (!v.end_int(test_segment.source(), test_segment, test_id, segment, id))
      return false;
  }
  if (!in(test_segment.target(), intersects) && has_on(segment, test_segment.target()))
  {
    intersects.insert(test_segment.target());
    if (!v.end_int(test_segment.target(), test_segment, test_id, segment, id))
      return false;
  }

  // If no endpoints intersect, check interiors
  if (intersects.size() == 0)
  {
    Point_2 ipoint;
    Segment_2 iseg;
    Object result = intersection(segment, test_segment);
    // First try our more robust test
    if (point_intersection_2(segment, test_segment, ipoint)) {
      if (ipoint.y() == 0) {
	TILER_DEBUG(logger, "Bad point: " << pp(ipoint) << " - " << pp(segment) << " - " << pp(test_segment));
      }
      if (!v.int_int(ipoint, segment, id, test_segment, test_id))
	return false;
    }
    // ...now CGAL
    else if (assign(ipoint, result)) {
      if (ipoint.y() == 0) {
	TILER_DEBUG(logger, "Bad point: " << pp(ipoint) << " - " << pp(segment) << " - " << pp(test_segment));
      }
      if (!v.int_int(ipoint, segment, id, test_segment, test_id))
	return false;
    }
    else if (assign(iseg, result)) 
      throw logic_error("Any segment intersects should have been found previously");
    else
      throw logic_error("No intersection was found");
  }
  return true;
}

//------------------------------------------------------------------------------
// sweep_line_single_comp
//------------------------------------------------------------------------------
template <typename SL_point_iter, typename Visitor>
Visitor sweep_line_single_comp(SL_point_iter points_begin, SL_point_iter points_end, 
			vector<Segment_2>& segments, Visitor v)
{

  typedef boost::unordered_set<size_t> In_set;
  typedef vector<In_set> In_set_arr;
  In_set in;
  
//   for (Points::iterator it = points.begin(); it != points.end(); ++it)
  for (SL_point_iter it = points_begin; it != points_end; ++it)
  {
    const SL_point& point = *it;
    size_t segment_idx = point.index();

    // If this is the first encounter with this segment,
    // iterate through all other segments that have one
    // visited endpoint and test for intersection
    if (in.find(segment_idx) == in.end())
    {
      const Segment_2& segment = segments[segment_idx];

      for (In_set::const_iterator in_it = in.begin(); in_it != in.end(); ++in_it)
      {
	const Segment_2& test_segment = segments[*in_it];
	if (!do_intersection(segment, point.component_id(), test_segment, point.component_id(), v))
	  return v;
      }

      TILER_TRACE(logger, "Adding " << segment_idx);
      in.insert(segment_idx);
    }
    else
    {
      TILER_TRACE(logger, "Removing " << segment_idx);
      in.erase(segment_idx);
    }
  }

  return v;
}

//------------------------------------------------------------------------------
// sweep_line_multi_comp
//------------------------------------------------------------------------------
template <typename SL_point_iter, typename Visitor>
Visitor sweep_line_multi_comp(SL_point_iter points_begin, SL_point_iter points_end, 
			vector<Segment_2>& segments, size_t num_components,
			Visitor v)
{

  typedef boost::unordered_set<size_t> In_set;
  typedef vector<In_set> In_set_arr;
  In_set_arr in_arr(num_components);
  
//   for (Points::iterator it = points.begin(); it != points.end(); ++it)
  for (SL_point_iter it = points_begin; it != points_end; ++it)
  {
    const SL_point& point = *it;
    size_t segment_idx = point.index();
    size_t id = point.component_id();
    In_set& in = in_arr[id];

    // If this is the first encounter with this segment,
    // iterate through all other segments that have one
    // visited endpoint and test for intersection
    if (in.find(segment_idx) == in.end())
    {
      const Segment_2& segment = segments[segment_idx];

      for (int i = 0; i < num_components; ++i)
      {
	if (i != id)
	{
	  In_set& other_in = in_arr[i];
	  for (In_set::const_iterator in_it = other_in.begin(); in_it != other_in.end(); ++in_it)
	  {
	    const Segment_2& test_segment = segments[*in_it];
	    if (!do_intersection(segment, id, test_segment, i, v))
	      return v;
	  }
	}
      }

      TILER_TRACE(logger, "Adding " << segment_idx);
      in.insert(segment_idx);
    }
    else
    {
      TILER_TRACE(logger, "Removing " << segment_idx);
      in.erase(segment_idx);
    }
  }

  return v;
}

//------------------------------------------------------------------------------
// sweep_line
//------------------------------------------------------------------------------
template <typename Segment_iter, typename Visitor>
Visitor sweep_line(Segment_iter segments_begin, Segment_iter segments_end, Visitor v)
{

  TILER_TRACE(logger, "Single-component sweep line");

  typedef vector<Segment_2> Segments;
  typedef vector<Line_2> Lines;
  typedef vector<SL_point> Points;

  Segments segments;
  for (Segment_iter it = segments_begin; it != segments_end; ++it)
    segments.push_back(Segment_2(it->source(), it->target()));

  Points points;
  points.reserve(segments.size() * 2);
  for (int i = 0; i < segments.size(); ++i)
  {
    Segment_2_undirected su(segments[i]);
    points.push_back(SL_point(segments[i].source(), i, su));
    points.push_back(SL_point(segments[i].target(), i, su));
  }
  sort(points.begin(), points.end());

  return sweep_line_single_comp(points.begin(), points.end(), segments, v);
}

//------------------------------------------------------------------------------
// sweep_line
//------------------------------------------------------------------------------
template <typename Segment_iter, typename Visitor>
Visitor sweep_line(Segment_iter segments0_begin, Segment_iter segments0_end, 
		   Segment_iter segments1_begin, Segment_iter segments1_end, 
		   Visitor v)
{

  TILER_TRACE(logger, "Multi-component sweep line");

  typedef vector<Segment_2> Segments;
  typedef vector<Line_2> Lines;
  typedef vector<SL_point> Points;

  Segments segments;
  for (Segment_iter it = segments0_begin; it != segments0_end; ++it)
    segments.push_back(Segment_2(it->source(), it->target()));

  Points points;
  points.reserve(segments.size() * 2);
  int i = 0;
  for (; i < segments.size(); ++i)
  {
    Segment_2_undirected su(segments[i]);
    SL_point s(segments[i].source(), i, su, 0);
    SL_point t(segments[i].target(), i, su, 0);
//     points.push_back((s<t)?s:t);
    points.push_back(s);
    points.push_back(t);
  }

  for (Segment_iter it = segments1_begin; it != segments1_end; ++it)
    segments.push_back(Segment_2(it->source(), it->target()));

  points.reserve(segments.size() * 2);
  for (; i < segments.size(); ++i)
  {
    Segment_2_undirected su(segments[i]);
    SL_point s(segments[i].source(), i, su, 1);
    SL_point t(segments[i].target(), i, su, 1);
//     points.push_back((s<t)?s:t);
    points.push_back(s);
    points.push_back(t);
//     points.push_back(SL_point(segments[i].source(), i, su, 1));
// //     points.push_back(SL_point(segments[i].target(), i, su, 1));
  }

  sort(points.begin(), points.end());

  return sweep_line_multi_comp(points.begin(), points.end(), segments, 2, v);
}

//------------------------------------------------------------------------------
// sweep_line
//------------------------------------------------------------------------------
template <typename Visitor>
Visitor sweep_line_multi(const std::vector<Polygon_2>& polygons, 
		   Visitor v)
{

  typedef vector<Segment_2> Segments;
  typedef vector<Line_2> Lines;
  typedef vector<SL_point> Points;

  Segments segments;
  Points points;

  int id = 0;
  for (std::vector<Polygon_2>::const_iterator c_it = polygons.begin(); c_it != polygons.end(); ++c_it)
  {
    int i = segments.size();

    for (Polygon_2::Edge_const_iterator it = c_it->edges_begin(); it != c_it->edges_end(); ++it)
      segments.push_back(Segment_2(it->source(), it->target()));

    points.reserve(segments.size() * 2);
    for (; i < segments.size(); ++i)
    {
      Segment_2_undirected su(segments[i]);
      points.push_back(SL_point(segments[i].source(), i, su, id));
      points.push_back(SL_point(segments[i].target(), i, su, id));
    }
    ++id;
  }

  sort(points.begin(), points.end());

  return sweep_line_multi_comp(points.begin(), points.end(), segments, id, v);
}

} // namespace cvc::tiler

#endif
