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

#include "contour_utils.h"
#include "sweep_line_visitors.h"

namespace cvc::tiler {

size_t vertex_idx(Contour_handle contour, Polygon_2::Vertex_iterator vertex)
{
  return vertex - contour->polygon().vertices_begin();
}

size_t vertex_idx(Contour_handle contour, Polygon_2::Vertex_circulator vertex)
{
  return (vertex - contour->polygon().vertices_circulator()) % contour->polygon().size();
}

/// InputIterator iterates over Contour_handle objects.
/// OutputIterator iterates over std::pair<Contour_handle, Contour_handle> objects.
/// @param intersecting_pairs pairs of contours that intersect each other
template <typename InputIterator, typename OutputIterator>
void get_intersecting_pairs(InputIterator begin, InputIterator end, OutputIterator intersecting_pairs)
{

  std::list<Segment_2> segments;
  boost::unordered_map<Segment_2, Contour_handle> seg2Contour;
  vector<Polygon_2> polygons;
  for (InputIterator it = begin; it != end; ++it) {
    Contour_handle contour = *it;
    const Polygon_2& polygon = contour->polygon();
    segments.insert(segments.end(), polygon.edges_begin(), polygon.edges_end());
    for (Polygon_2::Edge_const_iterator eit = polygon.edges_begin(); eit != polygon.edges_end(); ++eit) {
      seg2Contour[*eit] = contour;
    }
    polygons.push_back(contour->polygon());
  }
  
  list<SL_intersection> ints;
  // get_intersections(segments.begin(), segments.end(), back_inserter(ints), true, false);

  // typedef Intersection_visitor<Output_iter> Visitor;
  // sweep_line(segments_begin, segments_end, filter_visitor(Visitor(intersections), end_internal, end_end));
  typedef Intersection_visitor<back_insert_iterator<list<SL_intersection> > > Visitor;
  sweep_line_multi(polygons, Visitor(back_inserter(ints)));

  typedef boost::unordered_set<pair<Contour_handle, Contour_handle> > Pair_set;
  Pair_set unique_pairs;
  for (list<SL_intersection>::iterator it = ints.begin(); it != ints.end(); ++it) {
    const list<Segment_2>& ints = it->interiors();
    const list<Segment_2>& ends = it->ends();
    list<Segment_2> segs(ints.begin(), ints.end());
    segs.insert(segs.end(), ends.begin(), ends.end());
    if (segs.size() != 2) {
      throw logic_error("Unexpected number of segments in intersection");
    }
    Contour_handle p = seg2Contour[*segs.begin()];
    Contour_handle q = seg2Contour[*segs.rbegin()];
    if (p.get() > q.get()) {
      swap(p, q);
    }
    unique_pairs.insert(make_pair(p, q));
  }
  for (Pair_set::const_iterator it = unique_pairs.begin(); it != unique_pairs.end(); ++it) {
    *intersecting_pairs++ = *it;
  }
}

template
void get_intersecting_pairs(list<Contour_handle>::const_iterator begin, 
			    list<Contour_handle>::const_iterator end, 
			    back_insert_iterator<list<pair<Contour_handle, Contour_handle> > > intersecting_pairs);

template
void get_intersecting_pairs(list<Contour_handle>::iterator begin, 
			    list<Contour_handle>::iterator end, 
			    back_insert_iterator<list<pair<Contour_handle, Contour_handle> > > intersecting_pairs);

template
void get_intersecting_pairs(vector<Contour_handle>::const_iterator begin, 
			    vector<Contour_handle>::const_iterator end, 
			    back_insert_iterator<list<pair<Contour_handle, Contour_handle> > > intersecting_pairs);

template
void get_intersecting_pairs(vector<Contour_handle>::iterator begin, 
			    vector<Contour_handle>::iterator end, 
			    back_insert_iterator<list<pair<Contour_handle, Contour_handle> > > intersecting_pairs);

} // namespace cvc::tiler
