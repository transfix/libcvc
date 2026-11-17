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

//
// Ported: CGAL's straight-skeleton offsets (create_interior_skeleton_and_offset_polygons_2,
// create_exterior_skeleton_and_offset_polygons_2 and arrange_offset_polygons_2, all GPL) became
// cvc::planar::wavefront_offset, which steps Clipper2 through the skeleton's events and returns
// the arranged polygons with holes; tools/cgal_ab finds it equal to CGAL's skeleton offsets in all
// 50 of its cases. fix_simple() quantised CGAL's inexact constructions to 10 decimal places; the
// planar module's output is exact on its grid, so only the duplicate removal stays.

#include <iterator>
#include <sstream>
#include <string>

#include "common.h"
#include "offset_polygon.h"
#include "planar_bridge.h"
#include "print_utils.h"

namespace cvc::tiler {

//------------------------------------------------------------
// fix_simple
//
// Removes vertices equal to the previous one or to the first.
//------------------------------------------------------------
Polygon_2 fix_simple(const Polygon_2& P) 
{
  Polygon_2 temp;
  for (Polygon_2::Vertex_const_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    Point_2 p = *it;
    if (temp.size() == 0 || (!xy_equal(temp[temp.size()-1], p) && !xy_equal(temp[0], p))) {
      temp.push_back(p);
    }
  }
  return temp;
}

Polygon_with_holes_2 fix_simple(const Polygon_with_holes_2& P) 
{
  Polygon_with_holes_2 ret(fix_simple(P.outer_boundary()));
  for (Polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
    ret.add_hole(fix_simple(*it));
  }
  return ret;
}

//------------------------------------------------------------
// offset_polygon
//
// Erosion (offset < 0) gives every polygon the inward offset splits into; dilation gives the one
// polygon (with any holes the dilation closes off) the outward offset makes.
//------------------------------------------------------------
template <typename Out_iter>
void offset_polygon(const Polygon_2& polygon, Number_type offset, Out_iter out) {
  if (polygon.size() < 3 || !polygon.is_simple()) {
    stringstream ss;
    ss << "offset_polygon: input polygon is not simple:"
       << "  polygon = " << pp(polygon) ;
    throw logic_error(ss.str());
  }

  TILER_TRACE(logger, "P = " << pp(polygon));

  Number_type z = polygon[0].z();
  // Room for the offset around the polygon.
  Bbox_2 box = polygon.bbox();
  const double grow = std::abs(offset);
  box = Bbox_2(box.xmin() - grow, box.ymin() - grow, box.xmax() + grow, box.ymax() + grow);
  const planar::grid g = fit_grid(box);
  const planar::polygon_set in{planar::polygon(to_planar(polygon, g))};

  planar::polygon_set offset_polygons = planar::wavefront_offset(in, g.to_grid(offset));
  if (offset >= 0 && offset_polygons.size() > 1) {
    // The dilation of one polygon is one polygon; keep the largest should rounding split it.
    std::size_t best = 0;
    for (std::size_t i = 1; i < offset_polygons.size(); ++i)
      if (planar::area(offset_polygons[i]) > planar::area(offset_polygons[best]))
        best = i;
    offset_polygons = planar::polygon_set{offset_polygons[best]};
  }

  for (const planar::polygon& op : offset_polygons) {
    Polygon_with_holes_2 ret = fix_simple(from_planar(op, g, z));
    if (ret.outer_boundary().size() > 0 && !ret.outer_boundary().is_simple()) {
      stringstream ss;
      ss << "Offset polygon is not simple.  offset = " << offset 
         << "   Original polygon = " << pp(polygon) << "  Offset polygon = " << pp(ret.outer_boundary())
         << "\n\n***** This is probably failing because some of the points are collinear.  Try removing "
         << "collinear points in the contours first.  This can be done using the -e option.";
      throw runtime_error(ss.str());
    }
    *out++ = ret;
  }
}

Polygon_with_holes_2 offset_polygon_positive(const Polygon_2& polygon, Number_type offset)
{
  vector<Polygon_with_holes_2> polys;
  offset_polygon(polygon, offset, back_inserter(polys));
  if (polys.size() != 1) throw logic_error("unexpected number of polygons");
  return polys[0];
}

template <typename Out_iter>
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, Out_iter out)
{
  offset_polygon(polygon, offset, out);
}

template
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, 
                             back_insert_iterator<list<Polygon_with_holes_2> > out);
template
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, 
                             back_insert_iterator<vector<Polygon_with_holes_2> > out);

} // namespace cvc::tiler
