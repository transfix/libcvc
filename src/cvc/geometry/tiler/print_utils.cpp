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

#include <iomanip>
#include <sstream>

#include "print_utils.h"

namespace cvc::tiler {

namespace {
constexpr int pp_precision = 6;
}

std::string pp(const Point_2& point)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  if (point.id() != DEFAULT_ID())
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; " << point.id() << ")";
  else
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; -1)";

  return out.str();
}

std::string pp(const Point_3& point)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  if (point.id() != DEFAULT_ID())
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; " << point.id() << ")";
  else
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; -1)";
  return out.str();
}

std::string pp(const Segment_2& segment)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << "[" << pp(segment.source()) << ", " << pp(segment.target()) << "]";
  return out.str();
}

std::string pp(const Segment_3& segment)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << "[" << pp(segment.source()) << ", " << pp(segment.target()) << "]";
  return out.str();
}

std::string pp_id(const Segment_2& segment)
{
  std::stringstream out;
  size_t a = segment.source().id();
  size_t b = segment.target().id();
  if (a > b)
    swap(a, b);
  out << a << "--" << b;
  return out.str();
}

std::string pp_id(const Segment_3& segment)
{
  std::stringstream out;
  size_t a = segment.source().id();
  size_t b = segment.target().id();
  if (a > b)
    swap(a, b);
  out << a << "--" << b;
  return out.str();
}

std::string pp(const Point_3& v0, const Point_3& v1)
{
  Segment_3 segment(v0.point_3(), v1.point_3());
  return pp(segment);
}

std::string pp(const Tiling_region& region)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << region;
  return out.str();
}

// std::string pp(const Polygon& P)
// {
//   Polygon::Vertex_const_iterator vit;
//   std::stringstream out;
//  out << std::setprecision(pp_precision);

//   out << "[ " << P.size() << " vertices:";
//   for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
//     out << " (" << *vit << ')';
//   out << " ]";
//   return out.str();
// }

std::string pp(const Polygon_2& P)
{
  Polygon_2::Vertex_const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << P.size() << " vertices:";
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Polygon_with_holes_2& pwh)
{
  std::stringstream out;
  out << "[Outer boundary: " << pp(pwh.outer_boundary()) << "]  ";
  for (Polygon_with_holes_2::Hole_const_iterator it = pwh.holes_begin(); it != pwh.holes_end(); ++it) {
    out << "Hole: " << pp(*it) << "  ";
  }
  return out.str();
}

std::string pp(const Polyline_2& polyline)
{
  Polyline_2::const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << polyline.size() << " vertices:";
  for (vit = polyline.begin(); vit != polyline.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Polyline_3& polyline)
{
  Polyline_3::const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << polyline.size() << " vertices:";
  for (vit = polyline.begin(); vit != polyline.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Slice& slice)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);

  list<string> components;
  slice.components(back_inserter(components));
  for (list<string>::const_iterator it = components.begin();
       it != components.end();
       ++it)
  {
    out << *it << ": ";
    for (Slice::Contour_const_iterator cit = slice.begin(*it);
	 cit != slice.end(*it);
	 ++cit)
    {
      out << pp((*cit)->polygon());
    }
  }
  
  return out.str();
}

} // namespace cvc::tiler
