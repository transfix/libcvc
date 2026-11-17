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
// Ported: only the pp() formatters stay, for exception messages. The gnuplot, raw and .ser
// writers were debug output (dropped, roadmap §3), and the print precision was a process-wide
// static that callers changed and restored; here it is a constant.

#ifndef CVC_TILER_PRINT_UTILS_H
#define CVC_TILER_PRINT_UTILS_H

#include <sstream>
#include <string>

#include "Contour.h"
#include "Polyline.h"
#include "Segment_3_undirected.h"
#include "Slice.h"
#include "Tiling_region.h"

namespace cvc::tiler {

std::string pp(const Point_2& point);
std::string pp(const Point_3& point);
std::string pp(const Segment_2& segment);
std::string pp(const Segment_3& segment);
inline std::string pp(const Segment_3_undirected& segment) { return pp(segment.segment()); }
std::string pp_id(const Segment_2& segment);
std::string pp_id(const Segment_3& segment);
inline std::string pp_id(const Segment_3_undirected& segment) { return pp_id(segment.segment()); }
std::string pp(const Point_3& v0, const Point_3& v1);
std::string pp(const Tiling_region& region);
std::string pp(const Polygon_2& polygon);
std::string pp(const Polygon_with_holes_2& pwh);
std::string pp(const Polyline_2& polyline);
std::string pp(const Polyline_3& polyline);
std::string pp(const Slice& slice);

template <typename Tri>
std::string pp_tri(const Tri& triangle)
{
  std::stringstream out;
  out << "[ ";
  for (int i = 0; i < 3; ++i)
    out << pp(triangle[i]);
  out << " ]";
  return out.str();
}

template <typename Curve>
std::string pp_curve(const Curve& curve)
{
  typename Curve::const_iterator vit;
  std::stringstream out;
  out << "[ ";
  for (vit = curve.begin(); vit != curve.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

} // namespace cvc::tiler

#endif
