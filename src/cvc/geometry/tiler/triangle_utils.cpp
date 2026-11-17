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

#include "triangle_utils.h"

#include "print_utils.h"

namespace cvc::tiler {

void Triangle::check_valid() const {

  // if (_points[0] == _points[1] || _points[1] == _points[2] || _points[0] == _points[2]) {
  //   // throw logic_error("Triangle is degenerate: " + pp_tri(*this));
  //   TILER_WARN(logger, "Triangle is degenerate: " << pp_tri(*this));
  // }
  // for (int i = 0; i < 3; ++i) {
  //   if (sqrt(squared_distance(_points[i], _points[(i+1)%3])) < 0.0000000000001) {
  //     TILER_WARN(logger, "Triangle is nearly degenerate: " << pp_tri(*this));
  //     break;
  //   }
  // }
  for (int i = 0; i < 3; ++i) {
    if (_points[i].id() == 1826) {
      TILER_TRACE(logger, "Triangle with 1826: " << pp_tri(*this));
      break;
    }
  }
  // for (int i = 0; i < 3; ++i) {
  //   if (_points[i].id() == 0) {
  //     int id = _points[i].id();
  //   }
  // }
}

bool Triangle::is_degenerate() const {
  for (int i = 0; i < 3; ++i) {
    if (xyz_equal(_points[i], _points[(i + 1) % 3])) {
      return true;
    }
  }
  return false;
}

std::string tri_pp(const Point_2 &point) { return pp(point); }

std::string tri_pp(const Point_3 &point) { return pp(point); }

std::string tri_pp(const Segment_2 &segment) { return pp(segment); }

std::string tri_pp(const Segment_3 &segment) { return pp(segment); }

} // namespace cvc::tiler
