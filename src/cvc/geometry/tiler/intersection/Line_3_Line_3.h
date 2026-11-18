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

#ifndef CEP_INTERSECTION_LINE_3_LINE_3_H // -*- C++ -*-
#define CEP_INTERSECTION_LINE_3_LINE_3_H

#include "../common.h"

namespace cvc::tiler {
namespace CEP {
namespace intersection {

namespace Line_3_Line_3 {

inline Object coplanar_nonparallel_intersection(const Point_3 &p, const Point_3 &q,
                                                const Point_3 &r, const Point_3 &s) {

  typedef Number_type FT;

  Vector_3 d1 = q - p;
  Vector_3 d2 = s - r;
  Vector_3 b = r - p;

  FT num = (d2 * d2) * (d1 * b) - (d1 * d2) * (d2 * b);
  FT den = (d1 * d1) * (d2 * d2) - (d1 * d2) * (d1 * d2);

  FT zero(0);

  Point_3 i_point = p + d1 * num / den;
  return make_object(i_point);
}

inline Object coplanar_intersection(const Point_3 &p, const Point_3 &q, const Point_3 &r,
                                    const Point_3 &s) {

  Vector_3 d1 = q - p;
  Vector_3 d2 = s - r;

  if (cross_product(d1, d2) == Vector_3(0, 0, 0))
    return Object();

  return coplanar_nonparallel_intersection(p, q, r, s);
}

} // namespace Line_3_Line_3
} // namespace intersection
} // namespace CEP
} // namespace cvc::tiler

#endif
