/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// What cvc::planar's sources share (inc/cvc/geometry/planar.h): range checks and messages.
// Never installed.

#ifndef CVC_GEOMETRY_PLANAR_INTERNAL_H
#define CVC_GEOMETRY_PLANAR_INTERNAL_H

#include <cvc/geometry/planar.h>
#include <string>

namespace cvc {
namespace planar {
namespace detail {

inline bool in_range(const point &p) {
  return p.x >= -coord_max && p.x <= coord_max && p.y >= -coord_max && p.y <= coord_max;
}

inline std::string str(const point &p) {
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")";
}

// Throws planar_error("<fn>: vertex (x, y) is outside the grid's range +-2147483647") for the
// first point of a ring (or polygon, or set) outside +-coord_max.
void check_range(const char *fn, const point &p);
void check_range(const char *fn, const ring &r);
void check_range(const char *fn, const polygon &p);
void check_range(const char *fn, const polygon_set &s);

// The same, for a result: "<fn>: the result leaves the grid's range ... (fit the grid with more
// room)".
void check_result_range(const char *fn, const polygon_set &s);

} // namespace detail
} // namespace planar
} // namespace cvc

#endif
