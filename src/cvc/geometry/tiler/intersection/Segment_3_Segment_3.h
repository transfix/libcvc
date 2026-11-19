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

#ifndef CEP_INTERSECTION_SEGMENT_3_SEGMENT_3_H
#define CEP_INTERSECTION_SEGMENT_3_SEGMENT_3_H

#include "../common.h"
#include "Line_3_Line_3.h"
#include "Segment_3_Point_3.h"

namespace cvc::tiler {
namespace CEP {
namespace intersection {

inline bool collinear_do_intersect(const Segment_3 &s1, const Segment_3 &s2) {

  return collinear_are_ordered_along_line(s1.source(), s2.source(), s1.target()) ||
         collinear_are_ordered_along_line(s1.source(), s2.target(), s1.target()) ||
         collinear_are_ordered_along_line(s2.source(), s1.source(), s2.target());
}

inline bool coplanar_do_intersect(const Segment_3 &s1, const Segment_3 &s2) {
  if (s1.is_degenerate())
    return s2.has_on(s1.source());
  if (s2.is_degenerate())
    return s1.has_on(s2.source());

  if (s1.has_on(s2.source()) || s1.has_on(s2.target()) || s2.has_on(s1.source()) ||
      s2.has_on(s1.target()))
    return true;

  if (collinear(s1.source(), s1.target(), s2.source()) ||
      collinear(s2.source(), s2.target(), s1.source()))
    return false;

  return coplanar_orientation(s1.source(), s1.target(), s2.source(), s2.target()) == NEGATIVE &&
         coplanar_orientation(s2.source(), s2.target(), s1.source(), s1.target()) == NEGATIVE;
}

inline bool do_intersect(const Segment_3 &s1, const Segment_3 &s2) {
  if (s1.is_degenerate())
    return s2.has_on(s1.source());
  if (s2.is_degenerate())
    return s1.has_on(s2.source());

  if (s1.has_on(s2.source()) || s1.has_on(s2.target()) || s2.has_on(s1.source()) ||
      s2.has_on(s1.target()))
    return true;

  if (!coplanar(s1.source(), s1.target(), s2.source(), s2.target()))
    return false;

  if (collinear(s1.source(), s1.target(), s2.source()) ||
      collinear(s2.source(), s2.target(), s1.source()))
    return false;

  return coplanar_orientation(s1.source(), s1.target(), s2.source(), s2.target()) == NEGATIVE &&
         coplanar_orientation(s2.source(), s2.target(), s1.source(), s1.target()) == NEGATIVE;
}

namespace Segment_3_Segment_3 {

inline int classify_point(const Segment_3 &S, const Point_3 &p) {

  const Point_3 &s = S.source();
  const Point_3 &t = S.target();

  if (p == s)
    return 2;
  if (p == t)
    return 4;

  if (collinear_are_ordered_along_line(p, s, t))
    return 1;
  if (collinear_are_ordered_along_line(s, p, t))
    return 3;

  return 5;
}
} // namespace Segment_3_Segment_3

inline Object collinear_intersection(const Segment_3 &seg1, const Segment_3 &seg2) {
  const Point_3 &s1 = seg1.source();
  const Point_3 &t1 = seg1.target();
  const Point_3 &s2 = seg2.source();
  const Point_3 &t2 = seg2.target();

  int s1_class = Segment_3_Segment_3::classify_point(seg2, s1);
  int t1_class = Segment_3_Segment_3::classify_point(seg2, t1);

  switch (s1_class) {
  case 1:
    switch (t1_class) {
    case 1:
      return Object();
    case 2:
      return make_object(s2);
    case 3:
      return make_object(Segment_3(s2, t1));
    case 4: // fall through
    case 5:
      return make_object(seg2);
    }
  case 2:
    switch (t1_class) {
    case 1: // fall through
    case 2:
      return make_object(s2);
    case 3: // fall through
    case 4:
      return make_object(seg1);
    case 5:
      return make_object(seg2);
    }
  case 3:
    switch (t1_class) {
    case 1: // fall through
    case 2:
      return make_object(Segment_3(s2, s1));
    case 3: // fall through
    case 4:
      return make_object(seg1);
    case 5:
      return make_object(Segment_3(s1, t2));
    }
  case 4:
    switch (t1_class) {
    case 1: // fall through
    case 2:
      return make_object(seg2);
    case 3:
      return make_object(seg1);
    case 4: // fall through
    case 5:
      return make_object(t2);
    }
  case 5:
    switch (t1_class) {
    case 1: // fall through
    case 2:
      return make_object(seg2);
    case 3:
      return make_object(Segment_3(t1, t2));
    case 4:
      return make_object(t2);
    case 5:
      return Object();
    }
  }

  return Object();
}

inline Object intersection(const Segment_3 &s1, const Segment_3 &s2) {
  if (s1.is_degenerate())
    return CEP::intersection::intersection(s2, s1.source());
  if (s2.is_degenerate())
    return CEP::intersection::intersection(s1, s2.source());

  if (collinear(s1, s2.source())) {
    if (collinear(s1, s2.target())) {
      return collinear_intersection(s1, s2);
    } else {
      return CEP::intersection::intersection(s1, s2.source());
    }
  } else if (collinear(s1, s2.target())) {
    return CEP::intersection::intersection(s1, s2.target());
  } else if (collinear(s2, s1.source())) {
    return CEP::intersection::intersection(s2, s1.source());
  } else if (collinear(s2, s1.target())) {
    return CEP::intersection::intersection(s2, s1.target());
  }

  if (!coplanar(s1.source(), s1.target(), s2.source(), s2.target()))
    return Object();

  if (coplanar_orientation(s1.source(), s1.target(), s2.source(), s2.target()) != NEGATIVE ||
      coplanar_orientation(s2.source(), s2.target(), s1.source(), s1.target()) != NEGATIVE)
    return Object();

  return CEP::intersection::Line_3_Line_3::coplanar_nonparallel_intersection(
      s1.source(), s1.target(), s2.source(), s2.target());
}

} // namespace intersection
} // namespace CEP
} // namespace cvc::tiler

#endif
