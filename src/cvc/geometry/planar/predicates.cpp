/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's exact predicates (inc/cvc/geometry/planar.h): Shewchuk's adaptive orient2d and
// incircle (cvcpkg's shewchuk-predicates, public domain). Grid coordinates are within +-2^31, so
// their doubles are exact and so are the signs.

#include <algorithm>
#include <cvc/geometry/planar.h>
#include <shewchuk/predicates.hpp>

namespace cvc {
namespace planar {

namespace {

shewchuk::point2 sp(const point &p) { return {double(p.x), double(p.y)}; }
shewchuk::point2 sp(const pointd &p) { return {p.x, p.y}; }

} // namespace

int orientation(const point &a, const point &b, const point &c) {
  return shewchuk::sign(shewchuk::orient2d(sp(a), sp(b), sp(c)));
}

int orientation(const pointd &a, const pointd &b, const pointd &c) {
  return shewchuk::sign(shewchuk::orient2d(sp(a), sp(b), sp(c)));
}

int in_circle(const point &a, const point &b, const point &c, const point &d) {
  return shewchuk::sign(shewchuk::incircle(sp(a), sp(b), sp(c), sp(d)));
}

int in_circle(const pointd &a, const pointd &b, const pointd &c, const pointd &d) {
  return shewchuk::sign(shewchuk::incircle(sp(a), sp(b), sp(c), sp(d)));
}

bool on_segment(const point &p, const point &a, const point &b) {
  return orientation(a, b, p) == 0 && std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}

bool segments_intersect(const point &a, const point &b, const point &c, const point &d) {
  const int o1 = orientation(a, b, c), o2 = orientation(a, b, d);
  const int o3 = orientation(c, d, a), o4 = orientation(c, d, b);
  if (o1 != o2 && o3 != o4)
    return true;
  // Collinear or touching: an endpoint on the other segment.
  return (o1 == 0 && on_segment(c, a, b)) || (o2 == 0 && on_segment(d, a, b)) ||
         (o3 == 0 && on_segment(a, c, d)) || (o4 == 0 && on_segment(b, c, d));
}

} // namespace planar
} // namespace cvc
