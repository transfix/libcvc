/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's snapping grid and range checks (inc/cvc/geometry/planar.h).

#include "planar_internal.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace cvc {
namespace planar {

namespace {

std::string num(double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.17g", v);
  return buf;
}

} // namespace

grid grid::fit(double xmin, double ymin, double xmax, double ymax) {
  if (!std::isfinite(xmin) || !std::isfinite(ymin) || !std::isfinite(xmax) || !std::isfinite(ymax))
    throw planar_error("grid::fit: the box has a non-finite coordinate");
  if (xmin > xmax || ymin > ymax)
    throw planar_error("grid::fit: the box is empty (min > max): [" + num(xmin) + ", " + num(xmax) +
                       "] x [" + num(ymin) + ", " + num(ymax) + "]");
  const double half = std::max(xmax - xmin, ymax - ymin) / 2;
  if (!std::isfinite(half))
    throw planar_error("grid::fit: the box is too large for doubles");
  grid g;
  if (half > 0) {
    // The smallest power of two `unit` with half / unit <= fit_extent.
    int e = 0;
    const double m = std::frexp(half / double(fit_extent), &e); // = m * 2^e, m in [0.5, 1)
    g.unit = std::ldexp(1.0, m == 0.5 ? e - 1 : e);
    if (!(g.unit > 0) || !std::isfinite(g.unit))
      throw planar_error("grid::fit: no grid unit fits a box of half-extent " + num(half));
  }
  // The origin at the centre, on a multiple of the unit: every grid point's world position is
  // then a multiple of the unit, exact in a double.
  g.origin_x = std::round(xmin / 2 / g.unit + xmax / 2 / g.unit) * g.unit;
  g.origin_y = std::round(ymin / 2 / g.unit + ymax / 2 / g.unit) * g.unit;
  return g;
}

point grid::snap(double x, double y) const {
  if (!std::isfinite(x) || !std::isfinite(y))
    throw planar_error("grid::snap: non-finite coordinate (" + num(x) + ", " + num(y) + ")");
  const double gx = std::round((x - origin_x) / unit), gy = std::round((y - origin_y) / unit);
  if (!(std::fabs(gx) <= double(coord_max)) || !(std::fabs(gy) <= double(coord_max)))
    throw planar_error("grid::snap: (" + num(x) + ", " + num(y) +
                       ") is outside the grid's range: grid point (" + num(gx) + ", " + num(gy) +
                       "), beyond +-" + std::to_string(coord_max) + "; fit the grid to the data");
  return point(coord(gx), coord(gy));
}

std::vector<point> grid::snap(const std::vector<pointd> &ps) const {
  std::vector<point> out;
  out.reserve(ps.size());
  for (const pointd &p : ps)
    out.push_back(snap(p));
  return out;
}

pointd grid::to_world(const point &p) const {
  return pointd(origin_x + double(p.x) * unit, origin_y + double(p.y) * unit);
}

pointd grid::to_world(const pointd &p) const {
  return pointd(origin_x + p.x * unit, origin_y + p.y * unit);
}

pointd grid::to_grid(const pointd &world) const {
  return pointd((world.x - origin_x) / unit, (world.y - origin_y) / unit);
}

namespace detail {

void check_range(const char *fn, const point &p) {
  if (!in_range(p))
    throw planar_error(std::string(fn) + ": vertex " + str(p) + " is outside the grid's range +-" +
                       std::to_string(coord_max));
}

void check_range(const char *fn, const ring &r) {
  for (const point &p : r)
    check_range(fn, p);
}

void check_range(const char *fn, const polygon &p) {
  check_range(fn, p.outer);
  for (const ring &h : p.holes)
    check_range(fn, h);
}

void check_range(const char *fn, const polygon_set &s) {
  for (const polygon &p : s)
    check_range(fn, p);
}

void check_result_range(const char *fn, const polygon_set &s) {
  auto check = [fn](const ring &r) {
    for (const point &p : r)
      if (!in_range(p))
        throw planar_error(std::string(fn) + ": the result reaches " + str(p) +
                           ", outside the grid's range +-" + std::to_string(coord_max) +
                           "; fit the grid with more room");
  };
  for (const polygon &p : s) {
    check(p.outer);
    for (const ring &h : p.holes)
      check(h);
  }
}

} // namespace detail

} // namespace planar
} // namespace cvc
