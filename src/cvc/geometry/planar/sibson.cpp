/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's Sibson (natural-neighbour) coordinates and interpolation
// (inc/cvc/geometry/planar.h), on a Delaunay triangulation from delaunay(). They replace CGAL's GPL
// natural_neighbor_coordinates_2 and linear_interpolation, which ContourTiler's interp.cpp used to
// lift untiled-region vertices off the slice planes.
//
// Inserting q into the triangulation would destroy the triangles whose circumcircles hold q (the
// conflict region); its boundary vertices are q's natural neighbours. Each neighbour v gives up the
// part of its Voronoi cell that becomes q's: a convex polygon whose corners are the circumcentres
// of v's destroyed triangles and of the two new triangles (q, v's neighbours on the boundary, v).
// Sibson's coordinate of v is that area over the area of q's whole new cell.

#include "planar_internal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace cvc {
namespace planar {

namespace {

pointd as_d(const point &p) { return pointd(double(p.x), double(p.y)); }

// The circumcentre of a, b, c, relative to the origin o (which keeps the products small).
pointd circumcentre(const pointd &o, const pointd &a, const pointd &b, const pointd &c) {
  const double ax = a.x - o.x, ay = a.y - o.y, bx = b.x - o.x, by = b.y - o.y, cx = c.x - o.x,
               cy = c.y - o.y;
  const double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
  const double a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
  return pointd((a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d,
                (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d);
}

// The area of a convex polygon given its corners in any order (sorted by angle about their mean).
double convex_area(std::vector<pointd> pts) {
  if (pts.size() < 3)
    return 0;
  pointd c(0, 0);
  for (const pointd &p : pts) {
    c.x += p.x;
    c.y += p.y;
  }
  c.x /= double(pts.size());
  c.y /= double(pts.size());
  std::sort(pts.begin(), pts.end(), [&c](const pointd &a, const pointd &b) {
    return std::atan2(a.y - c.y, a.x - c.x) < std::atan2(b.y - c.y, b.x - c.x);
  });
  double s = 0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const pointd &a = pts[i], &b = pts[(i + 1) % pts.size()];
    s += (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
  }
  return std::fabs(s) / 2;
}

bool on_closed_segment(const pointd &q, const pointd &a, const pointd &b) {
  return orientation(a, b, q) == 0 && std::min(a.x, b.x) <= q.x && q.x <= std::max(a.x, b.x) &&
         std::min(a.y, b.y) <= q.y && q.y <= std::max(a.y, b.y);
}

} // namespace

std::vector<natural_neighbor> sibson_coordinates(const triangulation &dt, const pointd &q) {
  if (!std::isfinite(q.x) || !std::isfinite(q.y))
    throw planar_error("planar::sibson_coordinates: the query point is not finite");
  const std::vector<point> &v = dt.vertices;
  const std::size_t nt = dt.triangles.size();
  if (dt.neighbors.size() != nt)
    throw planar_error("planar::sibson_coordinates: the triangulation has no neighbour table");
  if (nt == 0)
    return {};
  // The triangle holding q (closed), by a visibility walk: step across any edge q is strictly
  // beyond. In a Delaunay triangulation the walk cannot cycle; crossing a hull edge means q is
  // outside the hull. A scan of every triangle backs it up (for a triangulation that is not
  // Delaunay).
  int start = -1;
  bool outside = false;
  for (std::size_t steps = 0, t = 0; steps <= nt && start < 0 && !outside; ++steps) {
    const auto &tri = dt.triangles[t];
    bool moved = false;
    for (int k = 0; k < 3 && !moved; ++k) {
      const int i = int((k + steps) % 3);
      if (orientation(as_d(v[tri[(i + 1) % 3]]), as_d(v[tri[(i + 2) % 3]]), q) >= 0)
        continue;
      const int n = dt.neighbors[t][i];
      if (n < 0)
        outside = true;
      else
        t = std::size_t(n);
      moved = true;
    }
    if (!moved)
      start = int(t);
  }
  if (outside)
    return {}; // beyond a hull edge
  for (std::size_t t = 0; t < nt && start < 0; ++t) {
    const auto &tri = dt.triangles[t];
    const pointd a = as_d(v[tri[0]]), b = as_d(v[tri[1]]), c = as_d(v[tri[2]]);
    if (orientation(a, b, q) >= 0 && orientation(b, c, q) >= 0 && orientation(c, a, q) >= 0)
      start = int(t);
  }
  if (start < 0)
    return {}; // outside the convex hull
  const auto &st = dt.triangles[std::size_t(start)];
  for (int i = 0; i < 3; ++i) {
    const pointd p = as_d(v[st[i]]);
    if (p.x == q.x && p.y == q.y)
      return {{st[i], 1.0}};
  }
  // On the hull's boundary the new cell is unbounded: the coordinates are the edge's linear ones.
  for (int i = 0; i < 3; ++i) {
    if (dt.neighbors[std::size_t(start)][i] >= 0)
      continue;
    const int ia = st[(i + 1) % 3], ib = st[(i + 2) % 3];
    const pointd a = as_d(v[ia]), b = as_d(v[ib]);
    if (on_closed_segment(q, a, b)) {
      const double l = std::hypot(b.x - a.x, b.y - a.y);
      const double tb = std::hypot(q.x - a.x, q.y - a.y) / l;
      return {{ia, 1 - tb}, {ib, tb}};
    }
  }
  // The conflict region: triangles whose circumcircle holds q strictly, grown from `start`.
  std::vector<char> conflict(nt, 0);
  std::vector<int> stack = {start}, region;
  conflict[std::size_t(start)] = 1;
  while (!stack.empty()) {
    const int t = stack.back();
    stack.pop_back();
    region.push_back(t);
    for (int i = 0; i < 3; ++i) {
      const int n = dt.neighbors[std::size_t(t)][i];
      if (n < 0 || conflict[std::size_t(n)])
        continue;
      const auto &tri = dt.triangles[std::size_t(n)];
      if (in_circle(as_d(v[tri[0]]), as_d(v[tri[1]]), as_d(v[tri[2]]), q) > 0) {
        conflict[std::size_t(n)] = 1;
        stack.push_back(n);
      }
    }
  }
  // Its boundary, counterclockwise: next_on_boundary[a] = b for each boundary edge a -> b.
  std::map<int, int> next_on_boundary;
  for (int t : region)
    for (int i = 0; i < 3; ++i) {
      const int n = dt.neighbors[std::size_t(t)][i];
      if (n >= 0 && conflict[std::size_t(n)])
        continue; // inside the region (a hull edge, n < 0, bounds it; q is strictly inside)
      const auto &tri = dt.triangles[std::size_t(t)];
      next_on_boundary[tri[(i + 1) % 3]] = tri[(i + 2) % 3];
    }
  std::map<int, int> prev_on_boundary;
  for (const auto &e : next_on_boundary)
    prev_on_boundary[e.second] = e.first;
  // Each neighbour's stolen area.
  std::vector<natural_neighbor> out;
  double total = 0;
  const pointd o = q;
  for (const auto &e : next_on_boundary) {
    const int w = e.first, wn = e.second, wp = prev_on_boundary[w];
    std::vector<pointd> corners = {circumcentre(o, q, as_d(v[wp]), as_d(v[w])),
                                   circumcentre(o, q, as_d(v[w]), as_d(v[wn]))};
    for (int t : region) {
      const auto &tri = dt.triangles[std::size_t(t)];
      if (tri[0] == w || tri[1] == w || tri[2] == w)
        corners.push_back(circumcentre(o, as_d(v[tri[0]]), as_d(v[tri[1]]), as_d(v[tri[2]])));
    }
    const double a = convex_area(corners);
    out.push_back({w, a});
    total += a;
  }
  if (!(total > 0))
    return {};
  for (natural_neighbor &n : out)
    n.weight /= total;
  return out;
}

double sibson_interpolate(const triangulation &dt, const std::vector<double> &values,
                          const pointd &q) {
  if (values.size() != dt.vertices.size())
    throw planar_error("planar::sibson_interpolate: " + std::to_string(values.size()) +
                       " values for " + std::to_string(dt.vertices.size()) + " vertices");
  const std::vector<natural_neighbor> c = sibson_coordinates(dt, q);
  if (c.empty())
    return std::numeric_limits<double>::quiet_NaN();
  double s = 0;
  for (const natural_neighbor &n : c)
    s += n.weight * values[std::size_t(n.vertex)];
  return s;
}

} // namespace planar
} // namespace cvc
