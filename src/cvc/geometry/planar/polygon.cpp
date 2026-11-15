/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::planar's rings and polygons (inc/cvc/geometry/planar.h): areas, orientation, simplicity,
// point location, clean-up and nesting, all on the exact predicates.

#include "planar_internal.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <string>
#include <vector>

namespace cvc {
namespace planar {

namespace {

struct box {
  coord xmin, ymin, xmax, ymax;
  bool overlaps(const box &o) const {
    return xmin <= o.xmax && o.xmin <= xmax && ymin <= o.ymax && o.ymin <= ymax;
  }
  bool contains(const point &p) const {
    return xmin <= p.x && p.x <= xmax && ymin <= p.y && p.y <= ymax;
  }
};

box bounds(const ring &r) {
  box b{r[0].x, r[0].y, r[0].x, r[0].y};
  for (const point &p : r) {
    b.xmin = std::min(b.xmin, p.x);
    b.xmax = std::max(b.xmax, p.x);
    b.ymin = std::min(b.ymin, p.y);
    b.ymax = std::max(b.ymax, p.y);
  }
  return b;
}

// An edge of a ring, for the sweep below: its endpoints, its x range and its index.
struct edge {
  point a, b;
  coord xmin, xmax;
  std::size_t ring, index;
};

std::vector<edge> edges_of(const ring &r, std::size_t ring_index) {
  std::vector<edge> es;
  const std::size_t n = r.size();
  es.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const point &a = r[i], &b = r[(i + 1) % n];
    es.push_back({a, b, std::min(a.x, b.x), std::max(a.x, b.x), ring_index, i});
  }
  return es;
}

// Calls f(e, g) for every pair of edges whose x ranges overlap, until f returns true; returns
// whether one did. Sorted by xmin, each edge meets only those that start before it ends.
template <typename F> bool any_overlapping_pair(std::vector<edge> &es, F f) {
  std::sort(es.begin(), es.end(), [](const edge &l, const edge &r) { return l.xmin < r.xmin; });
  for (std::size_t i = 0; i < es.size(); ++i)
    for (std::size_t j = i + 1; j < es.size() && es[j].xmin <= es[i].xmax; ++j)
      if (f(es[i], es[j]))
        return true;
  return false;
}

// Do two edges of one ring of n vertices meet anywhere but at the vertex consecutive ones share?
bool edges_conflict(const edge &e, const edge &f, std::size_t n) {
  const std::size_t i = std::min(e.index, f.index), j = std::max(e.index, f.index);
  const bool consecutive = j == i + 1 || (i == 0 && j == n - 1);
  if (!consecutive)
    return segments_intersect(e.a, e.b, f.a, f.b);
  // Consecutive edges share one vertex v: they overlap only where the chain turns back on itself
  // (a spike), i.e. the other two endpoints are collinear with v and on the same side of it.
  const edge &first = (j == i + 1) ? (e.index == i ? e : f) : (e.index == j ? e : f);
  const edge &second = (&first == &e) ? f : e;
  const point &a = first.a, &v = first.b, &b = second.b;
  if (orientation(a, v, b) != 0)
    return false;
  const double dot = double(v.x - a.x) * double(b.x - v.x) + double(v.y - a.y) * double(b.y - v.y);
  return dot < 0;
}

} // namespace

double signed_area(const ring &r) {
  const std::size_t n = r.size();
  if (n < 3)
    return 0;
  // Relative to the first vertex, so the products stay as small as the ring.
  long double s = 0;
  const coord x0 = r[0].x, y0 = r[0].y;
  for (std::size_t i = 1; i + 1 < n; ++i) {
    const long double ax = (long double)(r[i].x - x0), ay = (long double)(r[i].y - y0);
    const long double bx = (long double)(r[i + 1].x - x0), by = (long double)(r[i + 1].y - y0);
    s += ax * by - bx * ay;
  }
  return double(s / 2);
}

double area(const polygon &p) {
  double a = std::fabs(signed_area(p.outer));
  for (const ring &h : p.holes)
    a -= std::fabs(signed_area(h));
  return a;
}

double area(const polygon_set &s) {
  double a = 0;
  for (const polygon &p : s)
    a += area(p);
  return a;
}

double perimeter(const ring &r) {
  double l = 0;
  const std::size_t n = r.size();
  for (std::size_t i = 0; i < n && n > 1; ++i) {
    const point &a = r[i], &b = r[(i + 1) % n];
    l += std::hypot(double(b.x - a.x), double(b.y - a.y));
  }
  return l;
}

double perimeter(const polygon &p) {
  double l = perimeter(p.outer);
  for (const ring &h : p.holes)
    l += perimeter(h);
  return l;
}

int orientation(const ring &r) {
  const std::size_t n = r.size();
  if (n < 3)
    return 0;
  std::size_t k = 0;
  for (std::size_t i = 1; i < n; ++i)
    if (r[i].y < r[k].y || (r[i].y == r[k].y && r[i].x < r[k].x))
      k = i;
  // The nearest vertices before and after k that differ from it.
  std::size_t p = (k + n - 1) % n, q = (k + 1) % n;
  while (p != k && r[p] == r[k])
    p = (p + n - 1) % n;
  while (q != k && r[q] == r[k])
    q = (q + 1) % n;
  if (p == k || q == k)
    return 0;
  const int o = orientation(r[p], r[k], r[q]);
  if (o != 0)
    return o;
  // A spike at the extreme vertex: fall back on the area's sign.
  const double a = signed_area(r);
  return (a > 0) - (a < 0);
}

bool is_simple(const ring &r) {
  const std::size_t n = r.size();
  if (n < 3)
    return false;
  ring sorted(r);
  std::sort(sorted.begin(), sorted.end());
  if (std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end())
    return false; // a repeated vertex
  std::vector<edge> es = edges_of(r, 0);
  return !any_overlapping_pair(
      es, [n](const edge &e, const edge &f) { return edges_conflict(e, f, n); });
}

bool is_simple(const polygon &p) {
  if (!is_simple(p.outer))
    return false;
  std::vector<edge> es = edges_of(p.outer, 0);
  for (std::size_t h = 0; h < p.holes.size(); ++h) {
    if (!is_simple(p.holes[h]))
      return false;
    const std::vector<edge> eh = edges_of(p.holes[h], h + 1);
    es.insert(es.end(), eh.begin(), eh.end());
  }
  // Each ring is simple, so only edges of different rings can meet.
  return !any_overlapping_pair(es, [](const edge &e, const edge &f) {
    return e.ring != f.ring && segments_intersect(e.a, e.b, f.a, f.b);
  });
}

location locate(const point &p, const ring &r) {
  const std::size_t n = r.size();
  if (n == 0)
    return location::outside;
  int winding = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const point &a = r[i], &b = r[(i + 1) % n];
    if (on_segment(p, a, b))
      return location::boundary;
    if (a.y <= p.y) {
      if (b.y > p.y && orientation(a, b, p) > 0)
        ++winding;
    } else if (b.y <= p.y && orientation(a, b, p) < 0) {
      --winding;
    }
  }
  return winding != 0 ? location::inside : location::outside;
}

location locate(const point &p, const polygon &poly) {
  const location l = locate(p, poly.outer);
  if (l != location::inside)
    return l;
  for (const ring &h : poly.holes) {
    const location lh = locate(p, h);
    if (lh == location::boundary)
      return location::boundary;
    if (lh == location::inside)
      return location::outside;
  }
  return location::inside;
}

location locate(const point &p, const polygon_set &s) {
  location best = location::outside;
  for (const polygon &poly : s) {
    const location l = locate(p, poly);
    if (l == location::inside)
      return l;
    if (l == location::boundary)
      best = l;
  }
  return best;
}

ring clean(const ring &r) {
  ring pts;
  pts.reserve(r.size());
  for (const point &p : r)
    if (pts.empty() || pts.back() != p)
      pts.push_back(p);
  while (pts.size() > 1 && pts.front() == pts.back())
    pts.pop_back();
  const std::size_t n = pts.size();
  if (n < 3)
    return pts;
  // A doubly linked cycle: a vertex equal to its predecessor, or collinear with its neighbours,
  // goes, and its neighbours are looked at again.
  std::vector<std::size_t> prev(n), next(n), work(n);
  std::vector<char> alive(n, 1);
  for (std::size_t i = 0; i < n; ++i) {
    prev[i] = (i + n - 1) % n;
    next[i] = (i + 1) % n;
  }
  std::iota(work.begin(), work.end(), std::size_t(0));
  std::size_t count = n;
  while (!work.empty() && count >= 3) {
    const std::size_t i = work.back();
    work.pop_back();
    if (!alive[i])
      continue;
    const std::size_t a = prev[i], b = next[i];
    if (pts[a] == pts[i] || orientation(pts[a], pts[i], pts[b]) == 0) {
      alive[i] = 0;
      next[a] = b;
      prev[b] = a;
      --count;
      work.push_back(a);
      work.push_back(b);
    }
  }
  ring out;
  out.reserve(count);
  std::size_t start = 0;
  while (!alive[start])
    ++start;
  std::size_t i = start;
  do {
    out.push_back(pts[i]);
    i = next[i];
  } while (i != start && out.size() < count);
  return out;
}

polygon normalize(const polygon &p) {
  polygon out;
  out.outer = clean(p.outer);
  if (out.outer.size() < 3)
    return polygon();
  if (orientation(out.outer) < 0)
    std::reverse(out.outer.begin(), out.outer.end());
  for (const ring &h : p.holes) {
    ring c = clean(h);
    if (c.size() < 3)
      continue;
    if (orientation(c) > 0)
      std::reverse(c.begin(), c.end());
    out.holes.push_back(std::move(c));
  }
  return out;
}

polygon_set nest(const std::vector<ring> &rings) {
  std::vector<ring> rs;
  std::vector<std::size_t> from; // input index, for messages
  for (std::size_t i = 0; i < rings.size(); ++i) {
    detail::check_range("planar::nest", rings[i]);
    ring c = clean(rings[i]);
    if (c.size() < 3)
      continue;
    if (!is_simple(c))
      throw planar_error("planar::nest: ring " + std::to_string(i) + " is not simple");
    rs.push_back(std::move(c));
    from.push_back(i);
  }
  const std::size_t m = rs.size();
  std::vector<box> boxes(m);
  for (std::size_t i = 0; i < m; ++i)
    boxes[i] = bounds(rs[i]);
  // No two rings may share a point.
  {
    std::vector<edge> es;
    for (std::size_t i = 0; i < m; ++i) {
      const std::vector<edge> e = edges_of(rs[i], i);
      es.insert(es.end(), e.begin(), e.end());
    }
    std::size_t bad_a = 0, bad_b = 0;
    const bool crossed = any_overlapping_pair(es, [&](const edge &e, const edge &f) {
      if (e.ring == f.ring || !segments_intersect(e.a, e.b, f.a, f.b))
        return false;
      bad_a = std::min(e.ring, f.ring);
      bad_b = std::max(e.ring, f.ring);
      return true;
    });
    if (crossed)
      throw planar_error("planar::nest: rings " + std::to_string(from[bad_a]) + " and " +
                         std::to_string(from[bad_b]) + " share a point");
  }
  // contains[i] lists the rings ring i lies inside. No boundaries meet, so any vertex of i is
  // strictly inside or strictly outside every other ring.
  std::vector<std::vector<std::size_t>> contains(m);
  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = 0; j < m; ++j)
      if (i != j && boxes[j].contains(rs[i][0]) && locate(rs[i][0], rs[j]) == location::inside)
        contains[i].push_back(j);
  polygon_set out;
  std::vector<std::size_t> polygon_of(m, std::size_t(-1));
  for (std::size_t i = 0; i < m; ++i)
    if (contains[i].size() % 2 == 0) {
      polygon_of[i] = out.size();
      ring r = rs[i];
      if (orientation(r) < 0)
        std::reverse(r.begin(), r.end());
      out.emplace_back(std::move(r));
    }
  for (std::size_t i = 0; i < m; ++i) {
    if (contains[i].size() % 2 == 0)
      continue;
    // The smallest ring around a hole is the one nested deepest.
    std::size_t parent = contains[i][0];
    for (std::size_t j : contains[i])
      if (contains[j].size() > contains[parent].size())
        parent = j;
    ring r = rs[i];
    if (orientation(r) > 0)
      std::reverse(r.begin(), r.end());
    out[polygon_of[parent]].holes.push_back(std::move(r));
  }
  return out;
}

} // namespace planar
} // namespace cvc
