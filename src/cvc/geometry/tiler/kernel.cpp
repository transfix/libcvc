/*
  Copyright 2026 The University of Texas at Austin

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

// The tiler kernel's predicates, intersections and polygon algorithms (kernel.h).

#include "kernel.h"

#include "../exact_predicates.h"

namespace cvc::tiler {

double exact_orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
  const double a[2] = {ax, ay}, b[2] = {bx, by}, c[2] = {cx, cy};
  return cvc::exact::orient2d(a, b, c);
}

double exact_orient3d(const double *a, const double *b, const double *c, const double *d) {
  return cvc::exact::orient3d(a, b, c, d);
}

Orientation coplanar_orientation(const Point_3 &p, const Point_3 &q, const Point_3 &r,
                                 const Point_3 &s) {
  // In the first coordinate plane where p, q, r project to a proper triangle, compare the sides
  // of pq that r and s are on.
  const double pxy = exact_orient2d(p.x(), p.y(), q.x(), q.y(), r.x(), r.y());
  if (pxy != 0)
    return Orientation(int(sign_of(pxy)) *
                       int(sign_of(exact_orient2d(p.x(), p.y(), q.x(), q.y(), s.x(), s.y()))));
  const double pyz = exact_orient2d(p.y(), p.z(), q.y(), q.z(), r.y(), r.z());
  if (pyz != 0)
    return Orientation(int(sign_of(pyz)) *
                       int(sign_of(exact_orient2d(p.y(), p.z(), q.y(), q.z(), s.y(), s.z()))));
  const double pxz = exact_orient2d(p.x(), p.z(), q.x(), q.z(), r.x(), r.z());
  return Orientation(int(sign_of(pxz)) *
                     int(sign_of(exact_orient2d(p.x(), p.z(), q.x(), q.z(), s.x(), s.z()))));
}

// ============================================================================================
// Intersections

namespace {

// [lo, hi] of two collinear segments' overlap along their line, lexicographically; false when
// they don't overlap.
bool collinear_overlap(const Segment_2 &a, const Segment_2 &b, Point_2 &lo, Point_2 &hi) {
  const Point_2 &amin = a.min(), &amax = a.max(), &bmin = b.min(), &bmax = b.max();
  lo = lexicographically_xy_smaller(amin, bmin) ? bmin : amin;
  hi = lexicographically_xy_smaller(amax, bmax) ? amax : bmax;
  return !lexicographically_xy_smaller(hi, lo);
}

// The point where the lines through segments a and b cross, for segments known to cross in their
// interiors (not at an endpoint).
Point_2 interior_crossing(const Segment_2 &a, const Segment_2 &b) {
  const Point_2 &p0 = a.source(), &p1 = a.target(), &q0 = b.source(), &q1 = b.target();
  // Exact coordinates where one segment is axis-parallel.
  if (p0.x() == p1.x() && q0.y() == q1.y())
    return Point_2(p0.x(), q0.y());
  if (q0.x() == q1.x() && p0.y() == p1.y())
    return Point_2(q0.x(), p0.y());
  const double dx = p1.x() - p0.x(), dy = p1.y() - p0.y();
  const double ex = q1.x() - q0.x(), ey = q1.y() - q0.y();
  const double denom = dx * ey - dy * ex;
  double alpha = ((q0.x() - p0.x()) * ey - (q0.y() - p0.y()) * ex) / denom;
  if (!(alpha == alpha))
    alpha = 0.5;
  alpha = std::min(1.0, std::max(0.0, alpha));
  double x = p0.x() + alpha * dx, y = p0.y() + alpha * dy;
  if (p0.x() == p1.x())
    x = p0.x();
  if (p0.y() == p1.y())
    y = p0.y();
  if (q0.x() == q1.x())
    x = q0.x();
  if (q0.y() == q1.y())
    y = q0.y();
  return Point_2(x, y);
}

} // namespace

bool do_intersect(const Segment_2 &a, const Segment_2 &b) {
  if (a.is_degenerate())
    return b.has_on(a.source());
  if (b.is_degenerate())
    return a.has_on(b.source());
  const Orientation o1 = orientation(a.source(), a.target(), b.source());
  const Orientation o2 = orientation(a.source(), a.target(), b.target());
  if (o1 != COLLINEAR && o1 == o2)
    return false;
  const Orientation o3 = orientation(b.source(), b.target(), a.source());
  const Orientation o4 = orientation(b.source(), b.target(), a.target());
  if (o3 != COLLINEAR && o3 == o4)
    return false;
  if (o1 == COLLINEAR && o2 == COLLINEAR) {
    Point_2 lo, hi;
    return collinear_overlap(a, b, lo, hi);
  }
  return true;
}

Object intersection(const Segment_2 &a, const Segment_2 &b) {
  if (!do_intersect(a, b))
    return Object();
  if (a.is_degenerate())
    return make_object(a.source());
  if (b.is_degenerate())
    return make_object(b.source());
  const Orientation o1 = orientation(a.source(), a.target(), b.source());
  const Orientation o2 = orientation(a.source(), a.target(), b.target());
  if (o1 == COLLINEAR && o2 == COLLINEAR) {
    Point_2 lo, hi;
    collinear_overlap(a, b, lo, hi);
    if (lo == hi)
      return make_object(lo);
    if (lexicographically_xy_smaller(a.source(), a.target()))
      return make_object(Segment_2(lo, hi));
    return make_object(Segment_2(hi, lo));
  }
  if (o1 == COLLINEAR)
    return make_object(b.source());
  if (o2 == COLLINEAR)
    return make_object(b.target());
  if (orientation(b.source(), b.target(), a.source()) == COLLINEAR)
    return make_object(a.source());
  if (orientation(b.source(), b.target(), a.target()) == COLLINEAR)
    return make_object(a.target());
  return make_object(interior_crossing(a, b));
}

Object intersection(const Line_2 &a, const Line_2 &b) {
  const double det = a.a() * b.b() - b.a() * a.b();
  if (det == 0) {
    // Parallel: the same line when the coefficients are proportional.
    const bool same = a.a() * b.c() == b.a() * a.c() && a.b() * b.c() == b.b() * a.c();
    if (same)
      return make_object(a);
    return Object();
  }
  return make_object(Point_2((a.b() * b.c() - b.b() * a.c()) / det,
                             (b.a() * a.c() - a.a() * b.c()) / det));
}

Object intersection(const Segment_2 &s, const Line_2 &l) {
  const Oriented_side os = l.oriented_side(s.source()), ot = l.oriented_side(s.target());
  if (os == ON_ORIENTED_BOUNDARY && ot == ON_ORIENTED_BOUNDARY) {
    if (s.is_degenerate())
      return make_object(s.source());
    return make_object(s);
  }
  if (os == ON_ORIENTED_BOUNDARY)
    return make_object(s.source());
  if (ot == ON_ORIENTED_BOUNDARY)
    return make_object(s.target());
  if (os == ot)
    return Object();
  Object o = intersection(s.supporting_line(), l);
  Point_2 p;
  if (assign(p, o))
    return make_object(p);
  return Object();
}

bool do_intersect(const Ray_2 &r, const Segment_2 &s) { return !intersection(r, s).empty(); }

Object intersection(const Ray_2 &r, const Segment_2 &s) {
  const Point_2 &o = r.source();
  const Vector_2 d = r.to_vector();
  const Orientation os = orientation(o, r.second_point(), s.source());
  const Orientation ot = orientation(o, r.second_point(), s.target());
  if (os == COLLINEAR && ot == COLLINEAR) {
    // Along the ray: keep the part of the segment at or past its source.
    const double ts = (s.source() - o) * d, tt = (s.target() - o) * d;
    const double tmin = std::min(ts, tt), tmax = std::max(ts, tt);
    if (tmax < 0)
      return Object();
    const Point_2 &far = (ts >= tt) ? s.source() : s.target();
    if (tmin >= 0) {
      if (s.is_degenerate())
        return make_object(s.source());
      return make_object(s);
    }
    if (tmax == 0)
      return make_object(o);
    return make_object(Segment_2(o, far));
  }
  if (os != COLLINEAR && os == ot)
    return Object();
  // The segment crosses or touches the ray's line: where, and is it on the ray?
  if (os == COLLINEAR)
    return r.collinear_has_on(s.source()) ? make_object(s.source()) : Object();
  if (ot == COLLINEAR)
    return r.collinear_has_on(s.target()) ? make_object(s.target()) : Object();
  // The segment crosses the ray's line at o + t d with t = cross(s0 - o, sv) / cross(d, sv), and
  // cross(s0 - o, sv) = orient(o, s0, s1); cross(d, sv) has the sign of ot (os and ot differ), so
  // t >= 0 is decided exactly.
  const Orientation num = orientation(o, s.source(), s.target());
  if (num == COLLINEAR)
    return make_object(o);
  if (num != ot)
    return Object();
  Object lo = intersection(s.supporting_line(), r.supporting_line());
  Point_2 p;
  if (assign(p, lo))
    return make_object(p);
  return Object();
}

// ============================================================================================
// Polygons

namespace {

// Do edge i and edge j (consecutive, sharing vertex v) overlap beyond it? They do when they are
// collinear and turn back on each other.
bool consecutive_overlap(const Point_2 &a, const Point_2 &v, const Point_2 &b) {
  if (orientation(a, v, b) != COLLINEAR)
    return false;
  return (a - v) * (b - v) > 0;
}

} // namespace

bool Polygon_2::is_simple() const {
  const std::size_t n = _v.size();
  if (n < 3)
    return false;
  for (std::size_t i = 0; i < n; ++i)
    if (_v[i] == _v[(i + 1) % n])
      return false;
  std::vector<Bbox_2> boxes(n);
  for (std::size_t i = 0; i < n; ++i)
    boxes[i] = edge(i).bbox();
  for (std::size_t i = 0; i < n; ++i) {
    // Consecutive edges i and i + 1 share vertex i + 1.
    if (consecutive_overlap(_v[i], _v[(i + 1) % n], _v[(i + 2) % n]))
      return false;
    for (std::size_t j = i + 2; j < n; ++j) {
      if (i == 0 && j == n - 1)
        continue; // consecutive through vertex 0
      if (!do_overlap(boxes[i], boxes[j]))
        continue;
      if (do_intersect(edge(i), edge(j)))
        return false;
    }
  }
  return true;
}

bool Polygon_2::is_convex() const {
  // Consecutive repeated vertices are ignored, as are collinear ones; a convex polygon turns one
  // way only and goes once around (its lexicographic order changes direction at most twice).
  std::vector<Point_2> v;
  for (const Point_2 &p : _v)
    if (v.empty() || !(v.back() == p))
      v.push_back(p);
  while (v.size() > 1 && v.front() == v.back())
    v.pop_back();
  const std::size_t n = v.size();
  if (n < 3)
    return true;
  bool cw = false, ccw = false;
  int changes = 0;
  bool order = lexicographically_xy_smaller(v[0], v[1]);
  for (std::size_t i = 0; i < n; ++i) {
    const Point_2 &a = v[i], &b = v[(i + 1) % n], &c = v[(i + 2) % n];
    const Orientation o = tiler::orientation(a, b, c);
    if (o == CLOCKWISE)
      cw = true;
    else if (o == COUNTERCLOCKWISE)
      ccw = true;
    const bool o2 = lexicographically_xy_smaller(b, c);
    if (o2 != order) {
      ++changes;
      order = o2;
    }
  }
  if (cw && ccw)
    return false;
  return changes <= 2;
}

Orientation Polygon_2::orientation() const {
  const std::size_t n = _v.size();
  if (n < 3)
    return COLLINEAR;
  std::size_t m = 0;
  for (std::size_t i = 1; i < n; ++i)
    if (lexicographically_xy_smaller(_v[i], _v[m]))
      m = i;
  return tiler::orientation(_v[(m + n - 1) % n], _v[m], _v[(m + 1) % n]);
}

Bounded_side Polygon_2::bounded_side(const Point_2 &p) const {
  const std::size_t n = _v.size();
  if (n == 0)
    return ON_UNBOUNDED_SIDE;
  bool inside = false;
  for (std::size_t i = 0; i < n; ++i) {
    const Point_2 &a = _v[i], &b = _v[(i + 1) % n];
    if (are_ordered_along_line(a, p, b))
      return ON_BOUNDARY;
    // Crossings of the ray to the right of p, each edge half-open in y.
    if ((a.y() <= p.y()) != (b.y() <= p.y())) {
      const Orientation o = tiler::orientation(a, b, p);
      if ((b.y() > a.y() && o == LEFT_TURN) || (b.y() < a.y() && o == RIGHT_TURN))
        inside = !inside;
    }
  }
  return inside ? ON_BOUNDED_SIDE : ON_UNBOUNDED_SIDE;
}

Oriented_side Polygon_2::oriented_side(const Point_2 &p) const {
  const Bounded_side b = bounded_side(p);
  if (b == ON_BOUNDARY)
    return ON_ORIENTED_BOUNDARY;
  const Orientation o = orientation();
  if (o == COLLINEAR)
    return ON_NEGATIVE_SIDE;
  const bool in = (b == ON_BOUNDED_SIDE);
  return (in == (o == COUNTERCLOCKWISE)) ? ON_POSITIVE_SIDE : ON_NEGATIVE_SIDE;
}

Number_type Polygon_2::area() const {
  Number_type result = 0;
  const std::size_t n = _v.size();
  if (n < 3)
    return result;
  for (std::size_t i = 1; i + 1 < n; ++i)
    result = result + tiler::area(_v[0], _v[i], _v[i + 1]);
  return result;
}

Bbox_2 Polygon_2::bbox() const {
  Bbox_2 b;
  for (const Point_2 &p : _v)
    b += p.bbox();
  return b;
}

Polygon_2::Vertex_const_iterator Polygon_2::left_vertex() const {
  return std::min_element(_v.begin(), _v.end(), [](const Point_2 &a, const Point_2 &b) {
    return lexicographically_xy_smaller(a, b);
  });
}

Polygon_2::Vertex_const_iterator Polygon_2::right_vertex() const {
  return std::max_element(_v.begin(), _v.end(), [](const Point_2 &a, const Point_2 &b) {
    return lexicographically_xy_smaller(a, b);
  });
}

namespace {
bool less_yx(const Point_2 &a, const Point_2 &b) {
  return a.y() < b.y() || (a.y() == b.y() && a.x() < b.x());
}
} // namespace

Polygon_2::Vertex_const_iterator Polygon_2::top_vertex() const {
  return std::max_element(_v.begin(), _v.end(), less_yx);
}

Polygon_2::Vertex_const_iterator Polygon_2::bottom_vertex() const {
  return std::min_element(_v.begin(), _v.end(), less_yx);
}

bool Polygon_2::operator==(const Polygon_2 &o) const {
  const std::size_t n = _v.size();
  if (n != o._v.size())
    return false;
  if (n == 0)
    return true;
  for (std::size_t s = 0; s < n; ++s) {
    if (!(_v[s] == o._v[0]))
      continue;
    bool same = true;
    for (std::size_t i = 0; i < n && same; ++i)
      same = _v[(s + i) % n] == o._v[i];
    if (same)
      return true;
  }
  return false;
}

std::ostream &operator<<(std::ostream &os, const Polygon_2 &p) {
  os << p.size();
  for (const Point_2 &q : p.container())
    os << ' ' << q.x() << ' ' << q.y();
  return os;
}

std::ostream &operator<<(std::ostream &os, const Polygon_with_holes_2 &p) {
  os << p.outer_boundary() << ' ' << p.number_of_holes();
  for (auto it = p.holes_begin(); it != p.holes_end(); ++it)
    os << ' ' << *it;
  return os;
}

} // namespace cvc::tiler
