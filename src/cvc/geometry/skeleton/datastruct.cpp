/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The cocone code's kernel and triangulation adapter (datastruct.h).

#include "datastruct.h"

#ifdef CVC_WITH_SHEWCHUK
#include <shewchuk/predicates.hpp>
#endif

namespace cvc::cocone {

namespace {

// CGAL's 3x3 determinant (CGAL/determinant.h), term for term.
double det3(double a00, double a01, double a02, double a10, double a11, double a12, double a20,
            double a21, double a22) {
  const double m01 = a00 * a11 - a10 * a01;
  const double m02 = a00 * a21 - a20 * a01;
  const double m12 = a10 * a21 - a20 * a11;
  return m01 * a22 - m02 * a12 + m12 * a02;
}
double det2(double a00, double a01, double a10, double a11) { return a00 * a11 - a10 * a01; }

inline int sgn(double v) { return (v > 0.0) - (v < 0.0); }

#ifdef CVC_WITH_SHEWCHUK
int orient3(const Point &a, const Point &b, const Point &c, const Point &d) {
  return -sgn(shewchuk::detail::orient3d(a.c, b.c, c.c, d.c)); // CGAL's sign
}
// The 2D orientation of three points projected along axis k (dropping coordinate k).
int orient2_drop(const Point &a, const Point &b, const Point &c, int k) {
  const int i = k == 0 ? 1 : 0, j = k == 2 ? 1 : 2;
  const double pa[2] = {a.c[i], a.c[j]}, pb[2] = {b.c[i], b.c[j]}, pc[2] = {c.c[i], c.c[j]};
  return sgn(shewchuk::detail::orient2d(pa, pb, pc));
}
#else
int orient3(const Point &a, const Point &b, const Point &c, const Point &d) {
  const Vector u = b - a, v = c - a, w = d - a;
  return -sgn(det3(u.c[0], u.c[1], u.c[2], v.c[0], v.c[1], v.c[2], w.c[0], w.c[1], w.c[2]));
}
int orient2_drop(const Point &a, const Point &b, const Point &c, int k) {
  const int i = k == 0 ? 1 : 0, j = k == 2 ? 1 : 2;
  return sgn((b.c[i] - a.c[i]) * (c.c[j] - a.c[j]) - (b.c[j] - a.c[j]) * (c.c[i] - a.c[i]));
}
#endif

} // namespace

double Triangle_3::squared_area() const {
  const Vector vq = p[1] - p[0], vr = p[2] - p[0];
  return cg::cross_product(vq, vr).squared_length() / 4.0;
}
bool Triangle_3::is_degenerate() const { return cg::collinear(p[0], p[1], p[2]); }

double Tetrahedron::volume() const {
  return det3(p[1].c[0] - p[0].c[0], p[1].c[1] - p[0].c[1], p[1].c[2] - p[0].c[2],
              p[2].c[0] - p[0].c[0], p[2].c[1] - p[0].c[1], p[2].c[2] - p[0].c[2],
              p[3].c[0] - p[0].c[0], p[3].c[1] - p[0].c[1], p[3].c[2] - p[0].c[2]) /
         6.0;
}
bool Tetrahedron::is_degenerate() const { return orient3(p[0], p[1], p[2], p[3]) == 0; }

namespace cg {

Point circumcenter(const Point &p, const Point &q, const Point &r, const Point &s) {
  const double dqx = q.c[0] - p.c[0], dqy = q.c[1] - p.c[1], dqz = q.c[2] - p.c[2];
  const double drx = r.c[0] - p.c[0], dry = r.c[1] - p.c[1], drz = r.c[2] - p.c[2];
  const double dsx = s.c[0] - p.c[0], dsy = s.c[1] - p.c[1], dsz = s.c[2] - p.c[2];
  const double r2 = drx * drx + dry * dry + drz * drz;
  const double q2 = dqx * dqx + dqy * dqy + dqz * dqz;
  const double s2 = dsx * dsx + dsy * dsy + dsz * dsz;
  const double num_x = det3(dqy, dqz, q2, dry, drz, r2, dsy, dsz, s2);
  const double num_y = det3(dqx, dqz, q2, drx, drz, r2, dsx, dsz, s2);
  const double num_z = det3(dqx, dqy, q2, drx, dry, r2, dsx, dsy, s2);
  const double den = det3(dqx, dqy, dqz, drx, dry, drz, dsx, dsy, dsz);
  const double inv = 1.0 / (2.0 * den);
  return {p.c[0] + num_x * inv, p.c[1] - num_y * inv, p.c[2] + num_z * inv};
}

Point circumcenter(const Point &p, const Point &q, const Point &r) {
  // CGAL's circumcenterC3 for three points: r translated to the origin.
  const double psx = p.c[0] - r.c[0], psy = p.c[1] - r.c[1], psz = p.c[2] - r.c[2];
  const double ps2 = psx * psx + psy * psy + psz * psz;
  const double qsx = q.c[0] - r.c[0], qsy = q.c[1] - r.c[1], qsz = q.c[2] - r.c[2];
  const double qs2 = qsx * qsx + qsy * qsy + qsz * qsz;
  const double rsx = psy * qsz - psz * qsy;
  const double rsy = psz * qsx - psx * qsz;
  const double rsz = psx * qsy - psy * qsx;
  const double num_x = ps2 * det2(qsy, qsz, rsy, rsz) - qs2 * det2(psy, psz, rsy, rsz);
  const double num_y = ps2 * det2(qsx, qsz, rsx, rsz) - qs2 * det2(psx, psz, rsx, rsz);
  const double num_z = ps2 * det2(qsx, qsy, rsx, rsy) - qs2 * det2(psx, psy, rsx, rsy);
  const double den = det3(psx, psy, psz, qsx, qsy, qsz, rsx, rsy, rsz);
  const double inv = 1.0 / (2.0 * den);
  return {r.c[0] + num_x * inv, r.c[1] - num_y * inv, r.c[2] + num_z * inv};
}

Orientation orientation(const Point &p, const Point &q, const Point &r, const Point &s) {
  return Orientation(orient3(p, q, r, s));
}

bool collinear(const Point &p, const Point &q, const Point &r) {
  return orient2_drop(p, q, r, 0) == 0 && orient2_drop(p, q, r, 1) == 0 &&
         orient2_drop(p, q, r, 2) == 0;
}

namespace {
// Along a line: the axis on which the points u != v differ most, to order points on their line.
int line_axis(const Point &u, const Point &v) {
  int k = 0;
  for (int i = 1; i < 3; ++i)
    if (std::fabs(v.c[i] - u.c[i]) > std::fabs(v.c[k] - u.c[k]))
      k = i;
  return k;
}
// Whether x, known collinear with u and v, lies on the closed segment uv (exact comparisons).
bool between(const Point &u, const Point &v, const Point &x) {
  if (u == v)
    return x == u;
  const int k = line_axis(u, v);
  return std::min(u.c[k], v.c[k]) <= x.c[k] && x.c[k] <= std::max(u.c[k], v.c[k]);
}
// Whether the closed segments uv and pq meet (exact). u may equal v.
bool segments_meet(const Point &u, const Point &v, const Point &p, const Point &q) {
  if (u == v)
    return collinear(p, q, u) && between(p, q, u);
  if (p == q)
    return collinear(u, v, p) && between(u, v, p);
  if (orient3(u, v, p, q) != 0)
    return false;                               // not coplanar
  if (collinear(u, v, p) && collinear(u, v, q)) // on one line: do they overlap
    return between(u, v, p) || between(u, v, q) || between(p, q, u) || between(p, q, v);
  // Coplanar, not on one line: the textbook test in a projection where the plane is not edge-on
  // (there 2D orientations are exact and collinearity is 3D collinearity).
  int k = 0;
  for (; k < 3; ++k)
    if (orient2_drop(u, v, p, k) != 0 || orient2_drop(u, v, q, k) != 0)
      break;
  if (k == 3)
    return false;
  const int o1 = orient2_drop(u, v, p, k), o2 = orient2_drop(u, v, q, k);
  const int o3 = orient2_drop(p, q, u, k), o4 = orient2_drop(p, q, v, k);
  if (o1 != o2 && o3 != o4)
    return true;
  return (o1 == 0 && between(u, v, p)) || (o2 == 0 && between(u, v, q)) ||
         (o3 == 0 && between(p, q, u)) || (o4 == 0 && between(p, q, v));
}
} // namespace

bool do_intersect(const Triangle_3 &t, const Segment &s) {
  const Point &a = t.p[0], &b = t.p[1], &c = t.p[2], &p = s.s, &q = s.t;
  if (collinear(a, b, c)) {
    // A degenerate triangle is the point or segment it spans (the legacy code passed these to
    // CGAL::do_intersect, whose precondition excludes them; geometrically this is the answer).
    // Its span: the two of a, b, c farthest apart along their line.
    const Point *u = &a, *v = &b;
    if (a == b) {
      v = &c;
    } else {
      const int k = line_axis(a, b);
      const Point *lo = &a, *hi = &a;
      for (const Point *x : {&a, &b, &c}) {
        if (x->c[k] < lo->c[k])
          lo = x;
        if (x->c[k] > hi->c[k])
          hi = x;
      }
      u = lo;
      v = hi;
    }
    return segments_meet(*u, *v, p, q);
  }
  const int op = orient3(a, b, c, p), oq = orient3(a, b, c, q);
  if (op != 0 && op == oq)
    return false; // both ends strictly on one side
  if (op == 0 && oq == 0) {
    // Coplanar: project along the axis the plane is most transverse to, where orientation
    // signs are exact on the input's own coordinates, and test in 2D: an end inside the
    // triangle, or the segment crossing an edge.
    int k = 0;
    for (int axis = 0; axis < 3; ++axis)
      if (orient2_drop(a, b, c, axis) != 0) {
        k = axis;
        break;
      }
    const int ot = orient2_drop(a, b, c, k);
    auto inside = [&](const Point &x) {
      return orient2_drop(a, b, x, k) * ot >= 0 && orient2_drop(b, c, x, k) * ot >= 0 &&
             orient2_drop(c, a, x, k) * ot >= 0;
    };
    if (inside(p) || inside(q))
      return true;
    auto crosses = [&](const Point &u, const Point &v) { // segment pq against segment uv
      const int o1 = orient2_drop(p, q, u, k), o2 = orient2_drop(p, q, v, k);
      const int o3 = orient2_drop(u, v, p, k), o4 = orient2_drop(u, v, q, k);
      return o1 * o2 <= 0 && o3 * o4 <= 0 && !(o1 == 0 && o2 == 0);
    };
    return crosses(a, b) || crosses(b, c) || crosses(c, a);
  }
  // The segment meets the plane (an end may lie on it): it meets the triangle exactly when its
  // line passes through the closed triangle, i.e. the three orientations do not disagree.
  const int s1 = orient3(p, q, a, b), s2 = orient3(p, q, b, c), s3 = orient3(p, q, c, a);
  const bool has_pos = s1 > 0 || s2 > 0 || s3 > 0, has_neg = s1 < 0 || s2 < 0 || s3 < 0;
  return !(has_pos && has_neg);
}

namespace {
// CGAL's line_from_pointsC2.
void line_from_points(const double p[2], const double q[2], double &a, double &b, double &c) {
  const double px = p[0], py = p[1], qx = q[0], qy = q[1];
  if (py == qy) {
    a = 0;
    if (qx > px) {
      b = 1;
      c = -py;
    } else if (qx == px) {
      b = 0;
      c = 0;
    } else {
      b = -1;
      c = py;
    }
  } else if (qx == px) {
    b = 0;
    if (qy > py) {
      a = -1;
      c = px;
    } else if (qy == py) {
      a = 0;
      c = 0;
    } else {
      a = 1;
      c = -px;
    }
  } else {
    a = py - qy;
    b = qx - px;
    c = -px * a - py * b;
  }
}
int compare(double x, double y) { return (x > y) - (x < y); }
} // namespace

bool segment_2_has_on(const double s[2], const double t[2], const double p[2]) {
  const double o[2][2] = {{t[0] - s[0], t[1] - s[1]}, {p[0] - s[0], p[1] - s[1]}};
  if (o[0][0] * o[1][1] - o[0][1] * o[1][0] != 0)
    return false; // not collinear (exact enough here: the segment lies on the x axis)
  // collinear_are_ordered_along_line(s, p, t)
  if (s[0] < p[0])
    return p[0] <= t[0];
  if (p[0] < s[0])
    return t[0] <= p[0];
  if (s[1] < p[1])
    return p[1] <= t[1];
  if (p[1] < s[1])
    return t[1] <= p[1];
  return true;
}

std::optional<std::array<double, 2>> ray_segment_point(const double rs[2], const double rt[2],
                                                       const double ss[2], const double st[2]) {
  double a1, b1, c1, a2, b2, c2;
  line_from_points(rs, rt, a1, b1, c1);
  line_from_points(ss, st, a2, b2, c2);
  const double denom = a1 * b2 - a2 * b1;
  if (denom == 0)
    return std::nullopt; // parallel, or collinear (an overlap is not a point)
  const double nom1 = b1 * c2 - b2 * c1, nom2 = a2 * c1 - a1 * c2;
  if (!std::isfinite(nom1) || !std::isfinite(nom2))
    return std::nullopt;
  const std::array<double, 2> x{nom1 / denom, nom2 / denom};
  if (!std::isfinite(x[0]) || !std::isfinite(x[1]))
    return std::nullopt;
  // Ray_2::collinear_has_on(x)
  bool on_ray;
  switch (compare(rs[0], rt[0])) {
  case -1:
    on_ray = compare(rs[0], x[0]) != 1;
    break;
  case 1:
    on_ray = compare(x[0], rs[0]) != 1;
    break;
  default:
    switch (compare(rs[1], rt[1])) {
    case -1:
      on_ray = compare(rs[1], x[1]) != 1;
      break;
    case 1:
      on_ray = compare(x[1], rs[1]) != 1;
      break;
    default:
      on_ray = true;
    }
  }
  if (!on_ray)
    return std::nullopt;
  // Segment_2::collinear_has_on(x): ordered along the line.
  bool on_seg;
  if (ss[0] != st[0])
    on_seg = (ss[0] <= x[0] && x[0] <= st[0]) || (st[0] <= x[0] && x[0] <= ss[0]);
  else
    on_seg = (ss[1] <= x[1] && x[1] <= st[1]) || (st[1] <= x[1] && x[1] <= ss[1]);
  if (!on_seg)
    return std::nullopt;
  return x;
}

} // namespace cg

// ============================================================================================
// Triangulation

void Triangulation::reset(tet_complex t) {
  _t = std::move(t);
  _points.clear();
  _vertices.clear();
  _cells.clear();
  _points.reserve(_t.points().size() + 1); // and the infinite vertex's; never reallocates
  _vertices.reserve(_t.points().size() + 1);
  _cells.reserve(_t.cells().size());
  _inf = _v_begin = _v_end = nullptr;
  _c_begin = _c_finite_end = _c_end = nullptr;
  _fill_at = 0;
  _filled = false;
}

// One unit: a vertex (with its point), then a cell, then the vertices' incident cells.
bool Triangulation::fill_one() {
  if (_filled)
    return true;
  const std::size_t n = _t.points().size(), nc = _t.cells().size();
  if (_fill_at < n) {
    _points.emplace_back(_t.points()[_fill_at]);
    VC_vertex v;
    v.tc = int(_fill_at);
    if (_t.is_vertex(int(_fill_at))) {
      v.pt = &_points.back();
      v.v_pole = _points.back();
    }
    _vertices.push_back(std::move(v));
  } else if (_fill_at == n) {
    // The infinite vertex. CGAL's had a default-constructed point; this one has the origin, so
    // that a stray read is harmless, and is_infinite() is how the code tells it apart.
    _points.emplace_back(0.0, 0.0, 0.0);
    VC_vertex v;
    v.tc = int(n);
    v.pt = &_points.back();
    _vertices.push_back(std::move(v));
    _inf = &_vertices.back();
    _v_begin = _vertices.data();
    _v_end = _vertices.data() + n;
  } else if (_fill_at < n + 1 + nc) {
    const std::size_t i = _fill_at - n - 1;
    const tet_complex::cell &k = _t.cells()[i];
    VC_cell c;
    c.tc = int(i);
    VC_cell *base = _cells.data(); // reserved: never reallocates
    for (int j = 0; j < 4; ++j) {
      c.V[j] = &_vertices[std::size_t(k.v[std::size_t(j)])];
      c.N[j] = base + k.n[std::size_t(j)];
    }
    _cells.push_back(c);
  } else {
    const std::size_t v = _fill_at - n - 1 - nc;
    if (v <= n) {
      const int ic = _t.incident_cell(int(v));
      if (ic >= 0)
        _vertices[v].c = &_cells[std::size_t(ic)];
    }
    if (v >= n) {
      _c_begin = _cells.data();
      _c_finite_end = _cells.data() + _t.num_finite_cells();
      _c_end = _cells.data() + _cells.size();
      _filled = true;
      return true;
    }
  }
  ++_fill_at;
  return false;
}

cg::Bounded_side Triangulation::side_of_sphere(Cell_handle c, const Point &p) const {
  return cg::Bounded_side(_t.side_of_sphere(c->tc, {p.c[0], p.c[1], p.c[2]}));
}

} // namespace cvc::cocone
