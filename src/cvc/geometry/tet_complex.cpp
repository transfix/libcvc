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

// cvc::tet_complex and its builder (cvc/geometry/tet_complex.h). The predicates and their
// perturbation follow CGAL's Delaunay_triangulation_3 (side_of_sphere, side_of_oriented_sphere,
// coplanar_side_of_bounded_circle with perturb = true), so that the triangulation is the one CGAL
// builds; the exact signs come from Shewchuk's predicates instead of CGAL's filtered kernel.

#include "exact_predicates.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cvc/geometry/tet_complex.h>
#include <limits>
#include <unordered_set>
#include <utility>

namespace cvc {

namespace {

using point = tet_complex::point;
using cell = tet_complex::cell;

// CGAL's next_around_edge (Triangulation_utils_3): for an edge through a cell's vertices i and j,
// the index k of the facet to cross to reach the next cell around the edge, in the order of
// CGAL's Cell_circulator.
constexpr int next_around_edge_table[4][4] = {
    {5, 2, 3, 1}, {3, 5, 0, 2}, {1, 3, 5, 0}, {2, 0, 1, 5}};
inline int next_around_edge(int i, int j) { return next_around_edge_table[i][j]; }

std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

// CGAL's compare_xyz(p, q) == SMALLER: the order the perturbation ranks points in.
inline bool lex_less(const point &p, const point &q) {
  if (p[0] != q[0])
    return p[0] < q[0];
  if (p[1] != q[1])
    return p[1] < q[1];
  return p[2] < q[2];
}

inline bool same_point(const point &p, const point &q) {
  return p[0] == q[0] && p[1] == q[1] && p[2] == q[2];
}

#ifdef CVC_WITH_SHEWCHUK

// ---- predicates, with CGAL's signs ----

inline int sgn(double v) { return (v > 0.0) - (v < 0.0); }

// CGAL::orientation(a, b, c, d): POSITIVE (+1) when d is on the side of plane abc from which a, b,
// c appear counterclockwise... which is the opposite of Shewchuk's orient3d sign.
inline int orientation(const point &a, const point &b, const point &c, const point &d) {
  return -sgn(exact::orient3d(a.data(), b.data(), c.data(), d.data()));
}

// CGAL's side_of_oriented_sphere(a, b, c, d, e) for orientation(a, b, c, d) == POSITIVE: +1 when e
// is inside the sphere.
inline int side_of_oriented_sphere(const point &a, const point &b, const point &c, const point &d,
                                   const point &e) {
  return -sgn(exact::insphere(a.data(), b.data(), c.data(), d.data(), e.data()));
}

// 2D orientation of three points in the xy, yz or xz plane (Shewchuk's orient2d is exact on the
// projected coordinates, which are the input's own doubles).
inline int orient_xy(const point &p, const point &q, const point &r) {
  const double a[2] = {p[0], p[1]}, b[2] = {q[0], q[1]}, c[2] = {r[0], r[1]};
  return sgn(exact::orient2d(a, b, c));
}
inline int orient_yz(const point &p, const point &q, const point &r) {
  const double a[2] = {p[1], p[2]}, b[2] = {q[1], q[2]}, c[2] = {r[1], r[2]};
  return sgn(exact::orient2d(a, b, c));
}
inline int orient_xz(const point &p, const point &q, const point &r) {
  const double a[2] = {p[0], p[2]}, b[2] = {q[0], q[2]}, c[2] = {r[0], r[2]};
  return sgn(exact::orient2d(a, b, c));
}

// CGAL's coplanar_orientation(p, q, r): the orientation of three coplanar points, in the first
// projection (xy, yz, xz) where they are not collinear; 0 when they are collinear.
inline int coplanar_orientation(const point &p, const point &q, const point &r) {
  int o = orient_xy(p, q, r);
  if (o != 0)
    return o;
  o = orient_yz(p, q, r);
  if (o != 0)
    return o;
  return orient_xz(p, q, r);
}

inline bool collinear(const point &p, const point &q, const point &r) {
  return orient_xy(p, q, r) == 0 && orient_yz(p, q, r) == 0 && orient_xz(p, q, r) == 0;
}

// CGAL's coplanar_side_of_bounded_circle(p, q, r, t) for four coplanar points, p, q, r not
// collinear: +1 when t is inside the circle through p, q, r, 0 on it, -1 outside. Exactly: the
// sphere through p, q, r and any point s off their plane meets the plane in that circle, so t
// (in the plane) is inside the circle exactly when it is inside the sphere. s is p moved along
// the axis most normal to the plane; whatever s rounds to, it serves once orient3d says it is
// off the plane.
int coplanar_side_of_bounded_circle(const point &p, const point &q, const point &r,
                                    const point &t) {
  const double u[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
  const double v[3] = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
  const double n[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                       u[0] * v[1] - u[1] * v[0]};
  int axes[3] = {0, 1, 2};
  std::sort(axes, axes + 3, [&](int a, int b) { return std::fabs(n[a]) > std::fabs(n[b]); });
  for (int k : axes) {
    point s = p;
    s[std::size_t(k)] += 1.0 + std::fabs(p[std::size_t(k)]);
    const double o = exact::orient3d(p.data(), q.data(), r.data(), s.data());
    if (o == 0.0)
      continue;
    return sgn(o) * sgn(exact::insphere(p.data(), q.data(), r.data(), s.data(), t.data()));
  }
  return 0; // unreachable for non-collinear p, q, r
}

#endif // CVC_WITH_SHEWCHUK

// splitmix64: the seeded hash behind the insertion rounds and the walk's facet choice.
inline std::uint64_t mix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

inline std::uint64_t spread3(std::uint64_t v) { // 19 bits -> every third bit of 57
  v &= 0x7ffff;
  v = (v | v << 32) & 0x1f00000000ffffULL;
  v = (v | v << 16) & 0x1f0000ff0000ffULL;
  v = (v | v << 8) & 0x100f00f00f00f00fULL;
  v = (v | v << 4) & 0x10c30c30c30c30c3ULL;
  v = (v | v << 2) & 0x1249249249249249ULL;
  return v;
}

} // namespace

// ============================================================================================
// tet_complex

int tet_complex::index(int c, int v) const {
  const cell &k = _cells[std::size_t(c)];
  for (int i = 0; i < 4; ++i)
    if (k.v[std::size_t(i)] == v)
      return i;
  return -1;
}

int tet_complex::neighbor_index(int c, int n) const {
  const cell &k = _cells[std::size_t(c)];
  for (int i = 0; i < 4; ++i)
    if (k.n[std::size_t(i)] == n)
      return i;
  return -1;
}

bool tet_complex::is_infinite_facet(int c, int i) const {
  const int inf = index(c, infinite_vertex());
  return inf >= 0 && inf != i;
}

bool tet_complex::is_infinite_edge(int c, int i, int j) const {
  return vertex(c, i) == infinite_vertex() || vertex(c, j) == infinite_vertex();
}

void tet_complex::incident_cells(int v, std::vector<int> &out) const {
  out.clear();
  const int start = _vertex_cell[std::size_t(v)];
  if (start < 0)
    return;
  // The out vector doubles as the visited set while it is small.
  std::unordered_set<int> seen;
  bool use_set = false;
  auto visited = [&](int c) {
    if (use_set)
      return seen.count(c) > 0;
    return std::find(out.begin(), out.end(), c) != out.end();
  };
  out.push_back(start);
  for (std::size_t k = 0; k < out.size(); ++k) {
    const int c = out[k];
    const int iv = index(c, v);
    for (int i = 0; i < 4; ++i) {
      if (i == iv)
        continue;
      const int n = neighbor(c, i);
      if (visited(n))
        continue;
      out.push_back(n);
      if (use_set) {
        seen.insert(n);
      } else if (out.size() > 64) {
        use_set = true;
        seen.insert(out.begin(), out.end());
      }
    }
  }
}

void tet_complex::cells_around_edge(int c, int i, int j, std::vector<int> &out) const {
  out.clear();
  const int s = vertex(c, i), t = vertex(c, j);
  int cur = c;
  do {
    out.push_back(cur);
    cur = neighbor(cur, next_around_edge(index(cur, s), index(cur, t)));
  } while (cur != c && out.size() <= _cells.size());
}

bool tet_complex::on_convex_hull(int v) const {
  std::vector<int> cs;
  incident_cells(v, cs);
  for (int c : cs)
    if (is_infinite_cell(c))
      return true;
  return false;
}

tet_complex::point tet_complex::compute_circumcentre(int c, double *squared_radius) const {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (is_infinite_cell(c)) {
    if (squared_radius)
      *squared_radius = nan;
    return {nan, nan, nan};
  }
  // CGAL's circumcenterC3 / circumcenter_translateC3 (and its 3x3 determinant), term for term,
  // so a Voronoi vertex here rounds as the legacy code's triang.dual(c) did.
  const point &p = point_of(vertex(c, 0)), &q = point_of(vertex(c, 1)), &r = point_of(vertex(c, 2)),
              &s = point_of(vertex(c, 3));
  const double dqx = q[0] - p[0], dqy = q[1] - p[1], dqz = q[2] - p[2];
  const double drx = r[0] - p[0], dry = r[1] - p[1], drz = r[2] - p[2];
  const double dsx = s[0] - p[0], dsy = s[1] - p[1], dsz = s[2] - p[2];
  auto det3 = [](double a00, double a01, double a02, double a10, double a11, double a12, double a20,
                 double a21, double a22) {
    const double m01 = a00 * a11 - a10 * a01;
    const double m02 = a00 * a21 - a20 * a01;
    const double m12 = a10 * a21 - a20 * a11;
    return m01 * a22 - m02 * a12 + m12 * a02;
  };
  const double r2 = drx * drx + dry * dry + drz * drz;
  const double q2 = dqx * dqx + dqy * dqy + dqz * dqz;
  const double s2 = dsx * dsx + dsy * dsy + dsz * dsz;
  const double num_x = det3(dqy, dqz, q2, dry, drz, r2, dsy, dsz, s2);
  const double num_y = det3(dqx, dqz, q2, drx, drz, r2, dsx, dsz, s2);
  const double num_z = det3(dqx, dqy, q2, drx, dry, r2, dsx, dsy, s2);
  const double den = det3(dqx, dqy, dqz, drx, dry, drz, dsx, dsy, dsz);
  const double inv = 1.0 / (2.0 * den);
  const double dx = num_x * inv, dy = -num_y * inv, dz = num_z * inv;
  if (squared_radius)
    *squared_radius = dx * dx + dy * dy + dz * dz;
  return {p[0] + dx, p[1] + dy, p[2] + dz};
}

std::vector<std::array<int, 4>> tet_complex::finite_tetrahedra() const {
  std::vector<std::array<int, 4>> out;
  out.reserve(std::size_t(_num_finite_cells));
  for (int c = 0; c < num_cells(); ++c) {
    if (is_infinite_cell(c))
      continue;
    std::array<int, 4> t = _cells[std::size_t(c)].v;
    std::sort(t.begin(), t.end());
    out.push_back(t);
  }
  std::sort(out.begin(), out.end());
  return out;
}

#ifdef CVC_WITH_SHEWCHUK
namespace {

// The perturbed predicates over a point array, with vertex ids: the perturbation ranks points
// by their coordinates, and needs to know which one is the query.
struct predicates {
  const point *pts;
  std::size_t *perturbed = nullptr; // counts ties the perturbation broke

  // CGAL's side_of_oriented_sphere(p0, p1, p2, p3, p, perturb = true), for a positively oriented
  // cell: +1 inside, -1 outside, never 0 (p distinct from the cell's vertices).
  int sphere(int v0, int v1, int v2, int v3, int q) const {
    const point &p0 = pts[v0], &p1 = pts[v1], &p2 = pts[v2], &p3 = pts[v3], &p = pts[q];
    const int os = side_of_oriented_sphere(p0, p1, p2, p3, p);
    if (os != 0)
      return os;
    if (perturbed)
      ++*perturbed;
    int ids[5] = {v0, v1, v2, v3, q};
    std::sort(ids, ids + 5, [&](int a, int b) { return lex_less(pts[a], pts[b]); });
    // The leading monomials of the perturbed determinant; two are enough (Devillers-Teillaud).
    for (int i = 4; i > 2; --i) {
      if (ids[i] == q)
        return -1; // p0..p3 are not coplanar and positively oriented
      int o = 0;
      if (ids[i] == v3 && (o = orientation(p0, p1, p2, p)) != 0)
        return o;
      if (ids[i] == v2 && (o = orientation(p0, p1, p, p3)) != 0)
        return o;
      if (ids[i] == v1 && (o = orientation(p0, p, p2, p3)) != 0)
        return o;
      if (ids[i] == v0 && (o = orientation(p, p1, p2, p3)) != 0)
        return o;
    }
    return -1;
  }

  // CGAL's coplanar_side_of_bounded_circle(p0, p1, p2, p, perturb = true): +1 inside, -1 outside.
  int circle(int v0, int v1, int v2, int q) const {
    const point &p0 = pts[v0], &p1 = pts[v1], &p2 = pts[v2], &p = pts[q];
    const int bs = coplanar_side_of_bounded_circle(p0, p1, p2, p);
    if (bs != 0)
      return bs;
    if (perturbed)
      ++*perturbed;
    int ids[4] = {v0, v1, v2, q};
    std::sort(ids, ids + 4, [&](int a, int b) { return lex_less(pts[a], pts[b]); });
    const int local = coplanar_orientation(p0, p1, p2);
    for (int i = 3; i > 0; --i) {
      if (ids[i] == q)
        return -1;
      int o = 0;
      if (ids[i] == v2 && (o = coplanar_orientation(p0, p1, p)) != 0)
        return o * local;
      if (ids[i] == v1 && (o = coplanar_orientation(p0, p, p2)) != 0)
        return o * local;
      if (ids[i] == v0 && (o = coplanar_orientation(p, p1, p2)) != 0)
        return o * local;
    }
    return -local;
  }

  // CGAL's side_of_sphere(c, p, perturb = true) == ON_BOUNDED_SIDE: whether q is in conflict
  // with cell k (inf: the infinite vertex's id).
  bool conflict(const cell &k, int inf, int q) const {
    int i3 = -1;
    for (int i = 0; i < 4; ++i)
      if (k.v[std::size_t(i)] == inf)
        i3 = i;
    if (i3 < 0)
      return sphere(k.v[0], k.v[1], k.v[2], k.v[3], q) > 0;
    int i0, i1, i2;
    if (i3 % 2 == 1) {
      i0 = (i3 + 1) & 3;
      i1 = (i3 + 2) & 3;
      i2 = (i3 + 3) & 3;
    } else {
      i0 = (i3 + 2) & 3;
      i1 = (i3 + 1) & 3;
      i2 = (i3 + 3) & 3;
    }
    const int a = k.v[std::size_t(i0)], b = k.v[std::size_t(i1)], c = k.v[std::size_t(i2)];
    const int o = orientation(pts[a], pts[b], pts[c], pts[q]);
    if (o != 0)
      return o > 0;
    return circle(a, b, c, q) > 0;
  }
};

} // namespace
#endif // CVC_WITH_SHEWCHUK

int tet_complex::side_of_sphere(int c, const point &p) const {
#ifdef CVC_WITH_SHEWCHUK
  const cell &k = _cells[std::size_t(c)];
  const int i3 = index(c, infinite_vertex());
  if (i3 < 0)
    return side_of_oriented_sphere(point_of(k.v[0]), point_of(k.v[1]), point_of(k.v[2]),
                                   point_of(k.v[3]), p);
  int i0, i1, i2; // CGAL's order for the finite facet
  if (i3 % 2 == 1) {
    i0 = (i3 + 1) & 3;
    i1 = (i3 + 2) & 3;
    i2 = (i3 + 3) & 3;
  } else {
    i0 = (i3 + 2) & 3;
    i1 = (i3 + 1) & 3;
    i2 = (i3 + 3) & 3;
  }
  const point &a = point_of(k.v[std::size_t(i0)]), &b = point_of(k.v[std::size_t(i1)]),
              &d = point_of(k.v[std::size_t(i2)]);
  const int o = orientation(a, b, d, p);
  if (o != 0)
    return o;
  return coplanar_side_of_bounded_circle(a, b, d, p);
#else
  (void)c;
  (void)p;
  throw tet_complex_error("tet_complex: libcvc was built without Shewchuk's predicates");
#endif
}

std::vector<std::string> tet_complex::validate(bool delaunay, std::size_t max_problems) const {
  std::vector<std::string> out;
  auto problem = [&](std::string s) {
    if (out.size() < max_problems)
      out.push_back(std::move(s));
  };
  const int inf = infinite_vertex();
  const int nc = num_cells();
  int finite = 0;
  for (int c = 0; c < nc && out.size() < max_problems; ++c) {
    const cell &k = _cells[std::size_t(c)];
    for (int i = 0; i < 4; ++i) {
      const int v = k.v[std::size_t(i)];
      if (v < 0 || v > inf) {
        problem(fmt("cell %d: vertex %d out of range", c, v));
        continue;
      }
      if (v != inf && !is_vertex(v))
        problem(fmt("cell %d: point %d is a duplicate, not a vertex", c, v));
      for (int j = i + 1; j < 4; ++j)
        if (k.v[std::size_t(j)] == v)
          problem(fmt("cell %d: vertex %d twice", c, v));
      const int n = k.n[std::size_t(i)];
      if (n < 0 || n >= nc) {
        problem(fmt("cell %d: neighbour %d out of range", c, n));
        continue;
      }
      const int m = neighbor_index(n, c);
      if (m < 0) {
        problem(fmt("cell %d: neighbour %d does not point back", c, n));
        continue;
      }
      for (int j = 0; j < 4; ++j)
        if (j != i && index(n, k.v[std::size_t(j)]) < 0)
          problem(
              fmt("cell %d facet %d: neighbour %d lacks vertex %d", c, i, n, k.v[std::size_t(j)]));
      if (index(n, v) >= 0)
        problem(fmt("cell %d facet %d: neighbour %d shares the opposite vertex", c, i, n));
    }
    finite += !is_infinite_cell(c);
  }
  if (finite != _num_finite_cells)
    problem(fmt("%d finite cells counted, %d recorded", finite, _num_finite_cells));
  // Euler: the closed complex is a triangulated 3-sphere, so V - E + F - C = 0 with F = 2C.
  {
    std::vector<std::uint64_t> edges;
    edges.reserve(std::size_t(nc) * 6);
    for (const cell &k : _cells)
      for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) {
          const auto a = std::uint32_t(std::min(k.v[std::size_t(i)], k.v[std::size_t(j)]));
          const auto b = std::uint32_t(std::max(k.v[std::size_t(i)], k.v[std::size_t(j)]));
          edges.push_back(std::uint64_t(a) << 32 | b);
        }
    std::sort(edges.begin(), edges.end());
    const auto ne = std::size_t(std::unique(edges.begin(), edges.end()) - edges.begin());
    const long long chi = (long long)(_num_vertices + 1) - (long long)ne + 2LL * nc - nc;
    if (chi != 0)
      problem(fmt("Euler characteristic %lld (V %d + 1, E %zu, C %d), not 0", chi, _num_vertices,
                  ne, nc));
  }
  int nv = 0;
  for (int v = 0; v < num_points(); ++v) {
    const int d = duplicate_of(v);
    if (d < 0 || d > v || !is_vertex(d) || !same_point(point_of(d), point_of(v)))
      problem(fmt("point %d: duplicate_of %d", v, d));
    if (d != v)
      continue;
    ++nv;
    const int c = incident_cell(v);
    if (c < 0 || c >= nc || index(c, v) < 0)
      problem(fmt("vertex %d: incident cell %d lacks it", v, c));
  }
  if (nv != _num_vertices)
    problem(fmt("%d vertices counted, %d recorded", nv, _num_vertices));
  if (nc > 0 && (incident_cell(inf) < 0 || index(incident_cell(inf), inf) < 0))
    problem("the infinite vertex's incident cell lacks it");
  if (has_circumcentres() == false && !_circumcentres.empty())
    problem("circumcentres cached for some cells only");
#ifdef CVC_WITH_SHEWCHUK
  if (!out.empty())
    return out;
  // Orientation: finite cells positive; an infinite cell turns negative when the vertex across
  // its finite facet (inside the hull) replaces the infinite vertex.
  for (int c = 0; c < nc && out.size() < max_problems; ++c) {
    const cell &k = _cells[std::size_t(c)];
    const int i3 = index(c, inf);
    std::array<point, 4> p;
    for (int i = 0; i < 4; ++i)
      p[std::size_t(i)] = i == i3 ? point{} : point_of(k.v[std::size_t(i)]);
    if (i3 < 0) {
      if (orientation(p[0], p[1], p[2], p[3]) <= 0)
        problem(fmt("cell %d is not positively oriented", c));
    } else {
      const int m = mirror_vertex(c, i3);
      if (m == inf) {
        problem(fmt("infinite cell %d's finite facet has an infinite cell behind it", c));
        continue;
      }
      p[std::size_t(i3)] = point_of(m);
      if (orientation(p[0], p[1], p[2], p[3]) >= 0)
        problem(fmt("infinite cell %d faces into the hull", c));
    }
  }
  if (!delaunay || !out.empty())
    return out;
  // Delaunay: no vertex in conflict with any cell, finite (empty circumsphere) or infinite
  // (every vertex on the inner side of each hull facet, or inside the hull facet's plane outside
  // its circumcircle), with the perturbation breaking ties as the build does.
  const predicates pr{_points.data()};
  for (int c = 0; c < nc && out.size() < max_problems; ++c) {
    const cell &k = _cells[std::size_t(c)];
    for (int v = 0; v < num_points(); ++v) {
      if (!is_vertex(v) || index(c, v) >= 0)
        continue;
      if (pr.conflict(k, inf, v)) {
        problem(fmt("%s cell %d (%d %d %d %d) is in conflict with vertex %d",
                    is_infinite_cell(c) ? "infinite" : "finite", c, k.v[0], k.v[1], k.v[2], k.v[3],
                    v));
        break;
      }
    }
  }
#else
  (void)delaunay;
#endif
  return out;
}

// ============================================================================================
// tet_complex_builder

bool tet_complex_available() {
#ifdef CVC_WITH_SHEWCHUK
  return true;
#else
  return false;
#endif
}

namespace {
enum class stage {
  checking,
  keying,
  sorting_count,
  sorting_move,
  seeding,
  inserting,
  compacting_map,
  compacting_move,
  compacting_points,
  circumcentres,
  done,
  failed
};
constexpr int radix_bits = 11;
constexpr int radix_passes = 6; // 66 bits >= the 61-bit keys (4 round bits, 57 Morton bits)
constexpr std::size_t radix_bins = std::size_t(1) << radix_bits;
} // namespace

struct tet_complex_builder::impl {
  std::shared_ptr<const std::vector<std::array<double, 3>>> input;
  tet_complex_options opt;
  std::atomic<bool> cancel{false};
  stage st = stage::checking;
  std::exception_ptr error;
  tet_complex_stats stats;
  double prog = 0.0;

  tet_complex out;
  std::size_t n = 0;  // points
  std::size_t at = 0; // the position within the current pass

  // checking / ordering. The radix sort's arrays are left uninitialised (new T[n] of a trivial
  // type touches no page), so no phase change costs O(n): every array is written in a counted
  // loop.
  point lo{}, hi{};
  int max_round = 0;
  std::unique_ptr<std::uint64_t[]> keys, keys2;
  std::unique_ptr<int[]> order, order2;
  int pass = 0;
  std::array<std::size_t, radix_bins> bins{};

  // seeding: the first four points not in one plane
  int seed_found = 0;
  std::array<int, 4> seed{};
  std::vector<char> seeded; // per point, whether it is one of the four

  // inserting
  std::vector<cell> &cells = out._cells;
  std::vector<int> free_cells;
  std::vector<std::uint32_t> visit; // per cell: 2 * epoch (+1 when in conflict)
  std::uint32_t epoch = 0;
  int hint = 0;
  int live_finite = 0;
  std::uint64_t walk_rng = 0;
  std::vector<int> conflict, stack;
  struct boundary_facet {
    int c, i;
  };
  std::vector<boundary_facet> boundary;
  std::vector<int> created;
  struct edge_slot {
    std::uint64_t key;
    int cell, index;
    std::uint32_t stamp;
  };
  std::vector<edge_slot> edge_table;
  std::uint32_t edge_stamp = 0;

  // compacting
  std::vector<int> remap;
  std::vector<cell> compact_target;
  int next_finite = 0, next_infinite = 0, move_pass = 0;

#ifdef CVC_WITH_SHEWCHUK
  predicates pr() { return predicates{out._points.data(), &stats.perturbed}; }
#endif

  const point &P(int v) const { return out._points[std::size_t(v)]; }
  int inf() const { return int(n); }

  void fail_input(const std::string &what) { throw tet_complex_error("tet_complex: " + what); }

  bool step(std::size_t budget);
  void do_check(std::size_t &units, std::size_t budget);
  void do_key(std::size_t &units, std::size_t budget);
  void do_sort_count(std::size_t &units, std::size_t budget);
  void do_sort_move(std::size_t &units, std::size_t budget);
#ifdef CVC_WITH_SHEWCHUK
  void do_seed(std::size_t &units, std::size_t budget);
  void do_insert(std::size_t &units, std::size_t budget);
  void make_first_cells();
  int new_cell(const cell &k);
  void free_cell(int c);
  std::size_t insert(int q); // returns the cells it touched
  int locate(int q, std::size_t &touched, int &duplicate);
  void rename_vertex(int from, int to);
#endif
  void do_compact_map(std::size_t &units, std::size_t budget);
  void do_compact_move(std::size_t &units, std::size_t budget);
  void do_compact_points(std::size_t &units, std::size_t budget);
  void do_circumcentres(std::size_t &units, std::size_t budget);
  void update_progress();
};

namespace {
inline bool out_of_budget(std::size_t units, std::size_t budget) {
  return budget != 0 && units >= budget;
}
} // namespace

bool tet_complex_builder::impl::step(std::size_t budget) {
  std::size_t units = 0;
  while (st != stage::done && !out_of_budget(units, budget)) {
    if (cancel.load(std::memory_order_acquire))
      return true; // the caller turns this into "cancelled"
    switch (st) {
    case stage::checking:
      do_check(units, budget);
      break;
    case stage::keying:
      do_key(units, budget);
      break;
    case stage::sorting_count:
      do_sort_count(units, budget);
      break;
    case stage::sorting_move:
      do_sort_move(units, budget);
      break;
#ifdef CVC_WITH_SHEWCHUK
    case stage::seeding:
      do_seed(units, budget);
      break;
    case stage::inserting:
      do_insert(units, budget);
      break;
#else
    case stage::seeding:
    case stage::inserting:
      break;
#endif
    case stage::compacting_map:
      do_compact_map(units, budget);
      break;
    case stage::compacting_move:
      do_compact_move(units, budget);
      break;
    case stage::compacting_points:
      do_compact_points(units, budget);
      break;
    case stage::circumcentres:
      do_circumcentres(units, budget);
      break;
    case stage::done:
    case stage::failed:
      break;
    }
  }
  update_progress();
  return st == stage::done;
}

// ---- checking: every coordinate finite, the bounding box ----
void tet_complex_builder::impl::do_check(std::size_t &units, std::size_t budget) {
  if (at == 0) {
#ifndef CVC_WITH_SHEWCHUK
    fail_input("libcvc was built without Shewchuk's predicates (CVC_WITH_SHEWCHUK), so it cannot "
               "build a Delaunay tetrahedralization");
#endif
    n = input->size();
    if (n < 4)
      fail_input(
          fmt("%zu points; a tetrahedral complex needs at least 4 points not in one plane", n));
    if (n >= std::size_t(std::numeric_limits<int>::max() / 8))
      fail_input(fmt("%zu points is more than this build can index", n));
    lo = {HUGE_VAL, HUGE_VAL, HUGE_VAL};
    hi = {-HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
  }
  const auto &pts = *input;
  for (; at < n && !out_of_budget(units, budget); ++at, ++units) {
    const auto &p = pts[at];
    for (int k = 0; k < 3; ++k) {
      if (!std::isfinite(p[std::size_t(k)]))
        fail_input(fmt("point %zu has a coordinate that is not finite (%g, %g, %g)", at, p[0], p[1],
                       p[2]));
      lo[std::size_t(k)] = std::min(lo[std::size_t(k)], p[std::size_t(k)]);
      hi[std::size_t(k)] = std::max(hi[std::size_t(k)], p[std::size_t(k)]);
    }
  }
  if (at == n) {
    at = 0;
    st = stage::keying;
    keys.reset(new std::uint64_t[n]);
    order.reset(new int[n]);
    // Filled point by point while keying (reserve only allocates).
    out._points.reserve(n);
    out._duplicate_of.reserve(n);
    out._vertex_cell.reserve(n + 1);
    seeded.reserve(n);
    // BRIO rounds: about 64 points in the first, then doubling, half of them in the last.
    max_round = 0;
    while ((n >> (max_round + 1)) >= 64 && max_round < 15)
      ++max_round;
  }
}

// ---- ordering: a key per point (BRIO round, then Morton code), then an LSD radix sort ----
void tet_complex_builder::impl::do_key(std::size_t &units, std::size_t budget) {
  const auto &pts = *input;
  double scale[3];
  for (int k = 0; k < 3; ++k) {
    const double span = hi[std::size_t(k)] - lo[std::size_t(k)];
    scale[k] = span > 0 ? 524287.0 / span : 0.0;
  }
  for (; at < n && !out_of_budget(units, budget); ++at, ++units) {
    std::uint64_t code = 0;
    for (int k = 0; k < 3; ++k) {
      double q = (pts[at][std::size_t(k)] - lo[std::size_t(k)]) * scale[k];
      q = std::min(std::max(q, 0.0), 524287.0);
      code |= spread3(std::uint64_t(q)) << k;
    }
    const std::uint64_t h = mix64(opt.seed ^ (std::uint64_t(at) * 0x2545f4914f6cdd1dULL));
    const int tz = std::countr_zero(h | (std::uint64_t(1) << max_round));
    const auto round = std::uint64_t(max_round - std::min(tz, max_round));
    keys[at] = round << 57 | code;
    order[at] = int(at);
    out._points.push_back(pts[at]);
    out._duplicate_of.push_back(int(at));
    out._vertex_cell.push_back(-1);
    seeded.push_back(0);
  }
  if (at == n) {
    out._vertex_cell.push_back(-1); // the infinite vertex's
    at = 0;
    pass = 0;
    bins.fill(0);
    keys2.reset(new std::uint64_t[n]);
    order2.reset(new int[n]);
    st = stage::sorting_count;
  }
}

void tet_complex_builder::impl::do_sort_count(std::size_t &units, std::size_t budget) {
  const int shift = pass * radix_bits;
  for (; at < n && !out_of_budget(units, budget); ++at, ++units)
    ++bins[std::size_t(keys[at] >> shift) & (radix_bins - 1)];
  if (at == n) {
    std::size_t sum = 0;
    for (std::size_t &b : bins) {
      const std::size_t c = b;
      b = sum;
      sum += c;
    }
    at = 0;
    st = stage::sorting_move;
  }
}

void tet_complex_builder::impl::do_sort_move(std::size_t &units, std::size_t budget) {
  const int shift = pass * radix_bits;
  for (; at < n && !out_of_budget(units, budget); ++at, ++units) {
    const std::size_t to = bins[std::size_t(keys[at] >> shift) & (radix_bins - 1)]++;
    keys2[to] = keys[at];
    order2[to] = order[at];
  }
  if (at == n) {
    keys.swap(keys2);
    order.swap(order2);
    at = 0;
    bins.fill(0);
    if (++pass < radix_passes) {
      st = stage::sorting_count;
    } else {
      keys.reset();
      keys2.reset();
      order2.reset();
      seed_found = 0;
      st = stage::seeding;
    }
  }
}

#ifdef CVC_WITH_SHEWCHUK

// ---- seeding: four points not in one plane, in insertion order ----
void tet_complex_builder::impl::do_seed(std::size_t &units, std::size_t budget) {
  if (seed_found == 0) {
    seed[0] = order[0];
    seed_found = 1;
    at = 1;
  }
  for (; at < n && seed_found < 4 && !out_of_budget(units, budget); ++at, ++units) {
    const int q = order[at];
    const point &p = P(q);
    bool ok = false;
    if (seed_found == 1)
      ok = !same_point(p, P(seed[0]));
    else if (seed_found == 2)
      ok = !collinear(P(seed[0]), P(seed[1]), p);
    else
      ok = orientation(P(seed[0]), P(seed[1]), P(seed[2]), p) != 0;
    if (ok)
      seed[std::size_t(seed_found++)] = q;
  }
  if (seed_found == 4) {
    make_first_cells();
    at = 0;
    st = stage::inserting;
  } else if (at >= n) {
    static constexpr const char *what[] = {"", "all %zu points are the same point",
                                           "all %zu points lie on one line",
                                           "all %zu points lie in one plane"};
    fail_input(fmt(what[seed_found], n) +
               "; a tetrahedral complex needs at least 4 points not in one plane");
  }
}

int tet_complex_builder::impl::new_cell(const cell &k) {
  int c;
  if (!free_cells.empty()) {
    c = free_cells.back();
    free_cells.pop_back();
    cells[std::size_t(c)] = k;
  } else {
    c = int(cells.size());
    cells.push_back(k);
    visit.push_back(0);
  }
  live_finite += k.v[0] != inf() && k.v[1] != inf() && k.v[2] != inf() && k.v[3] != inf();
  ++stats.created_cells;
  return c;
}

void tet_complex_builder::impl::free_cell(int c) {
  const cell &k = cells[std::size_t(c)];
  live_finite -= k.v[0] != inf() && k.v[1] != inf() && k.v[2] != inf() && k.v[3] != inf();
  cells[std::size_t(c)].v = {-1, -1, -1, -1};
  free_cells.push_back(c);
}

// The first tetrahedron and the four infinite cells around it.
void tet_complex_builder::impl::make_first_cells() {
  std::array<int, 4> s = seed;
  if (orientation(P(s[0]), P(s[1]), P(s[2]), P(s[3])) < 0)
    std::swap(s[2], s[3]);
  cells.reserve(8 * n + 16);
  visit.reserve(8 * n + 16);
  const int c0 = new_cell({s, {-1, -1, -1, -1}});
  int ic[4];
  for (int i = 0; i < 4; ++i) {
    // The infinite cell across facet i: vertex i becomes the infinite vertex, and two others
    // swap so that a point outside (across facet i) in its place orients the cell positively.
    cell k{s, {-1, -1, -1, -1}};
    k.v[std::size_t(i)] = inf();
    std::swap(k.v[std::size_t((i + 1) & 3)], k.v[std::size_t((i + 2) & 3)]);
    k.n[std::size_t(i)] = c0;
    ic[i] = new_cell(k);
    cells[std::size_t(c0)].n[std::size_t(i)] = ic[i];
  }
  // Infinite cells i and j share the facet through the infinite vertex and the edge of the
  // tetrahedron that avoids vertices i and j.
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      if (i == j)
        continue;
      cell &a = cells[std::size_t(ic[i])];
      // In cell ic[i], the facet opposite the vertex s[j].
      for (int f = 0; f < 4; ++f)
        if (a.v[std::size_t(f)] == s[std::size_t(j)])
          a.n[std::size_t(f)] = ic[j];
    }
  for (int v : s) {
    out._vertex_cell[std::size_t(v)] = c0;
    seeded[std::size_t(v)] = 1;
  }
  out._vertex_cell[std::size_t(inf())] = ic[0];
  out._num_vertices = 4;
  hint = c0;
  walk_rng = mix64(opt.seed ^ 0x77616c6bULL);
  stats.inserted = 4;
}

// The cell holding point q: a finite cell whose closure contains it, or an infinite cell whose
// hull facet it is strictly beyond. Sets `duplicate` when q equals a vertex. A visibility walk
// from the hint; in a Delaunay triangulation it always terminates. Each cell picks its first
// facet to test pseudo-randomly, and the facet just crossed is skipped.
int tet_complex_builder::impl::locate(int q, std::size_t &touched, int &duplicate) {
  const point &p = P(q);
  int c = hint;
  {
    const int ii = out.index(c, inf());
    if (ii >= 0)
      c = cells[std::size_t(c)].n[std::size_t(ii)];
  }
  int prev = -1;
  duplicate = -1;
  for (std::size_t steps = 0;; ++steps) {
    if (steps > cells.size())
      throw tet_complex_error("tet_complex: the locating walk did not end (internal error)");
    ++touched;
    ++stats.walk_steps;
    const cell &k = cells[std::size_t(c)];
    walk_rng = walk_rng * 6364136223846793005ULL + 1442695040888963407ULL;
    const int first = int(walk_rng >> 62);
    int next = -1;
    for (int t = 0; t < 4; ++t) {
      const int i = (first + t) & 3;
      const int nb = k.n[std::size_t(i)];
      if (nb == prev)
        continue;
      std::array<const point *, 4> v{&P(k.v[0]), &P(k.v[1]), &P(k.v[2]), &P(k.v[3])};
      v[std::size_t(i)] = &p;
      if (orientation(*v[0], *v[1], *v[2], *v[3]) < 0) {
        next = nb;
        break;
      }
    }
    if (next < 0) {
      for (int i = 0; i < 4; ++i)
        if (same_point(P(k.v[std::size_t(i)]), p))
          duplicate = k.v[std::size_t(i)];
      return c;
    }
    prev = c;
    c = next;
    if (out.index(c, inf()) >= 0)
      return c;
  }
}

// Point `to` (a duplicate with a smaller index) takes over vertex `from`.
void tet_complex_builder::impl::rename_vertex(int from, int to) {
  std::vector<int> cs;
  out.incident_cells(from, cs);
  for (int c : cs)
    cells[std::size_t(c)].v[std::size_t(out.index(c, from))] = to;
  out._vertex_cell[std::size_t(to)] = out._vertex_cell[std::size_t(from)];
  out._vertex_cell[std::size_t(from)] = -1;
  out._duplicate_of[std::size_t(from)] = to;
  out._duplicate_of[std::size_t(to)] = to;
}

std::size_t tet_complex_builder::impl::insert(int q) {
  std::size_t touched = 0;
  int dup = -1;
  const int start = locate(q, touched, dup);
  if (dup >= 0) {
    ++stats.duplicates;
    if (q < dup)
      rename_vertex(dup, q);
    else
      out._duplicate_of[std::size_t(q)] = dup;
    return touched;
  }
  // The conflict region, breadth first from the located cell (which is in conflict: q is inside
  // its closure, so strictly inside its circumsphere, or strictly beyond its hull facet).
  if (++epoch >= 0x7fffffffU) {
    std::fill(visit.begin(), visit.end(), 0U);
    epoch = 1;
  }
  const std::uint32_t outside = 2 * epoch, inside = 2 * epoch + 1;
  const predicates pred = pr();
  conflict.clear();
  boundary.clear();
  conflict.push_back(start);
  visit[std::size_t(start)] = inside;
  for (std::size_t k = 0; k < conflict.size(); ++k) {
    const int c = conflict[k];
    for (int i = 0; i < 4; ++i) {
      const int nb = cells[std::size_t(c)].n[std::size_t(i)];
      std::uint32_t &mark = visit[std::size_t(nb)];
      if (mark != inside && mark != outside) {
        ++touched;
        mark = pred.conflict(cells[std::size_t(nb)], inf(), q) ? inside : outside;
        if (mark == inside)
          conflict.push_back(nb);
      }
      if (mark == outside)
        boundary.push_back({c, i});
    }
  }
  stats.conflict_cells += conflict.size();
  stats.max_conflict = std::max(stats.max_conflict, conflict.size());

  // Re-star the cavity from q: a new cell per boundary facet, which replaces the conflict cell on
  // that facet with q in place of the facet's opposite vertex (so it keeps the orientation), and
  // links to the outside cell. The new cells link to each other across the facets through q,
  // matched by the cavity edge each contains (every edge of the cavity's boundary is in exactly
  // two boundary facets).
  created.clear();
  std::size_t table = 64;
  while (table < 6 * boundary.size())
    table <<= 1;
  if (edge_table.size() < table)
    edge_table.assign(table, edge_slot{0, -1, -1, 0});
  if (++edge_stamp == 0) {
    for (edge_slot &s : edge_table)
      s.stamp = 0;
    edge_stamp = 1;
  }
  const std::size_t mask = table - 1;
  for (const boundary_facet &b : boundary) {
    cell k = cells[std::size_t(b.c)];
    const int outer = k.n[std::size_t(b.i)];
    k.v[std::size_t(b.i)] = q;
    k.n = {-1, -1, -1, -1};
    k.n[std::size_t(b.i)] = outer;
    const int nc = new_cell(k);
    created.push_back(nc);
    cell &o = cells[std::size_t(outer)];
    for (int m = 0; m < 4; ++m)
      if (o.n[std::size_t(m)] == b.c)
        o.n[std::size_t(m)] = nc;
    for (int j = 0; j < 4; ++j) {
      if (j == b.i)
        continue;
      // The facet opposite v[j] holds q and the cavity edge of the other two vertices.
      int e[2], m = 0;
      for (int t = 0; t < 4; ++t)
        if (t != j && t != b.i)
          e[m++] = k.v[std::size_t(t)];
      const auto lo_v = std::uint32_t(std::min(e[0], e[1]));
      const auto hi_v = std::uint32_t(std::max(e[0], e[1]));
      const std::uint64_t key = std::uint64_t(lo_v) << 32 | hi_v;
      std::size_t h = std::size_t((key * 0x9e3779b97f4a7c15ULL) >> 40) & mask;
      for (;; h = (h + 1) & mask) {
        edge_slot &s = edge_table[h];
        if (s.stamp != edge_stamp) {
          s = {key, nc, j, edge_stamp};
          break;
        }
        if (s.key == key) {
          cells[std::size_t(nc)].n[std::size_t(j)] = s.cell;
          cells[std::size_t(s.cell)].n[std::size_t(s.index)] = nc;
          break;
        }
      }
    }
  }
  for (int c : conflict)
    free_cell(c);
  int finite_new = -1;
  for (int c : created) {
    const cell &k = cells[std::size_t(c)];
    bool fin = true;
    for (int v : k.v) {
      out._vertex_cell[std::size_t(v)] = c;
      fin &= v != inf();
    }
    if (fin)
      finite_new = c;
  }
  hint = finite_new >= 0 ? finite_new : created.back();
  ++out._num_vertices;
  ++stats.inserted;
  return touched + conflict.size();
}

// ---- inserting: one point per iteration; its units are the cells it touched ----
void tet_complex_builder::impl::do_insert(std::size_t &units, std::size_t budget) {
  for (; at < n && !out_of_budget(units, budget); ++at) {
    const int q = order[at];
    if (seeded[std::size_t(q)]) {
      ++units;
      continue;
    }
    units += insert(q);
  }
  if (at == n) {
    order.reset();
    std::vector<char>().swap(seeded);
    std::vector<std::uint32_t>().swap(visit);
    std::vector<edge_slot>().swap(edge_table);
    at = 0;
    remap.reserve(cells.size());
    out._num_finite_cells = live_finite;
    st = stage::compacting_map;
  }
}

#endif // CVC_WITH_SHEWCHUK

// ---- compacting: live cells renumbered, finite ones first, in the order they were made ----
void tet_complex_builder::impl::do_compact_map(std::size_t &units, std::size_t budget) {
  if (at == 0) {
    next_finite = 0;
    next_infinite = out._num_finite_cells;
  }
  for (; at < cells.size() && !out_of_budget(units, budget); ++at, ++units) {
    const cell &k = cells[at];
    if (k.v[0] < 0) {
      remap.push_back(-1);
      continue;
    }
    const bool fin = k.v[0] != inf() && k.v[1] != inf() && k.v[2] != inf() && k.v[3] != inf();
    remap.push_back(fin ? next_finite++ : next_infinite++);
  }
  if (at == cells.size()) {
    std::vector<int>().swap(free_cells);
    compact_target.reserve(std::size_t(next_infinite));
    at = 0;
    move_pass = 0;
    st = stage::compacting_move;
  }
}

// Two passes, the finite cells then the infinite ones, each appended in order: the new ids of
// each kind rise with the old ones, so the packed array is written front to back.
void tet_complex_builder::impl::do_compact_move(std::size_t &units, std::size_t budget) {
  const int split = out._num_finite_cells;
  for (; at < cells.size() && !out_of_budget(units, budget); ++at, ++units) {
    const int to = remap[at];
    if (to < 0 || (to < split) != (move_pass == 0))
      continue;
    cell k = cells[at];
    for (int &nb : k.n)
      nb = remap[std::size_t(nb)];
    compact_target.push_back(k);
  }
  if (at == cells.size()) {
    at = 0;
    if (++move_pass < 2)
      return;
    cells.swap(compact_target);
    std::vector<cell>().swap(compact_target);
    st = stage::compacting_points;
  }
}

// Vertex cells renumbered; duplicate chains (a vertex renamed to a smaller duplicate) resolved.
void tet_complex_builder::impl::do_compact_points(std::size_t &units, std::size_t budget) {
  for (; at <= n && !out_of_budget(units, budget); ++at, ++units) {
    int &vc = out._vertex_cell[at];
    if (vc >= 0)
      vc = remap[std::size_t(vc)];
    if (at < n) {
      int d = out._duplicate_of[at];
      while (out._duplicate_of[std::size_t(d)] != d)
        d = out._duplicate_of[std::size_t(d)];
      out._duplicate_of[at] = d;
    }
  }
  if (at > n) {
    std::vector<int>().swap(remap);
    at = 0;
    if (opt.circumcentres) {
      out._circumcentres.reserve(cells.size());
      st = stage::circumcentres;
    } else {
      st = stage::done;
    }
  }
}

void tet_complex_builder::impl::do_circumcentres(std::size_t &units, std::size_t budget) {
  for (; at < cells.size() && !out_of_budget(units, budget); ++at, ++units)
    out._circumcentres.push_back(out.compute_circumcentre(int(at)));
  if (at == cells.size()) {
    at = 0;
    st = stage::done;
  }
}

void tet_complex_builder::impl::update_progress() {
  // Phase spans, by their typical share of the time.
  const double dn = n > 0 ? double(n) : 1.0;
  const double dc = !cells.empty() ? double(cells.size()) : 1.0;
  double p = prog;
  switch (st) {
  case stage::checking:
    p = 0.01 * double(at) / dn;
    break;
  case stage::keying:
    p = 0.01 + 0.01 * double(at) / dn;
    break;
  case stage::sorting_count:
  case stage::sorting_move:
    p = 0.02 +
        0.06 * (double(pass) + (st == stage::sorting_move ? 0.5 : 0.0) + 0.5 * double(at) / dn) /
            radix_passes;
    break;
  case stage::seeding:
    p = 0.08;
    break;
  case stage::inserting:
    p = 0.08 + 0.82 * double(at) / dn;
    break;
  case stage::compacting_map:
    p = 0.90 + 0.02 * double(at) / dc;
    break;
  case stage::compacting_move:
    p = 0.92 + 0.02 * (double(move_pass) + double(at) / dc) / 2.0;
    break;
  case stage::compacting_points:
    p = 0.94 + 0.01 * double(at) / (dn + 1);
    break;
  case stage::circumcentres:
    p = 0.95 + 0.05 * double(at) / dc;
    break;
  case stage::done:
    p = 1.0;
    break;
  case stage::failed:
    break;
  }
  prog = std::max(prog, std::min(p, 1.0));
}

tet_complex_builder::tet_complex_builder(
    std::shared_ptr<const std::vector<std::array<double, 3>>> points,
    const tet_complex_options &opt)
    : _impl(std::make_unique<impl>()) {
  _impl->input =
      points ? std::move(points) : std::make_shared<const std::vector<std::array<double, 3>>>();
  _impl->opt = opt;
}

tet_complex_builder::tet_complex_builder(std::vector<std::array<double, 3>> points,
                                         const tet_complex_options &opt)
    : tet_complex_builder(
          std::make_shared<const std::vector<std::array<double, 3>>>(std::move(points)), opt) {}

tet_complex_builder::~tet_complex_builder() = default;
tet_complex_builder::tet_complex_builder(tet_complex_builder &&) noexcept = default;
tet_complex_builder &tet_complex_builder::operator=(tet_complex_builder &&) noexcept = default;

bool tet_complex_builder::step(std::size_t budget) {
  if (!_impl)
    return true;
  if (_impl->st == stage::failed) {
    if (_impl->cancel.load(std::memory_order_acquire))
      return true;
    std::rethrow_exception(_impl->error);
  }
  if (_impl->st == stage::done)
    return true;
  try {
    const bool ready = _impl->step(budget);
    if (_impl->cancel.load(std::memory_order_acquire)) {
      // Cancelled: done, with no result; no throw, so a result_job around it ends "cancelled".
      _impl->error = std::make_exception_ptr(tet_complex_error("tet_complex: cancelled"));
      _impl->st = stage::failed;
      _impl->out = tet_complex{};
      return true;
    }
    return ready;
  } catch (...) {
    _impl->error = std::current_exception();
    _impl->st = stage::failed;
    _impl->out = tet_complex{};
    throw;
  }
}

bool tet_complex_builder::done() const {
  return !_impl || _impl->st == stage::done || _impl->st == stage::failed;
}

const char *tet_complex_builder::phase() const {
  if (!_impl)
    return "done";
  switch (_impl->st) {
  case stage::checking:
    return "checking";
  case stage::keying:
  case stage::sorting_count:
  case stage::sorting_move:
    return "ordering";
  case stage::seeding:
  case stage::inserting:
    return "inserting";
  case stage::compacting_map:
  case stage::compacting_move:
  case stage::compacting_points:
    return "compacting";
  case stage::circumcentres:
    return "circumcentres";
  case stage::done:
    return "done";
  case stage::failed:
    return "failed";
  }
  return "done";
}

double tet_complex_builder::progress() const { return _impl ? _impl->prog : 1.0; }

std::string tet_complex_builder::status() const {
  if (!_impl)
    return "done";
  if (_impl->st == stage::failed) {
    try {
      std::rethrow_exception(_impl->error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (_impl->st == stage::done)
    return fmt("done: %zu points, %d vertices, %d finite cells", _impl->n, _impl->out._num_vertices,
               _impl->out._num_finite_cells);
  return fmt("%s %d%%", phase(), int(_impl->prog * 100));
}

void tet_complex_builder::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

const tet_complex_stats &tet_complex_builder::stats() const {
  static constexpr tet_complex_stats none{};
  return _impl ? _impl->stats : none;
}

tet_complex tet_complex_builder::take() {
  if (_impl && _impl->st == stage::failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || _impl->st != stage::done)
    throw tet_complex_error("tet_complex: take() before the build is done");
  tet_complex t = std::move(_impl->out);
  _impl.reset();
  return t;
}

tet_complex delaunay_tet_complex(std::vector<std::array<double, 3>> points,
                                 const tet_complex_options &opt) {
  tet_complex_builder b(std::move(points), opt);
  b.step(0);
  return b.take();
}

} // namespace cvc
