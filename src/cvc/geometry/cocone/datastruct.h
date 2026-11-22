/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the cocone and skeleton code)
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// The cocone code's data structures, on cvc::tet_complex instead of CGAL
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2). The skeletonization port vendors
// VolumeRover's Skeletonization (transfix/volrover src/Skeletonization, UT LGPL-2.1), whose cocone
// core is shared with TightCocone, PocketTunnel and Curation. That code was written against CGAL's
// Delaunay_triangulation_3 with custom vertex and cell bases (its datastruct.h); this header gives
// it the same names, so the vendored files change little:
//   - the kernel: Point, Vector, Ray, Segment, Triangle_3, Tetrahedron and the cg:: functions the
//     code called as CGAL:: ones (cross_product, circumcenter, orientation, do_intersect, ...),
//     with EPICK's semantics: constructions in doubles, term for term as CGAL's Cartesian kernel
//     computes them, and predicates exact (Shewchuk's);
//   - the handles: Vertex_handle and Cell_handle are pointers to VC_vertex and VC_cell, which carry
//     the legacy bases' fields and the vertex/neighbour pointers of the cell, filled from a
//     tet_complex;
//   - Facet, Edge, the iterators over finite and all cells, vertices, facets and edges (in CGAL's
//     orders: a facet is reported from its lower cell, an edge from the lowest cell around it),
//     and the Facet_circulator and Cell_circulator around an edge, which move as CGAL's do.
// Nothing here is process-wide: a Triangulation is one call's.

#ifndef CVC_GEOMETRY_SKELETON_DATASTRUCT_H
#define CVC_GEOMETRY_SKELETON_DATASTRUCT_H

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cvc/geometry/tet_complex.h>
#include <iterator>
#include <list>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// The legacy code's CGAL_assertion()s were compiled out in its release builds. They stay
// compiled out here; the few whose arguments had side effects were rewritten where they were.
#define SK_ASSERT(x) ((void)0)

namespace cvc::cocone {

using std::list;
using std::map;
using std::set;
using std::swap;
using std::vector;

// ============================================================================================
// The kernel.

struct Vector {
  double c[3] = {0, 0, 0};
  constexpr Vector() = default;
  constexpr Vector(double x, double y, double z) : c{x, y, z} {}
  double x() const { return c[0]; }
  double y() const { return c[1]; }
  double z() const { return c[2]; }
  double operator[](int i) const { return c[i]; }
  double squared_length() const { return c[0] * c[0] + c[1] * c[1] + c[2] * c[2]; }
  Vector operator-() const { return {-c[0], -c[1], -c[2]}; }
};
inline Vector operator+(const Vector &a, const Vector &b) {
  return {a.c[0] + b.c[0], a.c[1] + b.c[1], a.c[2] + b.c[2]};
}
inline Vector operator-(const Vector &a, const Vector &b) {
  return {a.c[0] - b.c[0], a.c[1] - b.c[1], a.c[2] - b.c[2]};
}
inline double operator*(const Vector &a, const Vector &b) { // the scalar product
  return a.c[0] * b.c[0] + a.c[1] * b.c[1] + a.c[2] * b.c[2];
}
inline Vector operator*(double s, const Vector &v) { return {s * v.c[0], s * v.c[1], s * v.c[2]}; }
inline Vector operator*(const Vector &v, double s) { return {v.c[0] * s, v.c[1] * s, v.c[2] * s}; }
inline Vector operator/(const Vector &v, double s) { return {v.c[0] / s, v.c[1] / s, v.c[2] / s}; }
inline bool operator==(const Vector &a, const Vector &b) {
  return a.c[0] == b.c[0] && a.c[1] == b.c[1] && a.c[2] == b.c[2];
}

struct Point {
  double c[3] = {0, 0, 0};
  constexpr Point() = default;
  constexpr Point(double x, double y, double z) : c{x, y, z} {}
  explicit Point(const tet_complex::point &p) : c{p[0], p[1], p[2]} {}
  double x() const { return c[0]; }
  double y() const { return c[1]; }
  double z() const { return c[2]; }
  double operator[](int i) const { return c[i]; }
};
inline Vector operator-(const Point &a, const Point &b) {
  return {a.c[0] - b.c[0], a.c[1] - b.c[1], a.c[2] - b.c[2]};
}
inline Point operator+(const Point &p, const Vector &v) {
  return {p.c[0] + v.c[0], p.c[1] + v.c[1], p.c[2] + v.c[2]};
}
inline Point operator-(const Point &p, const Vector &v) {
  return {p.c[0] - v.c[0], p.c[1] - v.c[1], p.c[2] - v.c[2]};
}
inline bool operator==(const Point &a, const Point &b) {
  return a.c[0] == b.c[0] && a.c[1] == b.c[1] && a.c[2] == b.c[2];
}
inline bool operator!=(const Point &a, const Point &b) { return !(a == b); }
inline bool operator<(const Point &a, const Point &b) { // CGAL's lexicographic xyz order
  if (a.c[0] != b.c[0])
    return a.c[0] < b.c[0];
  if (a.c[1] != b.c[1])
    return a.c[1] < b.c[1];
  return a.c[2] < b.c[2];
}

struct Segment {
  Point s, t;
  Segment() = default;
  Segment(const Point &a, const Point &b) : s(a), t(b) {}
  const Point &point(int i) const { return i == 0 ? s : t; }
  const Point &source() const { return s; }
  const Point &target() const { return t; }
};

struct Ray {
  Point s, t; // the source and a second point
  Ray() = default;
  Ray(const Point &a, const Point &b) : s(a), t(b) {}
  const Point &source() const { return s; }
  Vector to_vector() const { return t - s; }
};

struct Triangle_3 {
  Point p[3];
  Triangle_3() = default;
  Triangle_3(const Point &a, const Point &b, const Point &c) : p{a, b, c} {}
  const Point &vertex(int i) const { return p[i]; }
  double squared_area() const; // CGAL's: |(q - p) x (r - p)|^2 / 4
  bool is_degenerate() const;  // exactly collinear
};

struct Tetrahedron {
  Point p[4];
  Tetrahedron() = default;
  Tetrahedron(const Point &a, const Point &b, const Point &c, const Point &d) : p{a, b, c, d} {}
  double volume() const;      // CGAL's: det(p1 - p0, p2 - p0, p3 - p0) / 6, in doubles
  bool is_degenerate() const; // exactly coplanar
};

// The CGAL functions the vendored code calls, with CGAL's semantics (cg:: where it said CGAL::).
namespace cg {

enum Orientation { NEGATIVE = -1, COPLANAR = 0, POSITIVE = 1 };
enum Bounded_side { ON_UNBOUNDED_SIDE = -1, ON_BOUNDARY = 0, ON_BOUNDED_SIDE = 1 };

inline constexpr Point ORIGIN{0, 0, 0};
inline constexpr Vector NULL_VECTOR{0, 0, 0};

inline double to_double(double x) { return x; }
inline double sqrt(double x) { return std::sqrt(x); }
inline bool is_positive(double x) { return x > 0; }
inline bool is_negative(double x) { return x < 0; }
inline bool is_zero(double x) { return x == 0; }

template <class A, class B, class C> struct Triple {
  A first;
  B second;
  C third;
  Triple() = default;
  Triple(const A &a, const B &b, const C &c) : first(a), second(b), third(c) {}
};

inline Vector cross_product(const Vector &u, const Vector &v) {
  return {u.c[1] * v.c[2] - u.c[2] * v.c[1], u.c[2] * v.c[0] - u.c[0] * v.c[2],
          u.c[0] * v.c[1] - u.c[1] * v.c[0]};
}
inline Point midpoint(const Point &p, const Point &q) {
  return {(p.c[0] + q.c[0]) / 2, (p.c[1] + q.c[1]) / 2, (p.c[2] + q.c[2]) / 2};
}
// CGAL's circumcenterC3 for a tetrahedron and for a triangle in space, term for term.
Point circumcenter(const Point &p, const Point &q, const Point &r, const Point &s);
Point circumcenter(const Point &p, const Point &q, const Point &r);
// Exact (Shewchuk's predicates): CGAL::orientation, CGAL::collinear, and CGAL::do_intersect of a
// triangle and a segment in space (false for a degenerate triangle).
Orientation orientation(const Point &p, const Point &q, const Point &r, const Point &s);
bool collinear(const Point &p, const Point &q, const Point &r);
bool do_intersect(const Triangle_3 &t, const Segment &s);

// The 2D kernel pieces intersect.cpp uses: a ray against a segment, as CGAL's
// Ray_2_Segment_2_pair computes it (the lines' intersection, then the exact collinear-has-on
// tests on that point); a collinear overlap counts as no point, as CGAL::assign to a point fails.
template <class R> struct Point_2 {
  double c[2] = {0, 0};
  Point_2() = default;
  Point_2(double x, double y) : c{x, y} {}
  double x() const { return c[0]; }
  double y() const { return c[1]; }
};
template <class R> struct Vector_2 {
  double c[2];
};
template <class R> Vector_2<R> operator-(const Point_2<R> &a, const Point_2<R> &b) {
  return {{a.c[0] - b.c[0], a.c[1] - b.c[1]}};
}
template <class R> double operator*(const Vector_2<R> &a, const Vector_2<R> &b) {
  return a.c[0] * b.c[0] + a.c[1] * b.c[1];
}
template <class R> struct Segment_2 {
  Point_2<R> s, t;
  Segment_2() = default;
  Segment_2(const Point_2<R> &a, const Point_2<R> &b) : s(a), t(b) {}
  bool has_on(const Point_2<R> &p) const; // collinear and between (exact on p's doubles)
};
template <class R> struct Ray_2 {
  Point_2<R> s, t;
  Ray_2() = default;
  Ray_2(const Point_2<R> &a, const Point_2<R> &b) : s(a), t(b) {}
};
struct Object {
  std::optional<std::array<double, 2>> point;
};
std::optional<std::array<double, 2>> ray_segment_point(const double rs[2], const double rt[2],
                                                       const double ss[2], const double st[2]);
template <class R> Object intersection(const Ray_2<R> &r, const Segment_2<R> &s) {
  return Object{ray_segment_point(r.s.c, r.t.c, s.s.c, s.t.c)};
}
template <class R> bool assign(Point_2<R> &p, const Object &o) {
  if (!o.point)
    return false;
  p = Point_2<R>((*o.point)[0], (*o.point)[1]);
  return true;
}
bool segment_2_has_on(const double s[2], const double t[2], const double p[2]);
template <class R> bool Segment_2<R>::has_on(const Point_2<R> &p) const {
  return segment_2_has_on(s.c, t.c, p.c);
}

template <class R> using Vector_3 = Vector;

} // namespace cg

using Rep = int; // the legacy code's CGAL::Point_2<Rep>; the representation is doubles

// ============================================================================================
// The legacy vertex and cell bases (datastruct.h's VC_vertex and VC_cell), without CGAL.

enum COMPUTATION_STATUS {
  SURF,
  SUCCESS,
  COSPH,
  INFINITE_,
  NOT_FOUND,
  INTERSECT_PROBLEM,
  SAME_P,
  ERROR_
};
enum COLOR { BLACK, RED, BLUE, YELLOW };

struct VC_cell;
struct VC_vertex;
using Vertex_handle = VC_vertex *;
using Cell_handle = VC_cell *;

struct VC_vertex {
  // ---- the TDS part ----
  const Point *pt = nullptr; // into the Triangulation's points; null for the infinite vertex
  Cell_handle c = nullptr;   // a cell with this vertex
  int tc = -1;               // the tet_complex vertex id (the input point's index)
  const Point &point() const { return *pt; }
  Cell_handle cell() const { return c; }

  // ---- the legacy fields ----
  void set_normal(const Vector &v) { v_normal = v; }
  Vector normal() const { return v_normal; }
  void set_pole(const Point &p) { v_pole = p; }
  Point pole() const { return v_pole; }
  void set_width(const double &w) { v_width = w; }
  double width() const { return v_width; }
  void set_diameter(const double &d) { v_diameter = d; }
  double diameter() const { return v_diameter; }
  void set_flat(const bool b) { v_flat = b; }
  bool is_flat() const { return v_flat; }
  void set_convex_hull(const bool b) { v_chull = b; }
  bool on_convex_hull() const { return v_chull; }
  void set_isolated(const bool b) { v_isolated = b; }
  bool is_isolated() const { return v_isolated; }
  void set_umbrella(const bool b) { v_umbrella = b; }
  bool has_umbrella() const { return v_umbrella; }
  void set_on_smooth_surface(const bool b) { v_on_smooth_surface = b; }
  bool on_smooth_surface() const { return v_on_smooth_surface; }
  void set_pl(const bool b) { v_pl = b; }
  bool is_pl() const { return v_pl; }
  void set_cyl(const bool b) { v_cyl = b; }
  bool is_cyl() const { return v_cyl; }

  std::array<double, 3> nnd_vector{DBL_MAX, DBL_MAX, DBL_MAX}; // robust cocone
  int id = -1;
  bool tag = false;
  bool visited = false;
  bool bad = false;
  bool bad_neighbor = false;
  vector<int> bad_v;
  vector<Vector> normal_stack; // medial axis: the normals of the incident disk

private:
  Vector v_normal = cg::NULL_VECTOR;
  Point v_pole; // set to the point itself when the vertex is made
  double v_width = 0.0;
  double v_diameter = 0.0;
  bool v_flat = true;
  bool v_chull = false;
  bool v_umbrella = false;
  bool v_isolated = true;
  bool v_on_smooth_surface = false;
  bool v_pl = false;
  bool v_cyl = false;

  friend class Triangulation;
};

struct VC_cell {
  // ---- the TDS part ----
  Vertex_handle V[4] = {nullptr, nullptr, nullptr, nullptr};
  Cell_handle N[4] = {nullptr, nullptr, nullptr, nullptr};
  int tc = -1; // the tet_complex cell id
  Vertex_handle vertex(int i) const { return V[i]; }
  Cell_handle neighbor(int i) const { return N[i]; }
  int index(Vertex_handle v) const {
    for (int i = 0; i < 4; ++i)
      if (V[i] == v)
        return i;
    return -1;
  }
  int index(Cell_handle n) const {
    for (int i = 0; i < 4; ++i)
      if (N[i] == n)
        return i;
    return -1;
  }
  bool has_vertex(Vertex_handle v) const { return index(v) >= 0; }
  bool has_vertex(Vertex_handle v, int &i) const {
    i = index(v);
    return i >= 0;
  }

  // ---- the legacy fields ----
  void set_voronoi(const Point &p) { c_voronoi = p; }
  Point voronoi() const { return c_voronoi; }
  void set_cocone_flag(int i, bool b) { c_cocone_flag[i] = b; }
  bool cocone_flag(int i) const { return c_cocone_flag[i]; }
  void set_removable(int i, bool b) { c_removable[i] = b; }
  bool removable(int i) const { return c_removable[i]; }
  void set_cell_radius(const double r) { c_cell_radius = r; }
  double cell_radius() const { return c_cell_radius; }
  void set_bb(const bool b) { c_bb = b; }
  bool bb() const { return c_bb; }
  void set_deep_int(const int i, const bool b) { c_deep_int[i] = b; }
  bool deep_int(int i) const { return c_deep_int[i]; }
  void set_source(const int &i, const bool &b) { c_source[i] = b; }
  bool source(int i) const { return c_source[i]; }
  void set_terminus(const int &i, const bool &b) { c_terminus[i] = b; }
  bool terminus(int i) const { return c_terminus[i]; }
  void set_pl(const bool b) { c_pl = b; }
  bool is_pl() const { return c_pl; }
  void set_cyl(const bool b) { c_cyl = b; }
  bool is_cyl() const { return c_cyl; }
  void set_VF_on_medax(const int i, const int j, const bool val) {
    c_VF_on_medax[i][j] = val;
    c_VF_on_medax[j][i] = val;
  }
  bool VF_on_medax(int i, int j) const { return c_VF_on_medax[i][j]; }
  void set_VV_on_medax(const bool &b) { c_VV_on_medax = b; }
  bool VV_on_medax() const { return c_VV_on_medax; }
  void set_i1_saddle(const int i, const int j, const bool &b) {
    c_i1_saddle[i][j] = b;
    c_i1_saddle[j][i] = b;
  }
  bool i1_saddle(int i, int j) const { return c_i1_saddle[i][j]; }
  void set_VF_on_um_i1(const int i, const int j, const bool &b) {
    c_VF_on_um_i1[i][j] = b;
    c_VF_on_um_i1[j][i] = b;
    c_VV_on_um_i1 = c_VF_on_um_i1[0][1] || c_VF_on_um_i1[0][2] || c_VF_on_um_i1[0][3] ||
                    c_VF_on_um_i1[1][2] || c_VF_on_um_i1[1][3] || c_VF_on_um_i1[2][3];
  }
  bool VF_on_um_i1(int i, int j) const { return c_VF_on_um_i1[i][j]; }
  void set_VV_on_um_i1(const bool &b) { c_VV_on_um_i1 = b; }
  bool VV_on_um_i1() const { return c_VV_on_um_i1; }
  void set_cosph_pair(const int &i, const bool &b) { c_cosph_pair[i] = b; }
  bool cosph_pair(int i) const { return c_cosph_pair[i]; }
  void set_dirty(const bool &b) { c_dirty = b; }
  bool dirty() const { return c_dirty; }
  void set_in_helix_cluster(const bool &b) { c_in_helix_cluster = b; }
  bool in_helix_cluster() const { return c_in_helix_cluster; }
  void set_big_pl(const bool &b) { c_big_pl = b; }
  bool big_pl() const { return c_big_pl; }

  // Tight Cocone
  bool outside = false;
  bool bdy[4] = {};
  bool opaque[4] = {};
  bool transp = false;
  int umbrella_member[4][4] = {
      {-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1, -1}};
  // general purpose
  bool tag[4] = {};
  int id = -1;
  bool flag = false;
  bool visited = false;
  bool f_visited[4] = {};
  bool e_visited[4][4] = {};
  bool c_cosph = false;
  // graph membership in U2, and the planar cluster a VF belongs to
  int saddle_g_vid[4] = {-1, -1, -1, -1};
  int g_vid = -1;
  int patch_id[4][4] = {{-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1, -1}, {-1, -1, -1, -1}};

private:
  Point c_voronoi;
  bool c_cocone_flag[4] = {};
  bool c_removable[4] = {true, true, true, true};
  bool c_VV_on_medax = false;
  bool c_VF_on_medax[4][4] = {};
  bool c_i1_saddle[4][4] = {};
  bool c_VF_on_um_i1[4][4] = {};
  bool c_VV_on_um_i1 = false;
  double c_cell_radius = 0.0;
  bool c_deep_int[4] = {};
  bool c_bb = false;
  bool c_pl = false;
  bool c_cyl = false;
  bool c_source[4] = {};
  bool c_terminus[4] = {};
  bool c_dirty = false;
  bool c_cosph_pair[4] = {};
  bool c_in_helix_cluster = false;
  bool c_big_pl = false;
};

using Facet = std::pair<Cell_handle, int>;
using Edge = cg::Triple<Cell_handle, int, int>;

// CGAL's next_around_edge (Triangulation_utils_3).
inline int next_around_edge(int i, int j) {
  static constexpr int t[4][4] = {{5, 2, 3, 1}, {3, 5, 0, 2}, {1, 3, 5, 0}, {2, 0, 1, 5}};
  return t[i][j];
}

// ---- circulators around an edge, as CGAL's ----
class Facet_circulator {
public:
  Facet_circulator() = default;
  explicit Facet_circulator(const Edge &e)
      : _s(e.first->vertex(e.second)), _t(e.first->vertex(e.third)), _pos(e.first) {}
  Facet operator*() const {
    return Facet(_pos, next_around_edge(_pos->index(_s), _pos->index(_t)));
  }
  Facet_circulator &operator++() {
    _pos = _pos->neighbor(next_around_edge(_pos->index(_s), _pos->index(_t)));
    return *this;
  }
  Facet_circulator operator++(int) {
    Facet_circulator c = *this;
    ++*this;
    return c;
  }
  bool operator==(const Facet_circulator &o) const {
    return _pos == o._pos && _s == o._s && _t == o._t;
  }
  bool operator!=(const Facet_circulator &o) const { return !(*this == o); }

private:
  Vertex_handle _s = nullptr, _t = nullptr;
  Cell_handle _pos = nullptr;
};

class Cell_circulator {
public:
  Cell_circulator() = default;
  explicit Cell_circulator(const Edge &e)
      : _s(e.first->vertex(e.second)), _t(e.first->vertex(e.third)), _pos(e.first) {}
  VC_cell &operator*() const { return *_pos; }
  Cell_handle operator->() const { return _pos; }
  operator Cell_handle() const { return _pos; }
  Cell_circulator &operator++() {
    _pos = _pos->neighbor(next_around_edge(_pos->index(_s), _pos->index(_t)));
    return *this;
  }
  Cell_circulator operator++(int) {
    Cell_circulator c = *this;
    ++*this;
    return c;
  }
  bool operator==(const Cell_circulator &o) const {
    return _pos == o._pos && _s == o._s && _t == o._t;
  }
  bool operator!=(const Cell_circulator &o) const { return !(*this == o); }

private:
  Vertex_handle _s = nullptr, _t = nullptr;
  Cell_handle _pos = nullptr;
};

class Triangulation;

// ---- iterators ----
// Cells: the finite ones are first in a tet_complex, so both ranges are plain pointer ranges.
using FCI = Cell_handle;
using ACI = Cell_handle;

// Vertices: the points in input order, skipping duplicates (and the infinite vertex, for FVI).
class FVI {
public:
  FVI() = default;
  FVI(Vertex_handle p, Vertex_handle end) : _p(p), _end(end) { skip(); }
  VC_vertex &operator*() const { return *_p; }
  Vertex_handle operator->() const { return _p; }
  operator Vertex_handle() const { return _p; }
  FVI &operator++() {
    ++_p;
    skip();
    return *this;
  }
  FVI operator++(int) {
    FVI c = *this;
    ++*this;
    return c;
  }
  bool operator==(const FVI &o) const { return _p == o._p; }
  bool operator!=(const FVI &o) const { return _p != o._p; }

private:
  void skip() {
    while (_p != _end && _p->pt == nullptr)
      ++_p;
  }
  Vertex_handle _p = nullptr, _end = nullptr;
};
using AVI = FVI;

// Facets: (c, i) for every cell c in order and i = 0..3, when c is the lower of the two cells
// (CGAL's facet iterator), and for FFI when the facet is finite.
class FFI {
public:
  FFI() = default;
  FFI(Cell_handle p, Cell_handle end, Vertex_handle inf, bool finite_only)
      : _f(p, 0), _end(end), _inf(inf), _finite(finite_only) {
    if (_f.first != _end && !accept())
      ++*this;
  }
  const Facet &operator*() const { return _f; }
  const Facet *operator->() const { return &_f; }
  FFI &operator++() {
    do {
      if (++_f.second == 4) {
        _f.second = 0;
        ++_f.first;
      }
    } while (_f.first != _end && !accept());
    return *this;
  }
  FFI operator++(int) {
    FFI c = *this;
    ++*this;
    return c;
  }
  bool operator==(const FFI &o) const { return _f == o._f; }
  bool operator!=(const FFI &o) const { return !(_f == o._f); }

private:
  bool accept() const {
    Cell_handle c = _f.first;
    if (c->neighbor(_f.second) < c)
      return false;
    if (!_finite)
      return true;
    for (int k = 1; k < 4; ++k)
      if (c->vertex((_f.second + k) % 4) == _inf)
        return false;
    return true;
  }
  Facet _f{nullptr, 0};
  Cell_handle _end = nullptr;
  Vertex_handle _inf = nullptr;
  bool _finite = true;
};
using AFI = FFI;

// Edges: (c, i, j) for every cell c in order and (i, j) in (0,1) (0,2) (0,3) (1,2) (1,3) (2,3),
// when c is the lowest cell around the edge (CGAL's edge iterator), and for FEI when both ends
// are finite.
class FEI {
public:
  FEI() = default;
  FEI(Cell_handle p, Cell_handle end, Vertex_handle inf, bool finite_only)
      : _e(p, 0, 1), _end(end), _inf(inf), _finite(finite_only) {
    if (_e.first != _end && !accept())
      ++*this;
  }
  const Edge &operator*() const { return _e; }
  const Edge *operator->() const { return &_e; }
  FEI &operator++() {
    do {
      if (_e.second == 2) {
        _e.second = 0;
        _e.third = 1;
        ++_e.first;
      } else if (_e.third == 3) {
        ++_e.second;
        _e.third = _e.second + 1;
      } else {
        ++_e.third;
      }
    } while (_e.first != _end && !accept());
    return *this;
  }
  FEI operator++(int) {
    FEI c = *this;
    ++*this;
    return c;
  }
  bool operator==(const FEI &o) const {
    return _e.first == o._e.first && _e.second == o._e.second && _e.third == o._e.third;
  }
  bool operator!=(const FEI &o) const { return !(*this == o); }

private:
  bool accept() const {
    Cell_handle c = _e.first;
    if (_finite && (c->vertex(_e.second) == _inf || c->vertex(_e.third) == _inf))
      return false;
    Cell_circulator cc(_e), start = cc;
    for (++cc; cc != start; ++cc)
      if (Cell_handle(cc) < c)
        return false;
    return true;
  }
  Edge _e{nullptr, 0, 1};
  Cell_handle _end = nullptr;
  Vertex_handle _inf = nullptr;
  bool _finite = true;
};
using AEI = FEI;

// ============================================================================================
// The triangulation: a tet_complex, with a VC_vertex per point and a VC_cell per cell.

class Triangulation {
public:
  using Point = cvc::cocone::Point;

  Triangulation() = default;
  Triangulation(const Triangulation &) = delete;
  Triangulation &operator=(const Triangulation &) = delete;

  // Takes the complex. Then fill() makes the vertices and cells, one per call (so a stepper can
  // slice it), until it returns true.
  void reset(tet_complex t);
  bool fill_one(); // true once every vertex and cell is made
  std::size_t fill_total() const { return 2 * (_t.points().size() + 1) + _t.cells().size(); }
  bool filled() const { return _filled; }
  const tet_complex &complex() const { return _t; }

  // ---- the Delaunay_triangulation_3 calls the cocone code makes ----
  std::size_t number_of_vertices() const { return std::size_t(_t.num_vertices()); }
  Vertex_handle infinite_vertex() const { return _inf; }
  bool is_infinite(const VC_vertex *v) const { return v == _inf; }
  bool is_infinite(const VC_cell *c) const {
    return c->V[0] == _inf || c->V[1] == _inf || c->V[2] == _inf || c->V[3] == _inf;
  }
  bool is_infinite(const Facet &f) const {
    for (int k = 1; k < 4; ++k)
      if (f.first->vertex((f.second + k) % 4) == _inf)
        return true;
    return false;
  }
  bool is_infinite(const Edge &e) const {
    return e.first->vertex(e.second) == _inf || e.first->vertex(e.third) == _inf;
  }
  // Whether v is a vertex of facet (c, i); j is its index in c.
  bool has_vertex(Cell_handle c, int i, Vertex_handle v, int &j) const {
    j = c->index(v);
    return j >= 0 && j != i;
  }
  bool has_vertex(const Facet &f, Vertex_handle v, int &j) const {
    return has_vertex(f.first, f.second, v, j);
  }
  // Unperturbed: ON_BOUNDARY on a cospherical tie, as CGAL's side_of_sphere(c, p) by default.
  cg::Bounded_side side_of_sphere(Cell_handle c, const Point &p) const;

  Facet_circulator incident_facets(const Edge &e) const { return Facet_circulator(e); }
  Cell_circulator incident_cells(const Edge &e) const { return Cell_circulator(e); }

  FVI finite_vertices_begin() const { return FVI(_v_begin, _v_end); }
  FVI finite_vertices_end() const { return FVI(_v_end, _v_end); }
  FCI finite_cells_begin() const { return _c_begin; }
  FCI finite_cells_end() const { return _c_finite_end; }
  ACI all_cells_begin() const { return _c_begin; }
  ACI all_cells_end() const { return _c_end; }
  FFI finite_facets_begin() const { return FFI(_c_begin, _c_end, _inf, true); }
  FFI finite_facets_end() const { return FFI(_c_end, _c_end, _inf, true); }
  AFI all_facets_begin() const { return AFI(_c_begin, _c_end, _inf, false); }
  AFI all_facets_end() const { return AFI(_c_end, _c_end, _inf, false); }
  FEI finite_edges_begin() const { return FEI(_c_begin, _c_end, _inf, true); }
  FEI finite_edges_end() const { return FEI(_c_end, _c_end, _inf, true); }

  std::size_t num_finite_cells() const { return std::size_t(_c_finite_end - _c_begin); }
  std::size_t num_cells() const { return std::size_t(_c_end - _c_begin); }

private:
  tet_complex _t;
  vector<Point> _points;
  vector<VC_vertex> _vertices; // num_points + 1, the infinite vertex last
  vector<VC_cell> _cells;
  Vertex_handle _inf = nullptr, _v_begin = nullptr, _v_end = nullptr;
  Cell_handle _c_begin = nullptr, _c_finite_end = nullptr, _c_end = nullptr;
  std::size_t _fill_at = 0;
  bool _filled = false;
};

typedef Triangulation::Point TPoint;

struct cell_cluster {
  cell_cluster() : rep(0), is_outside(false), nxt(0), tail(0) {}
  cell_cluster(int i) : rep(i), is_outside(false), nxt(0), tail(i) {}
  int find() const { return rep; }
  bool outside() const { return is_outside; }
  int rep;
  bool is_outside;
  double sq_r;
  vector<Vector> dirs;
  cell_cluster *nxt;
  int tail;
};

} // namespace cvc::cocone

#endif // CVC_GEOMETRY_SKELETON_DATASTRUCT_H
