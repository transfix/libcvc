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

// The geometric kernel the vendored ContourTiler runs on, in place of CGAL's
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3). The legacy library used CGAL::Cartesian<double> with its
// own point type (Point_25_: x, y, a z and an id) and CGAL's Polygon_2; these classes keep the
// names and members the vendored code calls, so its code keeps its shape, and none of CGAL is
// included.
//
//  * Coordinates are doubles, as the legacy Number_type was. tile_contours() snaps every contour
//    vertex onto a cvc::planar::grid first (world = origin + unit * integer, exact in a double), so
//    the input the tiler sees is exact on the grid the planar module's Booleans, offsets and
//    triangulations run on.
//  * Predicates are exact: orientation and coplanarity are Shewchuk's orient2d and orient3d
//    (cvc::exact), where CGAL::Cartesian<double> evaluated them in floating point. The two agree
//    except on input so close to degenerate that rounding decides it.
//  * Constructions (an intersection point, a projection) round, as they did.
//  * Point_2 compares x and y only (operator==), and hashes x, y and z, as Point_25_ did; Point_3
//    compares all three.
//
// Nothing here holds state: no statics, no globals.

#ifndef CVC_GEOMETRY_TILER_KERNEL_H
#define CVC_GEOMETRY_TILER_KERNEL_H

#include <algorithm>
#include <any>
#include <boost/functional/hash.hpp>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace cvc::tiler {

typedef double Number_type;

#define DEFAULT_ID() std::numeric_limits<std::size_t>::max()

// ============================================================================================
// Signs and enumerations, with CGAL's values

enum Sign { NEGATIVE = -1, ZERO = 0, POSITIVE = 1 };
typedef Sign Orientation;
constexpr Orientation LEFT_TURN = POSITIVE;
constexpr Orientation RIGHT_TURN = NEGATIVE;
constexpr Orientation CLOCKWISE = NEGATIVE;
constexpr Orientation COUNTERCLOCKWISE = POSITIVE;
constexpr Orientation COLLINEAR = ZERO;
constexpr Orientation COPLANAR = ZERO;
typedef Sign Oriented_side;
constexpr Oriented_side ON_NEGATIVE_SIDE = NEGATIVE;
constexpr Oriented_side ON_ORIENTED_BOUNDARY = ZERO;
constexpr Oriented_side ON_POSITIVE_SIDE = POSITIVE;
typedef Sign Comparison_result;
constexpr Comparison_result SMALLER = NEGATIVE;
constexpr Comparison_result EQUAL = ZERO;
constexpr Comparison_result LARGER = POSITIVE;
enum Bounded_side { ON_UNBOUNDED_SIDE = -1, ON_BOUNDARY = 0, ON_BOUNDED_SIDE = 1 };
enum Angle { OBTUSE = -1, RIGHT = 0, ACUTE = 1 };

inline Sign sign_of(double v) { return v > 0 ? POSITIVE : (v < 0 ? NEGATIVE : ZERO); }
inline Sign opposite(Sign s) { return Sign(-int(s)); }

inline double to_double(double d) { return d; }

struct Origin {};
constexpr Origin ORIGIN{};

// Exact predicates (Shewchuk's, through cvc::exact; kernel.cpp).
double exact_orient2d(double ax, double ay, double bx, double by, double cx, double cy);
double exact_orient3d(const double *a, const double *b, const double *c, const double *d);

// ============================================================================================
// Boxes

class Bbox_2 {
public:
  Bbox_2()
      : _xmin(std::numeric_limits<double>::infinity()),
        _ymin(std::numeric_limits<double>::infinity()),
        _xmax(-std::numeric_limits<double>::infinity()),
        _ymax(-std::numeric_limits<double>::infinity()) {}
  Bbox_2(double xmin, double ymin, double xmax, double ymax)
      : _xmin(xmin), _ymin(ymin), _xmax(xmax), _ymax(ymax) {}
  double xmin() const { return _xmin; }
  double ymin() const { return _ymin; }
  double xmax() const { return _xmax; }
  double ymax() const { return _ymax; }
  double min(int i) const { return i == 0 ? _xmin : _ymin; }
  double max(int i) const { return i == 0 ? _xmax : _ymax; }
  Bbox_2 operator+(const Bbox_2 &b) const {
    return Bbox_2(std::min(_xmin, b._xmin), std::min(_ymin, b._ymin), std::max(_xmax, b._xmax),
                  std::max(_ymax, b._ymax));
  }
  Bbox_2 &operator+=(const Bbox_2 &b) { return *this = *this + b; }
  bool operator==(const Bbox_2 &b) const {
    return _xmin == b._xmin && _ymin == b._ymin && _xmax == b._xmax && _ymax == b._ymax;
  }

private:
  double _xmin, _ymin, _xmax, _ymax;
};

inline bool do_overlap(const Bbox_2 &a, const Bbox_2 &b) {
  return !(a.xmax() < b.xmin() || b.xmax() < a.xmin() || a.ymax() < b.ymin() ||
           b.ymax() < a.ymin());
}

class Bbox_3 {
public:
  Bbox_3()
      : _min{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity()},
        _max{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()} {}
  Bbox_3(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
      : _min{xmin, ymin, zmin}, _max{xmax, ymax, zmax} {}
  double xmin() const { return _min[0]; }
  double ymin() const { return _min[1]; }
  double zmin() const { return _min[2]; }
  double xmax() const { return _max[0]; }
  double ymax() const { return _max[1]; }
  double zmax() const { return _max[2]; }
  double min(int i) const { return _min[i]; }
  double max(int i) const { return _max[i]; }
  Bbox_3 operator+(const Bbox_3 &b) const {
    return Bbox_3(std::min(_min[0], b._min[0]), std::min(_min[1], b._min[1]),
                  std::min(_min[2], b._min[2]), std::max(_max[0], b._max[0]),
                  std::max(_max[1], b._max[1]), std::max(_max[2], b._max[2]));
  }

private:
  double _min[3], _max[3];
};

inline bool do_overlap(const Bbox_3 &a, const Bbox_3 &b) {
  for (int i = 0; i < 3; ++i)
    if (a.max(i) < b.min(i) || b.max(i) < a.min(i))
      return false;
  return true;
}

// ============================================================================================
// Vectors and directions

class Direction_2;

class Vector_2 {
public:
  Vector_2() : _x(0), _y(0) {}
  Vector_2(double x, double y) : _x(x), _y(y) {}
  double x() const { return _x; }
  double y() const { return _y; }
  double hx() const { return _x; }
  double hy() const { return _y; }
  double operator[](int i) const { return i == 0 ? _x : _y; }
  double cartesian(int i) const { return (*this)[i]; }
  double squared_length() const { return _x * _x + _y * _y; }
  Vector_2 operator+(const Vector_2 &v) const { return Vector_2(_x + v._x, _y + v._y); }
  Vector_2 operator-(const Vector_2 &v) const { return Vector_2(_x - v._x, _y - v._y); }
  Vector_2 operator-() const { return Vector_2(-_x, -_y); }
  double operator*(const Vector_2 &v) const { return _x * v._x + _y * v._y; } // dot product
  Vector_2 operator*(double s) const { return Vector_2(_x * s, _y * s); }
  Vector_2 operator/(double s) const { return Vector_2(_x / s, _y / s); }
  bool operator==(const Vector_2 &v) const { return _x == v._x && _y == v._y; }
  bool operator!=(const Vector_2 &v) const { return !(*this == v); }
  // Rotated a quarter turn: counterclockwise for COUNTERCLOCKWISE, clockwise for CLOCKWISE.
  Vector_2 perpendicular(Orientation o) const {
    return o == COUNTERCLOCKWISE ? Vector_2(-_y, _x) : Vector_2(_y, -_x);
  }
  Direction_2 direction() const;

private:
  double _x, _y;
};

inline Vector_2 operator*(double s, const Vector_2 &v) { return v * s; }

class Direction_2 {
public:
  Direction_2() : _dx(1), _dy(0) {}
  Direction_2(double dx, double dy) : _dx(dx), _dy(dy) {}
  explicit Direction_2(const Vector_2 &v) : _dx(v.x()), _dy(v.y()) {}
  double dx() const { return _dx; }
  double dy() const { return _dy; }
  double delta(int i) const { return i == 0 ? _dx : _dy; }
  Vector_2 vector() const { return Vector_2(_dx, _dy); }
  Vector_2 to_vector() const { return vector(); }
  Direction_2 operator-() const { return Direction_2(-_dx, -_dy); }
  Direction_2 perpendicular(Orientation o) const {
    return Direction_2(vector().perpendicular(o));
  }
  // Two directions are equal when one is a positive multiple of the other.
  bool operator==(const Direction_2 &d) const {
    return exact_orient2d(0, 0, _dx, _dy, d._dx, d._dy) == 0 && (_dx * d._dx + _dy * d._dy) > 0;
  }
  bool operator!=(const Direction_2 &d) const { return !(*this == d); }
  // Ordered by angle counterclockwise from the positive x axis, in [0, 2 pi).
  bool operator<(const Direction_2 &d) const { return compare_angle(d) == SMALLER; }
  bool operator>(const Direction_2 &d) const { return d < *this; }
  bool operator<=(const Direction_2 &d) const { return !(d < *this); }
  bool operator>=(const Direction_2 &d) const { return !(*this < d); }
  // Is this direction strictly between d1 and d2, turning counterclockwise from d1 to d2? Every
  // direction but d1 is between d1 and itself.
  bool counterclockwise_in_between(const Direction_2 &d1, const Direction_2 &d2) const {
    if (d1 < *this)
      return (*this < d2) || (d2 <= d1);
    return (*this < d2) && (d2 <= d1);
  }

private:
  // 0 for angles in [0, pi), 1 for [pi, 2 pi).
  int half() const { return (_dy < 0 || (_dy == 0 && _dx < 0)) ? 1 : 0; }
  Comparison_result compare_angle(const Direction_2 &d) const {
    const int h1 = half(), h2 = d.half();
    if (h1 != h2)
      return h1 < h2 ? SMALLER : LARGER;
    const double o = exact_orient2d(0, 0, _dx, _dy, d._dx, d._dy);
    if (o > 0)
      return SMALLER;
    if (o < 0)
      return LARGER;
    return EQUAL;
  }

  double _dx, _dy;
};

inline Direction_2 Vector_2::direction() const { return Direction_2(_x, _y); }

class Vector_3 {
public:
  Vector_3() : _v{0, 0, 0} {}
  Vector_3(double x, double y, double z) : _v{x, y, z} {}
  Vector_3(Origin) : _v{0, 0, 0} {}
  double x() const { return _v[0]; }
  double y() const { return _v[1]; }
  double z() const { return _v[2]; }
  double operator[](int i) const { return _v[i]; }
  double squared_length() const { return _v[0] * _v[0] + _v[1] * _v[1] + _v[2] * _v[2]; }
  Vector_3 operator+(const Vector_3 &v) const {
    return Vector_3(_v[0] + v._v[0], _v[1] + v._v[1], _v[2] + v._v[2]);
  }
  Vector_3 operator-(const Vector_3 &v) const {
    return Vector_3(_v[0] - v._v[0], _v[1] - v._v[1], _v[2] - v._v[2]);
  }
  Vector_3 operator-() const { return Vector_3(-_v[0], -_v[1], -_v[2]); }
  double operator*(const Vector_3 &v) const {
    return _v[0] * v._v[0] + _v[1] * v._v[1] + _v[2] * v._v[2];
  }
  Vector_3 operator*(double s) const { return Vector_3(_v[0] * s, _v[1] * s, _v[2] * s); }
  Vector_3 operator/(double s) const { return Vector_3(_v[0] / s, _v[1] / s, _v[2] / s); }
  bool operator==(const Vector_3 &v) const {
    return _v[0] == v._v[0] && _v[1] == v._v[1] && _v[2] == v._v[2];
  }
  bool operator!=(const Vector_3 &v) const { return !(*this == v); }

private:
  double _v[3];
};

inline Vector_3 operator*(double s, const Vector_3 &v) { return v * s; }

inline Vector_3 cross_product(const Vector_3 &a, const Vector_3 &b) {
  return Vector_3(a.y() * b.z() - a.z() * b.y(), a.z() * b.x() - a.x() * b.z(),
                  a.x() * b.y() - a.y() * b.x());
}

// ============================================================================================
// Points

class Point_3;

// The legacy Point_25_: a planar point that also carries the z of its slice and an id.
class Point_2 {
public:
  Point_2() : _id(DEFAULT_ID()) { _v[0] = _v[1] = _v[2] = 0; }
  Point_2(Origin) : _id(DEFAULT_ID()) { _v[0] = _v[1] = _v[2] = 0; }
  Point_2(Number_type x, Number_type y, Number_type z = 0) : _id(DEFAULT_ID()) {
    _v[0] = x;
    _v[1] = y;
    _v[2] = z;
  }
  Point_2(Number_type x, Number_type y, Number_type z, std::size_t id) : _id(id) {
    _v[0] = x;
    _v[1] = y;
    _v[2] = z;
  }

  const Number_type &x() const { return _v[0]; }
  const Number_type &y() const { return _v[1]; }
  const Number_type &z() const { return _v[2]; }
  const std::size_t &id() const { return _id; }
  const std::size_t &unique_id() const { return _id; }
  Number_type &x() { return _v[0]; }
  Number_type &y() { return _v[1]; }
  Number_type &z() { return _v[2]; }
  std::size_t &id() { return _id; }
  const Number_type &hx() const { return x(); }
  const Number_type &hy() const { return y(); }
  const Number_type &hz() const { return z(); }
  Number_type hw() const { return 1; }
  Number_type cartesian(int i) const { return _v[i]; }
  Number_type operator[](int i) const { return _v[i]; }
  static std::size_t default_id() { return DEFAULT_ID(); }

  Point_2 &point() { return *this; }
  const Point_2 &point() const { return *this; }
  // The bare planar point (z 0, no id), as the legacy point_2() gave a CGAL point.
  Point_2 point_2() const { return Point_2(x(), y()); }
  Point_3 point_3() const;
  operator Point_3() const;

  bool operator==(const Point_2 &p) const { return x() == p.x() && y() == p.y(); }
  bool operator!=(const Point_2 &p) const { return !(*this == p); }
  // Lexicographic in x, then y.
  bool operator<(const Point_2 &p) const { return x() < p.x() || (x() == p.x() && y() < p.y()); }
  bool operator>(const Point_2 &p) const { return p < *this; }

  Vector_2 operator-(const Point_2 &q) const { return Vector_2(x() - q.x(), y() - q.y()); }
  Point_2 operator+(const Vector_2 &v) const { return Point_2(x() + v.x(), y() + v.y(), z(), id()); }
  Point_2 operator-(const Vector_2 &v) const { return Point_2(x() - v.x(), y() - v.y(), z(), id()); }

  bool is_valid() const { return id() != DEFAULT_ID(); }
  Bbox_2 bbox() const { return Bbox_2(x(), y(), x(), y()); }

  friend std::ostream &operator<<(std::ostream &os, const Point_2 &p) {
    return os << p.x() << ' ' << p.y() << ' ' << p.z();
  }

private:
  Number_type _v[3];
  std::size_t _id;
};

// The legacy Point_3_id: the same data, compared in x, y and z.
class Point_3 : public Point_2 {
public:
  Point_3() : Point_2() {}
  Point_3(Origin o) : Point_2(o) {}
  Point_3(Number_type x, Number_type y, Number_type z = 0) : Point_2(x, y, z) {}
  Point_3(Number_type x, Number_type y, Number_type z, std::size_t id) : Point_2(x, y, z, id) {}

  Point_3 &point() { return *this; }
  const Point_3 &point() const { return *this; }
  Point_2 point_2() const { return Point_2(x(), y(), z(), id()); }
  Point_3 point_3() const { return *this; }

  bool operator==(const Point_3 &p) const { return x() == p.x() && y() == p.y() && z() == p.z(); }
  bool operator!=(const Point_3 &p) const { return !(*this == p); }
  // Lexicographic in x, y, then z.
  bool operator<(const Point_3 &p) const {
    if (x() != p.x())
      return x() < p.x();
    if (y() != p.y())
      return y() < p.y();
    return z() < p.z();
  }
  bool operator>(const Point_3 &p) const { return p < *this; }

  Vector_3 operator-(const Point_3 &q) const {
    return Vector_3(x() - q.x(), y() - q.y(), z() - q.z());
  }
  Point_3 operator+(const Vector_3 &v) const {
    return Point_3(x() + v.x(), y() + v.y(), z() + v.z(), id());
  }
  Point_3 operator-(const Vector_3 &v) const {
    return Point_3(x() - v.x(), y() - v.y(), z() - v.z(), id());
  }
  Bbox_3 bbox() const { return Bbox_3(x(), y(), z(), x(), y(), z()); }
};

inline Point_3 Point_2::point_3() const { return Point_3(x(), y(), z(), id()); }
inline Point_2::operator Point_3() const { return point_3(); }

inline Vector_3 operator-(const Point_3 &p, Origin) { return Vector_3(p.x(), p.y(), p.z()); }
inline Point_3 operator+(Origin, const Vector_3 &v) { return Point_3(v.x(), v.y(), v.z()); }

inline std::size_t hash_value(const Point_2 &p) {
  std::size_t seed = 0;
  boost::hash_combine(seed, p.x());
  boost::hash_combine(seed, p.y());
  boost::hash_combine(seed, p.z());
  return seed;
}
inline std::size_t hash_value(const Point_3 &p) {
  std::size_t seed = 0;
  boost::hash_combine(seed, p.x());
  boost::hash_combine(seed, p.y());
  boost::hash_combine(seed, p.z());
  return seed;
}

inline bool xy_equal(const Point_2 &a, const Point_2 &b) { return a.x() == b.x() && a.y() == b.y(); }
inline bool xyz_equal(const Point_2 &a, const Point_2 &b) {
  return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}

// ============================================================================================
// Predicates and constructions on points

inline Orientation orientation(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return sign_of(exact_orient2d(p.x(), p.y(), q.x(), q.y(), r.x(), r.y()));
}
inline bool collinear(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return orientation(p, q, r) == COLLINEAR;
}
inline bool left_turn(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return orientation(p, q, r) == LEFT_TURN;
}
inline bool right_turn(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return orientation(p, q, r) == RIGHT_TURN;
}

// 3D: positive when s is on the positive side of the plane through p, q, r (p, q, r turning
// counterclockwise seen from it).
inline Orientation orientation(const Point_3 &p, const Point_3 &q, const Point_3 &r,
                               const Point_3 &s) {
  const double a[3] = {p.x(), p.y(), p.z()}, b[3] = {q.x(), q.y(), q.z()},
               c[3] = {r.x(), r.y(), r.z()}, d[3] = {s.x(), s.y(), s.z()};
  // Shewchuk's orient3d is positive when d is below the plane of a, b, c (abc counterclockwise
  // seen from above): the opposite of CGAL's sign.
  return opposite(sign_of(exact_orient3d(a, b, c, d)));
}
inline bool coplanar(const Point_3 &p, const Point_3 &q, const Point_3 &r, const Point_3 &s) {
  return orientation(p, q, r, s) == COPLANAR;
}
// Three points in space are collinear when their projections on the three coordinate planes are.
inline bool collinear(const Point_3 &p, const Point_3 &q, const Point_3 &r) {
  return exact_orient2d(p.x(), p.y(), q.x(), q.y(), r.x(), r.y()) == 0 &&
         exact_orient2d(p.y(), p.z(), q.y(), q.z(), r.y(), r.z()) == 0 &&
         exact_orient2d(p.x(), p.z(), q.x(), q.z(), r.x(), r.z()) == 0;
}
// For coplanar p, q, r, s with p, q, r not collinear: POSITIVE when r and s are on the same side
// of the line pq, NEGATIVE on opposite sides, COLLINEAR when s is on it.
Orientation coplanar_orientation(const Point_3 &p, const Point_3 &q, const Point_3 &r,
                                 const Point_3 &s);

inline bool lexicographically_xy_smaller(const Point_2 &p, const Point_2 &q) {
  return p.x() < q.x() || (p.x() == q.x() && p.y() < q.y());
}
inline bool lexicographically_xy_smaller_or_equal(const Point_2 &p, const Point_2 &q) {
  return !lexicographically_xy_smaller(q, p);
}
inline bool lexicographically_xyz_smaller(const Point_2 &p, const Point_2 &q) {
  if (p.x() != q.x())
    return p.x() < q.x();
  if (p.y() != q.y())
    return p.y() < q.y();
  return p.z() < q.z();
}
inline Comparison_result compare_xy(const Point_2 &p, const Point_2 &q) {
  if (lexicographically_xy_smaller(p, q))
    return SMALLER;
  if (lexicographically_xy_smaller(q, p))
    return LARGER;
  return EQUAL;
}

inline Number_type squared_distance(const Point_2 &p, const Point_2 &q) {
  const double dx = p.x() - q.x(), dy = p.y() - q.y();
  return dx * dx + dy * dy;
}
inline Number_type squared_distance(const Point_3 &p, const Point_3 &q) {
  const double dx = p.x() - q.x(), dy = p.y() - q.y(), dz = p.z() - q.z();
  return dx * dx + dy * dy + dz * dz;
}
inline Comparison_result compare_distance_to_point(const Point_2 &p, const Point_2 &q,
                                                   const Point_2 &r) {
  return sign_of(squared_distance(p, q) - squared_distance(p, r));
}
inline bool has_smaller_distance_to_point(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return squared_distance(p, q) < squared_distance(p, r);
}
inline bool has_larger_distance_to_point(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return squared_distance(p, q) > squared_distance(p, r);
}
inline Comparison_result compare_squared_distance(const Point_2 &p, const Point_2 &q,
                                                  Number_type d2) {
  return sign_of(squared_distance(p, q) - d2);
}

// The sign of the angle at q: ACUTE, RIGHT or OBTUSE.
inline Angle angle(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  const double d = (p.x() - q.x()) * (r.x() - q.x()) + (p.y() - q.y()) * (r.y() - q.y());
  return d > 0 ? ACUTE : (d < 0 ? OBTUSE : RIGHT);
}

// For collinear p, q, r: is q on the closed segment [p, r]? (strictly between, for the second)
inline bool collinear_are_ordered_along_line(const Point_2 &p, const Point_2 &q,
                                             const Point_2 &r) {
  if (p.x() < q.x())
    return !(r.x() < q.x());
  if (q.x() < p.x())
    return !(q.x() < r.x());
  if (p.y() < q.y())
    return !(r.y() < q.y());
  if (q.y() < p.y())
    return !(q.y() < r.y());
  return true; // p == q
}
inline bool collinear_are_strictly_ordered_along_line(const Point_2 &p, const Point_2 &q,
                                                      const Point_2 &r) {
  if (p.x() < q.x())
    return q.x() < r.x();
  if (q.x() < p.x())
    return r.x() < q.x();
  if (p.y() < q.y())
    return q.y() < r.y();
  if (q.y() < p.y())
    return r.y() < q.y();
  return false;
}
inline bool are_ordered_along_line(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return collinear(p, q, r) && collinear_are_ordered_along_line(p, q, r);
}
// The same for points collinear in space: q on the closed segment [p, r].
inline bool collinear_are_ordered_along_line(const Point_3 &p, const Point_3 &q, const Point_3 &r) {
  for (int i = 0; i < 3; ++i) {
    if (p[i] < q[i])
      return !(r[i] < q[i]);
    if (q[i] < p[i])
      return !(q[i] < r[i]);
  }
  return true; // p == q
}

inline Point_2 midpoint(const Point_2 &p, const Point_2 &q) {
  return Point_2((p.x() + q.x()) / 2, (p.y() + q.y()) / 2);
}
inline Point_3 midpoint(const Point_3 &p, const Point_3 &q) {
  return Point_3((p.x() + q.x()) / 2, (p.y() + q.y()) / 2, (p.z() + q.z()) / 2);
}
inline Point_2 centroid(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return Point_2((p.x() + q.x() + r.x()) / 3, (p.y() + q.y() + r.y()) / 3);
}
inline Point_3 centroid(const Point_3 &p, const Point_3 &q, const Point_3 &r) {
  return Point_3((p.x() + q.x() + r.x()) / 3, (p.y() + q.y() + r.y()) / 3,
                 (p.z() + q.z() + r.z()) / 3);
}

// Twice-signed area / 2 of the triangle pqr: ((q - p) x (r - p)) / 2.
inline Number_type area(const Point_2 &p, const Point_2 &q, const Point_2 &r) {
  return ((q.x() - p.x()) * (r.y() - p.y()) - (r.x() - p.x()) * (q.y() - p.y())) / 2;
}

// ============================================================================================
// Lines, segments, rays and triangles

class Line_2;

class Segment_2 {
public:
  Segment_2() {}
  Segment_2(const Point_2 &s, const Point_2 &t) : _s(s), _t(t) {}
  const Point_2 &source() const { return _s; }
  const Point_2 &target() const { return _t; }
  const Point_2 &start() const { return _s; }
  const Point_2 &end() const { return _t; }
  const Point_2 &vertex(int i) const { return (i % 2 == 0) ? _s : _t; }
  const Point_2 &point(int i) const { return vertex(i); }
  const Point_2 &operator[](int i) const { return vertex(i); }
  // Lexicographically smaller and larger endpoints.
  const Point_2 &min() const { return lexicographically_xy_smaller(_t, _s) ? _t : _s; }
  const Point_2 &max() const { return lexicographically_xy_smaller(_t, _s) ? _s : _t; }
  Number_type squared_length() const { return squared_distance(_s, _t); }
  Vector_2 to_vector() const { return _t - _s; }
  Direction_2 direction() const { return Direction_2(to_vector()); }
  Line_2 supporting_line() const;
  Segment_2 opposite() const { return Segment_2(_t, _s); }
  bool is_degenerate() const { return _s == _t; }
  bool is_horizontal() const { return _s.y() == _t.y(); }
  bool is_vertical() const { return _s.x() == _t.x(); }
  bool has_on(const Point_2 &p) const { return are_ordered_along_line(_s, p, _t); }
  bool collinear_has_on(const Point_2 &p) const {
    return collinear_are_ordered_along_line(_s, p, _t);
  }
  Bbox_2 bbox() const { return _s.bbox() + _t.bbox(); }
  bool operator==(const Segment_2 &o) const { return _s == o._s && _t == o._t; }
  bool operator!=(const Segment_2 &o) const { return !(*this == o); }
  friend std::ostream &operator<<(std::ostream &os, const Segment_2 &s) {
    return os << s.source() << ' ' << s.target();
  }

private:
  Point_2 _s, _t;
};

// a x + b y + c = 0, oriented: the positive side is on the left of its direction (b, -a).
class Line_2 {
public:
  Line_2() : _a(0), _b(0), _c(0) {}
  Line_2(double a, double b, double c) : _a(a), _b(b), _c(c) {}
  Line_2(const Point_2 &p, const Point_2 &q) { from_points(p.x(), p.y(), q.x(), q.y()); }
  Line_2(const Point_2 &p, const Direction_2 &d) {
    from_points(p.x(), p.y(), p.x() + d.dx(), p.y() + d.dy());
  }
  Line_2(const Point_2 &p, const Vector_2 &v) {
    from_points(p.x(), p.y(), p.x() + v.x(), p.y() + v.y());
  }
  explicit Line_2(const Segment_2 &s) : Line_2(s.source(), s.target()) {}
  double a() const { return _a; }
  double b() const { return _b; }
  double c() const { return _c; }
  Vector_2 to_vector() const { return Vector_2(_b, -_a); }
  Direction_2 direction() const { return Direction_2(_b, -_a); }
  Line_2 opposite() const { return Line_2(-_a, -_b, -_c); }
  bool is_degenerate() const { return _a == 0 && _b == 0; }
  bool is_horizontal() const { return _a == 0; }
  bool is_vertical() const { return _b == 0; }
  double x_at_y(double y) const { return (-_b * y - _c) / _a; }
  double y_at_x(double x) const { return (-_a * x - _c) / _b; }
  // A point on the line, and further points along it.
  Point_2 point(int i = 0) const;
  Point_2 point(double i) const { return point(int(i)); }
  Oriented_side oriented_side(const Point_2 &p) const {
    return sign_of(_a * p.x() + _b * p.y() + _c);
  }
  bool has_on(const Point_2 &p) const { return oriented_side(p) == ON_ORIENTED_BOUNDARY; }
  bool has_on_boundary(const Point_2 &p) const { return has_on(p); }
  bool has_on_positive_side(const Point_2 &p) const { return oriented_side(p) == ON_POSITIVE_SIDE; }
  bool has_on_negative_side(const Point_2 &p) const { return oriented_side(p) == ON_NEGATIVE_SIDE; }
  // The orthogonal projection of p onto the line.
  Point_2 projection(const Point_2 &p) const {
    const double a2b2 = _a * _a + _b * _b;
    const double x = (_b * _b * p.x() - _a * _b * p.y() - _a * _c) / a2b2;
    const double y = (_a * _a * p.y() - _a * _b * p.x() - _b * _c) / a2b2;
    return Point_2(x, y);
  }
  // The line through p perpendicular to this one, turned a quarter counterclockwise from it.
  Line_2 perpendicular(const Point_2 &p) const { return Line_2(-_b, _a, _b * p.x() - _a * p.y()); }

private:
  void from_points(double px, double py, double qx, double qy) {
    if (py == qy) {
      _a = 0;
      if (qx > px) {
        _b = 1;
        _c = -py;
      } else if (qx == px) {
        _b = 0;
        _c = 0;
      } else {
        _b = -1;
        _c = py;
      }
    } else if (qx == px) {
      _b = 0;
      if (qy > py) {
        _a = -1;
        _c = px;
      } else {
        _a = 1;
        _c = -px;
      }
    } else {
      _a = py - qy;
      _b = qx - px;
      _c = -px * _a - py * _b;
    }
  }
  double _a, _b, _c;
};

inline Point_2 Line_2::point(int i) const {
  Point_2 p0;
  if (_b == 0)
    p0 = Point_2(-_c / _a, 0);
  else
    p0 = Point_2(0, -_c / _b);
  return Point_2(p0.x() + i * _b, p0.y() - i * _a);
}

inline Line_2 Segment_2::supporting_line() const { return Line_2(_s, _t); }

class Ray_2 {
public:
  Ray_2() {}
  Ray_2(const Point_2 &s, const Point_2 &second) : _s(s), _second(second) {}
  Ray_2(const Point_2 &s, const Direction_2 &d) : _s(s), _second(s.x() + d.dx(), s.y() + d.dy()) {}
  Ray_2(const Point_2 &s, const Vector_2 &v) : _s(s), _second(s.x() + v.x(), s.y() + v.y()) {}
  const Point_2 &source() const { return _s; }
  const Point_2 &start() const { return _s; }
  const Point_2 &second_point() const { return _second; }
  Point_2 point(int i) const {
    if (i == 0)
      return _s;
    if (i == 1)
      return _second;
    return Point_2(_s.x() + i * (_second.x() - _s.x()), _s.y() + i * (_second.y() - _s.y()));
  }
  Vector_2 to_vector() const { return _second - _s; }
  Direction_2 direction() const { return Direction_2(to_vector()); }
  Line_2 supporting_line() const { return Line_2(_s, _second); }
  Ray_2 opposite() const { return Ray_2(_s, Point_2(2 * _s.x() - _second.x(), 2 * _s.y() - _second.y())); }
  bool is_degenerate() const { return _s == _second; }
  // p on the ray: collinear with it and not behind its source.
  bool has_on(const Point_2 &p) const {
    if (p == _s)
      return true;
    if (!collinear(_s, _second, p))
      return false;
    return (p - _s) * (_second - _s) > 0;
  }
  bool collinear_has_on(const Point_2 &p) const { return p == _s || (p - _s) * (_second - _s) > 0; }
  // The same source and direction.
  bool operator==(const Ray_2 &r) const { return _s == r._s && direction() == r.direction(); }
  bool operator!=(const Ray_2 &r) const { return !(*this == r); }

private:
  Point_2 _s, _second;
};

class Triangle_2 {
public:
  Triangle_2() {}
  Triangle_2(const Point_2 &p, const Point_2 &q, const Point_2 &r) : _v{p, q, r} {}
  const Point_2 &vertex(int i) const { return _v[((i % 3) + 3) % 3]; }
  const Point_2 &operator[](int i) const { return vertex(i); }
  Orientation orientation() const { return tiler::orientation(_v[0], _v[1], _v[2]); }
  bool is_degenerate() const { return orientation() == COLLINEAR; }
  Number_type area() const { return tiler::area(_v[0], _v[1], _v[2]); }
  Bounded_side bounded_side(const Point_2 &p) const {
    const Orientation o = orientation();
    const Orientation o1 = tiler::orientation(_v[0], _v[1], p), o2 = tiler::orientation(_v[1], _v[2], p),
                      o3 = tiler::orientation(_v[2], _v[0], p);
    if (o == COLLINEAR)
      return ON_UNBOUNDED_SIDE;
    if (o1 == tiler::opposite(o) || o2 == tiler::opposite(o) || o3 == tiler::opposite(o))
      return ON_UNBOUNDED_SIDE;
    if (o1 == COLLINEAR || o2 == COLLINEAR || o3 == COLLINEAR)
      return ON_BOUNDARY;
    return ON_BOUNDED_SIDE;
  }
  bool has_on_bounded_side(const Point_2 &p) const { return bounded_side(p) == ON_BOUNDED_SIDE; }
  bool has_on_boundary(const Point_2 &p) const { return bounded_side(p) == ON_BOUNDARY; }
  bool has_on_unbounded_side(const Point_2 &p) const {
    return bounded_side(p) == ON_UNBOUNDED_SIDE;
  }
  // Inside a counterclockwise triangle is its positive side; inside a clockwise one, its negative.
  Oriented_side oriented_side(const Point_2 &p) const {
    const Bounded_side b = bounded_side(p);
    if (b == ON_BOUNDARY)
      return ON_ORIENTED_BOUNDARY;
    const Orientation o = orientation();
    if (o == COLLINEAR)
      return ON_ORIENTED_BOUNDARY;
    return ((b == ON_BOUNDED_SIDE) == (o == COUNTERCLOCKWISE)) ? ON_POSITIVE_SIDE : ON_NEGATIVE_SIDE;
  }
  bool has_on_positive_side(const Point_2 &p) const { return oriented_side(p) == ON_POSITIVE_SIDE; }
  bool has_on_negative_side(const Point_2 &p) const { return oriented_side(p) == ON_NEGATIVE_SIDE; }
  Triangle_2 opposite() const { return Triangle_2(_v[0], _v[2], _v[1]); }
  Bbox_2 bbox() const { return _v[0].bbox() + _v[1].bbox() + _v[2].bbox(); }

private:
  Point_2 _v[3];
};

class Segment_3 {
public:
  Segment_3() {}
  Segment_3(const Point_3 &s, const Point_3 &t) : _s(s), _t(t) {}
  const Point_3 &source() const { return _s; }
  const Point_3 &target() const { return _t; }
  const Point_3 &start() const { return _s; }
  const Point_3 &end() const { return _t; }
  const Point_3 &vertex(int i) const { return (i % 2 == 0) ? _s : _t; }
  const Point_3 &point(int i) const { return vertex(i); }
  const Point_3 &operator[](int i) const { return vertex(i); }
  const Point_3 &min() const { return (_t < _s) ? _t : _s; }
  const Point_3 &max() const { return (_t < _s) ? _s : _t; }
  Number_type squared_length() const { return squared_distance(_s, _t); }
  Vector_3 to_vector() const { return _t - _s; }
  Segment_3 opposite() const { return Segment_3(_t, _s); }
  bool is_degenerate() const { return _s == _t; }
  // p on the closed segment: collinear with it and between its ends.
  bool has_on(const Point_3 &p) const {
    if (!collinear(_s, p, _t))
      return false;
    return (p - _s) * (_t - _s) >= 0 && (p - _t) * (_s - _t) >= 0;
  }
  Bbox_3 bbox() const { return _s.bbox() + _t.bbox(); }
  bool operator==(const Segment_3 &o) const { return _s == o._s && _t == o._t; }
  bool operator!=(const Segment_3 &o) const { return !(*this == o); }
  friend std::ostream &operator<<(std::ostream &os, const Segment_3 &s) {
    return os << s.source() << ' ' << s.target();
  }

private:
  Point_3 _s, _t;
};

// A line in space through a point, along a vector.
class Line_3 {
public:
  Line_3() {}
  Line_3(const Point_3 &p, const Point_3 &q) : _p(p), _v(q - p) {}
  Line_3(const Point_3 &p, const Vector_3 &v) : _p(p), _v(v) {}
  explicit Line_3(const Segment_3 &s) : _p(s.source()), _v(s.to_vector()) {}
  Point_3 point() const { return _p; }
  Point_3 point(double t) const { return Point_3(_p.x() + t * _v.x(), _p.y() + t * _v.y(), _p.z() + t * _v.z()); }
  Vector_3 to_vector() const { return _v; }
  bool has_on(const Point_3 &q) const { return collinear(_p, point(1.0), q); }

private:
  Point_3 _p;
  Vector_3 _v;
};

inline std::size_t hash_value(const Segment_2 &s) {
  std::size_t seed = 0;
  boost::hash_combine(seed, hash_value(s.source()));
  boost::hash_combine(seed, hash_value(s.target()));
  return seed;
}
inline std::size_t hash_value(const Segment_3 &s) {
  std::size_t seed = 0;
  boost::hash_combine(seed, hash_value(s.source()));
  boost::hash_combine(seed, hash_value(s.target()));
  return seed;
}

// ============================================================================================
// Distances to lines and segments

inline Number_type squared_distance(const Point_2 &p, const Line_2 &l) {
  const double d = l.a() * p.x() + l.b() * p.y() + l.c();
  return d * d / (l.a() * l.a() + l.b() * l.b());
}
inline Number_type squared_distance(const Line_2 &l, const Point_2 &p) { return squared_distance(p, l); }

inline Number_type squared_distance(const Point_2 &p, const Segment_2 &s) {
  const Vector_2 d = s.target() - s.source();
  const Vector_2 w = p - s.source();
  const double t = w * d;
  if (t <= 0)
    return squared_distance(p, s.source());
  const double dd = d * d;
  if (t >= dd)
    return squared_distance(p, s.target());
  const double cr = d.x() * w.y() - d.y() * w.x();
  return cr * cr / dd;
}
inline Number_type squared_distance(const Segment_2 &s, const Point_2 &p) { return squared_distance(p, s); }

inline Number_type squared_distance(const Point_3 &p, const Segment_3 &s) {
  const Vector_3 d = s.target() - s.source();
  const Vector_3 w = p - s.source();
  const double t = w * d;
  if (t <= 0)
    return squared_distance(p, s.source());
  const double dd = d * d;
  if (t >= dd)
    return squared_distance(p, s.target());
  return cross_product(d, w).squared_length() / dd;
}

// ============================================================================================
// Intersections

// What an intersection returns: nothing, a point or a segment (CGAL::Object's interface).
class Object {
public:
  Object() = default;
  template <class T> explicit Object(T t) : _v(std::move(t)) {}
  bool empty() const { return !_v.has_value(); }
  bool is_empty() const { return empty(); }
  template <class T> bool is() const { return std::any_cast<T>(&_v) != nullptr; }
  template <class T> const T *cast() const { return std::any_cast<T>(&_v); }

private:
  std::any _v;
};

template <class T> Object make_object(const T &t) { return Object(t); }
template <class T> bool assign(T &t, const Object &o) {
  if (const T *p = o.cast<T>()) {
    t = *p;
    return true;
  }
  return false;
}
template <class T> const T *object_cast(const Object *o) { return o ? o->cast<T>() : nullptr; }

// Do the closed segments share a point? Exact.
bool do_intersect(const Segment_2 &a, const Segment_2 &b);
inline bool do_intersect(const Point_2 &p, const Segment_2 &s) { return s.has_on(p); }
inline bool do_intersect(const Segment_2 &s, const Point_2 &p) { return s.has_on(p); }
bool do_intersect(const Ray_2 &r, const Segment_2 &s);
inline bool do_intersect(const Segment_2 &s, const Ray_2 &r) { return do_intersect(r, s); }

// Empty, a Point_2 or a Segment_2. A point where the segments meet at an endpoint is that
// endpoint (with its z and id); a point inside both is constructed (z 0, no id). An overlap of
// collinear segments is a Segment_2 between input endpoints.
Object intersection(const Segment_2 &a, const Segment_2 &b);
// Empty, a Point_2 or a Line_2.
Object intersection(const Line_2 &a, const Line_2 &b);
// Empty, a Point_2 or a Segment_2.
Object intersection(const Segment_2 &s, const Line_2 &l);
inline Object intersection(const Line_2 &l, const Segment_2 &s) { return intersection(s, l); }
Object intersection(const Ray_2 &r, const Segment_2 &s);
inline Object intersection(const Segment_2 &s, const Ray_2 &r) { return intersection(r, s); }

// ============================================================================================
// Polygons

// A polygon's vertices in a random-access container; the last vertex joins the first.
class Polygon_2 {
public:
  typedef Point_2 Point_type;
  typedef std::vector<Point_2> Container;
  // As CGAL's Polygon_2 has them: both iterator types are the container's mutable iterator, even
  // from a const polygon (the vendored code writes a vertex's z and id through them).
  typedef Container::iterator Vertex_iterator;
  typedef Vertex_iterator Vertex_const_iterator;
  typedef Vertex_iterator const_iterator;
  typedef Vertex_iterator iterator;

  // A circulator over a container: ++ past the end goes to the start, -- before the start to the
  // end. Compares by position.
  template <class Cont, class It> class circulator {
  public:
    typedef std::bidirectional_iterator_tag iterator_category;
    typedef typename std::iterator_traits<It>::value_type value_type;
    typedef std::ptrdiff_t difference_type;
    typedef typename std::iterator_traits<It>::pointer pointer;
    typedef typename std::iterator_traits<It>::reference reference;
    circulator() : _c(nullptr) {}
    circulator(Cont *c, It it) : _c(c), _it(it) {}
    // A mutable circulator converts to a const one.
    template <class C2, class I2,
              class = std::enable_if_t<std::is_convertible_v<C2 *, Cont *> &&
                                       std::is_convertible_v<I2, It>>>
    circulator(const circulator<C2, I2> &o) : _c(o.container()), _it(o.current_iterator()) {}
    reference operator*() const { return *_it; }
    pointer operator->() const { return &*_it; }
    circulator &operator++() {
      if (++_it == _c->end())
        _it = _c->begin();
      return *this;
    }
    circulator operator++(int) {
      circulator t = *this;
      ++*this;
      return t;
    }
    circulator &operator--() {
      if (_it == _c->begin())
        _it = _c->end();
      --_it;
      return *this;
    }
    circulator operator--(int) {
      circulator t = *this;
      --*this;
      return t;
    }
    circulator operator+(difference_type n) const {
      circulator t = *this;
      if (n >= 0)
        while (n--)
          ++t;
      else
        while (n++)
          --t;
      return t;
    }
    circulator operator-(difference_type n) const { return *this + (-n); }
    // Positions apart in the container (not taken modulo its size).
    difference_type operator-(const circulator &o) const { return _it - o._it; }
    bool operator==(const circulator &o) const { return _it == o._it; }
    bool operator!=(const circulator &o) const { return _it != o._it; }
    bool operator==(std::nullptr_t) const { return _c == nullptr || _c->empty(); }
    bool operator!=(std::nullptr_t) const { return !(*this == nullptr); }
    It current_iterator() const { return _it; }
    Cont *container() const { return _c; }

  private:
    Cont *_c;
    It _it;
  };
  typedef circulator<Container, Vertex_iterator> Vertex_circulator;
  typedef Vertex_circulator Vertex_const_circulator;

  // The edges in order: edge i runs from vertex i to vertex i + 1 (the last to the first).
  class Edge_const_iterator {
  public:
    typedef std::random_access_iterator_tag iterator_category;
    typedef Segment_2 value_type;
    typedef std::ptrdiff_t difference_type;
    typedef const Segment_2 *pointer;
    typedef Segment_2 reference;
    Edge_const_iterator() : _c(nullptr), _i(0) {}
    Edge_const_iterator(const Container *c, std::size_t i) : _c(c), _i(i) {}
    Segment_2 operator*() const {
      return Segment_2((*_c)[_i], (*_c)[_i + 1 == _c->size() ? 0 : _i + 1]);
    }
    struct arrow {
      Segment_2 s;
      const Segment_2 *operator->() const { return &s; }
    };
    arrow operator->() const { return arrow{**this}; }
    Edge_const_iterator &operator++() {
      ++_i;
      return *this;
    }
    Edge_const_iterator operator++(int) {
      Edge_const_iterator t = *this;
      ++_i;
      return t;
    }
    Edge_const_iterator &operator--() {
      --_i;
      return *this;
    }
    Edge_const_iterator operator--(int) {
      Edge_const_iterator t = *this;
      --_i;
      return t;
    }
    Edge_const_iterator &operator+=(difference_type n) {
      _i += n;
      return *this;
    }
    Edge_const_iterator operator+(difference_type n) const { return Edge_const_iterator(_c, _i + n); }
    Edge_const_iterator operator-(difference_type n) const { return Edge_const_iterator(_c, _i - n); }
    difference_type operator-(const Edge_const_iterator &o) const {
      return difference_type(_i) - difference_type(o._i);
    }
    Segment_2 operator[](difference_type n) const { return *(*this + n); }
    bool operator==(const Edge_const_iterator &o) const { return _i == o._i; }
    bool operator!=(const Edge_const_iterator &o) const { return _i != o._i; }
    bool operator<(const Edge_const_iterator &o) const { return _i < o._i; }
    std::size_t index() const { return _i; }

  private:
    const Container *_c;
    std::size_t _i;
  };
  typedef Edge_const_iterator Edge_iterator;

  class Edge_const_circulator {
  public:
    typedef std::bidirectional_iterator_tag iterator_category;
    typedef Segment_2 value_type;
    typedef std::ptrdiff_t difference_type;
    typedef const Segment_2 *pointer;
    typedef Segment_2 reference;
    Edge_const_circulator() : _c(nullptr), _i(0) {}
    Edge_const_circulator(const Container *c, std::size_t i) : _c(c), _i(i) {}
    Segment_2 operator*() const {
      return Segment_2((*_c)[_i], (*_c)[_i + 1 == _c->size() ? 0 : _i + 1]);
    }
    Edge_const_iterator::arrow operator->() const { return Edge_const_iterator::arrow{**this}; }
    Edge_const_circulator &operator++() {
      _i = (_i + 1 == _c->size()) ? 0 : _i + 1;
      return *this;
    }
    Edge_const_circulator operator++(int) {
      Edge_const_circulator t = *this;
      ++*this;
      return t;
    }
    Edge_const_circulator &operator--() {
      _i = (_i == 0) ? _c->size() - 1 : _i - 1;
      return *this;
    }
    Edge_const_circulator operator--(int) {
      Edge_const_circulator t = *this;
      --*this;
      return t;
    }
    bool operator==(const Edge_const_circulator &o) const { return _i == o._i; }
    bool operator!=(const Edge_const_circulator &o) const { return _i != o._i; }

  private:
    const Container *_c;
    std::size_t _i;
  };
  typedef Edge_const_circulator Edge_circulator;

  Polygon_2() {}
  template <class It> Polygon_2(It first, It last) : _v(first, last) {}
  explicit Polygon_2(const Container &v) : _v(v) {}

  std::size_t size() const { return _v.size(); }
  bool is_empty() const { return _v.empty(); }
  bool empty() const { return _v.empty(); }
  void clear() { _v.clear(); }
  void push_back(const Point_2 &p) { _v.push_back(p); }
  Vertex_iterator insert(Vertex_iterator pos, const Point_2 &p) { return _v.insert(pos, p); }
  template <class It> void insert(Vertex_iterator pos, It first, It last) { _v.insert(pos, first, last); }
  Vertex_iterator insert(Vertex_circulator pos, const Point_2 &p) {
    return _v.insert(pos.current_iterator(), p);
  }
  Vertex_iterator erase(Vertex_iterator pos) { return _v.erase(pos); }
  Vertex_circulator erase(Vertex_circulator pos) {
    Vertex_iterator it = _v.erase(pos.current_iterator());
    if (it == _v.end())
      it = _v.begin();
    return Vertex_circulator(&_v, it);
  }
  Vertex_iterator erase(Vertex_iterator first, Vertex_iterator last) { return _v.erase(first, last); }
  void set(Vertex_iterator pos, const Point_2 &p) { *pos = p; }
  void set(Vertex_circulator pos, const Point_2 &p) { *pos.current_iterator() = p; }

  Vertex_iterator vertices_begin() const { return _v.begin(); }
  Vertex_iterator vertices_end() const { return _v.end(); }
  Vertex_circulator vertices_circulator() const { return Vertex_circulator(&_v, _v.begin()); }
  Edge_const_iterator edges_begin() const { return Edge_const_iterator(&_v, 0); }
  Edge_const_iterator edges_end() const { return Edge_const_iterator(&_v, _v.size()); }
  Edge_const_circulator edges_circulator() const { return Edge_const_circulator(&_v, 0); }

  const Point_2 &vertex(std::size_t i) const { return _v[i]; }
  const Point_2 &operator[](std::size_t i) const { return _v[i]; }
  Point_2 &operator[](std::size_t i) { return _v[i]; }
  Segment_2 edge(std::size_t i) const { return Segment_2(_v[i], _v[(i + 1) % _v.size()]); }
  Container &container() { return _v; }
  const Container &container() const { return _v; }

  // Predicates (polygon.cpp). Exact.
  bool is_simple() const;
  bool is_convex() const;
  // Of a simple polygon: the turn at its lexicographically smallest vertex.
  Orientation orientation() const;
  bool is_counterclockwise_oriented() const { return orientation() == COUNTERCLOCKWISE; }
  bool is_clockwise_oriented() const { return orientation() == CLOCKWISE; }
  bool is_collinear_oriented() const { return orientation() == COLLINEAR; }
  Bounded_side bounded_side(const Point_2 &p) const;
  Oriented_side oriented_side(const Point_2 &p) const;
  bool has_on_boundary(const Point_2 &p) const { return bounded_side(p) == ON_BOUNDARY; }
  bool has_on_bounded_side(const Point_2 &p) const { return bounded_side(p) == ON_BOUNDED_SIDE; }
  bool has_on_unbounded_side(const Point_2 &p) const {
    return bounded_side(p) == ON_UNBOUNDED_SIDE;
  }
  bool has_on_positive_side(const Point_2 &p) const { return oriented_side(p) == ON_POSITIVE_SIDE; }
  bool has_on_negative_side(const Point_2 &p) const { return oriented_side(p) == ON_NEGATIVE_SIDE; }
  // Signed area, positive when counterclockwise: the fan of triangles from the first vertex.
  Number_type area() const;
  Bbox_2 bbox() const;
  void reverse_orientation() {
    if (_v.size() > 1)
      std::reverse(_v.begin() + 1, _v.end());
  }
  Vertex_const_iterator left_vertex() const;
  Vertex_const_iterator right_vertex() const;
  Vertex_const_iterator top_vertex() const;
  Vertex_const_iterator bottom_vertex() const;

  bool operator==(const Polygon_2 &o) const;
  bool operator!=(const Polygon_2 &o) const { return !(*this == o); }

private:
  mutable Container _v;
};

inline Number_type polygon_area_2(Polygon_2::Vertex_const_iterator b,
                                  Polygon_2::Vertex_const_iterator e) {
  return Polygon_2(b, e).area();
}

std::ostream &operator<<(std::ostream &os, const Polygon_2 &p);

class Polygon_with_holes_2 {
public:
  typedef Polygon_2 Polygon;
  typedef std::vector<Polygon_2> Holes_container;
  typedef Holes_container::iterator Hole_iterator;
  typedef Holes_container::const_iterator Hole_const_iterator;

  Polygon_with_holes_2() {}
  explicit Polygon_with_holes_2(const Polygon_2 &outer) : _outer(outer) {}
  template <class It>
  Polygon_with_holes_2(const Polygon_2 &outer, It first, It last) : _outer(outer), _holes(first, last) {}

  Polygon_2 &outer_boundary() { return _outer; }
  const Polygon_2 &outer_boundary() const { return _outer; }
  Hole_iterator holes_begin() { return _holes.begin(); }
  Hole_iterator holes_end() { return _holes.end(); }
  Hole_const_iterator holes_begin() const { return _holes.begin(); }
  Hole_const_iterator holes_end() const { return _holes.end(); }
  std::size_t number_of_holes() const { return _holes.size(); }
  bool has_holes() const { return !_holes.empty(); }
  void add_hole(const Polygon_2 &h) { _holes.push_back(h); }
  void erase_hole(Hole_iterator it) { _holes.erase(it); }
  void clear() {
    _outer.clear();
    _holes.clear();
  }
  // An empty outer boundary stands for the whole plane, as CGAL's did.
  bool is_unbounded() const { return _outer.is_empty(); }
  Bbox_2 bbox() const { return _outer.bbox(); }

private:
  Polygon_2 _outer;
  Holes_container _holes;
};

std::ostream &operator<<(std::ostream &os, const Polygon_with_holes_2 &p);

} // namespace cvc::tiler

#endif // CVC_GEOMETRY_TILER_KERNEL_H
