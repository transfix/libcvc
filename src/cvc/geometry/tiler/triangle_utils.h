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

#ifndef __TRIANGLE_UTILS_H__
#define __TRIANGLE_UTILS_H__

#include <sstream>

#include "common.h"
#include "Segment_3_undirected.h"

namespace cvc::tiler {

class Triangle
{
public:
  Triangle() {}

//   Triangle(const Point_3& a, const Point_3& b, const Point_3& c)
//     : _points((Point_3[]){ a, b, c }) {}
  Triangle(const Point_3& a, const Point_3& b, const Point_3& c)
  {
    _points[0] = a;
    _points[1] = b;
    _points[2] = c;
    check_valid();
  }

  const Point_3& operator[](size_t i) const { return _points[i]; }
  Point_3& operator[](size_t i) { return _points[i]; }

  bool is_degenerate() const;

private:
  void check_valid() const;

private:
  Point_3 _points[3];
};

//------------------------------------------------------------------------------
// These functions assume a triangle with indices described in page 51 of lab
// book dated 6/1/09:
//
//           0
//          /\
//         /  \
//        /    \
//      0/      \2
//      /        \
//     /          \
//    /            \
//   /______________\
//  1       1         2
//
//------------------------------------------------------------------------------

template <typename Triangle>
const Point_3& vertex(size_t i, const Triangle& t)
{ return t[i%3]; }

template <typename Triangle>
Segment_3 edge(size_t i, const Triangle& t)
{ return Segment_3(vertex(i, t), vertex(i+1, t)); }

template <typename Triangle>
int index(const Point_3& v, const Triangle& t)
{
  for (size_t i = 0; i < 3; ++i)
    if (t[i] == v) 
      return i;
  return -1;
}

template <typename Triangle>
int index(const Segment_3& e, const Triangle& t)
{
  int so = index(e.source(), t);
  int ta = index(e.target(), t);
  if (so == -1 || ta == -1)
    return -1;
  if (so == 1 || ta == 1)
    return min(so, ta);
  return 2;
}

template <typename Triangle>
int index(const Segment_3_undirected& e, const Triangle& t)
{
  return index(e.segment(), t);
}

inline size_t next_idx(size_t i)
{ return (i+1)%3; }

inline size_t prev_idx(size_t i)
{ return (i+2)%3; }

inline size_t inc_idx(size_t i, int inc)
{ 
  if (inc < 0)
    inc = ((inc%3)+3)%3;
  return (i+inc)%3; 
}

std::string tri_pp(const Point_2& point);
std::string tri_pp(const Point_3& point);
std::string tri_pp(const Segment_2& segment);
std::string tri_pp(const Segment_3& segment);

template <typename Triangle>
int other_edge(const Segment_3& e, const Point_3& v, const Triangle& t)
{

  int ei = index(e, t);
  int vi = index(v, t);
  if (vi == ei)
    return prev_idx(ei);
  if (vi != next_idx(ei))
  {
    throw logic_error("edge and vertex are not adjacent: edge (" + std::to_string(ei) + "): " +
                      tri_pp(e) + ", vertex (" + std::to_string(vi) + "): " + tri_pp(v));
  }
  return next_idx(ei);
}

template <typename Triangle>
int other_edge(const Segment_3_undirected& e, const Point_3& v, const Triangle& t)
{
  return other_edge(e.segment(), v, t);
}

template <typename Triangle>
int other_vertex(const Point_3& v, const Segment_3& e, const Triangle& t)
{

  int ei = index(e, t);
  int vi = index(v, t);
  if (vi == ei)
    return next_idx(vi);
  if (vi != next_idx(ei))
  {
    throw logic_error("edge and vertex are not adjacent: edge (" + std::to_string(ei) + "): " +
                      tri_pp(e) + ", vertex (" + std::to_string(vi) + "): " + tri_pp(v));
  }
  return prev_idx(vi);
}

template <typename Triangle>
int other_vertex(const Point_3& v, const Segment_3_undirected& e, const Triangle& t)
{
  return other_vertex(v, e.segment(), t);
}

inline bool in_order(size_t i, size_t j)
{ return next_idx(i) == j; }

template <typename Triangle>
size_t index_ignore_order(const Segment_3& e, const Triangle& t)
{
  size_t a = index(e.source(), t);
  size_t b = index(e.source(), t);
  return min(a, b);
}

template <typename Triangle>
bool has_edge_ignore_order(const Segment_3& e, const Triangle& t)
{ return index_ignore_order(e, t) != -1; }

/// Returns the index of the triangle vertex opposite to the i'th edge
inline
size_t opposite_vertex(size_t i)
{
  return (i+2)%3;
}

/// Returns the index of the edge opposite the i'th vertex.
inline
size_t opposite_edge(size_t i)
{
  return (i+1)%3;
}

/// Returns the vertex of the triangle vertex opposite to edge e
template <typename Triangle>
const Point_3& opposite_vertex(const Segment_3& e, const Triangle& t)
{
  return vertex(opposite_vertex(index(e, t)), t);
}

/// Returns the edge opposite vertex v
template <typename Triangle>
const Segment_3& opposite_edge(const Point_3& v, const Triangle& t)
{
  return edge(opposite_edge(index(v, t)), t);
}

//------------------------------------------------------------------------------
// get_z
//
/// Returns the z value of the 2D point projected onto the triangle t.  No
/// check is made to ensure that p lies within t, and if it doesn't a z value
/// outside the z range of the triangle may be returned.
//------------------------------------------------------------------------------
template <typename Triangle>
Number_type get_z(const Triangle& t, const Point_2& point)
{
  Point_3 p = t[0], q = t[1], r = t[2];
  // The plane through p, q, r: its normal (p - r) x (q - r).
  const Number_type rpx = p.x() - r.x(), rpy = p.y() - r.y(), rpz = p.z() - r.z();
  const Number_type rqx = q.x() - r.x(), rqy = q.y() - r.y(), rqz = q.z() - r.z();
  const Number_type a = rpy * rqz - rqy * rpz;
  const Number_type b = rpz * rqx - rqz * rpx;
  const Number_type c = rpx * rqy - rqx * rpy;
  const Number_type d = -a * r.x() - b * r.y() - c * r.z();

  // ax + by + cz + d = 0
  return (a * point.x() + b * point.y() + d) / -c;
}

} // namespace cvc::tiler

#endif
