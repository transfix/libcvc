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

#ifndef __POLYLINE_H__
#define __POLYLINE_H__

#include "common.h"

#include <boost/functional/hash.hpp>

namespace cvc::tiler {

template <typename Point> class Polyline {
private:
  typedef typename std::vector<Point> Container;

public:
  typedef typename Container::iterator iterator;
  typedef typename Container::const_iterator const_iterator;
  typedef typename Container::reverse_iterator reverse_iterator;
  typedef typename Container::const_reverse_iterator const_reverse_iterator;

public:
  Polyline() {}

  Polyline(const Point_3 &p) { _vertices.push_back(p); }

  template <typename Vertex_iter> Polyline(Vertex_iter beg, Vertex_iter end) {
    _vertices.insert(_vertices.end(), beg, end);
    check_valid();
  }

  ~Polyline() {}

  void push_back(const Point &v) {
    _vertices.push_back(v);
    check_valid();
  }

  template <typename Vertex_iter> void insert(iterator loc, Vertex_iter beg, Vertex_iter end) {
    _vertices.insert(loc, beg, end);
    check_valid();
  }

  const Point &source() const { return _vertices[0]; }
  const Point &target() const { return _vertices[size() - 1]; }

  iterator begin() { return _vertices.begin(); }
  const_iterator begin() const { return _vertices.begin(); }
  iterator end() { return _vertices.end(); }
  const_iterator end() const { return _vertices.end(); }

  reverse_iterator rbegin() { return _vertices.rbegin(); }
  const_reverse_iterator rbegin() const { return _vertices.rbegin(); }
  reverse_iterator rend() { return _vertices.rend(); }
  const_reverse_iterator rend() const { return _vertices.rend(); }

  Point &operator[](int i) { return _vertices[i]; }
  const Point &operator[](int i) const { return _vertices[i]; }

  size_t size() const { return _vertices.size(); }

  bool operator==(const Polyline &p) const { return _vertices == p._vertices; }

private:
  void check_valid() const;

private:
  Container _vertices;
};

typedef Polyline<Point_2> Polyline_2;
typedef Polyline<Point_3> Polyline_3;

// As the points' hashes (kernel.h): Boost's 64-bit arithmetic, folded for the container.
inline std::size_t hash_value(const Polyline_2 &p) {
  std::uint64_t seed = 0;
  for (Polyline_2::const_iterator it = p.begin(); it != p.end(); ++it)
    hash64::combine(seed, hash64_value(*it));
  return hash64::fold(seed);
}

inline std::size_t hash_value(const Polyline_3 &p) {
  std::uint64_t seed = 0;
  for (Polyline_3::const_iterator it = p.begin(); it != p.end(); ++it)
    hash64::combine(seed, hash64_value(*it));
  return hash64::fold(seed);
}

} // namespace cvc::tiler

#endif
