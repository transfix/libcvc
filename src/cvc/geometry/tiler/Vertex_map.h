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

#ifndef __VERTEX_MAP_H__
#define __VERTEX_MAP_H__

#include "Contour.h"
#include "common.h"

#include <boost/unordered_map.hpp>
#include <map>
#include <vector>

namespace cvc::tiler {

template <typename T> class Vertex_map {
  //--------------------------------------------------------
  // Vertex_map class
  //--------------------------------------------------------
private:
  typedef std::vector<T> Values;

public:
  typedef typename Values::iterator iterator;
  typedef typename Values::const_iterator const_iterator;

public:
  Vertex_map() : _values(10000) {}
  Vertex_map(size_t size) : _values(size) { _values.resize(size); }
  ~Vertex_map() {}

  typename Values::const_reference operator[](const Point_3 &vertex) const {
    return _values[vertex.id()];
  }

  typename Values::reference operator[](const Point_3 &vertex) {
    _values.reserve((vertex.id() + 1) * 2);
    return _values[vertex.id()];
  }

  bool contains(const Point_3 &vertex) const { return vertex.id() < _values.size(); }

  void set(const Point_3 &vertex, const T &value) {
    _values.reserve(vertex.id() + 1);
    _values[vertex.id()] = value;
  }

  iterator begin() { return _values.begin(); }
  const_iterator begin() const { return _values.begin(); }
  iterator end() { return _values.end(); }
  const_iterator end() const { return _values.end(); }

private:
  Values _values;
};

} // namespace cvc::tiler

#endif
