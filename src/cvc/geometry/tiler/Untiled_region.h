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

#ifndef __UNTILED_REGION_H__
#define __UNTILED_REGION_H__

#include "common.h"

#include <list>

namespace cvc::tiler {

/// Basically is a collection of points representing a polygon.
/// Useful when it needs to support points directly above another.
class Untiled_region {
private:
  typedef std::list<Point_3> Container;

public:
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;

public:
  Untiled_region() {}

  template <typename PointIterator> Untiled_region(PointIterator begin, PointIterator end) {
    insert(begin, end);
  }

  ~Untiled_region() {}

  template <typename PointIterator> void insert(PointIterator begin, PointIterator end) {
    for (PointIterator it = begin; it != end; ++it)
      push_back(it->point_3());
  }

  void push_back(const Point_3 &point) { _points.push_back(point); }

  iterator begin() { return _points.begin(); }
  const_iterator begin() const { return _points.begin(); }
  iterator end() { return _points.end(); }
  const_iterator end() const { return _points.end(); }

  Container::reverse_iterator rbegin() { return _points.rbegin(); }
  Container::const_reverse_iterator rbegin() const { return _points.rbegin(); }
  Container::reverse_iterator rend() { return _points.rend(); }
  Container::const_reverse_iterator rend() const { return _points.rend(); }

  void reverse_orientation() { _points = Container(_points.rbegin(), _points.rend()); }

  size_t size() const { return _points.size(); }
  bool empty() const { return _points.empty(); }

private:
  Container _points;
};

} // namespace cvc::tiler

#endif
