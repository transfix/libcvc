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

#ifndef __VERTEX_COMPLETION_MAP_H__
#define __VERTEX_COMPLETION_MAP_H__

#include <boost/unordered_map.hpp>

#include "Tile.h"

namespace cvc::tiler {

class Vertex_completion_map
{
public:
  Vertex_completion_map() {}
  ~Vertex_completion_map() {}

  /// The segment must be in contour ordering
  void put(const Segment_3& segment)
  {
    //     -----------------*-----------------      slice i
    //                     / \
    //                    /   \
    //                   /     \
    //                  /       \
    //                 /         \
    //                /           \
    //               /             \
    // *------------*===============*------------*  slice j
    //              ^               ^
    //           source          target
    // 
    // source is forward-complete
    // target is backward-complete
    _forward[segment.source()];
    _backward[segment.target()];
  }

  bool is_complete(const Segment_3& segment)
  {
    return (_forward.find(segment.source()) != _forward.end() && _backward.find(segment.target()) != _backward.end());
  }

  bool is_complete(const Point_3& vertex)
  {
    return (_forward.find(vertex) != _forward.end() && _backward.find(vertex) != _backward.end());
  }

  bool is_complete(const Point_3& vertex, Walk_direction dir)
  {
    return (dir == Walk_direction::FORWARD) ? (_forward.find(vertex) != _forward.end()) : (_backward.find(vertex) != _backward.end());
  }

private:
  boost::unordered_map<Point_3, bool> _forward;
  boost::unordered_map<Point_3, bool> _backward;
};

} // namespace cvc::tiler

#endif
