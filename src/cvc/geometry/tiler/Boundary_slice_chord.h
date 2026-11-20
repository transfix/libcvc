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

#ifndef __BOUNDARY_SLICE_CHORD_H__
#define __BOUNDARY_SLICE_CHORD_H__

#include "Correspondences.h"
#include "Tiling_region.h"
#include "Vertex_completion_map.h"
#include "Vertex_map.h"
#include "common.h"

namespace cvc::tiler {

/// Boundary slice chord
class Boundary_slice_chord {
public:
  friend std::size_t hash_value(const Boundary_slice_chord &bsc) {
    return hash_value(bsc._segment);
  }

  bool operator==(const Boundary_slice_chord &bsc) const { return _segment == bsc._segment; }

  bool operator!=(const Boundary_slice_chord &bsc) const { return !(*this == bsc); }

  friend std::ostream &operator<<(std::ostream &out, const Boundary_slice_chord &bsc) {
    out << bsc.segment();
    //     Contour_vertex temp1, temp2;
    //     boost::tie(temp1, temp2) = bsc.vertices();
    //     out << temp1 << "  " << temp2;
    return out;
  }

public:
  Boundary_slice_chord() : _dir(Walk_direction::FORWARD) {}

  Boundary_slice_chord(const Segment_3 &segment, Walk_direction dir, Number_type seg_z)
      : _segment(segment), _dir(dir), _seg_z(seg_z) {
    if (segment.source().z() == segment.target().z())
      throw std::logic_error("segment points must lie on different slices");
    //     assert_order();
  }

  ~Boundary_slice_chord() {}

  Boundary_slice_chord opposite() const {
    return Boundary_slice_chord(_segment.opposite(), _dir, _seg_z);
  }

  const Segment_3 &segment() const { return _segment; }
  Walk_direction direction() const { return _dir; }

  bool is_source(const Point_3 &vertex) const { return vertex == _segment.source(); }

  /// slice that the contour segment of the adjoining tile is on.
  Number_type seg_z() const { return _seg_z; }

  //   bool is_endpoint(const Point_3& vertex) const
  //   {
  //     return vertex == _segment.source() || vertex == _segment.target();
  // //     return xyz_equal(vertex, boost::get<0>(_vertices)) || xyz_equal(vertex,
  // boost::get<1>(_vertices));
  //   }

private:
  //   void assert_order()
  //   {
  //     if (boost::get<0>(_vertices).z() >= boost::get<1>(_vertices).z())
  //       throw std::logic_error("first vertex must be on the bottom in a boundary slice chord");
  //   }

private:
  Segment_3 _segment;
  Walk_direction _dir;
  Number_type _seg_z;
};

} // namespace cvc::tiler

#endif
