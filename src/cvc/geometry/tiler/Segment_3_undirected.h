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

#ifndef __SEGMENT_3_UNDIRECTED__
#define __SEGMENT_3_UNDIRECTED__

#include "common.h"
#include "segment_utils.h"

namespace cvc::tiler {

/// There is no concept of a source or target in this segment.  There
/// is no direction.
///
/// (0,0,0)-(1,1,1) == (1,1,1)-(0,0,0)
template <typename Segment, typename Point> class Segment_undirected {
public:
  Segment_undirected() {}
  Segment_undirected(const Point &a, const Point &b) { *this = Segment(a, b); }
  Segment_undirected(const Segment &s) { *this = s; }
  ~Segment_undirected() {}

  Segment_undirected &operator=(const Segment &s) {
    _s = lexicographically_ordered(s);
    _orig = s;
    return *this;
  }

  bool operator==(const Segment_undirected &s) const { return _s == s._s; }

  bool operator<(const Segment_undirected &s) const { return lesser() < s.lesser(); }

  // Returns the endpoint that is lexicographically
  // less than the other
  Point lesser() const { return _s.source(); }

  // Returns the endpoint that is lexicographically
  // greater than the other
  Point greater() const { return _s.target(); }

  Point operator[](size_t index) { return _s[index]; }

  /// Returns the underlying segment
  const Segment &segment() const { return _s; }

  const Segment &orig_segment() const { return _orig; }

private:
  Segment _s;
  Segment _orig;
};

typedef Segment_undirected<Segment_2, Point_2> Segment_2_undirected;
typedef Segment_undirected<Segment_3, Point_3> Segment_3_undirected;

inline std::size_t hash_value(const Segment_2_undirected &s) { return hash_value(s.segment()); }

inline std::size_t hash_value(const Segment_3_undirected &s) { return hash_value(s.segment()); }

} // namespace cvc::tiler

#endif
