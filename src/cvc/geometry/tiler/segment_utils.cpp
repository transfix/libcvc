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

#include "segment_utils.h"

namespace cvc::tiler {

Segment_2 lexicographically_ordered(const Segment_2& s)
{
  if (lexicographically_xy_smaller(s.source(), s.target()))
    return s;
  return s.opposite();
}

Segment_3 lexicographically_ordered(const Segment_3& s)
{
  if (lexicographically_xyz_smaller(s.source(), s.target()))
    return s;
  return s.opposite();
}

bool equal_ignore_order(const Segment_3& a, const Segment_3& b)
{
  return lexicographically_ordered(a) == lexicographically_ordered(b);
}

bool equal_ignore_order(const Segment_2& a, const Segment_3& b)
{
  return lexicographically_ordered(project_3(a)) == lexicographically_ordered(b);
}

Segment_3 project_3(const Segment_2& s)
{
  return Segment_3(s.source().point_3(), s.target().point_3());
}

Segment_2 project_2(const Segment_3& s)
{
  return Segment_2(s.source().point_2(), s.target().point_2());
}

} // namespace cvc::tiler
