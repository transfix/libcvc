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

#ifndef __THEOREMS_H__
#define __THEOREMS_H__

#include "Boundary_slice_chords.h"
#include "Tiler_workspace.h"
#include "common.h"

namespace cvc::tiler {

bool test_all(const Segment_3 &chord, const Tiler_workspace &w);

bool test_theorem2(const Segment_3 &chord, const Tiler_workspace &w);

bool test_theorem3(const Segment_3 &segment, const Point_3 &opposite, Tiler_workspace &w);

bool test_theorem4(const Segment_3 &segment, const Point_3 &opposite, Tiler_workspace &w);

/// Returns false if the chords fail the test of Theorem 5, described in bajaj96.
bool test_theorem5(const Segment_3 &chord0, const Segment_3 &chord1);

bool test_theorem5(const Segment_3 &chord, const Boundary_slice_chords &bscs);

bool test_theorem5(const Segment_3 &chord0, const Segment_3 &chord1,
                   const Boundary_slice_chords &bscs);

/// This is a new test not presented in the paper
bool test_theorem8(const Segment_3 &segment, const Point_3 &opposite, const Tiler_workspace &w);

bool test_theorem9(const Segment_3 &segment, const Point_3 &opposite, Tiler_workspace &w);

} // namespace cvc::tiler

#endif
