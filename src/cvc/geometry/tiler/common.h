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
//
// The legacy common.h chose the number type and kernel: CGAL::Cartesian<double> with its own
// point (Cartesian_25), CGAL's Polygon_2 and Polygon_with_holes_2. The same names now come from
// the tiler kernel (kernel.h): doubles on cvc::planar's snapping grid, with Shewchuk's exact
// predicates. `Kernel` keeps the nested typedefs the vendored templates name.

#ifndef __COMMON_H__
#define __COMMON_H__

#include <iostream>
#include <limits>
#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "kernel.h"
#include "tiler_context.h"

namespace cvc::tiler {

// The legacy code is written against `using namespace std` (its headers said so at global scope);
// here it holds inside cvc::tiler only.
using namespace std;

#define TO_NT(d) d

struct Kernel {
  typedef Number_type FT;
  typedef Number_type RT;
  typedef cvc::tiler::Point_2 Point_2;
  typedef cvc::tiler::Point_3 Point_3;
  typedef cvc::tiler::Segment_2 Segment_2;
  typedef cvc::tiler::Segment_3 Segment_3;
  typedef cvc::tiler::Vector_2 Vector_2;
  typedef cvc::tiler::Vector_3 Vector_3;
  typedef cvc::tiler::Direction_2 Direction_2;
  typedef cvc::tiler::Line_2 Line_2;
  typedef cvc::tiler::Line_3 Line_3;
  typedef cvc::tiler::Ray_2 Ray_2;
  typedef cvc::tiler::Triangle_2 Triangle_2;
};
typedef Kernel PolygonTraits;

} // namespace cvc::tiler

#endif
