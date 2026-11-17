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

#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include "common.h"

namespace cvc::tiler {

/// Projects the 3D segment onto the plane with the z axis as the normal.
Segment_2 projection_z(const Segment_3 &chord);

/// Projects the 3D segment onto the plane with the x axis as the normal.
Segment_2 projection_x(const Segment_3 &chord);

/// Projects the 3D segment onto the plane with the y axis as the normal.
Segment_2 projection_y(const Segment_3 &chord);

} // namespace cvc::tiler

#endif
