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

#ifndef __POLYGON_DIFFERENCE_H__
#define __POLYGON_DIFFERENCE_H__

#include "common.h"

namespace cvc::tiler {

template <typename Out_iter>
void polygon_difference(const Polygon_2& P, const Polygon_2& Q, 
			Out_iter P_out, Out_iter Q_out);

template <typename Out_iter>
void polygon_difference(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
			Out_iter P_out, Out_iter Q_out);

} // namespace cvc::tiler

#endif
