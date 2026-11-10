/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// distance_transform.h — the exact Euclidean distance transform of a binary grid.
//
// cvc::algo holds grid algorithms that more than one libcvc library needs and none of them owns.
// It depends on the standard library only, so any library may use it: cvc::nav (its clearance
// fields and SDFs, cvc/nav/grid_nav.h) and cvc::world (the shore distance of
// cvc/world/water_bodies.h), which may not depend on each other.
//
//   std::vector<double> d2 = cvc::algo::edt2_squared(mask, rows, cols); // cells^2 to a seed
//   cvc::algo::edt2_squared_both(mask, rows, cols, to_seed, to_other);  // and to a non-seed
//
// The transform is Felzenszwalb and Huttenlocher's separable lower envelope of parabolas, columns
// (the y axis) first and then rows, in float64, bit-identical to GRL-SNAM's sdf_nav._edt2 (its
// parabola-envelope arithmetic in float32 would flip gradients on medial axes). The kernel takes
// an exact fast path for a binary mask: integer run sweeps for the columns, and the row pass over
// only the columns holding a seed (distance_transform.cpp says why it gives the same doubles).
// Distances are in cells, row-major, rows x cols; scale them by the cell size for metres (a square
// grid's). A cell with no seed anywhere in the grid gets about edt_far.
//
// Scratch is per thread and reused across calls, so a pool of workers each keeps one set and no
// call allocates beyond what it returns.

#ifndef CVC_ALGO_DISTANCE_TRANSFORM_H
#define CVC_ALGO_DISTANCE_TRANSFORM_H

#include <cstdint>
#include <functional>
#include <vector>

namespace cvc {
namespace algo {

// "No seed": the transform's infinity, and about what a cell gets when the grid has no seed.
constexpr double edt_far = 1e20;

// Squared Euclidean distance (cells^2) from each cell to the nearest cell with mask nonzero (a
// seed). Empty when rows or cols is not positive.
std::vector<double> edt2_squared(const std::uint8_t *mask, int rows, int cols);

// The transform of the mask and of its complement in one pass: to_seed[i] is the squared distance
// to the nearest seed (0 at a seed), to_other[i] to the nearest non-seed (0 at a non-seed). Both
// rows x cols, caller-owned, and must not overlap.
void edt2_squared_both(const std::uint8_t *mask, int rows, int cols, double *to_seed,
                       double *to_other);

// The transform a row at a time, for a caller that uses each row while it is hot (cvc::nav's
// build_sdf forms its SDF row from both). Rows come bottom-up: for row r, dst(r, to_seed,
// to_other) says where to write its cols distances (to_other only when want_other), the row is
// written, then done(r) is called. The two may point at the same scratch row each time.
using edt_row_dst = std::function<void(int r, double *&to_seed, double *&to_other)>;
using edt_row_done = std::function<void(int r)>;
void edt2_squared_rows(const std::uint8_t *mask, int rows, int cols, bool want_other,
                       const edt_row_dst &dst, const edt_row_done &done);

} // namespace algo
} // namespace cvc

#endif // CVC_ALGO_DISTANCE_TRANSFORM_H
