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

// grid.h — the export grid, in the consumer's exact frame (roadmap §7.1a).
//
// Bounds are corner-INCLUSIVE sample centres, so cell_w = extent / (cols - 1),
// matching planner.far_pair_in_free_space's to_world and MaterialField._to_grid.
// Row 0 maps to min_y (the C++/GRL-SNAM convention; the research BEV builder is
// max_y-first — the flip happens exactly once in the adapter, never here).
//
// The default 513×513 spanning 256.0 m gives cell_w == 0.5 exactly, which is
// (a) the source-BEV frame the material formulas were invented in and (b) the
// only cell size at which the C++ (hard_margin_m = 1.0) and Python (2*cell_w)
// hard margins agree. See §7.1a.

#ifndef CVC_WORLD_GRID_H
#define CVC_WORLD_GRID_H

#include <cmath>
#include <cstdint>

namespace cvc {
namespace world {

struct grid_spec {
  int rows = 513, cols = 513;
  double min_x = -128.0, min_y = -128.0, max_x = 128.0, max_y = 128.0;

  static constexpr const char *row_order = "min_y_first";

  double cell_w() const noexcept { return (max_x - min_x) / double(cols - 1); }
  double cell_h() const noexcept { return (max_y - min_y) / double(rows - 1); }

  // World coordinate of a sample centre (col c, row r).
  double world_x(int c) const noexcept { return min_x + c * cell_w(); }
  double world_y(int r) const noexcept { return min_y + r * cell_h(); }

  std::size_t count() const noexcept { return std::size_t(rows) * std::size_t(cols); }

  bool square(double eps = 1e-9) const noexcept { return std::fabs(cell_w() - cell_h()) < eps; }

  // Build a window of `half` metres about (cx, cy) at a given cell size, snapping
  // the sample count so cell_w is exactly `cell`. Default: 256 m @ 0.5 m -> 513².
  static grid_spec window(double cx, double cy, double half, double cell) {
    grid_spec g;
    const int n = int(std::lround(2.0 * half / cell)) + 1; // corner-inclusive
    g.rows = g.cols = n;
    g.min_x = cx - half;
    g.max_x = cx + half;
    g.min_y = cy - half;
    g.max_y = cy + half;
    return g;
  }
};

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_GRID_H
