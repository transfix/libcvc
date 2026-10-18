/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/grid_math.h>
#include <cvc/nav/terrain.h>
#include <stdexcept>

namespace cvc {
namespace nav {

double terrain_limit(const terrain_config &c) {
  double lim = c.max_grade;
  if (c.tip_ratio > 0.0)
    lim = std::min(lim, c.tip_margin * c.tip_ratio);
  return lim;
}

terrain_planes terrain_build(const float *height_m, int rows, int cols, double cell_w,
                             const terrain_config &c) {
  if (!height_m || rows < 1 || cols < 1 || !(cell_w > 0.0))
    throw std::runtime_error("cvc::nav::terrain_build: needs a height raster and cell_w > 0");
  if (!(c.max_grade > 0.0) || c.tip_ratio < 0.0 || !(c.tip_margin > 0.0) || !(c.gravity > 0.0))
    throw std::runtime_error("cvc::nav::terrain_build: max_grade, tip_margin and gravity must be > "
                             "0, tip_ratio >= 0");
  terrain_planes p;
  p.rows = rows;
  p.cols = cols;
  const long hw = static_cast<long>(rows) * cols;
  p.slope.assign(static_cast<std::size_t>(2 * hw), 0.0f);
  p.grade_cost.assign(static_cast<std::size_t>(hw), 0.0f);
  p.infeasible.assign(static_cast<std::size_t>(hw), 0);
  p.limit = terrain_limit(c);
  const double sin_lim = p.limit / std::sqrt(1.0 + p.limit * p.limit);
  const float inv_w = static_cast<float>(1.0 / cell_w);
  for (int r = 0; r < rows; ++r)
    for (int col = 0; col < cols; ++col) {
      const long i = static_cast<long>(r) * cols + col;
      // np.gradient along x (columns) and y (rows), per metre.
      const float gx = detail::grad1d(height_m + static_cast<long>(r) * cols, col, cols, 1) * inv_w;
      const float gy = detail::grad1d(height_m + col, r, rows, cols) * inv_w;
      p.slope[static_cast<std::size_t>(i)] = gx;
      p.slope[static_cast<std::size_t>(hw + i)] = gy;
      const double t = std::sqrt(static_cast<double>(gx) * gx + static_cast<double>(gy) * gy);
      const double s = t / std::sqrt(1.0 + t * t);
      p.grade_cost[static_cast<std::size_t>(i)] = static_cast<float>(std::min(1.0, s / sin_lim));
      p.infeasible[static_cast<std::size_t>(i)] = t > p.limit ? 1 : 0;
    }
  return p;
}

} // namespace nav
} // namespace cvc
