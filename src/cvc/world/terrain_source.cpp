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

#include <algorithm>
#include <cmath>
#include <cvc/world/grid.h>
#include <cvc/world/npy.h>
#include <cvc/world/terrain_source.h>
#include <stdexcept>

namespace cvc {
namespace world {

double terrain_source::slope_deg(double x, double y, double h_m) const {
  const double dzdx = (sample(x + h_m, y) - sample(x - h_m, y)) / (2.0 * h_m);
  const double dzdy = (sample(x, y + h_m) - sample(x, y - h_m)) / (2.0 * h_m);
  return std::atan(std::sqrt(dzdx * dzdx + dzdy * dzdy)) * 180.0 / 3.14159265358979323846;
}

raster_heightfield::raster_heightfield(int rows, int cols, double min_x, double min_y, double max_x,
                                       double max_y, std::vector<float> heights, double sea_level_m,
                                       double relief_m)
    : rows_(rows), cols_(cols), min_x_(min_x), min_y_(min_y), max_x_(max_x), max_y_(max_y),
      h_(std::move(heights)), sea_(sea_level_m), relief_(relief_m) {
  if (rows_ < 2 || cols_ < 2)
    throw std::invalid_argument("raster_heightfield: needs at least 2 x 2 samples");
  if (h_.size() != std::size_t(rows_) * std::size_t(cols_))
    throw std::invalid_argument("raster_heightfield: heights are not rows x cols");
  if (!(max_x_ > min_x_) || !(max_y_ > min_y_))
    throw std::invalid_argument("raster_heightfield: empty extent");
  const auto mm = std::minmax_element(h_.begin(), h_.end());
  lo_ = *mm.first;
  hi_ = *mm.second;
  if (relief_ <= 0.0)
    relief_ = std::max(1.0, double(hi_) - double(lo_));
}

double raster_heightfield::bilinear(const std::vector<float> &v, double x, double y) const {
  const double fx = std::min(double(cols_ - 1), std::max(0.0, (x - min_x_) / cell_w()));
  const double fy = std::min(double(rows_ - 1), std::max(0.0, (y - min_y_) / cell_h()));
  const int c0 = std::min(cols_ - 2, int(fx)), r0 = std::min(rows_ - 2, int(fy));
  const double tx = fx - c0, ty = fy - r0;
  const std::size_t i = std::size_t(r0) * cols_ + c0;
  const double a = v[i] + (v[i + 1] - v[i]) * tx;
  const double b = v[i + cols_] + (v[i + cols_ + 1] - v[i + cols_]) * tx;
  return a + (b - a) * ty;
}

double raster_heightfield::sample(double x, double y) const { return bilinear(h_, x, y); }

double raster_heightfield::slope_deg(double x, double y, double h_m) const {
  return terrain_source::slope_deg(x, y, std::max(h_m, std::max(cell_w(), cell_h())));
}

terrain_bounds raster_heightfield::bounds() const {
  return terrain_bounds{min_x_, min_y_, max_x_, max_y_, false};
}

bool raster_heightfield::has_mask(terrain_mask m) const {
  const int i = int(m);
  return i >= 0 && i < terrain_mask_count && !masks_[i].empty();
}

double raster_heightfield::mask(terrain_mask m, double x, double y) const {
  return has_mask(m) ? bilinear(masks_[int(m)], x, y) : 0.0;
}

void raster_heightfield::set_mask(terrain_mask m, std::vector<float> values) {
  const int i = int(m);
  if (i < 0 || i >= terrain_mask_count)
    throw std::invalid_argument("raster_heightfield: unknown mask");
  if (!values.empty() && values.size() != h_.size())
    throw std::invalid_argument("raster_heightfield: mask is not rows x cols");
  for (float &v : values)
    v = std::min(1.0f, std::max(0.0f, v));
  masks_[i] = std::move(values);
}

raster_heightfield sample_terrain(const terrain_source &t, const grid_spec &g) {
  std::vector<float> h(g.count());
  for (int r = 0; r < g.rows; ++r)
    for (int c = 0; c < g.cols; ++c)
      h[std::size_t(r) * g.cols + c] = float(t.sample(g.world_x(c), g.world_y(r)));
  raster_heightfield out(g.rows, g.cols, g.min_x, g.min_y, g.max_x, g.max_y, std::move(h),
                         t.sea_level_m(), t.relief_m());
  for (int k = 0; k < terrain_mask_count; ++k) {
    const terrain_mask m = terrain_mask(k);
    if (!t.has_mask(m))
      continue;
    std::vector<float> v(g.count());
    for (int r = 0; r < g.rows; ++r)
      for (int c = 0; c < g.cols; ++c)
        v[std::size_t(r) * g.cols + c] = float(t.mask(m, g.world_x(c), g.world_y(r)));
    out.set_mask(m, std::move(v));
  }
  return out;
}

raster_heightfield load_dem(const std::string &path, double half_m, double relief_m,
                            double floor_m) {
  int rows = 0, cols = 0;
  std::vector<float> h = read_npy(path, rows, cols);
  if (rows < 2 || cols < 2)
    throw std::runtime_error("a DEM needs at least 2 x 2 samples: " + path);
  for (float v : h)
    if (!std::isfinite(v))
      throw std::runtime_error("a DEM's samples must be finite (no NaN no-data): " + path);
  const auto mm = std::minmax_element(h.begin(), h.end());
  const double lo = *mm.first, span = double(*mm.second) - lo;
  // Anchored: the floor at floor_m, so the water level means the same thing on every generator;
  // the relief stretched to relief_m when one is asked for (a flat DEM stays flat).
  const double k = relief_m > 0.0 && span > 1e-9 ? relief_m / span : 1.0;
  for (float &v : h)
    v = float(floor_m + (v - lo) * k);
  const double half = std::max(1.0, half_m);
  return raster_heightfield(rows, cols, -half, -half, half, half, std::move(h), 0.0,
                            relief_m > 0.0 ? relief_m : 0.0);
}

} // namespace world
} // namespace cvc
