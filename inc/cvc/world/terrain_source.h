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

// terrain_source.h — the ground every world consumer reads, whatever made it.
//
// A terrain source is a height in metres at world (x, y), its slope, where it is defined, its datum
// (the level its heights are measured from and how much relief to expect, which the raster's
// height and slope classes scale by), and optional masks a generator knows better than a slope
// threshold does: where to build, where water runs, where the ground is a cliff.
//
//   * heightfield (heightfield.h): the analytic fBm relief, defined everywhere.
//   * raster_heightfield (here): a gridded relief, bilinear between its samples and clamped to its
//     edge outside them. It carries any generator's DEM (an elevation .npy, an eroded relief), and
//     sample_terrain rasterizes any source onto one.
//
// world_model, raster(), the scatter and the city take a terrain_source, never a concrete type.

#ifndef CVC_WORLD_TERRAIN_SOURCE_H
#define CVC_WORLD_TERRAIN_SOURCE_H

#include <cstddef>
#include <string>
#include <vector>

namespace cvc {
namespace world {

struct grid_spec;

// Where a source is defined (metres). An unbounded source (the analytic relief) is defined
// everywhere; the box is then meaningless.
struct terrain_bounds {
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  bool unbounded = true;
};

// A generator's own masks, each in [0, 1] (1 = the property holds there).
enum class terrain_mask : int {
  buildable = 0, // flat and firm enough for a lot
  channel,       // where water runs (a river bed, a ravine floor): no road or lot
  cliff,         // a scarp: no road or lot
};
constexpr int terrain_mask_count = 3;

class terrain_source {
public:
  virtual ~terrain_source() = default;

  // The ground height (metres) at world (x, y).
  virtual double sample(double x, double y) const = 0;
  // The slope in degrees, from a centred difference h_m either side.
  virtual double slope_deg(double x, double y, double h_m = 1.0) const;
  virtual terrain_bounds bounds() const = 0;
  // The datum: the level the heights are measured from (the raster's "below sea" class), and the
  // relief the height and slope classes scale by (metres, > 0).
  virtual double sea_level_m() const { return 0.0; }
  virtual double relief_m() const = 0;
  // A generator's mask, when it has one (has_mask false: mask() is 0 everywhere).
  virtual bool has_mask(terrain_mask) const { return false; }
  virtual double mask(terrain_mask, double /*x*/, double /*y*/) const { return 0.0; }
};

// A gridded relief: rows x cols samples over [min_x, max_x] x [min_y, max_y], corner-inclusive
// like grid_spec (row 0 = min_y, x fastest), bilinear between them and clamped to the edge
// outside. Masks are optional planes on the same grid.
class raster_heightfield : public terrain_source {
public:
  // relief_m <= 0 takes the samples' range (at least 1 m).
  raster_heightfield(int rows, int cols, double min_x, double min_y, double max_x, double max_y,
                     std::vector<float> heights, double sea_level_m = 0.0, double relief_m = 0.0);

  double sample(double x, double y) const override;
  // The centred difference is taken over at least one cell, so the slope is that of the relief,
  // not of the bilinear patch's kinks.
  double slope_deg(double x, double y, double h_m = 1.0) const override;
  terrain_bounds bounds() const override;
  double sea_level_m() const override { return sea_; }
  double relief_m() const override { return relief_; }
  bool has_mask(terrain_mask m) const override;
  double mask(terrain_mask m, double x, double y) const override;

  // A mask plane, rows x cols values in [0, 1] (clamped). Throws std::invalid_argument on a size
  // mismatch.
  void set_mask(terrain_mask m, std::vector<float> values);

  int rows() const noexcept { return rows_; }
  int cols() const noexcept { return cols_; }
  const std::vector<float> &heights() const noexcept { return h_; }
  double cell_w() const noexcept { return (max_x_ - min_x_) / double(cols_ - 1); }
  double cell_h() const noexcept { return (max_y_ - min_y_) / double(rows_ - 1); }
  float height_min() const noexcept { return lo_; }
  float height_max() const noexcept { return hi_; }

private:
  double bilinear(const std::vector<float> &v, double x, double y) const;
  int rows_, cols_;
  double min_x_, min_y_, max_x_, max_y_;
  std::vector<float> h_;
  std::vector<float> masks_[terrain_mask_count];
  float lo_ = 0.0f, hi_ = 0.0f;
  double sea_, relief_;
};

// Any source sampled onto g's grid (and its masks, where it has them): a raster_heightfield with
// the same datum.
raster_heightfield sample_terrain(const terrain_source &t, const grid_spec &g);

// A relief from an elevation .npy (a 2-D float32 / float64 / int16 / uint16 / int32 / uint8 array,
// C order, row 0 = min y) laid over [-half_m, half_m]^2 and anchored: its lowest sample at floor_m
// and its highest relief_m above that (relief_m <= 0 keeps the file's metres, only shifted). Throws
// std::runtime_error when the file is missing, malformed or under 2 x 2.
raster_heightfield load_dem(const std::string &path, double half_m, double relief_m,
                            double floor_m = 0.0);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_TERRAIN_SOURCE_H
