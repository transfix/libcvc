/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::world_grid: a world as the navigator sees it. One world (a generated world, a data
// bundle's city, a flat square; cvc::world::world_geometry) becomes the grid sim_world plans on:
// its occupancy with a wall round the edge, the terrain's height, the signed distance to anything
// blocked, and free points to start or aim agents at. This is the world-to-navigator glue the lab's
// Python carried (pycvc_gl.cars' World), in C++ for the world runtime (ARIADNE_WORLDS.md section
// 14).
//
//   const auto g = cvc::nav::make_grid(world, 2.0);       // the terrain's extent, 2 m cells
//   const auto sdf = cvc::nav::signed_distance(g);
//   const auto starts = cvc::nav::free_points(g, sdf, seed, n, 4.0, 10.0);
//   sim_world::config cfg;
//   cvc::nav::configure(cfg, g, 0.05);                    // rows, cols, extent, centre, scale
//
// The grid is world_planes' (corner samples, sample (r, c) at (min_x + c*cell, min_y + r*cell), row
// 0 at min_y), with square cells. occupancy is world_planes' occupancy (navigate-role meshes, hard
// ground, deep water) with the outermost rows and columns set, so no agent leaves the grid.

#ifndef CVC_NAV_WORLD_GRID_H
#define CVC_NAV_WORLD_GRID_H

#include <array>
#include <cstdint>
#include <vector>

namespace cvc {
namespace world {
class world_geometry;
}
namespace nav {

struct world_grid {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  std::vector<std::uint8_t> occupancy; // rows*cols, row-major, row 0 at min_y; 1 = blocked
  std::vector<float> height;           // rows*cols: the terrain's height, 0 without one

  double cell() const { return cols > 1 ? (max_x - min_x) / (cols - 1) : 0.0; }
  double center_x() const { return 0.5 * (min_x + max_x); }
  double center_y() const { return 0.5 * (min_y + max_y); }
  // The sample nearest (x, y), clamped to the grid.
  void sample_of(double x, double y, int &r, int &c) const;
  // Blocked, or off the grid.
  bool occupied(double x, double y) const;
  // The blocked samples, the wall included.
  std::size_t occupied_count() const;
};

// The grid of `world` over (min_x, min_y)-(max_x, max_y) with `cell` m samples: the extent is cut
// to whole cells from its minimum corner. `inflate` dilates the obstacles by that many cells before
// the wall is added. Throws std::invalid_argument for a cell <= 0 or fewer than 3 x 3 samples.
world_grid make_grid(const cvc::world::world_geometry &world, double min_x, double min_y,
                     double max_x, double max_y, double cell, int inflate = 0);
// The same over the world's terrain, or, with `half` > 0, a square that many metres either side of
// the terrain's centre (cut to the terrain). Throws std::invalid_argument without a terrain.
world_grid make_grid(const cvc::world::world_geometry &world, double cell, double half = 0.0,
                     int inflate = 0);
// An open square `half` m either side of the origin, walled at its edge, flat at 0.
world_grid flat_grid(double half, double cell);
// An open grid over (min_x, min_y)-(max_x, max_y), cut to whole cells and walled at its edge, flat
// at height `z`.
world_grid flat_grid(double min_x, double min_y, double max_x, double max_y, double cell,
                     float z = 0.0f);

// The signed distance (m) from each sample to the nearest blocked one: positive in the free
// samples, negative inside the blocked ones (to the nearest free one). rows*cols.
std::vector<float> signed_distance(const world_grid &g);

// `n` points at least `clearance` m from anything blocked (by `sdf`, signed_distance(g)) and at
// least `min_sep` m apart, drawn from `seed` the same way on every platform. Where no sample has
// that clearance, any free sample will do; a point that cannot be kept `min_sep` from the others
// after 400 draws takes the last one drawn. Throws std::runtime_error when nothing is free.
std::vector<std::array<double, 2>> free_points(const world_grid &g, const std::vector<float> &sdf,
                                               std::uint64_t seed, int n, double clearance,
                                               double min_sep);

// Fill a navigator's grid fields from `g`: rows, cols, the extent, the centre (cx, cy) and `scale`
// (normalized units per metre). Its other fields are left as they are.
template <class Config> void configure(Config &cfg, const world_grid &g, double scale) {
  cfg.rows = g.rows;
  cfg.cols = g.cols;
  cfg.min_x = g.min_x;
  cfg.min_y = g.min_y;
  cfg.max_x = g.max_x;
  cfg.max_y = g.max_y;
  cfg.cx = g.center_x();
  cfg.cy = g.center_y();
  cfg.scale = scale;
}

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_WORLD_GRID_H
