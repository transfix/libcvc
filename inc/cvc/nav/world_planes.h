/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::rasterize — the navigator's planes from the world's one
// representation (cvc::world::world_geometry; docs/roadmap/WORLD-GEOMETRY-
// ROADMAP.md, G4): the same terrain and buildings the renderer draws and the
// physics world collides with, sampled onto a nav grid.
//
//   const auto city = cvc::world::load_city_bundle(dir, {..., land_cover});
//   const auto planes = cvc::nav::rasterize(city.geometry, 96, 96);
//   sim_world::from_occupancy(cfg, planes.occupancy.data(), ...);
//
// The grid: corner samples, sample (r, c) at (min_x + c*dx, min_y + r*dy) with
// dx = (max_x - min_x)/(cols - 1), row 0 at min_y -- the convention every nav
// raster uses (occupancy, EDT, sim_world, material planes).
//
// Planes, one value per sample:
//   occupancy  1 where a navigate-role mesh covers the sample (its triangles
//              projected on the ground; a building's roof covers its footprint)
//              or the terrain's surface there is hard (deep water, a cliff);
//              then dilated by `inflate` 4-connected cells.
//   height     the terrain's height (cvc::height_field::height), 0 without one.
//   surface    the surface there: the highest covering face's, else the
//              terrain cell's, else the terrain item's.
//   risk_raw, hard, mu   that surface's rho, hard and mu (surface.h).
//
// The occupancy test is the demos' occupancy_from_model, unchanged: a sample
// is blocked when it lies inside a projected triangle of either winding, with
// no conservative fattening (that would seal dense downtown streets).

#ifndef CVC_NAV_WORLD_PLANES_H
#define CVC_NAV_WORLD_PLANES_H

#include <cstdint>
#include <cvc/world/world_geometry.h>
#include <vector>

namespace cvc {
namespace nav {

struct world_planes {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  std::vector<std::uint8_t> occupancy; // rows*cols, row-major, row 0 at min_y
  std::vector<float> height;
  std::vector<std::uint16_t> surface;
  std::vector<float> risk_raw;
  std::vector<std::uint8_t> hard;
  std::vector<float> mu;
};

// Rasterize `g` over the given extent. Throws std::invalid_argument for fewer
// than 2 x 2 samples or an empty extent.
world_planes rasterize(const cvc::world::world_geometry &g, int rows, int cols, double min_x,
                       double min_y, double max_x, double max_y, int inflate = 0);
// The same over the terrain's extent. Throws std::invalid_argument without a terrain.
world_planes rasterize(const cvc::world::world_geometry &g, int rows, int cols, int inflate = 0);

// Just the footprint of one mesh on the grid (the occupancy test above, without
// dilation): what occupancy_from_model computed for the demos.
std::vector<std::uint8_t> mesh_footprint(const cvc::geometry &mesh, int rows, int cols,
                                         double min_x, double min_y, double max_x, double max_y);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_WORLD_PLANES_H
