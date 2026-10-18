/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Land cover as world surfaces: the bridge between the navigator's material
// palette (material_raster.h) and the world's surface table (world/surface.h),
// docs/roadmap/WORLD-GEOMETRY-ROADMAP.md G4.
//
// cvc::world never links cvc::nav (the world bundle is a file format), so this
// lives here, on the nav side:
//
//   cvc::world::city_bundle_options o;
//   o.land_cover = cvc::nav::land_cover_provider(cvc::world::surface_registry::builtin());
//   const auto city = cvc::world::load_city_bundle(dir, o);  // cells carry land cover
//
// The two tables share a vocabulary: a surface's rf_material names a palette
// entry (reinforced_concrete, foliage, soil, water, rock, ...).

#ifndef CVC_NAV_LAND_COVER_H
#define CVC_NAV_LAND_COVER_H

#include <cstdint>
#include <cvc/world/surface.h>
#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// The surface for a palette id (material_raster.h's material_id) as land cover:
// open_air -> asphalt (the classifier's open ground: roads and fields),
// foliage -> tall_grass, soil -> dirt, water -> water_shallow, rock -> bare_rock,
// and the building materials to the building shells. `fallback` otherwise.
std::uint16_t surface_for_land_cover(const cvc::world::surface_registry &surfaces, int material,
                                     std::uint16_t fallback);

// The palette entry a surface carries: the one named by its rf_material, or -1.
int palette_id(const cvc::world::surface_class &surface);
// palette_id for every surface in the table, indexed by surface id: what
// cvc::physics::add_world_geometry takes to fill each material's nav_material.
std::vector<int> palette_ids(const cvc::world::surface_registry &surfaces);

// A cvc::world::city_bundle_options::land_cover provider: classifies the bundle's
// land-cover masks or satellite image (segment_scene_material) and returns one
// surface per terrain sample, `fallback` where there is nothing to classify. Empty
// (the terrain stays one surface) when the bundle has neither masks nor imagery.
std::function<std::vector<std::uint16_t>(const std::string &dir, int rows, int cols)>
land_cover_provider(const cvc::world::surface_registry &surfaces, std::uint16_t fallback = 3);

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_LAND_COVER_H
