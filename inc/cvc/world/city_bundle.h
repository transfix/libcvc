/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::world::load_city_bundle — a city scene bundle as the world's one
// representation (world_geometry.h): the terrain as a cvc::height_field and the
// buildings as one mesh, ready for the renderer, the physics world and the
// navigator alike (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3).
//
// The bundle (cvc-scenes, e.g. austin_south):
//   terrain.json     {"bounds": {min_x, min_y, max_x, max_y}, "rows", "cols",
//                     "grid": [[...] x rows]}: corner samples over the bounds
//   buildings.glb    the buildings, world metres, z up, already at terrain height
//   buildings_flat.glb  (optional) the same with flat roofs
//   satellite.png, foliage_mask.png, roads.png, water.png, open_fields.png
//                    (optional) aerial imagery and land-cover masks
//
// Conventions, measured on austin_south (5636 buildings) rather than assumed:
// terrain.json's grid runs NORTH FIRST (grid[0] is max_y): every building's base
// sits on the terrain read that way (RMS 0.46 m, 97.3% within 1 m) and 14.3 m off
// read south first. The loader flips it into the world frame, row 0 at the
// minimum y, so nothing downstream ever flips again. The buildings are z up (not
// glTF's y up) and their bases already sit at terrain height, so they are not
// lifted. Both are options, for a bundle written the other way.
//
// Surfaces: buildings take options.building_surface. Terrain cells take the
// surfaces options.land_cover returns, one per sample, reduced to one per cell by
// the majority of its four corners; without it every cell is
// options.terrain_surface. cvc::world never links cvc::nav (roadmap §14.4, enforced
// at configure time), so land-cover classification of the bundle's masks and
// imagery (cvc::nav::segment_scene_material) plugs in from outside through
// land_cover rather than being called here.

#ifndef CVC_WORLD_CITY_BUNDLE_H
#define CVC_WORLD_CITY_BUNDLE_H

#include <cvc/world/world_geometry.h>
#include <functional>
#include <string>

namespace cvc {
namespace world {

struct city_bundle_options {
  enum class row_order { north_first, south_first };
  enum class up_axis { z, y };

  row_order terrain_rows = row_order::north_first; // terrain.json's grid (see above)
  bool buildings = true;                           // load buildings.glb
  bool prefer_flat = false;                        // buildings_flat.glb when present
  up_axis buildings_up = up_axis::z;               // y: convert a standard glTF
  std::string building_surface = "building_concrete";
  std::string terrain_surface = "dirt"; // cells with no land cover
  // Per-sample surface ids for the terrain, rows x cols in the world's row order
  // (row 0 at the minimum y), or empty for none. Called with the bundle directory.
  std::function<std::vector<std::uint16_t>(const std::string &dir, int rows, int cols)> land_cover;
};

struct city_bundle {
  std::string dir;
  world_geometry geometry; // "terrain" and, when loaded, "buildings"
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0; // the terrain's extent
  std::string satellite;                                     // satellite.png's path, or empty
  bool buildings_loaded = false;
  std::string buildings_error; // why the buildings are missing, when they are
  bool land_cover_loaded = false;
};

// Load the bundle in `dir`. Throws std::runtime_error when terrain.json is missing
// or malformed, std::invalid_argument for a surface name not in the table or land
// cover of the wrong size; missing or unreadable buildings leave buildings_loaded
// false and say why in buildings_error (the terrain alone is still a world).
city_bundle load_city_bundle(const std::string &dir, const city_bundle_options &options = {});

// terrain.json's grid size and extent alone: what a reader of the bundle's other
// rasters (land-cover masks, imagery) lays them over. The height grid may be
// absent. Throws std::runtime_error like load_city_bundle.
struct terrain_extent {
  int rows = 0, cols = 0;
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
};
terrain_extent read_terrain_extent(const std::string &dir);

// One surface per cell from per-sample surfaces (rows x cols): the most common of
// its four corners, the south-west corner's on a tie.
std::vector<cvc::face_material_t> cell_surfaces(const std::vector<std::uint16_t> &samples, int rows,
                                                int cols);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_CITY_BUNDLE_H
