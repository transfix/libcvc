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

#include <cstdint>
#include <cvc/world/world_geometry.h>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace cvc {
class app;
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

// Version 2 (ARIADNE_WORLDS section 18.8): who made the world, so a client with the same generator
// can make it again from its seed and compare hashes. params is a JSON object's text.
struct city_bundle_generator {
  std::string name; // empty: not generated (a captured city)
  std::string version;
  std::uint64_t seed = 0;
  std::string params = "{}";
};

// The part of the world kept (a crop): the grid, the colliders and the edge walls follow it.
struct city_bundle_region {
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
};

struct city_bundle {
  std::string dir;
  world_geometry geometry; // "terrain" and, when loaded, "buildings"; version 2: every item
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0; // the terrain's extent
  std::string satellite;                                     // satellite.png's path, or empty
  bool buildings_loaded = false;
  std::string buildings_error; // why the buildings are missing, when they are
  bool land_cover_loaded = false;

  // Version 2 only (version 1 leaves them at their defaults).
  int version = 1;
  std::string hash; // world.json's content hash (city_bundle_hash)
  city_bundle_generator generator;
  std::optional<double> water_level;        // the still water's level (m), if the world has one
  std::optional<city_bundle_region> region; // the cut the world was made to, if any
};

// Load the bundle in `dir`. Throws std::runtime_error when terrain.json is missing
// or malformed, std::invalid_argument for a surface name not in the table or land
// cover of the wrong size; missing or unreadable buildings leave buildings_loaded
// false and say why in buildings_error (the terrain alone is still a world). The
// buildings are read with ctx's model handlers.
city_bundle load_city_bundle(app &ctx, const std::string &dir,
                             const city_bundle_options &options = {});

// The same from files held somewhere other than a directory (an archive unpacked in memory, a
// download): `read(name)` gives the bytes of the bundle's file `name` ("terrain.json",
// "buildings.glb", "buildings_flat.glb"), or null when it has none, and `label` names the bundle
// in errors. Nothing is read from disk, so the result has no `dir` or `satellite` path, and
// options.land_cover (which reads a directory) is not called. Throws as the directory form does.
using city_bundle_reader =
    std::function<std::shared_ptr<const std::string>(const std::string &name)>;
city_bundle load_city_bundle(app &ctx, const city_bundle_reader &read, const std::string &label,
                             const city_bundle_options &options = {});

// ── Version 2: writing (ARIADNE_WORLDS section 18.8) ────────────────────────────────────────────
//
// A world_geometry as a version 2 bundle, the files in memory (name -> bytes), so the browser
// writes what natively does and a hash needs no disk:
//   world.json            the manifest: format "cvc.city_bundle", version 2, the surface table by
//                         name (the ids in the files index it), every item (name, file, roles,
//                         surface, position, rotation), edge_walls, region, water_level, the
//                         generator, and the content hash
//   terrain.json          the terrain's extent and size, its heights in terrain.f32 (grid_file)
//   terrain.f32           little-endian float32 heights, rows x cols, north first
//   terrain_surfaces.u16  little-endian uint16, one surface per cell, north first (nav and
//                         contact materials), when the terrain has cell materials
//   <item>.glb            every mesh item: one mesh per surface its faces use, the material named
//                         by the surface; positions are 32-bit floats
// plus `extra` files as given (streets.json, ground.png). Version 1 readers read the terrain
// only if they know grid_file.
//
// The world's terrain is its first height field; it must have one. Throws std::invalid_argument
// when it has none, an item name is not a plain file name, or an extra file's name is one the
// bundle writes itself; unsupported_model_file_type when no backend writes glb.
using city_bundle_files = std::map<std::string, std::shared_ptr<const std::string>>;

struct city_bundle_write {
  city_bundle_generator generator;
  std::optional<double> water_level;
  std::optional<city_bundle_region> region;
  city_bundle_files extra; // more files, written and hashed as they are
};

city_bundle_files city_bundle_bytes(app &ctx, const world_geometry &world,
                                    const city_bundle_write &info = {});

// The content hash: SHA-256 (hex) over every file but world.json, in name order, each as its name,
// a NUL, and its bytes' SHA-256 (hex). The same contents hash the same, whoever wrote them.
std::string city_bundle_hash(const city_bundle_files &files);

// Write the files into `dir` (created if absent). Throws std::runtime_error on an I/O failure.
void write_city_bundle(const std::string &dir, const city_bundle_files &files);

// A reader over files in memory (city_bundle_bytes' result, an unpacked zip), for
// load_city_bundle.
city_bundle_reader city_bundle_memory_reader(const city_bundle_files &files);

// Version 2 is read by load_city_bundle when the bundle has a world.json: every item with its
// faces' surfaces, the terrain's cell surfaces, edge_walls, and the manifest's generator, water
// level, region and hash; the options' surfaces, row order and land cover do not apply. Throws
// std::runtime_error on a malformed manifest, a missing file it names, or a surface not in the
// table its ontology names.

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
