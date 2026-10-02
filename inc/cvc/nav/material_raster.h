/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// material_raster.h — pure-C++ (cvc::nav) generator of a scene MATERIAL raster from a scene bundle:
// classify the bundle's land cover into the 13-class palette (indexed like cvc::nav::kNumMaterials
// and grl_snam's MATERIALS) on the terrain grid, and derive per-material grip (mu) + terrain
// risk. This is the pure-libcvc twin of grl-snam's `grl-snam material-raster`
// (grl_snam.tools.material_raster) — SAME algorithm, SAME material.json output
// (cvc-scene-material/1) — so a libcvc-only toolchain can produce the material input the nav-stats
// scorecard buckets by + the drive reads for grip/reroute, with no Python. Prefers the bundle's
// authoritative land-cover MASKS (the scene pipeline's OSM + ML segmentation: foliage_mask.png land
// classes + roads/water/open_fields alpha masks) and falls back to a deterministic color
// classification of satellite.png when the masks are absent (a lean bundle).

#ifndef __CVC_NAV_MATERIAL_RASTER_H__
#define __CVC_NAV_MATERIAL_RASTER_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

// The 13-class palette (index == material id); MUST match grl_snam MATERIALS and any downstream
// table keyed by these ids. Only the drivable-surface ids are ever emitted by the classifier
// (open_air/foliage/soil/water/rock); the building materials are obstacle surfaces tagged from
// scene metadata, never ground.
enum material_id : int {
  MAT_REINFORCED_CONCRETE = 0,
  MAT_BRICK = 1,
  MAT_GLASS = 2,
  MAT_WOOD = 3,
  MAT_FOLIAGE = 4,
  MAT_DRYWALL = 5,
  MAT_METAL = 6,
  MAT_OPEN_AIR = 7,
  MAT_SOIL = 8,
  MAT_WATER = 9,
  MAT_GLASS_LAMINATED = 10,
  MAT_COMPOSITE_PANEL = 11,
  MAT_ROCK = 12,
  kMaterialCount = 13,
};

// Per-material dry grip mu (1 = full grip) + terrain risk (0..1) — the physical reading of the
// palette the drive/scorecard use (mirrors grl_snam.material_palette GRIP_MU / TERRAIN_RISK).
// Unlisted ids (building materials, unknowns) default to mu=1 / risk=0.
float material_mu(int id);
float material_risk(int id);
const char *material_name(int id); // palette name, or "" out of range

// A per-cell material-id raster over a world-metre grid, oriented row 0 == min_y / col 0 == min_x
// (the nav_samplers / sim_world occupancy convention), so a runtime sampler indexes it straight.
struct material_raster {
  int rows = 0, cols = 0;
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  std::vector<int> material_id; // [rows*cols] row-major

  bool empty() const { return rows <= 0 || cols <= 0 || material_id.empty(); }
  int at(int r, int c) const { return material_id[static_cast<std::size_t>(r) * cols + c]; }
  // Serialize as the cvc-scene-material/1 material.json (id grid + palette + mu/risk tables), the
  // same schema grl-snam writes. Compact (no spaces); floats are the shortest round-trip form.
  std::string to_json() const;
  // Write to_json() to `path`. Returns false on open failure.
  bool write_json(const std::string &path) const;
};

// Build a material raster for a scene bundle directory (with terrain.json for rows/cols/bounds).
// Prefers the land-cover masks (foliage_mask.png + roads/water/open_fields.png) if present; else
// classifies <dir>/satellite.png. Throws std::runtime_error if terrain.json is missing/malformed or
// neither masks nor satellite are present.
material_raster segment_scene_material(const std::string &bundle_dir);

// Lower-level entry points (used by segment_scene_material; exposed for arbitrary inputs + tests).
// Classify a north-up RGB(A) orthophoto (interleaved `rgb`, h*w*channels bytes) into the grid.
material_raster classify_satellite(const std::uint8_t *rgb, int img_w, int img_h, int channels,
                                   int rows, int cols, double min_x, double min_y, double max_x,
                                   double max_y);

} // namespace nav
} // namespace cvc

#endif
