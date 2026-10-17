/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstring>
#include <cvc/nav/land_cover.h>
#include <cvc/nav/material_raster.h>
#include <filesystem>

namespace cvc {
namespace nav {

std::uint16_t surface_for_land_cover(const cvc::world::surface_registry &surfaces, int material,
                                     std::uint16_t fallback) {
  const char *name = nullptr;
  switch (material) {
  case MAT_OPEN_AIR:
    name = "asphalt";
    break;
  case MAT_FOLIAGE:
    name = "tall_grass";
    break;
  case MAT_SOIL:
    name = "dirt";
    break;
  case MAT_WATER:
    name = "water_shallow";
    break;
  case MAT_ROCK:
    name = "bare_rock";
    break;
  case MAT_REINFORCED_CONCRETE:
    name = "building_concrete";
    break;
  case MAT_BRICK:
    name = "building_brick";
    break;
  case MAT_GLASS:
  case MAT_GLASS_LAMINATED:
    name = "building_glass";
    break;
  case MAT_DRYWALL:
    name = "building_drywall";
    break;
  case MAT_METAL:
  case MAT_COMPOSITE_PANEL:
    name = "building_metal";
    break;
  case MAT_WOOD:
    name = "tree_trunk";
    break;
  default:
    return fallback;
  }
  std::uint16_t id = fallback;
  surfaces.find(name, id);
  return id;
}

int palette_id(const cvc::world::surface_class &surface) {
  for (int id = 0; id < kMaterialCount; ++id)
    if (std::strcmp(material_name(id), surface.rf_material) == 0)
      return id;
  return -1;
}

std::vector<int> palette_ids(const cvc::world::surface_registry &surfaces) {
  std::vector<int> out(surfaces.size());
  for (std::size_t i = 0; i < out.size(); ++i)
    out[i] = palette_id(surfaces[static_cast<std::uint16_t>(i)]);
  return out;
}

std::function<std::vector<std::uint16_t>(const std::string &, int, int)>
land_cover_provider(const cvc::world::surface_registry &surfaces, std::uint16_t fallback) {
  const cvc::world::surface_registry *reg = &surfaces;
  return [reg, fallback](const std::string &dir, int rows, int cols) {
    namespace fs = std::filesystem;
    bool any = false;
    for (const char *f :
         {"foliage_mask.png", "roads.png", "water.png", "open_fields.png", "satellite.png"})
      any = any || fs::exists(fs::path(dir) / f);
    if (!any)
      return std::vector<std::uint16_t>{};
    const material_raster m = segment_scene_material(dir);
    if (m.rows != rows || m.cols != cols)
      return std::vector<std::uint16_t>{}; // a different grid: nothing to say per sample
    std::vector<std::uint16_t> out(m.material_id.size());
    for (std::size_t i = 0; i < out.size(); ++i)
      out[i] = surface_for_land_cover(*reg, m.material_id[i], fallback);
    return out;
  };
}

} // namespace nav
} // namespace cvc
