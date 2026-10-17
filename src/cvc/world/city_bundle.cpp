/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <cvc/world/city_bundle.h>
#include <filesystem>
#include <stdexcept>

namespace cvc {
namespace world {

namespace {

namespace fs = std::filesystem;

std::uint16_t surface_id(const surface_registry &surfaces, const std::string &name,
                         const char *what) {
  std::uint16_t id = 0;
  if (!surfaces.find(name, id))
    throw std::invalid_argument(std::string("load_city_bundle: ") + what + " '" + name +
                                "' is not in the surface table");
  return id;
}

struct terrain_grid {
  int rows = 0, cols = 0;
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  std::vector<float> heights; // row-major as stored in the file
};

boost::property_tree::ptree read_json_file(const fs::path &path) {
  if (!fs::exists(path))
    throw std::runtime_error("load_city_bundle: no terrain.json in " + path.parent_path().string());
  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_json(path.string(), pt);
  } catch (const std::exception &e) {
    throw std::runtime_error("load_city_bundle: " + path.string() + " is not JSON: " + e.what());
  }
  return pt;
}

// The fields every terrain.json has, grid or not: the one place they are read.
void read_extent(const boost::property_tree::ptree &pt, const fs::path &path, terrain_grid &t) {
  try {
    t.min_x = pt.get<double>("bounds.min_x");
    t.min_y = pt.get<double>("bounds.min_y");
    t.max_x = pt.get<double>("bounds.max_x");
    t.max_y = pt.get<double>("bounds.max_y");
    t.rows = pt.get<int>("rows");
    t.cols = pt.get<int>("cols");
  } catch (const std::exception &e) {
    throw std::runtime_error("load_city_bundle: " + path.string() + ": " + e.what());
  }
  if (t.rows < 2 || t.cols < 2 || !(t.max_x > t.min_x) || !(t.max_y > t.min_y))
    throw std::runtime_error("load_city_bundle: " + path.string() +
                             ": needs at least 2 x 2 samples over a non-empty extent");
}

terrain_grid read_terrain_json(const fs::path &path) {
  const boost::property_tree::ptree pt = read_json_file(path);
  terrain_grid t;
  read_extent(pt, path, t);
  const auto grid = pt.get_child_optional("grid");
  if (!grid || grid->size() != static_cast<std::size_t>(t.rows))
    throw std::runtime_error("load_city_bundle: " + path.string() +
                             ": the grid does not have `rows` rows");
  t.heights.reserve(static_cast<std::size_t>(t.rows) * t.cols);
  for (const auto &row : *grid) {
    if (row.second.size() != static_cast<std::size_t>(t.cols))
      throw std::runtime_error("load_city_bundle: " + path.string() +
                               ": a grid row does not have `cols` samples");
    for (const auto &v : row.second) {
      const double h = v.second.get_value<double>();
      if (!std::isfinite(h))
        throw std::runtime_error("load_city_bundle: " + path.string() + ": a height is not finite");
      t.heights.push_back(static_cast<float>(h));
    }
  }
  return t;
}

} // namespace

std::vector<cvc::face_material_t> cell_surfaces(const std::vector<std::uint16_t> &samples, int rows,
                                                int cols) {
  if (rows < 2 || cols < 2 || samples.size() != static_cast<std::size_t>(rows) * cols)
    throw std::invalid_argument("cell_surfaces: rows x cols samples are needed");
  std::vector<cvc::face_material_t> cells;
  cells.reserve(static_cast<std::size_t>(rows - 1) * (cols - 1));
  auto at = [&](int r, int c) { return samples[static_cast<std::size_t>(r) * cols + c]; };
  for (int r = 0; r + 1 < rows; ++r)
    for (int c = 0; c + 1 < cols; ++c) {
      const std::uint16_t corner[4] = {at(r, c), at(r, c + 1), at(r + 1, c + 1), at(r + 1, c)};
      std::uint16_t best = corner[0];
      int best_n = 0;
      for (const std::uint16_t v : corner) {
        int n = 0;
        for (const std::uint16_t w : corner)
          n += (w == v);
        if (n > best_n) {
          best = v;
          best_n = n;
        }
      }
      cells.push_back(best);
    }
  return cells;
}

terrain_extent read_terrain_extent(const std::string &dir) {
  const fs::path path = fs::path(dir) / "terrain.json";
  terrain_grid t;
  read_extent(read_json_file(path), path, t);
  return {t.rows, t.cols, t.min_x, t.min_y, t.max_x, t.max_y};
}

city_bundle load_city_bundle(const std::string &dir, const city_bundle_options &options) {
  const fs::path root(dir);
  city_bundle out;
  out.dir = dir;
  const surface_registry &surfaces = out.geometry.surfaces();
  const std::uint16_t ground = surface_id(surfaces, options.terrain_surface, "terrain_surface");
  const std::uint16_t building = surface_id(surfaces, options.building_surface, "building_surface");

  // --- terrain ----------------------------------------------------------------
  const terrain_grid t = read_terrain_json(root / "terrain.json");
  out.min_x = t.min_x;
  out.min_y = t.min_y;
  out.max_x = t.max_x;
  out.max_y = t.max_y;
  std::vector<float> heights(t.heights.size());
  for (int i = 0; i < t.rows; ++i) {
    // File row i is world row r: flipped when the file runs north first.
    const int r =
        options.terrain_rows == city_bundle_options::row_order::north_first ? t.rows - 1 - i : i;
    std::copy(t.heights.begin() + static_cast<std::size_t>(i) * t.cols,
              t.heights.begin() + static_cast<std::size_t>(i + 1) * t.cols,
              heights.begin() + static_cast<std::size_t>(r) * t.cols);
  }
  std::vector<cvc::face_material_t> cells;
  if (options.land_cover) {
    const std::vector<std::uint16_t> samples = options.land_cover(dir, t.rows, t.cols);
    if (!samples.empty()) {
      if (samples.size() != t.heights.size())
        throw std::invalid_argument("load_city_bundle: land_cover returned " +
                                    std::to_string(samples.size()) + " surfaces for " +
                                    std::to_string(t.heights.size()) + " samples");
      for (const std::uint16_t id : samples)
        if (id >= surfaces.size())
          throw std::invalid_argument("load_city_bundle: land_cover returned a surface id "
                                      "outside the surface table");
      cells = cell_surfaces(samples, t.rows, t.cols);
      out.land_cover_loaded = true;
    }
  }
  auto field = std::make_shared<cvc::height_field>(cvc::height_field::from_bounds(
      t.rows, t.cols, t.min_x, t.min_y, t.max_x, t.max_y, std::move(heights)));
  if (!cells.empty())
    field->set_cell_materials(std::move(cells));
  geometry_item terrain;
  terrain.name = "terrain";
  terrain.geometry = field;
  terrain.surface = ground;
  out.geometry.add(terrain);

  if (fs::exists(root / "satellite.png"))
    out.satellite = (root / "satellite.png").string();

  // --- buildings --------------------------------------------------------------
  if (!options.buildings) {
    out.buildings_error = "not requested";
    return out;
  }
  fs::path glb = root / "buildings.glb";
  if (options.prefer_flat && fs::exists(root / "buildings_flat.glb"))
    glb = root / "buildings_flat.glb";
  if (!fs::exists(glb)) {
    out.buildings_error = "no " + glb.filename().string() + " in " + dir;
    return out;
  }
  try {
    cvc::model m = cvc::read_model(glb.string());
    for (auto &mesh : m.meshes)
      mesh.geom.face_materials().assign(mesh.geom.num_faces(), building);
    auto mesh = std::make_shared<cvc::geometry>(m.merged());
    if (options.buildings_up == city_bundle_options::up_axis::y) {
      // y up to z up: (x, y, z) -> (x, -z, y), a rotation, so normals turn the same way.
      for (auto &p : mesh->points()) {
        const double y = p[1];
        p[1] = -p[2];
        p[2] = y;
      }
      for (auto &n : mesh->normals()) {
        const double y = n[1];
        n[1] = -n[2];
        n[2] = y;
      }
    }
    if (mesh->num_faces() == 0) {
      out.buildings_error = glb.filename().string() + " has no faces";
      return out;
    }
    geometry_item buildings;
    buildings.name = "buildings";
    buildings.geometry = mesh;
    buildings.surface = building;
    out.geometry.add(buildings);
    out.buildings_loaded = true;
  } catch (const std::exception &e) {
    out.buildings_error = glb.filename().string() + ": " + e.what();
  }
  return out;
}

} // namespace world
} // namespace cvc
