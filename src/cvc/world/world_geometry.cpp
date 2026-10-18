/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/world/world_geometry.h>
#include <set>
#include <stdexcept>

namespace cvc {
namespace world {

std::shared_ptr<const cvc::height_field> geometry_item::height_field() const {
  return std::dynamic_pointer_cast<const cvc::height_field>(geometry);
}

world_geometry::world_geometry(const surface_registry &surfaces) : _surfaces(&surfaces) {}

void world_geometry::add(geometry_item item) {
  if (item.name.empty())
    throw std::invalid_argument("world_geometry::add: an item needs a name");
  if (find(item.name))
    throw std::invalid_argument("world_geometry::add: an item called '" + item.name +
                                "' already exists");
  if (!item.geometry)
    throw std::invalid_argument("world_geometry::add: '" + item.name + "' has no geometry");
  const std::size_t n = _surfaces->size();
  if (item.surface >= n)
    throw std::invalid_argument("world_geometry::add: '" + item.name +
                                "': surface id outside the surface table");
  auto check = [&](const std::vector<cvc::face_material_t> &ids, const char *what) {
    for (const cvc::face_material_t id : ids)
      if (id >= n)
        throw std::invalid_argument("world_geometry::add: '" + item.name + "': a " + what +
                                    " is outside the surface table");
  };
  if (const auto hf = item.height_field())
    check(hf->cell_materials(), "cell material");
  else if (item.geometry->has_face_materials())
    check(item.geometry->const_face_materials(), "face material");
  const auto &q = item.rotation;
  const double len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (!(std::fabs(len - 1.0) < 1e-6))
    throw std::invalid_argument("world_geometry::add: '" + item.name +
                                "': the rotation is not a unit quaternion");
  for (const double p : item.position)
    if (!std::isfinite(p))
      throw std::invalid_argument("world_geometry::add: '" + item.name +
                                  "': the position is not finite");
  _items.push_back(std::move(item));
}

const geometry_item *world_geometry::find(const std::string &name) const {
  for (const geometry_item &i : _items)
    if (i.name == name)
      return &i;
  return nullptr;
}

std::shared_ptr<const cvc::height_field> world_geometry::terrain() const {
  for (const geometry_item &i : _items)
    if (auto hf = i.height_field())
      return hf;
  return nullptr;
}

world_bounds world_geometry::bounds() const {
  world_bounds b;
  auto grow = [&](double x, double y, double z) {
    const double v[3] = {x, y, z};
    for (int k = 0; k < 3; ++k) {
      b.min[k] = std::min(b.min[k], v[k]);
      b.max[k] = std::max(b.max[k], v[k]);
    }
  };
  for (const geometry_item &i : _items) {
    const auto &pts = i.geometry->const_points();
    if (pts.empty())
      continue;
    const bool placed = !i.height_field();
    const auto &q = i.rotation;
    for (const auto &p : pts) {
      double x = p[0], y = p[1], z = p[2];
      if (placed) {
        // v' = q v q*, then the translation.
        const double w = q[0], qx = q[1], qy = q[2], qz = q[3];
        const double tx = 2 * (qy * z - qz * y), ty = 2 * (qz * x - qx * z),
                     tz = 2 * (qx * y - qy * x);
        const double rx = x + w * tx + (qy * tz - qz * ty);
        const double ry = y + w * ty + (qz * tx - qx * tz);
        const double rz = z + w * tz + (qx * ty - qy * tx);
        x = rx + i.position[0];
        y = ry + i.position[1];
        z = rz + i.position[2];
      }
      if (b.empty()) {
        b.min = b.max = {{x, y, z}};
      } else {
        grow(x, y, z);
      }
    }
  }
  return b;
}

std::vector<std::uint16_t> world_geometry::surfaces_used() const {
  std::set<std::uint16_t> used;
  for (const geometry_item &i : _items) {
    if (const auto hf = i.height_field()) {
      if (hf->cell_materials().empty())
        used.insert(i.surface);
      else
        used.insert(hf->cell_materials().begin(), hf->cell_materials().end());
    } else if (i.geometry->has_face_materials()) {
      const auto &fm = i.geometry->const_face_materials();
      used.insert(fm.begin(), fm.end());
    } else {
      used.insert(i.surface);
    }
  }
  return std::vector<std::uint16_t>(used.begin(), used.end());
}

} // namespace world
} // namespace cvc
