/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/physics/world_geometry.h>
#include <limits>

namespace cvc {
namespace physics {

body_id world_bodies::find(const std::string &name) const {
  for (const auto &b : bodies)
    if (b.first == name)
      return b.second;
  return body_id{};
}

field_id world_bodies::find_field(const std::string &name) const {
  for (const auto &f : fields)
    if (f.first == name)
      return f.second;
  return field_id{};
}

world_bodies add_world_geometry(world &w, const cvc::world::world_geometry &g, double max_error,
                                const std::vector<int> &nav_materials) {
  world_bodies out;
  const cvc::world::surface_registry &surfaces = g.surfaces();
  out.materials.assign(surfaces.size(), 0);

  // One world material per surface used, reusing one of the same name.
  for (const std::uint16_t s : g.surfaces_used()) {
    const cvc::world::surface_class &c = surfaces[s];
    material_id id = 0;
    bool found = false;
    for (std::size_t i = 0; i < w.material_count() && !found; ++i)
      if (w.get_material(static_cast<material_id>(i)).name == c.name) {
        id = static_cast<material_id>(i);
        found = true;
      }
    if (!found)
      id = w.add_material({c.name, c.mu, c.restitution, c.rolling_resistance,
                           s < nav_materials.size() ? nav_materials[s] : -1});
    out.materials[s] = id;
  }

  // The rectangle and the heights the collide height fields cover, for the edge walls.
  constexpr double inf = std::numeric_limits<double>::infinity();
  double x0 = inf, y0 = inf, x1 = -inf, y1 = -inf, z0 = inf, z1 = -inf;
  material_id edge_material = 0;
  for (const cvc::world::geometry_item &item : g.items()) {
    if (!item.has(cvc::world::role_collide))
      continue;
    const auto field = item.height_field();
    if (field && field->cell_rows() > 0 && field->cell_cols() > 0) {
      if (x0 == inf)
        edge_material = out.materials[item.surface];
      x0 = std::min(x0, field->x0());
      y0 = std::min(y0, field->y0());
      x1 = std::max(x1, field->max_x());
      y1 = std::max(y1, field->max_y());
      for (const float h : field->heights())
        if (std::isfinite(h)) {
          z0 = std::min(z0, double(h));
          z1 = std::max(z1, double(h));
        }
    }
    body_desc d;
    d.motion = motion_type::static_;
    d.material = out.materials[item.surface];
    if (field) {
      d.shape = terrain{field, out.materials, max_error};
    } else {
      d.shape = mesh{item.geometry, out.materials};
      d.position = {item.position[0], item.position[1], item.position[2]};
      d.rotation = {item.rotation[0], item.rotation[1], item.rotation[2], item.rotation[3]};
    }
    out.bodies.emplace_back(item.name, w.add_body(d));
  }
  if (x0 < x1 && y0 < y1 && z0 <= z1) {
    const double t = world_bodies::kEdgeWallThickness, r = world_bodies::kEdgeWallReach;
    x0 -= world_bodies::kEdgeWallGap;
    y0 -= world_bodies::kEdgeWallGap;
    x1 += world_bodies::kEdgeWallGap;
    y1 += world_bodies::kEdgeWallGap;
    const double zlo = z0 - r, zhi = z1 + r;
    const double xc = 0.5 * (x0 + x1), yc = 0.5 * (y0 + y1);
    // West and east run the full length, past the corners; south and north fit between them.
    const double hx = 0.5 * (x1 - x0), hy = 0.5 * (y1 - y0) + t, hz = 0.5 * (zhi - zlo);
    const double walls[4][4] = {{x0 - 0.5 * t, yc, 0.5 * t, hy},
                                {x1 + 0.5 * t, yc, 0.5 * t, hy},
                                {xc, y0 - 0.5 * t, hx, 0.5 * t},
                                {xc, y1 + 0.5 * t, hx, 0.5 * t}};
    for (const auto &wall : walls) {
      body_desc d;
      d.motion = motion_type::static_;
      d.material = edge_material;
      d.shape = box{{wall[2], wall[3], hz}};
      d.position = {wall[0], wall[1], 0.5 * (zlo + zhi)};
      out.edges.push_back(w.add_body(d));
    }
  }
  for (const cvc::world::volume_item &v : g.volumes())
    if (v.has(cvc::world::role_collide))
      out.fields.emplace_back(v.name, w.add_field(v.field));
  return out;
}

} // namespace physics
} // namespace cvc
