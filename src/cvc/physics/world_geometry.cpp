/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/physics/world_geometry.h>

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

  for (const cvc::world::geometry_item &item : g.items()) {
    if (!item.has(cvc::world::role_collide))
      continue;
    body_desc d;
    d.motion = motion_type::static_;
    d.material = out.materials[item.surface];
    if (auto field = item.height_field()) {
      d.shape = terrain{field, out.materials, max_error};
    } else {
      d.shape = mesh{item.geometry, out.materials};
      d.position = {item.position[0], item.position[1], item.position[2]};
      d.rotation = {item.rotation[0], item.rotation[1], item.rotation[2], item.rotation[3]};
    }
    out.bodies.emplace_back(item.name, w.add_body(d));
  }
  for (const cvc::world::volume_item &v : g.volumes())
    if (v.has(cvc::world::role_collide))
      out.fields.emplace_back(v.name, w.add_field(v.field));
  return out;
}

} // namespace physics
} // namespace cvc
