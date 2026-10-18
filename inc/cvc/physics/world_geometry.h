/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::add_world_geometry — the world's static geometry as physics
// bodies, and its volumes as fields: the same height fields, meshes and volumes
// the renderer draws, never a copy (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3
// and G6).
//
//   const auto city = cvc::world::load_city_bundle(dir);
//   cvc::physics::world w(cfg);
//   const auto bodies = cvc::physics::add_world_geometry(w, city.geometry);
//   bodies.find("terrain");          // the terrain's body
//   bodies.find_field("lake");       // a volume's field
//
// Not standard-library only, unlike the rest of cvc/physics: it reads
// cvc::world::world_geometry.

#ifndef __CVC_PHYSICS_WORLD_GEOMETRY_H__
#define __CVC_PHYSICS_WORLD_GEOMETRY_H__

#include <cvc/physics/world.h>
#include <cvc/world/world_geometry.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace physics {

struct world_bodies {
  // One static body per item with role_collide, in item order.
  std::vector<std::pair<std::string, body_id>> bodies;
  // One field per volume with role_collide, in volume order.
  std::vector<std::pair<std::string, field_id>> fields;
  // surface id -> this world's material id, for every surface the items use (0, the
  // default material, for the rest).
  std::vector<material_id> materials;
  // The body of item `name`, or an invalid id.
  body_id find(const std::string &name) const;
  // The field of volume `name`, or an invalid id.
  field_id find_field(const std::string &name) const;
};

// Add a static body for every collide item of `g`: a height field as terrain, a
// mesh at its item's placement; and a field (world::add_field) for every collide
// volume. Each surface the items use becomes a world
// material named after the surface, with its mu, restitution and rolling
// resistance, and nav_material from `nav_materials` (indexed by surface id; -1
// for a surface it does not cover, or when it is empty). cvc::nav::palette_ids()
// makes that table; physics does not link the navigator, so it is passed in. A
// material of that name already in the world is reused, so a second
// world_geometry sharing surfaces adds no duplicates. `max_error` is the
// terrain's height tolerance (physics::terrain). Throws cvc::physics_error as
// world::add_body and world::add_field do; bodies and fields added before a
// failure stay.
world_bodies add_world_geometry(world &w, const cvc::world::world_geometry &g,
                                double max_error = 0.005,
                                const std::vector<int> &nav_materials = {});

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_WORLD_GEOMETRY_H__
