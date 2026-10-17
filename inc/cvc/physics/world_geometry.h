/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::add_world_geometry — the world's static geometry as physics
// bodies: the same height fields and meshes the renderer draws, never a copy
// (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G3).
//
//   const auto city = cvc::world::load_city_bundle(dir);
//   cvc::physics::world w(cfg);
//   const auto bodies = cvc::physics::add_world_geometry(w, city.geometry);
//   bodies.find("terrain");          // the terrain's body
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
  // surface id -> this world's material id, for every surface the items use (0, the
  // default material, for the rest).
  std::vector<material_id> materials;
  // The body of item `name`, or an invalid id.
  body_id find(const std::string &name) const;
};

// Add a static body for every collide item of `g`: a height field as terrain, a
// mesh at its item's placement. Each surface the items use becomes a world
// material named after the surface, with its mu, restitution and rolling
// resistance (nav_material stays -1 until the navigator reads the surface table
// itself). A material of that name already in the world is reused, so a
// second world_geometry sharing surfaces adds no duplicates. `max_error` is the
// terrain's height tolerance (physics::terrain). Throws cvc::physics_error as
// world::add_body does; bodies added before a failure stay.
world_bodies add_world_geometry(world &w, const cvc::world::world_geometry &g,
                                double max_error = 0.005);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_WORLD_GEOMETRY_H__
