/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics bodies: what a body is created from, and the state read back.

#ifndef __CVC_PHYSICS_BODY_H__
#define __CVC_PHYSICS_BODY_H__

#include <cstdint>
#include <cvc/physics/shape.h>
#include <cvc/physics/types.h>

namespace cvc {
namespace physics {

struct body_desc {
  physics::shape shape;
  motion_type motion = motion_type::dynamic;
  vec3 position; // world metres (ignored for terrain, which is placed by its own grid)
  quat rotation;
  vec3 linear_velocity;  // m/s
  vec3 angular_velocity; // rad/s
  material_id material = 0;
  double mass = 0.0;       // kg; 0 = density times the shape's volume
  double density = 1000.0; // kg/m^3, used when mass is 0
  double linear_damping = 0.05;
  double angular_damping = 0.05;
  bool allow_sleeping = true;
  std::uint64_t user_data = 0;
};

struct body_state {
  vec3 position; // the body's origin (the shape's origin), world metres
  quat rotation;
  vec3 linear_velocity;
  vec3 angular_velocity;
  bool active = false; // false while asleep, and always for static bodies
};

// What may change on a running body (world::set_body); world::get_body reads it back.
struct body_settings {
  material_id material = 0; // a sphere, box or hull's; a mesh or terrain's are its triangles'
                            // or cells' (world::set_material changes those)
  double mass = 0.0;        // kg, a dynamic body's (as the engine computed it when the
                            // description gave 0); 0 for static and kinematic bodies
  double linear_damping = 0.05;
  double angular_damping = 0.05;
  bool allow_sleeping = true;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BODY_H__
