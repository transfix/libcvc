/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A boat's hull as a fluid sees it (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D2): a closed
// triangle list in the body's frame, separate from the collision shape, that a fluid field
// pushes on triangle by triangle in place of the body's sample cells.
//
//   cvc::physics::body_desc boat;
//   boat.shape = cvc::physics::box{{2.0, 0.8, 0.4}};          // what it collides with
//   boat.hull = cvc::physics::box_hull({2.0, 0.8, 0.4});      // or the hull's own mesh
//   w.add_body(boat);
//
// Each step, for each fluid field, each triangle is split at the waterline (the water's top
// over its vertices: the free surface's height, or the field box's top) and its submerged part
// feels, against the water's velocity at it (u = water - hull):
//
//   hydrostatic pressure  rho g depth on the submerged part, along the inward normal, at its
//                         centre of pressure: the hull floats, trims and rights itself
//   viscous resistance    1/2 rho C_f(Re) S |u_t| u_t, u_t the tangential part of u, with the
//                         ITTC-57 line C_f = 0.075 / (log10 Re - 2)^2, Re = |u| length / nu
//                         (Re at least 1e5, where the line is 0.0083)
//   pressure drag         1/2 rho C_d S |u_n| u_n n on a face the water meets (u_n < 0), C_d
//                         the field's drag_coefficient; suction_coefficient in place of C_d on
//                         a face it leaves (u_n > 0)
//   slamming              on a face entering the water whose wetted area S grew since the last
//                         step: the added mass of the water it sets moving,
//                         slam_coefficient rho l dS/dt, times u_n n, l the cube root of the
//                         hull's volume
//   damping               the field's damping c times the immersed volume, c V_sub as the cells
//                         give, on the vertical part of u only: the energy a floating hull
//                         radiates in waves as it heaves, rolls and pitches. It is shared among
//                         the downward faces by the column of water over each, depth S |n_z|,
//                         so it does not grow as a submerged hull sinks deeper.
//
// The velocity terms are integrated implicitly in their own rate, as the cells' drag is (the
// horizontal ones in the rate without the damping, so the damping does not slow a hull's
// surge). The torque is also scaled in its rotational rate, over a low estimate of the body's
// least moment of inertia: half a solid box's over the hull's bounding box, or over the
// collision shape's where that is lower. So a light hull in a dense fluid does not overshoot
// the water's velocity, given a convex-hull collision shape that roughly fills its bounding box.
// Media and flows still push on the body's cells. The depth is measured along world z to the
// local water top, and the hull is assumed to lie within the field's box in x and y (a vertex
// outside it is dry) and above its bottom. add_body keeps its own copy of the mesh, so changing
// the caller's mesh afterwards does not change the body.
//
// A hull is part of the world's state: its digest and each triangle's last wetted area are in
// save() and state_hash(), and restore() refuses a world whose hulls differ.
//
// Standard library only, like the rest of cvc/physics.

#ifndef __CVC_PHYSICS_HULL_H__
#define __CVC_PHYSICS_HULL_H__

#include <array>
#include <cstdint>
#include <cvc/physics/types.h>
#include <memory>
#include <vector>

namespace cvc {
namespace physics {

struct hull_mesh {
  std::vector<vec3> vertices; // body-local metres, about the body's origin
  // Counter-clockwise seen from outside (outward normals), enclosing a positive volume.
  std::vector<std::array<std::uint32_t, 3>> triangles;
  double length = 0.0;                  // m, for the Reynolds number; 0: the extent along x
  double kinematic_viscosity = 1.19e-6; // m^2/s: sea water at 15 C (fresh, 20 C: 1.0e-6)
  double suction_coefficient = 0.0;     // pressure drag on the faces the water leaves
  double slam_coefficient = 0.5;        // 0.5 gives about a flat plate's added mass
};

// A box's hull, centred on the origin: 8 vertices and 12 triangles.
std::shared_ptr<const hull_mesh> box_hull(const vec3 &half_extents);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_HULL_H__
