/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A sim_world's terrain (P5, cvc/nav/terrain.h) in the state tree, beside nav_plant's keys. nav
// does not touch the state tree itself, so its terrain binds here, as the plant does.
//
//   cvc::physics::pull_nav_terrain(app, "scene", sim);    // each tick, before sim.step()
//   cvc::physics::publish_nav_terrain(app, "scene", sim); // after it
//
// The live settings, read by pull_nav_terrain() and seeded with the world's values when unset
// (an invalid value is ignored; a change rebuilds the terrain's planes through
// sim_world::set_terrain_config):
//   <prefix>.nav_terrain.{max_grade, tip_ratio, tip_margin, sigma}   the vehicle's limits
//   <prefix>.nav_terrain.{lam_grade, lam_grade_hard, k_sharp, d_hat_m}   the grade forces
//   <prefix>.nav_terrain.{block_infeasible, governors}   0 or 1
// and what the terrain is, written by publish_nav_terrain():
//   <prefix>.nav_terrain.{active, gravity, limit, infeasible_cells}
// Without a terrain pull does nothing and publish writes only active = 0. Each agent's entries
// into infeasible slope are nav_stats' (infeasible_slope_steps). Every key says what it is, as
// world_binding's do: a comment and the properties access, unit and (on a knob) a recommended
// min and max.

#ifndef __CVC_PHYSICS_NAV_TERRAIN_BINDING_H__
#define __CVC_PHYSICS_NAV_TERRAIN_BINDING_H__

#include <string>

namespace cvc {

class app;

namespace nav {
class sim_world;
}

namespace physics {

void pull_nav_terrain(cvc::app &ctx, const std::string &prefix, cvc::nav::sim_world &w);
void publish_nav_terrain(cvc::app &ctx, const std::string &prefix, const cvc::nav::sim_world &w);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_NAV_TERRAIN_BINDING_H__
