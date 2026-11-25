/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::scrub_plant: the checkpoint scrubber's plant hook (cvc/nav/nav_scrub.h's
// scrub_options::plant). Each deployed world gets a plant_host of its own, on the scenario's plant
// (any name plant_host::use accepts: "humvee", "roboracer", "dynamic:humvee", ...), and its
// physics world gets the world's blocked cells (sim_world::truth) as static boxes on the flat
// ground, `wall_height` m tall: each row's runs of cells, merged down the rows into rectangles. So
// a vehicle that drives into a building stops against it, where nav's kinematic bicycle, which has
// no collision response, drives through.
//
//   cvc::nav::scrub_scenario s = cvc::nav::default_scenario(4, 0, 600);
//   s.plant = cvc::physics::scrub_default_plant();   // "humvee" with Jolt, else "kinematic"
//   cvc::nav::scrub_options o;
//   o.plant = cvc::physics::scrub_plant();
//   std::vector<cvc::nav::scrub_entry> runs = cvc::nav::scrub(history, s, o);
//
// Without Jolt in the build a vehicle falls back to the kinematic bicycle (plant_host's rule), and
// the hook says so: the run log's plant is "kinematic".
//
// On a real world (cvc/nav/nav_scrub.h's grid_scenario, from a data bundle's city or a generated
// world), give the hook the world's geometry instead: each deployed world's plant collides with
// its collide items (its terrain and buildings, plant_host::set_geometry), in metres, so a Humvee
// drives the city's ground and stops against its buildings, not against boxes from the grid.
//
//   std::shared_ptr<const cvc::world::world_geometry> city = scene_world.world_geometry(node);
//   o.plant = cvc::physics::scrub_plant(city);

#ifndef __CVC_PHYSICS_SCRUB_PLANT_H__
#define __CVC_PHYSICS_SCRUB_PLANT_H__

#include <cvc/nav/nav_scrub.h>
#include <cvc/physics/plant_host.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace physics {

// The hook. `o` is each deployed world's plant_host's (its metres_per_world_unit is the scenario's
// frame: 1 for the built-in blocks, whose world is in metres).
cvc::nav::scrub_plant_fn scrub_plant(plant_host_options o = {}, double wall_height = 3.0);
// The hook on `world`'s geometry (not null): each plant_host's static colliders are its collide
// items, `nav_materials` its nav material ids per surface id (add_world_geometry's), and no wall
// boxes; the vehicles start on its terrain (`o`'s plant ground_height, unless it has one). The
// hook and each deployed run keep the geometry alive. The scenario's frame must be `world`'s, in
// metres (grid_scenario's), or `o`'s metres_per_world_unit its scale to them.
cvc::nav::scrub_plant_fn scrub_plant(std::shared_ptr<const cvc::world::world_geometry> world,
                                     plant_host_options o = {},
                                     std::vector<int> nav_materials = {});
// "humvee" when this build has Jolt, else "kinematic": the scrubber's default plant.
std::string scrub_default_plant();
// How many wall boxes scrub_plant gives a world with `truth` (rows x cols, nonzero blocked).
int scrub_wall_boxes(const std::uint8_t *truth, int rows, int cols);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_SCRUB_PLANT_H__
