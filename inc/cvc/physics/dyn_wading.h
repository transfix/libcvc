/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// How the dynamic plants (dyn_bike_plant, dyn_4w_plant) wade: vehicle_desc::wading (vehicle.h's
// Wading) on a model that drives on its own ground and never floats, from the water their world
// holds (world::water_at) at each tire at a substep's start. With a world and wading on, the water
//   drags   the vehicle along its travel, 1/2 rho C_d A v |v|: each tire's width times its
//           immersed height (wheel_drag) and the underbody's (underbody_drag), as the Jolt
//           vehicle's, in the model's own drag term (still water: the models know no current);
//   lifts   load off its tires, their displaced share (wheel_displacement) and the chassis box's
//           (displacement: it floods at once; flood_area is the Jolt vehicle's alone), so each
//           tire's grip falls with the load it keeps, to 0.05 (an aquaplaning tire's) at most;
//   soaks   each tire to the wet friction of the material under it (a lake bed's, under a water
//           class: cvc::world's surface table), from the model's dry mu, by 3 mm of standing
//           water, and aquaplanes it (tire_pressure), as the Jolt vehicle's;
//   floods  the engine for good past fording_depth at the front axle: no drive, the brakes still.
// The plants' chassis boxes in the world take the fluids' sealed-box push like any box, but each
// substep puts them back at the model's pose, so that push never reaches the model (a collision
// takes back only the plane's pose and twist). Without wading the models drive through water as
// on dry ground. A vehicle in water is not `exact` for a twin that replays the plant's model
// (ariadne's entity_plant): the water changes its drag and grip each substep.

#ifndef __CVC_PHYSICS_DYN_WADING_H__
#define __CVC_PHYSICS_DYN_WADING_H__

namespace cvc {
namespace physics {

// The water at one vehicle as of its last substep.
struct dyn_wading {
  bool wet = false;            // a tire or the underbody stood in water
  bool engine_flooded = false; // for good (state: the plants' saves keep it)
  double drag = 0.0;           // kg/m: the water's drag on the forward speed v is drag v |v|
  double lift = 0.0;           // N: the load the water lifted off the tires
  int n = 0;                   // tires (the bicycle's: its two axles)
  double x[4] = {0.0, 0.0, 0.0, 0.0}, y[4] = {0.0, 0.0, 0.0, 0.0}; // each one's contact (world m)
  double grip[4] = {1.0, 1.0, 1.0, 1.0};                           // its factor on the model's mu
  double depth[4] = {0.0, 0.0, 0.0, 0.0};                          // m of water over it
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_WADING_H__
