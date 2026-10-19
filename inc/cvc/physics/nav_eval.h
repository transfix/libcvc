/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::reach_on_plant: how a trained nav policy does on the physics plant, the score
// training never sees (doc 31: policies train on differentiable models, and Jolt rollouts
// evaluate them and select checkpoints).
//
//   cvc::physics::plant_reach_options o;           // 48 agents, 400 ticks, seeds 100..119
//   double r = cvc::physics::reach_on_plant(policy, cvc::nav::city_scene(96),
//                                           cvc::physics::vehicle_desc::roboracer(), 0.035, o);
//
// The scene (a coef_train training_scene, whose world has no metres) is read at the vehicle's
// scale: `wheelbase` normalized units are its wheelbase, so S = wheelbase / d.wheelbase() per
// metre. A physics world gets the scene's ground and its occupied cells as static boxes, one per
// row run; a sim_world of the scene's numbers (rr, d_hat, dt, vmax) drives `agents` vehicles
// through a nav_plant from scattered starts to goals for `ticks`, once per seed. The result is
// the mean fraction that reached their goals; `per_seed` gets each seed's, in `seeds` order.
//
// It plans with the scene's numbers by default (plan_with_vehicle false), as both the policy
// and its training did. Needs a physics backend with vehicles (Jolt); throws otherwise.

#ifndef __CVC_PHYSICS_NAV_EVAL_H__
#define __CVC_PHYSICS_NAV_EVAL_H__

#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/physics/vehicle.h>
#include <vector>

namespace cvc {
namespace physics {

struct plant_reach_options {
  int agents = 48;
  int ticks = 400;
  std::vector<unsigned> seeds =
      std::vector<unsigned>{100u, 101u, 102u, 103u, 104u, 105u, 106u, 107u, 108u, 109u,
                            110u, 111u, 112u, 113u, 114u, 115u, 116u, 117u, 118u, 119u};
  double mu = 0.9;              // the ground's and the buildings' friction
  double building_height = 1.5; // m
  bool plan_with_vehicle = false;
  float reach_tol = 0.15f; // normalized
  int nsub = 6;            // the kinematic reference's substeps a tick (D14)
  // Seeds run in parallel, each in its own world and sim_world: 0 = as many at once as the
  // hardware has threads, 1 = one at a time. A seed's result is the same either way.
  int threads = 0;
};

double reach_on_plant(const cvc::nav::coef_mlp &policy, const cvc::nav::training_scene &scene,
                      const vehicle_desc &d, double wheelbase,
                      const plant_reach_options &o = plant_reach_options(),
                      std::vector<double> *per_seed = nullptr);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_NAV_EVAL_H__
