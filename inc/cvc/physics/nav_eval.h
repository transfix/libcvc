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
// through a nav_plant from scattered starts to goals for `ticks`, once per seed (or to goals near
// their starts: plant_reach_options::goal_max). The result is the mean fraction that reached their
// goals; `per_seed` gets each seed's, in `seeds` order.
//
// It plans with the scene's numbers by default (plan_with_vehicle false), as both the policy
// and its training did. Needs a physics backend with vehicles (Jolt); throws otherwise.
//
// score_on_plant runs the same drives and also returns what they cost (the P8c gate): the energy
// every agent drew from its source over the run, as the P3 ledger counts it
// (nav_plant::energy_step summed), and the distance their chassis drove. With
// plant_reach_options::predict_energy it also logs each run (plant_log) and gives what the power
// model `power` says the same motion drew (evaluate_power), so a model can be judged on the
// runs a trained policy makes, not only on those it was fitted to.

#ifndef __CVC_PHYSICS_NAV_EVAL_H__
#define __CVC_PHYSICS_NAV_EVAL_H__

#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_power.h>
#include <cvc/physics/plant_log.h>
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
  // Local goals (normalized units): goal_max > 0 draws the starts from the scene's largest free
  // component and each goal goal_min..goal_max from its start (training_scene::
  // sample_starts_goals), so reach is scored at the goal distances a policy is deployed at.
  // 0 = scattered starts and goals anywhere free, the default.
  float goal_min = 0.0f, goal_max = 0.0f;
  // score_on_plant: predict each run's energy with `power` too (plant_drive_score::model_J).
  bool predict_energy = false;
  cvc::nav::dyn_power_si power{};
};

// One run's score, or a mean of runs'.
struct plant_drive_score {
  double reach = 0.0;      // the fraction of agents that reached their goals
  double energy_J = 0.0;   // drawn by all its agents, moving or not
  double distance_m = 0.0; // their chassis' paths, summed
  double model_J = 0.0;    // the power model's energy over the same ticks (predict_energy)
  // Where it went: the agents' P3 ledgers at the run's end, every field summed (a mean's, the
  // seeds' mean). charge and fuel_left are sums too, so read them per agent.
  vehicle_energy ledger{};
  // The fleet's draw per kilometre driven, Wh/km; 0 if nothing moved.
  double wh_per_km() const {
    return distance_m > 0.0 ? energy_J / 3600.0 / (distance_m / 1000.0) : 0.0;
  }
  double model_wh_per_km() const {
    return distance_m > 0.0 ? model_J / 3600.0 / (distance_m / 1000.0) : 0.0;
  }
};

double reach_on_plant(const cvc::nav::coef_mlp &policy, const cvc::nav::training_scene &scene,
                      const vehicle_desc &d, double wheelbase,
                      const plant_reach_options &o = plant_reach_options(),
                      std::vector<double> *per_seed = nullptr);

// Several policies on the same seeds: each policy's mean reach, with `per_seed` (if given) each
// one's per-seed reach. Every (policy, seed) run shares the worker threads (`o.threads`), so a
// population scores as fast as one policy's seeds would on as many threads.
std::vector<double> reach_on_plant(const std::vector<cvc::nav::coef_mlp> &policies,
                                   const cvc::nav::training_scene &scene, const vehicle_desc &d,
                                   double wheelbase,
                                   const plant_reach_options &o = plant_reach_options(),
                                   std::vector<std::vector<double>> *per_seed = nullptr);

// reach_on_plant's runs with their energy and distance: each policy's mean over the seeds (each
// field the mean of the seeds'), with `per_seed` (if given) each one's per-seed scores. Reach is
// what reach_on_plant gives, bit for bit. `logs` (if given) gets every run's plant_log, its agents'
// runs one after another in (policy, seed) order: data to fit a plant model to (fit_power) on the
// scenes a policy is scored on.
std::vector<plant_drive_score>
score_on_plant(const std::vector<cvc::nav::coef_mlp> &policies,
               const cvc::nav::training_scene &scene, const vehicle_desc &d, double wheelbase,
               const plant_reach_options &o = plant_reach_options(),
               std::vector<std::vector<plant_drive_score>> *per_seed = nullptr,
               std::vector<std::vector<planar_sample>> *logs = nullptr);

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_NAV_EVAL_H__
