/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics plant-log scenes: nav driving physics vehicles (nav_plant on Jolt) through the
// scenes `cvc plant-log` logs, and the directory it writes from them (cvc/physics/plant_log.h).
//
//   plant_log_options o;
//   o.vehicle = "roboracer";
//   o.scene = "city";
//   o.resolve();                                        // checks it, fills the scene's defaults
//   const plant_log_drive d = drive_plant_log_scene(o, nav::coef_mlp::default_biased());
//   o.out = "runs/roboracer-city";
//   write_plant_log(o, d);                              // flights.npy, meta.json, residual.json
//
// Scenes:
//   swerve  open flat ground; each agent slaloms, its goal jumping from side to side, at a
//           seed-varied speed with a corner cap of lateral_g (the swerves past 0.3 g).
//   city    the coef trainer's city (city_scene(96)) scaled to the vehicle, as the plant reach
//           gate drives it.
//   bundle  a cvc::world city bundle (load_city_bundle): its buildings and terrain in the
//           physics world, its rasterized occupancy for nav (50 m in from the terrain's edge),
//           vehicles spawned on the terrain.
//   In city each agent gets a new random goal when it arrives (the trained drive alone does well
//   on so small a map). In bundle each drives vehicle routes (plan_vehicle_route with the plant's
//   footprint and turning circle, route_follower) to random goals 5-15% of the map away, a new
//   one at each arrival: the reactive drive alone stalls on long blocks.
// Logging starts after 0.5 s, once the vehicles have settled on their springs. Each seed is its
// own sim and physics world; the runs are deterministic given the options and the policy.
//
// The directory (write_plant_log):
//   flights.npy   float64 (F, T, 8): equal-length flights cut from the runs, rows
//                 [x, y, psi, vx, vy, r, drive, steer] (kPlanarWidth): the centre of mass on
//                 SE(2), its body twist, nav's acceleration command and the steering (steer
//                 actual or command). Row k's inputs acted from t_k-1 to t_k.
//   meta.json     the rate, the layout, the run's settings and each flight's seed and agent.
//   residual.json the one-tick residuals of diff::dyn_plant and the kinematic bicycle against
//                 the plant, binned by lateral g (also reported).
//   samples.csv   with samples: every tick of every run.
// fit_dyn fits dyn_bike's lags, tire stiffness and yaw inertia to the runs of every seed but the
// last (fit_dyn_bike) and reports the last seed's residual with the spec-sheet and the fitted
// vehicle (residual.json's "fit"). fit_power fits the power model (fit_power, P8c) to the ledger
// of the same runs and reports the last seed's energy with it (residual.json's "power").
// tools/liesphgp_pack.py turns one or more of these directories
// into a LieSPHGP dataset pickle.

#ifndef __CVC_PHYSICS_PLANT_LOG_SCENE_H__
#define __CVC_PHYSICS_PLANT_LOG_SCENE_H__

#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/physics/plant_log.h>
#include <string>
#include <vector>

namespace cvc {
namespace physics {

struct plant_log_options {
  // The drive.
  std::string vehicle = "humvee"; // a plant_host preset
  std::string scene = "swerve";   // swerve, city or bundle
  std::string bundle;             // the city bundle's directory (scene bundle)
  std::vector<unsigned> seeds = {100, 101, 102, 103};
  int agents = -1;         // < 0: 4 in swerve, 8 otherwise
  double seconds = 60.0;   // logged per seed
  double dt = 0.02;        // nav's tick, a whole number of 0.01 s physics steps
  double mu = 0.9;         // the ground's friction
  double lateral_g = -1.0; // nav's corner cap; < 0: 0.9 in swerve, 0.5 otherwise
  int grid = 600;          // bundle: the occupancy grid's cells a side
  int inflate = 0;         // bundle: obstacle inflation in cells
  double accel_margin = -1.0, brake_margin = -1.0; // the plant's; <= 0 keeps its defaults
  // The directory.
  std::string out;
  double flight_seconds = 5.0, stride_seconds = 5.0;
  steer_input steer = steer_input::actual;
  int nsub = 0; // residual substeps; 0: the physics steps a tick
  bool samples = false;
  bool fit_dyn = false;
  bool fit_power = false;
  // Progress, per-seed summaries and the residual tables; nullptr: none.
  std::FILE *report = nullptr;
  bool verbose = false; // also every agent's state every 5 s

  // Checks the options and fills the scene's defaults (agents, lateral_g, nsub); throws
  // std::invalid_argument.
  void resolve();
};

// What drive_plant_log_scene logged: one run per agent per seed, in seed order.
struct plant_log_drive {
  std::vector<std::vector<planar_sample>> runs;
  std::vector<unsigned> run_seed;
  std::vector<int> run_agent;
  double accel_margin = 0.0, brake_margin = 0.0; // what the plant planned with
};

// Drives every seed of the scene with `policy` (resolved options). Throws std::runtime_error when
// this libcvc has no Jolt backend.
plant_log_drive drive_plant_log_scene(const plant_log_options &o, const nav::coef_mlp &policy);

// Writes the directory o.out from a drive with the same options.
void write_plant_log(const plant_log_options &o, const plant_log_drive &d);

} // namespace physics
} // namespace cvc

#endif
