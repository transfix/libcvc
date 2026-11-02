/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::jolt_rollout: the Jolt vehicle as cvc::nav's training rollout (rollout_kind::jolt;
// docs/NAV_TRAINING.md, "Training on Jolt"). Jolt moves the vehicles, and the trainer
// differentiates the four-wheel port-Hamiltonian model along the states Jolt reached, its books
// calibrated to Jolt's own energy ledger (cvc::nav::calibrate_dyn4).
//
//   const vehicle_desc d = vehicle_desc::humvee();
//   cvc::nav::train_config cfg;
//   cfg.veh_L = 0.35f;  // the Humvee's 3.3 m at about 0.106 normalized units per metre
//   auto jolt = std::make_shared<cvc::physics::jolt_rollout>(d, cfg.veh_L);
//   cfg.rollout = cvc::nav::rollout_kind::jolt;
//   cfg.plant = jolt;
//   cfg.dyn4 = cvc::physics::nav_plant::dyn4_params_at(d, cfg.veh_L);
//   const auto ticks = cvc::nav::record_plant_ticks(scene, cfg, *jolt, 32, 200, 1);
//   const auto cal = cvc::nav::calibrate_dyn4(cfg.dyn4, veh, cfg.ground_mu, ticks);
//   cfg.dyn4 = cvc::nav::calibrated(cfg.dyn4, cal);
//   cfg.dyn4_price = cal.price;
//   cvc::nav::coef_trainer(cfg).train(scene);
//
// Each reset() builds a Jolt world: flat ground of the batch's friction and, with
// jolt_rollout_options::walls, the scene's obstacles as static boxes (its occupancy's runs of
// cells merged into rectangles) and each agent's own (rollout_batch::obstacles: a disc as a
// 16-sided prism, a box as a box); and a nav_plant driving one vehicle per agent through the
// driver model (nav_plant::command). The agents never meet: each drives its own copy of its
// world, laid out on a grid around its extent (the scene's or its start's, its own obstacles,
// rollout_batch::reach beyond them), so a batch is n independent runs in one world, which Jolt
// steps as separate islands (on worker threads with jolt_rollout_options::threads). The vehicles
// start at the height they rest at and settle for settle_ticks before the batch's first step;
// one with a start speed (rollout_batch::speed) is then launched at it, its chassis set moving
// along its heading (the wheels spin up in the first step, which costs a little speed). A start
// whose body (the chassis box and the wheels, as nav_plant::limits has it) would stand on a wall
// at its heading is turned, 15 degrees at a time either way, to the nearest heading at which it
// clears every wall by clearance_m (start_turn); with none, it keeps its heading.
//
// The frame: k = wheelbase / desc.wheelbase() normalized units per metre (the trainer's veh_L
// for its vehicle), metres_per_world_unit 1, at the scene's normalized origin. state() reads the
// vehicle back as cvc::nav::diff::dyn4_state: the rear axle's position and the heading (unwrapped
// along the run), the centre of mass's velocity in the heading's frame and the yaw rate, its
// height above where it settled and its vertical speed, roll (asin of the left axis's z) and
// pitch (asin of minus the forward axis's z, + nose down) and their rates (the angular velocity
// about the forward and left axes), the steering angle the actuator has, and the drive the
// model's actuator holds on the commands so far (dyn4_drive_actuator, the batch's model).
// lines() maps the ledger's step (nav_plant::ledger_step) onto the model's books, in the frame's
// energy (J x k^2): port = supplied + idle + drivetrain_split - drivetrain - brake, tire, drag =
// aero, rolling = rolling + medium, damper = suspension + chassis_damping, stored, residual;
// drawn = nav_plant::energy_step; contact when the chassis touched a wall or another vehicle.

#ifndef __CVC_PHYSICS_JOLT_ROLLOUT_H__
#define __CVC_PHYSICS_JOLT_ROLLOUT_H__

#include <cstdint>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn4_ledger.h>
#include <cvc/nav/rollout_plant.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <memory>
#include <vector>

namespace cvc {
namespace physics {

struct jolt_rollout_options {
  bool walls = true;         // the scene's obstacles in the world
  double wall_height = 2.0;  // m
  int threads = 0;           // world_config::threads
  int settle_ticks = 5;      // ticks at rest before a batch's first step
  double clearance_m = 0.25; // a start's body from the walls
  // Between two copies of the scene, as a share of its extent: room for a vehicle that leaves
  // it.
  double margin = 0.5;
  nav_plant_options plant; // the driver model's (its metres_per_world_unit and spawn_height are
                           // set by the rollout)
};

class jolt_rollout final : public cvc::nav::rollout_plant {
public:
  // `wheelbase`: the vehicle's in the training scene's normalized frame (the trainer's veh_L).
  // Throws cvc::physics_error for a vehicle without a wheelbase or a wheelbase <= 0, and
  // cvc::unsupported_exception in a build without Jolt.
  jolt_rollout(const vehicle_desc &desc, double wheelbase, jolt_rollout_options options = {});
  ~jolt_rollout() override;

  // ── cvc::nav::rollout_plant ──
  void reset(const cvc::nav::rollout_batch &b) override;
  int size() const override { return static_cast<int>(_agents.size()); }
  cvc::nav::diff::dyn4_state<float> state(int i) const override;
  void step(const float *delta_cmd, const float *a_cmd) override;
  cvc::nav::plant_energy_lines lines(int i) const override;
  std::vector<std::uint8_t> save() const override;
  void restore(const std::vector<std::uint8_t> &state) override;

  // ── the simulation ──
  double units_per_metre() const { return _k; }
  const vehicle_desc &desc() const { return _desc; }
  // The world and plant of the current batch (after reset()).
  world &physics_world();
  nav_plant &plant();
  // Static bodies the walls added for each copy of the scene.
  int wall_boxes() const { return _wall_boxes; }
  // How far agent i's start was turned from the batch's heading for its body to fit (rad).
  double start_turn(int i) const { return _agents.at(static_cast<std::size_t>(i)).turn; }
  // step()'s wall time per vehicle per step, averaged since reset() (microseconds).
  double step_us_per_vehicle() const;

private:
  struct agent {
    double ox = 0.0, oy = 0.0; // the copy's offset, normalized
    double z_rest = 0.0;       // the centre of mass's height at rest, m
    double psi = 0.0;          // the heading, unwrapped
    float a = 0.0f;            // the model's drive actuator
    double turn = 0.0;         // the start's heading less the batch's
  };
  cvc::nav::diff::dyn4_state<float> read(int i, double psi) const;

  vehicle_desc _desc;
  double _k = 1.0; // normalized units per metre
  jolt_rollout_options _o;
  double _spawn_height = 0.0;
  cvc::nav::diff::bike_veh _veh;
  cvc::nav::diff::dyn4_params _model;
  std::unique_ptr<world> _w;
  std::unique_ptr<nav_plant> _plant;
  std::vector<agent> _agents;
  int _wall_boxes = 0;
  double _step_us = 0.0;
  std::uint64_t _steps = 0;
};

// Sets cfg up to train on `desc` in Jolt: cfg.rollout = jolt, cfg.plant a new jolt_rollout at
// cfg.veh_L (with `options`), cfg.dyn4 nav_plant::dyn4_params_at(desc, cfg.veh_L) calibrated to it
// and cfg.dyn4_price its price. The calibration drives `agents` vehicles for `steps` steps on
// `scene` under the untrained policy (cvc::nav::record_plant_ticks, seed `seed`) and fits the
// model to them (cvc::nav::calibrate_dyn4), which it returns. cfg's veh_* are the controller's
// limits throughout: set them first.
cvc::nav::dyn4_calibration use_jolt_rollout(cvc::nav::train_config &cfg,
                                            const cvc::nav::training_scene &scene,
                                            const vehicle_desc &desc, int agents = 16,
                                            int steps = 200, unsigned seed = 1,
                                            jolt_rollout_options options = {});

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_JOLT_ROLLOUT_H__
