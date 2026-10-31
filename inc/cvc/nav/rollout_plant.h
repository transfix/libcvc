/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::rollout_plant: the vehicle simulation rollout_kind::jolt trains on
// (docs/NAV_TRAINING.md, "Training on Jolt"). The simulation runs the training rollout's forward,
// and the gradient comes from a differentiable model of the same vehicle, dyn_4w's port-Hamiltonian
// four-wheel plant (train_config::dyn4), evaluated at the states the simulation reached:
//
//   x[t+1] = f(x[t], u[t]) + xi[t],   xi[t] what the simulation did beyond the model, held fixed
//
// so the forward is the simulation's to the bit and the backward is the model's Jacobian along it
// (a model gradient on a real trajectory). The model's books are calibrated to the simulation's
// energy ledger (dyn4_ledger.h), so its gradient moves energy as the simulation does.
//
//   cvc::physics::jolt_rollout   the Jolt Humvee (or any vehicle_desc), physics/jolt_rollout.h
//   dyn4_rollout_plant           the model itself (xi = 0): the trainer's plumbing, checked
//                                against finite differences exactly
//
// Everything is in the training scene's normalized frame: lengths in its units, time in s, mass in
// kg, so energy is kg x length^2 / s^2 (a joule times k^2, k the normalized units per metre).

#ifndef __CVC_NAV_ROLLOUT_PLANT_H__
#define __CVC_NAV_ROLLOUT_PLANT_H__

#include <cstdint>
#include <cvc/nav/detail/diff_rollout.h>
#include <cvc/nav/detail/dyn_4w.h>
#include <vector>

namespace cvc {
namespace nav {

struct training_scene;

// One step's energy from the simulation's ledger, split as the model's books are
// (diff::dyn4_ph_lines): what the drivetrain delivered at the wheels, net of its own losses and
// the brakes (port); the tires' slip; drag; rolling resistance (a soft medium's included); the
// dampers (the suspension's and the chassis body's own); the change in what is stored; what the
// simulation's own books left over (residual). And what the source paid (drawn, the trainer's
// energy), and whether the vehicle touched anything (contact: no model of the vehicle alone
// accounts for a collision, so the books leave that step out).
struct plant_energy_lines {
  double drawn = 0.0;
  double port = 0.0, tire = 0.0, drag = 0.0, rolling = 0.0, damper = 0.0;
  double stored = 0.0, residual = 0.0;
  bool contact = false;
};

// What the source pays for the model's port: price.drive per unit of the port's positive power
// (1 / the drivetrain's and the source's efficiency) and price.idle, a power, whenever it runs.
// One step's energy at the state s it ended at is
//   hdt (drive softplus_w(port(s)) + idle),
// softplus_w(P) = w log(1 + exp(P / w)) with w = 1e-3 dyn4_full_power, smooth at P = 0.
// calibrate_dyn4 fits it to the simulation's `drawn`.
struct dyn4_energy_price {
  float drive = 0.0f; // source energy per unit of port work
  float idle = 0.0f;  // power, in the frame's units
};

// The power at full drive and top speed, drive m a_max vmax + idle: the energy term's scale, as
// diff::power_full is the power model's.
inline float dyn4_full_power(const dyn4_energy_price &price, const diff::dyn4_params &p,
                             const diff::bike_veh &v) {
  return price.drive * p.m * v.a_max * v.vmax + price.idle;
}

// One step's energy at the state s it ended at (dyn4_energy_price), on flat ground of friction mu,
// and its gradient on the state (g[14], in dyn4_state's order; optional).
float dyn4_step_energy(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                       const dyn4_energy_price &price, const diff::dyn4_state<float> &s,
                       float *g = nullptr);

// A training batch's vehicles.
// An obstacle of one agent's own (rollout_batch::obstacles), in the frame: a disc of radius r
// centred on (x, y), or a box of half extents (hx, hy) there, axis-aligned.
struct rollout_obstacle {
  enum class kind : std::uint8_t { disc, box };
  kind type = kind::box;
  float x = 0.0f, y = 0.0f;
  float r = 0.0f;             // disc
  float hx = 0.0f, hy = 0.0f; // box
};

struct rollout_batch {
  // The scene the agents share (its obstacles, for a plant that puts them in its world), or null
  // for agents with only their own (`obstacles`).
  const training_scene *scene = nullptr;
  diff::bike_veh veh;         // the controller's vehicle; hdt is the step
  diff::dyn4_params model;    // the model the gradient is, whose actuator state() reports
  dyn4_energy_price price;    // the model's energy, for a plant that has no ledger of its own
  float mu = 0.9f;            // the ground's friction, as the model sees it
  int n = 0;                  // vehicles
  const float *o = nullptr;   // [n*2]: each one's rear axle
  const float *psi = nullptr; // [n]: and heading
  // [n], optional: each one's forward speed at the start, along its heading (null: at rest).
  const float *speed = nullptr;
  // [n], optional: each agent's own obstacles, as well as the scene's (a material batch's discs
  // and hard cells).
  const std::vector<rollout_obstacle> *obstacles = nullptr;
  // How far an agent may get from its start (the frame's length): room a plant that lays agents
  // out side by side leaves around each one's obstacles. 0: the scene's extent.
  float reach = 0.0f;
};

class rollout_plant {
public:
  virtual ~rollout_plant() = default;
  // A batch: b.n vehicles, agent i's rear axle on (o[2i], o[2i+1]) heading psi[i], at rest or at
  // b.speed[i].
  virtual void reset(const rollout_batch &b) = 0;
  virtual int size() const = 0;
  // Agent i now, as the model's state: the rear axle's pose; the body twist at the centre of
  // gravity; the body's heave, roll and pitch from rest and their rates; the steering angle the
  // vehicle has; and the drive the model's actuator holds (dyn4_plant's step 1 on the commands
  // since reset(): a simulation has no such state of its own).
  virtual diff::dyn4_state<float> state(int i) const = 0;
  // One step of hdt: every agent's command, as bike_control gives it (the bicycle angle, the
  // acceleration).
  virtual void step(const float *delta_cmd, const float *a_cmd) = 0;
  // Agent i's energy over the last step; all zero after reset().
  virtual plant_energy_lines lines(int i) const = 0;
  // The whole state, to replay from it bit for bit: restore() then the same commands give the
  // same states and lines. A plant restores only a state of its own batch.
  virtual std::vector<std::uint8_t> save() const = 0;
  virtual void restore(const std::vector<std::uint8_t> &state) = 0;
};

// The model as the plant: dyn4_plant on flat ground of the batch's mu, with the batch's model.
// Its lines are the model's own books over each step (dyn4_ph_lines, the trapezoid between the
// step's ends), its drawn energy dyn4_step_energy at the batch's price. The batch's obstacles are
// left out: the model has no walls. Training on it is training on the model, through
// rollout_kind::jolt's code.
class dyn4_rollout_plant final : public rollout_plant {
public:
  void reset(const rollout_batch &b) override;
  int size() const override { return static_cast<int>(_s.size()); }
  diff::dyn4_state<float> state(int i) const override { return _s.at(i); }
  void step(const float *delta_cmd, const float *a_cmd) override;
  plant_energy_lines lines(int i) const override { return _lines.at(i); }
  std::vector<std::uint8_t> save() const override;
  void restore(const std::vector<std::uint8_t> &state) override;

private:
  diff::bike_veh _v;
  diff::dyn4_params _p;
  dyn4_energy_price _price;
  float _mu = 0.9f;
  std::vector<diff::dyn4_state<float>> _s;
  std::vector<plant_energy_lines> _lines;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_ROLLOUT_PLANT_H__
