/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_4w_plant: nav's agents on the dynamic four-wheel vehicle with suspension
// (cvc/nav/detail/dyn_4w.h, P8d) as their live plant, on the scene's terrain, with its
// port-Hamiltonian energy books.
//
//   dyn_4w_plant_options o;
//   o.plant.ground_height = terrain;   // the ground under the wheels, world metres
//   o.physics = &w;                    // the world the chassis collide in (optional)
//   cvc::physics::dyn_4w_plant plant({cvc::physics::vehicle_desc::humvee()}, o);
//   sim.set_plant(&plant);
//   for (;;) sim.step();
//   plant.books(0);                    // where agent 0's energy went
//
// The vehicle is dyn4_plant's: four contact patches, each corner's spring and damper on the
// ground under its wheel, per-wheel grip and load, Ackermann steering, and with grade_force (on
// here) gravity along the slope. Its parameters come from the same vehicle_desc as the Jolt plant
// (nav_plant::dyn4_si_of), as dyn_bike_plant's do. Agent i drives descs[i % size]; attach() puts
// its rear axle on the agent's start, at rest on the ground, its heave, roll and pitch fitted to
// the ground under its wheels.
//
// The ground is options.plant.ground_height (else the flat ground_z). The world, when given, holds
// the chassis as dyn_bike_plant's does (dyn_world.h): the vehicles collide with the buildings,
// with each other and with anything else in it, a collision taking the world's pose and twist back
// into the model (heave, roll and pitch stay the model's) and booking the energy that changed.
//
// Nav's acceleration goes through the Jolt plant's driver (dyn_driver.h; options.driver), as
// dyn_bike_plant's does. Each tick runs dyn4_plant's substeps under the held command and books
// every step (dyn4_plant_books):
//   stored now = stored at attach + drive - brake - dissipated + lever + grade + ground + limiter
//                + contact + residual
// stored is H: the kinetic energy, the springs' and gravity's on the heave. dissipated is the
// tires' slip, drag, rolling resistance and the dampers; lever the tires' lateral forces' one-way
// moments on roll and pitch; grade gravity along the slope; ground the springs' energy the new
// ground under the wheels added (with grade, second order per step); limiter the speed clamp's
// and the low-speed blend's; contact the collisions'; residual the integrator's error.

#ifndef __CVC_PHYSICS_DYN_4W_PLANT_H__
#define __CVC_PHYSICS_DYN_4W_PLANT_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/detail/dyn_4w.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/dyn_driver.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <memory>
#include <vector>

namespace cvc {
namespace physics {

namespace dyn {
class dyn_chassis;
}

struct dyn_4w_plant_options {
  // The plan, latency, margins, nav's frame in metres and the ground (ground_height, else
  // ground_z), as nav_plant reads them (its spawn, stepping and drift settings do not apply).
  nav_plant_options plant;
  double mu = 0.9;          // the ground's grip
  double substep = 0.01;    // s: the longest step without a world (with one: its fixed_dt)
  double drive_lag = -1.0;  // s: the drive force's lag; < 0 the vehicle's own
  double gravity = 9.80665; // m/s^2
  // Nav's acceleration as the Jolt plant's driver takes it (dyn_driver.h), so nav drives this plant
  // as it drives that one; false: as the model's drive force per unit mass, raw.
  bool driver = true;
  bool grade_force = true; // gravity along the slope acts on the body (dyn4_params::grade_force)
  // The world the chassis collide in (borrowed; null: no collisions), and its bodies the wheels
  // drive on (touching them is not a collision). attach() sets its contact reporting to all.
  world *physics = nullptr;
  std::vector<body_id> ground_bodies;
  // The chassis box's friction against what it hits (combined by the world's rule: the product,
  // by default, so 0.21 against a building's 0.7): a body panel scraping masonry or another body,
  // not a tire's grip. At 1, the default material's, a vehicle pressed against a wall by its own
  // tires grips it like a tire and crawls along it.
  double chassis_friction = 0.3;
};

// One agent's energy, in J: over the last tick, or since attach().
struct dyn_4w_books {
  double stored = 0.0;     // H at the end
  double drive = 0.0;      // the wheels' forces where they push (>= 0)
  double brake = 0.0;      // where they hold back (>= 0)
  double dissipated = 0.0; // the tires' slip, drag, rolling resistance, the dampers (>= 0)
  double lever = 0.0;      // the tires' lateral forces on roll and pitch (one-way)
  double grade = 0.0;      // gravity along the slope
  double ground = 0.0;     // the springs' energy the new ground under the wheels added
  double limiter = 0.0;    // the speed clamp's and the low-speed blend's
  double contact = 0.0;    // what the collisions changed (0 without a world)
  double residual = 0.0;   // the integrator's
  // Everything that moved, for the residual's scale.
  double flow() const;
};

class dyn_4w_plant final : public cvc::nav::vehicle_plant {
public:
  // `descs` must not be empty.
  explicit dyn_4w_plant(std::vector<vehicle_desc> descs, dyn_4w_plant_options options = {});
  ~dyn_4w_plant() override;
  dyn_4w_plant(const dyn_4w_plant &) = delete;
  dyn_4w_plant &operator=(const dyn_4w_plant &) = delete;

  // ── cvc::nav::vehicle_plant ──
  cvc::nav::plant_kind kind() const override { return cvc::nav::plant_kind::dynamic; }
  void attach(int n, const cvc::nav::plant_state *start,
              const cvc::world_units::normalized_frame &frame, double dt) override;
  // The rear axle, the heading, the forward speed, the steering's angle.
  cvc::nav::plant_state state(int i) const override;
  // As dyn_bike_plant's.
  cvc::nav::plant_limits limits(int i) const override;
  void command(int i, float steer, float accel) override;
  void advance() override;
  int reference_substeps() const override { return _nsub; }
  double energy_step(int i) const override;
  // As dyn_bike_plant's.
  int static_contacts_step(int i) const override;
  bool tracks_energy() const override { return true; }
  // Agent i's last tick as nav's plant ledger takes it: the body's attitude, roll rate, load
  // transfer and wheels on the ground are the model's (a wheel is off it while its spring is
  // unloaded), and with a world its contacts with the buildings and the other vehicles.
  cvc::nav::plant_sample sample(int i) const override;
  bool tracks_samples() const override { return true; }
  cvc::nav::plant_modulation modulation() const override { return _o.plant.modulation; }

  // ── the vehicles ──
  int size() const { return static_cast<int>(_agents.size()); }
  const vehicle_desc &desc(int i) const { return _agents.at(i).desc; }
  // The model, in SI: its parameters, its limits (the substep), and agent i's state (the rear
  // axle, the heading, the twist, heave, roll and pitch and their rates, the actuators). Its
  // heave is from the plant's reference height, ground_reference().
  const cvc::nav::diff::dyn4_params &params(int i) const { return _agents.at(i).p; }
  const cvc::nav::diff::bike_veh &step_limits(int i) const { return _agents.at(i).lim; }
  const cvc::nav::diff::dyn4_state<double> &body(int i) const { return _agents.at(i).s; }
  int substeps() const { return _nsub; }
  // The ground height (m) the model's heights are measured from: the ground under the first
  // agent's start.
  double ground_reference() const { return _z0; }
  // The ground (m) under a world point, as the model sees it.
  double ground_height(double x, double y) const;

  // ── the books ──
  const dyn_4w_books &books(int i) const { return _agents.at(i).total; }
  const dyn_4w_books &last_tick(int i) const { return _agents.at(i).tick; }
  double stored_at_attach(int i) const { return _agents.at(i).stored0; }

  // ── the world ──
  bool collides() const;
  body_id chassis(int i) const;
  int collisions_tick(int i) const { return _agents.at(i).hits_tick; }
  long collisions(int i) const { return _agents.at(i).hits; }
  // As dyn_bike_plant's.
  int vehicle_contact_step(int i) const { return _agents.at(i).touched; }

  const dyn_4w_plant_options &options() const { return _o; }

  // The ground sampler dyn4_plant reads: the plant's ground less its reference height.
  struct ground_sampler {
    const dyn_4w_plant *plant = nullptr;
    template <class S> void at(S wx, S wy, S &mu, S &height) const {
      mu = S(plant->_o.mu);
      height =
          S(plant->ground_height(static_cast<double>(wx), static_cast<double>(wy)) - plant->_z0);
    }
  };

private:
  struct agent {
    vehicle_desc desc;
    cvc::nav::diff::dyn4_params p;
    cvc::nav::diff::bike_veh lim{};
    cvc::nav::diff::dyn4_state<double> s{};
    double rear_x = 0.0;    // the rear axle, body x (m)
    double steer_cmd = 0.0; // rad
    double accel_cmd = 0.0; // m/s^2 along the heading
    dyn_driver driver;      // how the vehicle takes accel_cmd (dyn_driver)
    double stored0 = 0.0;   // J
    int hits_tick = 0;
    long hits = 0;
    int statics_tick = 0;     // contacts begun with static geometry over the last tick
    int touched = -1;         // the agent it touched deepest over the last tick; -1 none
    double touch_depth = 0.0; // m
    dyn_4w_books tick, total;
    cvc::nav::plant_sample sample;
  };

  void measure(agent &a, double vx0, double vy0, double psi0, bool tick) const;
  void chassis_pose(const agent &a, const cvc::nav::diff::dyn4_state<double> &s, vec3 &position,
                    quat &rotation) const;

  std::vector<vehicle_desc> _descs;
  dyn_4w_plant_options _o;
  cvc::world_units _units;
  cvc::world_units::normalized_frame _frame;
  double _dt = 0.0;
  double _z0 = 0.0;
  int _nsub = 1;
  std::vector<agent> _agents;
  std::unique_ptr<dyn::dyn_chassis> _chassis;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_4W_PLANT_H__
