/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_bike_plant: nav's agents on the dynamic single-track vehicle
// (cvc/nav/detail/dyn_bike.h, P8a) as their live plant, with its port-Hamiltonian energy books.
//
//   cvc::physics::dyn_bike_plant plant({cvc::physics::vehicle_desc::humvee()});
//   sim.set_plant(&plant); // the agents drive the Humvee's dynamic bicycle: tires, load
//                          // transfer and actuator lag, no Jolt world
//   for (;;) sim.step();
//   plant.books(0);        // where agent 0's energy went
//
// This is the plant the coefficient trainer's rollout_kind::dynamic trains through, from the same
// vehicle_desc as the Jolt plant (nav_plant::dyn_bike_si_of), so a policy trained on it drives
// on its own model here. Agent i drives descs[i % size]; attach() puts its rear axle on the
// agent's start, at rest. It needs no physics backend: no ground, no collisions, flat (no grade).
//
// Each tick runs dyn_plant ceil(dt / substep) times under the held command (the bicycle angle,
// and the acceleration along the heading), in SI, and books every step (dyn_plant_books):
//   kinetic now = kinetic at attach + drive - brake - dissipated + floor + grade + limiter
//                 + residual
// drive and brake are the axles' forces where they push and where they hold back, dissipated the
// tires' slip, drag and rolling resistance, floor the slips' speed floor, limiter the kinetic
// energy the speed clamp and the low-speed blend took, and residual the integrator's error (second
// order per step). Positions, speeds and accelerations cross into nav's normalized frame through
// world_units with the sim_world's frame, as nav_plant's do.

#ifndef __CVC_PHYSICS_DYN_BIKE_PLANT_H__
#define __CVC_PHYSICS_DYN_BIKE_PLANT_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <vector>

namespace cvc {
namespace physics {

struct dyn_bike_plant_options {
  // The plan, latency, margins and nav's frame in metres, as nav_plant reads them (its ground,
  // spawn, stepping and drift settings do not apply).
  nav_plant_options plant;
  double mu = 0.9;          // the ground's grip
  double substep = 0.01;    // s: the longest dyn_plant step; a tick takes ceil(dt / substep)
  double drive_lag = -1.0;  // s: the drive force's lag; < 0 the vehicle's own (dyn_bike_si_of)
  double gravity = 9.80665; // m/s^2
};

// One agent's energy, in J: over the last tick, or since attach().
struct dyn_bike_books {
  double kinetic = 0.0;    // 1/2 v^T M v at the end
  double drive = 0.0;      // the axles' forces where they push (>= 0)
  double brake = 0.0;      // where they hold back (>= 0)
  double dissipated = 0.0; // the tires' slip, drag and rolling resistance (>= 0)
  double floor = 0.0;      // the slips' speed floor
  double grade = 0.0;      // gravity along the heading (0 on this flat plant)
  double limiter = 0.0;    // the speed clamp's and the low-speed blend's (<= 0 as a rule)
  double residual = 0.0;   // the integrator's
  // Everything that moved, for the residual's scale.
  double flow() const;
};

class dyn_bike_plant final : public cvc::nav::vehicle_plant {
public:
  // `descs` must not be empty.
  explicit dyn_bike_plant(std::vector<vehicle_desc> descs, dyn_bike_plant_options options = {});
  ~dyn_bike_plant() override = default;
  dyn_bike_plant(const dyn_bike_plant &) = delete;
  dyn_bike_plant &operator=(const dyn_bike_plant &) = delete;

  // ── cvc::nav::vehicle_plant ──
  cvc::nav::plant_kind kind() const override { return cvc::nav::plant_kind::dynamic; }
  void attach(int n, const cvc::nav::plant_state *start,
              const cvc::world_units::normalized_frame &frame, double dt) override;
  // The rear axle, the heading, the forward speed, the steering's angle; tire_use is the most
  // used tire's slip angle over its peak (0 below 2 m/s, as nav_plant's).
  cvc::nav::plant_state state(int i) const override;
  // As nav_plant's: the vehicle's steering, braking and latency, and with plan_with_vehicle its
  // acceleration, top speed, cornering and size.
  cvc::nav::plant_limits limits(int i) const override;
  void command(int i, float steer, float accel) override;
  void advance() override;
  int reference_substeps() const override { return _nsub; }
  // The drive energy of agent i's last tick (J): what its axles' forces delivered.
  double energy_step(int i) const override;
  bool tracks_energy() const override { return true; }
  cvc::nav::plant_sample sample(int i) const override;
  bool tracks_samples() const override { return true; }
  cvc::nav::plant_modulation modulation() const override { return _o.plant.modulation; }

  // ── the vehicles ──
  int size() const { return static_cast<int>(_agents.size()); }
  const vehicle_desc &desc(int i) const { return _agents.at(i).desc; }
  // The model, in SI: its parameters, its limits (the substep), and agent i's state (the rear
  // axle, the heading, the body twist at the centre of gravity, the actuators).
  const cvc::nav::diff::dyn_bike_params &params(int i) const { return _agents.at(i).p; }
  const cvc::nav::diff::bike_veh &step_limits(int i) const { return _agents.at(i).lim; }
  const cvc::nav::diff::dyn_state<double> &body(int i) const { return _agents.at(i).s; }
  int substeps() const { return _nsub; }

  // ── the books ──
  const dyn_bike_books &books(int i) const { return _agents.at(i).total; }
  const dyn_bike_books &last_tick(int i) const { return _agents.at(i).tick; }
  // The kinetic energy at attach() (J), the books' start.
  double kinetic_at_attach(int i) const { return _agents.at(i).kinetic0; }

  const dyn_bike_plant_options &options() const { return _o; }

private:
  struct agent {
    vehicle_desc desc;
    cvc::nav::diff::dyn_bike_params p;
    cvc::nav::diff::bike_veh lim{};
    cvc::nav::diff::dyn_state<double> s{};
    double rear_x = 0.0;    // the rear axle, body x (m)
    double steer_cmd = 0.0; // rad
    double accel_cmd = 0.0; // m/s^2 along the heading
    double kinetic0 = 0.0;  // J
    dyn_bike_books tick, total;
    cvc::nav::plant_sample sample;
  };

  // Agent a's plant_sample; `tick`: over a tick of _dt from (vx0, vy0) at heading psi0.
  void measure(agent &a, double vx0, double vy0, double psi0, bool tick) const;

  std::vector<vehicle_desc> _descs;
  dyn_bike_plant_options _o;
  cvc::world_units _units;
  cvc::world_units::normalized_frame _frame;
  double _dt = 0.0;
  int _nsub = 1;
  std::vector<agent> _agents;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_BIKE_PLANT_H__
