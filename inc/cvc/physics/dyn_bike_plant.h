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
// agent's start, at rest.
//
// The ground. With options.plant.ground_height the vehicle climbs and descends it: dyn_plant
// takes the sine of the slope along the heading under the centre of gravity (over a wheelbase),
// and the books gain a grade line. Without it the ground is flat. With options.side_slope the
// slope across the heading acts too, and gravity is a potential (diff::dyn_plant_ground): the
// slopes are the ground's gradient at the centre of gravity, and the grade line is minus the
// potential's change, m g times the centre of gravity's rise.
//
// The world. With options.physics (plant_host gives it a Jolt world holding the scene's terrain
// and buildings) each vehicle's chassis box is a body in that world, and the vehicles collide with
// the buildings, with each other and with anything else in it (dyn_world.h): every substep the
// world takes one step with each box moving as its model moved, and a box that touched something
// that is not the ground (options.ground_bodies) takes the world's pose and twist back into its
// model, the energy that changed booked as the contact's. The substep is then the world's
// fixed_dt, and nav's tick must be a whole number of them. Without a world nothing collides.
//
// The driver. Nav's acceleration is what the vehicle is to achieve, and the plant takes it as the
// Jolt plant's driver does (dyn_driver.h): held on the brakes at a standstill, and the drive force
// the model gets corrected for the slope and for what the last substep fell short by, within the
// vehicle's max_accel and max_decel (options.driver; off, the command is the force, raw).
//
// Each tick runs dyn_plant's substeps under the held command (the bicycle angle, and the drive
// force per unit mass the driver asks for), in SI, and books every step (dyn_plant_books):
//   kinetic now = kinetic at attach + drive - brake - dissipated - slip + floor + grade + limiter
//                 + contact - contact_stored - contact_dissipated + contact_exchange + residual
// drive and brake are the axles' forces where they push and where they hold back, dissipated the
// tires' slip, drag and rolling resistance, slip (with options.wheel_spin, diff::dyn_spin_plant's
// spinning wheels; else 0) the tires' longitudinal slip, floor what the tires' secants leave,
// grade gravity along the slope, limiter the kinetic energy the speed clamp and the low-speed
// blend took, contact what the collisions changed, and residual the integrator's error (second
// order per step). Positions, speeds and accelerations cross into nav's normalized frame through
// world_units with the sim_world's frame, as nav_plant's do.

#ifndef __CVC_PHYSICS_DYN_BIKE_PLANT_H__
#define __CVC_PHYSICS_DYN_BIKE_PLANT_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/detail/dyn_contact.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/dyn_driver.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <functional>
#include <memory>
#include <vector>

namespace cvc {
namespace nav {
struct field_stack;
}
namespace physics {

namespace dyn {
class dyn_chassis;
}

struct dyn_bike_plant_options {
  // The plan, latency, margins, nav's frame in metres and the ground (ground_height, else
  // ground_z), as nav_plant reads them (its spawn, stepping and drift settings do not apply).
  nav_plant_options plant;
  double mu = 0.9;          // the ground's grip
  double substep = 0.01;    // s: the longest dyn_plant step without a world; a tick takes
                            // ceil(dt / substep) (with a world: the world's fixed_dt)
  double drive_lag = -1.0;  // s: the drive force's lag; < 0 the vehicle's own (dyn_bike_si_of)
  double gravity = 9.80665; // m/s^2
  // Nav's acceleration as the Jolt plant's driver takes it (dyn_driver.h), so nav drives this plant
  // as it drives that one; false: as the model's drive force per unit mass, raw.
  bool driver = true;
  // The world the chassis collide in (borrowed; null: no collisions), and its bodies the wheels
  // drive on (touching them is not a collision). attach() sets its contact reporting to all.
  world *physics = nullptr;
  std::vector<body_id> ground_bodies;
  // The chassis box's friction against what it hits (combined by the world's rule: the product,
  // by default, so 0.21 against a building's 0.7): a body panel scraping masonry or another body,
  // not a tire's grip. At 1, the default material's, a vehicle pressed against a wall by its own
  // tires grips it like a tire and crawls along it.
  double chassis_friction = 0.3;
  // Wheel spin (diff::dyn_spin_plant): each axle's wheels spin at their own speed, driven and
  // braked by torques, and the tires push with their longitudinal slip, so the vehicle spins its
  // wheels on low grip and locks them braking past it. The books gain a slip line, and the
  // kinetic energy the wheels' spin. Off (the default), diff::dyn_plant: each axle's force is the
  // drive's within its grip.
  bool wheel_spin = false;
  // Compliant contact (cvc/nav/detail/dyn_contact.h): each vehicle's footprint against
  // contact_field and, with contact_vehicles, the vehicles against each other, as a barrier energy
  // and a damper through the plant's external port. The books gain the contact lines. Off by
  // default; it does not need a world (options.physics, which collides the chassis in Jolt
  // instead: use one or the other).
  bool contact = false;
  cvc::nav::diff::dyn_contact_params contact_params;
  // The field, world metres: the signed distance at (x, y) (> 0 free) and its gradient. Empty:
  // none (the vehicles still meet each other). dyn_bike_plant::sdf_field builds one from nav's.
  std::function<void(double x, double y, double &phi, double &gx, double &gy)> contact_field;
  bool contact_vehicles = true;
  // The slope across the heading (diff::dyn_plant_ground, dyn_plant_books_ground): gravity acts
  // along and across the heading, from the ground's gradient under the centre of gravity (its
  // tangents, so m g h is the potential of the planar model), and the grade line is - dV, the
  // potential's change over the step. Off (the default), the sine along the heading alone, as
  // data. It reads options.plant.ground_height (flat without one) and takes wheel_spin too.
  bool side_slope = false;
};

// One agent's energy, in J: over the last tick, or since attach().
struct dyn_bike_books {
  double kinetic = 0.0;    // 1/2 v^T M v at the end (wheel_spin: and the wheels' spin)
  double drive = 0.0;      // the axles' forces where they push (>= 0); wheel_spin: the drive's
                           // torques at the wheels' speeds
  double brake = 0.0;      // where they hold back (>= 0); wheel_spin: the brakes' torques
  double dissipated = 0.0; // the tires' slip, drag and rolling resistance (>= 0)
  double slip = 0.0;       // wheel_spin: the tires' longitudinal slip (>= 0; 0 without)
  double floor = 0.0;      // what the tires' secants leave: round-off
  double grade = 0.0;      // gravity along the heading (0 on flat ground); side_slope: - dV
  double limiter = 0.0;    // the speed clamp's and the low-speed blend's (<= 0 as a rule)
  double contact = 0.0;    // what the collisions changed (0 without a world)
  // With options.contact: the barrier's energy at the end (a level, J), its change (stored), the
  // damper's work (dissipated, >= 0 as a rule) and what the vehicles passed each other through
  // their contacts (exchange: summed over a pair it is the integrator's error alone).
  double contact_energy = 0.0;
  double contact_stored = 0.0, contact_dissipated = 0.0, contact_exchange = 0.0;
  double residual = 0.0; // the integrator's
  // Everything that moved, for the residual's scale.
  double flow() const;
};

class dyn_bike_plant final : public cvc::nav::vehicle_plant {
public:
  // `descs` must not be empty.
  explicit dyn_bike_plant(std::vector<vehicle_desc> descs, dyn_bike_plant_options options = {});
  ~dyn_bike_plant() override;
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
  // The contacts agent i's chassis began with static geometry over the last tick (0 without a
  // world), as nav_plant counts them.
  int static_contacts_step(int i) const override;
  bool tracks_energy() const override { return true; }
  // Agent i's last tick as nav's plant ledger takes it: the body's attitude is the ground's
  // under it (the single track has no roll or pitch of its own), and with a world its contacts
  // with the buildings and the other vehicles. It has no load transfer, roll rate or wheels off
  // the ground (plant_sample::measured leaves them out).
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
  // Agent i's wheels' speeds (rad/s, + rolling forward; axle 0 the front, 1 the rear): with
  // wheel_spin their own, else the speed's at the rolling radius (they do not slip).
  double wheel_speed(int i, int axle) const;
  int substeps() const { return _nsub; }

  // ── the books ──
  const dyn_bike_books &books(int i) const { return _agents.at(i).total; }
  const dyn_bike_books &last_tick(int i) const { return _agents.at(i).tick; }
  // The kinetic energy at attach() (J), the books' start.
  double kinetic_at_attach(int i) const { return _agents.at(i).kinetic0; }

  // ── the world ──
  // Whether the chassis collide (options.physics was given).
  bool collides() const;
  // Agent i's chassis box in the world (invalid without one).
  body_id chassis(int i) const;
  // The substeps of agent i's last tick in which it collided, and all since attach().
  int collisions_tick(int i) const { return _agents.at(i).hits_tick; }
  long collisions(int i) const { return _agents.at(i).hits; }
  // Of them, the other agent whose chassis agent i touched over the last tick (the deepest
  // contact's; -1 none), as nav_plant::vehicle_contact_step.
  int vehicle_contact_step(int i) const { return _agents.at(i).touched; }

  // ── compliant contact (options.contact) ──
  // Agent i's outline about its centre of gravity: its chassis box and its wheels, as limits()
  // reads them.
  const cvc::nav::diff::dyn_footprint &footprint(int i) const { return _agents.at(i).fp; }
  // The contact on agent i at the start of its last substep: wrench, energy, touching points.
  const cvc::nav::diff::dyn_contact &contact_now(int i) const { return _agents.at(i).cnow; }
  // A contact field from nav's signed-distance grid (sim_world's; plane `plane`), in world metres
  // through `metres_per_world_unit`: phi bilinear between the grid's samples and its own gradient,
  // so the barrier's force is its energy's. Borrows fs.data: the grid must outlive the field.
  static std::function<void(double, double, double &, double &, double &)>
  sdf_field(const cvc::nav::field_stack &fs, int plane = 0, double metres_per_world_unit = 1.0);

  const dyn_bike_plant_options &options() const { return _o; }

private:
  struct agent {
    vehicle_desc desc;
    cvc::nav::diff::dyn_bike_params p;
    cvc::nav::diff::bike_veh lim{};
    cvc::nav::diff::dyn_state<double> s{};
    double om[2] = {0.0, 0.0}; // wheel_spin: the front and rear wheels' speeds (rad/s)
    cvc::nav::diff::dyn_footprint fp;              // the outline about the centre of gravity
    std::vector<cvc::nav::diff::dyn_point> points; // its contact points
    cvc::nav::diff::dyn_contact cnow;              // options.contact: at the last substep's start
    double rear_x = 0.0;       // the rear axle, body x (m)
    double half_track = 0.0;   // the widest axle's (m)
    double steer_cmd = 0.0;    // rad
    double accel_cmd = 0.0;    // m/s^2 along the heading
    dyn_driver driver;         // how the vehicle takes accel_cmd (dyn_driver)
    double kinetic0 = 0.0;     // J
    int hits_tick = 0;
    long hits = 0;
    int statics_tick = 0;     // contacts begun with static geometry over the last tick
    int touched = -1;         // the agent it touched deepest over the last tick; -1 none
    double touch_depth = 0.0; // m
    dyn_bike_books tick, total;
    cvc::nav::plant_sample sample;
  };

  // Agent a's plant_sample; `tick`: over a tick of _dt from (vx0, vy0) at heading psi0.
  void measure(agent &a, double vx0, double vy0, double psi0, bool tick) const;
  // options.contact: every agent's contact at the model states `s`, against the field (field[i])
  // and against the other agents (pair[i], its half of each pair's energy and dissipation).
  void contacts(const std::vector<cvc::nav::diff::dyn_state<double>> &s,
                std::vector<cvc::nav::diff::dyn_contact> &field,
                std::vector<cvc::nav::diff::dyn_contact> &pair) const;
  // Agent a's chassis box at state s: its origin, on the ground under it, and its attitude.
  void chassis_pose(const agent &a, const cvc::nav::diff::dyn_state<double> &s, vec3 &position,
                    quat &rotation) const;

  std::vector<vehicle_desc> _descs;
  dyn_bike_plant_options _o;
  cvc::world_units _units;
  cvc::world_units::normalized_frame _frame;
  double _dt = 0.0;
  int _nsub = 1;
  std::vector<agent> _agents;
  std::unique_ptr<dyn::dyn_chassis> _chassis;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_BIKE_PLANT_H__
