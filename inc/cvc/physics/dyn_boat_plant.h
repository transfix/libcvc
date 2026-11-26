/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::dyn_boat_plant: nav's agents on boats (boat.h, roadmap D3) as their live plant, in
// a physics world that holds their water, with an energy ledger like the dynamic plants' (roadmap
// docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D4).
//
//   cvc::physics::world w;                      // Jolt
//   w.add_field(sea);                           // a fluid field: calm, or with a fluid_surface
//   cvc::physics::dyn_boat_plant_options o;
//   o.physics = &w;
//   cvc::physics::dyn_boat_plant plant({cvc::physics::dyn_boat_desc::launch()}, o);
//   sim.set_plant(&plant);                      // the agents drive launches on the water
//   for (;;) sim.step();
//   plant.books(0);                             // where agent 0's energy went
//
// Agent i drives descs[i % size]: attach() adds a boat (world::add_boat) for each agent, its hull's
// origin over the agent's start, heading its way, upright and at rest, floating at its draft on the
// water there (float_height). The water is whatever fluid field the world has over the start
// (world::water_at), a still one or one with a free surface (a swell); a start with none is
// refused. Nav's point is the hull's origin: state() is its position, the forward axis's heading,
// and its horizontal velocity over the ground along that heading.
//
// The driver. Nav's command is a bicycle angle and an acceleration along the heading, as for the
// wheeled plants. Every world step the driver turns it into the boat's controls
// (world::set_boat_controls):
//   - the throttle from the thrust per unit mass that would give the acceleration: as dyn_driver.h
//     takes nav's acceleration (the command, half of what the last step missed by, and an integral
//     trim that learns at 2 per second what the water holds back, kept when the command changes
//     sign, since the water holds the boat back either way), over the propeller's thrust at the
//     boat's speed through the water, T_max (1 - u / max_inflow_speed); within 0.05 m/s^2 of no
//     command below 0.3 m/s it holds the boat still on its propeller instead;
//   - the rudder from the yaw rate the bicycle would turn at, r = u tan(delta) / L (L the hull's
//     length, the plant's wheelbase): the rudder for that curvature at full rudder's (tan(delta) /
//     tan(max_steer), max_steer = atan(L / turn_radius)), plus a proportional and integral term on
//     the yaw rate's error taken as a curvature, (r_cmd - r) / u. A rudder works reversed astern,
//     and so does that curvature, so the same law steers backing.
// With options.driver off the command is the controls, raw: the throttle the acceleration over the
// rated thrust per unit mass, the rudder tan(delta) / tan(max_steer).
//
// The books. Every world step the plant books the hull's mechanical energy (dyn_boat_books):
//   kinetic now = kinetic at attach + drive - brake - rudder - damping + water + gravity + residual
// kinetic is the rigid hull's, 1/2 m |v|^2 + 1/2 w.I w (its inertia the collision shape's: a box's
// or a sphere's exactly, a convex hull's from the hull_mesh as a solid). drive and brake are the
// thrust's power through the water, T u_a with u_a the propeller's velocity through the water along
// its push, where it pushes the boat on and where it holds it back (thrust reversed); rudder the
// work the rudder's force does against the water flowing past it, its drag's (>= 0 as a rule);
// damping the hull body's own linear and angular damping (0 for a boat whose resistance is the
// water's alone, as the presets); water what the water does to the hull (its buoyancy, resistance,
// damping and slamming, world::field_load_on) and the water's own motion's push on the propeller
// and the rudder (a swell's orbital velocity, a current); gravity m g along the centre of mass's
// fall. Each force's work is taken at the step's mean velocity, which is what the world's
// integrator moves the hull by, so residual is its error and a collision's work (the presets on
// open water: a few parts in 10^5 of the flow).
//
// Stepping. A tick is dt / fixed_dt world steps (a whole number, or attach() throws). With
// options.step_world (the default) advance() runs them. Off, a host that steps the world itself
// with the rest of what is in it calls before_world_step() before each of its steps and
// after_world_step() after it, and advance() ends the tick they ran and begins the next under the
// new command, as dyn_bike_plant's does.
//
// The boats stay in the world when the plant goes, unless it removes them (remove_vehicle, a new
// attach()). Positions, speeds and accelerations cross into nav's normalized frame through
// world_units with the sim_world's frame. Standard library and det:: math, so a run is a function
// of its inputs on every platform where the world is.

#ifndef __CVC_PHYSICS_DYN_BOAT_PLANT_H__
#define __CVC_PHYSICS_DYN_BOAT_PLANT_H__

#include <array>
#include <cvc/core/world_units.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/boat.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/world.h>
#include <string>
#include <vector>

namespace cvc {
namespace physics {

// One boat as the plant drives it: its description, and what nav plans with.
struct dyn_boat_desc {
  // The boat (boat.h). Its hull must have a hull_mesh: the plant floats the boat at the draft the
  // mesh gives and books the hull's energy from its shape.
  boat_desc boat;
  // Its top speed (m/s) through calm water at full throttle: nav's vmax for it.
  double max_speed = 3.0;
  // The radius (m) of its steady turn at full rudder and full throttle on calm water: nav's
  // bicycle turns no tighter (max_steer = atan(L / turn_radius), L the hull's length).
  double turn_radius = 20.0;
  // Its length overall (m), with what reaches past the hull (outboards, a stem head): the length
  // a renderer fits the boat's model to, about the hull body's origin (cvcGL's boat node). 0: the
  // hull_mesh's length.
  double length_overall = 0.0;

  // The presets, their max_speed and turn_radius measured on water of drag_coefficient 1 and
  // damping 1e4 (physics_dyn_boat_plant_test, PresetsDoWhatTheySay):
  //   launch   a 4 x 1.5 x 0.6 m box hull of 900 kg with a shaft propeller (2 kN ahead, 1 kN
  //            astern, no thrust past 8 m/s) and a 0.12 m^2 rudder at the stern;
  //            physics_boat_test's boat. 3.19 m/s, 15.4 m.
  //   jonboat  a 3.6 x 1.4 x 0.5 m flat box of 350 kg with a 1.5 kN outboard (no thrust past
  //            9 m/s) that swings 0.5 rad to steer, its leg a 0.03 m^2 rudder. 4.0 m/s, 4.1 m.
  //   tactical the vehicle-tactical-boat package's patrol boat: its own V hull, baked from the
  //            model into 16 stations (loft_hull; tools/boat_hulls/bake_loft.py), 6.85 m long
  //            and 2.33 m in beam, 7.8 m overall with its outboards; 2800 kg, floating 0.49 m
  //            over its keel; the hull's own lift (hull_mesh::lift_coefficient 0.56) for its
  //            grip sideways and its planing; its two outboards one thrust (8 kN ahead, 3.5 kN
  //            astern, none past 18 m/s) that swings 0.3 rad to steer, and their legs one
  //            0.2 m^2 rudder at the propellers that swings with them. 10.2 m/s, 32.3 m.
  static dyn_boat_desc launch();
  static dyn_boat_desc jonboat();
  static dyn_boat_desc tactical();
  // "launch", "jonboat" or "tactical"; another name throws cvc::physics_error.
  static dyn_boat_desc preset(const std::string &name);
  static std::vector<std::string> presets();
};

struct dyn_boat_plant_options {
  // The plan, latency, margins, nav's frame in metres and the plant ledger's modulation, as
  // nav_plant reads them (its ground, spawn, stepping and drift settings do not apply). The boat's
  // latency is the plan's plus its actuators' time constant.
  nav_plant_options plant;
  // The world the boats float in (borrowed, required): it must hold a fluid field over each
  // agent's start, and outlive the plant's use of it. attach() sets its contact reporting to all.
  world *physics = nullptr;
  // Nav's command through the driver (above); false: the controls, raw.
  bool driver = true;
  // Whether advance() steps the world (above).
  bool step_world = true;
};

// One agent's energy, in J: over the last tick, or since attach().
struct dyn_boat_books {
  double kinetic = 0.0;  // 1/2 m |v|^2 + 1/2 w.I w at the end
  double drive = 0.0;    // T u_a where the thrust pushes the boat on through the water (>= 0)
  double brake = 0.0;    // where it holds it back (>= 0)
  double rudder = 0.0;   // the rudder's force against the water past it (>= 0 as a rule)
  double damping = 0.0;  // the hull body's own damping (>= 0)
  double water = 0.0;    // the water's work on the hull, its propeller and its rudder
  double gravity = 0.0;  // m g along the centre of mass's fall
  double residual = 0.0; // the integrator's, and any collision's
  // Everything that moved, for the residual's scale.
  double flow() const;
};

class dyn_boat_plant final : public cvc::nav::vehicle_plant {
public:
  // `descs` must not be empty; each must be a boat world::add_boat takes, with a hull_mesh, a
  // max_speed and a turn_radius > 0. options.physics must be set.
  explicit dyn_boat_plant(std::vector<dyn_boat_desc> descs, dyn_boat_plant_options options = {});
  // Leaves its boats in the world (which may already be gone).
  ~dyn_boat_plant() override = default;
  dyn_boat_plant(const dyn_boat_plant &) = delete;
  dyn_boat_plant &operator=(const dyn_boat_plant &) = delete;

  // ── cvc::nav::vehicle_plant ──
  cvc::nav::plant_kind kind() const override { return cvc::nav::plant_kind::dynamic; }
  cvc::nav::plant_medium medium() const override { return cvc::nav::plant_medium::water; }
  // Removes the boats of an earlier attach() and adds one per agent. Throws cvc::physics_error
  // for a tick that is not a whole number of world steps, a start with no water under it, water
  // shallower than the boat's draft (the field's box, or a body without mass, such as the terrain,
  // above the boat's deepest point), or a boat too heavy to float; a refused attach leaves none of
  // its boats in the world (the earlier ones are gone).
  void attach(int n, const cvc::nav::plant_state *start,
              const cvc::world_units::normalized_frame &frame, double dt) override;
  // The hull's origin, the heading, the forward speed over the ground there, the steering (the
  // bicycle angle the driver holds the yaw rate for: nav's last command, within the lock), and the
  // hull's pitch and roll. The rudder itself, which the driver moves to hold that yaw rate, is
  // boat_state's: reporting it as the steering would feed the driver's corrections back into
  // nav's steering-rate limit.
  cvc::nav::plant_state state(int i) const override;
  // The boat's envelope: the bicycle's wheelbase (the hull's length) and its max_steer (from the
  // turn_radius), the rudder's rate (max_steer over the actuators' time constant), braking on
  // max_reverse_thrust / m within brake_margin, the latency (above), and with plan_with_vehicle
  // accel_margin x max_thrust / m, max_speed, the lateral acceleration of its tightest turn at
  // top speed (no more than lateral_g) and the hull's outline about its origin.
  cvc::nav::plant_limits limits(int i) const override;
  void command(int i, float steer, float accel) override;
  // A helm of agent i's own in place of nav's: its controls held (world::set_boat_controls) from
  // the next world step until release_controls(i), whatever nav commands. The books go on. Throws
  // cvc::physics_error for a throttle or rudder outside [-1, 1].
  void hold_controls(int i, const boat_controls &c);
  void release_controls(int i);
  bool held(int i) const { return _agents.at(static_cast<std::size_t>(i)).hold; }
  void advance() override;
  int reference_substeps() const override { return _nsub; }
  // The drive energy of agent i's last tick (J): what its thrust delivered through the water.
  double energy_step(int i) const override;
  // The contacts agent i's hull began with static geometry (a shore, a pier) over the last tick.
  int static_contacts_step(int i) const override;
  bool tracks_energy() const override { return true; }
  // Agent i's last tick as nav's plant ledger takes it: the centre of mass's velocity in the
  // heading's frame, the yaw rate, the steering (as state()'s), the hull's attitude and roll
  // rate, its accelerations over the tick, and its contacts with static geometry and the other
  // boats. It has no wheels or load transfer (plant_sample::measured leaves them out).
  cvc::nav::plant_sample sample(int i) const override;
  bool tracks_samples() const override { return true; }
  cvc::nav::plant_modulation modulation() const override { return _o.plant.modulation; }
  // Boats at run time: a new one takes descs[vehicle] (-1: descs[index % size]) at rest, as
  // attach() places them; a removed one's boat leaves the world.
  bool resizable() const override { return true; }
  int add_vehicle(const cvc::nav::plant_state &start, int vehicle = -1) override;
  void remove_vehicle(int i) override;
  // Another boat to add_vehicle() by its index, which this returns (checked as the constructor
  // checks its descs).
  int add_desc(dyn_boat_desc d);

  // ── the boats ──
  int size() const { return static_cast<int>(_agents.size()); }
  const dyn_boat_desc &desc(int i) const { return _agents.at(static_cast<std::size_t>(i)).desc; }
  boat_id boat(int i) const { return _agents.at(static_cast<std::size_t>(i)).boat; }
  body_id hull(int i) const { return _agents.at(static_cast<std::size_t>(i)).hull; }
  // world::boat_state of agent i's boat.
  physics::boat_state boat_state(int i) const;
  int substeps() const { return _nsub; }
  // The bicycle nav plans agent i with: its wheelbase (m) and steering lock (rad).
  double wheelbase(int i) const { return _agents.at(static_cast<std::size_t>(i)).L; }
  double max_steer(int i) const { return _agents.at(static_cast<std::size_t>(i)).max_steer; }

  // ── the books ──
  const dyn_boat_books &books(int i) const { return _agents.at(static_cast<std::size_t>(i)).total; }
  const dyn_boat_books &last_tick(int i) const {
    return _agents.at(static_cast<std::size_t>(i)).tick;
  }
  // The kinetic energy at attach() (J), the books' start: 0, at rest.
  double kinetic_at_attach(int i) const { return _agents.at(static_cast<std::size_t>(i)).kinetic0; }

  // ── a run continued ──
  // What agent i's next steps depend on that the world's save() does not hold: its driver's
  // memory (the speed loop's integral trim and the acceleration it last measured, the rudder's
  // integral trim) and nav's command it holds (rad, m/s^2).
  struct memory {
    double trim = 0.0, got = 0.0, yaw_trim = 0.0;
    double steer = 0.0, accel = 0.0;
  };
  memory remember(int i) const;
  // After the world was restored (world::restore) to a state saved with this plant's boats in it:
  // agent i's memory as remember() gave it then, and its boat as the world has it now, so the
  // steps from there are the ones the plant took from that state. A held helm (hold_controls)
  // stays as it is.
  void resume(int i, const memory &m);

  // ── the world ──
  world *physics() const { return _o.physics; }
  // With options.step_world off: the driver's controls for every boat before each of the host's
  // world steps, and the books after it. A tick's steps past substeps() keep its command.
  void before_world_step();
  void after_world_step();
  // The world was made afresh (world::operator= over it): its boats went with the old one, and
  // their ids may name new ones. The plant forgets them and the world, so removing a boat (or the
  // plant) touches nothing; it steps no world from then on.
  void world_gone();

  // ── hydrostatics ──
  // How far above the waterline the hull's origin floats, upright, in still water of `density`
  // (kg/m^3): the draft at which the hull_mesh displaces the hull's mass. Throws
  // cvc::physics_error for a boat without a hull_mesh, or one too heavy to float.
  static double float_height(const boat_desc &d, double mass, double density);
  // How deep the boat reaches below the waterline there (m): its keel, or its propeller or rudder
  // where they reach deeper. Throws as float_height does.
  static double draft(const boat_desc &d, double mass, double density);

  const dyn_boat_plant_options &options() const { return _o; }

private:
  // The hull as the books and the driver read it: its mass properties, body-local.
  struct hull_props {
    double mass = 0.0;
    vec3 com;                  // the centre of mass
    std::array<double, 9> I{}; // the inertia about it, row-major
    double linear_damping = 0.0, angular_damping = 0.0;
  };
  // The driver's memory: as dyn_driver for the speed, and the rudder's integral trim.
  struct driver_state {
    double trim = 0.0, got = 0.0;
    double yaw_trim = 0.0;
  };
  // A world step's start: the hull, its energy and the water at its propeller and rudder.
  struct step_start {
    body_state s;
    vec3 com; // world
    double kinetic = 0.0;
    double u = 0.0;    // nav's forward speed over the ground
    vec3 prop, rudder; // the points, world
    vec3 water_prop, water_rudder;
    bool on = false;
  };
  struct agent {
    dyn_boat_desc desc;
    boat_id boat;
    body_id hull;
    hull_props props;
    double L = 0.0, max_steer = 0.0, tan_max = 0.0;
    double half_width = 0.0, front = 0.0, rear = 0.0; // the outline about the origin (m)
    double steer_cmd = 0.0;                           // rad
    double accel_cmd = 0.0;                           // m/s^2 along the heading
    driver_state driver;
    bool hold = false; // hold_controls: `helm` in place of nav's command
    boat_controls helm;
    double kinetic0 = 0.0;
    dyn_boat_books tick, total;
    int statics_tick = 0;
    int touched = -1;
    double touch_depth = 0.0;
    physics::boat_state now; // as of the end of the last step (or attach)
    cvc::nav::plant_sample sample;
    // The tick's start (its measure()): the centre of mass's world velocity and the heading.
    double vwx0 = 0.0, vwy0 = 0.0, psi0 = 0.0;
    step_start start;
  };

  void check(const dyn_boat_desc &d) const;
  // A boat `desc` at rest at `start`, appended (attach's, add_vehicle's).
  void place(const cvc::nav::plant_state &start, const dyn_boat_desc &desc);
  // Agent a's hull: its kinetic energy, its centre of mass (world), nav's forward speed and
  // heading.
  double kinetic_of(const agent &a, const body_state &s) const;
  vec3 com_of(const agent &a, const body_state &s) const;
  void heading_speed(const agent &a, const body_state &s, double &psi, double &u) const;
  // The driver's controls for agent a now, and the step's start for the books.
  void drive(agent &a, double h);
  // The books of agent a's step that just ran, and the driver's measure of it.
  void book(agent &a, double h);
  void count_contacts();
  void begin_tick();
  void end_tick();
  void measure(agent &a, bool tick) const;

  std::vector<dyn_boat_desc> _descs;
  dyn_boat_plant_options _o;
  cvc::world_units _units;
  cvc::world_units::normalized_frame _frame;
  double _dt = 0.0;
  int _nsub = 1;
  bool _ticking = false; // step_world off: a tick has begun
  std::vector<agent> _agents;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DYN_BOAT_PLANT_H__
