/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics boats (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md D3): a hull body driven by a
// propeller and steered by a rudder or an outboard, whose forces the world applies every step.
//
//   cvc::physics::boat_desc d;
//   d.hull.shape = cvc::physics::box{{2.0, 0.75, 0.3}};    // what it collides with
//   d.hull.hull = cvc::physics::box_hull({2.0, 0.75, 0.3}); // what the water pushes on
//   d.hull.mass = 900.0;
//   d.propeller.position = {-2.1, 0.0, -0.35};
//   d.rudder.position = {-2.25, 0.0, -0.35};
//   d.rudder.area = 0.15;
//   const auto b = w.add_boat(d, {0, 0, 0});
//   w.set_boat_controls(b, {1.0, 0.3});   // full ahead, rudder 30 % to port
//   w.step();
//   const auto s = w.boat_state(b);       // speed through the water, heading, thrust, ...
//
// The boat is a dynamic body (boat_state::hull; world::state reads it) that the fluid fields
// float and resist as they do any body (hull.h: a hull_mesh, or the body's sample cells
// without one). Frame: body-local x forward, y left, z up. A positive rudder control turns the
// boat left (counter-clockwise seen from above) going ahead.
//
// Each world step, before the engine step, in ascending boat id order:
//   actuators   the throttle and rudder states move toward the controls as a first-order lag,
//               a += (command - a) (1 - e^(-dt / actuator_time_constant)); 0 is instantaneous.
//   propeller   while its position is under the free surface of a fluid field (the field's
//               surface height there, or its box's top), it pushes along its direction at its
//               position with T = |throttle| T_max (1 - u_a / max_inflow_speed), clamped to
//               [0, |throttle| T_max]: T_max is max_thrust ahead and max_reverse_thrust astern,
//               and u_a is the inflow, the propeller's velocity through the water along the way
//               it pushes. An outboard (outboard_max_angle > 0) swings its direction about the
//               body's z axis by the rudder state times outboard_max_angle, pushing the stern
//               right to turn left.
//   rudder      while its position is under the free surface, the water flowing past it, the
//               water's velocity there (the surface's orbital velocity, a current) less the
//               rudder's point velocity v + w x (p - com), meets it at an angle of attack alpha
//               in the body's horizontal plane, the rudder deflected by the rudder state times
//               max_angle (+ its trailing edge to port). With q = 1/2 rho A V^2:
//                 lift  q C_L(alpha) normal to the flow, C_L = lift_slope alpha up to
//                       stall_angle; past it the blade is a flat plate, whose normal force
//                       cross_flow_drag sin(alpha) gives C_L = cross_flow_drag sin(2 alpha) / 2:
//                       the lift drops at the stall and is 0 at 90 degrees;
//                 drag  q (drag_coefficient + cross_flow_drag sin^2 alpha) along the flow.
//               Water from astern (backing) meets the rudder trailing edge first, and alpha is
//               taken from that edge: a rudder works reversed going astern.
// Both forces go through world::add_force_at at their points, so they torque the hull about its
// centre of mass. The propeller's slipstream over the rudder is not modelled: at low speed an
// outboard steers and a rudder does not. The hull body's own linear_damping and angular_damping
// (body_desc's 0.05 by default) still apply and slow it as a resistance the water does not
// make; a boat whose resistance should be the water's alone sets them to 0.
//
// A boat is part of the world's state: each boat's description's digest, its controls, its
// actuators and the last step's forces are in save() and state_hash() (a world without boats
// saves exactly as before), and restore() refuses a world whose boats differ.
//
// Boats work on every backend (the null backend's forces move nothing). Standard library only,
// like the rest of cvc/physics.

#ifndef __CVC_PHYSICS_BOAT_H__
#define __CVC_PHYSICS_BOAT_H__

#include <cstdint>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>

namespace cvc {
namespace physics {

struct propeller_desc {
  vec3 position{-2.0, 0.0, -0.4};     // body-local metres: where the thrust acts
  vec3 direction{1.0, 0.0, 0.0};      // body-local: the way it pushes at throttle > 0 (normalised)
  double max_thrust = 2000.0;         // N at full throttle ahead with no inflow (bollard pull)
  double max_reverse_thrust = 1000.0; // N at full throttle astern with no inflow
  double max_inflow_speed = 10.0;     // m/s of inflow where the thrust falls to 0; 0: no falloff
  double outboard_max_angle = 0.0;    // rad an outboard swings at full rudder; 0: a fixed shaft
};

struct rudder_desc {
  vec3 position{-2.1, 0.0, -0.4}; // body-local metres: the centre of pressure
  double area = 0.0;              // m^2, one side; 0: no rudder
  double lift_slope = 2.5;        // dC_L / d alpha, per radian (a blade of aspect ratio ~1.5)
  double stall_angle = 0.65;      // rad, in (0, pi/2): where the lift peaks (37 degrees)
  double drag_coefficient = 0.01; // C_D at alpha = 0
  double cross_flow_drag = 1.2;   // a flat plate's C_D across the flow: C_D grows as sin^2 alpha
  double max_angle = 0.6;         // rad, the deflection at full rudder (34 degrees), [0, pi/2]
};

struct boat_desc {
  // The hull: a dynamic body with its collision shape and, usually, its hull_mesh. Its position
  // and rotation are ignored: add_boat places it.
  body_desc hull;
  propeller_desc propeller;
  rudder_desc rudder;
  double actuator_time_constant = 0.5; // s, for the throttle and the rudder; 0: instantaneous
};

// A boat handle; never reused.
struct boat_id {
  std::uint32_t value = 0xffffffffu;
  bool valid() const { return value != 0xffffffffu; }
  friend bool operator==(boat_id a, boat_id b) { return a.value == b.value; }
  friend bool operator!=(boat_id a, boat_id b) { return a.value != b.value; }
  friend bool operator<(boat_id a, boat_id b) { return a.value < b.value; }
};

// What the helm asks for (world::set_boat_controls); the actuators move toward it.
struct boat_controls {
  double throttle = 0.0; // [-1, 1]: + ahead, - astern
  double rudder = 0.0;   // [-1, 1]: + turns left going ahead (rudder and outboard)
};

struct boat_state {
  body_id hull;
  body_state body;
  double speed = 0.0;    // m/s through the water along the hull's forward axis
  double sway = 0.0;     // m/s through the water along its left axis
  vec3 water_velocity;   // m/s, the water's at the hull's centre of mass (0 out of the water)
  double heading = 0.0;  // rad: the forward axis's direction in the world xy plane, from +x
                         // toward +y
  double yaw_rate = 0.0; // rad/s about world z, + turning left
  boat_controls command; // what the actuators move toward
  double throttle = 0.0; // the throttle actuator now, [-1, 1]
  double rudder = 0.0;   // the rudder actuator now, [-1, 1]
  // From the last step:
  bool propeller_wet = false;
  double inflow = 0.0; // m/s, u_a: the propeller's velocity through the water along its push
  double thrust = 0.0; // N along the propeller's direction (< 0 astern)
  vec3 thrust_force;   // N, world
  double outboard_angle = 0.0; // rad the outboard swung, + turning left
  bool rudder_wet = false;
  double rudder_angle = 0.0;    // rad, + trailing edge to port
  double angle_of_attack = 0.0; // rad, the flow's on the rudder
  double rudder_lift = 0.0;     // N, signed: + to the flow's left (to starboard going ahead)
  double rudder_drag = 0.0;     // N
  vec3 rudder_force;            // N, world: lift and drag
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BOAT_H__
