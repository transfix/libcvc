/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_ARIADNE_VEHICLE_KINEMATICS_H__
#define __CVC_ARIADNE_VEHICLE_KINEMATICS_H__

#include <array>
#include <cstdint>
#include <cvc/physics/body.h>

namespace cvc {
namespace ariadne {

// ----------------
// A vehicle on the kinematic plant, as the world runs it (ARIADNE_WORLDS.md section 18.5, OW5b)
// ----------------
// WorldVehicles runs a `kinematic` vehicle on nav's bicycle: every navigator tick (tick_steps
// physics steps) it takes the tick's command (a steering angle and an acceleration), holds the
// acceleration to the top speed, and integrates the bicycle over the tick in nsub substeps; the
// chassis box then moves a step at a time along the straight line from where the last tick left it
// to where this one does. A participant driving one sets that command after each step from its
// drive axes (holon::drive_command). Nothing but the command moves it (nav keeps a kinematic box
// off the buildings by its plan, which a driver's command replaces), so a client that knows these
// numbers and the clock ticks the navigator ticks on runs it step for step as the relay does: the
// vehicle it drives is predicted, not played out a round trip behind its keys. The relay's poses
// correct what it cannot know (another vehicle's separation pushing it, a restart).

// What a client needs to run one (room_entity::drive carries it; WorldVehicles::kinematics).
struct kinematic_drive {
  // nav's bicycle: the wheelbase (m), the steering lock (rad), the top speed (m/s; backing up, a
  // quarter of it when allow_reverse) and the acceleration the top speed holds it to (m/s^2).
  double wheelbase = 0.0;
  double lock = 0.0;
  double vmax = 0.0;
  double a_max = 0.0;
  bool allow_reverse = true;
  // The rear axle along the chassis from its centre (m, negative behind): the bicycle's point.
  double rear_x = 0.0;
  // Physics steps per navigator tick, the bicycle's substeps per tick, and the clock ticks the
  // navigator ticks on: a step ending on tick T runs it when T % tick_steps == phase.
  int tick_steps = 0;
  int nsub = 1;
  std::uint64_t phase = 0;
  // The vehicle's envelope, which a driver's drive axes are shares of (holon::drive_envelope).
  double max_steer = 0.6, max_accel = 3.0, max_decel = 7.0;

  // Whether it is one to run: a tick, a wheelbase, a top speed.
  bool valid() const { return tick_steps > 0 && wheelbase > 0.0 && vmax > 0.0 && nsub > 0; }
  bool operator==(const kinematic_drive &) const = default;
};

// One of them as it runs, at the end of a physics step.
struct kinematic_vehicle {
  // The bicycle: its rear axle (world m), heading (rad from +x) and speed (m/s along it).
  double rx = 0.0, ry = 0.0, heading = 0.0, speed = 0.0;
  // The chassis box: its centre and yaw at the last navigator tick and at the next ([x, y, z,
  // yaw]), and the steps of this tick taken; where it is now, and its velocity over the last step.
  std::array<double, 4> from{}, to{}, box{};
  int sub = 0;
  double vx = 0.0, vy = 0.0;
  // The height of its centre (m): the ground under it is the world's, which a client does not
  // have, so it keeps the height it was put at.
  double z = 0.0;
  // The command its next navigator tick takes (rad, + left; m/s^2 along the heading).
  double steer_cmd = 0.0, accel_cmd = 0.0;
};

// The vehicle whose chassis is at CHASSIS (its centre, heading and velocity, as the pose stream
// carries it) at the end of the step ending on clock tick TICK, each step FIXED_DT s. The pose
// stream carries no angular velocity, so the box is taken as moving straight along its velocity
// through the tick it is in, and the bicycle at the tick's end as the box's heading there and its
// speed along it: exact at rest and on a straight line, close otherwise. The command is none.
kinematic_vehicle kinematic_vehicle_at(const kinematic_drive &k,
                                       const cvc::physics::body_state &chassis, std::uint64_t tick,
                                       double fixed_dt);

// One physics step of FIXED_DT s, ending on clock tick TICK: the navigator's tick first when it
// falls due (the bicycle over the tick, on the vehicle's command), then the box a step along.
void kinematic_vehicle_step(kinematic_vehicle &v, const kinematic_drive &k, std::uint64_t tick,
                            double fixed_dt);

// Its chassis as a body: the box's centre, its yaw as a rotation about +z, its velocity.
cvc::physics::body_state kinematic_vehicle_state(const kinematic_vehicle &v);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_VEHICLE_KINEMATICS_H__
