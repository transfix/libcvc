/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav's plant seam: the plain types a sim_world holds (cvc/nav/vehicle_plant.h has the
// interface). Kept apart so sim_world.h stays free of the core headers the interface needs.

#ifndef __CVC_NAV_PLANT_TYPES_H__
#define __CVC_NAV_PLANT_TYPES_H__

#include <cmath>
#include <cstdint>

namespace cvc {
namespace nav {

class vehicle_plant; // cvc/nav/vehicle_plant.h

// What integrates a sim_world's agents.
enum class plant_kind : std::uint8_t {
  kinematic, // the fused bicycle (no plant set): byte-identical to before the seam
  jolt,      // a cvc::physics world's vehicles
  dynamic,   // the dynamic single-track vehicle (cvc::physics::dyn_bike_plant), no physics world
};

// One agent as nav reads it back: the rear axle (the point nav's bicycle tracks, which has
// the least sideslip), the body's heading, and the signed forward speed at the rear axle.
struct plant_state {
  float x = 0.0f, y = 0.0f; // normalized
  float heading = 0.0f;     // rad, counter-clockwise from +x
  float speed = 0.0f;       // normalized / s along the heading (< 0 backing up)
  float steer = 0.0f;       // rad, the steering actuator's bicycle angle now
  // Measured, for the drive's detached modulators (doc 31 D9 H4); 0 from a plant that does not
  // know. tire_use is the most used tire's combined slip over its peak (1 = at the grip limit);
  // load_transfer is |left - right| / total wheel load (1 = one side's wheels unloaded).
  float tire_use = 0.0f;
  float load_transfer = 0.0f;
  float pitch = 0.0f, roll = 0.0f; // rad: nose up, and left side up
};

// How measured plant state modulates the drive (H4; vehicle_plant::modulation), all off at 0
// gain. Past their onsets, an
// agent's gamma (its speed damping) grows by tire_gain per unit of tire_use and by tip_gain
// per unit of load_transfer, so it slows near tire saturation and near tipping; its hard
// barriers' weight grows by tip_gain per unit of load_transfer too.
struct plant_modulation {
  float tire_onset = 0.7f, tire_gain = 0.0f;
  float tip_onset = 0.4f, tip_gain = 0.0f;
};

// What the drive respects in a plant's arm (doc 31 D14's plant-aware governors). A zero
// leaves nav's own value.
struct plant_limits {
  float wheelbase = 0.0f;  // normalized: the bicycle's L for this agent
  float max_steer = 0.0f;  // rad: the steering lock
  float steer_rate = 0.0f; // rad/s: the actuator's slew limit
  float a_brake = 0.0f;    // normalized / s^2: the deceleration the vehicle can achieve
  float latency = 0.0f;    // s: from a command to the vehicle's response, beyond the tick
  // The vehicle's own envelope, in place of veh_params' a_max, vmax and a_lat_max.
  float a_max = 0.0f; // normalized / s^2: the acceleration the drive plans with
  float vmax = 0.0f;  // normalized / s: the top speed (sim_world caps it at veh_params' vmax)
  float a_lat = 0.0f; // normalized / s^2: the lateral acceleration it corners at
  // The vehicle's size, in place of nav's footprint: half its width, and how far its body
  // reaches ahead of and behind the rear axle (normalized). With a width, the drive covers the
  // body with sim_world::kPlantDiscs footprint discs (veh_params' multi-disc footprint).
  float half_width = 0.0f;
  float front = 0.0f, rear = 0.0f;
};

// One agent's tick, compared against nav's kinematic bicycle (doc 31 D14's per-tick drift):
// from the plant's own state at the tick's start and the same command, the bicycle predicts
// where the agent should be; the plant says where it is.
struct drift_sample {
  float position = 0.0f;   // normalized: |plant - bicycle|
  float heading = 0.0f;    // rad, plant - bicycle, wrapped to (-pi, pi]
  float speed = 0.0f;      // normalized / s, plant - bicycle
  float a_lateral = 0.0f;  // normalized / s^2: the bicycle's |v^2 tan(delta) / L| this tick
  float stop_speed = 0.0f; // normalized / s: the stopping speed the tick's command respected
};

// One agent at the end of one tick as a physics plant measures it (vehicle_plant::sample), for
// nav_stats' plant ledger (nav_samplers::plant): its attitude, its speed, nav's commands and how
// much of the vehicle they used, its accelerations and how its wheels stand. SI and the physics
// world's frame (metres, radians, seconds), not nav's normalized frame. Where it overlaps
// cvc::physics::planar_sample (physics/plant_log.h) a field has its name and meaning, and the
// Jolt plant computes both alike: vx, vy, r, steer, grade, roll, steer_cmd, accel_cmd, a_lat,
// a_lon, contacts, vehicle_contact, wheels, wheels_down.
//
// `measured` says which of the fields a plant's model has: a plant without one (the dynamic
// single track has no left and right wheels, so no ltr) clears its bit, leaves the field at its
// default, and nav_stats reports the ledger items reduced from it as not applicable. The fields
// without a bit (velocities, steering, commands, accelerations, wheels) every plant measures.
struct plant_sample {
  enum : std::uint32_t {
    has_attitude = 1u << 0,        // grade, roll, up_z
    has_roll_rate = 1u << 1,       // roll_rate
    has_ltr = 1u << 2,             // ltr
    has_wheels_down = 1u << 3,     // wheels_down
    has_contacts = 1u << 4,        // contacts
    has_vehicle_contact = 1u << 5, // vehicle_contact
    has_all = (1u << 6) - 1u,
  };
  double vx = 0.0, vy = 0.0; // the centre of mass's velocity along the heading and to its left
  double r = 0.0;            // yaw rate (rad/s, + left)
  double steer = 0.0;        // the bicycle angle the vehicle has (rad, + left)
  double max_steer = 0.0;    // its steering lock (rad)
  double steer_cmd = 0.0;    // nav's command over the tick that ended here (rad)
  double accel_cmd = 0.0;    // m/s^2 along the heading, as nav sent it
  double grade = 0.0;        // sin of the body's pitch, + nose up
  double roll = 0.0;         // sin of the body's roll, + left side up
  double up_z = 1.0;         // the body's up axis's world z: 1 upright, 0 on its side, < 0 over
  double a_lat = 0.0;        // the centre of mass's acceleration over the tick, to the left
  double a_lon = 0.0;        // and along the heading (m/s^2)
  double roll_rate = 0.0;    // about the body's forward axis (rad/s, + left side rising)
  double ltr = 0.0;          // lateral load transfer: (left - right) / (left + right) of the
                             // wheels' loads, +-1 when one side carries it all (0 with none)
  int wheels = 0;            // the vehicle's wheels
  int wheels_down = 0;       // how many touch the ground
  int contacts = 0;          // contacts its body started with static geometry over the tick
  // The agent whose vehicle its body touched over the tick (begun or persisting; the deepest
  // contact's); -1 none, and at attach()
  int vehicle_contact = -1;
  std::uint32_t measured = has_all; // the has_ bits of the fields this plant measures
};

// nav's kinematic bicycle, integrated as the fused drive integrates it (drive.cpp's
// rollout_impl): nsub Euler steps of dt / nsub, the speed first (clamped to [sp_min, vmax]),
// then the heading, then the position along the new heading. The command is held through the
// tick, so this is the plant's kinematic reference (D14's adapter test and per-tick drift).
inline plant_state kinematic_step(plant_state s, float steer, float accel, float L, float dt,
                                  int nsub, float sp_min, float vmax) {
  const float h = dt / static_cast<float>(nsub);
  const float t = std::tan(steer);
  for (int k = 0; k < nsub; ++k) {
    s.speed = std::fmin(std::fmax(s.speed + h * accel, sp_min), vmax);
    s.heading = s.heading + h * (s.speed / L) * t;
    s.x = s.x + h * s.speed * std::cos(s.heading);
    s.y = s.y + h * s.speed * std::sin(s.heading);
  }
  s.steer = steer;
  return s;
}

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_PLANT_TYPES_H__
