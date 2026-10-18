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
};

// One agent as nav reads it back: the rear axle (the point nav's bicycle tracks, which has
// the least sideslip), the body's heading, and the signed forward speed at the rear axle.
struct plant_state {
  float x = 0.0f, y = 0.0f; // normalized
  float heading = 0.0f;     // rad, counter-clockwise from +x
  float speed = 0.0f;       // normalized / s along the heading (< 0 backing up)
  float steer = 0.0f;       // rad, the steering actuator's bicycle angle now
};

// What the drive respects in a plant's arm (doc 31 D14's plant-aware governors). A zero
// leaves nav's own value.
struct plant_limits {
  float wheelbase = 0.0f;  // normalized: the bicycle's L for this agent
  float max_steer = 0.0f;  // rad: the steering lock
  float steer_rate = 0.0f; // rad/s: the actuator's slew limit
  float a_brake = 0.0f;    // normalized / s^2: the deceleration the vehicle can achieve
  float latency = 0.0f;    // s: from a command to the vehicle's response, beyond the tick
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
