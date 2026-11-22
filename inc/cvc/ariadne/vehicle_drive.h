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

#ifndef __CVC_ARIADNE_VEHICLE_DRIVE_H__
#define __CVC_ARIADNE_VEHICLE_DRIVE_H__

#include <cstdint>
#include <string>

namespace cvc {
namespace ariadne {

// A vehicle on the dynamic plant, as a client runs it (ARIADNE_WORLDS.md section 18.5, OW5d;
// vehicle_dynamics.h runs it): what the client needs. room_entity::dynamics carries it, and
// WorldVehicles::dynamics fills it.
struct dynamic_drive {
  // The vehicle: its archetype's preset (humvee, roboracer) with its mass (kg), top speed (m/s),
  // acceleration and braking (m/s^2) and steering lock (rad) as the world gave them
  // (vehicle_desc_of with those as overrides makes the same vehicle_desc).
  std::string preset;
  double mass = 0.0, max_speed = 0.0, max_accel = 0.0, max_decel = 0.0, max_steer = 0.0;
  // The navigator's hold on a command (sim_world's, for a plant): the steering within `lock`
  // (rad: nav's and the fleet's vehicles' lock, the lower), the acceleration held to `vmax` (m/s)
  // within `a_hold` (m/s^2) per tick.
  double lock = 0.0, vmax = 0.0, a_hold = 0.0;
  // Physics steps per navigator tick, and the clock ticks the navigator ticks on: a step ending
  // on tick T runs it when T % tick_steps == phase.
  int tick_steps = 0;
  std::uint64_t phase = 0;

  // Whether it is one to run: a tick, a preset, a vehicle with a mass and a top speed.
  bool valid() const {
    return tick_steps > 0 && !preset.empty() && mass > 0.0 && vmax > 0.0 && max_steer > 0.0;
  }
  bool operator==(const dynamic_drive &) const = default;
};

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_VEHICLE_DRIVE_H__
