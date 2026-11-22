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

#ifndef __CVC_ARIADNE_VEHICLE_DYNAMICS_H__
#define __CVC_ARIADNE_VEHICLE_DYNAMICS_H__

#include <array>
#include <cstdint>
#include <cvc/ariadne/vehicle_drive.h>
#include <cvc/physics/body.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <memory>

namespace cvc {
namespace ariadne {

// ----------------
// A vehicle on the dynamic plant, as the world runs it (ARIADNE_WORLDS.md section 18.5, OW5d)
// ----------------
// WorldVehicles runs a `dynamic` vehicle on dyn_bike_plant, the dynamic single-track model: every
// navigator tick (tick_steps physics steps) the navigator takes the tick's command (a steering
// angle within the fleet's lock, an acceleration held to the top speed) and the plant holds it
// over the tick, one model substep around each world step, its chassis box a body in the world
// put where each substep starts, moving as the model moves. A participant driving one sets that
// command after each step from its drive axes (holon::drive_command), as on the kinematic plant.
// Its vehicle is its archetype's preset with five numbers (vehicle_desc_of), so a client that
// has them, the navigator's hold and the clock ticks it ticks on builds the same model and runs it
// step for step as the relay does, where nothing else touches the vehicle: the one it drives is
// predicted. The relay's poses correct what it cannot know (a wall, another vehicle, a slope: a
// client's model runs on flat ground). dynamic_drive (vehicle_drive.h) holds the numbers.

// One of them as it runs, at the end of a physics step.
struct dynamic_vehicle {
  // The model as the plant keeps it: the rear axle (world m), the heading, the body twist, the
  // actuators, the driver, the command it holds.
  cvc::physics::dyn_bike_saved model;
  // The chassis box: its centre and yaw ([x, y, z, yaw]) and its velocity, as the world's step
  // leaves the body it puts where the substep starts. The height is the one it was put at (a
  // client has no ground under it).
  std::array<double, 4> box{};
  double vx = 0.0, vy = 0.0, wz = 0.0;
  // The command its next navigator tick takes, before the navigator's hold (rad, + left; m/s^2).
  double steer_cmd = 0.0, accel_cmd = 0.0;
};

// Runs one: a dyn_bike_plant of that vehicle alone, without a world (it touches nothing), one
// model substep each physics step.
class dynamic_vehicle_model {
public:
  // Throws std::invalid_argument for a drive that is not valid() or a preset that is not one.
  dynamic_vehicle_model(const dynamic_drive &drive, double fixed_dt);
  ~dynamic_vehicle_model();
  dynamic_vehicle_model(const dynamic_vehicle_model &) = delete;
  dynamic_vehicle_model &operator=(const dynamic_vehicle_model &) = delete;

  const dynamic_drive &drive() const { return _drive; }
  double fixed_dt() const { return _dt; }

  // The vehicle whose chassis is at CHASSIS (its centre, heading and velocity, as the pose
  // stream carries it) at the end of the step ending on clock tick TICK. The pose stream carries
  // no yaw rate, steering angle, drive lag or driver: they are LIKE's (the model as it was
  // predicted for that tick), else at rest. The command is LIKE's, else none.
  dynamic_vehicle at(const cvc::physics::body_state &chassis, std::uint64_t tick,
                     const dynamic_vehicle *like = nullptr) const;
  // One physics step ending on clock tick TICK: the navigator's tick first when it falls due (the
  // command held, then given to the plant), then the model's substep and the box a step along.
  void step(dynamic_vehicle &v, std::uint64_t tick);
  // Its speed along its heading (m/s).
  static double speed(const dynamic_vehicle &v) { return v.model.body.vx; }

  // The plant it runs on (its one vehicle, agent 0).
  const cvc::physics::dyn_bike_plant &plant() const { return *_plant; }

private:
  dynamic_drive _drive;
  double _dt = 0.0;
  std::unique_ptr<cvc::physics::dyn_bike_plant> _plant;
};

// Its chassis as a body: the box's centre, its yaw as a rotation about +z, its velocity and spin.
cvc::physics::body_state dynamic_vehicle_state(const dynamic_vehicle &v);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_VEHICLE_DYNAMICS_H__
