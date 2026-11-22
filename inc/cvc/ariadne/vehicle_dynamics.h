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
#include <cvc/ariadne/avatar_physics.h>
#include <cvc/ariadne/vehicle_drive.h>
#include <cvc/physics/body.h>
#include <cvc/physics/dyn_4w_plant.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/world.h>
#include <functional>
#include <memory>
#include <variant>
#include <vector>

namespace cvc {
namespace ariadne {

// ----------------
// A vehicle on a dynamic plant, as the world runs it (ARIADNE_WORLDS.md section 18.5, OW5d, OW5e)
// ----------------
// WorldVehicles runs a `dynamic` vehicle on dyn_bike_plant, the dynamic single-track model, and a
// `dynamic4w` one on dyn_4w_plant, four wheels on their suspension (dynamic_drive::plant): every
// navigator tick (tick_steps physics steps) the navigator takes the tick's command (a steering
// angle within the fleet's lock, an acceleration held to the top speed) and the plant holds it
// over the tick, one model substep around each world step, its chassis box a body in the world
// put where each substep starts, moving as the model moves. A participant driving one sets that
// command after each step from its drive axes (holon::drive_command), as on the kinematic plant.
// Its vehicle is its archetype's preset with five numbers (vehicle_desc_of), so a client that
// has them, the navigator's hold and the clock ticks it ticks on builds the same model and runs it
// step for step as the relay does, where nothing else touches the vehicle: the one it drives is
// predicted. The relay's poses correct what it cannot know (a wall, another vehicle). Given the
// world's ground (OW5g: the ground the relay's vehicles drive on, vehicle_ground_height of the
// same document's grid and terrain), it drives on the relay's slopes too; without it, on flat
// ground at the plant's reference height. dynamic_drive (vehicle_drive.h) holds the numbers.
//
// A `jolt` vehicle (OW5i) is a Jolt vehicle in the relay's world (nav_plant): the engine's wheels,
// suspension, powertrain and driver, the navigator's command given to world::drive. A client runs
// it in a Jolt world of its own made as the relay's is (its step, gravity and solver) with the
// relay's statics (the replica's: OW5c, each body on its surface, OW5h) and that vehicle alone,
// and keeps the whole of that world (world::save) as the vehicle's state, so a sample restores
// what was predicted for its tick and replays from there.

// A jolt vehicle as its model keeps it: its world, saved (world::save) at the end of the step.
struct jolt_vehicle_saved {
  std::shared_ptr<const std::vector<std::uint8_t>> world;
};

// The world a jolt vehicle's model runs it in: the relay's statics, as the replica has them, and
// its gravity. No statics: flat ground at the drive's height.
struct jolt_vehicle_world {
  std::vector<avatar_static> statics;
  cvc::physics::vec3 gravity{0.0, 0.0, -9.80665};
};

// One of them as it runs, at the end of a physics step.
struct dynamic_vehicle {
  // The model as its plant keeps it: the rear axle (world m), the heading, the body twist, the
  // actuators, the driver, the command it holds; on dynamic4w also the suspension's heave, roll
  // and pitch; on jolt its world.
  std::variant<cvc::physics::dyn_bike_saved, cvc::physics::dyn_4w_saved, jolt_vehicle_saved> model;
  // The chassis box: its centre and yaw ([x, y, z, yaw]) and its velocity, as the world's step
  // leaves the body it puts where the substep starts. On the dynamic plant the height is the one
  // it was put at (a client has no ground under it); on dynamic4w it is the model's ride height,
  // carried a step at its heave rate, and the box leans with the body's pitch and roll (rad, as
  // the model measures them: + lowers the nose, + lifts the left side; 0 on the dynamic plant).
  // On jolt the box is the chassis body (its origin, as the relay's pose stream has it), leaning
  // as it does.
  std::array<double, 4> box{};
  double pitch = 0.0, roll = 0.0;
  double vx = 0.0, vy = 0.0, wz = 0.0;
  // The command its next navigator tick takes, before the navigator's hold (rad, + left; m/s^2).
  double steer_cmd = 0.0, accel_cmd = 0.0;
};

// Runs one: its plant with that vehicle alone, without a world (it touches nothing), one model
// substep each physics step; a jolt vehicle in a world of its own (above).
class dynamic_vehicle_model {
public:
  // Throws std::invalid_argument for a drive that is not valid() or a preset that is not one.
  // GROUND (world metres at x, y) is the ground it drives on; empty: flat, at drive.ground. A
  // dynamic4w model measures its heights from drive.ground either way, as the relay's plant does.
  // WORLD: a jolt vehicle's statics and gravity (ignored on the other plants); null: flat ground at
  // drive.ground and standard gravity. Throws cvc::unsupported_exception for a jolt vehicle in a
  // build without Jolt.
  using ground_fn = std::function<double(double, double)>;
  dynamic_vehicle_model(const dynamic_drive &drive, double fixed_dt, ground_fn ground = {},
                        const jolt_vehicle_world *world = nullptr);
  ~dynamic_vehicle_model();
  dynamic_vehicle_model(const dynamic_vehicle_model &) = delete;
  dynamic_vehicle_model &operator=(const dynamic_vehicle_model &) = delete;

  const dynamic_drive &drive() const { return _drive; }
  double fixed_dt() const { return _dt; }
  // Whether it drives on a ground it was given (else flat); a jolt model, on statics.
  bool on_ground() const { return static_cast<bool>(_ground) || _jolt_statics; }

  // The vehicle whose chassis is at CHASSIS (its centre, heading and velocity, as the pose
  // stream carries it) at the end of the step ending on clock tick TICK. The pose stream carries
  // no yaw rate, steering angle, drive lag or driver, nor the suspension's state: they are LIKE's
  // (the model as it was predicted for that tick, on the same plant), else at rest (on dynamic4w,
  // its suspension settled on the ground under it). The command is LIKE's, else none. On jolt,
  // LIKE's world, its chassis put at CHASSIS's position, heading and horizontal velocity (its
  // pitch, roll, vertical speed and spin LIKE's); without one the vehicle as it was added, at rest.
  dynamic_vehicle at(const cvc::physics::body_state &chassis, std::uint64_t tick,
                     const dynamic_vehicle *like = nullptr) const;
  // One physics step ending on clock tick TICK: the navigator's tick first when it falls due (the
  // command held, then given to the plant), then the model's substep and the box a step along.
  void step(dynamic_vehicle &v, std::uint64_t tick);
  // Its speed along its heading (m/s).
  static double speed(const dynamic_vehicle &v);

  // The plant it runs on (its one vehicle, agent 0): the one its drive names, the others null.
  const cvc::physics::dyn_bike_plant *bike() const { return _bike.get(); }
  const cvc::physics::dyn_4w_plant *four() const { return _four.get(); }
  // A jolt vehicle's world (its statics and the vehicle), and the vehicle in it.
  const avatar_world *jolt() const { return _jolt.get(); }
  cvc::physics::vehicle_id jolt_vehicle() const { return _jveh; }

private:
  dynamic_vehicle at_bike(const cvc::physics::body_state &chassis,
                          const dynamic_vehicle *like) const;
  dynamic_vehicle at_four(const cvc::physics::body_state &chassis,
                          const dynamic_vehicle *like) const;
  dynamic_vehicle at_jolt(const cvc::physics::body_state &chassis,
                          const dynamic_vehicle *like) const;
  void step_bike(dynamic_vehicle &v, bool tick);
  void step_four(dynamic_vehicle &v, bool tick);
  void step_jolt(dynamic_vehicle &v, bool tick);
  // The world as V holds it (restored unless it is the one the world was last left at), and V
  // from the world's chassis after a step or a placement (its saved world, box, attitude, twist).
  void load_jolt(const dynamic_vehicle &v) const;
  void keep_jolt(dynamic_vehicle &v) const;
  // The chassis's forward speed at its rear axle (m/s), as nav_plant reads it.
  double jolt_speed() const;

  dynamic_drive _drive;
  double _dt = 0.0;
  ground_fn _ground;
  std::unique_ptr<cvc::physics::dyn_bike_plant> _bike;
  std::unique_ptr<cvc::physics::dyn_4w_plant> _four;
  // jolt: its world, the vehicle and its chassis, its description, the world as the vehicle was
  // added (at() without a like), and the saved world the world was last left at.
  std::unique_ptr<avatar_world> _jolt;
  bool _jolt_statics = false;
  cvc::physics::vehicle_id _jveh;
  cvc::physics::body_id _jchassis;
  cvc::physics::vehicle_desc _jdesc;
  double _jrear = 0.0;
  std::shared_ptr<const std::vector<std::uint8_t>> _jadded;
  mutable std::shared_ptr<const std::vector<std::uint8_t>> _jlast;
};

// Its chassis as a body: the box's centre, its attitude (the yaw about +z, then the pitch and
// roll), its velocity and spin.
cvc::physics::body_state dynamic_vehicle_state(const dynamic_vehicle &v);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_VEHICLE_DYNAMICS_H__
