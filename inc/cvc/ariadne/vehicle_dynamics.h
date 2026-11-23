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
#include <cvc/physics/dyn_boat_plant.h>
#include <cvc/physics/field.h>
#include <cvc/physics/world.h>
#include <functional>
#include <memory>
#include <optional>
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
//
// A `dynamicboat` boat (OW5j) is a boat on the relay's water (dyn_boat_plant: its hull a body the
// water's field floats and drags, its driver turning the navigator's command into a throttle and
// a rudder every step). A client runs it the same way, in a Jolt world of its own with the
// relay's statics and its water (the replica's: the water node's field, or the still water the
// relay adds for its boats, boat_still_water), and its state is that world and the driver's memory
// (dyn_boat_plant::memory), which the world's save does not hold.

// A jolt vehicle or a boat as its model keeps it: its world, saved (world::save) at the end of
// the step; a boat's driver memory too. A boat not yet on the water (no sample has put it there)
// has no world.
struct jolt_vehicle_saved {
  std::shared_ptr<const std::vector<std::uint8_t>> world;
  cvc::physics::dyn_boat_plant::memory boat;
};

// The world a jolt vehicle's or a boat's model runs it in: the relay's statics and water, as the
// replica has them, and its gravity. No statics: a jolt vehicle on flat ground at the drive's
// height. A boat needs the water. OBSTACLE_ROOM: a box for the room's vehicles of each of these
// half extents and masses (OW5o; dynamic_vehicle_model::hold_obstacles; their keys and poses do
// not count), made with the world so its saved states keep the same bodies.
struct vehicle_obstacle;
struct jolt_vehicle_world {
  std::vector<avatar_static> statics;
  std::vector<cvc::physics::field> water;
  cvc::physics::vec3 gravity{0.0, 0.0, -9.80665};
  std::vector<vehicle_obstacle> obstacle_room;
};

// One of the room's vehicles as a client shows it (holon::hold_entity), for a jolt or boat
// model's chassis to meet (OW5o): a box of HALF_EXTENTS about its centre, AT where it is at the
// end of the coming step (its centre and rotation) and its velocity. MASS 0: kinematic, a vehicle
// nothing pushes on the relay either (the kinematic plant's box). MASS > 0 (kg): a body of that
// mass, put where it is shown before each step, so the chassis shoves it as the relay's shoves
// that vehicle (a jolt vehicle, a dynamic plant's box) or avatar (OW5p). Its surface is
// frictionless: an avatar's box slides so on the relay (avatar_world's material), and a vehicle
// shoved along rolls on its wheels. WALK_FORCE (N, > 0 for an avatar's): its walker, the most it
// pushes the body with (avatar_energy_policy::force_max). Knocked loose, the box is pushed back
// toward the velocity it is shown at with up to that force each step, upright, as the relay's
// walker pushes the avatar toward its intent.
struct vehicle_obstacle {
  std::uint64_t key = 0;
  cvc::physics::vec3 half_extents;
  double mass = 0.0;
  double walk_force = 0.0;
  cvc::physics::body_state at;
};

// One of them as it runs, at the end of a physics step.
struct dynamic_vehicle {
  // The model as its plant keeps it: the rear axle (world m), the heading, the body twist, the
  // actuators, the driver, the command it holds; on dynamic4w also the suspension's heave, roll
  // and pitch; on jolt and dynamicboat its world.
  std::variant<cvc::physics::dyn_bike_saved, cvc::physics::dyn_4w_saved, jolt_vehicle_saved> model;
  // The chassis box: its centre and yaw ([x, y, z, yaw]) and its velocity, as the world's step
  // leaves the body it puts where the substep starts. On the dynamic plant the height is the one
  // it was put at (a client has no ground under it); on dynamic4w it is the model's ride height,
  // carried a step at its heave rate, and the box leans with the body's pitch and roll (rad, as
  // the model measures them: + lowers the nose, + lifts the left side; 0 on the dynamic plant).
  // On jolt the box is the chassis body (its origin, as the relay's pose stream has it), leaning
  // as it does; on dynamicboat the hull's.
  std::array<double, 4> box{};
  double pitch = 0.0, roll = 0.0;
  double vx = 0.0, vy = 0.0, wz = 0.0;
  // The command its next navigator tick takes, before the navigator's hold (rad, + left; m/s^2).
  double steer_cmd = 0.0, accel_cmd = 0.0;
};

// Runs one: its plant with that vehicle alone, without a world (it touches nothing), one model
// substep each physics step; a jolt vehicle or a boat in a world of its own (above).
class dynamic_vehicle_model {
public:
  // Throws std::invalid_argument for a drive that is not valid() or a preset that is not one.
  // GROUND (world metres at x, y) is the ground it drives on; empty: flat, at drive.ground. A
  // dynamic4w model measures its heights from drive.ground either way, as the relay's plant does.
  // WORLD: a jolt vehicle's or a boat's statics, water and gravity (ignored on the other plants);
  // null: flat ground at drive.ground and standard gravity. Throws cvc::unsupported_exception for a
  // jolt vehicle or a boat in a build without Jolt, and std::invalid_argument for a boat without
  // water.
  using ground_fn = std::function<double(double, double)>;
  dynamic_vehicle_model(const dynamic_drive &drive, double fixed_dt, ground_fn ground = {},
                        const jolt_vehicle_world *world = nullptr);
  ~dynamic_vehicle_model();
  dynamic_vehicle_model(const dynamic_vehicle_model &) = delete;
  dynamic_vehicle_model &operator=(const dynamic_vehicle_model &) = delete;

  const dynamic_drive &drive() const { return _drive; }
  double fixed_dt() const { return _dt; }
  // Whether it drives on a ground it was given (else flat); a jolt model or a boat's, on statics.
  bool on_ground() const { return static_cast<bool>(_ground) || _jolt_statics; }

  // The vehicle whose chassis is at CHASSIS (its centre, heading and velocity, as the pose
  // stream carries it) at the end of the step ending on clock tick TICK. The pose stream carries
  // no drive lag or driver: they are LIKE's (the model as it was predicted for that tick, on the
  // same plant), else at rest (on dynamic4w, its suspension settled on the ground under it). With
  // STEER (the sample carries its angular velocity and steering angle, pose_sample::steer, OW5n)
  // the dynamic plants take the steering angle from it and the yaw rate (and on dynamic4w the roll
  // and pitch rates) from CHASSIS's angular velocity; without, they are LIKE's, else none. On
  // dynamic4w the suspension's pitch, roll and heave are CHASSIS's when it carries a pitch or roll
  // (the relay's box, OW5m; its heave from the box's height), else LIKE's; its heave rate is
  // LIKE's (the pose's vertical speed is the box's after the relay's step under gravity). The
  // command is LIKE's, else none. On jolt,
  // LIKE's world, its chassis put at CHASSIS's position, heading and horizontal velocity, and at
  // its pitch, roll and vertical speed when CHASSIS carries them (the relay's chassis's attitude,
  // OW5m; a turn about z alone leaves them LIKE's), its spin LIKE's; without LIKE the vehicle as
  // it was added, at rest; its spin CHASSIS's angular velocity with STEER (its steering is its
  // wheels', which follow the command).
  // On dynamicboat the same with the hull, the driver's memory LIKE's. A boat goes on the water at
  // the first CHASSIS over water deep enough for it (dyn_boat_plant::attach); until then it has no
  // world, and its steps leave it where it is. Its world's field time is TICK x fixed_dt, the
  // relay's at that tick (SceneWorld::follow anchors it to the clock), and a step adds fixed_dt,
  // so the boat rides the same waves as the relay's (OW5l).
  dynamic_vehicle at(const cvc::physics::body_state &chassis, std::uint64_t tick,
                     const dynamic_vehicle *like = nullptr,
                     std::optional<double> steer = std::nullopt) const;
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
  // A boat's world (jolt(): its statics and water) and its plant, once it is on the water (at()).
  bool is_boat() const { return _drive.plant == "dynamicboat"; }
  const cvc::physics::dyn_boat_plant *boat() const { return _boat.get(); }

  // THE ROOM'S VEHICLES (OW5o), on a jolt or boat model: its world holds a kinematic box for each
  // of jolt_vehicle_world::obstacle_room's sizes, out of the way until held. Before V's next step,
  // the boxes for OBSTACLES go where each is to be at that step's end (one keeps its box while it
  // is held; a box none of them names goes back out of the way), so the chassis meets them as the
  // relay's meets those vehicles. False, changing nothing, when they do not fit: a size with no
  // free box. The host then builds the model again with room for them. True and nothing on the
  // dynamic plants (they meet nothing, as on the relay) and for a boat not yet on the water.
  bool hold_obstacles(dynamic_vehicle &v, const std::vector<vehicle_obstacle> &obstacles);
  // Where the box held for KEY is in the model's world as the last step left it (OW5p); false
  // for a key no box holds.
  bool obstacle_state(std::uint64_t key, cvc::physics::body_state &out) const;
  // The boxes it has room for, and how many hold one of the room's vehicles now.
  std::size_t obstacle_room() const { return _boxes.size(); }
  std::size_t obstacles_held() const;

private:
  dynamic_vehicle at_bike(const cvc::physics::body_state &chassis, const dynamic_vehicle *like,
                          std::optional<double> steer) const;
  dynamic_vehicle at_four(const cvc::physics::body_state &chassis, const dynamic_vehicle *like,
                          std::optional<double> steer) const;
  dynamic_vehicle at_jolt(const cvc::physics::body_state &chassis, std::uint64_t tick,
                          const dynamic_vehicle *like, std::optional<double> steer) const;
  void step_bike(dynamic_vehicle &v, bool tick);
  void step_four(dynamic_vehicle &v, bool tick);
  void step_jolt(dynamic_vehicle &v, bool tick);
  void step_boat(dynamic_vehicle &v, bool tick);
  // A boat's plant with its boat on the water at CHASSIS (attach), unless it has one; false where
  // there is no water deep enough for it.
  bool ensure_boat(const cvc::physics::body_state &chassis) const;
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
  // added (at() without a like), and the saved world the world was last left at. dynamicboat: the
  // same world with the water, the boat's description and plant, and its hull as the chassis.
  std::unique_ptr<avatar_world> _jolt;
  bool _jolt_statics = false;
  cvc::physics::vehicle_id _jveh;
  mutable cvc::physics::body_id _jchassis;
  cvc::physics::dyn_boat_desc _bdesc;
  mutable std::unique_ptr<cvc::physics::dyn_boat_plant> _boat;
  cvc::physics::vehicle_desc _jdesc;
  double _jrear = 0.0;
  mutable double _jreach = 5.0; // m: the chassis's reach in plan from its centre (OW5p)
  mutable std::shared_ptr<const std::vector<std::uint8_t>> _jadded;
  mutable std::shared_ptr<const std::vector<std::uint8_t>> _jlast;
  // The room's vehicles' boxes (hold_obstacles): the body, its half extents, and the vehicle it
  // holds (KEY, when HELD).
  struct obstacle_box {
    cvc::physics::body_id id;
    cvc::physics::vec3 half;
    double mass = 0.0;
    std::uint64_t key = 0;
    bool held = false;
    int loose = 0; // steps a box with a mass has been left where the chassis knocked it (OW5p)
  };
  std::vector<obstacle_box> _boxes;
  void add_obstacle_room(const jolt_vehicle_world *world);
};

// Its chassis as a body: the box's centre, its attitude (the yaw about +z, then the pitch and
// roll), its velocity and spin.
cvc::physics::body_state dynamic_vehicle_state(const dynamic_vehicle &v);

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_VEHICLE_DYNAMICS_H__
