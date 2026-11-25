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

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/vehicle_dynamics.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <optional>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace ph = cvc::physics;

namespace {
// The steps of the navigator's tick taken by the end of the step ending on TICK, less one: 0 on
// the step the navigator ticks on (as vehicle_kinematics.cpp's).
int into_tick(const dynamic_drive &d, std::uint64_t tick) {
  const std::uint64_t n = static_cast<std::uint64_t>(d.tick_steps);
  return static_cast<int>((tick % n + n - d.phase % n) % n);
}

double yaw_of(const ph::quat &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

ph::quat mul(const ph::quat &a, const ph::quat &b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

// Rz(psi) Ry(th) Rx(phi), as the dynamic models measure the body's attitude.
ph::quat attitude(double psi, double th, double phi) {
  const ph::quat z{std::cos(0.5 * psi), 0.0, 0.0, std::sin(0.5 * psi)};
  const ph::quat y{std::cos(0.5 * th), 0.0, std::sin(0.5 * th), 0.0};
  const ph::quat x{std::cos(0.5 * phi), std::sin(0.5 * phi), 0.0, 0.0};
  return mul(mul(z, y), x);
}

ph::vec3 rotate(const ph::quat &q, const ph::vec3 &v) {
  const ph::vec3 u{q.x, q.y, q.z};
  const ph::vec3 t{2.0 * (u.y * v.z - u.z * v.y), 2.0 * (u.z * v.x - u.x * v.z),
                   2.0 * (u.x * v.y - u.y * v.x)};
  return {v.x + q.w * t.x + (u.y * t.z - u.z * t.y), v.y + q.w * t.y + (u.z * t.x - u.x * t.z),
          v.z + q.w * t.z + (u.x * t.y - u.y * t.x)};
}

// The navigator's tick (sim_world's override_command on a plant, in its floats): the steering
// within the lock, the acceleration held to the top speed within a tick, at speed `vx`.
void held(const dynamic_drive &d, double dt, double steer_cmd, double accel_cmd, double vx,
          float &steer, float &accel) {
  const float tick = static_cast<float>(static_cast<double>(d.tick_steps) * dt);
  const float lock = static_cast<float>(d.lock);
  steer = std::min(std::max(static_cast<float>(steer_cmd), -lock), lock);
  const float speed = static_cast<float>(vx);
  const float cap = static_cast<float>(d.vmax), a_b = static_cast<float>(d.a_hold);
  accel = static_cast<float>(accel_cmd);
  if (speed >= 0.0f)
    accel = std::min(accel, std::max((cap - speed) / tick, -a_b));
  else
    accel = std::max(accel, std::min((-cap - speed) / tick, a_b));
}
} // namespace

dynamic_vehicle_model::dynamic_vehicle_model(const dynamic_drive &drive, double fixed_dt,
                                             ground_fn ground, const jolt_vehicle_world *world)
    : _drive(drive), _dt(fixed_dt), _ground(std::move(ground)) {
  if (!drive.valid())
    throw std::invalid_argument("dynamic_vehicle_model: not a dynamic drive to run");
  if (!(fixed_dt > 0.0) || !std::isfinite(fixed_dt))
    throw std::invalid_argument("dynamic_vehicle_model: the step must be finite and > 0");
  if (drive.plant == "dynamicboat") {
    // A world as the relay's (above), with its statics and its water; the boat goes on the water
    // at the first sample over it (ensure_boat).
    if (!world || world->water.empty())
      throw std::invalid_argument("dynamic_vehicle_model: a boat needs the world's water");
    vehicle_archetype a;
    a.id = "driven";
    a.plant = drive.plant;
    a.preset = drive.preset;
    _bdesc = boat_desc_of(a, drive.mass, drive.max_speed); // WorldVehicles' boat for its spawn
    ph::world_config cfg;
    cfg.fixed_dt = fixed_dt;
    cfg.contacts = ph::contact_reporting::none;
    cfg.gravity = world->gravity;
    _jolt = std::make_unique<avatar_world>(cfg, avatar_body{});
    if (!world->statics.empty()) {
      _jolt->set_statics(world->statics);
      _jolt_statics = true;
    }
    for (const ph::field &f : world->water)
      _water_ids.push_back(_jolt->world().add_field(f));
    _waters.push_back({0, world->water});
    _water_serial.push_back(_water_next++);
    for (const timed_water &w : world->water_after) // what the water took on since (OW5q)
      set_water(w.water, w.from_tick);
    add_obstacle_room(world);
    return;
  }
  // The world's vehicle: its preset with the numbers it was given (vehicle_desc_of).
  vehicle_archetype a;
  a.id = "driven";
  a.plant = drive.plant;
  a.preset = drive.preset;
  a.overrides = {{"mass", drive.mass},
                 {"max_speed", drive.max_speed},
                 {"max_accel", drive.max_accel},
                 {"max_decel", drive.max_decel},
                 {"max_steer", drive.max_steer}};
  const ph::vehicle_desc desc = vehicle_desc_of(a);
  cvc::world_units::normalized_frame si; // metres: the commands go in SI
  si.scale = 1.0;
  const cvc::nav::plant_state start{};
  if (drive.plant == "jolt") {
    // A world as the relay's scene world makes one (SceneWorld::build: the defaults, its step and
    // gravity), its statics as the replica has them, and the vehicle: nav_plant adds it to the
    // world, and the world runs it.
    ph::world_config cfg;
    cfg.fixed_dt = fixed_dt;
    cfg.contacts = ph::contact_reporting::none;
    if (world)
      cfg.gravity = world->gravity;
    _jolt = std::make_unique<avatar_world>(cfg, avatar_body{});
    if (world && !world->statics.empty()) {
      _jolt->set_statics(world->statics);
      _jolt_statics = true;
    } else {
      // Flat ground at the drive's height.
      ph::body_desc g;
      g.motion = ph::motion_type::static_;
      g.shape = ph::box{{1000.0, 1000.0, 0.5}};
      g.position = {0.0, 0.0, drive.ground - 0.5};
      _jolt->world().add_body(g);
    }
    add_obstacle_room(world);
    _jdesc = desc;
    _jreach = std::hypot(desc.half_extents.x, desc.half_extents.y);
    for (const ph::axle_desc &ax : desc.axles) // the rearmost unsteered axle (nav_plant's)
      if (!ax.steered && (_jrear == 0.0 || ax.x < _jrear))
        _jrear = ax.x;
    double h = 0.0; // just above the wheels' full reach, as nav_plant adds one
    for (const ph::axle_desc &ax : desc.axles)
      h = std::max(h, -ax.z + ax.suspension_max + ax.wheel_radius);
    _jveh = _jolt->world().add_vehicle(desc, {0.0, 0.0, drive.ground + h + 0.03}, ph::quat{});
    _jchassis = _jolt->world().vehicle_state(_jveh).chassis;
    _jadded = std::make_shared<const std::vector<std::uint8_t>>(_jolt->world().save());
    _jlast = _jadded;
    return;
  }
  // WorldVehicles' plant options: one model substep a physics step. With the statics or room for
  // the room's vehicles (OW5p), the chassis box in a world of its own holding them, as the relay's
  // plant has it in its world (the plant steps it, one world step a substep), the ground not hit.
  if (world && (!world->statics.empty() || !world->obstacle_room.empty())) {
    ph::world_config cfg;
    cfg.fixed_dt = fixed_dt;
    cfg.gravity = world->gravity;
    _dworld = std::make_unique<avatar_world>(cfg, avatar_body{});
    if (!world->statics.empty())
      _dworld->set_statics(world->statics);
    add_obstacle_room(world);
  }
  if (drive.plant == "dynamic4w") {
    ph::dyn_4w_plant_options o;
    o.substep = fixed_dt;
    if (_dworld) {
      o.physics = &_dworld->world();
      o.ground_bodies = _dworld->ground_bodies();
    }
    o.plant.ground_z = drive.ground; // flat, at the relay's reference height, without a ground
    if (_ground)
      o.plant.ground_height = _ground;
    o.reference = drive.ground; // its heights from the relay's datum
    _four = std::make_unique<ph::dyn_4w_plant>(std::vector<ph::vehicle_desc>{desc}, o);
    _four->attach(1, &start, si, fixed_dt);
  } else {
    ph::dyn_bike_plant_options o;
    o.substep = fixed_dt;
    if (_ground)
      o.plant.ground_height = _ground;
    if (_dworld) {
      o.physics = &_dworld->world();
      o.ground_bodies = _dworld->ground_bodies();
    }
    _bike = std::make_unique<ph::dyn_bike_plant>(std::vector<ph::vehicle_desc>{desc}, o);
    _bike->attach(1, &start, si, fixed_dt);
  }
  if (_dworld)
    _dlast = std::make_shared<const std::vector<std::uint8_t>>(_dworld->world().save());
}

ph::world *dynamic_vehicle_model::room_world() const {
  if (_jolt)
    return &_jolt->world();
  if (_dworld)
    return &_dworld->world();
  return nullptr;
}

void dynamic_vehicle_model::load_dworld(const dynamic_vehicle &v) const {
  if (!_dworld || !v.world || v.world == _dlast)
    return;
  try {
    _dworld->world().restore(*v.world);
    _dlast = v.world;
  } catch (const std::exception &) {
    // Another model's world (the host made this one again): this one's as it is.
  }
}

void dynamic_vehicle_model::keep_dworld(dynamic_vehicle &v) const {
  if (!_dworld)
    return;
  _dlast = std::make_shared<const std::vector<std::uint8_t>>(_dworld->world().save());
  v.world = _dlast;
}

dynamic_vehicle_model::~dynamic_vehicle_model() {
  // A dynamic plant's chassis box is in _dworld (OW5p): the plant takes it out, so it goes first.
  _bike.reset();
  _four.reset();
}

double dynamic_vehicle_model::speed(const dynamic_vehicle &v) {
  if (std::holds_alternative<ph::dyn_bike_saved>(v.model))
    return std::get<ph::dyn_bike_saved>(v.model).body.vx;
  if (std::holds_alternative<ph::dyn_4w_saved>(v.model))
    return std::get<ph::dyn_4w_saved>(v.model).body.vx;
  return v.vx * std::cos(v.box[3]) + v.vy * std::sin(v.box[3]); // the chassis's, along its yaw
}

dynamic_vehicle dynamic_vehicle_model::at(const ph::body_state &chassis, std::uint64_t tick,
                                          const dynamic_vehicle *like,
                                          std::optional<double> steer) const {
  if (is_boat() && !ensure_boat(chassis)) {
    // Not on the water yet: where the sample has it, without a world, until one puts it there.
    dynamic_vehicle v;
    v.model = jolt_vehicle_saved{};
    v.box = {chassis.position.x, chassis.position.y, chassis.position.z, yaw_of(chassis.rotation)};
    return v;
  }
  if (_jolt)
    return at_jolt(chassis, tick, like, steer);
  return _four ? at_four(chassis, like, steer) : at_bike(chassis, like, steer);
}

void dynamic_vehicle_model::load_jolt(const dynamic_vehicle &v) const {
  const jolt_vehicle_saved *s = std::get_if<jolt_vehicle_saved>(&v.model);
  if (!s || !s->world)
    throw std::invalid_argument("dynamic_vehicle_model: not a jolt vehicle's state");
  if (s->world != _jlast) {
    water_to(s->water); // the sea it was saved on: a world restores onto no other (OW5q)
    _jolt->world().restore(*s->world);
    _jlast = s->world;
    if (_boat)
      _boat->resume(0, s->boat); // its driver as it was then
  }
}

bool dynamic_vehicle_model::ensure_boat(const ph::body_state &chassis) const {
  if (_boat)
    return true;
  // The relay's boat fleet's plant (WorldVehicles: the world steps it, its driver on), here
  // stepping its own world one step a tick: the driver acts on each world step either way.
  ph::dyn_boat_plant_options o;
  o.physics = &_jolt->world();
  auto plant = std::make_unique<ph::dyn_boat_plant>(std::vector<ph::dyn_boat_desc>{_bdesc}, o);
  cvc::world_units::normalized_frame si; // metres: the commands go in SI
  si.scale = 1.0;
  cvc::nav::plant_state start;
  start.x = static_cast<float>(chassis.position.x);
  start.y = static_cast<float>(chassis.position.y);
  start.heading = static_cast<float>(yaw_of(chassis.rotation));
  try {
    plant->attach(1, &start, si, _dt);
  } catch (const std::exception &) {
    return false; // no water there deep enough for it
  }
  _boat = std::move(plant);
  _jchassis = _boat->hull(0);
  if (_bdesc.length_overall > 0.0)
    _jreach = 0.5 * _bdesc.length_overall;
  _jadded = std::make_shared<const std::vector<std::uint8_t>>(_jolt->world().save());
  _jadded_water = _water_on;
  _jlast = _jadded;
  return true;
}

namespace {
// Where a box no vehicle holds waits: far under the world, each apart (kinematic boxes meet
// nothing that does not move, and nothing moves there).
ph::vec3 parked_at(std::size_t i) { return {60.0 * static_cast<double>(i), 0.0, -10000.0}; }
// A box of the pool fits an obstacle of the same size and mass.
template <class Box> bool fits(const Box &box, const vehicle_obstacle &o) {
  const ph::vec3 &a = box.half, &b = o.half_extents;
  return std::fabs(a.x - b.x) < 1e-6 && std::fabs(a.y - b.y) < 1e-6 &&
         std::fabs(a.z - b.z) < 1e-6 && std::fabs(box.mass - o.mass) < 1e-6;
}
} // namespace

void dynamic_vehicle_model::add_obstacle_room(const jolt_vehicle_world *world) {
  if (!world)
    return;
  // A box with a mass slides (OW5p): frictionless, as avatar_world makes an avatar's; made only
  // when one needs it, so a world without one keeps the materials it had.
  std::optional<ph::material_id> slick;
  for (const vehicle_obstacle &o : world->obstacle_room) {
    const ph::vec3 &half = o.half_extents;
    if (!(half.x > 0.0 && half.y > 0.0 && half.z > 0.0) || !std::isfinite(half.x + half.y + half.z))
      throw std::invalid_argument("dynamic_vehicle_model: an obstacle's half extents must be > 0");
    if (!(o.mass >= 0.0) || !std::isfinite(o.mass))
      throw std::invalid_argument("dynamic_vehicle_model: an obstacle's mass must be >= 0");
    ph::body_desc b;
    b.motion = o.mass > 0.0 ? ph::motion_type::dynamic : ph::motion_type::kinematic;
    b.mass = o.mass;
    b.shape = ph::box{half};
    b.position = parked_at(_boxes.size());
    if (o.mass > 0.0) {
      if (!slick) {
        ph::material m;
        m.name = "avatar";
        m.friction = 0.0f;
        m.restitution = 0.0f;
        slick = room_world()->add_material(m);
      }
      b.material = *slick;
    }
    obstacle_box box;
    box.id = room_world()->add_body(b);
    box.half = half;
    box.mass = o.mass;
    _boxes.push_back(box);
  }
}

bool dynamic_vehicle_model::obstacle_state(std::uint64_t key, ph::body_state &out) const {
  for (const obstacle_box &b : _boxes)
    if (b.held && b.key == key) {
      out = room_world()->state(b.id);
      return true;
    }
  return false;
}

std::size_t dynamic_vehicle_model::obstacles_held() const {
  std::size_t n = 0;
  for (const obstacle_box &b : _boxes)
    n += b.held ? 1 : 0;
  return n;
}

bool dynamic_vehicle_model::hold_obstacles(dynamic_vehicle &v,
                                           const std::vector<vehicle_obstacle> &obstacles) {
  if (!room_world())
    return obstacles.empty(); // a dynamic plant's model without a world: no room for any
  if (_jolt) {
    const jolt_vehicle_saved *s = std::get_if<jolt_vehicle_saved>(&v.model);
    if (!s || !s->world)
      return true; // a boat not on the water yet: no world to hold them in
  }
  // Each keeps the box it holds; the rest take a free box of their size.
  std::vector<int> slot(obstacles.size(), -1);
  std::vector<bool> taken(_boxes.size(), false);
  for (std::size_t i = 0; i < obstacles.size(); ++i)
    for (std::size_t j = 0; j < _boxes.size(); ++j)
      if (!taken[j] && _boxes[j].held && _boxes[j].key == obstacles[i].key &&
          fits(_boxes[j], obstacles[i])) {
        slot[i] = static_cast<int>(j);
        taken[j] = true;
        break;
      }
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    if (slot[i] >= 0)
      continue;
    for (std::size_t j = 0; j < _boxes.size(); ++j)
      if (!taken[j] && fits(_boxes[j], obstacles[i])) {
        slot[i] = static_cast<int>(j);
        taken[j] = true;
        break;
      }
    if (slot[i] < 0)
      return false; // no room: the host makes the model again with room for it
  }
  if (_jolt)
    load_jolt(v);
  else
    load_dworld(v);
  ph::world &w = *room_world();
  const ph::body_state chassis = w.state(_jolt   ? _jchassis
                                         : _four ? _four->chassis(0)
                                                 : _bike->chassis(0));
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    obstacle_box &b = _boxes[static_cast<std::size_t>(slot[i])];
    const ph::body_state &at = obstacles[i].at;
    const ph::body_state now = w.state(b.id);
    const ph::vec3 d = now.position - at.position;
    // Moving with it: driven to where it is at the step's end. Anywhere else (a box newly held, or
    // one a replay's saved world had elsewhere): put there, a step back along its velocity, so it
    // does not sweep through the world to get there. A box with a mass is put where it is shown
    // before every step, moving as it does: the step shoves it, and the next puts it where the
    // relay's shove left it. But one the last step knocked off where it was shown (OW5p) is left
    // to the world from there until where it is shown catches up with it: it is shown a playout's
    // delay behind the relay, so until then it is shown where it stood before the knock, and put
    // back there it would be knocked again every step. Clear of the chassis, it is put where it is
    // shown again after a second whatever; within reach of it (still being pushed), never.
    if (b.mass > 0.0) {
      constexpr double kKnocked = 0.1;  // m off where it was shown
      constexpr double kKnockedV = 0.5; // m/s off how it was shown moving
      const int most_loose = static_cast<int>(std::lround(1.0 / _dt));
      const ph::vec3 put_at = at.position - at.linear_velocity * _dt;
      const ph::vec3 off = now.position - put_at;
      const ph::vec3 dv = now.linear_velocity - at.linear_velocity;
      const ph::vec3 to = now.position - chassis.position;
      const double reach = _jreach + std::hypot(b.half.x, b.half.y) + 0.5;
      const bool in_reach = to.x * to.x + to.y * to.y < reach * reach;
      const bool knocked =
          b.held && b.key == obstacles[i].key && (in_reach || b.loose < most_loose) &&
          (ph::dot(off, off) > kKnocked * kKnocked || ph::dot(dv, dv) > kKnockedV * kKnockedV);
      if (knocked) {
        ++b.loose;
        const double walk = obstacles[i].walk_force;
        if (walk > 0.0) {
          // Its walker (avatar_world::drive_walker): toward the velocity it is shown at, in plan,
          // with at most WALK_FORCE; upright, as the walker keeps it.
          ph::body_state s = now;
          double dx = at.linear_velocity.x - s.linear_velocity.x;
          double dy = at.linear_velocity.y - s.linear_velocity.y;
          const double dv = std::sqrt(dx * dx + dy * dy), cap = walk / b.mass * _dt;
          if (dv > cap && dv > 0.0)
            dx *= cap / dv, dy *= cap / dv;
          s.linear_velocity.x += dx;
          s.linear_velocity.y += dy;
          s.rotation = at.rotation;
          s.angular_velocity = ph::vec3{0.0, 0.0, 0.0};
          w.set_state(b.id, s);
        }
      } else {
        b.loose = 0;
        ph::body_state put;
        put.position = put_at;
        put.rotation = at.rotation;
        put.linear_velocity = at.linear_velocity;
        put.angular_velocity = at.angular_velocity;
        put.active = true;
        w.set_state(b.id, put);
      }
    } else if (b.held && b.key == obstacles[i].key && ph::dot(d, d) < 4.0) {
      w.move_kinematic(b.id, at.position, at.rotation);
    } else {
      ph::body_state put;
      put.position = at.position - at.linear_velocity * _dt;
      put.rotation = at.rotation;
      put.linear_velocity = at.linear_velocity;
      put.active = true;
      w.set_state(b.id, put);
      w.move_kinematic(b.id, at.position, at.rotation);
    }
    b.key = obstacles[i].key;
    b.held = true;
  }
  for (std::size_t j = 0; j < _boxes.size(); ++j) {
    obstacle_box &b = _boxes[j];
    if (taken[j])
      continue;
    const ph::body_state now = w.state(b.id);
    const ph::vec3 home = parked_at(j);
    const ph::vec3 d = now.position - home;
    if (b.held || ph::dot(d, d) > 1.0) { // back out of the way
      ph::body_state put;
      put.position = home;
      put.active = true;
      w.set_state(b.id, put);
    }
    b.held = false;
    b.loose = 0;
  }
  return true;
}

void dynamic_vehicle_model::keep_jolt(dynamic_vehicle &v) const {
  ph::world &w = _jolt->world();
  const std::shared_ptr<const std::vector<std::uint8_t>> saved =
      std::make_shared<const std::vector<std::uint8_t>>(w.save());
  _jlast = saved;
  v.model = jolt_vehicle_saved{saved, _boat ? _boat->remember(0) : ph::dyn_boat_plant::memory{},
                               _water_on};
  const ph::body_state s = w.state(_jchassis);
  const ph::quat &q = s.rotation;
  v.box = {s.position.x, s.position.y, s.position.z, yaw_of(q)};
  v.pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
  v.roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  v.vx = s.linear_velocity.x;
  v.vy = s.linear_velocity.y;
  v.wz = s.angular_velocity.z;
}

double dynamic_vehicle_model::jolt_speed() const {
  // nav_plant::state's: the centre of mass's velocity carried to the rear axle, along the body.
  const ph::body_state b = _jolt->world().state(_jchassis);
  const ph::vec3 fwd = rotate(b.rotation, {1.0, 0.0, 0.0});
  const ph::vec3 ra = rotate(b.rotation, {_jrear, 0.0, 0.0});
  const ph::vec3 cm = rotate(b.rotation, _jdesc.center_of_mass);
  const ph::vec3 r{ra.x - cm.x, ra.y - cm.y, ra.z - cm.z};
  const ph::vec3 &w = b.angular_velocity;
  const ph::vec3 v{b.linear_velocity.x + (w.y * r.z - w.z * r.y),
                   b.linear_velocity.y + (w.z * r.x - w.x * r.z),
                   b.linear_velocity.z + (w.x * r.y - w.y * r.x)};
  return v.x * fwd.x + v.y * fwd.y + v.z * fwd.z;
}

dynamic_vehicle dynamic_vehicle_model::at_jolt(const ph::body_state &chassis, std::uint64_t tick,
                                               const dynamic_vehicle *like,
                                               std::optional<double> steer) const {
  dynamic_vehicle v;
  ph::world &w = _jolt->world();
  bool was = false;
  const jolt_vehicle_saved *ls = like ? std::get_if<jolt_vehicle_saved>(&like->model) : nullptr;
  if (ls && ls->world) {
    try {
      load_jolt(*like); // its world as predicted for the tick: the wheels, the powertrain, ...
      v.steer_cmd = like->steer_cmd;
      v.accel_cmd = like->accel_cmd;
      was = true;
    } catch (const std::exception &) {
      // Another model's world (other statics): from the vehicle at rest instead.
    }
  }
  if (!was) {
    water_to(_jadded_water);
    w.restore(*_jadded); // the vehicle as it was added, at rest, its command none
    _jlast = _jadded;
    if (_boat)
      _boat->resume(0, {});
  }
  // The relay's field time at the tick (SceneWorld::follow anchors it to the clock): a boat reads
  // its water's waves there, as the relay's does, and each step adds fixed_dt (OW5l).
  w.set_field_time(static_cast<double>(tick) * _dt);
  // The chassis where the sample has it: its position, heading and horizontal velocity, and its
  // pitch, roll and vertical speed when the sample carries them (the relay's chassis's, OW5m); a
  // sample with a heading alone (a turn about z, an older relay's) leaves those the predicted ones.
  // The spin is the sample's when it carries it (STEER, OW5n), else the predicted one.
  ph::body_state s = w.state(_jchassis);
  double pitch = 0.0, roll = 0.0;
  if (was) {
    const ph::quat &q = s.rotation;
    pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x))));
    roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
  } else {
    s.linear_velocity = {};
    s.angular_velocity = {};
  }
  s.position = chassis.position;
  const ph::quat &q = chassis.rotation;
  const double qn = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if ((q.x != 0.0 || q.y != 0.0) && qn > 0.0) {
    s.rotation = {q.w / qn, q.x / qn, q.y / qn, q.z / qn};
    s.linear_velocity.z = chassis.linear_velocity.z;
  } else {
    s.rotation = attitude(yaw_of(q), pitch, roll);
  }
  s.linear_velocity.x = chassis.linear_velocity.x;
  s.linear_velocity.y = chassis.linear_velocity.y;
  if (steer)
    s.angular_velocity = chassis.angular_velocity;
  w.set_state(_jchassis, s);
  // A jolt vehicle's wheels at the relay's steering angle (OW5r), the actuator's: a client that
  // joins mid-turn turns with the relay's car from the first step, not from straight wheels.
  if (steer && !_boat && _jveh.valid())
    w.set_vehicle_steer(_jveh, *steer);
  keep_jolt(v);
  return v;
}

dynamic_vehicle dynamic_vehicle_model::at_bike(const ph::body_state &chassis,
                                               const dynamic_vehicle *like,
                                               std::optional<double> steer) const {
  const ph::dyn_bike_saved *was = like ? std::get_if<ph::dyn_bike_saved>(&like->model) : nullptr;
  dynamic_vehicle v;
  if (like)
    load_dworld(*like); // with a world (OW5p): as predicted for the tick (the room's boxes)
  ph::dyn_bike_saved m;
  if (was) {
    m = *was;
    v.steer_cmd = like->steer_cmd;
    v.accel_cmd = like->accel_cmd;
  } else {
    m = _bike->save(0);
    m.body = {};
    m.wheels[0] = m.wheels[1] = 0.0;
    m.driver = {};
    m.steer_cmd = m.accel_cmd = 0.0;
  }
  // The box's pose and velocity back into the model, as the plant takes a world's (its settle):
  // the rear axle behind the box's centre, the twist at the centre of gravity.
  const double psi = yaw_of(chassis.rotation);
  const double c = std::cos(psi), s = std::sin(psi);
  const double rear_x = _bike->rear_axle(0);
  const double off = -rear_x - _bike->params(0).lr;
  m.body.x = chassis.position.x + rear_x * c;
  m.body.y = chassis.position.y + rear_x * s;
  m.body.psi = psi;
  // The yaw rate and the steering angle: the sample's when it carries them (OW5n), else LIKE's.
  const double r = steer ? chassis.angular_velocity.z : was ? was->body.r : 0.0;
  if (steer)
    m.body.delta = *steer;
  m.body.r = r;
  const double vgx = chassis.linear_velocity.x + r * off * s,
               vgy = chassis.linear_velocity.y - r * off * c;
  m.body.vx = c * vgx + s * vgy;
  m.body.vy = -s * vgx + c * vgy;
  v.model = m;
  v.box = {chassis.position.x, chassis.position.y, chassis.position.z, psi};
  v.vx = chassis.linear_velocity.x;
  v.vy = chassis.linear_velocity.y;
  v.wz = r;
  v.world = _dlast; // its box goes where the model is at its step (restore)
  return v;
}

dynamic_vehicle dynamic_vehicle_model::at_four(const ph::body_state &chassis,
                                               const dynamic_vehicle *like,
                                               std::optional<double> steer) const {
  const ph::dyn_4w_saved *was = like ? std::get_if<ph::dyn_4w_saved>(&like->model) : nullptr;
  dynamic_vehicle v;
  if (like)
    load_dworld(*like); // with a world (OW5p): as predicted for the tick (the room's boxes)
  ph::dyn_4w_saved m;
  if (was) {
    m = *was;
    v.steer_cmd = like->steer_cmd;
    v.accel_cmd = like->accel_cmd;
  } else {
    // At rest where the sample is, its suspension settled on the ground under its wheels, as the
    // plant places a vehicle.
    const double psi = yaw_of(chassis.rotation);
    const double rear_x = _four->rear_axle(0);
    cvc::nav::plant_state start;
    start.x = static_cast<float>(chassis.position.x + rear_x * std::cos(psi));
    start.y = static_cast<float>(chassis.position.y + rear_x * std::sin(psi));
    start.heading = static_cast<float>(psi);
    _four->remove_vehicle(0);
    _four->add_vehicle(start);
    if (_dworld) // its box made again in the world (OW5p): the world as it is now
      _dlast = std::make_shared<const std::vector<std::uint8_t>>(_dworld->world().save());
    m = _four->save(0);
    const double z = m.body.z, phi = m.body.phi, th = m.body.th;
    m.body = {};
    m.body.z = z;
    m.body.phi = phi;
    m.body.th = th;
    m.driver = {};
    m.steer_cmd = m.accel_cmd = 0.0;
  }
  // The box's horizontal pose and twist back into the model, as the plant takes a world's after a
  // collision: the centre of gravity from the box's centre, the rear axle behind it. When the
  // sample carries the chassis's pitch and roll (the relay's box, OW5m), the suspension takes them
  // and its heave from the box's height; else the box leans with the model's (a turn about z
  // alone: a relay that sends the heading only). The rates of heave, roll and pitch stay the
  // model's: the pose stream does not carry the latter two, and its vertical speed is the box's
  // after the relay's step under gravity, not the suspension's.
  const ph::dyn_4w_plant &pl = *_four;
  const double rear_x = pl.rear_axle(0);
  const double lr = pl.params(0).lr;
  const double off = -rear_x - lr;
  const double psi = yaw_of(chassis.rotation);
  const double c = std::cos(psi), s = std::sin(psi);
  const ph::vec3 to_cg{rear_x + lr, 0.0, pl.desc(0).center_of_mass.z};
  const ph::quat &q = chassis.rotation;
  const double qn = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if ((q.x != 0.0 || q.y != 0.0) && qn > 0.0) {
    const ph::quat u{q.w / qn, q.x / qn, q.y / qn, q.z / qn};
    m.body.th = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (u.w * u.y - u.z * u.x))));
    m.body.phi = std::atan2(2.0 * (u.w * u.x + u.y * u.z), 1.0 - 2.0 * (u.x * u.x + u.y * u.y));
    // The centre of gravity's height over the plant's datum (dyn_4w_plant::box_pose inverted).
    const ph::vec3 lever = rotate(attitude(psi, m.body.th, m.body.phi), to_cg);
    m.body.z = chassis.position.z + lever.z - pl.ground_reference() - pl.params(0).h;
  }
  const ph::vec3 d = rotate(attitude(psi, m.body.th, m.body.phi), to_cg);
  m.body.x = chassis.position.x + d.x - lr * c;
  m.body.y = chassis.position.y + d.y - lr * s;
  m.body.psi = psi;
  // The yaw, roll and pitch rates and the steering angle: the sample's when it carries them (the
  // relay's box's angular velocity, its plant's steering; OW5n), else LIKE's. The box spins as
  // the plant puts it: (c phid - s thd, s phid + c thd, r).
  const ph::vec3 &om = chassis.angular_velocity;
  const double r = steer ? om.z : was ? was->body.r : 0.0;
  if (steer) {
    m.body.delta = *steer;
    m.body.phid = c * om.x + s * om.y;
    m.body.thd = -s * om.x + c * om.y;
  }
  m.body.r = r;
  const double vgx = chassis.linear_velocity.x + r * off * s,
               vgy = chassis.linear_velocity.y - r * off * c;
  m.body.vx = c * vgx + s * vgy;
  m.body.vy = -s * vgx + c * vgy;
  v.model = m;
  v.box = {chassis.position.x, chassis.position.y, chassis.position.z, psi};
  v.pitch = m.body.th;
  v.roll = m.body.phi;
  v.vx = chassis.linear_velocity.x;
  v.vy = chassis.linear_velocity.y;
  v.wz = r;
  v.world = _dlast; // its box goes where the model is at its step (restore)
  return v;
}

void dynamic_vehicle_model::step(dynamic_vehicle &v, std::uint64_t tick) {
  const bool nav_tick = into_tick(_drive, tick) == 0;
  if (is_boat())
    step_boat(v, nav_tick);
  else if (_jolt)
    step_jolt(v, nav_tick);
  else if (_four)
    step_four(v, nav_tick);
  else
    step_bike(v, nav_tick);
}

void dynamic_vehicle_model::step_jolt(dynamic_vehicle &v, bool tick) {
  load_jolt(v);
  ph::world &w = _jolt->world();
  if (tick) {
    // The navigator's tick: its hold, then nav_plant::command's drive (forward or backward by
    // where the vehicle is to be moving at the tick's end; a few cm/s on its springs is rest).
    const double speed = jolt_speed();
    float steer = 0.0f, accel = 0.0f;
    held(_drive, _dt, v.steer_cmd, v.accel_cmd, speed, steer, accel);
    constexpr double kRestSpeed = 0.1; // m/s, nav_plant's
    const double v0 = std::fabs(speed) < kRestSpeed ? 0.0 : speed;
    const double a = static_cast<double>(accel);
    const bool reverse = v0 + static_cast<double>(_drive.tick_steps) * _dt * a < 0.0;
    ph::drive_command c;
    c.steer = steer;
    c.accel = reverse ? -a : a;
    c.reverse = reverse;
    w.drive(_jveh, c);
  }
  w.step();
  keep_jolt(v);
}

void dynamic_vehicle_model::set_water(const std::vector<ph::field> &water,
                                      std::uint64_t from_tick) {
  if (!is_boat() || !_jolt)
    return;
  // In tick order: a sea that goes in no later than one kept replaces it and those after it.
  while (!_waters.empty() && _waters.size() > 1 && _waters.back().from_tick >= from_tick) {
    _waters.pop_back();
    _water_serial.pop_back();
  }
  _waters.push_back({from_tick, water});
  _water_serial.push_back(_water_next++);
  while (_waters.size() > kWaterKept) {
    _waters.erase(_waters.begin());
    _water_serial.erase(_water_serial.begin());
  }
}

void dynamic_vehicle_model::water_for(std::uint64_t tick) const {
  // The newest water in by the step ending on TICK: its from_tick is behind it (SceneWorld puts a
  // keyframe's field in at the first step that ends past it); before them all, the first.
  std::size_t i = 0;
  for (std::size_t j = 0; j < _waters.size(); ++j)
    if (_waters[j].from_tick == 0 || tick > _waters[j].from_tick)
      i = j;
  put_water(i);
}

void dynamic_vehicle_model::water_to(std::uint64_t serial) const {
  for (std::size_t i = 0; i < _water_serial.size(); ++i)
    if (_water_serial[i] == serial) {
      put_water(i);
      return;
    }
}

void dynamic_vehicle_model::put_water(std::size_t i) const {
  if (_waters.empty() || i >= _waters.size() || _water_serial[i] == _water_on)
    return;
  ph::world &w = _jolt->world();
  const std::vector<ph::field> &water = _waters[i].water;
  if (water.size() == _water_ids.size()) {
    for (std::size_t k = 0; k < water.size(); ++k)
      w.set_field(_water_ids[k], water[k]);
  } else {
    for (const ph::field_id id : _water_ids)
      w.remove_field(id);
    _water_ids.clear();
    for (const ph::field &f : water)
      _water_ids.push_back(w.add_field(f));
  }
  _water_on = _water_serial[i];
}

void dynamic_vehicle_model::step_boat(dynamic_vehicle &v, bool tick) {
  const jolt_vehicle_saved *s = std::get_if<jolt_vehicle_saved>(&v.model);
  if (!s)
    throw std::invalid_argument("dynamic_vehicle_model::step: a vehicle of the other plant");
  if (!s->world || !_boat)
    return; // not on the water yet: it stays where it is until a sample puts it there
  load_jolt(v);
  // The water of the tick this step ends on (OW5q): the world's field time is the clock's (at()
  // set it to the sample's tick's, and each step adds a step).
  water_for(static_cast<std::uint64_t>(std::llround(_jolt->world().field_time() / _dt)) + 1);
  if (tick) {
    // The navigator's tick: its hold at the plant's speed, then the command to the driver.
    float steer = 0.0f, accel = 0.0f;
    held(_drive, _dt, v.steer_cmd, v.accel_cmd, _boat->state(0).speed, steer, accel);
    _boat->command(0, steer, accel);
  }
  _boat->advance(); // the driver's controls, one world step, the books
  keep_jolt(v);
}

void dynamic_vehicle_model::step_bike(dynamic_vehicle &v, bool tick) {
  if (!std::holds_alternative<ph::dyn_bike_saved>(v.model))
    throw std::invalid_argument("dynamic_vehicle_model::step: a vehicle of the other plant");
  load_dworld(v); // with a world (OW5p): as V left it, then its box where V's model is
  _bike->restore(0, std::get<ph::dyn_bike_saved>(v.model));
  if (tick) {
    float steer = 0.0f, accel = 0.0f;
    held(_drive, _dt, v.steer_cmd, v.accel_cmd, _bike->body(0).vx, steer, accel);
    _bike->command(0, steer, accel);
  }
  // The box where the substep starts: on the ground under it, leaning with it
  // (dyn_bike_plant::step_model's chassis_pose).
  ph::vec3 pos;
  ph::quat rot;
  _bike->box_pose(0, pos, rot);
  const double psi0 = _bike->body(0).psi;
  _bike->advance(); // one substep: one physics step (with a world, its step and what it hit)
  const ph::dyn_bike_saved after = _bike->save(0);
  v.model = after;
  keep_dworld(v);
  // Moving as the model moved, for the world's step: along the ground, turning about +z.
  const double rear_x = _bike->rear_axle(0);
  const double off = -rear_x - _bike->params(0).lr;
  const cvc::nav::diff::dyn_state<double> &o = after.body;
  const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
  v.vx = c1 * o.vx - s1 * o.vy - o.r * off * s1;
  v.vy = s1 * o.vx + c1 * o.vy + o.r * off * c1;
  v.wz = o.r;
  v.box = {pos.x + v.vx * _dt, pos.y + v.vy * _dt, pos.z, psi0 + o.r * _dt};
  // The ground's pitch and roll under it (Rz(psi) Ry(pitch) Rx(roll)).
  v.pitch = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (rot.w * rot.y - rot.z * rot.x))));
  v.roll = std::atan2(2.0 * (rot.w * rot.x + rot.y * rot.z),
                      1.0 - 2.0 * (rot.x * rot.x + rot.y * rot.y));
}

void dynamic_vehicle_model::step_four(dynamic_vehicle &v, bool tick) {
  if (!std::holds_alternative<ph::dyn_4w_saved>(v.model))
    throw std::invalid_argument("dynamic_vehicle_model::step: a vehicle of the other plant");
  load_dworld(v); // with a world (OW5p): as V left it, then its box where V's model is
  _four->restore(0, std::get<ph::dyn_4w_saved>(v.model));
  if (tick) {
    float steer = 0.0f, accel = 0.0f;
    held(_drive, _dt, v.steer_cmd, v.accel_cmd, _four->body(0).vx, steer, accel);
    _four->command(0, steer, accel);
  }
  // The box where the substep starts (dyn_4w_plant::substep_begin's chassis_pose).
  ph::vec3 pos;
  ph::quat rot;
  _four->box_pose(0, pos, rot);
  const double psi0 = _four->body(0).psi;
  _four->advance(); // one substep: one physics step (with a world, its step and what it hit)
  const ph::dyn_4w_saved after = _four->save(0);
  v.model = after;
  keep_dworld(v);
  // Moving as the model moved, for the world's step.
  const double rear_x = _four->rear_axle(0);
  const double off = -rear_x - _four->params(0).lr;
  const cvc::nav::diff::dyn4_state<double> &o = after.body;
  const double c1 = std::cos(o.psi), s1 = std::sin(o.psi);
  v.vx = c1 * o.vx - s1 * o.vy - o.r * off * s1;
  v.vy = s1 * o.vx + c1 * o.vy + o.r * off * c1;
  v.wz = o.r;
  v.box = {pos.x + v.vx * _dt, pos.y + v.vy * _dt, pos.z + o.zd * _dt, psi0 + o.r * _dt};
  v.pitch = o.th;
  v.roll = o.phi;
}

ph::body_state dynamic_vehicle_state(const dynamic_vehicle &v) {
  ph::body_state s;
  s.position = {v.box[0], v.box[1], v.box[2]};
  s.rotation = attitude(v.box[3], v.pitch, v.roll);
  s.linear_velocity = {v.vx, v.vy, 0.0};
  s.angular_velocity = {0.0, 0.0, v.wz};
  return s;
}

} // namespace ariadne
} // namespace cvc
