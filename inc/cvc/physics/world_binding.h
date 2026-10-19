/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::world_binding — a physics world configured from, and reported to,
// an app's state tree.
//
// Everything configurable about a world lives in the tree under one prefix
// (default "physics"), so a scene file, a UI, a Python script or a remote peer
// sets it and every one of them sees the same values. The world itself stays
// free of the tree (like world_clock and world_clock_binding): the binding reads
// the tree into the world on pull() and writes the world's telemetry on push(),
// Nothing here is global: a binding belongs to one app and one world.
//
// Thread-safe: every call locks the binding (and the world locks itself), so a
// UI thread may pull() a change while the simulation thread calls step(); the
// state tree is the cross-thread channel either way.
//
//   auto cfg = cvc::physics::world_binding::config_from_state(app);   // seeds defaults
//   cvc::physics::world w(cfg);
//   cvc::physics::world_binding bind(app, w);
//   bind.publish_vehicle("truck", truck);                               // its keys, below
//   ... per frame: bind.follow(app.world_clock());                    // pull, step, push
//   ... any thread: auto s = world_binding::snapshot_from_state(app);  // the whole world
//   ... Python:     json.loads(pycvc.state_get(app, "physics.snapshot_json"))
//
// Keys, relative to the prefix. A key that is unset is seeded with the world's
// value (the knob idiom: whatever set it first, a scene or a script, wins). Every key the
// binding owns says what it is, for other applications: its comment (cvc::state::comment) and
// its properties (cvc::state::property). Each has "access" (build | knob | command |
// telemetry); a number has "unit" (SI; "1" for a ratio); and a knob or command has a recommended
// "min" and "max", for a UI or a validator. The world validates on its own. An
// unreadable value is ignored, never thrown into a frame. pull() reads the tree only
// when something under the prefix changed since the last pull (every time if a key
// is a link into another scope), so an unchanged tree costs a pull nothing.
//
//   built with the world (config_from_state reads them; changing one later sets
//   restart_required instead of touching the running world):
//     backend                  string  null | jolt
//     fixed_dt                 double  seconds per step
//     threads                  int     worker threads in a step; 0 = none
//     determinism              string  same_binary | cross_platform
//     max_bodies, max_body_pairs, max_contact_constraints   int
//     origin.x, origin.y, origin.z                          double, metres
//
//   applied by every pull():
//     gravity.x, gravity.y, gravity.z      double  m/s^2
//     collision_steps, velocity_steps, position_steps     int
//     penetration_slop                     double  metres
//     friction_combine, restitution_combine
//                                          string  product | min | max | geometric_mean | average
//     contacts                             string  none | added_removed | all
//     materials.<name>.friction            double  one child per material; a new child adds
//     materials.<name>.restitution         double  a material, a changed value updates it from
//     materials.<name>.rolling_resistance  double  the next step. "default" is material 0.
//     materials.<name>.nav_material        int     Names must not contain '.'.
//   how often push() publishes, in steps (with fixed_dt = 0.01, 10 is 10 Hz):
//     publish_every                        int     step()/follow() push every N steps (1):
//                                                  the telemetry and the named actors' keys
//     snapshot_every                       int     the snapshot data, at most every push,
//                                                  every N steps (1); 0 = never
//     snapshot_json_every                  int     the snapshot as JSON every N steps; 0 =
//                                                  never, the default (JSON costs the most)
//     hash_every                           int     state_hash every N steps; 0 = never, the
//                                                  default (it digests the whole state)
//     publish_thread                       int     1: a background thread writes what a push
//                                                  publishes, so a step pays only for copying
//                                                  the snapshot; a newer publication replaces
//                                                  one it has not started. 0, the default:
//                                                  push() writes before it returns.
//
//   written by push():
//     tick, time, step_ms, bodies, active_bodies, contact_events, saturated
//     state_hash               string  16 hex digits (see hash_every)
//     version                  string  the backend and its build options
//     restart_required         bool    a build-time key differs from the running world
//     snapshot                 data    std::shared_ptr<const world_snapshot> in the
//                                      node's data(): every body, field, vehicle and
//                                      contact at one tick (snapshot_from_state reads it)
//     snapshot_json            string  that snapshot as JSON (snapshot_json.h) without the
//                                      static bodies
//     snapshot_json_static     string  the static bodies as JSON, rewritten only when one
//                                      is added, removed, moved or given a material (its
//                                      static_version matches snapshot_json's then)
//     publish_dropped          int     publications publish_thread skipped because a newer
//                                      one replaced them
//
//   named actors (publish_body, publish_vehicle, publish_field, publish): every push()
//   writes their telemetry from that one snapshot, so the keys of a push agree on a tick.
//     bodies.<name>.id, motion             the body id; static | kinematic | dynamic
//     bodies.<name>.position.{x,y,z}, rotation.{w,x,y,z}, velocity.{x,y,z},
//       angular_velocity.{x,y,z}, active, field_force.{x,y,z}, immersed_volume
//     vehicles.<name>.chassis, speed, steer, steer_command, throttle, brake,
//       hand_brake, gear, engine_rpm, accel_trim, position.*, rotation.*, velocity.*,
//       angular_velocity.*, aero_force.{x,y,z}, air_velocity.{x,y,z}
//     vehicles.<name>.wheels.<i>.{steer, angular_velocity, suspension_length, contact,
//       contact_point.{x,y,z}, load, longitudinal_slip, lateral_slip, friction,
//       rolling_resistance, medium_depth, sinkage, rut, medium_resistance}
//     vehicles.<name>.energy.{P_W, supplied_J, idle_J, brake_J, tire_J, aero_J, roll_J,
//       medium_J, drivetrain_J, drivetrain_split_J, suspension_J, chassis_damping_J, fields_J,
//       stored_J, residual_J, regen_J, source_J, soc, fuel_kg, fuel_left_kg, distance_m,
//       Wh_per_km}                         the energy ledger (vehicle.h's vehicle_energy)
//     fields.<name>.kind, ruts.cells, ruts.deepest
//   and read by every pull():
//     bodies.<name>.{material, mass, linear_damping, angular_damping, allow_sleeping}
//                                                         world::set_body; material by name
//     vehicles.<name>.command.{steer, accel, reverse}     a change drives it
//                                                         (world::drive)
//     vehicles.<name>.controls.{steer, throttle, brake, hand_brake, reverse, neutral}
//                                                         a change sets its pedals
//                                                         (world::set_controls)
//     vehicles.<name>.tuning.{max_steer, max_steer_rate, max_accel, max_decel, max_speed,
//       drag_coefficient, frontal_area, air_density, rolling_resistance,
//       engine_max_torque, engine_min_rpm, engine_max_rpm, shift_up_rpm, shift_down_rpm,
//       shift_up_rpm_light, shift_down_rpm_light, clutch_strength, converter_k,
//       converter_stall, converter_coupling, driveline_efficiency, engine_friction_curve}
//                                          engine_friction_curve is "rpm:N m" points,
//                                          "700:78, 2040:87, 3400:128" ("none": none)
//     vehicles.<name>.tuning.axles.<i>.{max_brake_torque, max_hand_brake_torque,
//       suspension_frequency, suspension_damping, anti_roll_stiffness}
//                                                         world::set_vehicle
//     fields.<name>.{drag_coefficient, damping, traction} world::set_field
//   Commands act on a change only, so a program may drive the same vehicle through
//   the world between changes. Knobs follow the material idiom: seeded with the
//   world's values, applied when the tree's value changes, an invalid one ignored.
//   An actor the world no longer has stops being published; its keys keep their
//   last values.

#ifndef __CVC_PHYSICS_WORLD_BINDING_H__
#define __CVC_PHYSICS_WORLD_BINDING_H__

#include <cstdint>
#include <cvc/physics/world.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

class app;
class world_clock;

namespace physics {

struct world_bodies;

class world_binding {
public:
  // A world_config read from the tree under `prefix`, seeding every unset key
  // with the default. Materials are not part of it: the binding adds them.
  static world_config config_from_state(cvc::app &ctx, const std::string &prefix = "physics");

  world_binding(cvc::app &ctx, physics::world &w, std::string prefix = "physics");
  // Joins the publish thread after it writes what it holds.
  ~world_binding();
  world_binding(const world_binding &) = delete;
  world_binding &operator=(const world_binding &) = delete;

  // Tree -> world: the runtime settings, then the material table and the named actors.
  void pull();
  // World -> tree: the telemetry keys, and what is due of the snapshot.
  void push();
  // pull(), one step, push() (every publish_every steps).
  step_report step();
  // pull(), world().follow(clock), push() if any step ran and one is due.
  std::uint64_t follow(const cvc::world_clock &clock);
  // Wait until the publish thread has written every publication handed to it; at once
  // without one.
  void flush();

  // The id of a material declared under <prefix>.materials (as of the last pull).
  // Throws cvc::physics_error for a name the tree does not declare.
  material_id material(const std::string &name) const;

  // Publish a body, a vehicle or a field under <prefix>.bodies.<name>, .vehicles.<name>
  // or .fields.<name> (the keys above). Publishing a name again rebinds it. Throws
  // cvc::physics_error for an empty name or one with a '.', and for an id the world
  // does not hold.
  void publish_body(const std::string &name, body_id id);
  void publish_vehicle(const std::string &name, vehicle_id id);
  void publish_field(const std::string &name, field_id id);
  // Every body and field add_world_geometry added, under their item and volume names
  // (a '.' in one becomes '_').
  void publish(const world_bodies &added);
  // Stop publishing a name; its keys keep their last values.
  void unpublish_body(const std::string &name);
  void unpublish_vehicle(const std::string &name);
  void unpublish_field(const std::string &name);

  // The snapshot published under <prefix>.snapshot, from any thread; null before the
  // first.
  static std::shared_ptr<const world_snapshot>
  snapshot_from_state(cvc::app &ctx, const std::string &prefix = "physics");
  // The last snapshot this binding published; null before the first.
  std::shared_ptr<const world_snapshot> last_snapshot() const;

  // The absolute state path of a key, e.g. key("gravity.z") = "physics.gravity.z".
  std::string key(const std::string &name) const;

  physics::world &world() { return _w; }
  const std::string &prefix() const { return _prefix; }

private:
  struct material_slot {
    material_id id = 0;
    physics::material applied;
  };

  // The command and control keys as last seen, and the knobs as last applied.
  struct body_slot {
    body_id id;
    body_settings tuned;
    std::string material; // the material key as last seen
  };
  struct vehicle_slot {
    vehicle_id id;
    std::vector<std::string> command, controls;
    vehicle_desc tuned;
  };
  struct field_slot {
    field_id id;
    double tuned[3] = {0.0, 0.0, 0.0}; // drag_coefficient, damping, traction
  };

  struct publication; // what one push writes (world_binding.cpp)
  struct background;  // the publish thread and the tree watch (world_binding.cpp)

  void seed();
  void pull_locked();
  void push_locked();
  void pull_materials();
  void pull_body(const std::string &name, body_slot &b);
  void pull_vehicle(const std::string &name, vehicle_slot &v);
  void pull_field(const std::string &name, field_slot &f);
  void gather(publication &p);
  void write(const publication &p) const;
  void start_publisher();
  void stop_publisher();

  mutable std::mutex _mutex;
  cvc::app &_ctx;
  physics::world &_w;
  std::string _prefix;
  bool _seeded = false;
  bool _links = false; // a key read through a link: pull() reads every time
  int _publish_every = 1;
  int _hash_every = 0;
  int _snapshot_every = 1;
  int _snapshot_json_every = 0;
  int _publish_thread = 0;
  std::shared_ptr<const world_snapshot> _snapshot;
  bool _json_published = false;
  std::uint64_t _json_tick = 0;
  bool _json_static_published = false;
  std::uint64_t _json_static_version = 0;
  bool _restart_required = false;
  step_report _last;
  std::map<std::string, material_slot> _materials;
  std::map<std::string, body_slot> _bodies;
  std::map<std::string, vehicle_slot> _vehicles;
  std::map<std::string, field_slot> _fields;
  std::unique_ptr<background> _bg;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_WORLD_BINDING_H__
