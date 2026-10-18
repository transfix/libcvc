/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::world — a rigid-body world stepped in fixed quanta.
//
//   cvc::physics::world w;                       // Jolt if the build has it, else throws
//   const auto ice = w.add_material({"ice", 0.1f});
//   auto ground = std::make_shared<cvc::height_field>(...);  // the world's own terrain
//   w.add_body({cvc::physics::terrain{ground}, cvc::physics::motion_type::static_});
//   const auto box = w.add_body({cvc::physics::box{{0.2, 0.2, 0.2}}});
//   for (int i = 0; i < 100; ++i) w.step();      // 1 s at the default 0.01 s quantum
//
// Time: one step() advances fixed_dt seconds. A world driven by an application's
// cvc::world_clock calls follow(clock) once per frame, which takes the steps due
// since the last call (fixed_dt must be a whole number of the clock's quanta; the
// libcvc default suite quantum is 0.01 s, so a world at fixed_dt = 0.01 steps once
// per quantum).
//
// Determinism: a run is a function of the initial state, the order of calls, the
// world_config and the backend build. state_hash() digests the whole simulation
// state, so two runs (or a run and a replay, or a native run and a wasm one under
// cross-platform determinism) can be compared step by step. A determinism mode
// the backend cannot deliver is refused at construction, never downgraded.
//
// Threads: the world is thread-safe. Every call locks it, and step() holds the
// lock for the whole engine step, so a call from another thread waits at most
// one step; accessors return copies, never references into the world. Any
// thread may add bodies, push forces, read state or step. world_config::threads
// adds worker threads inside step() itself. To read a running world from many
// threads without ever waiting on a step, read the telemetry a world_binding
// publishes to the state tree.
//
// The whole world at one tick: snapshot() copies its model (config, materials,
// fields, vehicle descriptions) and its state (every body, every vehicle and its
// wheels, what the fields did, the last step's contacts) under one lock, and a
// world_binding publishes that copy to the state tree as it steps.
//
// Errors throw cvc::physics_error (bad arguments, unknown or stale body ids,
// engine failures) or cvc::unsupported_exception (a backend or mode this build
// cannot provide).

#ifndef __CVC_PHYSICS_WORLD_H__
#define __CVC_PHYSICS_WORLD_H__

#include <cstddef>
#include <cstdint>
#include <cvc/physics/backend.h>
#include <cvc/physics/body.h>
#include <cvc/physics/field.h>
#include <cvc/physics/shape.h>
#include <cvc/physics/types.h>
#include <cvc/physics/vehicle.h>
#include <memory>
#include <optional>
#include <vector>

namespace cvc {

class world_clock;

namespace physics {

enum class determinism : std::uint8_t {
  same_binary,    // the same build on the same platform reproduces a run
  cross_platform, // every platform the backend build targets reproduces a run
};

// How two surfaces' material values combine at a contact.
enum class combine_rule : std::uint8_t {
  product,        // a * b; with the default material (friction 1) the other surface's value is
                  // used exactly, which is how a vehicle sees the terrain's μ
  min,            // min(a, b)
  max,            // max(a, b)
  geometric_mean, // sqrt(a * b)
  average,        // (a + b) / 2
};

enum class contact_reporting : std::uint8_t {
  none,
  added_removed, // a contact starting or ending
  all,           // also every step a contact persists
};

struct world_config {
  physics::backend backend = physics::backend::jolt;
  vec3 gravity{0.0, 0.0, -9.80665}; // m/s^2, standard gravity along -z
  double fixed_dt = 0.01;           // seconds per step
  int collision_steps = 2;          // collision sub-steps per step (Jolt: 0.005 s each by default)
  int velocity_steps = 10;          // solver velocity iterations
  int position_steps = 2;           // solver position iterations
  double penetration_slop = 0.002;  // metres two bodies may overlap before the solver pushes them
                                    // apart (Jolt's own default, 0.02, is sized for game props)
  int threads = 0; // worker threads inside step(): 0 = none (single-threaded job system)
  physics::determinism determinism = physics::determinism::cross_platform;
  combine_rule friction = combine_rule::product;
  combine_rule restitution = combine_rule::max;
  contact_reporting contacts = contact_reporting::added_removed;
  vec3 origin; // the floating origin: the engine works in float offsets from it
  std::uint32_t max_bodies = 65536;
  std::uint32_t max_body_pairs = 65536;
  std::uint32_t max_contact_constraints = 65536;
  std::size_t temp_allocator_bytes = 32u << 20; // per-step scratch memory
};

// A contact between two bodies, reported by the step that saw it start, persist
// or end. body_a < body_b; the normal points from a toward b.
struct contact_event {
  enum class kind : std::uint8_t { added, persisted, removed };
  kind type = kind::added;
  body_id body_a, body_b;
  std::uint32_t sub_shape_a = 0, sub_shape_b = 0; // the engine's sub-shape (triangle, cell) ids
  material_id material_a = 0, material_b = 0;     // the surfaces in contact
  float friction = 0.0f, restitution = 0.0f;      // what the solver used (not set for removed)
  vec3 point;                                     // world metres (not set for removed)
  vec3 normal;                                    // unit, a toward b (not set for removed)
  double depth = 0.0;                             // penetration, metres (not set for removed)
};

struct ray_hit {
  body_id body;
  double fraction = 0.0; // along the segment, in [0, 1]
  vec3 point;
  vec3 normal; // the surface normal at the hit
  material_id material = 0;
  std::uint32_t sub_shape = 0;
};

struct step_report {
  std::uint64_t tick = 0;   // the world's tick after the step
  double step_ms = 0.0;     // wall time the step took
  std::size_t contacts = 0; // events in contacts()
  std::uint32_t active_bodies = 0;
  bool saturated = false; // a fixed-size engine buffer filled and contacts were dropped: raise
                          // max_body_pairs / max_contact_constraints
};

// Several bodies' states and the tick they belong to, read under one lock, so a
// batch never mixes two steps (world::states).
struct state_batch {
  std::uint64_t tick = 0;                        // the world's tick when read
  std::vector<std::optional<body_state>> states; // one per id, in order; empty for an
                                                 // unknown or removed id
};

// One body in a world_snapshot.
struct body_snapshot {
  body_id id;
  motion_type motion = motion_type::dynamic;
  body_settings settings; // as added, then as set_body changed them
  std::uint64_t user_data = 0;
  body_state state;
  field_load fields; // what the fields did to it in the last step
};

// One field in a world_snapshot.
struct field_snapshot {
  field_id id;
  physics::field desc;       // as given (world::set_field)
  std::size_t rut_cells = 0; // rut cells a wheel has pressed (field::ruts)
  double rut_deepest = 0.0;  // m, the deepest rut
  vec3 lo, hi;               // its box, world metres
};

// One vehicle in a world_snapshot.
struct vehicle_snapshot {
  vehicle_id id;
  vehicle_desc desc; // as added, then tuned (world::set_vehicle)
  physics::vehicle_state state;
};

// The whole world at one tick (world::snapshot). Every list is in ascending id order.
struct world_snapshot {
  std::uint64_t tick = 0;
  double time = 0.0; // tick * fixed_dt
  world_config config;
  step_report report;                // the last step's
  std::vector<material> materials;   // by id
  std::vector<body_snapshot> bodies; // the dynamic and kinematic ones, chassis included
  // The static bodies, never null. They change only when one is added, removed, moved
  // or given a new material, so successive snapshots share one list until then, and
  // static_version tells when it changed.
  std::shared_ptr<const std::vector<body_snapshot>> static_bodies;
  std::uint64_t static_version = 0;
  std::vector<field_snapshot> fields;
  std::vector<vehicle_snapshot> vehicles;
  std::vector<contact_event> contacts; // the last step's, sorted as contacts() sorts them
};

class world {
public:
  explicit world(const world_config &cfg = world_config());
  ~world();
  world(const world &) = delete;
  world &operator=(const world &) = delete;
  world(world &&) noexcept;
  world &operator=(world &&) noexcept;

  world_config config() const;
  physics::backend backend() const;
  std::uint32_t capabilities() const;

  // Apply the settings that may change while the world runs: gravity, the
  // collision, velocity and position steps, the penetration slop, the combine
  // rules and contact reporting. The rest are fixed at construction (backend,
  // fixed_dt, threads, determinism, the engine buffer sizes, the origin, which
  // moves with rebase()); a different value for one of them throws
  // cvc::physics_error naming it and changes nothing.
  void reconfigure(const world_config &cfg);

  // --- materials ------------------------------------------------------------

  // Material 0 exists from construction: "default", friction 1, restitution 0.
  // With the default product rule a body on material 0 takes the other surface's
  // friction exactly. Materials are never removed.
  material_id add_material(const material &m);
  // New values for a material; contacts use them from the next step. Every plant
  // reads this one table, so this is how rain on asphalt or a resurfaced road
  // reaches a vehicle's tires and a box alike.
  void set_material(material_id id, const material &m);
  material get_material(material_id id) const;
  std::size_t material_count() const;

  // --- bodies ---------------------------------------------------------------

  body_id add_body(const body_desc &d);
  void remove_body(body_id id);
  bool contains(body_id id) const;
  std::size_t body_count() const;

  body_state state(body_id id) const;
  // The states of `ids` and the tick, under one lock: what a renderer samples
  // (pose_tracker.h). An unknown or removed id reads as empty instead of
  // throwing, since a body can be removed between two frames.
  state_batch states(const std::vector<body_id> &ids) const;
  // Teleport a body and set its velocities. Activates it.
  void set_state(body_id id, const body_state &s);
  // A force (N) applied at the centre of mass through the next step only.
  void add_force(body_id id, const vec3 &force);
  // An impulse (N·s) applied at the centre of mass now.
  void add_impulse(body_id id, const vec3 &impulse);
  // Drive a kinematic body so it reaches this pose at the end of the next step.
  void move_kinematic(body_id id, const vec3 &position, const quat &rotation);

  // A body's settings: as added, then as set_body changed them.
  body_settings get_body(body_id id) const;
  // Change a running body's settings from the next step on, and wake it. A static
  // body takes only a new material; a mesh or terrain body keeps its materials (they
  // are its triangles' or cells'; set_material changes those); only a dynamic body has
  // a mass. Throws cvc::physics_error for those changes, for values add_body refuses,
  // and for a vehicle's chassis (set_vehicle tunes a vehicle), changing nothing. Like
  // materials, the settings are not part of save().
  void set_body(body_id id, const body_settings &s);

  // --- fields (field.h) -----------------------------------------------------

  // A region of water, snow or wind that pushes on the dynamic bodies inside it,
  // from the next step on. Wakes the bodies in its box. Throws cvc::physics_error
  // for a field without a volume, a volume without voxels or with a flat box,
  // values that are not finite (or are negative), one or two velocity components,
  // or coefficients that are not finite and >= 0.
  field_id add_field(const field &f);
  // New values for a field (a different volume, a stronger wind). Wakes the bodies
  // in its old and new boxes. A rutted medium keeps its ruts when the new field lays
  // them on the same grid (the same box and rut_cell); otherwise they start over.
  void set_field(field_id id, const field &f);
  // The field as last given.
  field get_field(field_id id) const;
  void remove_field(field_id id);
  std::size_t field_count() const;
  // What the fields did to `id` in the last step: zero for a body no field
  // touched, and for a static, kinematic or sleeping body.
  field_load field_load_on(body_id id) const;
  // The ruts a medium with field::ruts keeps; empty for one without.
  rut_map field_ruts(field_id id) const;

  // --- vehicles (vehicle.h) ----------------------------------------------------

  // A wheeled vehicle with its chassis's origin at `position`. Throws cvc::physics_error for
  // a description it refuses (no axles, a steered axle with nothing behind it, values that
  // are not finite or out of range) and cvc::unsupported_exception on a backend without
  // cap_vehicles. Its chassis is a body (vehicle_state::chassis): world::state reads it, and
  // world::remove_body refuses it (remove the vehicle instead).
  vehicle_id add_vehicle(const vehicle_desc &d, const vec3 &position, const quat &rotation = {});
  void remove_vehicle(vehicle_id id);
  std::size_t vehicle_count() const;
  // From the next step on: the navigator's command, through the driver model...
  void drive(vehicle_id id, const drive_command &c);
  // ... or pedals of your own, past it. The steering goes through the actuator either way.
  void set_controls(vehicle_id id, const vehicle_controls &c);
  // The chassis, the actuator, the pedals the last step applied, the powertrain and every
  // wheel, from the last step.
  physics::vehicle_state vehicle_state(vehicle_id id) const;
  // The vehicle's description: as added, then as set_vehicle tuned it.
  vehicle_desc get_vehicle(vehicle_id id) const;
  // Tune a running vehicle, from the next step on. These may change:
  //   steering         max_steer, max_steer_rate
  //   the driver       max_accel, max_decel
  //   resistance       drag_coefficient, frontal_area, air_density, rolling_resistance
  //   the powertrain   engine_max_torque, engine_min_rpm, engine_max_rpm, shift_up_rpm,
  //                    shift_down_rpm, shift_up_rpm_light, shift_down_rpm_light,
  //                    engine_friction_curve, clutch_strength, converter_k,
  //                    converter_stall, converter_coupling, driveline_efficiency
  //   each axle        max_brake_torque, max_hand_brake_torque, suspension_frequency,
  //                    suspension_damping, anti_roll_stiffness (on an axle added with a
  //                    bar: one added without stays without)
  // The rest is fixed when the vehicle is added: a different value throws
  // cvc::physics_error naming it and changes nothing, as do values add_vehicle would
  // refuse. Like materials, a tuning is not part of save(): restore() needs a world
  // tuned the same.
  void set_vehicle(vehicle_id id, const vehicle_desc &d);

  // --- time -----------------------------------------------------------------

  step_report step();
  // n steps under one lock; the report is the last step's.
  step_report step(std::uint64_t n);
  // The steps due on `clock` since the last call; returns how many ran. The first
  // call anchors at the clock's current tick and runs none. A clock that went
  // backwards re-anchors (restoring this world to match is the caller's job).
  std::uint64_t follow(const cvc::world_clock &clock);
  std::uint64_t tick() const;
  double time() const; // tick * fixed_dt

  // The contact events of the last step, sorted by (body_a, sub_shape_a, body_b,
  // sub_shape_b, type) so their order never depends on worker threads.
  std::vector<contact_event> contacts() const;
  // The report of the last step (a default report before the first).
  step_report last_report() const;

  // The first surface on the segment from -> to, if any.
  std::optional<ray_hit> raycast(const vec3 &from, const vec3 &to) const;

  // --- state ----------------------------------------------------------------

  // The whole simulation state (bodies, velocities, sleep state, contact cache,
  // tick, origin). restore() needs a world holding the same bodies, created in the
  // same order, with the same materials and config.
  std::vector<std::uint8_t> save() const;
  void restore(const std::vector<std::uint8_t> &state);
  // A 64-bit digest of save()'s content.
  std::uint64_t state_hash() const;
  // Everything at this tick, under one lock (world_snapshot). It reads each dynamic and
  // kinematic body; the static ones are shared with the last snapshot unless one changed.
  world_snapshot snapshot() const;
  // The same for just these bodies, vehicles and fields, under one lock: `bodies` and
  // `static_bodies` hold the requested ones, an unknown id is left out, and the contacts
  // are not copied. What a world_binding reads for its named actors when no whole
  // snapshot is due, at a cost of those actors only.
  world_snapshot snapshot(const std::vector<body_id> &bodies,
                          const std::vector<vehicle_id> &vehicles,
                          const std::vector<field_id> &fields) const;

  // --- floating origin ------------------------------------------------------

  vec3 origin() const;
  // Move the origin without moving any body in world coordinates. The engine's
  // float positions change, so the run continues from slightly different numbers:
  // rebase at a deliberate point (a scene load, a long drive) and record it.
  void rebase(const vec3 &new_origin);

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_WORLD_H__
