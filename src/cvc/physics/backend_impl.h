/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The interface between cvc::physics::world and an engine. Private: not
// installed. Standard library only, because the Jolt backend compiles as its own
// target without libcvc's other include paths (src/cvc/CMakeLists.txt).
//
// world validates arguments and owns the tick, the material table, the contact
// list and the origin; a backend owns the bodies and the engine. Backends report
// errors as std exceptions; world turns them into cvc::physics_error.

#ifndef __CVC_PHYSICS_BACKEND_IMPL_H__
#define __CVC_PHYSICS_BACKEND_IMPL_H__

#include <cmath>
#include <cstdint>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// A shape as an engine takes it. world.cpp checks the public shapes and flattens
// the ones that reference the world's geometry (physics::mesh, physics::terrain)
// into these, with every material resolved to a world material id, so a backend
// never sees cvc::geometry and stays standard-library only.
struct mesh_data {
  std::vector<vec3> vertices;
  std::vector<std::uint32_t> indices;          // three per triangle
  std::vector<material_id> triangle_materials; // one per triangle
};

// Sample (r, c) at origin + (c * dx, r * dy, heights[r * cols + c]); NaN is a hole.
// Each cell's material is palette[cells[r * (cols - 1) + c]] (cells empty: entry 0).
struct terrain_data {
  int rows = 0, cols = 0;
  double dx = 1.0, dy = 1.0;
  vec3 origin;
  const float *heights = nullptr; // rows * cols; borrowed for the add_body call only
  std::vector<std::uint8_t> cells;
  std::vector<material_id> palette; // 1..256 world materials
  double max_error = 0.005;
};

using engine_shape = std::variant<sphere, box, convex_hull, mesh_data, terrain_data>;

// A vehicle as the backend builds it (vehicle.h; world.cpp has checked the description).
struct vehicle_setup {
  const vehicle_desc *desc = nullptr;
  vec3 position;
  quat rotation;
};

// A vehicle as the backend reports it after a step.
struct vehicle_readback {
  int gear = 0;
  double engine_rpm = 0.0;
  struct wheel {
    wheel_state state;           // the engine's view; the facade fills in the resistances
    vec3 longitudinal;           // the tire's forward direction on the ground (world, unit)
    double medium_modulus = 0.0; // k of the medium under it, N/m^3 (0: none)
    std::uint32_t medium_field = 0xffffffffu; // which medium field
  };
  std::vector<wheel> wheels;
  // The energy ledger's measurements over the step (J; vehicle.h), and what is stored as of
  // its last sample. `measured` is false for a vehicle without a ledger.
  struct energy_step {
    bool measured = false, sampled = false;
    double engine = 0.0, idle = 0.0, supplied = 0.0, drivetrain = 0.0, split = 0.0, brake = 0.0,
           tire = 0.0;
    double split_braked = 0.0; // the part of split in braked wheels
    double indicated = 0.0;    // the fuelled engine's work: shaft, friction, accessories
    double work[4] = {0.0, 0.0, 0.0, 0.0}; // aero, rolling, medium, fields on the chassis
    double suspension = 0.0, chassis_damping = 0.0, stored = 0.0, distance = 0.0;
    double kinetic = 0.0, potential = 0.0, spring = 0.0;
  } energy;
};

// The loads the facade puts on a vehicle's chassis in a step, at its centre of mass: aero,
// rolling, medium and fields, in that order.
struct vehicle_loads {
  vec3 force[4];
  vec3 torque[4];
};

// The medium under a wheel's ground contact (field.h): how deep it lies above the contact
// (m, along `up`), its bearing modulus and its traction (0: the ground's).
struct wheel_medium {
  double depth = 0.0;
  double modulus = 0.0;
  double traction = 0.0;
  double rut = 0.0;                  // m already pressed down there (field::ruts)
  std::uint32_t field = 0xffffffffu; // the medium field's id
};
// Asked during the step, possibly on worker threads: it must only read.
using medium_probe = std::function<bool(const vec3 &contact, const vec3 &up, wheel_medium &out)>;

// The sinkage of a rigid wheel (diameter, width) carrying `load` in a medium of bearing modulus
// k: Bekker with n = 1, load = (2/3) b k sqrt(D) z^(3/2).
inline double bekker_sinkage(double load, double width, double diameter, double modulus) {
  if (!(load > 0.0) || !(width > 0.0) || !(diameter > 0.0) || !(modulus > 0.0))
    return 0.0;
  return std::pow(3.0 * load / (2.0 * width * modulus * std::sqrt(diameter)), 2.0 / 3.0);
}

class world_backend {
public:
  // Defined in world.cpp: the key function, so this interface's vtable and RTTI are
  // emitted there and never in the Jolt backend's TU, which is built without RTTI.
  virtual ~world_backend();

  // A material is registered before any body uses it. set_material changes its
  // values between steps.
  virtual void add_material(material_id id, const material &m) = 0;
  virtual void set_material(material_id id, const material &m) = 0;

  // Apply world_config's runtime settings (world::reconfigure has checked the
  // rest are unchanged).
  virtual void reconfigure(const world_config &cfg) = 0;

  // `shape` is d.shape checked and flattened (d.shape itself is not read).
  virtual body_id add_body(const body_desc &d, const engine_shape &shape) = 0;
  virtual void remove_body(body_id id) = 0;
  virtual bool contains(body_id id) const = 0;
  virtual std::size_t body_count() const = 0;

  virtual body_state state(body_id id) const = 0;
  // The states of `ids`, all known, in order: world::snapshot's batch read. The world
  // holds its lock, so no step runs meanwhile.
  virtual void states(const std::vector<body_id> &ids, std::vector<body_state> &out) const {
    out.clear();
    out.reserve(ids.size());
    for (const body_id id : ids)
      out.push_back(state(id));
  }
  virtual void set_state(body_id id, const body_state &s) = 0;
  virtual void add_force(body_id id, const vec3 &f) = 0;
  virtual void add_impulse(body_id id, const vec3 &j) = 0;
  virtual void move_kinematic(body_id id, const vec3 &p, const quat &q) = 0;

  // For the field step (world.cpp), which runs before each step. The body's shape
  // volume, its centre of mass and which of `points` lie inside it, all in the
  // body's local frame about its origin. false if the backend cannot say; the
  // fields then leave the body alone.
  virtual bool shape_interior(body_id id, const std::vector<vec3> &points,
                              std::vector<std::uint8_t> &inside, double &volume,
                              vec3 &center_of_mass) const {
    (void)id, (void)points, (void)inside, (void)volume, (void)center_of_mass;
    return false;
  }
  // A force through the centre of mass and a torque, through the next step only.
  // Never wakes a sleeping body.
  virtual void add_field_load(body_id id, const vec3 &force, const vec3 &torque) = 0;
  // Wake a sleeping dynamic body.
  virtual void wake(body_id id) = 0;
  // A dynamic body's mass as the engine holds it (0 if it does not say).
  virtual double body_mass(body_id id) const {
    (void)id;
    return 0.0;
  }
  // world::set_body, checked by the facade: the material (a convex body's), and for a
  // moving body its damping, sleeping, and with `new_mass` its mass.
  virtual void set_body_settings(body_id id, const body_settings &s, bool new_mass) {
    (void)id, (void)s, (void)new_mass;
  }

  // Vehicles (vehicle.h). add_vehicle builds the chassis body and its wheels, returning the
  // chassis and a handle; false if the backend has no vehicles (cap_vehicles).
  virtual bool add_vehicle(const vehicle_setup &s, body_id &chassis, std::uint32_t &handle) {
    (void)s, (void)chassis, (void)handle;
    return false;
  }
  // Removes the wheels and the chassis.
  virtual void remove_vehicle(std::uint32_t handle) { (void)handle; }
  // The next step's inputs: each wheel's steer (rad, + left) and last load (N, for its
  // sinkage), and the pedals (forward in [-1, 1]: < 0 drives backwards).
  virtual void set_vehicle_inputs(std::uint32_t handle, const std::vector<double> &steer,
                                  const std::vector<double> &load, double forward, double brake,
                                  double hand_brake, bool neutral) {
    (void)handle, (void)steer, (void)load, (void)forward, (void)brake, (void)hand_brake,
        (void)neutral;
  }
  virtual void read_vehicle(std::uint32_t handle, vehicle_readback &out) const {
    (void)handle, (void)out;
  }
  // The step's chassis loads, for the energy ledger to charge to their causes.
  virtual void set_vehicle_loads(std::uint32_t handle, const vehicle_loads &l) {
    (void)handle, (void)l;
  }
  // A vehicle's state the engine's own save does not hold (the ledger's last sample), for
  // world::save / restore.
  virtual void save_vehicle(std::uint32_t handle, std::string &out) const {
    (void)handle, (void)out;
  }
  virtual void restore_vehicle(std::uint32_t handle, const std::string &in, std::size_t &pos) {
    (void)handle, (void)in, (void)pos;
  }
  // The engine's share of world::set_vehicle: the powertrain, and each wheel's brakes,
  // suspension spring and anti-roll bar, from `d` (checked by the facade).
  virtual void tune_vehicle(std::uint32_t handle, const vehicle_desc &d) { (void)handle, (void)d; }
  // The media the wheels sink into, and how deep the deepest is: a wheel looks that much
  // further for the ground under one.
  virtual void set_medium_probe(medium_probe probe) { (void)probe; }
  virtual void set_medium_reach(double reach) { (void)reach; }

  // One fixed_dt step. Appends this step's contact events (unsorted) to `out`.
  virtual void step(std::vector<contact_event> &out, step_report &report) = 0;

  virtual std::optional<ray_hit> raycast(const vec3 &from, const vec3 &to) const = 0;

  // The engine's own state bytes, and restoring them. `origin` is the origin the
  // bytes were saved under.
  virtual std::string save_engine() const = 0;
  virtual void restore_engine(const std::string &bytes, const vec3 &origin) = 0;

  // Move the origin, keeping every body's world position.
  virtual void rebase(const vec3 &new_origin) = 0;
};

std::unique_ptr<world_backend> make_null_backend(const world_config &cfg);
std::uint32_t null_capabilities();

#ifdef CVC_WITH_JOLT
// Defined in physics/jolt/jolt_backend.cpp.
std::unique_ptr<world_backend> make_jolt_backend(const world_config &cfg);
std::uint32_t jolt_capabilities();
std::string jolt_version();
#endif

} // namespace detail
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_BACKEND_IMPL_H__
