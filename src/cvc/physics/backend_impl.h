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

#include <cstdint>
#include <cvc/physics/world.h>
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
