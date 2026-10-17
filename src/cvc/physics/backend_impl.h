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
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

class world_backend {
public:
  virtual ~world_backend() = default;

  // A material is registered before any body uses it. set_material changes its
  // values between steps.
  virtual void add_material(material_id id, const material &m) = 0;
  virtual void set_material(material_id id, const material &m) = 0;

  // Apply world_config's runtime settings (world::reconfigure has checked the
  // rest are unchanged).
  virtual void reconfigure(const world_config &cfg) = 0;

  virtual body_id add_body(const body_desc &d) = 0;
  virtual void remove_body(body_id id) = 0;
  virtual bool contains(body_id id) const = 0;
  virtual std::size_t body_count() const = 0;

  virtual body_state state(body_id id) const = 0;
  virtual void set_state(body_id id, const body_state &s) = 0;
  virtual void add_force(body_id id, const vec3 &f) = 0;
  virtual void add_impulse(body_id id, const vec3 &j) = 0;
  virtual void move_kinematic(body_id id, const vec3 &p, const quat &q) = 0;

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
