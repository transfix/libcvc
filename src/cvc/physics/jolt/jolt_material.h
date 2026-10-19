/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc_material, the Jolt material the backend attaches to every shape and
// height-field cell.
//
// Its key function (the destructor) lives in jolt_material.cpp, the one file
// built with -fno-rtti on GCC/Clang. JPH::PhysicsMaterial's own key function is
// in libJolt, built without RTTI, so its typeinfo exists nowhere; a vtable for
// cvc_material emitted with RTTI on would reference it and fail to link. Emitting
// that vtable in a no-RTTI file means no file ever needs it.
//
// Keep -fno-rtti to that one file. GCC and Clang emit their own weak typeinfo for
// every std exception a no-RTTI file throws or catches. On macOS those copies
// bind every reference inside libcvc.dylib, so exceptions and boost::any stop
// matching across the library boundary; the earlier whole-backend -fno-rtti
// failed 1029 tests that way. jolt_material.cpp must not throw, catch or call
// anything inline that can throw.

#ifndef CVC_PHYSICS_JOLT_MATERIAL_H
#define CVC_PHYSICS_JOLT_MATERIAL_H

// clang-format off
#include <Jolt/Jolt.h> // first: every other Jolt header needs the configuration it sets
// clang-format on

#include <Jolt/Physics/Collision/PhysicsMaterial.h>
#include <cvc/physics/types.h>

namespace cvc {
namespace physics {
namespace detail {

class cvc_material final : public JPH::PhysicsMaterial {
public:
  cvc_material(material_id id, float friction, float restitution);
  ~cvc_material() override; // the key function: jolt_material.cpp
  material_id id;
  float friction;
  float restitution;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif
