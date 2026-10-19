/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Built with -fno-rtti on GCC/Clang (see jolt_vehicle_controller.h). It holds only what must
// own a vtable over a Jolt base: the constructors, the destructors (the key functions) and
// the factories, ConstructController and ConstructWheel. Everything else is in
// jolt_vehicle_energy.cpp, built with RTTI: a no-RTTI file that instantiates anything that can
// throw a std exception emits weak copies of its typeinfo, and on macOS those break exception and
// boost::any matching in all of libcvc (jolt_material.h). The energy ledger's first version here
// did, and failed 1082 tests there.

#include "jolt_vehicle_controller.h"

namespace cvc {
namespace physics {
namespace detail {

defined_wheel::defined_wheel(const JPH::WheelSettingsWV &s) : JPH::WheelWV(s) {
  mSuspensionLength = s.mSuspensionMaxLength; // where a wheel with no contact rests
  mContactPosition = JPH::RVec3::sZero();
  mContactPointVelocity = JPH::Vec3::sZero();
  mContactNormal = JPH::Vec3::sZero();
  mContactLongitudinal = JPH::Vec3::sZero();
  mContactLateral = JPH::Vec3::sZero();
  mAxlePlaneConstant = 0;
}

defined_wheel::~defined_wheel() = default;

ackermann_controller::ackermann_controller(const JPH::WheeledVehicleControllerSettings &s,
                                           JPH::VehicleConstraint &c)
    : JPH::WheeledVehicleController(s, c) {}

ackermann_controller::~ackermann_controller() = default;

JPH::Wheel *ackermann_controller::ConstructWheel(const JPH::WheelSettings &s) const {
  // jolt_backend.cpp gives every wheel a WheelSettingsWV, as the base class requires.
  return new defined_wheel(static_cast<const JPH::WheelSettingsWV &>(s));
}

ackermann_settings::ackermann_settings() = default;

ackermann_settings::~ackermann_settings() = default;

JPH::VehicleController *ackermann_settings::ConstructController(JPH::VehicleConstraint &c) const {
  return new ackermann_controller(*this, c);
}

} // namespace detail
} // namespace physics
} // namespace cvc
