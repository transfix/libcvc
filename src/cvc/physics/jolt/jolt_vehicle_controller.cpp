/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Built with -fno-rtti on GCC/Clang (see jolt_vehicle_controller.h): no throw, no catch,
// nothing that can throw.

#include "jolt_vehicle_controller.h"

#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

namespace cvc {
namespace physics {
namespace detail {

ackermann_controller::ackermann_controller(const JPH::WheeledVehicleControllerSettings &s,
                                           JPH::VehicleConstraint &c)
    : JPH::WheeledVehicleController(s, c) {}

ackermann_controller::~ackermann_controller() = default;

void ackermann_controller::PreCollide(float dt, JPH::PhysicsSystem &ps) {
  JPH::WheeledVehicleController::PreCollide(dt, ps);
  if (!steer)
    return;
  JPH::Wheels &wheels = mConstraint.GetWheels();
  for (size_t i = 0; i < wheels.size() && i < steer->size(); ++i)
    wheels[i]->SetSteerAngle((*steer)[i]);
}

ackermann_settings::ackermann_settings() = default;

ackermann_settings::~ackermann_settings() = default;

JPH::VehicleController *ackermann_settings::ConstructController(JPH::VehicleConstraint &c) const {
  return new ackermann_controller(*this, c);
}

} // namespace detail
} // namespace physics
} // namespace cvc
