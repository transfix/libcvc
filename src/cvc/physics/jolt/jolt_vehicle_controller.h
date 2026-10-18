/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// ackermann_controller and its settings, the Jolt vehicle controller the backend gives every
// vehicle.
//
// Jolt steers every wheel from one input; per-wheel Ackermann needs each wheel's own angle, so
// they are set after the base controller's own pass.
//
// Their key functions (the destructors) live in jolt_vehicle_controller.cpp, built with
// -fno-rtti on GCC/Clang, for the reason jolt_material.h gives: the key functions of
// JPH::WheeledVehicleController and its settings are in libJolt, built without RTTI, so their
// typeinfo exists nowhere, and a vtable emitted with RTTI on would reference it. The same rules
// hold: jolt_vehicle_controller.cpp must not throw, catch or call anything inline that can
// throw.

#ifndef CVC_PHYSICS_JOLT_VEHICLE_CONTROLLER_H
#define CVC_PHYSICS_JOLT_VEHICLE_CONTROLLER_H

// clang-format off
#include <Jolt/Jolt.h> // first: every other Jolt header needs the configuration it sets
// clang-format on

#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

class ackermann_controller final : public JPH::WheeledVehicleController {
public:
  ackermann_controller(const JPH::WheeledVehicleControllerSettings &s, JPH::VehicleConstraint &c);
  ~ackermann_controller() override;          // the key function: jolt_vehicle_controller.cpp
  const std::vector<float> *steer = nullptr; // rad, per wheel; the facade's, when set

protected:
  void PreCollide(float dt, JPH::PhysicsSystem &ps) override;
};

class ackermann_settings final : public JPH::WheeledVehicleControllerSettings {
public:
  ackermann_settings();
  ~ackermann_settings() override; // the key function: jolt_vehicle_controller.cpp
  JPH::VehicleController *ConstructController(JPH::VehicleConstraint &c) const override;
};

} // namespace detail
} // namespace physics
} // namespace cvc

#endif
