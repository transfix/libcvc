/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// ackermann_controller and its settings, the Jolt vehicle controller the backend gives every
// vehicle, and energy_tally, what it measures for the energy ledger (vehicle.h).
//
// Jolt steers every wheel from one input; per-wheel Ackermann needs each wheel's own angle, so
// they are set after the base controller's own pass.
//
// The energy ledger: with a tally attached, PostCollide and the friction solve are this file's
// copies of Jolt 5.6.0's (WheeledVehicleController.cpp), with measurements between their steps,
// so the engine's work, the idle speed's, the clutch's, the brakes' and the tires' are exact.
// The copies change no arithmetic: a measured run's state hash equals an unmeasured one's
// (physics_energy_test). Recheck them against Jolt's on every Jolt upgrade.
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
#include <cstdint>
#include <vector>

namespace cvc {
namespace physics {
namespace detail {

// One vehicle's energy measurements. The backend sizes the per-wheel arrays and sets the inputs
// before each step; the controller samples the vehicle at the start of each engine sub-step's
// PostCollide and adds each closed interval's terms to the sums; the backend reads the sums
// after the step and zeroes them. Joules throughout.
struct energy_tally {
  // Inputs, set by the backend before each step.
  double gravity[3] = {0.0, 0.0, -9.80665};
  double origin[3] = {0.0, 0.0, 0.0}; // the floating origin: potential energy is in world z
  // The loads the facade applies to the chassis this step (force xyz, torque xyz about the
  // centre of mass): aero, rolling, medium, fields.
  double loads[4][6] = {};

  // Sums since the backend last read them.
  double engine = 0.0;       // the engine torque's work
  double idle = 0.0;         // what Jolt's idle-speed clamp added to the engine
  double supplied = 0.0;     // positive work through the clutch
  double drivetrain = 0.0;   // engine and wheel damping, clutch slip, engine braking
  double split = 0.0;        // what the split drivetrain step adds (vehicle_energy)
  double split_braked = 0.0; // the part of it in braked wheels
  double indicated = 0.0;    // the fuelled engine's work (energy_source's Willans line)
  double brake = 0.0;
  double tire = 0.0;
  double work[4] = {};     // the loads' work on the chassis (aero, rolling, medium, fields)
  double suspension = 0.0; // the dampers
  double chassis_damping = 0.0;
  double stored = 0.0;   // the change in kinetic + potential + spring energy
  double distance = 0.0; // m

  // The last sample: the start of the interval in progress.
  bool sampled = false;
  double h = 0.0; // s, the interval's length (the sub-step that follows the sample)
  double kinetic = 0.0, potential = 0.0, spring = 0.0;
  double v[3] = {}, w[3] = {};      // chassis velocity (of its centre of mass) and angular velocity
  double rot_kinetic = 0.0;         // the chassis's rotational kinetic energy
  double interval_loads[4][6] = {}; // the loads acting through the interval
  std::vector<double> compression;  // m per wheel, -1 without contact
  std::vector<double> damping;      // N s/m per wheel: the suspension damper's coefficient
  std::vector<double> next_compression, next_damping; // the sample being taken

  // Scratch for the measured steps, per wheel.
  std::vector<float> omega;
  bool lateral_warm_start = false; // the first friction solve of a sub-step still to account

  void resize(std::size_t wheels) {
    compression.assign(wheels, -1.0);
    damping.assign(wheels, 0.0);
    next_compression.assign(wheels, -1.0);
    next_damping.assign(wheels, 0.0);
    omega.assign(wheels, 0.0f);
  }
  void clear_sums() {
    engine = idle = supplied = drivetrain = split = split_braked = indicated = brake = tire =
        suspension = chassis_damping = stored = distance = 0.0;
    for (double &x : work)
      x = 0.0;
  }
};

class ackermann_controller final : public JPH::WheeledVehicleController {
public:
  ackermann_controller(const JPH::WheeledVehicleControllerSettings &s, JPH::VehicleConstraint &c);
  ~ackermann_controller() override;          // the key function: jolt_vehicle_controller.cpp
  const std::vector<float> *steer = nullptr; // rad, per wheel; the facade's, when set
  energy_tally *tally = nullptr;             // measure the energy ledger, when set
  // The share of the engine's torque Jolt is given (vehicle_desc::driveline_efficiency):
  // the engine supplies 1 / this of the work Jolt's torque does, its gears lose the rest.
  double driveline_efficiency = 1.0;
  // The engine's motoring friction (vehicle_desc::engine_friction_curve, (rpm, N m)) and
  // accessory power (W): the closed-throttle drag, and the fuel's Willans line.
  std::vector<std::pair<double, double>> engine_friction;
  double accessory_power = 0.0;
  double friction_torque(double rpm) const; // N m, flat beyond the curve; 0 with none
  // The anti-roll bars' stiffness, N/m per axle (0 for none), for their spring energy.
  std::vector<double> anti_roll;

protected:
  void PreCollide(float dt, JPH::PhysicsSystem &ps) override;
  void PostCollide(float dt, JPH::PhysicsSystem &ps) override;
  bool SolveLongitudinalAndLateralConstraints(float dt) override;

private:
  void sample(float dt);
  void measured_post_collide(float dt);
  bool measured_friction(float dt);
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
