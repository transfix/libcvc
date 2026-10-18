/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics vehicles (docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md, P2): a wheeled
// vehicle in the physics world, driven the way the navigator drives, by a steering
// angle and an acceleration.
//
//   const auto car = w.add_vehicle(cvc::physics::vehicle_desc::humvee(), {0, 0, 1});
//   w.drive(car, {0.1, 2.0});              // steer 0.1 rad left, accelerate 2 m/s^2
//   w.step();
//   const auto s = w.vehicle_state(car);   // speed, steer, gear, every wheel
//
// The vehicle is a chassis body (a box, world::state reads it) on wheels that the
// engine's vehicle model (Jolt's) rolls over the world's bodies. Frame: body-local x
// forward, y left, z up; a positive steer turns left.
//
// Steering. drive() and set_controls() name the BICYCLE angle: the steer at the
// centre of the front axle of a single-track model. An actuator moves it toward the
// command at no more than max_steer_rate, clamped to max_steer, and each steered
// wheel takes its own Ackermann angle from it: tan(delta_i) = (x_i - x_r) tan(delta) /
// (L - y_i tan(delta)), x_r the rear axle, L the wheelbase, y_i the wheel's lateral
// offset. So cot(delta_out) - cot(delta_in) = track / L exactly.
//
// The driver. drive() takes the navigator's command, an acceleration a_cmd along the
// commanded direction (reverse = backwards), and turns it into a throttle OR a brake,
// never both, plus the gear (the transmission shifts on its own): a feed-forward of
// a_cmd / max_accel or a_cmd / max_decel, corrected by the measured acceleration. A
// vehicle moving the wrong way brakes first; one at rest with no command holds on its
// brakes. set_controls() bypasses the driver with pedals of your own.
//
// Tires. Each tire's friction is a slip curve that peaks at 1.0 (and falls to
// tire_sliding at full slide) times the friction of the ground under it: the material
// of the terrain cell or mesh face it touches, the same table every plant reads (in a
// medium with a traction of its own, that traction instead).
//
// Resistance, applied to the chassis each step from the last step's wheels:
//   aero     -1/2 rho C_d A |u| u at the centre of mass, u the velocity relative to the
//            air: a flow field's (field.h) density and wind where one covers the
//            chassis, else air_density and no wind. (Flow fields do not drag a
//            vehicle's chassis as they do other bodies: this is its drag.)
//   rolling  C_rr N against each rolling wheel, N its load and C_rr the ground
//            material's rolling_resistance (rolling_resistance here when it has none).
//   medium   a wheel in a medium (field.h: snow, mud, sand) sinks until it carries its
//            load: z = (3 N / (2 b k sqrt(D)))^(2/3), the rigid-wheel Bekker relation
//            with n = 1 (b the tire width, D its diameter, k the medium's bearing
//            modulus; N its load, as the medium has felt it over 0.1 s), at most the
//            medium's depth. It rides on the compacted surface,
//            depth - z above the ground, and is held back by the compaction resistance
//            b k z^2 / 2 (Bekker).
// Both rolling forces fade smoothly to zero at rest.
//
// Vehicles need a backend with cap_vehicles (Jolt); the null backend refuses them.
// Standard library only, like the rest of cvc/physics.

#ifndef __CVC_PHYSICS_VEHICLE_H__
#define __CVC_PHYSICS_VEHICLE_H__

#include <cstdint>
#include <cvc/physics/body.h>
#include <cvc/physics/types.h>
#include <utility>
#include <vector>

namespace cvc {
namespace physics {

// One axle: a left and a right wheel (wheel 2a is the left one of axle a, 2a + 1 the
// right one). Positions are in the chassis's frame, about the body origin.
struct axle_desc {
  double x = 0.0;          // forward position of the axle
  double half_track = 0.8; // lateral offset of each wheel from the centre line
  double z = 0.0;          // height of the suspension's attachment
  double wheel_radius = 0.4;
  double wheel_width = 0.3;
  double wheel_inertia = 1.0;         // kg m^2
  double wheel_angular_damping = 0.0; // 1/s; rolling losses are rolling_resistance's
  bool steered = false;
  bool driven = false;
  double max_brake_torque = 1500.0;   // N m per wheel
  double max_hand_brake_torque = 0.0; // N m per wheel
  double suspension_min = 0.1;        // m below the attachment, fully raised
  double suspension_max = 0.3;        // m below the attachment, fully drooped
  double suspension_frequency = 1.5;  // Hz
  double suspension_damping = 0.5;    // ratio
  double anti_roll_stiffness = 0.0;   // N/m between the two wheels; 0 = none
};

// Where a vehicle's energy comes from (the energy ledger, vehicle_energy). The source is
// bookkeeping on the engine's shaft work: it does not change the dynamics, except that an
// empty battery or tank gives no more throttle.
struct energy_source {
  enum class kind : std::uint8_t { none, battery, fuel };
  kind type = kind::none;
  // A battery: the motor draws shaft work / motor_efficiency, and braking returns
  // regen_efficiency of the braking energy, at most max_regen_power.
  double capacity = 0.0;         // J (Wh x 3600)
  double initial_charge = 1.0;   // state of charge when added, in [0, 1]
  double motor_efficiency = 0.9; // shaft work per electrical energy, (0, 1]
  double regen_efficiency = 0.0; // [0, 1]
  double max_regen_power = 0.0;  // W
  // Fuel, on a Willans line: the engine burns (shaft work + its friction + the accessories)
  // / indicated_efficiency of chemical energy while fuelled, that is under throttle or at
  // idle; on the overrun (throttle shut above idle) a diesel cuts its fuel and burns none.
  // The friction is vehicle_desc::engine_friction_curve's torque x w. tank > 0 tracks what
  // is left.
  double tank = 0.0;                 // kg when added; 0 = not tracked
  double indicated_efficiency = 0.4; // indicated work per chemical energy, (0, 1]
  double heating_value = 42.6e6;     // J/kg, the lower heating value (diesel)
  double accessory_power = 0.0;      // W the engine drives besides the vehicle
};

struct vehicle_desc {
  vec3 half_extents{2.25, 1.0, 0.5};   // the chassis box
  vec3 center_of_mass{0.0, 0.0, -0.3}; // body-local
  double mass = 2000.0;                // kg, the whole vehicle
  std::vector<axle_desc> axles;

  // Steering: the bicycle angle's limit and slew rate.
  double max_steer = 0.6;      // rad
  double max_steer_rate = 0.5; // rad/s

  // Powertrain (the transmission shifts automatically between gear_ratios).
  double engine_max_torque = 500.0; // N m
  double engine_min_rpm = 800.0;
  double engine_max_rpm = 4000.0;
  double engine_inertia = 0.5; // kg m^2
  std::vector<double> gear_ratios{2.66, 1.78, 1.3, 1.0, 0.74};
  double reverse_ratio = -2.9;
  double differential_ratio = 3.42;
  double shift_up_rpm = 3200.0;   // at full throttle
  double shift_down_rpm = 1600.0; // at full throttle
  // The shift points with the throttle shut, between which the automatic interpolates with
  // the throttle, as a real one shifts early when driven gently. 0: the full-throttle ones.
  double shift_up_rpm_light = 0.0;
  double shift_down_rpm_light = 0.0;
  // The engine's (motor's) net torque at the flywheel (shaft) against its speed: (rpm,
  // fraction of engine_max_torque) points in ascending rpm, linear between them, up to
  // engine_max_rpm. Empty, Jolt's default: 0.8 at rest, 1.0 at two thirds of max rpm, 0.8 at
  // max.
  std::vector<std::pair<double, double>> torque_curve;
  // The engine's motoring friction (with its pumping and its own pumps) against its speed:
  // (rpm, N m) points in ascending rpm, linear between them and flat beyond; empty, none.
  // It brakes the vehicle when the throttle is shut. Under throttle the torque curve is
  // already net of it, so none applies, but the fuel's Willans line burns for it.
  std::vector<std::pair<double, double>> engine_friction_curve;
  // The coupling between engine and gearbox (Jolt's clutch): its torque is
  // clutch_strength x the slip speed, N m s/rad: a torque converter's coupling, which
  // converter_k below limits and multiplies.
  double clutch_strength = 10.0;
  // A torque converter's capacity (rpm per sqrt(N m), its K-factor; 0 = none, the clutch
  // alone). With its output held, a converter carries (engine rpm / converter_k)^2 of torque,
  // which is what an idling engine creeps the vehicle with; the clutch's linear coupling
  // applies only below that. Without it, Jolt's idle-speed clamp drives the wheels through the
  // clutch with clutch_strength x the slip, far more than the brakes hold.
  double converter_k = 0.0;
  // The converter's torque ratio: converter_stall at stall (the gearbox input still), falling
  // linearly to 1 at the speed ratio converter_coupling, where it couples. Used with
  // converter_k > 0.
  double converter_stall = 2.0;
  double converter_coupling = 0.85;
  // The fraction of the engine's torque the driveline (gearbox, transfer case, axles,
  // hubs) delivers to the wheels. The rest its gears lose (the ledger's drivetrain).
  double driveline_efficiency = 1.0;

  // The driver model's scale: the acceleration at full throttle and the deceleration
  // at full brake it plans with (m/s^2). The measured response corrects them.
  double max_accel = 3.0;
  double max_decel = 7.0;

  // Resistance.
  double drag_coefficient = 0.6;     // C_d
  double frontal_area = 3.0;         // m^2
  double air_density = 1.225;        // kg/m^3, outside flow fields
  double rolling_resistance = 0.015; // C_rr on ground whose material has none

  // Tires: the slip at peak friction (longitudinal slip ratio; lateral slip angle in
  // radians) and the fraction of the peak left at full slide.
  double tire_peak_slip = 0.08;
  double tire_peak_slip_angle = 0.087; // 5 degrees
  double tire_sliding = 0.95;

  energy_source energy;
  // Measure the energy ledger (vehicle_state::energy). Off, the engine steps unmeasured
  // through Jolt's own controller, which the measured step reproduces exactly.
  bool energy_ledger = true;

  // The distance from the rear axle (the rearmost unsteered one) to the frontmost
  // steered one, and the steered axle's track; 0 when the axles do not define them.
  double wheelbase() const;
  double track() const;

  // An M1097A2 HMMWV, from AM General's specification sheet and TM 9-2320-280-10
  // (vehicle.cpp cites each value): 2676 kg at curb, 3.30 m wheelbase, 1.82 m track,
  // 37 x 12.5R16.5 tires, a 6.5 L diesel through a four-speed automatic, four-wheel drive.
  static vehicle_desc humvee();
  // The UT-AV lab's RoboRacer: a Traxxas Ford Fiesta ST Rally chassis (1/10 scale, 4WD) with
  // an ARRMA 3800 Kv motor behind a VESC and a 3S LiPo, from the lab's configuration
  // (vehicle.cpp cites each value): 0.324 m wheelbase, 108 mm tires, a 10 m/s speed cap.
  static vehicle_desc roboracer();
};

// A vehicle handle; never reused.
struct vehicle_id {
  std::uint32_t value = 0xffffffffu;
  bool valid() const { return value != 0xffffffffu; }
  friend bool operator==(vehicle_id a, vehicle_id b) { return a.value == b.value; }
  friend bool operator!=(vehicle_id a, vehicle_id b) { return a.value != b.value; }
  friend bool operator<(vehicle_id a, vehicle_id b) { return a.value < b.value; }
};

// The navigator's command: what the driver model turns into pedals.
struct drive_command {
  double steer = 0.0; // bicycle angle (rad, + left); the actuator slews to it
  double accel = 0.0; // a_cmd (m/s^2) along the commanded direction; < 0 slows down
  bool reverse = false;
};

// Pedals, past the driver model (world::set_controls). Throttle and brake together are
// allowed here, as a real driver could.
struct vehicle_controls {
  double steer = 0.0;      // bicycle angle (rad); still through the actuator
  double throttle = 0.0;   // [0, 1]
  double brake = 0.0;      // [0, 1]
  double hand_brake = 0.0; // [0, 1]
  bool reverse = false;
  bool neutral = false; // the transmission in neutral: the engine does not drive or brake
};

struct wheel_state {
  double steer = 0.0;             // rad, + left
  double angular_velocity = 0.0;  // rad/s, + rolling forward
  double rotation = 0.0;          // rad
  double suspension_length = 0.0; // m from the attachment
  bool contact = false;
  vec3 contact_point;  // world metres (on a medium, its compacted surface)
  vec3 contact_normal; // world, unit
  body_id contact_body;
  material_id contact_material = 0;
  double load = 0.0;               // N, the suspension's push on the ground
  double longitudinal_slip = 0.0;  // slip ratio
  double lateral_slip = 0.0;       // slip angle, rad
  double friction = 0.0;           // the ground's mu the tire used (material or medium)
  double rolling_resistance = 0.0; // N, applied against the wheel's rolling
  double medium_depth = 0.0;       // m of medium over the ground under the wheel
  double sinkage = 0.0;            // m the wheel has sunk into it (below the fresh surface)
  double rut = 0.0;                // m of rut it found there (field::ruts), before this pass
  double medium_resistance = 0.0;  // N, its compaction resistance
};

// A vehicle's energy ledger, in joules since it was added (P3; docs/PHYSICS_API.md).
//
// The vehicle is the chassis, its wheels and its drivetrain, engine included: `supplied` is
// the engine's (motor's) shaft work, the integral of T w, and `idle` what holding the engine at
// its idle speed added. Every term is measured each engine sub-step:
//   supplied + idle + fields + drivetrain_split
//     = brake + tire + aero + rolling + medium + drivetrain + suspension + chassis_damping
//       + stored + residual
// drivetrain_split and the residual are the simulation's own energy errors: the first one
// it is known to make (Jolt's drivetrain step), measured exactly; the second, the rest.
struct vehicle_energy {
  double supplied = 0.0;        // the engine's shaft work
  double idle = 0.0;            // what holding the engine at idle speed added (< 0: the rev limit)
  double brake = 0.0;           // the brakes, a locked wheel's sliding included
  double tire = 0.0;            // tire slip, longitudinal and lateral
  double aero = 0.0;            // aerodynamic drag (< 0 when a tailwind pushed)
  double rolling = 0.0;         // rolling resistance
  double medium = 0.0;          // compacting snow, mud or sand
  double drivetrain = 0.0;      // engine and wheel damping, clutch slip, engine braking
  double suspension = 0.0;      // the suspension dampers
  double chassis_damping = 0.0; // the chassis body's own damping
  double fields = 0.0;          // work fields did on the chassis (buoyancy, a current)
  // What Jolt's drivetrain step adds: it solves engine, clutch and wheels with its estimate of
  // the brake and ground torques, then undoes the estimate before the friction solve, which
  // leaves each driven wheel dL tau / I more energy (dL the clutch's angular impulse, tau the
  // estimate's). A numerical error, measured exactly; > 0 adds energy.
  double drivetrain_split = 0.0;
  double stored = 0.0;   // the change in kinetic + potential + spring energy
  double residual = 0.0; // supplied + idle + fields + drivetrain_split - dissipated - stored
  // Stored now: the kinetic energy of the chassis, wheels and engine, the potential energy
  // above where it was first sampled, and the suspension springs'. As of the ledger's last
  // sample, at most one engine sub-step before the step's end.
  double kinetic = 0.0, potential = 0.0, spring = 0.0;
  // The source's side.
  double engine = 0.0;    // the engine's shaft work again: what the source paid for
  double regen = 0.0;     // braking energy returned to a battery
  double source = 0.0;    // drawn from the battery (electrical) or the fuel (chemical)
  double charge = 0.0;    // a battery's state of charge, [0, 1]
  double fuel_used = 0.0; // kg
  double fuel_left = 0.0; // kg, with a tank
  double distance = 0.0;  // m the chassis travelled
  double power = 0.0;     // W supplied in the last step
  // Source energy per kilometre (the shaft work without a source), Wh/km; 0 before it has
  // moved.
  double wh_per_km() const {
    const double e = source > 0.0 ? source : engine;
    return distance > 0.0 ? e / 3600.0 / (distance / 1000.0) : 0.0;
  }
  double dissipated() const {
    return brake + tire + aero + rolling + medium + drivetrain + suspension + chassis_damping;
  }
};

struct vehicle_state {
  body_id chassis;
  body_state body;
  double speed = 0.0;                                   // m/s along the chassis's forward axis
  double steer = 0.0;                                   // the actuator's bicycle angle now (rad)
  double steer_command = 0.0;                           // what it slews toward
  double throttle = 0.0, brake = 0.0, hand_brake = 0.0; // applied this step
  int gear = 0;                                         // -1 reverse, 0 neutral, 1.. forward
  double engine_rpm = 0.0;
  double accel_trim = 0.0; // m/s^2, the driver model's integral trim (vehicle_set's driver)
  vec3 aero_force;         // N, the drag applied
  vec3 air_velocity;       // m/s, the air's (a flow field's wind, or still)
  std::vector<wheel_state> wheels;
  vehicle_energy energy; // its ledger since it was added
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_VEHICLE_H__
