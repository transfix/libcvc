/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "vehicle_step.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/exception.h>
#include <string>

namespace cvc {
namespace physics {

// --- presets --------------------------------------------------------------------

double vehicle_desc::wheelbase() const {
  double rear = 0.0, front = 0.0;
  bool have_rear = false, have_front = false;
  for (const axle_desc &a : axles) {
    if (!a.steered && (!have_rear || a.x < rear)) {
      rear = a.x;
      have_rear = true;
    }
    if (a.steered && (!have_front || a.x > front)) {
      front = a.x;
      have_front = true;
    }
  }
  return have_rear && have_front ? front - rear : 0.0;
}

double vehicle_desc::track() const {
  for (const axle_desc &a : axles)
    if (a.steered)
      return 2.0 * a.half_track;
  return 0.0;
}

// An M1097A2 HMMWV (the heavy variant). Values marked [AMG] are from AM General's M1097A2
// specification sheet (www.amgeneral.com, M1097A2R, AMP05072018DOM); [TM] from TM 9-2320-280-10,
// table 1-15; [DTIC] from the Army's dynamometer tests of its 6.5 L engine (TARDEC TFLRF,
// Frame and Blanks, DTIC ADA449196); [derived] follow from those; [est] are engineering
// estimates, to be replaced by measurements. physics_vehicle_test and physics_energy_test check
// the result against the field figures (acceleration, cruising range).
vehicle_desc vehicle_desc::humvee() {
  vehicle_desc d;
  d.half_extents = {2.42, 1.09, 0.45}; // [AMG] 4.84 m x 2.18 m overall (the body box)
  // [AMG] the centre of gravity at curb: 0.742 m above the ground, 1.537 m behind the front
  // axle, so 0.113 m ahead of the wheelbase's middle; the chassis origin rides about 1.02 m up.
  d.center_of_mass = {0.113, 0.0, -0.28};
  d.mass = 2676.0; // [AMG] curb weight; the GVW is 4672 kg (1996 kg payload)
  axle_desc a;
  a.half_track = 0.91; // [AMG] 1.82 m track
  a.z = -0.2;
  a.wheel_radius = 0.47;  // [AMG] 37 x 12.5R-16.5 tires: 0.94 m across
  a.wheel_width = 0.3175; // [AMG] 12.5 in
  a.wheel_inertia = 6.0;  // [est] a 36 kg tire and 20 kg two-piece wheel
  a.driven = true;        // [AMG] full-time four-wheel drive
  // [derived] Front-biased, as the dual-circuit system's proportioning is: at 0.8 g the [AMG]
  // centre of gravity puts 71% of the load on the front axle, so each axle's brakes reach the
  // friction limit together (about 3.5 and 1.4 kN m a wheel; the inboard discs act through the
  // 1.92 hubs).
  a.suspension_min = 0.2;
  a.suspension_max = 0.5;
  a.suspension_frequency = 1.3;    // [est]
  a.suspension_damping = 0.5;      // [est]
  a.anti_roll_stiffness = 15000.0; // [est]
  axle_desc front = a, rear = a;
  front.x = 1.65; // [AMG] 3.30 m wheelbase
  front.steered = true;
  front.max_brake_torque = 3600.0;
  rear.x = -1.65;
  rear.max_brake_torque = 1500.0;
  rear.max_hand_brake_torque = 3000.0; // [AMG] parking brake on the rear service brakes; [est]
  d.axles = {front, rear};
  d.max_steer = 0.457;     // [derived] the 7.62 m turning radius [AMG]: atan(3.30 / (7.62 - 0.91))
  d.max_steer_rate = 0.35; // [est] geometry-scene-gen's VehicleProfile("humvee")
  // [AMG] GEP 6.5 L V8, naturally aspirated: 119 kW at 3400 rpm, 393 N m at 1700 rpm.
  d.engine_max_torque = 393.0;
  d.engine_min_rpm = 700.0;  // [est] idle
  d.engine_max_rpm = 3600.0; // [est] the governor's no-load speed, above the 3400 rpm rating
  // [AMG] the two rated points; [est] the curve's shape between them, and the governor's
  // cut-off. 334 N m at 3400 rpm is the 119 kW rating.
  d.torque_curve = {{0.0, 0.70},    {700.0, 0.76},  {1000.0, 0.87}, {1700.0, 1.0},
                    {2500.0, 0.95}, {3400.0, 0.85}, {3600.0, 0.0}};
  d.engine_inertia = 0.6; // [est] crank, flexplate and converter impeller
  // [derived] Motoring friction, from the 6.5 L's ISO 8178 dynamometer modes [DTIC]: at
  // rated speed its fuel flow is a Willans line, 0.331 lb/hp-h over 20.3 lb/h at no load,
  // which is an indicated efficiency of 0.42 and 128 N m of friction; the intermediate
  // speed's 10% mode leaves 8.2 lb/h at no load (87 N m), and idle burns 2.6 lb/h (78 N m).
  // [est] the modes' speeds, which the table does not give: 3400 rpm rated, 2040 rpm
  // intermediate (ISO 8178's 60% of rated), 700 rpm idle.
  d.engine_friction_curve = {{700.0, 78.0}, {2040.0, 87.0}, {3400.0, 128.0}};
  d.clutch_strength = 40.0; // [est] a converter coupling that slips ~5% at peak torque
  // [est] the 4L80-E's converter stalling near 1900 rpm against the 393 N m peak: 53 N m of
  // creep at idle
  d.converter_k = 95.8;
  d.converter_stall = 2.0;     // [est] the converter's torque ratio at stall
  d.converter_coupling = 0.85; // [est] its coupling point
  // [est] gearbox 0.90, transfer case 0.98, two hypoid axles 0.96, geared hubs 0.97
  d.driveline_efficiency = 0.82;
  d.gear_ratios = {2.48, 1.48, 1.0, 0.75}; // [AMG] four-speed automatic
  d.reverse_ratio = -2.08;                 // [AMG]
  d.differential_ratio = 2.73 * 1.92;      // [AMG] hypoid 2.73 x geared hubs 1.92; transfer 1:1
  d.shift_up_rpm = 3200.0;                 // [est] at full throttle
  d.shift_down_rpm = 1400.0;               // [est]
  d.shift_up_rpm_light = 1600.0;           // [est] gently: early upshifts
  d.shift_down_rpm_light = 900.0;
  d.max_accel = 2.5;
  d.max_decel = 7.5;
  d.drag_coefficient = 0.7;     // [est] a box with an open bed; with 3.5 m^2, the 2.45 m^2 drag
  d.frontal_area = 3.5;         // area the Hummer H2's published 2.46 m^2 suggests
  d.rolling_resistance = 0.015; // [est] radial cross-country tires on a hard surface
  // [AMG] a 95 L (25 gal) tank of diesel (0.85 kg/L), burnt on the [DTIC] Willans line above;
  // physics_energy_test checks the cruising range against [TM]'s.
  d.energy.type = energy_source::kind::fuel;
  d.energy.tank = 95.0 * 0.85;
  d.energy.indicated_efficiency = 0.42; // [derived] the [DTIC] Willans line's slope
  d.energy.heating_value = 42.6e6;
  d.energy.accessory_power = 2500.0; // [est] the fan, alternator and steering pump
  return d;
}

// The UT-AV lab's RoboRacer. Values marked [lab] are from the lab's configuration
// (ut-av.pages.dev hardware/vesc_motor_driver; ut-amrl/ut_automata config/vesc.lua); [trx] from
// Traxxas' listing for the Ford Fiesta ST Rally (74154-4); [doc31] from the scope's spec table
// (cvc-engagement-docs dbg-technical/31, D5), which read the lab's URDF and nav2 config;
// [derived] follow from those; [est] are estimates to replace with measurements of the car.
vehicle_desc vehicle_desc::roboracer() {
  vehicle_desc d;
  d.half_extents = {0.2675, 0.1405, 0.05}; // [trx] 535 mm long, 281 mm wide
  d.center_of_mass = {0.0, 0.0, -0.02};    // [doc31] a centre of gravity about 0.08 m up
  d.mass = 3.5; // [doc31] 3.3-3.8 kg as built ([trx] 2.63-2.77 kg stock); [est] weigh it
  axle_desc a;
  a.half_track = 0.119; // [doc31] 0.238 m between wheel centres (281 mm less a tire)
  a.z = 0.0;
  a.wheel_radius = 0.054; // [doc31] 108 x 43 mm rally tires (the URDF suggests 90 mm: measure)
  a.wheel_width = 0.043;
  a.wheel_inertia = 1.4e-4;  // [est] a 70 g wheel and tire
  a.driven = true;           // [doc31] shaft-driven 4WD
  a.max_brake_torque = 0.57; // [est] the VESC's motor braking, ~60 A through the 15.15:1
  a.suspension_min = 0.01;
  a.suspension_max = 0.04;
  a.suspension_frequency = 4.0; // [est]
  a.suspension_damping = 0.6;   // [est]
  a.anti_roll_stiffness = 50.0; // [est]
  axle_desc front = a, rear = a;
  front.x = 0.162; // [lab] 0.324 m wheelbase ([trx] 12.8 in)
  front.steered = true;
  rear.x = -0.162;
  d.axles = {front, rear};
  d.max_steer = 0.425;    // [lab] max_steering_angle
  d.max_steer_rate = 3.0; // [doc31] the Traxxas 2056 servo, 0.23 s/60 deg at 6 V, slower at ~5 V
  // [lab] an ARRMA 3800 Kv brushless motor (4-pole). [est] the VESC's motor current limit,
  // ~60 A: 60 x 9.549 / 3800 = 0.15 N m.
  d.engine_max_torque = 0.15;
  d.engine_min_rpm = 0.0; // a motor does not idle
  // [lab] erpm_speed_limit 53740 (10 m/s): 26870 rpm on a 4-pole motor. Flat torque to the
  // VESC's speed limit.
  d.engine_max_rpm = 26870.0;
  d.torque_curve = {{0.0, 1.0}, {25500.0, 1.0}, {26870.0, 0.0}};
  d.engine_inertia = 2.0e-6;    // [est] a 540-size inrunner's rotor and the pinion
  d.clutch_strength = 10.0;     // direct drive: stiff for the motor's inertia
  d.driveline_efficiency = 0.9; // [est] spur and pinion, the gearbox and two diffs
  d.gear_ratios = {1.0};
  d.reverse_ratio = -1.0;
  // [derived] speed_to_erpm_gain 5356 [lab] is 2678 motor rpm per m/s; on 0.054 m tires that is
  // 15.15 motor turns per wheel turn (17T pinion, 83T spur [lab], and the Traxxas gearbox).
  d.differential_ratio = 15.15;
  d.shift_up_rpm = 26000.0;
  d.shift_down_rpm = 1.0;
  d.max_accel = 6.0;           // [lab] max_acceleration
  d.max_decel = 6.0;           // [lab] max_deceleration
  d.drag_coefficient = 0.5;    // [est]
  d.frontal_area = 0.035;      // [est] the body's 281 mm x ~125 mm
  d.rolling_resistance = 0.02; // [est] foam-insert rally tires on a lab floor
  // [doc31] a 3S LiPo (9.6-12.6 V [lab]); [est] 5000 mAh (55.5 Wh: read the pack's label), 85%
  // from pack to shaft, and the VESC's regenerative braking at 70% up to 150 W.
  d.energy.type = energy_source::kind::battery;
  d.energy.capacity = 55.5 * 3600.0;
  d.energy.motor_efficiency = 0.85;
  d.energy.regen_efficiency = 0.7;
  d.energy.max_regen_power = 150.0;
  return d;
}

namespace detail {

namespace {

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const vec3 &v) { return std::sqrt(dot(v, v)); }

vec3 rotate(const quat &q, const vec3 &v) {
  const double tx = 2 * (q.y * v.z - q.z * v.y), ty = 2 * (q.z * v.x - q.x * v.z),
               tz = 2 * (q.x * v.y - q.y * v.x);
  return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
          v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

void need(bool ok, const std::string &what) {
  if (!ok)
    throw cvc::physics_error("add_vehicle: " + what);
}
bool positive(double v) { return v > 0.0 && std::isfinite(v); }
bool finite(const vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Throws cvc::physics_error, prefixed with `op`, for a description add_vehicle refuses.
void validate(const vehicle_desc &d, const char *op = "add_vehicle") {
  const auto need = [op](bool ok, const std::string &what) {
    if (!ok)
      throw cvc::physics_error(std::string(op) + ": " + what);
  };
  need(positive(d.half_extents.x) && positive(d.half_extents.y) && positive(d.half_extents.z),
       "the chassis's half extents must be finite and > 0");
  need(finite(d.center_of_mass), "the centre of mass must be finite");
  need(positive(d.mass), "mass must be finite and > 0");
  need(!d.axles.empty(), "a vehicle needs at least one axle");
  bool steered = false, driven = false;
  for (const axle_desc &a : d.axles) {
    need(std::isfinite(a.x) && std::isfinite(a.z) && positive(a.half_track),
         "an axle's position must be finite and its half track > 0");
    need(positive(a.wheel_radius) && positive(a.wheel_width) && positive(a.wheel_inertia),
         "a wheel's radius, width and inertia must be finite and > 0");
    need(a.wheel_angular_damping >= 0.0 && a.max_brake_torque >= 0.0 &&
             a.max_hand_brake_torque >= 0.0 && a.anti_roll_stiffness >= 0.0,
         "damping, brake torques and anti-roll stiffness must be >= 0");
    need(a.suspension_min >= 0.0 && a.suspension_max > a.suspension_min,
         "the suspension needs 0 <= min < max");
    need(positive(a.suspension_frequency) && a.suspension_damping >= 0.0,
         "the suspension's frequency must be > 0 and its damping >= 0");
    steered = steered || a.steered;
    driven = driven || a.driven;
  }
  need(driven, "no axle is driven");
  need(!steered || d.wheelbase() > 0.0,
       "a steered axle needs an unsteered axle behind it (the rear axle Ackermann turns about)");
  need(d.max_steer > 0.0 && d.max_steer < 1.5, "max_steer must be in (0, 1.5) rad");
  need(positive(d.max_steer_rate), "max_steer_rate must be finite and > 0");
  need(positive(d.engine_max_torque) && positive(d.engine_max_rpm) && d.engine_min_rpm >= 0.0 &&
           d.engine_min_rpm < d.engine_max_rpm && positive(d.engine_inertia),
       "the engine needs a torque, an inertia and 0 <= min rpm < max rpm");
  need(!d.gear_ratios.empty(), "the transmission needs at least one gear");
  for (std::size_t i = 0; i < d.torque_curve.size(); ++i) {
    const auto &p = d.torque_curve[i];
    need(std::isfinite(p.first) && p.first >= 0.0 && p.first <= d.engine_max_rpm &&
             std::isfinite(p.second) && p.second >= 0.0 &&
             (i == 0 || p.first > d.torque_curve[i - 1].first),
         "torque_curve needs (rpm, fraction) points in ascending rpm within engine_max_rpm, "
         "fractions >= 0");
  }
  for (std::size_t i = 0; i < d.engine_friction_curve.size(); ++i) {
    const auto &p = d.engine_friction_curve[i];
    need(std::isfinite(p.first) && p.first >= 0.0 && std::isfinite(p.second) && p.second >= 0.0 &&
             (i == 0 || p.first > d.engine_friction_curve[i - 1].first),
         "engine_friction_curve needs (rpm, N m) points in ascending rpm, torques >= 0");
  }
  need(positive(d.clutch_strength), "clutch_strength must be finite and > 0");
  need(std::isfinite(d.converter_k) && d.converter_k >= 0.0, "converter_k must be finite and >= 0");
  need(std::isfinite(d.converter_stall) && d.converter_stall >= 1.0 && d.converter_coupling > 0.0 &&
           d.converter_coupling <= 1.0,
       "converter_stall must be >= 1 and converter_coupling in (0, 1]");
  need(d.driveline_efficiency > 0.0 && d.driveline_efficiency <= 1.0,
       "driveline_efficiency must be in (0, 1]");
  for (const double g : d.gear_ratios)
    need(positive(g), "gear ratios must be > 0");
  need(d.reverse_ratio < 0.0 && std::isfinite(d.reverse_ratio), "reverse_ratio must be < 0");
  need(positive(d.differential_ratio), "differential_ratio must be > 0");
  need(positive(d.max_accel) && positive(d.max_decel), "max_accel and max_decel must be > 0");
  need(d.drag_coefficient >= 0.0 && d.frontal_area >= 0.0 && d.air_density >= 0.0 &&
           d.rolling_resistance >= 0.0,
       "drag, area, air density and rolling resistance must be >= 0");
  const energy_source &e = d.energy;
  const auto fraction = [](double x, bool zero_ok) {
    return std::isfinite(x) && x <= 1.0 && (zero_ok ? x >= 0.0 : x > 0.0);
  };
  need(e.type <= energy_source::kind::fuel, "energy.type is not a kind of source");
  need(e.type != energy_source::kind::battery ||
           (positive(e.capacity) && fraction(e.initial_charge, true) &&
            fraction(e.motor_efficiency, false) && fraction(e.regen_efficiency, true) &&
            std::isfinite(e.max_regen_power) && e.max_regen_power >= 0.0),
       "a battery needs capacity > 0, initial_charge in [0, 1], motor_efficiency in (0, 1], "
       "regen_efficiency in [0, 1] and max_regen_power >= 0");
  need(e.type != energy_source::kind::fuel ||
           (std::isfinite(e.tank) && e.tank >= 0.0 && fraction(e.indicated_efficiency, false) &&
            positive(e.heating_value) && std::isfinite(e.accessory_power) &&
            e.accessory_power >= 0.0),
       "fuel needs tank >= 0, indicated_efficiency in (0, 1], heating_value > 0 and "
       "accessory_power >= 0");
  need(std::isfinite(d.shift_up_rpm_light) && d.shift_up_rpm_light >= 0.0 &&
           std::isfinite(d.shift_down_rpm_light) && d.shift_down_rpm_light >= 0.0,
       "the light-throttle shift points must be finite and >= 0");
  need(positive(d.tire_peak_slip) && positive(d.tire_peak_slip_angle) && d.tire_sliding > 0.0 &&
           d.tire_sliding <= 1.0,
       "the tire needs peak slips > 0 and 0 < tire_sliding <= 1");
}

// The first value of `b` that differs from `a` among those fixed when a vehicle is added
// (world::set_vehicle), or an empty string.
std::string fixed_change(const vehicle_desc &a, const vehicle_desc &b) {
  const auto same = [](const vec3 &p, const vec3 &q) {
    return p.x == q.x && p.y == q.y && p.z == q.z;
  };
  if (!same(a.half_extents, b.half_extents))
    return "half_extents";
  if (!same(a.center_of_mass, b.center_of_mass))
    return "center_of_mass";
  if (a.mass != b.mass)
    return "mass";
  if (a.axles.size() != b.axles.size())
    return "the number of axles";
  for (std::size_t i = 0; i < a.axles.size(); ++i) {
    const axle_desc &p = a.axles[i], &q = b.axles[i];
    const std::string at = "axles[" + std::to_string(i) + "].";
    if (p.x != q.x)
      return at + "x";
    if (p.half_track != q.half_track)
      return at + "half_track";
    if (p.z != q.z)
      return at + "z";
    if (p.wheel_radius != q.wheel_radius)
      return at + "wheel_radius";
    if (p.wheel_width != q.wheel_width)
      return at + "wheel_width";
    if (p.wheel_inertia != q.wheel_inertia)
      return at + "wheel_inertia";
    if (p.wheel_angular_damping != q.wheel_angular_damping)
      return at + "wheel_angular_damping";
    if (p.steered != q.steered)
      return at + "steered";
    if (p.driven != q.driven)
      return at + "driven";
    if (p.suspension_min != q.suspension_min)
      return at + "suspension_min";
    if (p.suspension_max != q.suspension_max)
      return at + "suspension_max";
    if ((p.anti_roll_stiffness > 0.0) != (q.anti_roll_stiffness > 0.0))
      return at + "anti_roll_stiffness to or from 0 (whether the axle has a bar)";
  }
  if (a.engine_inertia != b.engine_inertia)
    return "engine_inertia";
  if (a.torque_curve != b.torque_curve)
    return "torque_curve";
  if (a.gear_ratios != b.gear_ratios)
    return "gear_ratios";
  if (a.reverse_ratio != b.reverse_ratio)
    return "reverse_ratio";
  if (a.differential_ratio != b.differential_ratio)
    return "differential_ratio";
  if (a.tire_peak_slip != b.tire_peak_slip)
    return "tire_peak_slip";
  if (a.tire_peak_slip_angle != b.tire_peak_slip_angle)
    return "tire_peak_slip_angle";
  if (a.tire_sliding != b.tire_sliding)
    return "tire_sliding";
  const energy_source &p = a.energy, &q = b.energy;
  if (p.type != q.type || p.capacity != q.capacity || p.initial_charge != q.initial_charge ||
      p.motor_efficiency != q.motor_efficiency || p.regen_efficiency != q.regen_efficiency ||
      p.max_regen_power != q.max_regen_power || p.tank != q.tank ||
      p.indicated_efficiency != q.indicated_efficiency || p.heating_value != q.heating_value ||
      p.accessory_power != q.accessory_power)
    return "energy";
  if (a.energy_ledger != b.energy_ledger)
    return "energy_ledger";
  return std::string();
}

// Half the chord of a wheel of `radius` sunk `z`: half its contact patch's length.
double contact_half_length(double radius, double z) {
  return std::sqrt(std::max(0.0, 2.0 * radius * z - z * z));
}

// How fast a medium answers a wheel's load (s).
constexpr double kMediumTime = 0.1;

// A force smoothly signed by the velocity it resists: full at |v| >> 0.1 m/s, zero at rest.
double rolling_sign(double v) { return std::tanh(v / 0.1); }

template <class T> void put(std::string &out, const T &v) {
  out.append(reinterpret_cast<const char *>(&v), sizeof(T));
}
template <class T> void get(const std::string &in, std::size_t &pos, T &v) {
  if (pos + sizeof(T) > in.size())
    throw cvc::physics_error("restore: the vehicle state is truncated");
  std::memcpy(&v, in.data() + pos, sizeof(T));
  pos += sizeof(T);
}

// Field by field, never a struct with padding: the bytes feed state_hash.
void put_vec(std::string &out, const vec3 &v) {
  put(out, v.x);
  put(out, v.y);
  put(out, v.z);
}
void get_vec(const std::string &in, std::size_t &pos, vec3 &v) {
  get(in, pos, v.x);
  get(in, pos, v.y);
  get(in, pos, v.z);
}
void put_flag(std::string &out, bool b) { put(out, static_cast<std::uint8_t>(b ? 1 : 0)); }
bool get_flag(const std::string &in, std::size_t &pos) {
  std::uint8_t b = 0;
  get(in, pos, b);
  return b != 0;
}

} // namespace

// A vehicle_energy's values in a fixed order, for save and restore.
std::vector<double> energy_fields(const vehicle_energy &e) {
  return {e.supplied,
          e.brake,
          e.tire,
          e.aero,
          e.rolling,
          e.medium,
          e.drivetrain,
          e.suspension,
          e.chassis_damping,
          e.fields,
          e.stored,
          e.residual,
          e.kinetic,
          e.potential,
          e.spring,
          e.engine,
          e.idle,
          e.regen,
          e.source,
          e.charge,
          e.fuel_used,
          e.fuel_left,
          e.distance,
          e.power,
          e.drivetrain_split};
}

void set_energy_fields(vehicle_energy &e, const std::vector<double> &x) {
  double *const out[] = {&e.supplied,
                         &e.brake,
                         &e.tire,
                         &e.aero,
                         &e.rolling,
                         &e.medium,
                         &e.drivetrain,
                         &e.suspension,
                         &e.chassis_damping,
                         &e.fields,
                         &e.stored,
                         &e.residual,
                         &e.kinetic,
                         &e.potential,
                         &e.spring,
                         &e.engine,
                         &e.idle,
                         &e.regen,
                         &e.source,
                         &e.charge,
                         &e.fuel_used,
                         &e.fuel_left,
                         &e.distance,
                         &e.power,
                         &e.drivetrain_split};
  for (std::size_t i = 0; i < x.size() && i < sizeof(out) / sizeof(out[0]); ++i)
    *out[i] = x[i];
}

vehicle_set::rt &vehicle_set::at(vehicle_id id) {
  auto it = _v.find(id.value);
  if (it == _v.end())
    throw cvc::physics_error("unknown or removed vehicle id " + std::to_string(id.value));
  return it->second;
}
const vehicle_set::rt &vehicle_set::at(vehicle_id id) const {
  auto it = _v.find(id.value);
  if (it == _v.end())
    throw cvc::physics_error("unknown or removed vehicle id " + std::to_string(id.value));
  return it->second;
}

vehicle_id vehicle_set::add(world_backend &be, field_set &fields, const vehicle_desc &d,
                            const vec3 &position, const quat &rotation) {
  validate(d);
  need(_next < 0xffffffffu, "too many vehicles");
  vehicle_setup s;
  s.desc = &d;
  s.position = position;
  s.rotation = rotation;
  body_id chassis;
  std::uint32_t handle = 0;
  if (!be.add_vehicle(s, chassis, handle))
    throw cvc::unsupported_exception("cvc::physics: this backend has no vehicles (cap_vehicles); "
                                     "use the Jolt backend");
  // The chassis as the fields see it: a box about the body origin, its own centre of mass,
  // and no flow drag (the vehicle's aero is its drag).
  body_desc box_desc;
  box_desc.shape = box{d.half_extents};
  box_desc.mass = d.mass;
  fields.body_added(be, chassis, box_desc);
  fields.mark_vehicle(chassis, d.center_of_mass);

  rt v;
  v.desc = d;
  v.chassis = chassis;
  v.handle = handle;
  const std::size_t n = 2 * d.axles.size();
  v.rolling.assign(n, 0.0);
  v.medium.assign(n, 0.0);
  v.medium_load.assign(n, 0.0);
  v.readback.wheels.resize(n);
  if (d.energy.type == energy_source::kind::battery)
    v.energy.charge = d.energy.initial_charge;
  if (d.energy.type == energy_source::kind::fuel)
    v.energy.fuel_left = d.energy.tank;
  vehicle_id id;
  id.value = _next++;
  _v.emplace(id.value, std::move(v));
  return id;
}

void vehicle_set::remove(world_backend &be, field_set &fields, vehicle_id id) {
  rt &v = at(id);
  be.remove_vehicle(v.handle);
  fields.body_removed(v.chassis);
  _v.erase(id.value);
}

bool vehicle_set::is_chassis(body_id id) const {
  for (const auto &e : _v)
    if (e.second.chassis == id)
      return true;
  return false;
}

void vehicle_set::tune(world_backend &be, vehicle_id id, const vehicle_desc &d) {
  rt &v = at(id);
  validate(d, "set_vehicle");
  const std::string fixed = fixed_change(v.desc, d);
  if (!fixed.empty())
    throw cvc::physics_error("set_vehicle: " + fixed + " is fixed when the vehicle is added");
  be.tune_vehicle(v.handle, d);
  v.desc = d;
}

void vehicle_set::drive(vehicle_id id, const drive_command &c) {
  if (!std::isfinite(c.steer) || !std::isfinite(c.accel))
    throw cvc::physics_error("drive: the command must be finite");
  rt &v = at(id);
  v.raw = false;
  v.command = c;
}

void vehicle_set::set_controls(vehicle_id id, const vehicle_controls &c) {
  if (!std::isfinite(c.steer) || !std::isfinite(c.throttle) || !std::isfinite(c.brake) ||
      !std::isfinite(c.hand_brake))
    throw cvc::physics_error("set_controls: the controls must be finite");
  rt &v = at(id);
  v.raw = true;
  v.controls = c;
}

void vehicle_set::pre_step(world_backend &be, const field_set &fields, const world_config &cfg,
                           const std::vector<material> &materials) {
  const double dt = cfg.fixed_dt;
  if (!_v.empty())
    be.set_medium_reach(fields.medium_reach());
  for (auto &entry : _v) {
    rt &v = entry.second;
    const vehicle_desc &d = v.desc;
    const body_state s = be.state(v.chassis);
    const vec3 fwd = rotate(s.rotation, {1.0, 0.0, 0.0});
    const double speed = dot(s.linear_velocity, fwd);
    const double a_meas = v.have_last_speed ? (speed - v.last_speed) / dt : 0.0;
    v.last_speed = speed;
    v.have_last_speed = true;

    // The steering actuator: slew toward the command, at most max_steer_rate.
    v.steer_command = clamp(v.raw ? v.controls.steer : v.command.steer, -d.max_steer, d.max_steer);
    const double slew = d.max_steer_rate * dt;
    v.steer += clamp(v.steer_command - v.steer, -slew, slew);

    // Ackermann: each steered wheel about the rear axle's line.
    const double L = d.wheelbase();
    double rear = 0.0;
    bool have_rear = false;
    for (const axle_desc &a : d.axles)
      if (!a.steered && (!have_rear || a.x < rear)) {
        rear = a.x;
        have_rear = true;
      }
    const double t = std::tan(v.steer);
    const double relax = std::min(1.0, dt / kMediumTime);
    std::vector<double> steer(2 * d.axles.size(), 0.0), load(2 * d.axles.size(), 0.0);
    for (std::size_t ai = 0; ai < d.axles.size(); ++ai) {
      const axle_desc &a = d.axles[ai];
      for (int side = 0; side < 2; ++side) {
        const std::size_t w = 2 * ai + side;
        const double y = side == 0 ? a.half_track : -a.half_track;
        if (a.steered && L > 0.0)
          steer[w] = std::atan((a.x - rear) * t / (L - y * t));
        // The medium answers the load over kMediumTime: a wheel touching down settles into
        // its rut as the load builds, rather than drumming on it.
        const wheel_state &ws = v.readback.wheels[w].state;
        v.medium_load[w] += ((ws.contact ? ws.load : 0.0) - v.medium_load[w]) * relax;
        load[w] = v.medium_load[w];
      }
    }

    // Pedals: the driver model, or the caller's own.
    double throttle = 0.0, brake = 0.0, hand_brake = 0.0;
    bool reverse = false, neutral = false;
    if (v.raw) {
      throttle = clamp(v.controls.throttle, 0.0, 1.0);
      brake = clamp(v.controls.brake, 0.0, 1.0);
      hand_brake = clamp(v.controls.hand_brake, 0.0, 1.0);
      reverse = v.controls.reverse;
      neutral = v.controls.neutral;
    } else {
      reverse = v.command.reverse;
      const double dir = reverse ? -1.0 : 1.0;
      const double along = dir * speed;    // speed along the commanded direction
      const double want = v.command.accel; // a_cmd is already along it
      const double got = dir * a_meas;
      // One controller: the wanted acceleration, corrected by how far the measured one
      // misses it, is a throttle when positive and a brake when negative. So a_cmd = 0 keeps
      // the speed on a grade (braking against gravity downhill, pulling against it uphill).
      // The correction is proportional plus an integral trim, which learns what the
      // feed-forward on max_accel / max_decel gets wrong (a converter's launch, a grade, a
      // load); the trim restarts when the command turns from speeding up to slowing down, and
      // holds while the pedal it drives is saturated.
      constexpr double kDeadband = 0.05, kGain = 0.5, kHoldSpeed = 0.3, kTrim = 2.0; // 1/s
      if ((want > 0.0) != (v.trim_sign > 0.0)) {
        v.accel_trim = 0.0;
        v.trim_sign = want > 0.0 ? 1.0 : -1.0;
      }
      const double u = want + kGain * (want - got) + v.accel_trim;
      if (along < -kHoldSpeed) {
        brake = 1.0; // moving the wrong way: stop first
        v.accel_trim = 0.0;
      } else if (std::fabs(want) <= kDeadband && std::fabs(along) < kHoldSpeed) {
        brake = 1.0; // slow, with nothing asked: hold on the brakes (a grade included)
        v.accel_trim = 0.0;
      } else {
        if (u > kDeadband)
          throttle = clamp(u / d.max_accel, 0.0, 1.0);
        else if (u < -kDeadband)
          brake = clamp(-u / d.max_decel, 0.0, 1.0);
        const double e = want - got;
        if (!((throttle >= 1.0 && e > 0.0) || (brake >= 1.0 && e < 0.0)))
          v.accel_trim = clamp(v.accel_trim + kTrim * e * dt, -d.max_decel, d.max_accel);
      }
    }
    // An empty battery or tank gives no throttle.
    const energy_source &src = d.energy;
    if ((src.type == energy_source::kind::battery && !(v.energy.charge > 0.0)) ||
        (src.type == energy_source::kind::fuel && src.tank > 0.0 && !(v.energy.fuel_left > 0.0)))
      throttle = 0.0;
    v.throttle = throttle;
    v.brake = brake;
    v.hand_brake = hand_brake;
    be.set_vehicle_inputs(v.handle, steer, load, (reverse ? -1.0 : 1.0) * throttle, brake,
                          hand_brake, neutral);

    // The chassis's resistance, from the last step's wheels.
    const vec3 com = s.position + rotate(s.rotation, d.center_of_mass);
    double rho = d.air_density;
    vec3 wind;
    fields.air_at(com, rho, wind);
    const vec3 u = s.linear_velocity - wind;
    const double un = norm(u);
    const vec3 aero = u * (-0.5 * rho * d.drag_coefficient * d.frontal_area * un);
    vec3 force = aero, torque;
    vec3 rolling_force, rolling_torque, medium_force, medium_torque; // for the ledger
    v.aero_force = aero;
    v.air_velocity = wind;
    for (std::size_t w = 0; w < v.readback.wheels.size(); ++w) {
      const vehicle_readback::wheel &rw = v.readback.wheels[w];
      v.rolling[w] = v.medium[w] = 0.0;
      if (!rw.state.contact || !(rw.state.load > 0.0))
        continue;
      const vec3 arm = rw.state.contact_point - com;
      const vec3 vp = s.linear_velocity + cross(s.angular_velocity, arm);
      const double v_long = dot(vp, rw.longitudinal);
      const double sign = rolling_sign(v_long);
      const material_id mid = rw.state.contact_material;
      const double crr = mid < materials.size() && materials[mid].rolling_resistance > 0.0f
                             ? materials[mid].rolling_resistance
                             : d.rolling_resistance;
      v.rolling[w] = crr * rw.state.load;
      const axle_desc &a = d.axles[w / 2];
      const double z = rw.state.sinkage;
      // Only snow compacted deeper than the rut ahead of the tire resists (field::ruts):
      // b k (z^2 - rut^2) / 2, the work of pressing it from the rut down to z.
      double ahead = 0.0;
      if (z > 0.0 && rw.medium_field != 0xffffffffu) {
        const double half = contact_half_length(a.wheel_radius, z);
        const vec3 lead = rw.state.contact_point + rw.longitudinal * (v_long >= 0.0 ? half : -half);
        ahead = std::min(z, fields.rut_at(rw.medium_field, lead.x, lead.y));
      }
      v.medium[w] = 0.5 * a.wheel_width * rw.medium_modulus * (z * z - ahead * ahead);
      const vec3 f = rw.longitudinal * (-(v.rolling[w] + v.medium[w]) * sign);
      force = force + f;
      torque = torque + cross(arm, f);
      const vec3 fr = rw.longitudinal * (-v.rolling[w] * sign);
      const vec3 fm = rw.longitudinal * (-v.medium[w] * sign);
      rolling_force = rolling_force + fr;
      rolling_torque = rolling_torque + cross(arm, fr);
      medium_force = medium_force + fm;
      medium_torque = medium_torque + cross(arm, fm);
    }
    if (s.active)
      be.add_field_load(v.chassis, force, torque);
    // What the ledger charges each cause with: the loads acting through this step (none on
    // a sleeping chassis, which the fields and this step leave alone).
    vehicle_loads loads;
    if (s.active) {
      loads.force[0] = aero;
      loads.force[1] = rolling_force;
      loads.torque[1] = rolling_torque;
      loads.force[2] = medium_force;
      loads.torque[2] = medium_torque;
      const field_load fl = fields.load(v.chassis);
      loads.force[3] = fl.force;
      loads.torque[3] = fl.torque;
    }
    be.set_vehicle_loads(v.handle, loads);
  }
}

void vehicle_set::post_step(world_backend &be, field_set &fields, const world_config &cfg) {
  const bool ruts = fields.has_ruts();
  for (auto &entry : _v) {
    rt &v = entry.second;
    be.read_vehicle(v.handle, v.readback);
    account(v, cfg.fixed_dt);
    if (!ruts)
      continue;
    // Each wheel in a rutted medium presses its footprint into it: the half behind it as
    // it rolls (its rut), all of it standing still.
    const body_state s = be.state(v.chassis);
    const vec3 com = s.position + rotate(s.rotation, v.desc.center_of_mass);
    for (std::size_t w = 0; w < v.readback.wheels.size(); ++w) {
      const vehicle_readback::wheel &rw = v.readback.wheels[w];
      const double z = rw.state.sinkage;
      if (!rw.state.contact || !(z > 0.0) || rw.medium_field == 0xffffffffu)
        continue;
      const axle_desc &a = v.desc.axles[w / 2];
      const vec3 arm = rw.state.contact_point - com;
      const double v_long =
          dot(s.linear_velocity + cross(s.angular_velocity, arm), rw.longitudinal);
      double dx = rw.longitudinal.x, dy = rw.longitudinal.y;
      const double n = std::sqrt(dx * dx + dy * dy);
      if (!(n > 0.0))
        continue;
      dx /= n;
      dy /= n;
      if (v_long < 0.0) {
        dx = -dx;
        dy = -dy;
      }
      fields.press(rw.medium_field, rw.state.contact_point.x, rw.state.contact_point.y, dx, dy,
                   2.0 * contact_half_length(a.wheel_radius, z), a.wheel_width, z,
                   std::fabs(v_long) > 0.05);
    }
  }
}

// The step's measurements into the ledger's totals, and the source's side.
void vehicle_set::account(rt &v, double dt) {
  const vehicle_readback::energy_step &e = v.readback.energy;
  if (!e.measured)
    return;
  vehicle_energy &n = v.energy;
  n.engine += e.engine;
  n.idle += e.idle;
  n.supplied += e.supplied;
  n.drivetrain += e.drivetrain;
  n.drivetrain_split += e.split;
  n.brake += e.brake;
  n.tire += e.tire;
  n.aero -= e.work[0];
  n.rolling -= e.work[1];
  n.medium -= e.work[2];
  n.fields += e.work[3];
  n.suspension += e.suspension;
  n.chassis_damping += e.chassis_damping;
  n.stored += e.stored;
  n.distance += e.distance;
  n.residual = n.supplied + n.idle + n.fields + n.drivetrain_split - n.dissipated() - n.stored;
  if (e.sampled) {
    if (!v.have_potential0) {
      v.potential0 = e.potential;
      v.have_potential0 = true;
    }
    n.kinetic = e.kinetic;
    n.potential = e.potential - v.potential0;
    n.spring = e.spring;
  }
  n.power = dt > 0.0 ? e.supplied / dt : 0.0;

  const energy_source &src = v.desc.energy;
  if (src.type == energy_source::kind::battery) {
    // The motor draws its shaft work over its efficiency; braking returns
    // regen_efficiency of the braking energy, at most max_regen_power, while there is room.
    // The braking energy is the vehicle's own: not what the drivetrain step made in a braked
    // wheel (drivetrain_split), which the brakes take straight back.
    const double draw = std::max(0.0, e.engine + e.idle) / src.motor_efficiency;
    const double braking = std::max(0.0, e.brake - std::max(0.0, e.split_braked));
    double credit = src.regen_efficiency * std::min(braking, src.max_regen_power * dt);
    double next = n.charge - (draw - credit) / src.capacity;
    if (next > 1.0) {
      credit = std::max(0.0, credit - (next - 1.0) * src.capacity);
      next = 1.0;
    }
    n.charge = std::max(0.0, next);
    n.source += draw;
    n.regen += credit;
  } else if (src.type == energy_source::kind::fuel) {
    // The Willans line: what the engine did while fuelled (its shaft work, friction and
    // accessories, measured each sub-step) over its indicated efficiency.
    const double kg = std::max(0.0, e.indicated) / (src.indicated_efficiency * src.heating_value);
    n.fuel_used += kg;
    n.source += kg * src.heating_value;
    if (src.tank > 0.0)
      n.fuel_left = std::max(0.0, src.tank - n.fuel_used);
  }
}

vehicle_state vehicle_set::state(const world_backend &be, vehicle_id id) const {
  const rt &v = at(id);
  vehicle_state out;
  out.chassis = v.chassis;
  out.body = be.state(v.chassis);
  out.speed = dot(out.body.linear_velocity, rotate(out.body.rotation, {1.0, 0.0, 0.0}));
  out.steer = v.steer;
  out.steer_command = v.steer_command;
  out.throttle = v.throttle;
  out.brake = v.brake;
  out.hand_brake = v.hand_brake;
  out.gear = v.readback.gear;
  out.engine_rpm = v.readback.engine_rpm;
  out.aero_force = v.aero_force;
  out.air_velocity = v.air_velocity;
  out.wheels.reserve(v.readback.wheels.size());
  for (std::size_t w = 0; w < v.readback.wheels.size(); ++w) {
    wheel_state ws = v.readback.wheels[w].state;
    ws.rolling_resistance = v.rolling[w];
    ws.medium_resistance = v.medium[w];
    out.wheels.push_back(ws);
  }
  out.energy = v.energy;
  return out;
}

void vehicle_set::snapshot(const world_backend &be, std::vector<vehicle_snapshot> &out) const {
  out.reserve(out.size() + _v.size());
  for (const auto &e : _v) {
    vehicle_snapshot s;
    s.id.value = e.first;
    s.desc = e.second.desc;
    s.state = state(be, s.id);
    out.push_back(std::move(s));
  }
}

void vehicle_set::save(const world_backend &be, std::string &out) const {
  put(out, static_cast<std::uint64_t>(_v.size()));
  for (const auto &e : _v) {
    const rt &v = e.second;
    put(out, e.first);
    put(out, v.steer);
    put(out, v.steer_command);
    put_flag(out, v.raw);
    put(out, v.command.steer);
    put(out, v.command.accel);
    put_flag(out, v.command.reverse);
    put(out, v.controls.steer);
    put(out, v.controls.throttle);
    put(out, v.controls.brake);
    put(out, v.controls.hand_brake);
    put_flag(out, v.controls.reverse);
    put_flag(out, v.controls.neutral);
    put(out, v.last_speed);
    put_flag(out, v.have_last_speed);
    put(out, v.accel_trim);
    put(out, v.trim_sign);
    put(out, static_cast<std::uint64_t>(v.readback.wheels.size()));
    for (const double l : v.medium_load)
      put(out, l);
    for (const vehicle_readback::wheel &w : v.readback.wheels) {
      const wheel_state &s = w.state;
      put(out, s.load);
      put_flag(out, s.contact);
      put_vec(out, s.contact_point);
      put_vec(out, w.longitudinal);
      put(out, s.contact_material);
      put(out, s.sinkage);
      put(out, s.medium_depth);
      put(out, w.medium_modulus);
    }
    for (const double x : energy_fields(v.energy))
      put(out, x);
    put(out, v.potential0);
    put_flag(out, v.have_potential0);
    be.save_vehicle(v.handle, out);
  }
}

void vehicle_set::restore(world_backend &be, const std::string &in, std::size_t &pos) {
  std::uint64_t n = 0;
  get(in, pos, n);
  if (n != _v.size())
    throw cvc::physics_error("restore: this world does not hold the vehicles the state was "
                             "saved with");
  for (auto &e : _v) {
    rt &v = e.second;
    std::uint32_t key = 0;
    get(in, pos, key);
    if (key != e.first)
      throw cvc::physics_error("restore: the vehicles were added in a different order");
    get(in, pos, v.steer);
    get(in, pos, v.steer_command);
    v.raw = get_flag(in, pos);
    get(in, pos, v.command.steer);
    get(in, pos, v.command.accel);
    v.command.reverse = get_flag(in, pos);
    get(in, pos, v.controls.steer);
    get(in, pos, v.controls.throttle);
    get(in, pos, v.controls.brake);
    get(in, pos, v.controls.hand_brake);
    v.controls.reverse = get_flag(in, pos);
    v.controls.neutral = get_flag(in, pos);
    get(in, pos, v.last_speed);
    v.have_last_speed = get_flag(in, pos);
    get(in, pos, v.accel_trim);
    get(in, pos, v.trim_sign);
    std::uint64_t wheels = 0;
    get(in, pos, wheels);
    if (wheels != v.readback.wheels.size())
      throw cvc::physics_error("restore: a vehicle has a different number of wheels");
    for (double &l : v.medium_load)
      get(in, pos, l);
    for (vehicle_readback::wheel &w : v.readback.wheels) {
      wheel_state &s = w.state;
      get(in, pos, s.load);
      s.contact = get_flag(in, pos);
      get_vec(in, pos, s.contact_point);
      get_vec(in, pos, w.longitudinal);
      get(in, pos, s.contact_material);
      get(in, pos, s.sinkage);
      get(in, pos, s.medium_depth);
      get(in, pos, w.medium_modulus);
    }
    std::vector<double> energy = energy_fields(v.energy);
    for (double &x : energy)
      get(in, pos, x);
    set_energy_fields(v.energy, energy);
    get(in, pos, v.potential0);
    v.have_potential0 = get_flag(in, pos);
    be.restore_vehicle(v.handle, in, pos);
  }
}

} // namespace detail
} // namespace physics
} // namespace cvc
