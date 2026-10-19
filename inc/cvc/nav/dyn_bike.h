/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::dyn_bike_si: the dynamic single-track vehicle's parameters in SI, as a spec sheet
// gives them, and their conversion into nav's normalized frame for diff::dyn_plant
// (detail/dyn_bike.h; the vehicle-physics plan's P8a).
//
//   cvc::nav::dyn_bike_si si = cvc::physics::nav_plant::dyn_bike_si_of(vehicle_desc::humvee());
//   cvc::nav::diff::dyn_bike_params p = cvc::nav::dyn_bike_params_of(si, cvc::nav::frame_of(cfg));
//
// Each value converts by its length exponent e, as world_units does: SI -> world by
// metres_per_world_unit^-e, world -> normalized by S^e. Mass is unchanged; lengths, speeds and
// accelerations have e = 1, the yaw inertia e = 2, the drag factor (a force per speed squared)
// e = -1; angles, times and coefficients pass through.

#ifndef __CVC_NAV_DYN_BIKE_H__
#define __CVC_NAV_DYN_BIKE_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/detail/dyn_bike.h>

namespace cvc {
namespace nav {

struct dyn_bike_si {
  double mass = 2000.0;        // kg
  double yaw_inertia = 3000.0; // kg m^2, about the centre of gravity
  double cg_to_front = 1.4;    // m, to the front axle
  double cg_to_rear = 1.4;     // m, to the rear axle
  double cg_height = 0.6;      // m
  double mu = 0.9;             // the ground's friction
  // Pacejka lateral tire: B the stiffness factor (1/rad), C the shape factor in (1, 2). The
  // cornering stiffness per unit load is B C mu.
  double tire_B_front = 12.0;
  double tire_B_rear = 12.0;
  double tire_C = 1.2;
  double front_drive = 0.0; // the front axle's share of the drive force
  double front_brake = 0.6; // and of the brake force
  double drag = 0.0;        // kg/m: 1/2 rho C_d A
  double rolling_resistance = 0.0;
  double steer_lag = 0.0;        // s
  double steer_rate = 0.0;       // rad/s; 0 = unlimited
  double drive_lag = 0.0;        // s
  double blend_speed = 1.0;      // m/s: below it the vehicle turns like the kinematic bicycle
  double blend_width = 0.3;      // m/s
  double slip_speed_floor = 0.5; // m/s: the slip angles' speed floor
  double gravity = 9.81;         // m/s^2
  double min_axle_load = 0.05;   // of m g
};

inline diff::dyn_bike_params dyn_bike_params_of(const dyn_bike_si &si,
                                                const world_units::normalized_frame &f,
                                                double metres_per_world_unit = 1.0) {
  const double k = f.scale / metres_per_world_unit; // normalized units per metre
  const auto by = [&](double v, int e) {
    double s = 1.0;
    for (int i = 0; i < (e < 0 ? -e : e); ++i)
      s *= k;
    return static_cast<float>(e < 0 ? v / s : v * s);
  };
  diff::dyn_bike_params p;
  p.m = static_cast<float>(si.mass);
  p.Iz = by(si.yaw_inertia, 2);
  p.lf = by(si.cg_to_front, 1);
  p.lr = by(si.cg_to_rear, 1);
  p.h = by(si.cg_height, 1);
  p.g = by(si.gravity, 1);
  p.mu = static_cast<float>(si.mu);
  p.B_f = static_cast<float>(si.tire_B_front);
  p.B_r = static_cast<float>(si.tire_B_rear);
  p.C = static_cast<float>(si.tire_C);
  p.front_drive = static_cast<float>(si.front_drive);
  p.front_brake = static_cast<float>(si.front_brake);
  p.drag = by(si.drag, -1);
  p.c_rr = static_cast<float>(si.rolling_resistance);
  p.tau_steer = static_cast<float>(si.steer_lag);
  p.steer_rate = static_cast<float>(si.steer_rate);
  p.tau_drive = static_cast<float>(si.drive_lag);
  p.v_blend = by(si.blend_speed, 1);
  p.v_width = by(si.blend_width, 1);
  p.eps = by(si.slip_speed_floor, 1);
  p.fz_min = static_cast<float>(si.min_axle_load);
  return p;
}

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DYN_BIKE_H__
