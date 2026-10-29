/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::dyn4_si: the two-track vehicle's parameters in SI (the vehicle-physics plan's P8d),
// and their conversion into nav's normalized frame for diff::dyn4_plant (detail/dyn_4w.h).
//
//   cvc::nav::dyn4_si si = cvc::physics::nav_plant::dyn4_si_of(vehicle_desc::humvee());
//   cvc::nav::diff::dyn4_params p = cvc::nav::dyn4_params_of(si, cvc::nav::frame_of(cfg));
//
// It is P8a's dyn_bike_si (mass, yaw inertia, axles, tires, drive and brake splits, resistances,
// lags) plus what four wheels and a suspension add: the half tracks, the roll and pitch inertias,
// and each corner's spring and damper. The ground's friction is the ground sampler's, so
// `single.mu` is not used. Values convert by their length exponent, as dyn_bike_params_of does:
// tracks e = 1, the inertias e = 2; spring rates (N/m = kg/s^2) and damper rates (N s/m = kg/s)
// carry no length and pass through.

#ifndef __CVC_NAV_DYN_4W_H__
#define __CVC_NAV_DYN_4W_H__

#include <cvc/nav/detail/dyn_4w.h>
#include <cvc/nav/dyn_bike.h>

namespace cvc {
namespace nav {

struct dyn4_si {
  dyn_bike_si single;            // the single-track numbers (its mu unused: the ground's)
  double half_track_front = 0.8; // m
  double half_track_rear = 0.8;  // m
  double roll_inertia = 800.0;   // kg m^2, about the centre of gravity
  double pitch_inertia = 2500.0; // kg m^2
  double spring_front = 0.0;     // N/m per corner; 0 = rigid
  double spring_rear = 0.0;
  double damper_front = 0.0; // N s/m per corner; 0 = critical
  double damper_rear = 0.0;
  double min_wheel_load = 0.02; // the load floor's width, of m g
  bool grade_force = false;     // gravity along the ground acts on the body (opt-in)
};

inline diff::dyn4_params dyn4_params_of(const dyn4_si &si, const world_units::normalized_frame &f,
                                        double metres_per_world_unit = 1.0) {
  const diff::dyn_bike_params b = dyn_bike_params_of(si.single, f, metres_per_world_unit);
  const double k = f.scale / metres_per_world_unit; // normalized units per metre
  diff::dyn4_params p;
  p.m = b.m;
  p.Iz = b.Iz;
  p.lf = b.lf;
  p.lr = b.lr;
  p.h = b.h;
  p.g = b.g;
  p.B_f = b.B_f;
  p.B_r = b.B_r;
  p.C = b.C;
  p.front_drive = b.front_drive;
  p.front_brake = b.front_brake;
  p.drag = b.drag;
  p.c_rr = b.c_rr;
  p.tau_steer = b.tau_steer;
  p.steer_rate = b.steer_rate;
  p.tau_drive = b.tau_drive;
  p.v_blend = b.v_blend;
  p.v_width = b.v_width;
  p.eps = b.eps;
  p.iters = b.iters;
  p.tf = static_cast<float>(si.half_track_front * k);
  p.tr = static_cast<float>(si.half_track_rear * k);
  p.Ix = static_cast<float>(si.roll_inertia * k * k);
  p.Iy = static_cast<float>(si.pitch_inertia * k * k);
  p.k_f = static_cast<float>(si.spring_front);
  p.k_r = static_cast<float>(si.spring_rear);
  p.c_f = static_cast<float>(si.damper_front);
  p.c_r = static_cast<float>(si.damper_rear);
  p.fz_min = static_cast<float>(si.min_wheel_load);
  p.grade_force = si.grade_force ? 1 : 0;
  return p;
}

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DYN_4W_H__
