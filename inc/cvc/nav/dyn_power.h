/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::dyn_power_si: the dynamic vehicle's power model (detail/dyn_power.h, the
// vehicle-physics plan's P8c) in SI, as cvc::physics::fit_power fits it to the P3 ledger, and its
// conversion into nav's normalized frame for the trainer (train_config::power).
//
//   const cvc::physics::power_fit f = cvc::physics::fit_power(log.runs(), dt, desc);
//   cfg.power = cvc::nav::dyn_power_params_of(f.si, desc.mass, cvc::nav::frame_of(cfg));
//
// Each value converts by its length exponent e, as dyn_bike_params_of does: a power (W, kg m^2 /
// s^3) has e = 2, a power per speed (W s / m, a force) e = 1, a power per speed cubed (kg / m, as
// the drag factor) e = -1, the cornering coefficient (s^2 / m) e = -1; the mass and the
// dimensionless energy ratios pass through.

#ifndef __CVC_NAV_DYN_POWER_H__
#define __CVC_NAV_DYN_POWER_H__

#include <cvc/core/world_units.h>
#include <cvc/nav/detail/dyn_power.h>

namespace cvc {
namespace nav {

struct dyn_power_si {
  double k_pos = 0.0;  // source J per J of positive wheel work
  double k_neg = 0.0;  // per J of braking wheel work (<= 0: regeneration returns it)
  double p_idle = 0.0; // W drawn at rest
  double c_v = 0.0;    // W per m/s of speed
  double c_v3 = 0.0;   // W per (m/s)^3
  double c_lat = 0.0;  // cornering: W per kg (m/s^2)^2 (m/s), s^2 / m
  // Source J per J of the dynamic vehicle's own tire dissipation (diff::dyn_tire_power), lateral
  // and longitudinal; dimensionless, so they pass through unconverted.
  double tire_lat = 0.0, tire_lon = 0.0;
  double width = 0.0;  // W: the smooth parts' width; 0 = the trainer's default
};

inline diff::power_params dyn_power_params_of(const dyn_power_si &si, double mass_kg,
                                              const world_units::normalized_frame &f,
                                              double metres_per_world_unit = 1.0) {
  const double k = f.scale / metres_per_world_unit; // normalized units per metre
  diff::power_params p;
  p.m = static_cast<float>(mass_kg);
  p.k_pos = static_cast<float>(si.k_pos);
  p.k_neg = static_cast<float>(si.k_neg);
  p.p_idle = static_cast<float>(si.p_idle * k * k);
  p.c_v = static_cast<float>(si.c_v * k);
  p.c_v3 = static_cast<float>(si.c_v3 / k);
  p.c_lat = static_cast<float>(si.c_lat / k);
  p.tire_lat = static_cast<float>(si.tire_lat);
  p.tire_lon = static_cast<float>(si.tire_lon);
  p.w = static_cast<float>(si.width * k * k);
  return p;
}

// train_config::energy_ref from a Wh/km: 1 Wh/km is 3.6 J/m, an energy per length (a force),
// whose length exponent is 1.
inline float dyn_energy_ref_of(double wh_per_km, const world_units::normalized_frame &f,
                               double metres_per_world_unit = 1.0) {
  const double k = f.scale / metres_per_world_unit; // normalized units per metre
  return static_cast<float>(3.6 * wh_per_km * k);
}

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DYN_POWER_H__
