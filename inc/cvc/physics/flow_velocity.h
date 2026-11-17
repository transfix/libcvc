/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// flow_velocity.h — a flow's velocity that changes in space and time: the wind
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md E5, O14).
//
//   auto wind = std::make_shared<cvc::physics::wind_flow>(cvc::algo::wind_state{9, 0.5, 0.25});
//   cvc::physics::field air;                  // field.h
//   air.kind = cvc::physics::field_kind::flow;
//   air.wind = wind;                          // no volume: air, 1.225 kg/m^3, everywhere
//   w.add_field(air);
//
// A flow field (field.h) pushes every dynamic body with quadratic drag against the air, cell by
// cell, so torques follow: 1/2 rho C_d a |u| u, u the air's velocity minus the cell's, a the
// cell's share of the body's area across u. A flow with `wind` reads its velocity in place of the
// velocity volumes, at the world's field time (world::field_time, the clock a world follows), once
// per body at its centre of mass, and a vehicle's own aero drag (vehicle.h: its drag coefficient
// and frontal area) reads the air's velocity at its centre of mass the same way. So a floating
// crate drifts downwind, a car feels a crosswind, and a body in the lee of nothing feels the
// gusts. A flow with a wind may still have a volume (its density over a box) to blow in a box only.
//
// wind_flow is the scene's wind (cvc/algo/wind_field.h): the mean wind times the gust factor at
// (x, y, t), times the height profile cvc::algo::wind_profile(z - base_z) (the power law of an
// open surface layer, 0 at and below base_z). base_z is the ground's or the water's level the
// wind is measured over; a cell below it (under water) feels none.
//
// Like a fluid_surface, a flow_velocity must be deterministic (cvc/algo's det_math and lattice
// noise) and immutable once a field holds it, and its digest() names its content: the world
// folds it into save() and state_hash(), and restore() refuses a world whose winds differ from
// the state's. set_field with a new wind to change it.
//
// Standard library only, like the rest of cvc/physics (and cvc::algo, which depends on nothing).

#ifndef __CVC_PHYSICS_FLOW_VELOCITY_H__
#define __CVC_PHYSICS_FLOW_VELOCITY_H__

#include <cstdint>
#include <cvc/algo/wind_field.h>
#include <cvc/physics/types.h>
#include <mutex>

namespace cvc {
namespace physics {

class flow_velocity {
public:
  virtual ~flow_velocity() = default;
  // The flow's velocity (m/s) at p at time t.
  virtual vec3 velocity(const vec3 &p, double t) const = 0;
  // A 64-bit digest of everything velocity() depends on.
  virtual std::uint64_t digest() const = 0;

  // How the field step reads it over a body, so the wind costs O(bodies) a step: the flow's
  // velocity is reference(p, t) times profile(p.z), and it fills only the space above base()
  // (the ground or the water it blows over; a cell is clipped to it as to a free surface). The
  // step reads reference() once per body, at its centre of mass, and profile() at a few heights
  // across the body. The defaults: velocity() itself, the same at every height, everywhere.
  virtual vec3 reference(const vec3 &p, double t) const { return velocity(p, t); }
  virtual double profile(double z) const {
    (void)z;
    return 1.0;
  }
  virtual double base() const { return -1.0e300; }
};

// The scene's wind: wind_velocity(w, x, y, t) wind_profile(z - base_z), horizontal.
class wind_flow : public flow_velocity {
public:
  // Throws std::invalid_argument for a non-finite value or a negative speed or gustiness.
  explicit wind_flow(const cvc::algo::wind_state &w, double base_z = 0.0);
  vec3 velocity(const vec3 &p, double t) const override;
  std::uint64_t digest() const override;
  // The wind 10 m up at (p.x, p.y), the height profile over base_z, and base_z.
  vec3 reference(const vec3 &p, double t) const override;
  double profile(double z) const override;
  double base() const override { return _base; }

  const cvc::algo::wind_state &wind() const noexcept { return _w; }
  double base_z() const noexcept { return _base; }

private:
  cvc::algo::wind_state _w;
  double _base;
  double _cx, _cy;
  std::uint64_t _digest;
  // The gusts at the last time read (a cache: a pure function of t, so it changes no value).
  mutable std::mutex _mutex;
  mutable bool _have = false;
  mutable double _t = 0.0;
  mutable cvc::algo::gust_frame _frame;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FLOW_VELOCITY_H__
