/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A fluid field's free surface (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md B5, decision D2): the
// water's still level, the waves on it and the water's velocity, so a floating body rides the
// waves the renderer draws.
//
//   auto sea = std::make_shared<cvc::physics::wave_train_surface>(0.0, 30.0, trains);
//   cvc::physics::field water;
//   water.volume = box;      // 1000 kg/m^3, its top above the highest crest
//   water.surface = sea;     // the cells' tops clip to the surface, not the box
//   w.add_field(water);
//
// A fluid field with a surface is water up to height(x, y, t) and air above it, within its
// volume's box: each cell's immersed part is its box clipped to the field's box and below the
// surface over the cell's centre, and its drag and damping use velocity() in place of the
// field's velocity volumes. t is the world's field time (world::field_time), which follows the
// clock a world follows, so physics and a renderer drawing the same surface at the clock's time
// agree. Lakes, streams and the sea are all one type.
//
// A surface must be deterministic and give the same bits on every platform (use
// cvc/algo/det_math.h, never libm's transcendentals), and immutable once a field holds it: the
// world reads it from the step's thread. Its digest() names its content; the world folds each
// surface's digest into save() and state_hash(), and restore() refuses a world whose surfaces
// differ from the state's.
//
// Standard library only, like the rest of cvc/physics.

#ifndef __CVC_PHYSICS_FLUID_SURFACE_H__
#define __CVC_PHYSICS_FLUID_SURFACE_H__

#include <cstdint>
#include <cvc/physics/types.h>
#include <vector>

namespace cvc {
namespace physics {

class fluid_surface {
public:
  virtual ~fluid_surface() = default;
  // The still water's height at (x, y) (m, world z).
  virtual double level(double x, double y) const = 0;
  // The surface's height at (x, y) at time t (s): the level plus the waves.
  virtual double height(double x, double y, double t) const = 0;
  // The water's velocity (m/s) at p at time t: the waves' orbital motion, and any current.
  virtual vec3 velocity(const vec3 &p, double t) const = 0;
  // A 64-bit digest of everything the three above depend on.
  virtual std::uint64_t digest() const = 0;
};

// Still water at one level, optionally flowing at a uniform current.
class still_surface : public fluid_surface {
public:
  explicit still_surface(double level, const vec3 &current = {});
  double level(double x, double y) const override;
  double height(double x, double y, double t) const override;
  vec3 velocity(const vec3 &p, double t) const override;
  std::uint64_t digest() const override;

private:
  double _level;
  vec3 _current;
};

// Regular linear (Airy) wave trains on water of one depth: each a crest line travelling along
// dir_rad (from +x toward +y) at its period, with the finite-depth dispersion relation
// omega^2 = g k tanh(k d) and the orbital velocity decaying with depth as cosh(k(z + d)) /
// sinh(k d). Above the still level the velocity is the level's (linear theory does not reach
// into a crest), and below the bottom the bottom's.
struct wave_train {
  double amplitude = 0.5; // m, half the crest-to-trough height
  double period_s = 8.0;
  double dir_rad = 0.0;
  double phase = 0.0; // rad: eta = A cos(k . x - omega t + phase)
};

class wave_train_surface : public fluid_surface {
public:
  // depth_m <= 0 means deep water. Throws std::invalid_argument for a non-finite value, a
  // period not above 0, a negative amplitude or a gravity not above 0.
  wave_train_surface(double level, double depth_m, std::vector<wave_train> trains,
                     double gravity = 9.80665);
  double level(double x, double y) const override;
  double height(double x, double y, double t) const override;
  vec3 velocity(const vec3 &p, double t) const override;
  std::uint64_t digest() const override;

  const std::vector<wave_train> &trains() const { return _trains; }
  // Train i's wavenumber (rad/m) and angular frequency (rad/s).
  double wavenumber(std::size_t i) const { return _k[i]; }
  double omega(std::size_t i) const { return _omega[i]; }

private:
  double _level, _depth, _gravity;
  std::vector<wave_train> _trains;
  std::vector<double> _k, _omega, _kx, _ky;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FLUID_SURFACE_H__
