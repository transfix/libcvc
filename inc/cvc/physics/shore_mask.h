/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// shore_mask.h — a wake's coast from a shore grid (OCEAN-AND-WEATHER-ROADMAP.md C6): the mask the
// physics wake (field_wake::shore, field.h) is stepped under, the drawn wake window's
// (lsystem_coast's seaWakeMask, the water node's wake_mask).
//
//   auto shore = std::make_shared<cvc::physics::shore_grid_mask>(grid, slope, layer);
//   cvc::physics::field water = ...;  // a fluid with a free surface
//   water.wake.cells = 64;
//   water.wake.shore = shore;         // its waves reflect off the cliffs, soak into the beaches
//
// The grid is cvc::world::water_bodies' (O1): the still water's depth and the signed distance to
// the shore, + offshore. The mask is 0 on land (the shore distance not above 0), so a pond or a
// lake inside the window is walled on all its banks. In the water it ramps over the last 3 m to
// the shore from 1 down to 0.5 at the waterline, a beach that absorbs; where the shore is steep it
// is 1 up to the waterline, a wall that reflects. Steep is the larger of the shore layer's
// reflection K_r (when given) and the ground's slope (rise over run, when given) ramped smoothly
// from 0.25 to 0.6: m = beach + (1 - beach) steep. Each plane is read bilinearly, clamped to the
// grid, as sea_surface reads it.
//
// Deterministic and the same on every platform (IEEE arithmetic in a fixed order, built without
// FMA contraction); its digest covers the grid's planes it reads.

#ifndef __CVC_PHYSICS_SHORE_MASK_H__
#define __CVC_PHYSICS_SHORE_MASK_H__

#include <cstdint>
#include <cvc/ocean/shore.h>
#include <cvc/physics/field.h>
#include <memory>
#include <vector>

namespace cvc {
namespace physics {

class shore_grid_mask : public wake_shore {
public:
  // The beach's width (m) and the mask at its waterline, and the slopes the steep ramp runs over.
  static constexpr double kBeachWidth = 3.0;
  static constexpr double kBeachMask = 0.5;
  static constexpr double kSteepFrom = 0.25, kSteepTo = 0.6;

  // `slope`: the ground's rise over run on the grid's samples (empty: none); `layer`: the swell's
  // shore layer, for its reflection (null: none). Throws std::invalid_argument for a null grid,
  // one under 2 x 2 samples, without a shore distance on every sample, or with a slope plane of
  // another size.
  explicit shore_grid_mask(std::shared_ptr<const cvc::ocean::shore_grid> grid,
                           std::vector<float> slope = {},
                           std::shared_ptr<const cvc::ocean::shore_layer> layer = nullptr);

  double mask(double x, double y) const override;
  std::uint64_t digest() const override { return _digest; }

  const cvc::ocean::shore_grid &grid() const noexcept { return *_grid; }

private:
  double sample(const std::vector<float> &f, double x, double y) const;

  std::shared_ptr<const cvc::ocean::shore_grid> _grid;
  std::vector<float> _slope;
  std::shared_ptr<const cvc::ocean::shore_layer> _layer;
  std::uint64_t _digest = 0;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_SHORE_MASK_H__
