/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// ocean_surface.h — the sea as a fluid field's free surface (docs/roadmap/
// OCEAN-AND-WEATHER-ROADMAP.md, B2-B5): cvc::ocean::sea_surface, the CPU twin of the sea a
// renderer draws from the same cascades and shore layer, behind fluid_surface.
//
//   auto sea = std::make_shared<cvc::ocean::sea_surface>(cascades, params, grid, layer);
//   cvc::physics::field water;
//   water.kind = cvc::physics::field_kind::fluid;
//   water.volume = density;                      // its box holds the crests and the swash
//   water.surface = std::make_shared<cvc::physics::ocean_surface>(sea);
//   w.add_field(water);
//
// Bodies then ride the swell and the wind sea the picture shows, are carried shoreward by the
// breakers and up the beach by the swash, read at world::field_time() (the clock's time once
// follow() has anchored, which is the time a renderer draws at). The sea is deterministic and the
// same on every platform (sea_surface.h), and its digest goes into state_hash().
//
// cvc::ocean depends only on the standard library and cvc::algo, so physics stays free of the
// rest of libcvc.

#ifndef __CVC_PHYSICS_OCEAN_SURFACE_H__
#define __CVC_PHYSICS_OCEAN_SURFACE_H__

#include <cvc/ocean/sea_surface.h>
#include <cvc/physics/fluid_surface.h>
#include <memory>

namespace cvc {
namespace physics {

class ocean_surface : public fluid_surface {
public:
  // Throws std::invalid_argument for a null sea.
  explicit ocean_surface(std::shared_ptr<const cvc::ocean::sea_surface> sea);
  double level(double x, double y) const override;
  double height(double x, double y, double t) const override;
  vec3 velocity(const vec3 &p, double t) const override;
  std::uint64_t digest() const override;

  const cvc::ocean::sea_surface &sea() const noexcept { return *_sea; }

private:
  std::shared_ptr<const cvc::ocean::sea_surface> _sea;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_OCEAN_SURFACE_H__
