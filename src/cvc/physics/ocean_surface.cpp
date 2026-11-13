/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/physics/ocean_surface.h>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace physics {

namespace {

using sea_memo = cvc::ocean::sea_surface::source_memo;

// The caller's memo as the sea's, and back: the same fields.
sea_memo to_sea(const surface_memo &m) {
  sea_memo s;
  s.valid = m.valid;
  s.digest = m.digest;
  s.x = m.x;
  s.y = m.y;
  s.t = m.t;
  s.x0 = {m.found[0], m.found[1]};
  return s;
}

void keep(surface_memo &m, const sea_memo &s) {
  m.valid = s.valid;
  m.digest = s.digest;
  m.x = s.x;
  m.y = s.y;
  m.t = s.t;
  m.found[0] = s.x0[0];
  m.found[1] = s.x0[1];
}

} // namespace

ocean_surface::ocean_surface(std::shared_ptr<const cvc::ocean::sea_surface> sea)
    : _sea(std::move(sea)) {
  if (!_sea)
    throw std::invalid_argument("ocean_surface: the sea is null");
}

double ocean_surface::level(double, double) const { return _sea->params().level; }

double ocean_surface::height(double x, double y, double t) const { return _sea->height(x, y, t); }

vec3 ocean_surface::velocity(const vec3 &p, double t) const {
  const std::array<double, 3> v = _sea->velocity(p.x, p.y, p.z, t);
  return {v[0], v[1], v[2]};
}

double ocean_surface::height_with(double x, double y, double t, surface_memo &memo) const {
  sea_memo s = to_sea(memo);
  const double h = _sea->height(x, y, t, s);
  keep(memo, s);
  return h;
}

vec3 ocean_surface::velocity_with(const vec3 &p, double t, surface_memo &memo) const {
  sea_memo s = to_sea(memo);
  const std::array<double, 3> v = _sea->velocity(p.x, p.y, p.z, t, s);
  keep(memo, s);
  return {v[0], v[1], v[2]};
}

std::uint64_t ocean_surface::digest() const {
  // A tag of its own, so no other surface type shares a digest with it.
  return _sea->digest() ^ 0x6f6365616e5f7366ull; // "ocean_sf"
}

} // namespace physics
} // namespace cvc
