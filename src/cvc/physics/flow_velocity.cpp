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

// flow_velocity.cpp — the wind as a flow field's velocity (see flow_velocity.h).

#include <cmath>
#include <cstring>
#include <cvc/algo/det_math.h>
#include <cvc/physics/flow_velocity.h>
#include <stdexcept>

namespace cvc {
namespace physics {

namespace det = ::cvc::algo::det;

wind_flow::wind_flow(const cvc::algo::wind_state &w, double base_z) : _w(w), _base(base_z) {
  if (!std::isfinite(w.speed_mps) || !std::isfinite(w.dir_rad) || !std::isfinite(w.gustiness) ||
      !std::isfinite(base_z) || w.speed_mps < 0.0 || w.gustiness < 0.0)
    throw std::invalid_argument("wind_flow: the wind must be finite, its speed and gustiness >= 0");
  _cx = det::cos(w.dir_rad);
  _cy = det::sin(w.dir_rad);
  std::uint64_t h = 1469598103934665603ull;
  auto add = [&h](const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 1099511628211ull;
  };
  const char *tag = "cvc::physics::wind_flow v1";
  add(tag, std::strlen(tag));
  const double v[4] = {w.speed_mps, w.dir_rad, w.gustiness, base_z};
  add(v, sizeof v);
  _digest = h;
}

vec3 wind_flow::velocity(const vec3 &p, double t) const {
  const double prof = profile(p.z);
  if (!(prof > 0.0))
    return {};
  const vec3 r = reference(p, t);
  return {r.x * prof, r.y * prof, 0.0};
}

double wind_flow::profile(double z) const { return cvc::algo::wind_profile(z - _base); }

vec3 wind_flow::reference(const vec3 &p, double t) const {
  if (!(_w.speed_mps > 0.0))
    return {};
  cvc::algo::gust_frame f;
  {
    std::lock_guard<std::mutex> lock(_mutex);
    if (!_have || std::memcmp(&_t, &t, sizeof t) != 0) {
      _frame = cvc::algo::gust_frame_at(_w, t);
      _t = t;
      _have = true;
    }
    f = _frame;
  }
  const double s = _w.speed_mps * cvc::algo::wind_gust(f, p.x, p.y);
  return {s * _cx, s * _cy, 0.0};
}

std::uint64_t wind_flow::digest() const { return _digest; }

} // namespace physics
} // namespace cvc
