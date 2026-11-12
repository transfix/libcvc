/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See cvc/physics/fluid_surface.h. Built without FMA contraction, and its transcendentals are
// det_math's, so a surface gives the same bits on every platform.

#include "det_math.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/physics/fluid_surface.h>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace physics {

namespace {

// 64-bit FNV-1a over a double's bits.
struct digester {
  std::uint64_t h = 1469598103934665603ull;
  void add(const void *p, std::size_t n) {
    const auto *b = static_cast<const unsigned char *>(p);
    for (std::size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 1099511628211ull;
  }
  void add(double v) { add(&v, sizeof v); }
  void add(const char *tag) { add(tag, std::strlen(tag)); }
};

// k for omega^2 = g k tanh(k d): Newton's method from Eckart's approximation, a fixed number of
// steps (each squares the error; Eckart's start is within 5 %).
double wavenumber_for(double omega, double depth, double g) {
  const double k0 = omega * omega / g;
  if (!(depth > 0.0) || k0 * depth > 20.0)
    return k0; // deep water: tanh(k d) is 1 to a double
  double k = k0 / std::sqrt(det::tanh(k0 * depth));
  for (int i = 0; i < 8; ++i) {
    const double kd = k * depth, th = det::tanh(kd);
    const double f = g * k * th - omega * omega;
    const double df = g * th + g * kd * (1.0 - th * th);
    k -= f / df;
  }
  return k;
}

} // namespace

// --- still_surface ------------------------------------------------------------------------------

still_surface::still_surface(double level, const vec3 &current) : _level(level), _current(current) {
  if (!std::isfinite(level) || !std::isfinite(current.x) || !std::isfinite(current.y) ||
      !std::isfinite(current.z))
    throw std::invalid_argument("still_surface: the level and current must be finite");
}

double still_surface::level(double, double) const { return _level; }
double still_surface::height(double, double, double) const { return _level; }
vec3 still_surface::velocity(const vec3 &, double) const { return _current; }

std::uint64_t still_surface::digest() const {
  digester d;
  d.add("still_surface");
  d.add(_level);
  d.add(_current.x);
  d.add(_current.y);
  d.add(_current.z);
  return d.h;
}

// --- wave_train_surface -------------------------------------------------------------------------

wave_train_surface::wave_train_surface(double level, double depth_m, std::vector<wave_train> trains,
                                       double gravity)
    : _level(level), _depth(depth_m > 0.0 ? depth_m : 0.0), _gravity(gravity),
      _trains(std::move(trains)) {
  if (!std::isfinite(level) || !std::isfinite(depth_m))
    throw std::invalid_argument("wave_train_surface: the level and depth must be finite");
  if (!(gravity > 0.0) || !std::isfinite(gravity))
    throw std::invalid_argument("wave_train_surface: gravity must be finite and > 0");
  constexpr double kTau = 6.283185307179586;
  for (const wave_train &w : _trains) {
    if (!(w.amplitude >= 0.0) || !std::isfinite(w.amplitude) || !(w.period_s > 0.0) ||
        !std::isfinite(w.period_s) || !std::isfinite(w.dir_rad) || !std::isfinite(w.phase))
      throw std::invalid_argument("wave_train_surface: a train needs a finite amplitude >= 0, "
                                  "period > 0, direction and phase");
    const double omega = kTau / w.period_s;
    const double k = wavenumber_for(omega, _depth, _gravity);
    _omega.push_back(omega);
    _k.push_back(k);
    _kx.push_back(k * det::cos(w.dir_rad));
    _ky.push_back(k * det::sin(w.dir_rad));
  }
}

double wave_train_surface::level(double, double) const { return _level; }

double wave_train_surface::height(double x, double y, double t) const {
  double eta = 0.0;
  for (std::size_t i = 0; i < _trains.size(); ++i) {
    const wave_train &w = _trains[i];
    eta += w.amplitude * det::cos(_kx[i] * x + _ky[i] * y - _omega[i] * t + w.phase);
  }
  return _level + eta;
}

vec3 wave_train_surface::velocity(const vec3 &p, double t) const {
  vec3 v;
  // Depth below the still level, kept between the bottom and the level.
  double z = std::min(p.z - _level, 0.0);
  if (_depth > 0.0)
    z = std::max(z, -_depth);
  for (std::size_t i = 0; i < _trains.size(); ++i) {
    const wave_train &w = _trains[i];
    const double k = _k[i], kd = k * _depth;
    // The decay of the horizontal and vertical orbital speeds with depth.
    double ch, sh;
    if (!(_depth > 0.0) || kd > 20.0) {
      ch = sh = det::exp(k * z);
    } else {
      const double s = det::sinh(kd);
      ch = det::cosh(k * (z + _depth)) / s;
      sh = det::sinh(k * (z + _depth)) / s;
    }
    const double arg = _kx[i] * p.x + _ky[i] * p.y - _omega[i] * t + w.phase;
    const double aw = w.amplitude * _omega[i];
    const double u = aw * ch * det::cos(arg);
    v.x += u * (_kx[i] / k);
    v.y += u * (_ky[i] / k);
    v.z += aw * sh * det::sin(arg);
  }
  return v;
}

std::uint64_t wave_train_surface::digest() const {
  digester d;
  d.add("wave_train_surface");
  d.add(_level);
  d.add(_depth);
  d.add(_gravity);
  for (const wave_train &w : _trains) {
    d.add(w.amplitude);
    d.add(w.period_s);
    d.add(w.dir_rad);
    d.add(w.phase);
  }
  return d.h;
}

} // namespace physics
} // namespace cvc
