/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The transcendental functions a cvc::physics step calls, computed the same way on every
// platform (internal; not installed).
//
// IEEE 754 rounds +, -, *, / and sqrt correctly, so each of them gives the same bits on
// every target. libm's tan, atan, tanh, expm1 and pow are not correctly rounded, and each libm
// (MSVC's CRT, glibc, Apple's, musl in emscripten) misses in its own cases by an ULP. A step
// that calls them every frame therefore drifts apart across platforms within a few hundred
// steps, though Jolt is cross-platform deterministic. These versions use only the correctly
// rounded operations and exact scaling (frexp, ldexp, floor, copysign), in a fixed order,
// from det_math.cpp, which is built without FMA contraction. They are within a few ULP of the
// true value over the ranges the step uses (physics_det_math_test measures it), and they give
// the same bits wherever the build is IEEE double without contraction.
//
// Everything that feeds the state_hash calls these, never std::'s: vehicle.cpp (the steering
// actuator, Ackermann, the rolling-resistance sign), backend_impl.h (Bekker sinkage),
// field_step.cpp (the fields' implicit drag, a hull's drag area) and nav_plant.cpp (a spawned
// vehicle's rotation, and the pose it reports to the navigator).

#ifndef __CVC_PHYSICS_DET_MATH_H__
#define __CVC_PHYSICS_DET_MATH_H__

namespace cvc {
namespace physics {
namespace det {

double expm1(double x); // e^x - 1, accurate near 0
double tanh(double x);
double sin(double x); // sin, cos and tan: accurate for |x| < 1e6; the step's angles are well
double cos(double x); // inside
double tan(double x);
double atan(double x);
double atan2(double y, double x);
double asin(double x);
double cbrt(double x);
double pow_two_thirds(double x); // x^(2/3), the cube root of x squared

} // namespace det
} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_DET_MATH_H__
