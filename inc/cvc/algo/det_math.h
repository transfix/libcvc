/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// det_math.h — transcendental functions that give the same bits on every platform.
//
// IEEE 754 rounds +, -, *, / and sqrt correctly, so each of them gives the same bits on
// every target. libm's exp, log, pow, tan, atan, tanh and the rest are not correctly rounded,
// and each libm (MSVC's CRT, glibc, Apple's, musl in emscripten) misses in its own cases by an
// ULP. Anything that must reproduce across platforms therefore cannot call them:
//   * a cvc::physics step, whose state_hash a native run and a wasm run must agree on
//     (vehicle.cpp, backend_impl.h's Bekker sinkage, field_step.cpp, nav_plant.cpp);
//   * cvc::ocean's spectrum, h0 and shore layer, which physics will read (the
//     OCEAN-AND-WEATHER-ROADMAP's B5) and which must therefore be the same sea everywhere.
//
// These versions use only the correctly rounded operations and exact scaling (frexp, ldexp,
// floor, fmod, copysign), in a fixed order, from det_math.cpp, which is built without FMA
// contraction. They are within a few ULP of the true value over the ranges their callers use
// (algo_det_math_test measures each against libm), and they give the same bits wherever the
// build is IEEE double without contraction. A caller's own source needs no contraction either
// (-ffp-contract=off, /fp:precise), or its arithmetic around these calls differs instead.
//
// Standard library only, like the rest of cvc::algo.

#ifndef CVC_ALGO_DET_MATH_H
#define CVC_ALGO_DET_MATH_H

namespace cvc {
namespace algo {
namespace det {

double exp(double x);
double expm1(double x); // e^x - 1, accurate near 0
double log(double x);
double pow(double x, double y); // libm's special cases; y = 1, 2, -1 and 0.5 correctly rounded
double sinh(double x);
double cosh(double x);
double tanh(double x);
double sin(double x); // sin, cos and tan: accurate for |x| < 1e6
double cos(double x);
double tan(double x);
double atan(double x);
double atan2(double y, double x);
double asin(double x);
double acos(double x);
double hypot(double x, double y); // sqrt(x^2 + y^2) without overflow or underflow
double cbrt(double x);
double pow_two_thirds(double x); // x^(2/3), the cube root of x squared

} // namespace det
} // namespace algo
} // namespace cvc

#endif // CVC_ALGO_DET_MATH_H
