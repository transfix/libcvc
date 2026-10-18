/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::det (src/cvc/physics/det_math.h), the libm-free tan, atan, tanh, expm1 and
// x^(2/3) a physics step calls: within a few ULP of libm over the ranges the step uses (libm is
// not correctly rounded either, so the bounds are the distance measured from MSVC's, with no
// margin), libm's answers at the edges (NaN, infinities, zeros, the limits), and the same bits
// on every platform. The last is what they are for, so the Bits test pins a few outputs measured on
// Windows (MSVC, x86_64); a platform that disagrees must fix det_math, never re-baseline.
// det_math.cpp is compiled into this test (it is internal to libcvc).

#include "det_math.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>

namespace det = cvc::physics::det;

namespace {

// The distance in ULPs between two finite doubles of the same sign (or both zero).
std::int64_t ulps(double a, double b) {
  if (a == b)
    return 0;
  std::int64_t ia, ib;
  std::memcpy(&ia, &a, sizeof a);
  std::memcpy(&ib, &b, sizeof b);
  if (ia < 0)
    ia = std::numeric_limits<std::int64_t>::min() - ia;
  if (ib < 0)
    ib = std::numeric_limits<std::int64_t>::min() - ib;
  return ia > ib ? ia - ib : ib - ia;
}

std::uint64_t bits(double x) {
  std::uint64_t u;
  std::memcpy(&u, &x, sizeof x);
  return u;
}

// The worst distance from `ref` over n points evenly across [lo, hi].
template <class F, class G> std::int64_t worst(F f, G ref, double lo, double hi, int n = 20001) {
  std::int64_t w = 0;
  for (int i = 0; i < n; ++i) {
    const double x = lo + (hi - lo) * i / (n - 1);
    const std::int64_t d = ulps(f(x), ref(x));
    if (d > w)
      w = d;
  }
  return w;
}

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

} // namespace

TEST(PhysicsDetMath, Tan) {
  // The steering actuator's angles (a vehicle's max_steer is under 1 rad), and beyond.
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -1.2, 1.2), 3);
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -1e-6, 1e-6), 1);
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -100.0, 100.0, 4001), 8);
  EXPECT_EQ(bits(det::tan(0.0)), bits(0.0));
  EXPECT_EQ(bits(det::tan(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::tan(kNan)));
  EXPECT_TRUE(std::isnan(det::tan(kInf)));
}

TEST(PhysicsDetMath, Atan) {
  // Ackermann's wheel angles: atan of a ratio of lengths, any size.
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -4.0, 4.0), 4);
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -1e-5, 1e-5), 2);
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -1e6, 1e6), 2);
  EXPECT_EQ(det::atan(kInf), std::atan(kInf));
  EXPECT_EQ(det::atan(-kInf), std::atan(-kInf));
  EXPECT_EQ(bits(det::atan(-0.0)), bits(-0.0));
  EXPECT_LE(ulps(det::atan(1.0), std::atan(1.0)), 1);
  EXPECT_TRUE(std::isnan(det::atan(kNan)));
}

TEST(PhysicsDetMath, Tanh) {
  // The rolling-resistance sign, tanh(v / 0.1 m/s), and its tails.
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -3.0, 3.0), 3);
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -1e-6, 1e-6), 2);
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -40.0, 40.0), 3);
  EXPECT_EQ(det::tanh(1000.0), 1.0);
  EXPECT_EQ(det::tanh(-kInf), -1.0);
  EXPECT_EQ(bits(det::tanh(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::tanh(kNan)));
}

TEST(PhysicsDetMath, Expm1) {
  // A field's implicit drag, -expm1(-x) / x for its drag rate x >= 0; near 0 it must keep its
  // digits, which exp(x) - 1 would not.
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -50.0, 5.0), 3);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -1e-8, 1e-8), 2);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -0.5, 0.5), 3);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, 600.0, 709.0, 2001), 3);
  EXPECT_EQ(det::expm1(-kInf), -1.0);
  EXPECT_EQ(det::expm1(kInf), kInf);
  EXPECT_EQ(det::expm1(800.0), kInf);
  EXPECT_EQ(bits(det::expm1(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::expm1(kNan)));
}

TEST(PhysicsDetMath, CbrtAndTwoThirds) {
  // Bekker sinkage, (3 W / (2 b k sqrt(D)))^(2/3): tiny to large arguments. The reference for
  // x^(2/3) is libm's cbrt(x^2): std::pow(x, 2.0 / 3.0) is itself several ULP from x^(2/3) for
  // a small x, since 2.0 / 3.0 is not 2/3 (the error is ln x times 3.7e-17).
  const auto cbrt_sq = [](double x) { return std::cbrt(x * x); };
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, -8.0, 8.0), 2);
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, 1e-12, 1e-9), 2);
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, 1e6, 1e9), 2);
  EXPECT_LE(worst(det::pow_two_thirds, cbrt_sq, 1e-9, 1e-3), 2);
  EXPECT_LE(worst(det::pow_two_thirds, cbrt_sq, 0.0, 10.0), 2);
  EXPECT_EQ(det::cbrt(27.0), 3.0);
  EXPECT_EQ(det::cbrt(-0.125), -0.5);
  EXPECT_LE(ulps(det::cbrt(std::ldexp(1.0, -1074)), std::cbrt(std::ldexp(1.0, -1074))), 2);
  EXPECT_EQ(det::pow_two_thirds(0.0), 0.0);
  EXPECT_EQ(det::cbrt(kInf), kInf);
  EXPECT_TRUE(std::isnan(det::cbrt(kNan)));
}

TEST(PhysicsDetMath, SinCosAtan2Asin) {
  // nav_plant's spawn rotation (sin, cos of a heading and its half) and the pose it reports
  // (atan2 of the forward direction, asin of its rise).
  EXPECT_LE(worst(det::sin, [](double x) { return std::sin(x); }, -7.0, 7.0), 2);
  EXPECT_LE(worst(det::cos, [](double x) { return std::cos(x); }, -7.0, 7.0), 2);
  EXPECT_LE(worst(det::asin, [](double x) { return std::asin(x); }, -0.999, 0.999), 4);
  std::int64_t w = 0;
  for (int i = 0; i < 3600; ++i) {
    const double a = -3.14159 + 6.28318 * i / 3599.0;
    const double y = 2.5 * std::sin(a), x = 2.5 * std::cos(a);
    const std::int64_t d = ulps(det::atan2(y, x), std::atan2(y, x));
    if (d > w)
      w = d;
  }
  EXPECT_LE(w, 3) << "atan2 around the circle";
  // The edges, as std::atan2 gives them: exactly on the axes and at infinity, within an ULP on
  // the diagonals (pi/4 and its multiples are not representable).
  for (const double y : {0.0, -0.0, 1.0, -1.0, kInf, -kInf})
    for (const double x : {0.0, -0.0, 1.0, -1.0, kInf, -kInf}) {
      const bool exact = y == 0.0 || x == 0.0 || std::isinf(x) || std::isinf(y);
      if (exact)
        EXPECT_EQ(bits(det::atan2(y, x)), bits(std::atan2(y, x)))
            << "atan2(" << y << ", " << x << ")";
      else
        EXPECT_LE(ulps(det::atan2(y, x), std::atan2(y, x)), 1) << "atan2(" << y << ", " << x << ")";
    }
  EXPECT_EQ(det::asin(1.0), std::asin(1.0));
  EXPECT_EQ(det::asin(-1.0), std::asin(-1.0));
  EXPECT_EQ(bits(det::sin(-0.0)), bits(-0.0));
  EXPECT_EQ(det::cos(0.0), 1.0);
  EXPECT_TRUE(std::isnan(det::asin(1.5)));
  EXPECT_TRUE(std::isnan(det::sin(kInf)));
  EXPECT_TRUE(std::isnan(det::atan2(kNan, 1.0)));
}

// The point of det_math: these bits on every platform. Measured on Windows (MSVC, x86_64).
TEST(PhysicsDetMath, Bits) {
  EXPECT_EQ(bits(det::tan(0.3)), 0x3fd3cc2a44e29997ull);
  EXPECT_EQ(bits(det::tan(-0.0625)), 0xbfb005577854df01ull);
  EXPECT_EQ(bits(det::atan(0.7)), 0x3fe38b112d7bd4adull);
  EXPECT_EQ(bits(det::atan(-12.5)), 0xbff7daff85a63058ull);
  EXPECT_EQ(bits(det::tanh(0.05)), 0x3fa99424e535f6f8ull);
  EXPECT_EQ(bits(det::tanh(-2.5)), 0xbfef9258260a71c2ull);
  EXPECT_EQ(bits(det::expm1(-0.01)), 0xbf8460c0b518c09full);
  EXPECT_EQ(bits(det::expm1(3.75)), 0x4044c2b2d0a266e7ull);
  EXPECT_EQ(bits(det::pow_two_thirds(1234.5)), 0x405cc50594de128full);
  EXPECT_EQ(bits(det::pow_two_thirds(3.0e-7)), 0x3f077ed6f23574c2ull);
  EXPECT_EQ(bits(det::sin(0.7)), 0x3fe49d6e694619b8ull);
  EXPECT_EQ(bits(det::cos(2.5)), 0xbfe9a2f7ef858b7dull);
  EXPECT_EQ(bits(det::atan2(-0.3, -0.8)), 0xc00643382c07913aull);
  EXPECT_EQ(bits(det::asin(0.4)), 0x3fda564ac0e73a34ull);
}
