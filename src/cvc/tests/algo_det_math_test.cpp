/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::algo::det (cvc/algo/det_math.h), the libm-free functions a physics step and cvc::ocean
// call:
//   - within a few ULP of libm over the ranges their callers use. libm is not correctly rounded
//     either and misses in its own places, so a bound is the most measured against MSVC's and
//     glibc's libm, plus one ULP for the next libm's own error;
//   - libm's answers at the edges (NaN, infinities, zeros, the limits);
//   - the same bits on every platform, which is what they are for: the Bits test pins outputs
//     measured on Windows (MSVC, x86_64), and a platform that disagrees must fix det_math, never
//     re-baseline.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/algo/det_math.h>
#include <gtest/gtest.h>
#include <limits>

namespace det = cvc::algo::det;

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

// The same over n points spaced geometrically across [lo, hi], 0 < lo < hi: every magnitude.
template <class F, class G>
std::int64_t worst_log(F f, G ref, double lo, double hi, int n = 20001) {
  std::int64_t w = 0;
  const double llo = std::log(lo), lhi = std::log(hi);
  for (int i = 0; i < n; ++i) {
    const double x = std::exp(llo + (lhi - llo) * i / (n - 1));
    const std::int64_t d = ulps(f(x), ref(x));
    if (d > w)
      w = d;
  }
  return w;
}

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// Equal bits, or both NaN.
bool same(double a, double b) { return (a != a && b != b) || bits(a) == bits(b); }

} // namespace

TEST(AlgoDetMath, Tan) {
  // The steering actuator's angles (a vehicle's max_steer is under 1 rad), and beyond.
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -1.2, 1.2), 4);
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -1e-6, 1e-6), 2);
  EXPECT_LE(worst(det::tan, [](double x) { return std::tan(x); }, -100.0, 100.0, 4001), 9);
  EXPECT_EQ(bits(det::tan(0.0)), bits(0.0));
  EXPECT_EQ(bits(det::tan(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::tan(kNan)));
  EXPECT_TRUE(std::isnan(det::tan(kInf)));
}

TEST(AlgoDetMath, Atan) {
  // Ackermann's wheel angles: atan of a ratio of lengths, any size.
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -4.0, 4.0), 5);
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -1e-5, 1e-5), 3);
  EXPECT_LE(worst(det::atan, [](double x) { return std::atan(x); }, -1e6, 1e6), 3);
  EXPECT_EQ(det::atan(kInf), std::atan(kInf));
  EXPECT_EQ(det::atan(-kInf), std::atan(-kInf));
  EXPECT_EQ(bits(det::atan(-0.0)), bits(-0.0));
  EXPECT_LE(ulps(det::atan(1.0), std::atan(1.0)), 2);
  EXPECT_TRUE(std::isnan(det::atan(kNan)));
}

TEST(AlgoDetMath, Tanh) {
  // The rolling-resistance sign, tanh(v / 0.1 m/s), and its tails.
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -3.0, 3.0), 4);
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -1e-6, 1e-6), 4);
  EXPECT_LE(worst(det::tanh, [](double x) { return std::tanh(x); }, -40.0, 40.0), 4);
  EXPECT_EQ(det::tanh(1000.0), 1.0);
  EXPECT_EQ(det::tanh(-kInf), -1.0);
  EXPECT_EQ(bits(det::tanh(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::tanh(kNan)));
}

TEST(AlgoDetMath, Expm1) {
  // A field's implicit drag, -expm1(-x) / x for its drag rate x >= 0; near 0 it must keep its
  // digits, which exp(x) - 1 would not.
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -50.0, 5.0), 4);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -1e-8, 1e-8), 3);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, -0.5, 0.5), 4);
  EXPECT_LE(worst(det::expm1, [](double x) { return std::expm1(x); }, 600.0, 709.0, 2001), 4);
  EXPECT_EQ(det::expm1(-kInf), -1.0);
  EXPECT_EQ(det::expm1(kInf), kInf);
  EXPECT_EQ(det::expm1(800.0), kInf);
  EXPECT_EQ(bits(det::expm1(-0.0)), bits(-0.0));
  EXPECT_TRUE(std::isnan(det::expm1(kNan)));
}

TEST(AlgoDetMath, CbrtAndTwoThirds) {
  // Bekker sinkage, (3 W / (2 b k sqrt(D)))^(2/3): tiny to large arguments. The reference for
  // x^(2/3) is libm's cbrt(x^2): std::pow(x, 2.0 / 3.0) is itself several ULP from x^(2/3) for
  // a small x, since 2.0 / 3.0 is not 2/3 (the error is ln x times 3.7e-17).
  const auto cbrt_sq = [](double x) { return std::cbrt(x * x); };
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, -8.0, 8.0), 4);
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, 1e-12, 1e-9), 4);
  EXPECT_LE(worst(det::cbrt, [](double x) { return std::cbrt(x); }, 1e6, 1e9), 4);
  EXPECT_LE(worst(det::pow_two_thirds, cbrt_sq, 1e-9, 1e-3), 4);
  EXPECT_LE(worst(det::pow_two_thirds, cbrt_sq, 0.0, 10.0), 4);
  EXPECT_EQ(det::cbrt(27.0), 3.0);
  EXPECT_EQ(det::cbrt(-0.125), -0.5);
  EXPECT_LE(ulps(det::cbrt(std::ldexp(1.0, -1074)), std::cbrt(std::ldexp(1.0, -1074))), 4);
  EXPECT_EQ(det::pow_two_thirds(0.0), 0.0);
  EXPECT_EQ(det::cbrt(kInf), kInf);
  EXPECT_TRUE(std::isnan(det::cbrt(kNan)));
}

TEST(AlgoDetMath, SinCosAtan2Asin) {
  // nav_plant's spawn rotation (sin, cos of a heading and its half) and the pose it reports
  // (atan2 of the forward direction, asin of its rise).
  EXPECT_LE(worst(det::sin, [](double x) { return std::sin(x); }, -7.0, 7.0), 3);
  EXPECT_LE(worst(det::cos, [](double x) { return std::cos(x); }, -7.0, 7.0), 3);
  EXPECT_LE(worst(det::asin, [](double x) { return std::asin(x); }, -0.999, 0.999), 5);
  std::int64_t w = 0;
  for (int i = 0; i < 3600; ++i) {
    const double a = -3.14159 + 6.28318 * i / 3599.0;
    const double y = 2.5 * std::sin(a), x = 2.5 * std::cos(a);
    const std::int64_t d = ulps(det::atan2(y, x), std::atan2(y, x));
    if (d > w)
      w = d;
  }
  EXPECT_LE(w, 4) << "atan2 around the circle";
  // The edges, as std::atan2 gives them: exactly on the axes and at infinity, within an ULP on
  // the diagonals (pi/4 and its multiples are not representable).
  for (const double y : {0.0, -0.0, 1.0, -1.0, kInf, -kInf})
    for (const double x : {0.0, -0.0, 1.0, -1.0, kInf, -kInf}) {
      const bool exact = y == 0.0 || x == 0.0 || std::isinf(x) || std::isinf(y);
      if (exact)
        EXPECT_EQ(bits(det::atan2(y, x)), bits(std::atan2(y, x)))
            << "atan2(" << y << ", " << x << ")";
      else
        EXPECT_LE(ulps(det::atan2(y, x), std::atan2(y, x)), 2) << "atan2(" << y << ", " << x << ")";
    }
  EXPECT_EQ(det::asin(1.0), std::asin(1.0));
  EXPECT_EQ(det::asin(-1.0), std::asin(-1.0));
  EXPECT_EQ(bits(det::sin(-0.0)), bits(-0.0));
  EXPECT_EQ(det::cos(0.0), 1.0);
  EXPECT_TRUE(std::isnan(det::asin(1.5)));
  EXPECT_TRUE(std::isnan(det::sin(kInf)));
  EXPECT_TRUE(std::isnan(det::atan2(kNan, 1.0)));
}

TEST(AlgoDetMath, ExpAndLog) {
  // cvc::ocean's spectrum: the JONSWAP peak's e^(-x^2 / 2 sigma^2) and e^(-1.25 (wp/w)^4), the
  // fetch law's logs, Box-Muller's log of a uniform in (0, 1].
  const auto std_exp = [](double x) { return std::exp(x); };
  const auto std_log = [](double x) { return std::log(x); };
  EXPECT_LE(worst(det::exp, std_exp, -1.0, 1.0), 2);
  EXPECT_LE(worst(det::exp, std_exp, -1e-8, 1e-8), 2);
  EXPECT_LE(worst(det::exp, std_exp, -700.0, 709.0), 2);
  EXPECT_LE(worst(det::exp, std_exp, -745.0, -708.5, 4001), 2) << "into the subnormals";
  EXPECT_LE(worst_log(det::log, std_log, 1e-300, 1e300), 2);
  EXPECT_LE(worst(det::log, std_log, 0.5, 2.0), 2);
  EXPECT_LE(worst(det::log, std_log, 1.0 - 1e-6, 1.0 + 1e-6), 2) << "near 1, where log is small";
  EXPECT_LE(worst_log(det::log, std_log, 4.9e-324, 2.2e-308, 4001), 2) << "subnormals";
  EXPECT_EQ(bits(det::exp(0.0)), bits(1.0));
  EXPECT_EQ(det::exp(-kInf), 0.0);
  EXPECT_EQ(det::exp(kInf), kInf);
  EXPECT_EQ(det::exp(710.0), kInf);
  EXPECT_EQ(det::exp(-746.0), 0.0);
  EXPECT_TRUE(std::isnan(det::exp(kNan)));
  EXPECT_EQ(bits(det::log(1.0)), bits(0.0));
  EXPECT_EQ(det::log(0.0), -kInf);
  EXPECT_EQ(det::log(-0.0), -kInf);
  EXPECT_EQ(det::log(kInf), kInf);
  EXPECT_TRUE(std::isnan(det::log(-1.0)));
  EXPECT_TRUE(std::isnan(det::log(-kInf)));
  EXPECT_TRUE(std::isnan(det::log(kNan)));
}

TEST(AlgoDetMath, Pow) {
  // The spectrum's powers: an integer one (w^5, (wp/w)^4), the swell's directional cos^2s, the
  // JONSWAP peak gamma^r (r in (0, 1]), the fetch law's x^0.22 and the wave age's r^-2.5. log x
  // is carried in two parts and multiplied by y exactly, so a large y log x keeps its digits.
  std::int64_t w = 0;
  for (int i = 0; i <= 200; ++i)
    for (int j = 0; j <= 120; ++j) {
      const double x = std::exp(-6.9 + 13.8 * i / 200.0); // 1e-3 .. 1e3
      const double y = -3.0 + 6.0 * j / 120.0;
      const std::int64_t d = ulps(det::pow(x, y), std::pow(x, y));
      if (d > w)
        w = d;
    }
  EXPECT_LE(w, 2) << "x in [1e-3, 1e3], y in [-3, 3]";
  w = 0;
  for (int i = 0; i <= 200; ++i)
    for (int j = 0; j <= 100; ++j) {
      const double x = std::exp(-11.5 + 11.5 * i / 200.0); // 1e-5 .. 1: cos^2s
      const double y = 60.0 * j / 100.0;
      const std::int64_t d = ulps(det::pow(x, y), std::pow(x, y));
      if (d > w)
        w = d;
    }
  EXPECT_LE(w, 3) << "x in [1e-5, 1], y in [0, 60], y log x down to -690";
  w = 0;
  for (int i = 0; i <= 400; ++i)
    for (int n = -8; n <= 8; ++n) {
      const double x = -20.0 + 40.0 * i / 400.0;
      const std::int64_t d = ulps(det::pow(x, n), std::pow(x, n));
      if (d > w)
        w = d;
    }
  EXPECT_LE(w, 2) << "integer exponents";
  for (const double x : {0.3, 1.7, -2.5, 1e-200, 123456.789}) {
    EXPECT_EQ(bits(det::pow(x, 1.0)), bits(x));
    EXPECT_EQ(bits(det::pow(x, 2.0)), bits(x * x));
    EXPECT_EQ(bits(det::pow(x, -1.0)), bits(1.0 / x));
    if (x > 0.0)
      EXPECT_EQ(bits(det::pow(x, 0.5)), bits(std::sqrt(x)));
  }
  // libm's special cases (C99 Annex F): the same bits, or both NaN. Where the answer is not one
  // of 0, 1, inf or NaN it is a few ULP off, as above.
  const double vals[] = {0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 3.0, -3.0, kInf, -kInf, kNan};
  for (const double x : vals)
    for (const double y : vals) {
      const double d = det::pow(x, y), s = std::pow(x, y);
      const double a = std::fabs(s);
      if (a == 0.0 || a == 1.0 || std::isinf(s) || s != s)
        EXPECT_TRUE(same(d, s)) << "pow(" << x << ", " << y << ") = " << d << ", libm " << s;
      else
        EXPECT_LE(ulps(d, s), 2) << "pow(" << x << ", " << y << ")";
    }
  EXPECT_TRUE(std::isnan(det::pow(-8.0, 1.0 / 3.0)));
  EXPECT_EQ(det::pow(10.0, 400.0), kInf);
  EXPECT_EQ(det::pow(10.0, -400.0), 0.0);
}

TEST(AlgoDetMath, SinhAndCosh) {
  // Finite-depth dispersion's sech^2 and the orbital velocity's cosh(k(z + d)) / sinh(kd).
  const auto std_sinh = [](double x) { return std::sinh(x); };
  const auto std_cosh = [](double x) { return std::cosh(x); };
  EXPECT_LE(worst(det::sinh, std_sinh, -3.0, 3.0), 3);
  EXPECT_LE(worst(det::sinh, std_sinh, -1e-6, 1e-6), 3);
  EXPECT_LE(worst(det::sinh, std_sinh, -710.0, 710.0), 3);
  EXPECT_LE(worst(det::cosh, std_cosh, -3.0, 3.0), 3);
  EXPECT_LE(worst(det::cosh, std_cosh, -1e-3, 1e-3), 2);
  EXPECT_LE(worst(det::cosh, std_cosh, -710.0, 710.0), 3);
  EXPECT_EQ(bits(det::sinh(-0.0)), bits(-0.0));
  EXPECT_EQ(det::cosh(0.0), 1.0);
  EXPECT_EQ(det::sinh(kInf), kInf);
  EXPECT_EQ(det::sinh(-kInf), -kInf);
  EXPECT_EQ(det::cosh(-kInf), kInf);
  EXPECT_EQ(det::sinh(711.0), kInf);
  EXPECT_EQ(det::cosh(711.0), kInf);
  EXPECT_TRUE(std::isnan(det::sinh(kNan)));
  EXPECT_TRUE(std::isnan(det::cosh(kNan)));
}

TEST(AlgoDetMath, AcosAndHypot) {
  // The swash's wet window, acos of a height ratio; a wave vector's length.
  const auto std_acos = [](double x) { return std::acos(x); };
  EXPECT_LE(worst(det::acos, std_acos, -0.999, 0.999), 5);
  EXPECT_LE(worst(det::acos, std_acos, 0.999999, 1.0, 4001), 3);
  EXPECT_LE(worst(det::acos, std_acos, -1.0, -0.999999, 4001), 2);
  EXPECT_EQ(bits(det::acos(1.0)), bits(0.0));
  EXPECT_LE(ulps(det::acos(-1.0), std::acos(-1.0)), 1);
  EXPECT_LE(ulps(det::acos(0.0), std::acos(0.0)), 1);
  EXPECT_TRUE(std::isnan(det::acos(1.5)));
  EXPECT_TRUE(std::isnan(det::acos(kNan)));
  std::int64_t w = 0;
  for (int i = 0; i <= 300; ++i)
    for (int j = 0; j <= 300; ++j) {
      const double x = -100.0 + 200.0 * i / 300.0, y = -7.0 + 31.0 * j / 300.0;
      const std::int64_t d = ulps(det::hypot(x, y), std::hypot(x, y));
      if (d > w)
        w = d;
    }
  EXPECT_LE(w, 2);
  EXPECT_EQ(det::hypot(3.0, 4.0), 5.0);
  EXPECT_EQ(det::hypot(-3.0, 0.0), 3.0);
  EXPECT_EQ(bits(det::hypot(0.0, -0.0)), bits(0.0));
  EXPECT_LE(ulps(det::hypot(1e300, 1e300), std::hypot(1e300, 1e300)), 1) << "no overflow";
  EXPECT_LE(ulps(det::hypot(3e-320, 4e-320), std::hypot(3e-320, 4e-320)), 1) << "no underflow";
  EXPECT_EQ(det::hypot(kInf, kNan), kInf);
  EXPECT_EQ(det::hypot(kNan, -kInf), kInf);
  EXPECT_TRUE(std::isnan(det::hypot(kNan, 1.0)));
}

// The point of det_math: these bits on every platform. Measured on Windows (MSVC, x86_64).
TEST(AlgoDetMath, Bits) {
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
  // cvc::ocean's.
  const struct {
    const char *what;
    double v;
    std::uint64_t want;
  } more[] = {
      {"exp(1.5)", det::exp(1.5), 0x4011ed3fe64fc541ull},
      {"exp(-20.25)", det::exp(-20.25), 0x3e1b93de1e27ca3bull},
      {"log(7.25)", det::log(7.25), 0x3fffb22e98a1c24eull},
      {"log(0.003)", det::log(0.003), 0xc0173c8ffae4bedeull},
      {"pow(3.3, 0.7)", det::pow(3.3, 0.7), 0x400273c7cfb01256ull},
      {"pow(0.42, -2.5)", det::pow(0.42, -2.5), 0x40217ea55f4ad09full},
      {"pow(1.7, 5)", det::pow(1.7, 5.0), 0x402c65aaf78feef5ull},
      {"sinh(0.3)", det::sinh(0.3), 0x3fd37d42af54b926ull},
      {"cosh(4.5)", det::cosh(4.5), 0x404681ceb0641358ull},
      {"acos(0.2)", det::acos(0.2), 0x3ff5e9383efad0d0ull},
      {"hypot(0.3, 1.7)", det::hypot(0.3, 1.7), 0x3ffb9ecad3d97ff8ull},
  };
  for (const auto &m : more)
    EXPECT_EQ(bits(m.v), m.want) << m.what << " = 0x" << std::hex << bits(m.v);
}
