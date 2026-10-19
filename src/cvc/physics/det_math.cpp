/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See det_math.h. Built with -ffp-contract=off (/fp:precise on MSVC): a fused multiply-add
// rounds once where these expressions round twice, and would give other bits on a target
// with FMA.

#include "det_math.h"

#include <cmath>
#include <limits>

namespace cvc {
namespace physics {
namespace det {

namespace {

// Cody-Waite splits (fdlibm's): the high parts end in zero bits, so k * hi is exact for the
// multiples k the reductions below take.
constexpr double kLn2Hi = 6.93147180369123816490e-01; // low 32 bits zero
constexpr double kLn2Lo = 1.90821492927058770002e-10; // ln 2 - kLn2Hi
constexpr double kInvLn2 = 1.44269504088896338700e+00;
constexpr double kPio2Hi = 1.57079632673412561417e+00; // pi/2's first 33 bits
constexpr double kPio2Lo = 6.07710050650619224932e-11; // pi/2 - kPio2Hi
constexpr double kInvPio2 = 6.36619772367581382433e-01;
constexpr double kPiHi = 2.0 * kPio2Hi; // exact
constexpr double kPiLo = 2.0 * kPio2Lo;
constexpr double kPio4 = 7.85398163397448309616e-01;

// e^r - 1 for |r| <= ln2/2: the Taylor series to r^17/17!, by Horner,
// r (1 + r/2 (1 + r/3 (... (1 + r/17)))). The next term is below 1e-24 r.
double expm1_reduced(double r) {
  double p = 1.0;
  for (int n = 17; n >= 2; --n)
    p = 1.0 + r * p / n;
  return r * p;
}

// sin r and cos r for |r| <= pi/4: their series to r^19/19! and r^20/20!.
double sin_reduced(double r) {
  const double r2 = r * r;
  double p = 1.0;
  for (int n = 18; n >= 2; n -= 2)
    p = 1.0 - r2 * p / (n * (n + 1));
  return r * p;
}

double cos_reduced(double r) {
  const double r2 = r * r;
  double p = 1.0;
  for (int n = 19; n >= 1; n -= 2)
    p = 1.0 - r2 * p / (n * (n + 1));
  return p;
}

// x = k pi/2 + r, |r| <= pi/4, and the quadrant k mod 4 in [0, 4).
double reduce(double x, int &quadrant) {
  const double k = std::floor(x * kInvPio2 + 0.5);
  double q = std::fmod(k, 4.0);
  if (q < 0.0)
    q += 4.0;
  quadrant = static_cast<int>(q);
  return (x - k * kPio2Hi) - k * kPio2Lo;
}

} // namespace

double expm1(double x) {
  if (x != x)
    return x;
  if (x > 709.0)
    return std::numeric_limits<double>::infinity();
  if (x < -40.0)
    return -1.0; // e^-40 is below half an ULP of 1
  // x = k ln2 + r, |r| <= ln2/2; e^x - 1 = 2^k (e^r - 1) + (2^k - 1).
  const double k = std::floor(x * kInvLn2 + 0.5);
  if (k == 0.0)
    return expm1_reduced(x);
  const double r = (x - k * kLn2Hi) - k * kLn2Lo;
  const int ki = static_cast<int>(k);
  return std::ldexp(expm1_reduced(r), ki) + (std::ldexp(1.0, ki) - 1.0);
}

double tanh(double x) {
  if (x != x)
    return x;
  const double a = std::fabs(x);
  if (a > 22.0)
    return std::copysign(1.0, x); // 1 - tanh(22) is below half an ULP of 1
  // tanh a = (e^2a - 1) / (e^2a + 1), with e^2a - 1 taken whole so a small a keeps its digits.
  const double t = expm1(2.0 * a);
  return std::copysign(t / (t + 2.0), x);
}

double sin(double x) {
  if (!std::isfinite(x))
    return x - x; // NaN
  int q = 0;
  const double r = reduce(x, q);
  switch (q) {
  case 0:
    return sin_reduced(r);
  case 1:
    return cos_reduced(r);
  case 2:
    return -sin_reduced(r);
  default:
    return -cos_reduced(r);
  }
}

double cos(double x) {
  if (!std::isfinite(x))
    return x - x;
  int q = 0;
  const double r = reduce(x, q);
  switch (q) {
  case 0:
    return cos_reduced(r);
  case 1:
    return -sin_reduced(r);
  case 2:
    return -cos_reduced(r);
  default:
    return sin_reduced(r);
  }
}

double tan(double x) {
  if (!std::isfinite(x))
    return x - x;
  // tan x = sin r / cos r in quadrants 0 and 2, -cos r / sin r in 1 and 3.
  int q = 0;
  const double r = reduce(x, q);
  const double s = sin_reduced(r), c = cos_reduced(r);
  return q % 2 == 0 ? s / c : -c / s;
}

double atan(double x) {
  if (x != x)
    return x;
  double a = std::fabs(x);
  const bool inverted = a > 1.0;
  if (inverted)
    a = 1.0 / a; // atan a = pi/2 - atan(1/a)
  // Halve the angle, atan a = 2 atan(a / (1 + sqrt(1 + a^2))), until |a| <= 0.2 (at most twice:
  // tan(pi/16) = 0.199 is what two halvings leave of 1); then the series
  // a (1 - a^2/3 + a^4/5 - ...) to a^25/25 (the next term is below 1e-19 a).
  double scale = 1.0;
  while (a > 0.2) {
    a = a / (1.0 + std::sqrt(1.0 + a * a));
    scale *= 2.0;
  }
  const double a2 = a * a;
  double s = 1.0 / 25.0;
  for (int n = 23; n >= 1; n -= 2)
    s = 1.0 / n - a2 * s;
  double r = scale * (a * s);
  if (inverted)
    r = (kPio2Hi - r) + kPio2Lo;
  return std::copysign(r, x);
}

double atan2(double y, double x) {
  if (x != x || y != y)
    return x + y;
  if (std::isinf(x) || std::isinf(y)) {
    if (std::isinf(x) && std::isinf(y))
      return std::copysign(x > 0.0 ? kPio4 : 3.0 * kPio4, y);
    if (std::isinf(y))
      return std::copysign(kPio2Hi + kPio2Lo, y);
    return x > 0.0 ? std::copysign(0.0, y) : std::copysign(kPiHi + kPiLo, y);
  }
  if (x == 0.0) {
    if (y != 0.0)
      return std::copysign(kPio2Hi + kPio2Lo, y);
    return std::signbit(x) ? std::copysign(kPiHi + kPiLo, y) : y; // atan2(+-0, -0) = +-pi
  }
  const double r = atan(y / x);
  if (x > 0.0)
    return r;
  // The left half plane: pi added toward y's side, its low part last.
  return std::signbit(y) ? (r - kPiHi) - kPiLo : (r + kPiHi) + kPiLo;
}

double asin(double x) {
  if (!(std::fabs(x) <= 1.0))
    return (x - x) / (x - x); // NaN, for NaN and |x| > 1
  return atan2(x, std::sqrt((1.0 - x) * (1.0 + x)));
}

double cbrt(double x) {
  if (x == 0.0 || !std::isfinite(x))
    return x;
  // |x| = m 2^e, m in [0.5, 1); with e = 3q + rem, cbrt |x| = cbrt(m 2^rem) 2^q.
  int e = 0;
  const double m = std::frexp(std::fabs(x), &e);
  int q = e / 3, rem = e % 3;
  if (rem < 0) {
    rem += 3;
    q -= 1;
  }
  const double v = std::ldexp(m, rem); // [0.5, 4)
  // Newton from the chord through cbrt(0.5) and cbrt(4), within 10%: the error squares each
  // step, so six steps leave only the last rounding. Each step adds a correction to y rather
  // than recomputing it, so that rounding is the correction's, a small part of y.
  double y = 0.6803 + 0.2268 * v;
  for (int i = 0; i < 6; ++i)
    y -= (y - v / (y * y)) / 3.0;
  return std::copysign(std::ldexp(y, q), x);
}

double pow_two_thirds(double x) {
  // cbrt(x^2) carries a third of x^2's rounding; cbrt(x)^2 would double cbrt's. x^2 is used
  // where it neither overflows nor goes subnormal.
  const double a = std::fabs(x);
  if (a > 1e-150 && a < 1e150)
    return cbrt(x * x);
  const double c = cbrt(x);
  return c * c;
}

} // namespace det
} // namespace physics
} // namespace cvc
