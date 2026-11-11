/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See cvc/algo/det_math.h. Built with -ffp-contract=off (/fp:precise on MSVC): a fused
// multiply-add rounds once where these expressions round twice, and would give other bits on a
// target with FMA.

#include <cmath>
#include <cvc/algo/det_math.h>
#include <limits>
#include <utility>

namespace cvc {
namespace algo {
namespace det {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Cody-Waite splits (fdlibm's): the high parts end in zero bits, so k * hi is exact for the
// multiples k the reductions below take.
constexpr double kLn2Hi = 6.93147180369123816490e-01; // low 32 bits zero
constexpr double kLn2Lo = 1.90821492927058770002e-10; // ln 2 - kLn2Hi
constexpr double kInvLn2 = 1.44269504088896338700e+00;
constexpr double kSqrtHalf = 7.07106781186547524401e-01;
// e^x overflows above ln(DBL_MAX), and is below half the least subnormal under this.
constexpr double kExpMax = 7.09782712893383973096e+02;
constexpr double kExpMin = -7.45133219101941108420e+02;
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

double exp(double x) {
  if (x != x)
    return x;
  if (x > kExpMax)
    return kInf;
  if (x < kExpMin)
    return 0.0;
  // x = k ln2 + r, |r| <= ln2/2; e^x = 2^k e^r, and e^r = 1 + (e^r - 1) rounds once. ldexp is
  // exact, or rounds once into the subnormals.
  const double k = std::floor(x * kInvLn2 + 0.5);
  if (k == 0.0)
    return 1.0 + expm1_reduced(x);
  const double r = (x - k * kLn2Hi) - k * kLn2Lo;
  return std::ldexp(1.0 + expm1_reduced(r), static_cast<int>(k));
}

namespace {

// log x = t1 + t2 for a positive finite x: t1 = k ln2's high part, exact, and t2 the rest
// (|t2| < 0.35 + |k| 2e-10), rounded once more than its own last bit.
void log_parts(double x, double &t1, double &t2) {
  // x = m 2^k with m in [sqrt(1/2), sqrt(2)), and f = m - 1, exact. With s = f / (2 + f),
  // log(1 + f) = 2 atanh(s) = 2s + s R, R = 2z/3 + 2z^2/5 + ... (z = s^2, under 0.0295), which
  // fdlibm rearranges as f - (f^2/2 - s (f^2/2 + R)) so the large part f is added last.
  // R to 2z^12/25: the next term is below 1e-19 of the result.
  int e = 0;
  double m = std::frexp(x, &e); // [0.5, 1), subnormals included
  if (m < kSqrtHalf) {
    m *= 2.0;
    --e;
  }
  const double f = m - 1.0;
  const double k = e;
  const double s = f / (2.0 + f);
  const double z = s * s;
  double R = 0.0;
  for (int n = 12; n >= 1; --n)
    R = z * (2.0 / (2 * n + 1) + R);
  const double hfsq = 0.5 * f * f;
  t1 = k * kLn2Hi;
  t2 = f - (hfsq - (s * (hfsq + R) + k * kLn2Lo));
}

// a = hi + lo exactly, each half of a's significand (Veltkamp); |a| below 2^996.
void split(double a, double &hi, double &lo) {
  const double c = 134217729.0 * a; // 2^27 + 1
  hi = c - (c - a);
  lo = a - hi;
}

// a b = p + e exactly (Dekker), without a fused multiply-add.
void two_prod(double a, double b, double &p, double &e) {
  p = a * b;
  double ah, al, bh, bl;
  split(a, ah, al);
  split(b, bh, bl);
  e = ((ah * bh - p) + ah * bl + al * bh) + al * bl;
}

} // namespace

double log(double x) {
  if (x != x)
    return x;
  if (x < 0.0)
    return (x - x) / (x - x); // NaN, -inf included
  if (x == 0.0)
    return -kInf;
  if (x == kInf)
    return x;
  double t1, t2;
  log_parts(x, t1, t2);
  return t1 + t2;
}

namespace {

// log x = hi + lo to about 1e-19 absolute, for a positive finite x other than 1: what pow
// multiplies by y, where log's own last-bit rounding would grow y-fold.
void log_dd(double x, double &hi, double &lo) {
  int e = 0;
  double m = std::frexp(x, &e);
  if (m < kSqrtHalf) {
    m *= 2.0;
    --e;
  }
  const double f = m - 1.0; // exact
  const double k = e;
  // s = f / (2 + f) = sh + sl. f - sh (2 + f) = (f - 2 sh) - sh f: f - 2 sh is exact (the two are
  // within a factor of two, Sterbenz), sh f is taken exactly, and the difference of the two is
  // exact too, so sl carries only its own division's rounding.
  const double u = 2.0 + f;
  const double sh = f / u;
  double p, pe;
  two_prod(sh, f, p, pe);
  const double sl = (((f - 2.0 * sh) - p) - pe) / u;
  // log(1 + f) = 2 atanh(s) = 2 s + s R, R = 2z/3 + 2z^2/5 + ... as in log_parts; 2 sh is exact
  // and the rest, under 0.004, rounds below 1e-19.
  const double z = sh * sh;
  double R = 0.0;
  for (int n = 12; n >= 1; --n)
    R = z * (2.0 / (2 * n + 1) + R);
  const double a = 2.0 * sh;
  const double c = 2.0 * sl + sh * R;
  // + k ln2: k ln2_hi exact, its sum with a taken exactly (Knuth's two-sum), the low parts after.
  const double t1 = k * kLn2Hi;
  const double s1 = t1 + a;
  const double bb = s1 - t1;
  const double err = (t1 - (s1 - bb)) + (a - bb);
  const double rest = (err + c) + k * kLn2Lo;
  hi = s1 + rest;
  lo = rest - (hi - s1); // exact: |s1| >= |rest|
}

// x^y for a positive finite x other than 1 and a finite y: e^(y log x), with log x in two parts
// multiplied by y exactly, so the result keeps its digits however large y log x is.
double pow_positive(double x, double y) {
  double hi, lo;
  log_dd(x, hi, lo);
  const double p = y * hi;
  if (std::fabs(p) > 750.0)
    return exp(p); // overflows to inf or underflows to 0 whatever the low parts
  // |y| < 750 / |log x| < 7e18 here (x is not 1), so splitting y does not overflow.
  double ph, pl;
  two_prod(y, hi, ph, pl);
  const double d = pl + y * lo; // e^(ph + d) = e^ph (1 + d), d below 1e-13
  const double e = exp(ph);
  return e + e * d;
}

} // namespace

double pow(double x, double y) {
  // C99 Annex F's special cases, in its order.
  if (y == 0.0 || x == 1.0)
    return 1.0; // even for a NaN
  if (x != x || y != y)
    return x + y;
  const bool integer = std::isfinite(y) && std::floor(y) == y;
  const bool odd = integer && std::fabs(y) < 9007199254740992.0 && std::fmod(y, 2.0) != 0.0;
  if (std::isinf(y)) {
    const double a = std::fabs(x);
    if (a == 1.0)
      return 1.0;
    return (a < 1.0) == (y < 0.0) ? kInf : 0.0;
  }
  if (x == 0.0 || std::isinf(x)) {
    // pow(+-0, y) and pow(+-inf, y): 0 or inf by the sign of y (inverted for 0), the sign of x
    // kept for an odd integer y.
    const bool big = (x == 0.0) == (y < 0.0);
    const double r = big ? kInf : 0.0;
    return odd ? std::copysign(r, x) : r;
  }
  if (x < 0.0 && !integer)
    return (x - x) / (x - x); // NaN
  const double a = std::fabs(x);
  double r;
  if (y == 1.0)
    r = a;
  else if (y == 2.0)
    r = a * a; // each of these rounds once, correctly
  else if (y == -1.0)
    r = 1.0 / a;
  else if (y == 0.5)
    r = std::sqrt(a);
  else
    r = pow_positive(a, y); // squaring for an integer y would double its error each time
  return x < 0.0 && odd ? -r : r;
}

double sinh(double x) {
  if (x != x)
    return x;
  const double a = std::fabs(x);
  if (a < 22.0) {
    // With t = e^a - 1 taken whole, e^a - e^-a = t + t / (t + 1) keeps a small a's digits.
    const double t = expm1(a);
    return std::copysign(0.5 * (t + t / (t + 1.0)), x);
  }
  // e^-a is below an ULP of e^a; past 709 e^a itself overflows before half of it would.
  if (a < 709.0)
    return std::copysign(0.5 * exp(a), x);
  const double h = exp(0.5 * a);
  return std::copysign((0.5 * h) * h, x);
}

double cosh(double x) {
  if (x != x)
    return x;
  const double a = std::fabs(x);
  if (a < 0.5 * kLn2Hi) {
    // 1 + (e^a - 1)^2 / (2 e^a): the part above 1 keeps its digits.
    const double t = expm1(a);
    return 1.0 + (t * t) / (2.0 * (1.0 + t));
  }
  if (a < 22.0) {
    const double e = exp(a);
    return 0.5 * e + 0.5 / e;
  }
  if (a < 709.0)
    return 0.5 * exp(a);
  const double h = exp(0.5 * a);
  return (0.5 * h) * h;
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

double acos(double x) {
  if (!(std::fabs(x) <= 1.0))
    return (x - x) / (x - x); // NaN, for NaN and |x| > 1
  return atan2(std::sqrt((1.0 - x) * (1.0 + x)), x);
}

double hypot(double x, double y) {
  if (std::isinf(x) || std::isinf(y))
    return kInf; // even with a NaN, as libm
  if (x != x || y != y)
    return x + y;
  double a = std::fabs(x), b = std::fabs(y);
  if (a < b)
    std::swap(a, b);
  if (b == 0.0)
    return a;
  // Scale by a's power of two, which is exact (b may round into the subnormals, where its
  // square is far below an ULP of a's), so the squares neither overflow nor underflow.
  int e = 0;
  std::frexp(a, &e);
  const double as = std::ldexp(a, -e), bs = std::ldexp(b, -e);
  return std::ldexp(std::sqrt(as * as + bs * bs), e);
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
} // namespace algo
} // namespace cvc
