/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Forward-mode dual numbers for CoPH's learner: C3 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.3).
// They replace torch autograd in two places:
//
//   coph_dual<double, 2>                       the replay, one direction per policy gain, so one
//                                              pass gives the replay cost and d cost / d policy.
//   coph_dual<coph_dual<double, 1>, 1>         the damping Newton step: a hyper-dual, so one
//   (coph_hyper)                               evaluation of the plant loss gives its first and
//                                              second derivatives in the damping.
//
// A dual carries a value v and N derivatives d. Arithmetic follows the product and quotient rules;
// comparisons look at the value only (coph_value), as torch's control flow does. The elementary
// functions (exp, expm1, log1p, sqrt) live in coph_model.cpp's scalar traits, next to the double
// versions they extend, so the plant and the potential compute the same values on duals as on
// doubles.

#ifndef CVC_NAV_COPH_DUAL_H
#define CVC_NAV_COPH_DUAL_H

#include <array>
#include <type_traits>

namespace cvc {
namespace nav {

template <class T, int N> struct coph_dual {
  T v{};
  std::array<T, N> d{};

  coph_dual() = default;
  coph_dual(double x) : v(x) {} // NOLINT: constants enter expressions implicitly
  template <class U = T, class = std::enable_if_t<!std::is_same_v<U, double>>>
  coph_dual(const T &x) : v(x) {} // NOLINT

  // x as the i-th independent variable.
  static coph_dual variable(const T &x, int i) {
    coph_dual r(x);
    r.d[i] = T(1.0);
    return r;
  }

  coph_dual &operator+=(const coph_dual &o) { return *this = *this + o; }
  coph_dual &operator-=(const coph_dual &o) { return *this = *this - o; }
  coph_dual &operator*=(const coph_dual &o) { return *this = *this * o; }
  coph_dual &operator/=(const coph_dual &o) { return *this = *this / o; }
};

using coph_dual2 = coph_dual<double, 2>;
using coph_hyper = coph_dual<coph_dual<double, 1>, 1>;

template <class T, int N> coph_dual<T, N> operator-(const coph_dual<T, N> &a) {
  coph_dual<T, N> r;
  r.v = -a.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = -a.d[i];
  return r;
}

template <class T, int N>
coph_dual<T, N> operator+(const coph_dual<T, N> &a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r;
  r.v = a.v + b.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = a.d[i] + b.d[i];
  return r;
}

template <class T, int N>
coph_dual<T, N> operator-(const coph_dual<T, N> &a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r;
  r.v = a.v - b.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = a.d[i] - b.d[i];
  return r;
}

template <class T, int N>
coph_dual<T, N> operator*(const coph_dual<T, N> &a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r;
  r.v = a.v * b.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = a.d[i] * b.v + a.v * b.d[i];
  return r;
}

template <class T, int N>
coph_dual<T, N> operator/(const coph_dual<T, N> &a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r;
  r.v = a.v / b.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = (a.d[i] - r.v * b.d[i]) / b.v;
  return r;
}

// Mixed with plain doubles: the double is a constant.
template <class T, int N> coph_dual<T, N> operator+(const coph_dual<T, N> &a, double b) {
  coph_dual<T, N> r = a;
  r.v = a.v + b;
  return r;
}
template <class T, int N> coph_dual<T, N> operator+(double a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r = b;
  r.v = a + b.v;
  return r;
}
template <class T, int N> coph_dual<T, N> operator-(const coph_dual<T, N> &a, double b) {
  coph_dual<T, N> r = a;
  r.v = a.v - b;
  return r;
}
template <class T, int N> coph_dual<T, N> operator-(double a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r = -b;
  r.v = a - b.v;
  return r;
}
template <class T, int N> coph_dual<T, N> operator*(const coph_dual<T, N> &a, double b) {
  coph_dual<T, N> r;
  r.v = a.v * b;
  for (int i = 0; i < N; ++i)
    r.d[i] = a.d[i] * b;
  return r;
}
template <class T, int N> coph_dual<T, N> operator*(double a, const coph_dual<T, N> &b) {
  coph_dual<T, N> r;
  r.v = a * b.v;
  for (int i = 0; i < N; ++i)
    r.d[i] = a * b.d[i];
  return r;
}
template <class T, int N> coph_dual<T, N> operator/(const coph_dual<T, N> &a, double b) {
  coph_dual<T, N> r;
  r.v = a.v / b;
  for (int i = 0; i < N; ++i)
    r.d[i] = a.d[i] / b;
  return r;
}
template <class T, int N> coph_dual<T, N> operator/(double a, const coph_dual<T, N> &b) {
  return coph_dual<T, N>(a) / b;
}

// The value, through any nesting.
inline double coph_value(double x) { return x; }
template <class T, int N> double coph_value(const coph_dual<T, N> &x) { return coph_value(x.v); }

} // namespace nav
} // namespace cvc

#endif // CVC_NAV_COPH_DUAL_H
