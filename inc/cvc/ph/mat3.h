/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph's small linear algebra: 3x3 matrices (row-major std::array<T, 9>), the SO(2) log, and
// the positive-definite parameterization the port-Hamiltonian models use for M^-1 and D:
//
//   A = L L^T,   L = [[e^l0, 0, 0], [l3, e^l1, 0], [l4, l5, e^l2]]   (six levels, any real values)
//
// with its reverse: for a scalar loss with dA = dLoss/dA (any 3x3, not necessarily symmetric),
// dL = (dA + dA^T) L, and the levels get dL's lower triangle, the diagonal times e^l.
//
// Templated on the scalar (float or double). solve3 is Gaussian elimination with partial pivoting.

#ifndef __CVC_PH_MAT3_H__
#define __CVC_PH_MAT3_H__

#include <array>
#include <cmath>
#include <utility>

namespace cvc {
namespace ph {

template <class T> using vec3 = std::array<T, 3>;
template <class T> using mat3 = std::array<T, 9>;

template <class T> mat3<T> identity3() {
  return {T(1), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(1)};
}

template <class T> mat3<T> mul(const mat3<T> &a, const mat3<T> &b) {
  mat3<T> c{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      c[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return c;
}

template <class T> mat3<T> transpose(const mat3<T> &a) {
  return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]};
}

// A x
template <class T> vec3<T> matvec(const mat3<T> &a, const vec3<T> &x) {
  return {a[0] * x[0] + a[1] * x[1] + a[2] * x[2], a[3] * x[0] + a[4] * x[1] + a[5] * x[2],
          a[6] * x[0] + a[7] * x[1] + a[8] * x[2]};
}

// A^T x
template <class T> vec3<T> tmatvec(const mat3<T> &a, const vec3<T> &x) {
  return {a[0] * x[0] + a[3] * x[1] + a[6] * x[2], a[1] * x[0] + a[4] * x[1] + a[7] * x[2],
          a[2] * x[0] + a[5] * x[1] + a[8] * x[2]};
}

// acc += s x y^T
template <class T> void add_outer(mat3<T> &acc, const vec3<T> &x, const vec3<T> &y, T s = T(1)) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      acc[3 * i + j] += s * x[i] * y[j];
}

// x = A^-1 b by Gaussian elimination with partial pivoting.
template <class T> vec3<T> solve3(mat3<T> a, vec3<T> b) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int r = col + 1; r < 3; ++r)
      if (std::abs(a[3 * r + col]) > std::abs(a[3 * pivot + col]))
        pivot = r;
    if (pivot != col) {
      for (int j = 0; j < 3; ++j)
        std::swap(a[3 * col + j], a[3 * pivot + j]);
      std::swap(b[col], b[pivot]);
    }
    for (int r = col + 1; r < 3; ++r) {
      const T f = a[3 * r + col] / a[3 * col + col];
      for (int j = col; j < 3; ++j)
        a[3 * r + j] -= f * a[3 * col + j];
      b[r] -= f * b[col];
    }
  }
  vec3<T> x{};
  for (int r = 2; r >= 0; --r) {
    T s = b[r];
    for (int j = r + 1; j < 3; ++j)
      s -= a[3 * r + j] * x[j];
    x[r] = s / a[3 * r + r];
  }
  return x;
}

// x = A^-T b
template <class T> vec3<T> solve3_transposed(const mat3<T> &a, const vec3<T> &b) {
  return solve3(transpose(a), b);
}

// The lower-triangular factor of the six levels.
template <class T> mat3<T> spd_factor(const T *l) {
  using std::exp;
  return {exp(l[0]), T(0), T(0), l[3], exp(l[1]), T(0), l[4], l[5], exp(l[2])};
}

// A = L L^T.
template <class T> mat3<T> spd_from_levels(const T *l) {
  const mat3<T> f = spd_factor(l);
  return mul(f, transpose(f));
}

// dl += the levels' gradient for dA = dLoss/dA.
template <class T> void spd_from_levels_vjp(const T *l, const mat3<T> &dA, T *dl) {
  const mat3<T> f = spd_factor(l);
  mat3<T> sym{};
  for (int k = 0; k < 9; ++k)
    sym[k] = dA[k];
  const mat3<T> t = transpose(dA);
  for (int k = 0; k < 9; ++k)
    sym[k] += t[k];
  const mat3<T> dL = mul(sym, f);
  dl[0] += dL[0] * f[0];
  dl[1] += dL[4] * f[4];
  dl[2] += dL[8] * f[8];
  dl[3] += dL[3];
  dl[4] += dL[6];
  dl[5] += dL[7];
}

// The six levels of a symmetric positive-definite A (its Cholesky factor): the inverse of
// spd_from_levels.
template <class T> std::array<T, 6> spd_levels_of(const mat3<T> &a) {
  using std::log;
  using std::sqrt;
  const T l00 = sqrt(a[0]);
  const T l10 = a[3] / l00, l20 = a[6] / l00;
  const T l11 = sqrt(a[4] - l10 * l10);
  const T l21 = (a[7] - l20 * l10) / l11;
  const T l22 = sqrt(a[8] - l20 * l20 - l21 * l21);
  return {log(l00), log(l11), log(l22), l10, l20, l21};
}

// Log on SO(2): the angle in (-pi, pi]. Its derivative is 1 wherever it is used (away from the
// cut), so a reverse pass passes gradients straight through it.
template <class T> T wrap_angle(T a) {
  using std::atan2;
  using std::cos;
  using std::sin;
  return atan2(sin(a), cos(a));
}

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_MAT3_H__
