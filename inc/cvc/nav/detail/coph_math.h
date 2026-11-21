/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// coph_math.h: the small kernels the CoPH learner (coph_learner.cpp, C3) and its packets
// (coph_exchange.cpp, C4) share: the RBF features, a Cholesky factor and its solves, and the
// distance to a slab. Inline, so each translation unit compiles them with its own flags; every
// coph TU builds with -ffp-contract=off, so they are the Python's arithmetic, operation for
// operation.

#ifndef CVC_NAV_DETAIL_COPH_MATH_H
#define CVC_NAV_DETAIL_COPH_MATH_H

#include <algorithm>
#include <cmath>
#include <cvc/algo/det_math.h>
#include <cvc/nav/coph_ledger.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {
namespace detail {

// model.features: phi_m(q) = exp(-0.5 |(q - c_m) / width|^2) over the centres.
inline std::vector<double> coph_features(const coph_vec2 &q, const std::vector<coph_vec2> &centers,
                                         double width) {
  std::vector<double> phi;
  phi.reserve(centers.size());
  for (const coph_vec2 &c : centers) {
    const double dx = (q[0] - c[0]) / width, dy = (q[1] - c[1]) / width;
    phi.push_back(algo::det::exp(-0.5 * (dx * dx + dy * dy)));
  }
  return phi;
}

// Cholesky of a symmetric positive definite m x m matrix (row-major, the lower triangle read),
// in place: the lower factor. Throws std::runtime_error(what) when a pivot is not positive.
inline void coph_cholesky(std::vector<double> &a, int m,
                          const char *what = "coph: the matrix is not positive definite") {
  for (int j = 0; j < m; ++j) {
    double d = a[j * m + j];
    for (int k = 0; k < j; ++k)
      d -= a[j * m + k] * a[j * m + k];
    if (!(d > 0))
      throw std::runtime_error(what);
    d = std::sqrt(d);
    a[j * m + j] = d;
    for (int i = j + 1; i < m; ++i) {
      double s = a[i * m + j];
      for (int k = 0; k < j; ++k)
        s -= a[i * m + k] * a[j * m + k];
      a[i * m + j] = s / d;
    }
  }
}

// L^-1 b, with l a lower factor (row-major m x m).
inline std::vector<double> coph_forward(const std::vector<double> &l, int m,
                                        std::vector<double> b) {
  for (int i = 0; i < m; ++i) {
    for (int k = 0; k < i; ++k)
      b[i] -= l[i * m + k] * b[k];
    b[i] /= l[i * m + i];
  }
  return b;
}

// (L L^T)^-1 b, with l coph_cholesky's factor.
inline std::vector<double> coph_cholesky_solve(const std::vector<double> &l, int m,
                                               std::vector<double> b) {
  b = coph_forward(l, m, std::move(b));
  for (int i = m - 1; i >= 0; --i) {
    for (int k = i + 1; k < m; ++k)
      b[i] -= l[k * m + i] * b[k];
    b[i] /= l[i * m + i];
  }
  return b;
}

// The inverse of a symmetric positive definite matrix, column by column from its factor.
inline std::vector<double> coph_spd_inverse(std::vector<double> a, int m) {
  coph_cholesky(a, m);
  std::vector<double> inv(static_cast<std::size_t>(m) * m, 0.0);
  for (int j = 0; j < m; ++j) {
    std::vector<double> e(m, 0.0);
    e[j] = 1.0;
    const std::vector<double> col = coph_cholesky_solve(a, m, e);
    for (int i = 0; i < m; ++i)
      inv[i * m + j] = col[i];
  }
  return inv;
}

// shapes.slab_distance: the distance from q to the slab's surface (negative inside).
inline double coph_slab_distance(const coph_vec2 &q, const coph_shape &o) {
  const double px = o.x - o.hx, py = o.y - o.hy;
  const double d0 = (o.x + o.hx) - px, d1 = (o.y + o.hy) - py;
  const double t = std::min(
      std::max(((q[0] - px) * d0 + (q[1] - py) * d1) / ((d0 * d0 + d1 * d1) + 1e-20), 0.0), 1.0);
  const double ex = q[0] - (px + t * d0), ey = q[1] - (py + t * d1);
  return std::sqrt(ex * ex + ey * ey) - o.r;
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // CVC_NAV_DETAIL_COPH_MATH_H
