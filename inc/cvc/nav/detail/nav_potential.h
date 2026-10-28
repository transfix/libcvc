/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// detail/nav_potential.h: U_nav, the potential the navigator's conservative forces descend,
// evaluated as a number. The force laws (diff_rollout.h, material_rollout.h, drive.cpp) are
// written as closed-form derivatives; these are the potentials whose derivatives they are:
//
//   U_nav = beta/2 |o - g|^2 + sum_j alpha_j b_IPC(d_j) + lam_soft r~(o) + lam_hard b_sp(phi(o))
//           (+ the terrain's grade cost and slope barrier, the same two forms on its own plane)
//
// with the coefficients held fixed. Damping (-gamma v) is dissipation and the external force
// is a port: neither is part of U. Each barrier potential here is the one whose derivative the
// matching force primitive returns, its clamps included, so F = -grad U holds wherever the
// sampled planes are consistent (see nav_potential_test.cpp, the finite-difference gradcheck
// at every call site).
//
// Host and device (CVC_NP_HD), float, side-effect free. Nothing here is on a force path: the
// rollouts evaluate U only when a caller asks for it, so their outputs are unchanged. The *_t
// templates are the same formulas in any precision: the float functions are their float
// instantiations, op for op, and the per-step energy books (rollout_energy, material.h) evaluate
// them in double.

#ifndef CVC_NAV_DETAIL_NAV_POTENTIAL_H
#define CVC_NAV_DETAIL_NAV_POTENTIAL_H

#include <math.h>

#if defined(__CUDACC__)
#define CVC_NP_HD __host__ __device__
#else
#define CVC_NP_HD
#endif

namespace cvc {
namespace nav {

// The terms of U_nav at one pose. `terrain` is the terrain's grade cost plus its slope barrier
// (the deployed drive only; zero elsewhere).
struct potential_terms {
  float goal = 0.0f;    // beta/2 |o - g|^2
  float barrier = 0.0f; // sum_j alpha_j b_IPC(d_j), over obstacles or footprint discs
  float soft = 0.0f;    // lam_soft r~(o)
  float hard = 0.0f;    // lam_hard b_sp(phi(o)), in the force's own units (see drive_potential)
  float terrain = 0.0f; // lam_soft_T r~_T(o) + lam_hard_T b_sp(phi_T(o))
  CVC_NP_HD float total() const { return goal + barrier + soft + hard + terrain; }
};

namespace detail {

// The C math functions by precision: logf etc. for float (the functions below always called),
// log etc. for double.
CVC_NP_HD inline float np_log(float x) { return logf(x); }
CVC_NP_HD inline double np_log(double x) { return log(x); }
CVC_NP_HD inline float np_log1p(float x) { return log1pf(x); }
CVC_NP_HD inline double np_log1p(double x) { return log1p(x); }
CVC_NP_HD inline float np_exp(float x) { return expf(x); }
CVC_NP_HD inline double np_exp(double x) { return exp(x); }

// -(d - dh)^2 ln(d / dh), the IPC barrier on (0, dh).
template <class T> CVC_NP_HD inline T ipc_b_core_t(T d, T dh) {
  const T A = d - dh;
  return -(A * A) * np_log(d / dh);
}
CVC_NP_HD inline float ipc_b_core(float d, float dh) { return ipc_b_core_t<float>(d, dh); }

// The potential of diff::ipc_ and drive.cpp's ipc_dbdd: b(d) on [1e-6, dh) and 0 from dh up.
// Below 1e-6 those hold b'(1e-6), so b continues linearly with that slope.
CVC_NP_HD inline float ipc_b(float d, float dh) {
  const float lo = 1e-6f;
  const float dc = d < lo ? lo : d;
  if (!(dc < dh))
    return 0.0f;
  if (d >= lo)
    return ipc_b_core(d, dh);
  const float A = lo - dh;
  const float slope = -(2.0f * A * logf(lo / dh) + A * A / lo);
  return ipc_b_core(lo, dh) + slope * (d - lo);
}

// Where ipc_dbdd_pw's clamp starts: b'(d) = -max_grad. On (0, dh) b' rises monotonically from
// -inf to 0 (b'' > 0 there), so bisection finds the one crossing. Double, to a few ulps.
CVC_NP_HD inline double ipc_pw_knee(double dh, double max_grad = 200.0) {
  double lo = 0.0, hi = dh;
  for (int k = 0; k < 100; ++k) {
    const double mid = 0.5 * (lo + hi);
    const double A = mid - dh;
    const double bp = -(2.0 * A * log(mid / dh) + A * A / mid);
    if (bp < -max_grad)
      lo = mid;
    else
      hi = mid;
  }
  return hi;
}

// The potential of material_rollout.h's ipc_dbdd_pw: b(d) where its derivative is within the
// [-200, 200] clamp, continued linearly with slope -200 below the knee (and through d <= 1e-9,
// whose -500 the clamp also makes -200). Zero from dh up.
template <class T> CVC_NP_HD inline T ipc_b_pw_t(T d, T dh) {
  const T eps = T(1e-9f), max_grad = T(200.0f);
  if (!(d < dh))
    return T(0);
  if (d > eps) {
    const T bp = -(T(2) * (d - dh) * np_log(d / dh) + (d - dh) * (d - dh) / d);
    if (!(bp < -max_grad))
      return ipc_b_core_t<T>(d, dh);
  }
  const T knee = static_cast<T>(ipc_pw_knee(static_cast<double>(dh)));
  return ipc_b_core_t<T>(knee, dh) + max_grad * (knee - d);
}
CVC_NP_HD inline float ipc_b_pw(float d, float dh) { return ipc_b_pw_t<float>(d, dh); }

// The potential of sdf_barrier_db = -sigmoid(k (d_hat - phi)): b_sp(phi) = softplus(k (d_hat -
// phi)) / k, in phi's units.
template <class T> CVC_NP_HD inline T sdf_barrier_b_t(T phi, T k_sharp, T d_hat) {
  const T x = k_sharp * (d_hat - phi);
  const T sp = x > T(0) ? x + np_log1p(np_exp(-x)) : np_log1p(np_exp(x));
  return sp / k_sharp;
}
CVC_NP_HD inline float sdf_barrier_b(float phi, float k_sharp, float d_hat) {
  return sdf_barrier_b_t<float>(phi, k_sharp, d_hat);
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // CVC_NAV_DETAIL_NAV_POTENTIAL_H
