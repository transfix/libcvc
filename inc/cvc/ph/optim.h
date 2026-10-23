/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph's optimizers, written to match optax 0.2.8 step for step (the JAX trainer the
// port-Hamiltonian models were first fit with): a cosine decay, global-norm clipping, Adam, and
// Prodigy (Mishchenko and Defazio, "Prodigy: an expeditiously adaptive parameter-free learner",
// 2023; optax.contrib.prodigy). ph_optim_test holds them to optax's own numbers.
//
// Each works on a flat span of doubles. `count` is the number of updates already applied, as
// optax's schedules read it (0 on the first update).

#ifndef __CVC_PH_OPTIM_H__
#define __CVC_PH_OPTIM_H__

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cvc {
namespace ph {

// optax.cosine_decay_schedule(init, decay_steps, alpha) at `count`.
inline double cosine_decay(double init, int decay_steps, double alpha, int count) {
  const double t = static_cast<double>(std::min(count, decay_steps));
  const double cosine = 0.5 * (1.0 + std::cos(3.14159265358979323846 * t / decay_steps));
  return init * ((1.0 - alpha) * cosine + alpha);
}

// optax.clip_by_global_norm: scales g to norm `max_norm` when its norm is not below it. Returns
// the norm before clipping.
inline double clip_by_global_norm(double *g, std::size_t n, double max_norm) {
  double sq = 0.0;
  for (std::size_t i = 0; i < n; ++i)
    sq += g[i] * g[i];
  const double norm = std::sqrt(sq);
  if (!(norm < max_norm))
    for (std::size_t i = 0; i < n; ++i)
      g[i] = g[i] / norm * max_norm;
  return norm;
}

// optax.adam (eps outside the root, eps_root 0).
struct adam {
  double beta1 = 0.9, beta2 = 0.999, eps = 1e-8;
  std::vector<double> m, v;
  int count = 0;

  void init(std::size_t n) {
    m.assign(n, 0.0);
    v.assign(n, 0.0);
    count = 0;
  }
  // p -= lr * m_hat / (sqrt(v_hat) + eps); lr is the schedule at the count before this update.
  void update(double *p, const double *g, std::size_t n, double lr) {
    const int t = count + 1;
    const double c1 = 1.0 - std::pow(beta1, t), c2 = 1.0 - std::pow(beta2, t);
    for (std::size_t i = 0; i < n; ++i) {
      m[i] = beta1 * m[i] + (1.0 - beta1) * g[i];
      v[i] = beta2 * v[i] + (1.0 - beta2) * g[i] * g[i];
      p[i] -= lr * (m[i] / c1) / (std::sqrt(v[i] / c2) + eps);
    }
    count = t;
  }
};

// optax.contrib.prodigy, weight decay 0: Adam whose step size d (the distance-to-solution
// estimate) is learned. One instance per parameter group, as the JAX trainer has one per level.
//
// One deliberate difference: when a group's gradients have been exactly zero since the start,
// Prodigy's denominator is 0 and optax's d becomes NaN; here d stays where it is.
struct prodigy {
  double beta1 = 0.9, beta2 = 0.999, eps = 1e-8;
  double d0 = 1e-6;  // estim_lr0, the initial (under-)estimate of d
  double coef = 1.0; // estim_lr_coef
  bool safeguard_warmup = false;
  double d = 0.0; // estim_lr
  double numerator_weighted = 0.0;
  std::vector<double> exp_avg, exp_avg_sq, grad_sum, p0;
  int count = 0;

  void init(const double *p, std::size_t n) {
    exp_avg.assign(n, 0.0);
    exp_avg_sq.assign(n, 0.0);
    grad_sum.assign(n, 0.0);
    p0.assign(p, p + n);
    d = d0;
    numerator_weighted = 0.0;
    count = 0;
  }
  // `sched` is the learning-rate schedule at the count before this update (1 is Prodigy's base).
  void update(double *p, const double *g, std::size_t n, double sched) {
    const int t = count + 1;
    const double beta3 = std::sqrt(beta2);
    const double bc = std::sqrt(1.0 - std::pow(beta2, t)) / (1.0 - std::pow(beta1, t));
    const double dlr = d * sched * bc;
    double numerator_acum = 0.0;
    for (std::size_t i = 0; i < n; ++i)
      numerator_acum += g[i] * (p0[i] - p[i]);
    double denominator = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double dg = d * g[i];
      exp_avg[i] = beta1 * exp_avg[i] + (1.0 - beta1) * dg;
      exp_avg_sq[i] = beta2 * exp_avg_sq[i] + (1.0 - beta2) * dg * dg;
      grad_sum[i] = beta3 * grad_sum[i] + (safeguard_warmup ? d : dlr) * dg / d0;
      denominator += std::fabs(grad_sum[i]);
    }
    numerator_weighted = beta3 * numerator_weighted + (d / d0) * dlr * numerator_acum;
    if (denominator > 0.0)
      d = std::max(d, coef * numerator_weighted / denominator);
    for (std::size_t i = 0; i < n; ++i)
      p[i] -= dlr * exp_avg[i] / (std::sqrt(exp_avg_sq[i]) + d * eps);
    count = t;
  }
};

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_OPTIM_H__
