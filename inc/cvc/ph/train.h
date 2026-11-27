/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph training: fitting a component law's parameters natively, after C-PHAST (Shubham Bhardwaj
// and Chandrajit Bajaj; docs/roadmap/PH-COMPOSE-ROADMAP.md, Phase 4).
//
// The laws are differentiated by the tape in ad.h. canonical_t<T> is canonical_law's arithmetic
// written over a scalar type: with T = double it is canonical_law's own step to rounding (the
// tests hold it to that); with T = ad::var the same operations are recorded and one backward
// sweep gives the loss's gradient in every parameter, through the potential's and the mass's input
// derivatives and the Picard iterations of the implicit midpoint.
//
// The first study is the electrical one, an isolated DGU (the paper's Section 4.1 and Appendix
// A.5), calibrated from fully observed trajectories with
//
//   L_constraint: the instantaneous residuals at the measured state (current balance at the
//                 source, the resistive relation, the capacitor-node voltage),
//   L_step:       the next observation after one LocalStep (two substeps) against the measured,
//   L_joint = L_step + 0.01 L_constraint + 1e-8 |theta|^2,
//
// from a warm start fitted linear to the data, by Adam. The observation is the paper's Eq. 72:
// (q, phi, V_src, V_src - D i, v, -i).

#ifndef __CVC_PH_TRAIN_H__
#define __CVC_PH_TRAIN_H__

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/ph/ad.h>
#include <cvc/ph/compose.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace ph {

// ── the differentiable law ──────────────────────────────────────────────────────────────────────

template <class T> struct dense_t {
  int in = 0, out = 0;
  std::vector<T> W, b; // W: out x in, row-major
  activation act = activation::identity;
};

template <class T> struct mlp_t {
  std::vector<dense_t<T>> layers;
  bool empty() const { return layers.empty(); }

  // y = net(x); jac (out x in, row-major) when non-null, by forward tangents (mlp::eval's order).
  void eval(const T *x, T *y, T *jac = nullptr) const {
    const int in = layers.front().in;
    std::vector<T> a(x, x + in), Tm, z, Tz;
    if (jac) {
      Tm.assign(size_t(in) * in, T(0.0));
      for (int i = 0; i < in; ++i)
        Tm[size_t(i) * in + i] = T(1.0);
    }
    for (const dense_t<T> &L : layers) {
      z.assign(L.out, T(0.0));
      for (int o = 0; o < L.out; ++o) {
        T s = L.b[o];
        for (int i = 0; i < L.in; ++i)
          s += L.W[size_t(o) * L.in + i] * a[i];
        z[o] = s;
      }
      if (jac) {
        Tz.assign(size_t(L.out) * in, T(0.0));
        for (int o = 0; o < L.out; ++o)
          for (int i = 0; i < L.in; ++i)
            for (int c = 0; c < in; ++c)
              Tz[size_t(o) * in + c] += L.W[size_t(o) * L.in + i] * Tm[size_t(i) * in + c];
      }
      a.assign(L.out, T(0.0));
      for (int o = 0; o < L.out; ++o) {
        const T v = z[o];
        T f = v, df = T(1.0);
        switch (L.act) {
        case activation::identity:
          break;
        case activation::silu: {
          const T s = ad::sigmoid(v);
          f = v * s;
          df = s * (T(1.0) + v * (T(1.0) - s));
          break;
        }
        case activation::tanh:
          f = ad::tanh(v);
          df = T(1.0) - f * f;
          break;
        case activation::softplus:
          f = ad::softplus(v);
          df = ad::value(v) > 20 ? T(1.0) : ad::sigmoid(v);
          break;
        case activation::sigmoid:
          f = ad::sigmoid(v);
          df = f * (T(1.0) - f);
          break;
        }
        a[o] = f;
        if (jac)
          for (int c = 0; c < in; ++c)
            Tz[size_t(o) * in + c] *= df;
      }
      if (jac)
        Tm.swap(Tz);
    }
    std::copy(a.begin(), a.end(), y);
    if (jac)
      std::copy(Tm.begin(), Tm.end(), jac);
  }
};

// canonical_law (compose.h) over T: diagonal M and D, constant B and G, no capacitor curve.
template <class T> struct canonical_t {
  int n = 1;
  std::vector<T> K, center;
  double v_scale = 1.0;
  mlp_t<T> v_net;
  std::vector<T> mass;
  mlp_t<T> m_net;
  double m_min = 0.0;
  std::vector<T> damping;
  mlp_t<T> d_net;
  double d_min = 0.0;
  int ports = 0, inputs = 0;
  vecd B, G;
  conservative_scheme scheme = conservative_scheme::implicit_midpoint;

  T potential(const T *q, T *grad) const {
    T V(0.0);
    if (grad)
      std::fill(grad, grad + n, T(0.0));
    if (!K.empty()) {
      std::vector<T> d(n), Kd(n, T(0.0));
      for (int i = 0; i < n; ++i)
        d[i] = q[i] - center[i];
      for (int i = 0; i < n; ++i) {
        T s(0.0);
        for (int j = 0; j < n; ++j)
          s += K[size_t(i) * n + j] * d[j];
        Kd[i] = s;
      }
      T dd(0.0);
      for (int i = 0; i < n; ++i)
        dd += d[i] * Kd[i];
      V += T(0.5) * dd;
      if (grad)
        for (int i = 0; i < n; ++i)
          grad[i] += Kd[i];
    }
    if (!v_net.empty()) {
      T y(0.0);
      std::vector<T> J(n);
      v_net.eval(q, &y, grad ? J.data() : nullptr);
      V += T(v_scale) * y;
      if (grad)
        for (int i = 0; i < n; ++i)
          grad[i] += T(v_scale) * J[i];
    }
    return V;
  }

  void mass_of(const T *q, T *m, T *dm) const {
    if (m_net.empty()) {
      std::copy(mass.begin(), mass.end(), m);
      if (dm)
        std::fill(dm, dm + n * n, T(0.0));
      return;
    }
    std::vector<T> z(n), J(dm ? size_t(n) * n : 0);
    m_net.eval(q, z.data(), dm ? J.data() : nullptr);
    for (int j = 0; j < n; ++j) {
      m[j] = ad::softplus(z[j]) + T(m_min);
      if (dm) {
        const T s = ad::value(z[j]) > 20 ? T(1.0) : ad::sigmoid(z[j]);
        for (int i = 0; i < n; ++i)
          dm[size_t(j) * n + i] = s * J[size_t(j) * n + i];
      }
    }
  }

  void damping_of(const T *q, T *d) const {
    if (!d_net.empty()) {
      d_net.eval(q, d, nullptr);
      for (int j = 0; j < n; ++j)
        d[j] = ad::softplus(d[j]) + T(d_min);
    } else if (!damping.empty()) {
      std::copy(damping.begin(), damping.end(), d);
    } else {
      std::fill(d, d + n, T(0.0));
    }
  }

  bool has_damping() const { return !d_net.empty() || !damping.empty(); }

  void effort(const T *x, T *e) const {
    const T *q = x, *p = x + n;
    std::vector<T> m(n), dm(size_t(n) * n);
    const bool varying = !m_net.empty();
    mass_of(q, m.data(), varying ? dm.data() : nullptr);
    potential(q, e);
    if (varying)
      for (int j = 0; j < n; ++j) {
        const T w = T(-0.5) * p[j] * p[j] / (m[j] * m[j]);
        for (int i = 0; i < n; ++i)
          e[i] += w * dm[size_t(j) * n + i];
      }
    for (int j = 0; j < n; ++j)
      e[n + j] = p[j] / m[j];
  }

  void kick(double tau, const double *u, T *x) const {
    if (inputs == 0)
      return;
    const int N = 2 * n;
    for (int i = 0; i < N; ++i) {
      double g = 0;
      for (int k = 0; k < inputs; ++k)
        g += G[size_t(i) * inputs + k] * u[k];
      x[i] += T(tau * g);
    }
  }

  void damp(double tau, T *x) const {
    if (!has_damping())
      return;
    std::vector<T> m(n), d(n);
    mass_of(x, m.data(), nullptr);
    damping_of(x, d.data());
    const double c = 0.5 * tau;
    for (int j = 0; j < n; ++j) {
      const T a = T(c) * d[j] / m[j];
      x[n + j] *= (T(1.0) - a) / (T(1.0) + a);
    }
  }

  int conserve(double tau, T *x, const solve_opts &o) const {
    const int N = 2 * n;
    T *q = x, *p = x + n;
    std::vector<T> e(N);
    if (scheme == conservative_scheme::leapfrog) {
      std::vector<T> g(n), m(n);
      mass_of(q, m.data(), nullptr);
      potential(q, g.data());
      for (int j = 0; j < n; ++j)
        p[j] -= T(0.5 * tau) * g[j];
      for (int j = 0; j < n; ++j)
        q[j] += T(tau) * p[j] / m[j];
      potential(q, g.data());
      for (int j = 0; j < n; ++j)
        p[j] -= T(0.5 * tau) * g[j];
      return 0;
    }
    std::vector<T> x1(N), x2(N), mid(N);
    effort(x, e.data());
    for (int j = 0; j < n; ++j) {
      x1[j] = q[j] + T(tau) * e[n + j];
      x1[n + j] = p[j] - T(tau) * e[j];
    }
    for (int it = 1; it <= o.max_iter; ++it) {
      for (int i = 0; i < N; ++i)
        mid[i] = T(0.5) * (x[i] + x1[i]);
      effort(mid.data(), e.data());
      double dq = 0, dp = 0;
      for (int j = 0; j < n; ++j) {
        x2[j] = q[j] + T(tau) * e[n + j];
        x2[n + j] = p[j] - T(tau) * e[j];
        dq = std::max(dq, std::fabs(ad::value(x2[j]) - ad::value(x1[j])));
        dp = std::max(dp, std::fabs(ad::value(x2[n + j]) - ad::value(x1[n + j])));
      }
      x1.swap(x2);
      if (dq < o.tol && dp < o.tol) {
        std::copy(x1.begin(), x1.end(), x);
        return it;
      }
    }
    std::copy(x1.begin(), x1.end(), x);
    return -o.max_iter;
  }

  int local_step(double dt, int nsub, const double *u, T *x, const solve_opts &o) const {
    const double d = dt / nsub;
    int worst = 0;
    bool failed = false;
    for (int r = 0; r < nsub; ++r) {
      kick(0.5 * d, u, x);
      damp(0.5 * d, x);
      const int it = conserve(d, x, o);
      failed = failed || it < 0;
      worst = std::max(worst, std::abs(it));
      damp(0.5 * d, x);
      kick(0.5 * d, u, x);
    }
    return failed ? -worst : worst;
  }
};

// The trainable parameters of a canonical spec, flattened in a fixed order: the upper triangle of
// K (mirrored), the centre, then each present net's layers (W, then b), potential, mass and
// damping in turn, then the constant mass and damping entries where there is no net. v_scale and
// the floors (m_min, d_min) stay fixed.
vecd trainable_parameters(const canonical_spec &s);
canonical_spec with_parameters(const canonical_spec &shape, const vecd &theta);

// The spec over double (canonical_t<double>) and over the tape (each trainable parameter a fresh
// variable, in trainable_parameters' order; `vars` receives them).
canonical_t<double> law_over_double(const canonical_spec &s);
canonical_t<ad::var> law_over_tape(const canonical_spec &s, ad::tape &t,
                                   std::vector<ad::var> &vars);

// ── the isolated-DGU calibration ────────────────────────────────────────────────────────────────

// One calibration sample: the measured storage state, the held input, and the six measured channels
// (q, phi, e1, e2, e3, jV) now and one step later.
struct dgu_sample {
  double x[2] = {0, 0};
  double u[2] = {0, 0};
  double y[6] = {0, 0, 0, 0, 0, 0}, y_next[6] = {0, 0, 0, 0, 0, 0};
};

// Trajectories of a reference DGU law (a canonical_law with n = 1, inputs (I_in, V_src)),
// integrated by RK4 at dt / refine and sampled every dt, fully observed. Starts: capacitor voltage
// uniform in [v_lo, v_hi], flux uniform in [phi_lo, phi_hi]; the inputs held.
struct dgu_dataset_options {
  int trajectories = 32, transitions = 200;
  double dt = 0.01;
  int refine = 16;
  double v_lo = -1.5, v_hi = 1.5, phi_lo = -1.0, phi_hi = 1.0;
  double I_in = 0.1, V_src = 1.0;
  std::uint64_t seed = 1;
};
struct dgu_dataset {
  std::vector<dgu_sample> samples;     // every transition of every trajectory
  std::vector<std::vector<vecd>> runs; // each trajectory's states (transitions + 1 of them)
  double dt = 0.01, I_in = 0.1, V_src = 1.0;
};
dgu_dataset make_dgu_dataset(const block_law &reference, const dgu_dataset_options &o);

// The six channels (q, phi, V_src, V_src - D(q) i, v, -i) of a DGU law at x under u (Eq. 72).
template <class T> void dgu_observe(const canonical_t<T> &law, const T *x, const double *u, T *y) {
  T e[2], d[1];
  law.effort(x, e);
  law.damping_of(x, d);
  y[0] = x[0];
  y[1] = x[1];
  y[2] = T(u[1]);
  y[3] = T(u[1]) - d[0] * e[1];
  y[4] = e[0];
  y[5] = -e[1];
}

// A DGU law fitted linear to the data, as the warm start: C from charge against capacitor voltage,
// L from flux against current, R from the resistor's drop against current, least squares through
// the origin. Returned as a canonical spec of the trained family:
//   V(q) = 1/2 K q^2 + v_scale v_net(q)          (K = 1/C; v_net's last layer starts at zero)
//   M(q) = softplus(m_net(q)) + m_min            (its last layer starts at the bias giving L)
//   D(q) = softplus(d_net(q)) + d_min            (likewise R)
// with the nets' hidden layers random (scale 1 / sqrt(fan in), from seed).
struct dgu_family {
  std::vector<int> v_hidden = {32, 32}, m_hidden = {32, 32}, d_hidden = {32};
  activation act = activation::silu;
  double v_scale = 1.0, m_min = 1e-2, d_min = 1e-3;
  std::uint64_t seed = 7;
};
struct linear_fit {
  double C = 0, L = 0, R = 0;
};
linear_fit fit_linear_dgu(const dgu_dataset &d);
canonical_spec warm_start_dgu(const linear_fit &fit, const dgu_family &family);
// The fitted-linear law itself (the paper's fitted-linear control).
canonical_spec linear_dgu_spec(const linear_fit &fit);

// The losses over samples (indices into d.samples; all when empty) and, when grad is non-null,
// their gradient in trainable_parameters' order. threads > 1 splits the samples over that many
// tapes.
struct dgu_losses {
  double constraint = 0, step = 0, reg = 0, total = 0;
};
struct dgu_loss_weights {
  double step = 1.0, constraint = 0.01, reg = 1e-8;
  double dt = 0.01;
  int nsub = 2;
  solve_opts solve;
};
dgu_losses dgu_loss(const canonical_spec &law, const dgu_dataset &d,
                    const std::vector<int> &samples, const dgu_loss_weights &w,
                    vecd *grad = nullptr, int threads = 1);

struct dgu_train_options {
  int updates = 2000;
  int constraint_first = 0; // updates of L_constraint + reg alone before the joint loss (the
                            // paper's "constraint-first" schedule; 0 is "combined loss throughout")
  int batch = 128;
  double lr = 1e-4;
  dgu_loss_weights weights;
  int threads = 1;
  int log_every = 0; // record the losses every this many updates (0: only at the end)
  std::uint64_t seed = 11;
};
struct dgu_train_record {
  int update = 0;
  dgu_losses batch;
};
struct dgu_train_result {
  canonical_spec law;
  std::vector<dgu_train_record> history;
  int parameters = 0;
};
dgu_train_result train_dgu(const canonical_spec &init, const dgu_dataset &train,
                           const dgu_train_options &o);

// Open-loop rollouts of a DGU law on each run's start under the dataset's inputs, LocalStep(dt, 2),
// scored against the run: capacitor-voltage (e3) and inductor-current RMSE over every step after
// the start. finite is false when any rollout leaves the reals.
struct dgu_eval {
  double voltage_rmse = 0, current_rmse = 0;
  bool finite = true;
};
dgu_eval evaluate_dgu(const block_law &law, const block_law &reference, const dgu_dataset &test,
                      int nsub = 2, const solve_opts &solve = {});

} // namespace ph
} // namespace cvc

#endif
