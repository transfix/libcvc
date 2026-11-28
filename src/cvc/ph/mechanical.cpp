/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph mechanics (inc/cvc/ph/mechanical.h): the planar arm, the state from positions, the
// count-transfer coupling, the Go2's legs and contact.

#include <algorithm>
#include <array>
#include <cmath>
#include <cvc/ph/mechanical.h>
#include <cvc/ph/query.h>
#include <cvc/ph/rng.h>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace cvc {
namespace ph {

namespace {

[[noreturn]] void bad(const std::string &what) { throw std::invalid_argument("cvc::ph: " + what); }

constexpr double kPi = 3.14159265358979323846;

// Solves A x = b (n x n, row-major) in place by elimination with partial pivoting; false when
// singular.
bool solve(int n, vecd A, vecd &b) {
  for (int k = 0; k < n; ++k) {
    int piv = k;
    for (int r = k + 1; r < n; ++r)
      if (std::fabs(A[r * n + k]) > std::fabs(A[piv * n + k]))
        piv = r;
    if (!(std::fabs(A[piv * n + k]) > 0))
      return false;
    if (piv != k) {
      for (int c = 0; c < n; ++c)
        std::swap(A[k * n + c], A[piv * n + c]);
      std::swap(b[k], b[piv]);
    }
    for (int r = k + 1; r < n; ++r) {
      const double f = A[r * n + k] / A[k * n + k];
      for (int c = k; c < n; ++c)
        A[r * n + c] -= f * A[k * n + c];
      b[r] -= f * b[k];
    }
  }
  for (int k = n - 1; k >= 0; --k) {
    double s = b[k];
    for (int c = k + 1; c < n; ++c)
      s -= A[k * n + c] * b[c];
    b[k] = s / A[k * n + k];
  }
  return true;
}

// torch.nn.functional.softplus(x, beta) (threshold 20).
double softplus(double x, double beta) {
  return beta * x > 20 ? x : std::log1p(std::exp(beta * x)) / beta;
}

// x <- (I + c A)^-1 (I - c A) x for the n x n A.
void cayley(int n, const vecd &A, double c, double *x) {
  vecd M(size_t(n) * n), b(n);
  for (int i = 0; i < n; ++i) {
    double v = x[i];
    for (int j = 0; j < n; ++j) {
      M[size_t(i) * n + j] = (i == j ? 1.0 : 0.0) + c * A[size_t(i) * n + j];
      v -= c * A[size_t(i) * n + j] * x[j];
    }
    b[i] = v;
  }
  if (!solve(n, M, b))
    throw std::runtime_error("cvc::ph: singular arm coupling step");
  std::copy(b.begin(), b.end(), x);
}

double half_norm2(const vecd &r) {
  double s = 0;
  for (double v : r)
    s += v * v;
  return 0.5 * s;
}

// Levenberg-Marquardt from x on residual(x, r) (false: no residual there), with a central-
// difference Jacobian, as fit_history runs it: (J^T J + lambda diag(J^T J)) dx = -J^T r, lambda /
// 3 on a better step and x 4 on a worse one.
struct lm_result {
  vecd x;
  double cost = 0;
  int evals = 0;
};

lm_result levenberg_marquardt(vecd x, const std::function<bool(const vecd &, vecd &)> &residual,
                              int max_evals, double h, double ftol, double xtol) {
  const int P = static_cast<int>(x.size());
  lm_result res;
  vecd r;
  if (!residual(x, r))
    throw std::runtime_error("cvc::ph: no residual at the fit's start");
  res.evals = 1;
  double cost = half_norm2(r);
  const int rows = static_cast<int>(r.size());
  vecd J, rp, rm, rt, t;
  auto jacobian = [&]() {
    J.assign(size_t(rows) * P, 0.0);
    t = x;
    for (int j = 0; j < P; ++j) {
      t[j] = x[j] + h;
      const bool a = residual(t, rp);
      t[j] = x[j] - h;
      const bool b = residual(t, rm);
      t[j] = x[j];
      if (!a || !b)
        return false;
      for (int i = 0; i < rows; ++i)
        J[size_t(i) * P + j] = (rp[i] - rm[i]) / (2 * h);
    }
    return true;
  };
  double lambda = 1e-3;
  bool have_j = jacobian();
  vecd A(size_t(P) * P), g(P);
  while (have_j && P > 0 && res.evals < max_evals) {
    std::fill(A.begin(), A.end(), 0.0);
    std::fill(g.begin(), g.end(), 0.0);
    for (int i = 0; i < rows; ++i)
      for (int a = 0; a < P; ++a) {
        const double ja = J[size_t(i) * P + a];
        g[a] += ja * r[i];
        for (int b = 0; b < P; ++b)
          A[size_t(a) * P + b] += ja * J[size_t(i) * P + b];
      }
    vecd M = A, d(P);
    for (int a = 0; a < P; ++a) {
      M[size_t(a) * P + a] += lambda * std::max(A[size_t(a) * P + a], 1e-12);
      d[a] = -g[a];
    }
    if (!solve(P, M, d)) {
      lambda *= 10;
      continue;
    }
    vecd trial(P);
    double dn = 0, xn = 0;
    for (int a = 0; a < P; ++a) {
      trial[a] = x[a] + d[a];
      dn += d[a] * d[a];
      xn += x[a] * x[a];
    }
    const bool ok = residual(trial, rt);
    ++res.evals;
    const double tcost = ok ? half_norm2(rt) : std::numeric_limits<double>::infinity();
    if (tcost < cost) {
      const double drop = cost - tcost, old = cost;
      x = trial;
      r = rt;
      cost = tcost;
      lambda = std::max(lambda / 3, 1e-12);
      if (drop <= ftol * old || std::sqrt(dn) <= xtol * (xtol + std::sqrt(xn)))
        break;
      have_j = jacobian();
    } else {
      lambda *= 4;
      if (lambda > 1e16)
        break;
    }
  }
  res.x = x;
  res.cost = cost;
  return res;
}

const canonical_law *canonical(const block_law &L) {
  return dynamic_cast<const canonical_law *>(&L);
}

} // namespace

// ── the planar arm ──────────────────────────────────────────────────────────────────────────────

vecd arm_true_coupling(int n, const arm_params &p) {
  if (n < 1)
    bad("an arm needs at least one joint");
  vecd C(size_t(n) * n, 0.0);
  const double w = p.coupling / (p.normalize ? std::sqrt(double(n)) : 1.0);
  for (int i = 0; i + 1 < n; ++i) {
    C[size_t(i) * n + i + 1] = w;
    C[size_t(i + 1) * n + i] = -w;
  }
  return C;
}

void arm_true_step(int n, const arm_params &p, const vecd &C, double *q, double *qdot,
                   const double *tau) {
  const bool coupled = p.coupling != 0 && n >= 2;
  vecd A;
  if (coupled) {
    A.resize(size_t(n) * n);
    for (size_t i = 0; i < A.size(); ++i)
      A[i] = C[i] / p.inertia;
  }
  const double c = 0.25 * p.dt; // phast: 0.5 x (step = dt / 2)
  if (coupled)
    cayley(n, A, c, qdot);
  vecd q0(q, q + n);
  for (int i = 0; i < n; ++i) {
    const double alpha = (tau[i] - p.damping * qdot[i] - p.gravity * std::sin(q[i])) / p.inertia;
    qdot[i] += p.dt * alpha;
    q[i] += p.dt * qdot[i];
  }
  if (coupled)
    cayley(n, A, c, qdot);
  for (int i = 0; i < n; ++i) {
    q[i] =
        -p.limit + softplus(q[i] + p.limit, p.limit_beta) - softplus(q[i] - p.limit, p.limit_beta);
    qdot[i] = (q[i] - q0[i]) / p.dt;
  }
}

std::vector<arm_trajectory> arm_dataset(int n, int count, int steps, std::uint64_t seed,
                                        const arm_params &p) {
  if (n < 1 || count < 0 || steps < 0)
    bad("arm_dataset needs n >= 1, count >= 0 and steps >= 0");
  const vecd C = arm_true_coupling(n, p);
  rng g(seed);
  std::vector<arm_trajectory> out(count);
  for (arm_trajectory &tr : out) {
    vecd q(n), qd(n), phase(n), freq(n), tau(n);
    for (int i = 0; i < n; ++i) { // phast's state layout: q_1, qdot_1, q_2, qdot_2, ...
      q[i] = p.init_scale * g.normal();
      qd[i] = p.init_scale * g.normal();
    }
    for (int i = 0; i < n; ++i)
      phase[i] = 2 * kPi * g.uniform();
    for (int i = 0; i < n; ++i)
      freq[i] = p.torque_freq * (0.5 + g.uniform());
    tr.q.push_back(q);
    tr.qdot.push_back(qd);
    for (int k = 0; k < steps; ++k) {
      const double t = k * p.dt;
      for (int i = 0; i < n; ++i)
        tau[i] = p.torque_scale * std::sin(2 * kPi * freq[i] * t + phase[i]);
      tr.tau.push_back(tau);
      arm_true_step(n, p, C, q.data(), qd.data(), tau.data());
      tr.q.push_back(q);
      tr.qdot.push_back(qd);
    }
  }
  return out;
}

law_ptr arm_joint(const arm_params &p) {
  canonical_spec s;
  s.n = 1;
  s.mass = {p.inertia};
  s.damping = {p.damping};
  s.cosine = {p.gravity};
  s.ports = 1;
  s.B = {0.0, 1.0};
  s.inputs = 1;
  s.G = {0.0, 1.0};
  s.scheme = conservative_scheme::leapfrog;
  return std::make_shared<canonical_law>(std::move(s));
}

vecd count_coupling(const vecd &U, int smax, int s) {
  if (smax < 1 || U.size() != size_t(smax) * smax)
    bad("count_coupling needs U of smax x smax");
  if (s < 1 || s > smax)
    bad("count_coupling needs 1 <= s <= smax");
  vecd C(size_t(s) * s);
  const double w = 1.0 / std::sqrt(double(s));
  for (int a = 0; a < s; ++a)
    for (int b = 0; b < s; ++b)
      C[size_t(a) * s + b] = w * (U[size_t(a) * smax + b] - U[size_t(b) * smax + a]);
  return C;
}

composite arm_system(law_ptr joint, int n, const vecd &C_hat) {
  if (n < 1)
    bad("arm_system needs n >= 1");
  if (!C_hat.empty() && C_hat.size() != size_t(n) * n)
    bad("arm_system: the coupling must be n x n");
  composite s;
  std::vector<int> j;
  for (int i = 0; i < n; ++i)
    j.push_back(s.add_block(joint, "j" + std::to_string(i)));
  if (!C_hat.empty() && n >= 2)
    s.connect_skew(j, 0, C_hat);
  return s;
}

transition_opts arm_transition(const arm_params &p) {
  transition_opts o;
  o.dt = p.dt;
  o.substeps = 1;
  o.local_substeps = 1;
  o.coupling = coupling_scheme::cayley;
  o.order = split_order::inputs_outside;
  return o;
}

// ── the state from positions ────────────────────────────────────────────────────────────────────

std::vector<int> position_channels(const composite &s) {
  std::vector<int> c;
  for (int i = 0; i < s.blocks(); ++i)
    if (const canonical_law *L = canonical(s.law(i)))
      for (int j = 0; j < L->spec().n; ++j)
        c.push_back(s.state_offset(i) + j);
  return c;
}

vecd canonicalize(const composite &s, const vecd &q, const vecd &qdot) {
  const size_t nq = position_channels(s).size();
  if (q.size() != nq || qdot.size() != nq)
    bad("canonicalize needs one position and one velocity per position channel");
  vecd x(s.state_dim(), 0.0), m;
  size_t k = 0;
  for (int i = 0; i < s.blocks(); ++i) {
    const canonical_law *L = canonical(s.law(i));
    if (!L)
      continue;
    const int n = L->spec().n, o = s.state_offset(i);
    m.resize(n);
    L->mass(q.data() + k, m.data());
    for (int j = 0; j < n; ++j) {
      x[o + j] = q[k + j];
      x[o + n + j] = m[j] * qdot[k + j];
    }
    k += n;
  }
  return x;
}

vecd observe(const composite &s, const transition_opts &o, const std::vector<vecd> &positions,
             const std::vector<vecd> &inputs, const observer_options &obs) {
  const std::vector<int> ch = position_channels(s);
  const size_t L = positions.size();
  if (L < 2)
    bad("observe needs at least two frames");
  if (inputs.size() != L - 1)
    bad("observe needs one input row between each pair of frames");
  for (const vecd &q : positions)
    if (q.size() != ch.size())
      bad("observe: a frame has " + std::to_string(q.size()) + " positions for " +
          std::to_string(ch.size()) + " channels");
  if (obs.kind == observer_kind::finite_difference) {
    vecd qd(ch.size());
    for (size_t j = 0; j < ch.size(); ++j)
      qd[j] = (positions[L - 1][j] - positions[L - 2][j]) / o.dt;
    return canonicalize(s, positions[L - 1], qd);
  }
  history h;
  h.inputs = inputs;
  h.channels = ch;
  h.samples = positions;
  for (size_t k = 0; k < L; ++k)
    h.sample_steps.push_back(static_cast<int>(k));
  history_fit_options f;
  f.step = o;
  f.space = fit_space::state;
  f.prior_mean.assign(s.state_dim(), 0.0);
  f.prior_scale.assign(s.state_dim(), obs.momentum_scale);
  for (size_t j = 0; j < ch.size(); ++j) {
    f.prior_mean[ch[j]] = positions[0][j];
    f.prior_scale[ch[j]] = obs.position_scale;
  }
  f.noise_floor = obs.noise_floor;
  f.max_evals = obs.max_evals;
  return fit_history(s, h, f).x_now;
}

std::vector<vecd> takeover(const composite &s, const transition_opts &o, const vecd &x0,
                           const std::vector<vecd> &inputs) {
  const std::vector<int> ch = position_channels(s);
  if (x0.size() != size_t(s.state_dim()))
    bad("takeover: the start has the wrong size");
  std::vector<vecd> out;
  vecd x = x0, q(ch.size());
  for (const vecd &u : inputs) {
    if (u.size() != size_t(s.input_dim()))
      bad("takeover: an input row has the wrong size");
    step(s, o, u.data(), x.data());
    for (size_t j = 0; j < ch.size(); ++j)
      q[j] = x[ch[j]];
    out.push_back(q);
  }
  return out;
}

// ── the count-transfer study ────────────────────────────────────────────────────────────────────

namespace {

// One takeover window: its observed frames and inputs, then the inputs and positions to predict.
struct window {
  std::vector<vecd> seen, seen_inputs, inputs, future;
};

std::vector<window> windows_of(const std::vector<arm_trajectory> &data, const takeover_options &t) {
  if (t.context < 2 || t.horizon < 1 || t.windows < 1)
    bad("takeover needs context >= 2, horizon >= 1 and windows >= 1");
  std::vector<window> out;
  const int span = t.context + t.horizon;
  for (const arm_trajectory &tr : data)
    for (int w = 0; w < t.windows; ++w) {
      const int a = w * span, t0 = a + t.context - 1;
      if (t0 + t.horizon > static_cast<int>(tr.tau.size()))
        bad("takeover: a trajectory is shorter than its windows");
      window win;
      for (int k = a; k <= t0; ++k)
        win.seen.push_back(tr.q[k]);
      for (int k = a; k < t0; ++k)
        win.seen_inputs.push_back(tr.tau[k]);
      for (int k = t0; k < t0 + t.horizon; ++k) {
        win.inputs.push_back(tr.tau[k]);
        win.future.push_back(tr.q[k + 1]);
      }
      out.push_back(std::move(win));
    }
  return out;
}

} // namespace

double takeover_mse(const composite &s, const transition_opts &o,
                    const std::vector<arm_trajectory> &data, const takeover_options &t) {
  double sum = 0;
  size_t count = 0;
  for (const window &w : windows_of(data, t)) {
    const vecd x = observe(s, o, w.seen, w.seen_inputs, t.observer);
    const std::vector<vecd> pred = takeover(s, o, x, w.inputs);
    for (size_t k = 0; k < pred.size(); ++k)
      for (size_t j = 0; j < pred[k].size(); ++j) {
        const double e = pred[k][j] - w.future[k][j];
        sum += e * e;
        ++count;
      }
  }
  return count ? sum / double(count) : 0.0;
}

coupling_fit_result fit_arm_coupling(law_ptr joint, int n, const std::vector<arm_trajectory> &data,
                                     const transition_opts &o, const coupling_fit_options &f) {
  if (n < 1 || n > f.smax)
    bad("fit_arm_coupling needs 1 <= n_train <= smax");
  const std::vector<window> wins = windows_of(data, f.takeover);
  // The skew entries above the diagonal, row by row.
  auto coupling = [&](const vecd &w) {
    vecd C(size_t(n) * n, 0.0);
    size_t k = 0;
    for (int a = 0; a < n; ++a)
      for (int b = a + 1; b < n; ++b) {
        C[size_t(a) * n + b] = w[k];
        C[size_t(b) * n + a] = -w[k];
        ++k;
      }
    return C;
  };
  vecd w(size_t(n) * (n - 1) / 2, 0.0);
  coupling_fit_result res;
  res.smax = f.smax;
  res.mse_start = takeover_mse(arm_system(joint, n), o, data, f.takeover);
  for (int round = 0; round < f.rounds && !w.empty(); ++round) {
    const composite now = arm_system(joint, n, coupling(w));
    std::vector<vecd> starts;
    for (const window &win : wins)
      starts.push_back(observe(now, o, win.seen, win.seen_inputs, f.takeover.observer));
    auto residual = [&](const vecd &wt, vecd &r) {
      const composite s = arm_system(joint, n, coupling(wt));
      r.clear();
      for (size_t i = 0; i < wins.size(); ++i) {
        const std::vector<vecd> pred = takeover(s, o, starts[i], wins[i].inputs);
        for (size_t k = 0; k < pred.size(); ++k)
          for (size_t j = 0; j < pred[k].size(); ++j)
            r.push_back(pred[k][j] - wins[i].future[k][j]);
      }
      for (double v : r)
        if (!std::isfinite(v))
          return false;
      return true;
    };
    const lm_result lm =
        levenberg_marquardt(w, residual, f.max_evals, f.jacobian_step, f.ftol, f.xtol);
    w = lm.x;
    res.evals += lm.evals;
  }
  res.U.assign(size_t(f.smax) * f.smax, 0.0);
  const double scale = std::sqrt(double(n));
  size_t k = 0;
  for (int a = 0; a < n; ++a)
    for (int b = a + 1; b < n; ++b)
      res.U[size_t(a) * f.smax + b] = scale * w[k++];
  res.mse =
      takeover_mse(arm_system(joint, n, count_coupling(res.U, f.smax, n)), o, data, f.takeover);
  return res;
}

// ── the Go2's legs ──────────────────────────────────────────────────────────────────────────────
//
// The legs' joint offsets, axes, foot position and body inertials below are numbers from
// mujoco_menagerie's unitree_go2/go2.xml, which carries this notice:
//
//   Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice, this
//     list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

const char *to_string(go2_leg leg) {
  switch (leg) {
  case go2_leg::front_left:
    return "FL";
  case go2_leg::front_right:
    return "FR";
  case go2_leg::rear_left:
    return "RL";
  case go2_leg::rear_right:
    return "RR";
  }
  return "FL";
}

go2_leg go2_leg_of(const std::string &s) {
  for (go2_leg l :
       {go2_leg::front_left, go2_leg::front_right, go2_leg::rear_left, go2_leg::rear_right})
    if (s == to_string(l))
      return l;
  bad("unknown Go2 leg '" + s + "' (FL, FR, RL, RR)");
}

namespace {

using vec3 = std::array<double, 3>;
using mat3 = std::array<double, 9>; // row-major

// One body of a leg in go2.xml: its inertial (mass, centre, principal axes as a w, x, y, z
// quaternion, principal moments).
struct body_inertial {
  double mass;
  vec3 com;
  std::array<double, 4> quat;
  vec3 diag;
};

// The legs' bodies, hip, thigh, calf (go2.xml, mujoco_menagerie unitree_go2).
std::array<body_inertial, 3> leg_inertials(go2_leg leg) {
  const bool left = leg == go2_leg::front_left || leg == go2_leg::rear_left;
  const bool front = leg == go2_leg::front_left || leg == go2_leg::front_right;
  const vec3 calf_d{0.0014901, 0.00146356, 5.31397e-05};
  const vec3 thigh_d{0.00594973, 0.00584149, 0.000878787};
  const vec3 hip_d{0.00088403, 0.000596003, 0.000479967};
  body_inertial hip{
      0.678, {front ? -0.0054 : 0.0054, left ? 0.00194 : -0.00194, -0.000105}, {}, hip_d};
  if (leg == go2_leg::front_left)
    hip.quat = {0.497014, 0.499245, 0.505462, 0.498237};
  else if (leg == go2_leg::front_right)
    hip.quat = {0.498237, 0.505462, 0.499245, 0.497014};
  else if (leg == go2_leg::rear_left)
    hip.quat = {0.505462, 0.498237, 0.497014, 0.499245};
  else
    hip.quat = {0.499245, 0.497014, 0.498237, 0.505462};
  const body_inertial thigh = left ? body_inertial{1.152,
                                                   {-0.00374, -0.0223, -0.0327},
                                                   {0.829533, 0.0847635, -0.0200632, 0.551623},
                                                   thigh_d}
                                   : body_inertial{1.152,
                                                   {-0.00374, 0.0223, -0.0327},
                                                   {0.551623, -0.0200632, 0.0847635, 0.829533},
                                                   thigh_d};
  const body_inertial calf = left ? body_inertial{0.241352,
                                                  {0.00629595, -0.000622121, -0.141417},
                                                  {0.710672, 0.00154099, -0.00450087, 0.703508},
                                                  calf_d}
                                  : body_inertial{0.241352,
                                                  {0.00629595, 0.000622121, -0.141417},
                                                  {0.703508, -0.00450087, 0.00154099, 0.710672},
                                                  calf_d};
  return {hip, thigh, calf};
}

vec3 turn(const mat3 &R, const vec3 &v) {
  return {R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
          R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}

mat3 compose(const mat3 &A, const mat3 &B) {
  mat3 C{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
}

mat3 axis_angle(const vec3 &a, double t) {
  const double c = std::cos(t), s = std::sin(t), v = 1 - c;
  return {c + a[0] * a[0] * v,        a[0] * a[1] * v - a[2] * s, a[0] * a[2] * v + a[1] * s,
          a[1] * a[0] * v + a[2] * s, c + a[1] * a[1] * v,        a[1] * a[2] * v - a[0] * s,
          a[2] * a[0] * v - a[1] * s, a[2] * a[1] * v + a[0] * s, c + a[2] * a[2] * v};
}

mat3 of_quat(const std::array<double, 4> &q0) {
  const double n = std::sqrt(q0[0] * q0[0] + q0[1] * q0[1] + q0[2] * q0[2] + q0[3] * q0[3]);
  const double w = q0[0] / n, x = q0[1] / n, y = q0[2] / n, z = q0[3] / n;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y),
          2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)};
}

} // namespace

serial_chain go2_foot(go2_leg leg) {
  const bool left = leg == go2_leg::front_left || leg == go2_leg::rear_left;
  const bool front = leg == go2_leg::front_left || leg == go2_leg::front_right;
  serial_chain c;
  c.offset = {{front ? 0.1934 : -0.1934, left ? 0.0465 : -0.0465, 0.0},
              {0.0, left ? 0.0955 : -0.0955, 0.0},
              {0.0, 0.0, -0.213}};
  c.axis = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};
  c.point = {-0.002, 0.0, -0.213};
  c.components = {0, 1, 2};
  return c;
}

vecd go2_leg_inertia(go2_leg leg, const vecd &q) {
  if (q.size() != 3)
    bad("go2_leg_inertia needs three joint angles");
  const serial_chain chain = go2_foot(leg);
  const auto bodies = leg_inertials(leg);
  // Each body's frame (the joint at its origin) in the base frame.
  std::array<vec3, 3> origin, w;
  std::array<mat3, 3> R;
  mat3 P{1, 0, 0, 0, 1, 0, 0, 0, 1};
  vec3 o{0, 0, 0};
  for (int j = 0; j < 3; ++j) {
    const vec3 d = turn(P, chain.offset[j]);
    for (int k = 0; k < 3; ++k)
      o[k] += d[k];
    origin[j] = o;
    w[j] = turn(P, chain.axis[j]);
    P = compose(P, axis_angle(chain.axis[j], q[j]));
    R[j] = P;
  }
  // M_ii = armature + sum over the bodies the joint carries of m |w_i x (c - o_i)|^2 + w_i . I w_i.
  const double armature = 0.01;
  vecd M(3, armature);
  for (int j = 0; j < 3; ++j) {
    const body_inertial &b = bodies[j];
    const vec3 dc = turn(R[j], b.com);
    const vec3 c{origin[j][0] + dc[0], origin[j][1] + dc[1], origin[j][2] + dc[2]};
    const mat3 F = compose(R[j], of_quat(b.quat)); // the principal axes in the base frame
    for (int i = 0; i <= j; ++i) {
      const vec3 l{c[0] - origin[i][0], c[1] - origin[i][1], c[2] - origin[i][2]};
      const vec3 v{w[i][1] * l[2] - w[i][2] * l[1], w[i][2] * l[0] - w[i][0] * l[2],
                   w[i][0] * l[1] - w[i][1] * l[0]};
      double rot = 0;
      for (int k = 0; k < 3; ++k) {
        const double a = F[0 * 3 + k] * w[i][0] + F[1 * 3 + k] * w[i][1] + F[2 * 3 + k] * w[i][2];
        rot += b.diag[k] * a * a;
      }
      M[i] += b.mass * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) + rot;
    }
  }
  return M;
}

law_ptr go2_leg_law(go2_leg leg, const go2_leg_options &o) {
  canonical_spec s;
  s.n = 3;
  s.mass = go2_leg_inertia(leg, o.q_ref);
  s.damping.assign(3, o.damping);
  s.port_points = {go2_foot(leg)};
  s.inputs = 3;
  s.G.assign(6 * 3, 0.0);
  for (int j = 0; j < 3; ++j)
    s.G[size_t(3 + j) * 3 + j] = 1.0;
  s.input_points = {go2_foot(leg)};
  s.scheme = conservative_scheme::leapfrog;
  return std::make_shared<canonical_law>(std::move(s));
}

// ── contact ─────────────────────────────────────────────────────────────────────────────────────

int attach_contact(composite &s, int block, int port, double stiffness, double damping,
                   std::string name) {
  const int c = s.add_block(std::make_shared<contact_law>(stiffness), std::move(name));
  s.connect(block, port, c, 0, -1.0);
  s.connect_damper(block, port, damping, c);
  return c;
}

} // namespace ph
} // namespace cvc
