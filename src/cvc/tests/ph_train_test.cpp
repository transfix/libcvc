/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph training (inc/cvc/ph/ad.h, inc/cvc/ph/train.h):
//   - the tape's derivatives against finite differences;
//   - the law over double is the runtime canonical_law, to rounding (one step and its efforts);
//   - the loss's tape gradient against finite differences of the loss, and threads agreeing;
//   - the linear fit recovering a linear DGU's C, L and R;
//   - a short run on the reference nonlinear DGU beating its fitted-linear warm start on held-out
//     open-loop rollouts.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ph/ad.h>
#include <cvc/ph/compose.h>
#include <cvc/ph/microgrid.h>
#include <cvc/ph/rng.h>
#include <cvc/ph/train.h>
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace cvc::ph;

std::string sci(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3e", v);
  return b;
}

// ── the tape ───────────────────────────────────────────────────────────────────────────────────

template <class T> T expression(const T &x, const T &y) {
  using namespace ad;
  const T a = x * y + T(2.0) * x - y / (T(1.5) + x * x);
  return silu(a) + tanh(y) * softplus(x - y) + sigmoid(a * y) + exp(T(0.3) * x) * log1p(y * y) +
         sqrt(T(1.0) + x * x) - (-a);
}

TEST(PhTrain, TheTapesDerivativesMatchFiniteDifferences) {
  rng g(3);
  for (int k = 0; k < 50; ++k) {
    const double x0 = 2 * g.normal(), y0 = 2 * g.normal();
    ad::tape t;
    const ad::var x = ad::variable(t, x0), y = ad::variable(t, y0);
    const ad::var f = expression(x, y);
    EXPECT_DOUBLE_EQ(f.v, expression(x0, y0));
    t.backward(f.i);
    const double h = 1e-6;
    const double dx = (expression(x0 + h, y0) - expression(x0 - h, y0)) / (2 * h);
    const double dy = (expression(x0, y0 + h) - expression(x0, y0 - h)) / (2 * h);
    EXPECT_NEAR(t.adjoint(x.i), dx, 1e-6 * (1 + std::fabs(dx)));
    EXPECT_NEAR(t.adjoint(y.i), dy, 1e-6 * (1 + std::fabs(dy)));
  }
  // A constant records nothing.
  ad::tape t;
  const ad::var c = ad::var(2.0) * ad::var(3.0) + ad::var(1.0);
  EXPECT_TRUE(c.constant());
  EXPECT_EQ(t.size(), 0u);
}

// ── the law over double is the runtime law ─────────────────────────────────────────────────────

mlp random_mlp(rng &g, int in, int hidden, int out, activation act) {
  mlp net;
  const int dims[3] = {in, hidden, out};
  for (int l = 0; l < 2; ++l) {
    dense_layer L;
    L.in = dims[l];
    L.out = dims[l + 1];
    for (int i = 0; i < L.in * L.out; ++i)
      L.W.push_back(g.normal() / std::sqrt(double(L.in)));
    for (int i = 0; i < L.out; ++i)
      L.b.push_back(0.3 * g.normal());
    L.act = l == 0 ? act : activation::identity;
    net.layers.push_back(L);
  }
  return net;
}

canonical_spec random_spec(rng &g, int n) {
  canonical_spec s;
  s.n = n;
  s.K.assign(size_t(n) * n, 0.0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j <= i; ++j)
      s.K[size_t(i) * n + j] = s.K[size_t(j) * n + i] = i == j ? 1.2 : 0.2 * g.normal();
  s.center.assign(n, 0.1);
  s.v_scale = 0.4;
  s.v_net = random_mlp(g, n, 6, 1, activation::silu);
  s.m_net = random_mlp(g, n, 5, n, activation::tanh);
  s.m_min = 0.3;
  s.d_net = random_mlp(g, n, 4, n, activation::softplus);
  s.d_min = 0.05;
  s.inputs = 2;
  for (int i = 0; i < 2 * n * 2; ++i)
    s.G.push_back(g.normal());
  s.ports = 1;
  for (int i = 0; i < 2 * n; ++i)
    s.B.push_back(g.normal());
  return s;
}

TEST(PhTrain, TheLawOverDoubleIsTheRuntimeLaw) {
  rng g(5);
  for (int n : {1, 2})
    for (int k = 0; k < 10; ++k) {
      const canonical_spec s = random_spec(g, n);
      const canonical_law law(s);
      const canonical_t<double> twin = law_over_double(s);
      vecd x(2 * n), e(2 * n), e2(2 * n);
      for (double &v : x)
        v = g.normal();
      law.effort(x.data(), e.data());
      twin.effort(x.data(), e2.data());
      for (int i = 0; i < 2 * n; ++i)
        EXPECT_NEAR(e[i], e2[i], 1e-15 * (1 + std::fabs(e[i])));
      const double u[2] = {0.3 * g.normal(), g.normal()};
      vecd a = x, b = x;
      const solve_opts so;
      const int ia = law.local_step(0.01, 2, u, a.data(), so),
                ib = twin.local_step(0.01, 2, u, b.data(), so);
      EXPECT_EQ(ia, ib);
      for (int i = 0; i < 2 * n; ++i)
        EXPECT_NEAR(a[i], b[i], 1e-15 * (1 + std::fabs(a[i])));
    }
}

TEST(PhTrain, ParametersRoundTrip) {
  rng g(9);
  for (int n : {1, 2}) {
    const canonical_spec s = random_spec(g, n);
    const vecd theta = trainable_parameters(s);
    const canonical_spec back = with_parameters(s, theta);
    EXPECT_EQ(trainable_parameters(back), theta);
    EXPECT_THROW(with_parameters(s, vecd(theta.begin(), theta.end() - 1)), std::invalid_argument);
  }
  // A spec with constant mass and damping trains those constants.
  const linear_fit f{2.2, 1.8, 1.2};
  EXPECT_EQ(trainable_parameters(linear_dgu_spec(f)).size(), 4u); // K, centre, L, R
}

// ── the losses and their gradient ──────────────────────────────────────────────────────────────

dgu_family small_family() {
  dgu_family f;
  f.v_hidden = {6, 6};
  f.m_hidden = {5};
  f.d_hidden = {4};
  return f;
}

TEST(PhTrain, TheTapeGivesTheLossesGradient) {
  dgu_dataset_options o;
  o.trajectories = 3;
  o.transitions = 5;
  const law_ptr ref = reference_dgu();
  const dgu_dataset d = make_dgu_dataset(*ref, o);
  canonical_spec s = warm_start_dgu(fit_linear_dgu(d), small_family());
  // Move the nets' last layers off zero, so every parameter has a gradient.
  rng g(4);
  vecd theta = trainable_parameters(s);
  for (double &v : theta)
    v += 0.05 * g.normal();
  s = with_parameters(s, theta);
  dgu_loss_weights w;
  w.solve.tol = 1e-14; // converged solves: the loss is then smooth in the parameters
  w.solve.max_iter = 100;
  vecd grad;
  const dgu_losses l = dgu_loss(s, d, {}, w, &grad);
  ASSERT_EQ(grad.size(), theta.size());
  EXPECT_GT(l.step, 0.0);
  EXPECT_GT(l.constraint, 0.0);
  double worst = 0;
  for (size_t p = 0; p < theta.size(); ++p) {
    const double h = 1e-6 * (1 + std::fabs(theta[p]));
    vecd tp = theta, tm = theta;
    tp[p] += h;
    tm[p] -= h;
    const double fd = (dgu_loss(with_parameters(s, tp), d, {}, w).total -
                       dgu_loss(with_parameters(s, tm), d, {}, w).total) /
                      (2 * h);
    worst = std::max(worst, std::fabs(fd - grad[p]) / (1e-6 + std::fabs(fd)));
  }
  RecordProperty("worst_relative_gradient_error", sci(worst));
  EXPECT_LT(worst, 1e-4);
  // Four threads add up to the same gradient.
  vecd g4;
  const dgu_losses l4 = dgu_loss(s, d, {}, w, &g4, 4);
  EXPECT_NEAR(l4.total, l.total, 1e-15 * l.total);
  for (size_t p = 0; p < grad.size(); ++p)
    EXPECT_NEAR(g4[p], grad[p], 1e-12 * (1 + std::fabs(grad[p])));
}

TEST(PhTrain, TheLinearFitRecoversALinearDgu) {
  const law_ptr lin = linear_dgu(2.2, 1.8, 1.2);
  dgu_dataset_options o;
  o.trajectories = 4;
  o.transitions = 50;
  const linear_fit f = fit_linear_dgu(make_dgu_dataset(*lin, o));
  EXPECT_NEAR(f.C, 2.2, 1e-9);
  EXPECT_NEAR(f.L, 1.8, 1e-9);
  EXPECT_NEAR(f.R, 1.2, 1e-9);
  // And the losses of the exact law vanish to the integrator's difference from the reference.
  const dgu_dataset d = make_dgu_dataset(*lin, o);
  const dgu_losses l = dgu_loss(linear_dgu_spec(f), d, {}, dgu_loss_weights{});
  EXPECT_LT(l.constraint, 1e-18);
  EXPECT_LT(l.step, 1e-12);
}

// ── a short run ────────────────────────────────────────────────────────────────────────────────

TEST(PhTrain, ARunBeatsTheLinearWarmStartOnHeldOutRollouts) {
  // The reference nonlinear DGU (q = 2.2 (v + v^3/3), 1.8 H, 1.2 ohm), the paper's starts and
  // inputs (v in [-1.5, 1.5] V, phi in [-1, 1] Wb; 0.1 A in, 1 V source), a small family, a short
  // combined-loss run; scored on held-out 2 s rollouts.
  const law_ptr ref = reference_dgu();
  dgu_dataset_options o;
  o.trajectories = 8;
  o.transitions = 100;
  o.seed = 21;
  const dgu_dataset train = make_dgu_dataset(*ref, o);
  o.trajectories = 4;
  o.transitions = 200;
  o.seed = 22;
  const dgu_dataset test = make_dgu_dataset(*ref, o);
  const linear_fit f = fit_linear_dgu(train);
  dgu_family fam;
  fam.v_hidden = {16, 16};
  fam.m_hidden = {8};
  fam.d_hidden = {8};
  const canonical_spec init = warm_start_dgu(f, fam);
  dgu_train_options to;
  to.updates = 400;
  to.batch = 64;
  to.lr = 3e-3;
  to.threads = 4;
  to.log_every = 100;
  const auto t0 = std::chrono::steady_clock::now();
  const dgu_train_result r = train_dgu(init, train, to);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const canonical_law trained(r.law), linear(linear_dgu_spec(f));
  const dgu_eval e_lin = evaluate_dgu(linear, *ref, test), e_tr = evaluate_dgu(trained, *ref, test);
  const dgu_losses l0 = dgu_loss(init, train, {}, to.weights),
                   l1 = dgu_loss(r.law, train, {}, to.weights);
  std::cout << "PhTrain: " << r.parameters << " parameters, " << to.updates << " updates in "
            << secs << " s; joint loss " << l0.total << " -> " << l1.total
            << "; held-out rollouts: linear " << 1e3 * e_lin.voltage_rmse << " mV / "
            << 1e3 * e_lin.current_rmse << " mA, trained " << 1e3 * e_tr.voltage_rmse << " mV / "
            << 1e3 * e_tr.current_rmse << " mA\n";
  RecordProperty("linear_voltage_rmse_V", sci(e_lin.voltage_rmse));
  RecordProperty("trained_voltage_rmse_V", sci(e_tr.voltage_rmse));
  ASSERT_TRUE(e_tr.finite);
  EXPECT_LT(l1.total, 0.5 * l0.total);
  EXPECT_LT(e_tr.voltage_rmse, e_lin.voltage_rmse);
  EXPECT_LT(e_tr.current_rmse, e_lin.current_rmse);
  ASSERT_FALSE(r.history.empty());
  EXPECT_EQ(r.history.back().update, to.updates);
  // The trained law is an ordinary canonical law: it writes out and reads back.
  const law_ptr back = parse_law(law_json(trained));
  double x[2] = {1.0, 0.2}, ea[2], eb[2];
  trained.effort(x, ea);
  back->effort(x, eb);
  EXPECT_EQ(ea[0], eb[0]);
  EXPECT_EQ(ea[1], eb[1]);
}

} // namespace
