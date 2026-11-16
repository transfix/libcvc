/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph composition (inc/cvc/ph/compose.h): the structure a composed port-Hamiltonian system
// must have whatever its laws are, and the accuracy of its transitions. Nothing here needs a
// golden file:
//
//   - the power balance at random states of random heterogeneous systems (the pairing does no
//     work; dH/dt = -loss + supplied);
//   - the split step's energy residual falling as dt^2 with Euler coupling and dt^3 with midpoint
//     coupling, and its global first and second order against RK4;
//   - the whole-field implicit midpoint balancing a linear system exactly;
//   - a DC microgrid (linear DGUs, RL lines on a chain, and a fifth DGU added at one of two
//     buses), written as a cvc-ph-system/1 file, against a fine-step RK4 reference;
//   - the incidence pairing's signs, rewiring, malformed specs and files.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <cvc/ph/rng.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;
using namespace cvc::ph;

vecd numbers(const pt::ptree &t) {
  vecd out;
  if (t.empty()) {
    out.push_back(std::stod(t.data()));
    return out;
  }
  for (const auto &kv : t) {
    vecd v = numbers(kv.second);
    out.insert(out.end(), v.begin(), v.end());
  }
  return out;
}

std::string to_json(const pt::ptree &t) {
  std::ostringstream o;
  pt::write_json(o, t, false);
  return o.str();
}

std::string sci(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3e", v);
  return b;
}

// ── a random heterogeneous system ──────────────────────────────────────────────────────────────

mlp random_net(rng &g, int in, int hidden, int out, activation act) {
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

law_ptr random_canonical(rng &g, int n, int ports, int inputs, bool nets) {
  canonical_spec s;
  s.n = n;
  s.K.assign(size_t(n) * n, 0.0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j <= i; ++j)
      s.K[i * n + j] = s.K[j * n + i] = (i == j ? 1.5 : 0.2 * g.normal());
  s.center.assign(n, 0.1);
  if (nets) {
    s.v_net = random_net(g, n, 6, 1, activation::silu);
    s.v_scale = 0.3;
    s.m_net = random_net(g, n, 5, n, activation::tanh);
    s.m_min = 0.2;
    s.d_net = random_net(g, n, 4, n, activation::sigmoid);
    s.d_min = 0.05;
  } else {
    s.mass.assign(n, 1.3);
    s.damping.assign(n, 0.4);
  }
  s.ports = ports;
  s.inputs = inputs;
  for (int i = 0; i < 2 * n * ports; ++i)
    s.B.push_back(g.normal());
  for (int i = 0; i < 2 * n * inputs; ++i)
    s.G.push_back(g.normal());
  return std::make_shared<canonical_law>(s);
}

law_ptr random_linear(rng &g, int n, int ports, int inputs) {
  linear_spec s;
  s.n = n;
  s.ports = ports;
  s.inputs = inputs;
  vecd A(size_t(n) * n), S(size_t(n) * n);
  for (auto &v : A)
    v = g.normal();
  for (auto &v : S)
    v = g.normal();
  s.Q.assign(size_t(n) * n, 0.0);
  s.R.assign(size_t(n) * n, 0.0);
  s.J.assign(size_t(n) * n, 0.0);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      for (int k = 0; k < n; ++k) {
        s.Q[i * n + j] += A[i * n + k] * A[j * n + k] / n; // A A^T / n, PSD
        s.R[i * n + j] += 0.3 * S[i * n + k] * S[j * n + k] / n;
      }
      s.J[i * n + j] = g.normal();
    }
  for (int i = 0; i < n; ++i) {
    s.Q[i * n + i] += 0.5;
    for (int j = 0; j < i; ++j) {
      s.J[i * n + j] = -s.J[j * n + i];
      s.R[i * n + j] = s.R[j * n + i];
      s.Q[i * n + j] = s.Q[j * n + i];
    }
    s.J[i * n + i] = 0;
  }
  for (int i = 0; i < n * ports; ++i)
    s.B.push_back(g.normal());
  for (int i = 0; i < n * inputs; ++i)
    s.G.push_back(g.normal());
  return std::make_shared<linear_law>(s);
}

composite random_system(unsigned seed) {
  rng g(seed);
  composite s;
  s.add_block(random_canonical(g, 2, 2, 1, true), "a");
  s.add_block(random_canonical(g, 1, 1, 2, true), "b");
  s.add_block(random_linear(g, 3, 2, 1), "c");
  s.add_block(random_canonical(g, 2, 1, 0, false), "d");
  s.add_block(random_linear(g, 1, 1, 0), "e");
  // Rectangular pairings between heterogeneous ports, and a dense skew rule over three blocks.
  s.connect(0, 0, 2, 1, g.normal());
  s.connect(0, 1, 1, 0, g.normal());
  s.connect(2, 0, 3, 0, g.normal());
  s.connect(1, 0, 4, 0, g.normal());
  vecd C(9);
  for (auto &v : C)
    v = g.normal();
  s.connect_skew({0, 2, 3}, 0, C);
  return s;
}

vecd random_vec(rng &g, int n, double scale) {
  vecd v(n);
  for (auto &x : v)
    x = scale * g.normal();
  return v;
}

TEST(PhCompose, InterconnectionDoesNoWorkAndPowerBalances) {
  // The pairing's power u_hat . y_hat = -y_hat . C y_hat = 0, and dH/dt = -e.R e + e.G u.
  for (unsigned seed = 1; seed <= 20; ++seed) {
    const composite s = random_system(seed);
    rng g(1000 + seed);
    for (int k = 0; k < 10; ++k) {
      const vecd x = random_vec(g, s.state_dim(), 1.0), u = random_vec(g, s.input_dim(), 1.0);
      const balance_check c = check_balance(s, x.data(), u.data());
      EXPECT_LT(c.rate_error, 1e-13) << "seed " << seed;
      EXPECT_LT(c.internal_error, 1e-15) << "seed " << seed;
      EXPECT_EQ(c.skew_error, 0.0);
      // And the dissipation term is a loss: never negative.
      EXPECT_GE(s.powers(x.data(), u.data()).loss, 0.0);
    }
  }
}

TEST(PhCompose, EffortsAreTheEnergyGradient) {
  const composite s = random_system(3);
  rng g(7);
  for (int k = 0; k < 5; ++k) {
    vecd x = random_vec(g, s.state_dim(), 0.8), e(s.state_dim());
    s.effort(x.data(), e.data());
    for (int i = 0; i < s.state_dim(); ++i) {
      const double h = 1e-6, x0 = x[i];
      x[i] = x0 + h;
      const double Hp = s.energy(x.data());
      x[i] = x0 - h;
      const double Hm = s.energy(x.data());
      x[i] = x0;
      EXPECT_NEAR(e[i], (Hp - Hm) / (2 * h), 1e-7 * (1 + std::fabs(e[i]))) << "coordinate " << i;
    }
  }
}

TEST(PhCompose, NetworkJacobiansMatchFiniteDifferences) {
  rng g(11);
  for (activation a : {activation::identity, activation::silu, activation::tanh,
                       activation::softplus, activation::sigmoid}) {
    const mlp net = random_net(g, 3, 7, 2, a);
    vecd x = random_vec(g, 3, 1.0), y(2), J(6), yp(2), ym(2);
    net.eval(x.data(), y.data(), J.data());
    for (int i = 0; i < 3; ++i) {
      const double h = 1e-6, x0 = x[i];
      x[i] = x0 + h;
      net.eval(x.data(), yp.data());
      x[i] = x0 - h;
      net.eval(x.data(), ym.data());
      x[i] = x0;
      for (int o = 0; o < 2; ++o)
        EXPECT_NEAR(J[o * 3 + i], (yp[o] - ym[o]) / (2 * h), 1e-8);
    }
  }
}

// ── the transition ─────────────────────────────────────────────────────────────────────────────

double one_step_residual(const composite &s, coupling_scheme c, double dt, const vecd &x0,
                         const vecd &u) {
  transition_opts o;
  o.dt = dt;
  o.coupling = c;
  o.solve.tol = 1e-14;
  o.solve.max_iter = 100;
  vecd x = x0;
  step(s, o, u.data(), x.data());
  return std::fabs(step_books(s, dt, x0.data(), x.data(), u.data()).residual);
}

TEST(PhCompose, SplitStepResidualFallsAsDtSquaredOrCubed) {
  // Euler coupling is first order (its energy defect per step is O(dt^2)); midpoint coupling makes
  // the transition second order (O(dt^3) per step).
  const composite s = random_system(5);
  rng g(5);
  const vecd x0 = random_vec(g, s.state_dim(), 0.7), u = random_vec(g, s.input_dim(), 0.5);
  for (coupling_scheme c : {coupling_scheme::euler, coupling_scheme::midpoint}) {
    const double r1 = one_step_residual(s, c, 0.02, x0, u),
                 r2 = one_step_residual(s, c, 0.01, x0, u),
                 r3 = one_step_residual(s, c, 0.005, x0, u);
    const double order = 0.5 * (std::log2(r1 / r2) + std::log2(r2 / r3));
    const double want = c == coupling_scheme::euler ? 2.0 : 3.0;
    EXPECT_NEAR(order, want, 0.35) << to_string(c) << " residuals " << r1 << " " << r2 << " " << r3;
  }
}

double error_at_one_second(const composite &s, coupling_scheme c, double dt, const vecd &x0,
                           const vecd &u, const vecd &ref) {
  transition_opts o;
  o.dt = dt;
  o.coupling = c;
  o.solve.tol = 1e-14;
  o.solve.max_iter = 100;
  const rollout_result r = rollout(s, o, x0, u, static_cast<int>(std::lround(1.0 / dt)));
  double e = 0;
  for (size_t i = 0; i < ref.size(); ++i)
    e = std::max(e, std::fabs(r.x.back()[i] - ref[i]));
  return e;
}

TEST(PhCompose, EulerCouplingIsFirstOrderMidpointSecond) {
  const composite s = random_system(9);
  rng g(9);
  const vecd x0 = random_vec(g, s.state_dim(), 0.5), u = random_vec(g, s.input_dim(), 0.5);
  transition_opts ref_o;
  ref_o.scheme = step_scheme::rk4;
  ref_o.dt = 1.0 / 4096;
  const vecd ref = rollout(s, ref_o, x0, u, 4096).x.back();
  for (coupling_scheme c : {coupling_scheme::euler, coupling_scheme::midpoint}) {
    const double e1 = error_at_one_second(s, c, 0.02, x0, u, ref),
                 e2 = error_at_one_second(s, c, 0.01, x0, u, ref);
    const double order = std::log2(e1 / e2);
    EXPECT_NEAR(order, c == coupling_scheme::euler ? 1.0 : 2.0, 0.3)
        << to_string(c) << " " << e1 << " " << e2;
  }
}

TEST(PhCompose, ImplicitMidpointBalancesALinearSystemExactly) {
  // For a quadratic H the implicit midpoint's books (midpoint rule) close to the solver tolerance.
  rng g(21);
  composite s;
  s.add_block(random_linear(g, 3, 1, 1), "a");
  s.add_block(random_linear(g, 2, 2, 1), "b");
  s.add_block(random_linear(g, 1, 1, 0), "c");
  s.connect(0, 0, 1, 1, 0.7);
  s.connect(1, 0, 2, 0, -1.3);
  vecd x = random_vec(g, s.state_dim(), 1.0);
  const vecd u = random_vec(g, s.input_dim(), 1.0);
  solve_opts so;
  so.tol = 1e-15;
  so.max_iter = 200;
  double worst = 0, total = 0;
  for (int k = 0; k < 200; ++k) {
    const vecd x0 = x;
    const step_stats st = step_implicit_midpoint(s, 0.01, u.data(), x.data(), so);
    EXPECT_TRUE(st.converged);
    const books b = step_books_midpoint(s, 0.01, x0.data(), x.data(), u.data());
    worst = std::max(worst, std::fabs(b.residual));
    total += std::fabs(b.dH);
  }
  EXPECT_LT(worst, 1e-12 * std::max(1.0, total));
}

TEST(PhCompose, LosslessSplitStepKeepsEnergyClose) {
  // No dissipation, no inputs: the split step's books are its residual alone, O(dt^3) per step.
  rng g(4);
  composite s;
  linear_spec a;
  a.n = 2;
  a.Q = {2.0, 0.0, 0.0, 0.5};
  a.J = {0.0, 1.0, -1.0, 0.0};
  a.ports = 1;
  a.B = {0.0, 1.0};
  linear_spec b = a;
  b.Q = {1.0, 0.0, 0.0, 1.5};
  s.add_block(std::make_shared<linear_law>(a));
  s.add_block(std::make_shared<linear_law>(b));
  s.connect(0, 0, 1, 0, 0.8);
  vecd x = {1.0, 0.2, -0.3, 0.5};
  const double H0 = s.energy(x.data());
  transition_opts o;
  o.dt = 0.01;
  const rollout_result r = rollout(s, o, x, {}, 1000);
  double drift = 0;
  for (const vecd &xk : r.x)
    drift = std::max(drift, std::fabs(s.energy(xk.data()) - H0));
  EXPECT_LT(drift, 1e-4 * H0);
  for (const books &k : r.ledger) {
    EXPECT_EQ(k.supplied, 0.0);
    EXPECT_EQ(k.dissipated, 0.0);
    EXPECT_DOUBLE_EQ(k.residual, k.dH);
  }
}

// ── a DC microgrid ─────────────────────────────────────────────────────────────────────────────
// Linear DGUs (capacitor C = 2.2 F, filter L = 1.8 H, R = 1.2 ohm; state: charge, flux; inputs:
// the current into the bus and the source voltage) and RL lines (0.8 H), as a cvc-ph-system/1 file.

std::string grid_json(int n, const std::vector<std::pair<int, int>> &edges, const vecd &R) {
  std::ostringstream o;
  o.precision(17);
  o << R"({"format": "cvc-ph-system/1", "laws": {"dgu": {"format": "cvc-ph-law/1",)"
    << R"( "kind": "canonical", "n": 1, "potential": {"K": [[)" << 1.0 / 2.2 << R"(]]},)"
    << R"( "mass": {"diag": [1.8]}, "damping": {"diag": [1.2]}, "ports": 1, "B": [[1], [0]],)"
    << R"( "inputs": 2, "G": [[1, 0], [0, 1]], "scheme": "leapfrog"})";
  for (size_t k = 0; k < edges.size(); ++k)
    o << R"(, "line)" << k << R"(": {"format": "cvc-ph-law/1", "kind": "linear", "n": 1, "Q": [)"
      << 1.0 / 0.8 << R"(], "R": [)" << R[k] << R"(], "ports": 1, "B": [1]})";
  o << R"(}, "blocks": [)";
  for (int i = 0; i < n; ++i)
    o << (i ? ", " : "") << R"({"name": "dgu)" << i << R"(", "law": "dgu"})";
  for (size_t k = 0; k < edges.size(); ++k)
    o << R"(, {"name": "line)" << k << R"(", "law": "line)" << k << R"("})";
  o << R"(], "couplings": [{"type": "incidence", "buses": [)";
  for (int i = 0; i < n; ++i)
    o << (i ? ", " : "") << "\"dgu" << i << "\"";
  o << R"(], "lines": [)";
  for (size_t k = 0; k < edges.size(); ++k)
    o << (k ? ", " : "") << "\"line" << k << "\"";
  o << R"(], "A": [)";
  for (int i = 0; i < n; ++i) {
    o << (i ? ", [" : "[");
    for (size_t k = 0; k < edges.size(); ++k)
      o << (k ? ", " : "") << (edges[k].first == i ? -1 : edges[k].second == i ? 1 : 0);
    o << "]";
  }
  o << "]}]}";
  return o.str();
}

// Bus voltages 0.5-1.3 V, DGU currents -0.25-0.35 A, line currents -0.1-0.1 A, as charge and flux.
vecd grid_start(int n, int m) {
  rng g(42);
  vecd x;
  for (int i = 0; i < n; ++i) {
    x.push_back(2.2 * (0.5 + 0.8 * g.uniform()));
    x.push_back(1.8 * (-0.25 + 0.6 * g.uniform()));
  }
  for (int k = 0; k < m; ++k)
    x.push_back(0.8 * (-0.1 + 0.2 * g.uniform()));
  return x;
}

// Per DGU: the current into the bus (minus a 0.05-0.2 A load) and the source voltage (0.9-1.2 V).
vecd grid_inputs(int n) {
  rng g(7);
  vecd u;
  for (int i = 0; i < n; ++i) {
    u.push_back(-(0.05 + 0.15 * g.uniform()));
    u.push_back(0.9 + 0.3 * g.uniform());
  }
  return u;
}

double max_diff(const vecd &a, const vecd &b) {
  double d = 0;
  for (size_t i = 0; i < a.size(); ++i)
    d = std::max(d, std::fabs(a[i] - b[i]));
  return d;
}

// Two seconds of the split step against RK4 at dt / 64: the largest state error over the rollout.
double grid_error(const composite &s, const vecd &x0, const vecd &u, coupling_scheme c) {
  transition_opts o;
  o.coupling = c;
  transition_opts ref;
  ref.scheme = step_scheme::rk4;
  ref.dt = o.dt / 64;
  const rollout_result a = rollout(s, o, x0, u, 200), b = rollout(s, ref, x0, u, 200 * 64);
  double e = 0;
  for (int k = 0; k <= 200; ++k)
    e = std::max(e, max_diff(a.x[k], b.x[64 * k]));
  return e;
}

TEST(PhCompose, AMicrogridFollowsAFineReference) {
  const composite s = parse_system(grid_json(4, {{0, 1}, {1, 2}, {2, 3}}, {0.3, 0.4, 0.5}));
  ASSERT_EQ(s.state_dim(), 11);
  ASSERT_EQ(s.input_dim(), 8);
  const vecd x0 = grid_start(4, 3), u = grid_inputs(4);
  const double euler = grid_error(s, x0, u, coupling_scheme::euler),
               mid = grid_error(s, x0, u, coupling_scheme::midpoint);
  RecordProperty("euler_error", sci(euler));
  RecordProperty("midpoint_error", sci(mid));
  EXPECT_LT(euler, 2e-3); // 8.0e-4 measured (Linux, GCC 13)
  EXPECT_LT(mid, 2e-5);   // 3.4e-6
  EXPECT_LT(50 * mid, euler);
  // And its books close: per step, the residual is small next to what moved.
  transition_opts o;
  const rollout_result r = rollout(s, o, x0, u, 200);
  for (const books &b : r.ledger)
    EXPECT_LT(std::fabs(b.residual), 1e-6);
}

TEST(PhCompose, AnAddedDguHasADifferentFutureAtEachBus) {
  // The four-unit chain one second in, plus a fifth DGU (1.0 V, 0.1 A) and its line (0.4 ohm, no
  // current yet) to bus 0 or to bus 2: the existing storage states are kept, the added ones
  // supplied, and the same laws give two different futures, each following its fine reference.
  const composite now = parse_system(grid_json(4, {{0, 1}, {1, 2}, {2, 3}}, {0.3, 0.4, 0.5}));
  transition_opts o;
  const vecd x1 = rollout(now, o, grid_start(4, 3), grid_inputs(4), 100).x.back();
  vecd x0(x1.begin(), x1.begin() + 8);
  x0.push_back(2.2 * 1.0);
  x0.push_back(1.8 * 0.1);
  x0.insert(x0.end(), x1.begin() + 8, x1.end());
  x0.push_back(0.0);
  vecd u = grid_inputs(4);
  u.push_back(-0.1);
  u.push_back(1.05);
  vecd ends[2];
  for (int k = 0; k < 2; ++k) {
    const int bus = k == 0 ? 0 : 2;
    const composite s =
        parse_system(grid_json(5, {{0, 1}, {1, 2}, {2, 3}, {bus, 4}}, {0.3, 0.4, 0.5, 0.4}));
    EXPECT_LT(grid_error(s, x0, u, coupling_scheme::midpoint), 1e-4) << "bus " << bus;
    ends[k] = rollout(s, o, x0, u, 200).x.back();
  }
  EXPECT_GT(max_diff(ends[0], ends[1]), 1e-3);
}

// ── wiring ─────────────────────────────────────────────────────────────────────────────────────

law_ptr capacitor_bus(double C) {
  canonical_spec s; // q = C v, a negligible-flux inductor in series
  s.n = 1;
  s.K = {1.0 / C};
  s.mass = {1.0};
  s.ports = 1;
  s.B = {1.0, 0.0};
  return std::make_shared<canonical_law>(s);
}

law_ptr line(double L, double R) {
  linear_spec s;
  s.n = 1;
  s.Q = {1.0 / L};
  s.R = {R};
  s.ports = 1;
  s.B = {1.0};
  return std::make_shared<linear_law>(s);
}

TEST(PhCompose, IncidencePairsVoltagesWithLineCurrents) {
  // A line from bus 0 (sending, -1) to bus 1 (receiving, +1): its positive current leaves bus 0
  // and enters bus 1, and v_0 - v_1 drives its flux.
  composite s;
  const int b0 = s.add_block(capacitor_bus(2.0), "bus0"),
            b1 = s.add_block(capacitor_bus(4.0), "bus1");
  const int l = s.add_block(line(0.5, 0.1), "line");
  s.connect_incidence({b0, b1}, 0, {l}, 0, {-1.0, 1.0});
  const vecd x = {2.0 * 1.5, 0.0, 4.0 * 1.0, 0.0, 0.5 * 0.3}; // v = 1.5, 1.0 V; i_line = 0.3 A
  vecd f(5);
  s.coupling_field(x.data(), f.data());
  EXPECT_NEAR(f[0], -0.3, 1e-15); // dq_0 = -i_line
  EXPECT_NEAR(f[2], 0.3, 1e-15);  // dq_1 = +i_line
  EXPECT_NEAR(f[4], 0.5, 1e-15);  // dphi = v_0 - v_1
  EXPECT_EQ(f[1], 0.0);
  EXPECT_EQ(f[3], 0.0);
  // u_hat . y_hat: the bus side gains v^T A i, the line side loses it.
  EXPECT_NEAR(s.powers(x.data(), nullptr).internal, 0.0, 1e-15);
}

TEST(PhCompose, RewiringKeepsEveryLaw) {
  const law_ptr dgu = capacitor_bus(2.2), l = line(0.8, 0.4);
  auto build = [&](int bus) {
    composite s;
    for (int i = 0; i < 3; ++i)
      s.add_block(dgu, "dgu" + std::to_string(i));
    s.add_block(l, "line01");
    s.add_block(l, "new");
    vecd A(6, 0.0); // buses x lines
    A[0 * 2 + 0] = -1;
    A[1 * 2 + 0] = 1;
    A[bus * 2 + 1] = -1;
    A[2 * 2 + 1] = 1;
    s.connect_incidence({0, 1, 2}, 0, {3, 4}, 0, A);
    return s;
  };
  const composite a = build(0), b = build(1);
  for (int i = 0; i < a.blocks(); ++i)
    EXPECT_EQ(a.law_ptr_at(i).get(), b.law_ptr_at(i).get());
  const vecd x0 = {2.2, 0.0, 2.0, 0.0, 3.0, 0.0, 0.0, 0.0};
  transition_opts o;
  const vecd xa = rollout(a, o, x0, {}, 100).x.back(), xb = rollout(b, o, x0, {}, 100).x.back();
  EXPECT_GT(std::fabs(xa[0] - xb[0]), 1e-3);
  EXPECT_NE(a.pairing(), b.pairing());
}

TEST(PhCompose, PairingRefusesSelfAndBadPorts) {
  composite s;
  const int a = s.add_block(line(1, 0));
  const int b = s.add_block(line(1, 0));
  EXPECT_THROW(s.connect(a, 0, a, 0), std::invalid_argument);
  EXPECT_THROW(s.connect(a, 1, b, 0), std::invalid_argument);
  EXPECT_THROW(s.connect(a, 0, 7, 0), std::invalid_argument);
  EXPECT_THROW(s.add_block(nullptr), std::invalid_argument);
  s.add_block(line(1, 0), "x");
  EXPECT_THROW(s.add_block(line(1, 0), "x"), std::invalid_argument);
}

// ── laws and files ─────────────────────────────────────────────────────────────────────────────

TEST(PhCompose, LawsRefuseMalformedSpecs) {
  linear_spec s;
  s.n = 2;
  s.Q = {1, 0, 0, 1};
  s.J = {0, 1, 1, 0}; // symmetric, not skew
  EXPECT_THROW(linear_law{s}, std::invalid_argument);
  s.J = {0, 1, -1, 0};
  s.R = {1, 0.5, 0, 1}; // not symmetric
  EXPECT_THROW(linear_law{s}, std::invalid_argument);
  canonical_spec c;
  c.n = 1;
  c.mass = {0.0};
  EXPECT_THROW(canonical_law{c}, std::invalid_argument);
  rng g(1);
  c.mass = {};
  c.m_net = random_net(g, 1, 3, 1, activation::tanh);
  c.m_min = 0.1;
  c.scheme = conservative_scheme::leapfrog; // needs constant mass
  EXPECT_THROW(canonical_law{c}, std::invalid_argument);
}

TEST(PhCompose, FilesReportWhatIsWrong) {
  EXPECT_THROW(parse_law(R"({"format": "cvc-ph-law/9", "kind": "linear"})"), std::runtime_error);
  EXPECT_THROW(parse_law(R"({"format": "cvc-ph-law/1", "kind": "spring"})"), std::runtime_error);
  EXPECT_THROW(parse_law(R"({"format": "cvc-ph-law/1", "kind": "linear", "n": 2, "Q": [1, 0, 0, 1],
                             "J": [[0, 1], [1, 0]]})"),
               std::runtime_error);
  EXPECT_THROW(
      parse_system(
          R"({"format": "cvc-ph-system/1", "laws": {}, "blocks": [{"name": "a", "law": "x"}]})"),
      std::runtime_error);
  EXPECT_THROW(load_system("/nonexistent/system.json"), std::runtime_error);
  const law_ptr l =
      parse_law(R"({"format": "cvc-ph-law/1", "kind": "linear", "n": 1, "Q": [2], "R": [0.5],
                                  "ports": 1, "B": [1]})");
  EXPECT_EQ(l->kind(), "linear");
  EXPECT_EQ(l->state_dim(), 1);
  const double x = 3.0;
  EXPECT_DOUBLE_EQ(l->energy(&x), 9.0);
}

TEST(PhCompose, ASystemRoundTripsThroughFiles) {
  const fs::path dir = fs::temp_directory_path() / "ph_compose_test";
  fs::create_directories(dir);
  pt::ptree sys;
  {
    std::istringstream in(grid_json(4, {{0, 1}, {1, 2}, {2, 3}}, {0.3, 0.4, 0.5}));
    pt::read_json(in, sys);
  }
  // The laws given as paths, relative to the system file.
  pt::ptree laws;
  for (const auto &kv : sys.get_child("laws")) {
    pt::write_json((dir / (kv.first + ".json")).string(), kv.second);
    laws.put(kv.first, kv.first + ".json");
  }
  pt::ptree linked = sys;
  linked.put_child("laws", laws);
  pt::write_json((dir / "system.json").string(), linked);
  const composite a = load_system((dir / "system.json").string());
  const composite b = parse_system(to_json(sys));
  EXPECT_EQ(a.state_dim(), b.state_dim());
  EXPECT_EQ(a.pairing(), b.pairing());
  const vecd x = grid_start(4, 3);
  EXPECT_EQ(a.energy(x.data()), b.energy(x.data()));
  fs::remove_all(dir);
}

} // namespace
