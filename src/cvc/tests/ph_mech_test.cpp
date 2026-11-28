/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph mechanics (inc/cvc/ph/mechanical.h): attachment-point ports, Algorithm 2's step (Cayley
// coupling, inputs outside), the planar arm's count-transfer study (the paper's Table 10, KNOWN
// local physics), the state from positions, the Go2's legs and compliant contact. With
// CVC_WITH_MUJOCO, the legs' feet are checked against MuJoCo's own Go2.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <cvc/ph/mechanical.h>
#include <cvc/ph/rng.h>
#include <cvc/physics/mujoco_robot.h>
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

double uniform(rng &g, double lo, double hi) { return lo + (hi - lo) * g.uniform(); }

// A random skew n x n.
vecd random_skew(rng &g, int n, double scale) {
  vecd C(size_t(n) * n, 0.0);
  for (int a = 0; a < n; ++a)
    for (int b = a + 1; b < n; ++b) {
      const double w = scale * g.normal();
      C[size_t(a) * n + b] = w;
      C[size_t(b) * n + a] = -w;
    }
  return C;
}

double kinetic(const composite &s, const vecd &x) {
  double T = 0;
  for (int i = 0; i < s.blocks(); ++i) {
    const auto *L = dynamic_cast<const canonical_law *>(&s.law(i));
    const int n = L->spec().n, o = s.state_offset(i);
    vecd m(n);
    L->mass(x.data() + o, m.data());
    for (int j = 0; j < n; ++j)
      T += 0.5 * x[o + n + j] * x[o + n + j] / m[j];
  }
  return T;
}

// ── attachment points ──────────────────────────────────────────────────────────────────────────

TEST(PhMech, ChainIsThePlanarArmsClosedForm) {
  const double l1 = 0.7, l2 = 0.4;
  serial_chain c;
  c.offset = {{0, 0, 0}, {l1, 0, 0}};
  c.axis = {{0, 0, 1}, {0, 0, 1}};
  c.point = {l2, 0, 0};
  rng g(3);
  for (int k = 0; k < 20; ++k) {
    const double q[2] = {uniform(g, -3, 3), uniform(g, -3, 3)};
    double r[3], J[6];
    c.eval(q, r, J);
    const double c1 = std::cos(q[0]), s1 = std::sin(q[0]);
    const double c12 = std::cos(q[0] + q[1]), s12 = std::sin(q[0] + q[1]);
    EXPECT_NEAR(r[0], l1 * c1 + l2 * c12, 1e-15);
    EXPECT_NEAR(r[1], l1 * s1 + l2 * s12, 1e-15);
    EXPECT_NEAR(r[2], 0.0, 1e-15);
    EXPECT_NEAR(J[0], -l1 * s1 - l2 * s12, 1e-15);
    EXPECT_NEAR(J[1], -l2 * s12, 1e-15);
    EXPECT_NEAR(J[2], l1 * c1 + l2 * c12, 1e-15);
    EXPECT_NEAR(J[3], l2 * c12, 1e-15);
    EXPECT_NEAR(J[4], 0.0, 1e-15);
    EXPECT_NEAR(J[5], 0.0, 1e-15);
  }
}

TEST(PhMech, Go2FeetAndTheirJacobians) {
  // At q = 0 the foot is the sum of go2.xml's offsets; the Jacobian matches central differences.
  const serial_chain fl = go2_foot(go2_leg::front_left);
  const double z[3] = {0, 0, 0};
  double r[3];
  fl.eval(z, r);
  EXPECT_NEAR(r[0], 0.1934 - 0.002, 1e-15);
  EXPECT_NEAR(r[1], 0.0465 + 0.0955, 1e-15);
  EXPECT_NEAR(r[2], -0.426, 1e-15);
  rng g(5);
  double worst = 0;
  for (go2_leg leg :
       {go2_leg::front_left, go2_leg::front_right, go2_leg::rear_left, go2_leg::rear_right}) {
    const serial_chain c = go2_foot(leg);
    for (int k = 0; k < 10; ++k) {
      double q[3] = {uniform(g, -1, 1), uniform(g, -1.5, 3.4), uniform(g, -2.7, -0.8)};
      double J[9], rp[3], rm[3];
      c.eval(q, r, J);
      for (int j = 0; j < 3; ++j) {
        const double h = 1e-6, q0 = q[j];
        q[j] = q0 + h;
        c.eval(q, rp);
        q[j] = q0 - h;
        c.eval(q, rm);
        q[j] = q0;
        for (int i = 0; i < 3; ++i)
          worst = std::max(worst, std::fabs(J[i * 3 + j] - (rp[i] - rm[i]) / (2 * h)));
      }
    }
    const vecd M = go2_leg_inertia(leg, {0.0, 0.9, -1.8});
    std::cout << "Go2 " << to_string(leg) << " inertia diagonal at home: " << M[0] << ", " << M[1]
              << ", " << M[2] << " kg m^2\n";
    for (double m : M)
      EXPECT_GT(m, 0.01);
  }
  EXPECT_LT(worst, 1e-9);
}

TEST(PhMech, AttachmentPortsAreCollocated) {
  // Eq. 49: the port's output is the point's velocity (dr/dq) qdot, and a force F at the point
  // does the work F . rdot = tau . qdot with tau = (dr/dq)^T F, so the leg's books stay closed.
  const law_ptr leg = go2_leg_law(go2_leg::front_left);
  EXPECT_EQ(leg->port_dim(), 3);
  EXPECT_EQ(leg->input_dim(), 6);
  const serial_chain foot = go2_foot(go2_leg::front_left);
  const auto *L = dynamic_cast<const canonical_law *>(leg.get());
  rng g(7);
  for (int k = 0; k < 10; ++k) {
    vecd x = {uniform(g, -1, 1), uniform(g, 0, 2), uniform(g, -2.5, -1),
              g.normal(),        g.normal(),       g.normal()};
    vecd e(6), B(18), G(36);
    leg->effort(x.data(), e.data());
    leg->port_map(x.data(), B.data());
    leg->input_map(x.data(), G.data());
    double r[3], J[9];
    foot.eval(x.data(), r, J);
    vecd m(3);
    L->mass(x.data(), m.data());
    for (int i = 0; i < 3; ++i) {
      double y = 0, rdot = 0;
      for (int a = 0; a < 6; ++a)
        y += B[a * 3 + i] * e[a];
      for (int j = 0; j < 3; ++j)
        rdot += J[i * 3 + j] * x[3 + j] / m[j];
      EXPECT_NEAR(y, rdot, 1e-14);
      for (int j = 0; j < 3; ++j) {
        EXPECT_EQ(B[j * 3 + i], 0.0);                    // no configuration rows
        EXPECT_EQ(G[j * 6 + 3 + i], 0.0);                // the foot force's columns too
        EXPECT_EQ(G[(3 + j) * 6 + 3 + i], J[i * 3 + j]); // (dr/dq)^T
      }
    }
  }
  // The books of a leg pushed at its foot close: power balance at random states.
  composite s;
  s.add_block(leg, "leg");
  EXPECT_LT(check_balance_random(s, 50, 11).worst.rate_error, 1e-12);
}

// ── Algorithm 2's step ─────────────────────────────────────────────────────────────────────────

TEST(PhMech, CayleyCouplingKeepsKineticEnergy) {
  // Each coupling half-step holds q and keeps p . M(q)^-1 p (Eq. 64 and below) for constant and
  // configuration-dependent mass alike; the explicit schemes do not.
  rng g(13);
  const int n = 6;
  canonical_spec net;
  net.n = 1;
  net.m_net.layers = {
      dense_layer{1, 4, {0.7, -1.1, 0.4, 0.9}, {0.1, 0.0, -0.2, 0.3}, activation::tanh},
      dense_layer{4, 1, {0.5, -0.3, 0.8, 0.2}, {0.4}, activation::identity}};
  net.m_min = 0.2;
  net.cosine = {9.81};
  net.ports = 1;
  net.B = {0.0, 1.0};
  for (const law_ptr &joint : {arm_joint(), law_ptr(std::make_shared<canonical_law>(net))}) {
    const composite s = arm_system(joint, n, random_skew(g, n, 1.5));
    double worst_c = 0, worst_e = 0;
    for (int k = 0; k < 20; ++k) {
      vecd x(2 * n);
      for (double &v : x)
        v = 2 * g.normal();
      const double T0 = kinetic(s, x);
      vecd xc = x, xe = x;
      coupling_step(s, coupling_scheme::cayley, 0.05, xc.data());
      coupling_step(s, coupling_scheme::euler, 0.05, xe.data());
      for (int i = 0; i < s.blocks(); ++i)
        EXPECT_EQ(xc[s.state_offset(i)], x[s.state_offset(i)]); // q held
      worst_c = std::max(worst_c, std::fabs(kinetic(s, xc) - T0) / T0);
      worst_e = std::max(worst_e, std::fabs(kinetic(s, xe) - T0) / T0);
    }
    std::cout << joint->kind() << " joints: kinetic energy change per coupling half-step, cayley "
              << sci(worst_c) << ", euler " << sci(worst_e) << "\n";
    EXPECT_LT(worst_c, 1e-13);
    EXPECT_GT(worst_e, 1e-4);
  }
}

TEST(PhMech, Algorithm2IsSecondOrder) {
  // The arm with a coupling, held torques: Algorithm 2's step (inputs outside, Cayley coupling,
  // leapfrog) against RK4 at dt / 64, over 1 s. Halving dt quarters the error; the books'
  // residual per step falls by about eight.
  rng g(17);
  const int n = 4;
  const composite s = arm_system(arm_joint(), n, random_skew(g, n, 0.8));
  vecd x0(2 * n), u(n);
  for (double &v : x0)
    v = 0.5 * g.normal();
  for (double &v : u)
    v = g.normal();
  auto run = [&](double dt, split_order order) {
    transition_opts o = arm_transition();
    o.dt = dt;
    o.order = order;
    const rollout_result r = rollout(s, o, x0, u, static_cast<int>(std::lround(1.0 / dt)));
    double worst = 0;
    for (const books &b : r.ledger)
      worst = std::max(worst, std::fabs(b.residual));
    return std::make_pair(r.x.back(), worst);
  };
  transition_opts fine;
  fine.scheme = step_scheme::rk4;
  fine.dt = 0.02 / 64;
  const vecd ref = rollout(s, fine, x0, u, 64 * 50).x.back();
  auto err = [&](const vecd &x) {
    double d = 0;
    for (size_t i = 0; i < x.size(); ++i)
      d = std::max(d, std::fabs(x[i] - ref[i]));
    return d;
  };
  for (split_order order : {split_order::inputs_outside, split_order::coupling_outside}) {
    const auto a = run(0.02, order), b = run(0.01, order);
    const double ea = err(a.first), eb = err(b.first);
    std::cout << to_string(order) << ": error at 1 s " << sci(ea) << " (dt 0.02), " << sci(eb)
              << " (dt 0.01), ratio " << ea / eb << "; worst residual " << sci(a.second) << ", "
              << sci(b.second) << "\n";
    EXPECT_GT(ea / eb, 3.3);
    EXPECT_LT(ea / eb, 4.7);
    EXPECT_GT(a.second / b.second, 5.0);
  }
}

// ── the planar arm ─────────────────────────────────────────────────────────────────────────────

TEST(PhMech, CountCouplingIsSkewAndRescales) {
  // Eq. 208: C_hat(s) is skew at every s, and the entries trained at s0 scale by sqrt(s0 / s).
  rng g(19);
  const int smax = 8;
  vecd U(size_t(smax) * smax);
  for (double &v : U)
    v = g.normal();
  const vecd C4 = count_coupling(U, smax, 4), C8 = count_coupling(U, smax, 8);
  for (int a = 0; a < 8; ++a)
    for (int b = 0; b < 8; ++b) {
      EXPECT_DOUBLE_EQ(C8[a * 8 + b], -C8[b * 8 + a]);
      if (a < 4 && b < 4)
        EXPECT_NEAR(C8[a * 8 + b], std::sqrt(4.0 / 8.0) * C4[a * 4 + b], 1e-15);
    }
  // The simulator's coupling is the chain, 1 / sqrt(n), skew.
  const vecd T = arm_true_coupling(4, {});
  EXPECT_DOUBLE_EQ(T[0 * 4 + 1], 0.5);
  EXPECT_DOUBLE_EQ(T[1 * 4 + 0], -0.5);
  EXPECT_DOUBLE_EQ(T[0 * 4 + 2], 0.0);
}

TEST(PhMech, ObserverRecoversTheStateOnTheModelsOwnData) {
  // Positions from the model itself (5 frames): the window fit recovers the last frame's state to
  // the fit's tolerance; finite differences carry the velocity's O(dt) lag.
  rng g(23);
  const int n = 4;
  const composite s = arm_system(arm_joint(), n, arm_true_coupling(n, {}));
  const transition_opts o = arm_transition();
  double worst_fit = 0, worst_fd = 0;
  for (int k = 0; k < 10; ++k) {
    vecd x(2 * n);
    for (double &v : x)
      v = 0.5 * g.normal();
    std::vector<vecd> pos, inputs;
    const std::vector<int> ch = position_channels(s);
    for (int f = 0; f < 5; ++f) {
      vecd q;
      for (int c : ch)
        q.push_back(x[c]);
      pos.push_back(q);
      if (f < 4) {
        vecd u(n);
        for (double &v : u)
          v = g.normal();
        inputs.push_back(u);
        step(s, o, u.data(), x.data());
      }
    }
    observer_options fit, fd;
    fd.kind = observer_kind::finite_difference;
    const vecd a = observe(s, o, pos, inputs, fit), b = observe(s, o, pos, inputs, fd);
    for (size_t i = 0; i < x.size(); ++i) {
      worst_fit = std::max(worst_fit, std::fabs(a[i] - x[i]));
      worst_fd = std::max(worst_fd, std::fabs(b[i] - x[i]));
    }
  }
  std::cout << "observer error on the model's own data: window fit " << sci(worst_fit)
            << ", finite differences " << sci(worst_fd) << "\n";
  EXPECT_LT(worst_fit, 1e-3); // the prior's pull and the fit's tolerance
  EXPECT_GT(worst_fd, 1e-2);
}

TEST(PhMech, ArmCountTransfer) {
  // Table 10's KNOWN column: the per-joint law is known, the coupling is fitted at N = 4 (Eq. 208)
  // and the same model is re-instantiated at N = 2 ... 8 without refitting. Takeover: the state
  // from 5 observed joint-position frames, then 50 steps under the supplied torques. The paper
  // reports 0.0017 at N = 4 and 0.0014 - 0.0029 over the counts (three seeds; 200 training and 20
  // test trajectories per count). The data here are the same simulator's, on this file's seeds.
  const arm_params p;
  const law_ptr joint = arm_joint(p);
  const transition_opts o = arm_transition(p);
  coupling_fit_options f;
  f.takeover.observer.kind = observer_kind::window_fit;
  const std::vector<arm_trajectory> train = arm_dataset(4, 60, 60, 101, p);
  const coupling_fit_result fit = fit_arm_coupling(joint, 4, train, o, f);
  std::cout << "coupling fit at N = 4: takeover MSE " << sci(fit.mse_start) << " -> "
            << sci(fit.mse) << " (" << fit.evals << " evaluations)\n";
  // The simulator's clamp recomputes qdot from the positions after its closing coupling half-step
  // (arm_true_step), so its coupling acts for half of each step: C_true / 2 is what the data
  // carry, and what the fit should find.
  const vecd C4 = count_coupling(fit.U, fit.smax, 4), T4 = arm_true_coupling(4, p);
  double coupling_err = 0;
  for (size_t i = 0; i < C4.size(); ++i)
    coupling_err = std::max(coupling_err, std::fabs(C4[i] - 0.5 * T4[i]));
  std::cout << "  fitted C_hat(4) against C_true / 2: max |difference| " << sci(coupling_err)
            << " (C_true / 2's entries are 0.25)\n  C_hat(4) above the diagonal:";
  for (int a = 0; a < 4; ++a)
    for (int b = a + 1; b < 4; ++b)
      std::cout << " (" << a << "," << b << ") " << C4[a * 4 + b];
  std::cout << "\n";
  EXPECT_LT(fit.mse, 0.05 * fit.mse_start);
  EXPECT_LT(coupling_err, 0.01);
  std::cout << "  N   no coupling   fitted (fit)   fitted (finite differences)\n";
  for (int n : {2, 3, 4, 5, 6, 8}) {
    const std::vector<arm_trajectory> test = arm_dataset(n, 20, 60, 200 + n, p);
    takeover_options t = f.takeover, t_fd = f.takeover;
    t_fd.observer.kind = observer_kind::finite_difference;
    const composite bare = arm_system(joint, n),
                    model = arm_system(joint, n, count_coupling(fit.U, fit.smax, n));
    const double m0 = takeover_mse(bare, o, test, t), m1 = takeover_mse(model, o, test, t),
                 m2 = takeover_mse(model, o, test, t_fd);
    std::cout << "  " << n << "   " << sci(m0) << "     " << sci(m1) << "      " << sci(m2) << "\n";
    EXPECT_LT(m1, 0.01);
    EXPECT_LE(m1, m0 * 1.05);
  }
}

// ── the Go2's legs and contact ─────────────────────────────────────────────────────────────────

TEST(PhMech, LegComesToRestOnCompliantGround) {
  // A Go2 front-left leg with its hip fixed (a test rig), loaded by 20 N down at the foot, falls
  // onto ground 2 cm below its foot. The contact (storage k/2 s^2, Hunt-Crossley loss) catches it,
  // the joints' damping and the contact's settle it, and the ground ends up carrying the load.
  //
  // How the step orders the load against the contact matters. With the inputs outside (Algorithm
  // 2), the load's kick and the contact's reaction fall in different substeps: inside every step
  // the kick leaves the foot a velocity, the contact's damper dissipates it, and the endpoint
  // books never see that loss, so it lands in their residual, and the leg creeps. Both shrink as
  // dt^2. With the inputs in the port subflow (inputs_with_coupling) the implicit midpoint
  // balances the load and the contact together: the leg stops and the books close.
  composite s;
  const int leg = s.add_block(go2_leg_law(go2_leg::front_left), "leg");
  const int ground = attach_contact(s, leg, 2, 2.0e4, 2.0e5, "ground");
  const vecd q0 = {0.0, 0.9, -1.8};
  vecd x(s.state_dim(), 0.0);
  for (int j = 0; j < 3; ++j)
    x[s.state_offset(leg) + j] = q0[j];
  x[s.state_offset(ground)] = -0.02; // 2 cm clear
  vecd u(s.input_dim(), 0.0);
  u[s.input_offset(leg) + 5] = -20.0; // the foot's z force
  struct outcome {
    double force, supplied, dissipated, residual, creep;
    vecd end;
  };
  auto run = [&](split_order order, double dt) {
    transition_opts o;
    o.dt = dt;
    o.substeps = 1;
    o.coupling = coupling_scheme::cayley;
    o.order = order;
    const int steps = static_cast<int>(std::lround(4.0 / dt)); // 4 s
    const rollout_result r = rollout(s, o, x, u, steps);
    outcome out{0, 0, 0, 0, 0, r.x.back()};
    for (const books &b : r.ledger) {
      out.supplied += b.supplied;
      out.dissipated += b.dissipated;
      out.residual += b.residual;
    }
    vecd e(s.state_dim());
    s.effort(out.end.data(), e.data());
    out.force = e[s.state_offset(ground)];
    // The joints' drift over the last second.
    const vecd &a = r.x[steps - static_cast<int>(std::lround(1.0 / dt))];
    for (int j = 0; j < 3; ++j)
      out.creep = std::max(out.creep, std::fabs(out.end[j] - a[j]));
    return out;
  };
  for (split_order order : {split_order::inputs_outside, split_order::inputs_with_coupling}) {
    const outcome a = run(order, 5e-4), b = run(order, 2.5e-4);
    std::cout << to_string(order) << ": after 4 s the ground carries " << a.force
              << " N (load 20 N), compression " << a.end[s.state_offset(ground)] * 1e3
              << " mm; work " << a.supplied << " J, loss " << a.dissipated << " J; books' residual "
              << sci(a.residual) << " (dt 5e-4), " << sci(b.residual)
              << " (dt 2.5e-4); joints' drift over the last second " << sci(a.creep) << ", "
              << sci(b.creep) << " rad\n";
    EXPECT_NEAR(a.force, 20.0, 0.01);
    // The energy balance overall: H_end - H_0 = work - loss + the residuals.
    EXPECT_NEAR(s.energy(a.end.data()) - s.energy(x.data()), a.supplied - a.dissipated + a.residual,
                1e-9);
    if (order == split_order::inputs_outside) {
      EXPECT_GT(std::fabs(a.residual), 0.01 * a.supplied);
      EXPECT_GT(std::fabs(a.residual / b.residual), 3.5); // second order
      EXPECT_GT(a.creep, 1e-4);
    } else {
      EXPECT_LT(std::fabs(a.residual), 2e-3 * a.supplied);
      EXPECT_LT(a.creep, 1e-9);
    }
  }
}

TEST(PhMech, FilesRoundTrip) {
  composite s;
  const int leg = s.add_block(go2_leg_law(go2_leg::rear_right), "leg");
  attach_contact(s, leg, 2, 1.5e4, 1e5, "ground");
  const composite arm = arm_system(arm_joint(), 3, arm_true_coupling(3, {}));
  for (const composite *c : {static_cast<const composite *>(&s), &arm}) {
    const composite back = parse_system(system_json(*c));
    EXPECT_EQ(describe(back), describe(*c));
    EXPECT_EQ(system_json(back), system_json(*c));
    rng g(29);
    vecd x(c->state_dim()), u(c->input_dim()), f1(x.size()), f2(x.size());
    for (double &v : x)
      v = 0.3 * g.normal();
    for (double &v : u)
      v = g.normal();
    c->field(x.data(), u.data(), f1.data());
    back.field(x.data(), u.data(), f2.data());
    for (size_t i = 0; i < x.size(); ++i)
      EXPECT_DOUBLE_EQ(f1[i], f2[i]);
  }
}

// ── MuJoCo's Go2 ───────────────────────────────────────────────────────────────────────────────

TEST(PhMech, Go2FeetMatchMujoco) {
  // The chains' feet against MuJoCo's own forward kinematics of go2.xml: the foot geom's centre
  // from the calf body's pose, in the base frame, at the home keyframe and at postures reached
  // under random joint targets.
  using namespace cvc::physics;
  if (!mujoco_robot::available())
    GTEST_SKIP() << "built without MuJoCo (CVC_WITH_MUJOCO)";
  mujoco_robot_options mo;
  mo.model = std::string(CVC_MUJOCO_MENAGERIE_DIR) + "/unitree_go2/scene.xml";
  mo.keyframe = "home";
  mo.mode = actuator_mode::position;
  mujoco_robot go2(mo);
  auto rotate = [](const quat &q, const vec3 &v) {
    // v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    const vec3 u{q.x, q.y, q.z};
    auto cross = [](const vec3 &a, const vec3 &b) {
      return vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };
    const vec3 t = 2.0 * cross(u, v);
    return v + q.w * t + cross(u, t);
  };
  auto inverse_rotate = [&](const quat &q, const vec3 &v) {
    return rotate(quat{q.w, -q.x, -q.y, -q.z}, v);
  };
  rng g(31);
  double worst = 0;
  for (int pose = 0; pose < 4; ++pose) {
    if (pose > 0) {
      for (int a = 0; a < go2.actuators(); ++a)
        go2.command(a, a % 3 == 0   ? uniform(g, -0.5, 0.5)
                       : a % 3 == 1 ? uniform(g, 0.3, 1.4)
                                    : uniform(g, -2.4, -1.0));
      go2.step(200);
    }
    const std::vector<double> qpos = go2.qpos();
    const vec3 base = go2.base_position();
    const quat R = go2.base_rotation();
    for (go2_leg leg :
         {go2_leg::front_left, go2_leg::front_right, go2_leg::rear_left, go2_leg::rear_right}) {
      const std::string p = to_string(leg);
      double q[3];
      for (int j = 0; j < 3; ++j)
        q[j] = qpos[go2.actuator_qpos(go2.actuator(p + (j == 0   ? "_hip"
                                                        : j == 1 ? "_thigh"
                                                                 : "_calf")))];
      double r[3];
      go2_foot(leg).eval(q, r);
      const int calf = go2.body(p + "_calf");
      const vec3 foot_world =
          go2.body_position(calf) + rotate(go2.body_rotation(calf), vec3{-0.002, 0.0, -0.213});
      const vec3 foot_base = inverse_rotate(R, foot_world - base);
      worst = std::max({worst, std::fabs(foot_base.x - r[0]), std::fabs(foot_base.y - r[1]),
                        std::fabs(foot_base.z - r[2])});
    }
  }
  std::cout << "Go2 feet, chain against MuJoCo: max |difference| " << sci(worst) << " m\n";
  EXPECT_LT(worst, 1e-9);
}

} // namespace
