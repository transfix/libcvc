/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph microgrids (inc/cvc/ph/microgrid.h) and the composition pieces they need: the capacitor
// curve, the physical-to-storage map, assigning a candidate's start, and the file writer.
//
// The candidate study is the electrical studies' network with their reference DGU laws:
//   - a four-DGU chain (lines 0.3 / 0.4 / 0.5 ohm, 0.8 H), run for one second from a sampled
//     physical state;
//   - then the same DGU law and a 0.4 ohm / 0.8 H line added at bus 0 or at bus 2;
//   - 200 steps of 0.01 s under held inputs.
// Here the laws are known, so the errors are the transition's own: the split step with Euler or
// midpoint coupling, against RK4 at dt / 16 (dt / 64 gives the same scores to six digits). The scores are voltage, DGU-current and line-current
// RMSE, the peak line-current error, and whether each scheme orders the two candidates by peak
// line current as the reference does.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <cvc/ph/microgrid.h>
#include <cvc/ph/rng.h>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
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

double max_diff(const vecd &a, const vecd &b) {
  double d = 0;
  for (size_t i = 0; i < a.size(); ++i)
    d = std::max(d, std::fabs(a[i] - b[i]));
  return d;
}

const std::vector<line_spec> kChain = {{0, 1, 0.3, 0.8}, {1, 2, 0.4, 0.8}, {2, 3, 0.5, 0.8}};

// ── the capacitor curve ────────────────────────────────────────────────────────────────────────

TEST(PhMicrogrid, TheReferenceCapacitorCurveInverts) {
  const law_ptr dgu = reference_dgu();
  const auto &c = dynamic_cast<const canonical_law &>(*dgu);
  for (double v = -1.5; v <= 1.5; v += 0.125) {
    const double q = 2.2 * (v + v * v * v / 3.0);
    EXPECT_NEAR(c.capacitor_voltage(q), v, 1e-14 * (1 + std::fabs(v)));
    const double x[2] = {q, 1.8 * 0.2};
    double e[2];
    dgu->effort(x, e);
    EXPECT_NEAR(e[0], v, 1e-14 * (1 + std::fabs(v)));
    EXPECT_NEAR(e[1], 0.2, 1e-15);
    // H = 2.2 (v^2 / 2 + v^4 / 4) + phi^2 / (2 L).
    const double H = 2.2 * (v * v / 2 + v * v * v * v / 4) + x[1] * x[1] / (2 * 1.8);
    EXPECT_NEAR(dgu->energy(x), H, 1e-14 * (1 + H));
  }
}

TEST(PhMicrogrid, ALinearCurveIsTheQuadraticCapacitor) {
  const law_ptr a = linear_dgu(2.2, 1.8, 1.2, conservative_scheme::implicit_midpoint);
  const law_ptr b = nonlinear_dgu({0.0, 2.2}, 1.8, 1.2);
  rng g(3);
  for (int k = 0; k < 20; ++k) {
    const double x[2] = {uniform(g, -3, 3), uniform(g, -1, 1)};
    double ea[2], eb[2];
    a->effort(x, ea);
    b->effort(x, eb);
    EXPECT_NEAR(a->energy(x), b->energy(x), 1e-14);
    EXPECT_NEAR(ea[0], eb[0], 1e-14);
    EXPECT_EQ(ea[1], eb[1]);
  }
}

TEST(PhMicrogrid, ANonlinearDguBalancesItsPower) {
  const microgrid m = make_microgrid(reference_dgu(), 4, kChain);
  const balance_summary b = check_balance_random(m.system, 200, 9, 1.0);
  EXPECT_LT(b.worst.rate_error, 1e-12);
  EXPECT_LT(b.worst.internal_error, 1e-15);
  EXPECT_EQ(b.worst.skew_error, 0.0);
}

// ── physical states ────────────────────────────────────────────────────────────────────────────

vecd sample_physical(rng &g, int buses, int lines) {
  // Bus voltages 0.5-1.3 V, DGU currents -0.25-0.35 A, line currents -0.1-0.1 A.
  vecd e;
  for (int i = 0; i < buses; ++i) {
    e.push_back(uniform(g, 0.5, 1.3));
    e.push_back(uniform(g, -0.25, 0.35));
  }
  for (int k = 0; k < lines; ++k)
    e.push_back(uniform(g, -0.1, 0.1));
  return e;
}

vecd sample_inputs(rng &g, int buses) {
  // Per DGU: minus a 0.05-0.2 A load, and a 0.9-1.2 V source.
  vecd u;
  for (int i = 0; i < buses; ++i) {
    u.push_back(-uniform(g, 0.05, 0.2));
    u.push_back(uniform(g, 0.9, 1.2));
  }
  return u;
}

TEST(PhMicrogrid, StorageInvertsThePhysicalState) {
  for (const law_ptr &dgu : {reference_dgu(), linear_dgu(2.2, 1.8, 1.2)}) {
    const microgrid m = make_microgrid(dgu, 4, kChain);
    rng g(11);
    for (int k = 0; k < 20; ++k) {
      const vecd e = sample_physical(g, 4, 3);
      const storage_result s = storage_from_efforts(m.system, e);
      ASSERT_TRUE(s.converged) << "sample " << k;
      EXPECT_LT(s.residual, 1e-9);
      EXPECT_LE(s.iterations, 30);
      EXPECT_LT(max_diff(physical_state(m, s.x), e), 1e-9);
    }
  }
}

TEST(PhMicrogrid, StorageSaysWhenItStopsShort) {
  const microgrid m = make_microgrid(reference_dgu(), 2, {{0, 1, 0.3, 0.8}});
  rng g(5);
  const vecd e = sample_physical(g, 2, 1);
  const storage_result s = storage_from_efforts(m.system, e, {}, 0);
  EXPECT_FALSE(s.converged);
  EXPECT_GT(s.residual, 1e-3);
}

// ── assigning a candidate's start ──────────────────────────────────────────────────────────────

TEST(PhMicrogrid, AssignStateKeepsExistingBlocksAndAddsSuppliedOnes) {
  const law_ptr dgu = reference_dgu();
  const microgrid now = make_microgrid(dgu, 4, kChain);
  std::vector<line_spec> lines = kChain;
  lines.push_back({2, 4, 0.4, 0.8});
  const microgrid cand = make_microgrid(dgu, 5, lines);
  rng g(2);
  const vecd x_now = storage_from_efforts(now.system, sample_physical(g, 4, 3)).x;
  const vecd x =
      assign_state(now.system, x_now, cand.system, {{"dgu4", {2.2, 0.18}}, {"line3", {0.0}}});
  ASSERT_EQ(x.size(), size_t(cand.system.state_dim()));
  for (int i = 0; i < now.system.blocks(); ++i) {
    const int j = cand.system.find(now.system.name(i));
    ASSERT_GE(j, 0);
    for (int k = 0; k < now.system.law(i).state_dim(); ++k)
      EXPECT_EQ(x[cand.system.state_offset(j) + k], x_now[now.system.state_offset(i) + k]);
  }
  EXPECT_EQ(x[cand.system.state_offset(cand.system.find("dgu4"))], 2.2);
  EXPECT_EQ(x[cand.system.state_offset(cand.system.find("line3"))], 0.0);
  EXPECT_THROW(assign_state(now.system, x_now, cand.system, {{"dgu4", {2.2, 0.18}}}),
               std::invalid_argument);
  EXPECT_THROW(assign_state(now.system, x_now, cand.system, {{"dgu4", {2.2}}, {"line3", {0.0}}}),
               std::invalid_argument);
}

// ── the file writer ────────────────────────────────────────────────────────────────────────────

TEST(PhMicrogrid, ASystemWrittenOutReadsBackTheSame) {
  // The nonlinear network, plus a learned-style canonical block (nets for V, M and D) on a dense
  // skew rule, so every field of both law kinds is written.
  microgrid m = make_microgrid(reference_dgu(), 3, {{0, 1, 0.3, 0.8}, {2, 1, 0.5, 0.9}});
  canonical_spec c;
  c.n = 1;
  c.K = {0.5};
  c.center = {0.1};
  c.v_scale = 0.3;
  auto net = [](int out, activation act) {
    mlp n;
    dense_layer a;
    a.in = 1;
    a.out = 3;
    a.W = {0.4, -0.7, 1.1};
    a.b = {0.1, 0.0, -0.2};
    a.act = act;
    dense_layer b;
    b.in = 3;
    b.out = out;
    b.W = {0.3, 0.5, -0.6};
    b.b = {0.05};
    n.layers = {a, b};
    return n;
  };
  c.v_net = net(1, activation::silu);
  c.m_net = net(1, activation::tanh);
  c.m_min = 0.2;
  c.d_net = net(1, activation::sigmoid);
  c.d_min = 0.05;
  c.ports = 1;
  c.B = {0.0, 1.0};
  const law_ptr extra = std::make_shared<canonical_law>(c);
  const int a = m.system.add_block(extra, "arm"), b = m.system.add_block(extra);
  m.system.connect_skew({a, b}, 0, {0.0, 0.7, -0.2, 0.0});
  const composite back = parse_system(system_json(m.system));
  ASSERT_EQ(back.state_dim(), m.system.state_dim());
  ASSERT_EQ(back.blocks(), m.system.blocks());
  EXPECT_EQ(back.pairing(), m.system.pairing());
  EXPECT_EQ(back.name(b), "b" + std::to_string(b));
  rng g(17);
  for (int k = 0; k < 10; ++k) {
    vecd x(back.state_dim()), u(back.input_dim()), fa(x.size()), fb(x.size());
    for (double &v : x)
      v = g.normal();
    for (double &v : u)
      v = g.normal();
    EXPECT_EQ(back.energy(x.data()), m.system.energy(x.data()));
    m.system.field(x.data(), u.data(), fa.data());
    back.field(x.data(), u.data(), fb.data());
    EXPECT_EQ(fa, fb);
  }
  // And it writes the same text again.
  EXPECT_EQ(system_json(back), system_json(parse_system(system_json(back))));
}

// ── the candidate study ────────────────────────────────────────────────────────────────────────

struct scheme_score {
  double voltage = 0, dgu_current = 0, line_current = 0, peak = 0, residual = 0;
  int correct = 0, resolved = 0, trajectories = 0;
};

TEST(PhMicrogrid, TheCandidateStudyFollowsItsReference) {
  const law_ptr dgu = reference_dgu();
  const microgrid now = make_microgrid(dgu, 4, kChain);
  std::vector<microgrid> cands;
  for (int bus : {0, 2}) {
    std::vector<line_spec> lines = kChain;
    lines.push_back({bus, 4, 0.4, 0.8});
    cands.push_back(make_microgrid(dgu, 5, lines));
  }
  const microgrid lone = make_microgrid(dgu, 1, {}); // the added DGU on its own, for its storage
  transition_opts ref;
  ref.scheme = step_scheme::rk4;
  ref.dt = 0.01 / 16;
  transition_opts split[2];
  split[0].coupling = coupling_scheme::euler;
  split[1].coupling = coupling_scheme::midpoint;
  scheme_score score[2];
  const int episodes = 16;
  for (int ep = 0; ep < episodes; ++ep) {
    rng g(1000 + ep);
    const storage_result start = storage_from_efforts(now.system, sample_physical(g, 4, 3));
    ASSERT_TRUE(start.converged);
    const vecd u_now = sample_inputs(g, 4);
    // One second of the actual system, by the reference.
    const vecd x_now = rollout(now.system, ref, start.x, u_now, 100 * 16).x.back();
    // The supplied states of the added DGU and line, and the added DGU's inputs.
    const vecd added_e = {uniform(g, 0.5, 1.3), uniform(g, -0.25, 0.35)};
    const storage_result added = storage_from_efforts(lone.system, added_e);
    ASSERT_TRUE(added.converged);
    const vecd line_x = {0.8 * uniform(g, -0.1, 0.1)};
    vecd u = u_now;
    const vecd u_add = sample_inputs(g, 1);
    u.insert(u.end(), u_add.begin(), u_add.end());
    double peaks_ref[2], peaks[2][2];
    for (int c = 0; c < 2; ++c) {
      const microgrid &m = cands[c];
      const vecd x0 =
          assign_state(now.system, x_now, m.system, {{"dgu4", added.x}, {"line3", line_x}});
      const rollout_result fine = rollout(m.system, ref, x0, u, 200 * 16);
      std::vector<vecd> truth;
      for (size_t k = 0; k < fine.x.size(); k += 16)
        truth.push_back(fine.x[k]);
      peaks_ref[c] = peak_line_current(m, truth);
      for (int s = 0; s < 2; ++s) {
        const rollout_result r = rollout(m.system, split[s], x0, u, 200);
        for (const vecd &x : r.x)
          for (double v : x)
            ASSERT_TRUE(std::isfinite(v));
        const grid_errors e = compare_trajectories(m, r.x, truth);
        score[s].voltage += e.voltage_rmse;
        score[s].dgu_current += e.dgu_current_rmse;
        score[s].line_current += e.line_current_rmse;
        score[s].peak += e.peak_line_current_error;
        for (const books &b : r.ledger)
          score[s].residual = std::max(score[s].residual, std::fabs(b.residual));
        ++score[s].trajectories;
        peaks[s][c] = e.peak_line_current_a;
      }
    }
    // Ordering: which candidate carries the larger peak line current.
    if (std::fabs(peaks_ref[0] - peaks_ref[1]) > 2e-6)
      for (int s = 0; s < 2; ++s) {
        ++score[s].resolved;
        if ((peaks[s][0] > peaks[s][1]) == (peaks_ref[0] > peaks_ref[1]))
          ++score[s].correct;
      }
  }
  const char *names[2] = {"euler", "midpoint"};
  for (int s = 0; s < 2; ++s) {
    const double n = score[s].trajectories;
    std::cout << "PhMicrogrid study, " << names[s] << " coupling, " << score[s].trajectories
              << " trajectories: voltage RMSE " << 1e3 * score[s].voltage / n << " mV, DGU current "
              << 1e3 * score[s].dgu_current / n << " mA, line current "
              << 1e3 * score[s].line_current / n << " mA, peak error " << 1e3 * score[s].peak / n
              << " mA, orderings " << score[s].correct << "/" << score[s].resolved
              << ", worst step residual " << sci(score[s].residual) << " J\n";
    RecordProperty(std::string(names[s]) + "_voltage_rmse_mV", sci(1e3 * score[s].voltage / n));
    RecordProperty(std::string(names[s]) + "_dgu_current_rmse_mA",
                   sci(1e3 * score[s].dgu_current / n));
    RecordProperty(std::string(names[s]) + "_line_current_rmse_mA",
                   sci(1e3 * score[s].line_current / n));
    RecordProperty(std::string(names[s]) + "_peak_error_mA", sci(1e3 * score[s].peak / n));
    EXPECT_EQ(score[s].trajectories, 2 * episodes);
  }
  // Midpoint coupling beats Euler on every group, and orders every resolved pair as the
  // reference does.
  EXPECT_LT(score[1].voltage, score[0].voltage);
  EXPECT_LT(score[1].dgu_current, score[0].dgu_current);
  EXPECT_LT(score[1].line_current, score[0].line_current);
  EXPECT_LT(score[1].peak, score[0].peak);
  EXPECT_GT(score[1].resolved, episodes / 2);
  EXPECT_EQ(score[1].correct, score[1].resolved);
  // Measured on Linux (GCC 13): midpoint 5.1e-7 V, 7.1e-7 A, 2.9e-7 A, 5.7e-8 A; Euler 7.9e-5 V,
  // 2.0e-5 A, 2.0e-4 A, 2.4e-4 A.
  const double n = score[1].trajectories;
  EXPECT_LT(score[1].voltage / n, 5e-6);
  EXPECT_LT(score[1].dgu_current / n, 5e-6);
  EXPECT_LT(score[1].line_current / n, 5e-6);
  EXPECT_LT(score[1].peak / n, 1e-6);
  EXPECT_LT(score[0].voltage / n, 1e-3);
  EXPECT_LT(score[0].line_current / n, 2e-3);
}

} // namespace
