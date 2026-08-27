/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// nav_material_test.cpp — the material-aware kernels (cvc/nav/material.h).
//
// The BIT surfaces (material_build, witness_gate) are pinned against golden
// values generated from the Python normative reference
// (GRL-SNAM grl_snam/material.py); the full randomized cross-language sweep
// lives in GRL-SNAM's tests/test_material_parity.py. Here: goldens, the gate
// truth table, batch==serial byte-identity across thread counts, the
// null-material rollout byte-identity guarantee, and sim_world integration.

#include <cmath>
#include <cstring>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using namespace cvc::nav;

namespace {

std::uint32_t f32bits(float v) {
  std::uint32_t u;
  std::memcpy(&u, &v, 4);
  return u;
}

// The 8x8 material_build golden case, generated from the Python reference
// (MaterialGrid on bounds ±10, scale 0.1, sigma 1.0; see the GRL-SNAM PR).
const float kRiskRaw[64] = {
    0.51f, 0.57f, 0.51f, 0.97f, 0.61f, 0.57f, 0.29f, 0.55f, 0.47f, 0.61f, 0.93f, 0.25f, 0.31f,
    0.39f, 0.27f, 0.35f, 0.94f, 0.38f, 0.77f, 0.04f, 0.30f, 0.70f, 0.45f, 0.89f, 0.44f, 0.62f,
    0.59f, 0.60f, 0.66f, 0.51f, 0.19f, 0.07f, 0.25f, 0.10f, 0.11f, 0.28f, 0.59f, 0.34f, 0.78f,
    0.97f, 0.33f, 0.24f, 0.69f, 0.57f, 0.77f, 0.62f, 0.28f, 0.91f, 0.57f, 0.72f, 0.67f, 0.77f,
    0.75f, 0.41f, 0.04f, 0.48f, 0.90f, 0.14f, 0.78f, 0.66f, 0.71f, 0.22f, 0.08f, 0.27f};

const std::uint32_t kGoldRisk[64] = {
    0x3f0b9c08u, 0x3f151fadu, 0x3f208a8cu, 0x3f1f18c3u, 0x3f0d910bu, 0x3ef1d053u, 0x3eda15b5u,
    0x3ee42decu, 0x3f15ecfdu, 0x3f196389u, 0x3f178b2eu, 0x3f03cbb6u, 0x3eeb6309u, 0x3ee38c59u,
    0x3edf16ecu, 0x3eedf0e5u, 0x3f191ff6u, 0x3f13db79u, 0x3f09ab7au, 0x3ee8bda7u, 0x3edfc94fu,
    0x3eeb0c27u, 0x3eec034au, 0x3ef9ad87u, 0x3ef7e68au, 0x3ef1d539u, 0x3eef3ca0u, 0x3eed8d3au,
    0x3ef8c367u, 0x3efa634du, 0x3ef6a1aeu, 0x3f015954u, 0x3eb7fc46u, 0x3ebcfff0u, 0x3ed520b5u,
    0x3ef81029u, 0x3f07d108u, 0x3f053057u, 0x3f0a8686u, 0x3f1d25a1u, 0x3ecb968fu, 0x3ed876b6u,
    0x3efead2eu, 0x3f1244e9u, 0x3f15548au, 0x3f03cd6fu, 0x3f0164f2u, 0x3f19b3dcu, 0x3f0a2e70u,
    0x3f09e49fu, 0x3f1a3f05u, 0x3f26e63au, 0x3f195999u, 0x3ee3c58cu, 0x3eb985dcu, 0x3ed7f847u,
    0x3f219bc2u, 0x3f12eda5u, 0x3f1f707du, 0x3f2a3bdfu, 0x3f134504u, 0x3ec1344au, 0x3e847c01u,
    0x3e919c85u};

const std::uint32_t kGoldPhi[64] = {
    0x41762dd0u, 0x414c70c5u, 0x4124d340u, 0x41014caeu, 0x40cc70c5u, 0x40b6db6eu, 0x40cc70c5u,
    0x41014caeu, 0x41691919u, 0x413c7c1eu, 0x41108fafu, 0x40cc70c5u, 0x40814caeu, 0x4036db6eu,
    0x40814caeu, 0x40cc70c5u, 0x413c7c1eu, 0x4136db6eu, 0x41092492u, 0x40b6db6eu, 0x4036db6eu,
    0x00000000u, 0x4036db6eu, 0x40b6db6eu, 0x41108fafu, 0x41092492u, 0x41108fafu, 0x40cc70c5u,
    0x40814caeu, 0x4036db6eu, 0x40814caeu, 0x40cc70c5u, 0x40cc70c5u, 0x40b6db6eu, 0x40cc70c5u,
    0x41014caeu, 0x40cc70c5u, 0x40b6db6eu, 0x40cc70c5u, 0x41014caeu, 0x40814caeu, 0x4036db6eu,
    0x40814caeu, 0x40cc70c5u, 0x41108fafu, 0x41092492u, 0x41108fafu, 0x4124d340u, 0x4036db6eu,
    0x00000000u, 0x4036db6eu, 0x40b6db6eu, 0x41092492u, 0x4136db6eu, 0x413c7c1eu, 0x414c70c5u,
    0x40814caeu, 0x4036db6eu, 0x40814caeu, 0x40cc70c5u, 0x41108fafu, 0x413c7c1eu, 0x41691919u,
    0x41762dd0u};

material_planes golden_build() {
  std::uint8_t hard[64] = {0};
  hard[2 * 8 + 5] = 1;
  hard[6 * 8 + 1] = 1;
  return material_build(kRiskRaw, hard, 8, 8, 20.0 / 7.0, 0.1, 1.0);
}

// An open grid with uniform risk/clearance for gate truth-table cases.
struct gate_world {
  int rows = 40, cols = 40;
  std::vector<float> risk, clear;
  std::vector<std::uint8_t> hard;
  gate_world(float r = 0.0f, float c = 10.0f)
      : risk(rows * cols, r), clear(rows * cols, c), hard(rows * cols, 0) {}
  gate_decision run(double pr, double pc, double gr, double gc, const gate_params &p) const {
    return witness_gate(risk.data(), hard.data(), clear.data(), rows, cols, pr, pc, gr, gc, p);
  }
};

} // namespace

TEST(NavMaterial, BuildMatchesPythonGoldensBitwise) {
  const material_planes mp = golden_build();
  for (int i = 0; i < 64; ++i) {
    EXPECT_EQ(f32bits(mp.risk[i]), kGoldRisk[i]) << "risk " << i;
    EXPECT_EQ(f32bits(mp.phi_m[i]), kGoldPhi[i]) << "phi " << i;
  }
}

TEST(NavMaterial, BuildPhiIsMetresEdt) {
  // hard at (2,5): 3-4-5 triangle to (5,1) -> 5 cells * cell_w... use simple
  // axis distances instead: (2,2) is 3 columns from (2,5).
  const material_planes mp = golden_build();
  const double cell_w = 20.0 / 7.0;
  EXPECT_EQ(mp.phi_m[2 * 8 + 5], 0.0f);
  EXPECT_FLOAT_EQ(mp.phi_m[2 * 8 + 2], static_cast<float>(3.0 * cell_w));
}

TEST(NavMaterial, BuildConstantRiskSurvivesBlur) {
  std::vector<float> risk(64, 0.7f);
  std::uint8_t hard[64] = {0};
  const material_planes mp = material_build(risk.data(), hard, 8, 8, 1.0, 1.0, 1.0);
  for (int i = 0; i < 64; ++i)
    EXPECT_NEAR(mp.risk[i], 0.7f, 2e-7f);
}

TEST(NavMaterial, BuildRejectsTinyGrids) {
  std::vector<float> risk(3 * 40, 0.0f);
  std::vector<std::uint8_t> hard(3 * 40, 0);
  EXPECT_THROW(material_build(risk.data(), hard.data(), 3, 40, 1.0, 1.0, 1.0),
               std::invalid_argument);
}

TEST(NavMaterialGate, ActivatesOnCheaperLateralCorridor) {
  gate_world w(0.05f);
  for (int r = 15; r < 26; ++r)
    for (int c = 22; c < 40; ++c)
      w.risk[r * w.cols + c] = 0.9f; // band across the nominal ray
  gate_params p;
  const gate_decision g = w.run(20.0, 20.0, 20.0, 35.0, p);
  EXPECT_TRUE(g.active);
  EXPECT_GT(g.feasible_count, 0);
  EXPECT_GE(g.nominal_risk, p.material_trigger);
  EXPECT_GE(g.nominal_risk - g.best_risk, p.improvement_margin);
}

TEST(NavMaterialGate, UniformRiskAndLowTriggerStayOff) {
  gate_world hi(0.9f);
  gate_params p;
  const gate_decision g1 = hi.run(20.0, 20.0, 20.0, 35.0, p);
  EXPECT_FALSE(g1.active);
  EXPECT_GT(g1.feasible_count, 0);
  gate_world lo(0.1f);
  const gate_decision g2 = lo.run(20.0, 20.0, 20.0, 35.0, p);
  EXPECT_FALSE(g2.active);
  EXPECT_LT(g2.nominal_risk, p.material_trigger);
}

TEST(NavMaterialGate, HardAndClearanceKillFeasibility) {
  gate_world w(0.9f);
  std::fill(w.clear.begin(), w.clear.end(), 0.5f); // under the 1.0 m margin
  gate_params p;
  const gate_decision g = w.run(20.0, 20.0, 20.0, 35.0, p);
  EXPECT_EQ(g.feasible_count, 0);
  EXPECT_FALSE(g.active);
  EXPECT_TRUE(std::isinf(g.best_risk));
}

TEST(NavMaterialGate, ProgressFilterAndZeroGoalDistance) {
  gate_world w(0.9f);
  gate_params p;
  EXPECT_EQ(w.run(20.0, 20.0, 20.0, 21.0, p).feasible_count, 0); // goal 1 cell away
  const gate_decision g = w.run(20.0, 20.0, 20.0, 20.0, p);      // degenerate
  EXPECT_FALSE(g.active);
  EXPECT_EQ(g.feasible_count, 0);
}

TEST(NavMaterialGate, HalfEvenRoundingOnTheExactAxisRay) {
  // Direction (0,1) is exact in the table; samples from col 4.5 land on 5.5,
  // 6.5, 7.5, 8.5 -> round-half-even cells 6, 6, 8, 8.
  gate_world w(0.0f);
  w.risk[10 * w.cols + 6] = 0.6f;
  w.risk[10 * w.cols + 8] = 0.2f;
  gate_params p;
  p.horizon_cells = 4;
  // walk the nominal ray directly via a goal straight down the +col axis.
  // The plane stores float32, so the expected mean is built from the f32
  // constants (0.6f != 0.6): cells 6, 6, 8, 8 — half-even, never 5/7/9.
  const gate_decision g = w.run(10.0, 4.5, 10.0, 30.5, p);
  const double expect = (static_cast<double>(0.6f) + static_cast<double>(0.6f) +
                         static_cast<double>(0.2f) + static_cast<double>(0.2f)) /
                        4.0;
  EXPECT_EQ(g.nominal_risk, expect);
}

TEST(NavMaterialGate, BatchMatchesSerialBytesAcrossThreadCounts) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<float> ur(0.0f, 1.0f);
  const int rows = 33, cols = 29, N = 23;
  std::vector<float> risk(rows * cols), clear(rows * cols);
  std::vector<std::uint8_t> hard(rows * cols);
  for (auto &v : risk)
    v = ur(rng);
  for (auto &v : clear)
    v = 4.0f * ur(rng);
  for (auto &v : hard)
    v = ur(rng) < 0.05f ? 1 : 0;
  std::vector<double> pos(2 * N), goal(2 * N);
  for (int i = 0; i < N; ++i) {
    pos[2 * i] = (rows - 1) * ur(rng);
    pos[2 * i + 1] = (cols - 1) * ur(rng);
    goal[2 * i] = (rows - 1) * ur(rng);
    goal[2 * i + 1] = (cols - 1) * ur(rng);
  }
  gate_params p;
  for (int threads : {1, 4, 8}) {
    std::vector<std::uint8_t> act(N);
    std::vector<double> nom(N), best(N);
    std::vector<std::int32_t> cnt(N);
    witness_gate_batch(risk.data(), hard.data(), clear.data(), rows, cols, pos.data(), goal.data(),
                       N, p, act.data(), nom.data(), best.data(), cnt.data(), threads);
    for (int i = 0; i < N; ++i) {
      const gate_decision g =
          witness_gate(risk.data(), hard.data(), clear.data(), rows, cols, pos[2 * i],
                       pos[2 * i + 1], goal[2 * i], goal[2 * i + 1], p);
      EXPECT_EQ(act[i] != 0, g.active) << "agent " << i << " threads " << threads;
      EXPECT_EQ(std::memcmp(&nom[i], &g.nominal_risk, 8), 0);
      EXPECT_EQ(std::memcmp(&best[i], &g.best_risk, 8), 0);
      EXPECT_EQ(cnt[i], g.feasible_count);
    }
  }
}

namespace {

// A tiny field + agent set for the rollout tests.
struct rollout_world {
  int H = 16, W = 16, N = 24;
  std::vector<float> field;
  field_stack fs;
  veh_params v;
  std::vector<float> o, th, sp, goal, al, be, ga;
  rollout_world() {
    field.assign(3 * H * W, 0.0f);
    for (int r = 0; r < H; ++r)
      for (int c = 0; c < W; ++c) {
        field[r * W + c] = 0.05f * static_cast<float>((r * 7 + c * 3) % 11) + 0.1f;
        field[H * W + r * W + c] = 0.6f;
        field[2 * H * W + r * W + c] = 0.8f;
      }
    fs.data = field.data();
    fs.M = 1;
    fs.H = H;
    fs.W = W;
    fs.mnx = -10;
    fs.mny = -10;
    fs.mxx = 10;
    fs.mxy = 10;
    fs.cx = 0;
    fs.cy = 0;
    fs.S = 0.1;
    v.rr = 0.15f;
    v.d_hat = 0.35f;
    v.dt = 0.06f;
    v.nsub = 2;
    std::mt19937 rng(5);
    std::uniform_real_distribution<float> u(-0.8f, 0.8f);
    o.resize(2 * N);
    goal.resize(2 * N);
    th.resize(N);
    sp.resize(N);
    al.assign(N, 1.0f);
    be.assign(N, 3.0f);
    ga.assign(N, 4.0f);
    for (int i = 0; i < N; ++i) {
      o[2 * i] = u(rng);
      o[2 * i + 1] = u(rng);
      goal[2 * i] = u(rng);
      goal[2 * i + 1] = u(rng);
      th[i] = u(rng);
      sp[i] = 0.3f + 0.2f * u(rng);
    }
  }
};

} // namespace

TEST(NavMaterialRollout, NullMaterialIsByteIdentical) {
  rollout_world w;
  std::vector<float> o1 = w.o, th1 = w.th, sp1 = w.sp, mc1(w.N);
  std::vector<float> o2 = w.o, th2 = w.th, sp2 = w.sp, mc2(w.N);
  bicycle_rollout(w.fs, o1.data(), th1.data(), sp1.data(), w.goal.data(), w.al.data(), w.be.data(),
                  w.ga.data(), w.N, nullptr, w.v, mc1.data(), 1);
  material_drive md; // stack == nullptr
  bicycle_rollout_material(w.fs, o2.data(), th2.data(), sp2.data(), w.goal.data(), w.al.data(),
                           w.be.data(), w.ga.data(), w.N, nullptr, w.v, md, mc2.data(), 1);
  EXPECT_EQ(std::memcmp(o1.data(), o2.data(), o1.size() * 4), 0);
  EXPECT_EQ(std::memcmp(th1.data(), th2.data(), w.N * 4), 0);
  EXPECT_EQ(std::memcmp(sp1.data(), sp2.data(), w.N * 4), 0);
  EXPECT_EQ(std::memcmp(mc1.data(), mc2.data(), w.N * 4), 0);
}

TEST(NavMaterialRollout, MaterialForcesChangeTheTrajectory) {
  rollout_world w;
  // one material plane: constant risk gradient pushing -x, no hazard nearby
  const int hw = w.H * w.W;
  std::vector<float> mstack(6 * hw, 0.0f);
  for (int i = 0; i < hw; ++i) {
    mstack[0 * hw + i] = 0.5f;   // risk
    mstack[1 * hw + i] = 100.0f; // phi_m far -> hazard force underflows to 0
    mstack[2 * hw + i] = 2.0f;   // dr/dx
  }
  material_stack ms;
  ms.data = mstack.data();
  ms.M = 1;
  ms.H = w.H;
  ms.W = w.W;
  ms.mnx = -10;
  ms.mny = -10;
  ms.mxx = 10;
  ms.mxy = 10;
  ms.cx = 0;
  ms.cy = 0;
  ms.S = 0.1;
  std::vector<float> lam_s(w.N, 0.5f), lam_h(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = lam_s.data();
  md.lam_hard = lam_h.data();
  std::vector<float> o1 = w.o, th1 = w.th, sp1 = w.sp, mc1(w.N);
  std::vector<float> o2 = w.o, th2 = w.th, sp2 = w.sp, mc2(w.N);
  bicycle_rollout(w.fs, o1.data(), th1.data(), sp1.data(), w.goal.data(), w.al.data(), w.be.data(),
                  w.ga.data(), w.N, nullptr, w.v, mc1.data(), 1);
  bicycle_rollout_material(w.fs, o2.data(), th2.data(), sp2.data(), w.goal.data(), w.al.data(),
                           w.be.data(), w.ga.data(), w.N, nullptr, w.v, md, mc2.data(), 1);
  int diff = 0;
  for (int i = 0; i < 2 * w.N; ++i)
    if (o1[i] != o2[i])
      ++diff;
  EXPECT_GT(diff, 0) << "material force had no effect";
  // and with zero lambdas + zero gradients the effect vanishes to exact zero
  std::fill(lam_s.begin(), lam_s.end(), 0.0f);
  std::fill(lam_h.begin(), lam_h.end(), 0.0f);
  std::vector<float> o3 = w.o, th3 = w.th, sp3 = w.sp, mc3(w.N);
  bicycle_rollout_material(w.fs, o3.data(), th3.data(), sp3.data(), w.goal.data(), w.al.data(),
                           w.be.data(), w.ga.data(), w.N, nullptr, w.v, md, mc3.data(), 1);
  EXPECT_EQ(std::memcmp(o1.data(), o3.data(), o1.size() * 4), 0);
}

TEST(NavMaterialRollout, ThreadCountDeterminism) {
  rollout_world w;
  const int hw = w.H * w.W;
  std::vector<float> mstack(6 * hw, 0.0f);
  for (int i = 0; i < hw; ++i) {
    mstack[0 * hw + i] = 0.4f;
    mstack[1 * hw + i] = 5.0f;
    mstack[2 * hw + i] = 1.0f;
    mstack[4 * hw + i] = 0.7f;
  }
  material_stack ms;
  ms.data = mstack.data();
  ms.M = 1;
  ms.H = w.H;
  ms.W = w.W;
  ms.mnx = -10;
  ms.mny = -10;
  ms.mxx = 10;
  ms.mxy = 10;
  ms.cx = 0;
  ms.cy = 0;
  ms.S = 0.1;
  std::vector<float> lam_s(w.N, 0.5f), lam_h(w.N, 1.0f);
  material_drive md;
  md.stack = &ms;
  md.lam_soft = lam_s.data();
  md.lam_hard = lam_h.data();
  std::vector<float> o1 = w.o, th1 = w.th, sp1 = w.sp, mc1(w.N);
  std::vector<float> o8 = w.o, th8 = w.th, sp8 = w.sp, mc8(w.N);
  bicycle_rollout_material(w.fs, o1.data(), th1.data(), sp1.data(), w.goal.data(), w.al.data(),
                           w.be.data(), w.ga.data(), w.N, nullptr, w.v, md, mc1.data(), 1);
  bicycle_rollout_material(w.fs, o8.data(), th8.data(), sp8.data(), w.goal.data(), w.al.data(),
                           w.be.data(), w.ga.data(), w.N, nullptr, w.v, md, mc8.data(), 8);
  EXPECT_EQ(std::memcmp(o1.data(), o8.data(), o1.size() * 4), 0);
  EXPECT_EQ(std::memcmp(mc1.data(), mc8.data(), w.N * 4), 0);
}

TEST(NavMaterialSimWorld, MaterialAvoidanceAndDeterminism) {
  // Open 48x48 world, one agent driving +x through a hazard blob grazing the
  // path. With material set, the agent must never enter a hard cell; without,
  // it drives straight through. Runs must be byte-identical across repeats.
  const int n = 48;
  std::vector<std::uint8_t> occ(n * n, 0);
  sim_world::config cfg;
  cfg.rows = n;
  cfg.cols = n;
  cfg.min_x = -100;
  cfg.min_y = -100;
  cfg.max_x = 100;
  cfg.max_y = 100;
  cfg.scale = 0.05;
  cfg.veh.rr = 0.15f;
  cfg.veh.d_hat = 0.35f;
  cfg.veh.dt = 0.06f;
  cfg.veh.nsub = 1;
  cfg.freeze_sense = true;

  std::vector<float> risk(n * n, 0.0f);
  std::vector<std::uint8_t> hard(n * n, 0);
  const int cr = 25, cc = 24, rad2 = 16; // blob straddling the y=0 goal line
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      if ((r - cr) * (r - cr) + (c - cc) * (c - cc) <= rad2) {
        hard[r * n + c] = 1;
        risk[r * n + c] = 1.0f;
      }

  auto run = [&](bool with_material) {
    const float o0[2] = {-4.5f * 0.5f, 0.0f}; // normalized (-45, 0)... scale 0.05
    const float g0[2] = {4.5f * 0.5f, 0.0f};
    const float col[3] = {1, 1, 1};
    sim_world w(cfg, occ.data(), occ.data(), coef_mlp::default_biased(), o0, g0, col, 1);
    if (with_material) {
      material_config mc;
      mc.gate.horizon_cells = 8;
      w.set_material(risk.data(), hard.data(), mc);
    }
    int hard_hits = 0;
    std::vector<float> trace;
    for (int t = 0; t < 500; ++t) {
      w.step(4);
      float pos[2], head, spd;
      int mode;
      std::uint8_t reach;
      w.snapshot(pos, &head, &spd, &mode, &reach);
      trace.push_back(pos[0]);
      trace.push_back(pos[1]);
      const double cxc = (pos[0] - cfg.min_x) / (cfg.max_x - cfg.min_x) * (n - 1);
      const double cyc = (pos[1] - cfg.min_y) / (cfg.max_y - cfg.min_y) * (n - 1);
      const int r = std::min(std::max(static_cast<int>(std::rint(cyc)), 0), n - 1);
      const int c = std::min(std::max(static_cast<int>(std::rint(cxc)), 0), n - 1);
      if (hard[r * n + c])
        ++hard_hits;
      if (reach)
        break;
    }
    return std::make_pair(hard_hits, trace);
  };

  const auto plain = run(false);
  const auto mat = run(true);
  EXPECT_GT(plain.first, 0) << "baseline never touched the hazard - setup broken";
  EXPECT_EQ(mat.first, 0) << "material sim_world entered the hazard";
  const auto mat2 = run(true);
  ASSERT_EQ(mat.second.size(), mat2.second.size());
  EXPECT_EQ(std::memcmp(mat.second.data(), mat2.second.data(), mat.second.size() * 4), 0);
}
