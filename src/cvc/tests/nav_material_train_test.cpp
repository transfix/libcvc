/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_train_test — the P5 RELEASE GATE: a finite-difference gradcheck of
// the FULL material training loss w.r.t. the CoefEnergyNetMaterial weights. This
// exercises the whole chain — model forward -> surrogate rollout -> loss, then
// loss grads -> integrate_surrogate_material_vjp -> coef_energy_net::backward_one
// -> weight grads — against a torch-independent central finite difference.
//
// The CVaR quantile is detached (a per-step constant): material_loss_and_grad
// returns the base-point eta, and the finite difference passes it back as
// material_loss's frozen_eta, so the FD sees the SAME detached quantile the
// analytic gradient used (otherwise the recomputed-quantile term the source
// never differentiates would corrupt the check near the tail threshold).
//
// L_multi is out of scope here (a separate geometry rollout — see
// material_train.h); this gates the material-path loss.

#include "coef_energy_test_model.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/material_train.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

using cvc::nav::coef_energy_net;
using cvc::nav::material_batch;
using cvc::nav::material_loss;
using cvc::nav::material_loss_and_grad;
using cvc::nav::material_loss_config;

namespace {

float U(std::mt19937 &rng, float lo, float hi) {
  return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

// Owns all batch arrays; `view()` hands out a material_batch of borrowed spans.
struct Batch {
  int B, N, P, Hp, Wp;
  std::vector<float> obs_feats, goal_feats, risk_patch, o0, v0, goal, C, R, rollout_patch, rr,
      d_hat, dt, o_tgt, v_tgt, gamma_o;
  std::vector<std::uint8_t> obs_mask;
  std::vector<int> H;
  material_batch view() const {
    material_batch b;
    b.B = B;
    b.N = N;
    b.P = P;
    b.Hp = Hp;
    b.Wp = Wp;
    b.obs_feats = obs_feats.data();
    b.obs_mask = obs_mask.data();
    b.goal_feats = goal_feats.data();
    b.risk_patch = risk_patch.data();
    b.o0 = o0.data();
    b.v0 = v0.data();
    b.goal = goal.data();
    b.C = C.data();
    b.R = R.data();
    b.rollout_patch = rollout_patch.data();
    b.rr = rr.data();
    b.d_hat = d_hat.data();
    b.dt = dt.data();
    b.H = H.data();
    b.o_tgt = o_tgt.data();
    b.v_tgt = v_tgt.data();
    b.gamma_o = gamma_o.data();
    return b;
  }
};

Batch make_batch(std::mt19937 &rng, int B, int N, int P, int Hp, int Wp) {
  Batch b;
  b.B = B;
  b.N = N;
  b.P = P;
  b.Hp = Hp;
  b.Wp = Wp;
  b.obs_feats.resize((std::size_t)B * N * 6);
  b.obs_mask.resize((std::size_t)B * N, 1);
  b.goal_feats.resize((std::size_t)B * 4);
  b.risk_patch.resize((std::size_t)B * 2 * P * P);
  b.o0.resize(2 * B);
  b.v0.resize(2 * B);
  b.goal.resize(2 * B);
  b.C.resize((std::size_t)B * N * 2);
  b.R.resize((std::size_t)B * N);
  b.rollout_patch.resize((std::size_t)B * 6 * Hp * Wp);
  b.rr.resize(B);
  b.d_hat.resize(B);
  b.dt.resize(B);
  b.o_tgt.resize(2 * B);
  b.v_tgt.resize(2 * B);
  b.gamma_o.resize(B);
  b.H.resize(B);

  for (int i = 0; i < B; ++i) {
    b.o0[2 * i] = U(rng, -0.5f, 0.5f);
    b.o0[2 * i + 1] = U(rng, -0.5f, 0.5f);
    b.v0[2 * i] = U(rng, -0.1f, 0.1f);
    b.v0[2 * i + 1] = U(rng, -0.1f, 0.1f);
    b.goal[2 * i] = U(rng, 1.5f, 2.5f);
    b.goal[2 * i + 1] = U(rng, 1.5f, 2.5f);
    b.rr[i] = 0.5f;
    b.d_hat[i] = 3.0f;
    b.dt[i] = 0.1f;
    b.H[i] = (i % 2 == 0) ? 2 : 3;
    b.o_tgt[2 * i] = b.goal[2 * i] + U(rng, -0.3f, 0.3f);
    b.o_tgt[2 * i + 1] = b.goal[2 * i + 1] + U(rng, -0.3f, 0.3f);
    b.v_tgt[2 * i] = U(rng, -0.2f, 0.2f);
    b.v_tgt[2 * i + 1] = U(rng, -0.2f, 0.2f);
    b.gamma_o[i] = U(rng, 3.0f, 5.0f);
    for (int j = 0; j < N; ++j) {
      const float ang = U(rng, 0.0f, 6.283185f), dist = U(rng, 1.0f, 1.4f); // moderate clearance
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(rng, 0.4f, 0.6f);
      // obs_feats [cx,cy,R,W,goal-cx,goal-cy] (data; the mask masks j==2 below)
      b.obs_feats[(i * N + j) * 6 + 0] = b.C[(i * N + j) * 2];
      b.obs_feats[(i * N + j) * 6 + 1] = b.C[(i * N + j) * 2 + 1];
      b.obs_feats[(i * N + j) * 6 + 2] = b.R[i * N + j];
      b.obs_feats[(i * N + j) * 6 + 3] = U(rng, 0.5f, 1.5f);
      b.obs_feats[(i * N + j) * 6 + 4] = b.goal[2 * i] - b.C[(i * N + j) * 2];
      b.obs_feats[(i * N + j) * 6 + 5] = b.goal[2 * i + 1] - b.C[(i * N + j) * 2 + 1];
    }
    b.obs_mask[i * N + (N - 1)] = 0; // one padded obstacle per agent
    b.goal_feats[4 * i] = b.goal[2 * i] - b.o0[2 * i];
    b.goal_feats[4 * i + 1] = b.goal[2 * i + 1] - b.o0[2 * i + 1];
    b.goal_feats[4 * i + 2] = std::sqrt(b.goal_feats[4 * i] * b.goal_feats[4 * i] +
                                        b.goal_feats[4 * i + 1] * b.goal_feats[4 * i + 1]);
    b.goal_feats[4 * i + 3] = 1.0f;
    // model risk_patch (2ch): ch0 risk in (0,1), ch1 hard mask
    for (int c = 0; c < P * P; ++c) {
      b.risk_patch[((std::size_t)i * 2 + 0) * P * P + c] = U(rng, 0.1f, 0.9f);
      b.risk_patch[((std::size_t)i * 2 + 1) * P * P + c] = (U(rng, 0, 1) > 0.7f) ? 1.0f : 0.0f;
    }
    // rollout patch (6ch): risk (0,1), phi (0,50), gentle gradients
    for (int r = 0; r < Hp; ++r)
      for (int c = 0; c < Wp; ++c) {
        const float fx = (float)c / (Wp - 1), fy = (float)r / (Hp - 1);
        const std::size_t base = ((std::size_t)i * 6) * Hp * Wp + (std::size_t)r * Wp + c;
        const std::size_t pp = (std::size_t)Hp * Wp;
        b.rollout_patch[base + 0 * pp] = 0.35f + 0.25f * std::sin(2.0f * fx + 1.3f * fy);
        b.rollout_patch[base + 1 * pp] = 2.0f + 1.2f * std::cos(1.7f * fx - 0.9f * fy);
        b.rollout_patch[base + 2 * pp] = 0.20f * std::cos(2.1f * fx);
        b.rollout_patch[base + 3 * pp] = 0.20f * std::sin(1.5f * fy);
        b.rollout_patch[base + 4 * pp] = 0.15f * std::sin(1.1f * fx + fy);
        b.rollout_patch[base + 5 * pp] = 0.15f * std::cos(0.8f * fx - fy);
      }
  }
  return b;
}

TEST(NavMaterialTrain, FullChainGradcheck) {
  std::mt19937 rng(2024);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch batch = make_batch(rng, /*B=*/16, /*N=*/3, P, /*Hp=*/13, /*Wp=*/13);
  const material_batch mb = batch.view();
  material_loss_config cfg; // train_material.py defaults

  // analytic gradient (fresh CVaR eta, returned for the FD to freeze)
  coef_energy_net::param_grads grads = m.zero_grads();
  float eta = 0.0f;
  const double L0 = material_loss_and_grad(m, mb, cfg, grads, &eta);
  ASSERT_TRUE(std::isfinite(L0));

  const std::vector<std::string> names = m.param_names();
  double gnorm2 = 0.0;
  for (const auto &nm : names)
    for (float x : grads.at(nm))
      gnorm2 += (double)x * x;
  const double gnorm = std::sqrt(gnorm2);
  ASSERT_GT(gnorm, 1e-3);

  const float eps = 1e-3f;
  auto floss = [&]() { return material_loss(m, mb, cfg, eta); }; // frozen eta

  // (1) directional FD along the full analytic gradient
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double Lp = floss();
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] -= 2.0f * eps * (float)(g[i] / gnorm);
  }
  const double Lm = floss();
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    for (std::size_t i = 0; i < w.size(); ++i)
      w[i] += eps * (float)(g[i] / gnorm);
  }
  const double dd_fd = (Lp - Lm) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gnorm) / (std::fabs(dd_fd) + gnorm + 1e-9);

  // (2) per-tensor spot check (large-gradient params only, where FD is reliable)
  double worst = 0.0;
  std::string worst_nm;
  int checked = 0;
  for (const auto &nm : names) {
    std::vector<float> &w = m.mutable_param(nm);
    const std::vector<float> &g = grads.at(nm);
    int taken = 0;
    for (std::size_t i = 0; i < w.size() && taken < 5; ++i) {
      if (std::fabs(g[i]) < 5e-2)
        continue;
      const float orig = w[i];
      w[i] = orig + eps;
      const double lp = floss();
      w[i] = orig - eps;
      const double lm = floss();
      w[i] = orig;
      const double fd = (lp - lm) / (2.0 * eps);
      const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-4);
      if (rel > worst) {
        worst = rel;
        worst_nm = nm;
      }
      ++taken;
      ++checked;
    }
  }

  std::printf("[material-train-e2e] L=%.4f |g|=%.3f dir_rel=%.3e checked=%d worst_rel=%.3e (%s)\n",
              L0, gnorm, dir_rel, checked, worst, worst_nm.c_str());
  EXPECT_LT(dir_rel, 2e-2) << "full loss->weights gradient fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a weight gradient disagrees with finite differences (" << worst_nm
                         << ")";
  EXPECT_GE(checked, 30);
}

} // namespace
