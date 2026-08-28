/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_optim_test — P5-P4: the host optimizer + checkpoint IO.
//   * CvcnmRoundTrip: serialize() -> load_from_memory() reproduces byte-identical
//     weights, so the model forward is bit-for-bit unchanged (a C++-trained
//     policy checkpoints losslessly into the .cvcnm the forward/CUDA paths read).
//   * TrainingReducesLoss: Adam + global-norm clip + cosine LR actually DESCENDS
//     the material loss on a fixed batch — an end-to-end check that the gradient
//     (gradchecked in nav_material_train_test) has the right sign and scale, the
//     coef_train TrainingReducesLoss discipline applied to the material trainer.

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
using cvc::nav::cosine_lr;
using cvc::nav::material_adam;
using cvc::nav::material_batch;
using cvc::nav::material_loss_and_grad;
using cvc::nav::material_loss_config;

namespace {

float U(std::mt19937 &rng, float lo, float hi) {
  return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
}

struct Batch {
  int B, N, P, Hp, Wp;
  std::vector<float> obs_feats, goal_feats, risk_patch, o0, v0, goal, C, R, rollout_patch, rr,
      d_hat, dt, o_tgt, v_tgt, gamma_o;
  std::vector<std::uint8_t> obs_mask;
  std::vector<int> H;
  material_batch view() const {
    material_batch b;
    b.B = B, b.N = N, b.P = P, b.Hp = Hp, b.Wp = Wp;
    b.obs_feats = obs_feats.data(), b.obs_mask = obs_mask.data(), b.goal_feats = goal_feats.data();
    b.risk_patch = risk_patch.data(), b.o0 = o0.data(), b.v0 = v0.data(), b.goal = goal.data();
    b.C = C.data(), b.R = R.data(), b.rollout_patch = rollout_patch.data(), b.rr = rr.data();
    b.d_hat = d_hat.data(), b.dt = dt.data(), b.H = H.data(), b.o_tgt = o_tgt.data();
    b.v_tgt = v_tgt.data(), b.gamma_o = gamma_o.data();
    return b;
  }
};

Batch make_batch(std::mt19937 &rng, int B, int N, int P, int Hp, int Wp) {
  Batch b;
  b.B = B, b.N = N, b.P = P, b.Hp = Hp, b.Wp = Wp;
  b.obs_feats.resize((std::size_t)B * N * 6);
  b.obs_mask.assign((std::size_t)B * N, 1);
  b.goal_feats.resize((std::size_t)B * 4);
  b.risk_patch.resize((std::size_t)B * 2 * P * P);
  b.o0.resize(2 * B), b.v0.resize(2 * B), b.goal.resize(2 * B);
  b.C.resize((std::size_t)B * N * 2), b.R.resize((std::size_t)B * N);
  b.rollout_patch.resize((std::size_t)B * 6 * Hp * Wp);
  b.rr.resize(B), b.d_hat.resize(B), b.dt.resize(B);
  b.o_tgt.resize(2 * B), b.v_tgt.resize(2 * B), b.gamma_o.resize(B), b.H.resize(B);
  for (int i = 0; i < B; ++i) {
    b.o0[2 * i] = U(rng, -0.5f, 0.5f), b.o0[2 * i + 1] = U(rng, -0.5f, 0.5f);
    b.v0[2 * i] = U(rng, -0.1f, 0.1f), b.v0[2 * i + 1] = U(rng, -0.1f, 0.1f);
    b.goal[2 * i] = U(rng, 1.5f, 2.5f), b.goal[2 * i + 1] = U(rng, 1.5f, 2.5f);
    b.rr[i] = 0.5f, b.d_hat[i] = 3.0f, b.dt[i] = 0.1f, b.H[i] = (i % 2 == 0) ? 2 : 3;
    b.o_tgt[2 * i] = b.goal[2 * i] + U(rng, -0.3f, 0.3f);
    b.o_tgt[2 * i + 1] = b.goal[2 * i + 1] + U(rng, -0.3f, 0.3f);
    b.v_tgt[2 * i] = U(rng, -0.2f, 0.2f), b.v_tgt[2 * i + 1] = U(rng, -0.2f, 0.2f);
    b.gamma_o[i] = U(rng, 3.0f, 5.0f);
    for (int j = 0; j < N; ++j) {
      const float ang = U(rng, 0.0f, 6.28f), dist = U(rng, 1.0f, 1.4f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(rng, 0.4f, 0.6f);
      b.obs_feats[(i * N + j) * 6 + 0] = b.C[(i * N + j) * 2];
      b.obs_feats[(i * N + j) * 6 + 1] = b.C[(i * N + j) * 2 + 1];
      b.obs_feats[(i * N + j) * 6 + 2] = b.R[i * N + j];
      b.obs_feats[(i * N + j) * 6 + 3] = U(rng, 0.5f, 1.5f);
      b.obs_feats[(i * N + j) * 6 + 4] = b.goal[2 * i] - b.C[(i * N + j) * 2];
      b.obs_feats[(i * N + j) * 6 + 5] = b.goal[2 * i + 1] - b.C[(i * N + j) * 2 + 1];
    }
    b.goal_feats[4 * i] = b.goal[2 * i] - b.o0[2 * i];
    b.goal_feats[4 * i + 1] = b.goal[2 * i + 1] - b.o0[2 * i + 1];
    b.goal_feats[4 * i + 2] = std::sqrt(b.goal_feats[4 * i] * b.goal_feats[4 * i] +
                                        b.goal_feats[4 * i + 1] * b.goal_feats[4 * i + 1]);
    b.goal_feats[4 * i + 3] = 1.0f;
    for (int c = 0; c < P * P; ++c) {
      b.risk_patch[((std::size_t)i * 2 + 0) * P * P + c] = U(rng, 0.1f, 0.9f);
      b.risk_patch[((std::size_t)i * 2 + 1) * P * P + c] = (U(rng, 0, 1) > 0.7f) ? 1.0f : 0.0f;
    }
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

// forward outputs concatenated, for the round-trip comparison.
std::vector<float> forward_all(const coef_energy_net &m, const Batch &b) {
  std::vector<float> out;
  for (int i = 0; i < b.B; ++i) {
    std::vector<float> al(b.N, 0.f);
    float be, ga, ls, lh, mu;
    m.forward_one(b.obs_feats.data() + (std::size_t)i * b.N * 6, b.obs_mask.data() + i * b.N, b.N,
                  b.goal_feats.data() + (std::size_t)i * 4,
                  b.risk_patch.data() + (std::size_t)i * 2 * b.P * b.P, b.P, al.data(), &be, &ga,
                  &ls, &lh, &mu);
    out.insert(out.end(), al.begin(), al.end());
    out.push_back(be), out.push_back(ga), out.push_back(ls), out.push_back(lh), out.push_back(mu);
  }
  return out;
}

TEST(NavMaterialOptim, CvcnmRoundTrip) {
  std::mt19937 rng(11);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch b = make_batch(rng, 5, 3, P, 13, 13);
  const std::vector<float> before = forward_all(m, b);

  const std::vector<unsigned char> blob = m.serialize();
  coef_energy_net m2 = coef_energy_net::load_from_memory(blob.data(), blob.size());
  EXPECT_EQ(m2.arch_hash(), m.arch_hash());
  EXPECT_EQ(m2.patch_size(), m.patch_size());
  // weights byte-identical -> forward byte-identical
  const std::vector<float> after = forward_all(m2, b);
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t i = 0; i < before.size(); ++i)
    EXPECT_EQ(before[i], after[i]) << "output " << i << " changed across .cvcnm round-trip";
}

TEST(NavMaterialOptim, TrainingReducesLoss) {
  std::mt19937 rng(202);
  const int P = 16;
  coef_energy_net m = cvc_test::build_test_model(rng, P);
  const Batch b = make_batch(rng, 16, 3, P, 13, 13); // fixed batch: overfit it
  const material_batch mb = b.view();
  material_loss_config cfg;

  material_adam opt(m, /*grad_clip=*/5.0f);
  const int steps = 70;
  const float lr0 = 1e-3f, lr_min = lr0 * 0.1f; // a touch higher than 1e-4 to move in few steps

  double L_first = 0.0, L_last = 0.0;
  for (int s = 0; s < steps; ++s) {
    coef_energy_net::param_grads g = m.zero_grads();
    const double L = material_loss_and_grad(m, mb, cfg, g, nullptr);
    if (s == 0)
      L_first = L;
    L_last = L;
    opt.step(m, g, cosine_lr(lr0, lr_min, s, steps));
  }
  std::printf("[material-optim] L_first=%.4f L_last=%.4f ratio=%.3f\n", L_first, L_last,
              L_last / L_first);
  EXPECT_TRUE(std::isfinite(L_last));
  EXPECT_LT(L_last, L_first) << "Adam did not reduce the loss";
  EXPECT_LT(L_last, 0.8 * L_first) << "loss did not decrease substantially over training";
}

} // namespace
