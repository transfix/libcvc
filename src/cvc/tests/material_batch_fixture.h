/*
  Copyright 2007-2011 The University of Texas at Austin
        Authors: Joe Rivera <transfix@ices.utexas.edu>
  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_batch_fixture.h — a synthetic, self-owning `material_batch` for the
// P5 material-training tests (CPU gradcheck and the CUDA end-to-end parity). One
// padded obstacle per agent is masked; risk/rollout patches carry gentle
// gradients. Shared so the CPU and CUDA tests exercise byte-identical inputs.

#ifndef CVC_TEST_MATERIAL_BATCH_FIXTURE_H
#define CVC_TEST_MATERIAL_BATCH_FIXTURE_H

#include <cmath>
#include <cstdint>
#include <cvc/nav/material_train.h>
#include <random>
#include <vector>

namespace cvc_test {

inline float U(std::mt19937 &rng, float lo, float hi) {
  return lo + (hi - lo) * static_cast<float>(std::uniform_real_distribution<double>(0.0, 1.0)(rng));
}

// Owns all batch arrays; `view()` hands out a material_batch of borrowed spans.
struct Batch {
  int B, N, P, Hp, Wp;
  std::vector<float> obs_feats, goal_feats, risk_patch, o0, v0, goal, C, R, rollout_patch, rr,
      d_hat, dt, o_tgt, v_tgt, gamma_o;
  std::vector<std::uint8_t> obs_mask;
  std::vector<int> H;
  cvc::nav::material_batch view() const {
    cvc::nav::material_batch b;
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

inline Batch make_batch(std::mt19937 &rng, int B, int N, int P, int Hp, int Wp) {
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
      // obs_feats [cx,cy,R,W,goal-cx,goal-cy] (data; the mask masks j==N-1 below)
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

} // namespace cvc_test

#endif
