/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_rollout_grad_test — GRADCHECK for the P5 training backward of the
// obstacle-list material surrogate rollout (integrate_surrogate_material_vjp).
//
// The load-bearing test is a central finite-difference gradcheck of the
// hand-written reverse-mode adjoint (the patch-sample position VJP, the IPC and
// SDF-barrier derivatives, and the semi-implicit BPTT chain) against a numeric
// gradient of a scalar loss over the rollout outputs. Torch-INDEPENDENT ground
// truth: if the analytic gradient equals the numeric one, the backward is
// correct with no reference to autograd — the same discipline as
// nav_coef_train_test, and the correctness gate for the whole P5 stack that
// builds on this adjoint (transformer/CNN backward, CVaR loss, Adam).
//
// The batch is constructed at interior points: every step's obstacle clearance d
// stays inside (eps, d_hat) (away from the ipc_piecewise jump at d=d_hat and the
// vp branch at d<=eps), the sampled risk stays in (0,1) and phi in (0,50) (away
// from the clamps), and the horizon is short so the min_clear argmin is stable
// under the FD perturbation. hard_count is a step-function count with identically
// zero gradient, so it takes no seed and is excluded from the loss.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/material.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using cvc::nav::integrate_surrogate_material;
using cvc::nav::integrate_surrogate_material_vjp;
using cvc::nav::surrogate_material_params;

namespace {

// A controlled batch whose data (o0/v0/goal/C/R/mask/patch/rr/d_hat/dt/H) is
// fixed; only the learned coefficients (alphas/beta/gamma/lam_soft/lam_hard) are
// the gradcheck parameters. Sizes: B agents, N obstacles, (6,P,P) patch.
struct Batch {
  int B = 6, N = 3, P = 13;
  std::vector<float> o0, v0, goal, C, R, alphas, beta, gamma, lam_soft, lam_hard, patch, rr, d_hat,
      dt;
  std::vector<std::uint8_t> mask;
  std::vector<int> H;
  surrogate_material_params p;
};

Batch make_batch(unsigned seed) {
  Batch b;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  const int B = b.B, N = b.N, P = b.P;
  b.p.margin_factor = 0.5f;
  b.p.mass = 1.0f;
  b.p.d_hat_sdf = 3.0f;
  b.p.k_sharp = 5.0f;

  b.o0.resize(B * 2);
  b.v0.resize(B * 2);
  b.goal.resize(B * 2);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.resize(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
  b.lam_soft.resize(B);
  b.lam_hard.resize(B);
  b.patch.assign((size_t)B * 6 * P * P, 0.0f);
  b.rr.resize(B);
  b.d_hat.resize(B);
  b.dt.resize(B);
  b.H.resize(B);

  for (int i = 0; i < B; ++i) {
    b.o0[2 * i] = U(-0.5f, 0.5f);
    b.o0[2 * i + 1] = U(-0.5f, 0.5f);
    b.v0[2 * i] = U(-0.1f, 0.1f);
    b.v0[2 * i + 1] = U(-0.1f, 0.1f);
    b.goal[2 * i] = U(1.5f, 2.5f);
    b.goal[2 * i + 1] = U(1.5f, 2.5f);
    b.beta[i] = U(0.8f, 1.2f);
    b.gamma[i] = U(0.2f, 0.5f);
    b.lam_soft[i] = U(0.7f, 1.3f);
    b.lam_hard[i] = U(0.7f, 1.3f);
    b.rr[i] = 0.5f;
    b.d_hat[i] = 3.0f;
    b.dt[i] = 0.1f;
    b.H[i] = (i % 2 == 0) ? 2 : 3; // short + ragged (min_clear at frozen steps)
    // Obstacles placed so d = |o0 - C| - (R + margin*rr) sits comfortably inside
    // (eps, d_hat): pick a distance ~1.6 and R ~ 0.6, rr_eff = 0.25 -> d ~ 0.75.
    for (int j = 0; j < N; ++j) {
      const float ang = U(0.0f, 6.283185f), dist = U(1.5f, 1.9f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(0.5f, 0.7f);
      b.alphas[i * N + j] = U(0.3f, 0.8f);
    }
    // Patch channels: risk in (0,1) interior, phi in (0,50) interior, gentle
    // gradient channels — all smooth so the bilinear position VJP is exercised
    // without hitting a clamp.
    for (int r = 0; r < P; ++r) {
      for (int c = 0; c < P; ++c) {
        const float fx = (float)c / (P - 1), fy = (float)r / (P - 1);
        const size_t base = ((size_t)i * 6) * P * P + (size_t)r * P + c;
        const size_t pp = (size_t)P * P;
        b.patch[base + 0 * pp] = 0.35f + 0.25f * std::sin(2.0f * fx + 1.3f * fy); // risk (0,1)
        b.patch[base + 1 * pp] = 2.0f + 1.2f * std::cos(1.7f * fx - 0.9f * fy);   // phi (0,50)
        b.patch[base + 2 * pp] = 0.20f * std::cos(2.1f * fx);                     // dr/dx
        b.patch[base + 3 * pp] = 0.20f * std::sin(1.5f * fy);                     // dr/dy
        b.patch[base + 4 * pp] = 0.15f * std::sin(1.1f * fx + fy);                // dphi/dx
        b.patch[base + 5 * pp] = 0.15f * std::cos(0.8f * fx - fy);                // dphi/dy
      }
    }
  }
  return b;
}

// Pack/unpack the coefficient parameters into one flat vector:
// [alphas(B*N) | beta(B) | gamma(B) | lam_soft(B) | lam_hard(B)].
int num_params(const Batch &b) { return b.B * b.N + 4 * b.B; }

void unpack(const Batch &b, const std::vector<float> &p, std::vector<float> &al,
            std::vector<float> &be, std::vector<float> &ga, std::vector<float> &ls,
            std::vector<float> &lh) {
  int k = 0;
  al.assign(p.begin() + k, p.begin() + k + b.B * b.N);
  k += b.B * b.N;
  be.assign(p.begin() + k, p.begin() + k + b.B);
  k += b.B;
  ga.assign(p.begin() + k, p.begin() + k + b.B);
  k += b.B;
  ls.assign(p.begin() + k, p.begin() + k + b.B);
  k += b.B;
  lh.assign(p.begin() + k, p.begin() + k + b.B);
}

std::vector<float> pack(const Batch &b) {
  std::vector<float> p;
  p.insert(p.end(), b.alphas.begin(), b.alphas.end());
  p.insert(p.end(), b.beta.begin(), b.beta.end());
  p.insert(p.end(), b.gamma.begin(), b.gamma.end());
  p.insert(p.end(), b.lam_soft.begin(), b.lam_soft.end());
  p.insert(p.end(), b.lam_hard.begin(), b.lam_hard.end());
  return p;
}

// Fixed per-output loss weights (the upstream grad seeds). hard_count omitted.
struct Q {
  std::vector<float> oT, vT, mc, cr, ar; // [B*2],[B*2],[B],[B],[B]
};

Q make_q(const Batch &b, unsigned seed) {
  Q q;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  q.oT.resize(b.B * 2);
  q.vT.resize(b.B * 2);
  q.mc.resize(b.B);
  q.cr.resize(b.B);
  q.ar.resize(b.B);
  for (int i = 0; i < b.B; ++i) {
    q.oT[2 * i] = U(-1.0f, 1.0f);
    q.oT[2 * i + 1] = U(-1.0f, 1.0f);
    q.vT[2 * i] = U(-1.0f, 1.0f);
    q.vT[2 * i + 1] = U(-1.0f, 1.0f);
    q.mc[i] = U(-0.5f, 0.5f); // modest weight on the kinked min_clear
    q.cr[i] = U(-1.0f, 1.0f);
    q.ar[i] = U(-1.0f, 1.0f);
  }
  return q;
}

// L = sum_b  q.oT . oT_b + q.vT . vT_b + q.mc*min_clear_b + q.cr*cum_risk_b
//            + q.ar*arc_b   (double accumulation to stay above the FP noise floor)
double loss_forward(const Batch &b, const std::vector<float> &params, const Q &q) {
  std::vector<float> al, be, ga, ls, lh;
  unpack(b, params, al, be, ga, ls, lh);
  std::vector<float> o(b.o0), v(b.v0);
  std::vector<float> mc(b.B), cr(b.B), hc(b.B), ar(b.B);
  integrate_surrogate_material(o.data(), v.data(), b.goal.data(), b.C.data(), b.R.data(),
                               b.mask.data(), al.data(), be.data(), ga.data(), ls.data(), lh.data(),
                               b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(),
                               b.B, b.N, b.P, b.P, b.p, mc.data(), cr.data(), hc.data(), ar.data(),
                               1);
  double L = 0.0;
  for (int i = 0; i < b.B; ++i) {
    L += (double)q.oT[2 * i] * o[2 * i] + (double)q.oT[2 * i + 1] * o[2 * i + 1];
    L += (double)q.vT[2 * i] * v[2 * i] + (double)q.vT[2 * i + 1] * v[2 * i + 1];
    L += (double)q.mc[i] * mc[i] + (double)q.cr[i] * cr[i] + (double)q.ar[i] * ar[i];
  }
  return L;
}

TEST(NavMaterialRolloutGrad, GradcheckMatchesFiniteDifference) {
  const Batch b = make_batch(21);
  const Q q = make_q(b, 99);
  const int P = num_params(b);

  // Analytic gradient via the VJP (seeds = the loss weights q).
  std::vector<float> al, be, ga, ls, lh;
  std::vector<float> base = pack(b);
  unpack(b, base, al, be, ga, ls, lh);
  std::vector<float> g_al(b.B * b.N, 0.0f), g_be(b.B, 0.0f), g_ga(b.B, 0.0f), g_ls(b.B, 0.0f),
      g_lh(b.B, 0.0f);
  integrate_surrogate_material_vjp(
      b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(), al.data(),
      be.data(), ga.data(), ls.data(), lh.data(), b.patch.data(), b.rr.data(), b.d_hat.data(),
      b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p, q.oT.data(), q.vT.data(), q.mc.data(),
      q.cr.data(), q.ar.data(), g_al.data(), g_be.data(), g_ga.data(), g_ls.data(), g_lh.data(), 1);
  // Flatten analytic grad into the packed layout.
  std::vector<float> g;
  g.insert(g.end(), g_al.begin(), g_al.end());
  g.insert(g.end(), g_be.begin(), g_be.end());
  g.insert(g.end(), g_ga.begin(), g_ga.end());
  g.insert(g.end(), g_ls.begin(), g_ls.end());
  g.insert(g.end(), g_lh.begin(), g_lh.end());
  ASSERT_EQ((int)g.size(), P);

  double gnorm = 0.0;
  for (float x : g)
    gnorm += (double)x * x;
  gnorm = std::sqrt(gnorm);
  ASSERT_GT(gnorm, 1e-4);

  // (1) Directional finite difference along the analytic gradient direction.
  const float eps = 2e-3f;
  std::vector<float> pp(base), pm(base);
  double gdotg = 0.0;
  for (int i = 0; i < P; ++i) {
    const double u = g[i] / gnorm;
    pp[i] = base[i] + eps * (float)u;
    pm[i] = base[i] - eps * (float)u;
    gdotg += (double)g[i] * u;
  }
  const double dd_fd = (loss_forward(b, pp, q) - loss_forward(b, pm, q)) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gdotg) / (std::fabs(dd_fd) + std::fabs(gdotg) + 1e-9);

  // (2) Per-parameter finite difference (params above the FP noise floor).
  int checked = 0;
  double worst_rel = 0.0;
  for (int i = 0; i < P; ++i) {
    if (std::fabs(g[i]) < 1e-3)
      continue;
    std::vector<float> plus(base), minus(base);
    plus[i] = base[i] + eps;
    minus[i] = base[i] - eps;
    const double fd = (loss_forward(b, plus, q) - loss_forward(b, minus, q)) / (2.0 * eps);
    const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6);
    worst_rel = std::max(worst_rel, rel);
    ++checked;
  }

  std::printf("[mat-rollout-gradcheck] params=%d |g|=%.4f dir_rel=%.3e checked=%d worst_rel=%.3e\n",
              P, gnorm, dir_rel, checked, worst_rel);
  EXPECT_LT(dir_rel, 2e-2) << "material-rollout backward fails the directional FD check";
  EXPECT_LT(worst_rel, 5e-2) << "a coefficient gradient disagrees with finite differences";
  EXPECT_GE(checked, 8) << "too few params cleared the FD noise floor";
}

// The backward must be thread-count invariant (the parallel_for is over agents,
// each fully independent — no cross-agent accumulation).
TEST(NavMaterialRolloutGrad, ThreadDeterminism) {
  const Batch b = make_batch(7);
  const Q q = make_q(b, 5);
  std::vector<float> al, be, ga, ls, lh;
  std::vector<float> base = pack(b);
  unpack(b, base, al, be, ga, ls, lh);

  auto run = [&](int nt, std::vector<float> &g_al, std::vector<float> &g_be,
                 std::vector<float> &g_ga, std::vector<float> &g_ls, std::vector<float> &g_lh) {
    g_al.assign(b.B * b.N, 0.0f);
    g_be.assign(b.B, 0.0f);
    g_ga.assign(b.B, 0.0f);
    g_ls.assign(b.B, 0.0f);
    g_lh.assign(b.B, 0.0f);
    integrate_surrogate_material_vjp(
        b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(), al.data(),
        be.data(), ga.data(), ls.data(), lh.data(), b.patch.data(), b.rr.data(), b.d_hat.data(),
        b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p, q.oT.data(), q.vT.data(), q.mc.data(),
        q.cr.data(), q.ar.data(), g_al.data(), g_be.data(), g_ga.data(), g_ls.data(), g_lh.data(),
        nt);
  };
  std::vector<float> a1, b1, c1, d1, e1, a8, b8, c8, d8, e8;
  run(1, a1, b1, c1, d1, e1);
  run(8, a8, b8, c8, d8, e8);
  EXPECT_EQ(a1, a8);
  EXPECT_EQ(b1, b8);
  EXPECT_EQ(c1, c8);
  EXPECT_EQ(d1, d8);
  EXPECT_EQ(e1, e8);
}

} // namespace
