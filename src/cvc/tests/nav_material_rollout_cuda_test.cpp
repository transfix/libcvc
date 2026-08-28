/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_material_rollout_cuda_test — CUDA-vs-CPU parity for the material surrogate
// rollout forward + VJP. The device kernels reuse the SAME CVC_HD primitives as
// the CPU (so the CPU finite-difference gradcheck already validates the adjoint);
// this confirms the GPU transcription matches the host: forward float-equivalent
// and the coefficient gradients agree to rel < 5e-3 / cos > 0.9999. Built with
// CVC_ENABLE_CUDA; a trivial skip without it (or without a device).

#include <gtest/gtest.h>

#ifndef CVC_ENABLE_CUDA
TEST(NavMaterialRolloutCuda, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/material.h>
#include <random>
#include <vector>

using namespace cvc::nav;

namespace {

struct Batch {
  int B = 24, N = 3, P = 13;
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
  b.o0.resize(2 * B);
  b.v0.resize(2 * B);
  b.goal.resize(2 * B);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.assign(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
  b.lam_soft.resize(B);
  b.lam_hard.resize(B);
  b.patch.assign((std::size_t)B * 6 * P * P, 0.0f);
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
    b.H[i] = (i % 3) + 2; // 2..4
    for (int j = 0; j < N; ++j) {
      const float ang = U(0.0f, 6.283185f), dist = U(1.5f, 1.9f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(0.5f, 0.7f);
      b.alphas[i * N + j] = U(0.3f, 0.8f);
    }
    b.mask[i * N + (N - 1)] = 0;
    for (int r = 0; r < P; ++r)
      for (int c = 0; c < P; ++c) {
        const float fx = (float)c / (P - 1), fy = (float)r / (P - 1);
        const std::size_t base = ((std::size_t)i * 6) * P * P + (std::size_t)r * P + c;
        const std::size_t pp = (std::size_t)P * P;
        b.patch[base + 0 * pp] = 0.35f + 0.25f * std::sin(2.0f * fx + 1.3f * fy);
        b.patch[base + 1 * pp] = 2.0f + 1.2f * std::cos(1.7f * fx - 0.9f * fy);
        b.patch[base + 2 * pp] = 0.20f * std::cos(2.1f * fx);
        b.patch[base + 3 * pp] = 0.20f * std::sin(1.5f * fy);
        b.patch[base + 4 * pp] = 0.15f * std::sin(1.1f * fx + fy);
        b.patch[base + 5 * pp] = 0.15f * std::cos(0.8f * fx - fy);
      }
  }
  return b;
}

void rel_cos(const std::vector<float> &a, const std::vector<float> &c, double &worst_rel,
             double &cosine) {
  worst_rel = 0.0;
  double dot = 0.0, na = 0.0, nc = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    dot += (double)a[i] * c[i];
    na += (double)a[i] * a[i];
    nc += (double)c[i] * c[i];
    const double denom = std::fabs(a[i]) + std::fabs(c[i]) + 1e-5;
    worst_rel = std::max(worst_rel, std::fabs((double)a[i] - c[i]) / denom);
  }
  cosine = (na > 0 && nc > 0) ? dot / (std::sqrt(na) * std::sqrt(nc)) : 1.0;
}

} // namespace

TEST(NavMaterialRolloutCuda, ForwardMatchesCpu) {
  if (!material_rollout_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  const Batch b = make_batch(5);
  std::vector<float> o_cpu(b.o0), v_cpu(b.v0), mc_c(b.B), cr_c(b.B), hc_c(b.B), arc_c(b.B);
  std::vector<float> o_gpu(b.o0), v_gpu(b.v0), mc_g(b.B), cr_g(b.B), hc_g(b.B), arc_g(b.B);
  integrate_surrogate_material(o_cpu.data(), v_cpu.data(), b.goal.data(), b.C.data(), b.R.data(),
                               b.mask.data(), b.alphas.data(), b.beta.data(), b.gamma.data(),
                               b.lam_soft.data(), b.lam_hard.data(), b.patch.data(), b.rr.data(),
                               b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p,
                               mc_c.data(), cr_c.data(), hc_c.data(), arc_c.data(), 1);
  integrate_surrogate_material_cuda(
      o_gpu.data(), v_gpu.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
      b.alphas.data(), b.beta.data(), b.gamma.data(), b.lam_soft.data(), b.lam_hard.data(),
      b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p,
      mc_g.data(), cr_g.data(), hc_g.data(), arc_g.data());
  double wr, cs, worst = 0.0;
  for (auto pr : {std::pair<std::vector<float> *, std::vector<float> *>{&o_cpu, &o_gpu},
                  {&v_cpu, &v_gpu},
                  {&mc_c, &mc_g},
                  {&cr_c, &cr_g},
                  {&hc_c, &hc_g},
                  {&arc_c, &arc_g}}) {
    rel_cos(*pr.first, *pr.second, wr, cs);
    worst = std::max(worst, wr);
  }
  std::printf("[mat-cuda-fwd] worst_rel=%.3e\n", worst);
  EXPECT_LT(worst, 1e-4) << "CUDA forward disagrees with CPU";
}

TEST(NavMaterialRolloutCuda, VjpMatchesCpu) {
  if (!material_rollout_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  const Batch b = make_batch(9);
  // random upstream grads
  std::mt19937 rng(3);
  auto RV = [&](int n) {
    std::vector<float> v(n);
    for (auto &x : v)
      x = (float)std::uniform_real_distribution<double>(-1.0, 1.0)(rng);
    return v;
  };
  const std::vector<float> g_oT = RV(2 * b.B), g_vT = RV(2 * b.B), g_mc = RV(b.B), g_cr = RV(b.B),
                           g_arc = RV(b.B);
  auto run = [&](bool cuda, std::vector<float> &ga, std::vector<float> &gb, std::vector<float> &gg,
                 std::vector<float> &gls, std::vector<float> &glh) {
    ga.assign(b.B * b.N, 0.0f);
    gb.assign(b.B, 0.0f);
    gg.assign(b.B, 0.0f);
    gls.assign(b.B, 0.0f);
    glh.assign(b.B, 0.0f);
    if (cuda)
      integrate_surrogate_material_vjp_cuda(
          b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
          b.alphas.data(), b.beta.data(), b.gamma.data(), b.lam_soft.data(), b.lam_hard.data(),
          b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P,
          b.p, g_oT.data(), g_vT.data(), g_mc.data(), g_cr.data(), g_arc.data(), ga.data(),
          gb.data(), gg.data(), gls.data(), glh.data());
    else
      integrate_surrogate_material_vjp(
          b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
          b.alphas.data(), b.beta.data(), b.gamma.data(), b.lam_soft.data(), b.lam_hard.data(),
          b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P,
          b.p, g_oT.data(), g_vT.data(), g_mc.data(), g_cr.data(), g_arc.data(), ga.data(),
          gb.data(), gg.data(), gls.data(), glh.data(), 1);
  };
  std::vector<float> a_c, b_c, g_c, ls_c, lh_c, a_g, b_g, g_g, ls_g, lh_g;
  run(false, a_c, b_c, g_c, ls_c, lh_c);
  run(true, a_g, b_g, g_g, ls_g, lh_g);
  double worst = 0.0, mincos = 1.0, wr, cs;
  const char *nm[5] = {"alphas", "beta", "gamma", "lam_soft", "lam_hard"};
  std::vector<float> *C_[5] = {&a_c, &b_c, &g_c, &ls_c, &lh_c};
  std::vector<float> *G_[5] = {&a_g, &b_g, &g_g, &ls_g, &lh_g};
  for (int k = 0; k < 5; ++k) {
    rel_cos(*C_[k], *G_[k], wr, cs);
    worst = std::max(worst, wr);
    mincos = std::min(mincos, cs);
    (void)nm;
  }
  std::printf("[mat-cuda-vjp] worst_rel=%.3e min_cos=%.6f\n", worst, mincos);
  EXPECT_LT(worst, 5e-3) << "CUDA gradient disagrees with CPU";
  EXPECT_GT(mincos, 0.9999) << "CUDA gradient direction diverges from CPU";
}

#endif // CVC_ENABLE_CUDA
