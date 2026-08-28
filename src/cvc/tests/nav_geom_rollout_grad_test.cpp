/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_geom_rollout_grad_test — finite-difference gradchecks for the P5-P3b
// geometry rollout adjoint (integrate_surrogate_v2_vjp) and the multi-start
// penalty (multi_start_penalty), both w.r.t. the geometry coefficients
// (alphas, beta, gamma). Torch-independent ground truth, same discipline as the
// material rollout gradcheck.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/geom_rollout.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using cvc::nav::geom_rollout_params;
using cvc::nav::integrate_surrogate_v2;
using cvc::nav::integrate_surrogate_v2_vjp;
using cvc::nav::multi_start_params;
using cvc::nav::multi_start_penalty;

namespace {

struct GB {
  int B = 6, N = 3;
  std::vector<float> o0, v0, goal, C, R, alphas, beta, gamma, rr, d_hat, dt;
  std::vector<std::uint8_t> mask;
  std::vector<int> H;
  geom_rollout_params gp;
};

GB make_batch(unsigned seed) {
  GB b;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  const int B = b.B, N = b.N;
  b.gp.margin_factor = 0.5f;
  b.gp.mass = 1.0f;
  b.o0.resize(2 * B);
  b.v0.resize(2 * B);
  b.goal.resize(2 * B);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.assign(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
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
    b.rr[i] = 0.5f;
    b.d_hat[i] = 3.0f;
    b.dt[i] = 0.1f;
    b.H[i] = (i % 2 == 0) ? 2 : 3;
    for (int j = 0; j < N; ++j) {
      const float ang = U(0.0f, 6.283185f), dist = U(1.5f, 1.9f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(0.5f, 0.7f);
      b.alphas[i * N + j] = U(0.3f, 0.8f);
    }
    b.mask[i * N + (N - 1)] = 0; // one padded obstacle
  }
  return b;
}

int num_params(const GB &b) { return b.B * b.N + 2 * b.B; }
void unpack(const GB &b, const std::vector<float> &p, std::vector<float> &al,
            std::vector<float> &be, std::vector<float> &ga) {
  al.assign(p.begin(), p.begin() + b.B * b.N);
  be.assign(p.begin() + b.B * b.N, p.begin() + b.B * b.N + b.B);
  ga.assign(p.begin() + b.B * b.N + b.B, p.end());
}
std::vector<float> pack(const GB &b) {
  std::vector<float> p(b.alphas);
  p.insert(p.end(), b.beta.begin(), b.beta.end());
  p.insert(p.end(), b.gamma.begin(), b.gamma.end());
  return p;
}

struct Q {
  std::vector<float> oT, vT, mc;
};
Q make_q(const GB &b, unsigned seed) {
  Q q;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  q.oT.resize(2 * b.B);
  q.vT.resize(2 * b.B);
  q.mc.resize(b.B);
  for (int i = 0; i < b.B; ++i) {
    q.oT[2 * i] = U(-1, 1);
    q.oT[2 * i + 1] = U(-1, 1);
    q.vT[2 * i] = U(-1, 1);
    q.vT[2 * i + 1] = U(-1, 1);
    q.mc[i] = U(-0.5f, 0.5f);
  }
  return q;
}

double v2_loss(const GB &b, const std::vector<float> &params, const Q &q) {
  std::vector<float> al, be, ga;
  unpack(b, params, al, be, ga);
  std::vector<float> o(b.o0), v(b.v0), mc(b.B);
  integrate_surrogate_v2(o.data(), v.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
                         al.data(), be.data(), ga.data(), b.rr.data(), b.d_hat.data(), b.dt.data(),
                         b.H.data(), b.B, b.N, b.gp, mc.data(), 1);
  double L = 0.0;
  for (int i = 0; i < b.B; ++i)
    L += (double)q.oT[2 * i] * o[2 * i] + (double)q.oT[2 * i + 1] * o[2 * i + 1] +
         (double)q.vT[2 * i] * v[2 * i] + (double)q.vT[2 * i + 1] * v[2 * i + 1] +
         (double)q.mc[i] * mc[i];
  return L;
}

TEST(NavGeomRolloutGrad, V2GradcheckMatchesFiniteDifference) {
  const GB b = make_batch(31);
  const Q q = make_q(b, 61);
  const int P = num_params(b);
  std::vector<float> base = pack(b), al, be, ga;
  unpack(b, base, al, be, ga);
  std::vector<float> g_al(b.B * b.N, 0.f), g_be(b.B, 0.f), g_ga(b.B, 0.f);
  integrate_surrogate_v2_vjp(b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(),
                             b.mask.data(), al.data(), be.data(), ga.data(), b.rr.data(),
                             b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.gp, q.oT.data(),
                             q.vT.data(), q.mc.data(), g_al.data(), g_be.data(), g_ga.data(), 1);
  std::vector<float> g(g_al);
  g.insert(g.end(), g_be.begin(), g_be.end());
  g.insert(g.end(), g_ga.begin(), g_ga.end());
  double gnorm = 0.0;
  for (float x : g)
    gnorm += (double)x * x;
  gnorm = std::sqrt(gnorm);
  ASSERT_GT(gnorm, 1e-4);
  const float eps = 2e-3f;
  std::vector<float> pp(base), pm(base);
  double gdotg = 0.0;
  for (int i = 0; i < P; ++i) {
    const double u = g[i] / gnorm;
    pp[i] = base[i] + eps * (float)u;
    pm[i] = base[i] - eps * (float)u;
    gdotg += (double)g[i] * u;
  }
  const double dd = (v2_loss(b, pp, q) - v2_loss(b, pm, q)) / (2.0 * eps);
  const double dir_rel = std::fabs(dd - gdotg) / (std::fabs(dd) + std::fabs(gdotg) + 1e-9);
  double worst = 0.0;
  int checked = 0;
  for (int i = 0; i < P; ++i) {
    if (std::fabs(g[i]) < 1e-3)
      continue;
    std::vector<float> plus(base), minus(base);
    plus[i] = base[i] + eps;
    minus[i] = base[i] - eps;
    const double fd = (v2_loss(b, plus, q) - v2_loss(b, minus, q)) / (2.0 * eps);
    worst = std::max(worst, std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6));
    ++checked;
  }
  std::printf("[geom-v2] |g|=%.4f dir_rel=%.3e checked=%d worst=%.3e\n", gnorm, dir_rel, checked,
              worst);
  EXPECT_LT(dir_rel, 2e-2);
  EXPECT_LT(worst, 5e-2);
  EXPECT_GE(checked, 6);
}

// A batch where multi_start's rollout PENETRATES further (weak barriers + a goal
// beyond the nearest obstacle), so min_clear is achieved mid-rollout and the
// penalty has a non-trivial gradient into the coefficients. (When the policy
// steers away, min_clear sits at the fixed data start and the gradient is
// legitimately zero — nothing for the finite difference to check.)
GB make_ms_batch(unsigned seed) {
  GB b;
  b.B = 6;
  b.N = 2;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  const int B = b.B, N = b.N;
  b.gp.margin_factor = 0.5f;
  b.gp.mass = 1.0f;
  b.o0.resize(2 * B);
  b.v0.resize(2 * B);
  b.goal.resize(2 * B);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.assign(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
  b.rr.resize(B);
  b.d_hat.resize(B);
  b.dt.resize(B);
  b.H.resize(B);
  for (int i = 0; i < B; ++i) {
    const float ox = U(-0.1f, 0.1f), oy = U(-0.1f, 0.1f);
    b.o0[2 * i] = ox;
    b.o0[2 * i + 1] = oy;
    b.v0[2 * i] = U(0.0f, 0.15f); // drifting toward the obstacle/goal
    b.v0[2 * i + 1] = U(-0.05f, 0.05f);
    b.goal[2 * i] = ox + 3.0f; // goal well beyond the nearest obstacle
    b.goal[2 * i + 1] = oy + U(-0.2f, 0.2f);
    b.beta[i] = U(1.1f, 1.4f); // strong goal pull
    b.gamma[i] = U(0.2f, 0.35f);
    b.rr[i] = 0.5f;
    b.d_hat[i] = 3.0f;
    b.dt[i] = 0.12f;
    b.H[i] = 3;
    // obstacle 0 = the nearest (o_ms starts near it); obstacle 1 off to the side
    b.C[(i * N + 0) * 2] = ox + 1.0f;
    b.C[(i * N + 0) * 2 + 1] = oy + U(-0.05f, 0.05f);
    b.C[(i * N + 1) * 2] = ox + U(-0.1f, 0.1f);
    b.C[(i * N + 1) * 2 + 1] = oy + 1.3f;
    b.R[i * N + 0] = 0.35f;
    b.R[i * N + 1] = 0.35f;
    b.alphas[i * N + 0] = U(0.010f, 0.020f); // weak barrier -> the agent penetrates
    b.alphas[i * N + 1] = U(0.010f, 0.020f);
  }
  return b;
}

double ms_loss(const GB &b, const std::vector<float> &params, const multi_start_params &mp) {
  std::vector<float> al, be, ga;
  unpack(b, params, al, be, ga);
  return multi_start_penalty(al.data(), be.data(), ga.data(), b.o0.data(), b.v0.data(),
                             b.goal.data(), b.C.data(), b.R.data(), b.mask.data(), b.rr.data(),
                             b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, mp, nullptr,
                             nullptr, nullptr, 1);
}

TEST(NavGeomRolloutGrad, MultiStartGradcheckMatchesFiniteDifference) {
  const GB b = make_ms_batch(17);
  multi_start_params mp;
  mp.margin_factor = 0.5f;
  const int P = num_params(b);
  std::vector<float> base = pack(b), al, be, ga;
  unpack(b, base, al, be, ga);
  std::vector<float> g_al(b.B * b.N, 0.f), g_be(b.B, 0.f), g_ga(b.B, 0.f);
  const double L = multi_start_penalty(al.data(), be.data(), ga.data(), b.o0.data(), b.v0.data(),
                                       b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
                                       b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B,
                                       b.N, mp, g_al.data(), g_be.data(), g_ga.data(), 1);
  ASSERT_TRUE(std::isfinite(L));
  std::vector<float> g(g_al);
  g.insert(g.end(), g_be.begin(), g_be.end());
  g.insert(g.end(), g_ga.begin(), g_ga.end());
  double gnorm = 0.0;
  for (float x : g)
    gnorm += (double)x * x;
  gnorm = std::sqrt(gnorm);
  ASSERT_GT(gnorm, 1e-5);
  // A small FD step: this penetrating config is stiff (large barrier 2nd
  // derivatives) and sits near the min_clear argmin kink, so a smaller step
  // stays local and away from an argmin flip. The directional check over the
  // whole gradient is the robust proof; the per-tensor spot check restricts to
  // large-|g| params where the central FD is well above its conditioning floor.
  const float eps = 5e-4f;
  std::vector<float> pp(base), pm(base);
  double gdotg = 0.0;
  for (int i = 0; i < P; ++i) {
    const double u = g[i] / gnorm;
    pp[i] = base[i] + eps * (float)u;
    pm[i] = base[i] - eps * (float)u;
    gdotg += (double)g[i] * u;
  }
  const double dd = (ms_loss(b, pp, mp) - ms_loss(b, pm, mp)) / (2.0 * eps);
  const double dir_rel = std::fabs(dd - gdotg) / (std::fabs(dd) + std::fabs(gdotg) + 1e-9);
  double worst = 0.0;
  int checked = 0;
  for (int i = 0; i < P; ++i) {
    if (std::fabs(g[i]) < 1.0f) // large-gradient params only (well-conditioned FD)
      continue;
    std::vector<float> plus(base), minus(base);
    plus[i] = base[i] + eps;
    minus[i] = base[i] - eps;
    const double fd = (ms_loss(b, plus, mp) - ms_loss(b, minus, mp)) / (2.0 * eps);
    worst = std::max(worst, std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-4));
    ++checked;
  }
  std::printf("[geom-multistart] L=%.5f |g|=%.4f dir_rel=%.3e checked=%d worst=%.3e\n", L, gnorm,
              dir_rel, checked, worst);
  EXPECT_LT(dir_rel, 2e-2) << "multi_start backward fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a large-gradient coefficient disagrees with finite differences";
}

} // namespace
