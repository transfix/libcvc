/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_geom_rollout_grad_test — finite-difference gradchecks for the P5-P3b
// geometry rollout adjoint (integrate_surrogate_v2_vjp) and the multi-start
// penalty (multi_start_penalty), both w.r.t. the geometry coefficients
// (alphas, beta, gamma), under both integrators and with sampled starts at a
// fixed seed. Torch-independent ground truth, same discipline as the material
// rollout gradcheck. The NavGeomRolloutParity cases then pin the draw (bit for
// bit) and the loss/grads (to float tolerance) against goldens generated from
// GRL-SNAM's surrogate_robust.py by gen_nav_geom_multistart_golden.py.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/nav/geom_rollout.h>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <stdexcept>
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

void v2_gradcheck(bool semi_implicit) {
  GB b = make_batch(31);
  b.gp.semi_implicit = semi_implicit;
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
  std::printf("[geom-v2 %s] |g|=%.4f dir_rel=%.3e checked=%d worst=%.3e\n",
              semi_implicit ? "semi" : "explicit", gnorm, dir_rel, checked, worst);
  EXPECT_LT(dir_rel, 2e-2);
  EXPECT_LT(worst, 5e-2);
  EXPECT_GE(checked, 6);
}

TEST(NavGeomRolloutGrad, V2GradcheckMatchesFiniteDifference) { v2_gradcheck(false); }

// GRL-SNAM's integrator: v first, then the position steps with v_{n+1}. A
// different adjoint (v_{n+1} collects the position step's adjoint), so its own
// check.
TEST(NavGeomRolloutGrad, V2SemiImplicitGradcheckMatchesFiniteDifference) { v2_gradcheck(true); }

// The two integrators really differ: same inputs, different trajectory.
TEST(NavGeomRolloutGrad, SemiImplicitIsADifferentIntegrator) {
  GB b = make_batch(31);
  std::vector<float> o1(b.o0), v1(b.v0), mc1(b.B), o2(b.o0), v2(b.v0), mc2(b.B);
  integrate_surrogate_v2(o1.data(), v1.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
                         b.alphas.data(), b.beta.data(), b.gamma.data(), b.rr.data(),
                         b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.gp, mc1.data(), 1);
  b.gp.semi_implicit = true;
  integrate_surrogate_v2(o2.data(), v2.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
                         b.alphas.data(), b.beta.data(), b.gamma.data(), b.rr.data(),
                         b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.gp, mc2.data(), 1);
  double dmax = 0.0;
  for (int i = 0; i < 2 * b.B; ++i)
    dmax = std::max(dmax, (double)std::fabs(o1[i] - o2[i]));
  EXPECT_GT(dmax, 1e-4);
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

struct MSOut {
  double L = 0.0;
  std::vector<float> g_al, g_be, g_ga;
};

MSOut ms_eval(const GB &b, const multi_start_params &mp, int num_threads = 1) {
  MSOut r;
  r.g_al.assign(b.B * b.N, 0.f);
  r.g_be.assign(b.B, 0.f);
  r.g_ga.assign(b.B, 0.f);
  r.L = multi_start_penalty(b.alphas.data(), b.beta.data(), b.gamma.data(), b.o0.data(),
                            b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
                            b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, mp,
                            r.g_al.data(), r.g_be.data(), r.g_ga.data(), num_threads);
  return r;
}

bool bit_equal(const MSOut &a, const MSOut &b) {
  return a.L == b.L && a.g_al == b.g_al && a.g_be == b.g_be && a.g_ga == b.g_ga;
}

// The sampled-start settings GRL-SNAM #113 defaults to (frac ~ U(0.8, 0.98)),
// at a small ms_count and a FIXED seed: the starts are then constants of the
// coefficients, so the central finite difference sees the same draw the
// analytic gradient did.
multi_start_params sampled(bool semi_implicit, std::uint32_t seed = 11) {
  multi_start_params mp;
  mp.ms_count = 4;
  mp.frac_lo = 0.8;
  mp.frac_hi = 0.98;
  mp.seed = seed;
  mp.semi_implicit = semi_implicit;
  return mp;
}

void ms_gradcheck(const multi_start_params &mp, const char *tag) {
  const GB b = make_ms_batch(17);
  const int P = num_params(b);
  const std::vector<float> base = pack(b);
  const MSOut a = ms_eval(b, mp);
  ASSERT_TRUE(std::isfinite(a.L));
  std::vector<float> g(a.g_al);
  g.insert(g.end(), a.g_be.begin(), a.g_be.end());
  g.insert(g.end(), a.g_ga.begin(), a.g_ga.end());
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
  std::printf("[geom-multistart %s] L=%.5f |g|=%.4f dir_rel=%.3e checked=%d worst=%.3e\n", tag, a.L,
              gnorm, dir_rel, checked, worst);
  EXPECT_LT(dir_rel, 2e-2) << "multi_start backward fails the directional FD check";
  EXPECT_LT(worst, 5e-2) << "a large-gradient coefficient disagrees with finite differences";
  EXPECT_GE(checked, 4);
}

TEST(NavGeomRolloutGrad, MultiStartGradcheckMatchesFiniteDifference) {
  ms_gradcheck(multi_start_params{}, "legacy explicit");
}

// Semi-implicit applies each step's acceleration at once, so on this batch at
// the explicit-tuned dt' = 4*dt its starts land deep in the barrier's stiff,
// kinked band: there the central FD only settles onto the analytic gradient
// below eps ~ 2e-5 (an eps sweep converges to it param by param), too fine for
// a float rollout across the board. dt' = 2.5*dt keeps the same physics in the
// well-conditioned range. The 4*dt semi-implicit gradient itself is pinned
// against torch autograd by NavGeomRolloutParity.
multi_start_params semi_for_fd(multi_start_params mp) {
  mp.semi_implicit = true;
  mp.ms_dt_mult = 2.5f;
  return mp;
}

TEST(NavGeomRolloutGrad, MultiStartSemiImplicitGradcheckMatchesFiniteDifference) {
  ms_gradcheck(semi_for_fd(multi_start_params{}), "legacy semi");
}

TEST(NavGeomRolloutGrad, MultiStartSampledGradcheckMatchesFiniteDifference) {
  ms_gradcheck(sampled(false), "sampled explicit");
}

TEST(NavGeomRolloutGrad, MultiStartSampledSemiImplicitGradcheckMatchesFiniteDifference) {
  ms_gradcheck(semi_for_fd(sampled(true)), "sampled semi");
}

// ── sampling semantics ──────────────────────────────────────────────────────

// A fixed seed is a fixed function: bit-identical loss and grads on a repeat
// call, and across thread counts (the K*B rollouts are independent agents and
// the reduction over the K copies is sequential).
TEST(NavGeomRolloutGrad, MultiStartSameSeedIsBitIdentical) {
  const GB b = make_ms_batch(17);
  for (bool semi : {false, true}) {
    const MSOut a = ms_eval(b, sampled(semi), 1);
    EXPECT_TRUE(bit_equal(a, ms_eval(b, sampled(semi), 1)));
    EXPECT_TRUE(bit_equal(a, ms_eval(b, sampled(semi), 4)));
  }
}

TEST(NavGeomRolloutGrad, MultiStartSeedChangesTheDraw) {
  const GB b = make_ms_batch(17);
  const MSOut a = ms_eval(b, sampled(false, 11)), c = ms_eval(b, sampled(false, 12));
  EXPECT_NE(a.L, c.L);
  // ... and sampling moves the value off the legacy single start.
  EXPECT_NE(a.L, ms_eval(b, multi_start_params{}).L);
}

// A degenerate range draws nothing, so the seed is moot and ms_count identical
// starts are the legacy single start — bit for bit, the pre-sampling port.
TEST(NavGeomRolloutGrad, MultiStartDegenerateRangeIsTheLegacyStart) {
  const GB b = make_ms_batch(17);
  for (bool semi : {false, true}) {
    multi_start_params legacy;
    legacy.semi_implicit = semi;
    multi_start_params many = legacy;
    many.ms_count = 10;
    many.seed = 99;
    EXPECT_TRUE(bit_equal(ms_eval(b, legacy), ms_eval(b, many)));
  }
}

TEST(NavGeomRolloutGrad, MultiStartZeroCountIsZero) {
  const GB b = make_ms_batch(17);
  multi_start_params mp = sampled(false);
  mp.ms_count = 0;
  const MSOut a = ms_eval(b, mp);
  EXPECT_EQ(a.L, 0.0);
  for (float x : a.g_be)
    EXPECT_EQ(x, 0.0f);
}

TEST(NavGeomRolloutGrad, MultiStartRejectsABadFracRange) {
  const GB b = make_ms_batch(17);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double bad[][2] = {{0.9, 0.8}, {-0.1, 0.5}, {0.5, 1.0}, {nan, 0.5}};
  for (const auto &r : bad) {
    multi_start_params mp;
    mp.frac_lo = r[0];
    mp.frac_hi = r[1];
    EXPECT_THROW(ms_eval(b, mp), std::invalid_argument) << r[0] << ", " << r[1];
    std::vector<float> f(b.B);
    EXPECT_THROW(cvc::nav::multi_start_fracs(mp, b.B, f.data()), std::invalid_argument);
  }
}

// An agent whose obstacle slots are all padding has no clearance to score. It
// used to start at o0 - frac * inf * 0 = NaN, which the adjoint spread into its
// beta/gamma grads (0 * NaN), poisoning the whole weight gradient downstream.
TEST(NavGeomRolloutGrad, MultiStartAgentWithNoObstacleContributesZero) {
  GB b = make_ms_batch(17);
  const MSOut full = ms_eval(b, multi_start_params{});
  for (int j = 0; j < b.N; ++j)
    b.mask[2 * b.N + j] = 0; // agent 2: no valid obstacle
  for (const multi_start_params &mp : {multi_start_params{}, sampled(false), sampled(true)}) {
    const MSOut a = ms_eval(b, mp);
    ASSERT_TRUE(std::isfinite(a.L));
    for (float x : a.g_al)
      ASSERT_TRUE(std::isfinite(x));
    for (int i = 0; i < b.B; ++i) {
      ASSERT_TRUE(std::isfinite(a.g_be[i]) && std::isfinite(a.g_ga[i])) << "agent " << i;
    }
    EXPECT_EQ(a.g_be[2], 0.0f);
    EXPECT_EQ(a.g_ga[2], 0.0f);
    for (int j = 0; j < b.N; ++j)
      EXPECT_EQ(a.g_al[2 * b.N + j], 0.0f);
  }
  // The other agents are untouched: the legacy grads match the full batch's.
  const MSOut masked = ms_eval(b, multi_start_params{});
  EXPECT_EQ(masked.g_be[0], full.g_be[0]);
  EXPECT_EQ(masked.g_ga[4], full.g_ga[4]);
}

// ── parity with the Python reference (GRL-SNAM grl_snam/surrogate_robust.py) ─

// clang-format off
// ---- BEGIN GENERATED by gen_nav_geom_multistart_golden.py (do not edit) ----
constexpr int kPB = 5, kPN = 3;
constexpr unsigned kPSeed = 7u;
const float kPO0[] = {0.0f, 0.0f, 0.0500000007f, -0.0799999982f, -0.0599999987f, 0.0299999993f, 0.0f, 0.0f, 0.200000003f, 0.100000001f};
const float kPV0[] = {0.100000001f, 0.0f, 0.119999997f, 0.0299999993f, 0.0500000007f, -0.0199999996f, 0.0199999996f, 0.00999999978f, 0.100000001f, 0.100000001f};
const float kPGoal[] = {3.0f, 0.100000001f, 3.04999995f, -0.230000004f, 2.74000001f, 0.230000004f, 3.0f, 0.0f, 3.20000005f, 3.0999999f};
const float kPC[] = {1.0f, 0.0199999996f, 0.0500000007f, 1.29999995f, 9.0f, 9.0f, 1.04999995f, -0.100000001f, 0.0f, 1.25f, 9.0f, 9.0f, 0.949999988f, 0.0500000007f, -0.100000001f, -1.29999995f, 9.0f, 9.0f, 0.5f, 0.0f, 0.0f, 1.39999998f, 9.0f, 9.0f, 1.0f, 0.0f, 0.0f, 1.0f, 9.0f, 9.0f};
const float kPR[] = {0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f, 0.349999994f};
const float kPAlphas[] = {0.0120000001f, 0.0179999992f, 0.5f, 0.0149999997f, 0.0109999999f, 0.5f, 0.0189999994f, 0.0140000004f, 0.5f, 0.0160000008f, 0.0130000003f, 0.5f, 0.0170000009f, 0.0120000001f, 0.5f};
const float kPBeta[] = {1.20000005f, 1.29999995f, 1.14999998f, 1.25f, 1.10000002f};
const float kPGamma[] = {0.25f, 0.300000012f, 0.219999999f, 0.280000001f, 0.200000003f};
const float kPRr[] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
const float kPDHat[] = {3.0f, 3.0f, 3.0f, 3.0f, 3.0f};
const float kPDt[] = {0.119999997f, 0.119999997f, 0.119999997f, 0.119999997f, 0.119999997f};
const std::uint8_t kPMask[] = {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0};
const int kPH[] = {3, 3, 3, 3, 3};
// 4 starts x 5 agents, frac (0.8, 0.98), seed kPSeed: float bit patterns
const std::uint32_t kPFracBits[] = {0x3f657301u, 0x3f55f5fbu, 0x3f6b2d2eu, 0x3f6b11cbu, 0x3f578691u, 0x3f606308u, 0x3f5657b0u, 0x3f69d179u, 0x3f5da23fu, 0x3f7406c6u, 0x3f74321fu, 0x3f662fe6u, 0x3f5a047eu, 0x3f564ea5u, 0x3f614f4cu, 0x3f5d5b1cu, 0x3f6dfec1u, 0x3f502a8eu, 0x3f797e70u, 0x3f51c44du};
struct parity_golden {
  bool semi_implicit;
  int ms_count;
  double frac_lo, frac_hi;
  double L;
  float g_alphas[15], g_beta[5], g_gamma[5];
};
const parity_golden kPGoldens[] = {
    // kLegacyExplicit
    {false, 1, 0.9, 0.9, 2.8691627979278564,
     {-181.279419f, 1.10982263f, 0.0f, -178.573105f, 2.68028212f, 0.0f, 0.0f, 0.0f, -0.0f, -182.976196f, -0.543673635f, 0.0f, -0.0f, -0.0f, -0.0f},
     {2.3985579f, 2.35408306f, -0.0f, 2.70347333f, -0.0f},
     {-0.0903934091f, -0.108409889f, 0.0f, -0.0190507062f, 0.0f}},
    // kLegacySemi
    {true, 1, 0.9, 0.9, 4.6118292808532715,
     {-195.896484f, -37.0686569f, 0.0f, -397.554565f, -12.4150867f, 0.0f, -136.760315f, -4.15582752f, 0.0f, -312.563599f, -34.6037178f, 0.0f, -0.0f, -0.0f, -0.0f},
     {2.43219209f, 4.87806988f, 2.90345573f, 4.38248062f, -0.0f},
     {-0.148410276f, -0.397356302f, 0.261367708f, -0.15759325f, 0.0f}},
    // kSampledExplicit
    {false, 4, 0.8, 0.98, 3.232287883758545,
     {-177.756607f, 1.05290103f, 0.0f, -160.033096f, 2.4824965f, 0.0f, -22.1830959f, 0.38263303f, 0.0f, -182.975204f, -0.544004381f, 0.0f, -0.0f, -0.0f, -0.0f},
     {2.40130663f, 2.39441609f, 0.409636557f, 2.70350742f, -0.0f},
     {-0.0904545486f, -0.110411569f, -0.00863776542f, -0.0190505348f, 0.0f}},
    // kSampledSemi
    {true, 4, 0.8, 0.98, 4.854466915130615,
     {-146.977844f, -39.3948479f, 0.0f, -230.308655f, -27.4902897f, 0.0f, -38.1005402f, -4.31770325f, 0.0f, -312.589325f, -34.6005402f, 0.0f, -0.0f, -0.0f, -0.0f},
     {1.79187512f, 2.53213358f, 1.18654704f, 4.38297367f, -0.0f},
     {-0.0860269815f, -0.266419828f, 0.162369326f, -0.157629848f, 0.0f}},
};
// torch 2.8.0+cpu
// ---- END GENERATED ----
// clang-format on

GB parity_batch() {
  GB b;
  b.B = kPB;
  b.N = kPN;
  const int B = kPB, N = kPN;
  b.o0.assign(kPO0, kPO0 + 2 * B);
  b.v0.assign(kPV0, kPV0 + 2 * B);
  b.goal.assign(kPGoal, kPGoal + 2 * B);
  b.C.assign(kPC, kPC + 2 * B * N);
  b.R.assign(kPR, kPR + B * N);
  b.mask.assign(kPMask, kPMask + B * N);
  b.alphas.assign(kPAlphas, kPAlphas + B * N);
  b.beta.assign(kPBeta, kPBeta + B);
  b.gamma.assign(kPGamma, kPGamma + B);
  b.rr.assign(kPRr, kPRr + B);
  b.d_hat.assign(kPDHat, kPDHat + B);
  b.dt.assign(kPDt, kPDt + B);
  b.H.assign(kPH, kPH + B);
  return b;
}

// The draw is torch.rand's, bit for bit: the same seed reproduces the Python's
// start fractions exactly (start-major, B per start).
TEST(NavGeomRolloutParity, FracsMatchTorchDraw) {
  multi_start_params mp;
  mp.ms_count = 4;
  mp.frac_lo = 0.8;
  mp.frac_hi = 0.98;
  mp.seed = kPSeed;
  constexpr int n = static_cast<int>(sizeof(kPFracBits) / sizeof(kPFracBits[0]));
  static_assert(n == 4 * kPB, "golden is 4 starts x kPB agents");
  std::vector<float> f(n);
  cvc::nav::multi_start_fracs(mp, kPB, f.data());
  for (int i = 0; i < n; ++i) {
    std::uint32_t u;
    std::memcpy(&u, &f[i], sizeof u);
    EXPECT_EQ(u, kPFracBits[i]) << "frac " << i;
  }
}

// Loss and coefficient grads against torch autograd on GRL-SNAM's own function
// (semi-implicit) and on it with the fork's explicit update order. The scene
// covers penetrating starts, the feasibility fallback (an agent already inside
// an obstacle) and an agent with no valid obstacle. Float32 on both sides with
// different reduction orders, so a tolerance rather than bit equality.
TEST(NavGeomRolloutParity, MultiStartMatchesPythonReference) {
  const GB b = parity_batch();
  for (const parity_golden &gold : kPGoldens) {
    multi_start_params mp;
    mp.ms_count = gold.ms_count;
    mp.frac_lo = gold.frac_lo;
    mp.frac_hi = gold.frac_hi;
    mp.seed = kPSeed;
    mp.semi_implicit = gold.semi_implicit;
    const MSOut a = ms_eval(b, mp);
    const char *tag = gold.semi_implicit ? "semi" : "explicit";
    EXPECT_NEAR(a.L, gold.L, 1e-5 * std::fabs(gold.L)) << tag << " ms_count=" << gold.ms_count;
    std::vector<float> g(a.g_al), want(gold.g_alphas, gold.g_alphas + kPB * kPN);
    g.insert(g.end(), a.g_be.begin(), a.g_be.end());
    g.insert(g.end(), a.g_ga.begin(), a.g_ga.end());
    want.insert(want.end(), gold.g_beta, gold.g_beta + kPB);
    want.insert(want.end(), gold.g_gamma, gold.g_gamma + kPB);
    double gmax = 0.0, dot = 0.0, n1 = 0.0, n2 = 0.0, worst = 0.0;
    for (float x : want)
      gmax = std::max(gmax, (double)std::fabs(x));
    for (std::size_t i = 0; i < g.size(); ++i) {
      dot += (double)g[i] * want[i];
      n1 += (double)g[i] * g[i];
      n2 += (double)want[i] * want[i];
      worst = std::max(worst, std::fabs((double)g[i] - want[i]) /
                                  (std::fabs((double)want[i]) + 1e-4 * gmax));
    }
    const double cos = dot / std::sqrt(n1 * n2);
    std::printf("[geom-parity %s K=%d] L=%.9g (py %.9g) cos=%.9f worst_rel=%.3e\n", tag,
                gold.ms_count, a.L, gold.L, cos, worst);
    EXPECT_GT(cos, 1.0 - 1e-6) << tag << " ms_count=" << gold.ms_count;
    EXPECT_LT(worst, 1e-3) << tag << " ms_count=" << gold.ms_count;
  }
}

} // namespace
