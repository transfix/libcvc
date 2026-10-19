/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_material_bike_rollout_test: the material rollout on the kinematic bicycle
// (integrate_bicycle_material) and its adjoint.
//
// The load-bearing test is a central finite-difference gradcheck of the
// hand-written adjoint against a numeric gradient of a scalar loss over every
// differentiable output (oT, thT, spT, min_clear, cum_risk, arc_length). It is
// torch-independent ground truth, the same discipline as the surrogate's
// gradcheck. The bicycle has real branches (behind, the governors, the clamps),
// so the batch sits at interior points:
//   - agents start heading roughly at the goal, below the speed limit;
//   - obstacle clearance stays inside (eps, d_hat);
//   - the patch samples stay inside their clamps;
//   - the horizon is short, so no branch flips under the perturbation.
// The behaviour tests check what the controller is for: it drives to the goal,
// respects its limits, keeps clear of an obstacle, and the soft material force
// steers it off risk.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/material.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using cvc::nav::bicycle_material_params;
using cvc::nav::integrate_bicycle_material;
using cvc::nav::integrate_bicycle_material_vjp;
using cvc::nav::surrogate_material_params;

namespace {

struct Batch {
  int B = 6, N = 3, P = 13;
  std::vector<float> o0, th0, sp0, goal, C, R, alphas, beta, gamma, lam_soft, lam_hard, patch, rr,
      d_hat, dt;
  std::vector<std::uint8_t> mask;
  std::vector<int> H;
  surrogate_material_params p;
  bicycle_material_params v;
  // The dynamic vehicle (dyn_batch): the same controller driving dyn_plant, x0 its
  // further state (vy, r, delta, a), sp0 its longitudinal speed.
  bool dyn = false;
  cvc::nav::diff::dyn_bike_params dp;
  std::vector<float> x0;
};

// Gentle smooth patch channels: risk in (0,1), phi in (0,50), small gradients.
void fill_patch(Batch &b, int i) {
  const int P = b.P;
  for (int r = 0; r < P; ++r)
    for (int c = 0; c < P; ++c) {
      const float fx = (float)c / (P - 1), fy = (float)r / (P - 1);
      const size_t base = ((size_t)i * 6) * P * P + (size_t)r * P + c, pp = (size_t)P * P;
      b.patch[base + 0 * pp] = 0.35f + 0.25f * std::sin(2.0f * fx + 1.3f * fy);
      b.patch[base + 1 * pp] = 2.0f + 1.2f * std::cos(1.7f * fx - 0.9f * fy);
      b.patch[base + 2 * pp] = 0.20f * std::cos(2.1f * fx);
      b.patch[base + 3 * pp] = 0.20f * std::sin(1.5f * fy);
      b.patch[base + 4 * pp] = 0.15f * std::sin(1.1f * fx + fy);
      b.patch[base + 5 * pp] = 0.15f * std::cos(0.8f * fx - fy);
    }
}

Batch make_batch(unsigned seed, int B = 6, int N = 3) {
  Batch b;
  b.B = B;
  b.N = N;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  b.v.L = 0.3f;
  b.v.delta_max = 0.6f;
  // Limits and gains sized so the command stays inside its clamps: a saturated
  // clamp passes no gradient, which is right but leaves nothing to check.
  b.v.a_max = 4.0f;
  b.v.a_lat_max = 1.5f;
  b.v.k_steer = 0.8f;
  b.v.vmax = 1.5f;
  b.o0.resize(B * 2);
  b.th0.resize(B);
  b.sp0.resize(B);
  b.goal.resize(B * 2);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.assign(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
  b.lam_soft.resize(B);
  b.lam_hard.resize(B);
  b.patch.assign((size_t)B * 6 * b.P * b.P, 0.0f);
  b.rr.assign(B, 0.5f);
  b.d_hat.assign(B, 3.0f);
  b.dt.assign(B, 0.1f);
  b.H.resize(B);
  for (int i = 0; i < B; ++i) {
    b.o0[2 * i] = U(-0.5f, 0.5f);
    b.o0[2 * i + 1] = U(-0.5f, 0.5f);
    b.goal[2 * i] = U(1.5f, 2.5f);
    b.goal[2 * i + 1] = U(1.5f, 2.5f);
    b.th0[i] = std::atan2(b.goal[2 * i + 1] - b.o0[2 * i + 1], b.goal[2 * i] - b.o0[2 * i]) +
               U(-0.2f, 0.2f);
    b.sp0[i] = U(0.3f, 0.6f);
    b.beta[i] = U(0.8f, 1.2f);
    b.gamma[i] = U(0.2f, 0.5f);
    b.lam_soft[i] = U(0.7f, 1.3f);
    b.lam_hard[i] = U(0.7f, 1.3f);
    b.H[i] = (i % 2 == 0) ? 2 : 3; // short + ragged (min_clear at frozen steps)
    for (int j = 0; j < N; ++j) {
      const float ang = U(0.0f, 6.283185f), dist = U(1.5f, 1.9f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(0.5f, 0.7f);
      b.alphas[i * N + j] = U(0.03f, 0.08f);
    }
    fill_patch(b, i);
  }
  return b;
}

struct outputs {
  std::vector<float> o, th, sp, minc, cum, hard, arc, x;
};

outputs run(const Batch &b, const std::vector<float> &al, const std::vector<float> &be,
            const std::vector<float> &ga, const std::vector<float> &ls,
            const std::vector<float> &lh, int threads = 1) {
  outputs out;
  out.o = b.o0;
  out.th = b.th0;
  out.sp = b.sp0;
  out.minc.resize(b.B);
  out.cum.resize(b.B);
  out.hard.resize(b.B);
  out.arc.resize(b.B);
  if (b.dyn) {
    out.x = b.x0;
    cvc::nav::integrate_dynamic_material(
        out.o.data(), out.th.data(), out.sp.data(), out.x.data(), b.goal.data(), b.C.data(),
        b.R.data(), b.mask.data(), al.data(), be.data(), ga.data(), ls.data(), lh.data(),
        b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P,
        b.p, b.v, b.dp, out.minc.data(), out.cum.data(), out.hard.data(), out.arc.data(), threads);
    return out;
  }
  integrate_bicycle_material(out.o.data(), out.th.data(), out.sp.data(), b.goal.data(), b.C.data(),
                             b.R.data(), b.mask.data(), al.data(), be.data(), ga.data(), ls.data(),
                             lh.data(), b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(),
                             b.H.data(), b.B, b.N, b.P, b.P, b.p, b.v, out.minc.data(),
                             out.cum.data(), out.hard.data(), out.arc.data(), threads);
  return out;
}

// Fixed random weights on every differentiable output.
struct seeds {
  std::vector<float> o, th, sp, minc, cum, arc, x;
};

seeds make_seeds(int B, unsigned seed) {
  std::mt19937 rng(seed);
  auto U = [&]() { return (float)std::uniform_real_distribution<double>(-1.0, 1.0)(rng); };
  seeds q;
  for (int i = 0; i < 2 * B; ++i)
    q.o.push_back(U());
  for (int i = 0; i < B; ++i) {
    q.th.push_back(U());
    q.sp.push_back(U());
    q.minc.push_back(U());
    q.cum.push_back(U());
    q.arc.push_back(U());
  }
  for (int i = 0; i < 4 * B; ++i) // after the rest, so the bicycle's draws are unchanged
    q.x.push_back(U());
  return q;
}

double scalar_loss(const outputs &y, const seeds &q, int B) {
  double L = 0.0;
  for (int i = 0; i < 2 * B; ++i)
    L += (double)q.o[i] * y.o[i];
  for (int i = 0; i < B; ++i)
    L += (double)q.th[i] * y.th[i] + (double)q.sp[i] * y.sp[i] + (double)q.minc[i] * y.minc[i] +
         (double)q.cum[i] * y.cum[i] + (double)q.arc[i] * y.arc[i];
  for (size_t i = 0; i < y.x.size(); ++i) // the dynamic vehicle's further state
    L += (double)q.x[i] * y.x[i];
  return L;
}

// [alphas | beta | gamma | lam_soft | lam_hard]
std::vector<float> pack(const Batch &b) {
  std::vector<float> p = b.alphas;
  p.insert(p.end(), b.beta.begin(), b.beta.end());
  p.insert(p.end(), b.gamma.begin(), b.gamma.end());
  p.insert(p.end(), b.lam_soft.begin(), b.lam_soft.end());
  p.insert(p.end(), b.lam_hard.begin(), b.lam_hard.end());
  return p;
}

double loss_at(const Batch &b, const std::vector<float> &p, const seeds &q) {
  const int BN = b.B * b.N, B = b.B;
  const std::vector<float> al(p.begin(), p.begin() + BN), be(p.begin() + BN, p.begin() + BN + B),
      ga(p.begin() + BN + B, p.begin() + BN + 2 * B),
      ls(p.begin() + BN + 2 * B, p.begin() + BN + 3 * B),
      lh(p.begin() + BN + 3 * B, p.begin() + BN + 4 * B);
  return scalar_loss(run(b, al, be, ga, ls, lh), q, B);
}

std::vector<float> analytic_grad(const Batch &b, const seeds &q, int threads = 1) {
  const int B = b.B, N = b.N;
  std::vector<float> gal(B * N, 0.0f), gbe(B, 0.0f), gga(B, 0.0f), gls(B, 0.0f), glh(B, 0.0f);
  if (b.dyn)
    cvc::nav::integrate_dynamic_material_vjp(
        b.o0.data(), b.th0.data(), b.sp0.data(), b.x0.data(), b.goal.data(), b.C.data(), b.R.data(),
        b.mask.data(), b.alphas.data(), b.beta.data(), b.gamma.data(), b.lam_soft.data(),
        b.lam_hard.data(), b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), B,
        N, b.P, b.P, b.p, b.v, b.dp, q.o.data(), q.th.data(), q.sp.data(), q.x.data(),
        q.minc.data(), q.cum.data(), q.arc.data(), gal.data(), gbe.data(), gga.data(), gls.data(),
        glh.data(), threads);
  else
    integrate_bicycle_material_vjp(
        b.o0.data(), b.th0.data(), b.sp0.data(), b.goal.data(), b.C.data(), b.R.data(),
        b.mask.data(), b.alphas.data(), b.beta.data(), b.gamma.data(), b.lam_soft.data(),
        b.lam_hard.data(), b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), B,
        N, b.P, b.P, b.p, b.v, q.o.data(), q.th.data(), q.sp.data(), q.minc.data(), q.cum.data(),
        q.arc.data(), gal.data(), gbe.data(), gga.data(), gls.data(), glh.data(), threads);
  std::vector<float> g = gal;
  g.insert(g.end(), gbe.begin(), gbe.end());
  g.insert(g.end(), gga.begin(), gga.end());
  g.insert(g.end(), gls.begin(), gls.end());
  g.insert(g.end(), glh.begin(), glh.end());
  return g;
}

// `eps` is the directional check's step along the unit gradient. The dynamic batches use
// 1e-3: at 2e-3 the step crosses one of the controller's branches. (dyn-short, along the
// gradient: fd 26.77 / 30.27 / 33.4698 / 33.4698 against 33.4699 analytic at eps 4e-3 /
// 2e-3 / 1e-3 / 5e-4; the bicycle's "short" batch drifts the same way, 12.29 / 12.65 /
// 12.976 against 12.975.)
void gradcheck(const Batch &b, const char *tag, float eps = 2e-3f) {
  const seeds q = make_seeds(b.B, 99);
  const std::vector<float> base = pack(b);
  const std::vector<float> g = analytic_grad(b, q);
  const int P = static_cast<int>(base.size());
  double gnorm = 0.0;
  for (float x : g)
    gnorm += (double)x * x;
  gnorm = std::sqrt(gnorm);
  ASSERT_GT(gnorm, 1e-4);

  // Directional check along the gradient.
  std::vector<float> pp(P), pm(P);
  double gdotg = 0.0;
  for (int i = 0; i < P; ++i) {
    const double u = g[i] / gnorm;
    pp[i] = base[i] + eps * (float)u;
    pm[i] = base[i] - eps * (float)u;
    gdotg += (double)g[i] * u;
  }
  const double dd_fd = (loss_at(b, pp, q) - loss_at(b, pm, q)) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gdotg) / (std::fabs(dd_fd) + std::fabs(gdotg) + 1e-9);

  // Per-coefficient checks above the FD noise floor (about 1e-7 * |L| / eps).
  //
  // The controller has real discontinuities: "behind", the hard-steer floor, and the
  // approach cut-off in the stop governor. A finite difference that straddles one
  // measures a jump, not a slope, so a coefficient whose forward and backward
  // differences disagree is skipped and counted, never compared. (Measured on the
  // no-reverse batch: one coefficient's forward difference jumps between +1e-3
  // and +2e-3, while every difference at 1e-3 and below converges on the analytic
  // value.)
  const float e1 = 1e-3f;
  double worst_rel = 0.0;
  int checked = 0, kinked = 0, worst_i = -1;
  const double L0 = loss_at(b, base, q);
  for (int i = 0; i < P; ++i) {
    if (std::fabs(g[i]) < 5e-3)
      continue;
    std::vector<float> plus = base, minus = base;
    plus[i] = base[i] + e1;
    minus[i] = base[i] - e1;
    const double lp = loss_at(b, plus, q), lm = loss_at(b, minus, q);
    const double fwd = (lp - L0) / e1, bwd = (L0 - lm) / e1;
    if (std::fabs(fwd - bwd) > 0.1 * (std::fabs(fwd) + std::fabs(bwd)) + 1e-3) {
      ++kinked;
      continue;
    }
    const double fd = 0.5 * (fwd + bwd);
    const double rel = std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6);
    if (rel > worst_rel) {
      worst_rel = rel;
      worst_i = i;
    }
    ++checked;
  }
  std::printf("[mat-bike-gradcheck %s] params=%d |g|=%.4f dir_rel=%.3e checked=%d kinked=%d "
              "worst_rel=%.3e (param %d)\n",
              tag, P, gnorm, dir_rel, checked, kinked, worst_rel, worst_i);
  EXPECT_LT(dir_rel, 2e-2) << "the bicycle material adjoint fails the directional FD check";
  EXPECT_LT(worst_rel, 5e-2) << "a coefficient gradient disagrees with finite differences";
  EXPECT_GE(checked, 8) << "too few coefficients cleared the FD noise floor";
  EXPECT_LE(kinked, checked / 4) << "too many coefficients sit on a controller discontinuity";
}

// The fixture's vehicle on the dynamic plant: its 0.3 wheelbase, a unit mass with its
// inertia mid-wheelbase, grip (mu g 8.8) above the controller's 4.0 and 1.5, short lags.
Batch dyn_batch(unsigned seed, int B = 6, int N = 3) {
  Batch b = make_batch(seed, B, N);
  b.dyn = true;
  cvc::nav::diff::dyn_bike_params &d = b.dp;
  d.m = 1.0f;
  d.lf = d.lr = 0.5f * b.v.L;
  d.Iz = d.m * d.lf * d.lr;
  d.h = 0.05f;
  d.g = 9.81f;
  d.mu = 0.9f;
  d.B_f = 10.0f;
  d.B_r = 12.0f;
  d.C = 1.2f;
  d.front_drive = 0.0f;
  d.front_brake = 0.6f;
  d.tau_steer = 0.05f;
  d.tau_drive = 0.05f;
  d.eps = 0.05f;
  d.v_blend = 0.1f;
  d.v_width = 0.03f;
  b.x0.assign(static_cast<size_t>(B) * 4, 0.0f);
  return b;
}

} // namespace

TEST(NavMaterialBikeRollout, GradcheckMatchesFiniteDifference) {
  gradcheck(make_batch(7), "short");
}

TEST(NavMaterialBikeRollout, GradcheckWithoutReverse) {
  Batch b = make_batch(11);
  b.v.allow_reverse = false;
  for (int &h : b.H)
    h += 2; // longer horizons
  gradcheck(b, "no-reverse");
}

TEST(NavMaterialBikeRollout, ThreadDeterminism) {
  const Batch b = make_batch(3, 32, 4);
  const outputs a = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard, 1);
  const outputs c = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard, 8);
  EXPECT_EQ(a.o, c.o);
  EXPECT_EQ(a.th, c.th);
  EXPECT_EQ(a.sp, c.sp);
  EXPECT_EQ(a.minc, c.minc);
  EXPECT_EQ(a.cum, c.cum);
  const seeds q = make_seeds(b.B, 5);
  EXPECT_EQ(analytic_grad(b, q, 1), analytic_grad(b, q, 8));
}

// With nothing in the way and no material, the controller drives to the goal and
// never leaves its speed range.
TEST(NavMaterialBikeRollout, DrivesToTheGoalWithinItsLimits) {
  Batch b = make_batch(21, 8, 1);
  std::fill(b.mask.begin(), b.mask.end(), 0);
  std::fill(b.lam_soft.begin(), b.lam_soft.end(), 0.0f);
  std::fill(b.lam_hard.begin(), b.lam_hard.end(), 0.0f);
  for (int i = 0; i < b.B; ++i)
    b.th0[i] += (i % 2 ? 1.0f : -1.0f) * 1.2f; // start well off the goal bearing
  std::fill(b.H.begin(), b.H.end(), 80);
  // Step one at a time to watch the speed.
  outputs st = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard);
  Batch one = b;
  std::fill(one.H.begin(), one.H.end(), 1);
  outputs y{b.o0, b.th0, b.sp0, {}, {}, {}, {}};
  for (int s = 0; s < 80; ++s) {
    one.o0 = y.o;
    one.th0 = y.th;
    one.sp0 = y.sp;
    y = run(one, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard);
    for (int i = 0; i < b.B; ++i) {
      ASSERT_LE(y.sp[i], b.v.vmax + 1e-6f);
      ASSERT_GE(y.sp[i], -0.25f * b.v.vmax - 1e-6f);
    }
  }
  for (int i = 0; i < b.B; ++i) {
    const float d0 = std::hypot(b.goal[2 * i] - b.o0[2 * i], b.goal[2 * i + 1] - b.o0[2 * i + 1]);
    const float d1 = std::hypot(b.goal[2 * i] - st.o[2 * i], b.goal[2 * i + 1] - st.o[2 * i + 1]);
    EXPECT_LT(d1, 0.25f * d0) << "agent " << i << " did not close on its goal";
  }
}

// An obstacle on the straight line to the goal: the barrier and the stop governor
// keep the vehicle clear of it.
TEST(NavMaterialBikeRollout, KeepsClearOfAnObstacleInTheWay) {
  Batch b = make_batch(5, 4, 1);
  std::fill(b.lam_soft.begin(), b.lam_soft.end(), 0.0f);
  std::fill(b.lam_hard.begin(), b.lam_hard.end(), 0.0f);
  std::fill(b.H.begin(), b.H.end(), 60);
  for (int i = 0; i < b.B; ++i) {
    const float mx = 0.5f * (b.o0[2 * i] + b.goal[2 * i]),
                my = 0.5f * (b.o0[2 * i + 1] + b.goal[2 * i + 1]);
    b.C[2 * i] = mx;
    b.C[2 * i + 1] = my;
    b.R[i] = 0.3f;
    b.alphas[i] = 1.0f;
    b.th0[i] = std::atan2(b.goal[2 * i + 1] - b.o0[2 * i + 1], b.goal[2 * i] - b.o0[2 * i]);
  }
  const outputs y = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard);
  for (int i = 0; i < b.B; ++i)
    EXPECT_GT(y.minc[i], 0.0f) << "agent " << i << " touched the obstacle";
}

// A risk ramp across the path: with the soft material force on, the vehicle
// collects less risk than with it off.
TEST(NavMaterialBikeRollout, SoftMaterialForceSteersOffRisk) {
  Batch b = make_batch(9, 6, 1);
  std::fill(b.mask.begin(), b.mask.end(), 0);
  std::fill(b.lam_hard.begin(), b.lam_hard.end(), 0.0f);
  std::fill(b.H.begin(), b.H.end(), 25);
  const int P = b.P;
  for (int i = 0; i < b.B; ++i) {
    b.th0[i] = std::atan2(b.goal[2 * i + 1] - b.o0[2 * i + 1], b.goal[2 * i] - b.o0[2 * i]);
    // Risk rises to the left of the start-to-goal line, with its gradient.
    const float gx = b.goal[2 * i] - b.o0[2 * i], gy = b.goal[2 * i + 1] - b.o0[2 * i + 1];
    const float n = std::hypot(gx, gy), lx = -gy / n, ly = gx / n;
    for (int r = 0; r < P; ++r)
      for (int c = 0; c < P; ++c) {
        const float offx = c - 0.5f * (P - 1), offy = r - 0.5f * (P - 1); // patch pixels
        const float side = 0.08f * (offx * lx + offy * ly);
        const size_t base = ((size_t)i * 6) * P * P + (size_t)r * P + c, pp = (size_t)P * P;
        b.patch[base + 0 * pp] = std::min(std::max(0.5f + side, 0.0f), 1.0f);
        b.patch[base + 2 * pp] = 0.08f * lx;
        b.patch[base + 3 * pp] = 0.08f * ly;
      }
  }
  std::vector<float> off(b.B, 0.0f), on(b.B, 3.0f);
  const outputs y0 = run(b, b.alphas, b.beta, b.gamma, off, b.lam_hard);
  const outputs y1 = run(b, b.alphas, b.beta, b.gamma, on, b.lam_hard);
  double c0 = 0.0, c1 = 0.0;
  for (int i = 0; i < b.B; ++i) {
    c0 += y0.cum[i];
    c1 += y1.cum[i];
  }
  EXPECT_LT(c1, 0.95 * c0) << "the soft material force did not reduce the risk collected";
}

// L_multi on the bicycle: its gradient w.r.t. alphas, beta and gamma against finite
// differences (kink-aware, as above), over three sampled starts.
TEST(NavMaterialBikeRollout, MultiStartPenaltyGradcheck) {
  Batch b = make_batch(13, 6, 3);
  cvc::nav::multi_start_params mp;
  mp.ms_count = 3;
  mp.frac_lo = 0.6;
  mp.frac_hi = 0.8;
  mp.seed = 4;
  mp.ms_h = 3;
  mp.tau = 0.5f;
  const int B = b.B, N = b.N;
  auto pen = [&](const std::vector<float> &al, const std::vector<float> &be,
                 const std::vector<float> &ga, float *gal, float *gbe, float *gga) {
    return cvc::nav::multi_start_penalty_bicycle(
        al.data(), be.data(), ga.data(), b.o0.data(), b.th0.data(), b.sp0.data(), b.goal.data(),
        b.C.data(), b.R.data(), b.mask.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), B, N, mp,
        b.p, b.v, gal, gbe, gga);
  };
  std::vector<float> gal(B * N, 0.0f), gbe(B, 0.0f), gga(B, 0.0f);
  const double L0 = pen(b.alphas, b.beta, b.gamma, gal.data(), gbe.data(), gga.data());
  ASSERT_TRUE(std::isfinite(L0));
  ASSERT_GT(L0, 0.0);
  std::vector<float> p = b.alphas, g = gal;
  p.insert(p.end(), b.beta.begin(), b.beta.end());
  p.insert(p.end(), b.gamma.begin(), b.gamma.end());
  g.insert(g.end(), gbe.begin(), gbe.end());
  g.insert(g.end(), gga.begin(), gga.end());
  auto at = [&](const std::vector<float> &q) {
    const std::vector<float> al(q.begin(), q.begin() + B * N),
        be(q.begin() + B * N, q.begin() + B * N + B), ga(q.begin() + B * N + B, q.end());
    return pen(al, be, ga, nullptr, nullptr, nullptr);
  };
  const float e = 1e-3f;
  int checked = 0, kinked = 0;
  double worst = 0.0;
  for (size_t i = 0; i < p.size(); ++i) {
    if (std::fabs(g[i]) < 1e-3)
      continue;
    std::vector<float> plus = p, minus = p;
    plus[i] += e;
    minus[i] -= e;
    const double fwd = (at(plus) - L0) / e, bwd = (L0 - at(minus)) / e;
    if (std::fabs(fwd - bwd) > 0.1 * (std::fabs(fwd) + std::fabs(bwd)) + 1e-4) {
      ++kinked;
      continue;
    }
    const double fd = 0.5 * (fwd + bwd);
    worst = std::max(worst, std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6));
    ++checked;
  }
  std::printf("[mat-bike-lmulti] L=%.4f checked=%d kinked=%d worst_rel=%.3e\n", L0, checked, kinked,
              worst);
  EXPECT_GE(checked, 6);
  EXPECT_LT(worst, 5e-2);
  EXPECT_LE(kinked, checked / 4);
}

// ── The same controller on the dynamic vehicle (P8b) ─────────────────────────

TEST(NavMaterialDynRollout, GradcheckMatchesFiniteDifference) {
  gradcheck(dyn_batch(7), "dyn-short", 1e-3f);
}

TEST(NavMaterialDynRollout, GradcheckWithoutReverse) {
  Batch b = dyn_batch(11);
  b.v.allow_reverse = false;
  for (int &h : b.H)
    h += 2;
  gradcheck(b, "dyn-no-reverse", 1e-3f);
}

TEST(NavMaterialDynRollout, ThreadDeterminism) {
  const Batch b = dyn_batch(3, 32, 4);
  const outputs a = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard, 1);
  const outputs c = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard, 8);
  EXPECT_EQ(a.o, c.o);
  EXPECT_EQ(a.x, c.x);
  EXPECT_EQ(a.minc, c.minc);
  const seeds q = make_seeds(b.B, 5);
  EXPECT_EQ(analytic_grad(b, q, 1), analytic_grad(b, q, 8));
}

// With nothing in the way and no material, the controller drives the dynamic vehicle to
// the goal.
TEST(NavMaterialDynRollout, DrivesToTheGoal) {
  Batch b = dyn_batch(21, 8, 1);
  std::fill(b.mask.begin(), b.mask.end(), 0);
  std::fill(b.lam_soft.begin(), b.lam_soft.end(), 0.0f);
  std::fill(b.lam_hard.begin(), b.lam_hard.end(), 0.0f);
  for (int i = 0; i < b.B; ++i)
    b.th0[i] += (i % 2 ? 1.0f : -1.0f) * 1.2f; // start well off the goal bearing
  std::fill(b.H.begin(), b.H.end(), 80);
  const outputs y = run(b, b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard);
  for (int i = 0; i < b.B; ++i) {
    const float d0 = std::hypot(b.goal[2 * i] - b.o0[2 * i], b.goal[2 * i + 1] - b.o0[2 * i + 1]);
    const float d1 = std::hypot(b.goal[2 * i] - y.o[2 * i], b.goal[2 * i + 1] - y.o[2 * i + 1]);
    EXPECT_LT(d1, 0.25f * d0) << "agent " << i << " did not close on its goal";
    EXPECT_LE(y.sp[i], b.v.vmax + 1e-6f);
  }
}

// L_multi on the dynamic vehicle, against finite differences as on the bicycle.
TEST(NavMaterialDynRollout, MultiStartPenaltyGradcheck) {
  Batch b = dyn_batch(13, 6, 3);
  cvc::nav::multi_start_params mp;
  mp.ms_count = 3;
  mp.frac_lo = 0.6;
  mp.frac_hi = 0.8;
  mp.seed = 4;
  mp.ms_h = 3;
  mp.tau = 0.5f;
  const int B = b.B, N = b.N;
  auto pen = [&](const std::vector<float> &al, const std::vector<float> &be,
                 const std::vector<float> &ga, float *gal, float *gbe, float *gga) {
    return cvc::nav::multi_start_penalty_dynamic(
        al.data(), be.data(), ga.data(), b.o0.data(), b.th0.data(), b.sp0.data(), b.goal.data(),
        b.C.data(), b.R.data(), b.mask.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), B, N, mp,
        b.p, b.v, b.dp, gal, gbe, gga);
  };
  std::vector<float> gal(B * N, 0.0f), gbe(B, 0.0f), gga(B, 0.0f);
  const double L0 = pen(b.alphas, b.beta, b.gamma, gal.data(), gbe.data(), gga.data());
  ASSERT_TRUE(std::isfinite(L0));
  ASSERT_GT(L0, 0.0);
  std::vector<float> p = b.alphas, g = gal;
  p.insert(p.end(), b.beta.begin(), b.beta.end());
  p.insert(p.end(), b.gamma.begin(), b.gamma.end());
  g.insert(g.end(), gbe.begin(), gbe.end());
  g.insert(g.end(), gga.begin(), gga.end());
  auto at = [&](const std::vector<float> &q) {
    const std::vector<float> al(q.begin(), q.begin() + B * N),
        be(q.begin() + B * N, q.begin() + B * N + B), ga(q.begin() + B * N + B, q.end());
    return pen(al, be, ga, nullptr, nullptr, nullptr);
  };
  const float e = 1e-3f;
  int checked = 0, kinked = 0;
  double worst = 0.0;
  for (size_t i = 0; i < p.size(); ++i) {
    if (std::fabs(g[i]) < 1e-3)
      continue;
    std::vector<float> plus = p, minus = p;
    plus[i] += e;
    minus[i] -= e;
    const double fwd = (at(plus) - L0) / e, bwd = (L0 - at(minus)) / e;
    if (std::fabs(fwd - bwd) > 0.1 * (std::fabs(fwd) + std::fabs(bwd)) + 1e-4) {
      ++kinked;
      continue;
    }
    const double fd = 0.5 * (fwd + bwd);
    worst = std::max(worst, std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6));
    ++checked;
  }
  std::printf("[mat-dyn-lmulti] L=%.4f checked=%d kinked=%d worst_rel=%.3e\n", L0, checked, kinked,
              worst);
  EXPECT_GE(checked, 6);
  EXPECT_LT(worst, 5e-2);
  EXPECT_LE(kinked, checked / 4);
}
