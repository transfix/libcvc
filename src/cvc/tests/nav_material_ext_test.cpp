/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_material_ext_test: an external force in the material rollouts (material_ext,
// material.h), for a caller's external force in material training.
//
// - A null channel and a zero force are the rollout without one, bit for bit, forward and
//   reverse, on all three integrators (surrogate, bicycle, dynamic).
// - The coefficient gradients through a position-dependent force (a scheduled force patch,
//   whose J chains into the position adjoint) match central finite differences, as
//   nav_material_bike_rollout_test checks them without one; so do the coefficient gradients
//   through a held callback, and dL/dF_ext per step (the values source) along its gradient.
// - A callback that gives the patch's own values and J is the patch, bit for bit.
// - On the bicycle the force reaches the steering bias only with `steer`.
// - Malformed channels throw.
// - Material.Golden writes tests/data/material_ext_golden.json for GRL-SNAM's torch twins.
//
// The batch is nav_material_bike_rollout_test's: interior points, short ragged horizons, so no
// branch flips under the differences.

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/material.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using cvc::nav::bicycle_material_params;
using cvc::nav::material_ext;
using cvc::nav::surrogate_material_params;

namespace {

enum class kind { surrogate, bicycle, dynamic };
const char *name_of(kind k) {
  return k == kind::surrogate ? "surrogate" : (k == kind::bicycle ? "bicycle" : "dynamic");
}

struct Batch {
  kind k = kind::bicycle;
  int B = 6, N = 3, P = 13;
  std::vector<float> o0, th0, sp0, v0, goal, C, R, alphas, beta, gamma, lam_soft, lam_hard, patch,
      rr, d_hat, dt, x0;
  std::vector<std::uint8_t> mask;
  std::vector<int> H;
  surrogate_material_params p;
  bicycle_material_params v;
  cvc::nav::diff::dyn_bike_params dp;
  // The force: K = 2 patches per agent, switching at step 2 for the odd agents.
  int K = 2, steps = 4;
  std::vector<float> fpatch;
  std::vector<int> fsched;
};

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

// The force patches: smooth fields of about 0.3, varying across the agent's path (a cell is a
// length unit; the agents move about one), different per patch and per agent.
void fill_force(Batch &b, int i) {
  const int P = b.P;
  const size_t pp = (size_t)P * P;
  for (int k = 0; k < b.K; ++k)
    for (int r = 0; r < P; ++r)
      for (int c = 0; c < P; ++c) {
        const float x = (float)c / (P - 1), y = (float)r / (P - 1), ph = 0.7f * k + 0.3f * i;
        const size_t base = ((size_t)i * b.K + k) * 2 * pp + (size_t)r * P + c;
        b.fpatch[base] = 0.3f * std::sin(5.0f * x + 2.0f * y + ph);
        b.fpatch[base + pp] = 0.3f * std::cos(3.0f * x - 4.0f * y + ph);
      }
}

Batch make_batch(kind k, unsigned seed, int B = 6, int N = 3, int P = 13) {
  Batch b;
  b.k = k;
  b.B = B;
  b.N = N;
  b.P = P;
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  b.v.L = 0.3f;
  b.v.delta_max = 0.6f;
  b.v.a_max = 4.0f;
  b.v.a_lat_max = 1.5f;
  b.v.k_steer = 0.8f;
  b.v.vmax = 1.5f;
  b.o0.resize(B * 2);
  b.th0.resize(B);
  b.sp0.resize(B);
  b.v0.resize(B * 2);
  b.goal.resize(B * 2);
  b.C.resize(B * N * 2);
  b.R.resize(B * N);
  b.mask.assign(B * N, 1);
  b.alphas.resize(B * N);
  b.beta.resize(B);
  b.gamma.resize(B);
  b.lam_soft.resize(B);
  b.lam_hard.resize(B);
  b.patch.assign((size_t)B * 6 * P * P, 0.0f);
  b.fpatch.assign((size_t)B * b.K * 2 * P * P, 0.0f);
  b.fsched.assign((size_t)B * b.steps, 0);
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
    b.v0[2 * i] = b.sp0[i] * std::cos(b.th0[i]);
    b.v0[2 * i + 1] = b.sp0[i] * std::sin(b.th0[i]);
    b.beta[i] = U(0.8f, 1.2f);
    b.gamma[i] = U(0.2f, 0.5f);
    b.lam_soft[i] = U(0.7f, 1.3f);
    b.lam_hard[i] = U(0.7f, 1.3f);
    b.H[i] = (i % 2 == 0) ? 3 : 4;
    for (int j = 0; j < N; ++j) {
      const float ang = U(0.0f, 6.283185f), dist = U(1.5f, 1.9f);
      b.C[(i * N + j) * 2] = b.o0[2 * i] + dist * std::cos(ang);
      b.C[(i * N + j) * 2 + 1] = b.o0[2 * i + 1] + dist * std::sin(ang);
      b.R[i * N + j] = U(0.5f, 0.7f);
      b.alphas[i * N + j] = U(0.03f, 0.08f);
    }
    fill_patch(b, i);
    fill_force(b, i);
    for (int s = 0; s < b.steps; ++s)
      b.fsched[(size_t)i * b.steps + s] = (s >= 2 && i % 2 == 1) ? 1 : 0;
  }
  if (k == kind::dynamic) {
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
  }
  return b;
}

material_ext patch_ext(const Batch &b, bool schedule = true) {
  material_ext e;
  e.patch = b.fpatch.data();
  e.K = b.K;
  e.patch_of_step = schedule ? b.fsched.data() : nullptr;
  e.steps = b.steps;
  return e;
}

struct outputs {
  std::vector<float> o, th, sp, x, minc, cum, hard, arc;
};

struct coefs {
  std::vector<float> al, be, ga, ls, lh;
};
coefs coefs_of(const Batch &b) { return {b.alphas, b.beta, b.gamma, b.lam_soft, b.lam_hard}; }

outputs run(const Batch &b, const coefs &c, const material_ext *ext, int threads = 1) {
  outputs out;
  out.o = b.o0;
  out.minc.resize(b.B);
  out.cum.resize(b.B);
  out.hard.resize(b.B);
  out.arc.resize(b.B);
  if (b.k == kind::surrogate) {
    out.th = b.v0; // the velocity, in the heading's slot
    cvc::nav::integrate_surrogate_material(
        out.o.data(), out.th.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(),
        c.al.data(), c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(), b.patch.data(),
        b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p,
        out.minc.data(), out.cum.data(), out.hard.data(), out.arc.data(), threads, ext);
    return out;
  }
  out.th = b.th0;
  out.sp = b.sp0;
  if (b.k == kind::dynamic) {
    out.x = b.x0;
    cvc::nav::integrate_dynamic_material(
        out.o.data(), out.th.data(), out.sp.data(), out.x.data(), b.goal.data(), b.C.data(),
        b.R.data(), b.mask.data(), c.al.data(), c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(),
        b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P,
        b.p, b.v, b.dp, out.minc.data(), out.cum.data(), out.hard.data(), out.arc.data(), threads,
        ext);
    return out;
  }
  cvc::nav::integrate_bicycle_material(
      out.o.data(), out.th.data(), out.sp.data(), b.goal.data(), b.C.data(), b.R.data(),
      b.mask.data(), c.al.data(), c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(),
      b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), b.B, b.N, b.P, b.P, b.p,
      b.v, out.minc.data(), out.cum.data(), out.hard.data(), out.arc.data(), threads, ext);
  return out;
}

// Fixed weights on every differentiable output: o, th (the surrogate's v), sp, x, min_clear,
// cum_risk, arc_length.
struct seeds {
  std::vector<float> o, th, sp, x, minc, cum, arc;
};
seeds make_seeds(const Batch &b, unsigned seed) {
  std::mt19937 rng(seed);
  auto U = [&]() { return (float)std::uniform_real_distribution<double>(-1.0, 1.0)(rng); };
  seeds q;
  const int B = b.B;
  for (int i = 0; i < 2 * B; ++i)
    q.o.push_back(U());
  for (int i = 0; i < (b.k == kind::surrogate ? 2 * B : B); ++i)
    q.th.push_back(U());
  for (int i = 0; i < B; ++i) {
    q.sp.push_back(U());
    q.minc.push_back(U());
    q.cum.push_back(U());
    q.arc.push_back(U());
  }
  for (int i = 0; i < 4 * B; ++i)
    q.x.push_back(U());
  return q;
}

double scalar_loss(const Batch &b, const outputs &y, const seeds &q) {
  double L = 0.0;
  for (size_t i = 0; i < y.o.size(); ++i)
    L += (double)q.o[i] * y.o[i];
  for (size_t i = 0; i < y.th.size(); ++i)
    L += (double)q.th[i] * y.th[i];
  for (size_t i = 0; i < y.sp.size(); ++i)
    L += (double)q.sp[i] * y.sp[i];
  for (size_t i = 0; i < y.x.size(); ++i)
    L += (double)q.x[i] * y.x[i];
  for (int i = 0; i < b.B; ++i)
    L += (double)q.minc[i] * y.minc[i] + (double)q.cum[i] * y.cum[i] + (double)q.arc[i] * y.arc[i];
  return L;
}

// [alphas | beta | gamma | lam_soft | lam_hard]
std::vector<float> pack(const coefs &c) {
  std::vector<float> p = c.al;
  for (const std::vector<float> *v : {&c.be, &c.ga, &c.ls, &c.lh})
    p.insert(p.end(), v->begin(), v->end());
  return p;
}
coefs unpack(const Batch &b, const std::vector<float> &p) {
  const int BN = b.B * b.N, B = b.B;
  coefs c;
  c.al.assign(p.begin(), p.begin() + BN);
  c.be.assign(p.begin() + BN, p.begin() + BN + B);
  c.ga.assign(p.begin() + BN + B, p.begin() + BN + 2 * B);
  c.ls.assign(p.begin() + BN + 2 * B, p.begin() + BN + 3 * B);
  c.lh.assign(p.begin() + BN + 3 * B, p.begin() + BN + 4 * B);
  return c;
}

// The coefficient gradient (packed) of scalar_loss, and dL/dF_ext into g_fext if given.
std::vector<float> analytic_grad(const Batch &b, const coefs &c, const seeds &q,
                                 const material_ext *ext, float *g_fext = nullptr,
                                 int threads = 1) {
  const int B = b.B, N = b.N;
  coefs g{std::vector<float>(B * N, 0.0f), std::vector<float>(B, 0.0f), std::vector<float>(B, 0.0f),
          std::vector<float>(B, 0.0f), std::vector<float>(B, 0.0f)};
  if (b.k == kind::surrogate)
    cvc::nav::integrate_surrogate_material_vjp(
        b.o0.data(), b.v0.data(), b.goal.data(), b.C.data(), b.R.data(), b.mask.data(), c.al.data(),
        c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(), b.patch.data(), b.rr.data(),
        b.d_hat.data(), b.dt.data(), b.H.data(), B, N, b.P, b.P, b.p, q.o.data(), q.th.data(),
        q.minc.data(), q.cum.data(), q.arc.data(), g.al.data(), g.be.data(), g.ga.data(),
        g.ls.data(), g.lh.data(), threads, ext, g_fext);
  else if (b.k == kind::dynamic)
    cvc::nav::integrate_dynamic_material_vjp(
        b.o0.data(), b.th0.data(), b.sp0.data(), b.x0.data(), b.goal.data(), b.C.data(), b.R.data(),
        b.mask.data(), c.al.data(), c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(),
        b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), B, N, b.P, b.P, b.p,
        b.v, b.dp, q.o.data(), q.th.data(), q.sp.data(), q.x.data(), q.minc.data(), q.cum.data(),
        q.arc.data(), g.al.data(), g.be.data(), g.ga.data(), g.ls.data(), g.lh.data(), threads, ext,
        g_fext);
  else
    cvc::nav::integrate_bicycle_material_vjp(
        b.o0.data(), b.th0.data(), b.sp0.data(), b.goal.data(), b.C.data(), b.R.data(),
        b.mask.data(), c.al.data(), c.be.data(), c.ga.data(), c.ls.data(), c.lh.data(),
        b.patch.data(), b.rr.data(), b.d_hat.data(), b.dt.data(), b.H.data(), B, N, b.P, b.P, b.p,
        b.v, q.o.data(), q.th.data(), q.sp.data(), q.minc.data(), q.cum.data(), q.arc.data(),
        g.al.data(), g.be.data(), g.ga.data(), g.ls.data(), g.lh.data(), threads, ext, g_fext);
  return pack(g);
}

// nav_material_bike_rollout_test's gradcheck: a directional check along the gradient, and
// per-coefficient checks above the noise floor, skipping (and counting) a coefficient whose
// forward and backward differences disagree (a controller discontinuity).
// The floor is |g| >= 1e-2, not that test's 5e-3: at these 3 and 4 step horizons a coefficient
// near 5e-3 has forward and backward differences 10-16% apart, with or without the force
// (measured on the dynamic batch: g 0.0069 against fd 0.0075 without it, g -0.0055 against
// -0.0050 with it), while every larger one agrees.
void gradcheck(const Batch &b, const material_ext *ext, const char *tag, float eps = 2e-3f) {
  const seeds q = make_seeds(b, 99);
  const coefs c0 = coefs_of(b);
  const std::vector<float> base = pack(c0);
  const std::vector<float> g = analytic_grad(b, c0, q, ext);
  const auto loss_at = [&](const std::vector<float> &p) {
    return scalar_loss(b, run(b, unpack(b, p), ext), q);
  };
  const int P = static_cast<int>(base.size());
  double gnorm = 0.0;
  for (float x : g)
    gnorm += (double)x * x;
  gnorm = std::sqrt(gnorm);
  ASSERT_GT(gnorm, 1e-4);
  std::vector<float> pp(P), pm(P);
  double gdotg = 0.0;
  for (int i = 0; i < P; ++i) {
    const double u = g[i] / gnorm;
    pp[i] = base[i] + eps * (float)u;
    pm[i] = base[i] - eps * (float)u;
    gdotg += (double)g[i] * u;
  }
  const double dd_fd = (loss_at(pp) - loss_at(pm)) / (2.0 * eps);
  const double dir_rel = std::fabs(dd_fd - gdotg) / (std::fabs(dd_fd) + std::fabs(gdotg) + 1e-9);
  const float e1 = 1e-3f;
  double worst_rel = 0.0;
  int checked = 0, kinked = 0;
  const double L0 = loss_at(base);
  for (int i = 0; i < P; ++i) {
    if (std::fabs(g[i]) < 1e-2) // the noise floor at these horizons (see above)
      continue;
    std::vector<float> plus = base, minus = base;
    plus[i] = base[i] + e1;
    minus[i] = base[i] - e1;
    const double fwd = (loss_at(plus) - L0) / e1, bwd = (L0 - loss_at(minus)) / e1;
    if (std::fabs(fwd - bwd) > 0.1 * (std::fabs(fwd) + std::fabs(bwd)) + 1e-3) {
      ++kinked;
      continue;
    }
    const double fd = 0.5 * (fwd + bwd);
    worst_rel =
        std::max(worst_rel, std::fabs(fd - g[i]) / (std::fabs(fd) + std::fabs(g[i]) + 1e-6));
    ++checked;
  }
  std::printf("[material-ext gradcheck %s %s] |g|=%.4f dir_rel=%.3e checked=%d kinked=%d "
              "worst_rel=%.3e\n",
              name_of(b.k), tag, gnorm, dir_rel, checked, kinked, worst_rel);
  EXPECT_LT(dir_rel, 2e-2);
  EXPECT_LT(worst_rel, 5e-2);
  EXPECT_GE(checked, 8);
  EXPECT_LE(kinked, checked / 4);
}

void expect_same(const outputs &a, const outputs &b) {
  const std::vector<float> outputs::*fields[] = {&outputs::o,    &outputs::th,   &outputs::sp,
                                                 &outputs::x,    &outputs::minc, &outputs::cum,
                                                 &outputs::hard, &outputs::arc};
  for (auto f : fields) {
    ASSERT_EQ((a.*f).size(), (b.*f).size());
    for (size_t i = 0; i < (a.*f).size(); ++i)
      EXPECT_EQ((a.*f)[i], (b.*f)[i]);
  }
}

// A callback that samples a Batch's force patch at the pose, as the patch source does, with the
// schedule; `hold` is the channel's.
void patch_callback(void *user, int b, int s, float ox, float oy, float *f, float *J) {
  const Batch &bt = *static_cast<const Batch *>(user);
  const int k = bt.fsched[(size_t)b * bt.steps + s];
  const long chan = (long)bt.P * bt.P;
  const float *px = bt.fpatch.data() + ((long)b * bt.K + k) * 2 * chan, *py = px + chan;
  const float offx = ox - bt.o0[2 * b], offy = oy - bt.o0[2 * b + 1];
  f[0] = cvc::nav::detail::patch_sample_channel(px, bt.P, bt.P, offx, offy);
  f[1] = cvc::nav::detail::patch_sample_channel(py, bt.P, bt.P, offx, offy);
  cvc::nav::detail::patch_sample_channel_grad(px, bt.P, bt.P, offx, offy, J[0], J[1]);
  cvc::nav::detail::patch_sample_channel_grad(py, bt.P, bt.P, offx, offy, J[2], J[3]);
}

const kind kKinds[] = {kind::surrogate, kind::bicycle, kind::dynamic};

} // namespace

TEST(NavMaterialExt, ANullOrZeroForceIsTheRolloutWithoutOne) {
  for (const kind k : kKinds) {
    SCOPED_TRACE(name_of(k));
    Batch b = make_batch(k, 7);
    const coefs c = coefs_of(b);
    const seeds q = make_seeds(b, 99);
    const outputs none = run(b, c, nullptr);
    const std::vector<float> g_none = analytic_grad(b, c, q, nullptr);
    std::fill(b.fpatch.begin(), b.fpatch.end(), 0.0f);
    const material_ext zero = patch_ext(b);
    expect_same(none, run(b, c, &zero));
    const std::vector<float> g_zero = analytic_grad(b, c, q, &zero);
    ASSERT_EQ(g_none.size(), g_zero.size());
    for (size_t i = 0; i < g_none.size(); ++i)
      EXPECT_EQ(g_none[i], g_zero[i]);
  }
}

TEST(NavMaterialExt, TheForceMovesTheRollout) {
  for (const kind k : kKinds) {
    SCOPED_TRACE(name_of(k));
    const Batch b = make_batch(k, 7);
    const material_ext e = patch_ext(b);
    const outputs none = run(b, coefs_of(b), nullptr), with = run(b, coefs_of(b), &e);
    double moved = 0.0;
    for (size_t i = 0; i < none.o.size(); ++i)
      moved = std::max(moved, (double)std::fabs(with.o[i] - none.o[i]));
    std::printf("[material-ext %s] the force patch moves the end positions by up to %.4f\n",
                name_of(k), moved);
    EXPECT_GT(moved, 1e-3);
  }
}

TEST(NavMaterialExt, GradcheckThroughAForcePatchSurrogate) {
  const Batch b = make_batch(kind::surrogate, 7);
  const material_ext e = patch_ext(b);
  gradcheck(b, &e, "patch");
}

TEST(NavMaterialExt, GradcheckThroughAForcePatchBicycle) {
  const Batch b = make_batch(kind::bicycle, 7);
  const material_ext e = patch_ext(b);
  gradcheck(b, &e, "patch");
}

TEST(NavMaterialExt, GradcheckThroughAForcePatchDynamic) {
  const Batch b = make_batch(kind::dynamic, 7);
  const material_ext e = patch_ext(b);
  gradcheck(b, &e, "patch", 1e-3f); // nav_material_bike_rollout_test's dynamic step
}

TEST(NavMaterialExt, GradcheckThroughAHeldCallback) {
  for (const kind k : {kind::surrogate, kind::bicycle}) {
    Batch b = make_batch(k, 7);
    material_ext e;
    e.sample = patch_callback;
    e.user = &b;
    e.hold = 2;
    gradcheck(b, &e, "held callback");
  }
}

// A callback giving the patch's own values and J is the patch source, forward and reverse.
TEST(NavMaterialExt, ACallbackGivingThePatchIsThePatch) {
  for (const kind k : kKinds) {
    SCOPED_TRACE(name_of(k));
    Batch b = make_batch(k, 7);
    const coefs c = coefs_of(b);
    const seeds q = make_seeds(b, 99);
    const material_ext pe = patch_ext(b);
    material_ext ce;
    ce.sample = patch_callback;
    ce.user = &b;
    expect_same(run(b, c, &pe), run(b, c, &ce));
    std::vector<float> fp((size_t)b.B * b.steps * 2, 0.0f), fc = fp;
    material_ext ce2 = ce;
    ce2.steps = b.steps;
    const std::vector<float> gp = analytic_grad(b, c, q, &pe, fp.data()),
                             gc = analytic_grad(b, c, q, &ce2, fc.data());
    for (size_t i = 0; i < gp.size(); ++i)
      EXPECT_EQ(gp[i], gc[i]);
    for (size_t i = 0; i < fp.size(); ++i)
      EXPECT_EQ(fp[i], fc[i]);
  }
}

// dL/dF_ext per step (the values source) against a central difference along it.
TEST(NavMaterialExt, DLdFMatchesFiniteDifferences) {
  for (const kind k : kKinds) {
    SCOPED_TRACE(name_of(k));
    const Batch b = make_batch(k, 7);
    const coefs c = coefs_of(b);
    const seeds q = make_seeds(b, 99);
    std::mt19937 rng(3);
    std::vector<float> F((size_t)b.B * b.steps * 2);
    for (float &x : F)
      x = 0.3f * (float)std::uniform_real_distribution<double>(-1.0, 1.0)(rng);
    material_ext e;
    e.F = F.data();
    e.steps = b.steps;
    std::vector<float> gF(F.size(), 0.0f);
    analytic_grad(b, c, q, &e, gF.data());
    double gn = 0.0;
    for (float x : gF)
      gn += (double)x * x;
    gn = std::sqrt(gn);
    ASSERT_GT(gn, 1e-4);
    const float eps = 1e-3f;
    std::vector<float> Fp = F, Fm = F;
    for (size_t i = 0; i < F.size(); ++i) {
      Fp[i] += eps * (float)(gF[i] / gn);
      Fm[i] -= eps * (float)(gF[i] / gn);
    }
    material_ext ep = e, em = e;
    ep.F = Fp.data();
    em.F = Fm.data();
    const double fd =
        (scalar_loss(b, run(b, c, &ep), q) - scalar_loss(b, run(b, c, &em), q)) / (2.0 * eps);
    const double rel = std::fabs(fd - gn) / (std::fabs(fd) + gn);
    std::printf("[material-ext dL/dF %s] |g| %.4f, finite difference %.4f, rel %.2e\n", name_of(k),
                gn, fd, rel);
    EXPECT_LT(rel, 2e-2);
  }
}

// On the bicycle and the dynamic vehicle, the force reaches the steering bias only with steer.
TEST(NavMaterialExt, TheForceSteersOnlyWithSteer) {
  for (const kind k : {kind::bicycle, kind::dynamic}) {
    SCOPED_TRACE(name_of(k));
    const Batch b = make_batch(k, 7);
    material_ext on = patch_ext(b), off = on;
    off.steer = 0;
    const outputs a = run(b, coefs_of(b), &on), c = run(b, coefs_of(b), &off);
    double dth = 0.0;
    for (int i = 0; i < b.B; ++i)
      dth = std::max(dth, (double)std::fabs(a.th[i] - c.th[i]));
    EXPECT_GT(dth, 1e-4);
  }
}

TEST(NavMaterialExt, MalformedChannelsThrow) {
  Batch b = make_batch(kind::bicycle, 7);
  const coefs c = coefs_of(b);
  std::vector<float> F((size_t)b.B * b.steps * 2, 0.0f), J((size_t)b.B * b.steps * 4, 0.0f);
  const auto throws = [&](material_ext e, const char *why) {
    EXPECT_THROW(run(b, c, &e), std::invalid_argument) << why;
  };
  throws(material_ext{}, "no source");
  material_ext two = patch_ext(b);
  two.F = F.data();
  throws(two, "two sources");
  material_ext jf;
  jf.patch = b.fpatch.data();
  jf.K = b.K;
  jf.J = J.data();
  throws(jf, "J without F");
  material_ext shortF;
  shortF.F = F.data();
  shortF.steps = 2;
  throws(shortF, "steps < max(H)");
  material_ext hold0;
  hold0.sample = patch_callback;
  hold0.user = &b;
  hold0.hold = 0;
  throws(hold0, "hold 0");
  material_ext k0 = patch_ext(b);
  k0.K = 0;
  throws(k0, "K 0");
  b.fsched[3] = 2;
  throws(patch_ext(b), "a patch index past K");
}

// The C++ side of GRL-SNAM's parity gate for its torch twins of the material rollouts with the
// external force: the three integrators on a small batch with a scheduled force patch, each
// case's inputs, outputs, coefficient gradients and dL/dF_ext for fixed output seeds.
// CVC_MATERIAL_EXT_GOLDEN_WRITE=1 rewrites tests/data/material_ext_golden.json; otherwise this
// build reproduces it to 1e-5 relative (the rollouts are float and use libm).
TEST(NavMaterialExt, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "material_ext_golden.json";
  std::ostringstream js;
  std::vector<double> now;
  const auto num = [](double x) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.9g", x);
    return std::string(buf);
  };
  const auto arr = [&](const auto &v) {
    std::string out = "[";
    for (size_t i = 0; i < v.size(); ++i)
      out += (i ? ", " : "") + num((double)v[i]);
    return out + "]";
  };
  js << "{\n \"format\": \"cvc-material-ext-golden-1\",\n \"cases\": [\n";
  bool first = true;
  for (const kind k : kKinds) {
    const Batch b = make_batch(k, 11, /*B=*/3, /*N=*/2, /*P=*/9);
    const material_ext e = patch_ext(b);
    const coefs c = coefs_of(b);
    const seeds q = make_seeds(b, 5);
    const outputs y = run(b, c, &e);
    std::vector<float> gF((size_t)b.B * b.steps * 2, 0.0f);
    const coefs g = unpack(b, analytic_grad(b, c, q, &e, gF.data()));
    const std::vector<float> &gFc = gF;
    for (const std::vector<float> *v : {&y.o, &y.th, &y.sp, &y.x, &y.minc, &y.cum, &y.hard, &y.arc,
                                        &g.al, &g.be, &g.ga, &g.ls, &g.lh, &gFc})
      for (float x : *v)
        now.push_back(x);
    js << (first ? "" : ",\n") << "  {\"integrator\": \"" << name_of(k) << "\", \"B\": " << b.B
       << ", \"N\": " << b.N << ", \"Hp\": " << b.P << ", \"Wp\": " << b.P << ", \"K\": " << b.K
       << ", \"steps\": " << b.steps << ", \"steer\": 1,\n"
       << "   \"params\": {\"margin_factor\": " << num(b.p.margin_factor)
       << ", \"mass\": " << num(b.p.mass) << ", \"d_hat_sdf\": " << num(b.p.d_hat_sdf)
       << ", \"k_sharp\": " << num(b.p.k_sharp) << "},\n"
       << "   \"vehicle\": {\"L\": " << num(b.v.L) << ", \"delta_max\": " << num(b.v.delta_max)
       << ", \"a_max\": " << num(b.v.a_max) << ", \"a_lat_max\": " << num(b.v.a_lat_max)
       << ", \"k_steer\": " << num(b.v.k_steer) << ", \"vmax\": " << num(b.v.vmax)
       << ", \"allow_reverse\": " << (b.v.allow_reverse ? 1 : 0) << "},\n";
    if (k == kind::dynamic) {
      const auto &d = b.dp;
      js << "   \"dynamic\": {\"m\": " << num(d.m) << ", \"Iz\": " << num(d.Iz)
         << ", \"lf\": " << num(d.lf) << ", \"lr\": " << num(d.lr) << ", \"h\": " << num(d.h)
         << ", \"g\": " << num(d.g) << ", \"mu\": " << num(d.mu) << ", \"B_f\": " << num(d.B_f)
         << ", \"B_r\": " << num(d.B_r) << ", \"C\": " << num(d.C)
         << ", \"front_drive\": " << num(d.front_drive)
         << ", \"front_brake\": " << num(d.front_brake) << ", \"drag\": " << num(d.drag)
         << ", \"c_rr\": " << num(d.c_rr) << ", \"tau_steer\": " << num(d.tau_steer)
         << ", \"steer_rate\": " << num(d.steer_rate) << ", \"tau_drive\": " << num(d.tau_drive)
         << ", \"v_blend\": " << num(d.v_blend) << ", \"v_width\": " << num(d.v_width)
         << ", \"eps\": " << num(d.eps) << ", \"fz_min\": " << num(d.fz_min)
         << ", \"iters\": " << d.iters << "},\n";
    }
    js << "   \"o0\": " << arr(b.o0) << ",\n   \"v0\": " << arr(b.v0)
       << ",\n   \"th0\": " << arr(b.th0) << ",\n   \"sp0\": " << arr(b.sp0)
       << ",\n   \"x0\": " << arr(b.x0) << ",\n   \"goal\": " << arr(b.goal)
       << ",\n   \"C\": " << arr(b.C) << ",\n   \"R\": " << arr(b.R)
       << ",\n   \"mask\": " << arr(std::vector<int>(b.mask.begin(), b.mask.end()))
       << ",\n   \"alphas\": " << arr(b.alphas) << ",\n   \"beta\": " << arr(b.beta)
       << ",\n   \"gamma\": " << arr(b.gamma) << ",\n   \"lam_soft\": " << arr(b.lam_soft)
       << ",\n   \"lam_hard\": " << arr(b.lam_hard) << ",\n   \"rollout_patch\": " << arr(b.patch)
       << ",\n   \"rr\": " << arr(b.rr) << ",\n   \"d_hat\": " << arr(b.d_hat)
       << ",\n   \"dt\": " << arr(b.dt) << ",\n   \"H\": " << arr(b.H)
       << ",\n   \"ext_patch\": " << arr(b.fpatch) << ",\n   \"patch_of_step\": " << arr(b.fsched)
       << ",\n   \"seeds\": {\"o\": " << arr(q.o) << ", \"th\": " << arr(q.th)
       << ", \"sp\": " << arr(q.sp) << ", \"x\": " << arr(q.x) << ", \"min_clear\": " << arr(q.minc)
       << ", \"cum_risk\": " << arr(q.cum) << ", \"arc_length\": " << arr(q.arc) << "},\n"
       << "   \"out\": {\"o\": " << arr(y.o) << ", \"th\": " << arr(y.th)
       << ", \"sp\": " << arr(y.sp) << ", \"x\": " << arr(y.x) << ", \"min_clear\": " << arr(y.minc)
       << ", \"cum_risk\": " << arr(y.cum) << ", \"hard_count\": " << arr(y.hard)
       << ", \"arc_length\": " << arr(y.arc) << "},\n"
       << "   \"grad\": {\"alphas\": " << arr(g.al) << ", \"beta\": " << arr(g.be)
       << ", \"gamma\": " << arr(g.ga) << ", \"lam_soft\": " << arr(g.ls)
       << ", \"lam_hard\": " << arr(g.lh) << ", \"fext\": " << arr(gF) << "}}";
    first = false;
  }
  js << "\n ]\n}\n";
  for (double x : now)
    ASSERT_TRUE(std::isfinite(x));
  const char *write = std::getenv("CVC_MATERIAL_EXT_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &cs : want.get_child("cases")) {
    for (const char *key :
         {"o", "th", "sp", "x", "min_clear", "cum_risk", "hard_count", "arc_length"})
      for (const auto &v : cs.second.get_child("out").get_child(key))
        golden.push_back(v.second.get_value<double>());
    for (const char *key : {"alphas", "beta", "gamma", "lam_soft", "lam_hard", "fext"})
      for (const auto &v : cs.second.get_child("grad").get_child(key))
        golden.push_back(v.second.get_value<double>());
  }
  ASSERT_EQ(golden.size(), now.size());
  for (size_t i = 0; i < now.size(); ++i)
    EXPECT_LE(std::fabs(now[i] - golden[i]),
              1e-5 * std::max(1.0, std::max(std::fabs(now[i]), std::fabs(golden[i]))))
        << "value " << i << ": golden " << golden[i] << ", now " << now[i];
}
