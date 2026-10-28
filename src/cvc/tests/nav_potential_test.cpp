/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_potential_test.cpp: U_nav, the navigator's potential (detail/nav_potential.h), evaluated
// along every rollout, and the finite-difference gradcheck that each call site's conservative
// force is -grad U (doc 20's rung 1).
//
//   - The barrier primitives: b' is the derivative the force primitives return, clamps included.
//   - Each call site on exact (linear) planes, where the sampled gradients are the gradients of
//     the sampled values: the coef_train surrogate and bicycle (diff_rollout.h), the material
//     surrogate, bicycle and dynamic rollouts, and the deployed drive (plain, material, terrain,
//     footprint, the plant arm). Each site's own force against central differences of the U it
//     logs, coefficients held fixed. The external force is a port and is never part of U.
//   - The same on the city scene and built material planes, where the planes are sampled from a
//     grid: the residual there is the grid's (stored normals and gradient planes against the
//     gradient of the bilinear values), measured and bounded.
//   - Asking for U changes nothing: every rollout is byte-identical with and without it.
//   - On the point-mass material surrogate, U is the integrator's potential energy: KE + U falls
//     by the damping's work to O(dt).
//   - The per-step energy books (the white paper's Eq. 38): r_E is second order and its sharp
//     check third; the clamp's cross term closes the books; an unlogged force fails the cheap
//     bound and explicit Euler the sharp check. sim_world books context work across a tick and
//     min_gap's push as projection work.
//   - NavPotential.Golden writes or checks tests/data/nav_potential_golden.json, the values
//     GRL-SNAM's torch twin (grl_snam/nav_potential.py) is held to; NavPotential.BooksGolden
//     does the same for the books, tests/data/nav_energy_books_golden.json.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/detail/diff_rollout.h>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/detail/nav_potential.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/material.h>
#include <cvc/nav/sim_world.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

// Central differences of a float function of position, in double.
void fd_grad(const std::function<float(float, float)> &U, float x, float y, float h, double &gx,
             double &gy) {
  gx = (static_cast<double>(U(x + h, y)) - U(x - h, y)) / (2.0 * h);
  gy = (static_cast<double>(U(x, y + h)) - U(x, y - h)) / (2.0 * h);
}

// |F - (-grad U)| relative to max(|F|, floor).
double rel_residual(double fx, double fy, double gx, double gy, double floor) {
  const double ex = fx + gx, ey = fy + gy;
  return std::sqrt(ex * ex + ey * ey) / std::max(std::sqrt(fx * fx + fy * fy), floor);
}

// A world grid over [-10, 10]^2, the planes' (row, col) <-> (y, x) as field_stack has them.
struct grid {
  int H = 41, W = 41;
  double lo = -10.0, hi = 10.0, S = 0.1; // normalized = world * S (centre 0)
  double wx(int c) const { return lo + (hi - lo) * c / (W - 1); }
  double wy(int r) const { return lo + (hi - lo) * r / (H - 1); }
};

// An exact SDF plane: a wall along the line n . w = c0, phi = S (n . w - c0) in normalized units
// and the unit normal n, so bilinear sampling reproduces both exactly.
struct linear_sdf {
  grid g;
  std::vector<float> data;
  field_stack fs;
  linear_sdf(double nx, double ny, double c0) {
    const double m = std::sqrt(nx * nx + ny * ny);
    nx /= m;
    ny /= m;
    const int hw = g.H * g.W;
    data.assign(3 * hw, 0.0f);
    for (int r = 0; r < g.H; ++r)
      for (int c = 0; c < g.W; ++c) {
        data[r * g.W + c] = static_cast<float>(g.S * (nx * g.wx(c) + ny * g.wy(r) - c0));
        data[hw + r * g.W + c] = static_cast<float>(nx);
        data[2 * hw + r * g.W + c] = static_cast<float>(ny);
      }
    fs.data = data.data();
    fs.M = 1;
    fs.H = g.H;
    fs.W = g.W;
    fs.mnx = fs.mny = g.lo;
    fs.mxx = fs.mxy = g.hi;
    fs.cx = fs.cy = 0.0;
    fs.S = g.S;
  }
  diff::field diff_field() const { return to_diff(fs); }
  static diff::field to_diff(const field_stack &fs) {
    diff::field F;
    F.data = fs.data;
    F.H = fs.H;
    F.W = fs.W;
    F.S = static_cast<float>(fs.S);
    F.cx = static_cast<float>(fs.cx);
    F.cy = static_cast<float>(fs.cy);
    F.mnx = static_cast<float>(fs.mnx);
    F.mny = static_cast<float>(fs.mny);
    F.mxx = static_cast<float>(fs.mxx);
    F.mxy = static_cast<float>(fs.mxy);
    return F;
  }
};

// Exact material planes, material_build's convention: r~ linear in world (its gradient channel per
// normalized unit) and phi_m linear in metres (its gradient channel per metre).
struct linear_material {
  grid g;
  std::vector<float> data;
  material_stack ms;
  linear_material(double r0, double rx, double ry, double p0, double px, double py) {
    const int hw = g.H * g.W;
    data.assign(6 * hw, 0.0f);
    for (int r = 0; r < g.H; ++r)
      for (int c = 0; c < g.W; ++c) {
        const int k = r * g.W + c;
        data[k] = static_cast<float>(r0 + rx * g.wx(c) + ry * g.wy(r));
        data[hw + k] = static_cast<float>(p0 + px * g.wx(c) + py * g.wy(r));
        data[2 * hw + k] = static_cast<float>(rx / g.S);
        data[3 * hw + k] = static_cast<float>(ry / g.S);
        data[4 * hw + k] = static_cast<float>(px);
        data[5 * hw + k] = static_cast<float>(py);
      }
    ms.data = data.data();
    ms.M = 1;
    ms.H = g.H;
    ms.W = g.W;
    ms.mnx = ms.mny = g.lo;
    ms.mxx = ms.mxy = g.hi;
    ms.cx = ms.cy = 0.0;
    ms.S = g.S;
  }
};

// A rollout patch (6, Hp, Wp) with linear channels in the batch's units (pixel offsets from the
// centre), the gradient channels exactly the value channels' gradients.
std::vector<float> linear_patch(int Hp, int Wp, float r0, float rx, float ry, float p0, float px,
                                float py) {
  std::vector<float> p(6 * Hp * Wp);
  const int hw = Hp * Wp;
  for (int r = 0; r < Hp; ++r)
    for (int c = 0; c < Wp; ++c) {
      const float x = c - 0.5f * (Wp - 1), y = r - 0.5f * (Hp - 1);
      const int k = r * Wp + c;
      p[k] = r0 + rx * x + ry * y;
      p[hw + k] = p0 + px * x + py * y;
      p[2 * hw + k] = rx;
      p[3 * hw + k] = ry;
      p[4 * hw + k] = px;
      p[5 * hw + k] = py;
    }
  return p;
}

} // namespace

// ── the barrier primitives ──────────────────────────────────────────────────────────

TEST(NavPotential, BarrierPotentialsDifferentiateToTheForcePrimitives) {
  for (const float dh : {0.35f, 2.0f, 6.0f}) {
    // diff::ipc_ (and drive.cpp's ipc_dbdd, the same formula): through the 1e-6 floor.
    int n = 0;
    for (int k = 0; k <= 400; ++k) {
      const float d = -0.05f * dh + 1.15f * dh * k / 400.0f;
      // The step scales with d: b' steepens as 1/d toward the wall.
      const float h = 1e-3f * std::max(std::fabs(d), 1e-2f * dh);
      if (std::fabs(d - dh) < 3 * h || std::fabs(d - 1e-6f) < 3 * h)
        continue; // the kinks
      const double fd =
          (static_cast<double>(detail::ipc_b(d + h, dh)) - detail::ipc_b(d - h, dh)) / (2.0 * h);
      const double f = diff::ipc_(d, dh);
      EXPECT_LE(std::fabs(fd - f), 2e-3 * std::max(1.0, std::fabs(f)))
          << "ipc_b' at d " << d << ", dh " << dh << ": fd " << fd << ", ipc_ " << f;
      ++n;
    }
    EXPECT_GT(n, 370);
    // ipc_dbdd_pw, the material rollouts' barrier, clamped to [-200, 200]: through the knee.
    const float knee = static_cast<float>(detail::ipc_pw_knee(dh));
    EXPECT_NEAR(detail::ipc_dbdd_pw(knee * 1.001f, dh), -200.0f, 2.0f);
    for (int k = 0; k <= 400; ++k) {
      const float d = -0.05f * dh + 1.15f * dh * k / 400.0f;
      const float h = 1e-3f * std::max(std::fabs(d), 1e-2f * dh);
      if (std::fabs(d - dh) < 3 * h || std::fabs(d - knee) < 3 * h || std::fabs(d) < 3 * h)
        continue;
      const double fd =
          (static_cast<double>(detail::ipc_b_pw(d + h, dh)) - detail::ipc_b_pw(d - h, dh)) /
          (2.0 * h);
      const double f = detail::ipc_dbdd_pw(d, dh);
      EXPECT_LE(std::fabs(fd - f), 2e-3 * std::max(1.0, std::fabs(f)))
          << "ipc_b_pw' at d " << d << ", dh " << dh << ": fd " << fd << ", ipc_dbdd_pw " << f;
    }
    // Continuous at the knee and at dh.
    EXPECT_NEAR(detail::ipc_b_pw(knee * (1 - 1e-5f), dh), detail::ipc_b_pw(knee * (1 + 1e-5f), dh),
                1e-3f * std::max(1.0f, detail::ipc_b_pw(knee, dh)));
    EXPECT_NEAR(detail::ipc_b_pw(dh * 0.9999f, dh), 0.0f, 1e-6f);
  }
  // The softplus hazard barrier.
  for (const float k_sharp : {1.25f, 5.0f})
    for (const float dh : {3.0f, 12.0f})
      for (int k = 0; k <= 200; ++k) {
        const float phi = -2.0f + 40.0f * k / 200.0f, h = 1e-3f;
        const double fd = (static_cast<double>(detail::sdf_barrier_b(phi + h, k_sharp, dh)) -
                           detail::sdf_barrier_b(phi - h, k_sharp, dh)) /
                          (2.0 * h);
        EXPECT_NEAR(fd, detail::sdf_barrier_db(phi, k_sharp, dh), 2e-3)
            << "phi " << phi << ", k " << k_sharp << ", dh " << dh;
      }
}

// ── the coef_train sites (diff_rollout.h) ─────────────────────────────────────────

TEST(NavPotential, SurrogateAndBicycleForcesAreMinusGradUOnAnExactField) {
  const linear_sdf wall(0.6, -0.8, -3.0); // phi = 0 on a line through the scene
  const diff::field F = wall.diff_field();
  const float rr = 0.15f, d_hat = 0.35f, hdt = 0.06f;
  diff::bike_veh bv{};
  bv.rr = rr;
  bv.d_hat = d_hat;
  bv.vmax = 50.0f;
  bv.L = 0.035f;
  bv.delta_max = 0.6f;
  bv.a_max = 1.5f;
  bv.a_lat_max = 1.0f;
  bv.k_steer = 0.8f;
  bv.hdt = hdt;
  bv.allow_reverse = 1;
  std::mt19937 rng(11);
  std::uniform_real_distribution<float> u(-0.8f, 0.8f), co(0.2f, 3.0f);
  int n = 0, near_wall = 0;
  double worst = 0.0;
  for (int k = 0; k < 400; ++k) {
    const float ox = u(rng), oy = u(rng), gx = u(rng), gy = u(rng), al = co(rng), be = co(rng);
    const float phi = diff::sample_fwd(F, ox, oy).phi;
    if (phi - rr < 0.02f)
      continue; // keep clear of the barrier's floor and of the inside
    // The surrogate's force, read off its step: v = 0, no damping, so v1 = hdt * F.
    float ox1, oy1, vx1, vy1;
    diff::surr_step(F, ox, oy, 0.0f, 0.0f, gx, gy, al, be, 0.0f, rr, d_hat, 1e9f, hdt, ox1, oy1,
                    vx1, vy1);
    const double fx = vx1 / hdt, fy = vy1 / hdt;
    // The bicycle's, from bike_control itself.
    float force[2], dc, ac;
    diff::bike_control(F, ox, oy, 0.3f, 0.2f, gx, gy, al, be, 1.0f, bv, dc, ac, force);
    EXPECT_NEAR(force[0], fx, 1e-5 * std::max(1.0, std::fabs(fx)));
    EXPECT_NEAR(force[1], fy, 1e-5 * std::max(1.0, std::fabs(fy)));
    double gxu, gyu;
    fd_grad(
        [&](float x, float y) {
          return diff::surr_potential(F, x, y, gx, gy, al, be, rr, d_hat).total();
        },
        ox, oy, 1e-3f, gxu, gyu);
    const double rel = rel_residual(force[0], force[1], gxu, gyu, 1e-2);
    worst = std::max(worst, rel);
    EXPECT_LE(rel, 2e-3) << "pose (" << ox << ", " << oy << ") phi " << phi << ": F (" << force[0]
                         << ", " << force[1] << "), -grad U (" << -gxu << ", " << -gyu << ")";
    near_wall += phi - rr < d_hat;
    ++n;
  }
  EXPECT_GT(n, 200);
  EXPECT_GT(near_wall, 20); // the barrier is engaged at enough of them
  std::printf(
      "  surrogate/bicycle on an exact field: %d poses (%d in the barrier), worst rel %.2e\n", n,
      near_wall, worst);
}

TEST(NavPotential, CoefTrainLogsUAndStaysByteIdentical) {
  const training_scene scene = city_scene(48);
  for (const rollout_kind kind : {rollout_kind::surrogate, rollout_kind::bicycle}) {
    train_config cfg;
    cfg.rollout = kind;
    cfg.hidden = 16;
    coef_trainer tr(cfg, 3);
    const int n = 8, window = 6;
    std::vector<float> o(2 * n), goal(2 * n), v(2 * n, 0.0f);
    scene.sample_starts_goals(n, 5, o.data(), goal.data());
    if (kind == rollout_kind::bicycle)
      for (int i = 0; i < n; ++i)
        v[2 * i] = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]);
    std::vector<float> g1, g2, o1(2 * n), o2(2 * n), v1(2 * n), v2(2 * n), U(n * window, NAN);
    const double L1 = tr.loss_and_grad(scene, o.data(), v.data(), goal.data(), n, window, &g1,
                                       o1.data(), v1.data());
    const double L2 = tr.loss_and_grad(scene, o.data(), v.data(), goal.data(), n, window, &g2,
                                       o2.data(), v2.data(), nullptr, nullptr, U.data());
    EXPECT_EQ(std::memcmp(&L1, &L2, sizeof L1), 0);
    EXPECT_EQ(std::memcmp(g1.data(), g2.data(), g1.size() * 4), 0);
    EXPECT_EQ(std::memcmp(o1.data(), o2.data(), o1.size() * 4), 0);
    EXPECT_EQ(std::memcmp(v1.data(), v2.data(), v1.size() * 4), 0);
    for (const float x : U) {
      EXPECT_TRUE(std::isfinite(x));
      EXPECT_GE(x, 0.0f); // goal spring and barrier are both >= 0 outside the walls
    }
  }
}

// ── the material rollouts ──────────────────────────────────────────────────────────

namespace {

// A small material batch on a linear patch: B agents, N discs, the patch centred on each start.
struct material_batch_fixture {
  int B = 24, N = 4, Hp = 33, Wp = 33;
  std::vector<float> o, v, th, sp, x4, goal, C, R, alphas, beta, gamma, ls, lh, patch, rr, dh, dt;
  std::vector<std::uint8_t> mask;
  std::vector<int> H;
  surrogate_material_params p;
  bicycle_material_params bv;
  diff::dyn_bike_params dyn;
  explicit material_batch_fixture(int horizon, unsigned seed = 7) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f), co(0.3f, 2.0f);
    o.resize(2 * B);
    v.assign(2 * B, 0.0f);
    th.resize(B);
    sp.assign(B, 0.0f);
    x4.assign(4 * B, 0.0f);
    goal.resize(2 * B);
    C.resize(2 * B * N);
    R.resize(B * N);
    alphas.resize(B * N);
    mask.assign(B * N, 1);
    beta.resize(B);
    gamma.resize(B);
    ls.resize(B);
    lh.resize(B);
    rr.assign(B, 1.0f);
    dh.assign(B, 3.0f);
    dt.assign(B, 0.05f);
    H.assign(B, horizon);
    const std::vector<float> one = linear_patch(Hp, Wp, 0.45f, 0.012f, -0.009f, 4.0f, 0.06f, 0.04f);
    for (int b = 0; b < B; ++b) {
      o[2 * b] = 4.0f * u(rng);
      o[2 * b + 1] = 4.0f * u(rng);
      th[b] = 3.0f * u(rng);
      goal[2 * b] = o[2 * b] + 8.0f * u(rng);
      goal[2 * b + 1] = o[2 * b + 1] + 8.0f * u(rng);
      for (int j = 0; j < N; ++j) {
        // Discs whose barrier band (d < dh = 3) the start sits in for most agents.
        const float ang = 6.2831853f * (j + 0.3f * u(rng)) / N;
        const float dist = 3.6f + 1.5f * u(rng);
        C[2 * (b * N + j)] = o[2 * b] + dist * std::cos(ang);
        C[2 * (b * N + j) + 1] = o[2 * b + 1] + dist * std::sin(ang);
        R[b * N + j] = 1.0f + 0.3f * u(rng);
        alphas[b * N + j] = co(rng);
      }
      mask[b * N + N - 1] = b % 3 != 0; // some padded discs
      beta[b] = 0.1f * co(rng);
      gamma[b] = co(rng);
      ls[b] = 2.0f * co(rng);
      lh[b] = 2.0f * co(rng);
      patch.insert(patch.end(), one.begin(), one.end());
    }
    p.d_hat_sdf = 5.0f; // the hazard barrier engaged on the patch (phi ~ 4 near the start)
    bv.L = 2.5f;
    bv.vmax = 3.0f;
    bv.a_max = 2.0f;
    bv.a_lat_max = 2.0f;
    dyn.lf = 1.2f;
    dyn.lr = 1.3f;
  }
  potential_terms U_at(int b, float px, float py) const {
    return detail::material_potential(px, py, goal[2 * b], goal[2 * b + 1], &C[2 * b * N],
                                      &R[b * N], &mask[b * N], &alphas[b * N], N, beta[b], ls[b],
                                      lh[b], dh[b], p.margin_factor * rr[b],
                                      &patch[6 * b * Hp * Wp], &patch[(6 * b + 1) * Hp * Wp], Hp,
                                      Wp, o[2 * b], o[2 * b + 1], p.k_sharp, p.d_hat_sdf);
  }
};

enum class mat_site { surrogate, bicycle, dynamic };

// One rollout of the fixture through `site`, its outputs and (optionally) its energy record.
struct mat_run {
  std::vector<float> o, a, b, x, mc, cr, hc, al;
};
mat_run run_site(const material_batch_fixture &f, mat_site site, rollout_energy *e) {
  mat_run r;
  r.o = f.o;
  r.mc.resize(f.B);
  r.cr.resize(f.B);
  r.hc.resize(f.B);
  r.al.resize(f.B);
  switch (site) {
  case mat_site::surrogate:
    r.a = f.v;
    integrate_surrogate_material(r.o.data(), r.a.data(), f.goal.data(), f.C.data(), f.R.data(),
                                 f.mask.data(), f.alphas.data(), f.beta.data(), f.gamma.data(),
                                 f.ls.data(), f.lh.data(), f.patch.data(), f.rr.data(), f.dh.data(),
                                 f.dt.data(), f.H.data(), f.B, f.N, f.Hp, f.Wp, f.p, r.mc.data(),
                                 r.cr.data(), r.hc.data(), r.al.data(), 1, nullptr, e);
    break;
  case mat_site::bicycle:
    r.a = f.th;
    r.b = f.sp;
    integrate_bicycle_material(
        r.o.data(), r.a.data(), r.b.data(), f.goal.data(), f.C.data(), f.R.data(), f.mask.data(),
        f.alphas.data(), f.beta.data(), f.gamma.data(), f.ls.data(), f.lh.data(), f.patch.data(),
        f.rr.data(), f.dh.data(), f.dt.data(), f.H.data(), f.B, f.N, f.Hp, f.Wp, f.p, f.bv,
        r.mc.data(), r.cr.data(), r.hc.data(), r.al.data(), 1, nullptr, e);
    break;
  case mat_site::dynamic:
    r.a = f.th;
    r.b = f.sp;
    r.x = f.x4;
    integrate_dynamic_material(
        r.o.data(), r.a.data(), r.b.data(), r.x.data(), f.goal.data(), f.C.data(), f.R.data(),
        f.mask.data(), f.alphas.data(), f.beta.data(), f.gamma.data(), f.ls.data(), f.lh.data(),
        f.patch.data(), f.rr.data(), f.dh.data(), f.dt.data(), f.H.data(), f.B, f.N, f.Hp, f.Wp,
        f.p, f.bv, f.dyn, r.mc.data(), r.cr.data(), r.hc.data(), r.al.data(), 1, nullptr, e);
    break;
  }
  return r;
}

const char *name_of(mat_site s) {
  return s == mat_site::surrogate ? "surrogate" : (s == mat_site::bicycle ? "bicycle" : "dynamic");
}

} // namespace

TEST(NavPotential, MaterialRolloutForcesAreMinusGradU) {
  const material_batch_fixture f(5);
  const int maxH = 5;
  for (const mat_site site : {mat_site::surrogate, mat_site::bicycle, mat_site::dynamic}) {
    std::vector<float> U(f.B * (maxH + 1)), T(4 * f.B * (maxH + 1)), Fc(2 * f.B * maxH);
    rollout_energy e;
    e.potential = U.data();
    e.terms = T.data();
    e.force = Fc.data();
    run_site(f, site, &e);
    double worst = 0.0;
    int engaged = 0;
    for (int b = 0; b < f.B; ++b) {
      const float ox = f.o[2 * b], oy = f.o[2 * b + 1];
      // The site's logged U at its start is the shared evaluator's, bit for bit.
      const potential_terms u0 = f.U_at(b, ox, oy);
      EXPECT_EQ(u0.total(), U[b * (maxH + 1)]) << name_of(site) << " agent " << b;
      EXPECT_EQ(u0.goal, T[4 * b * (maxH + 1)]);
      EXPECT_EQ(u0.hard, T[4 * b * (maxH + 1) + 3]);
      double gx, gy;
      fd_grad([&](float x, float y) { return f.U_at(b, x, y).total(); }, ox, oy, 1e-3f, gx, gy);
      const double fx = Fc[2 * b * maxH], fy = Fc[2 * b * maxH + 1];
      const double rel = rel_residual(fx, fy, gx, gy, 1e-2);
      worst = std::max(worst, rel);
      EXPECT_LE(rel, 2e-3) << name_of(site) << " agent " << b << ": F (" << fx << ", " << fy
                           << "), -grad U (" << -gx << ", " << -gy << ")";
      engaged += u0.barrier > 0.0f;
    }
    EXPECT_GT(engaged, f.B / 2) << name_of(site);
    std::printf("  material %s: %d agents (%d with a disc barrier engaged), worst rel %.2e\n",
                name_of(site), f.B, engaged, worst);
  }
}

TEST(NavPotential, MaterialRolloutsAreByteIdenticalWithTheRecord) {
  const material_batch_fixture f(12, 9);
  const int maxH = 12;
  for (const mat_site site : {mat_site::surrogate, mat_site::bicycle, mat_site::dynamic}) {
    const mat_run a = run_site(f, site, nullptr);
    std::vector<float> U(f.B * (maxH + 1)), T(4 * f.B * (maxH + 1)), Fc(2 * f.B * maxH);
    rollout_energy e;
    e.potential = U.data();
    e.terms = T.data();
    e.force = Fc.data();
    const mat_run b = run_site(f, site, &e);
    for (const auto &pr :
         {std::make_pair(&a.o, &b.o), std::make_pair(&a.a, &b.a), std::make_pair(&a.b, &b.b),
          std::make_pair(&a.x, &b.x), std::make_pair(&a.mc, &b.mc), std::make_pair(&a.cr, &b.cr),
          std::make_pair(&a.hc, &b.hc), std::make_pair(&a.al, &b.al)}) {
      ASSERT_EQ(pr.first->size(), pr.second->size());
      EXPECT_EQ(std::memcmp(pr.first->data(), pr.second->data(), pr.first->size() * 4), 0)
          << name_of(site);
    }
    // The end state's U is the evaluator's at the rollout's end.
    for (int bb = 0; bb < f.B; ++bb)
      EXPECT_EQ(U[bb * (maxH + 1) + maxH], f.U_at(bb, b.o[2 * bb], b.o[2 * bb + 1]).total());
    // Any one pointer alone is fine.
    rollout_energy only;
    std::vector<float> U2(U.size());
    only.potential = U2.data();
    run_site(f, site, &only);
    EXPECT_EQ(std::memcmp(U.data(), U2.data(), U.size() * 4), 0);
    // The per-step books: the point mass keeps them, still byte-identical; the vehicles refuse.
    std::vector<float> ke(U.size()), steps(7 * f.B * maxH);
    rollout_energy books = e;
    books.kinetic = ke.data();
    float *col[7];
    for (int c = 0; c < 7; ++c)
      col[c] = steps.data() + c * f.B * maxH;
    books.w_damp = col[0];
    books.w_port = col[1];
    books.w_clamp = col[2];
    books.w_cross = col[3];
    books.residual = col[4];
    books.residual_sharp = col[5];
    books.bound = col[6];
    books.sharp = true;
    if (site != mat_site::surrogate) {
      EXPECT_THROW(run_site(f, site, &books), std::invalid_argument) << name_of(site);
      continue;
    }
    const mat_run c = run_site(f, site, &books);
    for (const auto &pr :
         {std::make_pair(&a.o, &c.o), std::make_pair(&a.a, &c.a), std::make_pair(&a.mc, &c.mc),
          std::make_pair(&a.cr, &c.cr), std::make_pair(&a.hc, &c.hc), std::make_pair(&a.al, &c.al)})
      EXPECT_EQ(std::memcmp(pr.first->data(), pr.second->data(), pr.first->size() * 4), 0);
    for (const float x : steps)
      EXPECT_TRUE(std::isfinite(x));
  }
}

// On the point mass, U is the integrator's potential energy: E = 1/2 m |v|^2 + U falls by the
// damping's work, sum gamma |v|^2 dt, with an error that shrinks linearly with dt (semi-implicit
// Euler). One second of rollout from a moving start, mass 1.
TEST(NavPotential, MaterialSurrogateEnergyBalance) {
  double err_prev = 0.0;
  for (const float dt : {0.02f, 0.01f, 0.005f}) {
    material_batch_fixture f(1, 21);
    const int steps = static_cast<int>(std::lround(1.0f / dt));
    for (int b = 0; b < f.B; ++b) {
      f.dt[b] = dt;
      f.H[b] = steps;
      f.v[2 * b] = 0.5f;
      f.v[2 * b + 1] = -0.3f;
    }
    std::vector<float> U(f.B * (steps + 1));
    rollout_energy e;
    e.potential = U.data();
    run_site(f, mat_site::surrogate, &e);
    // The velocity after each step, from the same rollout cut at that step.
    std::vector<double> work(f.B, 0.0), ke1(f.B, 0.0);
    for (int s = 0; s < steps; ++s) {
      material_batch_fixture g = f;
      for (int b = 0; b < f.B; ++b)
        g.H[b] = s + 1;
      const mat_run r = run_site(g, mat_site::surrogate, nullptr);
      for (int b = 0; b < f.B; ++b) {
        const double vx = r.a[2 * b], vy = r.a[2 * b + 1];
        work[b] += f.gamma[b] * (vx * vx + vy * vy) * dt;
        ke1[b] = 0.5 * (vx * vx + vy * vy);
      }
    }
    double worst = 0.0;
    for (int b = 0; b < f.B; ++b) {
      const double ke0 = 0.5 * (f.v[2 * b] * f.v[2 * b] + f.v[2 * b + 1] * f.v[2 * b + 1]);
      const double U0 = U[b * (steps + 1)], U1 = U[b * (steps + 1) + steps];
      const double dE = (ke1[b] + U1) - (ke0 + U0);
      worst = std::max(worst, std::fabs(dE + work[b]) / std::max(1.0, ke0 + std::fabs(U0)));
    }
    std::printf("  energy balance, dt %.3f: worst |dE + damping work| / E0 = %.3e\n", dt, worst);
    if (err_prev > 0.0)
      EXPECT_LT(worst, 0.7 * err_prev) << "the balance error does not shrink with dt";
    EXPECT_LT(worst, 0.05);
    err_prev = worst;
  }
}

// ── the per-step energy books (rollout_energy, book_step) ────────────────────────────────

namespace {

// One step (H = 1) of the material surrogate from the fixture's start at step dt, with the
// per-step books and the sharp check, a port force F [B*2] on.
struct step_record {
  std::vector<float> U, Fc, ke, w_damp, w_port, w_clamp, w_cross, r, r_sharp, bound;
};
step_record surrogate_step_books(material_batch_fixture f, float dt, const std::vector<float> &F) {
  for (int b = 0; b < f.B; ++b) {
    f.dt[b] = dt;
    f.H[b] = 1;
  }
  step_record k;
  for (auto *x : {&k.w_damp, &k.w_port, &k.w_clamp, &k.w_cross, &k.r, &k.r_sharp, &k.bound})
    x->assign(f.B, NAN);
  k.U.assign(2 * f.B, NAN);
  k.ke.assign(2 * f.B, NAN);
  k.Fc.assign(2 * f.B, NAN);
  rollout_energy e;
  e.potential = k.U.data();
  e.force = k.Fc.data();
  e.kinetic = k.ke.data();
  e.w_damp = k.w_damp.data();
  e.w_port = k.w_port.data();
  e.w_clamp = k.w_clamp.data();
  e.w_cross = k.w_cross.data();
  e.residual = k.r.data();
  e.residual_sharp = k.r_sharp.data();
  e.bound = k.bound.data();
  e.sharp = true;
  material_ext ext;
  ext.F = F.data();
  ext.steps = 1;
  std::vector<float> o = f.o, v = f.v, mc(f.B), cr(f.B), hc(f.B), al(f.B);
  integrate_surrogate_material(o.data(), v.data(), f.goal.data(), f.C.data(), f.R.data(),
                               f.mask.data(), f.alphas.data(), f.beta.data(), f.gamma.data(),
                               f.ls.data(), f.lh.data(), f.patch.data(), f.rr.data(), f.dh.data(),
                               f.dt.data(), f.H.data(), f.B, f.N, f.Hp, f.Wp, f.p, mc.data(),
                               cr.data(), hc.data(), al.data(), 1, &ext, &e);
  return k;
}

double log2_ratio(double a, double b) { return std::log(a / b) / std::log(2.0); }

} // namespace

// The white paper's Eq. 38 on the material surrogate, one step from a moving start with damping
// and a port: r_E scales as dt^2 and r_E - r_hat_E as dt^3, (r_E - r_hat_E)/dt^3 staying within
// a factor of 2 of each agent's constant as dt halves from 0.02 to 0.0025.
TEST(NavPotential, SurrogateEnergyIdentityPerStep) {
  material_batch_fixture f(1, 29);
  std::vector<float> F(2 * f.B);
  for (int b = 0; b < f.B; ++b) {
    f.v[2 * b] = 2.5f * std::cos(0.7f * b);
    f.v[2 * b + 1] = 2.5f * std::sin(0.7f * b);
    F[2 * b] = 0.8f * std::sin(1.3f * b);
    F[2 * b + 1] = -0.6f * std::cos(0.9f * b);
  }
  // The books' double evaluator is the float rollout's potential and force law.
  {
    const step_record k = surrogate_step_books(f, 0.01f, F);
    for (int b = 0; b < f.B; ++b) {
      const detail::material_eval<double> e = detail::material_energy<double>(
          f.o[2 * b], f.o[2 * b + 1], f.goal[2 * b], f.goal[2 * b + 1], &f.C[2 * b * f.N],
          &f.R[b * f.N], &f.mask[b * f.N], &f.alphas[b * f.N], f.N, f.beta[b], f.ls[b], f.lh[b],
          f.dh[b], f.p.margin_factor * f.rr[b], &f.patch[6 * b * f.Hp * f.Wp], f.Hp, f.Wp,
          f.o[2 * b], f.o[2 * b + 1], f.p.k_sharp, f.p.d_hat_sdf);
      EXPECT_NEAR(e.U, k.U[2 * b], 1e-5 * std::max(1.0, std::fabs(e.U))) << "agent " << b;
      EXPECT_LE(rel_residual(e.fx, e.fy, -k.Fc[2 * b], -k.Fc[2 * b + 1], 1.0), 1e-5)
          << "agent " << b;
      const double vx = f.v[2 * b], vy = f.v[2 * b + 1];
      EXPECT_NEAR(k.ke[2 * b], 0.5 * (vx * vx + vy * vy), 1e-6);
      EXPECT_EQ(k.w_clamp[b], 0.0f); // no speed clamp in this rollout
      EXPECT_EQ(k.w_cross[b], 0.0f);
    }
  }
  const std::vector<float> dts = {0.02f, 0.01f, 0.005f, 0.0025f};
  std::vector<step_record> rec;
  for (const float dt : dts)
    rec.push_back(surrogate_step_books(f, dt, F));
  std::vector<double> sum_r(dts.size(), 0.0), sum_d(dts.size(), 0.0);
  double port = 0.0;
  for (std::size_t k = 0; k < dts.size(); ++k)
    for (int b = 0; b < f.B; ++b) {
      const double r = rec[k].r[b], d = r - rec[k].r_sharp[b];
      EXPECT_LE(std::fabs(r), rec[k].bound[b]) << "dt " << dts[k] << ", agent " << b;
      sum_r[k] += std::fabs(r);
      sum_d[k] += std::fabs(d);
      port += std::fabs(rec[k].w_port[b]);
    }
  EXPECT_GT(port, 0.0);
  // Per agent: r_E / dt^2 and (r_E - r_hat_E) / dt^3 within a factor of 2 per halving of dt. The
  // constants are the limits of Taylor expansions, so this asks the coarsest step to be in their
  // range: at most a quarter of the velocity changed by it (two agents sit so deep in a barrier
  // that they need a finer dt), and r_E / dt^2 not cancelling (|c2| above 5% of the bound's
  // scale).
  int checked2 = 0, checked3 = 0;
  double worst2 = 1.0, worst3 = 1.0;
  const double h_fine = dts.back();
  for (int b = 0; b < f.B; ++b) {
    const double ax = rec.back().Fc[2 * b] - f.gamma[b] * f.v[2 * b] + F[2 * b];
    const double ay = rec.back().Fc[2 * b + 1] - f.gamma[b] * f.v[2 * b + 1] + F[2 * b + 1];
    if (dts[0] * std::hypot(ax, ay) > 0.25 * std::hypot(f.v[2 * b], f.v[2 * b + 1]))
      continue;
    const auto c2 = [&](std::size_t k) { return rec[k].r[b] / (dts[k] * dts[k]); };
    const auto c3 = [&](std::size_t k) {
      return (rec[k].r[b] - static_cast<double>(rec[k].r_sharp[b])) / std::pow(dts[k], 3);
    };
    const bool cond2 =
        std::fabs(c2(dts.size() - 1)) > 0.05 * rec.back().bound[b] / (h_fine * h_fine);
    for (std::size_t k = 0; k + 1 < dts.size(); ++k) {
      const double q2 = c2(k) / c2(k + 1), q3 = c3(k) / c3(k + 1);
      if (cond2) {
        EXPECT_GE(q2, 0.5) << "r_E / dt^2, agent " << b << ", dt " << dts[k];
        EXPECT_LE(q2, 2.0) << "r_E / dt^2, agent " << b << ", dt " << dts[k];
        worst2 = std::max(worst2, std::max(q2, 1.0 / q2));
        ++checked2;
      }
      EXPECT_GE(q3, 0.5) << "(r_E - r_hat_E) / dt^3, agent " << b << ", dt " << dts[k];
      EXPECT_LE(q3, 2.0) << "(r_E - r_hat_E) / dt^3, agent " << b << ", dt " << dts[k];
      worst3 = std::max(worst3, std::max(q3, 1.0 / q3));
      ++checked3;
    }
  }
  EXPECT_GE(checked2, 3 * (2 * f.B / 3));
  EXPECT_GE(checked3, 3 * (5 * f.B / 6));
  for (std::size_t k = 0; k + 1 < dts.size(); ++k) {
    const double p2 = log2_ratio(sum_r[k], sum_r[k + 1]), p3 = log2_ratio(sum_d[k], sum_d[k + 1]);
    std::printf("  books, dt %.4f -> %.4f: order of r_E %.3f, of r_E - r_hat_E %.3f\n", dts[k],
                dts[k + 1], p2, p3);
    EXPECT_NEAR(p2, 2.0, 0.2);
    EXPECT_NEAR(p3, 3.0, 0.3);
  }
  std::printf("  books: %d/%d agent-halvings checked; worst ratio %.3f (r_E/dt^2), %.3f "
              "((r_E - r_hat_E)/dt^3)\n",
              checked2, checked3, worst2, worst3);
}

namespace {

// The coef_train surrogate's conservative force at o (F_bar + F_goal = -grad surr_potential), as
// bike_control returns it.
void surr_force(const diff::field &F, float ox, float oy, float gx, float gy, float al, float be,
                const diff::bike_veh &bv, double &fx, double &fy) {
  float force[2], dc, ac;
  diff::bike_control(F, ox, oy, 0.3f, 0.2f, gx, gy, al, be, 1.0f, bv, dc, ac, force);
  fx = force[0];
  fy = force[1];
}

} // namespace

// diff::surr_step with its vmax clamp cutting hard: the position moves with vt, not v', and the
// identity closes only once W_cross (and W_clamp) are booked; left out, the step fails the bound.
TEST(NavPotential, ClampWorkClosesTheBooks) {
  const linear_sdf wall(0.6, -0.8, -3.0);
  const diff::field F = wall.diff_field();
  const float rr = 0.15f, d_hat = 0.35f, hdt = 0.03f, vmax = 0.5f, ga = 0.4f;
  diff::bike_veh bv{};
  bv.rr = rr;
  bv.d_hat = d_hat;
  bv.vmax = 50.0f;
  bv.L = 0.035f;
  bv.delta_max = 0.6f;
  bv.a_max = 1.5f;
  bv.a_lat_max = 1.0f;
  bv.k_steer = 0.8f;
  bv.hdt = hdt;
  bv.allow_reverse = 1;
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> u(-0.8f, 0.8f), co(0.2f, 3.0f), turn(-0.6f, 0.6f);
  int n = 0, closes = 0, fails_without_cross = 0, near_wall = 0;
  double worst = 0.0, worst_sharp = 0.0;
  for (int k = 0; k < 400; ++k) {
    const float ox = u(rng), oy = u(rng), gx = u(rng), gy = u(rng), al = co(rng), be = co(rng);
    const float phi = diff::sample_fwd(F, ox, oy).phi;
    if (phi - rr < 0.05f)
      continue;
    double f0x, f0y;
    surr_force(F, ox, oy, gx, gy, al, be, bv, f0x, f0y);
    const double fn = std::sqrt(f0x * f0x + f0y * f0y);
    if (fn < 0.1)
      continue;
    // Moving at 3 vmax, roughly along the force, so the clamp cuts v' to a third.
    const double a = std::atan2(f0y, f0x) + turn(rng);
    const float vx = static_cast<float>(1.5 * std::cos(a)),
                vy = static_cast<float>(1.5 * std::sin(a));
    float ox1, oy1, vx1, vy1;
    diff::surr_step(F, ox, oy, vx, vy, gx, gy, al, be, ga, rr, d_hat, vmax, hdt, ox1, oy1, vx1,
                    vy1);
    double f1x, f1y;
    surr_force(F, ox1, oy1, gx, gy, al, be, bv, f1x, f1y);
    book_step_args s;
    s.dt = hdt;
    s.gamma = ga;
    s.v[0] = vx;
    s.v[1] = vy;
    s.vp[0] = vx + hdt * (f0x - ga * vx);
    s.vp[1] = vy + hdt * (f0y - ga * vy);
    s.vt[0] = vx1;
    s.vt[1] = vy1;
    s.dx[0] = static_cast<double>(ox1) - ox;
    s.dx[1] = static_cast<double>(oy1) - oy;
    s.g[0] = -f0x;
    s.g[1] = -f0y;
    s.g1[0] = -f1x;
    s.g1[1] = -f1y;
    s.U0 = diff::surr_potential(F, ox, oy, gx, gy, al, be, rr, d_hat).total();
    s.U1 = diff::surr_potential(F, ox1, oy1, gx, gy, al, be, rr, d_hat).total();
    // vt' grad^2 U vt from the force along vt, as the rollout's sharp check takes it.
    const double h = 0.5 * hdt;
    double fpx, fpy, fmx, fmy;
    surr_force(F, static_cast<float>(ox + h * vx1), static_cast<float>(oy + h * vy1), gx, gy, al,
               be, bv, fpx, fpy);
    surr_force(F, static_cast<float>(ox - h * vx1), static_cast<float>(oy - h * vy1), gx, gy, al,
               be, bv, fmx, fmy);
    s.vHv = -(vx1 * (fpx - fmx) + vy1 * (fpy - fmy)) / (2.0 * h);
    const step_books bk = book_step(s);
    const double vp = std::hypot(s.vp[0], s.vp[1]);
    EXPECT_GT(vp, 2.0 * vmax); // the clamp cut hard
    EXPECT_NEAR(std::hypot(vx1, vy1), vmax, 1e-5);
    EXPECT_GT(bk.w_clamp, 0.0);
    closes += std::fabs(bk.residual) <= bk.bound;
    fails_without_cross += std::fabs(bk.residual + bk.w_cross) > bk.bound;
    worst = std::max(worst, std::fabs(bk.residual) / bk.bound);
    worst_sharp = std::max(worst_sharp, std::fabs(bk.residual - bk.residual_sharp) / bk.bound);
    near_wall += phi - rr < d_hat;
    ++n;
  }
  EXPECT_GT(n, 200);
  EXPECT_GT(near_wall, 20);
  EXPECT_EQ(closes, n) << "with W_cross booked every clamped step closes within the bound";
  EXPECT_GE(fails_without_cross, 9 * n / 10) << "left out, W_cross is work of order dt";
  EXPECT_LT(worst_sharp, 0.25); // the sharp check: third order against the bound's second
  std::printf("  clamp: %d steps (%d in the barrier); |r_E|/bound worst %.3f; without W_cross %d "
              "fail; |r_E - r_hat_E|/bound worst %.3f\n",
              n, near_wall, worst, fails_without_cross, worst_sharp);
}

// A port force the books leave out is work of order dt, against a bound of order dt^2: on the
// material surrogate with F_ext booked every step closes within the bound, and with it unbooked
// (r_E + W_port) the steps fail it.
TEST(NavPotential, UnloggedForceRaisesTheAlarm) {
  const int steps = 10;
  material_batch_fixture f(steps, 13);
  std::vector<float> F(2 * f.B * steps);
  for (int b = 0; b < f.B; ++b) {
    f.dt[b] = 0.002f;
    const float c = std::cos(0.7f * b), s = std::sin(0.7f * b);
    f.v[2 * b] = c;
    f.v[2 * b + 1] = s;
    for (int k = 0; k < steps; ++k) { // 3 units along the start's heading
      F[2 * (b * steps + k)] = 3.0f * c;
      F[2 * (b * steps + k) + 1] = 3.0f * s;
    }
  }
  std::vector<float> r(f.B * steps), bound(f.B * steps), wp(f.B * steps);
  rollout_energy e;
  e.residual = r.data();
  e.bound = bound.data();
  e.w_port = wp.data();
  material_ext ext;
  ext.F = F.data();
  ext.steps = steps;
  std::vector<float> o = f.o, v = f.v, mc(f.B), cr(f.B), hc(f.B), al(f.B);
  integrate_surrogate_material(o.data(), v.data(), f.goal.data(), f.C.data(), f.R.data(),
                               f.mask.data(), f.alphas.data(), f.beta.data(), f.gamma.data(),
                               f.ls.data(), f.lh.data(), f.patch.data(), f.rr.data(), f.dh.data(),
                               f.dt.data(), f.H.data(), f.B, f.N, f.Hp, f.Wp, f.p, mc.data(),
                               cr.data(), hc.data(), al.data(), 1, &ext, &e);
  int booked = 0, alarms = 0;
  double worst_unbooked = 1e30;
  for (int k = 0; k < f.B * steps; ++k) {
    booked += std::fabs(r[k]) <= bound[k];
    const double unbooked = std::fabs(static_cast<double>(r[k]) + wp[k]) / bound[k];
    alarms += unbooked > 1.0;
    worst_unbooked = std::min(worst_unbooked, unbooked);
  }
  EXPECT_EQ(booked, f.B * steps);
  EXPECT_GE(alarms, 95 * f.B * steps / 100);
  std::printf("  unlogged port: %d/%d steps raise the alarm; smallest |r_E + W_port|/bound %.2f\n",
              alarms, f.B * steps, worst_unbooked);
}

// The explicit Euler of the native multi-start rollout (integrate_surrogate_v2, semi_implicit
// false) gains energy every step, by exactly w^2 dt^2 on a harmonic mode: a bias. Per step its
// residual is second order like the semi-implicit one's, so the cheap bound does not see it; the
// sharp check does, sitting dt^2 |g|^2 off r_E where the semi-implicit step meets it exactly (a
// quadratic potential has a constant Hessian). Circular orbits of a goal spring, one period.
TEST(NavPotential, ExplicitEulerRaisesTheAlarm) {
  const int B = 8;
  const float beta = 4.0f, dt = 0.02f; // w = 2, w dt = 0.04
  const double w = std::sqrt(static_cast<double>(beta)), gain = w * w * dt * dt;
  const int S = static_cast<int>(std::lround(6.283185307179586 / (w * dt)));
  std::vector<float> o0(2 * B), v0(2 * B), goal(2 * B, 0.0f);
  for (int b = 0; b < B; ++b) {
    const float rad = 0.5f + 0.1f * b, a = 0.8f * b, sgn = b % 2 ? 1.0f : -1.0f;
    o0[2 * b] = rad * std::cos(a);
    o0[2 * b + 1] = rad * std::sin(a);
    v0[2 * b] = -sgn * static_cast<float>(w) * rad * std::sin(a); // circular: |v| = w |x|
    v0[2 * b + 1] = sgn * static_cast<float>(w) * rad * std::cos(a);
  }
  const std::vector<float> C(2 * B, 50.0f), R(B, 0.0f), al(B, 0.0f), be(B, beta), ga(B, 0.0f);
  const std::vector<float> rr(B, 0.0f), dh(B, 1.0f), dtv(B, dt);
  const std::vector<std::uint8_t> mask(B, 0); // no discs: U = beta/2 |x - g|^2
  const std::vector<int> H(B, 1);
  for (const bool semi : {false, true}) {
    geom_rollout_params p;
    p.semi_implicit = semi;
    std::vector<float> o = o0, v = v0, mc(B);
    std::vector<double> H0(B);
    for (int b = 0; b < B; ++b)
      H0[b] = 0.5 * (v[2 * b] * v[2 * b] + v[2 * b + 1] * v[2 * b + 1]) +
              0.5 * beta * (o[2 * b] * o[2 * b] + o[2 * b + 1] * o[2 * b + 1]);
    double worst_gain = 0.0, sharp_lo = 1e30, sharp_hi = 0.0, cheap_hi = 0.0;
    int positive = 0, n = 0;
    for (int s = 0; s < S; ++s) {
      const std::vector<float> op = o, vp = v;
      integrate_surrogate_v2(o.data(), v.data(), goal.data(), C.data(), R.data(), mask.data(),
                             al.data(), be.data(), ga.data(), rr.data(), dh.data(), dtv.data(),
                             H.data(), B, 1, p, mc.data());
      for (int b = 0; b < B; ++b) {
        book_step_args a;
        a.dt = dt;
        for (int c = 0; c < 2; ++c) {
          a.v[c] = vp[2 * b + c];
          a.vp[c] = a.vt[c] = v[2 * b + c];
          a.dx[c] = static_cast<double>(o[2 * b + c]) - op[2 * b + c];
          a.g[c] = beta * static_cast<double>(op[2 * b + c]);
          a.g1[c] = beta * static_cast<double>(o[2 * b + c]);
        }
        const double x0 = op[2 * b], y0 = op[2 * b + 1], x1 = o[2 * b], y1 = o[2 * b + 1];
        a.U0 = 0.5 * beta * (x0 * x0 + y0 * y0);
        a.U1 = 0.5 * beta * (x1 * x1 + y1 * y1);
        a.vHv = beta * (a.vt[0] * a.vt[0] + a.vt[1] * a.vt[1]);
        const step_books k = book_step(a);
        const double Hs = 0.5 * (a.v[0] * a.v[0] + a.v[1] * a.v[1]) + a.U0;
        const double sharp = (k.residual - k.residual_sharp) / k.bound;
        positive += k.residual > 0.0;
        if (!semi)
          worst_gain = std::max(worst_gain, std::fabs(k.delta_H / Hs - gain) / gain);
        sharp_lo = std::min(sharp_lo, sharp);
        sharp_hi = std::max(sharp_hi, std::fabs(sharp));
        cheap_hi = std::max(cheap_hi, std::fabs(k.residual) / k.bound);
        if (semi) {
          EXPECT_LE(std::fabs(k.residual), k.bound) << "step " << s << ", agent " << b;
        }
        ++n;
      }
    }
    double drift = 0.0;
    for (int b = 0; b < B; ++b) {
      const double H1 = 0.5 * (v[2 * b] * v[2 * b] + v[2 * b + 1] * v[2 * b + 1]) +
                        0.5 * beta * (o[2 * b] * o[2 * b] + o[2 * b + 1] * o[2 * b + 1]);
      drift = std::max(drift, std::fabs(H1 / H0[b] - (semi ? 1.0 : std::pow(1.0 + gain, S))));
    }
    if (!semi) {
      EXPECT_EQ(positive, n) << "explicit Euler gains energy at every step";
      EXPECT_LT(worst_gain, 2e-3) << "the gain per step is w^2 dt^2";
      EXPECT_LT(drift, 1e-3); // (1 + w^2 dt^2)^S over the period
      EXPECT_GT(sharp_lo, 0.3) << "the sharp check alarms at every step";
    } else {
      EXPECT_LT(sharp_hi, 1e-3) << "the semi-implicit step meets the sharp check";
      EXPECT_LT(drift, 2.0 * w * dt); // the modified energy's O(dt) offset, not a gain
    }
    std::printf("  %s Euler, %d steps: |H_T/H_0 - expected| %.2e; (r_E - r_hat_E)/bound from "
                "%.2e, |.| up to %.2e; |r_E|/bound up to %.3f\n",
                semi ? "semi-implicit" : "explicit", S, drift, sharp_lo, sharp_hi, cheap_hi);
  }
}

// ── the deployed drive (drive.cpp) ─────────────────────────────────────────────────

namespace {

// A drive_step over n agents with telemetry, every variant the drive has: a material stack,
// a terrain stack, a footprint, the plant arm. Checks tel.force against -grad drive_potential at
// each start pose, the coefficients those the telemetry reports, and returns the residuals.
struct drive_setup {
  int n = 0;
  std::vector<float> o, th, sp, carrot;
  veh_params v;
  // the optional parts
  std::vector<float> lam_soft, lam_hard;
  material_drive md;
  bool use_mat = false;
  terrain_drive ter;
  std::vector<float> body_off;
  std::vector<float> pl_out_s, pl_out_a, pl_out_v, pl_hard, pl_lat, pl_brake, pl_now, pl_rate;
  plant_drive pd;
};

drive_setup make_drive(int n, unsigned seed, float half) {
  drive_setup d;
  d.n = n;
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> u(-half, half), ang(-3.1f, 3.1f), spd(0.0f, 0.6f);
  d.o.resize(2 * n);
  d.carrot.resize(2 * n);
  d.th.resize(n);
  d.sp.resize(n);
  for (int i = 0; i < n; ++i) {
    d.o[2 * i] = u(rng);
    d.o[2 * i + 1] = u(rng);
    d.carrot[2 * i] = u(rng);
    d.carrot[2 * i + 1] = u(rng);
    d.th[i] = ang(rng);
    d.sp[i] = spd(rng);
  }
  d.v.rr = 0.15f;
  d.v.d_hat = 0.35f;
  d.v.dt = 0.06f;
  d.v.nsub = 2;
  d.lam_soft.assign(n, 0.0f);
  d.lam_hard.assign(n, 0.0f);
  for (int i = 0; i < n; ++i) {
    d.lam_soft[i] = 0.4f + 0.1f * (i % 5);
    d.lam_hard[i] = 0.8f + 0.1f * (i % 3);
  }
  return d;
}

void with_material(drive_setup &d, const material_stack &ms) {
  d.md.stack = &ms;
  d.md.lam_soft = d.lam_soft.data();
  d.md.lam_hard = d.lam_hard.data();
  d.use_mat = true;
}

void with_terrain(drive_setup &d, const material_stack &ts) {
  d.ter.stack = &ts;
  d.ter.lam_soft = 0.7f;
  d.ter.lam_hard = 0.5f;
  d.v.terrain = &d.ter;
}

void with_footprint(drive_setup &d) {
  d.body_off = {0.0f, 0.5f * d.v.L, d.v.L};
  d.v.n_body = 3;
  d.v.body_offsets = d.body_off.data();
  d.v.body_rr = 0.1f;
  d.v.body_gain = 1.0f / 3.0f;
}

void with_plant(drive_setup &d) {
  const int n = d.n;
  d.pl_out_s.assign(n, 0.0f);
  d.pl_out_a.assign(n, 0.0f);
  d.pl_out_v.assign(n, 0.0f);
  d.pl_hard.assign(n, 1.0f);
  for (int i = 0; i < n; ++i)
    d.pl_hard[i] = 1.0f + 0.25f * (i % 4);
  d.pl_lat.assign(n, 0.8f);
  d.pl_brake.assign(n, 2.0f);
  d.pl_now.assign(n, 0.05f);
  d.pl_rate.assign(n, 0.35f);
  d.pd.steer_cmd = d.pl_out_s.data();
  d.pd.accel_cmd = d.pl_out_a.data();
  d.pd.v_stop = d.pl_out_v.data();
  d.pd.hard_scale = d.pl_hard.data();
  d.pd.a_lat = d.pl_lat.data();
  d.pd.a_brake = d.pl_brake.data();
  d.pd.steer_now = d.pl_now.data();
  d.pd.steer_rate = d.pl_rate.data();
  d.v.plant = &d.pd;
}

struct drive_out {
  std::vector<float> o, th, sp, mc, s_cmd, a_cmd, v_stop;
  std::vector<drive_telemetry> tel;
};

drive_out run_drive(drive_setup &d, const field_stack &fs, bool tel) {
  const coef_mlp model = coef_mlp::default_biased();
  drive_out r;
  r.o = d.o;
  r.th = d.th;
  r.sp = d.sp;
  r.mc.resize(d.n);
  if (tel)
    r.tel.resize(d.n);
  drive_telemetry *tp = tel ? r.tel.data() : nullptr;
  if (d.use_mat)
    drive_step_material(fs, r.o.data(), r.th.data(), r.sp.data(), d.carrot.data(), model, d.n,
                        nullptr, d.v, d.md, r.mc.data(), 1, tp);
  else
    drive_step(fs, r.o.data(), r.th.data(), r.sp.data(), d.carrot.data(), model, d.n, nullptr, d.v,
               r.mc.data(), 1, nullptr, tp);
  if (d.v.plant) {
    r.s_cmd = d.pl_out_s;
    r.a_cmd = d.pl_out_a;
    r.v_stop = d.pl_out_v;
  }
  return r;
}

// Per-agent relative residuals of tel.force against -grad U; agents whose (nearest) clearance is
// below `min_clear` are skipped (the barrier's floor and the inside of walls).
std::vector<double> drive_residuals(drive_setup &d, const field_stack &fs, float h, float min_clear,
                                    int *checked_barrier = nullptr) {
  const drive_out r = run_drive(d, fs, true);
  std::vector<double> res;
  int engaged = 0;
  for (int i = 0; i < d.n; ++i) {
    const drive_telemetry &t = r.tel[i];
    if (!(t.clearance > min_clear))
      continue;
    const material_drive *mat = d.use_mat ? &d.md : nullptr;
    const float ox = d.o[2 * i], oy = d.o[2 * i + 1], th = d.th[i];
    const float gx = d.carrot[2 * i], gy = d.carrot[2 * i + 1];
    const potential_terms u0 = drive_potential(fs, ox, oy, th, gx, gy, i, nullptr, d.v, t.alpha,
                                               t.beta, mat, t.lam_soft, t.lam_hard);
    EXPECT_EQ(u0.total(), t.potential) << "agent " << i;
    EXPECT_EQ(u0.barrier, t.terms.barrier);
    EXPECT_EQ(u0.hard, t.terms.hard);
    EXPECT_EQ(u0.terrain, t.terms.terrain);
    double gxu, gyu;
    fd_grad(
        [&](float x, float y) {
          return drive_potential(fs, x, y, th, gx, gy, i, nullptr, d.v, t.alpha, t.beta, mat,
                                 t.lam_soft, t.lam_hard)
              .total();
        },
        ox, oy, h, gxu, gyu);
    res.push_back(rel_residual(t.force_x, t.force_y, gxu, gyu, 1e-2));
    engaged += u0.barrier > 0.0f;
  }
  if (checked_barrier)
    *checked_barrier = engaged;
  return res;
}

double max_of(const std::vector<double> &x) {
  double m = 0.0;
  for (const double v : x)
    m = std::max(m, v);
  return m;
}

double quantile(std::vector<double> x, double q) {
  if (x.empty())
    return 0.0;
  std::sort(x.begin(), x.end());
  return x[static_cast<std::size_t>(q * (x.size() - 1))];
}

} // namespace

TEST(NavPotential, DriveForceIsMinusGradUOnExactPlanes) {
  const linear_sdf wall(0.6, -0.8, -3.0);
  const linear_material mat(0.5, 0.02, -0.015, 6.0, 0.3, 0.2);
  const linear_material terr(0.3, -0.01, 0.02, 9.0, -0.25, 0.3);
  struct variant {
    const char *name;
    bool m, t, fp, plant;
  };
  for (const variant &vr :
       {variant{"plain", false, false, false, false},
        variant{"material", true, false, false, false},
        variant{"terrain", false, true, false, false},
        variant{"footprint", false, false, true, false},
        variant{"plant arm", false, false, false, true}, variant{"all", true, true, true, true}}) {
    drive_setup d = make_drive(300, 17, 0.8f);
    if (vr.m)
      with_material(d, mat.ms);
    if (vr.t)
      with_terrain(d, terr.ms);
    if (vr.fp)
      with_footprint(d);
    if (vr.plant)
      with_plant(d);
    int engaged = 0;
    const std::vector<double> res = drive_residuals(d, wall.fs, 1e-3f, 0.02f, &engaged);
    EXPECT_GT(res.size(), 150u) << vr.name;
    EXPECT_GT(engaged, 20) << vr.name;
    EXPECT_LE(max_of(res), 2e-3) << vr.name;
    std::printf("  drive %-9s on exact planes: %zu poses (%d in the barrier), worst rel %.2e\n",
                vr.name, res.size(), engaged, max_of(res));
  }
}

TEST(NavPotential, DriveIsByteIdenticalWithTheNewTelemetry) {
  const linear_sdf wall(0.6, -0.8, -3.0);
  const linear_material mat(0.5, 0.02, -0.015, 6.0, 0.3, 0.2);
  const linear_material terr(0.3, -0.01, 0.02, 9.0, -0.25, 0.3);
  for (int variant = 0; variant < 2; ++variant) {
    drive_setup d = make_drive(64, 23, 0.8f);
    with_material(d, mat.ms);
    with_terrain(d, terr.ms);
    with_footprint(d);
    if (variant)
      with_plant(d);
    const drive_out a = run_drive(d, wall.fs, false);
    const drive_out b = run_drive(d, wall.fs, true);
    for (const auto &pr :
         {std::make_pair(&a.o, &b.o), std::make_pair(&a.th, &b.th), std::make_pair(&a.sp, &b.sp),
          std::make_pair(&a.mc, &b.mc), std::make_pair(&a.s_cmd, &b.s_cmd),
          std::make_pair(&a.a_cmd, &b.a_cmd), std::make_pair(&a.v_stop, &b.v_stop)}) {
      ASSERT_EQ(pr.first->size(), pr.second->size());
      EXPECT_EQ(std::memcmp(pr.first->data(), pr.second->data(), pr.first->size() * 4), 0)
          << "variant " << variant;
    }
  }
}

// The city scene and material planes built by material_build: the planes are sampled from a grid,
// so the barrier's stored unit normal and the material's gradient planes are central differences,
// not the gradients of the bilinear values the potential reads. The residual is the grid's.
TEST(NavPotential, DriveForceOnTheCityScene) {
  const training_scene scene = city_scene(96);
  const field_stack fs = scene.field();
  // Material planes on the scene's grid: smooth risk bumps and three hard patches.
  const int rows = scene.rows, cols = scene.cols;
  std::vector<float> risk(rows * cols, 0.0f);
  std::vector<std::uint8_t> hard(rows * cols, 0);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const double x = c / double(cols - 1), y = r / double(rows - 1);
      risk[r * cols + c] = static_cast<float>(
          0.8 * std::exp(-((x - 0.3) * (x - 0.3) + (y - 0.6) * (y - 0.6)) / 0.02) +
          0.6 * std::exp(-((x - 0.7) * (x - 0.7) + (y - 0.25) * (y - 0.25)) / 0.01));
      hard[r * cols + c] = (r > rows / 2 && r < rows / 2 + 4 && c > 10 && c < 18) ||
                           (c > cols - 20 && c < cols - 16 && r > 20 && r < 30);
    }
  const double cell_w = (scene.max_x - scene.min_x) / (cols - 1);
  const material_planes mp =
      material_build(risk.data(), hard.data(), rows, cols, cell_w, scene.scale, 1.0);
  std::vector<float> mdata;
  for (const auto *pl : {&mp.risk, &mp.phi_m, &mp.grad_rx, &mp.grad_ry, &mp.grad_px, &mp.grad_py})
    mdata.insert(mdata.end(), pl->begin(), pl->end());
  material_stack ms;
  ms.data = mdata.data();
  ms.M = 1;
  ms.H = rows;
  ms.W = cols;
  ms.mnx = scene.min_x;
  ms.mny = scene.min_y;
  ms.mxx = scene.max_x;
  ms.mxy = scene.max_y;
  ms.cx = scene.cx;
  ms.cy = scene.cy;
  ms.S = scene.scale;

  const int n = 512;
  for (int with_mat = 0; with_mat < 2; ++with_mat) {
    drive_setup d = make_drive(n, 31, 0.5f);
    scene.sample_starts_goals(n, 41, d.o.data(), d.carrot.data());
    if (with_mat)
      with_material(d, ms);
    // A cell is 0.0105 normalized here; the step stays inside one.
    const float h = 1e-3f;
    int engaged = 0;
    const std::vector<double> res = drive_residuals(d, fs, h, 0.02f, &engaged);
    const double p50 = quantile(res, 0.5), p90 = quantile(res, 0.9), p99 = quantile(res, 0.99);
    std::printf("  city drive%s: %zu poses (%d in the barrier): rel residual p50 %.2e, p90 %.2e, "
                "p99 %.2e, max %.2e\n",
                with_mat ? " + material" : "", res.size(), engaged, p50, p90, p99, max_of(res));
    EXPECT_GT(res.size(), 400u);
    EXPECT_GT(engaged, 100);
    // The grid's error, not a sign or a scale (measured: p50 1.2e-4, p90 3.4e-4, max 8.7e-3).
    EXPECT_LT(p50, 1e-3);
    EXPECT_LT(p90, 3e-3);
    EXPECT_LT(max_of(res), 5e-2);
  }
}

// ── the golden for GRL-SNAM's torch twin ─────────────────────────────────────────────

namespace {

// The goldens' material surrogate: 3 agents, 3 discs (one padded), 6 steps (agent 2 stops after
// 4), on a curved 9x9 patch whose gradient channels are not its values' gradients.
struct golden_surrogate {
  int B = 3, N = 3, Hp = 9, Wp = 9, Hs = 6;
  std::vector<float> patch;
  std::vector<float> o0 = {0.0f, 0.0f, 1.0f, -0.5f, -1.0f, 0.8f};
  std::vector<float> v0 = {0.2f, -0.1f, 0.0f, 0.0f, -0.3f, 0.2f};
  std::vector<float> goal = {6.0f, 3.0f, -4.0f, 5.0f, 2.0f, -6.0f};
  std::vector<float> C = {2.5f,  0.5f, -1.5f, -2.0f, 0.5f, 3.5f, 3.0f,  1.0f,  0.0f,
                          -2.5f, 1.0f, 2.0f,  -2.5f, 2.0f, 1.0f, -1.5f, -3.0f, 2.0f};
  std::vector<float> R = {0.8f, 0.6f, 1.0f, 0.7f, 0.9f, 0.5f, 0.6f, 0.6f, 0.8f};
  std::vector<std::uint8_t> mask = {1, 1, 1, 1, 1, 0, 1, 1, 1};
  std::vector<float> alphas = {1.2f, 0.7f, 2.0f, 0.9f, 1.5f, 3.0f, 0.4f, 1.1f, 0.8f};
  std::vector<float> beta = {0.15f, 0.08f, 0.2f}, gamma = {1.0f, 2.0f, 0.5f};
  std::vector<float> ls = {1.5f, 0.5f, 2.5f}, lh = {2.0f, 1.0f, 0.5f};
  std::vector<float> rr = {1.0f, 1.2f, 0.8f}, dh = {3.0f, 2.5f, 3.5f};
  std::vector<float> dt = {0.05f, 0.05f, 0.05f};
  std::vector<int> H = {6, 6, 4};
  surrogate_material_params p;
  golden_surrogate() : patch(B * 6 * Hp * Wp) {
    for (int b = 0; b < B; ++b)
      for (int r = 0; r < Hp; ++r)
        for (int c = 0; c < Wp; ++c) {
          const float x = c - 4.0f, y = r - 4.0f;
          float *pb = &patch[b * 6 * Hp * Wp];
          const int k = r * Wp + c, hw = Hp * Wp;
          pb[k] = 0.5f + 0.4f * std::sin(0.35f * x + 0.2f * b) * std::cos(0.25f * y);
          pb[hw + k] = 2.0f + 0.05f * (x - 1.0f) * (x - 1.0f) + 0.3f * y + 0.1f * b;
          pb[2 * hw + k] = 0.14f * std::cos(0.35f * x + 0.2f * b) * std::cos(0.25f * y);
          pb[3 * hw + k] = -0.1f * std::sin(0.35f * x + 0.2f * b) * std::sin(0.25f * y);
          pb[4 * hw + k] = 0.1f * (x - 1.0f);
          pb[5 * hw + k] = 0.3f;
        }
  }
  // Linear channels whose gradient channels are their values' gradients (linear_patch), so the
  // force is -grad U exactly and the books see only the integrator: with the curved patch a step
  // books the gap between the two, work of order dt.
  void use_linear_patch() {
    patch.clear();
    for (int b = 0; b < B; ++b) {
      const std::vector<float> one =
          linear_patch(Hp, Wp, 0.5f, 0.03f, -0.02f, 2.0f + 0.1f * b, 0.1f, 0.3f);
      patch.insert(patch.end(), one.begin(), one.end());
    }
  }
  // The rollout from (o, v) in place, with an optional port force and energy record.
  void run(std::vector<float> &o, std::vector<float> &v, const material_ext *ext,
           rollout_energy *e) const {
    std::vector<float> mc(B), cr(B), hc(B), al(B);
    integrate_surrogate_material(o.data(), v.data(), goal.data(), C.data(), R.data(), mask.data(),
                                 alphas.data(), beta.data(), gamma.data(), ls.data(), lh.data(),
                                 patch.data(), rr.data(), dh.data(), dt.data(), H.data(), B, N, Hp,
                                 Wp, p, mc.data(), cr.data(), hc.data(), al.data(), 1, ext, e);
  }
};

} // namespace

// The values grl_snam/nav_potential.py is held to (float tier): the barrier potentials on a sweep,
// U along a material surrogate rollout (potential, terms, force per step) on a curved patch, and
// drive_potential on a small curved field with a material stack and a footprint.
// CVC_NAV_POTENTIAL_GOLDEN_WRITE=1 rewrites tests/data/nav_potential_golden.json; otherwise this
// build must reproduce it to 1e-6 relative.
TEST(NavPotential, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "nav_potential_golden.json";
  std::vector<double> now; // every computed value, in file order
  std::ostringstream js;
  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.9g", x);
    return std::string(b);
  };
  const auto arr = [&](const float *v, std::size_t n, bool record) {
    std::string out = "[";
    for (std::size_t k = 0; k < n; ++k) {
      out += (k ? ", " : "") + num(v[k]);
      if (record)
        now.push_back(v[k]);
    }
    return out + "]";
  };
  const auto arri = [&](const int *v, std::size_t n) {
    std::string out = "[";
    for (std::size_t k = 0; k < n; ++k)
      out += (k ? ", " : "") + std::to_string(v[k]);
    return out + "]";
  };
  js << "{\n \"format\": \"cvc-nav-potential-golden-1\",\n";

  // 1. Barrier primitives.
  std::vector<float> d, dh_s, b_ipc, b_pw, phi, ks, dhs, b_sp;
  for (const float dh : {0.35f, 3.0f})
    for (int k = 0; k <= 24; ++k) {
      const float x = -0.02f * dh + 1.1f * dh * k / 24.0f + (k == 0 ? 0.0f : 1e-3f * dh * k);
      d.push_back(x);
      dh_s.push_back(dh);
      b_ipc.push_back(detail::ipc_b(x, dh));
      b_pw.push_back(detail::ipc_b_pw(x, dh));
    }
  for (const float k_sharp : {1.25f, 5.0f})
    for (int k = 0; k <= 12; ++k) {
      const float p = -1.0f + 2.0f * k;
      phi.push_back(p);
      ks.push_back(k_sharp);
      dhs.push_back(k_sharp > 2.0f ? 3.0f : 12.0f);
      b_sp.push_back(detail::sdf_barrier_b(p, k_sharp, dhs.back()));
    }
  js << " \"barriers\": {\"d\": " << arr(d.data(), d.size(), false)
     << ",\n  \"d_hat\": " << arr(dh_s.data(), dh_s.size(), false)
     << ",\n  \"ipc_b\": " << arr(b_ipc.data(), b_ipc.size(), true)
     << ",\n  \"ipc_b_pw\": " << arr(b_pw.data(), b_pw.size(), true)
     << ",\n  \"phi\": " << arr(phi.data(), phi.size(), false)
     << ",\n  \"k_sharp\": " << arr(ks.data(), ks.size(), false)
     << ",\n  \"d_hat_sdf\": " << arr(dhs.data(), dhs.size(), false)
     << ",\n  \"sdf_barrier_b\": " << arr(b_sp.data(), b_sp.size(), true) << "},\n";

  // 2. A material surrogate rollout on a curved patch: 3 agents, 3 discs, 6 steps.
  {
    const golden_surrogate g;
    const int B = g.B, N = g.N, Hp = g.Hp, Wp = g.Wp, Hs = g.Hs;
    std::vector<float> o = g.o0, v = g.v0;
    std::vector<float> U(B * (Hs + 1)), T(4 * B * (Hs + 1)), Fc(2 * B * Hs);
    rollout_energy e;
    e.potential = U.data();
    e.terms = T.data();
    e.force = Fc.data();
    g.run(o, v, nullptr, &e);
    std::vector<int> maski(g.mask.begin(), g.mask.end());
    js << " \"material_rollout\": {\"B\": " << B << ", \"N\": " << N << ", \"Hp\": " << Hp
       << ", \"Wp\": " << Wp << ", \"margin_factor\": " << num(g.p.margin_factor)
       << ", \"mass\": " << num(g.p.mass) << ", \"d_hat_sdf\": " << num(g.p.d_hat_sdf)
       << ", \"k_sharp\": " << num(g.p.k_sharp)
       << ",\n  \"o0\": " << arr(g.o0.data(), g.o0.size(), false)
       << ", \"v0\": " << arr(g.v0.data(), g.v0.size(), false)
       << ",\n  \"goal\": " << arr(g.goal.data(), g.goal.size(), false)
       << ",\n  \"C\": " << arr(g.C.data(), g.C.size(), false)
       << ",\n  \"R\": " << arr(g.R.data(), g.R.size(), false)
       << ", \"mask\": " << arri(maski.data(), maski.size())
       << ",\n  \"alphas\": " << arr(g.alphas.data(), g.alphas.size(), false)
       << ",\n  \"beta\": " << arr(g.beta.data(), B, false)
       << ", \"gamma\": " << arr(g.gamma.data(), B, false)
       << ", \"lam_soft\": " << arr(g.ls.data(), B, false)
       << ", \"lam_hard\": " << arr(g.lh.data(), B, false)
       << ",\n  \"rr\": " << arr(g.rr.data(), B, false)
       << ", \"d_hat\": " << arr(g.dh.data(), B, false)
       << ", \"dt\": " << arr(g.dt.data(), B, false) << ", \"H\": " << arri(g.H.data(), B)
       << ",\n  \"patch\": " << arr(g.patch.data(), g.patch.size(), false)
       << ",\n  \"potential\": " << arr(U.data(), U.size(), true)
       << ",\n  \"terms\": " << arr(T.data(), T.size(), true)
       << ",\n  \"force\": " << arr(Fc.data(), Fc.size(), true)
       << ",\n  \"oT\": " << arr(o.data(), o.size(), true)
       << ", \"vT\": " << arr(v.data(), v.size(), true) << "},\n";
  }

  // 3. drive_potential on a small curved field, a material stack and a footprint.
  {
    const int Hf = 12, Wf = 12, hw = Hf * Wf;
    std::vector<float> field(3 * hw), mat(6 * hw);
    for (int r = 0; r < Hf; ++r)
      for (int c = 0; c < Wf; ++c) {
        const float wx = -5.0f + 10.0f * c / (Wf - 1), wy = -5.0f + 10.0f * r / (Hf - 1);
        const float rad = std::sqrt((wx - 1.0f) * (wx - 1.0f) + (wy + 0.5f) * (wy + 0.5f));
        const int k = r * Wf + c;
        field[k] = 0.2f * (rad - 1.5f); // a disc of radius 1.5 m, normalized (S = 0.2)
        field[hw + k] = rad > 0 ? (wx - 1.0f) / rad : 1.0f;
        field[2 * hw + k] = rad > 0 ? (wy + 0.5f) / rad : 0.0f;
        mat[k] = 0.5f + 0.3f * std::sin(0.4f * wx) * std::cos(0.3f * wy);
        mat[hw + k] = 4.0f + 0.4f * wx - 0.2f * wy + 0.05f * wx * wy;
        mat[2 * hw + k] = 0.6f * std::cos(0.4f * wx) * std::cos(0.3f * wy);
        mat[3 * hw + k] = -0.45f * std::sin(0.4f * wx) * std::sin(0.3f * wy);
        mat[4 * hw + k] = 0.4f + 0.05f * wy;
        mat[5 * hw + k] = -0.2f + 0.05f * wx;
      }
    field_stack fsx;
    fsx.data = field.data();
    fsx.M = 1;
    fsx.H = Hf;
    fsx.W = Wf;
    fsx.mnx = fsx.mny = -5.0;
    fsx.mxx = fsx.mxy = 5.0;
    fsx.cx = fsx.cy = 0.0;
    fsx.S = 0.2;
    material_stack ms;
    ms.data = mat.data();
    ms.M = 1;
    ms.H = Hf;
    ms.W = Wf;
    ms.mnx = ms.mny = -5.0;
    ms.mxx = ms.mxy = 5.0;
    ms.cx = ms.cy = 0.0;
    ms.S = 0.2;
    const std::vector<float> o = {-0.3f, 0.2f, 0.5f, 0.05f, 0.1f, -0.6f, -0.7f, -0.7f};
    const std::vector<float> th = {0.3f, -2.0f, 1.2f, 2.9f};
    const std::vector<float> goal = {0.6f, 0.6f, -0.5f, 0.3f, 0.4f, 0.4f, 0.0f, 0.0f};
    const std::vector<float> al = {1.0f, 2.0f, 0.5f, 1.5f}, be = {3.0f, 1.0f, 2.0f, 0.5f};
    std::vector<float> lsd = {0.5f, 0.0f, 1.0f, 0.7f}, lhd = {1.0f, 2.0f, 0.0f, 0.4f};
    material_drive md;
    md.stack = &ms;
    md.lam_soft = lsd.data();
    md.lam_hard = lhd.data();
    md.k_sharp = 1.25f;
    md.d_hat_m = 12.0f;
    std::vector<float> plain_T, fp_T;
    for (int fp = 0; fp < 2; ++fp) {
      veh_params v;
      v.rr = 0.15f;
      v.d_hat = 0.35f;
      const std::vector<float> off = {0.0f, 0.05f, 0.1f};
      if (fp) {
        v.n_body = 3;
        v.body_offsets = off.data();
        v.body_rr = 0.1f;
        v.body_gain = 1.0f / 3.0f;
      }
      for (int i = 0; i < 4; ++i) {
        const potential_terms u =
            drive_potential(fsx, o[2 * i], o[2 * i + 1], th[i], goal[2 * i], goal[2 * i + 1], i,
                            nullptr, v, al[i], be[i], &md, lsd[i], lhd[i]);
        std::vector<float> &out = fp ? fp_T : plain_T;
        for (const float t : {u.goal, u.barrier, u.soft, u.hard, u.terrain})
          out.push_back(t);
      }
    }
    js << " \"drive\": {\"H\": " << Hf << ", \"W\": " << Wf
       << ", \"bounds\": [-5, -5, 5, 5], \"center\": [0, 0], \"S\": 0.2"
       << ",\n  \"field\": " << arr(field.data(), field.size(), false)
       << ",\n  \"material\": " << arr(mat.data(), mat.size(), false)
       << ",\n  \"k_sharp\": 1.25, \"d_hat_m\": 12, \"rr\": 0.15, \"d_hat\": 0.35"
       << ", \"body_offsets\": [0, 0.05, 0.1], \"body_rr\": 0.1, \"body_gain\": "
       << num(1.0f / 3.0f) << ",\n  \"o\": " << arr(o.data(), o.size(), false)
       << ", \"th\": " << arr(th.data(), th.size(), false)
       << ",\n  \"goal\": " << arr(goal.data(), goal.size(), false)
       << ", \"alpha\": " << arr(al.data(), al.size(), false)
       << ", \"beta\": " << arr(be.data(), be.size(), false)
       << ",\n  \"lam_soft\": " << arr(lsd.data(), lsd.size(), false)
       << ", \"lam_hard\": " << arr(lhd.data(), lhd.size(), false)
       << ",\n  \"terms_order\": [\"goal\", \"barrier\", \"soft\", \"hard\", \"terrain\"]"
       << ",\n  \"terms\": " << arr(plain_T.data(), plain_T.size(), true)
       << ",\n  \"terms_footprint\": " << arr(fp_T.data(), fp_T.size(), true) << "}\n}\n";
  }

  const char *write = std::getenv("CVC_NAV_POTENTIAL_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  const auto take = [&](const char *sect, const char *key) {
    for (const auto &x : want.get_child(sect).get_child(key))
      golden.push_back(x.second.get_value<double>());
  };
  take("barriers", "ipc_b");
  take("barriers", "ipc_b_pw");
  take("barriers", "sdf_barrier_b");
  for (const char *k : {"potential", "terms", "force", "oT", "vT"})
    take("material_rollout", k);
  take("drive", "terms");
  take("drive", "terms_footprint");
  ASSERT_EQ(golden.size(), now.size());
  for (std::size_t k = 0; k < now.size(); ++k)
    EXPECT_LE(std::fabs(now[k] - golden[k]),
              1e-6 * std::max(1.0, std::max(std::fabs(now[k]), std::fabs(golden[k]))))
        << "value " << k << ": golden " << golden[k] << ", now " << now[k];
}

namespace {

// The coef_train surrogate's conservative force at o, F_bar + F_goal, in surr_step's op order.
void surr_force_at(const diff::field &F, float ox, float oy, float gx, float gy, float al, float be,
                   float rr, float d_hat, float &fx, float &fy) {
  const diff::sample s = diff::sample_fwd(F, ox, oy);
  const float ipc = diff::ipc_(s.phi - rr, d_hat);
  fx = -(al * ipc) * s.nx - be * (ox - gx);
  fy = -(al * ipc) * s.ny - be * (oy - gy);
}

} // namespace

// The values the torch twin's books are held to (tests/data/nav_energy_books_golden.json):
//   - bound: energy_residual_bound on a sweep, mass and kappa_H included (double).
//   - clamp: a diff::surr_step rollout whose vmax clamp cuts every step, booked per step as
//     ClampWorkClosesTheBooks books it: the states (float), book_step's arguments and its books
//     (double), so the twin's book_step is held to them exactly.
//   - sharp: the material surrogate of NavPotential.Golden on a linear patch, with a port force
//     and the sharp check: every book rollout_energy has (agent 2 stops early: its frozen steps
//     book zero).
//   - unlogged: the same rollout at dt 0.002 under a strong port force, where leaving W_port out
//     of the books fails the bound at every step (the alarm).
// CVC_NAV_ENERGY_BOOKS_GOLDEN_WRITE=1 rewrites the file; otherwise this build must reproduce it
// to 1e-6 relative: of max(1, |x|) for states, potentials and kinetic energies, as
// NavPotential.Golden, and of max(|x|, the step's bound) for the books.
TEST(NavPotential, BooksGolden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "nav_energy_books_golden.json";
  struct checked {
    std::string sect, key;
    std::vector<double> now, floor;
  };
  std::vector<checked> now; // every computed value, by section and key
  std::ostringstream js;
  const auto num = [](double x, int digits) {
    char b[40];
    std::snprintf(b, sizeof b, "%.*g", digits, x);
    return std::string(b);
  };
  const auto arr = [&](const auto *v, std::size_t n, int digits) {
    std::string out = "[";
    for (std::size_t k = 0; k < n; ++k)
      out += (k ? ", " : "") + num(static_cast<double>(v[k]), digits);
    return out + "]";
  };
  // "key": [...], its values checked against max(|x|, floor(k)).
  const auto put = [&](const char *sect, const char *key, const auto *v, std::size_t n, int digits,
                       const std::function<double(std::size_t)> &floor) {
    checked c{sect, key, {}, {}};
    for (std::size_t k = 0; k < n; ++k) {
      c.now.push_back(static_cast<double>(v[k]));
      c.floor.push_back(floor(k));
    }
    now.push_back(c);
    return "\"" + std::string(key) + "\": " + arr(v, n, digits);
  };
  const auto one = [](std::size_t) { return 1.0; };
  const auto none = [](std::size_t) { return 0.0; };
  js << "{\n \"format\": \"cvc-nav-energy-books-golden-1\",\n";

  // 1. energy_residual_bound.
  {
    std::vector<double> dt, g, ga, v, F, L, vt, m, kh, out;
    for (int k = 0; k < 12; ++k) {
      dt.push_back(k % 3 == 0 ? 0.002 : (k % 3 == 1 ? 0.01 : 0.05));
      g.push_back(0.3 + 0.7 * k);
      ga.push_back(0.25 * (k % 4));
      v.push_back(0.5 + 0.2 * k);
      F.push_back(k % 2 ? 0.0 : 1.5 + 0.1 * k);
      L.push_back(0.1 * k * k);
      vt.push_back(0.4 + 0.15 * k);
      m.push_back(k % 4 == 3 ? 2.5 : 1.0);
      kh.push_back(k % 5 == 4 ? 3.0 : 2.0);
      out.push_back(
          energy_residual_bound(dt[k], g[k], ga[k], v[k], F[k], L[k], vt[k], m[k], kh[k]));
    }
    js << " \"bound\": {\"dt\": " << arr(dt.data(), dt.size(), 17)
       << ", \"g\": " << arr(g.data(), g.size(), 17)
       << ",\n  \"gamma\": " << arr(ga.data(), ga.size(), 17)
       << ", \"v\": " << arr(v.data(), v.size(), 17)
       << ",\n  \"F\": " << arr(F.data(), F.size(), 17)
       << ", \"L_hat\": " << arr(L.data(), L.size(), 17)
       << ",\n  \"vt\": " << arr(vt.data(), vt.size(), 17)
       << ", \"mass\": " << arr(m.data(), m.size(), 17)
       << ", \"kappa_H\": " << arr(kh.data(), kh.size(), 17) << ",\n  "
       << put("bound", "bound", out.data(), out.size(), 17, none) << "},\n";
  }

  // 2. diff::surr_step with its vmax clamp, on ClampWorkClosesTheBooks' wall (exact on any grid,
  // so a 5x5 one; phi = 0.6 x - 0.8 y + 0.3 normalized): three agents in its barrier band moving
  // along it and one clear of it, starting at 3 vmax with the goal spring along their motion, so
  // the clamp cuts every step.
  {
    const int B = 4, S = 8, Hf = 5, Wf = 5, hw = Hf * Wf;
    const float hdt = 0.03f, vmax = 0.5f, rr = 0.15f, d_hat = 0.35f;
    std::vector<float> field(3 * hw);
    for (int r = 0; r < Hf; ++r)
      for (int c = 0; c < Wf; ++c) {
        const double wx = -10.0 + 20.0 * c / (Wf - 1), wy = -10.0 + 20.0 * r / (Hf - 1);
        field[r * Wf + c] = static_cast<float>(0.1 * (0.6 * wx - 0.8 * wy + 3.0));
        field[hw + r * Wf + c] = 0.6f;
        field[2 * hw + r * Wf + c] = -0.8f;
      }
    field_stack fsx;
    fsx.data = field.data();
    fsx.M = 1;
    fsx.H = Hf;
    fsx.W = Wf;
    fsx.mnx = fsx.mny = -10.0;
    fsx.mxx = fsx.mxy = 10.0;
    fsx.cx = fsx.cy = 0.0;
    fsx.S = 0.1;
    const diff::field F = linear_sdf::to_diff(fsx);
    const std::vector<float> o0 = {0.1f, 0.0f, -0.1f, -0.05f, 0.3f, 0.1f, 0.6f, -0.3f};
    const std::vector<float> goal = {0.9f, 0.6f, -0.9f, -0.65f, -0.42f, -0.44f, -0.4f, 0.5f};
    const std::vector<float> al = {1.0f, 2.0f, 0.6f, 1.5f}, be = {2.5f, 2.0f, 3.0f, 1.5f};
    const std::vector<float> ga = {0.4f, 0.2f, 0.6f, 0.3f};
    std::vector<float> v0(2 * B);
    for (int b = 0; b < B; ++b) {
      const float ex = goal[2 * b] - o0[2 * b], ey = goal[2 * b + 1] - o0[2 * b + 1];
      const float n = std::sqrt(ex * ex + ey * ey);
      v0[2 * b] = 3.0f * vmax * ex / n;
      v0[2 * b + 1] = 3.0f * vmax * ey / n;
    }
    // Per (agent, step): the states, book_step's arguments and its books.
    std::vector<float> os(2 * B * (S + 1)), vs(2 * B * (S + 1));
    std::vector<double> av, avp, avt, adx, ag, ag1, aU0, aU1, avHv;
    std::vector<double> kin, dH, wd, wp, wc, wx, res, rsh, bnd;
    int in_band = 0;
    for (int b = 0; b < B; ++b) {
      float ox = o0[2 * b], oy = o0[2 * b + 1], vx = v0[2 * b], vy = v0[2 * b + 1];
      const float gx = goal[2 * b], gy = goal[2 * b + 1];
      for (int s = 0; s <= S; ++s) {
        const std::size_t row = static_cast<std::size_t>(b) * (S + 1) + s;
        os[2 * row] = ox;
        os[2 * row + 1] = oy;
        vs[2 * row] = vx;
        vs[2 * row + 1] = vy;
        if (s == S)
          break;
        in_band += diff::sample_fwd(F, ox, oy).phi - rr < d_hat;
        float f0x, f0y, f1x, f1y, ox1, oy1, vx1, vy1;
        surr_force_at(F, ox, oy, gx, gy, al[b], be[b], rr, d_hat, f0x, f0y);
        diff::surr_step(F, ox, oy, vx, vy, gx, gy, al[b], be[b], ga[b], rr, d_hat, vmax, hdt, ox1,
                        oy1, vx1, vy1);
        surr_force_at(F, ox1, oy1, gx, gy, al[b], be[b], rr, d_hat, f1x, f1y);
        book_step_args a;
        a.dt = hdt;
        a.gamma = ga[b];
        const float f0[2] = {f0x, f0y}, f1[2] = {f1x, f1y}, v[2] = {vx, vy}, vt[2] = {vx1, vy1};
        const float x0[2] = {ox, oy}, x1[2] = {ox1, oy1};
        for (int c = 0; c < 2; ++c) {
          a.v[c] = v[c];
          a.vp[c] = a.v[c] + a.dt * (static_cast<double>(f0[c]) - a.gamma * a.v[c]);
          a.vt[c] = vt[c];
          a.dx[c] = static_cast<double>(x1[c]) - x0[c];
          a.g[c] = -static_cast<double>(f0[c]);
          a.g1[c] = -static_cast<double>(f1[c]);
        }
        a.U0 = diff::surr_potential(F, ox, oy, gx, gy, al[b], be[b], rr, d_hat).total();
        a.U1 = diff::surr_potential(F, ox1, oy1, gx, gy, al[b], be[b], rr, d_hat).total();
        // vt' grad^2 U vt from the force at x -+ vt dt/2 (float positions), as the clamp test.
        const double h = 0.5 * a.dt;
        float fpx, fpy, fmx, fmy;
        surr_force_at(F, static_cast<float>(ox + h * vx1), static_cast<float>(oy + h * vy1), gx, gy,
                      al[b], be[b], rr, d_hat, fpx, fpy);
        surr_force_at(F, static_cast<float>(ox - h * vx1), static_cast<float>(oy - h * vy1), gx, gy,
                      al[b], be[b], rr, d_hat, fmx, fmy);
        a.vHv = -(a.vt[0] * (static_cast<double>(fpx) - fmx) +
                  a.vt[1] * (static_cast<double>(fpy) - fmy)) /
                (2.0 * h);
        const step_books k = book_step(a);
        EXPECT_GT(k.w_clamp, 0.0) << "agent " << b << ", step " << s << ": the clamp is off";
        EXPECT_LE(std::fabs(k.residual), k.bound) << "agent " << b << ", step " << s;
        for (int c = 0; c < 2; ++c) {
          av.push_back(a.v[c]);
          avp.push_back(a.vp[c]);
          avt.push_back(a.vt[c]);
          adx.push_back(a.dx[c]);
          ag.push_back(a.g[c]);
          ag1.push_back(a.g1[c]);
        }
        aU0.push_back(a.U0);
        aU1.push_back(a.U1);
        avHv.push_back(a.vHv);
        kin.push_back(k.kinetic);
        dH.push_back(k.delta_H);
        wd.push_back(k.w_damp);
        wp.push_back(k.w_port);
        wc.push_back(k.w_clamp);
        wx.push_back(k.w_cross);
        res.push_back(k.residual);
        rsh.push_back(k.residual_sharp);
        bnd.push_back(k.bound);
        ox = ox1;
        oy = oy1;
        vx = vx1;
        vy = vy1;
      }
    }
    EXPECT_GE(in_band, 2 * S); // the barrier is engaged along the way
    const auto by_bound = [&](std::size_t k) { return bnd[k]; };
    js << " \"clamp\": {\"B\": " << B << ", \"steps\": " << S << ", \"H\": " << Hf
       << ", \"W\": " << Wf << ", \"bounds\": [-10, -10, 10, 10], \"center\": [0, 0], \"S\": 0.1"
       << ",\n  \"field\": " << arr(field.data(), field.size(), 9)
       << ",\n  \"hdt\": " << num(hdt, 9) << ", \"vmax\": " << num(vmax, 9)
       << ", \"rr\": " << num(rr, 9) << ", \"d_hat\": " << num(d_hat, 9)
       << ",\n  \"o0\": " << arr(o0.data(), o0.size(), 9)
       << ", \"v0\": " << arr(v0.data(), v0.size(), 9)
       << ",\n  \"goal\": " << arr(goal.data(), goal.size(), 9)
       << ", \"alpha\": " << arr(al.data(), B, 9) << ", \"beta\": " << arr(be.data(), B, 9)
       << ", \"gamma\": " << arr(ga.data(), B, 9) << ",\n  "
       << put("clamp", "o", os.data(), os.size(), 9, one) << ",\n  "
       << put("clamp", "v", vs.data(), vs.size(), 9, one)
       << ",\n  \"args\": {\"v\": " << arr(av.data(), av.size(), 17)
       << ",\n   \"vp\": " << arr(avp.data(), avp.size(), 17)
       << ",\n   \"vt\": " << arr(avt.data(), avt.size(), 17)
       << ",\n   \"dx\": " << arr(adx.data(), adx.size(), 17)
       << ",\n   \"g\": " << arr(ag.data(), ag.size(), 17)
       << ",\n   \"g1\": " << arr(ag1.data(), ag1.size(), 17)
       << ",\n   \"U0\": " << arr(aU0.data(), aU0.size(), 17)
       << ",\n   \"U1\": " << arr(aU1.data(), aU1.size(), 17)
       << ",\n   \"vHv\": " << arr(avHv.data(), avHv.size(), 17) << "},\n  \"books\": {"
       << put("clamp", "kinetic", kin.data(), kin.size(), 17, one) << ",\n   "
       << put("clamp", "delta_H", dH.data(), dH.size(), 17, by_bound) << ",\n   "
       << put("clamp", "w_damp", wd.data(), wd.size(), 17, by_bound) << ",\n   "
       << put("clamp", "w_port", wp.data(), wp.size(), 17, by_bound) << ",\n   "
       << put("clamp", "w_clamp", wc.data(), wc.size(), 17, by_bound) << ",\n   "
       << put("clamp", "w_cross", wx.data(), wx.size(), 17, by_bound) << ",\n   "
       << put("clamp", "residual", res.data(), res.size(), 17, by_bound) << ",\n   "
       << put("clamp", "residual_sharp", rsh.data(), rsh.size(), 17, by_bound) << ",\n   "
       << put("clamp", "bound", bnd.data(), bnd.size(), 17, by_bound) << "}},\n";
  }

  // 3 and 4. The material surrogate's books from its energy hook.
  struct books_run {
    std::vector<float> U, ke, wd, wp, wc, wx, r, rs, bound, o, v;
  };
  golden_surrogate g;
  g.use_linear_patch();
  const auto run_books = [&](const golden_surrogate &gs, const std::vector<float> &v0,
                             const std::vector<float> &Fext, bool sharp) {
    books_run k;
    const int B = gs.B, S = *std::max_element(gs.H.begin(), gs.H.end());
    k.U.assign(B * (S + 1), NAN);
    k.ke.assign(B * (S + 1), NAN);
    for (auto *x : {&k.wd, &k.wp, &k.wc, &k.wx, &k.r, &k.rs, &k.bound})
      x->assign(B * S, NAN);
    rollout_energy e;
    e.potential = k.U.data();
    e.kinetic = k.ke.data();
    e.w_damp = k.wd.data();
    e.w_port = k.wp.data();
    e.w_clamp = k.wc.data();
    e.w_cross = k.wx.data();
    e.residual = k.r.data();
    e.residual_sharp = sharp ? k.rs.data() : nullptr;
    e.bound = k.bound.data();
    e.sharp = sharp;
    material_ext ext;
    ext.F = Fext.data();
    ext.steps = S;
    k.o = gs.o0;
    k.v = v0;
    gs.run(k.o, k.v, &ext, &e);
    return k;
  };
  const auto material_inputs = [&](const golden_surrogate &gs, const std::vector<float> &v0,
                                   const std::vector<float> &Fext) {
    std::vector<int> maski(gs.mask.begin(), gs.mask.end());
    std::ostringstream s;
    s << "\"B\": " << gs.B << ", \"N\": " << gs.N << ", \"Hp\": " << gs.Hp << ", \"Wp\": " << gs.Wp
      << ", \"margin_factor\": " << num(gs.p.margin_factor, 9)
      << ", \"mass\": " << num(gs.p.mass, 9) << ", \"d_hat_sdf\": " << num(gs.p.d_hat_sdf, 9)
      << ", \"k_sharp\": " << num(gs.p.k_sharp, 9)
      << ",\n  \"o0\": " << arr(gs.o0.data(), gs.o0.size(), 9)
      << ", \"v0\": " << arr(v0.data(), v0.size(), 9)
      << ",\n  \"goal\": " << arr(gs.goal.data(), gs.goal.size(), 9)
      << ",\n  \"C\": " << arr(gs.C.data(), gs.C.size(), 9)
      << ",\n  \"R\": " << arr(gs.R.data(), gs.R.size(), 9)
      << ", \"mask\": " << arr(maski.data(), maski.size(), 9)
      << ",\n  \"alphas\": " << arr(gs.alphas.data(), gs.alphas.size(), 9)
      << ",\n  \"beta\": " << arr(gs.beta.data(), gs.B, 9)
      << ", \"gamma\": " << arr(gs.gamma.data(), gs.B, 9)
      << ", \"lam_soft\": " << arr(gs.ls.data(), gs.B, 9)
      << ", \"lam_hard\": " << arr(gs.lh.data(), gs.B, 9)
      << ",\n  \"rr\": " << arr(gs.rr.data(), gs.B, 9)
      << ", \"d_hat\": " << arr(gs.dh.data(), gs.B, 9) << ", \"dt\": " << arr(gs.dt.data(), gs.B, 9)
      << ", \"H\": " << arr(gs.H.data(), gs.B, 9)
      << ",\n  \"F_ext\": " << arr(Fext.data(), Fext.size(), 9)
      << ",\n  \"patch\": " << arr(gs.patch.data(), gs.patch.size(), 9);
    return s.str();
  };
  const auto books_json = [&](const char *sect, const books_run &k, bool sharp) {
    const auto by_bound = [&k](std::size_t i) { return static_cast<double>(k.bound[i]); };
    std::string out = put(sect, "potential", k.U.data(), k.U.size(), 9, one) + ",\n  " +
                      put(sect, "kinetic", k.ke.data(), k.ke.size(), 9, one) + ",\n  " +
                      put(sect, "w_damp", k.wd.data(), k.wd.size(), 9, by_bound) + ",\n  " +
                      put(sect, "w_port", k.wp.data(), k.wp.size(), 9, by_bound) + ",\n  " +
                      put(sect, "w_clamp", k.wc.data(), k.wc.size(), 9, by_bound) + ",\n  " +
                      put(sect, "w_cross", k.wx.data(), k.wx.size(), 9, by_bound) + ",\n  " +
                      put(sect, "residual", k.r.data(), k.r.size(), 9, by_bound) + ",\n  ";
    if (sharp)
      out += put(sect, "residual_sharp", k.rs.data(), k.rs.size(), 9, by_bound) + ",\n  ";
    return out + put(sect, "bound", k.bound.data(), k.bound.size(), 9, by_bound) + ",\n  " +
           put(sect, "oT", k.o.data(), k.o.size(), 9, one) + ", " +
           put(sect, "vT", k.v.data(), k.v.size(), 9, one);
  };

  // 3. A port force and the sharp check.
  {
    const int B = g.B, S = g.Hs;
    std::vector<float> Fext(2 * B * S);
    for (int b = 0; b < B; ++b)
      for (int s = 0; s < S; ++s) {
        Fext[2 * (b * S + s)] = 0.8f * std::sin(1.3f * b + 0.4f * s);
        Fext[2 * (b * S + s) + 1] = -0.6f * std::cos(0.9f * b + 0.3f * s);
      }
    const books_run k = run_books(g, g.v0, Fext, true);
    for (int b = 0; b < B; ++b)
      for (int s = 0; s < S; ++s) {
        const int i = b * S + s;
        EXPECT_EQ(k.wc[i], 0.0f);
        EXPECT_EQ(k.wx[i], 0.0f);
        if (s < g.H[b]) {
          EXPECT_LE(std::fabs(k.r[i]), k.bound[i]) << "agent " << b << ", step " << s;
          EXPECT_NE(k.wp[i], 0.0f);
        } else { // frozen
          for (const float x : {k.wd[i], k.wp[i], k.r[i], k.rs[i], k.bound[i]})
            EXPECT_EQ(x, 0.0f) << "agent " << b << ", step " << s;
        }
      }
    js << " \"sharp\": {" << material_inputs(g, g.v0, Fext) << ",\n  "
       << books_json("sharp", k, true) << "},\n";
  }

  // 4. An unlogged port: 3 units of force along each agent's heading at dt 0.002, booked as W_port
  // (every step closes within the bound) and then left out (r_E + W_port fails it every step).
  {
    golden_surrogate gs = g;
    const int B = gs.B, S = 10;
    std::vector<float> v0(2 * B), Fext(2 * B * S);
    for (int b = 0; b < B; ++b) {
      gs.dt[b] = 0.002f;
      gs.H[b] = S;
      const float c = std::cos(0.7f * b), s = std::sin(0.7f * b);
      v0[2 * b] = c;
      v0[2 * b + 1] = s;
      for (int k = 0; k < S; ++k) {
        Fext[2 * (b * S + k)] = 3.0f * c;
        Fext[2 * (b * S + k) + 1] = 3.0f * s;
      }
    }
    const books_run k = run_books(gs, v0, Fext, false);
    std::vector<int> alarm(B * S);
    double smallest = 1e30;
    for (int i = 0; i < B * S; ++i) {
      EXPECT_LE(std::fabs(k.r[i]), k.bound[i]) << "step " << i;
      const double unbooked = std::fabs(static_cast<double>(k.r[i]) + k.wp[i]) / k.bound[i];
      alarm[i] = unbooked > 1.0;
      smallest = std::min(smallest, unbooked);
    }
    EXPECT_GT(smallest, 3.0) << "every step alarms, with a margin"; // measured: 6.6
    std::printf("  books golden: unlogged port, smallest |r_E + W_port|/bound %.1f\n", smallest);
    js << " \"unlogged\": {" << material_inputs(gs, v0, Fext) << ",\n  "
       << books_json("unlogged", k, false)
       << ",\n  \"alarm\": " << arr(alarm.data(), alarm.size(), 9) << "}\n}\n";
  }

  const char *write = std::getenv("CVC_NAV_ENERGY_BOOKS_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  for (const checked &c : now) {
    boost::property_tree::ptree sect = want.get_child(c.sect);
    if (c.sect == "clamp" && c.key != "o" && c.key != "v")
      sect = sect.get_child("books");
    std::vector<double> golden;
    for (const auto &x : sect.get_child(c.key))
      golden.push_back(x.second.get_value<double>());
    ASSERT_EQ(golden.size(), c.now.size()) << c.sect << "." << c.key;
    for (std::size_t k = 0; k < golden.size(); ++k)
      EXPECT_LE(std::fabs(c.now[k] - golden[k]),
                1e-6 * std::max({std::fabs(c.now[k]), std::fabs(golden[k]), c.floor[k]}))
          << c.sect << "." << c.key << "[" << k << "]: golden " << golden[k] << ", now "
          << c.now[k];
  }
}

// ── the tick's books in sim_world (drive_telemetry) ─────────────────────────────────────────

namespace {

// An open 200 m field (the NavDriveTelemetry setup), normalized = world * 0.02.
sim_world::config open_field() {
  sim_world::config cfg;
  cfg.rows = 64;
  cfg.cols = 64;
  cfg.min_x = cfg.min_y = -100;
  cfg.max_x = cfg.max_y = 100;
  cfg.scale = 0.02;
  cfg.veh.rr = 3.0f;
  cfg.veh.d_hat = 7.0f;
  cfg.veh.dt = 0.06f;
  cfg.veh.nsub = 1;
  cfg.freeze_sense = true;
  return cfg;
}

struct tick_run {
  std::vector<drive_telemetry> first, before, after;
  std::vector<float> pos;
};

// Three ticks, then (optionally) agents 0 and 2 retargeted 1 normalized unit ahead of where they
// are, then one more tick.
tick_run run_goal_change(bool capture, bool retarget) {
  const sim_world::config cfg = open_field();
  const int N = 4;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(cfg.rows) * cfg.cols, 0);
  sim_world w = sim_world::from_occupancy(cfg, occ.data(), coef_mlp::default_biased(), N, 7);
  w.set_capture_drive_telemetry(capture);
  tick_run r;
  const auto tel = [&](std::vector<drive_telemetry> &out) {
    if (capture)
      out.assign(w.drive_telemetry_data(), w.drive_telemetry_data() + N);
  };
  w.step(1);
  tel(r.first);
  w.step(1);
  w.step(1);
  tel(r.before);
  std::vector<float> pos(2 * N), hd(N), sp(N);
  std::vector<int> md(N);
  std::vector<std::uint8_t> rc(N);
  if (retarget) {
    w.snapshot(pos.data(), hd.data(), sp.data(), md.data(), rc.data());
    for (const int i : {0, 2})
      w.retarget(i, static_cast<float>(pos[2 * i] * cfg.scale) + 1.0f,
                 static_cast<float>(pos[2 * i + 1] * cfg.scale));
  }
  w.step(1);
  tel(r.after);
  w.snapshot(pos.data(), hd.data(), sp.data(), md.data(), rc.data());
  r.pos = pos;
  return r;
}

} // namespace

// A goal changed between two ticks is context work: U at the new tick's start pose with the new
// context (drive_telemetry::potential) minus U at the same pose with the old one, which, with
// nothing moving the agents between the ticks, is the old tick's potential_end bit for bit.
// Capturing the books changes no output.
TEST(NavPotential, SimWorldBooksTheContextWorkOfAGoalChange) {
  const tick_run off = run_goal_change(false, true);
  const tick_run on = run_goal_change(true, true);
  const tick_run still = run_goal_change(true, false);
  ASSERT_EQ(off.pos.size(), on.pos.size());
  EXPECT_EQ(std::memcmp(off.pos.data(), on.pos.data(), on.pos.size() * 4), 0);
  const int N = static_cast<int>(on.after.size());
  for (int i = 0; i < N; ++i) {
    EXPECT_EQ(on.first[i].context_work, 0.0f) << "no context before the first captured tick";
    // No override, no min_gap: nothing moves an agent after the drive.
    EXPECT_EQ(on.before[i].projection_work, 0.0f);
    EXPECT_EQ(on.after[i].projection_work, 0.0f);
    // The ledger closes across the tick boundary exactly.
    EXPECT_EQ(on.after[i].context_work, on.after[i].potential - on.before[i].potential_end)
        << "agent " << i;
    EXPECT_EQ(still.after[i].context_work, still.after[i].potential - still.before[i].potential_end)
        << "agent " << i;
    const bool moved = i == 0 || i == 2;
    std::printf("  agent %d%s: context work %+.4f (without the goal change %+.4f), U %.4f -> "
                "%.4f\n",
                i, moved ? " (retargeted)" : "", on.after[i].context_work,
                still.after[i].context_work, on.before[i].potential_end, on.after[i].potential);
    if (moved) {
      // The carrot comes in from 1.8 to 1 unit: the goal spring's energy drops.
      EXPECT_LT(on.after[i].context_work, 0.0f);
      EXPECT_GT(std::fabs(on.after[i].context_work),
                10.0f * std::fabs(still.after[i].context_work));
    } else {
      EXPECT_EQ(on.after[i].context_work, still.after[i].context_work);
    }
  }
}

// min_gap's push after the drive is projection work: potential_end minus U where the drive left
// the agent, nonzero for the pushed pair and exactly zero for the rest.
TEST(NavPotential, SimWorldBooksTheMinGapPushAsProjectionWork) {
  sim_world::config cfg = open_field();
  cfg.min_gap = 0.5f;
  const int N = 4;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(cfg.rows) * cfg.cols, 0);
  sim_world w = sim_world::from_occupancy(cfg, occ.data(), coef_mlp::default_biased(), N, 7);
  w.set_capture_drive_telemetry(true);
  ASSERT_TRUE(w.set_pose(0, 0.0f, 0.0f, 0.0f));
  ASSERT_TRUE(w.set_pose(1, 0.1f, 0.0f, 0.0f));
  ASSERT_TRUE(w.set_pose(2, -1.5f, 1.5f, 0.0f));
  ASSERT_TRUE(w.set_pose(3, 1.5f, -1.5f, 0.0f));
  w.step(1);
  const drive_telemetry *t = w.drive_telemetry_data();
  ASSERT_NE(t, nullptr);
  EXPECT_NE(t[0].projection_work, 0.0f);
  EXPECT_NE(t[1].projection_work, 0.0f);
  EXPECT_EQ(t[2].projection_work, 0.0f);
  EXPECT_EQ(t[3].projection_work, 0.0f);
  std::printf("  min_gap push: projection work %+.4f, %+.4f\n", t[0].projection_work,
              t[1].projection_work);
}
