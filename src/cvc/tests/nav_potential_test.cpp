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
//   - NavPotential.Golden writes or checks tests/data/nav_potential_golden.json, the values
//     GRL-SNAM's torch twin (grl_snam/nav_potential.py) is held to.

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
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
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
    const int B = 3, N = 3, Hp = 9, Wp = 9, Hs = 6;
    std::vector<float> patch(B * 6 * Hp * Wp);
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
    std::vector<float> o = {0.0f, 0.0f, 1.0f, -0.5f, -1.0f, 0.8f};
    std::vector<float> v = {0.2f, -0.1f, 0.0f, 0.0f, -0.3f, 0.2f};
    const std::vector<float> goal = {6.0f, 3.0f, -4.0f, 5.0f, 2.0f, -6.0f};
    const std::vector<float> C = {2.5f,  0.5f, -1.5f, -2.0f, 0.5f, 3.5f, 3.0f,  1.0f,  0.0f,
                                  -2.5f, 1.0f, 2.0f,  -2.5f, 2.0f, 1.0f, -1.5f, -3.0f, 2.0f};
    const std::vector<float> R = {0.8f, 0.6f, 1.0f, 0.7f, 0.9f, 0.5f, 0.6f, 0.6f, 0.8f};
    const std::vector<std::uint8_t> mask = {1, 1, 1, 1, 1, 0, 1, 1, 1};
    const std::vector<float> alphas = {1.2f, 0.7f, 2.0f, 0.9f, 1.5f, 3.0f, 0.4f, 1.1f, 0.8f};
    const std::vector<float> beta = {0.15f, 0.08f, 0.2f}, gamma = {1.0f, 2.0f, 0.5f};
    const std::vector<float> ls = {1.5f, 0.5f, 2.5f}, lh = {2.0f, 1.0f, 0.5f};
    const std::vector<float> rr = {1.0f, 1.2f, 0.8f}, dh = {3.0f, 2.5f, 3.5f};
    const std::vector<float> dt = {0.05f, 0.05f, 0.05f};
    const std::vector<int> H = {6, 6, 4};
    surrogate_material_params p;
    std::vector<float> mc(B), cr(B), hc(B), al(B);
    std::vector<float> U(B * (Hs + 1)), T(4 * B * (Hs + 1)), Fc(2 * B * Hs);
    rollout_energy e;
    e.potential = U.data();
    e.terms = T.data();
    e.force = Fc.data();
    const std::vector<float> o0 = o, v0 = v;
    integrate_surrogate_material(o.data(), v.data(), goal.data(), C.data(), R.data(), mask.data(),
                                 alphas.data(), beta.data(), gamma.data(), ls.data(), lh.data(),
                                 patch.data(), rr.data(), dh.data(), dt.data(), H.data(), B, N, Hp,
                                 Wp, p, mc.data(), cr.data(), hc.data(), al.data(), 1, nullptr, &e);
    std::vector<int> maski(mask.begin(), mask.end());
    js << " \"material_rollout\": {\"B\": " << B << ", \"N\": " << N << ", \"Hp\": " << Hp
       << ", \"Wp\": " << Wp << ", \"margin_factor\": " << num(p.margin_factor)
       << ", \"mass\": " << num(p.mass) << ", \"d_hat_sdf\": " << num(p.d_hat_sdf)
       << ", \"k_sharp\": " << num(p.k_sharp) << ",\n  \"o0\": " << arr(o0.data(), o0.size(), false)
       << ", \"v0\": " << arr(v0.data(), v0.size(), false)
       << ",\n  \"goal\": " << arr(goal.data(), goal.size(), false)
       << ",\n  \"C\": " << arr(C.data(), C.size(), false)
       << ",\n  \"R\": " << arr(R.data(), R.size(), false)
       << ", \"mask\": " << arri(maski.data(), maski.size())
       << ",\n  \"alphas\": " << arr(alphas.data(), alphas.size(), false)
       << ",\n  \"beta\": " << arr(beta.data(), B, false)
       << ", \"gamma\": " << arr(gamma.data(), B, false)
       << ", \"lam_soft\": " << arr(ls.data(), B, false)
       << ", \"lam_hard\": " << arr(lh.data(), B, false)
       << ",\n  \"rr\": " << arr(rr.data(), B, false) << ", \"d_hat\": " << arr(dh.data(), B, false)
       << ", \"dt\": " << arr(dt.data(), B, false) << ", \"H\": " << arri(H.data(), B)
       << ",\n  \"patch\": " << arr(patch.data(), patch.size(), false)
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
