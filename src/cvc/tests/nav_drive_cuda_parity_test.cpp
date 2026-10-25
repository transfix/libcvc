/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_drive_cuda_parity_test — the fused GPU drive (drive_step_cuda, drive_step_material_cuda)
// against the CPU drive (drive_step, drive_step_material) with everything the GPU now applies:
// a material stack (its soft and hard forces), a terrain (grade force, infeasible-slope barrier,
// grade cost as risk), the slope governors, and nets that read the risk and slope look-ahead
// columns and carry learned lam heads (a v1 softplus head and a two-head sigmoid net). Each
// tick starts both drives from the CPU's state and compares one tick, so the float-equivalence
// contract (~1e-5) is checked tick by tick rather than through a diverging trajectory. Built
// with CVC_ENABLE_CUDA; the parity cases skip without a device, the refusals need none.

#include <gtest/gtest.h>

#ifndef CVC_ENABLE_CUDA
TEST(NavDriveCudaParity, SkippedNoCuda) { GTEST_SKIP() << "built without CVC_ENABLE_CUDA"; }
#else

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/material.h>
#include <random>
#include <stdexcept>
#include <vector>

using namespace cvc::nav;

namespace {

constexpr int kN = 65;
constexpr double kMin = -100.0, kMax = 100.0, kScale = 0.05;
// The material and terrain stacks sit on their own frames, bounds that are not floats, so the
// device's extent (taken in double, as the CPU takes it) is exercised.
constexpr double kMatMinX = -90.3, kMatMaxX = 110.7, kMatMinY = -104.1, kMatMaxY = 96.9;

template <class G> void frame(G &g, double mnx, double mny, double mxx, double mxy) {
  g.mnx = mnx;
  g.mny = mny;
  g.mxx = mxx;
  g.mxy = mxy;
  g.cx = 0.0;
  g.cy = 0.0;
  g.S = kScale;
}

double world_x(const double mn, const double mx, int c) { return mn + c * (mx - mn) / (kN - 1); }

// A circular building (centre (40, 10) m, radius 15 m): analytic phi and outward normal.
struct world {
  std::vector<float> field;    // [3][H][W]
  std::vector<float> mat, ter; // [1][6][H][W] each
  std::vector<float> slope;    // [2][H][W]
  std::vector<float> mu;       // [H][W]
  std::vector<float> lam_soft, lam_hard;
  field_stack fs;
  material_stack ms, ts;
  friction_field grip, sl;
  terrain_drive td;

  explicit world(int n) {
    const long HW = (long)kN * kN;
    field.assign(3 * HW, 0.0f);
    mat.assign(6 * HW, 0.0f);
    ter.assign(6 * HW, 0.0f);
    slope.assign(2 * HW, 0.0f);
    mu.assign(HW, 1.0f);
    for (int r = 0; r < kN; ++r)
      for (int c = 0; c < kN; ++c) {
        const long i = (long)r * kN + c;
        const double x = world_x(kMin, kMax, c), y = world_x(kMin, kMax, r);
        const double dx = x - 40.0, dy = y - 10.0, d = std::sqrt(dx * dx + dy * dy) + 1e-9;
        field[i] = (float)((d - 15.0) * kScale);
        field[HW + i] = (float)(dx / d);
        field[2 * HW + i] = (float)(dy / d);
        // Grip: a slick band across the middle.
        mu[i] = (float)(0.55 + 0.4 * std::tanh((std::fabs(y) - 20.0) / 15.0));
        // Slope (dz/dx, dz/dy): a hill, z = 6 exp(-((x+30)^2 + (y-30)^2) / 2500).
        const double hx = x + 30.0, hy = y - 30.0,
                     z = 6.0 * std::exp(-(hx * hx + hy * hy) / 2500.0);
        slope[i] = (float)(-2.0 * hx / 2500.0 * z);
        slope[HW + i] = (float)(-2.0 * hy / 2500.0 * z);
      }
    // The material and terrain planes on their own frame: risk r~ in [0, 1], phi in metres
    // around the barrier's reach (d_hat_m 12), grad r~ per normalized unit, grad phi per metre.
    for (int r = 0; r < kN; ++r)
      for (int c = 0; c < kN; ++c) {
        const long i = (long)r * kN + c;
        const double x = world_x(kMatMinX, kMatMaxX, c), y = world_x(kMatMinY, kMatMaxY, r);
        const double ax = x / 30.0, ay = y / 40.0;
        mat[i] = (float)(0.5 + 0.4 * std::sin(ax) * std::cos(ay));
        mat[HW + i] = (float)(10.0 + 7.0 * std::cos(x / 25.0 - y / 35.0));
        mat[2 * HW + i] = (float)(0.4 * std::cos(ax) * std::cos(ay) / 30.0 / kScale);
        mat[3 * HW + i] = (float)(-0.4 * std::sin(ax) * std::sin(ay) / 40.0 / kScale);
        mat[4 * HW + i] = (float)(-7.0 * std::sin(x / 25.0 - y / 35.0) / 25.0);
        mat[5 * HW + i] = (float)(7.0 * std::sin(x / 25.0 - y / 35.0) / 35.0);
        const double bx = (x + 20.0) / 45.0, by = (y - 15.0) / 45.0;
        const double g = std::exp(-(bx * bx + by * by));
        ter[i] = (float)(0.9 * g);
        ter[HW + i] = (float)(6.0 + 14.0 * (1.0 - g));
        ter[2 * HW + i] = (float)(-0.9 * g * 2.0 * bx / 45.0 / kScale);
        ter[3 * HW + i] = (float)(-0.9 * g * 2.0 * by / 45.0 / kScale);
        ter[4 * HW + i] = (float)(14.0 * g * 2.0 * bx / 45.0);
        ter[5 * HW + i] = (float)(14.0 * g * 2.0 * by / 45.0);
      }
    fs.data = field.data();
    fs.M = 1;
    fs.H = kN;
    fs.W = kN;
    frame(fs, kMin, kMin, kMax, kMax);
    ms.data = mat.data();
    ms.M = 1;
    ms.H = kN;
    ms.W = kN;
    frame(ms, kMatMinX, kMatMinY, kMatMaxX, kMatMaxY);
    ts = ms;
    ts.data = ter.data();
    grip.data = mu.data();
    grip.M = 1;
    grip.H = kN;
    grip.W = kN;
    frame(grip, kMin, kMin, kMax, kMax);
    sl = grip;
    sl.data = slope.data();
    sl.M = 2;
    td.stack = &ts;
    td.lam_soft = 0.5f;
    td.lam_hard = 1.0f;
    lam_soft.assign(n, 0.0f);
    lam_hard.assign(n, 0.0f);
    for (int i = 0; i < n; ++i) {
      lam_soft[i] = 0.4f + 0.02f * (float)i;
      lam_hard[i] = 0.8f + 0.03f * (float)i;
    }
  }

  material_drive material() const {
    material_drive md;
    md.stack = &ms;
    md.lam_soft = lam_soft.data();
    md.lam_hard = lam_hard.data();
    md.lam_soft_scale = 0.7f;
    md.lam_hard_scale = 1.3f;
    return md;
  }
};

veh_params base_veh() {
  veh_params v;
  v.rr = 0.15f;
  v.d_hat = 0.35f;
  v.dt = 0.06f;
  v.vmax = 0.9f;
  v.L = 0.035f;
  v.delta_max = 0.6f;
  v.a_max = 1.5f;
  v.a_lat_max = 1.0f;
  v.k_steer = 0.8f;
  v.nsub = 2;
  v.allow_reverse = true;
  return v;
}

// A small random 2-layer net (in -> 16 SiLU -> out) at the shipped bias basin.
coef_mlp make_net(int in, int out, std::uint32_t flags, unsigned seed) {
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  const int H = 16;
  std::vector<float> w0(H * in), b0(H), w1(out * H), b1(out);
  for (float &x : w0)
    x = U(-0.5f, 0.5f);
  for (float &x : b0)
    x = U(-0.2f, 0.2f);
  for (float &x : w1)
    x = U(-0.4f, 0.4f);
  for (float &x : b1)
    x = U(-0.2f, 0.2f);
  const bool sig = (flags & coef_mlp::kFlagLamSigmoid) != 0;
  std::vector<float> ob = {1.0f, 3.0f, 4.0f};
  if (!sig)
    for (int k = 3; k < out; ++k)
      ob.push_back(0.6f);
  return coef_mlp::from_layers(in, out, {H, out}, {in, H}, {1, 0}, {w0, w1}, {b0, b1}, ob, flags,
                               4.0f, 8.0f);
}

struct state {
  std::vector<float> o, th, sp, carrot;
};

state make_state(int n, unsigned seed) {
  std::mt19937 rng(seed);
  auto U = [&](float lo, float hi) {
    return lo + (hi - lo) * (float)std::uniform_real_distribution<double>(0.0, 1.0)(rng);
  };
  state s;
  while ((int)s.th.size() < n) {
    // Normalized positions over the hill, near the building and over the risk patches, but not
    // inside the building (centre (2, 0.5), radius 0.75): there the IPC barrier is clamped at
    // d = 1e-6, where its derivative is ~1e5, so an ulp of log() is a visible steer. The
    // float-equivalence contract is for drivable space.
    const float x = U(-3.0f, 3.0f), y = U(-2.5f, 2.5f), a = U(-3.14159f, 3.14159f);
    if (std::hypot(x - 2.0f, y - 0.5f) < 1.1f)
      continue;
    const float d = U(0.2f, 1.8f);
    s.o.push_back(x);
    s.o.push_back(y);
    s.th.push_back(U(-3.14159f, 3.14159f));
    s.sp.push_back(U(-0.1f, 0.8f));
    s.carrot.push_back(x + d * std::cos(a));
    s.carrot.push_back(y + d * std::sin(a));
  }
  return s;
}

enum class path { plain, material };

// The worst telemetry difference, relative where a value can be large (a barrier's potential and
// force near a wall), and whether every agent's barrier flag agrees.
struct tel_diff {
  double worst = 0.0;
  bool binding_agrees = true;
  void add(float a, float b) {
    const double d = std::fabs((double)a - b) / std::max(1.0, std::fabs((double)a));
    worst = std::max(worst, d);
  }
  void add(const drive_telemetry &a, const drive_telemetry &b) {
    for (auto m : {&drive_telemetry::alpha, &drive_telemetry::beta, &drive_telemetry::gamma,
                   &drive_telemetry::lam_soft, &drive_telemetry::lam_hard, &drive_telemetry::mu,
                   &drive_telemetry::mrisk, &drive_telemetry::v_risk, &drive_telemetry::ext_fx,
                   &drive_telemetry::ext_fy, &drive_telemetry::steer, &drive_telemetry::curvature,
                   &drive_telemetry::clearance, &drive_telemetry::potential,
                   &drive_telemetry::force_x, &drive_telemetry::force_y})
      add(a.*m, b.*m);
    for (auto m : {&potential_terms::goal, &potential_terms::barrier, &potential_terms::soft,
                   &potential_terms::hard, &potential_terms::terrain})
      add(a.terms.*m, b.terms.*m);
    binding_agrees = binding_agrees && a.binding == b.binding;
  }
};

// `ticks` ticks of the CPU drive; before each, the GPU drive steps from the same state and the
// two are compared. Returns the worst absolute difference over the state; `td` (optional)
// collects the telemetry's.
double run_parity(const world &w, const veh_params &v, const coef_mlp &net, path p, int n,
                  int ticks, unsigned seed, tel_diff *td = nullptr) {
  state s = make_state(n, seed);
  const material_drive md = w.material();
  double worst = 0.0;
  for (int t = 0; t < ticks; ++t) {
    std::vector<float> o2 = s.o, th2 = s.th, sp2 = s.sp, mc(n), mc2(n);
    std::vector<drive_telemetry> tc(n), tg(n);
    drive_telemetry *tcp = td ? tc.data() : nullptr, *tgp = td ? tg.data() : nullptr;
    if (p == path::plain) {
      drive_step(w.fs, s.o.data(), s.th.data(), s.sp.data(), s.carrot.data(), net, n, nullptr, v,
                 mc.data(), 1, nullptr, tcp);
      drive_step_cuda(w.fs, o2.data(), th2.data(), sp2.data(), s.carrot.data(), net, n, v,
                      mc2.data(), tgp);
    } else {
      drive_step_material(w.fs, s.o.data(), s.th.data(), s.sp.data(), s.carrot.data(), net, n,
                          nullptr, v, md, mc.data(), 1, tcp);
      drive_step_material_cuda(w.fs, o2.data(), th2.data(), sp2.data(), s.carrot.data(), net, n, v,
                               md, mc2.data(), tgp);
    }
    if (td)
      for (int i = 0; i < n; ++i)
        td->add(tc[i], tg[i]);
    for (int i = 0; i < n; ++i) {
      worst = std::max(worst, (double)std::fabs(s.o[2 * i] - o2[2 * i]));
      worst = std::max(worst, (double)std::fabs(s.o[2 * i + 1] - o2[2 * i + 1]));
      worst = std::max(worst, (double)std::fabs(s.th[i] - th2[i]));
      worst = std::max(worst, (double)std::fabs(s.sp[i] - sp2[i]));
      worst = std::max(worst, (double)std::fabs(mc[i] - mc2[i]));
    }
  }
  return worst;
}

// How far the CPU drive with `v` / `p` moves from the plain geometry drive in one tick, so a
// parity case cannot pass by exercising nothing.
double effect(const world &w, const veh_params &v, const coef_mlp &net, path p, int n,
              unsigned seed) {
  state a = make_state(n, seed), b = a;
  std::vector<float> mc(n);
  const material_drive md = w.material();
  veh_params flat = base_veh();
  flat.grip = v.grip;
  if (p == path::plain)
    drive_step(w.fs, a.o.data(), a.th.data(), a.sp.data(), a.carrot.data(), net, n, nullptr, v,
               mc.data(), 1);
  else
    drive_step_material(w.fs, a.o.data(), a.th.data(), a.sp.data(), a.carrot.data(), net, n,
                        nullptr, v, md, mc.data(), 1);
  // The same net through the geometry drive, its terrain columns read off the same terrain
  // but no force, governor or material applied.
  veh_params feats_only = flat;
  terrain_drive inert = w.td;
  inert.lam_soft = 0.0f;
  inert.lam_hard = 0.0f;
  if (v.terrain)
    feats_only.terrain = &inert;
  drive_step(w.fs, b.o.data(), b.th.data(), b.sp.data(), b.carrot.data(), net, n, nullptr,
             feats_only, mc.data(), 1);
  double d = 0.0;
  for (int i = 0; i < n; ++i)
    d = std::max(d, (double)std::fabs(a.sp[i] - b.sp[i]) + std::fabs(a.th[i] - b.th[i]));
  return d;
}

constexpr int kAgents = 96, kTicks = 12;
constexpr double kTol = 1e-5;

} // namespace

TEST(NavDriveCudaParity, TerrainRiskNetMatchesDriveStep) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // A risk net on the plain drive reads the terrain's grade cost (terrain risk is material
  // risk); the terrain's force and the slope governors apply as drive_step applies them.
  const world w(kAgents);
  veh_params v = base_veh();
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  const coef_mlp net = make_net(6, 3, coef_mlp::kFlagFeatRisk, 11);
  ASSERT_TRUE(net.has_risk());
  const double worst = run_parity(w, v, net, path::plain, kAgents, kTicks, 3);
  std::printf("[drive-cuda terrain-risk] worst=%.3e effect=%.3e\n", worst,
              effect(w, v, net, path::plain, kAgents, 3));
  EXPECT_LT(worst, kTol);
  EXPECT_GT(effect(w, v, net, path::plain, kAgents, 3), 1e-3);
}

TEST(NavDriveCudaParity, SlopeNetMatchesDriveStep) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  const world w(kAgents);
  veh_params v = base_veh();
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  v.grip = &w.grip;
  // grip, then slope: coef_feats' column order.
  const coef_mlp net = make_net(7, 3, coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatSlope, 12);
  ASSERT_TRUE(net.has_slope() && net.has_mu());
  const double worst = run_parity(w, v, net, path::plain, kAgents, kTicks, 4);
  std::printf("[drive-cuda slope] worst=%.3e\n", worst);
  EXPECT_LT(worst, kTol);
}

TEST(NavDriveCudaParity, MaterialFixedLamMatchesDriveStepMaterial) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // A geometry net on the material drive: the fixed lam columns weight the material force.
  const world w(kAgents);
  const veh_params v = base_veh();
  const coef_mlp net = make_net(5, 3, 0, 13);
  const double worst = run_parity(w, v, net, path::material, kAgents, kTicks, 5);
  std::printf("[drive-cuda material] worst=%.3e effect=%.3e\n", worst,
              effect(w, v, net, path::material, kAgents, 5));
  EXPECT_LT(worst, kTol);
  EXPECT_GT(effect(w, v, net, path::material, kAgents, 5), 1e-3);
}

TEST(NavDriveCudaParity, RiskLamHeadNetMatchesDriveStepMaterial) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // A v1 lam-head net (col 3 = softplus lam_soft, times lam_soft_scale; lam_hard the fixed
  // column) whose risk column reads the worse of the material and the terrain.
  const world w(kAgents);
  veh_params v = base_veh();
  v.terrain = &w.td;
  const coef_mlp net = make_net(6, 4, coef_mlp::kFlagFeatRisk, 14);
  ASSERT_TRUE(net.has_lam() && !net.has_lam_hard() && !net.lam_sigmoid());
  const double worst = run_parity(w, v, net, path::material, kAgents, kTicks, 6);
  std::printf("[drive-cuda risk+lam] worst=%.3e\n", worst);
  EXPECT_LT(worst, kTol);
}

TEST(NavDriveCudaParity, TwoHeadSigmoidNetWithEverythingMatchesDriveStepMaterial) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // Everything at once: grip, risk and slope columns, a two-head sigmoid lam net (lam_max *
  // sigmoid, times the authority scales), the material stack, the terrain and the governors,
  // with a three-disc footprint.
  const world w(kAgents);
  veh_params v = base_veh();
  const float off[3] = {0.0f, 0.0175f, 0.035f};
  v.body_offsets = off;
  v.n_body = 3;
  v.body_rr = 0.012f;
  v.body_gain = 1.0f / 3.0f;
  v.grip = &w.grip;
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  const coef_mlp net = make_net(8, 5,
                                coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk |
                                    coef_mlp::kFlagFeatSlope | coef_mlp::kFlagLamSigmoid,
                                15);
  ASSERT_TRUE(net.lam_sigmoid() && net.has_lam_hard());
  const double worst = run_parity(w, v, net, path::material, kAgents, kTicks, 7);
  std::printf("[drive-cuda everything] worst=%.3e effect=%.3e\n", worst,
              effect(w, v, net, path::material, kAgents, 7));
  EXPECT_LT(worst, kTol);
  EXPECT_GT(effect(w, v, net, path::material, kAgents, 7), 1e-3);
}

TEST(NavDriveCudaParity, TelemetryMatchesTheCpuDrive) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // drive_telemetry, field by field, U_nav's terms included: on the material drive with the
  // terrain and the two-head net (mrisk the worse of material and terrain risk, the learned
  // lams), and on the plain drive with the terrain alone (mrisk and the lams the terrain's).
  const world w(kAgents);
  veh_params v = base_veh();
  const float off[3] = {0.0f, 0.0175f, 0.035f};
  v.body_offsets = off;
  v.n_body = 3;
  v.body_rr = 0.012f;
  v.body_gain = 1.0f / 3.0f;
  v.grip = &w.grip;
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  const coef_mlp two_head = make_net(8, 5,
                                     coef_mlp::kFlagFeatMu | coef_mlp::kFlagFeatRisk |
                                         coef_mlp::kFlagFeatSlope | coef_mlp::kFlagLamSigmoid,
                                     17);
  tel_diff mat_td;
  EXPECT_LT(run_parity(w, v, two_head, path::material, kAgents, kTicks, 10, &mat_td), kTol);
  veh_params v1 = base_veh();
  v1.terrain = &w.td;
  tel_diff plain_td;
  EXPECT_LT(run_parity(w, v1, make_net(6, 3, coef_mlp::kFlagFeatRisk, 18), path::plain, kAgents,
                       kTicks, 11, &plain_td),
            kTol);
  std::printf("[drive-cuda telemetry] material=%.3e plain=%.3e\n", mat_td.worst, plain_td.worst);
  EXPECT_LT(mat_td.worst, kTol);
  EXPECT_LT(plain_td.worst, kTol);
  EXPECT_TRUE(mat_td.binding_agrees);
  EXPECT_TRUE(plain_td.binding_agrees);
}

TEST(NavDriveCudaParity, RiskGovernorMatchesTheCpuDrive) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // veh_params::risk_governor, its cap in the telemetry too: on the material drive over the
  // material and the terrain, with the slope governors so the look-ahead brakes with gravity,
  // and on the plain drive over the terrain alone, its weights zero (the governor reads its risk
  // regardless).
  const world w(kAgents);
  veh_params v = base_veh();
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  v.risk_governor = true;
  v.risk_vmin = 0.1f;
  v.risk_onset = 0.3f;
  v.risk_lookahead = 0.6f;
  v.risk_probes = 3;
  const coef_mlp net = make_net(5, 3, 0, 19);
  tel_diff mat_td;
  const double mat_worst = run_parity(w, v, net, path::material, kAgents, kTicks, 12, &mat_td);
  terrain_drive inert = w.td;
  inert.lam_soft = 0.0f;
  inert.lam_hard = 0.0f;
  veh_params v1 = v;
  v1.terrain = &inert;
  v1.slope = nullptr;
  tel_diff plain_td;
  const double plain_worst = run_parity(w, v1, net, path::plain, kAgents, kTicks, 13, &plain_td);
  // and the governor is doing something: the same drives without it end faster
  veh_params off = v, off1 = v1;
  off.risk_governor = false;
  off1.risk_governor = false;
  const auto top = [&](const veh_params &vp, path p, unsigned seed) {
    state s = make_state(kAgents, seed);
    const material_drive md = w.material();
    std::vector<float> mc(kAgents);
    for (int t = 0; t < kTicks; ++t)
      if (p == path::plain)
        drive_step(w.fs, s.o.data(), s.th.data(), s.sp.data(), s.carrot.data(), net, kAgents,
                   nullptr, vp, mc.data(), 1);
      else
        drive_step_material(w.fs, s.o.data(), s.th.data(), s.sp.data(), s.carrot.data(), net,
                            kAgents, nullptr, vp, md, mc.data(), 1);
    double sum = 0.0;
    for (float x : s.sp)
      sum += std::fabs(x);
    return sum;
  };
  std::printf("[drive-cuda risk governor] material=%.3e plain=%.3e tel=%.3e/%.3e\n", mat_worst,
              plain_worst, mat_td.worst, plain_td.worst);
  EXPECT_LT(mat_worst, kTol);
  EXPECT_LT(plain_worst, kTol);
  EXPECT_LT(mat_td.worst, kTol);
  EXPECT_LT(plain_td.worst, kTol);
  EXPECT_LT(top(v, path::material, 12), top(off, path::material, 12) - 1e-3);
  EXPECT_LT(top(v1, path::plain, 13), top(off1, path::plain, 13) - 1e-3);
}

TEST(NavDriveCudaParity, LamNetWithoutMaterialMatchesDriveStep) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // drive_step runs a lam-head net with no material: the heads have nothing to weight, so only
  // alpha/beta/gamma drive. drive_step_cuda does the same.
  const world w(kAgents);
  veh_params v = base_veh();
  v.grip = &w.grip;
  const coef_mlp net = make_net(6, 5, coef_mlp::kFlagFeatMu | coef_mlp::kFlagLamSigmoid, 16);
  const double worst = run_parity(w, v, net, path::plain, kAgents, kTicks, 8);
  EXPECT_LT(worst, kTol);
}

TEST(NavDriveCudaParity, RolloutHonorsTerrainAndSlope) {
  if (!drive_cuda_available())
    GTEST_SKIP() << "no CUDA device";
  // The unfused bicycle_rollout_cuda applies veh_params::terrain and slope as bicycle_rollout
  // does.
  const world w(kAgents);
  veh_params v = base_veh();
  v.terrain = &w.td;
  v.slope = &w.sl;
  v.gravity = (float)(9.80665 * kScale);
  state s = make_state(kAgents, 9);
  std::vector<float> al(kAgents, 1.0f), be(kAgents, 3.0f), ga(kAgents, 4.0f);
  std::vector<float> o2 = s.o, th2 = s.th, sp2 = s.sp, mc(kAgents), mc2(kAgents);
  bicycle_rollout(w.fs, s.o.data(), s.th.data(), s.sp.data(), s.carrot.data(), al.data(), be.data(),
                  ga.data(), kAgents, nullptr, v, mc.data(), 1);
  bicycle_rollout_cuda(w.fs, o2.data(), th2.data(), sp2.data(), s.carrot.data(), al.data(),
                       be.data(), ga.data(), kAgents, v, mc2.data());
  for (int i = 0; i < kAgents; ++i) {
    EXPECT_NEAR(s.o[2 * i], o2[2 * i], kTol);
    EXPECT_NEAR(s.o[2 * i + 1], o2[2 * i + 1], kTol);
    EXPECT_NEAR(s.th[i], th2[i], kTol);
    EXPECT_NEAR(s.sp[i], sp2[i], kTol);
  }
}

// What is still refused is refused host-side, before any device call, so these pass on a
// device-less CUDA build too.
TEST(NavDriveCudaParity, RefusesWhatItCannotApply) {
  const world w(2);
  std::vector<float> o(4, 0.0f), th(2, 0.0f), sp(2, 0.3f), carrot(4, 1.0f), mc(2);
  auto plain = [&](const coef_mlp &net, const veh_params &v) {
    drive_step_cuda(w.fs, o.data(), th.data(), sp.data(), carrot.data(), net, 2, v, mc.data());
  };
  // A risk net with neither a material stack nor a terrain has nothing to read.
  EXPECT_THROW(plain(make_net(6, 3, coef_mlp::kFlagFeatRisk, 1), base_veh()), std::runtime_error);
  // A slope net needs the terrain.
  EXPECT_THROW(plain(make_net(6, 3, coef_mlp::kFlagFeatSlope, 2), base_veh()), std::runtime_error);
  // The plant arm is not ported.
  plant_drive pd;
  veh_params vp = base_veh();
  vp.plant = &pd;
  EXPECT_THROW(plain(make_net(5, 3, 0, 3), vp), std::runtime_error);
  // A material drive with no fixed lam_hard column and a net with no head for it.
  material_drive md = w.material();
  md.lam_hard = nullptr;
  EXPECT_THROW(drive_step_material_cuda(w.fs, o.data(), th.data(), sp.data(), carrot.data(),
                                        make_net(5, 4, 0, 4), 2, base_veh(), md, mc.data()),
               std::runtime_error);
}

#endif // CVC_ENABLE_CUDA
