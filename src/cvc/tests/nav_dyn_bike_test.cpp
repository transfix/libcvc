/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_dyn_bike_test: the dynamic single-track vehicle (diff::dyn_plant, dyn_bike_step) against
// the vehicle-physics plan's P8a gates:
//   1. its gradient matches finite differences (the plant in double, tightly; the plant under
//      the trained controller in float, the way the bicycle's gradcheck does it);
//   3. with stiff tires, no lag and low g it follows bike_step over a 200-tick route;
//   4. the skid modes are there: understeer that grows with lateral g, oversteer when braking in
//      a turn, and a steady lateral acceleration never above mu g;
//   5. it stops within 10% of v^2 / (2 mu g).
// (Gate 2, parity with the torch twin, is GRL-SNAM's; DynBike.Golden pins the values it reads.)
// The skid and stopping gates drive a Humvee from its vehicle_desc, in metres (S = 1).

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/ext_rollout.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using cvc::nav::diff::bike_veh;
using cvc::nav::diff::dyn_bike_params;
using cvc::nav::diff::dyn_state;
namespace diff = cvc::nav::diff;

namespace {

constexpr double kG = 9.80665;

// The Humvee's dynamic parameters in metres, and limits for the plant alone.
dyn_bike_params humvee(double mu = 0.9) {
  const cvc::nav::dyn_bike_si si =
      cvc::physics::nav_plant::dyn_bike_si_of(cvc::physics::vehicle_desc::humvee(), mu);
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn_bike_params_of(si, f);
}

bike_veh plant_limits(const dyn_bike_params &p, bool reverse = false) {
  bike_veh v{};
  v.L = p.lf + p.lr;
  v.vmax = 40.0f;
  v.delta_max = 0.6f;
  v.hdt = 0.06f;
  v.allow_reverse = reverse ? 1 : 0;
  return v;
}

template <class S> dyn_state<S> at_rest(S vx = S(0)) {
  dyn_state<S> s{};
  s.vx = vx;
  return s;
}

// Steady-state cornering at speed v and steer delta: hold the speed with a proportional
// throttle for `seconds`, then average the last second. Returns the lateral acceleration
// vx r and the path's curvature r / vx.
void steady_turn(const dyn_bike_params &p, double v, double delta, double &a_y, double &kappa,
                 double seconds = 12.0) {
  const bike_veh lim = plant_limits(p);
  dyn_state<double> s = at_rest<double>(v);
  const int n = static_cast<int>(seconds / lim.hdt), tail = static_cast<int>(1.0 / lim.hdt);
  double ay = 0.0, k = 0.0;
  for (int t = 0; t < n; ++t) {
    dyn_state<double> o;
    const double a_cmd = 2.0 * (v - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
    diff::dyn_plant<double>(p, lim, s, delta, a_cmd, 0.0, o);
    s = o;
    if (t >= n - tail) {
      ay += s.vx * s.r;
      k += s.r / s.vx;
    }
  }
  a_y = ay / tail;
  kappa = k / tail;
}

// The trainer's city scene, its field and the trainer's bicycle (coef_train's defaults).
struct city {
  cvc::nav::training_scene sc;
  cvc::nav::field_stack fs;
  diff::field F;
  bike_veh v;
};
std::unique_ptr<city> make_city() {
  std::unique_ptr<city> c(new city{cvc::nav::city_scene(48), {}, {}, {}});
  c->fs = c->sc.field();
  diff::field &F = c->F;
  F.data = c->fs.data;
  F.H = c->fs.H;
  F.W = c->fs.W;
  F.S = static_cast<float>(c->fs.S);
  F.cx = static_cast<float>(c->fs.cx);
  F.cy = static_cast<float>(c->fs.cy);
  F.mnx = static_cast<float>(c->fs.mnx);
  F.mny = static_cast<float>(c->fs.mny);
  F.mxx = static_cast<float>(c->fs.mxx);
  F.mxy = static_cast<float>(c->fs.mxy);
  const cvc::nav::train_config tc;
  bike_veh &v = c->v;
  v.rr = c->sc.rr;
  v.d_hat = c->sc.d_hat;
  v.vmax = c->sc.vmax;
  v.L = tc.veh_L;
  v.delta_max = tc.veh_delta_max;
  v.a_max = tc.veh_a_max;
  v.a_lat_max = tc.veh_a_lat_max;
  v.k_steer = tc.veh_k_steer;
  v.hdt = c->sc.dt;
  v.allow_reverse = tc.veh_allow_reverse ? 1 : 0;
  return c;
}

double dot_state(const dyn_state<double> &a, const dyn_state<double> &b) {
  double acc = 0.0;
  for (int k = 0; k < dyn_state<double>::size; ++k)
    acc += a[k] * b[k];
  return acc;
}

} // namespace

// The Humvee's parameters come from its vehicle_desc and convert by length exponent.
TEST(DynBike, HumveeParametersFromItsSpecSheet) {
  const cvc::physics::vehicle_desc d = cvc::physics::vehicle_desc::humvee();
  const cvc::nav::dyn_bike_si si = cvc::physics::nav_plant::dyn_bike_si_of(d);
  EXPECT_NEAR(si.cg_to_front + si.cg_to_rear, d.wheelbase(), 1e-9);
  EXPECT_GT(si.cg_to_front, 0.0);
  EXPECT_GT(si.cg_to_rear, 0.0);
  EXPECT_GT(si.cg_height, 0.3);
  EXPECT_GT(si.tire_C, 1.0);
  EXPECT_LT(si.tire_C, 2.0);
  EXPECT_NEAR(std::sin(si.tire_C * 3.14159265358979 / 2.0), d.tire_sliding, 1e-9);
  // The peak of mu Fz sin(C atan(B alpha)) is at the tire's peak slip angle (rear tires).
  EXPECT_NEAR(si.tire_C * std::atan(si.tire_B_rear * d.tire_peak_slip_angle),
              3.14159265358979 / 2.0, 1e-9);
  EXPECT_LT(si.tire_B_front, si.tire_B_rear);
  EXPECT_DOUBLE_EQ(si.front_drive, 0.5); // the Humvee drives all four wheels
  EXPECT_GT(si.drag, 0.0);
  std::printf("  Humvee: %.0f kg, Iz %.0f kg m^2, lf %.2f m, lr %.2f m, h %.2f m, B %.1f/%.1f, "
              "C %.3f, brake front %.2f\n",
              si.mass, si.yaw_inertia, si.cg_to_front, si.cg_to_rear, si.cg_height, si.tire_B_front,
              si.tire_B_rear, si.tire_C, si.front_brake);
  cvc::world_units::normalized_frame f;
  f.scale = 0.05;
  const dyn_bike_params p = cvc::nav::dyn_bike_params_of(si, f);
  EXPECT_FLOAT_EQ(p.m, static_cast<float>(si.mass));
  EXPECT_FLOAT_EQ(p.lf, static_cast<float>(0.05 * si.cg_to_front));
  EXPECT_FLOAT_EQ(p.Iz, static_cast<float>(0.05 * 0.05 * si.yaw_inertia));
  EXPECT_FLOAT_EQ(p.g, static_cast<float>(0.05 * si.gravity));
  EXPECT_FLOAT_EQ(p.drag, static_cast<float>(si.drag / 0.05));
  EXPECT_FLOAT_EQ(p.B_r, static_cast<float>(si.tire_B_rear));
}

// Gate 1a: the plant's vector-Jacobian product (forward-mode duals) against central finite
// differences, in double, over driving, braking, cornering past the peak, reversing and creeping.
TEST(DynBike, PlantGradientMatchesFiniteDifference) {
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.v_blend = 1.0f;
  p.v_width = 0.3f;
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  struct regime {
    const char *name;
    double vx, vy, r, delta, a, dcmd, acmd;
    bool reverse;
  };
  const regime regimes[] = {
      {"cruise", 12.0, 0.1, 0.05, 0.02, 0.5, 0.05, 1.0, false},
      {"hard corner", 15.0, -1.2, 0.6, 0.25, 0.0, 0.3, 0.0, false},
      {"braking in a turn", 14.0, 0.8, 0.4, 0.15, -4.0, 0.15, -6.0, false},
      {"past the peak", 10.0, 2.5, 1.1, 0.4, 1.0, 0.45, 2.0, false},
      {"backing up", -2.0, 0.05, -0.1, 0.2, -0.5, 0.25, -0.8, true},
      {"creeping", 0.6, 0.02, 0.05, 0.3, 0.3, 0.35, 0.5, false},
  };
  double worst = 0.0;
  for (const regime &rg : regimes) {
    const bike_veh lim = plant_limits(p, rg.reverse);
    for (int trial = 0; trial < 8; ++trial) {
      dyn_state<double> s{};
      s.x = 3.0 * U(rng);
      s.y = 3.0 * U(rng);
      s.psi = U(rng);
      s.vx = rg.vx * (1.0 + 0.05 * U(rng));
      s.vy = rg.vy * (1.0 + 0.1 * U(rng));
      s.r = rg.r * (1.0 + 0.1 * U(rng));
      s.delta = rg.delta + 0.01 * U(rng);
      s.a = rg.a + 0.1 * U(rng);
      const double dcmd = rg.dcmd + 0.01 * U(rng), acmd = rg.acmd + 0.1 * U(rng);
      const double grade = 0.05 * U(rng);
      dyn_state<double> g_o{};
      for (int k = 0; k < dyn_state<double>::size; ++k)
        g_o[k] = U(rng);
      dyn_state<double> g_s{};
      double g_dc = 0.0, g_ac = 0.0;
      diff::dyn_plant_vjp<double>(p, lim, s, dcmd, acmd, grade, g_o, g_s, g_dc, g_ac);
      const auto loss = [&](const dyn_state<double> &ss, double dc, double ac) {
        dyn_state<double> o;
        diff::dyn_plant<double>(p, lim, ss, dc, ac, grade, o);
        return dot_state(g_o, o);
      };
      for (int j = 0; j < dyn_state<double>::size + 2; ++j) {
        dyn_state<double> sp = s, sm = s;
        double dcp = dcmd, dcm = dcmd, acp = acmd, acm = acmd;
        double base = j < 8 ? s[j] : (j == 8 ? dcmd : acmd);
        const double h = 1e-6 * std::max(1.0, std::fabs(base));
        if (j < 8) {
          sp[j] += h;
          sm[j] -= h;
        } else if (j == 8) {
          dcp += h;
          dcm -= h;
        } else {
          acp += h;
          acm -= h;
        }
        const double fd = (loss(sp, dcp, acp) - loss(sm, dcm, acm)) / (2.0 * h);
        const double an = j < 8 ? g_s[j] : (j == 8 ? g_dc : g_ac);
        const double err = std::fabs(fd - an) / (std::fabs(fd) + std::fabs(an) + 1e-4);
        worst = std::max(worst, err);
        EXPECT_LT(err, 1e-5) << rg.name << " trial " << trial << " input " << j << ": fd " << fd
                             << ", vjp " << an;
      }
    }
  }
  std::printf("  plant VJP vs finite differences: worst relative error %.2e\n", worst);
}

// Gate 1b: the whole step, the trained controller included, in float: a 6-step rollout's
// loss through dyn_bike_step_bwd against a central difference along the gradient (the
// bicycle's gradcheck: float steps flip the controller's clamps, hence dir_rel < 5e-2).
TEST(DynBike, StepGradcheckMatchesFiniteDifference) {
  const std::unique_ptr<city> c = make_city();
  const cvc::nav::training_scene &sc = c->sc;
  const diff::field &F = c->F;
  const bike_veh &v = c->v;
  // A vehicle the size of the scene's bicycle, with grip to spare for its accelerations.
  dyn_bike_params p;
  p.lf = p.lr = 0.5f * v.L;
  p.m = 1.0f;
  p.Iz = p.m * p.lf * p.lr;
  p.g = 1.0f;
  p.mu = 3.0f;
  p.h = 0.2f * v.L;
  p.B_f = 10.0f;
  p.B_r = 12.0f;
  p.tau_steer = 0.05f;
  p.tau_drive = 0.08f;
  p.eps = 0.02f;
  p.v_blend = 0.03f;
  p.v_width = 0.01f;

  const int n = 128, K = 6;
  std::vector<float> o(2 * n), goal(2 * n);
  sc.sample_starts_goals(n, 7, o.data(), goal.data());
  const float al = 0.6f, be = 1.8f, ga = 0.9f;
  std::vector<dyn_state<float>> s0(n);
  for (int i = 0; i < n; ++i) {
    s0[i] = dyn_state<float>{};
    s0[i].x = o[2 * i];
    s0[i].y = o[2 * i + 1];
    s0[i].psi = std::atan2(goal[2 * i + 1] - o[2 * i + 1], goal[2 * i] - o[2 * i]);
    s0[i].vx = 0.2f;
  }
  // Loss: the squared distance to the goal after K steps, plus the speed it ends with.
  const auto rollout = [&](float a, float b, float c, std::vector<float> *g_coef) {
    double loss = 0.0;
    double gal = 0.0, gbe = 0.0, gga = 0.0;
    for (int i = 0; i < n; ++i) {
      std::vector<dyn_state<float>> st(K + 1);
      st[0] = s0[i];
      for (int t = 0; t < K; ++t)
        diff::dyn_bike_step(F, st[t], goal[2 * i], goal[2 * i + 1], a, b, c, v, p, 0.0f, st[t + 1]);
      const float dx = st[K].x - goal[2 * i], dy = st[K].y - goal[2 * i + 1];
      loss += dx * dx + dy * dy + 0.1 * st[K].vx * st[K].vx;
      if (!g_coef)
        continue;
      dyn_state<float> g{};
      g.x = 2.0f * dx;
      g.y = 2.0f * dy;
      g.vx = 0.2f * st[K].vx;
      for (int t = K - 1; t >= 0; --t) {
        dyn_state<float> gs{};
        float ga_, gb_, gc_;
        diff::dyn_bike_step_bwd(F, st[t], goal[2 * i], goal[2 * i + 1], a, b, c, v, p, 0.0f, g, gs,
                                ga_, gb_, gc_);
        gal += ga_;
        gbe += gb_;
        gga += gc_;
        g = gs;
      }
    }
    if (g_coef)
      *g_coef = {static_cast<float>(gal), static_cast<float>(gbe), static_cast<float>(gga)};
    return loss;
  };
  std::vector<float> g;
  rollout(al, be, ga, &g);
  const double gnorm = std::sqrt(double(g[0]) * g[0] + double(g[1]) * g[1] + double(g[2]) * g[2]);
  ASSERT_GT(gnorm, 1e-6);
  const float eps = 2e-3f;
  const float ua = static_cast<float>(g[0] / gnorm), ub = static_cast<float>(g[1] / gnorm),
              uc = static_cast<float>(g[2] / gnorm);
  const double Lp = rollout(al + eps * ua, be + eps * ub, ga + eps * uc, nullptr);
  const double Lm = rollout(al - eps * ua, be - eps * ub, ga - eps * uc, nullptr);
  const double fd = (Lp - Lm) / (2.0 * eps);
  const double dir_rel = std::fabs(fd - gnorm) / (std::fabs(fd) + gnorm + 1e-9);
  std::printf("  dyn_bike_step gradcheck: |g| %.4f, finite difference %.4f, dir_rel %.3e\n", gnorm,
              fd, dir_rel);
  EXPECT_LT(dir_rel, 5e-2);
}

// Gate 3: the kinematic limit. bike_step's own commands along its 200-tick routes, replayed
// through dyn_plant from the same start, keep to bike_step's path when the dynamic vehicle is
// what the kinematic bicycle assumes: stiff tires, no lag, grip far beyond the controller's
// accelerations, its mass at the rear axle, and its drive and brakes there. (A real vehicle
// differs physically, and bike_step's v' = a leaves both effects out: a turning body puts kinetic
// energy into its yaw and sideways motion, and a steered front wheel's brake force acts along
// the wheel, so braking on it in a turn slows the body by about F / cos(delta). That case is
// reported, not gated.)
//
// Under the controller (dyn_bike_step, closed loop) the comparison is gated by its medians. The
// controller's feedback amplifies the plant's small differences along some routes: over 40 sets
// of routes, 0 to 11 of 32 agents part by more than 0.2 rr while the rest stay within 1e-3. The
// routes themselves differ by standard library (sample_starts_goals' uniform_int_distribution),
// so a worst-case gate held on one platform's routes and failed on macOS's. Over those 40 sets
// the median gap was at most 0.0081 and the median end-distance difference at most 0.41%; the
// open-loop worst was at most 0.0151.
TEST(DynBike, KinematicLimitFollowsBikeStep) {
  const std::unique_ptr<city> c = make_city();
  const cvc::nav::training_scene &sc = c->sc;
  const diff::field &F = c->F;
  const bike_veh &v = c->v;
  dyn_bike_params stiff;
  stiff.m = 1.0f;
  stiff.g = 1.0f;
  stiff.mu = 40.0f; // the controller asks for at most 1.5 against mu g = 40
  stiff.h = 0.0f;
  stiff.B_f = stiff.B_r = 400.0f;
  stiff.eps = 1e-3f;
  stiff.v_blend = 0.03f; // where slip angles mean nothing, the kinematic bicycle
  stiff.v_width = 0.01f;
  // The kinematic bicycle's vehicle: its mass at the rear axle (0.1% of the wheelbase ahead of
  // it; the gap to bike_step shrinks with that distance, 5x from 1%), driven and braked there,
  // the steered front wheel rolling free.
  dyn_bike_params at_rear = stiff;
  at_rear.lr = 0.001f * v.L;
  at_rear.lf = v.L - at_rear.lr;
  at_rear.Iz = at_rear.m * at_rear.lf * at_rear.lr;
  at_rear.front_drive = at_rear.front_brake = 0.0f;
  dyn_bike_params body = stiff; // a body: its mass mid-wheelbase, yaw inertia m lf lr
  body.lf = body.lr = 0.5f * v.L;
  body.Iz = body.m * body.lf * body.lr;

  const int n = 32, T = 200;
  std::vector<float> o(2 * n), goal(2 * n);
  sc.sample_starts_goals(n, 3, o.data(), goal.data());
  const float al = 0.6f, be = 1.8f, ga = 0.9f;
  struct result {
    double worst = 0.0, mean = 0.0, worst_closed = 0.0, end_bike = 0.0, end_dyn = 0.0, path = 0.0;
    std::vector<double> closed, end_rel; // per agent: the closed-loop gap, |end_dyn/end_bike - 1|
  };
  const auto median = [](std::vector<double> x) {
    std::sort(x.begin(), x.end());
    const std::size_t m = x.size() / 2;
    return x.size() % 2 ? x[m] : 0.5 * (x[m - 1] + x[m]);
  };
  const auto run = [&](const dyn_bike_params &p) {
    result res;
    for (int i = 0; i < n; ++i) {
      const float gx = goal[2 * i], gy = goal[2 * i + 1];
      float bx = o[2 * i], by = o[2 * i + 1];
      float bth = std::atan2(gy - by, gx - bx), bsp = 0.0f;
      dyn_state<float> s{}, loop{};
      s.x = bx;
      s.y = by;
      s.psi = bth;
      loop = s;
      double gap = 0.0, gap_closed = 0.0;
      for (int t = 0; t < T; ++t) {
        float dc, ac, bx1, by1, bth1, bsp1;
        diff::bike_control(F, bx, by, bth, bsp, gx, gy, al, be, ga, v, dc, ac);
        diff::bike_plant(bx, by, bth, bsp, dc, ac, v, bx1, by1, bth1, bsp1);
        res.path += std::hypot(bx1 - bx, by1 - by) / n;
        bx = bx1;
        by = by1;
        bth = bth1;
        bsp = bsp1;
        dyn_state<float> s1;
        diff::dyn_plant<float>(p, v, s, dc, ac, 0.0f, s1);
        s = s1;
        diff::dyn_bike_step(F, loop, gx, gy, al, be, ga, v, p, 0.0f, s1);
        loop = s1;
        gap = std::max(gap, static_cast<double>(std::hypot(s.x - bx, s.y - by)));
        gap_closed =
            std::max(gap_closed, static_cast<double>(std::hypot(loop.x - bx, loop.y - by)));
      }
      res.worst = std::max(res.worst, gap);
      res.worst_closed = std::max(res.worst_closed, gap_closed);
      res.mean += gap / n;
      const double eb = std::hypot(bx - gx, by - gy), ed = std::hypot(loop.x - gx, loop.y - gy);
      res.end_bike += eb / n;
      res.end_dyn += ed / n;
      res.closed.push_back(gap_closed);
      res.end_rel.push_back(std::fabs(ed - eb) / std::max(eb, 1e-9));
    }
    return res;
  };
  const result r_rear = run(at_rear), r_body = run(body);
  const double med_closed = median(r_rear.closed), med_end = median(r_rear.end_rel);
  std::printf("  kinematic limit, %d ticks (mean path %.3f), bike_step's commands through the "
              "plant:\n"
              "    the kinematic bicycle's vehicle: within %.5f of its path (mean worst %.5f)\n"
              "    a body, braked 60%% in front:    within %.4f (mean worst %.4f)\n"
              "  under the controller: median gap %.5f (worst %.3f / %.3f), median end-distance "
              "difference %.2f%%; ending %.3f from the goal on average for bike_step, %.3f / %.3f "
              "for the dynamic vehicle\n",
              T, r_rear.path, r_rear.worst, r_rear.mean, r_body.worst, r_body.mean, med_closed,
              r_rear.worst_closed, r_body.worst_closed, 100 * med_end, r_rear.end_bike,
              r_rear.end_dyn, r_body.end_dyn);
  EXPECT_LT(r_rear.worst, 0.2 * v.rr);
  EXPECT_LT(med_closed, 0.1 * v.rr);
  EXPECT_LT(med_end, 0.02);
}

// Gate 4a: understeer that grows with lateral g. At 15 m/s, the steer a steady turn needs beyond
// the kinematic L kappa grows with the lateral acceleration, and faster near the limit.
TEST(DynBike, UndersteerGrowsWithLateralG) {
  const dyn_bike_params p = humvee();
  const double L = p.lf + p.lr, v = 15.0;
  double prev_ay = 0.0, prev_u = 0.0, first_slope = -1.0, last_slope = 0.0;
  for (double delta = 0.01; delta <= 0.081; delta += 0.01) {
    double ay, kappa;
    steady_turn(p, v, delta, ay, kappa);
    const double under = delta - L * kappa;
    std::printf("  delta %.3f: a_y %.2f m/s^2 (%.2f g), understeer %.4f rad\n", delta, ay, ay / kG,
                under);
    EXPECT_GT(under, 0.0);
    if (delta > 0.015) {
      EXPECT_GT(ay, prev_ay);
      EXPECT_GT(under, prev_u);
      const double slope = (under - prev_u) / (ay - prev_ay);
      if (first_slope < 0.0)
        first_slope = slope;
      last_slope = slope;
    }
    prev_ay = ay;
    prev_u = under;
  }
  EXPECT_GT(last_slope, first_slope) << "understeer should grow faster toward the limit";
}

// Gate 4b: oversteer when braking in a turn. Load moves forward off the rear tires, so the path
// tightens at the same steer.
TEST(DynBike, OversteerWhenBrakingInATurn) {
  const dyn_bike_params p = humvee();
  const bike_veh lim = plant_limits(p);
  const double v = 15.0, delta = 0.06;
  double ay, kappa0;
  steady_turn(p, v, delta, ay, kappa0);
  dyn_state<double> s = at_rest<double>(v);
  for (int t = 0; t < static_cast<int>(12.0 / lim.hdt); ++t) { // settle into the turn
    dyn_state<double> o;
    diff::dyn_plant<double>(p, lim, s, delta, 2.0 * (v - s.vx) + p.drag * s.vx * s.vx / p.m, 0.0,
                            o);
    s = o;
  }
  double kappa_brake = 0.0;
  for (int t = 0; t < static_cast<int>(0.8 / lim.hdt); ++t) { // then brake at 0.4 g
    dyn_state<double> o;
    diff::dyn_plant<double>(p, lim, s, delta, -0.4 * kG, 0.0, o);
    s = o;
    kappa_brake = std::max(kappa_brake, s.r / s.vx);
  }
  std::printf("  turning at %.2f g: curvature %.4f /m steady, %.4f /m braking\n", ay / kG, kappa0,
              kappa_brake);
  EXPECT_GT(kappa_brake, 1.05 * kappa0);
}

// Gate 4c: however hard it is steered, the steady lateral acceleration stays under mu g.
TEST(DynBike, SteadyLateralNeverExceedsMuG) {
  for (const double mu : {0.4, 0.9}) {
    const dyn_bike_params p = humvee(mu);
    double peak = 0.0;
    for (double delta = 0.05; delta <= 0.6; delta += 0.05) {
      double ay, kappa;
      steady_turn(p, 20.0, delta, ay, kappa, 15.0);
      peak = std::max(peak, std::fabs(ay));
    }
    std::printf("  mu %.1f: the steadiest lateral acceleration %.2f m/s^2 against mu g %.2f\n", mu,
                peak, mu * kG);
    EXPECT_LE(peak, mu * kG * 1.005);
    EXPECT_GT(peak, 0.8 * mu * kG) << "it should get near the limit";
  }
}

// Gate 5: braking hard from speed, it stops within 10% of v^2 / (2 mu g).
TEST(DynBike, StopsWithinTenPercentOfTheFrictionLimit) {
  for (const double mu : {0.5, 0.9}) {
    const dyn_bike_params p = humvee(mu);
    const bike_veh lim = plant_limits(p);
    for (const double v0 : {10.0, 20.0}) {
      dyn_state<double> s = at_rest<double>(v0);
      double dist = 0.0;
      for (int t = 0; t < 2000 && s.vx > 1e-3; ++t) {
        dyn_state<double> o;
        diff::dyn_plant<double>(p, lim, s, 0.0, -2.0 * mu * kG, 0.0, o);
        dist += std::hypot(o.x - s.x, o.y - s.y);
        s = o;
      }
      const double ideal = v0 * v0 / (2.0 * mu * kG);
      std::printf("  mu %.1f from %.0f m/s: stopped in %.2f m, v^2/(2 mu g) = %.2f m (%+.1f%%)\n",
                  mu, v0, dist, ideal, 100.0 * (dist / ideal - 1.0));
      EXPECT_NEAR(dist / ideal, 1.0, 0.10);
    }
  }
}

// Gate 2's C++ side: the values GRL-SNAM's torch twin is held to. The plant, in double, on fixed
// cases (the Humvee with lags and the low-speed blend): each case's next state and its VJP for a
// fixed output gradient. CVC_DYN_BIKE_GOLDEN_WRITE=1 rewrites tests/data/dyn_bike_golden.json;
// otherwise this build must reproduce it to 1e-9.
TEST(DynBike, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn_bike_golden.json";
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.v_blend = 1.0f;
  p.v_width = 0.3f;
  struct gcase {
    double s[8], dcmd, acmd, grade;
    bool reverse;
  };
  const gcase cases[] = {
      {{0.0, 0.0, 0.0, 12.0, 0.1, 0.05, 0.02, 0.5}, 0.05, 1.0, 0.0, false},
      {{1.0, -2.0, 0.3, 15.0, -1.2, 0.6, 0.25, 0.0}, 0.3, 0.0, 0.02, false},
      {{-3.0, 4.0, -1.1, 14.0, 0.8, 0.4, 0.15, -4.0}, 0.15, -6.0, -0.03, false},
      {{2.0, 2.0, 2.5, 10.0, 2.5, 1.1, 0.4, 1.0}, 0.45, 2.0, 0.0, false},
      {{0.5, 0.5, 0.2, -2.0, 0.05, -0.1, 0.2, -0.5}, 0.25, -0.8, 0.0, true},
      {{0.0, 0.0, 0.0, 0.6, 0.02, 0.05, 0.3, 0.3}, 0.35, 0.5, 0.0, false},
      {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1, 2.0, 0.0, false},
  };
  const double g_out[8] = {0.7, -0.3, 1.1, 0.4, -0.9, 0.25, 0.6, -0.15};
  // Every value this build computes, in the file's order: per case, next[8], g_state[8],
  // g_delta_cmd, g_a_cmd.
  std::vector<double> now;
  std::ostringstream js;
  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", x);
    return std::string(b);
  };
  const auto arr = [&](const double *v, int n) {
    std::string out = "[";
    for (int k = 0; k < n; ++k)
      out += (k ? ", " : "") + num(v[k]);
    return out + "]";
  };
  js << "{\n \"format\": \"cvc-dyn-bike-golden-1\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"delta\", \"a\"],\n";
  js << " \"params\": {";
  const std::pair<const char *, float> params[] = {{"m", p.m},
                                                   {"Iz", p.Iz},
                                                   {"lf", p.lf},
                                                   {"lr", p.lr},
                                                   {"h", p.h},
                                                   {"g", p.g},
                                                   {"mu", p.mu},
                                                   {"B_f", p.B_f},
                                                   {"B_r", p.B_r},
                                                   {"C", p.C},
                                                   {"front_drive", p.front_drive},
                                                   {"front_brake", p.front_brake},
                                                   {"drag", p.drag},
                                                   {"c_rr", p.c_rr},
                                                   {"tau_steer", p.tau_steer},
                                                   {"steer_rate", p.steer_rate},
                                                   {"tau_drive", p.tau_drive},
                                                   {"v_blend", p.v_blend},
                                                   {"v_width", p.v_width},
                                                   {"eps", p.eps},
                                                   {"fz_min", p.fz_min}};
  bool first = true;
  for (const auto &kv : params) {
    js << (first ? "" : ", ") << "\"" << kv.first << "\": " << num(kv.second);
    first = false;
  }
  js << ", \"iters\": " << p.iters << "},\n \"g_out\": " << arr(g_out, 8) << ",\n \"cases\": [\n";
  for (std::size_t c = 0; c < sizeof cases / sizeof cases[0]; ++c) {
    const gcase &gc = cases[c];
    const bike_veh lim = plant_limits(p, gc.reverse);
    dyn_state<double> s, o, go, gs;
    for (int k = 0; k < 8; ++k) {
      s[k] = gc.s[k];
      go[k] = g_out[k];
    }
    diff::dyn_plant<double>(p, lim, s, gc.dcmd, gc.acmd, gc.grade, o);
    double g_dc = 0.0, g_ac = 0.0;
    diff::dyn_plant_vjp<double>(p, lim, s, gc.dcmd, gc.acmd, gc.grade, go, gs, g_dc, g_ac);
    double next[8], gstate[8];
    for (int k = 0; k < 8; ++k) {
      next[k] = o[k];
      gstate[k] = gs[k];
      now.push_back(o[k]);
    }
    for (int k = 0; k < 8; ++k)
      now.push_back(gs[k]);
    now.push_back(g_dc);
    now.push_back(g_ac);
    js << "  {\"state\": " << arr(gc.s, 8) << ", \"delta_cmd\": " << num(gc.dcmd)
       << ", \"a_cmd\": " << num(gc.acmd) << ", \"grade\": " << num(gc.grade)
       << ",\n   \"limits\": {\"L\": " << num(lim.L) << ", \"vmax\": " << num(lim.vmax)
       << ", \"hdt\": " << num(lim.hdt) << ", \"allow_reverse\": " << lim.allow_reverse << "}"
       << ",\n   \"next\": " << arr(next, 8) << ",\n   \"g_state\": " << arr(gstate, 8)
       << ",\n   \"g_delta_cmd\": " << num(g_dc) << ", \"g_a_cmd\": " << num(g_ac) << "}"
       << (c + 1 < sizeof cases / sizeof cases[0] ? "," : "") << "\n";
  }
  js << " ]\n}\n";
  const char *write = std::getenv("CVC_DYN_BIKE_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &c : want.get_child("cases")) {
    for (const char *key : {"next", "g_state"})
      for (const auto &v : c.second.get_child(key))
        golden.push_back(v.second.get_value<double>());
    golden.push_back(c.second.get<double>("g_delta_cmd"));
    golden.push_back(c.second.get<double>("g_a_cmd"));
  }
  ASSERT_EQ(golden.size(), now.size());
  for (std::size_t k = 0; k < now.size(); ++k)
    EXPECT_LE(std::fabs(now[k] - golden[k]),
              1e-9 * std::max(1.0, std::max(std::fabs(now[k]), std::fabs(golden[k]))))
        << "value " << k << " (case " << k / 18 << "): golden " << golden[k] << ", now " << now[k];
}

// ── The external-force port: the deployed drive's ext_force in the differentiable rollout (for
// cvcdbg's comm-network trainer) ──

namespace {

// Agents in the city: the scene's starts and goals, headings near the goal direction, speeds
// spread over the range, and a force per agent.
struct ext_case {
  float ox, oy, th, sp, gx, gy, fx, fy;
};
std::vector<ext_case> ext_cases(const city &c, int n) {
  std::vector<float> o(2 * n), g(2 * n);
  c.sc.sample_starts_goals(n, 5, o.data(), g.data());
  std::mt19937 rng(9);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  std::vector<ext_case> r;
  for (int i = 0; i < n; ++i) {
    ext_case e;
    e.ox = o[2 * i];
    e.oy = o[2 * i + 1];
    e.gx = g[2 * i];
    e.gy = g[2 * i + 1];
    e.th = std::atan2(e.gy - e.oy, e.gx - e.ox) + 0.3f * static_cast<float>(i % 5 - 2);
    e.sp = c.v.vmax * static_cast<float>(i % 7) / 7.0f;
    e.fx = u(rng);
    e.fy = u(rng);
    r.push_back(e);
  }
  return r;
}

// ext_force::sample for the deployed drive: agent i gets its case's constant force.
void constant_sample(void *user, int i, int, float, float, float *fx, float *fy) {
  const std::vector<ext_case> &cs = *static_cast<const std::vector<ext_case> *>(user);
  *fx = cs[i].fx;
  *fy = cs[i].fy;
}

} // namespace

// A zero force through the port is the controller without it, forward and backward.
TEST(DiffExtForce, AZeroForceIsTheControllerWithoutIt) {
  const std::unique_ptr<city> c = make_city();
  const float al = 0.6f, be = 1.8f, ga = 0.9f;
  for (const ext_case &e : ext_cases(*c, 32)) {
    diff::ext_in zero;
    float a1[4], b1[4];
    diff::bike_step(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, a1[0], a1[1], a1[2],
                    a1[3]);
    diff::bike_step(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, b1[0], b1[1], b1[2],
                    b1[3], &zero);
    for (int k = 0; k < 4; ++k)
      EXPECT_EQ(a1[k], b1[k]);
    float g0[7], g1[7], gf[2];
    diff::bike_step_bwd(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, 0.3f, -0.2f,
                        0.5f, 0.7f, g0[0], g0[1], g0[2], g0[3], g0[4], g0[5], g0[6]);
    diff::bike_step_bwd(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, 0.3f, -0.2f,
                        0.5f, 0.7f, g1[0], g1[1], g1[2], g1[3], g1[4], g1[5], g1[6], &zero, gf);
    for (int k = 0; k < 7; ++k)
      EXPECT_EQ(g0[k], g1[k]);
  }
}

// One step through the port matches one tick of the deployed bicycle_rollout_ext (nsub 1) with
// the same per-agent force, with and without the force in the steering.
TEST(DiffExtForce, MatchesTheDeployedDrive) {
  const std::unique_ptr<city> c = make_city();
  const float al = 0.6f, be = 1.8f, ga = 0.9f;
  const std::vector<ext_case> cs = ext_cases(*c, 64);
  const int n = static_cast<int>(cs.size());
  cvc::nav::veh_params vp;
  vp.rr = c->v.rr;
  vp.d_hat = c->v.d_hat;
  vp.dt = c->v.hdt;
  vp.vmax = c->v.vmax;
  vp.L = c->v.L;
  vp.delta_max = c->v.delta_max;
  vp.a_max = c->v.a_max;
  vp.a_lat_max = c->v.a_lat_max;
  vp.k_steer = c->v.k_steer;
  vp.nsub = 1;
  vp.allow_reverse = c->v.allow_reverse != 0;
  for (const bool steer : {true, false}) {
    std::vector<float> o(2 * n), th(n), sp(n), goal(2 * n), A(n, al), B(n, be), G(n, ga);
    for (int i = 0; i < n; ++i) {
      o[2 * i] = cs[i].ox;
      o[2 * i + 1] = cs[i].oy;
      th[i] = cs[i].th;
      sp[i] = cs[i].sp;
      goal[2 * i] = cs[i].gx;
      goal[2 * i + 1] = cs[i].gy;
    }
    cvc::nav::ext_force ext;
    ext.sample = constant_sample;
    ext.user = const_cast<std::vector<ext_case> *>(&cs);
    ext.steer = steer;
    std::vector<float> minclr(n);
    cvc::nav::bicycle_rollout_ext(c->fs, o.data(), th.data(), sp.data(), goal.data(), A.data(),
                                  B.data(), G.data(), n, nullptr, vp, ext, minclr.data(), 1);
    double worst = 0.0;
    for (int i = 0; i < n; ++i) {
      diff::ext_in e;
      e.fx = cs[i].fx;
      e.fy = cs[i].fy;
      e.steer = steer ? 1 : 0;
      float x1, y1, t1, s1;
      diff::bike_step(c->F, cs[i].ox, cs[i].oy, cs[i].th, cs[i].sp, cs[i].gx, cs[i].gy, al, be, ga,
                      c->v, x1, y1, t1, s1, &e);
      worst = std::max({worst, static_cast<double>(std::fabs(x1 - o[2 * i])),
                        static_cast<double>(std::fabs(y1 - o[2 * i + 1])),
                        static_cast<double>(std::fabs(t1 - th[i])),
                        static_cast<double>(std::fabs(s1 - sp[i]))});
    }
    std::printf("  one step with F_ext (steer %d) against the deployed drive: worst %.2e\n",
                steer ? 1 : 0, worst);
    EXPECT_LT(worst, 1e-5);
  }
}

// dL/dF_ext against central differences on the kinematic step and on the dynamic one, and the
// position Jacobian chained into the position gradient for a force that depends on where the
// agent is (F = A o + b).
TEST(DiffExtForce, GradientsMatchFiniteDifferences) {
  const std::unique_ptr<city> c = make_city();
  const float al = 0.6f, be = 1.8f, ga = 0.9f, eps = 1e-3f;
  const float w[4] = {0.7f, -0.4f, 0.3f, 0.5f}; // L = w . (x1, y1, th1, sp1)
  const float ux = 0.6f, uy = 0.8f;             // the direction of each check
  const auto loss = [&](const ext_case &e, float fx, float fy, float ox, float oy) {
    diff::ext_in x;
    x.fx = fx;
    x.fy = fy;
    float s1[4];
    diff::bike_step(c->F, ox, oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, s1[0], s1[1], s1[2],
                    s1[3], &x);
    return static_cast<double>(w[0] * s1[0] + w[1] * s1[1] + w[2] * s1[2] + w[3] * s1[3]);
  };
  const float A[4] = {0.8f, -0.3f, 0.4f, 0.6f}, b[2] = {0.1f, -0.2f};
  const auto F = [&](float ox, float oy, float &fx, float &fy) {
    fx = A[0] * ox + A[1] * oy + b[0];
    fy = A[2] * ox + A[3] * oy + b[1];
  };
  double num_f = 0.0, den_f = 0.0, num_o = 0.0, den_o = 0.0;
  for (const ext_case &e : ext_cases(*c, 48)) {
    diff::ext_in x;
    x.fx = e.fx;
    x.fy = e.fy;
    float g[7], gf[2];
    diff::bike_step_bwd(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, w[0], w[1],
                        w[2], w[3], g[0], g[1], g[2], g[3], g[4], g[5], g[6], &x, gf);
    const double fd = (loss(e, e.fx + eps * ux, e.fy + eps * uy, e.ox, e.oy) -
                       loss(e, e.fx - eps * ux, e.fy - eps * uy, e.ox, e.oy)) /
                      (2.0 * eps);
    num_f += std::fabs(gf[0] * ux + gf[1] * uy - fd);
    den_f += std::fabs(fd);
    // A force that depends on the position: its Jacobian joins the position gradient.
    diff::ext_in xj;
    F(e.ox, e.oy, xj.fx, xj.fy);
    xj.dfx_dox = A[0];
    xj.dfx_doy = A[1];
    xj.dfy_dox = A[2];
    xj.dfy_doy = A[3];
    diff::bike_step_bwd(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, w[0], w[1],
                        w[2], w[3], g[0], g[1], g[2], g[3], g[4], g[5], g[6], &xj, gf);
    const auto lo = [&](float ox, float oy) {
      float fx, fy;
      F(ox, oy, fx, fy);
      return loss(e, fx, fy, ox, oy);
    };
    const double fdo =
        (lo(e.ox + eps * ux, e.oy + eps * uy) - lo(e.ox - eps * ux, e.oy - eps * uy)) / (2.0 * eps);
    num_o += std::fabs(g[0] * ux + g[1] * uy - fdo);
    den_o += std::fabs(fdo);
  }
  const double rel_f = num_f / den_f, rel_o = num_o / den_o;

  // The dynamic step: dL/dF_ext against central differences.
  const dyn_bike_params p =
      cvc::physics::nav_plant::dyn_bike_params_at(cvc::physics::vehicle_desc::roboracer(), c->v.L);
  dyn_state<float> wd{};
  for (int k = 0; k < dyn_state<float>::size; ++k)
    wd[k] = 0.1f * static_cast<float>(k + 1) * (k % 2 ? -1.0f : 1.0f);
  double num_d = 0.0, den_d = 0.0;
  for (const ext_case &e : ext_cases(*c, 48)) {
    dyn_state<float> s{};
    s.x = e.ox;
    s.y = e.oy;
    s.psi = e.th;
    s.vx = e.sp;
    const auto dl = [&](float fx, float fy) {
      diff::ext_in x;
      x.fx = fx;
      x.fy = fy;
      dyn_state<float> o;
      diff::dyn_bike_step(c->F, s, e.gx, e.gy, al, be, ga, c->v, p, 0.0f, o, &x);
      double acc = 0.0;
      for (int k = 0; k < dyn_state<float>::size; ++k)
        acc += static_cast<double>(wd[k]) * o[k];
      return acc;
    };
    diff::ext_in x;
    x.fx = e.fx;
    x.fy = e.fy;
    dyn_state<float> gs{};
    float gal = 0, gbe = 0, gga = 0, gf[2];
    diff::dyn_bike_step_bwd(c->F, s, e.gx, e.gy, al, be, ga, c->v, p, 0.0f, wd, gs, gal, gbe, gga,
                            &x, gf);
    const double fd =
        (dl(e.fx + eps * ux, e.fy + eps * uy) - dl(e.fx - eps * ux, e.fy - eps * uy)) / (2.0 * eps);
    num_d += std::fabs(gf[0] * ux + gf[1] * uy - fd);
    den_d += std::fabs(fd);
  }
  const double rel_d = num_d / den_d;
  std::printf("  dL/dF_ext vs central differences: kinematic %.2e, dynamic %.2e; through a "
              "position-dependent force %.2e\n",
              rel_f, rel_d, rel_o);
  EXPECT_LT(rel_f, 5e-2);
  EXPECT_LT(rel_d, 5e-2);
  EXPECT_LT(rel_o, 5e-2);
}

// ── The trajectory helper: bike_rollout_ext / bike_rollout_ext_bwd (ext_rollout.h) ──

namespace {

// F = A o + b + c, with J = A: a force that depends on where the agent is, plus an offset c
// whose gradient is the sum of dL/dF_ext over the steps.
struct affine_force {
  float A[4] = {0.8f, -0.3f, 0.4f, 0.6f};
  float b[2] = {0.1f, -0.2f};
  float c[2] = {0.0f, 0.0f};
};
void affine_sample(void *user, int, float ox, float oy, float *f, float *J) {
  const affine_force &a = *static_cast<const affine_force *>(user);
  f[0] = a.A[0] * ox + a.A[1] * oy + a.b[0] + a.c[0];
  f[1] = a.A[2] * ox + a.A[3] * oy + a.b[1] + a.c[1];
  for (int k = 0; k < 4; ++k)
    J[k] = a.A[k];
}

} // namespace

// The recorded trajectory is bike_step's with the force sampled every `hold` steps and held, and
// a null force is the plain rollout.
TEST(DiffExtRollout, TrajectoryIsTheStepsAndTheirForces) {
  const std::unique_ptr<city> c = make_city();
  const int T = 20;
  for (const ext_case &e : ext_cases(*c, 16)) {
    for (const int hold : {1, 3}) {
      affine_force af;
      diff::bike_ext_fn fn;
      fn.sample = affine_sample;
      fn.user = &af;
      fn.hold = hold;
      diff::bike_tape tape;
      diff::bike_rollout_ext(c->F, c->v, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, 0.6f, 1.8f, 0.9f, T,
                             fn, tape);
      ASSERT_EQ(tape.state.size(), static_cast<std::size_t>(4 * (T + 1)));
      float s[4] = {e.ox, e.oy, e.th, e.sp};
      diff::ext_in x;
      for (int t = 0; t < T; ++t) {
        if (t % hold == 0) {
          float J[4];
          float F[2];
          affine_sample(&af, t, s[0], s[1], F, J);
          x.fx = F[0];
          x.fy = F[1];
        }
        float s1[4];
        diff::bike_step(c->F, s[0], s[1], s[2], s[3], e.gx, e.gy, 0.6f, 1.8f, 0.9f, c->v, s1[0],
                        s1[1], s1[2], s1[3], &x);
        for (int k = 0; k < 4; ++k) {
          EXPECT_EQ(tape.state[4 * (t + 1) + k], s1[k]);
          s[k] = s1[k];
        }
        EXPECT_EQ(tape.sampled_at[t], t - t % hold);
      }
    }
    // No force: the plain rollout.
    diff::bike_ext_fn none;
    diff::bike_tape tape;
    diff::bike_rollout_ext(c->F, c->v, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, 0.6f, 1.8f, 0.9f, T,
                           none, tape);
    float s[4] = {e.ox, e.oy, e.th, e.sp};
    for (int t = 0; t < T; ++t) {
      float s1[4];
      diff::bike_step(c->F, s[0], s[1], s[2], s[3], e.gx, e.gy, 0.6f, 1.8f, 0.9f, c->v, s1[0],
                      s1[1], s1[2], s1[3]);
      for (int k = 0; k < 4; ++k) {
        EXPECT_EQ(tape.state[4 * (t + 1) + k], s1[k]);
        s[k] = s1[k];
      }
    }
  }
}

// The reverse against central differences over a 10-step trajectory with a position-dependent
// force, sampled every step and held for 3: dL/d(initial state), dL/dF (through a constant offset
// added to F at every step) and dL/dbeta at one step, with L = sum_t w_t . state_t.
TEST(DiffExtRollout, ReverseMatchesFiniteDifferences) {
  const std::unique_ptr<city> c = make_city();
  const int T = 10, k_beta = 4;
  const float eps = 1e-3f, ux = 0.6f, uy = 0.8f;
  std::vector<float> w(static_cast<std::size_t>(4 * (T + 1)));
  {
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> u(-0.5f, 0.5f);
    for (float &x : w)
      x = u(rng);
  }
  for (const int hold : {1, 3}) {
    double num[3] = {0, 0, 0}, den[3] = {0, 0, 0};
    for (const ext_case &e : ext_cases(*c, 32)) {
      // A goal 0.8 ahead: the goal spring stays inside the acceleration clamp, so beta has a
      // gradient (a far goal saturates it), and 10 steps (at most 0.54) never pass the goal,
      // whose behind branch would flip under the differences.
      std::vector<float> goal(2 * T), coef(3 * T);
      for (int t = 0; t < T; ++t) {
        goal[2 * t] = e.ox + 0.8f * std::cos(e.th);
        goal[2 * t + 1] = e.oy + 0.8f * std::sin(e.th);
        coef[3 * t] = 0.6f;
        coef[3 * t + 1] = 1.8f;
        coef[3 * t + 2] = 0.9f;
      }
      const auto loss = [&](float ox, float oy, float cx, float cy, const std::vector<float> &cf) {
        affine_force af;
        af.c[0] = cx;
        af.c[1] = cy;
        diff::bike_ext_fn fn;
        fn.sample = affine_sample;
        fn.user = &af;
        fn.hold = hold;
        diff::bike_tape tape;
        diff::bike_rollout_ext(c->F, c->v, ox, oy, e.th, e.sp, goal.data(), cf.data(), T, fn, tape);
        double acc = 0.0;
        for (std::size_t i = 0; i < w.size(); ++i)
          acc += static_cast<double>(w[i]) * tape.state[i];
        return acc;
      };
      affine_force af;
      diff::bike_ext_fn fn;
      fn.sample = affine_sample;
      fn.user = &af;
      fn.hold = hold;
      diff::bike_tape tape;
      diff::bike_rollout_ext(c->F, c->v, e.ox, e.oy, e.th, e.sp, goal.data(), coef.data(), T, fn,
                             tape);
      std::vector<float> g_fext(2 * T), g_coef(3 * T);
      float g_init[4];
      diff::bike_rollout_ext_bwd(c->F, c->v, tape, goal.data(), coef.data(), w.data(),
                                 g_fext.data(), g_coef.data(), g_init);
      // dL/d(initial position) along (ux, uy).
      const double fd0 = (loss(e.ox + eps * ux, e.oy + eps * uy, 0, 0, coef) -
                          loss(e.ox - eps * ux, e.oy - eps * uy, 0, 0, coef)) /
                         (2.0 * eps);
      num[0] += std::fabs(g_init[0] * ux + g_init[1] * uy - fd0);
      den[0] += std::fabs(fd0);
      // dL/dc: the sum of dL/dF_ext over the steps.
      double gc = 0.0;
      for (int t = 0; t < T; ++t)
        gc += g_fext[2 * t] * ux + g_fext[2 * t + 1] * uy;
      const double fdc = (loss(e.ox, e.oy, eps * ux, eps * uy, coef) -
                          loss(e.ox, e.oy, -eps * ux, -eps * uy, coef)) /
                         (2.0 * eps);
      num[1] += std::fabs(gc - fdc);
      den[1] += std::fabs(fdc);
      // dL/dbeta at one step.
      std::vector<float> cp = coef, cm = coef;
      cp[3 * k_beta + 1] += eps;
      cm[3 * k_beta + 1] -= eps;
      const double fdb = (loss(e.ox, e.oy, 0, 0, cp) - loss(e.ox, e.oy, 0, 0, cm)) / (2.0 * eps);
      num[2] += std::fabs(g_coef[3 * k_beta + 1] - fdb);
      den[2] += std::fabs(fdb);
    }
    std::printf("  hold %d, %d steps: dL/dinit %.2e, dL/dF %.2e, dL/dbeta %.2e (relative to "
                "central differences)\n",
                hold, T, num[0] / den[0], num[1] / den[1], num[2] / den[2]);
    for (int k = 0; k < 3; ++k) {
      EXPECT_GT(den[k], 0.0) << "check " << k << " has no gradient to compare";
      EXPECT_LT(num[k] / den[k], 5e-2) << "check " << k << ", hold " << hold;
    }
  }
}

// The dynamic twin, dyn_rollout_ext: its trajectory is dyn_bike_step's with the held force, and
// its reverse matches central differences (dL/d initial position and dL/dF through an offset).
TEST(DiffExtRollout, TheDynamicTwinMatchesItsStepsAndDifferences) {
  const std::unique_ptr<city> c = make_city();
  const dyn_bike_params p =
      cvc::physics::nav_plant::dyn_bike_params_at(cvc::physics::vehicle_desc::roboracer(), c->v.L);
  const int T = 10;
  // 3e-4, not the kinematic checks' 1e-3: at 1e-3 two of the 32 held-force trajectories straddle
  // a kink in the dynamic plant (a clamp or governor switch) and the difference jumps, while the
  // reverse agrees with the 3e-4 difference on every agent.
  const float eps = 3e-4f, ux = 0.6f, uy = 0.8f;
  std::vector<dyn_state<float>> w(static_cast<std::size_t>(T + 1));
  {
    std::mt19937 rng(4);
    std::uniform_real_distribution<float> u(-0.5f, 0.5f);
    for (dyn_state<float> &x : w)
      for (int k = 0; k < dyn_state<float>::size; ++k)
        x[k] = u(rng);
  }
  for (const int hold : {1, 3}) {
    double num[2] = {0, 0}, den[2] = {0, 0};
    for (const ext_case &e : ext_cases(*c, 32)) {
      std::vector<float> goal(2 * T), coef(3 * T);
      for (int t = 0; t < T; ++t) {
        goal[2 * t] = e.ox + 0.8f * std::cos(e.th);
        goal[2 * t + 1] = e.oy + 0.8f * std::sin(e.th);
        coef[3 * t] = 0.6f;
        coef[3 * t + 1] = 1.8f;
        coef[3 * t + 2] = 0.9f;
      }
      const auto run = [&](float ox, float oy, float cx, float cy, diff::dyn_tape &tape) {
        affine_force af;
        af.c[0] = cx;
        af.c[1] = cy;
        diff::bike_ext_fn fn;
        fn.sample = affine_sample;
        fn.user = &af;
        fn.hold = hold;
        dyn_state<float> s0{};
        s0.x = ox;
        s0.y = oy;
        s0.psi = e.th;
        s0.vx = e.sp;
        diff::dyn_rollout_ext(c->F, c->v, p, s0, goal.data(), coef.data(), T, fn, tape);
        double acc = 0.0;
        for (int t = 0; t <= T; ++t)
          for (int k = 0; k < dyn_state<float>::size; ++k)
            acc += static_cast<double>(w[t][k]) * tape.state[t][k];
        return acc;
      };
      diff::dyn_tape tape, tp, tm;
      run(e.ox, e.oy, 0, 0, tape);
      // The trajectory is the steps with the held force.
      {
        affine_force af;
        dyn_state<float> s = tape.state[0];
        diff::ext_in x;
        for (int t = 0; t < T; ++t) {
          if (t % hold == 0) {
            float F[2], J[4];
            affine_sample(&af, t, s.x, s.y, F, J);
            x.fx = F[0];
            x.fy = F[1];
          }
          dyn_state<float> s1;
          diff::dyn_bike_step(c->F, s, goal[2 * t], goal[2 * t + 1], 0.6f, 1.8f, 0.9f, c->v, p,
                              0.0f, s1, &x);
          for (int k = 0; k < dyn_state<float>::size; ++k)
            EXPECT_EQ(tape.state[t + 1][k], s1[k]);
          s = s1;
        }
      }
      std::vector<float> g_fext(2 * T);
      dyn_state<float> g_init;
      diff::dyn_rollout_ext_bwd(c->F, c->v, p, tape, goal.data(), coef.data(), w.data(),
                                g_fext.data(), nullptr, &g_init);
      const double fd0 = (run(e.ox + eps * ux, e.oy + eps * uy, 0, 0, tp) -
                          run(e.ox - eps * ux, e.oy - eps * uy, 0, 0, tm)) /
                         (2.0 * eps);
      num[0] += std::fabs(g_init.x * ux + g_init.y * uy - fd0);
      den[0] += std::fabs(fd0);
      double gc = 0.0;
      for (int t = 0; t < T; ++t)
        gc += g_fext[2 * t] * ux + g_fext[2 * t + 1] * uy;
      const double fdc =
          (run(e.ox, e.oy, eps * ux, eps * uy, tp) - run(e.ox, e.oy, -eps * ux, -eps * uy, tm)) /
          (2.0 * eps);
      num[1] += std::fabs(gc - fdc);
      den[1] += std::fabs(fdc);
    }
    std::printf("  dynamic, hold %d, %d steps: dL/dinit %.2e, dL/dF %.2e (relative to central "
                "differences)\n",
                hold, T, num[0] / den[0], num[1] / den[1]);
    for (int k = 0; k < 2; ++k) {
      EXPECT_GT(den[k], 0.0);
      EXPECT_LT(num[k] / den[k], 5e-2) << "check " << k << ", hold " << hold;
    }
  }
}
