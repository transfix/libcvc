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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/ext_rollout.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <filesystem>
#include <fstream>
#include <functional>
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
// vx r and the path's curvature r / vx. `a_body`, when given, receives the body-frame lateral
// acceleration over that second, vy' + r vx: vx r alone is it only in a steady turn, and past
// its limit a vehicle drifts.
void steady_turn(const dyn_bike_params &p, double v, double delta, double &a_y, double &kappa,
                 double seconds = 12.0, double *a_body = nullptr) {
  const bike_veh lim = plant_limits(p);
  dyn_state<double> s = at_rest<double>(v);
  const int n = static_cast<int>(seconds / lim.hdt), tail = static_cast<int>(1.0 / lim.hdt);
  double ay = 0.0, k = 0.0, vy0 = 0.0;
  for (int t = 0; t < n; ++t) {
    if (t == n - tail)
      vy0 = s.vy;
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
  if (a_body)
    *a_body = a_y + (s.vy - vy0) / (tail * lim.hdt);
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
  // The cornering stiffness per unit load, B C, is kJoltTireStiffness times Jolt's linear ramp
  // to its peak, 1 / alpha_peak; front and rear alike.
  EXPECT_NEAR(si.tire_B_rear * si.tire_C * d.tire_peak_slip_angle,
              cvc::physics::nav_plant::kJoltTireStiffness, 1e-9);
  EXPECT_DOUBLE_EQ(si.tire_B_front, si.tire_B_rear);
  EXPECT_DOUBLE_EQ(si.front_drive, 0.5); // the Humvee drives all four wheels
  // A geared drive through a torque converter: the force lags, drag and rolling resistance stay.
  EXPECT_DOUBLE_EQ(si.drive_lag, 0.1);
  EXPECT_GT(si.drag, 0.0);
  EXPECT_GT(si.rolling_resistance, 0.0);
  // The RoboRacer's direct drive answers within the tick, the driver model making up for drag and
  // rolling resistance; an explicit drive lag still wins.
  const cvc::physics::vehicle_desc rr = cvc::physics::vehicle_desc::roboracer();
  const cvc::nav::dyn_bike_si sr = cvc::physics::nav_plant::dyn_bike_si_of(rr);
  EXPECT_DOUBLE_EQ(sr.drive_lag, 0.0);
  EXPECT_DOUBLE_EQ(sr.drag, 0.0);
  EXPECT_DOUBLE_EQ(sr.rolling_resistance, 0.0);
  EXPECT_DOUBLE_EQ(cvc::physics::nav_plant::dyn_bike_si_of(rr, 0.9, 1.0, 0.05).drive_lag, 0.05);
  EXPECT_NEAR(sr.tire_B_rear * sr.tire_C * rr.tire_peak_slip_angle,
              cvc::physics::nav_plant::kJoltTireStiffness, 1e-9);
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
// The lateral acceleration is the body's, vy' + r vx: past its limit the vehicle drifts, where vx r
// alone overstates it.
TEST(DynBike, SteadyLateralNeverExceedsMuG) {
  for (const double mu : {0.4, 0.9}) {
    const dyn_bike_params p = humvee(mu);
    double peak = 0.0;
    for (double delta = 0.05; delta <= 0.6; delta += 0.05) {
      double ay, kappa, a_body;
      steady_turn(p, 20.0, delta, ay, kappa, 15.0, &a_body);
      peak = std::max(peak, std::fabs(a_body));
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

// ── The plant in port-Hamiltonian form (dyn_ph_*, doc 20's rungs 2 and 3) ─────────────────────
// H = 1/2 p^T M^-1 p, p' = (J - R) v + G u + f_eps - m g grade e_x, u = (F_xf, F_xr), with R the
// tires' secants, drag and rolling resistance. These gates hold dyn_plant to that form at every
// slip, sliding included, book its steps, and report where the tires' tangent stops being
// positive semi-definite past the Pacejka peak while the secant does not. dyn_plant's arithmetic
// is unchanged (DynBike.Golden still pins it).

namespace {

constexpr double kTwoPi = 6.283185307179586; // M_PI is not portable (MSVC)

// A Humvee state anywhere in the envelope: forward or reversing, the wheel up to 0.5 rad, and
// sideways and yaw rates well past the tires' peaks.
dyn_state<double> sliding_state(std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn_state<double> s{};
  const double speed = 3.0 + 25.0 * 0.5 * (1.0 + u(rng));
  s.vx = u(rng) < -0.7 ? -std::min(speed, 8.0) : speed;
  s.delta = 0.5 * u(rng);
  s.vy = 4.0 * u(rng);
  s.r = 1.2 * u(rng);
  s.a = 5.0 * u(rng);
  s.psi = u(rng);
  return s;
}

// The tires' lateral force and the body-frame lateral acceleration Q_y / m at a state (dyn_plant's
// step 4 at the state, its own a and delta): a_y = vy' + r vx.
double lateral_g(const dyn_bike_params &p, const dyn_state<double> &s) {
  diff::dyn_ph_parts<double> q;
  diff::dyn_ph_parts_at(p, s, q);
  return (q.Fyf * q.cd + q.Fyr) / (p.m * p.g);
}

// Whether either tire is past its peak (its tangent stiffness negative).
bool past_peak(const dyn_bike_params &p, const dyn_state<double> &s) {
  double Dt[2][2], kf, kr, af, ar;
  diff::dyn_ph_tangent_D(p, s, Dt, kf, kr, af, ar);
  return kf < 0.0 || kr < 0.0;
}

} // namespace

// J is skew and R is a sum of positive multiples of outer products at every slip, so R is
// positive semi-definite sliding or not; with no axle force the inputs supply nothing, however the
// wheel turns.
TEST(DynBikePH, TheStructureIsPortHamiltonianAtEverySlip) {
  for (const double mu : {0.9, 0.6}) {
    const dyn_bike_params p = humvee(mu);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int past = 0;
    for (int k = 0; k < 500; ++k) {
      dyn_state<double> s = sliding_state(rng);
      if (k % 5 == 0)
        s.a = 0.0;
      double pm[3], J[3][3], R[3][3];
      diff::dyn_ph_momentum(p, s, pm);
      diff::dyn_ph_J(pm, J);
      diff::dyn_ph_R(p, s, R);
      diff::dyn_ph_parts<double> q;
      diff::dyn_ph_parts_at(p, s, q);
      const double v[3] = {s.vx, s.vy, s.r};
      double vJv = 0.0, norm = 0.0;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
          EXPECT_EQ(J[i][j], -J[j][i]);
          EXPECT_EQ(R[i][j], R[j][i]);
          vJv += v[i] * J[i][j] * v[j];
          norm = std::max(norm, std::fabs(R[i][j]));
        }
      EXPECT_NEAR(vJv, 0.0, 1e-9 * (1.0 + pm[0] * pm[0]));
      // The coefficients of R's three outer products are nonnegative: it is PSD.
      EXPECT_GT(q.Dx, 0.0);
      EXPECT_GE(q.Sf / q.cd, 0.0) << "front secant at vx " << s.vx << ", vy " << s.vy;
      EXPECT_GE(q.Sr, 0.0) << "rear secant at vx " << s.vx << ", vy " << s.vy;
      for (int t = 0; t < 4; ++t) {
        const double w[3] = {u(rng), u(rng), u(rng)};
        double wRw = 0.0;
        for (int i = 0; i < 3; ++i)
          for (int j = 0; j < 3; ++j)
            wRw += w[i] * R[i][j] * w[j];
        EXPECT_GE(wRw, -1e-12 * norm);
      }
      // dH/dp = v: the Hamiltonian's gradient in the momenta is the twist.
      EXPECT_NEAR(diff::dyn_ph_hamiltonian(p, s), 0.5 * (pm[0] * s.vx + pm[1] * s.vy + pm[2] * s.r),
                  1e-9 * (1 + pm[0] * s.vx));
      // Steering is not a port: with the axles' forces at zero nothing is supplied.
      if (s.a == 0.0)
        EXPECT_EQ(diff::dyn_ph_rates_at(p, s, 0.0).supplied, 0.0);
      past += past_peak(p, s);
    }
    std::printf("  mu %.1f: %d of 500 states have a tire past its peak\n", mu, past);
    EXPECT_GT(past, 100) << "the states do not slide";
  }
}

// One very short dyn_plant step's momentum change against the port-Hamiltonian vector field at
// every slip, forward and reversing, flat and on a 3% grade, with the actuators holding (commands
// equal to their state) so u is the state's own. The low-speed blend is a per-step limiter, not
// part of the field, so it is off here (dyn_plant_books charges it).
TEST(DynBikePH, TheVectorFieldIsThePlantsAtEverySlip) {
  for (const double mu : {0.9, 0.6}) {
    dyn_bike_params p = humvee(mu);
    p.v_blend = 0.0f;
    bike_veh lim = plant_limits(p, true);
    lim.hdt = 1e-5f;
    std::mt19937 rng(5);
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
      const dyn_state<double> s = sliding_state(rng);
      const double grade = (k % 2) * 0.03;
      dyn_state<double> o;
      diff::dyn_plant<double>(p, lim, s, s.delta, s.a, grade, o);
      double p0[3], p1[3], f[3];
      diff::dyn_ph_momentum(p, s, p0);
      diff::dyn_ph_momentum(p, o, p1);
      diff::dyn_ph_rhs(p, s, grade, f);
      // Relative to the forces' scale on each axis (m g on the momenta, m g L on the yaw).
      const double scale[3] = {p.m * p.g, p.m * p.g, p.m * p.g * (p.lf + p.lr)};
      for (int i = 0; i < 3; ++i) {
        const double fd = (p1[i] - p0[i]) / lim.hdt;
        const double rel = std::fabs(fd - f[i]) / scale[i];
        worst = std::max(worst, rel);
        EXPECT_LT(rel, 1e-3) << "axis " << i << ": plant " << fd << ", pH " << f[i] << " at vx "
                             << s.vx << ", vy " << s.vy << ", r " << s.r << ", delta " << s.delta;
      }
    }
    std::printf(
        "  vector field vs dyn_plant at every slip, mu %.1f: worst |dp/dt - pH| = %.2e m g\n", mu,
        worst);
  }
}

namespace {

// One run's books: per-step mean |residual|, and the sums.
struct run_books {
  double mean_abs_residual = 0.0, residual = 0.0, flow = 0.0, dH = 0.0, limiter = 0.0;
  double dissipated = 0.0, supplied = 0.0;
};

// dyn_plant_books along a trajectory: `swerve` weaves at 0.7 Hz past the peak with the throttle
// working; otherwise the Humvee brakes hard to a stop in a turn, so the speed clamp and the
// low-speed blend act.
run_books book_run(const dyn_bike_params &p, double hdt, bool swerve, double seconds) {
  bike_veh lim = plant_limits(p);
  lim.hdt = static_cast<float>(hdt);
  dyn_state<double> s = at_rest<double>(swerve ? 15.0 : 9.0);
  const int n = static_cast<int>(std::lround(seconds / hdt));
  run_books rb;
  const double h0 = diff::dyn_ph_hamiltonian(p, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd = swerve ? 0.3 * std::sin(kTwoPi * 0.7 * time) : 0.35;
    const double acmd = swerve ? 1.5 * std::sin(kTwoPi * time / 3.0) : -6.0;
    dyn_state<double> o;
    diff::dyn_ph_books<double> b;
    diff::dyn_plant_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b);
    rb.mean_abs_residual += std::fabs(b.residual);
    rb.residual += b.residual;
    rb.limiter += b.limiter;
    rb.dissipated += b.dissipated;
    rb.supplied += b.supplied;
    rb.flow += std::fabs(b.dissipated) + std::fabs(b.supplied) + std::fabs(b.floor) +
               std::fabs(b.grade) + std::fabs(b.limiter);
    s = o;
  }
  rb.mean_abs_residual /= n;
  rb.dH = diff::dyn_ph_hamiltonian(p, s) - h0;
  return rb;
}

} // namespace

// The gate: every step's books close but for the integrator's error, which is second order per
// step (so first order over a run), sliding or not, with the limiters charged when they act. The
// actuators lag as they do in training; u is their output.
TEST(DynBikePH, StepBooksCloseToSecondOrder) {
  const dyn_bike_params p = humvee(0.6);
  // Per step, the residual falls by about 4 per halving of hdt.
  double prev = 0.0;
  for (const double hdt : {0.02, 0.01, 0.005, 0.0025}) {
    const run_books rb = book_run(p, hdt, true, 6.0);
    std::printf("  swerve past the peak, hdt %.4f s: dH %.4g J, supplied %.4g, dissipated %.4g, "
                "mean |residual| per step %.3e J, |sum| / flow %.2e\n",
                hdt, rb.dH, rb.supplied, rb.dissipated, rb.mean_abs_residual,
                std::fabs(rb.residual) / rb.flow);
    if (prev > 0.0) {
      const double ratio = prev / rb.mean_abs_residual;
      EXPECT_GT(ratio, 3.0) << "hdt " << hdt;
      EXPECT_LT(ratio, 5.5) << "hdt " << hdt;
    }
    prev = rb.mean_abs_residual;
    if (hdt <= 0.01)
      EXPECT_LT(std::fabs(rb.residual) / rb.flow, 5e-3) << "hdt " << hdt;
  }
  // Braking to a stop in a turn: the clamp at zero speed and the blend take kinetic energy that
  // the books charge to the limiters; the rest closes as before.
  for (const double hdt : {0.01, 0.005}) {
    const run_books rb = book_run(p, hdt, false, 3.0);
    std::printf("  braking to a stop, hdt %.4f s: dH %.4g J, limiters %.4g J, |sum residual| / "
                "flow %.2e\n",
                hdt, rb.dH, rb.limiter, std::fabs(rb.residual) / rb.flow);
    EXPECT_LT(rb.limiter, 0.0) << "the limiters did not act";
    EXPECT_LT(std::fabs(rb.residual) / rb.flow, 5e-3) << "hdt " << hdt;
  }
}

// f_eps is the speed floor's: it shrinks with the floor and is gone without it.
TEST(DynBikePH, TheFloorTermVanishesWithTheFloor) {
  dyn_bike_params p = humvee();
  double prev = 1e300;
  for (const double eps : {0.5, 0.05, 0.005, 1e-9}) {
    p.eps = static_cast<float>(eps);
    std::mt19937 rng(7);
    double worst = 0.0;
    for (int k = 0; k < 300; ++k) {
      const dyn_state<double> s = sliding_state(rng);
      const diff::dyn_ph_rates<double> r = diff::dyn_ph_rates_at(p, s, 0.0);
      worst = std::max(worst, std::fabs(r.floor) / (r.dissipated + std::fabs(r.supplied) + 1.0));
    }
    std::printf("  speed floor %.0e m/s: worst |floor power| / (dissipated + |supplied|) = %.2e\n",
                eps, worst);
    EXPECT_LT(worst, prev);
    prev = worst;
  }
  EXPECT_LT(prev, 1e-7);
}

// The report: the tires' tangent (dyn_ph_tangent_D), the linearization's damping, along steer
// ramps to the lock and swerves at several speeds, binned by lateral g like the plant's drift
// (D14). Below the peak it is positive semi-definite; past the Pacejka peak a tire's force falls
// as its slip grows and the tangent loses definiteness, so the motion can be locally unstable
// there. The secant, the energy's dissipation, stays positive semi-definite in every sample. With
// the tires fitted to Jolt's (B C = kJoltTireStiffness / tire_peak_slip_angle) the Humvee
// understeers: its front tire passes the peak first, from about half its grip, and the ramps
// plateau short of mu g.
TEST(DynBikePH, TheSecantStaysPSDWhereTheTangentDoesNot) {
  for (const double mu : {0.9, 0.6}) {
    const dyn_bike_params p = humvee(mu);
    bike_veh lim = plant_limits(p);
    lim.hdt = 0.01f;
    const double edges[] = {0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.8, 0.9, 1e9};
    const int nb = 8;
    std::vector<int> count(nb, 0), indef(nb, 0), front(nb, 0), rear(nb, 0);
    std::vector<double> lam_min(nb, 1e300);
    int secant_negative = 0;
    const auto record = [&](const dyn_state<double> &s) {
      const double g = std::fabs(lateral_g(p, s));
      int b = 0;
      while (b + 1 < nb && g >= edges[b + 1])
        ++b;
      double Dt[2][2], kf, kr, af, ar;
      diff::dyn_ph_tangent_D(p, s, Dt, kf, kr, af, ar);
      const double tr = Dt[0][0] + Dt[1][1], det = Dt[0][0] * Dt[1][1] - Dt[0][1] * Dt[1][0];
      const double lmin = 0.5 * tr - std::sqrt(std::max(0.0, 0.25 * tr * tr - det));
      ++count[b];
      indef[b] += lmin < 0.0;
      front[b] += kf < 0.0;
      rear[b] += kr < 0.0;
      lam_min[b] = std::min(lam_min[b], lmin / (p.m / 1.0));
      diff::dyn_ph_parts<double> q;
      diff::dyn_ph_parts_at(p, s, q);
      secant_negative += q.Sf / q.cd < 0.0 || q.Sr < 0.0;
    };
    for (const double v : {8.0, 12.0, 16.0, 20.0}) {
      // A steady ramp of the steer to the lock, the speed held by the throttle: past the limit.
      dyn_state<double> s = at_rest<double>(v);
      const int n = static_cast<int>(25.0 / lim.hdt);
      for (int t = 0; t < n; ++t) {
        const double dcmd = lim.delta_max * t / double(n);
        const double acmd = 2.0 * (v - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
        dyn_state<double> o;
        diff::dyn_plant<double>(p, lim, s, dcmd, acmd, 0.0, o);
        s = o;
        record(s);
      }
      // A swerve: a 0.2 rad steer sine at 0.6 Hz, coasting.
      s = at_rest<double>(v);
      for (int t = 0; t < static_cast<int>(6.0 / lim.hdt); ++t) {
        const double dcmd = 0.2 * std::sin(kTwoPi * 0.6 * t * lim.hdt);
        dyn_state<double> o;
        diff::dyn_plant<double>(p, lim, s, dcmd, 0.0, 0.0, o);
        s = o;
        record(s);
      }
    }
    std::printf("  tangent by lateral g (Humvee, mu %.1f):\n", mu);
    std::printf("    %-10s %8s %10s %8s %8s %14s\n", "g", "samples", "indefinite", "front", "rear",
                "min lambda/m");
    for (int b = 0; b < nb; ++b) {
      if (!count[b])
        continue;
      char lab[32];
      if (edges[b + 1] > 1e8)
        std::snprintf(lab, sizeof lab, ">= %.1f", edges[b]);
      else
        std::snprintf(lab, sizeof lab, "%.1f-%.1f", edges[b], edges[b + 1]);
      std::printf("    %-10s %8d %9.1f%% %7.1f%% %7.1f%% %14.3g\n", lab, count[b],
                  100.0 * indef[b] / count[b], 100.0 * front[b] / count[b],
                  100.0 * rear[b] / count[b], lam_min[b]);
      // Below half the grip both tires are short of their peak: PSD everywhere.
      if (edges[b + 1] <= 0.5 * mu + 1e-9)
        EXPECT_EQ(indef[b], 0) << "mu " << mu << ", bin " << lab;
      // Understeer: the rear tire never passes its peak.
      EXPECT_EQ(rear[b], 0) << "mu " << mu << ", bin " << lab;
    }
    // Above half the grip the ramps take the front tire past its peak: the tangent is indefinite
    // there.
    int top = 0;
    for (int b = 0; b < nb; ++b)
      if (edges[b] >= 0.5 * mu - 1e-9)
        top += indef[b];
    EXPECT_GT(top, 0) << "mu " << mu << ": the ramps never passed the peak";
    // The secant never is.
    EXPECT_EQ(secant_negative, 0) << "mu " << mu;
  }
}

// ── The external-force port: the deployed drive's ext_force in the differentiable rollout ──

namespace {

// Agents in the city: the scene's starts and goals, headings near the goal direction, speeds
// spread over the range, and a force per agent.
struct ext_case {
  float ox, oy, th, sp, gx, gy, fx, fy;
};
//
// The starts are jittered off the scene's sampled cell centres, which are the SDF's grid
// nodes: the bilinear field has a kink there, which a central difference straddles.
// CVC_EXT_CASE_SEED replaces the seeds (the scene's starts too), for sweeping the checks over
// other draws, as another standard library's would be.
std::vector<ext_case> ext_cases(const city &c, int n) {
  const char *seed = std::getenv("CVC_EXT_CASE_SEED");
  std::vector<float> o(2 * n), g(2 * n);
  c.sc.sample_starts_goals(n, seed ? 100 + std::atoi(seed) : 5, o.data(), g.data());
  std::mt19937 rng(seed ? static_cast<unsigned>(std::atoi(seed)) : 9u);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  std::vector<ext_case> r;
  for (int i = 0; i < n; ++i) {
    ext_case e;
    e.ox = o[2 * i] + 0.04f * u(rng);
    e.oy = o[2 * i + 1] + 0.04f * u(rng);
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

// One finite-difference check summed over cases, given each case's loss as a function of the
// offset t along the checked direction. A case is skipped and counted, not compared, when its
// difference does not measure the slope:
//   - a jump: the forward and backward one-sided differences disagree (by more than 20% of the
//     larger and an absolute 1e-3), as where the step straddles one of the controller's
//     switches (a clamp, a governor, the behind branch);
//   - unresolved: the central differences at eps and eps / 2 disagree by more than 5% of the
//     larger and 1e-3, as where the slope is sharply peaked within the step (near a steep
//     barrier: 24.9 at 1e-3, 31.4 at 3e-4 and 31.3 at 1e-4, against the reverse's 31.2).
// Which cases do either depends on the scene's draws, which differ by standard library
// (sample_starts_goals' uniform_int_distribution): libc++'s draws failed the old unfiltered sums
// (dL/dF 0.38 on macOS) while MSVC's and libstdc++'s passed.
struct fd_sum {
  double num = 0.0, den = 0.0;
  int used = 0, kinked = 0;
  void add(double an, const std::function<double(double)> &f, double eps) {
    const double l0 = f(0.0), lp = f(eps), lm = f(-eps), lp2 = f(0.5 * eps), lm2 = f(-0.5 * eps);
    const double fwd = (lp - l0) / eps, bwd = (l0 - lm) / eps;
    const double c1 = 0.5 * (fwd + bwd), c2 = (lp2 - lm2) / eps;
    const auto big = [](double a, double b) { return std::max(std::fabs(a), std::fabs(b)); };
    if (std::fabs(fwd - bwd) > 0.2 * big(fwd, bwd) + 1e-3 ||
        std::fabs(c1 - c2) > 0.05 * big(c1, c2) + 1e-3) {
      ++kinked;
      return;
    }
    num += std::fabs(an - c1);
    den += std::fabs(c1);
    ++used;
  }
  double rel() const { return den > 0.0 ? num / den : 0.0; }
};

// rel < tol over the cases used, with at most a quarter skipped.
void expect_fd(const fd_sum &f, double tol, const std::string &what) {
  EXPECT_GT(f.den, 0.0) << what << ": no gradient to compare";
  EXPECT_LT(f.rel(), tol) << what;
  EXPECT_LE(f.kinked, (f.used + f.kinked) / 4) << what << ": too many cases on a switch";
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
  fd_sum sf, so;
  for (const ext_case &e : ext_cases(*c, 48)) {
    diff::ext_in x;
    x.fx = e.fx;
    x.fy = e.fy;
    float g[7], gf[2];
    diff::bike_step_bwd(c->F, e.ox, e.oy, e.th, e.sp, e.gx, e.gy, al, be, ga, c->v, w[0], w[1],
                        w[2], w[3], g[0], g[1], g[2], g[3], g[4], g[5], g[6], &x, gf);
    sf.add(
        gf[0] * ux + gf[1] * uy,
        [&](double t) { return loss(e, e.fx + (float)t * ux, e.fy + (float)t * uy, e.ox, e.oy); },
        eps);
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
    so.add(
        g[0] * ux + g[1] * uy,
        [&](double t) { return lo(e.ox + (float)t * ux, e.oy + (float)t * uy); }, eps);
  }

  // The dynamic step: dL/dF_ext against central differences.
  const dyn_bike_params p =
      cvc::physics::nav_plant::dyn_bike_params_at(cvc::physics::vehicle_desc::roboracer(), c->v.L);
  dyn_state<float> wd{};
  for (int k = 0; k < dyn_state<float>::size; ++k)
    wd[k] = 0.1f * static_cast<float>(k + 1) * (k % 2 ? -1.0f : 1.0f);
  fd_sum sd;
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
    sd.add(
        gf[0] * ux + gf[1] * uy,
        [&](double t) { return dl(e.fx + (float)t * ux, e.fy + (float)t * uy); }, eps);
  }
  std::printf("  dL/dF_ext vs central differences: kinematic %.2e, dynamic %.2e; through a "
              "position-dependent force %.2e (cases on a switch: %d, %d, %d)\n",
              sf.rel(), sd.rel(), so.rel(), sf.kinked, sd.kinked, so.kinked);
  expect_fd(sf, 5e-2, "kinematic dL/dF_ext");
  expect_fd(sd, 5e-2, "dynamic dL/dF_ext");
  expect_fd(so, 5e-2, "through a position-dependent force");
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
    fd_sum chk[3];
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
      chk[0].add(
          g_init[0] * ux + g_init[1] * uy,
          [&](double t) { return loss(e.ox + (float)t * ux, e.oy + (float)t * uy, 0, 0, coef); },
          eps);
      // dL/dc: the sum of dL/dF_ext over the steps.
      double gc = 0.0;
      for (int t = 0; t < T; ++t)
        gc += g_fext[2 * t] * ux + g_fext[2 * t + 1] * uy;
      chk[1].add(
          gc, [&](double t) { return loss(e.ox, e.oy, (float)t * ux, (float)t * uy, coef); }, eps);
      // dL/dbeta at one step.
      chk[2].add(
          g_coef[3 * k_beta + 1],
          [&](double t) {
            std::vector<float> cf = coef;
            cf[3 * k_beta + 1] += (float)t;
            return loss(e.ox, e.oy, 0, 0, cf);
          },
          1e-2); // beta's effect at one step is small: at 1e-3, float noise is a fair share
    }
    std::printf("  hold %d, %d steps: dL/dinit %.2e, dL/dF %.2e, dL/dbeta %.2e (relative to "
                "central differences; cases on a switch %d, %d, %d)\n",
                hold, T, chk[0].rel(), chk[1].rel(), chk[2].rel(), chk[0].kinked, chk[1].kinked,
                chk[2].kinked);
    for (int k = 0; k < 3; ++k)
      expect_fd(chk[k], 5e-2, "check " + std::to_string(k) + ", hold " + std::to_string(hold));
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
    fd_sum chk[2];
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
      chk[0].add(
          g_init.x * ux + g_init.y * uy,
          [&](double t) { return run(e.ox + (float)t * ux, e.oy + (float)t * uy, 0, 0, tp); }, eps);
      double gc = 0.0;
      for (int t = 0; t < T; ++t)
        gc += g_fext[2 * t] * ux + g_fext[2 * t + 1] * uy;
      chk[1].add(
          gc, [&](double t) { return run(e.ox, e.oy, (float)t * ux, (float)t * uy, tp); }, eps);
    }
    std::printf("  dynamic, hold %d, %d steps: dL/dinit %.2e, dL/dF %.2e (relative to central "
                "differences; cases on a switch %d, %d)\n",
                hold, T, chk[0].rel(), chk[1].rel(), chk[0].kinked, chk[1].kinked);
    for (int k = 0; k < 2; ++k)
      expect_fd(chk[k], 5e-2,
                "dynamic check " + std::to_string(k) + ", hold " + std::to_string(hold));
  }
}

// The torch twin's gate (GRL-SNAM): bike_rollout_ext and its reverse on fixed cases, with the
// field they sample, written to tests/data/ext_rollout_golden.json. Six agents, 12 steps each,
// per-step goals and coefficients, an affine force per agent (F = A o + b, J = A) held for 1 or
// 3 steps, one agent whose force does not steer; the loss is L = sum_t w_t . state_t. The torch
// twin replays it in float64 and checks this float reverse against autograd.
// CVC_EXT_ROLLOUT_GOLDEN_WRITE=1 rewrites the file; otherwise this build must reproduce it (the
// controller is float and uses libm, so to 1e-4, not bit for bit across platforms).
TEST(DiffExtRollout, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "ext_rollout_golden.json";
  const int R = 32, C = 32, T = 12;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(R * C), 0);
  for (int r = 13; r <= 18; ++r)
    for (int c = 12; c <= 19; ++c)
      occ[static_cast<std::size_t>(r * C + c)] = 1;
  const cvc::nav::training_scene sc =
      cvc::nav::occupancy_scene(occ.data(), R, C, -100.0, -100.0, 100.0, 100.0, 0.05);
  const cvc::nav::field_stack fst = sc.field();
  diff::field F;
  F.data = fst.data;
  F.H = fst.H;
  F.W = fst.W;
  F.S = static_cast<float>(fst.S);
  F.cx = static_cast<float>(fst.cx);
  F.cy = static_cast<float>(fst.cy);
  F.mnx = static_cast<float>(fst.mnx);
  F.mny = static_cast<float>(fst.mny);
  F.mxx = static_cast<float>(fst.mxx);
  F.mxy = static_cast<float>(fst.mxy);
  const cvc::nav::train_config tc;
  bike_veh v{};
  v.rr = sc.rr;
  v.d_hat = sc.d_hat;
  v.vmax = sc.vmax;
  v.L = tc.veh_L;
  v.delta_max = tc.veh_delta_max;
  v.a_max = tc.veh_a_max;
  v.a_lat_max = tc.veh_a_lat_max;
  v.k_steer = tc.veh_k_steer;
  v.hdt = sc.dt;
  v.allow_reverse = tc.veh_allow_reverse ? 1 : 0;

  struct gcase {
    float s0[4], g0[2], A[4], b[2];
    int hold, steer;
  };
  const gcase cases[] = {
      {{-2.5f, -0.5f, 0.0f, 0.3f}, {2.5f, 0.3f}, {0.2f, -0.1f, 0.05f, 0.15f}, {0.1f, 0.2f}, 1, 1},
      {{-2.4f, 0.6f, 0.2f, 0.6f}, {2.4f, -0.4f}, {-0.3f, 0.2f, 0.1f, -0.2f}, {0.0f, -0.3f}, 3, 1},
      {{1.8f, -2.2f, 1.4f, 0.1f}, {-0.5f, 2.6f}, {0.4f, 0.0f, 0.0f, 0.4f}, {-0.2f, 0.1f}, 1, 1},
      {{0.0f, 2.0f, -1.6f, 0.8f}, {0.3f, -2.5f}, {0.1f, 0.3f, -0.3f, 0.1f}, {0.3f, 0.0f}, 3, 1},
      {{-1.5f, 1.4f, -0.4f, 0.5f}, {1.6f, 0.9f}, {0.25f, 0.1f, -0.1f, 0.25f}, {-0.1f, -0.1f}, 1, 1},
      {{2.2f, 0.4f, 3.0f, 0.4f}, {-2.6f, -0.2f}, {0.15f, -0.2f, 0.2f, 0.15f}, {0.2f, 0.1f}, 3, 0},
  };
  // The loss weights, the same rule in the file and in the twin.
  std::vector<float> w(static_cast<std::size_t>(4 * (T + 1)));
  for (int t = 0; t <= T; ++t)
    for (int k = 0; k < 4; ++k)
      w[4 * t + k] = 0.1f * static_cast<float>((3 * t + k) % 7 - 3);

  struct ctx {
    float A[4], b[2];
  };
  const auto sample = [](void *user, int, float ox, float oy, float *f, float *J) {
    const ctx &c = *static_cast<const ctx *>(user);
    f[0] = c.A[0] * ox + c.A[1] * oy + c.b[0];
    f[1] = c.A[2] * ox + c.A[3] * oy + c.b[1];
    for (int k = 0; k < 4; ++k)
      J[k] = c.A[k];
  };
  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.9g", x);
    return std::string(b);
  };
  const auto arr = [&](const float *x, std::size_t n) {
    std::string out = "[";
    for (std::size_t k = 0; k < n; ++k)
      out += (k ? ", " : "") + num(x[k]);
    return out + "]";
  };
  std::vector<double> now;
  std::ostringstream js;
  js << "{\n \"format\": \"cvc-ext-rollout-golden-1\",\n";
  js << " \"note\": \"diff::bike_rollout_ext (float) on fixed cases; L = sum_t w[t] . "
        "(ox, oy, th, sp)_t\",\n";
  const std::size_t HW = static_cast<std::size_t>(fst.H) * fst.W;
  js << " \"field\": {\"H\": " << fst.H << ", \"W\": " << fst.W << ", \"bounds\": [" << num(fst.mnx)
     << ", " << num(fst.mny) << ", " << num(fst.mxx) << ", " << num(fst.mxy) << "], \"center\": ["
     << num(fst.cx) << ", " << num(fst.cy) << "], \"scale\": " << num(fst.S)
     << ",\n  \"phi\": " << arr(fst.data, HW) << ",\n  \"nx\": " << arr(fst.data + HW, HW)
     << ",\n  \"ny\": " << arr(fst.data + 2 * HW, HW) << "},\n";
  js << " \"veh\": {\"rr\": " << num(v.rr) << ", \"d_hat\": " << num(v.d_hat)
     << ", \"vmax\": " << num(v.vmax) << ", \"L\": " << num(v.L)
     << ", \"delta_max\": " << num(v.delta_max) << ", \"a_max\": " << num(v.a_max)
     << ", \"a_lat_max\": " << num(v.a_lat_max) << ", \"k_steer\": " << num(v.k_steer)
     << ", \"hdt\": " << num(v.hdt) << ", \"allow_reverse\": " << v.allow_reverse << "},\n";
  js << " \"T\": " << T << ",\n \"w\": " << arr(w.data(), w.size()) << ",\n \"cases\": [\n";
  for (std::size_t ci = 0; ci < sizeof cases / sizeof cases[0]; ++ci) {
    const gcase &gc = cases[ci];
    std::vector<float> goal(2 * T), coef(3 * T);
    for (int t = 0; t < T; ++t) {
      goal[2 * t] = gc.g0[0] + 0.01f * static_cast<float>(t);
      goal[2 * t + 1] = gc.g0[1] - 0.005f * static_cast<float>(t);
      coef[3 * t] = 0.6f + 0.01f * static_cast<float>(t);
      coef[3 * t + 1] = 1.8f - 0.02f * static_cast<float>(t);
      coef[3 * t + 2] = 0.9f + 0.005f * static_cast<float>(t);
    }
    ctx cx;
    for (int k = 0; k < 4; ++k)
      cx.A[k] = gc.A[k];
    cx.b[0] = gc.b[0];
    cx.b[1] = gc.b[1];
    diff::bike_ext_fn fn;
    fn.sample = sample;
    fn.user = &cx;
    fn.hold = gc.hold;
    fn.steer = gc.steer;
    diff::bike_tape tape;
    diff::bike_rollout_ext(F, v, gc.s0[0], gc.s0[1], gc.s0[2], gc.s0[3], goal.data(), coef.data(),
                           T, fn, tape);
    std::vector<float> g_fext(2 * T), g_coef(3 * T);
    float g_init[4];
    diff::bike_rollout_ext_bwd(F, v, tape, goal.data(), coef.data(), w.data(), g_fext.data(),
                               g_coef.data(), g_init);
    for (const float x : tape.state)
      now.push_back(x);
    for (const float x : g_fext)
      now.push_back(x);
    for (const float x : g_coef)
      now.push_back(x);
    for (const float x : g_init)
      now.push_back(x);
    js << "  {\"state0\": " << arr(gc.s0, 4) << ", \"A\": " << arr(gc.A, 4)
       << ", \"b\": " << arr(gc.b, 2) << ", \"hold\": " << gc.hold << ", \"steer\": " << gc.steer
       << ",\n   \"goals\": " << arr(goal.data(), goal.size())
       << ",\n   \"coefs\": " << arr(coef.data(), coef.size())
       << ",\n   \"states\": " << arr(tape.state.data(), tape.state.size())
       << ",\n   \"g_fext\": " << arr(g_fext.data(), g_fext.size())
       << ",\n   \"g_coef\": " << arr(g_coef.data(), g_coef.size())
       << ",\n   \"g_init\": " << arr(g_init, 4) << "}"
       << (ci + 1 < sizeof cases / sizeof cases[0] ? "," : "") << "\n";
  }
  js << " ]\n}\n";
  const char *write = std::getenv("CVC_EXT_ROLLOUT_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &c : want.get_child("cases"))
    for (const char *key : {"states", "g_fext", "g_coef", "g_init"})
      for (const auto &x : c.second.get_child(key))
        golden.push_back(x.second.get_value<double>());
  ASSERT_EQ(golden.size(), now.size());
  for (std::size_t k = 0; k < now.size(); ++k)
    EXPECT_LE(std::fabs(now[k] - golden[k]),
              1e-4 * std::max(1.0, std::max(std::fabs(now[k]), std::fabs(golden[k]))))
        << "value " << k << ": golden " << golden[k] << ", now " << now[k];
}
