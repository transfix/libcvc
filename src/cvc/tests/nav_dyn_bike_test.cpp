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
