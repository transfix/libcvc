/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// P8d's gates for diff::dyn4_plant, the two-track vehicle with suspension (nav/detail/dyn_4w.h).
// The Humvee in metres (S = 1): P8a's single-track numbers from its vehicle_desc, plus its tracks,
// its corners' springs and dampers (suspension frequency and damping ratio) and the chassis box's
// roll and pitch inertia.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_4w.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/ext_rollout.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace diff = cvc::nav::diff;
using diff::bike_veh;
using diff::dyn4_params;
using diff::dyn4_state;
using diff::dyn_bike_params;
using diff::dyn_state;

namespace {

constexpr double kG = 9.80665;

dyn_bike_params humvee1() {
  const cvc::nav::dyn_bike_si si =
      cvc::physics::nav_plant::dyn_bike_si_of(cvc::physics::vehicle_desc::humvee());
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn_bike_params_of(si, f);
}

dyn4_params humvee4() {
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn4_params_of(
      cvc::physics::nav_plant::dyn4_si_of(cvc::physics::vehicle_desc::humvee()), f);
}

bike_veh limits(float L) {
  bike_veh v{};
  v.L = L;
  v.vmax = 40.0f;
  v.delta_max = 0.6f;
  v.hdt = 0.06f;
  v.allow_reverse = 0;
  return v;
}

// Flat ground whose friction is mu_left on the vehicle's left of y = 0 and mu_right on its right,
// blended over a few centimetres (so the wheels sit cleanly on one side or the other).
struct split_ground {
  double mu_left = 0.3, mu_right = 0.9;
  template <class S> void at(S, S wy, S &mu, S &height) const {
    const S t = S(0.5) + S(0.5) * diff::s_tanh(wy / S(0.05));
    mu = S(mu_right) + t * S(mu_left - mu_right);
    height = S(0);
  }
};

// Flat ground with a round bump: height A exp(-|w - c|^2 / r^2).
struct bump_ground {
  double mu = 0.9, cx = 0.0, cy = 0.0, A = 0.08, r = 0.4;
  template <class S> void at(S wx, S wy, S &mu_out, S &height) const {
    const S dx = wx - S(cx), dy = wy - S(cy);
    mu_out = S(mu);
    height = S(A) * diff::s_exp(-(dx * dx + dy * dy) / S(r * r));
  }
};

template <class G>
std::vector<dyn4_state<double>> drive4(const dyn4_params &p, const bike_veh &v, const G &ground,
                                       dyn4_state<double> s, int n, double (*dcmd)(int),
                                       double (*acmd)(int)) {
  std::vector<dyn4_state<double>> out{s};
  for (int t = 0; t < n; ++t) {
    dyn4_state<double> o;
    diff::dyn4_plant<double>(p, v, ground, s, dcmd(t), acmd(t), o);
    s = o;
    out.push_back(s);
  }
  return out;
}

dyn4_state<double> rolling(double vx) {
  dyn4_state<double> s{};
  s.vx = vx;
  return s;
}

} // namespace

// The Humvee's four-wheel numbers come from its vehicle_desc: P8a's single-track ones, the axles'
// half tracks, the chassis box's inertias, and each corner's spring and damper from its axle's
// suspension frequency and damping ratio at the corner's static load.
TEST(Dyn4, HumveeParametersFromItsSpecSheet) {
  const cvc::physics::vehicle_desc d = cvc::physics::vehicle_desc::humvee();
  const cvc::nav::dyn4_si si = cvc::physics::nav_plant::dyn4_si_of(d);
  const dyn4_params p = humvee4();
  const dyn_bike_params b = humvee1();
  EXPECT_FLOAT_EQ(p.m, b.m);
  EXPECT_FLOAT_EQ(p.lf, b.lf);
  EXPECT_FLOAT_EQ(p.lr, b.lr);
  EXPECT_FLOAT_EQ(p.h, b.h);
  EXPECT_FLOAT_EQ(p.B_f, b.B_f);
  double f_front = 0.0, z_front = 0.0;
  for (const cvc::physics::axle_desc &a : d.axles)
    if (a.steered) {
      EXPECT_DOUBLE_EQ(si.half_track_front, a.half_track);
      f_front = a.suspension_frequency;
      z_front = a.suspension_damping;
    }
  // The front corner's natural frequency and damping ratio, back from its spring and damper.
  const double mc =
      0.5 * d.mass * si.single.cg_to_rear / (si.single.cg_to_front + si.single.cg_to_rear);
  EXPECT_NEAR(std::sqrt(si.spring_front / mc) / (2.0 * 3.14159265358979), f_front, 1e-9);
  EXPECT_NEAR(si.damper_front / (2.0 * std::sqrt(si.spring_front * mc)), z_front, 1e-9);
  EXPECT_GT(si.roll_inertia, 0.0);
  EXPECT_GT(si.pitch_inertia, si.roll_inertia); // a long, narrow box
  EXPECT_GT(p.tf, 0.5f);
  EXPECT_GT(p.tr, 0.5f);
}

// At rest on flat ground with no commands it stays where it is, its loads the static ones.
TEST(Dyn4, RestsOnFlatGround) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  const diff::flat_ground ground;
  dyn4_state<double> s{};
  for (int t = 0; t < 200; ++t) {
    dyn4_state<double> o;
    diff::dyn4_plant<double>(p, v, ground, s, 0.0, 0.0, o);
    s = o;
  }
  EXPECT_NEAR(s.x, 0.0, 1e-9);
  EXPECT_NEAR(s.y, 0.0, 1e-9);
  EXPECT_NEAR(s.z, 0.0, 1e-6);
  EXPECT_NEAR(s.phi, 0.0, 1e-9);
  EXPECT_NEAR(s.th, 0.0, 1e-6);
  diff::dyn4_wheels<double> w;
  diff::dyn4_wheel_state(p, v, ground, s, w);
  const double total = w.Fz[0] + w.Fz[1] + w.Fz[2] + w.Fz[3];
  EXPECT_NEAR(total, p.m * p.g, 1e-3 * p.m * p.g);
}

// With no track and rigid corners on flat ground it is P8a's single-track vehicle: the same
// manoeuvre (steer, throttle, then brake from 15 m/s) through both plants.
TEST(Dyn4, SingleTrackLimitIsDynPlant) {
  dyn4_params p4 = humvee4();
  p4.tf = p4.tr = 0.0f;
  p4.k_f = p4.k_r = p4.c_f = p4.c_r = 0.0f; // rigid
  dyn_bike_params p1 = humvee1();
  p1.mu = 0.9f;
  const bike_veh v = limits(p4.lf + p4.lr);
  const diff::flat_ground ground; // mu 0.9
  dyn4_state<double> s4 = rolling(15.0);
  dyn_state<double> s1{};
  s1.vx = 15.0;
  double worst_pos = 0.0, worst_v = 0.0, path = 0.0;
  for (int t = 0; t < 150; ++t) {
    const double dc = 0.05, ac = t < 50 ? 0.5 : (t < 100 ? -3.0 : 0.0);
    dyn4_state<double> o4;
    dyn_state<double> o1;
    diff::dyn4_plant<double>(p4, v, ground, s4, dc, ac, o4);
    diff::dyn_plant<double>(p1, v, s1, dc, ac, 0.0, o1);
    path += std::hypot(o1.x - s1.x, o1.y - s1.y);
    s4 = o4;
    s1 = o1;
    worst_pos = std::max(worst_pos, std::hypot(s4.x - s1.x, s4.y - s1.y));
    worst_v = std::max({worst_v, std::fabs(s4.vx - s1.vx), std::fabs(s4.vy - s1.vy)});
  }
  std::printf("  single-track limit, 150 steps (%.1f m): positions within %.3f m, velocities "
              "within %.3f m/s\n",
              path, worst_pos, worst_v);
  EXPECT_LT(worst_pos, 0.01 * path);
  EXPECT_LT(worst_v, 0.2);
}

// Steady states balance their moments: in a steady turn the roll springs carry h x the lateral
// force (sum y_i Fz_i = -h FY, the outside wheels loaded), and in steady braking the front axle
// carries the quasi-static m (g lr - a h) / L.
TEST(Dyn4, SteadyLoadTransferBalances) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  const diff::flat_ground ground;
  // A steady left turn at 12 m/s, the speed held by a proportional throttle.
  dyn4_state<double> s = rolling(12.0);
  for (int t = 0; t < 400; ++t) {
    dyn4_state<double> o;
    const double a_cmd = 2.0 * (12.0 - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
    diff::dyn4_plant<double>(p, v, ground, s, 0.06, a_cmd, o);
    s = o;
  }
  diff::dyn4_wheels<double> w;
  diff::dyn4_wheel_state(p, v, ground, s, w);
  double roll = 0.0;
  for (int i = 0; i < 4; ++i)
    roll += w.yi[i] * w.Fz[i];
  const double FY = p.m * s.r * s.vx; // steady: m (vy' + r vx), vy' = 0
  std::printf("  steady turn: a_y %.2f m/s^2, roll %.4f rad, sum y Fz %.0f N m against -h FY "
              "%.0f; outside (right) loads %.0f / %.0f N, inside %.0f / %.0f\n",
              s.r * s.vx, s.phi, roll, -p.h * FY, w.Fz[1], w.Fz[3], w.Fz[0], w.Fz[2]);
  EXPECT_GT(s.r, 0.0);
  EXPECT_NEAR(roll, -p.h * FY, 0.03 * std::fabs(p.h * FY));
  EXPECT_GT(w.Fz[1], w.Fz[0]);
  EXPECT_GT(w.Fz[3], w.Fz[2]);

  // Steady braking at 3 m/s^2 (held until the pitch settles; the speed stays well above 0).
  const double a = -3.0;
  s = rolling(25.0);
  for (int t = 0; t < 60; ++t) {
    dyn4_state<double> o;
    diff::dyn4_plant<double>(p, v, ground, s, 0.0, a, o);
    s = o;
  }
  diff::dyn4_wheel_state(p, v, ground, s, w);
  const double L = p.lf + p.lr, front = w.Fz[0] + w.Fz[1];
  // The deceleration the tires actually deliver, with drag and rolling resistance.
  const double a_eff = s.a; // the actuator's force per unit mass, settled
  const double want = p.m * (p.g * p.lr - a_eff * p.h) / L;
  std::printf("  steady braking %.1f m/s^2: front axle %.0f N against quasi-static %.0f N "
              "(static %.0f), pitch %.4f rad\n",
              a_eff, front, want, p.m * p.g * p.lr / L, s.th);
  EXPECT_NEAR(front, want, 0.03 * want);
  EXPECT_GT(s.th, 0.0); // the nose dips
}

// Braking with the left wheels on ice (mu 0.3) and the right on dry ground (0.9): the right side
// brakes harder, so the vehicle yaws toward it (to the right, psi < 0).
TEST(Dyn4, MuSplitBrakingYawsTowardHighMu) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  const split_ground ground;
  dyn4_state<double> s = rolling(15.0);
  for (int t = 0; t < 30; ++t) {
    dyn4_state<double> o;
    diff::dyn4_plant<double>(p, v, ground, s, 0.0, -7.0, o);
    s = o;
  }
  std::printf("  mu-split braking, 1.8 s from 15 m/s: heading %.4f rad, yaw rate %.4f rad/s, "
              "speed %.2f m/s\n",
              s.psi, s.r, s.vx);
  EXPECT_LT(s.psi, -0.005);
}

// A bump under the left wheels' path: the front left's load rises above its static load as it
// passes, more than the front right's.
TEST(Dyn4, ABumpLoadsItsWheel) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  bump_ground ground;
  ground.cx = 5.0;
  ground.cy = p.tf;
  dyn4_state<double> s = rolling(5.0);
  s.x = -10.0;
  double fl_max = 0.0, fr_max = 0.0, fz0 = 0.0;
  for (int t = 0; t < 120; ++t) {
    diff::dyn4_wheels<double> w;
    diff::dyn4_wheel_state(p, v, ground, s, w);
    fz0 = w.Fz0[0];
    fl_max = std::max(fl_max, w.Fz[0]);
    fr_max = std::max(fr_max, w.Fz[1]);
    dyn4_state<double> o;
    const double a_cmd = 1.0 * (5.0 - s.vx);
    diff::dyn4_plant<double>(p, v, ground, s, 0.0, a_cmd, o);
    s = o;
  }
  std::printf("  bump (8 cm) under the left wheels: front left peaks at %.2f x its static load, "
              "front right at %.2f x\n",
              fl_max / fz0, fr_max / fz0);
  EXPECT_GT(fl_max, 1.2 * fz0);
  EXPECT_GT(fl_max - fz0, 2.0 * (fr_max - fz0));
}

// The plant's VJP (forward-mode duals) against central differences in double, in several regimes:
// cruising, cornering, braking in a turn, crawling, and rolling over a bump (the terrain's
// gradient flows through the ground sampler).
TEST(Dyn4, PlantGradientMatchesFiniteDifference) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  bump_ground bump;
  bump.cx = 1.0;
  bump.cy = 0.5;
  bump.A = 0.06;
  bump.r = 0.8;
  struct gcase {
    dyn4_state<double> s;
    double dc, ac;
    bool on_bump;
  };
  std::vector<gcase> cases;
  {
    gcase c{rolling(12.0), 0.02, 0.5, false};
    cases.push_back(c);
    c.s = rolling(14.0);
    c.s.vy = 0.4;
    c.s.r = 0.3;
    c.s.phi = 0.01;
    c.s.delta = 0.08;
    c.dc = 0.1;
    c.ac = 0.1; // not 0: a zero drive force is the kink where front_drive hands over to
                // front_brake, and a central difference across it is not a gradient
    cases.push_back(c);
    c.s.a = -2.0;
    c.ac = -4.0;
    c.s.th = 0.004;
    c.s.thd = 0.01;
    cases.push_back(c);
    c = gcase{rolling(0.6), 0.3, 0.4, false};
    c.s.delta = 0.2;
    cases.push_back(c);
    c = gcase{rolling(6.0), 0.05, 0.3, true};
    c.s.x = 0.0;
    c.s.y = 0.2;
    c.s.zd = 0.05;
    cases.push_back(c);
  }
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst = 0.0;
  for (const gcase &gc : cases) {
    dyn4_state<double> g_o;
    for (int k = 0; k < dyn4_state<double>::size; ++k)
      g_o[k] = u(rng);
    const auto loss = [&](const dyn4_state<double> &s, double dc, double ac) {
      dyn4_state<double> o;
      if (gc.on_bump)
        diff::dyn4_plant<double>(p, v, bump, s, dc, ac, o);
      else
        diff::dyn4_plant<double>(p, v, diff::flat_ground{}, s, dc, ac, o);
      double acc = 0.0;
      for (int k = 0; k < dyn4_state<double>::size; ++k)
        acc += g_o[k] * o[k];
      return acc;
    };
    dyn4_state<double> g_s;
    double g_dc = 0.0, g_ac = 0.0;
    if (gc.on_bump)
      diff::dyn4_plant_vjp<double>(p, v, bump, gc.s, gc.dc, gc.ac, g_o, g_s, g_dc, g_ac);
    else
      diff::dyn4_plant_vjp<double>(p, v, diff::flat_ground{}, gc.s, gc.dc, gc.ac, g_o, g_s, g_dc,
                                   g_ac);
    const int n = dyn4_state<double>::size;
    for (int j = 0; j < n + 2; ++j) {
      const double eps = 1e-6;
      dyn4_state<double> sp = gc.s, sm = gc.s;
      double dcp = gc.dc, dcm = gc.dc, acp = gc.ac, acm = gc.ac;
      if (j < n) {
        sp[j] += eps;
        sm[j] -= eps;
      } else if (j == n) {
        dcp += eps;
        dcm -= eps;
      } else {
        acp += eps;
        acm -= eps;
      }
      const double fd = (loss(sp, dcp, acp) - loss(sm, dcm, acm)) / (2.0 * eps);
      const double an = j < n ? g_s[j] : (j == n ? g_dc : g_ac);
      worst = std::max(worst, std::fabs(an - fd) / std::max(1.0, std::fabs(fd)));
    }
  }
  std::printf("  dyn4_plant VJP vs central differences, %zu regimes: worst relative %.2e\n",
              cases.size(), worst);
  EXPECT_LT(worst, 1e-5);
}

// The envelopes P8a meets, on four wheels: steady lateral acceleration never above mu g + 10%,
// and stopping within 10% of v^2 / (2 mu g).
TEST(Dyn4, EnvelopesOnFourWheels) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  for (const double mu : {0.4, 0.9}) {
    diff::flat_ground ground;
    ground.mu = static_cast<float>(mu);
    double peak = 0.0;
    for (const double delta : {0.05, 0.1, 0.2, 0.35}) {
      dyn4_state<double> s = rolling(15.0);
      // The body's lateral acceleration over the tail, vy' + r vx: once the tires slide the
      // vehicle drifts and r vx alone overstates it (as DynBike.SteadyLateralNeverExceedsMuG).
      double ay = 0.0, vy0 = 0.0;
      const int n = 200, tail = 17;
      for (int t = 0; t < n; ++t) {
        if (t == n - tail)
          vy0 = s.vy;
        dyn4_state<double> o;
        const double a_cmd = 2.0 * (15.0 - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
        diff::dyn4_plant<double>(p, v, ground, s, delta, a_cmd, o);
        s = o;
        if (t >= n - tail)
          ay += s.vx * s.r / tail;
      }
      peak = std::max(peak, std::fabs(ay + (s.vy - vy0) / (tail * v.hdt)));
    }
    std::printf("  mu %.1f: the steadiest lateral acceleration %.2f m/s^2 = %.2f mu g\n", mu, peak,
                peak / (mu * kG));
    EXPECT_LE(peak, 1.1 * mu * kG);

    dyn4_state<double> s = rolling(20.0);
    double dist = 0.0;
    for (int t = 0; t < 2000 && s.vx > 0.01; ++t) {
      dyn4_state<double> o;
      diff::dyn4_plant<double>(p, v, ground, s, 0.0, -30.0, o);
      dist += std::hypot(o.x - s.x, o.y - s.y);
      s = o;
    }
    const double want = 20.0 * 20.0 / (2.0 * mu * kG);
    std::printf("  mu %.1f: stopping from 20 m/s in %.1f m against v^2/(2 mu g) = %.1f m\n", mu,
                dist, want);
    EXPECT_NEAR(dist, want, 0.1 * want);
  }
}

// One step under the trained controller (dyn4_bike_step) and its backward, through bike_control,
// against central differences in float: a RoboRacer in the trainer's city (nav_plant::
// dyn4_params_at, the city's veh_L), from scattered starts toward their goals.
TEST(Dyn4, StepGradcheckThroughTheController) {
  const cvc::nav::training_scene sc = cvc::nav::city_scene(48);
  const cvc::nav::field_stack fs = sc.field();
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
  const dyn4_params p =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::roboracer(), tc.veh_L);
  const diff::flat_ground ground;
  const int n = 16;
  std::vector<float> o(2 * n), g(2 * n);
  sc.sample_starts_goals(n, 7, o.data(), g.data());
  const float al = 0.6f, be = 1.8f, ga = 0.9f, eps = 1e-3f;
  std::mt19937 rng(5);
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  double num = 0.0, den = 0.0;
  for (int i = 0; i < n; ++i) {
    dyn4_state<float> s{};
    // Off the SDF's grid nodes: the scene's starts sit exactly on them (cell centres), where the
    // bilinear field has a kink that a central difference straddles.
    s.x = o[2 * i] + 0.04f * u(rng);
    s.y = o[2 * i + 1] + 0.04f * u(rng);
    s.psi = std::atan2(g[2 * i + 1] - s.y, g[2 * i] - s.x) + 0.2f * u(rng);
    s.vx = 0.3f + 0.4f * (u(rng) + 1.0f);
    s.vy = 0.02f * u(rng);
    s.r = 0.3f * u(rng);
    dyn4_state<float> w, dir;
    for (int k = 0; k < dyn4_state<float>::size; ++k) {
      w[k] = u(rng);
      dir[k] = 0.0f;
    }
    // A direction in the planar state (the controller reads the pose and the speed).
    dir.x = u(rng);
    dir.y = u(rng);
    dir.psi = u(rng);
    dir.vx = u(rng);
    dir.vy = u(rng);
    dir.r = u(rng);
    const auto loss = [&](float t) {
      dyn4_state<float> st = s, out;
      for (int k = 0; k < dyn4_state<float>::size; ++k)
        st[k] += t * dir[k];
      diff::dyn4_bike_step(F, st, g[2 * i], g[2 * i + 1], al, be, ga, v, p, ground, out);
      double acc = 0.0;
      for (int k = 0; k < dyn4_state<float>::size; ++k)
        acc += static_cast<double>(w[k]) * out[k];
      return acc;
    };
    dyn4_state<float> gs{};
    float gal = 0, gbe = 0, gga = 0;
    diff::dyn4_bike_step_bwd(F, s, g[2 * i], g[2 * i + 1], al, be, ga, v, p, ground, w, gs, gal,
                             gbe, gga);
    double an = 0.0;
    for (int k = 0; k < dyn4_state<float>::size; ++k)
      an += static_cast<double>(gs[k]) * dir[k];
    const double fd = (loss(eps) - loss(-eps)) / (2.0 * eps);
    num += std::fabs(an - fd);
    den += std::fabs(fd);
  }
  std::printf("  dyn4_bike_step through the controller, %d agents: directional relative error "
              "%.2e\n",
              n, num / den);
  EXPECT_LT(num / den, 5e-2);
}

// The P2 envelopes Jolt's wheeled Humvee meets (physics_vehicle_test, measured 2026-10): its
// skidpad's peak lateral acceleration at mu 0.7 is 6.85 m/s^2, and it stops from 20 m/s at mu 0.8
// in 25.90 m. The four-wheel plant, on the same vehicle_desc, lands within 10% of both.
TEST(Dyn4, MatchesJoltsHumveeEnvelopes) {
  const dyn4_params p = humvee4();
  const bike_veh v = limits(p.lf + p.lr);
  diff::flat_ground ground;
  ground.mu = 0.7f;
  double peak = 0.0;
  for (const double delta : {0.05, 0.1, 0.2, 0.35}) {
    dyn4_state<double> s = rolling(15.0);
    double ay = 0.0;
    const int n = 200, tail = 17;
    for (int t = 0; t < n; ++t) {
      dyn4_state<double> o;
      const double a_cmd = 2.0 * (15.0 - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
      diff::dyn4_plant<double>(p, v, ground, s, delta, a_cmd, o);
      s = o;
      if (t >= n - tail)
        ay += std::fabs(s.vx * s.r) / tail;
    }
    peak = std::max(peak, ay);
  }
  ground.mu = 0.8f;
  dyn4_state<double> s = rolling(20.0);
  double dist = 0.0;
  for (int t = 0; t < 2000 && s.vx > 0.01; ++t) {
    dyn4_state<double> o;
    diff::dyn4_plant<double>(p, v, ground, s, 0.0, -30.0, o);
    dist += std::hypot(o.x - s.x, o.y - s.y);
    s = o;
  }
  std::printf("  against Jolt's Humvee: skidpad peak %.2f m/s^2 at mu 0.7 (Jolt 6.85); stopping "
              "from 20 m/s %.2f m at mu 0.8 (Jolt 25.90)\n",
              peak, dist);
  EXPECT_NEAR(peak, 6.85, 0.1 * 6.85);
  EXPECT_NEAR(dist, 25.90, 0.1 * 25.90);
}

// The P2 Ackermann gate, as Jolt's Humvee meets it (physics_vehicle_test): each front wheel takes
// its Ackermann angle, cot(right) - cot(left) = track / wheelbase on a left turn. And the
// vehicle turns on that geometry: slowly, held at delta, the rear axle's circle is the kinematic
// L / tan(delta) to 1%, from the tires alone (the low-speed blend off).
TEST(Dyn4, TheFrontWheelsFollowAckermann) {
  const dyn4_params p = humvee4();
  const double L = double(p.lf) + p.lr, tf = p.tf;
  EXPECT_NEAR(L, 3.30, 1e-6);
  EXPECT_NEAR(2.0 * tf, 1.82, 1e-6);
  const auto cot = [](double a) { return std::cos(a) / std::sin(a); };
  for (const double delta : {0.05, 0.2, 0.45, -0.3}) {
    const double t = std::tan(delta);
    const double left = diff::dyn4_ackermann(L, tf, t), right = diff::dyn4_ackermann(L, -tf, t);
    EXPECT_NEAR(left, std::atan(L * t / (L - tf * t)), 1e-12);
    EXPECT_NEAR(cot(right) - cot(left), 2.0 * tf / L, 1e-12) << delta;
    EXPECT_GT(std::fabs(delta > 0 ? left : right), std::fabs(delta))
        << "the inner wheel steers more";
  }
  dyn4_params tires = p;
  tires.v_blend = 0.0f;
  const bike_veh v = limits(p.lf + p.lr);
  const diff::flat_ground ground;
  for (const double delta : {0.15, 0.3, 0.45}) {
    dyn4_state<double> s = rolling(3.0);
    for (int t = 0; t < 400; ++t) {
      dyn4_state<double> o;
      const double a_cmd = 2.0 * (3.0 - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
      diff::dyn4_plant<double>(tires, v, ground, s, delta, a_cmd, o);
      s = o;
    }
    const double rear_speed = std::hypot(s.vx, s.vy - p.lr * s.r);
    const double R = rear_speed / s.r, want = L / std::tan(delta);
    std::printf("  delta %.2f at %.1f m/s: the rear axle's circle %.3f m, L / tan(delta) %.3f m "
                "(%+.2f%%)\n",
                delta, s.vx, R, want, 100.0 * (R / want - 1.0));
    EXPECT_NEAR(R, want, 0.01 * want);
  }
}

namespace {

// The golden's ground: the friction split along y = 0 (left mu_left, right mu_right, blended
// over split_width) and a round bump, so the twin is held to the ground's gradients too.
struct golden_ground {
  double mu_left = 0.5, mu_right = 0.9, split_width = 0.3;
  double A = 0.06, cx = 4.0, cy = 3.0, r = 0.8;
  template <class S> void at(S wx, S wy, S &mu, S &height) const {
    const S t = S(0.5) + S(0.5) * diff::s_tanh(wy / S(split_width));
    mu = S(mu_right) + t * S(mu_left - mu_right);
    const S dx = wx - S(cx), dy = wy - S(cy);
    height = S(A) * diff::s_exp(-(dx * dx + dy * dy) / S(r * r));
  }
};

} // namespace

// The C++ side of the parity gate for GRL-SNAM's torch twin: the values it is held to. The plant,
// in double, on fixed cases on golden_ground: the Humvee with its springs (and lags and the
// low-speed blend), and again with rigid corners. Each case's next state and its VJP for a fixed
// output gradient. CVC_DYN4_GOLDEN_WRITE=1 rewrites tests/data/dyn4_golden.json; otherwise this
// build must reproduce it to 1e-9.
TEST(Dyn4, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn4_golden.json";
  dyn4_params sprung = humvee4();
  sprung.tau_steer = 0.05f;
  sprung.v_blend = 1.0f;
  sprung.v_width = 0.3f;
  dyn4_params rigid = sprung;
  rigid.k_f = rigid.k_r = rigid.c_f = rigid.c_r = 0.0f;
  const dyn4_params psets[2] = {sprung, rigid};
  const golden_ground ground;
  struct gcase {
    int params;
    double s[14], dcmd, acmd;
    bool reverse;
  };
  // x, y, psi, vx, vy, r, z, zd, phi, phid, th, thd, delta, a
  const gcase cases[] = {
      {0, {0, 0, 0, 12, 0.1, 0.05, 0, 0, 0, 0, 0, 0, 0.02, 0.5}, 0.05, 1.0, false},
      {0,
       {1, -2, 0.3, 15, -0.6, 0.5, 0.01, -0.02, 0.02, 0.05, -0.01, 0.03, 0.2, 0},
       0.25,
       0.0,
       false},
      {0,
       {-3, 4, -1.1, 14, 0.4, 0.2, -0.005, 0.01, -0.01, 0.0, 0.02, -0.05, 0.1, -4},
       0.1,
       -6.0,
       false},
      {0, {0.7, 3.5, 0.05, 8, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0.05, 0}, 0.1, 0.5, false},
      {0, {-5, 0.1, 0.02, 18, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, -3}, 0.0, -8.0, false},
      {0, {0, 0, 0, 0.6, 0.02, 0.05, 0, 0, 0, 0, 0, 0, 0.3, 0.3}, 0.35, 0.5, false},
      {0, {0.5, 0.5, 0.2, -2, 0.05, -0.1, 0, 0, 0.01, 0, 0, 0, 0.2, -0.5}, 0.25, -0.8, true},
      {0, {2, 2, 2.5, 10, 2.5, 1.1, 0.02, 0.1, 0.04, -0.1, 0, 0.05, 0.4, 1}, 0.45, 2.0, false},
      {0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0.1, 2.0, false},
      {1, {1, -2, 0.3, 15, -0.6, 0.5, 0, 0, 0, 0, 0, 0, 0.2, 0}, 0.25, 0.0, false},
      {1, {0.7, 3.5, 0.05, 8, 0.1, 0.1, 0, 0, 0, 0, 0, 0, 0.05, -2}, 0.1, -4.0, false},
  };
  const int n = dyn4_state<double>::size;
  const double g_out[14] = {0.7,  -0.3, 1.1, 0.4,  -0.9, 0.25, 0.8,
                            -0.2, 0.5,  0.3, -0.6, 0.15, 0.6,  -0.15};
  // Every value this build computes, in the file's order: per case, next[14], g_state[14],
  // g_delta_cmd, g_a_cmd.
  std::vector<double> now;
  std::ostringstream js;
  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", x);
    return std::string(b);
  };
  const auto arr = [&](const double *v, int k) {
    std::string out = "[";
    for (int i = 0; i < k; ++i)
      out += (i ? ", " : "") + num(v[i]);
    return out + "]";
  };
  js << "{\n \"format\": \"cvc-dyn4-golden-1\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"z\", \"zd\", \"phi\", "
        "\"phid\", \"th\", \"thd\", \"delta\", \"a\"],\n";
  js << " \"params\": [\n";
  for (int k = 0; k < 2; ++k) {
    const dyn4_params &p = psets[k];
    const std::pair<const char *, float> params[] = {{"m", p.m},
                                                     {"Iz", p.Iz},
                                                     {"Ix", p.Ix},
                                                     {"Iy", p.Iy},
                                                     {"lf", p.lf},
                                                     {"lr", p.lr},
                                                     {"tf", p.tf},
                                                     {"tr", p.tr},
                                                     {"h", p.h},
                                                     {"g", p.g},
                                                     {"k_f", p.k_f},
                                                     {"k_r", p.k_r},
                                                     {"c_f", p.c_f},
                                                     {"c_r", p.c_r},
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
    js << "  {";
    bool first = true;
    for (const auto &kv : params) {
      js << (first ? "" : ", ") << "\"" << kv.first << "\": " << num(kv.second);
      first = false;
    }
    js << ", \"iters\": " << p.iters << "}" << (k ? "" : ",") << "\n";
  }
  js << " ],\n \"ground\": {\"mu_left\": " << num(ground.mu_left)
     << ", \"mu_right\": " << num(ground.mu_right)
     << ", \"split_width\": " << num(ground.split_width) << ", \"A\": " << num(ground.A)
     << ", \"cx\": " << num(ground.cx) << ", \"cy\": " << num(ground.cy)
     << ", \"r\": " << num(ground.r) << "},\n";
  js << " \"g_out\": " << arr(g_out, n) << ",\n \"cases\": [\n";
  const std::size_t ncase = sizeof cases / sizeof cases[0];
  for (std::size_t c = 0; c < ncase; ++c) {
    const gcase &gc = cases[c];
    const dyn4_params &p = psets[gc.params];
    bike_veh lim = limits(p.lf + p.lr);
    lim.allow_reverse = gc.reverse ? 1 : 0;
    dyn4_state<double> s, o, go, gs;
    for (int k = 0; k < n; ++k) {
      s[k] = gc.s[k];
      go[k] = g_out[k];
    }
    diff::dyn4_plant<double>(p, lim, ground, s, gc.dcmd, gc.acmd, o);
    double g_dc = 0.0, g_ac = 0.0;
    diff::dyn4_plant_vjp<double>(p, lim, ground, s, gc.dcmd, gc.acmd, go, gs, g_dc, g_ac);
    double next[14], gstate[14];
    for (int k = 0; k < n; ++k) {
      next[k] = o[k];
      gstate[k] = gs[k];
      now.push_back(o[k]);
    }
    for (int k = 0; k < n; ++k)
      now.push_back(gs[k]);
    now.push_back(g_dc);
    now.push_back(g_ac);
    js << "  {\"params\": " << gc.params << ", \"state\": " << arr(gc.s, n)
       << ", \"delta_cmd\": " << num(gc.dcmd) << ", \"a_cmd\": " << num(gc.acmd)
       << ",\n   \"limits\": {\"L\": " << num(lim.L) << ", \"vmax\": " << num(lim.vmax)
       << ", \"hdt\": " << num(lim.hdt) << ", \"allow_reverse\": " << lim.allow_reverse << "}"
       << ",\n   \"next\": " << arr(next, n) << ",\n   \"g_state\": " << arr(gstate, n)
       << ",\n   \"g_delta_cmd\": " << num(g_dc) << ", \"g_a_cmd\": " << num(g_ac) << "}"
       << (c + 1 < ncase ? "," : "") << "\n";
  }
  js << " ]\n}\n";
  for (const double x : now)
    ASSERT_TRUE(std::isfinite(x));
  const char *write = std::getenv("CVC_DYN4_GOLDEN_WRITE");
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
        << "value " << k << " (case " << k / (2 * n + 2) << "): golden " << golden[k] << ", now "
        << now[k];
}

// ── The external-force port on the four-wheel plant (ext_in, dyn4_rollout_ext) ─────────────────

namespace {

// The controller's frame and vehicle on a small city (the step gradcheck's).
struct ext_city {
  cvc::nav::training_scene sc = cvc::nav::city_scene(48);
  cvc::nav::field_stack fs = sc.field();
  diff::field F{};
  bike_veh v{};
  ext_city() {
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
    const cvc::nav::train_config tc;
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
  }
};

// The ground the port's checks drive on, in the controller's normalized frame: the friction split
// along y = 0 and a low bump, so the force's gradient runs through per-wheel grip and heave too.
struct ext_ground {
  double mu_left = 0.6, mu_right = 0.9, split_width = 0.2;
  double A = 0.01, cx = 0.3, cy = 0.2, r = 0.5;
  template <class S> void at(S wx, S wy, S &mu, S &height) const {
    const S t = S(0.5) + S(0.5) * diff::s_tanh(wy / S(split_width));
    mu = S(mu_right) + t * S(mu_left - mu_right);
    const S dx = wx - S(cx), dy = wy - S(cy);
    height = S(A) * diff::s_exp(-(dx * dx + dy * dy) / S(r * r));
  }
};

// An affine force F = A o + b (J = A), the checks' and the golden's.
struct affine_ext {
  float A[4] = {0.15f, -0.1f, 0.05f, 0.2f}, b[2] = {0.1f, -0.15f};
};
void affine_ext_sample(void *user, int, float ox, float oy, float *f, float *J) {
  const affine_ext &a = *static_cast<const affine_ext *>(user);
  f[0] = a.A[0] * ox + a.A[1] * oy + a.b[0];
  f[1] = a.A[2] * ox + a.A[3] * oy + a.b[1];
  for (int k = 0; k < 4; ++k)
    J[k] = a.A[k];
}

// A directional gradient check summed over cases: |analytic - central difference| against
// |central difference|, where the difference measures a slope. A case is skipped and counted when
// its one-sided differences disagree (the step straddles a controller or plant switch: a clamp,
// the governor) or its central differences at eps and eps / 2 do (the slope is sharply peaked
// within the step), as nav_dyn_bike_test's fd_sum does.
struct fd_acc {
  double num = 0.0, den = 0.0;
  int cases = 0, skipped = 0;
  template <class Loss> void add(double an, const Loss &loss, double eps) {
    ++cases;
    const double l0 = loss(0.0), lp = loss(eps), lm = loss(-eps), lp2 = loss(0.5 * eps),
                 lm2 = loss(-0.5 * eps);
    const double fp = (lp - l0) / eps, fm = (l0 - lm) / eps;
    const double c1 = (lp - lm) / (2.0 * eps), c2 = (lp2 - lm2) / eps;
    const double scale = std::max({std::fabs(fp), std::fabs(fm), 1e-3});
    if (std::fabs(fp - fm) > 0.1 * scale || std::fabs(c1 - c2) > 0.05 * scale) {
      ++skipped;
      return;
    }
    num += std::fabs(an - c1);
    den += std::fabs(c1);
  }
  double rel() const { return den > 0.0 ? num / den : num; }
  // Within tol of the differences (absolutely, where they are all zero), with at most `quota`
  // of the cases skipped.
  void expect(double tol, const std::string &what, double quota = 0.25) const {
    EXPECT_LE(num, tol * den + 1e-6) << what;
    EXPECT_LE(skipped, static_cast<int>(quota * cases))
        << what << ": " << skipped << " of " << cases << " skipped";
  }
};

// The checks' draws: their own seed, or CVC_EXT_CASE_SEED's (nav_dyn_bike_test's knob) for
// sweeping them over other draws, as another standard library's would be.
unsigned ext_seed(unsigned own) {
  const char *seed = std::getenv("CVC_EXT_CASE_SEED");
  return seed ? static_cast<unsigned>(std::atoi(seed)) * 7u + own : own;
}

dyn4_state<float> ext_start(float ox, float oy, float th, float sp) {
  dyn4_state<float> s{};
  s.x = ox;
  s.y = oy;
  s.psi = th;
  s.vx = sp;
  return s;
}

} // namespace

// A zero force, steering or not, is the step without one, bit for bit; and a null ext_in is the
// step as it was.
TEST(Dyn4ExtForce, AZeroForceIsTheStepWithoutIt) {
  const ext_city c;
  const dyn4_params p =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::roboracer(), c.v.L);
  const ext_ground ground;
  const int n = 16;
  std::vector<float> o(2 * n), g(2 * n);
  c.sc.sample_starts_goals(n, 11, o.data(), g.data());
  for (int i = 0; i < n; ++i) {
    const dyn4_state<float> s = ext_start(
        o[2 * i], o[2 * i + 1], std::atan2(g[2 * i + 1] - o[2 * i + 1], g[2 * i] - o[2 * i]), 0.5f);
    dyn4_state<float> a, b;
    diff::dyn4_bike_step(c.F, s, g[2 * i], g[2 * i + 1], 0.6f, 1.8f, 0.9f, c.v, p, ground, a);
    for (const int steer : {0, 1}) {
      diff::ext_in z;
      z.steer = steer;
      diff::dyn4_bike_step(c.F, s, g[2 * i], g[2 * i + 1], 0.6f, 1.8f, 0.9f, c.v, p, ground, b, &z);
      for (int k = 0; k < dyn4_state<float>::size; ++k)
        EXPECT_EQ(a[k], b[k]) << "agent " << i << ", component " << k << ", steer " << steer;
    }
  }
}

// One step's reverse against central differences: dL/dF_ext, and the position's gradient through
// a force that depends on the position (J^T dL/dF_ext), steering and not.
TEST(Dyn4ExtForce, StepGradientsMatchFiniteDifferences) {
  const ext_city c;
  const dyn4_params p =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::roboracer(), c.v.L);
  const ext_ground ground;
  const int n = 24;
  std::vector<float> o(2 * n), g(2 * n);
  c.sc.sample_starts_goals(n, ext_seed(13), o.data(), g.data());
  std::mt19937 rng(ext_seed(17));
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  const float eps = 1e-3f;
  for (const int steer : {1, 0}) {
    fd_acc chkF, chkO;
    for (int i = 0; i < n; ++i) {
      // Off the SDF's grid nodes, as the step gradcheck.
      const float ox = o[2 * i] + 0.04f * u(rng), oy = o[2 * i + 1] + 0.04f * u(rng);
      dyn4_state<float> s =
          ext_start(ox, oy, std::atan2(g[2 * i + 1] - oy, g[2 * i] - ox) + 0.2f * u(rng),
                    0.3f + 0.3f * (u(rng) + 1.0f));
      s.vy = 0.02f * u(rng);
      s.r = 0.2f * u(rng);
      affine_ext af;
      dyn4_state<float> w;
      for (int k = 0; k < dyn4_state<float>::size; ++k)
        w[k] = u(rng);
      const float ux = u(rng), uy = u(rng);
      // L(x, y, F): F either free (cx, cy added to the affine force) or following the position.
      const auto loss = [&](float dx, float dy, float cx, float cy) {
        dyn4_state<float> st = s, out;
        st.x += dx;
        st.y += dy;
        float F[2], J[4];
        affine_ext_sample(&af, 0, st.x, st.y, F, J);
        diff::ext_in e;
        e.fx = F[0] + cx;
        e.fy = F[1] + cy;
        e.steer = steer;
        diff::dyn4_bike_step(c.F, st, g[2 * i], g[2 * i + 1], 0.6f, 1.8f, 0.9f, c.v, p, ground, out,
                             &e);
        double acc = 0.0;
        for (int k = 0; k < dyn4_state<float>::size; ++k)
          acc += static_cast<double>(w[k]) * out[k];
        return acc;
      };
      float F[2], J[4];
      affine_ext_sample(&af, 0, s.x, s.y, F, J);
      diff::ext_in e;
      e.fx = F[0];
      e.fy = F[1];
      e.dfx_dox = J[0];
      e.dfx_doy = J[1];
      e.dfy_dox = J[2];
      e.dfy_doy = J[3];
      e.steer = steer;
      dyn4_state<float> gs{};
      float gal = 0, gbe = 0, gga = 0, gf[2] = {0.0f, 0.0f};
      diff::dyn4_bike_step_bwd(c.F, s, g[2 * i], g[2 * i + 1], 0.6f, 1.8f, 0.9f, c.v, p, ground, w,
                               gs, gal, gbe, gga, &e, gf);
      chkF.add(
          gf[0] * ux + gf[1] * uy,
          [&](double t) {
            return loss(0, 0, static_cast<float>(t) * ux, static_cast<float>(t) * uy);
          },
          eps);
      chkO.add(
          gs.x * ux + gs.y * uy,
          [&](double t) {
            return loss(static_cast<float>(t) * ux, static_cast<float>(t) * uy, 0, 0);
          },
          eps);
    }
    std::printf("  dyn4 step with a force, steer %d, %d agents: dL/dF %.2e, dL/do through J "
                "%.2e (relative to central differences; cases on a switch %d, %d)\n",
                steer, n, chkF.rel(), chkO.rel(), chkF.skipped, chkO.skipped);
    // Not steering, the force reaches the vehicle only through the speed command, which these
    // starts often hold at its limit: both sides are zero there, so the bound is absolute too.
    chkF.expect(5e-2, "dL/dF, steer " + std::to_string(steer));
    chkO.expect(5e-2, "dL/do, steer " + std::to_string(steer));
  }
}

// The trajectory is its steps with the held force, a null sampler is the rollout without one,
// and the reverse matches central differences in the initial position and in a force added at
// every step, for a force sampled every step and held over three.
TEST(Dyn4ExtRollout, TrajectoryIsItsStepsAndTheReverseMatchesDifferences) {
  const ext_city c;
  const dyn4_params p =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::roboracer(), c.v.L);
  const ext_ground ground;
  const int T = 10, n = 24;
  std::vector<float> o(2 * n), g(2 * n);
  c.sc.sample_starts_goals(n, ext_seed(19), o.data(), g.data());
  std::vector<dyn4_state<float>> w(static_cast<std::size_t>(T + 1));
  {
    std::mt19937 rng(4);
    std::uniform_real_distribution<float> u(-0.5f, 0.5f);
    for (dyn4_state<float> &x : w)
      for (int k = 0; k < dyn4_state<float>::size; ++k)
        x[k] = u(rng);
  }
  const float eps = 3e-4f, ux = 0.6f, uy = 0.8f;
  std::mt19937 rng(ext_seed(23));
  std::uniform_real_distribution<float> u(-1.0f, 1.0f);
  for (const int hold : {1, 3}) {
    fd_acc chk[2];
    for (int i = 0; i < n; ++i) {
      const float ox = o[2 * i] + 0.04f * u(rng), oy = o[2 * i + 1] + 0.04f * u(rng);
      const float th = std::atan2(g[2 * i + 1] - oy, g[2 * i] - ox) + 0.2f * u(rng);
      std::vector<float> goal(2 * T), coef(3 * T);
      for (int t = 0; t < T; ++t) {
        goal[2 * t] = g[2 * i];
        goal[2 * t + 1] = g[2 * i + 1];
        coef[3 * t] = 0.6f;
        coef[3 * t + 1] = 1.8f;
        coef[3 * t + 2] = 0.9f;
      }
      const auto run = [&](float dx, float dy, float cx, float cy, diff::dyn4_tape &tape) {
        affine_ext af;
        af.b[0] += cx;
        af.b[1] += cy;
        diff::bike_ext_fn fn;
        fn.sample = affine_ext_sample;
        fn.user = &af;
        fn.hold = hold;
        diff::dyn4_rollout_ext(c.F, c.v, p, ground, ext_start(ox + dx, oy + dy, th, 0.5f),
                               goal.data(), coef.data(), T, fn, tape);
        double acc = 0.0;
        for (int t = 0; t <= T; ++t)
          for (int k = 0; k < dyn4_state<float>::size; ++k)
            acc += static_cast<double>(w[t][k]) * tape.state[t][k];
        return acc;
      };
      diff::dyn4_tape tape, tp;
      run(0, 0, 0, 0, tape);
      // The trajectory is the steps with the held force.
      {
        affine_ext af;
        dyn4_state<float> s = tape.state[0];
        diff::ext_in x;
        for (int t = 0; t < T; ++t) {
          if (t % hold == 0) {
            float F[2], J[4];
            affine_ext_sample(&af, t, s.x, s.y, F, J);
            x.fx = F[0];
            x.fy = F[1];
          }
          dyn4_state<float> s1;
          diff::dyn4_bike_step(c.F, s, goal[2 * t], goal[2 * t + 1], 0.6f, 1.8f, 0.9f, c.v, p,
                               ground, s1, &x);
          for (int k = 0; k < dyn4_state<float>::size; ++k)
            EXPECT_EQ(tape.state[t + 1][k], s1[k]);
          s = s1;
        }
      }
      // No sampler: the plain rollout.
      if (hold == 1) {
        diff::dyn4_tape bare;
        diff::dyn4_rollout_ext(c.F, c.v, p, ground, tape.state[0], goal.data(), coef.data(), T,
                               diff::bike_ext_fn{}, bare);
        dyn4_state<float> s = tape.state[0];
        for (int t = 0; t < T; ++t) {
          dyn4_state<float> s1;
          diff::dyn4_bike_step(c.F, s, goal[2 * t], goal[2 * t + 1], 0.6f, 1.8f, 0.9f, c.v, p,
                               ground, s1);
          for (int k = 0; k < dyn4_state<float>::size; ++k)
            EXPECT_EQ(bare.state[t + 1][k], s1[k]);
          s = s1;
        }
      }
      std::vector<float> g_fext(2 * T);
      dyn4_state<float> g_init;
      diff::dyn4_rollout_ext_bwd(c.F, c.v, p, ground, tape, goal.data(), coef.data(), w.data(),
                                 g_fext.data(), nullptr, &g_init);
      chk[0].add(
          g_init.x * ux + g_init.y * uy,
          [&](double t) {
            return run(static_cast<float>(t) * ux, static_cast<float>(t) * uy, 0, 0, tp);
          },
          eps);
      double gc = 0.0;
      for (int t = 0; t < T; ++t)
        gc += g_fext[2 * t] * ux + g_fext[2 * t + 1] * uy;
      // b shifts the force at every sampled step, so held steps share it: the summed dL/dF.
      chk[1].add(
          gc,
          [&](double t) {
            return run(0, 0, static_cast<float>(t) * ux, static_cast<float>(t) * uy, tp);
          },
          eps);
    }
    std::printf("  dyn4 rollout with a force, hold %d, %d steps: dL/dinit %.2e, dL/dF %.2e "
                "(relative to central differences; cases on a switch %d, %d)\n",
                hold, T, chk[0].rel(), chk[1].rel(), chk[0].skipped, chk[1].skipped);
    // Ten steps of the four-wheel plant under the controller cross more switches than one: a third
    // of the cases may sit on one, and the initial position's gradient, which runs through every
    // step's plant and clamps (not this port), is held to 1e-1. The port's dL/dF stays at 5e-2.
    // Over CVC_EXT_CASE_SEED 1..100 the worst were 3.4e-2 (dL/dF) and 6.6e-2 (dL/dinit).
    chk[0].expect(1e-1, "dL/dinit, hold " + std::to_string(hold), 1.0 / 3.0);
    chk[1].expect(5e-2, "dL/dF, hold " + std::to_string(hold), 1.0 / 3.0);
  }
}

// The torch twin's gate (GRL-SNAM): dyn4_rollout_ext and its reverse on fixed cases, with the
// field and the ground they sample, written to tests/data/dyn4_ext_rollout_golden.json. Four
// agents, 10 steps each, per-step goals and coefficients, an affine force per agent (F = A o + b,
// J = A) held for 1 or 3 steps, one whose force does not steer; L = sum_t w_t . state_t over the
// 14 state components. CVC_DYN4_EXT_GOLDEN_WRITE=1 rewrites the file; otherwise this build must
// reproduce it (float, libm: to 1e-4, not bit for bit across platforms).
TEST(Dyn4ExtRollout, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn4_ext_rollout_golden.json";
  const int R = 32, C = 32, T = 10;
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
  const dyn4_params p =
      cvc::physics::nav_plant::dyn4_params_at(cvc::physics::vehicle_desc::roboracer(), v.L);
  const ext_ground ground;

  struct gcase {
    float s0[4], g0[2], A[4], b[2];
    int hold, steer;
  };
  const gcase cases[] = {
      {{-2.5f, -0.5f, 0.0f, 0.3f}, {2.5f, 0.3f}, {0.2f, -0.1f, 0.05f, 0.15f}, {0.1f, 0.2f}, 1, 1},
      {{-2.4f, 0.6f, 0.2f, 0.6f}, {2.4f, -0.4f}, {-0.3f, 0.2f, 0.1f, -0.2f}, {0.0f, -0.3f}, 3, 1},
      {{1.8f, -2.2f, 1.4f, 0.1f}, {-0.5f, 2.6f}, {0.4f, 0.0f, 0.0f, 0.4f}, {-0.2f, 0.1f}, 1, 1},
      {{2.2f, 0.4f, 3.0f, 0.4f}, {-2.6f, -0.2f}, {0.15f, -0.2f, 0.2f, 0.15f}, {0.2f, 0.1f}, 3, 0},
  };
  const int n = dyn4_state<float>::size;
  std::vector<dyn4_state<float>> w(static_cast<std::size_t>(T + 1));
  for (int t = 0; t <= T; ++t)
    for (int k = 0; k < n; ++k)
      w[t][k] = 0.1f * static_cast<float>((3 * t + k) % 7 - 3);

  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.9g", x);
    return std::string(b);
  };
  const auto arr = [&](const float *x, std::size_t k) {
    std::string out = "[";
    for (std::size_t i = 0; i < k; ++i)
      out += (i ? ", " : "") + num(x[i]);
    return out + "]";
  };
  const auto states = [&](const std::vector<dyn4_state<float>> &ss) {
    std::vector<float> flat;
    for (const dyn4_state<float> &s : ss)
      for (int k = 0; k < n; ++k)
        flat.push_back(s[k]);
    return flat;
  };
  std::vector<double> now;
  std::ostringstream js;
  js << "{\n \"format\": \"cvc-dyn4-ext-rollout-golden-1\",\n";
  js << " \"note\": \"diff::dyn4_rollout_ext (float) on fixed cases; L = sum_t w[t] . state_t\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"z\", \"zd\", \"phi\", "
        "\"phid\", \"th\", \"thd\", \"delta\", \"a\"],\n";
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
  const std::pair<const char *, float> params[] = {{"m", p.m},
                                                   {"Iz", p.Iz},
                                                   {"Ix", p.Ix},
                                                   {"Iy", p.Iy},
                                                   {"lf", p.lf},
                                                   {"lr", p.lr},
                                                   {"tf", p.tf},
                                                   {"tr", p.tr},
                                                   {"h", p.h},
                                                   {"g", p.g},
                                                   {"k_f", p.k_f},
                                                   {"k_r", p.k_r},
                                                   {"c_f", p.c_f},
                                                   {"c_r", p.c_r},
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
  js << " \"params\": {";
  for (std::size_t k = 0; k < sizeof params / sizeof params[0]; ++k)
    js << (k ? ", " : "") << "\"" << params[k].first << "\": " << num(params[k].second);
  js << ", \"iters\": " << p.iters << "},\n";
  js << " \"ground\": {\"mu_left\": " << num(ground.mu_left)
     << ", \"mu_right\": " << num(ground.mu_right)
     << ", \"split_width\": " << num(ground.split_width) << ", \"A\": " << num(ground.A)
     << ", \"cx\": " << num(ground.cx) << ", \"cy\": " << num(ground.cy)
     << ", \"r\": " << num(ground.r) << "},\n";
  const std::vector<float> wflat = states(w);
  js << " \"T\": " << T << ",\n \"w\": " << arr(wflat.data(), wflat.size()) << ",\n \"cases\": [\n";
  const std::size_t ncase = sizeof cases / sizeof cases[0];
  for (std::size_t ci = 0; ci < ncase; ++ci) {
    const gcase &gc = cases[ci];
    std::vector<float> goal(2 * T), coef(3 * T);
    for (int t = 0; t < T; ++t) {
      goal[2 * t] = gc.g0[0] + 0.01f * static_cast<float>(t);
      goal[2 * t + 1] = gc.g0[1] - 0.005f * static_cast<float>(t);
      coef[3 * t] = 0.6f + 0.01f * static_cast<float>(t);
      coef[3 * t + 1] = 1.8f - 0.02f * static_cast<float>(t);
      coef[3 * t + 2] = 0.9f + 0.005f * static_cast<float>(t);
    }
    affine_ext af;
    for (int k = 0; k < 4; ++k)
      af.A[k] = gc.A[k];
    af.b[0] = gc.b[0];
    af.b[1] = gc.b[1];
    diff::bike_ext_fn fn;
    fn.sample = affine_ext_sample;
    fn.user = &af;
    fn.hold = gc.hold;
    fn.steer = gc.steer;
    diff::dyn4_tape tape;
    diff::dyn4_rollout_ext(F, v, p, ground, ext_start(gc.s0[0], gc.s0[1], gc.s0[2], gc.s0[3]),
                           goal.data(), coef.data(), T, fn, tape);
    std::vector<float> g_fext(2 * T), g_coef(3 * T);
    dyn4_state<float> g_init;
    diff::dyn4_rollout_ext_bwd(F, v, p, ground, tape, goal.data(), coef.data(), w.data(),
                               g_fext.data(), g_coef.data(), &g_init);
    const std::vector<float> st = states(tape.state), gi = states({g_init});
    for (const std::vector<float> *x :
         std::initializer_list<const std::vector<float> *>{&st, &g_fext, &g_coef, &gi})
      for (const float y : *x)
        now.push_back(y);
    js << "  {\"state0\": " << arr(gc.s0, 4) << ", \"A\": " << arr(gc.A, 4)
       << ", \"b\": " << arr(gc.b, 2) << ", \"hold\": " << gc.hold << ", \"steer\": " << gc.steer
       << ",\n   \"goals\": " << arr(goal.data(), goal.size())
       << ",\n   \"coefs\": " << arr(coef.data(), coef.size())
       << ",\n   \"states\": " << arr(st.data(), st.size())
       << ",\n   \"g_fext\": " << arr(g_fext.data(), g_fext.size())
       << ",\n   \"g_coef\": " << arr(g_coef.data(), g_coef.size())
       << ",\n   \"g_init\": " << arr(gi.data(), gi.size()) << "}" << (ci + 1 < ncase ? "," : "")
       << "\n";
  }
  js << " ]\n}\n";
  for (const double x : now)
    ASSERT_TRUE(std::isfinite(x));
  const char *write = std::getenv("CVC_DYN4_EXT_GOLDEN_WRITE");
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
