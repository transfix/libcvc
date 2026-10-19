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
#include <cmath>
#include <cstdio>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/dyn_4w.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <gtest/gtest.h>
#include <random>
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
                                       dyn4_state<double> s, int n,
                                       double (*dcmd)(int), double (*acmd)(int)) {
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
