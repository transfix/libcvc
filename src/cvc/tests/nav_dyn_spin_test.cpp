/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_dyn_spin_test: the wheel-spin plants (diff::dyn_spin_plant, the single track's; and
// diff::dyn4_spin_plant, the four-wheel one's), opt-in beside dyn_plant and dyn4_plant. Each
// wheel's speed is a state, driven and braked by torques, and its tire's longitudinal force comes
// from its slip, so the vehicles spin their wheels and lock them. The gates hold them as
// nav_dyn_bike_test's DynBikePH.* and nav_dyn_4w_test's Dyn4PH.* hold the plants they extend:
//   - the port-Hamiltonian structure at every slip: J skew, R positive semi-definite (the
//     longitudinal slip's secant along the patch's slip direction), the port (wheel torque, wheel
//     speed);
//   - the vector field against a very short plant step;
//   - the step's books closing to second order;
//   - the gradient (forward-mode duals) against finite differences;
//   - the behaviour: a spinning wheel on low grip, a locked one under hard braking, and dyn_plant's
//     motion where nothing slips;
//   - the goldens GRL-SNAM's torch twins are held to (DynSpinPH.Golden, Dyn4SpinPH.Golden).
// The Humvee drives every gate, in metres (S = 1), from its vehicle_desc.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/dyn_4w.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using cvc::nav::diff::bike_veh;
using cvc::nav::diff::dyn_bike_params;
using cvc::nav::diff::dyn_spin_state;
using cvc::nav::diff::dyn_state;
namespace diff = cvc::nav::diff;

namespace {

constexpr double kTwoPi = 6.283185307179586; // M_PI is not portable (MSVC)

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

// Rolling at speed vx: the wheels' rims at the ground's speed.
dyn_spin_state<double> rolling(const dyn_bike_params &p, double vx) {
  dyn_spin_state<double> s{};
  s.b.vx = vx;
  s.om_f = s.om_r = vx / p.r_w;
  return s;
}

// A Humvee state anywhere in the envelope (nav_dyn_bike_test's sliding_state), each wheel
// rolling, spinning, locked or turning backward against the ground.
dyn_spin_state<double> spin_state(const dyn_bike_params &p, std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn_spin_state<double> s{};
  dyn_state<double> &b = s.b;
  const double speed = 3.0 + 25.0 * 0.5 * (1.0 + u(rng));
  b.vx = u(rng) < -0.7 ? -std::min(speed, 8.0) : speed;
  b.delta = 0.5 * u(rng);
  b.vy = 4.0 * u(rng);
  b.r = 1.2 * u(rng);
  b.a = 5.0 * u(rng);
  b.psi = u(rng);
  const double g[2] = {b.vx * std::cos(b.delta) + (b.vy + p.lf * b.r) * std::sin(b.delta), b.vx};
  double *om[2] = {&s.om_f, &s.om_r};
  for (int w = 0; w < 2; ++w) {
    const double mode = 0.5 * (1.0 + u(rng));
    double rim;
    if (mode < 0.4)
      rim = g[w] * (1.0 + 0.05 * u(rng)); // rolling, a little slip
    else if (mode < 0.65)
      rim = g[w] + std::fabs(g[w]) * 1.5 * (1.0 + u(rng)); // spinning
    else if (mode < 0.85)
      rim = 0.0; // locked
    else
      rim = -g[w] * 0.5 * (1.0 + u(rng)); // turning against the ground
    *om[w] = rim / p.r_w;
  }
  return s;
}

// A wheel's slip ratio past the Pacejka peak, tan(pi / 2C) / B.
bool past_peak(double C, double B, double eps, double sigma, double ground) {
  const double kappa = sigma / std::sqrt(ground * ground + eps * eps);
  return std::fabs(kappa) > std::tan(kTwoPi / (4.0 * C)) / B;
}

} // namespace

// The Humvee's wheels come from its vehicle_desc: the radius, both axles' wheels' inertia, and a
// longitudinal tire whose peak is Jolt's and whose sliding fraction is the lateral tire's.
TEST(DynSpin, HumveeWheelsFromItsSpecSheet) {
  const cvc::physics::vehicle_desc d = cvc::physics::vehicle_desc::humvee();
  const cvc::nav::dyn_bike_si si = cvc::physics::nav_plant::dyn_bike_si_of(d);
  EXPECT_DOUBLE_EQ(si.wheel_radius, 0.47);
  EXPECT_DOUBLE_EQ(si.wheel_inertia_front, 2.0 * d.axles[0].wheel_inertia);
  EXPECT_DOUBLE_EQ(si.wheel_inertia_rear, 2.0 * d.axles[1].wheel_inertia);
  EXPECT_DOUBLE_EQ(si.tire_C_long, si.tire_C);
  EXPECT_NEAR(std::tan(kTwoPi / (4.0 * si.tire_C_long)) / si.tire_B_long, d.tire_peak_slip, 1e-12);
  EXPECT_NEAR(std::sin(si.tire_C_long * kTwoPi / 4.0), d.tire_sliding, 1e-9);
  cvc::world_units::normalized_frame f;
  f.scale = 0.05;
  const dyn_bike_params p = cvc::nav::dyn_bike_params_of(si, f);
  EXPECT_FLOAT_EQ(p.r_w, static_cast<float>(0.05 * 0.47));
  EXPECT_FLOAT_EQ(p.J_f, static_cast<float>(0.05 * 0.05 * si.wheel_inertia_front));
  EXPECT_FLOAT_EQ(p.B_x, static_cast<float>(si.tire_B_long));
  std::printf("  Humvee wheels: r %.2f m, J %.1f / %.1f kg m^2 an axle, B_x %.1f, C_x %.3f "
              "(peak at slip %.3f)\n",
              si.wheel_radius, si.wheel_inertia_front, si.wheel_inertia_rear, si.tire_B_long,
              si.tire_C_long, d.tire_peak_slip);
}

// J is skew, R symmetric with nonnegative coefficients on its outer products (so positive
// semi-definite) at every slip, rolling, spinning, locked or turning backward; dH/dx = (v, om);
// with no torque nothing is supplied, and the brakes only ever take energy.
TEST(DynSpinPH, TheStructureIsPortHamiltonianAtEverySlip) {
  for (const double mu : {0.9, 0.3}) {
    const dyn_bike_params p = humvee(mu);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int sliding = 0;
    double worst_psd = 0.0;
    for (int k = 0; k < 500; ++k) {
      dyn_spin_state<double> s = spin_state(p, rng);
      if (k % 5 == 0)
        s.b.a = 0.0;
      double x[5], J[5][5], R[5][5];
      diff::dyn_spin_ph_momentum(p, s, x);
      diff::dyn_spin_ph_J(x, J);
      diff::dyn_spin_ph_R(p, s, R);
      diff::dyn_spin_ph_parts<double> q;
      diff::dyn_spin_ph_parts_at(p, s, q);
      const double e[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
      double eJe = 0.0, norm = 0.0;
      for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
          EXPECT_EQ(J[i][j], -J[j][i]);
          EXPECT_EQ(R[i][j], R[j][i]);
          eJe += e[i] * J[i][j] * e[j];
          norm = std::max(norm, std::fabs(R[i][j]));
        }
      EXPECT_NEAR(eJe, 0.0, 1e-9 * (1.0 + x[0] * x[0]));
      EXPECT_GE(q.Kf, 0.0) << "front longitudinal secant at slip " << q.sf;
      EXPECT_GE(q.Kr, 0.0) << "rear longitudinal secant at slip " << q.sr;
      EXPECT_GE(q.b.Sf, 0.0);
      EXPECT_GE(q.b.Sr, 0.0);
      for (int t = 0; t < 4; ++t) {
        double w[5], wRw = 0.0;
        for (int i = 0; i < 5; ++i)
          w[i] = u(rng);
        for (int i = 0; i < 5; ++i)
          for (int j = 0; j < 5; ++j)
            wRw += w[i] * R[i][j] * w[j];
        worst_psd = std::min(worst_psd, wRw / norm);
        EXPECT_GE(wRw, -1e-12 * norm);
      }
      // dH/dx = (v, om): H is the quadratic form of x and e.
      double xe = 0.0;
      for (int i = 0; i < 5; ++i)
        xe += x[i] * e[i];
      EXPECT_NEAR(diff::dyn_spin_ph_hamiltonian(p, s), 0.5 * xe, 1e-9 * (1.0 + xe));
      const diff::dyn_spin_ph_rates<double> r = diff::dyn_spin_ph_rates_at(p, s, 0.0);
      EXPECT_GE(r.braked, 0.0);
      EXPECT_GE(r.slip, 0.0);
      // The port: y^T u = om^T u is the drive's shaft power less the brakes'.
      double u2[2];
      diff::dyn_spin_ph_input(p, s, u2);
      EXPECT_NEAR(s.om_f * u2[0] + s.om_r * u2[1], r.supplied - r.braked,
                  1e-9 * (1.0 + std::fabs(r.supplied) + r.braked));
      if (s.b.a == 0.0) {
        EXPECT_EQ(r.supplied, 0.0);
        EXPECT_EQ(r.braked, 0.0);
      }
      sliding +=
          past_peak(p.C_x, p.B_x, p.eps, q.sf, q.gf) || past_peak(p.C_x, p.B_x, p.eps, q.sr, q.gr);
    }
    std::printf("  mu %.1f: %d of 500 states have a wheel past its longitudinal peak; least "
                "w^T R w / |R| %.2e\n",
                mu, sliding, worst_psd);
    EXPECT_GT(sliding, 200) << "the wheels do not slide";
  }
}

// One very short dyn_spin_plant step's change in x = (M v, J om) against the port-Hamiltonian
// vector field at every slip, forward and reversing, flat and on a 3% grade, the actuators holding.
// The low-speed blend is a limiter, not part of the field, so it is off here. The step is 1e-7 s:
// a tire near its root is stiff against its wheel (r_w^2 K / J up to ~4e3 / s above 3 m/s), and
// the implicit step's departure from the field at its start is about hdt times that.
TEST(DynSpinPH, TheVectorFieldIsThePlantsAtEverySlip) {
  for (const double mu : {0.9, 0.3}) {
    dyn_bike_params p = humvee(mu);
    p.v_blend = 0.0f;
    bike_veh lim = plant_limits(p, true);
    lim.hdt = 1e-7f;
    std::mt19937 rng(5);
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
      const dyn_spin_state<double> s = spin_state(p, rng);
      const double grade = (k % 2) * 0.03;
      dyn_spin_state<double> o;
      diff::dyn_spin_plant<double>(p, lim, s, s.b.delta, s.b.a, grade, o);
      double x0[5], x1[5], f[5];
      diff::dyn_spin_ph_momentum(p, s, x0);
      diff::dyn_spin_ph_momentum(p, o, x1);
      diff::dyn_spin_ph_rhs(p, s, grade, f);
      const double mg = p.m * p.g;
      const double scale[5] = {mg, mg, mg * (p.lf + p.lr), mg * p.r_w, mg * p.r_w};
      for (int i = 0; i < 5; ++i) {
        const double fd = (x1[i] - x0[i]) / lim.hdt;
        const double rel = std::fabs(fd - f[i]) / scale[i];
        worst = std::max(worst, rel);
        EXPECT_LT(rel, 1e-3) << "axis " << i << ": plant " << fd << ", pH " << f[i] << " at vx "
                             << s.b.vx << ", om " << s.om_f << " / " << s.om_r;
      }
    }
    std::printf("  vector field vs dyn_spin_plant at every slip, mu %.1f: worst %.2e of m g "
                "(m g r_w on the wheels)\n",
                mu, worst);
  }
}

// dyn_spin_ph_forces_at is the field less the structure: J dH/dx + Q is dyn_spin_ph_rhs, and
// dH/dx^T Q is the rates' balance (the structure moves no energy), at every slip, on a grade. A
// discrete-gradient step builds on exactly these two.
TEST(DynSpinPH, TheForcesAreTheFieldLessTheStructure) {
  const dyn_bike_params p = humvee(0.3);
  std::mt19937 rng(9);
  double worst_field = 0.0, worst_power = 0.0;
  for (int k = 0; k < 500; ++k) {
    const dyn_spin_state<double> s = spin_state(p, rng);
    const double grade = (k % 3) * 0.04;
    double Q[5], f[5], x[5], J[5][5];
    diff::dyn_spin_ph_forces_at(p, s, grade, Q);
    diff::dyn_spin_ph_rhs(p, s, grade, f);
    diff::dyn_spin_ph_momentum(p, s, x);
    diff::dyn_spin_ph_J(x, J);
    const double e[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
    const double mg = p.m * p.g;
    const double scale[5] = {mg, mg, mg * (p.lf + p.lr), mg * p.r_w, mg * p.r_w};
    double eQ = 0.0, eQ_abs = 0.0;
    for (int i = 0; i < 5; ++i) {
      double Je = 0.0;
      for (int j = 0; j < 5; ++j)
        Je += J[i][j] * e[j];
      worst_field = std::max(worst_field, std::fabs(Je + Q[i] - f[i]) / scale[i]);
      eQ += e[i] * Q[i];
      eQ_abs += std::fabs(e[i] * Q[i]);
    }
    const diff::dyn_spin_ph_rates<double> r = diff::dyn_spin_ph_rates_at(p, s, grade);
    const double balance = r.supplied - r.braked - r.dissipated - r.slip + r.floor + r.grade;
    worst_power = std::max(worst_power, std::fabs(eQ - balance) / (eQ_abs + 1.0));
    // With the external port's wrench, Q gains it on the body and nothing on the wheels.
    const double W[3] = {0.3 * mg, -0.2 * mg, 0.1 * mg * p.lf};
    double QW[5];
    diff::dyn_spin_ph_forces_at(p, s, grade, QW, W);
    for (int i = 0; i < 5; ++i)
      EXPECT_EQ(QW[i], i < 3 ? Q[i] + W[i] : Q[i]) << "momentum " << i;
  }
  std::printf("  J dH/dx + Q vs the field: worst %.1e of m g; dH/dx^T Q vs the rates: %.1e of "
              "its terms\n",
              worst_field, worst_power);
  EXPECT_LT(worst_field, 1e-12);
  EXPECT_LT(worst_power, 1e-12);
}

namespace {

struct run_books {
  double mean_abs_residual = 0.0, residual = 0.0, flow = 0.0, dH = 0.0;
  double supplied = 0.0, braked = 0.0, slip = 0.0, dissipated = 0.0, limiter = 0.0;
  double peak_kappa = 0.0; // the largest |slip ratio| any wheel reached
};

// dyn_spin_plant_books along a trajectory. `launch`: from 2 m/s on the ground given, full drive
// for 2 s (the wheels spin) then full brakes (they lock) to a stop, the wheel a little turned;
// otherwise nav_dyn_bike_test's swerve: a 0.7 Hz weave past the lateral peak, the throttle working.
run_books book_run(const dyn_bike_params &p, double hdt, bool launch, double seconds) {
  bike_veh lim = plant_limits(p);
  lim.hdt = static_cast<float>(hdt);
  dyn_spin_state<double> s = rolling(p, launch ? 2.0 : 15.0);
  const int n = static_cast<int>(std::lround(seconds / hdt));
  run_books rb;
  const double h0 = diff::dyn_spin_ph_hamiltonian(p, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd = launch ? 0.1 : 0.3 * std::sin(kTwoPi * 0.7 * time);
    const double acmd = launch ? (time < 2.0 ? 4.0 : -9.0) : 1.5 * std::sin(kTwoPi * time / 3.0);
    dyn_spin_state<double> o;
    diff::dyn_spin_ph_books<double> b;
    diff::dyn_spin_plant_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b);
    rb.mean_abs_residual += std::fabs(b.residual);
    rb.residual += b.residual;
    rb.supplied += b.supplied;
    rb.braked += b.braked;
    rb.slip += b.slip;
    rb.dissipated += b.dissipated;
    rb.limiter += b.limiter;
    rb.flow += std::fabs(b.supplied) + b.braked + b.slip + std::fabs(b.dissipated) +
               std::fabs(b.floor) + std::fabs(b.grade) + std::fabs(b.limiter);
    diff::dyn_spin_ph_parts<double> q;
    diff::dyn_spin_ph_parts_at(p, o, q);
    const double e2 = double(p.eps) * p.eps;
    rb.peak_kappa = std::max({rb.peak_kappa, std::fabs(q.sf) / std::sqrt(q.gf * q.gf + e2),
                              std::fabs(q.sr) / std::sqrt(q.gr * q.gr + e2)});
    s = o;
  }
  rb.mean_abs_residual /= n;
  rb.dH = diff::dyn_spin_ph_hamiltonian(p, s) - h0;
  return rb;
}

} // namespace

// The gate: every step's books close but for the integrator's error, second order per step,
// through a weave past the lateral peak and through a launch that spins the wheels and a stop that
// locks them, from the live plant's 0.01 s step down. A spinning or locked tire's slip is stiff
// against its wheel (r_w^2 K / J of order 1e3 / s): while hdt is long beside that, the implicit
// step relaxes the slip within it and the residual falls faster than 4 per halving, to 4 once
// hdt is short.
TEST(DynSpinPH, StepBooksCloseToSecondOrder) {
  struct scenario {
    const char *name;
    double mu;
    bool launch;
    double seconds;
  };
  for (const scenario &sc : {scenario{"swerve past the peak, mu 0.6", 0.6, false, 6.0},
                             scenario{"launch and lock, mu 0.3", 0.3, true, 4.0}}) {
    const dyn_bike_params p = humvee(sc.mu);
    double prev = 0.0;
    for (const double hdt : {0.01, 0.005, 0.0025, 0.00125}) {
      const run_books rb = book_run(p, hdt, sc.launch, sc.seconds);
      std::printf("  %s, hdt %.4f s: dH %.4g J, drive %.4g, brakes %.4g, slip %.4g, tires and "
                  "drag %.4g, limiter %.3g; peak |kappa| %.2f; mean |residual| %.3e J, |sum| / "
                  "flow %.2e\n",
                  sc.name, hdt, rb.dH, rb.supplied, rb.braked, rb.slip, rb.dissipated, rb.limiter,
                  rb.peak_kappa, rb.mean_abs_residual, std::fabs(rb.residual) / rb.flow);
      if (prev > 0.0) {
        const double ratio = prev / rb.mean_abs_residual;
        EXPECT_GT(ratio, 3.0) << sc.name << ", hdt " << hdt;
        EXPECT_LT(ratio, sc.launch ? 10.0 : 5.5) << sc.name << ", hdt " << hdt;
      }
      prev = rb.mean_abs_residual;
      EXPECT_LT(std::fabs(rb.residual) / rb.flow, 5e-3) << sc.name << ", hdt " << hdt;
      if (sc.launch)
        EXPECT_GT(rb.peak_kappa, 0.5) << "the wheels neither spun nor locked";
    }
  }
}

// What the wheels add. On ice-like ground (mu 0.3) a full drive spins them: the slip ratio runs
// past the peak and the vehicle gains no more than the ground's grip allows. Braking past the grip
// (mu 0.6) locks them: the wheels stop while the vehicle still moves, and it stops later than the
// grip would let a vehicle that does not lock. Where nothing slips, the motion is dyn_plant's with
// the wheels' spin added to the mass along the heading, m + sum J / r_w^2.
TEST(DynSpin, SpinsOnLowGripLocksUnderHardBrakingRollsLikeDynPlant) {
  {
    const dyn_bike_params p = humvee(0.3);
    bike_veh lim = plant_limits(p);
    lim.hdt = 0.01f;
    dyn_spin_state<double> s = rolling(p, 5.0);
    for (int t = 0; t < 100; ++t) {
      dyn_spin_state<double> o;
      diff::dyn_spin_plant<double>(p, lim, s, 0.0, 4.0, 0.0, o);
      s = o;
    }
    const double kf = (p.r_w * s.om_f - s.b.vx) / s.b.vx, kr = (p.r_w * s.om_r - s.b.vx) / s.b.vx;
    std::printf("  full drive on mu 0.3 for 1 s from 5 m/s: %.2f m/s, slip ratios %.2f front, "
                "%.2f rear (peak %.3f)\n",
                s.b.vx, kf, kr, std::tan(kTwoPi / (4.0 * p.C_x)) / p.B_x);
    EXPECT_GT(kr, 0.2);
    EXPECT_GT(kf, 0.2);
    EXPECT_LT(s.b.vx - 5.0, 0.3 * p.g * 1.01) << "faster than the grip allows";
    EXPECT_GT(s.b.vx - 5.0, 0.3 * p.g * 0.4) << "the spinning tires gave nothing";
  }
  {
    // The brakes' sign is smoothed over the slip's speed floor at the rim (eps), so a locked
    // wheel creeps at about that: locked is a slip ratio past -0.8. dyn_plant's force, the drive's
    // saturated at the grip, never slides.
    const dyn_bike_params p = humvee(0.6);
    bike_veh lim = plant_limits(p);
    lim.hdt = 0.01f;
    dyn_spin_state<double> s = rolling(p, 20.0);
    dyn_state<double> d = s.b;
    double x = 0.0, xd = 0.0, least_kappa = 0.0;
    for (int t = 0; t < 1000 && (s.b.vx > 0.05 || d.vx > 0.05); ++t) {
      dyn_spin_state<double> o;
      dyn_state<double> od;
      diff::dyn_spin_plant<double>(p, lim, s, 0.0, -8.0, 0.0, o);
      diff::dyn_plant<double>(p, lim, d, 0.0, -8.0, 0.0, od);
      if (o.b.vx > 5.0)
        least_kappa = std::min(least_kappa, (p.r_w * o.om_f - o.b.vx) / o.b.vx);
      x += 0.5 * (s.b.vx + o.b.vx) * lim.hdt;
      xd += 0.5 * (d.vx + od.vx) * lim.hdt;
      s = o;
      d = od;
    }
    std::printf("  braking at 8 m/s^2 from 20 m/s on mu 0.6: the front wheels' slip ratio reaches "
                "%.2f; stops in %.1f m (dyn_plant %.1f m)\n",
                least_kappa, x, xd);
    EXPECT_LT(least_kappa, -0.8) << "the front wheels did not lock";
    EXPECT_GT(x, xd * 1.02) << "a locked wheel slides with less than its peak";
  }
  {
    // A gentle weave on a gentle throttle on dry ground. With the wheels' inertia at 1% of the
    // Humvee's the slip stays small and the path is dyn_plant's within centimetres.
    dyn_bike_params p = humvee(0.9);
    p.J_f *= 0.01f;
    p.J_r *= 0.01f;
    bike_veh lim = plant_limits(p);
    lim.hdt = 0.01f;
    dyn_spin_state<double> s = rolling(p, 10.0);
    dyn_state<double> d = s.b;
    double worst = 0.0;
    for (int t = 0; t < 500; ++t) {
      const double dcmd = 0.1 * std::sin(kTwoPi * 0.3 * t * lim.hdt), acmd = 0.8;
      dyn_spin_state<double> o;
      dyn_state<double> od;
      diff::dyn_spin_plant<double>(p, lim, s, dcmd, acmd, 0.0, o);
      diff::dyn_plant<double>(p, lim, d, dcmd, acmd, 0.0, od);
      s = o;
      d = od;
      worst = std::max(worst, std::hypot(s.b.x - d.x, s.b.y - d.y));
    }
    std::printf("  a gentle weave for 5 s, the wheels' inertia at 1%%: dyn_spin_plant within %.3f "
                "m of dyn_plant, %.4f m/s apart at the end\n",
                worst, std::fabs(s.b.vx - d.vx));
    EXPECT_LT(worst, 0.03);
  }
  {
    // With the Humvee's wheels, a steady drive straight ahead (no drag) accelerates the vehicle
    // and spins up its wheels: m a = F_x, (J_f + J_r) a / r_w^2 = the torques less r_w F_x.
    dyn_bike_params p = humvee(0.9);
    p.drag = 0.0f;
    p.c_rr = 0.0f;
    bike_veh lim = plant_limits(p);
    lim.hdt = 0.01f;
    dyn_spin_state<double> s = rolling(p, 10.0);
    s.b.a = 0.8; // the drive's actuator already there
    for (int t = 0; t < 200; ++t) {
      dyn_spin_state<double> o;
      diff::dyn_spin_plant<double>(p, lim, s, 0.0, 0.8, 0.0, o);
      s = o;
    }
    const double m_eff = p.m + (p.J_f + p.J_r) / (double(p.r_w) * p.r_w);
    const double want = 10.0 + 2.0 * p.m * 0.8 / m_eff;
    std::printf("  a 0.8 m/s^2 drive for 2 s from 10 m/s: %.4f m/s; m + sum J / r^2 = %.0f kg "
                "gives %.4f, m alone 11.6\n",
                s.b.vx, m_eff, want);
    EXPECT_NEAR(s.b.vx, want, 0.01 * (want - 10.0));
  }
}

// The wheel-spin plant's vector-Jacobian product (forward-mode duals) against central finite
// differences, in double: rolling, spinning, locked, cornering past the peak, braking.
TEST(DynSpin, PlantGradientMatchesFiniteDifference) {
  const dyn_bike_params p = humvee(0.6);
  bike_veh lim = plant_limits(p, true);
  lim.hdt = 0.02f;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst = 0.0;
  for (int k = 0; k < 40; ++k) {
    const dyn_spin_state<double> s = spin_state(p, rng);
    const double dcmd = s.b.delta + 0.1 * u(rng);
    // Away from a = 0, where the torques switch between the drive and the brakes.
    const double acmd = (s.b.a >= 0.0 ? 1.0 : -1.0) * (1.0 + 4.0 * std::fabs(u(rng)));
    dyn_spin_state<double> g_o, g_s;
    for (int i = 0; i < dyn_spin_state<double>::size; ++i)
      g_o[i] = u(rng);
    double g_dc = 0.0, g_ac = 0.0;
    diff::dyn_spin_plant_vjp<double>(p, lim, s, dcmd, acmd, 0.0, g_o, g_s, g_dc, g_ac);
    const auto loss = [&](const dyn_spin_state<double> &x, double dc, double ac) {
      dyn_spin_state<double> o;
      diff::dyn_spin_plant<double>(p, lim, x, dc, ac, 0.0, o);
      double acc = 0.0;
      for (int i = 0; i < dyn_spin_state<double>::size; ++i)
        acc += g_o[i] * o[i];
      return acc;
    };
    for (int j = 0; j < dyn_spin_state<double>::size + 2; ++j) {
      const double h = 1e-6;
      dyn_spin_state<double> sp = s, sm = s;
      double dcp = dcmd, dcm = dcmd, acp = acmd, acm = acmd;
      if (j < dyn_spin_state<double>::size) {
        sp[j] += h;
        sm[j] -= h;
      } else if (j == dyn_spin_state<double>::size) {
        dcp += h;
        dcm -= h;
      } else {
        acp += h;
        acm -= h;
      }
      const double fd = (loss(sp, dcp, acp) - loss(sm, dcm, acm)) / (2.0 * h);
      const double an = j < dyn_spin_state<double>::size
                            ? g_s[j]
                            : (j == dyn_spin_state<double>::size ? g_dc : g_ac);
      const double rel = std::fabs(fd - an) / (1.0 + std::fabs(fd));
      worst = std::max(worst, rel);
      EXPECT_LT(rel, 1e-5) << "state " << k << ", input " << j << ": duals " << an << ", fd " << fd;
    }
  }
  std::printf("  dyn_spin_plant_vjp vs central differences: worst %.2e\n", worst);
}

namespace {

// Golden-file helpers, as nav_dyn_bike_test's DynBikePH.Golden writes and reads its file: every
// value the build computes in the file's order, with what it is, and the JSON.
struct golden_writer {
  std::vector<double> now;
  std::vector<std::string> what;
  std::ostringstream js;
  static std::string num(double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", x);
    return std::string(b);
  }
  static std::string arr(const double *v, int n) {
    std::string out = "[";
    for (int k = 0; k < n; ++k)
      out += (k ? ", " : "") + num(v[k]);
    return out + "]";
  }
  static std::string obj(const char *const *keys, const double *v, int n) {
    std::string out = "{";
    for (int k = 0; k < n; ++k)
      out += std::string(k ? ", " : "") + "\"" + keys[k] + "\": " + num(v[k]);
    return out + "}";
  }
  void record(const char *name, const char *group, const char *const *keys, const double *v,
              int n) {
    for (int k = 0; k < n; ++k) {
      now.push_back(v[k]);
      std::string label = std::string(name) + " " + group;
      if (n > 1)
        label += "." + (keys ? std::string(keys[k]) : std::to_string(k));
      what.push_back(label);
    }
  }
};

// Checks `now` against the file's values to 1e-9 relative: `read` appends one case's golden
// values in the order they were recorded.
void check_golden(
    const std::filesystem::path &path, const golden_writer &w,
    const std::function<void(const boost::property_tree::ptree &, std::vector<double> &)> &read) {
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &c : want.get_child("cases"))
    read(c.second, golden);
  ASSERT_EQ(golden.size(), w.now.size());
  for (std::size_t k = 0; k < w.now.size(); ++k)
    EXPECT_LE(std::fabs(w.now[k] - golden[k]),
              1e-9 * std::max(1.0, std::max(std::fabs(w.now[k]), std::fabs(golden[k]))))
        << w.what[k] << ": golden " << golden[k] << ", now " << w.now[k];
}

void read_named(const boost::property_tree::ptree &c, const char *group, const char *const *keys,
                int n, std::vector<double> &out) {
  for (int k = 0; k < n; ++k)
    out.push_back(c.get<double>(std::string(group) + "." + keys[k]));
}

void read_listed(const boost::property_tree::ptree &c, const char *key, std::vector<double> &out) {
  for (const auto &v : c.get_child(key))
    out.push_back(v.second.get_value<double>());
}

} // namespace

// The values GRL-SNAM's torch twin of the wheel-spin plant, its form and its books is held to, on
// DynBike.Golden's Humvee (lags and the low-speed blend) with its wheels. Per case: the parts
// (dyn_spin_ph_parts_at) and the rates (dyn_spin_ph_rates_at) at its state; one
// dyn_spin_plant_books step from it, with the twist the step reached before its limiters and its
// books; and the gradient of a fixed weighting of the books on the state and the commands, by
// forward-mode duals. The cases roll, corner, spin, lock and brake a reversing vehicle.
// CVC_DYN_SPIN_PH_GOLDEN_WRITE=1 rewrites tests/data/dyn_spin_ph_golden.json; otherwise this build
// must reproduce it to 1e-9.
TEST(DynSpinPH, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn_spin_ph_golden.json";
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.v_blend = 1.0f;
  p.v_width = 0.3f;
  struct gcase {
    const char *name;
    double s[10], dcmd, acmd, grade;
    bool reverse;
  };
  const gcase cases[] = {
      {"cruising", {0.0, 0.0, 0.0, 12.0, 0.1, 0.05, 0.02, 0.5, 25.6, 25.55}, 0.05, 1.0, 0.0, false},
      {"hard corner",
       {1.0, -2.0, 0.3, 15.0, -1.2, 0.6, 0.25, 0.0, 32.0, 31.5},
       0.3,
       0.0,
       0.02,
       false},
      {"trail braking, the front locking",
       {-3.0, 4.0, -1.1, 14.0, 0.8, 0.4, 0.15, -4.0, 5.0, 28.0},
       0.15,
       -6.0,
       -0.03,
       false},
      {"wheelspin launch",
       {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.05, 3.0, 15.0, 18.0},
       0.05,
       4.0,
       0.0,
       false},
      {"past the peak",
       {2.0, 2.0, 2.5, 10.0, 2.5, 1.1, 0.4, 1.0, 22.0, 23.0},
       0.45,
       2.0,
       0.0,
       false},
      {"locked skid", {0.0, 0.0, 0.0, 8.0, 0.3, 0.1, 0.1, -5.0, 0.0, 0.0}, 0.1, -7.0, 0.0, false},
      {"creeping", {0.0, 0.0, 0.0, 0.6, 0.02, 0.05, 0.3, 0.3, 1.3, 1.25}, 0.35, 0.5, 0.0, false},
      {"pulling away", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1, 2.0, 0.0, false},
      {"braking backward",
       {0.5, 0.5, 0.2, -2.0, 0.05, -0.1, 0.2, -0.5, -4.2, -4.3},
       0.25,
       -0.8,
       0.0,
       true},
  };
  const char *const part_keys[] = {"u",  "sg",   "uf",   "Fxf",  "Fxr",  "Fyf", "Fyr", "wf", "wr",
                                   "Sf", "Sr",   "Dx",   "cd",   "gf",   "gr",  "sf",  "sr", "Kf",
                                   "Kr", "Td_f", "Td_r", "Tb_f", "Tb_r", "sgf", "sgr", "Tf", "Tr"};
  const char *const rate_keys[] = {"dissipated", "slip", "supplied", "braked", "floor", "grade"};
  const char *const free_keys[] = {"vx", "vy", "r"};
  const char *const book_keys[] = {"dH",    "dissipated", "slip",    "supplied", "braked",
                                   "floor", "grade",      "limiter", "residual"};
  const double g_books[9] = {0.9, -0.4, 0.3, 0.7, -0.5, 1.3, -0.6, 0.8, 1.1};
  golden_writer w;
  std::ostringstream &js = w.js;
  js << "{\n \"format\": \"cvc-dyn-spin-ph-golden-1\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"delta\", \"a\", \"om_f\", "
        "\"om_r\"],\n";
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
                                                   {"fz_min", p.fz_min},
                                                   {"r_w", p.r_w},
                                                   {"J_f", p.J_f},
                                                   {"J_r", p.J_r},
                                                   {"B_x", p.B_x},
                                                   {"C_x", p.C_x}};
  bool first = true;
  for (const auto &kv : params) {
    js << (first ? "" : ", ") << "\"" << kv.first << "\": " << golden_writer::num(kv.second);
    first = false;
  }
  js << ", \"iters\": " << p.iters << "},\n \"g_books\": " << golden_writer::arr(g_books, 9)
     << ",\n \"cases\": [\n";
  const int ncase = static_cast<int>(sizeof cases / sizeof cases[0]);
  for (int c = 0; c < ncase; ++c) {
    const gcase &gc = cases[c];
    const bike_veh lim = plant_limits(p, gc.reverse);
    dyn_spin_state<double> s;
    for (int k = 0; k < 10; ++k)
      s[k] = gc.s[k];
    diff::dyn_spin_ph_parts<double> q;
    diff::dyn_spin_ph_parts_at(p, s, q);
    const double parts[] = {q.b.u,   q.b.sg,  q.b.uf, q.b.Fxf, q.b.Fxr, q.b.Fyf, q.b.Fyr,
                            q.b.wf,  q.b.wr,  q.b.Sf, q.b.Sr,  q.b.Dx,  q.b.cd,  q.gf,
                            q.gr,    q.sf,    q.sr,   q.Kf,    q.Kr,    q.Td[0], q.Td[1],
                            q.Tb[0], q.Tb[1], q.sgf,  q.sgr,   q.Tf,    q.Tr};
    const diff::dyn_spin_ph_rates<double> rt = diff::dyn_spin_ph_rates_at(p, s, gc.grade);
    const double rates[] = {rt.dissipated, rt.slip, rt.supplied, rt.braked, rt.floor, rt.grade};
    dyn_spin_state<double> o, o_books;
    diff::dyn_step_free<double> seen;
    diff::dyn_spin_plant<double>(p, lim, s, gc.dcmd, gc.acmd, gc.grade, o, &seen);
    diff::dyn_spin_ph_books<double> b;
    diff::dyn_spin_plant_books<double>(p, lim, s, gc.dcmd, gc.acmd, gc.grade, o_books, b);
    for (int k = 0; k < 10; ++k)
      EXPECT_EQ(o_books[k], o[k]) << gc.name << ": the books' step is not dyn_spin_plant's";
    double next[10];
    for (int k = 0; k < 10; ++k)
      next[k] = o[k];
    const double twist[] = {seen.vx, seen.vy, seen.r};
    const double books[] = {b.dH,    b.dissipated, b.slip,    b.supplied, b.braked,
                            b.floor, b.grade,      b.limiter, b.residual};
    typedef diff::dual<double> D;
    double grad[12];
    for (int j = 0; j < 12; ++j) {
      dyn_spin_state<D> ds, od;
      for (int k = 0; k < 10; ++k)
        ds[k] = D(gc.s[k], k == j ? 1.0 : 0.0);
      diff::dyn_spin_ph_books<D> bd;
      diff::dyn_spin_plant_books<D>(p, lim, ds, D(gc.dcmd, j == 10 ? 1.0 : 0.0),
                                    D(gc.acmd, j == 11 ? 1.0 : 0.0), D(gc.grade), od, bd);
      const D bv[] = {bd.dH,    bd.dissipated, bd.slip,    bd.supplied, bd.braked,
                      bd.floor, bd.grade,      bd.limiter, bd.residual};
      grad[j] = 0.0;
      for (int k = 0; k < 9; ++k)
        grad[j] += g_books[k] * bv[k].d;
    }
    w.record(gc.name, "parts", part_keys, parts, 27);
    w.record(gc.name, "rates", rate_keys, rates, 6);
    w.record(gc.name, "next", nullptr, next, 10);
    w.record(gc.name, "free", free_keys, twist, 3);
    w.record(gc.name, "books", book_keys, books, 9);
    w.record(gc.name, "g_state", nullptr, grad, 10);
    w.record(gc.name, "g_delta_cmd", nullptr, grad + 10, 1);
    w.record(gc.name, "g_a_cmd", nullptr, grad + 11, 1);
    js << "  {\"name\": \"" << gc.name << "\", \"state\": " << golden_writer::arr(gc.s, 10)
       << ", \"delta_cmd\": " << golden_writer::num(gc.dcmd)
       << ", \"a_cmd\": " << golden_writer::num(gc.acmd)
       << ", \"grade\": " << golden_writer::num(gc.grade)
       << ",\n   \"limits\": {\"L\": " << golden_writer::num(lim.L)
       << ", \"vmax\": " << golden_writer::num(lim.vmax)
       << ", \"hdt\": " << golden_writer::num(lim.hdt)
       << ", \"allow_reverse\": " << lim.allow_reverse << "}"
       << ",\n   \"parts\": " << golden_writer::obj(part_keys, parts, 27)
       << ",\n   \"rates\": " << golden_writer::obj(rate_keys, rates, 6)
       << ",\n   \"next\": " << golden_writer::arr(next, 10)
       << ",\n   \"free\": " << golden_writer::obj(free_keys, twist, 3)
       << ",\n   \"books\": " << golden_writer::obj(book_keys, books, 9)
       << ",\n   \"g_state\": " << golden_writer::arr(grad, 10)
       << ",\n   \"g_delta_cmd\": " << golden_writer::num(grad[10])
       << ", \"g_a_cmd\": " << golden_writer::num(grad[11]) << "}" << (c + 1 < ncase ? "," : "")
       << "\n";
  }
  js << " ]\n}\n";
  const char *write = std::getenv("CVC_DYN_SPIN_PH_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  check_golden(path, w, [&](const boost::property_tree::ptree &c, std::vector<double> &out) {
    read_named(c, "parts", part_keys, 27, out);
    read_named(c, "rates", rate_keys, 6, out);
    read_listed(c, "next", out);
    read_named(c, "free", free_keys, 3, out);
    read_named(c, "books", book_keys, 9, out);
    read_listed(c, "g_state", out);
    out.push_back(c.get<double>("g_delta_cmd"));
    out.push_back(c.get<double>("g_a_cmd"));
  });
}

// ── The four-wheel vehicle with its wheels spinning ────────────────────────────────────────

namespace {

using diff::dyn4_params;
using diff::dyn4_spin_state;

dyn4_params humvee4(double mu_unused = 0.9) {
  (void)mu_unused;
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn4_params_of(
      cvc::physics::nav_plant::dyn4_si_of(cvc::physics::vehicle_desc::humvee()), f);
}

bike_veh limits4(const dyn4_params &p, bool reverse = false) {
  bike_veh v{};
  v.L = p.lf + p.lr;
  v.vmax = 40.0f;
  v.delta_max = 0.6f;
  v.hdt = 0.06f;
  v.allow_reverse = reverse ? 1 : 0;
  return v;
}

// Each patch's speed along its wheel at s (the form's T_i^T xi), from the parts.
void patch_speeds(const dyn4_params &p, const bike_veh &v, const diff::flat_ground &g,
                  const diff::dyn4_state<double> &s, double out[4]) {
  diff::dyn4_ph_parts<double> q;
  diff::dyn4_ph_parts_at(p, v, g, s, q);
  const double xi[6] = {s.vx, s.vy, s.r, s.zd, s.phid, s.thd};
  for (int i = 0; i < 4; ++i) {
    out[i] = 0.0;
    for (int k = 0; k < 6; ++k)
      out[i] += q.T[i][k] * xi[k];
  }
}

dyn4_spin_state<double> rolling4(const dyn4_params &p, double vx) {
  dyn4_spin_state<double> s{};
  s.b.vx = vx;
  for (int i = 0; i < 4; ++i)
    s.om[i] = vx / p.r_w;
  return s;
}

// A Humvee state anywhere in the envelope, sprung, each wheel rolling, spinning, locked or turning
// backward against the ground.
dyn4_spin_state<double> spin_state4(const dyn4_params &p, const bike_veh &v,
                                    const diff::flat_ground &g, std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn4_spin_state<double> s{};
  diff::dyn4_state<double> &b = s.b;
  const double speed = 3.0 + 25.0 * 0.5 * (1.0 + u(rng));
  b.vx = u(rng) < -0.7 ? -std::min(speed, 8.0) : speed;
  b.delta = 0.5 * u(rng);
  b.vy = 4.0 * u(rng);
  b.r = 1.2 * u(rng);
  b.a = 5.0 * u(rng);
  b.psi = u(rng);
  b.z = 0.03 * u(rng);
  b.zd = 0.2 * u(rng);
  b.phi = 0.03 * u(rng);
  b.phid = 0.3 * u(rng);
  b.th = 0.03 * u(rng);
  b.thd = 0.3 * u(rng);
  double gs[4];
  patch_speeds(p, v, g, b, gs);
  for (int i = 0; i < 4; ++i) {
    const double mode = 0.5 * (1.0 + u(rng));
    double rim;
    if (mode < 0.4)
      rim = gs[i] * (1.0 + 0.05 * u(rng));
    else if (mode < 0.65)
      rim = gs[i] + std::fabs(gs[i]) * 1.5 * (1.0 + u(rng));
    else if (mode < 0.85)
      rim = 0.0;
    else
      rim = -gs[i] * 0.5 * (1.0 + u(rng));
    s.om[i] = rim / p.r_w;
  }
  return s;
}

} // namespace

// The four-wheel form with spinning wheels: J skew, R symmetric with nonnegative coefficients on
// its outer products (positive semi-definite), dH/dx = (dH/dq, xi, om), the port's power the
// drive's less the brakes', at every slip and steering angle, sprung, on two frictions.
TEST(Dyn4SpinPH, TheStructureIsPortHamiltonianAtEverySlip) {
  for (const double mu : {0.9, 0.3}) {
    const dyn4_params p = humvee4();
    const bike_veh v = limits4(p);
    diff::flat_ground g;
    g.mu = static_cast<float>(mu);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int sliding = 0;
    double worst_psd = 0.0;
    for (int k = 0; k < 500; ++k) {
      dyn4_spin_state<double> s = spin_state4(p, v, g, rng);
      if (k % 5 == 0)
        s.b.a = 0.0;
      double x[13], e[13], J[13][13], R[13][13];
      diff::dyn4_spin_ph_coords(p, s, x);
      diff::dyn4_spin_ph_grad(p, v, g, s, e);
      diff::dyn4_spin_ph_J(x, J);
      diff::dyn4_spin_ph_R(p, v, g, s, R);
      diff::dyn4_spin_ph_parts<double> q;
      diff::dyn4_spin_ph_parts_at(p, v, g, s, q);
      double eJe = 0.0, eJe_abs = 0.0, norm = 0.0;
      for (int a = 0; a < 13; ++a)
        for (int b = 0; b < 13; ++b) {
          EXPECT_EQ(J[a][b], -J[b][a]);
          EXPECT_EQ(R[a][b], R[b][a]);
          eJe += e[a] * J[a][b] * e[b];
          eJe_abs += std::fabs(e[a] * J[a][b] * e[b]);
          norm = std::max(norm, std::fabs(R[a][b]));
        }
      EXPECT_LE(std::fabs(eJe), 1e-12 * (1.0 + eJe_abs));
      for (int i = 0; i < 4; ++i) {
        EXPECT_GE(q.sl.K[i], 0.0) << "wheel " << i << " at slip " << q.sl.sig[i];
        EXPECT_GE(q.b.Sw[i], 0.0);
        sliding += past_peak(p.C_x, p.B_x, p.eps, q.sl.sig[i], q.sl.g[i]);
      }
      for (int t = 0; t < 4; ++t) {
        double w[13], wRw = 0.0;
        for (int a = 0; a < 13; ++a)
          w[a] = u(rng);
        for (int a = 0; a < 13; ++a)
          for (int b = 0; b < 13; ++b)
            wRw += w[a] * R[a][b] * w[b];
        worst_psd = std::min(worst_psd, wRw / norm);
        EXPECT_GE(wRw, -1e-12 * norm);
      }
      // dH/dx on the wheels: H's differences in om_i over J_i.
      for (int i = 0; i < 4; ++i) {
        const double J_i = i < 2 ? p.J_f : p.J_r, d = 1e-6;
        dyn4_spin_state<double> sp = s, sm = s;
        sp.om[i] += d;
        sm.om[i] -= d;
        const double fd = (diff::dyn4_spin_ph_hamiltonian(p, v, g, sp) -
                           diff::dyn4_spin_ph_hamiltonian(p, v, g, sm)) /
                          (2.0 * d * J_i);
        EXPECT_NEAR(fd, e[9 + i], 1e-5 * (1.0 + std::fabs(e[9 + i])));
      }
      const diff::dyn4_spin_ph_rates<double> r = diff::dyn4_spin_ph_rates_at(p, v, g, s);
      EXPECT_GE(r.braked, 0.0);
      EXPECT_GE(r.slip, 0.0);
      double u4[4], yu = 0.0;
      diff::dyn4_spin_ph_input(p, v, g, s, u4);
      for (int i = 0; i < 4; ++i)
        yu += s.om[i] * u4[i];
      EXPECT_NEAR(yu, r.supplied - r.braked, 1e-9 * (1.0 + std::fabs(r.supplied) + r.braked));
      // The rates are the form's: -e^T R e is the dissipation and the slip together.
      double eRe = 0.0;
      for (int a = 0; a < 13; ++a)
        for (int b = 0; b < 13; ++b)
          eRe += e[a] * R[a][b] * e[b];
      EXPECT_NEAR(eRe, r.dissipated + r.slip, 1e-9 * (1.0 + eRe));
    }
    std::printf("  mu %.1f: %d of 2000 wheels past their longitudinal peak; least w^T R w / |R| "
                "%.2e\n",
                mu, sliding, worst_psd);
    EXPECT_GT(sliding, 800);
  }
}

// One very short dyn4_spin_plant step against the vector field at every slip, forward and
// reversing, the actuators holding (1e-7 s, as DynSpinPH's).
TEST(Dyn4SpinPH, TheVectorFieldIsThePlantsAtEverySlip) {
  dyn4_params p = humvee4();
  p.v_blend = 0.0f;
  bike_veh lim = limits4(p, true);
  lim.hdt = 1e-7f;
  for (const double mu : {0.9, 0.3}) {
    diff::flat_ground g;
    g.mu = static_cast<float>(mu);
    const double mg = p.m * p.g, L = p.lf + p.lr, t = std::max(p.tf, p.tr);
    const double scale[13] = {1.0,
                              1.0,
                              1.0,
                              mg,
                              mg,
                              mg * L,
                              mg,
                              mg * t,
                              mg * L,
                              mg * p.r_w / 4,
                              mg * p.r_w / 4,
                              mg * p.r_w / 4,
                              mg * p.r_w / 4};
    std::mt19937 rng(5);
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
      const dyn4_spin_state<double> s = spin_state4(p, lim, g, rng);
      dyn4_spin_state<double> o;
      diff::dyn4_spin_plant<double>(p, lim, g, s, s.b.delta, s.b.a, o);
      double x0[13], x1[13], f[13];
      diff::dyn4_spin_ph_coords(p, s, x0);
      diff::dyn4_spin_ph_coords(p, o, x1);
      diff::dyn4_spin_ph_rhs(p, lim, g, s, f);
      for (int a = 0; a < 13; ++a) {
        const double fd = (x1[a] - x0[a]) / lim.hdt;
        const double rel = std::fabs(fd - f[a]) / scale[a];
        worst = std::max(worst, rel);
        EXPECT_LT(rel, 1e-3) << "mu " << mu << ", coordinate " << a << ": plant " << fd << ", pH "
                             << f[a] << " at vx " << s.b.vx;
      }
    }
    std::printf("  vector field vs dyn4_spin_plant at every slip, mu %.1f: worst %.2e of its "
                "scale\n",
                mu, worst);
  }
}

namespace {

// dyn4_spin_plant_books along book_run's trajectories (a launch that spins and a stop that locks,
// or the weave past the lateral peak), on flat ground of friction mu.
run_books book_run4(const dyn4_params &p, double mu, double hdt, bool launch, double seconds) {
  bike_veh lim = limits4(p);
  lim.hdt = static_cast<float>(hdt);
  diff::flat_ground g;
  g.mu = static_cast<float>(mu);
  dyn4_spin_state<double> s = rolling4(p, launch ? 2.0 : 15.0);
  const int n = static_cast<int>(std::lround(seconds / hdt));
  run_books rb;
  const double h0 = diff::dyn4_spin_ph_hamiltonian(p, lim, g, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd = launch ? 0.1 : 0.3 * std::sin(kTwoPi * 0.7 * time);
    const double acmd = launch ? (time < 2.0 ? 4.0 : -9.0) : 1.5 * std::sin(kTwoPi * time / 3.0);
    dyn4_spin_state<double> o;
    diff::dyn4_spin_ph_books<double> b;
    diff::dyn4_spin_plant_books<double>(p, lim, g, s, dcmd, acmd, o, b);
    rb.mean_abs_residual += std::fabs(b.residual);
    rb.residual += b.residual;
    rb.supplied += b.supplied;
    rb.braked += b.braked;
    rb.slip += b.slip;
    rb.dissipated += b.dissipated;
    rb.limiter += b.limiter;
    rb.flow += std::fabs(b.supplied) + b.braked + b.slip + std::fabs(b.dissipated) +
               std::fabs(b.lever) + std::fabs(b.grade) + std::fabs(b.ground) + std::fabs(b.limiter);
    diff::dyn4_spin_ph_parts<double> q;
    diff::dyn4_spin_ph_parts_at(p, lim, g, o, q);
    const double e2 = double(p.eps) * p.eps;
    for (int i = 0; i < 4; ++i)
      rb.peak_kappa =
          std::max(rb.peak_kappa, std::fabs(q.sl.sig[i]) / std::sqrt(q.sl.g[i] * q.sl.g[i] + e2));
    s = o;
  }
  rb.mean_abs_residual /= n;
  rb.dH = diff::dyn4_spin_ph_hamiltonian(p, lim, g, s) - h0;
  return rb;
}

} // namespace

// The four-wheel gate: every step's books close but for the integrator's error, second order per
// step (faster at the coarse steps, as DynSpinPH's), through the weave past the lateral peak and
// through a launch that spins every wheel and a stop that locks them.
TEST(Dyn4SpinPH, StepBooksCloseToSecondOrder) {
  const dyn4_params p = humvee4();
  struct scenario {
    const char *name;
    double mu;
    bool launch;
    double seconds;
  };
  for (const scenario &sc : {scenario{"swerve past the peak, mu 0.6", 0.6, false, 6.0},
                             scenario{"launch and lock, mu 0.3", 0.3, true, 4.0}}) {
    double prev = 0.0;
    for (const double hdt : {0.01, 0.005, 0.0025, 0.00125}) {
      const run_books rb = book_run4(p, sc.mu, hdt, sc.launch, sc.seconds);
      std::printf("  %s, hdt %.4f s: dH %.4g J, drive %.4g, brakes %.4g, slip %.4g, tires, drag "
                  "and dampers %.4g, limiter %.3g; peak |kappa| %.2f; mean |residual| %.3e J, "
                  "|sum| / flow %.2e\n",
                  sc.name, hdt, rb.dH, rb.supplied, rb.braked, rb.slip, rb.dissipated, rb.limiter,
                  rb.peak_kappa, rb.mean_abs_residual, std::fabs(rb.residual) / rb.flow);
      if (prev > 0.0) {
        const double ratio = prev / rb.mean_abs_residual;
        EXPECT_GT(ratio, 3.0) << sc.name << ", hdt " << hdt;
        EXPECT_LT(ratio, sc.launch ? 10.0 : 5.5) << sc.name << ", hdt " << hdt;
      }
      prev = rb.mean_abs_residual;
      EXPECT_LT(std::fabs(rb.residual) / rb.flow, 5e-3) << sc.name << ", hdt " << hdt;
      if (sc.launch)
        EXPECT_GT(rb.peak_kappa, 0.5) << "the wheels neither spun nor locked";
    }
  }
}

// The four-wheel wheel-spin plant's vector-Jacobian product against central differences.
TEST(Dyn4Spin, PlantGradientMatchesFiniteDifference) {
  const dyn4_params p = humvee4();
  bike_veh lim = limits4(p, true);
  lim.hdt = 0.02f;
  diff::flat_ground g;
  g.mu = 0.6f;
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  const int n = dyn4_spin_state<double>::size;
  double worst = 0.0;
  for (int k = 0; k < 20; ++k) {
    const dyn4_spin_state<double> s = spin_state4(p, lim, g, rng);
    const double dcmd = s.b.delta + 0.1 * u(rng);
    const double acmd = (s.b.a >= 0.0 ? 1.0 : -1.0) * (1.0 + 4.0 * std::fabs(u(rng)));
    dyn4_spin_state<double> g_o, g_s;
    for (int i = 0; i < n; ++i)
      g_o[i] = u(rng);
    double g_dc = 0.0, g_ac = 0.0;
    diff::dyn4_spin_plant_vjp<double>(p, lim, g, s, dcmd, acmd, g_o, g_s, g_dc, g_ac);
    const auto loss = [&](const dyn4_spin_state<double> &x, double dc, double ac) {
      dyn4_spin_state<double> o;
      diff::dyn4_spin_plant<double>(p, lim, g, x, dc, ac, o);
      double acc = 0.0;
      for (int i = 0; i < n; ++i)
        acc += g_o[i] * o[i];
      return acc;
    };
    for (int j = 0; j < n + 2; ++j) {
      const double h = 1e-6;
      dyn4_spin_state<double> sp = s, sm = s;
      double dcp = dcmd, dcm = dcmd, acp = acmd, acm = acmd;
      if (j < n) {
        sp[j] += h;
        sm[j] -= h;
      } else if (j == n) {
        dcp += h;
        dcm -= h;
      } else {
        acp += h;
        acm -= h;
      }
      const double fd = (loss(sp, dcp, acp) - loss(sm, dcm, acm)) / (2.0 * h);
      const double an = j < n ? g_s[j] : (j == n ? g_dc : g_ac);
      const double rel = std::fabs(fd - an) / (1.0 + std::fabs(fd));
      worst = std::max(worst, rel);
      EXPECT_LT(rel, 1e-5) << "state " << k << ", input " << j << ": duals " << an << ", fd " << fd;
    }
  }
  std::printf("  dyn4_spin_plant_vjp vs central differences: worst %.2e\n", worst);
}

// The values GRL-SNAM's torch twin of the four-wheel wheel-spin plant is held to: the Humvee with
// its springs on flat ground (mu 0.9), Dyn4.Golden's lags. Per case: the wheels' slips and the
// rates at its state, one dyn4_spin_plant_books step with its books, and the gradient of a fixed
// weighting of the books on the state and the commands, by forward-mode duals.
// CVC_DYN4_SPIN_PH_GOLDEN_WRITE=1 rewrites tests/data/dyn4_spin_ph_golden.json; otherwise this
// build must reproduce it to 1e-9.
TEST(Dyn4SpinPH, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn4_spin_ph_golden.json";
  dyn4_params p = humvee4();
  p.tau_steer = 0.05f;
  p.v_blend = 1.0f;
  p.v_width = 0.3f;
  diff::flat_ground g; // mu 0.9
  struct gcase {
    const char *name;
    double s[18], dcmd, acmd;
    bool reverse;
  };
  // x, y, psi, vx, vy, r, z, zd, phi, phid, th, thd, delta, a, then the four wheels' speeds.
  const gcase cases[] = {
      {"cruising",
       {0, 0, 0, 12, 0.1, 0.05, 0.005, 0.01, 0.002, 0.02, -0.003, 0.01, 0.02, 0.5, 25.6, 25.5, 25.6,
        25.55},
       0.05,
       1.0,
       false},
      {"hard corner",
       {1, -2, 0.3, 15, -1.2, 0.6, 0.0, -0.05, 0.02, 0.1, 0.0, 0.0, 0.25, 0.0, 33.0, 31.0, 32.5,
        30.5},
       0.3,
       0.0,
       false},
      {"trail braking, the fronts locking",
       {-3, 4, -1.1, 14, 0.8, 0.4, -0.01, 0.0, 0.01, 0.0, 0.02, 0.05, 0.15, -4.0, 4.0, 6.0, 28.0,
        27.0},
       0.15,
       -6.0,
       false},
      {"wheelspin launch",
       {0, 0, 0, 2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0.05, 3.0, 15.0, 14.0, 18.0, 17.5},
       0.05,
       4.0,
       false},
      {"locked skid",
       {0, 0, 0, 8.0, 0.3, 0.1, 0, 0, 0, 0, 0.01, 0, 0.1, -5.0, 0, 0, 0, 0},
       0.1,
       -7.0,
       false},
      {"pulling away", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0.1, 2.0, false},
      {"braking backward",
       {0.5, 0.5, 0.2, -2.0, 0.05, -0.1, 0, 0, 0, 0, 0, 0, 0.2, -0.5, -4.2, -4.3, -4.25, -4.3},
       0.25,
       -0.8,
       true},
  };
  const char *const rate_keys[] = {"dissipated", "slip", "supplied", "braked", "lever", "grade"};
  const char *const free_keys[] = {"vx", "vy", "r"};
  const char *const book_keys[] = {"dH",    "dissipated", "slip",   "supplied", "braked",
                                   "lever", "grade",      "ground", "limiter",  "residual"};
  const double g_books[10] = {0.9, -0.4, 0.3, 0.7, -0.5, 1.3, -0.6, 0.4, 0.8, 1.1};
  golden_writer w;
  std::ostringstream &js = w.js;
  js << "{\n \"format\": \"cvc-dyn4-spin-ph-golden-1\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"z\", \"zd\", \"phi\", "
        "\"phid\", \"th\", \"thd\", \"delta\", \"a\", \"om_fl\", \"om_fr\", \"om_rl\", "
        "\"om_rr\"],\n";
  js << " \"ground\": {\"mu\": " << golden_writer::num(g.mu) << "},\n \"params\": {";
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
                                                   {"fz_min", p.fz_min},
                                                   {"drive_gain", p.drive_gain},
                                                   {"brake_gain", p.brake_gain},
                                                   {"resist_comp", p.resist_comp},
                                                   {"r_w", p.r_w},
                                                   {"J_f", p.J_f},
                                                   {"J_r", p.J_r},
                                                   {"B_x", p.B_x},
                                                   {"C_x", p.C_x}};
  bool first = true;
  for (const auto &kv : params) {
    js << (first ? "" : ", ") << "\"" << kv.first << "\": " << golden_writer::num(kv.second);
    first = false;
  }
  js << ", \"iters\": " << p.iters << ", \"grade_force\": " << p.grade_force
     << "},\n \"g_books\": " << golden_writer::arr(g_books, 10) << ",\n \"cases\": [\n";
  const int ncase = static_cast<int>(sizeof cases / sizeof cases[0]);
  for (int c = 0; c < ncase; ++c) {
    const gcase &gc = cases[c];
    const bike_veh lim = limits4(p, gc.reverse);
    dyn4_spin_state<double> s;
    for (int k = 0; k < 18; ++k)
      s[k] = gc.s[k];
    diff::dyn4_spin_ph_parts<double> q;
    diff::dyn4_spin_ph_parts_at(p, lim, g, s, q);
    double slips[24];
    for (int i = 0; i < 4; ++i) {
      slips[i] = q.b.Fx[i];
      slips[4 + i] = q.b.Fy[i];
      slips[8 + i] = q.sl.g[i];
      slips[12 + i] = q.sl.sig[i];
      slips[16 + i] = q.sl.K[i];
      slips[20 + i] = q.u[i];
    }
    const diff::dyn4_spin_ph_rates<double> rt = diff::dyn4_spin_ph_rates_at(p, lim, g, s);
    const double rates[] = {rt.dissipated, rt.slip, rt.supplied, rt.braked, rt.lever, rt.grade};
    dyn4_spin_state<double> o, o_books;
    diff::dyn_step_free<double> seen;
    diff::dyn4_spin_plant<double>(p, lim, g, s, gc.dcmd, gc.acmd, o, &seen);
    diff::dyn4_spin_ph_books<double> b;
    diff::dyn4_spin_plant_books<double>(p, lim, g, s, gc.dcmd, gc.acmd, o_books, b);
    for (int k = 0; k < 18; ++k)
      EXPECT_EQ(o_books[k], o[k]) << gc.name << ": the books' step is not dyn4_spin_plant's";
    double next[18];
    for (int k = 0; k < 18; ++k)
      next[k] = o[k];
    const double twist[] = {seen.vx, seen.vy, seen.r};
    const double books[] = {b.dH,    b.dissipated, b.slip,   b.supplied, b.braked,
                            b.lever, b.grade,      b.ground, b.limiter,  b.residual};
    typedef diff::dual<double> D;
    double grad[20];
    for (int j = 0; j < 20; ++j) {
      dyn4_spin_state<D> ds, od;
      for (int k = 0; k < 18; ++k)
        ds[k] = D(gc.s[k], k == j ? 1.0 : 0.0);
      diff::dyn4_spin_ph_books<D> bd;
      diff::dyn4_spin_plant_books<D>(p, lim, g, ds, D(gc.dcmd, j == 18 ? 1.0 : 0.0),
                                     D(gc.acmd, j == 19 ? 1.0 : 0.0), od, bd);
      const D bv[] = {bd.dH,    bd.dissipated, bd.slip,   bd.supplied, bd.braked,
                      bd.lever, bd.grade,      bd.ground, bd.limiter,  bd.residual};
      grad[j] = 0.0;
      for (int k = 0; k < 10; ++k)
        grad[j] += g_books[k] * bv[k].d;
    }
    w.record(gc.name, "wheels", nullptr, slips, 24);
    w.record(gc.name, "rates", rate_keys, rates, 6);
    w.record(gc.name, "next", nullptr, next, 18);
    w.record(gc.name, "free", free_keys, twist, 3);
    w.record(gc.name, "books", book_keys, books, 10);
    w.record(gc.name, "g_state", nullptr, grad, 18);
    w.record(gc.name, "g_delta_cmd", nullptr, grad + 18, 1);
    w.record(gc.name, "g_a_cmd", nullptr, grad + 19, 1);
    js << "  {\"name\": \"" << gc.name << "\", \"state\": " << golden_writer::arr(gc.s, 18)
       << ", \"delta_cmd\": " << golden_writer::num(gc.dcmd)
       << ", \"a_cmd\": " << golden_writer::num(gc.acmd)
       << ",\n   \"limits\": {\"L\": " << golden_writer::num(lim.L)
       << ", \"vmax\": " << golden_writer::num(lim.vmax)
       << ", \"hdt\": " << golden_writer::num(lim.hdt)
       << ", \"allow_reverse\": " << lim.allow_reverse << "}"
       << ",\n   \"wheels\": {\"Fx\": " << golden_writer::arr(slips, 4)
       << ", \"Fy\": " << golden_writer::arr(slips + 4, 4)
       << ", \"ground\": " << golden_writer::arr(slips + 8, 4)
       << ", \"sigma\": " << golden_writer::arr(slips + 12, 4)
       << ", \"K\": " << golden_writer::arr(slips + 16, 4)
       << ", \"u\": " << golden_writer::arr(slips + 20, 4) << "}"
       << ",\n   \"rates\": " << golden_writer::obj(rate_keys, rates, 6)
       << ",\n   \"next\": " << golden_writer::arr(next, 18)
       << ",\n   \"free\": " << golden_writer::obj(free_keys, twist, 3)
       << ",\n   \"books\": " << golden_writer::obj(book_keys, books, 10)
       << ",\n   \"g_state\": " << golden_writer::arr(grad, 18)
       << ",\n   \"g_delta_cmd\": " << golden_writer::num(grad[18])
       << ", \"g_a_cmd\": " << golden_writer::num(grad[19]) << "}" << (c + 1 < ncase ? "," : "")
       << "\n";
  }
  js << " ]\n}\n";
  const char *write = std::getenv("CVC_DYN4_SPIN_PH_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  check_golden(path, w, [&](const boost::property_tree::ptree &c, std::vector<double> &out) {
    for (const char *k :
         {"wheels.Fx", "wheels.Fy", "wheels.ground", "wheels.sigma", "wheels.K", "wheels.u"})
      read_listed(c, k, out);
    read_named(c, "rates", rate_keys, 6, out);
    read_listed(c, "next", out);
    read_named(c, "free", free_keys, 3, out);
    read_named(c, "books", book_keys, 10, out);
    read_listed(c, "g_state", out);
    out.push_back(c.get<double>("g_delta_cmd"));
    out.push_back(c.get<double>("g_a_cmd"));
  });
}
