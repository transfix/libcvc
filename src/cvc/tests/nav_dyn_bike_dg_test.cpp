/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_dyn_bike_dg_test: the discrete-gradient step of the dynamic single-track vehicle
// (diff::dyn_plant_dg, detail/dyn_bike_dg.h) against dyn_plant and its port-Hamiltonian form:
//   - its momenta's rate is the form's vector field, and its actuators are dyn_plant's, to the bit;
//   - its books close to the solver's tolerance at every hdt, sliding past the tires' peak and
//     braking to a stop through the limiters, where dyn_plant's close to second order per step;
//   - it is second order where dyn_plant is first;
//   - coasting, it never gains energy, at nav's 0.06 s step and the live plant's 6 x 0.01 s
//     substeps, with the Humvee's tires and much stiffer ones;
//   - what a step costs against dyn_plant's.
// And with the wheels spinning (diff::dyn_spin_plant_dg) against dyn_spin_plant:
//   - its rate is dyn_spin_ph_rhs's, Q from the hook dyn_spin_ph_forces_at with and without a
//     wrench, and its actuators are dyn_spin_plant's, to the bit;
//   - its books close to the solver's tolerance through spinning, locked and sliding wheels and an
//     external wrench, with the wheels' rows at the midpoint and at the step's end;
//   - second order with the wheels at the midpoint, first with them at the end, and nearer than
//     dyn_spin_plant either way;
//   - the wheels' stiff slip: locking, spinning and coasting at 0.06 s and 0.01 s, the slip's
//     swings against dyn_spin_plant's, and no energy gained coasting;
//   - what a step costs against dyn_spin_plant's, and where its Newton steps go.
// The Humvee from its vehicle_desc, in metres (S = 1), as nav_dyn_bike_test drives it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/nav/detail/dyn_bike_dg.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/vehicle.h>
#include <gtest/gtest.h>
#include <random>
#include <vector>

using cvc::nav::diff::bike_veh;
using cvc::nav::diff::dyn_bike_params;
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

bike_veh plant_limits(const dyn_bike_params &p, double hdt = 0.06, bool reverse = false) {
  bike_veh v{};
  v.L = p.lf + p.lr;
  v.vmax = 40.0f;
  v.delta_max = 0.6f;
  v.hdt = static_cast<float>(hdt);
  v.allow_reverse = reverse ? 1 : 0;
  return v;
}

// A Humvee state anywhere in the envelope (nav_dyn_bike_test's): forward or reversing, the wheel up
// to 0.5 rad, and sideways and yaw rates well past the tires' peaks.
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

// One step of either scheme, with its books.
void step_books(bool dg, const dyn_bike_params &p, const bike_veh &lim, const dyn_state<double> &s,
                double dcmd, double acmd, double grade, dyn_state<double> &o,
                diff::dyn_ph_books<double> &b, diff::dyn_dg_solve *rep = nullptr) {
  if (dg)
    diff::dyn_plant_dg_books<double>(p, lim, s, dcmd, acmd, grade, o, b, rep);
  else
    diff::dyn_plant_books<double>(p, lim, s, dcmd, acmd, grade, o, b);
}

// One run's books: per-step mean and worst |residual|, the sums, and the solves.
struct run_books {
  double mean_abs_residual = 0.0, worst_abs_residual = 0.0, residual = 0.0, flow = 0.0;
  double dH = 0.0, limiter = 0.0, dissipated = 0.0, supplied = 0.0;
  int steps = 0, unconverged = 0, max_iters = 0;
  double mean_iters = 0.0;
};

// nav_dyn_bike_test's book_run: `swerve` weaves at 0.7 Hz past the peak with the throttle working;
// otherwise the Humvee brakes hard to a stop in a turn, so the speed clamp and the blend act.
run_books book_run(bool dg, const dyn_bike_params &p, double hdt, bool swerve, double seconds) {
  const bike_veh lim = plant_limits(p, hdt);
  dyn_state<double> s{};
  s.vx = swerve ? 15.0 : 9.0;
  const int n = static_cast<int>(std::lround(seconds / hdt));
  run_books rb;
  const double h0 = diff::dyn_ph_hamiltonian(p, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd = swerve ? 0.3 * std::sin(kTwoPi * 0.7 * time) : 0.35;
    const double acmd = swerve ? 1.5 * std::sin(kTwoPi * time / 3.0) : -6.0;
    dyn_state<double> o;
    diff::dyn_ph_books<double> b;
    diff::dyn_dg_solve rep;
    step_books(dg, p, lim, s, dcmd, acmd, 0.0, o, b, &rep);
    rb.mean_abs_residual += std::fabs(b.residual);
    rb.worst_abs_residual = std::max(rb.worst_abs_residual, std::fabs(b.residual));
    rb.residual += b.residual;
    rb.limiter += b.limiter;
    rb.dissipated += b.dissipated;
    rb.supplied += b.supplied;
    rb.flow += std::fabs(b.dissipated) + std::fabs(b.supplied) + std::fabs(b.floor) +
               std::fabs(b.grade) + std::fabs(b.limiter);
    if (dg) {
      rb.unconverged += !rep.converged;
      rb.max_iters = std::max(rb.max_iters, rep.iters);
      rb.mean_iters += rep.iters;
    }
    s = o;
  }
  rb.steps = n;
  rb.mean_abs_residual /= n;
  rb.mean_iters /= n;
  rb.dH = diff::dyn_ph_hamiltonian(p, s) - h0;
  return rb;
}

} // namespace

// The momenta's rate the step solves with, J v + Q from one dyn_ph_parts_at, is dyn_ph_rhs's at
// every slip, forward and reversing, flat and on a 3% grade.
TEST(DynBikeDG, TheRateIsThePortHamiltonianField) {
  for (const double mu : {0.9, 0.6}) {
    const dyn_bike_params p = humvee(mu);
    std::mt19937 rng(5);
    const double scale[3] = {p.m * p.g, p.m * p.g, p.m * p.g * (p.lf + p.lr)};
    double worst = 0.0;
    for (int k = 0; k < 400; ++k) {
      const dyn_state<double> s = sliding_state(rng);
      const double grade = (k % 2) * 0.03;
      double f[3], g[3];
      diff::dyn_dg_rate(p, s, grade, f);
      diff::dyn_ph_rhs(p, s, grade, g);
      for (int i = 0; i < 3; ++i)
        worst = std::max(worst, std::fabs(f[i] - g[i]) / scale[i]);
    }
    std::printf("  mu %.1f: worst |rate - dyn_ph_rhs| = %.2e of m g\n", mu, worst);
    EXPECT_LT(worst, 1e-12);
  }
}

// Its actuators are dyn_plant's step 1 to the bit, in double and in float, lags and slew limit on.
TEST(DynBikeDG, TheActuatorsAreDynPlants) {
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.steer_rate = 0.6f;
  p.tau_drive = 0.1f;
  const bike_veh lim = plant_limits(p);
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int k = 0; k < 200; ++k) {
    const dyn_state<double> s = sliding_state(rng);
    const double dcmd = 0.6 * u(rng), acmd = 6.0 * u(rng);
    dyn_state<double> a, b;
    diff::dyn_plant<double>(p, lim, s, dcmd, acmd, 0.0, a);
    diff::dyn_plant_dg<double>(p, lim, s, dcmd, acmd, 0.0, b);
    EXPECT_EQ(a.delta, b.delta);
    EXPECT_EQ(a.a, b.a);
    dyn_state<float> sf, af, bf;
    for (int i = 0; i < dyn_state<float>::size; ++i)
      sf[i] = static_cast<float>(s[i]);
    diff::dyn_plant<float>(p, lim, sf, (float)dcmd, (float)acmd, 0.0f, af);
    diff::dyn_plant_dg<float>(p, lim, sf, (float)dcmd, (float)acmd, 0.0f, bf);
    EXPECT_EQ(af.delta, bf.delta);
    EXPECT_EQ(af.a, bf.a);
  }
}

// The gate (DynBikePH.StepBooksCloseToSecondOrder's runs): every step's books close to the
// solver's tolerance, at nav's 0.06 s step and finer, sliding past the peak with the throttle
// working and braking to a stop through the clamp and the blend. dyn_plant's residual on the same
// runs is printed beside it.
TEST(DynBikeDG, StepBooksCloseExactly) {
  const dyn_bike_params p = humvee(0.6);
  for (const double hdt : {0.06, 0.02, 0.01, 0.005, 0.0025}) {
    const run_books dg = book_run(true, p, hdt, true, 6.0);
    const run_books pl = book_run(false, p, hdt, true, 6.0);
    const double per_step_flow = dg.flow / dg.steps;
    std::printf("  swerve past the peak, hdt %.4f s: discrete gradient worst |residual| %.2e J "
                "(%.1e of the step's flow), Newton %.2f steps (max %d); dyn_plant mean "
                "|residual| %.3e J, |sum| / flow %.2e\n",
                hdt, dg.worst_abs_residual, dg.worst_abs_residual / per_step_flow, dg.mean_iters,
                dg.max_iters, pl.mean_abs_residual, std::fabs(pl.residual) / pl.flow);
    EXPECT_EQ(dg.unconverged, 0) << "hdt " << hdt;
    EXPECT_LT(dg.worst_abs_residual / per_step_flow, 1e-10) << "hdt " << hdt;
    EXPECT_LT(dg.mean_abs_residual, 1e-6 * pl.mean_abs_residual) << "hdt " << hdt;
  }
  for (const double hdt : {0.06, 0.01, 0.005}) {
    const run_books dg = book_run(true, p, hdt, false, 3.0);
    const double per_step_flow = dg.flow / dg.steps;
    std::printf("  braking to a stop, hdt %.4f s: dH %.4g J, limiters %.4g J, worst |residual| "
                "%.2e J (%.1e of the step's flow)\n",
                hdt, dg.dH, dg.limiter, dg.worst_abs_residual,
                dg.worst_abs_residual / per_step_flow);
    EXPECT_EQ(dg.unconverged, 0) << "hdt " << hdt;
    EXPECT_LT(dg.limiter, 0.0) << "the limiters did not act";
    EXPECT_LT(dg.worst_abs_residual / per_step_flow, 1e-10) << "hdt " << hdt;
  }
}

// Second order where dyn_plant is first: a turn-in past the peak from 15 m/s on constant commands
// (so the actuators' exact lags add no error of their own), the twist and pose at 2 s against each
// scheme's own solution at hdt / 64.
TEST(DynBikeDG, SecondOrderWhereDynPlantIsFirst) {
  dyn_bike_params p = humvee(0.6);
  p.v_blend = 0.0f;
  const double T = 2.0;
  for (const bool dg : {true, false}) {
    auto run = [&](double hdt) {
      const bike_veh lim = plant_limits(p, hdt);
      dyn_state<double> s{};
      s.vx = 15.0;
      const int n = static_cast<int>(std::lround(T / hdt));
      for (int t = 0; t < n; ++t) {
        dyn_state<double> o;
        if (dg)
          diff::dyn_plant_dg<double>(p, lim, s, 0.25, 1.0, 0.0, o);
        else
          diff::dyn_plant<double>(p, lim, s, 0.25, 1.0, 0.0, o);
        s = o;
      }
      return s;
    };
    const double h_ref = 0.04 / 64.0;
    const dyn_state<double> ref = run(h_ref);
    double prev = 0.0;
    for (const double hdt : {0.04, 0.02, 0.01, 0.005}) {
      const dyn_state<double> s = run(hdt);
      const double L = p.lf + p.lr;
      const double err = std::fabs(s.vx - ref.vx) + std::fabs(s.vy - ref.vy) +
                         L * std::fabs(s.r - ref.r) + std::fabs(s.x - ref.x) / T +
                         std::fabs(s.y - ref.y) / T;
      std::printf("  %s, hdt %.3f s: error at %.0f s %.3e%s",
                  dg ? "discrete gradient" : "dyn_plant", hdt, T, err, prev > 0.0 ? "" : "\n");
      if (prev > 0.0) {
        const double ratio = prev / err;
        std::printf(" (%.2fx smaller)\n", ratio);
        if (dg) {
          EXPECT_GT(ratio, 3.4) << "hdt " << hdt;
          EXPECT_LT(ratio, 4.6) << "hdt " << hdt;
        } else {
          EXPECT_LT(ratio, 2.6) << "hdt " << hdt;
        }
      }
      prev = err;
    }
  }
}

namespace {

// A coasting run's energy: the axles carry no force (a = 0), the ground is flat and the blend is
// off, so the solved twist may only lose energy, every step. Also counts the rear slip's swings
// past its root (sign changes of its sideways velocity) and the steps' Newton work.
struct coast_run {
  double worst_gain = 0.0; // the largest dH over H, any step
  double H0 = 0.0, H1 = 0.0;
  int swings = 0, steps = 0, unconverged = 0, max_iters = 0;
  bool finite = true;
};

coast_run coast(bool dg, const dyn_bike_params &p, dyn_state<double> s, double hdt, int ticks,
                int substeps, double delta) {
  const bike_veh lim = plant_limits(p, hdt, true);
  coast_run c;
  c.H0 = diff::dyn_ph_hamiltonian(p, s);
  double wr_prev = s.vy - p.lr * s.r;
  for (int t = 0; t < ticks * substeps; ++t) {
    dyn_state<double> o;
    diff::dyn_ph_books<double> b;
    diff::dyn_dg_solve rep;
    step_books(dg, p, lim, s, delta, 0.0, 0.0, o, b, &rep);
    const double H = diff::dyn_ph_hamiltonian(p, s);
    c.worst_gain = std::max(c.worst_gain, (b.dH - b.limiter) / std::max(H, 1e-12));
    if (dg) {
      c.unconverged += !rep.converged;
      c.max_iters = std::max(c.max_iters, rep.iters);
    }
    // A swing: the slip's sideways velocity changes sign by more than 1 mm/s either side.
    const double wr = o.vy - p.lr * o.r;
    if (std::fabs(wr) > 1e-3) {
      c.swings += std::fabs(wr_prev) > 1e-3 && (wr > 0.0) != (wr_prev > 0.0);
      wr_prev = wr;
    }
    for (int i = 0; i < dyn_state<double>::size; ++i)
      c.finite = c.finite && std::isfinite(o[i]);
    s = o;
    ++c.steps;
  }
  c.H1 = diff::dyn_ph_hamiltonian(p, s);
  return c;
}

} // namespace

// Stiff tires: the Humvee's, 4x and 16x stiffer (Pacejka B), at nav's single 0.06 s step and the
// live plant's 6 x 0.01 s substeps, coasting at full lock for 6 s from a crawl to a slide, with the
// low-speed blend off (the gate) and on (the plant as it runs). Every step solves, stays finite and
// never gains energy (dH <= limiter to round-off, the ports supplying nothing). Where each run ends
// is printed against the same scheme at 0.5 ms, beside dyn_plant's. The midpoint rule is A-stable
// but not L-stable: a tire far stiffer than the step swings its slip past zero and back, decaying,
// where dyn_plant's implicit Euler does not, and the swings are printed against dyn_plant's.
TEST(DynBikeDG, StiffTires) {
  for (const bool blend : {false, true})
    for (const double stiff : {1.0, 4.0, 16.0}) {
      dyn_bike_params p = humvee(0.9);
      p.B_f = static_cast<float>(p.B_f * stiff);
      p.B_r = static_cast<float>(p.B_r * stiff);
      if (!blend)
        p.v_blend = 0.0f;
      for (const double v0 : {0.5, 3.0, 12.0}) {
        dyn_state<double> s{};
        s.vx = v0;
        s.vy = 0.4 * v0; // sliding sideways into the turn
        s.r = -0.3;
        s.delta = 0.6;
        const coast_run ref = coast(true, p, s, 0.0005, 12000, 1, 0.6);
        const coast_run pref = coast(false, p, s, 0.0005, 12000, 1, 0.6);
        for (const int sub : {1, 6}) {
          const double hdt = 0.06 / sub;
          const coast_run dg = coast(true, p, s, hdt, 100, sub, 0.6);
          const coast_run pl = coast(false, p, s, hdt, 100, sub, 0.6);
          std::printf("  blend %s, tires x%-2.0f, %d x %.2f s, from %4.1f m/s: H %.4g -> %.4g J "
                      "(0.5 ms: %.4g; dyn_plant %.4g, 0.5 ms: %.4g), worst step gain %+.1e of H, "
                      "slip swings %d (dyn_plant %d), Newton max %d\n",
                      blend ? "on " : "off", stiff, sub, hdt, v0, dg.H0, dg.H1, ref.H1, pl.H1,
                      pref.H1, dg.worst_gain, dg.swings, pl.swings, dg.max_iters);
          EXPECT_TRUE(dg.finite);
          EXPECT_EQ(dg.unconverged, 0);
          if (!blend) {
            EXPECT_LT(dg.worst_gain, 1e-12)
                << "tires x" << stiff << ", " << sub << " substeps, " << v0 << " m/s";
            EXPECT_LE(dg.H1, dg.H0);
          }
        }
      }
    }
}

// Stiff tires reach the kinematic bicycle in a steady turn, as dyn_plant's do (DynBike's gate 3):
// 16x the Humvee's tires, a slow 5 m/s turn held by the throttle, both step configurations. The
// midpoint rule's fixed point is the vector field's, so the yaw rate settles on vx tan(delta) / L
// to the tires' compliance.
TEST(DynBikeDG, StiffTiresReachTheKinematicBicycle) {
  dyn_bike_params p = humvee(0.9);
  p.B_f *= 16.0f;
  p.B_r *= 16.0f;
  const double L = p.lf + p.lr, delta = 0.2, v = 5.0;
  for (const bool dg : {true, false})
    for (const int sub : {1, 6}) {
      const bike_veh lim = plant_limits(p, 0.06 / sub);
      dyn_state<double> s{};
      s.vx = v;
      for (int t = 0; t < 200 * sub; ++t) {
        dyn_state<double> o;
        const double a_cmd = 2.0 * (v - s.vx) + p.drag * s.vx * s.vx / p.m + p.c_rr * p.g;
        if (dg)
          diff::dyn_plant_dg<double>(p, lim, s, delta, a_cmd, 0.0, o);
        else
          diff::dyn_plant<double>(p, lim, s, delta, a_cmd, 0.0, o);
        s = o;
      }
      const double r_k = s.vx * std::tan(delta) / L;
      std::printf("  %s, %d x %.2f s: after 12 s r %.5f rad/s against the kinematic %.5f "
                  "(%.2f%%), vy %.4f m/s\n",
                  dg ? "discrete gradient" : "dyn_plant", sub, 0.06 / sub, s.r, r_k,
                  100.0 * (s.r - r_k) / r_k, s.vy);
      EXPECT_NEAR(s.r, r_k, 0.02 * r_k);
    }
}

// What a step costs: the discrete-gradient step and its books against dyn_plant and its books, in
// double and float, on the swerve past the peak at nav's 0.06 s and the live plant's 0.01 s.
TEST(DynBikeDG, CostPerStep) {
  const dyn_bike_params p = humvee(0.6);
  for (const double hdt : {0.06, 0.01}) {
    const bike_veh lim = plant_limits(p, hdt);
    const int n = 20000;
    std::vector<double> dcmd(n), acmd(n);
    for (int t = 0; t < n; ++t) {
      dcmd[t] = 0.3 * std::sin(kTwoPi * 0.7 * t * hdt);
      acmd[t] = 1.5 * std::sin(kTwoPi * t * hdt / 3.0);
    }
    auto time_it = [&](auto &&stepper) {
      dyn_state<double> s{};
      s.vx = 15.0;
      double sink = 0.0;
      const auto t0 = std::chrono::steady_clock::now();
      for (int t = 0; t < n; ++t) {
        dyn_state<double> o;
        stepper(s, dcmd[t], acmd[t], o);
        s = o;
        sink += o.vx;
      }
      const auto t1 = std::chrono::steady_clock::now();
      EXPECT_TRUE(std::isfinite(sink));
      return std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
    };
    const double plant =
        time_it([&](const dyn_state<double> &s, double d, double a, dyn_state<double> &o) {
          diff::dyn_plant<double>(p, lim, s, d, a, 0.0, o);
        });
    const double dg =
        time_it([&](const dyn_state<double> &s, double d, double a, dyn_state<double> &o) {
          diff::dyn_plant_dg<double>(p, lim, s, d, a, 0.0, o);
        });
    const double plant_books =
        time_it([&](const dyn_state<double> &s, double d, double a, dyn_state<double> &o) {
          diff::dyn_ph_books<double> b;
          diff::dyn_plant_books<double>(p, lim, s, d, a, 0.0, o, b);
        });
    const double dg_books =
        time_it([&](const dyn_state<double> &s, double d, double a, dyn_state<double> &o) {
          diff::dyn_ph_books<double> b;
          diff::dyn_plant_dg_books<double>(p, lim, s, d, a, 0.0, o, b);
        });
    const double dg_float =
        time_it([&](const dyn_state<double> &s, double d, double a, dyn_state<double> &o) {
          dyn_state<float> sf, of;
          for (int i = 0; i < dyn_state<float>::size; ++i)
            sf[i] = static_cast<float>(s[i]);
          diff::dyn_plant_dg<float>(p, lim, sf, (float)d, (float)a, 0.0f, of);
          for (int i = 0; i < dyn_state<float>::size; ++i)
            o[i] = of[i];
        });
    std::printf("  hdt %.2f s, per step: dyn_plant %.0f ns, discrete gradient %.0f ns (%.1fx); "
                "with books %.0f ns vs %.0f ns (%.1fx); discrete gradient in float %.0f ns\n",
                hdt, plant, dg, dg / plant, plant_books, dg_books, dg_books / plant_books,
                dg_float);
    EXPECT_GT(dg, 0.0);
  }
}

// ── With the wheels spinning: diff::dyn_spin_plant_dg ─────────────────────────────────────

using cvc::nav::diff::dyn_spin_state;

namespace {

// Rolling at speed vx: the wheels' rims at the ground's speed.
dyn_spin_state<double> rolling(const dyn_bike_params &p, double vx) {
  dyn_spin_state<double> s{};
  s.b.vx = vx;
  s.om_f = s.om_r = vx / p.r_w;
  return s;
}

// sliding_state with each wheel rolling, spinning, locked or turning backward against the ground
// (nav_dyn_spin_test's spin_state).
dyn_spin_state<double> spin_state(const dyn_bike_params &p, std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn_spin_state<double> s{};
  s.b = sliding_state(rng);
  const dyn_state<double> &b = s.b;
  const double g[2] = {b.vx * std::cos(b.delta) + (b.vy + p.lf * b.r) * std::sin(b.delta), b.vx};
  double *om[2] = {&s.om_f, &s.om_r};
  for (int w = 0; w < 2; ++w) {
    const double mode = 0.5 * (1.0 + u(rng));
    double rim;
    if (mode < 0.4)
      rim = g[w] * (1.0 + 0.05 * u(rng));
    else if (mode < 0.65)
      rim = g[w] + std::fabs(g[w]) * 1.5 * (1.0 + u(rng));
    else if (mode < 0.85)
      rim = 0.0;
    else
      rim = -g[w] * 0.5 * (1.0 + u(rng));
    *om[w] = rim / p.r_w;
  }
  return s;
}

diff::dyn_spin_dg_options spin_opts(double theta, int substeps = 1) {
  diff::dyn_spin_dg_options o;
  o.theta_wheels = theta;
  o.substeps = substeps;
  return o;
}

// The patches' slip velocities, r_w om - t^T v, front and rear.
void slips(const dyn_bike_params &p, const dyn_spin_state<double> &s, double sg[2]) {
  diff::dyn_spin_ph_parts<double> q;
  diff::dyn_spin_ph_parts_at(p, s, q);
  sg[0] = q.sf;
  sg[1] = q.sr;
}

struct spin_books {
  double worst_abs_residual = 0.0, mean_abs_residual = 0.0, residual = 0.0, flow = 0.0;
  double dH = 0.0, supplied = 0.0, braked = 0.0, slip = 0.0, dissipated = 0.0, port = 0.0;
  double scheme = 0.0, least_scheme = 0.0, limiter = 0.0, peak_kappa = 0.0;
  int steps = 0, unconverged = 0, max_iters = 0, secant = 0;
  double mean_iters = 0.0;
};

// nav_dyn_spin_test's book_run. `launch`: from 2 m/s, full drive for 2 s (the wheels spin), then
// full brakes (they lock) to a stop, the wheel a little turned; otherwise the swerve past the
// lateral peak. `wrench` adds a constant external wrench (the contact's port). theta < 0 runs
// dyn_spin_plant_books instead.
spin_books spin_run(const dyn_bike_params &p, double hdt, bool launch, double seconds, double theta,
                    int substeps = 1, const double *wrench = nullptr) {
  const bike_veh lim = plant_limits(p, hdt);
  dyn_spin_state<double> s = rolling(p, launch ? 2.0 : 15.0);
  const int n = static_cast<int>(std::lround(seconds / hdt));
  const diff::dyn_spin_dg_options opt = spin_opts(theta, substeps);
  spin_books rb;
  const double h0 = diff::dyn_spin_ph_hamiltonian(p, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd = launch ? 0.1 : 0.3 * std::sin(kTwoPi * 0.7 * time);
    const double acmd = launch ? (time < 2.0 ? 4.0 : -9.0) : 1.5 * std::sin(kTwoPi * time / 3.0);
    dyn_spin_state<double> o;
    double res, flow;
    if (theta >= 0.0) {
      diff::dyn_spin_dg_books<double> b;
      diff::dyn_dg_solve rep;
      diff::dyn_spin_plant_dg_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b, wrench, &rep, opt);
      res = b.residual;
      rb.supplied += b.supplied;
      rb.braked += b.braked;
      rb.slip += b.slip;
      rb.dissipated += b.dissipated;
      rb.port += b.port;
      rb.scheme += b.scheme;
      rb.least_scheme = std::min(rb.least_scheme, b.scheme);
      rb.limiter += b.limiter;
      flow = std::fabs(b.supplied) + b.braked + b.slip + std::fabs(b.dissipated) +
             std::fabs(b.floor) + std::fabs(b.grade) + std::fabs(b.port) + std::fabs(b.scheme) +
             std::fabs(b.limiter);
      rb.unconverged += !rep.converged;
      rb.max_iters = std::max(rb.max_iters, rep.iters);
      rb.secant += rep.secant;
      rb.mean_iters += rep.iters;
    } else {
      diff::dyn_spin_ph_books<double> b;
      double port = 0.0;
      diff::dyn_spin_plant_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b, wrench, &port);
      res = b.residual;
      rb.supplied += b.supplied;
      rb.braked += b.braked;
      rb.slip += b.slip;
      rb.dissipated += b.dissipated;
      rb.port += port;
      rb.limiter += b.limiter;
      flow = std::fabs(b.supplied) + b.braked + b.slip + std::fabs(b.dissipated) +
             std::fabs(b.floor) + std::fabs(b.grade) + std::fabs(port) + std::fabs(b.limiter);
    }
    rb.worst_abs_residual = std::max(rb.worst_abs_residual, std::fabs(res));
    rb.mean_abs_residual += std::fabs(res);
    rb.residual += res;
    rb.flow += flow;
    diff::dyn_spin_ph_parts<double> q;
    diff::dyn_spin_ph_parts_at(p, o, q);
    const double e2 = double(p.eps) * p.eps;
    rb.peak_kappa = std::max({rb.peak_kappa, std::fabs(q.sf) / std::sqrt(q.gf * q.gf + e2),
                              std::fabs(q.sr) / std::sqrt(q.gr * q.gr + e2)});
    s = o;
  }
  rb.steps = n;
  rb.mean_abs_residual /= n;
  rb.mean_iters /= n;
  rb.dH = diff::dyn_spin_ph_hamiltonian(p, s) - h0;
  return rb;
}

} // namespace

// The momenta's rate the spin step solves with, J(M e) e + Q(e) with Q from the plant's hook
// dyn_spin_ph_forces_at, is dyn_spin_ph_rhs's at every slip: rolling, spinning, locked, wheels
// turning against the ground, on a grade. A wrench adds itself to the body's rows alone.
TEST(DynSpinDG, TheRateIsThePortHamiltonianField) {
  for (const double mu : {0.9, 0.3}) {
    const dyn_bike_params p = humvee(mu);
    std::mt19937 rng(7);
    const double mg = p.m * p.g;
    const double scale[5] = {mg, mg, mg * (p.lf + p.lr), mg * p.r_w, mg * p.r_w};
    const double W[3] = {0.3 * mg, -0.2 * mg, 0.1 * mg * p.lf};
    double worst = 0.0, worst_w = 0.0, worst_one = 0.0;
    for (int k = 0; k < 500; ++k) {
      const dyn_spin_state<double> s = spin_state(p, rng);
      const double grade = (k % 3) * 0.04;
      double f[5], g[5], fw[5], f1[5], fw1[5];
      diff::dyn_spin_dg_rate<double>(p, s, grade, nullptr, f);
      diff::dyn_spin_ph_rhs(p, s, grade, g);
      diff::dyn_spin_dg_rate<double>(p, s, grade, W, fw);
      diff::dyn_spin_dg_rate_one<double>(p, s, grade, nullptr, f1);
      diff::dyn_spin_dg_rate_one<double>(p, s, grade, W, fw1);
      for (int i = 0; i < 5; ++i) {
        worst = std::max(worst, std::fabs(f[i] - g[i]) / scale[i]);
        worst_w = std::max(worst_w, std::fabs(fw[i] - f[i] - (i < 3 ? W[i] : 0.0)) / scale[i]);
        worst_one = std::max(
            {worst_one, std::fabs(f1[i] - f[i]) / scale[i], std::fabs(fw1[i] - fw[i]) / scale[i]});
      }
    }
    std::printf("  mu %.1f: worst |rate - dyn_spin_ph_rhs| = %.2e of m g; the wrench's rows "
                "%.2e; the Jacobian's one-parts rate %.2e\n",
                mu, worst, worst_w, worst_one);
    EXPECT_LT(worst, 1e-12);
    EXPECT_LT(worst_w, 1e-12);
    EXPECT_LT(worst_one, 1e-12);
  }
}

// Its actuators are dyn_spin_plant's (dyn_plant_core's step 1) to the bit, in double and float,
// with the lags and the slew limit on.
TEST(DynSpinDG, TheActuatorsAreDynSpinPlants) {
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.steer_rate = 0.6f;
  p.tau_drive = 0.1f;
  const bike_veh lim = plant_limits(p);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int k = 0; k < 100; ++k) {
    const dyn_spin_state<double> s = spin_state(p, rng);
    const double dcmd = 0.6 * u(rng), acmd = 6.0 * u(rng);
    dyn_spin_state<double> a, b;
    diff::dyn_spin_plant<double>(p, lim, s, dcmd, acmd, 0.0, a);
    diff::dyn_spin_plant_dg<double>(p, lim, s, dcmd, acmd, 0.0, b);
    EXPECT_EQ(a.b.delta, b.b.delta);
    EXPECT_EQ(a.b.a, b.b.a);
    dyn_spin_state<float> sf, af, bf;
    for (int i = 0; i < dyn_spin_state<float>::size; ++i)
      sf[i] = static_cast<float>(s[i]);
    diff::dyn_spin_plant<float>(p, lim, sf, (float)dcmd, (float)acmd, 0.0f, af);
    diff::dyn_spin_plant_dg<float>(p, lim, sf, (float)dcmd, (float)acmd, 0.0f, bf);
    EXPECT_EQ(af.b.delta, bf.b.delta);
    EXPECT_EQ(af.b.a, bf.b.a);
  }
}

// The gate: every step's books close to the solver's tolerance, at nav's 0.06 s and finer, with
// the wheels' rows at the midpoint (th 1/2, scheme 0) and at the step's end (th 1, scheme >= 0):
// the swerve past the lateral peak, a launch that spins the wheels and a stop that locks them, and
// the swerve against a constant external wrench (the port). dyn_spin_plant_books' residual on the
// same runs is printed beside it.
TEST(DynSpinDG, StepBooksCloseExactly) {
  struct scenario {
    const char *name;
    double mu;
    bool launch, wrench;
    double seconds;
  };
  for (const scenario &sc : {scenario{"swerve past the peak, mu 0.6", 0.6, false, false, 6.0},
                             scenario{"launch and lock, mu 0.3", 0.3, true, false, 4.0},
                             scenario{"swerve against a wrench, mu 0.6", 0.6, false, true, 6.0}}) {
    const dyn_bike_params p = humvee(sc.mu);
    const double mg = p.m * p.g;
    const double W[3] = {-0.1 * mg, 0.05 * mg, 0.02 * mg * p.lf};
    const double *w = sc.wrench ? W : nullptr;
    for (const double hdt : {0.06, 0.02, 0.01, 0.005, 0.0025}) {
      const spin_books pl = spin_run(p, hdt, sc.launch, sc.seconds, -1.0, 1, w);
      for (const double theta : {0.5, 1.0}) {
        const spin_books dg = spin_run(p, hdt, sc.launch, sc.seconds, theta, 1, w);
        const double per_step_flow = dg.flow / dg.steps;
        std::printf("  %s, hdt %.4f s, th %.1f: worst |residual| %.2e J (%.1e of the step's "
                    "flow); dH %.4g J, drive %.4g, brakes %.4g, slip %.4g, tires %.4g, port %.4g, "
                    "scheme %.4g, limiter %.3g; peak |kappa| %.2f; Newton %.2f (max %d, %d "
                    "secant); dyn_spin_plant mean |residual| %.3e J\n",
                    sc.name, hdt, theta, dg.worst_abs_residual,
                    dg.worst_abs_residual / per_step_flow, dg.dH, dg.supplied, dg.braked, dg.slip,
                    dg.dissipated, dg.port, dg.scheme, dg.limiter, dg.peak_kappa, dg.mean_iters,
                    dg.max_iters, dg.secant, pl.mean_abs_residual);
        EXPECT_EQ(dg.unconverged, 0) << sc.name << ", hdt " << hdt << ", th " << theta;
        EXPECT_LT(dg.worst_abs_residual / per_step_flow, 1e-10)
            << sc.name << ", hdt " << hdt << ", th " << theta;
        EXPECT_GE(dg.least_scheme, 0.0);
        if (theta == 0.5)
          EXPECT_EQ(dg.scheme, 0.0);
        if (sc.launch)
          EXPECT_GT(dg.peak_kappa, 0.5) << "the wheels neither spun nor locked";
        if (sc.wrench)
          EXPECT_LT(dg.port, 0.0) << "the wrench did no work";
      }
    }
  }
}

// The order: a turn-in on a steady drive from 10 m/s on constant commands (blend off), the twist,
// the wheels and the pose at 2 s against each scheme's own solution at an eighth of its finest
// step, over nav's range (0.04 to 0.005 s) and a fine one (2 to 0.25 ms). With the wheels' rows at
// the midpoint (th 1/2) the step is second order on both, through the wheels' stiff slip; at the
// step's end (th 1) it is first order, and still ten times nearer than dyn_spin_plant at each step.
TEST(DynSpinDG, Order) {
  dyn_bike_params p = humvee(0.9);
  p.v_blend = 0.0f;
  const double T = 2.0, L = p.lf + p.lr;
  for (const bool fine : {false, true}) {
    double err[3][4]; // th 1/2, th 1, dyn_spin_plant; by step
    const double thetas[3] = {0.5, 1.0, -1.0};
    for (int k = 0; k < 3; ++k) {
      const double theta = thetas[k];
      auto run = [&](double hdt) {
        const bike_veh lim = plant_limits(p, hdt);
        dyn_spin_state<double> s = rolling(p, 10.0);
        const diff::dyn_spin_dg_options opt = spin_opts(theta);
        const int n = static_cast<int>(std::lround(T / hdt));
        for (int t = 0; t < n; ++t) {
          dyn_spin_state<double> o;
          if (theta >= 0.0)
            diff::dyn_spin_plant_dg<double>(p, lim, s, 0.15, 1.0, 0.0, o, nullptr, nullptr, nullptr,
                                            opt);
          else
            diff::dyn_spin_plant<double>(p, lim, s, 0.15, 1.0, 0.0, o);
          s = o;
        }
        return s;
      };
      const double h0 = fine ? 0.002 : 0.04;
      const dyn_spin_state<double> ref = run(h0 / 64.0);
      for (int j = 0; j < 4; ++j) {
        const double hdt = h0 / (1 << j);
        const dyn_spin_state<double> s = run(hdt);
        err[k][j] = std::fabs(s.b.vx - ref.b.vx) + std::fabs(s.b.vy - ref.b.vy) +
                    L * std::fabs(s.b.r - ref.b.r) +
                    p.r_w * (std::fabs(s.om_f - ref.om_f) + std::fabs(s.om_r - ref.om_r)) +
                    std::fabs(s.b.x - ref.b.x) / T + std::fabs(s.b.y - ref.b.y) / T;
        const char *name = k == 0 ? "th 1/2" : (k == 1 ? "th 1" : "dyn_spin_plant");
        std::printf("  %-14s hdt %.5f s: error at %.0f s %.3e", name, hdt, T, err[k][j]);
        if (j > 0)
          std::printf(" (%.2fx smaller)", err[k][j - 1] / err[k][j]);
        std::printf("\n");
      }
    }
    for (int j = 1; j < 4; ++j) {
      const double mid = err[0][j - 1] / err[0][j], end = err[1][j - 1] / err[1][j];
      EXPECT_GT(mid, 3.4) << (fine ? "fine" : "nav's range") << ", step " << j;
      EXPECT_LT(mid, 4.6) << (fine ? "fine" : "nav's range") << ", step " << j;
      EXPECT_GT(end, 1.7) << (fine ? "fine" : "nav's range") << ", step " << j;
      EXPECT_LT(end, 2.6) << (fine ? "fine" : "nav's range") << ", step " << j;
    }
    for (int j = 0; j < 4; ++j)
      EXPECT_LT(err[1][j], 0.2 * err[2][j]) << (fine ? "fine" : "nav's range") << ", step " << j;
  }
}

namespace {

// A run's slip behaviour: swings (the slip velocity's step changes alternating in sign, each over
// 1 mm/s, either wheel), the worst step's energy gain where nothing is supplied, the end state.
struct slip_run {
  int swings = 0, steps = 0, unconverged = 0, max_iters = 0, secant = 0;
  double worst_gain = 0.0;
  bool finite = true;
  dyn_spin_state<double> end;
};

slip_run slip_drive(const dyn_bike_params &p, dyn_spin_state<double> s, double hdt, double seconds,
                    double dcmd, double acmd, double theta, int substeps) {
  const bike_veh lim = plant_limits(p, hdt, true);
  const diff::dyn_spin_dg_options opt = spin_opts(theta, substeps);
  slip_run c;
  double sg_prev[2], d_prev[2] = {0.0, 0.0};
  slips(p, s, sg_prev);
  const int n = static_cast<int>(std::lround(seconds / hdt));
  for (int t = 0; t < n; ++t) {
    dyn_spin_state<double> o;
    double gain;
    const double H = diff::dyn_spin_ph_hamiltonian(p, s);
    if (theta >= 0.0) {
      diff::dyn_spin_dg_books<double> b;
      diff::dyn_dg_solve rep;
      diff::dyn_spin_plant_dg_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b, nullptr, &rep, opt);
      gain = b.dH - b.limiter - b.supplied;
      c.unconverged += !rep.converged;
      c.max_iters = std::max(c.max_iters, rep.iters);
      c.secant += rep.secant;
    } else {
      diff::dyn_spin_ph_books<double> b;
      diff::dyn_spin_plant_books<double>(p, lim, s, dcmd, acmd, 0.0, o, b);
      gain = b.dH - b.limiter - b.supplied;
    }
    c.worst_gain = std::max(c.worst_gain, gain / std::max(H, 1e-12));
    double sg[2];
    slips(p, o, sg);
    for (int w = 0; w < 2; ++w) {
      const double d = sg[w] - sg_prev[w];
      if (std::fabs(d) > 1e-3) {
        c.swings += std::fabs(d_prev[w]) > 1e-3 && (d > 0.0) != (d_prev[w] > 0.0);
        d_prev[w] = d;
      }
      sg_prev[w] = sg[w];
    }
    for (int i = 0; i < dyn_spin_state<double>::size; ++i)
      c.finite = c.finite && std::isfinite(o[i]);
    s = o;
    ++c.steps;
  }
  c.end = s;
  return c;
}

} // namespace

// The wheels' stiff slip. Locking under hard braking (mu 0.6 from 20 m/s), spinning under full
// drive on ice (mu 0.3 from 5 m/s), and coasting from a crawl to a cruise with the rims a half
// faster than the ground (nothing supplied, nothing braked), at nav's single 0.06 s step and the
// live plant's 0.01 s, with the wheels' rows at the midpoint and at the step's end, and the
// midpoint split into substeps. Every step solves and stays finite; coasting, no step gains
// energy. The slip's swings are printed against dyn_spin_plant's, and where each run ends against
// the same scheme at 0.5 ms.
TEST(DynSpinDG, StiffWheelSlip) {
  struct scenario {
    const char *name;
    double mu, v0, rim, seconds, dcmd, acmd;
  };
  const scenario scs[] = {
      {"lock: brake 8 m/s^2 from 20 m/s, mu 0.6", 0.6, 20.0, 1.0, 2.5, 0.0, -8.0},
      {"spin: drive 4 m/s^2 from 5 m/s, mu 0.3", 0.3, 5.0, 1.0, 2.0, 0.0, 4.0},
      {"coast from 0.5 m/s, rims 1.5x", 0.9, 0.5, 1.5, 3.0, 0.3, 0.0},
      {"coast from 3 m/s, rims 1.5x", 0.9, 3.0, 1.5, 3.0, 0.3, 0.0},
      {"coast from 12 m/s, rims 1.5x", 0.9, 12.0, 1.5, 3.0, 0.3, 0.0},
  };
  for (const scenario &sc : scs) {
    // The blend off: a limiter applied once a step, it would make the runs differ by their step
    // count alone (DynBikeDG.StiffTires' gate does the same).
    dyn_bike_params p = humvee(sc.mu);
    p.v_blend = 0.0f;
    dyn_spin_state<double> s = rolling(p, sc.v0);
    s.om_f *= sc.rim;
    s.om_r *= sc.rim;
    const bool coasting = sc.acmd == 0.0;
    struct cfg {
      double hdt, theta;
      int sub;
    };
    // The reference: the wheels' rows at the end at 0.1 ms. The midpoint still swings at 0.5 ms
    // (h lambda > 2 below 3 m/s).
    const slip_run ref = slip_drive(p, s, 0.0001, sc.seconds, sc.dcmd, sc.acmd, 1.0, 1);
    slip_run plant[2];
    for (const cfg &c :
         {cfg{0.06, -1.0, 1}, cfg{0.06, 0.5, 1}, cfg{0.06, 0.5, 5}, cfg{0.06, 0.5, 25},
          cfg{0.06, 1.0, 1}, cfg{0.01, -1.0, 1}, cfg{0.01, 0.5, 1}, cfg{0.01, 1.0, 1}}) {
      const slip_run r = slip_drive(p, s, c.hdt, sc.seconds, sc.dcmd, sc.acmd, c.theta, c.sub);
      const double vx_err = r.end.b.vx - ref.end.b.vx;
      const double om_err = p.r_w * (r.end.om_r - ref.end.om_r);
      char name[48];
      if (c.theta < 0.0)
        std::snprintf(name, sizeof(name), "dyn_spin_plant");
      else
        std::snprintf(name, sizeof(name), "th %.1f, %d substep%s", c.theta, c.sub,
                      c.sub > 1 ? "s" : "");
      std::printf("  %s; hdt %.2f s, %-20s: swings %3d in %d steps, end vx %+.3f m/s and rear rim "
                  "%+.3f m/s off 0.1 ms, worst step gain %+.1e of H, Newton max %d (%d secant)\n",
                  sc.name, c.hdt, name, r.swings, r.steps, vx_err, om_err, r.worst_gain,
                  r.max_iters, r.secant);
      EXPECT_TRUE(r.finite);
      const int at = c.hdt > 0.03 ? 0 : 1;
      if (c.theta < 0.0)
        plant[at] = r;
      if (c.theta >= 0.0) {
        EXPECT_EQ(r.unconverged, 0) << sc.name << ", " << name << ", hdt " << c.hdt;
        if (coasting)
          EXPECT_LT(r.worst_gain, 1e-12) << sc.name << ", " << name << ", hdt " << c.hdt;
      }
      if (c.theta == 1.0) {
        // The default: no more swings than dyn_spin_plant's, give or take, and where it ends
        // as close to the reference.
        EXPECT_LE(r.swings, plant[at].swings + 3) << sc.name << ", hdt " << c.hdt;
        EXPECT_LT(std::fabs(vx_err), c.hdt > 0.03 ? 0.15 : 0.01) << sc.name;
        EXPECT_LT(std::fabs(om_err), c.hdt > 0.03 ? 0.6 : 0.02) << sc.name;
      }
    }
  }
}

// What a step costs, against dyn_spin_plant's, on the swerve past the peak at nav's 0.06 s and the
// live plant's 0.01 s: the wheels' rows at the end and at the midpoint, with books, and in float.
TEST(DynSpinDG, CostPerStep) {
  const dyn_bike_params p = humvee(0.6);
  for (const double hdt : {0.06, 0.01}) {
    const bike_veh lim = plant_limits(p, hdt);
    const int n = 10000;
    std::vector<double> dcmd(n), acmd(n);
    for (int t = 0; t < n; ++t) {
      dcmd[t] = 0.3 * std::sin(kTwoPi * 0.7 * t * hdt);
      acmd[t] = 1.5 * std::sin(kTwoPi * t * hdt / 3.0);
    }
    auto time_it = [&](auto &&stepper) {
      dyn_spin_state<double> s = rolling(p, 15.0);
      double sink = 0.0;
      const auto t0 = std::chrono::steady_clock::now();
      for (int t = 0; t < n; ++t) {
        dyn_spin_state<double> o;
        stepper(s, dcmd[t], acmd[t], o);
        s = o;
        sink += o.b.vx;
      }
      const auto t1 = std::chrono::steady_clock::now();
      EXPECT_TRUE(std::isfinite(sink));
      return std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
    };
    typedef dyn_spin_state<double> st;
    const diff::dyn_spin_dg_options end = spin_opts(1.0), midpt = spin_opts(0.5);
    const double plant = time_it([&](const st &s, double d, double a, st &o) {
      diff::dyn_spin_plant<double>(p, lim, s, d, a, 0.0, o);
    });
    const double plant_books = time_it([&](const st &s, double d, double a, st &o) {
      diff::dyn_spin_ph_books<double> b;
      diff::dyn_spin_plant_books<double>(p, lim, s, d, a, 0.0, o, b);
    });
    const double dg = time_it([&](const st &s, double d, double a, st &o) {
      diff::dyn_spin_plant_dg<double>(p, lim, s, d, a, 0.0, o, nullptr, nullptr, nullptr, end);
    });
    const double dg_mid = time_it([&](const st &s, double d, double a, st &o) {
      diff::dyn_spin_plant_dg<double>(p, lim, s, d, a, 0.0, o, nullptr, nullptr, nullptr, midpt);
    });
    const double dg_books = time_it([&](const st &s, double d, double a, st &o) {
      diff::dyn_spin_dg_books<double> b;
      diff::dyn_spin_plant_dg_books<double>(p, lim, s, d, a, 0.0, o, b, nullptr, nullptr, end);
    });
    const double dg_float = time_it([&](const st &s, double d, double a, st &o) {
      dyn_spin_state<float> sf, of;
      for (int i = 0; i < dyn_spin_state<float>::size; ++i)
        sf[i] = static_cast<float>(s[i]);
      diff::dyn_spin_plant_dg<float>(p, lim, sf, (float)d, (float)a, 0.0f, of);
      for (int i = 0; i < dyn_spin_state<float>::size; ++i)
        o[i] = of[i];
    });
    std::printf("  hdt %.2f s, per step: dyn_spin_plant %.0f ns; th 1 %.0f ns (%.1fx), th 1/2 "
                "%.0f ns; with books %.0f ns vs %.0f ns (%.1fx); th 1 in float %.0f ns\n",
                hdt, plant, dg, dg / plant, dg_mid, plant_books, dg_books, dg_books / plant_books,
                dg_float);
    // Where the time goes: the solves' Newton steps over the same run, untimed.
    for (const diff::dyn_spin_dg_options *opt : {&end, &midpt}) {
      dyn_spin_state<double> s = rolling(p, 15.0);
      int iters = 0, most = 0, secant = 0, unconverged = 0;
      double vmin = s.b.vx, vmax = s.b.vx;
      for (int t = 0; t < n; ++t) {
        dyn_spin_state<double> o;
        diff::dyn_dg_solve rep;
        diff::dyn_spin_plant_dg<double>(p, lim, s, dcmd[t], acmd[t], 0.0, o, nullptr, nullptr, &rep,
                                        *opt);
        iters += rep.iters;
        most = std::max(most, rep.iters);
        secant += rep.secant;
        unconverged += !rep.converged;
        vmin = std::min(vmin, o.b.vx);
        vmax = std::max(vmax, o.b.vx);
        s = o;
      }
      std::printf("    th %.1f: Newton %.2f steps a solve (max %d, %d secant, %d unconverged), "
                  "vx %.2f to %.2f m/s\n",
                  opt->theta_wheels, double(iters) / n, most, secant, unconverged, vmin, vmax);
      EXPECT_EQ(unconverged, 0);
    }
    EXPECT_GT(dg, 0.0);
  }
}
