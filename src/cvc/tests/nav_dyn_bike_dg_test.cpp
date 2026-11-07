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
