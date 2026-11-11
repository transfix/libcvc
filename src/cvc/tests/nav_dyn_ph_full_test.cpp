/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// nav_dyn_ph_full_test: the full (q, p) port-Hamiltonian form of the dynamic vehicles beside the
// momentum-only one (diff::dyn_ph_rhs_full, dyn4_ph_rhs_full), and the single track's slope across
// its heading with gravity as a potential in H (dyn_plant_ground, dyn_plant_books_ground). The
// gates:
//   - the structure [[0, T(q)], [-T(q)^T, J(p)]]: skew, Poisson (the Jacobi identity by central
//     differences), and -T^T dV/dq the slopes' force, with no moment;
//   - the pose update: dyn_plant's is q1 = q0 + hdt T(q1) v1 to round-off, a first-order step of
//     q' = T(q) v;
//   - the vector field: a very short plant step on a ground, and with the slopes as data, against
//     dyn_ph_rhs_full in all six coordinates; dyn4's twelve, with grade_force;
//   - the books on a side slope, closed loop with the body sliding: with gravity in H they close
//     to second order; booked without the slope across the heading, or with the slopes' sines,
//     they drift;
//   - the golden GRL-SNAM's torch twin is held to (DynPHFull.Golden).
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
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using cvc::nav::diff::bike_veh;
using cvc::nav::diff::dyn4_params;
using cvc::nav::diff::dyn4_state;
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

dyn4_params humvee4() {
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn4_params_of(
      cvc::physics::nav_plant::dyn4_si_of(cvc::physics::vehicle_desc::humvee()), f);
}

bike_veh plant_limits(double L, bool reverse = false) {
  bike_veh v{};
  v.L = static_cast<float>(L);
  v.vmax = 40.0f;
  v.delta_max = 0.6f;
  v.hdt = 0.06f;
  v.allow_reverse = reverse ? 1 : 0;
  return v;
}

// A hillside: rising sy to the +y side and sx along +x, with swells of A, centred on (x0, y0).
struct hillside {
  double mu = 0.9, sx = 0.04, sy = 0.15, A = 1.5, kx = 0.05, ky = 0.07, x0 = 0.0, y0 = 0.0;
  template <class S> void at(S x, S y, S &mu_out, S &h) const {
    mu_out = S(mu);
    const S dx = x - S(x0), dy = y - S(y0);
    h = S(sx) * dx + S(sy) * dy + S(A) * diff::s_sin(S(kx) * dx) * diff::s_cos(S(ky) * dy);
  }
  double height(double x, double y) const {
    double mu_out, h;
    at(x, y, mu_out, h);
    return h;
  }
  // Its gradient, exactly.
  void grad(double x, double y, double &gx, double &gy) const {
    const double dx = x - x0, dy = y - y0;
    gx = sx + A * kx * std::cos(kx * dx) * std::cos(ky * dy);
    gy = sy - A * ky * std::sin(kx * dx) * std::sin(ky * dy);
  }
};

// A Humvee state anywhere in the envelope and anywhere on the hillside: forward or reversing, the
// wheel up to 0.5 rad, and sideways and yaw rates well past the tires' peaks.
dyn_state<double> sliding_state(std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn_state<double> s{};
  const double speed = 3.0 + 25.0 * 0.5 * (1.0 + u(rng));
  s.vx = u(rng) < -0.7 ? -std::min(speed, 8.0) : speed;
  s.delta = 0.5 * u(rng);
  s.vy = 4.0 * u(rng);
  s.r = 1.2 * u(rng);
  s.a = 5.0 * u(rng);
  s.x = 30.0 * u(rng);
  s.y = 30.0 * u(rng);
  s.psi = 3.0 * u(rng);
  return s;
}

// The state's x = (q, p), and a state with only what the structure reads (the pose and momenta).
void coords(const dyn_bike_params &p, const dyn_state<double> &s, double x[6]) {
  x[0] = s.x;
  x[1] = s.y;
  x[2] = s.psi;
  x[3] = p.m * s.vx;
  x[4] = p.m * s.vy;
  x[5] = p.Iz * s.r;
}
dyn_state<double> state_of(const dyn_bike_params &p, const double x[6]) {
  dyn_state<double> s{};
  s.x = x[0];
  s.y = x[1];
  s.psi = x[2];
  s.vx = x[3] / p.m;
  s.vy = x[4] / p.m;
  s.r = x[5] / p.Iz;
  return s;
}

// The Jacobi identity of a structure J(x) on n coordinates, by central differences (steps h[l]):
// the worst |sum over (i, j, k) cyclic of J_il d_l J_jk| against the sum of its terms' sizes.
template <int N, class F> double jacobi_defect(const double x[N], const double h[N], const F &Jof) {
  double J[N][N];
  Jof(x, J);
  static double dJ[N][N][N];
  for (int l = 0; l < N; ++l) {
    double xp[N], xm[N], Jp[N][N], Jm[N][N];
    for (int a = 0; a < N; ++a)
      xp[a] = xm[a] = x[a];
    xp[l] += h[l];
    xm[l] -= h[l];
    Jof(xp, Jp);
    Jof(xm, Jm);
    for (int a = 0; a < N; ++a)
      for (int b = 0; b < N; ++b)
        dJ[l][a][b] = (Jp[a][b] - Jm[a][b]) / (2.0 * h[l]);
  }
  double worst = 0.0;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      for (int k = 0; k < N; ++k) {
        double sum = 0.0, size = 0.0;
        for (int l = 0; l < N; ++l) {
          const double t[3] = {J[i][l] * dJ[l][j][k], J[j][l] * dJ[l][k][i], J[k][l] * dJ[l][i][j]};
          for (const double v : t) {
            sum += v;
            size += std::fabs(v);
          }
        }
        if (size > 0.0)
          worst = std::max(worst, std::fabs(sum) / size);
      }
  return worst;
}

} // namespace

// ── The single track ───────────────────────────────────────────────────────────────────────

// The structure on (q, p) is skew at every state and moves no energy; it is Poisson (Jacobi by
// central differences, the coadjoint block and T(q) together); dV/dq is the ground's gradient at
// the centre of gravity (differenced over 1e-3 L); and -T^T dV/dq is - m g (along, across, 0): the
// slopes' force, with no moment. The momentum part of the full field is dyn_ph_rhs's.
TEST(DynPHFull, TheStructureIsSkewAndPoisson) {
  const dyn_bike_params p = humvee(0.6);
  const hillside hill;
  // In double, as the form takes the float parameters.
  const double mg = static_cast<double>(p.m) * p.g, L = static_cast<double>(p.lf) + p.lr;
  std::mt19937 rng(3);
  double worst_energy = 0.0, worst_jacobi = 0.0, worst_grad = 0.0, worst_force = 0.0;
  for (int k = 0; k < 300; ++k) {
    const dyn_state<double> s = sliding_state(rng);
    double J[6][6];
    diff::dyn_ph_J_full(p, s, J);
    double e[6];
    diff::dyn_ph_grad_q(p, hill, s, e);
    e[3] = s.vx;
    e[4] = s.vy;
    e[5] = s.r;
    double eJe = 0.0, eJe_abs = 0.0;
    for (int a = 0; a < 6; ++a)
      for (int b = 0; b < 6; ++b) {
        EXPECT_EQ(J[a][b], -J[b][a]);
        eJe += e[a] * J[a][b] * e[b];
        eJe_abs += std::fabs(e[a] * J[a][b] * e[b]);
      }
    worst_energy = std::max(worst_energy, std::fabs(eJe) / eJe_abs);
    // Jacobi: J is linear in p, so any step is exact there; T's sines want a small one.
    double x[6];
    coords(p, s, x);
    const double h[6] = {1e-3, 1e-3, 1e-5, 1.0, 1.0, 1.0};
    worst_jacobi =
        std::max(worst_jacobi, jacobi_defect<6>(x, h, [&](const double *xx, double(*Jx)[6]) {
                   diff::dyn_ph_J_full(p, state_of(p, xx), Jx);
                 }));
    // dV/dq against the ground's exact gradient at the centre of gravity.
    const double cx = s.x + p.lr * std::cos(s.psi), cy = s.y + p.lr * std::sin(s.psi);
    double gx, gy;
    hill.grad(cx, cy, gx, gy);
    const double exact[3] = {mg * gx, mg * gy,
                             mg * p.lr * (std::cos(s.psi) * gy - std::sin(s.psi) * gx)};
    for (int a = 0; a < 3; ++a)
      worst_grad = std::max(worst_grad, std::fabs(e[a] - exact[a]) / mg);
    // -T^T dV/dq: the slopes' force along and across the heading, and no moment.
    double T[3][3], along, across;
    diff::dyn_ph_T(p, s.psi, T);
    diff::dyn_ph_slope(p, hill, s, along, across);
    double f[3];
    for (int a = 0; a < 3; ++a)
      f[a] = -(T[0][a] * e[0] + T[1][a] * e[1] + T[2][a] * e[2]);
    worst_force = std::max({worst_force, std::fabs(f[0] + mg * along) / mg,
                            std::fabs(f[1] + mg * across) / mg, std::fabs(f[2]) / (mg * L)});
    // The data overload's momentum part is dyn_ph_rhs's, to the bit, without a side slope.
    double qd[3], pd[3], pm[3];
    diff::dyn_ph_rhs_full(p, s, 0.03, 0.0, qd, pd);
    diff::dyn_ph_rhs(p, s, 0.03, pm);
    for (int a = 0; a < 3; ++a)
      EXPECT_EQ(pd[a], pm[a]);
  }
  std::printf("  300 states on the hillside: |e^T J e| / sum |terms| <= %.1e; Jacobi defect <= "
              "%.1e; dV/dq within %.1e m g of the exact gradient; -T^T dV/dq within %.1e of the "
              "slopes' force\n",
              worst_energy, worst_jacobi, worst_grad, worst_force);
  EXPECT_LT(worst_energy, 1e-14);
  EXPECT_LT(worst_jacobi, 1e-8);
  EXPECT_LT(worst_grad, 1e-6);
  EXPECT_LT(worst_force, 1e-14);
}

// dyn_plant's pose update is q1 = q0 + hdt T(q1) v1, the step's new heading and twist, at every
// slip, on flat ground and on the hillside: so it is a first-order step of q' = T(q) v, its error
// against T(q0) v0 halving with hdt.
TEST(DynPHFull, ThePoseUpdateIsTv) {
  const dyn_bike_params p = humvee(0.6);
  const hillside hill;
  const bike_veh lim = plant_limits(p.lf + p.lr, true);
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst = 0.0;
  for (int k = 0; k < 400; ++k) {
    const dyn_state<double> s = sliding_state(rng);
    const double dcmd = 0.5 * u(rng), acmd = 4.0 * u(rng);
    dyn_state<double> o;
    if (k % 2)
      diff::dyn_plant<double>(p, lim, s, dcmd, acmd, 0.0, o);
    else
      diff::dyn_plant_ground<double>(p, lim, hill, s, dcmd, acmd, o);
    double T[3][3];
    diff::dyn_ph_T(p, o.psi, T);
    const double v1[3] = {o.vx, o.vy, o.r};
    const double dq[3] = {o.x - s.x, o.y - s.y, o.psi - s.psi};
    for (int a = 0; a < 3; ++a) {
      const double want = lim.hdt * (T[a][0] * v1[0] + T[a][1] * v1[1] + T[a][2] * v1[2]);
      const double err = std::fabs(dq[a] - want) / (1.0 + std::fabs(s[a]));
      worst = std::max(worst, err);
      EXPECT_LT(err, 1e-14) << "coordinate " << a << " at vx " << s.vx;
    }
  }
  std::printf("  dyn_plant's pose step vs hdt T(q1) v1, 400 states: worst %.1e\n", worst);
  // The step against q' = T(q0) v0 over a swerve's first step, sliding on the hillside.
  dyn_state<double> s{};
  s.vx = 14.0;
  s.vy = -1.5;
  s.r = 0.6;
  s.delta = 0.2;
  s.a = 1.0;
  s.psi = 0.4;
  double prev = 0.0;
  for (const double hdt : {0.04, 0.02, 0.01, 0.005}) {
    bike_veh l = lim;
    l.hdt = static_cast<float>(hdt);
    dyn_state<double> o;
    diff::dyn_plant_ground<double>(p, l, hill, s, 0.35, 2.0, o);
    double qd[3], pd[3];
    diff::dyn_ph_rhs_full(p, hill, s, qd, pd);
    const double dq[3] = {o.x - s.x, o.y - s.y, o.psi - s.psi};
    double err = 0.0;
    for (int a = 0; a < 3; ++a)
      err = std::max(err, std::fabs(dq[a] / l.hdt - qd[a]));
    std::printf("    hdt %.3f s: |(q1 - q0) / hdt - T(q0) v0| = %.3e\n", hdt, err);
    if (prev > 0.0) {
      EXPECT_GT(prev / err, 1.7) << "hdt " << hdt;
      EXPECT_LT(prev / err, 2.3) << "hdt " << hdt;
    }
    prev = err;
  }
}

// One very short dyn_plant step against the full form's vector field in all six coordinates, at
// every slip, forward and reversing: on the hillside (dyn_plant_ground against the ground's
// overload, gravity in H) and with the slopes as data (grade and side, f_g). The actuators hold
// (commands equal to their state) and the low-speed blend is off, as DynBikePH's.
TEST(DynPHFull, TheVectorFieldIsTheFullFormsAtEverySlip) {
  const hillside hill;
  for (const double mu : {0.9, 0.6}) {
    dyn_bike_params p = humvee(mu);
    p.v_blend = 0.0f;
    bike_veh lim = plant_limits(p.lf + p.lr, true);
    lim.hdt = 1e-5f;
    const double mg = p.m * p.g, L = p.lf + p.lr;
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    double worst_q = 0.0, worst_p = 0.0;
    for (int k = 0; k < 400; ++k) {
      const dyn_state<double> s = sliding_state(rng);
      const bool on_ground = k % 2 == 0;
      const double grade = 0.2 * u(rng), side = 0.2 * u(rng);
      dyn_state<double> o;
      double qd[3], pd[3];
      if (on_ground) {
        diff::dyn_plant_ground<double>(p, lim, hill, s, s.delta, s.a, o);
        diff::dyn_ph_rhs_full(p, hill, s, qd, pd);
      } else {
        diff::dyn_plant<double>(p, lim, s, s.delta, s.a, grade, o, nullptr, nullptr, &side);
        diff::dyn_ph_rhs_full(p, s, grade, side, qd, pd);
      }
      double x0[6], x1[6];
      coords(p, s, x0);
      coords(p, o, x1);
      const double scale[6] = {1.0 + std::fabs(s.vx) + std::fabs(s.vy) + p.lr * std::fabs(s.r),
                               1.0 + std::fabs(s.vx) + std::fabs(s.vy) + p.lr * std::fabs(s.r),
                               1.0,
                               mg,
                               mg,
                               mg * L};
      for (int a = 0; a < 6; ++a) {
        const double fd = (x1[a] - x0[a]) / lim.hdt, f = a < 3 ? qd[a] : pd[a - 3];
        const double rel = std::fabs(fd - f) / scale[a];
        (a < 3 ? worst_q : worst_p) = std::max(a < 3 ? worst_q : worst_p, rel);
        EXPECT_LT(rel, 1e-3) << (on_ground ? "hillside" : "data") << ", coordinate " << a
                             << ": plant " << fd << ", pH " << f << " at vx " << s.vx << ", vy "
                             << s.vy << ", r " << s.r << ", delta " << s.delta;
      }
    }
    std::printf("  full vector field vs dyn_plant at every slip, mu %.1f: worst |dq/dt - T v| = "
                "%.2e of the speed, |dp/dt - pH| = %.2e m g\n",
                mu, worst_q, worst_p);
  }
}

namespace {

// One closed-loop run's books on the hillside, booked four ways (see below).
struct side_books {
  double mean_abs = 0.0, sum = 0.0, flow = 0.0;
};
struct side_run {
  side_books h;        // gravity in H: dyn_plant_books_ground, the grade line - dV
  side_books momentum; // the same steps in the momentum-only books: the grade line - m g along vx
  double max_beta = 0.0, mean_vy = 0.0, drop = 0.0;
};

void add(side_books &k, const diff::dyn_ph_books<double> &b, int n) {
  k.mean_abs += std::fabs(b.residual) / n;
  k.sum += b.residual;
  k.flow += std::fabs(b.supplied) + std::fabs(b.dissipated) + std::fabs(b.floor) +
            std::fabs(b.grade) + std::fabs(b.limiter);
}

// A driver holds the line y = 0 along +x at 12 m/s across the hillside, weaving at 0.4 Hz. The
// slope pulls the body toward -y, so the tires slip sideways to hold it, and the weave slides it.
// `plant`: 0, the plant with the slope across the heading (dyn_plant_ground); 1, without it (the
// slope along the heading only); 2, with both slopes' sines rather than their tangents. Every run
// is booked against - dV (V the centre of gravity's m g h); the side-slope plant's steps are also
// booked in the momentum-only form, whose grade line is the slope along the heading's power.
side_run hill_run(const dyn_bike_params &p, const hillside &hill, double hdt, int plant,
                  double seconds) {
  bike_veh lim = plant_limits(p.lf + p.lr);
  lim.hdt = static_cast<float>(hdt);
  dyn_state<double> s{};
  s.x = -40.0;
  s.vx = 12.0;
  const int n = static_cast<int>(std::lround(seconds / hdt));
  side_run run;
  const double V0 = diff::dyn_ph_potential(p, hill, s);
  for (int t = 0; t < n; ++t) {
    const double time = t * hdt;
    const double dcmd =
        std::clamp(-0.04 * s.y - 0.6 * s.psi + 0.12 * std::sin(kTwoPi * 0.4 * time), -0.5, 0.5);
    const double acmd = std::clamp(0.8 * (12.0 - s.vx), -4.0, 3.0);
    dyn_state<double> o;
    diff::dyn_ph_books<double> b;
    if (plant == 0) {
      diff::dyn_plant_books_ground<double>(p, lim, hill, s, dcmd, acmd, o, b);
      // The same step in the momentum-only books.
      double along, across;
      diff::dyn_ph_slope(p, hill, s, along, across);
      dyn_state<double> o2;
      diff::dyn_ph_books<double> b2;
      diff::dyn_plant_books<double>(p, lim, s, dcmd, acmd, along, o2, b2, nullptr, nullptr, nullptr,
                                    &across);
      EXPECT_EQ(o2.x, o.x);
      EXPECT_EQ(o2.vy, o.vy);
      add(run.momentum, b2, n);
    } else {
      double along, across;
      diff::dyn_ph_slope(p, hill, s, along, across);
      if (plant == 2) {
        along = along / std::sqrt(1.0 + along * along);
        across = across / std::sqrt(1.0 + across * across);
      }
      diff::dyn_plant_books<double>(p, lim, s, dcmd, acmd, along, o, b, nullptr, nullptr, nullptr,
                                    plant == 2 ? &across : nullptr);
      const double dV = diff::dyn_ph_potential(p, hill, o) - diff::dyn_ph_potential(p, hill, s);
      b.residual = b.residual + b.grade + dV;
      b.grade = -dV;
    }
    add(run.h, b, n);
    run.max_beta = std::max(run.max_beta, std::fabs(std::atan2(o.vy, std::fabs(o.vx))));
    run.mean_vy += o.vy / n;
    s = o;
  }
  run.drop = diff::dyn_ph_potential(p, hill, s) - V0;
  return run;
}

} // namespace

// The closed-loop gate on a side slope: with the slope across the heading in the plant and gravity
// in H, every step's books close but for the integrator's error, second order per step. Booking the
// same steps in the momentum-only form (no potential, gravity the slope along the heading's power)
// leaves the lateral component's work unbooked, and a plant without the slope across the heading
// does work against gravity that is not - dV: both drift, by an amount no smaller step removes. The
// slopes' sines leave m g (tan - sin) unbooked, the same way.
TEST(DynPHFull, SideSlopeBooksCloseWithGravityInH) {
  const dyn_bike_params p = humvee(0.6);
  const hillside hill;
  double prev = 0.0, prev_mom = 0.0, prev_side = 0.0;
  for (const double hdt : {0.02, 0.01, 0.005, 0.0025}) {
    const side_run in_h = hill_run(p, hill, hdt, 0, 6.0);
    const side_run no_side = hill_run(p, hill, hdt, 1, 6.0);
    const side_run sines = hill_run(p, hill, hdt, 2, 6.0);
    const double rel = std::fabs(in_h.h.sum) / in_h.h.flow;
    const double rel_mom = std::fabs(in_h.momentum.sum) / in_h.momentum.flow;
    const double rel_side = std::fabs(no_side.h.sum) / no_side.h.flow;
    const double rel_sin = std::fabs(sines.h.sum) / sines.h.flow;
    std::printf("  hdt %.4f s: gravity in H: dV %.4g J, body slip up to %.3f rad (mean vy %.3f "
                "m/s), mean |residual| per step %.3e J, |sum| / flow %.2e; momentum-only books: "
                "|sum| / flow %.2e; plant without the side slope: %.2e; with the sines: %.2e\n",
                hdt, in_h.drop, in_h.max_beta, in_h.mean_vy, in_h.h.mean_abs, rel, rel_mom,
                rel_side, rel_sin);
    if (prev > 0.0) {
      const double ratio = prev / in_h.h.mean_abs;
      EXPECT_GT(ratio, 3.0) << "hdt " << hdt;
      EXPECT_LT(ratio, 5.5) << "hdt " << hdt;
      // The drifts do not shrink with the step.
      EXPECT_GT(rel_mom / prev_mom, 0.7) << "hdt " << hdt;
      EXPECT_GT(rel_side / prev_side, 0.7) << "hdt " << hdt;
    }
    prev = in_h.h.mean_abs;
    prev_mom = rel_mom;
    prev_side = rel_side;
    EXPECT_GT(in_h.max_beta, 0.02) << "the body does not slide";
    if (hdt <= 0.01) {
      EXPECT_LT(rel, 2e-3) << "hdt " << hdt;
      EXPECT_GT(rel_mom, 30.0 * rel) << "hdt " << hdt;
      EXPECT_GT(rel_side, 5.0 * rel) << "hdt " << hdt;
    }
    if (hdt <= 0.005)
      EXPECT_GT(rel_sin, 3.0 * rel) << "hdt " << hdt;
  }
}

// The live plant's slope across the heading takes the wheel-spin plant too: on the hillside with
// its wheels, dyn_spin_plant_books_ground's books close as dyn_plant_books_ground's.
TEST(DynPHFull, TheWheelSpinPlantTakesTheSlopeAcrossItsHeading) {
  const dyn_bike_params p = humvee(0.6);
  const hillside hill;
  double prev = 0.0;
  for (const double hdt : {0.01, 0.005, 0.0025}) {
    bike_veh lim = plant_limits(p.lf + p.lr);
    lim.hdt = static_cast<float>(hdt);
    diff::dyn_spin_state<double> s{};
    s.b.x = -40.0;
    s.b.vx = 12.0;
    s.om_f = s.om_r = 12.0 / p.r_w;
    const int n = static_cast<int>(std::lround(6.0 / hdt));
    double mean_abs = 0.0, sum = 0.0, flow = 0.0;
    for (int t = 0; t < n; ++t) {
      const double time = t * hdt;
      const double dcmd = std::clamp(
          -0.04 * s.b.y - 0.6 * s.b.psi + 0.12 * std::sin(kTwoPi * 0.4 * time), -0.5, 0.5);
      const double acmd = std::clamp(0.8 * (12.0 - s.b.vx), -4.0, 3.0);
      diff::dyn_spin_state<double> o;
      diff::dyn_spin_ph_books<double> b;
      diff::dyn_spin_plant_books_ground<double>(p, lim, hill, s, dcmd, acmd, o, b);
      mean_abs += std::fabs(b.residual) / n;
      sum += b.residual;
      flow += std::fabs(b.supplied) + std::fabs(b.braked) + std::fabs(b.dissipated) +
              std::fabs(b.slip) + std::fabs(b.floor) + std::fabs(b.grade) + std::fabs(b.limiter);
      s = o;
    }
    std::printf("  wheel spin on the hillside, hdt %.4f s: mean |residual| per step %.3e J, |sum| "
                "/ flow %.2e\n",
                hdt, mean_abs, std::fabs(sum) / flow);
    EXPECT_LT(std::fabs(sum) / flow, 2e-3) << "hdt " << hdt;
    if (prev > 0.0)
      EXPECT_GT(prev / mean_abs, 2.5) << "hdt " << hdt;
    prev = mean_abs;
  }
}

// ── The four-wheel vehicle ────────────────────────────────────────────────────────────────

namespace {

// A gentle hillside for the sprung vehicle, level with its rest height near (4, 1.5).
hillside hill4() {
  hillside h;
  h.sx = 0.08;
  h.sy = 0.12;
  h.A = 0.05;
  h.kx = 1.0 / 1.3;
  h.ky = 1.0 / 1.7;
  h.x0 = 4.0;
  h.y0 = 1.5;
  return h;
}

// nav_dyn_4w_test's sliding4, the body on the ground's plane under it.
dyn4_state<double> sliding4(const dyn4_params &p, const hillside &hill, std::mt19937 &rng) {
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  dyn4_state<double> s{};
  const double speed = 3.0 + 25.0 * 0.5 * (1.0 + u(rng));
  s.vx = u(rng) < -0.7 ? -std::min(speed, 8.0) : speed;
  s.delta = 0.5 * u(rng);
  s.vy = 4.0 * u(rng);
  s.r = 1.2 * u(rng);
  s.a = 5.0 * u(rng);
  s.x = 4.0 + 2.0 * u(rng);
  s.y = 1.5 + 2.0 * u(rng);
  s.psi = 3.0 * u(rng);
  const double cx = s.x + p.lr * std::cos(s.psi), cy = s.y + p.lr * std::sin(s.psi);
  double gx, gy;
  hill.grad(cx, cy, gx, gy);
  const double along = std::cos(s.psi) * gx + std::sin(s.psi) * gy;
  const double across = std::cos(s.psi) * gy - std::sin(s.psi) * gx;
  s.z = hill.height(cx, cy) + 0.03 * u(rng);
  s.th = -along + 0.02 * u(rng);
  s.phi = across + 0.03 * u(rng);
  s.zd = 0.2 * u(rng);
  s.phid = 0.2 * u(rng);
  s.thd = 0.2 * u(rng);
  return s;
}

// x = (g, q, p) as the state's coordinates, and each one's mass (p = M xi).
const int kCoord4[12] = {0, 1, 2, 6, 8, 10, 3, 4, 5, 7, 9, 11};
double coord_mass4(const dyn4_params &p, int a) {
  const double M[12] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, p.m, p.m, p.Iz, p.m, p.Ix, p.Iy};
  return M[a];
}

} // namespace

// The structure on (g, q, p) is skew at every state and moves no energy; it is Poisson; dH/dg is
// the springs' energy's gradient in the pose (against H's central differences); and -T^T dH/dg is
// dyn4_grade's force and moment, to round-off: the grade force is the full form's structure.
TEST(Dyn4PHFull, TheStructureIsSkewAndPoissonAndTheGradeForceIsIts) {
  const dyn4_params p = humvee4();
  const hillside hill = hill4();
  const bike_veh v = plant_limits(p.lf + p.lr);
  const double mg = p.m * p.g, L = p.lf + p.lr;
  std::mt19937 rng(3);
  double worst_energy = 0.0, worst_jacobi = 0.0, worst_grad = 0.0, worst_grade = 0.0;
  for (int k = 0; k < 200; ++k) {
    const dyn4_state<double> s = sliding4(p, hill, rng);
    double J[12][12], e[12];
    diff::dyn4_ph_J_full(p, s, J);
    diff::dyn4_ph_grad_full(p, v, hill, s, e);
    double eJe = 0.0, eJe_abs = 0.0;
    for (int a = 0; a < 12; ++a)
      for (int b = 0; b < 12; ++b) {
        EXPECT_EQ(J[a][b], -J[b][a]);
        eJe += e[a] * J[a][b] * e[b];
        eJe_abs += std::fabs(e[a] * J[a][b] * e[b]);
      }
    worst_energy = std::max(worst_energy, std::fabs(eJe) / eJe_abs);
    if (k < 40) {
      double x[12];
      diff::dyn4_ph_coords_full(p, s, x);
      const double h[12] = {1e-3, 1e-3, 1e-5, 1e-3, 1e-3, 1e-3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
      worst_jacobi =
          std::max(worst_jacobi, jacobi_defect<12>(x, h, [&](const double *xx, double(*Jx)[12]) {
                     dyn4_state<double> t = s;
                     for (int a = 0; a < 12; ++a)
                       t[kCoord4[a]] = xx[a] / coord_mass4(p, a);
                     diff::dyn4_ph_J_full(p, t, Jx);
                   }));
    }
    // dH/dg against H's central differences in the pose.
    for (int a = 0; a < 3; ++a) {
      const double d = 1e-6;
      dyn4_state<double> sp = s, sm = s;
      sp[kCoord4[a]] += d;
      sm[kCoord4[a]] -= d;
      const double fd =
          (diff::dyn4_ph_hamiltonian(p, v, hill, sp) - diff::dyn4_ph_hamiltonian(p, v, hill, sm)) /
          (2.0 * d);
      worst_grad = std::max(worst_grad, std::fabs(fd - e[a]) / (mg * L));
    }
    // -T^T dH/dg and dyn4_grade.
    diff::dyn4_wheels<double> W;
    diff::dyn4_wheel_state(p, v, hill, s, W);
    double fg[3], T[3][3];
    diff::dyn4_grade(p, hill, s, W, fg);
    diff::dyn4_ph_T(p, s.psi, T);
    const double scale[3] = {mg, mg, mg * L};
    for (int a = 0; a < 3; ++a) {
      const double f = -(T[0][a] * e[0] + T[1][a] * e[1] + T[2][a] * e[2]);
      worst_grade = std::max(worst_grade, std::fabs(f - fg[a]) / scale[a]);
    }
  }
  std::printf("  dyn4, 200 states on the hillside: |e^T J e| / sum |terms| <= %.1e; Jacobi defect "
              "<= %.1e; dH/dg within %.1e m g L of H's differences; -T^T dH/dg within %.1e of "
              "dyn4_grade\n",
              worst_energy, worst_jacobi, worst_grad, worst_grade);
  EXPECT_LT(worst_energy, 1e-12);
  EXPECT_LT(worst_jacobi, 1e-8);
  EXPECT_LT(worst_grad, 1e-6);
  EXPECT_LT(worst_grade, 1e-12);
}

// One very short dyn4_plant step with grade_force against the full form's vector field in all
// twelve coordinates, at every slip, on the hillside; its pose update is q1 = q0 + hdt T(q1) v1,
// as the single track's. Without grade_force the plant's field is this one less -T^T dH/dg: the
// hill for free, printed as the gap.
TEST(Dyn4PHFull, TheVectorFieldIsTheFullFormsWithTheGradeForce) {
  dyn4_params p = humvee4();
  p.v_blend = 0.0f;
  p.grade_force = 1;
  dyn4_params pn = p;
  pn.grade_force = 0;
  const hillside hill = hill4();
  bike_veh lim = plant_limits(p.lf + p.lr, true);
  lim.hdt = 1e-5f;
  const double mg = p.m * p.g, L = p.lf + p.lr, t = std::max(p.tf, p.tr);
  std::mt19937 rng(5);
  double worst = 0.0, gap = 0.0, worst_pose = 0.0;
  for (int k = 0; k < 300; ++k) {
    const dyn4_state<double> s = sliding4(p, hill, rng);
    const double speed = 1.0 + std::fabs(s.vx) + std::fabs(s.vy) + p.lr * std::fabs(s.r);
    const double scale[12] = {speed, speed, 1.0, 1.0, 1.0, 1.0, mg, mg, mg * L, mg, mg * t, mg * L};
    dyn4_state<double> o, on;
    diff::dyn4_plant<double>(p, lim, hill, s, s.delta, s.a, o);
    diff::dyn4_plant<double>(pn, lim, hill, s, s.delta, s.a, on);
    double x0[12], x1[12], xn[12], f[12];
    diff::dyn4_ph_coords_full(p, s, x0);
    diff::dyn4_ph_coords_full(p, o, x1);
    diff::dyn4_ph_coords_full(p, on, xn);
    diff::dyn4_ph_rhs_full(p, lim, hill, s, f);
    for (int a = 0; a < 12; ++a) {
      const double fd = (x1[a] - x0[a]) / lim.hdt;
      const double rel = std::fabs(fd - f[a]) / scale[a];
      worst = std::max(worst, rel);
      EXPECT_LT(rel, 1e-3) << "coordinate " << a << ": plant " << fd << ", pH " << f[a] << " at vx "
                           << s.vx << ", vy " << s.vy << ", r " << s.r;
      if (a >= 6 && a < 9)
        gap = std::max(gap, std::fabs((xn[a] - x0[a]) / lim.hdt - f[a]) / scale[a]);
    }
    // The pose update, at nav's step.
    bike_veh l = lim;
    l.hdt = 0.06f;
    diff::dyn4_plant<double>(p, l, hill, s, 0.3, 1.0, o);
    double T[3][3];
    diff::dyn4_ph_T(p, o.psi, T);
    const double v1[3] = {o.vx, o.vy, o.r}, dq[3] = {o.x - s.x, o.y - s.y, o.psi - s.psi};
    for (int a = 0; a < 3; ++a) {
      const double want = l.hdt * (T[a][0] * v1[0] + T[a][1] * v1[1] + T[a][2] * v1[2]);
      worst_pose = std::max(worst_pose, std::fabs(dq[a] - want) / (1.0 + std::fabs(s[a])));
    }
  }
  std::printf("  dyn4 full vector field vs dyn4_plant with grade_force, 300 states on the "
              "hillside: worst %.2e of its scale; without grade_force the momenta miss up to %.2e "
              "of theirs; pose step vs hdt T(q1) v1 %.1e\n",
              worst, gap, worst_pose);
  EXPECT_GT(gap, 1e-2) << "the hillside's slope force is too small to see";
  EXPECT_LT(worst_pose, 1e-14);
}

// ── The golden ────────────────────────────────────────────────────────────────────────────

namespace {

// Golden-file helpers, as nav_dyn_spin_test's.
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

// The values GRL-SNAM's torch twin of the full form is held to, on DynBike.Golden's Humvee (lags
// and the low-speed blend) on the hillside. Per case: the ground's slopes, V and H at the state;
// dV/dq and T(q); the full field on the ground (q', p') and with the case's grade and side as data
// (p'); the rates on the ground; one dyn_plant_books_ground step, with the twist it reached
// before its limiters and its books (grade = - dV); and the gradient of a fixed weighting of the
// books on the state and the commands, by forward-mode duals through the ground.
// CVC_DYN_PH_FULL_GOLDEN_WRITE=1 rewrites tests/data/dyn_ph_full_golden.json; otherwise this build
// must reproduce it to 1e-9.
TEST(DynPHFull, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn_ph_full_golden.json";
  dyn_bike_params p = humvee();
  p.tau_steer = 0.05f;
  p.v_blend = 1.0f;
  p.v_width = 0.3f;
  const hillside hill;
  struct gcase {
    const char *name;
    double s[8], dcmd, acmd, grade, side;
    bool reverse;
  };
  const gcase cases[] = {
      {"across the slope",
       {-20.0, 0.0, 0.0, 12.0, -0.3, 0.02, 0.05, 0.5},
       0.05,
       1.0,
       0.0,
       0.15,
       false},
      {"up the slope, cornering",
       {5.0, -10.0, 1.4, 15.0, -1.2, 0.6, 0.25, 0.0},
       0.3,
       0.0,
       0.12,
       0.02,
       false},
      {"down the slope, braking",
       {12.0, 25.0, -1.7, 14.0, 0.8, 0.4, 0.15, -4.0},
       0.15,
       -6.0,
       -0.1,
       0.05,
       false},
      {"sliding sideways on a swell",
       {30.0, 8.0, 2.5, 10.0, 2.5, 1.1, 0.4, 1.0},
       0.45,
       2.0,
       0.03,
       -0.12,
       false},
      {"creeping", {-3.0, 3.0, 0.7, 0.6, 0.02, 0.05, 0.3, 0.3}, 0.35, 0.5, 0.05, 0.1, false},
      {"reversing downhill",
       {0.5, 0.5, -1.4, -2.0, 0.05, -0.1, 0.2, -0.5},
       0.25,
       -0.8,
       -0.02,
       -0.15,
       true},
  };
  const char *const ground_keys[] = {"along", "across", "V", "H"};
  const char *const rate_keys[] = {"dissipated", "supplied", "floor", "grade"};
  const char *const free_keys[] = {"vx", "vy", "r"};
  const char *const book_keys[] = {"dH",    "dissipated", "supplied", "floor",
                                   "grade", "limiter",    "residual"};
  const double g_books[7] = {0.9, -0.4, 0.7, 1.3, -0.6, 0.8, 1.1};
  golden_writer w;
  std::ostringstream &js = w.js;
  js << "{\n \"format\": \"cvc-dyn-ph-full-golden-1\",\n";
  js << " \"order\": [\"x\", \"y\", \"psi\", \"vx\", \"vy\", \"r\", \"delta\", \"a\"],\n";
  js << " \"ground\": {\"kind\": \"hillside\", \"formula\": \"sx (x - x0) + sy (y - y0) + A sin(kx "
        "(x - x0)) cos(ky (y - y0))\", \"sx\": "
     << golden_writer::num(hill.sx) << ", \"sy\": " << golden_writer::num(hill.sy)
     << ", \"A\": " << golden_writer::num(hill.A) << ", \"kx\": " << golden_writer::num(hill.kx)
     << ", \"ky\": " << golden_writer::num(hill.ky) << ", \"x0\": " << golden_writer::num(hill.x0)
     << ", \"y0\": " << golden_writer::num(hill.y0)
     << ", \"gradient_step\": \"1e-3 (lf + lr)\"},\n";
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
    js << (first ? "" : ", ") << "\"" << kv.first << "\": " << golden_writer::num(kv.second);
    first = false;
  }
  js << ", \"iters\": " << p.iters << "},\n \"g_books\": " << golden_writer::arr(g_books, 7)
     << ",\n \"cases\": [\n";
  const int ncase = static_cast<int>(sizeof cases / sizeof cases[0]);
  for (int c = 0; c < ncase; ++c) {
    const gcase &gc = cases[c];
    const bike_veh lim = plant_limits(p.lf + p.lr, gc.reverse);
    dyn_state<double> s;
    for (int k = 0; k < 8; ++k)
      s[k] = gc.s[k];
    double along, across;
    diff::dyn_ph_slope(p, hill, s, along, across);
    const double ground[] = {along, across, diff::dyn_ph_potential(p, hill, s),
                             diff::dyn_ph_hamiltonian(p, hill, s)};
    double dV[3], T[3][3], qd[3], pd[3], pdd[3], qdd[3];
    diff::dyn_ph_grad_q(p, hill, s, dV);
    diff::dyn_ph_T(p, s.psi, T);
    diff::dyn_ph_rhs_full(p, hill, s, qd, pd);
    diff::dyn_ph_rhs_full(p, s, gc.grade, gc.side, qdd, pdd);
    double Tf[9];
    for (int a = 0; a < 9; ++a)
      Tf[a] = T[a / 3][a % 3];
    const diff::dyn_ph_rates<double> rt = diff::dyn_ph_rates_at(p, hill, s);
    const double rates[] = {rt.dissipated, rt.supplied, rt.floor, rt.grade};
    dyn_state<double> o, o_books;
    diff::dyn_step_free<double> seen;
    diff::dyn_plant_ground<double>(p, lim, hill, s, gc.dcmd, gc.acmd, o, &seen);
    diff::dyn_ph_books<double> b;
    diff::dyn_plant_books_ground<double>(p, lim, hill, s, gc.dcmd, gc.acmd, o_books, b);
    for (int k = 0; k < 8; ++k)
      EXPECT_EQ(o_books[k], o[k]) << gc.name << ": the books' step is not dyn_plant_ground's";
    double next[8];
    for (int k = 0; k < 8; ++k)
      next[k] = o[k];
    const double twist[] = {seen.vx, seen.vy, seen.r};
    const double books[] = {b.dH,    b.dissipated, b.supplied, b.floor,
                            b.grade, b.limiter,    b.residual};
    typedef diff::dual<double> D;
    double grad[10];
    for (int j = 0; j < 10; ++j) {
      dyn_state<D> ds, od;
      for (int k = 0; k < 8; ++k)
        ds[k] = D(gc.s[k], k == j ? 1.0 : 0.0);
      diff::dyn_ph_books<D> bd;
      diff::dyn_plant_books_ground<D>(p, lim, hill, ds, D(gc.dcmd, j == 8 ? 1.0 : 0.0),
                                      D(gc.acmd, j == 9 ? 1.0 : 0.0), od, bd);
      const D bv[] = {bd.dH,    bd.dissipated, bd.supplied, bd.floor,
                      bd.grade, bd.limiter,    bd.residual};
      grad[j] = 0.0;
      for (int k = 0; k < 7; ++k)
        grad[j] += g_books[k] * bv[k].d;
    }
    w.record(gc.name, "ground", ground_keys, ground, 4);
    w.record(gc.name, "grad_q", nullptr, dV, 3);
    w.record(gc.name, "T", nullptr, Tf, 9);
    w.record(gc.name, "qdot", nullptr, qd, 3);
    w.record(gc.name, "pdot", nullptr, pd, 3);
    w.record(gc.name, "pdot_data", nullptr, pdd, 3);
    w.record(gc.name, "rates", rate_keys, rates, 4);
    w.record(gc.name, "next", nullptr, next, 8);
    w.record(gc.name, "free", free_keys, twist, 3);
    w.record(gc.name, "books", book_keys, books, 7);
    w.record(gc.name, "g_state", nullptr, grad, 8);
    w.record(gc.name, "g_delta_cmd", nullptr, grad + 8, 1);
    w.record(gc.name, "g_a_cmd", nullptr, grad + 9, 1);
    js << "  {\"name\": \"" << gc.name << "\", \"state\": " << golden_writer::arr(gc.s, 8)
       << ", \"delta_cmd\": " << golden_writer::num(gc.dcmd)
       << ", \"a_cmd\": " << golden_writer::num(gc.acmd)
       << ", \"grade\": " << golden_writer::num(gc.grade)
       << ", \"side\": " << golden_writer::num(gc.side)
       << ",\n   \"limits\": {\"L\": " << golden_writer::num(lim.L)
       << ", \"vmax\": " << golden_writer::num(lim.vmax)
       << ", \"hdt\": " << golden_writer::num(lim.hdt)
       << ", \"allow_reverse\": " << lim.allow_reverse << "}"
       << ",\n   \"ground\": " << golden_writer::obj(ground_keys, ground, 4)
       << ",\n   \"grad_q\": " << golden_writer::arr(dV, 3)
       << ",\n   \"T\": " << golden_writer::arr(Tf, 9)
       << ",\n   \"qdot\": " << golden_writer::arr(qd, 3)
       << ", \"pdot\": " << golden_writer::arr(pd, 3)
       << ", \"pdot_data\": " << golden_writer::arr(pdd, 3)
       << ",\n   \"rates\": " << golden_writer::obj(rate_keys, rates, 4)
       << ",\n   \"next\": " << golden_writer::arr(next, 8)
       << ",\n   \"free\": " << golden_writer::obj(free_keys, twist, 3)
       << ",\n   \"books\": " << golden_writer::obj(book_keys, books, 7)
       << ",\n   \"g_state\": " << golden_writer::arr(grad, 8)
       << ",\n   \"g_delta_cmd\": " << golden_writer::num(grad[8])
       << ", \"g_a_cmd\": " << golden_writer::num(grad[9]) << "}" << (c + 1 < ncase ? "," : "")
       << "\n";
  }
  js << " ]\n}\n";
  const char *write = std::getenv("CVC_DYN_PH_FULL_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &c : want.get_child("cases")) {
    const boost::property_tree::ptree &g = c.second;
    read_named(g, "ground", ground_keys, 4, golden);
    read_listed(g, "grad_q", golden);
    read_listed(g, "T", golden);
    read_listed(g, "qdot", golden);
    read_listed(g, "pdot", golden);
    read_listed(g, "pdot_data", golden);
    read_named(g, "rates", rate_keys, 4, golden);
    read_listed(g, "next", golden);
    read_named(g, "free", free_keys, 3, golden);
    read_named(g, "books", book_keys, 7, golden);
    read_listed(g, "g_state", golden);
    golden.push_back(g.get<double>("g_delta_cmd"));
    golden.push_back(g.get<double>("g_a_cmd"));
  }
  ASSERT_EQ(golden.size(), w.now.size());
  for (std::size_t k = 0; k < w.now.size(); ++k)
    EXPECT_LE(std::fabs(w.now[k] - golden[k]),
              1e-9 * std::max(1.0, std::max(std::fabs(w.now[k]), std::fabs(golden[k]))))
        << w.what[k] << ": golden " << golden[k] << ", now " << w.now[k];
}
