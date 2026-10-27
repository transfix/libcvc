/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph's SE(2) port-Hamiltonian vehicle (inc/cvc/ph/se2_vehicle.h): parity with the JAX
// package it ports, and the structure it must have whatever its weights are.
//
// tests/data/ph_se2_golden.json was generated once, outside CI, from the JAX package in float64
// (a fit of the Humvee to Jolt runs: 64 features, 1,691 parameters): the model's pieces and one
// Lie-IMEX step at 16 states, with and without a noise increment, and a 25-step rollout. Double
// must reproduce it to rounding; float to float precision.
//
// Analytic checks need no golden: the energy balance dH/dt = -v.D(v)v + v.G(v)u of the drift (the
// coadjoint and potential terms do no work), the hand-written linear-tire bicycle the model reduces
// to with its GPs off, and the step's second order.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ph/mat3.h>
#include <cvc/ph/se2_vehicle.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace {

using namespace cvc::ph;
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

fs::path golden_path() { return fs::path(__FILE__).parent_path() / "data" / "ph_se2_golden.json"; }

const pt::ptree &golden() {
  static const pt::ptree tree = [] {
    pt::ptree t;
    pt::read_json(golden_path().string(), t);
    return t;
  }();
  return tree;
}

const se2_model &golden_model() {
  static const se2_model m = se2_model::load(golden_path().string());
  return m;
}

std::vector<double> numbers(const pt::ptree &node, const std::string &key) {
  std::vector<double> out;
  for (const auto &kv : node.get_child(key))
    out.push_back(kv.second.get_value<double>());
  return out;
}

// |a - b| <= tol (1 + |b|)
void expect_close(const std::vector<double> &got, const std::vector<double> &want, double tol,
                  const std::string &what) {
  ASSERT_EQ(got.size(), want.size()) << what;
  for (std::size_t i = 0; i < got.size(); ++i)
    EXPECT_LE(std::fabs(got[i] - want[i]), tol * (1.0 + std::fabs(want[i])))
        << what << "[" << i << "]: got " << got[i] << ", JAX " << want[i];
}

template <class C> std::vector<double> as_doubles(const C &c) {
  return std::vector<double>(c.begin(), c.end());
}

// The worst |a - b| / (1 + |b|) over a case set, for the printout.
double worst_rel(const std::vector<double> &got, const std::vector<double> &want) {
  double w = 0.0;
  for (std::size_t i = 0; i < got.size(); ++i)
    w = std::max(w, std::fabs(got[i] - want[i]) / (1.0 + std::fabs(want[i])));
  return w;
}

TEST(PhSe2, LoadsTheGoldenModel) {
  const se2_model &m = golden_model();
  EXPECT_EQ(m.setup.control_dim, 2);
  EXPECT_EQ(m.setup.features, 64);
  EXPECT_TRUE(m.setup.v_gp);
  EXPECT_EQ(m.params.size(), 1691u);    // the Phase A count, filter noise included
  EXPECT_NEAR(m.interval, 0.02, 1e-15); // the median step of the dataset's float times
}

TEST(PhSe2, SaveAndLoadRoundTripsExactly) {
  const se2_model &m = golden_model();
  const se2_model back = se2_model::from_json(m.to_json());
  std::vector<double> a, b;
  m.params.for_each_leaf([&](const char *, const double *d, std::size_t n, se2_leaf_group) {
    a.insert(a.end(), d, d + n);
  });
  back.params.for_each_leaf([&](const char *, const double *d, std::size_t n, se2_leaf_group) {
    b.insert(b.end(), d, d + n);
  });
  ASSERT_EQ(a, b);
  EXPECT_EQ(m.setup.D.frequency, back.setup.D.frequency);
  EXPECT_EQ(m.setup.V.phase, back.setup.V.phase);
  EXPECT_EQ(m.control_scale, back.control_scale);
  EXPECT_EQ(m.provenance, back.provenance);
}

// Double against JAX float64, piece by piece and one step, at 16 states.
TEST(PhSe2, MatchesJaxInDouble) {
  const se2_model &m = golden_model();
  const se2_vehicle<double> veh(m.setup, se2_draw<double>(m), m.twist_scale);
  expect_close(as_doubles(veh.inverse_mass()), numbers(golden(), "inverse_mass"), 1e-13,
               "inverse_mass");
  double worst_step = 0.0;
  int n = 0;
  for (const auto &kv : golden().get_child("forward")) {
    const pt::ptree &c = kv.second;
    const auto s = numbers(c, "state");
    const auto inc = numbers(c, "increment");
    const std::string tag = "case " + std::to_string(n++);
    double f[6];
    veh.vector_field(s.data(), s.data() + 6, f);
    expect_close(std::vector<double>(f, f + 6), numbers(c, "vector_field"), 1e-13,
                 tag + " vector_field");
    expect_close(as_doubles(veh.effective_damping(s.data())), numbers(c, "effective_damping"),
                 1e-13, tag + " effective_damping");
    expect_close(as_doubles(veh.dissipation({s[3], s[4], s[5]})), numbers(c, "dissipation"), 1e-13,
                 tag + " dissipation");
    double G[6];
    veh.control_matrix_at({s[3], s[4], s[5]}, G);
    expect_close(std::vector<double>(G, G + 6), numbers(c, "control_matrix"), 1e-13,
                 tag + " control_matrix");
    EXPECT_NEAR(veh.hamiltonian(s.data()), c.get<double>("hamiltonian"),
                1e-13 * (1.0 + std::fabs(c.get<double>("hamiltonian"))))
        << tag;
    EXPECT_NEAR(veh.potential(s[0], s[1]), c.get<double>("potential"), 1e-13) << tag;
    expect_close(as_doubles(veh.grad_potential(s[0], s[1])), numbers(c, "grad_potential"), 1e-13,
                 tag + " grad_potential");
    double next[6], noisy[6];
    veh.step(s.data(), s.data() + 6, m.interval, nullptr, next);
    veh.step(s.data(), s.data() + 6, m.interval, inc.data(), noisy);
    const auto want = numbers(c, "step"), want_inc = numbers(c, "step_increment");
    expect_close(std::vector<double>(next, next + 6), want, 1e-13, tag + " step");
    expect_close(std::vector<double>(noisy, noisy + 6), want_inc, 1e-13, tag + " step_increment");
    worst_step = std::max({worst_step, worst_rel(std::vector<double>(next, next + 6), want),
                           worst_rel(std::vector<double>(noisy, noisy + 6), want_inc)});
  }
  EXPECT_EQ(n, 16);
  std::printf("  one step, double vs JAX float64: worst |d| / (1 + |x|) %.2e over %d states\n",
              worst_step, n);
}

// The same in float: float tier (the JAX fits trained in float32).
TEST(PhSe2, MatchesJaxInFloat) {
  const se2_model &m = golden_model();
  const se2_vehicle<float> veh(m.setup, se2_draw<float>(m), m.twist_scale);
  double worst = 0.0;
  for (const auto &kv : golden().get_child("forward")) {
    const pt::ptree &c = kv.second;
    const auto s = numbers(c, "state");
    const auto inc = numbers(c, "increment");
    std::vector<float> sf(s.begin(), s.end()), incf(inc.begin(), inc.end());
    float f[6], next[6], noisy[6];
    veh.vector_field(sf.data(), sf.data() + 6, f);
    veh.step(sf.data(), sf.data() + 6, static_cast<float>(m.interval), nullptr, next);
    veh.step(sf.data(), sf.data() + 6, static_cast<float>(m.interval), incf.data(), noisy);
    // The drift's magnitude sets its scale; the step's state carries positions of 100s of metres.
    const auto vf = numbers(c, "vector_field");
    double scale = 1.0;
    for (double x : vf)
      scale = std::max(scale, std::fabs(x));
    for (int i = 0; i < 6; ++i)
      EXPECT_LE(std::fabs(f[i] - vf[i]), 2e-5 * scale) << "vector_field " << i;
    expect_close(std::vector<double>(next, next + 6), numbers(c, "step"), 2e-6, "float step");
    expect_close(std::vector<double>(noisy, noisy + 6), numbers(c, "step_increment"), 2e-6,
                 "float step_increment");
    worst = std::max(worst, worst_rel(std::vector<double>(next, next + 6), numbers(c, "step")));
  }
  std::printf("  one step, float vs JAX float64: worst |d| / (1 + |x|) %.2e\n", worst);
}

TEST(PhSe2, RolloutMatchesJax) {
  const se2_model &m = golden_model();
  const pt::ptree &r = golden().get_child("rollout");
  const int steps = r.get<int>("steps");
  const auto x0 = numbers(r, "initial");
  const auto u = numbers(r, "controls");
  const auto want = numbers(r, "path");
  const se2_vehicle<double> veh(m.setup, se2_draw<double>(m), m.twist_scale);
  std::vector<double> path(6 * (steps + 1));
  veh.rollout(x0.data(), u.data(), steps, m.interval, m.substeps, path.data());
  const std::vector<double> got(path.begin() + 6, path.end());
  expect_close(got, want, 1e-12, "rollout");
  std::printf("  25-step rollout, double vs JAX: worst %.2e\n", worst_rel(got, want));
}

// dH/dt along the drift is -v.D(v)v + v.G(v)u: ad*_v p does no work, and the potential's force
// returns exactly what moving through V costs. With the fitted GPs, at every golden state.
TEST(PhSe2, EnergyBalanceIsExact) {
  const se2_model &m = golden_model();
  const se2_vehicle<double> veh(m.setup, se2_draw<double>(m), m.twist_scale);
  for (const auto &kv : golden().get_child("forward")) {
    const auto s = numbers(kv.second, "state");
    double f[6];
    veh.vector_field(s.data(), s.data() + 6, f);
    se2_stage<double> st;
    veh.stage(s.data(), s.data() + 6, st);
    // dH/dt = grad V . d(x, y) + p . dv   (dH/dv = M v = p)
    const double dH = st.grad_v[0] * f[0] + st.grad_v[1] * f[1] + st.p[0] * f[3] + st.p[1] * f[4] +
                      st.p[2] * f[5];
    const vec3<double> dv = matvec(st.D, st.v);
    double supplied = 0.0, dissipated = 0.0;
    for (int i = 0; i < 3; ++i) {
      dissipated += st.v[i] * dv[i];
      supplied += st.v[i] * (st.G[2 * i] * s[6] + st.G[2 * i + 1] * s[7]);
    }
    EXPECT_GE(dissipated, -1e-12); // D is positive definite by construction
    EXPECT_NEAR(dH, supplied - dissipated, 1e-11 * (1.0 + std::fabs(supplied) + dissipated));
    // The supplied power is the output conjugate to the input: y . u, y = G(v)^T v.
    double y[2];
    veh.output(s.data(), y);
    EXPECT_NEAR(y[0] * s[6] + y[1] * s[7], supplied, 1e-12 * (1.0 + std::fabs(supplied)));
  }
}

// M is the inverse of M^-1.
TEST(PhSe2, MassInvertsTheInverseMass) {
  const se2_model &m = golden_model();
  const se2_vehicle<double> veh(m.setup, se2_draw<double>(m), m.twist_scale);
  const mat3<double> I = mul(veh.mass(), veh.inverse_mass());
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NEAR(I[3 * i + j], i == j ? 1.0 : 0.0, 1e-12);
}

// The diffusion's energy: over one short step, H's mean gain with the noise, less the step
// without it, is noise_power() h. Antithetic pairs (z, -z) cancel the terms odd in the noise, so
// what is left is the quadratic one the drift's balance does not book.
TEST(PhSe2, NoisePowerIsTheEnergyTheDiffusionAdds) {
  const se2_model &m = golden_model();
  const se2_vehicle<double> veh(m.setup, se2_draw<double>(m), m.twist_scale);
  const auto s = numbers(golden().get_child("forward").begin()->second, "state");
  const double h = 1e-4;
  double base[6];
  veh.step(s.data(), s.data() + 6, h, nullptr, base);
  const double h0 = veh.hamiltonian(base);
  const auto &sigma = veh.weights().diffusion;
  std::mt19937 rng(11);
  std::normal_distribution<double> z(0.0, 1.0);
  const int n = 20000;
  double sum = 0.0, sum2 = 0.0;
  for (int k = 0; k < n; ++k) {
    double plus[3], minus[3], a[6], b[6];
    for (int i = 0; i < 3; ++i) {
      plus[i] = std::sqrt(h) * sigma[i] * z(rng);
      minus[i] = -plus[i];
    }
    veh.step(s.data(), s.data() + 6, h, plus, a);
    veh.step(s.data(), s.data() + 6, h, minus, b);
    const double d = 0.5 * (veh.hamiltonian(a) + veh.hamiltonian(b)) - h0;
    sum += d;
    sum2 += d * d;
  }
  const double mean = sum / n, se = std::sqrt(std::max(0.0, sum2 / n - mean * mean) / n);
  const double want = veh.noise_power() * h;
  std::printf("  noise energy over %.0e s: %.4e J measured (se %.1e), noise_power h = %.4e J\n", h,
              mean, se, want);
  EXPECT_GT(want, 0.0);
  EXPECT_NEAR(mean, want, 4.0 * se + 0.02 * want);
}

// With its GP weights zero and its levels set from a linear-tire bicycle frozen at 10 m/s, the
// model's drift is that bicycle's body-frame equations, written out by hand.
se2_model bicycle_model() {
  const double m = 2000.0, Iz = 3000.0, lf = 1.4, lr = 1.6, Cf = 8e4, Cr = 9e4, u0 = 10.0;
  se2_init init;
  init.features = 8;
  init.v_gp = false;
  init.initial_randomness = 0.0;
  se2_model model = se2_model::initialize(init, 7, {1.0, 1.0, 1.0}, {1.0, 1.0}, 0.02);
  for (se2_gp *g : {&model.params.D, &model.params.G, &model.params.V})
    std::fill(g->mean.begin(), g->mean.end(), 0.0);
  const mat3<double> minv = {1 / m, 0, 0, 0, 1 / m, 0, 0, 0, 1 / Iz};
  const auto lm = spd_levels_of(minv);
  std::copy(lm.begin(), lm.end(), model.params.minv_level.begin());
  mat3<double> D{};
  D[0] = 30.0;
  D[4] = (Cf + Cr) / u0 + 1e-9;
  D[5] = D[7] = (lf * Cf - lr * Cr) / u0;
  D[8] = (lf * lf * Cf + lr * lr * Cr) / u0 + 1e-9;
  const auto ld = spd_levels_of(D);
  model.params.D.level.assign(ld.begin(), ld.end());
  model.params.G.level = {m, 0.0, 0.0, Cf, 0.0, lf * Cf};
  model.params.V.level = {0.0, 0.0};
  return model;
}

TEST(PhSe2, ReducesToTheLinearTireBicycle) {
  const se2_model model = bicycle_model();
  const se2_vehicle<double> veh(model.setup, se2_draw<double>(model), model.twist_scale);
  const double m = 2000.0, Iz = 3000.0, lf = 1.4, lr = 1.6, Cf = 8e4, Cr = 9e4, u0 = 10.0;
  const double s[8] = {3.0, -2.0, 0.4, 10.0, 0.3, 0.2, 0.5, 0.05};
  double f[6];
  veh.vector_field(s, s + 6, f);
  const double vx = s[3], vy = s[4], r = s[5], drive = s[6], steer = s[7];
  const double ax = (-30.0 * vx + m * drive) / m + vy * r;
  const double fy = Cf * (steer - (vy + lf * r) / u0) - Cr * (vy - lr * r) / u0;
  const double ay = fy / m - vx * r;
  const double ar = (lf * Cf * (steer - (vy + lf * r) / u0) + lr * Cr * (vy - lr * r) / u0) / Iz;
  EXPECT_NEAR(f[0], std::cos(0.4) * vx - std::sin(0.4) * vy, 1e-12);
  EXPECT_NEAR(f[1], std::sin(0.4) * vx + std::cos(0.4) * vy, 1e-12);
  EXPECT_NEAR(f[2], r, 1e-15);
  EXPECT_NEAR(f[3], ax, 1e-9);
  EXPECT_NEAR(f[4], ay, 1e-9);
  EXPECT_NEAR(f[5], ar, 1e-9);
}

// Lie-IMEX is second order: against a fine RK4 of the same drift over one 0.02 s interval, the
// error falls about 4x per halving of the step.
TEST(PhSe2, StepIsSecondOrder) {
  const se2_model model = bicycle_model();
  const se2_vehicle<double> veh(model.setup, se2_draw<double>(model), model.twist_scale);
  const double s[6] = {3.0, -2.0, 0.4, 10.0, 0.3, 0.2};
  const double u[2] = {0.5, 0.05};
  const auto rk4 = [&](const double *x0, double h, int n, double *out) {
    double x[6];
    std::copy(x0, x0 + 6, x);
    for (int k = 0; k < n; ++k) {
      double k1[6], k2[6], k3[6], k4[6], t[6];
      veh.vector_field(x, u, k1);
      for (int i = 0; i < 6; ++i)
        t[i] = x[i] + 0.5 * h * k1[i];
      veh.vector_field(t, u, k2);
      for (int i = 0; i < 6; ++i)
        t[i] = x[i] + 0.5 * h * k2[i];
      veh.vector_field(t, u, k3);
      for (int i = 0; i < 6; ++i)
        t[i] = x[i] + h * k3[i];
      veh.vector_field(t, u, k4);
      for (int i = 0; i < 6; ++i)
        x[i] += h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    std::copy(x, x + 6, out);
  };
  // The bicycle's lateral modes are fast (~ -100 /s), so the comparison starts at 8 substeps,
  // where every step is inside the asymptotic range.
  double ref[6];
  rk4(s, 0.02 / 4000, 4000, ref);
  std::vector<double> err;
  for (int sub : {8, 16, 32, 64}) {
    double out[6];
    veh.interval_step(s, u, 0.02, sub, nullptr, out);
    double e = 0.0;
    for (int i = 0; i < 6; ++i)
      e = std::max(e, std::fabs(out[i] - ref[i]));
    err.push_back(e);
  }
  for (std::size_t i = 1; i < err.size(); ++i) {
    const double ratio = err[i - 1] / err[i];
    std::printf("  substeps x2: error %.3e -> %.3e (x%.2f)\n", err[i - 1], err[i], ratio);
    EXPECT_GT(ratio, 3.5);
    EXPECT_LT(ratio, 4.5);
  }
}

// spd_from_levels' reverse against central differences.
TEST(PhMat3, SpdReverseMatchesFiniteDifferences) {
  const double l[6] = {0.3, -0.7, 1.1, 0.4, -0.25, 0.9};
  const mat3<double> w = {0.3, -1.2, 0.5, 2.0, 0.1, -0.4, 0.7, 0.9, -1.5}; // loss = <w, A>
  double dl[6] = {};
  spd_from_levels_vjp(l, w, dl);
  for (int i = 0; i < 6; ++i) {
    double lp[6], lm[6];
    std::copy(l, l + 6, lp);
    std::copy(l, l + 6, lm);
    lp[i] += 1e-6;
    lm[i] -= 1e-6;
    const auto ap = spd_from_levels(lp), am = spd_from_levels(lm);
    double fd = 0.0;
    for (int k = 0; k < 9; ++k)
      fd += w[k] * (ap[k] - am[k]) / 2e-6;
    EXPECT_NEAR(dl[i], fd, 1e-7 * (1.0 + std::fabs(fd))) << i;
  }
  const auto a = spd_from_levels(l);
  const auto back = spd_levels_of(a);
  for (int i = 0; i < 6; ++i)
    EXPECT_NEAR(back[i], l[i], 1e-12);
}

TEST(PhMat3, Solve3PivotsAndSolves) {
  const mat3<double> a = {0.0, 2.0, 1.0, 1.0, 0.0, 3.0, 4.0, 1.0, 0.0}; // a zero first pivot
  const vec3<double> x = {1.5, -2.0, 0.25};
  const vec3<double> b = matvec(a, x);
  const vec3<double> got = solve3(a, b);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(got[i], x[i], 1e-14);
  const vec3<double> bt = tmatvec(a, x);
  const vec3<double> gt = solve3_transposed(a, bt);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(gt[i], x[i], 1e-14);
}

// The features are a Matern kernel's: their inner products approach k(r) = (1 + sqrt5 r +
// 5/3 r^2) exp(-sqrt5 r) as the count grows (nu 5/2, ell 1).
TEST(PhRff, FeaturesApproachTheMaternKernel) {
  rng r(11);
  const rff f = rff::draw(r, 3, 20000, 2.5, 1.0);
  const double a[3] = {0.1, -0.2, 0.3};
  std::vector<double> ta(f.count), tb(f.count);
  for (double dist : {0.0, 0.3, 1.0, 2.0}) {
    const double b[3] = {0.1 + dist, -0.2, 0.3};
    f.angles(a, ta.data());
    f.angles(b, tb.data());
    double k = 0.0;
    for (int i = 0; i < f.count; ++i)
      k += 2.0 / f.count * std::cos(ta[i]) * std::cos(tb[i]);
    const double s5 = std::sqrt(5.0) * dist;
    const double want = (1.0 + s5 + s5 * s5 / 3.0) * std::exp(-s5);
    EXPECT_NEAR(k, want, 0.03) << "distance " << dist;
  }
}

TEST(PhRng, IsTheSameSequenceEverywhere) {
  // The integers come from integer arithmetic alone (no libm), so every platform agrees on them
  // (the pinned values are an independent Python transcription of xoshiro256** and splitmix64).
  rng r(42);
  const std::uint64_t first[3] = {r.next(), r.next(), r.next()};
  std::printf("  rng(42): %016llx %016llx %016llx\n", static_cast<unsigned long long>(first[0]),
              static_cast<unsigned long long>(first[1]), static_cast<unsigned long long>(first[2]));
  EXPECT_EQ(first[0], 0x15780b2e0c2ec716ULL);
  EXPECT_EQ(first[1], 0x6104d9866d113a7eULL);
  EXPECT_EQ(first[2], 0xae17533239e499a1ULL);
  rng a(7), b(7), c = rng::derive(7, 1), d = rng::derive(7, 2);
  for (int i = 0; i < 100; ++i)
    ASSERT_EQ(a.next(), b.next());
  EXPECT_NE(c.next(), d.next());
  // Moments: uniform, normal, chi-square(5).
  rng m(3);
  const int n = 200000;
  double su = 0, sn = 0, sn2 = 0, sc = 0;
  for (int i = 0; i < n; ++i) {
    su += m.uniform();
    const double z = m.normal();
    sn += z;
    sn2 += z * z;
    sc += m.chi_square(5.0);
  }
  EXPECT_NEAR(su / n, 0.5, 0.005);
  EXPECT_NEAR(sn / n, 0.0, 0.01);
  EXPECT_NEAR(sn2 / n, 1.0, 0.01);
  EXPECT_NEAR(sc / n, 5.0, 0.05);
}

} // namespace
