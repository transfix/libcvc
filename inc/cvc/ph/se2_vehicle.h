/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::se2: a learned port-Hamiltonian ground vehicle on SE(2), a native port of the SE(2)
// variant of LieSPHGP (stochastic port-Hamiltonian Gaussian processes on Lie groups,
// CVC-Lab/LieSPHGP; its SE(2) vehicle package was written for the Phase A fit to Jolt). No
// physical constant anywhere; everything is learned from logged runs.
//
// State s = (x, y, psi, v) with v = (v_x, v_y, r) the body twist, inputs u (control_dim of them,
// divided by their training RMS); momenta p = M v and H = 1/2 p . M^-1 p + V(x, y):
//
// clang-format off
//   d(x, y) = R(psi) (v_x, v_y) dt,   dpsi = r dt
//   dp = ( ad*_v p - [R(psi)^T grad V; 0] - D(v) v + G(v) u ) dt,
//   ad*_v p = (p_y r, -p_x r, p_x v_y - p_y v_x)
//   dv = M^-1 dp + Sigma dW                                  (Sigma diagonal: 3 numbers)
// clang-format on
//
// The drift's energy balance is dH/dt = -v^T D v + y^T u, y = G(v)^T v (output()); the noise adds
// noise_power() on average.
//
// ad*_v p is SE(2)'s coadjoint action: the r v_y and -r v_x terms of a vehicle's body-frame
// equations. Each piece is a learned level plus a Gaussian-process residual on random features
// (rff.h), q(W) = N(mean, diag e^(2 log_std)):
//
//   M^-1 = spd(level)                               a constant (a rigid body's inertia)
//   D(v) = spd(level + GP_D(v / twist_scale))        positive definite: it can only dissipate
//   G(v) = level + GP_G(v / twist_scale)             3 x control_dim, row-major
//   V(x, y) = level . (x, y) + GP_V((x, y) / V_length_scale)
//
// spd() is mat3.h's six-level Cholesky form. twist_scale is the RMS of each twist channel over the
// training flights. Only M^-1 D, M^-1 G and M^-1 grad V are identifiable, plus M's shape through
// ad*_v p.
//
// One step of size h (Lie-IMEX: the damping implicit, everything else explicit; SO(2) is abelian,
// so the heading stays a continuous angle). With e = M^-1 (ad*_v p - body + G u) and K = M^-1 D(v)
// at each stage, and the twist increment eta = sqrt(h) Sigma z:
//
// clang-format off
//   (I + h K_1) v_p = v + h e_1 + eta,
//   (x, y)_p = (x, y) + h R(psi) v,                          psi_p = psi + h r
//   (I + h/2 K_2) v' = v + h/2 (e_1 + e_2 - K_1 v) + eta,
//   (x, y)' = (x, y) + h/2 (R(psi) v + R(psi_p) v_p),        psi' = psi + h/2 (r + r_p)
// clang-format on
//
// It is second order (a Heun predictor-corrector with the damping implicit), not structure-
// preserving: it keeps no discrete energy identity.
//
// se2_vehicle evaluates one weight draw (the posterior mean, or a sample) and is templated on the
// scalar, so the same code runs in float and double. Parity with the JAX package is a test
// (ph_se2_test, against goldens generated once from it).

#ifndef __CVC_PH_SE2_VEHICLE_H__
#define __CVC_PH_SE2_VEHICLE_H__

#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/ph/mat3.h>
#include <cvc/ph/rff.h>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ph {

constexpr int kSe2MaxControls = 4;
constexpr int kSe2State = 6; // x, y, psi, v_x, v_y, r

// A Gaussian-process subnetwork's trainable values.
struct se2_gp {
  std::vector<double> mean;    // [features x outputs], row-major
  std::vector<double> log_std; // [features x outputs]
  std::vector<double> level;   // D: 6 levels; G: 3 x control_dim, row-major; V: 2
};

// Which optimizer a parameter leaf belongs to (LieSPHGP's train.py: Prodigy on each subnetwork's
// level, Adam on everything else).
enum class se2_leaf_group { adam, D_level, G_level, V_level, Minv_level };

struct se2_params {
  std::array<double, 6> minv_level{};
  se2_gp D, G, V;
  std::array<double, 3> process_log_sigma{}; // the diffusion Sigma, log
  std::vector<double> likelihood_log_sigma;  // [position, heading, velocity, yaw rate]; empty
                                             // when the model was not fit by the filter

  // fn(name, data, size, group) for every leaf in LieSPHGP's checkpoint order (sorted keys).
  template <class F> void for_each_leaf(F &&fn) {
    fn("D.level", D.level.data(), D.level.size(), se2_leaf_group::D_level);
    fn("D.log_std", D.log_std.data(), D.log_std.size(), se2_leaf_group::adam);
    fn("D.mean", D.mean.data(), D.mean.size(), se2_leaf_group::adam);
    fn("G.level", G.level.data(), G.level.size(), se2_leaf_group::G_level);
    fn("G.log_std", G.log_std.data(), G.log_std.size(), se2_leaf_group::adam);
    fn("G.mean", G.mean.data(), G.mean.size(), se2_leaf_group::adam);
    fn("Minv.level", minv_level.data(), minv_level.size(), se2_leaf_group::Minv_level);
    fn("V.level", V.level.data(), V.level.size(), se2_leaf_group::V_level);
    fn("V.log_std", V.log_std.data(), V.log_std.size(), se2_leaf_group::adam);
    fn("V.mean", V.mean.data(), V.mean.size(), se2_leaf_group::adam);
    if (!likelihood_log_sigma.empty())
      fn("likelihood.log_sigma", likelihood_log_sigma.data(), likelihood_log_sigma.size(),
         se2_leaf_group::adam);
    fn("process.log_sigma", process_log_sigma.data(), process_log_sigma.size(),
       se2_leaf_group::adam);
  }
  template <class F> void for_each_leaf(F &&fn) const {
    const_cast<se2_params *>(this)->for_each_leaf(
        [&](const char *name, double *data, std::size_t size, se2_leaf_group group) {
          fn(name, static_cast<const double *>(data), size, group);
        });
  }
  std::size_t size() const;
  // The same shapes, every value zero (a gradient).
  se2_params zeros_like() const;
};

// The random features and the options that are not trained.
struct se2_setup {
  int control_dim = 2;
  int features = 64;
  double matern_nu = 2.5, matern_length = 1.0;
  bool v_gp = true;
  double v_length = 50.0; // V's GP reads (x, y) / v_length
  rff D, G, V;            // inputs 3, 3, 2

  int g_outputs() const { return 3 * control_dim; }
};

// How a model is initialized (LieSPHGP's model.* keys; train.py's defaults for the Phase A fits).
struct se2_init {
  int control_dim = 2;
  int features = 64;
  double matern_nu = 2.5, matern_length = 1.0;
  bool v_gp = true;
  double v_length = 50.0;
  double initial_log_std = -5.0;
  double initial_inverse_mass = 1.0;          // M^-1's level starts at this times I
  double initial_dissipation = 1.0;           // and D's
  double initial_control_gp_mean_scale = 0.0; // G's GP mean starts at this times N(0, 1) / n
  double initial_diffusion = 0.1;
  double initial_observation_sigma = 0.1; // only with_likelihood
  double initial_randomness = 0.1;        // every level and noise scale + this times N(0, 1)
  bool with_likelihood = false;
};

// A model: its features, parameters, the data statistics it reads its inputs with, and where it
// came from. The file is JSON, format "cvc-ph-se2/1".
struct se2_model {
  se2_setup setup;
  se2_params params;
  std::array<double, 3> twist_scale{1.0, 1.0, 1.0};
  std::vector<double> control_scale; // the inputs are divided by these (their training RMS)
  double interval = 0.02;            // s between samples
  int substeps = 1;                  // Lie-IMEX steps per interval
  std::vector<std::pair<std::string, std::string>> provenance;

  // A fresh model: features drawn and parameters initialized from `seed` (LieSPHGP's
  // initialize_parameters with this library's random numbers).
  static se2_model initialize(const se2_init &init, std::uint64_t seed,
                              const std::array<double, 3> &twist_scale,
                              const std::vector<double> &control_scale, double interval,
                              int substeps = 1);

  void save(const std::string &path) const;
  std::string to_json() const;
  static se2_model load(const std::string &path); // throws std::runtime_error
  static se2_model from_json(const std::string &text);
};

// One weight draw: what the vector field reads.
template <class T> struct se2_weights {
  int control_dim = 2;
  std::vector<T> wD, wG, wV; // the GP weights
  std::array<T, 6> minv_level{}, d_level{};
  std::vector<T> g_level; // 3 x control_dim
  std::array<T, 2> v_level{};
  std::array<T, 3> diffusion{};
};

// The posterior mean (W = mean), or W = mean + e^log_std z with z given ([features x outputs] for
// each of D, G, V; null for the mean).
template <class T>
se2_weights<T> se2_draw(const se2_model &m, const double *zD = nullptr, const double *zG = nullptr,
                        const double *zV = nullptr) {
  se2_weights<T> w;
  w.control_dim = m.setup.control_dim;
  const auto draw = [](const se2_gp &gp, const double *z, std::vector<T> &out) {
    out.resize(gp.mean.size());
    for (std::size_t i = 0; i < gp.mean.size(); ++i)
      out[i] = static_cast<T>(z ? gp.mean[i] + std::exp(gp.log_std[i]) * z[i] : gp.mean[i]);
  };
  draw(m.params.D, zD, w.wD);
  draw(m.params.G, zG, w.wG);
  draw(m.params.V, zV, w.wV);
  for (int i = 0; i < 6; ++i) {
    w.minv_level[i] = static_cast<T>(m.params.minv_level[i]);
    w.d_level[i] = static_cast<T>(m.params.D.level[i]);
  }
  w.g_level.assign(m.params.G.level.begin(), m.params.G.level.end());
  for (int i = 0; i < 2; ++i)
    w.v_level[i] = static_cast<T>(m.params.V.level[i]);
  for (int i = 0; i < 3; ++i)
    w.diffusion[i] = static_cast<T>(std::exp(m.params.process_log_sigma[i]));
  return w;
}

// Everything one evaluation of the drift computes, kept for a reverse pass.
template <class T> struct se2_stage {
  T c = T(0), s = T(0); // cos, sin of the heading
  vec3<T> v{};          // the twist
  std::array<T, kSe2MaxControls> u{};
  std::array<T, 2> dxy{};            // R(psi) (v_x, v_y)
  vec3<T> p{};                       // M v
  std::array<T, 2> grad_v{}, body{}; // grad V (world), R^T grad V
  vec3<T> coad{};                    // ad*_v p
  std::array<T, 6> d_levels{};       // D's level + GP
  mat3<T> D{};
  std::array<T, 3 * kSe2MaxControls> G{};
  vec3<T> q{};                              // ad*_v p - body + G u
  vec3<T> e{};                              // M^-1 q
  mat3<T> K{};                              // M^-1 D
  vec3<T> z{};                              // v / twist_scale
  std::vector<T> theta_D, theta_G, theta_V; // the features' angles
};

template <class T> class se2_vehicle {
public:
  se2_vehicle(const se2_setup &setup, se2_weights<T> weights, const std::array<double, 3> &scale)
      : setup_(&setup), w_(std::move(weights)) {
    for (int i = 0; i < 3; ++i)
      twist_scale_[i] = static_cast<T>(scale[i]);
    minv_ = spd_from_levels(w_.minv_level.data());
  }

  const se2_setup &setup() const { return *setup_; }
  const se2_weights<T> &weights() const { return w_; }
  const std::array<T, 3> &twist_scale() const { return twist_scale_; }
  int control_dim() const { return w_.control_dim; }

  const mat3<T> &inverse_mass() const { return minv_; }

  // D's six levels at the twist (level + GP); theta receives the features' angles.
  std::array<T, 6> dissipation_levels(const vec3<T> &v, vec3<T> &z, T *theta) const {
    for (int i = 0; i < 3; ++i)
      z[i] = v[i] / twist_scale_[i];
    std::array<T, 6> gp{};
    setup_->D.gp(z.data(), w_.wD.data(), 6, gp.data(), theta);
    for (int i = 0; i < 6; ++i)
      gp[i] += w_.d_level[i];
    return gp;
  }

  mat3<T> dissipation(const vec3<T> &v) const {
    vec3<T> z;
    std::vector<T> theta(setup_->D.count);
    const auto l = dissipation_levels(v, z, theta.data());
    return spd_from_levels(l.data());
  }

  // G(v), 3 x control_dim row-major, into out.
  void control_matrix(const vec3<T> &z, T *out, T *theta) const {
    const int n = setup_->g_outputs();
    setup_->G.gp(z.data(), w_.wG.data(), n, out, theta);
    for (int i = 0; i < n; ++i)
      out[i] += w_.g_level[i];
  }
  void control_matrix_at(const vec3<T> &v, T *out) const {
    vec3<T> z;
    for (int i = 0; i < 3; ++i)
      z[i] = v[i] / twist_scale_[i];
    std::vector<T> theta(setup_->G.count);
    control_matrix(z, out, theta.data());
  }

  T potential(T x, T y) const {
    T value = w_.v_level[0] * x + w_.v_level[1] * y;
    if (setup_->v_gp) {
      const T inv = static_cast<T>(1.0 / setup_->v_length);
      const T zz[2] = {x * inv, y * inv};
      std::vector<T> theta(setup_->V.count);
      T out = T(0);
      setup_->V.gp(zz, w_.wV.data(), 1, &out, theta.data());
      value += out;
    }
    return value;
  }

  // grad V in the world frame; theta receives the V features' angles.
  std::array<T, 2> grad_potential(T x, T y, T *theta) const {
    std::array<T, 2> g{w_.v_level[0], w_.v_level[1]};
    if (setup_->v_gp) {
      const T inv = static_cast<T>(1.0 / setup_->v_length);
      const T zz[2] = {x * inv, y * inv};
      setup_->V.angles(zz, theta);
      const T a = static_cast<T>(setup_->V.amplitude());
      for (int k = 0; k < setup_->V.count; ++k) {
        using std::sin;
        const T f = -a * sin(theta[k]) * w_.wV[k] * inv;
        g[0] += f * static_cast<T>(setup_->V.frequency[2 * k]);
        g[1] += f * static_cast<T>(setup_->V.frequency[2 * k + 1]);
      }
    }
    return g;
  }
  std::array<T, 2> grad_potential(T x, T y) const {
    std::vector<T> theta(setup_->V.count);
    return grad_potential(x, y, theta.data());
  }

  T hamiltonian(const T *state) const {
    const vec3<T> v{state[3], state[4], state[5]};
    const vec3<T> p = solve3(minv_, v);
    return T(0.5) * (p[0] * v[0] + p[1] * v[1] + p[2] * v[2]) + potential(state[0], state[1]);
  }

  // M = (M^-1)^-1, row-major (symmetric).
  mat3<T> mass() const {
    mat3<T> m{};
    for (int j = 0; j < 3; ++j) {
      vec3<T> e{};
      e[j] = T(1);
      const vec3<T> col = solve3(minv_, e);
      for (int i = 0; i < 3; ++i)
        m[3 * i + j] = col[i];
    }
    return m;
  }

  // y = G(v)^T v at the state's twist (control_dim outputs, into y): the output conjugate to the
  // input, so y . u is the power G u supplies.
  void output(const T *state, T *y) const {
    const vec3<T> v{state[3], state[4], state[5]};
    std::array<T, 3 * kSe2MaxControls> G{};
    control_matrix_at(v, G.data());
    const int nu = w_.control_dim;
    for (int j = 0; j < nu; ++j) {
      T acc = T(0);
      for (int i = 0; i < 3; ++i)
        acc += G[nu * i + j] * v[i];
      y[j] = acc;
    }
  }

  // The power the diffusion injects on average, 1/2 tr(Sigma^T M Sigma): the increment is added
  // to the twist, so dH = v^T M dv + 1/2 dv^T M dv and, Sigma constant (Ito and Stratonovich
  // agree), E[dH] = (-v^T D v + y^T u + noise_power()) dt. Pathwise the drift alone is passive;
  // the noise is booked here, not by D.
  T noise_power() const {
    const mat3<T> m = mass();
    T acc = T(0);
    for (int i = 0; i < 3; ++i)
      acc += m[4 * i] * w_.diffusion[i] * w_.diffusion[i];
    return T(0.5) * acc;
  }

  // Every piece of the drift at (state, u); st's feature buffers are sized here.
  void stage(const T *state, const T *u, se2_stage<T> &st) const {
    using std::cos;
    using std::sin;
    st.theta_D.resize(setup_->D.count);
    st.theta_G.resize(setup_->G.count);
    st.theta_V.resize(setup_->V.count);
    const int nu = w_.control_dim;
    for (int j = 0; j < nu; ++j)
      st.u[j] = u[j];
    st.c = cos(state[2]);
    st.s = sin(state[2]);
    st.v = {state[3], state[4], state[5]};
    st.dxy = {st.c * st.v[0] - st.s * st.v[1], st.s * st.v[0] + st.c * st.v[1]};
    st.p = solve3(minv_, st.v);
    st.grad_v = grad_potential(state[0], state[1], st.theta_V.data());
    st.body = {st.c * st.grad_v[0] + st.s * st.grad_v[1],
               -st.s * st.grad_v[0] + st.c * st.grad_v[1]};
    st.coad = {st.p[1] * st.v[2], -st.p[0] * st.v[2], st.p[0] * st.v[1] - st.p[1] * st.v[0]};
    st.d_levels = dissipation_levels(st.v, st.z, st.theta_D.data());
    st.D = spd_from_levels(st.d_levels.data());
    control_matrix(st.z, st.G.data(), st.theta_G.data());
    for (int i = 0; i < 3; ++i) {
      T gu = T(0);
      for (int j = 0; j < nu; ++j)
        gu += st.G[i * nu + j] * st.u[j];
      st.q[i] = st.coad[i] + gu - (i < 2 ? st.body[i] : T(0));
    }
    st.e = matvec(minv_, st.q);
    st.K = mul(minv_, st.D);
  }

  // The drift (6): (dx, dy, dpsi, dv), the damping included.
  void vector_field(const T *state, const T *u, T *out) const {
    se2_stage<T> st;
    stage(state, u, st);
    const vec3<T> kv = matvec(st.K, st.v);
    out[0] = st.dxy[0];
    out[1] = st.dxy[1];
    out[2] = st.v[2];
    for (int i = 0; i < 3; ++i)
      out[3 + i] = st.e[i] - kv[i];
  }

  // M^-1 D(v), the matrix the step treats implicitly.
  mat3<T> effective_damping(const T *state) const {
    return mul(minv_, dissipation({state[3], state[4], state[5]}));
  }

  // One Lie-IMEX step of size h from state (6) under u; increment (3, may be null) is the twist's
  // noise increment eta. st1 and st2 receive the two stages; vp the predictor's twist.
  void step(const T *state, const T *u, T h, const T *increment, T *out, se2_stage<T> &st1,
            se2_stage<T> &st2, vec3<T> &vp) const {
    stage(state, u, st1);
    const vec3<T> &v = st1.v;
    const vec3<T> eta = increment ? vec3<T>{increment[0], increment[1], increment[2]} : vec3<T>{};
    mat3<T> a1 = identity3<T>();
    for (int k = 0; k < 9; ++k)
      a1[k] += h * st1.K[k];
    vp = solve3(a1, vec3<T>{v[0] + h * st1.e[0] + eta[0], v[1] + h * st1.e[1] + eta[1],
                            v[2] + h * st1.e[2] + eta[2]});
    const T pred[6] = {state[0] + h * st1.dxy[0],
                       state[1] + h * st1.dxy[1],
                       state[2] + h * v[2],
                       vp[0],
                       vp[1],
                       vp[2]};
    stage(pred, u, st2);
    const vec3<T> k1v = matvec(st1.K, v);
    const T hh = T(0.5) * h;
    mat3<T> a2 = identity3<T>();
    for (int k = 0; k < 9; ++k)
      a2[k] += hh * st2.K[k];
    vec3<T> right;
    for (int i = 0; i < 3; ++i)
      right[i] = v[i] + hh * (st1.e[i] + st2.e[i] - k1v[i]) + eta[i];
    const vec3<T> vn = solve3(a2, right);
    out[0] = state[0] + hh * (st1.dxy[0] + st2.dxy[0]);
    out[1] = state[1] + hh * (st1.dxy[1] + st2.dxy[1]);
    out[2] = state[2] + hh * (v[2] + vp[2]);
    out[3] = vn[0];
    out[4] = vn[1];
    out[5] = vn[2];
  }
  void step(const T *state, const T *u, T h, const T *increment, T *out) const {
    se2_stage<T> st1, st2;
    vec3<T> vp;
    step(state, u, h, increment, out, st1, st2, vp);
  }

  // One observation interval of `substeps` steps under the held u. noise (substeps x 3 standard
  // normals) may be null: the drift alone.
  void interval_step(const T *state, const T *u, T interval, int substeps, const T *noise,
                     T *out) const {
    const T h = interval / static_cast<T>(substeps);
    const T root = std::sqrt(h);
    T cur[6];
    for (int i = 0; i < 6; ++i)
      cur[i] = state[i];
    for (int k = 0; k < substeps; ++k) {
      T inc[3];
      if (noise)
        for (int i = 0; i < 3; ++i)
          inc[i] = root * w_.diffusion[i] * noise[3 * k + i];
      step(cur, u, h, noise ? inc : nullptr, out);
      for (int i = 0; i < 6; ++i)
        cur[i] = out[i];
    }
  }

  // path[(t + 1) * 6 + ...] for t < steps from state; controls [steps x control_dim]; path[0..5] is
  // the start.
  void rollout(const T *state, const T *controls, int steps, T interval, int substeps,
               T *path) const {
    for (int i = 0; i < 6; ++i)
      path[i] = state[i];
    for (int t = 0; t < steps; ++t)
      interval_step(path + 6 * t, controls + static_cast<std::size_t>(t) * w_.control_dim, interval,
                    substeps, nullptr, path + 6 * (t + 1));
  }

private:
  const se2_setup *setup_;
  se2_weights<T> w_;
  std::array<T, 3> twist_scale_{};
  mat3<T> minv_{};
};

} // namespace ph
} // namespace cvc

#endif // __CVC_PH_SE2_VEHICLE_H__
