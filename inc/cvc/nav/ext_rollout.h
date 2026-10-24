/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::diff::bike_rollout_ext: one agent's kinematic-bicycle trajectory with an external
// force, and its reverse. The trajectory-level form of the ext_in port in diff_rollout.h, for a
// trainer whose loss reads the whole path and whose force depends on where the agent is
// (for example a loss on goal reach and on exposure to the force along the path).
//
//   diff::bike_ext_fn fn;              // the force, sampled at a pose
//   fn.sample = my_force;              // writes F (drive frame) and J = dF/do (row-major 2x2)
//   fn.user = &my_state;
//   diff::bike_tape tape;
//   diff::bike_rollout_ext(F, v, ox, oy, th, sp, goal, coef, T, fn, tape);  // tape.state: T + 1
//   ...  // dL/dstate at every step, from the loss
//   diff::bike_rollout_ext_bwd(F, v, tape, goal, coef, g_state, g_fext, g_coef, g_init);
//
// - Step t drives `bike_step` with the goal goal[2t..2t+1] and the coefficients coef[3t..3t+2]
//   (alpha, beta, gamma). Both are exogenous: recorded from the deployed carrot FSM and coef_mlp,
//   with no derivative through the pose.
// - The force is sampled at step t's pose every `hold` steps and held in between, as a harness
//   that computes it once per frame of `hold` substeps does. hold = 1 samples it every step,
//   which is the deployed drive's per-substep ext_force (one diff step = one substep).
// - The reverse takes dL/dstate at every step 0..T (ox, oy, th, sp), and returns dL/dF_ext at
//   every step, dL/dcoef at every step (pass null to skip), and dL/d(initial state). With
//   hold > 1, a window's dL/dF_ext is summed and goes through J at the step where F was sampled.
// - A null `sample` is the rollout without a force, bit for bit (the ext_in pointer is null).
// - Host only (std::vector). One agent per call: batch on threads; agents are independent.

#ifndef __CVC_NAV_EXT_ROLLOUT_H__
#define __CVC_NAV_EXT_ROLLOUT_H__

#include <cvc/nav/detail/diff_rollout.h>
#include <cvc/nav/detail/dyn_bike.h>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {
namespace diff {

struct bike_ext_fn {
  // F_ext at the pose (ox, oy) of step t, in the drive frame: f[2], and its Jacobian
  // J = dF/do row-major (dfx/dox, dfx/doy, dfy/dox, dfy/doy), or zeros. Called once per sampled
  // step, in order.
  void (*sample)(void *user, int t, float ox, float oy, float *f, float *J) = nullptr;
  void *user = nullptr;
  int steer = 1; // the force also steers, like ext_force::steer
  int hold = 1;  // sample every `hold` steps and hold F between
};

struct bike_tape {
  int T = 0;
  bool has_ext = false; // a force was sampled (bike_ext_fn::sample set)
  int steer = 1;
  std::vector<float> state;    // [(T+1)*4]: (ox, oy, th, sp) at steps 0..T
  std::vector<float> fext;     // [T*2]: the force step t used
  std::vector<float> jac;      // [T*4]: J where F was sampled (zeros on held steps)
  std::vector<int> sampled_at; // [T]: the step whose pose step t's force was sampled at
};

inline void bike_rollout_ext(const field &f, const bike_veh &v, float ox, float oy, float th,
                             float sp, const float *goal, const float *coef, int T,
                             const bike_ext_fn &ext, bike_tape &tape) {
  if (T < 0 || ext.hold < 1)
    throw std::invalid_argument("cvc::nav::diff::bike_rollout_ext: needs T >= 0 and hold >= 1");
  tape.T = T;
  tape.has_ext = ext.sample != nullptr;
  tape.steer = ext.steer;
  tape.state.assign(static_cast<std::size_t>(4 * (T + 1)), 0.0f);
  tape.fext.assign(static_cast<std::size_t>(2 * T), 0.0f);
  tape.jac.assign(static_cast<std::size_t>(4 * T), 0.0f);
  tape.sampled_at.assign(static_cast<std::size_t>(T), 0);
  float *s = tape.state.data();
  s[0] = ox;
  s[1] = oy;
  s[2] = th;
  s[3] = sp;
  ext_in e;
  e.steer = ext.steer;
  int at = 0;
  for (int t = 0; t < T; ++t) {
    const float *st = s + 4 * t;
    float *s1 = s + 4 * (t + 1);
    if (ext.sample && t % ext.hold == 0) {
      float F[2] = {0.0f, 0.0f}, J[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      ext.sample(ext.user, t, st[0], st[1], F, J);
      e.fx = F[0];
      e.fy = F[1];
      for (int k = 0; k < 4; ++k)
        tape.jac[4 * t + k] = J[k];
      at = t;
    }
    tape.fext[2 * t] = e.fx;
    tape.fext[2 * t + 1] = e.fy;
    tape.sampled_at[t] = at;
    bike_step(f, st[0], st[1], st[2], st[3], goal[2 * t], goal[2 * t + 1], coef[3 * t],
              coef[3 * t + 1], coef[3 * t + 2], v, s1[0], s1[1], s1[2], s1[3],
              ext.sample ? &e : nullptr);
  }
}

// The same with one goal and one set of coefficients for every step.
inline void bike_rollout_ext(const field &f, const bike_veh &v, float ox, float oy, float th,
                             float sp, float gx, float gy, float al, float be, float ga, int T,
                             const bike_ext_fn &ext, bike_tape &tape) {
  std::vector<float> goal(static_cast<std::size_t>(2 * (T > 0 ? T : 0))),
      coef(static_cast<std::size_t>(3 * (T > 0 ? T : 0)));
  for (int t = 0; t < T; ++t) {
    goal[2 * t] = gx;
    goal[2 * t + 1] = gy;
    coef[3 * t] = al;
    coef[3 * t + 1] = be;
    coef[3 * t + 2] = ga;
  }
  bike_rollout_ext(f, v, ox, oy, th, sp, goal.data(), coef.data(), T, ext, tape);
}

// g_state[(T+1)*4] in; g_fext[T*2], g_coef[T*3] and g_init[4] out, each optional (null). `goal`
// and `coef` are the forward's.
inline void bike_rollout_ext_bwd(const field &f, const bike_veh &v, const bike_tape &tape,
                                 const float *goal, const float *coef, const float *g_state,
                                 float *g_fext, float *g_coef, float *g_init) {
  const int T = tape.T;
  const bool has_ext = tape.has_ext;
  const float *s = tape.state.data();
  // The adjoint of the state after step t: dL/dstate_{t+1}, all paths.
  float a[4] = {g_state[4 * T], g_state[4 * T + 1], g_state[4 * T + 2], g_state[4 * T + 3]};
  float win[2] = {0.0f, 0.0f}; // dL/dF summed over the current hold window
  for (int t = T - 1; t >= 0; --t) {
    const float *st = s + 4 * t;
    ext_in e;
    e.fx = tape.fext[2 * t];
    e.fy = tape.fext[2 * t + 1];
    e.steer = tape.steer;
    float gox, goy, gth, gsp, gal, gbe, gga, gf[2] = {0.0f, 0.0f};
    bike_step_bwd(f, st[0], st[1], st[2], st[3], goal[2 * t], goal[2 * t + 1], coef[3 * t],
                  coef[3 * t + 1], coef[3 * t + 2], v, a[0], a[1], a[2], a[3], gox, goy, gth, gsp,
                  gal, gbe, gga, has_ext ? &e : nullptr, has_ext ? gf : nullptr);
    if (g_fext) {
      g_fext[2 * t] = gf[0];
      g_fext[2 * t + 1] = gf[1];
    }
    if (g_coef) {
      g_coef[3 * t] = gal;
      g_coef[3 * t + 1] = gbe;
      g_coef[3 * t + 2] = gga;
    }
    if (has_ext) {
      win[0] += gf[0];
      win[1] += gf[1];
      if (tape.sampled_at[t] == t) { // F was sampled here: the window's gradient through J
        const float *J = tape.jac.data() + 4 * t;
        gox += win[0] * J[0] + win[1] * J[2];
        goy += win[0] * J[1] + win[1] * J[3];
        win[0] = win[1] = 0.0f;
      }
    }
    a[0] = g_state[4 * t] + gox;
    a[1] = g_state[4 * t + 1] + goy;
    a[2] = g_state[4 * t + 2] + gth;
    a[3] = g_state[4 * t + 3] + gsp;
  }
  if (g_init)
    for (int k = 0; k < 4; ++k)
      g_init[k] = a[k];
}

// The dynamic vehicle's twin (dyn_bike_step: the same controller into dyn_plant): the state is
// dyn_state (x, y, psi, vx, vy, r, delta, a), and the force works the same way.
struct dyn_tape {
  int T = 0;
  bool has_ext = false;
  int steer = 1;
  std::vector<dyn_state<float>> state; // [T+1]
  std::vector<float> fext;             // [T*2]
  std::vector<float> jac;              // [T*4]
  std::vector<int> sampled_at;         // [T]
};

inline void dyn_rollout_ext(const field &f, const bike_veh &v, const dyn_bike_params &p,
                            const dyn_state<float> &s0, const float *goal, const float *coef, int T,
                            const bike_ext_fn &ext, dyn_tape &tape, float grade = 0.0f) {
  if (T < 0 || ext.hold < 1)
    throw std::invalid_argument("cvc::nav::diff::dyn_rollout_ext: needs T >= 0 and hold >= 1");
  tape.T = T;
  tape.has_ext = ext.sample != nullptr;
  tape.steer = ext.steer;
  tape.state.assign(static_cast<std::size_t>(T + 1), s0);
  tape.fext.assign(static_cast<std::size_t>(2 * T), 0.0f);
  tape.jac.assign(static_cast<std::size_t>(4 * T), 0.0f);
  tape.sampled_at.assign(static_cast<std::size_t>(T), 0);
  ext_in e;
  e.steer = ext.steer;
  int at = 0;
  for (int t = 0; t < T; ++t) {
    const dyn_state<float> &st = tape.state[t];
    if (ext.sample && t % ext.hold == 0) {
      float F[2] = {0.0f, 0.0f}, J[4] = {0.0f, 0.0f, 0.0f, 0.0f};
      ext.sample(ext.user, t, st.x, st.y, F, J);
      e.fx = F[0];
      e.fy = F[1];
      for (int k = 0; k < 4; ++k)
        tape.jac[4 * t + k] = J[k];
      at = t;
    }
    tape.fext[2 * t] = e.fx;
    tape.fext[2 * t + 1] = e.fy;
    tape.sampled_at[t] = at;
    dyn_bike_step(f, st, goal[2 * t], goal[2 * t + 1], coef[3 * t], coef[3 * t + 1],
                  coef[3 * t + 2], v, p, grade, tape.state[t + 1], ext.sample ? &e : nullptr);
  }
}

// g_state[T+1] in; g_fext[T*2], g_coef[T*3] and g_init out, each optional (null).
inline void dyn_rollout_ext_bwd(const field &f, const bike_veh &v, const dyn_bike_params &p,
                                const dyn_tape &tape, const float *goal, const float *coef,
                                const dyn_state<float> *g_state, float *g_fext, float *g_coef,
                                dyn_state<float> *g_init, float grade = 0.0f) {
  const int T = tape.T;
  dyn_state<float> a = g_state[T];
  float win[2] = {0.0f, 0.0f};
  for (int t = T - 1; t >= 0; --t) {
    ext_in e;
    e.fx = tape.fext[2 * t];
    e.fy = tape.fext[2 * t + 1];
    e.steer = tape.steer;
    dyn_state<float> gs{};
    float gal = 0.0f, gbe = 0.0f, gga = 0.0f, gf[2] = {0.0f, 0.0f};
    dyn_bike_step_bwd(f, tape.state[t], goal[2 * t], goal[2 * t + 1], coef[3 * t], coef[3 * t + 1],
                      coef[3 * t + 2], v, p, grade, a, gs, gal, gbe, gga,
                      tape.has_ext ? &e : nullptr, tape.has_ext ? gf : nullptr);
    if (g_fext) {
      g_fext[2 * t] = gf[0];
      g_fext[2 * t + 1] = gf[1];
    }
    if (g_coef) {
      g_coef[3 * t] = gal;
      g_coef[3 * t + 1] = gbe;
      g_coef[3 * t + 2] = gga;
    }
    if (tape.has_ext) {
      win[0] += gf[0];
      win[1] += gf[1];
      if (tape.sampled_at[t] == t) {
        const float *J = tape.jac.data() + 4 * t;
        gs.x += win[0] * J[0] + win[1] * J[2];
        gs.y += win[0] * J[1] + win[1] * J[3];
        win[0] = win[1] = 0.0f;
      }
    }
    for (int k = 0; k < dyn_state<float>::size; ++k)
      a[k] = g_state[t][k] + gs[k];
  }
  if (g_init)
    *g_init = a;
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_EXT_ROLLOUT_H__
