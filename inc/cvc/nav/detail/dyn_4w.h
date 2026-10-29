/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// dyn_4w.h: the differentiable two-track (four-wheel) vehicle with suspension, the
// vehicle-physics plan's P8d. It extends P8a's single-track plant (dyn_bike.h) to the four
// contact patches a real car has, so a policy trains through
// per-wheel grip, terrain under each wheel and the load each corner carries. bike_control
// commands it exactly as it commands dyn_plant.
//
// State, in nav's normalized frame (lengths x S, time in s, mass in kg):
//   (x, y)  the rear axle's midpoint, as bike_step's o;  psi  the heading;
//   vx, vy  the body-frame velocity at the centre of gravity;  r  the yaw rate;
//   z, zd   the body's heave from its rest height on flat ground, and its rate;
//   phi, phid  its roll (+ lifts the left side), and rate;  th, thd  its pitch (+ lowers the
//           nose), and rate;
//   delta   the virtual centre wheel's steering angle;  a  the drive and brake force per unit
//           mass, both behind their actuators (as P8a).
//
// The wheels sit at body-frame (x_i, y_i) from the centre of gravity: front left (lf, tf), front
// right (lf, -tf), rear left (-lr, tr), rear right (-lr, -tr). Corner i's height above its rest
// is c_i . (z, phi, th) with c_i = (1, y_i, -x_i).
//
// One step of hdt, given (delta_cmd, a_cmd) and a ground sampler G (G.at(wx, wy, mu, height) at a
// world point; the plant is templated on it, so its height's gradient flows through the duals):
//   1. Actuators, as P8a.
//   2. Loads. Each corner's spring-damper: F_i = Fz0_i + k_i (height_i - c_i . q) - c_i' (c_i .
//   qd),
//      Fz0_i the static share of m g, floored smoothly at zero (softplus of width fz_min m g): a
//      tire pushes, never pulls. From the current vertical state, so load transfer lags the tire
//      forces by one step.
//   3. Longitudinal. The drive or brake force splits between the axles (front_drive /
//      front_brake), then evenly left and right; each wheel saturates smoothly at mu_i Fz_i and
//      leaves the friction ellipse's remainder for cornering (P8a's dyn_sat, dyn_lateral_cap).
//      The speed is predicted from these and the resistances, then corrected after step 4.
//      With grade_force (off by default) gravity along the ground acts on the body as well: each
//      spring's horizontal reaction on the slope under its wheel, -F_i grad h there, F_i its
//      linear spring force (step 5's) and grad h the ground's, by central differences over
//      1e-3 L (dyn4_grade). It acts on (vx, vy, r) only, from the step's start. Without it the
//      ground only lifts the springs: the vehicle does not roll down a hill.
//   4. Lateral and yaw. Each front wheel's angle is the Ackermann angle for the virtual one:
//      tan delta_i = L tan delta / (L - y_i tan delta). A wheel's lateral velocity in its own
//      frame is w_i = -(vx - r y_i) sin delta_i + (vy + r x_i) cos delta_i, its slip
//      alpha_i = sgn(u_i) ... as P8a, and its force Pacejka's. (vy, r) advance by implicit Euler,
//      each tire linearized by its secant through its root in w_i (dyn_secant_w).
//   5. Vertical. (z, phi, th) advance by implicit Euler on the linear springs and dampers, driven
//      by the ground under each wheel and the tire forces' moments about the centre of gravity
//      (h above the ground): roll + h FY, pitch - h FX. A 3x3 solve.
//   6. Low speed, as P8a: (vy, r) blend toward the kinematic bicycle's.
//   7. Pose, as P8a: the rear axle moves with (vx1, vy1 - lr r1) at psi1.
// With zero tracks, rigid corners and flat ground it reduces to P8a's vehicle, with the
// longitudinal load transfer from the pitch springs rather than quasi-statically (the gates
// check the two agree).
//
// Left out: wheel spin, the unsprung masses, camber and the anti-roll bars.
//
// The plant in port-Hamiltonian form, and each step's energy books: dyn4_ph_* and
// dyn4_plant_books, below.

#ifndef __CVC_NAV_DETAIL_DYN_4W_H__
#define __CVC_NAV_DETAIL_DYN_4W_H__

#include <cvc/nav/detail/dyn_bike.h>

namespace cvc {
namespace nav {
namespace diff {

struct dyn4_params {
  float m = 1.0f;   // kg
  float Iz = 1.0f;  // kg x length^2: yaw inertia about the centre of gravity
  float Ix = 1.0f;  // roll inertia
  float Iy = 1.0f;  // pitch inertia
  float lf = 0.5f;  // length: centre of gravity to the front axle
  float lr = 0.5f;  // length: to the rear axle
  float tf = 0.0f;  // length: half the front track
  float tr = 0.0f;  // length: half the rear track
  float h = 0.0f;   // length: the centre of gravity's height above the ground at rest
  float g = 9.81f;  // length / s^2
  float k_f = 0.0f; // kg / s^2: a front corner's spring rate; 0 = rigid (very stiff)
  float k_r = 0.0f; // a rear corner's
  float c_f = 0.0f; // kg / s: a front corner's damping; 0 = critical for its spring
  float c_r = 0.0f; // a rear corner's
  float B_f = 12.0f, B_r = 12.0f, C = 1.2f; // Pacejka, as dyn_bike_params
  float front_drive = 0.0f, front_brake = 0.6f;
  float drag = 0.0f, c_rr = 0.0f;
  float tau_steer = 0.0f, steer_rate = 0.0f, tau_drive = 0.0f;
  float v_blend = 0.0f, v_width = 1.0f, eps = 0.1f;
  float fz_min = 0.02f; // the load floor's width, as a fraction of m g
  int iters = 3;
  int grade_force = 0; // 1: gravity along the ground's slope acts on the body (opt-in; see step 3)
};

template <class S> struct dyn4_state {
  S x, y, psi, vx, vy, r, z, zd, phi, phid, th, thd, delta, a;
  static const int size = 14;
  CVC_HD S &operator[](int k) {
    S *p[14] = {&x, &y, &psi, &vx, &vy, &r, &z, &zd, &phi, &phid, &th, &thd, &delta, &a};
    return *p[k];
  }
  CVC_HD const S &operator[](int k) const { return (*const_cast<dyn4_state *>(this))[k]; }
};

// Flat ground of one friction.
struct flat_ground {
  float mu = 0.9f;
  template <class S> CVC_HD void at(S, S, S &mu_out, S &height) const {
    mu_out = S(mu);
    height = S(0);
  }
};

// The corners' rates: a spring rate of 0 means rigid, which here is a stiff spring whose
// undamped period is a few steps; damping of 0 means critical damping.
CVC_HD inline float dyn4_rigid_k(float m_corner, float hdt) {
  const float w = 1.0f / hdt; // natural frequency 1 / hdt: stiff, still resolved
  return m_corner * w * w;
}

CVC_HD inline float s_log(float x) { return logf(x); }
CVC_HD inline double s_log(double x) { return log(x); }
template <class T> CVC_HD inline dual<T> s_log(dual<T> a) { return dual<T>(s_log(a.v), a.d / a.v); }

// A smooth floor at zero, softplus at width e: e log(1 + exp(F / e)). Above 30 e it is F (the
// rest is below 1e-13 e), so a tire under a normal load carries exactly its spring's force.
template <class S> CVC_HD inline S dyn4_soft_pos(S F, S e) {
  const S q = F / e;
  if (s_val(q) > 30.0)
    return F;
  return e * s_log(S(1) + s_exp(q));
}

// The four wheels at state s: their body-frame offsets from the centre of gravity, static loads,
// corner rates, the ground under them (friction and height) and their normal loads (step 2 of
// the header comment). The plant uses it, and so can a caller that wants the loads.
template <class S> struct dyn4_wheels {
  S xi[4], yi[4], Fz0[4], k[4], c[4], mu[4], hg[4], Fz[4];
};
template <class S, class G>
CVC_HD inline void dyn4_wheel_state(const dyn4_params &p, const bike_veh &v, const G &ground,
                                    const dyn4_state<S> &s, dyn4_wheels<S> &w) {
  const S m = S(p.m), lf = S(p.lf), lr = S(p.lr), tf = S(p.tf), tr = S(p.tr), g = S(p.g);
  const S L = lf + lr;
  const S xs[4] = {lf, lf, -lr, -lr}, ys[4] = {tf, -tf, tr, -tr};
  for (int i = 0; i < 4; ++i) {
    const bool front = i < 2;
    w.xi[i] = xs[i];
    w.yi[i] = ys[i];
    w.Fz0[i] = S(0.5) * m * g * (front ? lr : lf) / L;
    const float kc = front ? p.k_f : p.k_r, cc = front ? p.c_f : p.c_r;
    const float m_corner = 0.25f * p.m;
    const float kk = kc > 0.0f ? kc : dyn4_rigid_k(m_corner, v.hdt);
    w.k[i] = S(kk);
    w.c[i] = S(cc > 0.0f ? cc : 2.0f * sqrtf(kk * m_corner));
  }
  // The ground under each wheel, at the current pose.
  const S cps = s_cos(s.psi), sps = s_sin(s.psi);
  const S cgx = s.x + lr * cps, cgy = s.y + lr * sps; // the centre of gravity
  for (int i = 0; i < 4; ++i)
    ground.at(cgx + w.xi[i] * cps - w.yi[i] * sps, cgy + w.xi[i] * sps + w.yi[i] * cps, w.mu[i],
              w.hg[i]);
  // Loads, from the current vertical state.
  const S e = S(p.fz_min) * m * g;
  for (int i = 0; i < 4; ++i) {
    const S defl = w.hg[i] - (s.z + w.yi[i] * s.phi - w.xi[i] * s.th);
    const S rate = s.zd + w.yi[i] * s.phid - w.xi[i] * s.thd;
    w.Fz[i] = dyn4_soft_pos(w.Fz0[i] + w.k[i] * defl - w.c[i] * rate, e);
  }
}

// A front wheel's Ackermann angle, at body-frame y from the centreline, for the virtual centre
// wheel's tan(delta): tan delta_i = L tan delta / (L - y tan delta). The rear wheels do not steer.
template <class S> CVC_HD inline S dyn4_ackermann(S L, S y, S tan_delta) {
  return s_atan(L * tan_delta / (L - y * tan_delta));
}

// Gravity along the ground, for grade_force: each spring's horizontal reaction on the slope under
// its wheel, -F_i grad h(P_i), as a force and yaw moment on the body, f = (F_x, F_y, M_z) on
// (vx, vy, r). F_i = Fz0_i + k_i (hg_i - c_i . q) is the spring's linear force, so f is minus the
// gradient of the springs' energy in the pose (dyn4_ph_hamiltonian): conservative. The slope is
// the ground's, by central differences over 1e-3 L, so any sampler serves. w is
// dyn4_wheel_state's at s.
template <class S, class G>
CVC_HD inline void dyn4_grade(const dyn4_params &p, const G &ground, const dyn4_state<S> &s,
                              const dyn4_wheels<S> &w, S f[3]) {
  const S lr = S(p.lr), e = S(1e-3) * (S(p.lf) + lr);
  const S cps = s_cos(s.psi), sps = s_sin(s.psi);
  const S cgx = s.x + lr * cps, cgy = s.y + lr * sps;
  f[0] = f[1] = f[2] = S(0);
  for (int i = 0; i < 4; ++i) {
    const S wx = cgx + w.xi[i] * cps - w.yi[i] * sps, wy = cgy + w.xi[i] * sps + w.yi[i] * cps;
    S mu, h_xp, h_xm, h_yp, h_ym;
    ground.at(wx + e, wy, mu, h_xp);
    ground.at(wx - e, wy, mu, h_xm);
    ground.at(wx, wy + e, mu, h_yp);
    ground.at(wx, wy - e, mu, h_ym);
    const S gx = (h_xp - h_xm) / (S(2) * e), gy = (h_yp - h_ym) / (S(2) * e);
    const S Fs = w.Fz0[i] + w.k[i] * (w.hg[i] - (s.z + w.yi[i] * s.phi - w.xi[i] * s.th));
    const S fx = -Fs * (cps * gx + sps * gy), fy = -Fs * (cps * gy - sps * gx);
    f[0] = f[0] + fx;
    f[1] = f[1] + fy;
    f[2] = f[2] + w.xi[i] * fy - w.yi[i] * fx;
  }
}

// One step of the plant; see the header comment. `seen`, when given, gets the twist the step
// reached before its limiters (dyn_step_free, for dyn4_plant_books).
template <class S, class G>
CVC_HD inline void dyn4_plant(const dyn4_params &p, const bike_veh &v, const G &ground,
                              const dyn4_state<S> &s, S delta_cmd, S a_cmd, dyn4_state<S> &o,
                              dyn_step_free<S> *seen = nullptr) {
  const S hdt = S(v.hdt), one = S(1), zero = S(0);
  const S m = S(p.m), Iz = S(p.Iz), Ix = S(p.Ix), Iy = S(p.Iy), lf = S(p.lf), lr = S(p.lr);
  const S h = S(p.h), g = S(p.g), Cp = S(p.C), L = lf + lr;

  // 1. Actuators.
  const S k_steer = p.tau_steer > 0.0f ? one - s_exp(-hdt / S(p.tau_steer)) : one;
  S step = (delta_cmd - s.delta) * k_steer;
  if (p.steer_rate > 0.0f) {
    const S R = S(p.steer_rate) * hdt;
    step = R * s_tanh(step / R);
  }
  const S delta1 = s.delta + step;
  const S k_drive = p.tau_drive > 0.0f ? one - s_exp(-hdt / S(p.tau_drive)) : one;
  const S a1 = s.a + (a_cmd - s.a) * k_drive;

  // 2. The wheels, the ground under them and their loads.
  dyn4_wheels<S> W;
  dyn4_wheel_state(p, v, ground, s, W);
  const S *xi = W.xi, *yi = W.yi, *Fz0 = W.Fz0, *k = W.k, *c = W.c, *mu_i = W.mu, *hg = W.hg;
  const S *Fz = W.Fz;
  // Gravity along the ground, when asked.
  S Fg[3] = {zero, zero, zero};
  if (p.grade_force)
    dyn4_grade(p, ground, s, W, Fg);

  // 3. Longitudinal: each wheel's drive or brake force within its grip, and what that leaves.
  const S F_drive = m * a1;
  const S share = S(s_val(a1) >= s_val(zero) ? p.front_drive : p.front_brake);
  S Fx[4], Dy[4], dw[4], cdw[4], sdw[4];
  // The front wheels at their Ackermann angles.
  const S td = s_tan(delta1);
  for (int i = 0; i < 4; ++i) {
    const S axle_share = i < 2 ? share : one - share;
    const S cap = mu_i[i] * Fz[i];
    Fx[i] = dyn_sat(S(0.5) * F_drive * axle_share, cap);
    Dy[i] = dyn_lateral_cap(Fx[i], cap);
    dw[i] = i < 2 ? dyn4_ackermann(L, yi[i], td) : zero;
    cdw[i] = s_cos(dw[i]);
    sdw[i] = s_sin(dw[i]);
  }
  const S eps2 = S(p.eps) * S(p.eps);
  const S sp_min = S(v.allow_reverse ? -0.25f * v.vmax : 0.0f), vmax = S(v.vmax);
  const S u0 = s_sqrt(s.vx * s.vx + eps2), sg0 = s.vx / u0;
  const S F_res = -S(p.drag) * s.vx * u0 - S(p.c_rr) * m * g * sg0;
  S FXlong = zero;
  for (int i = 0; i < 4; ++i)
    FXlong = FXlong + Fx[i] * cdw[i];
  S FXpred = FXlong + F_res;
  if (p.grade_force)
    FXpred = FXpred + Fg[0];
  const S vx_p = dyn_clamp(s.vx + hdt * (FXpred / m), sp_min, vmax);

  // 4. Lateral and yaw: implicit Euler for (vy, r), four tires.
  const S u = s_sqrt(vx_p * vx_p + eps2), sg = vx_p / u;
  const S Bw[4] = {S(p.B_f), S(p.B_f), S(p.B_r), S(p.B_r)};
  S vy1 = s.vy, r1 = s.r;
  S Fy_applied[4] = {zero, zero, zero, zero};
  const int iters = p.iters > 0 ? p.iters : 1;
  for (int it = 0; it < iters; ++it) {
    S FY = zero, MZ = zero, J11 = zero, J12 = zero, J21 = zero, J22 = zero;
    S Fy[4], St[4], dwdvy[4], dwdr[4];
    for (int i = 0; i < 4; ++i) {
      // The wheel's velocity in its own frame, the forward speed from the prediction.
      const S vxi = vx_p - r1 * yi[i], vyi = vy1 + r1 * xi[i];
      const S wi = -vxi * sdw[i] + vyi * cdw[i];
      const S ui = s_sqrt((vxi * cdw[i] + vyi * sdw[i]) * (vxi * cdw[i] + vyi * sdw[i]) + eps2);
      const S alpha = -s_atan2(wi, ui);
      Fy[i] = dyn_lateral(alpha, Bw[i], Cp, Dy[i]);
      // The root: zero lateral velocity in the wheel's frame.
      St[i] = dyn_secant_w(Fy[i], alpha, wi, zero, Bw[i], Cp, Dy[i], ui);
      dwdvy[i] = cdw[i];
      dwdr[i] = yi[i] * sdw[i] + xi[i] * cdw[i];
      // On the body: the lateral force's components, and the longitudinal one's.
      const S fby = Fy[i] * cdw[i] + Fx[i] * sdw[i];
      const S fbx = -Fy[i] * sdw[i] + Fx[i] * cdw[i];
      FY = FY + fby;
      MZ = MZ + xi[i] * fby - yi[i] * fbx;
      // d(body force, moment)/d(vy, r) through the secant: dFy = -St dw.
      const S dfy_y = -St[i] * cdw[i], dfx_y = St[i] * sdw[i]; // per unit dw
      J11 = J11 + dfy_y * dwdvy[i];
      J12 = J12 + dfy_y * dwdr[i];
      J21 = J21 + (xi[i] * dfy_y - yi[i] * dfx_y) * dwdvy[i];
      J22 = J22 + (xi[i] * dfy_y - yi[i] * dfx_y) * dwdr[i];
    }
    if (p.grade_force) {
      FY = FY + Fg[1];
      MZ = MZ + Fg[2];
    }
    const S f_vy = FY / m - r1 * vx_p, f_r = MZ / Iz;
    const S A11 = one - hdt * (J11 / m), A12 = -hdt * (J12 / m - vx_p);
    const S A21 = -hdt * (J21 / Iz), A22 = one - hdt * (J22 / Iz);
    S det = A11 * A22 - A12 * A21;
    if (s_val(det) < s_val(S(0.05)))
      det = S(0.05);
    const S b1 = s.vy + hdt * f_vy - vy1, b2 = s.r + hdt * f_r - r1;
    const S dvy = (A22 * b1 - A12 * b2) / det, dr = (A11 * b2 - A21 * b1) / det;
    vy1 = vy1 + dvy;
    r1 = r1 + dr;
    for (int i = 0; i < 4; ++i)
      Fy_applied[i] = Fy[i] - St[i] * (dwdvy[i] * dvy + dwdr[i] * dr);
  }

  // 3, corrected: the speed with the lateral forces the implicit step applied.
  S FXb = F_res, FYb = zero;
  for (int i = 0; i < 4; ++i) {
    FXb = FXb + Fx[i] * cdw[i] - Fy_applied[i] * sdw[i];
    FYb = FYb + Fy_applied[i] * cdw[i] + Fx[i] * sdw[i];
  }
  S FXv = FXb; // with gravity along the ground, which does not pitch the body (step 5)
  if (p.grade_force)
    FXv = FXb + Fg[0];
  const S vx1_free = s.vx + hdt * (FXv / m + r1 * vy1);
  const S vx1 = dyn_clamp(vx1_free, sp_min, vmax);
  if (seen) {
    seen->vx = vx1_free;
    seen->vy = vy1;
    seen->r = r1;
  }

  // 5. Vertical: implicit Euler on (z, phi, th) with the corners' linear springs and dampers.
  // M qdd = sum_i c_i (Fz0_i + k_i (hg_i - c_i . q) - c_i' (c_i . qd)) - (m g, 0, 0)
  //         + (0, h FY, -h FX);  (M + hdt D + hdt^2 K) qd1 = M qd0 + hdt (G - K q0).
  S Kq[3][3] = {{zero, zero, zero}, {zero, zero, zero}, {zero, zero, zero}};
  S Dq[3][3] = {{zero, zero, zero}, {zero, zero, zero}, {zero, zero, zero}};
  S Gv[3] = {-m * g, h * FYb, -h * (FXb - F_res)};
  for (int i = 0; i < 4; ++i) {
    const S ci[3] = {one, yi[i], -xi[i]};
    for (int a = 0; a < 3; ++a) {
      Gv[a] = Gv[a] + ci[a] * (Fz0[i] + k[i] * hg[i]);
      for (int b = 0; b < 3; ++b) {
        Kq[a][b] = Kq[a][b] + k[i] * ci[a] * ci[b];
        Dq[a][b] = Dq[a][b] + c[i] * ci[a] * ci[b];
      }
    }
  }
  const S Mv[3] = {m, Ix, Iy};
  const S q0[3] = {s.z, s.phi, s.th}, qd0[3] = {s.zd, s.phid, s.thd};
  S A[3][3], rhs[3];
  for (int a = 0; a < 3; ++a) {
    rhs[a] = Mv[a] * qd0[a] + hdt * Gv[a];
    for (int b = 0; b < 3; ++b) {
      A[a][b] = hdt * Dq[a][b] + hdt * hdt * Kq[a][b] + (a == b ? Mv[a] : zero);
      rhs[a] = rhs[a] - hdt * Kq[a][b] * q0[b];
    }
  }
  // Cramer's rule on the symmetric positive definite 3x3.
  const S det3 = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                 A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                 A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
  S qd1[3];
  for (int col = 0; col < 3; ++col) {
    S Ac[3][3];
    for (int a = 0; a < 3; ++a)
      for (int b = 0; b < 3; ++b)
        Ac[a][b] = b == col ? rhs[a] : A[a][b];
    qd1[col] = (Ac[0][0] * (Ac[1][1] * Ac[2][2] - Ac[1][2] * Ac[2][1]) -
                Ac[0][1] * (Ac[1][0] * Ac[2][2] - Ac[1][2] * Ac[2][0]) +
                Ac[0][2] * (Ac[1][0] * Ac[2][1] - Ac[1][1] * Ac[2][0])) /
               det3;
  }

  // 6. Low speed: toward the kinematic bicycle.
  if (p.v_blend > 0.0f) {
    const S w = one / (one + s_exp((s_sqrt(vx1 * vx1 + eps2) - S(p.v_blend)) / S(p.v_width)));
    const S r_k = vx1 * s_tan(delta1) / L, vy_k = lr * r_k;
    vy1 = vy1 + w * (vy_k - vy1);
    r1 = r1 + w * (r_k - r1);
  }

  // 7. Pose.
  const S psi1 = s.psi + hdt * r1;
  const S v_side = vy1 - lr * r1;
  const S c1 = s_cos(psi1), s1 = s_sin(psi1);
  o.x = s.x + hdt * (vx1 * c1 - v_side * s1);
  o.y = s.y + hdt * (vx1 * s1 + v_side * c1);
  o.psi = psi1;
  o.vx = vx1;
  o.vy = vy1;
  o.r = r1;
  o.zd = qd1[0];
  o.phid = qd1[1];
  o.thd = qd1[2];
  o.z = s.z + hdt * qd1[0];
  o.phi = s.phi + hdt * qd1[1];
  o.th = s.th + hdt * qd1[2];
  o.delta = delta1;
  o.a = a1;
}

// The plant's vector-Jacobian product on forward-mode duals, as dyn_plant_vjp: gradients on its
// outputs -> on its state and commands. The ground's height gradient flows when G samples in the
// dual type.
template <class T, class G>
CVC_HD inline void dyn4_plant_vjp(const dyn4_params &p, const bike_veh &v, const G &ground,
                                  const dyn4_state<T> &s, T delta_cmd, T a_cmd,
                                  const dyn4_state<T> &g_o, dyn4_state<T> &g_s, T &g_delta_cmd,
                                  T &g_a_cmd) {
  typedef dual<T> D;
  const int n = dyn4_state<T>::size;
  for (int j = 0; j < n + 2; ++j) {
    dyn4_state<D> ds, o;
    for (int kk = 0; kk < n; ++kk)
      ds[kk] = D(s[kk], kk == j ? T(1) : T(0));
    dyn4_plant<D>(p, v, ground, ds, D(delta_cmd, j == n ? T(1) : T(0)),
                  D(a_cmd, j == n + 1 ? T(1) : T(0)), o);
    T acc = T(0);
    for (int kk = 0; kk < n; ++kk)
      acc += g_o[kk] * o[kk].d;
    if (j < n)
      g_s[j] = acc;
    else if (j == n)
      g_delta_cmd = acc;
    else
      g_a_cmd = acc;
  }
}

// ── The plant in port-Hamiltonian form ────────────────────────────────────────────────────
// dyn_bike.h's form (dyn_ph_*) on four wheels with suspension, at every slip. The body's state is
// its vertical displacement q = (z, phi, th) and the momenta p = M xi of its twist
// xi = (vx, vy, r, zd, phid, thd), M = diag(m, m, Iz, m, Ix, Iy); the pose (x, y, psi) enters only
// through the ground under the wheels. In x = (q, p), nine coordinates, dyn4_plant's vector field
// (its step as hdt -> 0, the actuators holding) is exactly
//
//   H = 1/2 xi^T M xi + sum_i 1/2 k_i l_i^2 + m g z,   l_i = Fz0_i / k_i + hg_i - c_i . q
//   x' = (J(p) - R) dH/dx + G u + f_lever,   dH/dx = (dH/dq, xi),   u = (F_x1, ..., F_x4),
//   y = G^T dH/dx
//
//   H     the kinetic energy, the corners' springs (l_i a spring's compression; the vertical step's
//         springs are linear, the softplus floor is only on the tires' loads; a rigid corner's k_i
//         is dyn4_rigid_k's, set by hdt) and gravity on heave. sum_i c_i Fz0_i = (m g, 0, 0): the
//         preloads hold the body at q = 0 on flat ground.
//   J     [[0, 0, I], [0, J_h(p_h), 0], [-I, 0, 0]] over (q, p_h, p_v): canonical in the vertical,
//         SE(2)'s coadjoint action in the horizontal (dyn_ph_J). Skew, so it moves no energy.
//   R     blockdiag(0, R_h, R_v). R_h = c_x e_x e_x^T + sum_i S_i n_i n_i^T: c_x = drag u +
//         c_rr m g / u, drag and rolling resistance on vx (u = sqrt(vx^2 + eps^2)); n_i =
//         (-sin d_i, cos d_i, x_i cos d_i + y_i sin d_i) carries wheel i's lateral force onto
//         (vx, vy, r), so n_i^T v is w_i, the lateral velocity in the wheel's own frame that its
//         slip alpha_i = -atan2(w_i, u_i) is measured in; and S_i = -F_yi / w_i (dyn4_ph_secant)
//         is the tire's secant through its root w_i = 0, >= 0 at any slip because a Pacejka force
//         (C < 2) always opposes its slip. R_v = sum_i c'_i c_i c_i^T, the dampers. So R is
//         positive semi-definite sliding or not, at any steering angle.
//   G     columns T_i = (cos d_i, sin d_i, x_i sin d_i - y_i cos d_i, 0, h sin d_i, -h cos d_i) on
//         the twist: wheel i's drive or brake force (within its grip) along the wheel, at its
//         contact patch h below the centre of gravity. y_i is that patch's speed along the wheel,
//         roll and pitch included, and y^T u the power the forces deliver. Steering is not an
//         input: the wheels' angles d_i only turn n_i and T_i, so they do no work.
//   f_lever  (0, h sum_i F_yi cos d_i, h sum_i F_yi sin d_i) on (p_z, p_phi, p_th): the tires'
//         lateral forces' moments on roll and pitch. The slips do not see the body's roll and
//         pitch rates (a patch's lateral velocity has no h phid in it), so this coupling is
//         one-way: neither skew nor dissipative. It is booked on its own line and vanishes with h.
//   f_eps = 0. Each slip is measured in its wheel's frame and rooted at w_i = 0, so the secant
//         carries the whole lateral force and nothing is left at the speed floor (dyn_bike's front
//         slip, in the body's frame and rooted at the floored speed, leaves dyn_ph_floor).
//   f_grade  with grade_force on (dyn4_ph_grade): dyn4_grade's (F_x, F_y, M_z) on p_h, zero
//         otherwise. It is minus the gradient of H's springs in the pose g, through the ground
//         under the wheels: with g in the state, T*SE(2)'s canonical structure puts it in J and the
//         terrain's energy is in H (where the single track can only book its grade as a power).
//         Booked on its own line, its power cancels the ground's:
//         xi^T f_grade = -sum_i k_i l_i hg_i'.
//
// So dH/dt = -dH/dx^T R dH/dx + y^T u + xi^T f_lever + xi^T f_grade + sum_i k_i l_i hg_i': passive
// with respect to the wheels' forces but for the lever, at every slip. The last term is the ground
// lifting the springs as the wheels roll over it (none on flat ground); without grade_force it is
// energy from outside, the vehicle climbing a hill for free, and with it the grade line cancels it.
// The inputs are the actuators' outputs (the state's delta and a). dyn4_plant's step departs from
// the vector field by its integrator (which holds the ground and the loads at the step's start)
// and by its limiters (the speed clamp, the low-speed blend); dyn4_plant_books charges the
// limiters and the ground's new heights their energy and leaves the integrator's error, second
// order per step, as the residual. These are evaluations: dyn4_plant's arithmetic does not depend
// on them.

// A tire's secant stiffness through its root w = 0: -F / w, and D C B / u, its limit, at w = 0.
// dyn_secant_w (the implicit step's) is the same but where |B alpha| < 1e-4, where it takes the
// slope D C B u / (u^2 + w^2), within 1e-8 of this; the form takes the exact secant, so S w is
// the whole force.
template <class S> CVC_HD inline S dyn4_ph_secant(S F, S w, S B, S C, S D, S u) {
  if (s_val(w) == 0.0)
    return D * C * B / u;
  return -F / w;
}

// Everything the form is built from, at a state: the wheels (dyn4_wheel_state) and dyn4_plant's
// steps 3 and 4 at the state's own twist, delta and a.
template <class S> struct dyn4_ph_parts {
  dyn4_wheels<S> W; // offsets, static loads, springs, dampers, the ground under them, the loads
  S cx;             // drag and rolling resistance on vx
  S Fx[4];          // each wheel's drive or brake force: the inputs
  S Fy[4];          // each tire's lateral force
  S w[4], ui[4];    // each wheel's lateral velocity in its own frame, and its floored speed
  S Sw[4];          // the secants (dyn4_ph_secant)
  S N[4][6];        // each tire's lateral force on the twist (vx, vy, r, zd, phid, thd)
  S T[4][6];        // each wheel's drive or brake force on the twist
};

template <class S, class G>
CVC_HD inline void dyn4_ph_parts_at(const dyn4_params &p, const bike_veh &v, const G &ground,
                                    const dyn4_state<S> &s, dyn4_ph_parts<S> &q) {
  const S m = S(p.m), g = S(p.g), h = S(p.h), Cp = S(p.C), L = S(p.lf) + S(p.lr);
  const S zero = S(0), one = S(1);
  dyn4_wheel_state(p, v, ground, s, q.W);
  const S eps2 = S(p.eps) * S(p.eps);
  const S u0 = s_sqrt(s.vx * s.vx + eps2);
  q.cx = S(p.drag) * u0 + S(p.c_rr) * m * g / u0;
  const S F_drive = m * s.a;
  const S share = S(s_val(s.a) >= 0.0 ? p.front_drive : p.front_brake);
  const S td = s_tan(s.delta);
  const S Bw[4] = {S(p.B_f), S(p.B_f), S(p.B_r), S(p.B_r)};
  for (int i = 0; i < 4; ++i) {
    const S xi = q.W.xi[i], yi = q.W.yi[i];
    const S cap = q.W.mu[i] * q.W.Fz[i];
    q.Fx[i] = dyn_sat(S(0.5) * F_drive * (i < 2 ? share : one - share), cap);
    const S Dy = dyn_lateral_cap(q.Fx[i], cap);
    const S dw = i < 2 ? dyn4_ackermann(L, yi, td) : zero;
    const S cw = s_cos(dw), sw = s_sin(dw);
    const S vxi = s.vx - s.r * yi, vyi = s.vy + s.r * xi;
    q.w[i] = -vxi * sw + vyi * cw;
    const S ul = vxi * cw + vyi * sw;
    q.ui[i] = s_sqrt(ul * ul + eps2);
    q.Fy[i] = dyn_lateral(-s_atan2(q.w[i], q.ui[i]), Bw[i], Cp, Dy);
    q.Sw[i] = dyn4_ph_secant(q.Fy[i], q.w[i], Bw[i], Cp, Dy, q.ui[i]);
    const S n[6] = {-sw, cw, xi * cw + yi * sw, zero, h * cw, h * sw};
    const S t[6] = {cw, sw, xi * sw - yi * cw, zero, h * sw, -h * cw};
    for (int k = 0; k < 6; ++k) {
      q.N[i][k] = n[k];
      q.T[i][k] = t[k];
    }
  }
}

// The energy H; see above.
template <class S, class G>
CVC_HD inline S dyn4_ph_hamiltonian(const dyn4_params &p, const bike_veh &v, const G &ground,
                                    const dyn4_state<S> &s) {
  dyn4_wheels<S> W;
  dyn4_wheel_state(p, v, ground, s, W);
  const S m = S(p.m);
  S H = S(0.5) * (m * (s.vx * s.vx + s.vy * s.vy + s.zd * s.zd) + S(p.Iz) * s.r * s.r +
                  S(p.Ix) * s.phid * s.phid + S(p.Iy) * s.thd * s.thd);
  for (int i = 0; i < 4; ++i) {
    const S l = W.Fz0[i] / W.k[i] + W.hg[i] - (s.z + W.yi[i] * s.phi - W.xi[i] * s.th);
    H = H + S(0.5) * W.k[i] * l * l;
  }
  return H + m * S(p.g) * s.z;
}

// x = (q, p): the vertical displacement and the momenta M xi.
template <class S>
CVC_HD inline void dyn4_ph_coords(const dyn4_params &p, const dyn4_state<S> &s, S x[9]) {
  const S m = S(p.m);
  x[0] = s.z;
  x[1] = s.phi;
  x[2] = s.th;
  x[3] = m * s.vx;
  x[4] = m * s.vy;
  x[5] = S(p.Iz) * s.r;
  x[6] = m * s.zd;
  x[7] = S(p.Ix) * s.phid;
  x[8] = S(p.Iy) * s.thd;
}

// dH/dx = (dH/dq, xi): the springs' and gravity's force on q, negated, and the twist.
template <class S, class G>
CVC_HD inline void dyn4_ph_grad(const dyn4_params &p, const bike_veh &v, const G &ground,
                                const dyn4_state<S> &s, S e[9]) {
  dyn4_wheels<S> W;
  dyn4_wheel_state(p, v, ground, s, W);
  e[0] = S(p.m) * S(p.g);
  e[1] = S(0);
  e[2] = S(0);
  for (int i = 0; i < 4; ++i) {
    const S kl = W.k[i] * (W.Fz0[i] / W.k[i] + W.hg[i] - (s.z + W.yi[i] * s.phi - W.xi[i] * s.th));
    e[0] = e[0] - kl;
    e[1] = e[1] - kl * W.yi[i];
    e[2] = e[2] + kl * W.xi[i];
  }
  e[3] = s.vx;
  e[4] = s.vy;
  e[5] = s.r;
  e[6] = s.zd;
  e[7] = s.phid;
  e[8] = s.thd;
}

// J(p): canonical in the vertical, the coadjoint structure in the horizontal.
template <class S> CVC_HD inline void dyn4_ph_J(const S x[9], S J[9][9]) {
  for (int a = 0; a < 9; ++a)
    for (int b = 0; b < 9; ++b)
      J[a][b] = S(0);
  S Jh[3][3];
  dyn_ph_J(x + 3, Jh);
  for (int a = 0; a < 3; ++a) {
    J[a][6 + a] = S(1);
    J[6 + a][a] = S(-1);
    for (int b = 0; b < 3; ++b)
      J[3 + a][3 + b] = Jh[a][b];
  }
}

// R, the dissipation: positive semi-definite at every slip.
template <class S, class G>
CVC_HD inline void dyn4_ph_R(const dyn4_params &p, const bike_veh &v, const G &ground,
                             const dyn4_state<S> &s, S R[9][9]) {
  dyn4_ph_parts<S> q;
  dyn4_ph_parts_at(p, v, ground, s, q);
  for (int a = 0; a < 9; ++a)
    for (int b = 0; b < 9; ++b)
      R[a][b] = S(0);
  for (int a = 0; a < 3; ++a)
    for (int b = a; b < 3; ++b) {
      S hor = S(0), ver = S(0);
      for (int i = 0; i < 4; ++i) {
        const S ci[3] = {S(1), q.W.yi[i], -q.W.xi[i]};
        hor = hor + q.Sw[i] * (q.N[i][a] * q.N[i][b]);
        ver = ver + q.W.c[i] * (ci[a] * ci[b]);
      }
      R[3 + a][3 + b] = hor;
      R[3 + b][3 + a] = hor;
      R[6 + a][6 + b] = ver;
      R[6 + b][6 + a] = ver;
    }
  R[3][3] = R[3][3] + q.cx;
}

// G, the input map for u = (F_x1, ..., F_x4): each wheel's force along it, at its contact patch.
template <class S, class G>
CVC_HD inline void dyn4_ph_G(const dyn4_params &p, const bike_veh &v, const G &ground,
                             const dyn4_state<S> &s, S Gm[9][4]) {
  dyn4_ph_parts<S> q;
  dyn4_ph_parts_at(p, v, ground, s, q);
  for (int i = 0; i < 4; ++i)
    for (int a = 0; a < 9; ++a)
      Gm[a][i] = a < 3 ? S(0) : q.T[i][a - 3];
}

// u: the wheels' drive or brake forces, from the actuator's a, within their grip.
template <class S, class G>
CVC_HD inline void dyn4_ph_input(const dyn4_params &p, const bike_veh &v, const G &ground,
                                 const dyn4_state<S> &s, S u[4]) {
  dyn4_ph_parts<S> q;
  dyn4_ph_parts_at(p, v, ground, s, q);
  for (int i = 0; i < 4; ++i)
    u[i] = q.Fx[i];
}

// f_lever: the tires' lateral forces' moments on roll and pitch, which their slips do not feel.
template <class S, class G>
CVC_HD inline void dyn4_ph_lever(const dyn4_params &p, const bike_veh &v, const G &ground,
                                 const dyn4_state<S> &s, S f[9]) {
  dyn4_ph_parts<S> q;
  dyn4_ph_parts_at(p, v, ground, s, q);
  for (int a = 0; a < 9; ++a)
    f[a] = S(0);
  for (int i = 0; i < 4; ++i)
    for (int k = 3; k < 6; ++k)
      f[3 + k] = f[3 + k] + q.Fy[i] * q.N[i][k];
}

// f_grade: with grade_force, gravity along the ground (dyn4_grade) on p_h; zero without.
template <class S, class G>
CVC_HD inline void dyn4_ph_grade(const dyn4_params &p, const bike_veh &v, const G &ground,
                                 const dyn4_state<S> &s, S f[9]) {
  for (int a = 0; a < 9; ++a)
    f[a] = S(0);
  if (!p.grade_force)
    return;
  dyn4_wheels<S> W;
  dyn4_wheel_state(p, v, ground, s, W);
  dyn4_grade(p, ground, s, W, f + 3);
}

// x' = (J - R) dH/dx + G u + f_lever + f_grade at the state: dyn4_plant's vector field.
template <class S, class G>
CVC_HD inline void dyn4_ph_rhs(const dyn4_params &p, const bike_veh &v, const G &ground,
                               const dyn4_state<S> &s, S xdot[9]) {
  S x[9], e[9], J[9][9], R[9][9], Gm[9][4], u[4], f[9], fg[9];
  dyn4_ph_coords(p, s, x);
  dyn4_ph_grad(p, v, ground, s, e);
  dyn4_ph_J(x, J);
  dyn4_ph_R(p, v, ground, s, R);
  dyn4_ph_G(p, v, ground, s, Gm);
  dyn4_ph_input(p, v, ground, s, u);
  dyn4_ph_lever(p, v, ground, s, f);
  dyn4_ph_grade(p, v, ground, s, fg);
  for (int a = 0; a < 9; ++a) {
    S acc = f[a] + fg[a];
    for (int i = 0; i < 4; ++i)
      acc = acc + Gm[a][i] * u[i];
    for (int b = 0; b < 9; ++b)
      acc = acc + (J[a][b] - R[a][b]) * e[b];
    xdot[a] = acc;
  }
}

// The power balance at a state, on the ground under it: dH/dt = supplied - dissipated + lever +
// grade (+ the ground's, as the wheels move to new heights; dyn4_plant_books books it per step).
template <class S> struct dyn4_ph_rates {
  S dissipated; // dH/dx^T R dH/dx >= 0: the tires' slip, drag, rolling resistance, the dampers
  S supplied;   // y^T u: the wheels' forces at their contact patches' speeds
  S lever;      // xi^T f_lever: the tires' lateral forces on roll and pitch
  S grade;      // xi^T f_grade: gravity along the ground (grade_force; 0 without it)
};

template <class S, class G>
CVC_HD inline dyn4_ph_rates<S> dyn4_ph_rates_at(const dyn4_params &p, const bike_veh &v,
                                                const G &ground, const dyn4_state<S> &s) {
  S e[9], R[9][9], Gm[9][4], u[4], f[9], fg[9];
  dyn4_ph_grad(p, v, ground, s, e);
  dyn4_ph_R(p, v, ground, s, R);
  dyn4_ph_G(p, v, ground, s, Gm);
  dyn4_ph_input(p, v, ground, s, u);
  dyn4_ph_lever(p, v, ground, s, f);
  dyn4_ph_grade(p, v, ground, s, fg);
  dyn4_ph_rates<S> out;
  out.dissipated = S(0);
  out.supplied = S(0);
  out.lever = S(0);
  out.grade = S(0);
  for (int a = 0; a < 9; ++a) {
    for (int b = 0; b < 9; ++b)
      out.dissipated = out.dissipated + e[a] * R[a][b] * e[b];
    for (int i = 0; i < 4; ++i)
      out.supplied = out.supplied + e[a] * Gm[a][i] * u[i];
    out.lever = out.lever + e[a] * f[a];
    out.grade = out.grade + e[a] * fg[a];
  }
  return out;
}

// One step's energy books, in J: dH = supplied - dissipated + lever + grade + ground + limiter +
// residual. The powers are the trapezoid between the step's start (with the inputs it applied,
// the actuators' outputs) and the state it reached before its limiters, both on the ground the
// step stood on (the start's pose: the pose enters H and the rates only through the ground).
// limiter is the kinetic energy the speed clamp and the low-speed blend then added (< 0:
// removed); ground is the springs' energy the new ground under the wheels added at the step's end
// (none on flat ground; with grade_force, grade + ground is second order in hdt per step);
// residual is the integrator's error, second order in hdt per step.
template <class S> struct dyn4_ph_books {
  S dH, dissipated, supplied, lever, grade, ground, limiter, residual;
};

// dyn4_plant's step with its books. The step is dyn4_plant's, to the last bit.
template <class S, class G>
CVC_HD inline void dyn4_plant_books(const dyn4_params &p, const bike_veh &v, const G &ground,
                                    const dyn4_state<S> &s, S delta_cmd, S a_cmd, dyn4_state<S> &o,
                                    dyn4_ph_books<S> &b) {
  dyn_step_free<S> seen;
  dyn4_plant<S>(p, v, ground, s, delta_cmd, a_cmd, o, &seen);
  dyn4_state<S> s0 = s, f = o;
  s0.delta = o.delta;
  s0.a = o.a;
  f.x = s.x;
  f.y = s.y;
  f.psi = s.psi;
  dyn4_state<S> e = f;
  e.vx = seen.vx;
  e.vy = seen.vy;
  e.r = seen.r;
  const dyn4_ph_rates<S> r0 = dyn4_ph_rates_at(p, v, ground, s0),
                         r1 = dyn4_ph_rates_at(p, v, ground, e);
  const S half = S(0.5) * S(v.hdt);
  b.dissipated = half * (r0.dissipated + r1.dissipated);
  b.supplied = half * (r0.supplied + r1.supplied);
  b.lever = half * (r0.lever + r1.lever);
  b.grade = half * (r0.grade + r1.grade);
  const S h0 = dyn4_ph_hamiltonian(p, v, ground, s), he = dyn4_ph_hamiltonian(p, v, ground, e),
          hf = dyn4_ph_hamiltonian(p, v, ground, f), h1 = dyn4_ph_hamiltonian(p, v, ground, o);
  b.dH = h1 - h0;
  b.limiter = hf - he;
  b.ground = h1 - hf;
  b.residual = b.dH - (b.supplied - b.dissipated + b.lever + b.grade + b.ground + b.limiter);
}

// One step under the trained controller, and its backward: bike_control's commands from the
// rear axle's pose and the speed vx. `ext`, when given, is an external force joined to the
// controller's (ext_in, diff_rollout.h), as dyn_bike_step takes it; the backward then returns
// dL/dF_ext in g_fext[2] (optional) and adds J^T dL/dF_ext to the position's gradient.
template <class G>
CVC_HD inline void dyn4_bike_step(const field &f, const dyn4_state<float> &s, float gx, float gy,
                                  float al, float be, float ga, const bike_veh &v,
                                  const dyn4_params &p, const G &ground, dyn4_state<float> &o,
                                  const ext_in *ext = nullptr) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd, nullptr, ext);
  dyn4_plant<float>(p, v, ground, s, delta_cmd, a_cmd, o);
}

template <class G>
CVC_HD inline void
dyn4_bike_step_bwd(const field &f, const dyn4_state<float> &s, float gx, float gy, float al,
                   float be, float ga, const bike_veh &v, const dyn4_params &p, const G &ground,
                   const dyn4_state<float> &g_o, dyn4_state<float> &g_s, float &gal, float &gbe,
                   float &gga, const ext_in *ext = nullptr, float *g_fext = nullptr) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd, nullptr, ext);
  float g_dc, g_ac;
  dyn4_plant_vjp<float>(p, v, ground, s, delta_cmd, a_cmd, g_o, g_s, g_dc, g_ac);
  float gox, goy, gth, gsp;
  bike_control_bwd(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, g_dc, g_ac, gox, goy, gth, gsp,
                   gal, gbe, gga, ext, g_fext);
  g_s.x += gox;
  g_s.y += goy;
  g_s.psi += gth;
  g_s.vx += gsp;
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_DYN_4W_H__
