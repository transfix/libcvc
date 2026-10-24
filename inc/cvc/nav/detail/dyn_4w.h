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

// One step of the plant; see the header comment.
template <class S, class G>
CVC_HD inline void dyn4_plant(const dyn4_params &p, const bike_veh &v, const G &ground,
                              const dyn4_state<S> &s, S delta_cmd, S a_cmd, dyn4_state<S> &o) {
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
  const S vx_p = dyn_clamp(s.vx + hdt * ((FXlong + F_res) / m), sp_min, vmax);

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
  const S vx1 = dyn_clamp(s.vx + hdt * (FXb / m + r1 * vy1), sp_min, vmax);

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
