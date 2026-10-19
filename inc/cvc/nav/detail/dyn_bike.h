/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// dyn_bike.h: the differentiable dynamic single-track vehicle (the vehicle-physics plan's P8a,
// doc 19 section 1). bike_control (diff_rollout.h) commands it exactly as it commands the
// kinematic bicycle, so a policy trains through tires, load transfer and actuator lag with the
// controller and its adjoint unchanged.
//
// State, in nav's normalized frame (lengths x S, time in s, mass in kg):
//   (x, y)  the rear axle, as bike_step's o;  psi  the heading;
//   vx, vy  the body-frame velocity at the centre of gravity (vx is bike_step's speed: it is the
//           same at every point on the body's axis);  r  the yaw rate;
//   delta   the road-wheel steering angle;  a  the drive and brake force per unit mass, both
//           behind their actuators.
//
// One step of hdt, given (delta_cmd, a_cmd) and the grade along the heading (sin of the slope,
// + uphill):
//   1. Actuators. The steering follows its command with a first-order lag, its change limited
//      smoothly to steer_rate x hdt (R tanh(d / R)); the force follows with its own lag. Lags
//      are exact for the step (1 - exp(-hdt / tau)), so none goes unstable at nav's step.
//   2. Loads. Quasi-static longitudinal load transfer from the actuator's force per unit mass:
//      Fz_f = m (g lr - a h) / L, Fz_r = m (g lf + a h) / L, floored at fz_min x m g.
//   3. Longitudinal. The drive or brake force splits between the axles, each saturating
//      smoothly at mu Fz (F / (1 + (F / mu Fz)^8)^(1/8)); the grip left for cornering is the
//      friction ellipse's, longitudinal first. The speed is predicted from these, drag, rolling
//      resistance and grade, then corrected after step 4 with the lateral forces the implicit
//      step applied: vx1 = vx + hdt (FX / m + r1 vy1), clamped to [sp_min, vmax] like the
//      kinematic plant.
//   4. Lateral and yaw. Slip angles with the new speed, smooth through zero speed:
//      alpha_f = sgn(vx1) delta - atan2(vy + lf r, u), alpha_r = -atan2(vy - lr r, u),
//      u = sqrt(vx1^2 + eps^2), sgn(vx1) = vx1 / u (vx1 the predicted speed). Pacejka lateral
//      forces D sin(C atan(B alpha)), D the grip left, infinitely smooth with a sign-correct
//      slope past the peak.
//      (vy, r) advance by implicit Euler, solved with `iters` steps from the current state, J
//      the Jacobian of (vy', r') in (vy, r) with each tire as its secant through its root in
//      lateral velocity (dyn_secant_w): stiff tires stay stable at nav's 0.06 s step and reach
//      the kinematic bicycle, and a sliding tire pulls its slip toward zero rather than past it,
//      even at a crawl with the wheel at full lock.
//   5. Low speed. A sigmoid in |vx1| around v_blend blends (vy, r) toward the kinematic
//      bicycle's, r = vx1 tan(delta) / L and vy = lr r, where slip angles mean nothing.
//   6. Pose. psi1 = psi + hdt r1; the rear axle moves with (vx1, vy1 - lr r1) at psi1, so in the
//      kinematic limit the step is bike_step's.
// Left out on purpose (doc 19): wheel spin, suspension, lateral load transfer, contact loss.
//
// dyn_plant is written once over a scalar type: float in training, double in the tests, and a
// forward-mode dual for its adjoint (doc 19: "a dual-number template for the plant is cheap and
// deletes the plant's hand reverse"). dyn_bike_step_bwd runs it once per input with a unit
// tangent, then hands the commands' gradients to bike_control_bwd. Everything is CVC_HD.

#ifndef __CVC_NAV_DETAIL_DYN_BIKE_H__
#define __CVC_NAV_DETAIL_DYN_BIKE_H__

#include <cvc/nav/detail/diff_rollout.h>

namespace cvc {
namespace nav {
namespace diff {

// ── Scalars: float, double and a forward-mode dual ─────────────────────────────────────────

CVC_HD inline float s_sqrt(float x) { return sqrtf(x); }
CVC_HD inline float s_sin(float x) { return sinf(x); }
CVC_HD inline float s_cos(float x) { return cosf(x); }
CVC_HD inline float s_tan(float x) { return tanf(x); }
CVC_HD inline float s_atan(float x) { return atanf(x); }
CVC_HD inline float s_atan2(float y, float x) { return atan2f(y, x); }
CVC_HD inline float s_tanh(float x) { return tanhf(x); }
CVC_HD inline float s_exp(float x) { return expf(x); }
CVC_HD inline float s_val(float x) { return x; }
CVC_HD inline double s_sqrt(double x) { return sqrt(x); }
CVC_HD inline double s_sin(double x) { return sin(x); }
CVC_HD inline double s_cos(double x) { return cos(x); }
CVC_HD inline double s_tan(double x) { return tan(x); }
CVC_HD inline double s_atan(double x) { return atan(x); }
CVC_HD inline double s_atan2(double y, double x) { return atan2(y, x); }
CVC_HD inline double s_tanh(double x) { return tanh(x); }
CVC_HD inline double s_exp(double x) { return exp(x); }
CVC_HD inline double s_val(double x) { return x; }

// A value and one tangent.
template <class T> struct dual {
  T v, d;
  CVC_HD dual() : v(0), d(0) {}
  CVC_HD dual(T value) : v(value), d(0) {}
  CVC_HD dual(T value, T tangent) : v(value), d(tangent) {}
};
template <class T> CVC_HD inline dual<T> operator+(dual<T> a, dual<T> b) {
  return dual<T>(a.v + b.v, a.d + b.d);
}
template <class T> CVC_HD inline dual<T> operator-(dual<T> a, dual<T> b) {
  return dual<T>(a.v - b.v, a.d - b.d);
}
template <class T> CVC_HD inline dual<T> operator-(dual<T> a) { return dual<T>(-a.v, -a.d); }
template <class T> CVC_HD inline dual<T> operator*(dual<T> a, dual<T> b) {
  return dual<T>(a.v * b.v, a.d * b.v + a.v * b.d);
}
template <class T> CVC_HD inline dual<T> operator/(dual<T> a, dual<T> b) {
  return dual<T>(a.v / b.v, (a.d * b.v - a.v * b.d) / (b.v * b.v));
}
template <class T> CVC_HD inline dual<T> s_sqrt(dual<T> a) {
  const T s = s_sqrt(a.v);
  return dual<T>(s, s > T(0) ? a.d / (T(2) * s) : T(0));
}
template <class T> CVC_HD inline dual<T> s_sin(dual<T> a) {
  return dual<T>(s_sin(a.v), a.d * s_cos(a.v));
}
template <class T> CVC_HD inline dual<T> s_cos(dual<T> a) {
  return dual<T>(s_cos(a.v), -a.d * s_sin(a.v));
}
template <class T> CVC_HD inline dual<T> s_tan(dual<T> a) {
  const T t = s_tan(a.v);
  return dual<T>(t, a.d * (T(1) + t * t));
}
template <class T> CVC_HD inline dual<T> s_atan(dual<T> a) {
  return dual<T>(s_atan(a.v), a.d / (T(1) + a.v * a.v));
}
template <class T> CVC_HD inline dual<T> s_atan2(dual<T> y, dual<T> x) {
  const T den = x.v * x.v + y.v * y.v;
  return dual<T>(s_atan2(y.v, x.v), den > T(0) ? (x.v * y.d - y.v * x.d) / den : T(0));
}
template <class T> CVC_HD inline dual<T> s_tanh(dual<T> a) {
  const T t = s_tanh(a.v);
  return dual<T>(t, a.d * (T(1) - t * t));
}
template <class T> CVC_HD inline dual<T> s_exp(dual<T> a) {
  const T e = s_exp(a.v);
  return dual<T>(e, a.d * e);
}
template <class T> CVC_HD inline T s_val(dual<T> a) { return a.v; }

// ── The vehicle ───────────────────────────────────────────────────────────────────────────

// In nav's normalized frame; nav/dyn_bike.h builds it from SI values (dyn_bike_si). The
// controller's bike_veh::L must be lf + lr.
struct dyn_bike_params {
  float m = 1.0f;    // kg
  float Iz = 1.0f;   // kg x length^2: the yaw inertia about the centre of gravity
  float lf = 0.5f;   // length: centre of gravity to the front axle
  float lr = 0.5f;   // length: centre of gravity to the rear axle
  float h = 0.0f;    // length: the centre of gravity's height
  float g = 9.81f;   // length / s^2
  float mu = 0.9f;   // the ground's friction
  float B_f = 12.0f; // 1/rad: Pacejka stiffness factor, front axle
  float B_r = 12.0f; // 1/rad: rear axle
  float C = 1.2f;    // Pacejka shape factor, in (1, 2): sin(C pi / 2) is what is left sliding
  float front_drive = 0.0f; // the front axle's share of a drive force
  float front_brake = 0.6f; // its share of a brake force
  float drag = 0.0f;        // kg / length: 1/2 rho C_d A
  float c_rr = 0.0f;        // rolling resistance coefficient
  float tau_steer = 0.0f;   // s: the steering's lag; 0 = none
  float steer_rate = 0.0f;  // rad/s: its slew limit; 0 = none
  float tau_drive = 0.0f;   // s: the force's lag; 0 = none
  float v_blend = 0.0f;     // length / s: below it, (vy, r) blend to the kinematic bicycle's
  float v_width = 1.0f;     // length / s: the blend's width
  float eps = 0.1f;         // length / s: the slip angles' speed floor
  float fz_min = 0.05f;     // the least load on an axle, as a fraction of m g
  int iters = 3;            // Newton steps of the implicit lateral solve
};

template <class S> struct dyn_state {
  S x, y, psi, vx, vy, r, delta, a;
  static const int size = 8;
  // The k-th component, in the order above.
  CVC_HD S &operator[](int k) {
    switch (k) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return psi;
    case 3:
      return vx;
    case 4:
      return vy;
    case 5:
      return r;
    case 6:
      return delta;
    default:
      return a;
    }
  }
  CVC_HD const S &operator[](int k) const { return (*const_cast<dyn_state *>(this))[k]; }
};

// A force saturated smoothly at cap: F / (1 + (F / cap)^8)^(1/8).
template <class S> CVC_HD inline S dyn_sat(S F, S cap) {
  const S q = (F * F) / (cap * cap);
  const S q4 = (q * q) * (q * q);
  return F / s_sqrt(s_sqrt(s_sqrt(S(1) + q4)));
}

// The lateral peak an axle has left beside a longitudinal force Fx (already within cap): the
// friction ellipse with the longitudinal force first, sqrt(cap^2 - (1 - e^2) Fx^2), so a locked
// or spinning axle keeps e = 5% of its grip sideways and the root stays smooth.
template <class S> CVC_HD inline S dyn_lateral_cap(S Fx, S cap) {
  return s_sqrt(cap * cap - S(0.9975) * Fx * Fx);
}

template <class S> CVC_HD inline S dyn_clamp(S x, S lo, S hi) {
  if (s_val(x) < s_val(lo))
    return lo;
  if (s_val(x) > s_val(hi))
    return hi;
  return x;
}

// Pacejka's lateral force, D sin(C atan(B alpha)).
template <class S> CVC_HD inline S dyn_lateral(S alpha, S B, S C, S D) {
  return D * s_sin(C * s_atan(B * alpha));
}

// A tire's stiffness against its lateral velocity w, as the secant through the force's root w0
// (where its slip is zero): -F / (w - w0). The force falls as w rises, so the secant is positive
// at any slip and any speed, and the linear model F = -S (w - w0) has the tire's own root: an
// implicit step on it never pushes the slip past zero, sliding or not, fast or at a crawl, where
// a slip angle's derivative u / (u^2 + w^2) vanishes. At zero slip it is the slope there,
// D C B u / (u^2 + w^2).
template <class S> CVC_HD inline S dyn_secant_w(S F, S alpha, S w, S w0, S B, S C, S D, S u) {
  const S ba = B * alpha;
  if (s_val(ba) * s_val(ba) < 1e-8)
    return D * C * B * u / (u * u + w * w);
  return -F / (w - w0);
}

// One step of the plant; see the header comment. grade is data (its gradient is not taken).
template <class S>
CVC_HD inline void dyn_plant(const dyn_bike_params &p, const bike_veh &v, const dyn_state<S> &s,
                             S delta_cmd, S a_cmd, S grade, dyn_state<S> &o) {
  const S hdt = S(v.hdt), zero = S(0), one = S(1);
  const S m = S(p.m), Iz = S(p.Iz), lf = S(p.lf), lr = S(p.lr), h = S(p.h), g = S(p.g);
  const S mu = S(p.mu), Cp = S(p.C), L = lf + lr;

  // 1. Actuators. (Computed in S, so a double plant is double throughout and a twin can match it
  // to rounding.)
  const S k_steer = p.tau_steer > 0.0f ? one - s_exp(-hdt / S(p.tau_steer)) : one;
  S step = (delta_cmd - s.delta) * k_steer;
  if (p.steer_rate > 0.0f) {
    const S R = S(p.steer_rate) * hdt;
    step = R * s_tanh(step / R);
  }
  const S delta1 = s.delta + step;
  const S k_drive = p.tau_drive > 0.0f ? one - s_exp(-hdt / S(p.tau_drive)) : one;
  const S a1 = s.a + (a_cmd - s.a) * k_drive;

  // 2. Loads.
  const S fz_floor = S(p.fz_min) * m * g;
  S Fzf = m * (g * lr - a1 * h) / L, Fzr = m * (g * lf + a1 * h) / L;
  if (s_val(Fzf) < s_val(fz_floor))
    Fzf = fz_floor;
  if (s_val(Fzr) < s_val(fz_floor))
    Fzr = fz_floor;
  const S cap_f = mu * Fzf, cap_r = mu * Fzr;
  const S F_drive = m * a1;
  const S share = S(s_val(a1) >= s_val(zero) ? p.front_drive : p.front_brake);
  // Each axle's drive or brake force within its grip, and what grip that leaves for cornering.
  const S Fxf = dyn_sat(F_drive * share, cap_f), Fxr = dyn_sat(F_drive * (one - share), cap_r);
  const S Dyf = dyn_lateral_cap(Fxf, cap_f), Dyr = dyn_lateral_cap(Fxr, cap_r);
  const S cd = s_cos(delta1), sd = s_sin(delta1);
  const S eps2 = S(p.eps) * S(p.eps);

  const S sp_min = S(v.allow_reverse ? -0.25f * v.vmax : 0.0f), vmax = S(v.vmax);

  // 3. The speed, predicted from the drive force and the resistances alone. The tires' lateral
  // forces enter only once step 4 has solved for them: in the stiff limit they are constraint
  // forces, which a slip times a stiffness gets badly wrong. For the same reason nothing
  // longitudinal reads a lateral force before the solve.
  const S u0 = s_sqrt(s.vx * s.vx + eps2), sg0 = s.vx / u0;
  const S F_res = -S(p.drag) * s.vx * u0 - S(p.c_rr) * m * g * sg0 - m * g * grade;
  const S vx_p = dyn_clamp(s.vx + hdt * ((Fxf * cd + Fxr + F_res) / m), sp_min, vmax);

  // 4. Lateral and yaw: implicit Euler for (vy, r) with the predicted speed, solved by `iters`
  // Newton-like steps from the current state, each linearizing the tires by their secants through
  // their roots at its iterate: (I - hdt J) d = x0 + hdt f(x) - x.
  const S u = s_sqrt(vx_p * vx_p + eps2), sg = vx_p / u;
  const S Bf = S(p.B_f), Br = S(p.B_r);
  const S w0f =
      u * s_tan(sg * delta1); // the front's lateral velocity at zero slip; the rear's is 0
  S vy1 = s.vy, r1 = s.r, Fyf_step = S(0);
  const int iters = p.iters > 0 ? p.iters : 1;
  for (int it = 0; it < iters; ++it) {
    const S wf = vy1 + lf * r1, wr = vy1 - lr * r1;
    const S af = sg * delta1 - s_atan2(wf, u), ar = -s_atan2(wr, u);
    const S Fyf = dyn_lateral(af, Bf, Cp, Dyf), Fyr = dyn_lateral(ar, Br, Cp, Dyr);
    const S FY = Fxf * sd + Fyf * cd + Fyr;
    const S MZ = lf * (Fxf * sd + Fyf * cd) - lr * Fyr;
    const S f_vy = FY / m - r1 * vx_p, f_r = MZ / Iz;
    const S Sf_t = dyn_secant_w(Fyf, af, wf, w0f, Bf, Cp, Dyf, u);
    const S Sr = dyn_secant_w(Fyr, ar, wr, S(0), Br, Cp, Dyr, u);
    const S Sf = Sf_t * cd;
    // dFyf/d(vy, r) = -Sf_t (1, lf); dFyr/d(vy, r) = -Sr (1, -lr).
    const S J11 = (-Sf - Sr) / m;
    const S J12 = (-Sf * lf + Sr * lr) / m - vx_p;
    const S J21 = (-Sf * lf + Sr * lr) / Iz;
    const S J22 = (-Sf * lf * lf - Sr * lr * lr) / Iz;
    const S A11 = one - hdt * J11, A12 = -hdt * J12, A21 = -hdt * J21, A22 = one - hdt * J22;
    S det = A11 * A22 - A12 * A21;
    if (s_val(det) < s_val(S(0.05)))
      det = S(0.05);
    const S b1 = s.vy + hdt * f_vy - vy1, b2 = s.r + hdt * f_r - r1;
    const S dvy = (A22 * b1 - A12 * b2) / det, dr = (A11 * b2 - A21 * b1) / det;
    vy1 = vy1 + dvy;
    r1 = r1 + dr;
    // The front force this linearization applies.
    Fyf_step = Fyf - Sf_t * (dvy + lf * dr);
  }

  // 3, corrected: the speed with the front lateral force the implicit step applied.
  const S FX = Fxf * cd - Fyf_step * sd + Fxr + F_res;
  const S vx1 = dyn_clamp(s.vx + hdt * (FX / m + r1 * vy1), sp_min, vmax);

  // 5. Low speed: toward the kinematic bicycle.
  if (p.v_blend > 0.0f) {
    const S w = one / (one + s_exp((s_sqrt(vx1 * vx1 + eps2) - S(p.v_blend)) / S(p.v_width)));
    const S r_k = vx1 * s_tan(delta1) / L, vy_k = lr * r_k;
    vy1 = vy1 + w * (vy_k - vy1);
    r1 = r1 + w * (r_k - r1);
  }

  // 6. Pose.
  const S psi1 = s.psi + hdt * r1;
  const S v_side = vy1 - lr * r1; // the rear axle's sideways velocity
  const S c1 = s_cos(psi1), s1 = s_sin(psi1);
  o.x = s.x + hdt * (vx1 * c1 - v_side * s1);
  o.y = s.y + hdt * (vx1 * s1 + v_side * c1);
  o.psi = psi1;
  o.vx = vx1;
  o.vy = vy1;
  o.r = r1;
  o.delta = delta1;
  o.a = a1;
}

// The plant's vector-Jacobian product: gradients on its outputs -> on its state and commands.
template <class T>
CVC_HD inline void dyn_plant_vjp(const dyn_bike_params &p, const bike_veh &v, const dyn_state<T> &s,
                                 T delta_cmd, T a_cmd, T grade, const dyn_state<T> &g_o,
                                 dyn_state<T> &g_s, T &g_delta_cmd, T &g_a_cmd) {
  typedef dual<T> D;
  const int n = dyn_state<T>::size;
  for (int j = 0; j < n + 2; ++j) {
    dyn_state<D> ds, o;
    for (int k = 0; k < n; ++k)
      ds[k] = D(s[k], k == j ? T(1) : T(0));
    dyn_plant<D>(p, v, ds, D(delta_cmd, j == n ? T(1) : T(0)), D(a_cmd, j == n + 1 ? T(1) : T(0)),
                 D(grade), o);
    T acc = T(0);
    for (int k = 0; k < n; ++k)
      acc += g_o[k] * o[k].d;
    if (j < n)
      g_s[j] = acc;
    else if (j == n)
      g_delta_cmd = acc;
    else
      g_a_cmd = acc;
  }
}

// ── The vehicle under the trained controller ──────────────────────────────────────────────

// One step: bike_control's commands from the rear axle's pose and the speed vx, through the
// dynamic plant.
CVC_HD inline void dyn_bike_step(const field &f, const dyn_state<float> &s, float gx, float gy,
                                 float al, float be, float ga, const bike_veh &v,
                                 const dyn_bike_params &p, float grade, dyn_state<float> &o) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd);
  dyn_plant<float>(p, v, s, delta_cmd, a_cmd, grade, o);
}

// Backward of dyn_bike_step: gradients on the next state -> on the state and (al, be, ga).
CVC_HD inline void dyn_bike_step_bwd(const field &f, const dyn_state<float> &s, float gx, float gy,
                                     float al, float be, float ga, const bike_veh &v,
                                     const dyn_bike_params &p, float grade,
                                     const dyn_state<float> &g_o, dyn_state<float> &g_s, float &gal,
                                     float &gbe, float &gga) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd);
  float g_dc, g_ac;
  dyn_plant_vjp<float>(p, v, s, delta_cmd, a_cmd, grade, g_o, g_s, g_dc, g_ac);
  float gox, goy, gth, gsp;
  bike_control_bwd(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, g_dc, g_ac, gox, goy, gth, gsp,
                   gal, gbe, gga);
  g_s.x += gox;
  g_s.y += goy;
  g_s.psi += gth;
  g_s.vx += gsp;
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_DYN_BIKE_H__
