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
//   4. Lateral and yaw. Slip angles in each wheel's frame with the new speed, smooth through zero
//      speed, as dyn_4w's: alpha = -atan2(w, u_w), w the axle's sideways velocity in its wheel's
//      frame and u_w its forward speed there, floored (sqrt(. ^2 + eps^2)). The front wheel is
//      turned by delta: w_f = cos(delta) (vy + lf r) - sin(delta) vx1, u_wf from
//      cos(delta) vx1 + sin(delta) (vy + lf r); the rear's is w_r = vy - lr r, u_wr from vx1 (vx1
//      the predicted speed). Pacejka lateral forces D sin(C atan(B alpha)), D the grip left,
//      infinitely smooth with a sign-correct slope past the peak.
//      (vy, r) advance by implicit Euler, solved with `iters` steps from the current state, J
//      the Jacobian of (vy', r') in (vy, r) with each tire as its secant through its root, zero
//      sideways velocity in its wheel's frame (dyn_secant_w): stiff tires stay stable at nav's
//      0.06 s step and reach the kinematic bicycle, and a sliding tire pulls its slip toward zero
//      rather than past it, even at a crawl with the wheel at full lock.
//   5. Low speed. A sigmoid in |vx1| around v_blend blends (vy, r) toward the kinematic
//      bicycle's, r = vx1 tan(delta) / L and vy = lr r, where slip angles mean nothing.
//   6. Pose. psi1 = psi + hdt r1; the rear axle moves with (vx1, vy1 - lr r1) at psi1, so in the
//      kinematic limit the step is bike_step's.
// Left out on purpose (doc 19): suspension, lateral load transfer, contact loss. Wheel spin is
// opt-in: dyn_spin_plant (below) gives each axle's wheels their own speed, driven and braked by
// torques, and its tires a longitudinal force from their slip; dyn_plant has none.
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
  // The wheels, for dyn_spin_plant (dyn_plant reads none of these).
  float r_w = 0.4f;  // length: the wheels' rolling radius
  float J_f = 1.0f;  // kg x length^2: the front axle's wheels' spin inertia, both together
  float J_r = 1.0f;  // the rear axle's
  float B_x = 12.0f; // Pacejka stiffness factor of the longitudinal force, per unit slip ratio
  float C_x = 1.2f;  // its shape factor, in (1, 2)
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

// dyn_spin_plant's state: dyn_plant's, and each axle's wheel speed.
template <class S> struct dyn_spin_state {
  dyn_state<S> b; // the body and the actuators, as dyn_plant's
  S om_f, om_r;   // rad/s: the front and rear wheels' angular speed, + rolling forward
  static const int size = 10;
  // The k-th component: b's eight, then om_f, om_r.
  CVC_HD S &operator[](int k) { return k < 8 ? b[k] : (k == 8 ? om_f : om_r); }
  CVC_HD const S &operator[](int k) const { return (*const_cast<dyn_spin_state *>(this))[k]; }
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

// The twist one step reached before its limiters: the speed before the clamp to [sp_min, vmax],
// (vy, r) before the low-speed blend. dyn_plant fills it when asked; the energy books
// (dyn_plant_books) charge the limiters the kinetic energy between it and the step's result.
template <class S> struct dyn_step_free {
  S vx, vy, r;
};

// ── Wheel spin (dyn_spin_plant) ───────────────────────────────────────────────────────────
// Each axle's wheels spin at om (rad/s), with spin inertia J and rolling radius r_w. The
// actuator's a becomes torques on them, split between the axles as dyn_plant splits its force:
// a >= 0 drives, each axle's T_d = r_w x its share (front_drive) of m a; a < 0 brakes, each axle's
// T_b = r_w x its share (front_brake) of m |a|, against the wheel's rolling: - T_b sg(om), sg the
// sign smoothed over the slip's speed floor at the rim, om / sqrt(om^2 + (eps / r_w)^2). A brake
// stops a wheel and holds it, so a negative a never drives backward (as Jolt's driver, which
// brakes for it). The torques, the brake's sign aside, are those a wheel that does not slip
// passes on as dyn_plant's forces.
// The tire pushes the body along its wheel, t (t_f = (cos delta, sin delta, lf sin delta),
// t_r = (1, 0, 0) on (vx, vy, r)), with Pacejka's longitudinal force of its slip ratio:
//   F_x = D sin(C_x atan(B_x kappa)),   kappa = sigma / u,   sigma = r_w om - t^T v,
// sigma the contact patch's slip velocity (the rim's speed over the ground's along the wheel), u
// the ground's speed along the wheel floored as the slip angles' are, and D = mu F_z, the axle's
// whole grip; the grip left for cornering is the friction ellipse's as in dyn_plant, with this
// force. The wheel feels - r_w F_x. A wheel that spins (sigma > 0) or locks (sigma < 0) slides
// once |kappa| passes its peak, tan(pi / (2 C_x)) / B_x, and takes the grip it slides with from
// the cornering.

// The two axles' drive and brake torques for the actuator's a (see above): Td the drive's, Tb the
// brakes' magnitude.
template <class S>
CVC_HD inline void dyn_spin_torques(const dyn_bike_params &p, S a, S Td[2], S Tb[2]) {
  const bool drive = s_val(a) >= 0.0;
  const S share = S(drive ? p.front_drive : p.front_brake);
  const S T = S(p.r_w) * S(p.m) * a;
  const S Tf = T * share, Tr = T * (S(1) - share);
  Td[0] = drive ? Tf : S(0);
  Td[1] = drive ? Tr : S(0);
  Tb[0] = drive ? S(0) : -Tf;
  Tb[1] = drive ? S(0) : -Tr;
}

// A tire's longitudinal force at slip velocity sigma over the floored ground speed u.
template <class S> CVC_HD inline S dyn_longitudinal(S sigma, S u, S B, S C, S D) {
  return D * s_sin(C * s_atan(B * (sigma / u)));
}

// Its stiffness against sigma, as the secant through its root sigma = 0: F / sigma, >= 0 at any
// slip because a Pacejka force (C < 2) has its slip's sign. Where |B kappa| < 1e-4 it is the slope
// there, D C B / u (within 1e-8 of the secant), as dyn_secant_w is.
template <class S> CVC_HD inline S dyn_secant_x(S F, S sigma, S B, S C, S D, S u) {
  const S bk = B * (sigma / u);
  if (s_val(bk) * s_val(bk) < 1e-8)
    return D * C * B / u;
  return F / sigma;
}

// The longitudinal step with the wheels spinning: (vx, om_f, om_r) by implicit Euler on the tires'
// longitudinal forces and the wheels' torques, the forces and the brakes linearized by their
// secants through their roots (no slip; a still wheel), `iters` steps from the current state:
// (M + hdt R_x) d = M (z0 + hdt f(z) - z), M = diag(m, J_f, J_r), R_x = K_f d_f d_f^T +
// K_r d_r d_r^T + diag(0, Kb_f, Kb_r), d = (-t_x, r_w e_wheel). That is symmetric positive
// definite, so the solve never fails, stiff tires stay stable at nav's step, and a locking or
// spinning wheel's slip is pulled toward zero rather than past it. The front's ground speed along
// its wheel takes vy and r from the step's start. Gives the speed (unclamped), the wheels' new
// speeds and the forces the last linearization applied.
template <class S>
CVC_HD inline void dyn_spin_long_step(const dyn_bike_params &p, S hdt, const dyn_state<S> &s,
                                      const S om0[2], S a1, S cd, S sd, S cap_f, S cap_r, S F_res,
                                      S &vx1, S om1[2], S &Fxf, S &Fxr) {
  const S m = S(p.m), lf = S(p.lf), rw = S(p.r_w), Jf = S(p.J_f), Jr = S(p.J_r);
  const S Bx = S(p.B_x), Cx = S(p.C_x), eps2 = S(p.eps) * S(p.eps), oe2 = eps2 / (rw * rw);
  S Td[2], Tb[2];
  dyn_spin_torques(p, a1, Td, Tb);
  const S side_f = (s.vy + lf * s.r) * sd; // the front's ground speed along its wheel, less vx's
  S vx = s.vx, of = om0[0], orr = om0[1];
  Fxf = S(0);
  Fxr = S(0);
  const int iters = p.iters > 0 ? p.iters : 1;
  for (int it = 0; it < iters; ++it) {
    const S gf = vx * cd + side_f, gr = vx;
    const S uf = s_sqrt(gf * gf + eps2), ur = s_sqrt(gr * gr + eps2);
    const S sf = rw * of - gf, sr = rw * orr - gr;
    const S Ff = dyn_longitudinal(sf, uf, Bx, Cx, cap_f);
    const S Fr = dyn_longitudinal(sr, ur, Bx, Cx, cap_r);
    const S Kf = dyn_secant_x(Ff, sf, Bx, Cx, cap_f, uf);
    const S Kr = dyn_secant_x(Fr, sr, Bx, Cx, cap_r, ur);
    const S Kbf = Tb[0] / s_sqrt(of * of + oe2), Kbr = Tb[1] / s_sqrt(orr * orr + oe2);
    // M (z0 + hdt f(z) - z).
    const S b0 = m * (s.vx - vx) + hdt * (Ff * cd + Fr + F_res);
    const S b1 = Jf * (om0[0] - of) + hdt * (Td[0] - Kbf * of - rw * Ff);
    const S b2 = Jr * (om0[1] - orr) + hdt * (Td[1] - Kbr * orr - rw * Fr);
    // M + hdt R_x: the wheels' rows couple only through vx.
    const S A00 = m + hdt * (Kf * cd * cd + Kr), A01 = -hdt * Kf * cd * rw, A02 = -hdt * Kr * rw;
    const S A11 = Jf + hdt * (Kf * rw * rw + Kbf), A22 = Jr + hdt * (Kr * rw * rw + Kbr);
    const S d0 = (b0 - A01 * b1 / A11 - A02 * b2 / A22) / (A00 - A01 * A01 / A11 - A02 * A02 / A22);
    const S d1 = (b1 - A01 * d0) / A11, d2 = (b2 - A02 * d0) / A22;
    vx = vx + d0;
    of = of + d1;
    orr = orr + d2;
    // The forces this linearization applies.
    Fxf = Ff + Kf * (rw * d1 - cd * d0);
    Fxr = Fr + Kr * (rw * d2 - d0);
  }
  vx1 = vx;
  om1[0] = of;
  om1[1] = orr;
}

// One step of the plant (om0 null), or of the wheel-spin plant (om0 and om1 its wheels' speeds
// before and after); see the header comment. grade is data (its gradient is not taken).
template <class S>
CVC_HD inline void dyn_plant_core(const dyn_bike_params &p, const bike_veh &v,
                                  const dyn_state<S> &s, const S *om0, S delta_cmd, S a_cmd,
                                  S grade, dyn_state<S> &o, S *om1, dyn_step_free<S> *seen) {
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
  const S cd = s_cos(delta1), sd = s_sin(delta1);
  const S eps2 = S(p.eps) * S(p.eps);

  const S sp_min = S(v.allow_reverse ? -0.25f * v.vmax : 0.0f), vmax = S(v.vmax);

  // 3. The speed, predicted from the drive force and the resistances alone. The tires' lateral
  // forces enter only once step 4 has solved for them: in the stiff limit they are constraint
  // forces, which a slip times a stiffness gets badly wrong. For the same reason nothing
  // longitudinal reads a lateral force before the solve.
  const S u0 = s_sqrt(s.vx * s.vx + eps2), sg0 = s.vx / u0;
  const S F_res = -S(p.drag) * s.vx * u0 - S(p.c_rr) * m * g * sg0 - m * g * grade;
  // Each axle's drive or brake force within its grip. With the wheels spinning, the tires'
  // longitudinal forces from their slip, solved with the speed and the wheels'
  // (dyn_spin_long_step).
  S Fxf, Fxr, vx_p;
  if (!om0) {
    Fxf = dyn_sat(F_drive * share, cap_f);
    Fxr = dyn_sat(F_drive * (one - share), cap_r);
    vx_p = dyn_clamp(s.vx + hdt * ((Fxf * cd + Fxr + F_res) / m), sp_min, vmax);
  } else {
    S vx_s;
    dyn_spin_long_step(p, hdt, s, om0, a1, cd, sd, cap_f, cap_r, F_res, vx_s, om1, Fxf, Fxr);
    vx_p = dyn_clamp(vx_s, sp_min, vmax);
    // A linearization's force may pass the grip by its error: the ellipse reads it within.
    Fxf = dyn_clamp(Fxf, -cap_f, cap_f);
    Fxr = dyn_clamp(Fxr, -cap_r, cap_r);
  }
  // What grip that leaves for cornering.
  const S Dyf = dyn_lateral_cap(Fxf, cap_f), Dyr = dyn_lateral_cap(Fxr, cap_r);

  // 4. Lateral and yaw: implicit Euler for (vy, r) with the predicted speed, solved by `iters`
  // Newton-like steps from the current state, each linearizing the tires by their secants through
  // their roots at its iterate: (I - hdt J) d = x0 + hdt f(x) - x.
  const S u = s_sqrt(vx_p * vx_p + eps2);
  const S Bf = S(p.B_f), Br = S(p.B_r);
  S vy1 = s.vy, r1 = s.r, Fyf_step = S(0);
  const int iters = p.iters > 0 ? p.iters : 1;
  for (int it = 0; it < iters; ++it) {
    // Each axle's velocity in its wheel's frame: the front's sideways wf and forward speed uf
    // (floored), the rear's sideways wr. Each tire's root is wf or wr = 0.
    const S vf = vy1 + lf * r1;
    const S wf = -vx_p * sd + vf * cd, wr = vy1 - lr * r1;
    const S uf = s_sqrt((vx_p * cd + vf * sd) * (vx_p * cd + vf * sd) + eps2);
    const S af = -s_atan2(wf, uf), ar = -s_atan2(wr, u);
    const S Fyf = dyn_lateral(af, Bf, Cp, Dyf), Fyr = dyn_lateral(ar, Br, Cp, Dyr);
    const S FY = Fxf * sd + Fyf * cd + Fyr;
    const S MZ = lf * (Fxf * sd + Fyf * cd) - lr * Fyr;
    const S f_vy = FY / m - r1 * vx_p, f_r = MZ / Iz;
    const S Sf_t = dyn_secant_w(Fyf, af, wf, S(0), Bf, Cp, Dyf, uf);
    const S Sr = dyn_secant_w(Fyr, ar, wr, S(0), Br, Cp, Dyr, u);
    // dwf/d(vy, r) = cos(delta) (1, lf), so dFyf/d(vy, r) = -Sf (1, lf), and on the body's y,
    // d(Fyf cos delta)/d(vy, r) = -Sfy (1, lf); dFyr/d(vy, r) = -Sr (1, -lr).
    const S Sf = Sf_t * cd, Sfy = Sf * cd;
    const S J11 = (-Sfy - Sr) / m;
    const S J12 = (-Sfy * lf + Sr * lr) / m - vx_p;
    const S J21 = (-Sfy * lf + Sr * lr) / Iz;
    const S J22 = (-Sfy * lf * lf - Sr * lr * lr) / Iz;
    const S A11 = one - hdt * J11, A12 = -hdt * J12, A21 = -hdt * J21, A22 = one - hdt * J22;
    S det = A11 * A22 - A12 * A21;
    if (s_val(det) < s_val(S(0.05)))
      det = S(0.05);
    const S b1 = s.vy + hdt * f_vy - vy1, b2 = s.r + hdt * f_r - r1;
    const S dvy = (A22 * b1 - A12 * b2) / det, dr = (A11 * b2 - A21 * b1) / det;
    vy1 = vy1 + dvy;
    r1 = r1 + dr;
    // The front force this linearization applies.
    Fyf_step = Fyf - Sf * (dvy + lf * dr);
  }

  // 3, corrected: the speed with the front lateral force the implicit step applied.
  const S FX = Fxf * cd - Fyf_step * sd + Fxr + F_res;
  const S vx1_free = s.vx + hdt * (FX / m + r1 * vy1);
  const S vx1 = dyn_clamp(vx1_free, sp_min, vmax);
  if (seen) {
    seen->vx = vx1_free;
    seen->vy = vy1;
    seen->r = r1;
  }

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

// One step of the plant; see the header comment. grade is data (its gradient is not taken).
// `seen`, when given, gets the twist the step reached before its limiters (dyn_plant_books).
template <class S>
CVC_HD inline void dyn_plant(const dyn_bike_params &p, const bike_veh &v, const dyn_state<S> &s,
                             S delta_cmd, S a_cmd, S grade, dyn_state<S> &o,
                             dyn_step_free<S> *seen = nullptr) {
  dyn_plant_core<S>(p, v, s, nullptr, delta_cmd, a_cmd, grade, o, nullptr, seen);
}

// One step of the wheel-spin plant: dyn_plant's with the wheels spinning (see "Wheel spin" above),
// their speeds advanced with the speed. The low-speed blend and the speed clamp act on the body
// alone.
template <class S>
CVC_HD inline void dyn_spin_plant(const dyn_bike_params &p, const bike_veh &v,
                                  const dyn_spin_state<S> &s, S delta_cmd, S a_cmd, S grade,
                                  dyn_spin_state<S> &o, dyn_step_free<S> *seen = nullptr) {
  const S om0[2] = {s.om_f, s.om_r};
  S om1[2];
  dyn_plant_core<S>(p, v, s.b, om0, delta_cmd, a_cmd, grade, o.b, om1, seen);
  o.om_f = om1[0];
  o.om_r = om1[1];
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

// The wheel-spin plant's vector-Jacobian product, as dyn_plant_vjp's.
template <class T>
CVC_HD inline void dyn_spin_plant_vjp(const dyn_bike_params &p, const bike_veh &v,
                                      const dyn_spin_state<T> &s, T delta_cmd, T a_cmd, T grade,
                                      const dyn_spin_state<T> &g_o, dyn_spin_state<T> &g_s,
                                      T &g_delta_cmd, T &g_a_cmd) {
  typedef dual<T> D;
  const int n = dyn_spin_state<T>::size;
  for (int j = 0; j < n + 2; ++j) {
    dyn_spin_state<D> ds, o;
    for (int k = 0; k < n; ++k)
      ds[k] = D(s[k], k == j ? T(1) : T(0));
    dyn_spin_plant<D>(p, v, ds, D(delta_cmd, j == n ? T(1) : T(0)),
                      D(a_cmd, j == n + 1 ? T(1) : T(0)), D(grade), o);
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

// ── The plant in port-Hamiltonian form ────────────────────────────────────────────────────
// Doc 20's rungs 2 and 3, at the plant, at every slip. The body is a planar rigid body: the pose
// q = (x, y, psi) on SE(2) and the body momenta p = M v, with v = (vx, vy, r) the body twist at the
// centre of gravity and M = diag(m, m, Iz). dyn_plant's vector field (its step as hdt -> 0, the
// actuators holding) is exactly
//
//   H(p) = 1/2 p^T M^-1 p,   dH/dp = v
//   p' = (J(p) - R(v, delta)) v + G(delta) u + f_eps - m g grade e_x,   u = (F_xf, F_xr),
//   y = G^T v
//
//   J(p)  [[0, 0, p_y], [0, 0, -p_x], [-p_y, p_x, 0]]: SE(2)'s coadjoint action, J(p) v = ad*_v p.
//         It gives the + r vy in vx' and - r vx in vy' (and, with equal masses, nothing in r').
//         Skew, so it moves no energy; a Lie-Poisson structure, so Jacobi holds (unlike the
//         kinematic bicycle, whose no-slip constraint is not integrable).
//   R     D_x e_x e_x^T + S_f n_f n_f^T + S_r n_r n_r^T. D_x = drag u + c_rr m g / u is drag and
//         rolling resistance, opposing vx (u = sqrt(vx^2 + eps^2), as the plant smooths them).
//         n_f = (-sin delta, cos delta, lf cos delta) and n_r = (0, 1, -lr) carry each tire's
//         lateral force onto (vx, vy, r), and n^T v is its contact patch's sideways velocity in
//         its wheel's frame, the slip's w. S_f, S_r are the tires' secant stiffnesses through
//         their roots at w = 0 (dyn_secant_w), >= 0 at any slip because a Pacejka force (C < 2)
//         always opposes its slip. So R is positive semi-definite sliding or not, at any delta.
//   G     [t_f, t_r], t_f = (cos delta, sin delta, lf sin delta), t_r = (1, 0, 0): each axle's
//         drive or brake force (within its grip) along its wheel. y^T u is the power those forces
//         deliver at the wheels' ground speed. Steering is not an input: delta only turns n_f and
//         t_f, so it does no work on the body.
//   f_eps n_f (F_yf + S_f n_f^T v) + n_r (F_yr + S_r n_r^T v): what the secants leave of the
//         tires' forces. Each slip is its wheel's frame's, rooted where n^T v = 0, so this is only
//         the secants' rounding (at zero slip, where dyn_secant_w takes the slope): round-off. The
//         speed floor eps enters the slips' forward speeds alone, which the force's sign does not
//         read. It is booked on its own line all the same.
//   grade gravity along the heading, booked as the power - m g grade vx (the plant has no side
//         slope, so H carries no potential).
//
// So dH/dt = -v^T R v + y^T u + v^T f_eps - m g grade vx, with v^T f_eps at round-off: passive
// with respect to the axle forces, at every slip. The inputs are the actuators' outputs (the
// state's delta and a): their lags are outside the body. dyn_plant's step departs from the vector
// field by its integrator (implicit Euler on (vy, r), a predicted then corrected vx) and by its
// limiters (the speed clamp, the low-speed blend toward the kinematic bicycle); dyn_plant_books
// charges each step's limiters their kinetic energy and leaves the integrator's error, second order
// per step, as the residual. Past a tire's peak its force falls as slip grows and the tangent
// (dyn_ph_tangent_D), the linearization's damping, loses definiteness: the motion can be locally
// unstable there, but the secant R, the energy's, stays positive semi-definite. These are
// evaluations: dyn_plant's arithmetic does not depend on them.

// The kinetic energy 1/2 v^T M v.
template <class S>
CVC_HD inline S dyn_ph_hamiltonian(const dyn_bike_params &p, const dyn_state<S> &s) {
  return S(0.5) * (S(p.m) * (s.vx * s.vx + s.vy * s.vy) + S(p.Iz) * s.r * s.r);
}

// p = M v.
template <class S>
CVC_HD inline void dyn_ph_momentum(const dyn_bike_params &p, const dyn_state<S> &s, S pm[3]) {
  pm[0] = S(p.m) * s.vx;
  pm[1] = S(p.m) * s.vy;
  pm[2] = S(p.Iz) * s.r;
}

// J(p), the coadjoint structure.
template <class S> CVC_HD inline void dyn_ph_J(const S pm[3], S J[3][3]) {
  const S z = S(0);
  J[0][0] = z;
  J[0][1] = z;
  J[0][2] = pm[1];
  J[1][0] = z;
  J[1][1] = z;
  J[1][2] = -pm[0];
  J[2][0] = -pm[1];
  J[2][1] = pm[0];
  J[2][2] = z;
}

// Each axle's grip, mu F_z, its load from the state's own a (dyn_plant's step 2).
template <class S>
CVC_HD inline void dyn_ph_caps(const dyn_bike_params &p, const dyn_state<S> &s, S &cap_f,
                               S &cap_r) {
  const S m = S(p.m), g = S(p.g), lf = S(p.lf), lr = S(p.lr), h = S(p.h), L = lf + lr;
  const S fz_floor = S(p.fz_min) * m * g;
  S Fzf = m * (g * lr - s.a * h) / L, Fzr = m * (g * lf + s.a * h) / L;
  if (s_val(Fzf) < s_val(fz_floor))
    Fzf = fz_floor;
  if (s_val(Fzr) < s_val(fz_floor))
    Fzr = fz_floor;
  cap_f = S(p.mu) * Fzf;
  cap_r = S(p.mu) * Fzr;
}

// Each axle's drive or brake force within its grip, and the lateral grip it leaves (dyn_plant's
// step 2 at the state's own a): Fxf, Fxr, Dyf, Dyr.
template <class S>
CVC_HD inline void dyn_ph_axles(const dyn_bike_params &p, const dyn_state<S> &s, S &Fxf, S &Fxr,
                                S &Dyf, S &Dyr) {
  const S m = S(p.m);
  S cap_f, cap_r;
  dyn_ph_caps(p, s, cap_f, cap_r);
  const S F_drive = m * s.a;
  const S share = S(s_val(s.a) >= 0.0 ? p.front_drive : p.front_brake);
  Fxf = dyn_sat(F_drive * share, cap_f);
  Fxr = dyn_sat(F_drive * (S(1) - share), cap_r);
  Dyf = dyn_lateral_cap(Fxf, cap_f);
  Dyr = dyn_lateral_cap(Fxr, cap_r);
}

// The lateral grip each axle has left: Dyf, Dyr.
template <class S>
CVC_HD inline void dyn_ph_grip(const dyn_bike_params &p, const dyn_state<S> &s, S &Dyf, S &Dyr) {
  S Fxf, Fxr;
  dyn_ph_axles(p, s, Fxf, Fxr, Dyf, Dyr);
}

// Everything the form is built from, at a state.
template <class S> struct dyn_ph_parts {
  S u, sg;        // the floored speed sqrt(vx^2 + eps^2) (the rear slip's), and sg = vx / u
  S uf;           // the front wheel's floored forward speed, in its frame
  S Fxf, Fxr;     // the axles' drive or brake forces: the inputs
  S Fyf, Fyr;     // the tires' lateral forces
  S wf, wr;       // the axles' sideways velocities in their wheels' frames: n_f^T v, n_r^T v
  S Sf, Sr;       // the secant stiffnesses through wf, wr = 0 (dyn_secant_w)
  S Dx;           // drag and rolling resistance on vx
  S cd;           // cos delta
  S nf[3], nr[3]; // each tire's lateral force on (vx, vy, r)
  S tf[3], tr[3]; // each axle's drive force on (vx, vy, r)
};

// dyn_ph_parts_at's lateral half: the slips, the speeds, the tires' lateral forces with the grip
// Dyf, Dyr their axles leave them, the secants, the resistances and the directions (all of q but
// Fxf, Fxr).
template <class S>
CVC_HD inline void dyn_ph_parts_lateral(const dyn_bike_params &p, const dyn_state<S> &s, S Dyf,
                                        S Dyr, dyn_ph_parts<S> &q) {
  const S lf = S(p.lf), lr = S(p.lr), Cp = S(p.C), Bf = S(p.B_f), Br = S(p.B_r), z = S(0);
  q.u = s_sqrt(s.vx * s.vx + S(p.eps) * S(p.eps));
  q.sg = s.vx / q.u;
  const S cd = s_cos(s.delta), sd = s_sin(s.delta);
  q.cd = cd;
  const S vf = s.vy + lf * s.r;
  q.wf = -s.vx * sd + vf * cd;
  q.wr = s.vy - lr * s.r;
  q.uf = s_sqrt((s.vx * cd + vf * sd) * (s.vx * cd + vf * sd) + S(p.eps) * S(p.eps));
  const S af = -s_atan2(q.wf, q.uf), ar = -s_atan2(q.wr, q.u);
  q.Fyf = dyn_lateral(af, Bf, Cp, Dyf);
  q.Fyr = dyn_lateral(ar, Br, Cp, Dyr);
  q.Sf = dyn_secant_w(q.Fyf, af, q.wf, z, Bf, Cp, Dyf, q.uf);
  q.Sr = dyn_secant_w(q.Fyr, ar, q.wr, z, Br, Cp, Dyr, q.u);
  q.Dx = S(p.drag) * q.u + S(p.c_rr) * S(p.m) * S(p.g) / q.u;
  q.nf[0] = -sd;
  q.nf[1] = cd;
  q.nf[2] = lf * cd;
  q.nr[0] = z;
  q.nr[1] = S(1);
  q.nr[2] = -lr;
  q.tf[0] = cd;
  q.tf[1] = sd;
  q.tf[2] = lf * sd;
  q.tr[0] = S(1);
  q.tr[1] = z;
  q.tr[2] = z;
}

template <class S>
CVC_HD inline void dyn_ph_parts_at(const dyn_bike_params &p, const dyn_state<S> &s,
                                   dyn_ph_parts<S> &q) {
  S Dyf, Dyr;
  dyn_ph_axles(p, s, q.Fxf, q.Fxr, Dyf, Dyr);
  dyn_ph_parts_lateral(p, s, Dyf, Dyr, q);
}

// R(v, delta), the dissipation: positive semi-definite at every slip.
template <class S>
CVC_HD inline void dyn_ph_R(const dyn_bike_params &p, const dyn_state<S> &s, S R[3][3]) {
  dyn_ph_parts<S> q;
  dyn_ph_parts_at(p, s, q);
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) {
      R[i][j] = q.Sf * (q.nf[i] * q.nf[j]) + q.Sr * (q.nr[i] * q.nr[j]);
      R[j][i] = R[i][j];
    }
  R[0][0] = R[0][0] + q.Dx;
}

// G(delta), the input map for u = (F_xf, F_xr): each axle's force along its wheel.
template <class S>
CVC_HD inline void dyn_ph_G(const dyn_bike_params &p, const dyn_state<S> &s, S G[3][2]) {
  dyn_ph_parts<S> q;
  dyn_ph_parts_at(p, s, q);
  for (int i = 0; i < 3; ++i) {
    G[i][0] = q.tf[i];
    G[i][1] = q.tr[i];
  }
}

// u = (F_xf, F_xr): the axles' drive or brake forces, from the actuator's a, within their grip.
template <class S>
CVC_HD inline void dyn_ph_input(const dyn_bike_params &p, const dyn_state<S> &s, S u[2]) {
  S Dyf, Dyr;
  dyn_ph_axles(p, s, u[0], u[1], Dyf, Dyr);
}

// f_eps, what the secant dissipation leaves of the tires' lateral forces: round-off, with both
// slips in their wheels' frames.
template <class S>
CVC_HD inline void dyn_ph_floor(const dyn_bike_params &p, const dyn_state<S> &s, S f[3]) {
  dyn_ph_parts<S> q;
  dyn_ph_parts_at(p, s, q);
  const S v[3] = {s.vx, s.vy, s.r};
  S nfv = S(0), nrv = S(0);
  for (int i = 0; i < 3; ++i) {
    nfv = nfv + q.nf[i] * v[i];
    nrv = nrv + q.nr[i] * v[i];
  }
  const S ef = q.Fyf + q.Sf * nfv, er = q.Fyr + q.Sr * nrv;
  for (int i = 0; i < 3; ++i)
    f[i] = q.nf[i] * ef + q.nr[i] * er;
}

// p' = (J - R) v + G u + f_eps - m g grade e_x at the state: dyn_plant's vector field.
template <class S>
CVC_HD inline void dyn_ph_rhs(const dyn_bike_params &p, const dyn_state<S> &s, S grade, S pdot[3]) {
  S pm[3], J[3][3], R[3][3], G[3][2], u[2], f[3];
  dyn_ph_momentum(p, s, pm);
  dyn_ph_J(pm, J);
  dyn_ph_R(p, s, R);
  dyn_ph_G(p, s, G);
  dyn_ph_input(p, s, u);
  dyn_ph_floor(p, s, f);
  const S v[3] = {s.vx, s.vy, s.r};
  for (int i = 0; i < 3; ++i) {
    S acc = G[i][0] * u[0] + G[i][1] * u[1] + f[i];
    for (int j = 0; j < 3; ++j)
      acc = acc + (J[i][j] - R[i][j]) * v[j];
    pdot[i] = acc;
  }
  pdot[0] = pdot[0] - S(p.m) * S(p.g) * grade;
}

// The power balance at a state: dH/dt = supplied - dissipated + floor + grade.
template <class S> struct dyn_ph_rates {
  S dissipated; // v^T R v >= 0: the tires' slip, drag and rolling resistance
  S supplied;   // y^T u, y = G^T v: the axles' forces at their wheels' ground speed
  S floor;      // v^T f_eps: what the secants leave, round-off
  S grade;      // - m g grade vx: gravity along the heading
};

template <class S>
CVC_HD inline dyn_ph_rates<S> dyn_ph_rates_at(const dyn_bike_params &p, const dyn_state<S> &s,
                                              S grade) {
  S R[3][3], G[3][2], u[2], f[3];
  dyn_ph_R(p, s, R);
  dyn_ph_G(p, s, G);
  dyn_ph_input(p, s, u);
  dyn_ph_floor(p, s, f);
  const S v[3] = {s.vx, s.vy, s.r};
  dyn_ph_rates<S> out;
  out.dissipated = S(0);
  out.supplied = S(0);
  out.floor = S(0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      out.dissipated = out.dissipated + v[i] * R[i][j] * v[j];
    out.supplied = out.supplied + v[i] * (G[i][0] * u[0] + G[i][1] * u[1]);
    out.floor = out.floor + v[i] * f[i];
  }
  out.grade = -S(p.m) * S(p.g) * grade * s.vx;
  return out;
}

// One step's energy books, in J: dH = supplied - dissipated + floor + grade + limiter + residual.
// The powers are the trapezoid between the step's start (with the inputs it applied, the
// actuators' outputs) and the twist it reached before its limiters; limiter is the kinetic energy
// the speed clamp and the low-speed blend then added (< 0: removed); residual is the integrator's
// error, second order in hdt per step.
template <class S> struct dyn_ph_books {
  S dH, dissipated, supplied, floor, grade, limiter, residual;
};

// dyn_plant's step with its books. The step is dyn_plant's, to the last bit.
template <class S>
CVC_HD inline void dyn_plant_books(const dyn_bike_params &p, const bike_veh &v,
                                   const dyn_state<S> &s, S delta_cmd, S a_cmd, S grade,
                                   dyn_state<S> &o, dyn_ph_books<S> &b) {
  dyn_step_free<S> seen;
  dyn_plant<S>(p, v, s, delta_cmd, a_cmd, grade, o, &seen);
  dyn_state<S> s0 = s, e = o;
  s0.delta = o.delta;
  s0.a = o.a;
  e.vx = seen.vx;
  e.vy = seen.vy;
  e.r = seen.r;
  const dyn_ph_rates<S> r0 = dyn_ph_rates_at(p, s0, grade), r1 = dyn_ph_rates_at(p, e, grade);
  const S half = S(0.5) * S(v.hdt);
  b.dissipated = half * (r0.dissipated + r1.dissipated);
  b.supplied = half * (r0.supplied + r1.supplied);
  b.floor = half * (r0.floor + r1.floor);
  b.grade = half * (r0.grade + r1.grade);
  const S h0 = dyn_ph_hamiltonian(p, s), he = dyn_ph_hamiltonian(p, e),
          h1 = dyn_ph_hamiltonian(p, o);
  b.dH = h1 - h0;
  b.limiter = h1 - he;
  b.residual = b.dH - (b.supplied - b.dissipated + b.floor + b.grade + b.limiter);
}

// ── The wheel-spin plant in port-Hamiltonian form ─────────────────────────────────────────
// dyn_spin_plant's vector field, as dyn_ph_*'s is dyn_plant's, with the wheels' angular momenta in
// the state: x = (p, L), L = (J_f om_f, J_r om_r), five coordinates, and
//
//   H = 1/2 v^T M v + 1/2 (J_f om_f^2 + J_r om_r^2),   dH/dx = (v, om)
//   x' = (J(p) - R) dH/dx + G u + f_eps - m g grade e_x,   u = (T_f, T_r),   y = G^T dH/dx = om
//
//   J     blockdiag(dyn_ph_J(p), 0): the structure couples nothing to the wheels.
//   R     blockdiag(R_body, 0) + K_f d_f d_f^T + K_r d_r d_r^T. R_body is dyn_ph_R's, each tire's
//         lateral grip what its longitudinal force leaves. d_f = (-t_f, r_w, 0) and
//         d_r = (-t_r, 0, r_w), so d^T dH/dx = r_w om - t^T v is the patch's slip velocity sigma:
//         the direction of the wheel's surface speed less the ground's. K = F_x / sigma (exact:
//         dyn_spin_ph_secant) is the tire's longitudinal secant through its root, >= 0 at any slip
//         because a Pacejka force (C_x < 2) has its slip's sign. So R is positive semi-definite at
//         every slip, rolling, spinning or locked, at any delta.
//   G     [e_4, e_5]: each axle's torque on its wheels. u = T_d - T_b sg(om), the drive's and the
//         brakes' (dyn_spin_torques), and y^T u = om^T u is their shaft power: the port is (wheel
//         torque, wheel speed). The brakes' part, - T_b om sg(om), is <= 0 at every state.
//         Steering does no work, as in dyn_ph_*.
//   f_eps the lateral secants' rounding, as dyn_ph_floor's; the longitudinal secant leaves none.
//
// So dH/dt = - v^T R_body v - sum K sigma^2 + om^T T_d - sum T_b om sg(om) + v^T f_eps
// - m g grade vx: the tires' lateral slip, drag and rolling resistance, the tires' longitudinal
// slip, the drive's shaft power, the brakes', the rounding and the grade. Passive with respect to
// the wheel torques at every slip. dyn_spin_plant_books books each step; dyn_spin_plant's
// arithmetic does not depend on these.

// A tire's longitudinal secant through its root, exactly: F / sigma, and D C B / u, its limit, at
// sigma = 0. (dyn_secant_x, the implicit step's, takes the slope where |B kappa| < 1e-4.)
template <class S> CVC_HD inline S dyn_spin_ph_secant(S F, S sigma, S B, S C, S D, S u) {
  if (s_val(sigma) == 0.0)
    return D * C * B / u;
  return F / sigma;
}

// Everything the wheel-spin form is built from, at a state.
template <class S> struct dyn_spin_ph_parts {
  dyn_ph_parts<S> b; // dyn_ph_parts_at's, with Fxf, Fxr the tires' longitudinal forces
  S gf, gr;          // each wheel's ground speed along it, t^T v
  S sf, sr;          // the patches' slip velocities, r_w om - t^T v
  S Kf, Kr;          // the longitudinal secants
  S Td[2], Tb[2];    // the drive's torques and the brakes' magnitudes (dyn_spin_torques)
  S sgf, sgr;        // the brakes' smoothed signs of the wheels' speeds
  S Tf, Tr;          // the inputs: T_d - T_b sg(om)
};

template <class S>
CVC_HD inline void dyn_spin_ph_parts_at(const dyn_bike_params &p, const dyn_spin_state<S> &s,
                                        dyn_spin_ph_parts<S> &q) {
  const dyn_state<S> &b = s.b;
  const S rw = S(p.r_w), Bx = S(p.B_x), Cx = S(p.C_x), eps2 = S(p.eps) * S(p.eps);
  S cap_f, cap_r;
  dyn_ph_caps(p, b, cap_f, cap_r);
  const S cd = s_cos(b.delta), sd = s_sin(b.delta);
  q.gf = b.vx * cd + (b.vy + S(p.lf) * b.r) * sd;
  q.gr = b.vx;
  const S uf = s_sqrt(q.gf * q.gf + eps2), ur = s_sqrt(q.gr * q.gr + eps2);
  q.sf = rw * s.om_f - q.gf;
  q.sr = rw * s.om_r - q.gr;
  q.b.Fxf = dyn_longitudinal(q.sf, uf, Bx, Cx, cap_f);
  q.b.Fxr = dyn_longitudinal(q.sr, ur, Bx, Cx, cap_r);
  q.Kf = dyn_spin_ph_secant(q.b.Fxf, q.sf, Bx, Cx, cap_f, uf);
  q.Kr = dyn_spin_ph_secant(q.b.Fxr, q.sr, Bx, Cx, cap_r, ur);
  dyn_ph_parts_lateral(p, b, dyn_lateral_cap(q.b.Fxf, cap_f), dyn_lateral_cap(q.b.Fxr, cap_r), q.b);
  dyn_spin_torques(p, b.a, q.Td, q.Tb);
  const S oe2 = eps2 / (rw * rw);
  q.sgf = s.om_f / s_sqrt(s.om_f * s.om_f + oe2);
  q.sgr = s.om_r / s_sqrt(s.om_r * s.om_r + oe2);
  q.Tf = q.Td[0] - q.Tb[0] * q.sgf;
  q.Tr = q.Td[1] - q.Tb[1] * q.sgr;
}

// The energy: the body's kinetic energy and the wheels' spin.
template <class S>
CVC_HD inline S dyn_spin_ph_hamiltonian(const dyn_bike_params &p, const dyn_spin_state<S> &s) {
  return dyn_ph_hamiltonian(p, s.b) +
         S(0.5) * (S(p.J_f) * s.om_f * s.om_f + S(p.J_r) * s.om_r * s.om_r);
}

// x = (M v, J_f om_f, J_r om_r).
template <class S>
CVC_HD inline void dyn_spin_ph_momentum(const dyn_bike_params &p, const dyn_spin_state<S> &s,
                                        S x[5]) {
  dyn_ph_momentum(p, s.b, x);
  x[3] = S(p.J_f) * s.om_f;
  x[4] = S(p.J_r) * s.om_r;
}

// J(p): the body's coadjoint structure, nothing on the wheels.
template <class S> CVC_HD inline void dyn_spin_ph_J(const S x[5], S J[5][5]) {
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j)
      J[i][j] = S(0);
  S Jb[3][3];
  dyn_ph_J(x, Jb);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      J[i][j] = Jb[i][j];
}

// R, the dissipation: positive semi-definite at every slip.
template <class S>
CVC_HD inline void dyn_spin_ph_R(const dyn_bike_params &p, const dyn_spin_state<S> &s, S R[5][5]) {
  dyn_spin_ph_parts<S> q;
  dyn_spin_ph_parts_at(p, s, q);
  const S rw = S(p.r_w), z = S(0);
  const S df[5] = {-q.b.tf[0], -q.b.tf[1], -q.b.tf[2], rw, z};
  const S dr[5] = {-q.b.tr[0], -q.b.tr[1], -q.b.tr[2], z, rw};
  for (int i = 0; i < 5; ++i)
    for (int j = i; j < 5; ++j) {
      S body = z;
      if (i < 3 && j < 3)
        body = q.b.Sf * (q.b.nf[i] * q.b.nf[j]) + q.b.Sr * (q.b.nr[i] * q.b.nr[j]);
      R[i][j] = body + q.Kf * (df[i] * df[j]) + q.Kr * (dr[i] * dr[j]);
      R[j][i] = R[i][j];
    }
  R[0][0] = R[0][0] + q.b.Dx;
}

// G, the input map for u = (T_f, T_r): each axle's torque on its wheels' momentum.
template <class S> CVC_HD inline void dyn_spin_ph_G(S G[5][2]) {
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 2; ++j)
      G[i][j] = S(i == 3 + j ? 1 : 0);
}

// u = (T_f, T_r): the drive's torques less the brakes'.
template <class S>
CVC_HD inline void dyn_spin_ph_input(const dyn_bike_params &p, const dyn_spin_state<S> &s, S u[2]) {
  dyn_spin_ph_parts<S> q;
  dyn_spin_ph_parts_at(p, s, q);
  u[0] = q.Tf;
  u[1] = q.Tr;
}

// f_eps: what the lateral secants leave of the tires' lateral forces (round-off).
template <class S>
CVC_HD inline void dyn_spin_ph_floor(const dyn_bike_params &p, const dyn_spin_state<S> &s, S f[5]) {
  dyn_spin_ph_parts<S> q;
  dyn_spin_ph_parts_at(p, s, q);
  const S ef = q.b.Fyf + q.b.Sf * q.b.wf, er = q.b.Fyr + q.b.Sr * q.b.wr;
  for (int i = 0; i < 3; ++i)
    f[i] = q.b.nf[i] * ef + q.b.nr[i] * er;
  f[3] = S(0);
  f[4] = S(0);
}

// x' = (J - R) dH/dx + G u + f_eps - m g grade e_x at the state: dyn_spin_plant's vector field.
template <class S>
CVC_HD inline void dyn_spin_ph_rhs(const dyn_bike_params &p, const dyn_spin_state<S> &s, S grade,
                                   S xdot[5]) {
  S x[5], J[5][5], R[5][5], G[5][2], u[2], f[5];
  dyn_spin_ph_momentum(p, s, x);
  dyn_spin_ph_J(x, J);
  dyn_spin_ph_R(p, s, R);
  dyn_spin_ph_G(G);
  dyn_spin_ph_input(p, s, u);
  dyn_spin_ph_floor(p, s, f);
  const S e[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
  for (int i = 0; i < 5; ++i) {
    S acc = G[i][0] * u[0] + G[i][1] * u[1] + f[i];
    for (int j = 0; j < 5; ++j)
      acc = acc + (J[i][j] - R[i][j]) * e[j];
    xdot[i] = acc;
  }
  xdot[0] = xdot[0] - S(p.m) * S(p.g) * grade;
}

// Q, every force on x but the structure's, at the state: -R dH/dx + G u + f_eps - m g grade e_x
// on (vx, vy, r, om_f, om_r)'s momenta, so dyn_spin_ph_rhs is J dH/dx + Q. A discrete-gradient
// step takes J and Q at its midpoint; dH/dx^T Q there is dyn_spin_ph_rates_at's balance, supplied -
// braked - dissipated - slip + floor + grade, so its books close with those rates.
template <class S>
CVC_HD inline void dyn_spin_ph_forces_at(const dyn_bike_params &p, const dyn_spin_state<S> &s,
                                         S grade, S Q[5]) {
  S R[5][5], G[5][2], u[2], f[5];
  dyn_spin_ph_R(p, s, R);
  dyn_spin_ph_G(G);
  dyn_spin_ph_input(p, s, u);
  dyn_spin_ph_floor(p, s, f);
  const S e[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
  for (int i = 0; i < 5; ++i) {
    S acc = G[i][0] * u[0] + G[i][1] * u[1] + f[i];
    for (int j = 0; j < 5; ++j)
      acc = acc - R[i][j] * e[j];
    Q[i] = acc;
  }
  Q[0] = Q[0] - S(p.m) * S(p.g) * grade;
}

// The power balance at a state: dH/dt = supplied - braked - dissipated - slip + floor + grade.
template <class S> struct dyn_spin_ph_rates {
  S dissipated; // v^T R_body v >= 0: the tires' lateral slip, drag and rolling resistance
  S slip;       // sum K sigma^2 >= 0: the tires' longitudinal slip
  S supplied;   // om^T T_d: the drive's shaft power
  S braked;     // sum T_b om sg(om) >= 0: the brakes'
  S floor;      // v^T f_eps: what the lateral secants leave, round-off
  S grade;      // - m g grade vx
};

template <class S>
CVC_HD inline dyn_spin_ph_rates<S> dyn_spin_ph_rates_at(const dyn_bike_params &p,
                                                        const dyn_spin_state<S> &s, S grade) {
  dyn_spin_ph_parts<S> q;
  dyn_spin_ph_parts_at(p, s, q);
  const dyn_ph_parts<S> &b = q.b;
  dyn_spin_ph_rates<S> out;
  out.dissipated = b.Sf * b.wf * b.wf + b.Sr * b.wr * b.wr + b.Dx * s.b.vx * s.b.vx;
  out.slip = q.Kf * q.sf * q.sf + q.Kr * q.sr * q.sr;
  out.supplied = s.om_f * q.Td[0] + s.om_r * q.Td[1];
  out.braked = q.Tb[0] * s.om_f * q.sgf + q.Tb[1] * s.om_r * q.sgr;
  // v^T f_eps: n_f^T v = wf, n_r^T v = wr.
  out.floor = b.wf * (b.Fyf + b.Sf * b.wf) + b.wr * (b.Fyr + b.Sr * b.wr);
  out.grade = -S(p.m) * S(p.g) * grade * s.b.vx;
  return out;
}

// One step's energy books, in J: dH = supplied - braked - dissipated - slip + floor + grade +
// limiter + residual, as dyn_ph_books with the wheels: the trapezoid between the step's start (the
// actuators' outputs) and the state it reached before its limiters, the wheels' new speeds
// included; limiter the kinetic energy the speed clamp and the low-speed blend changed (they act
// on the body alone); residual the integrator's error, second order in hdt per step.
template <class S> struct dyn_spin_ph_books {
  S dH, dissipated, slip, supplied, braked, floor, grade, limiter, residual;
};

// dyn_spin_plant's step with its books. The step is dyn_spin_plant's, to the last bit.
template <class S>
CVC_HD inline void dyn_spin_plant_books(const dyn_bike_params &p, const bike_veh &v,
                                        const dyn_spin_state<S> &s, S delta_cmd, S a_cmd, S grade,
                                        dyn_spin_state<S> &o, dyn_spin_ph_books<S> &b) {
  dyn_step_free<S> seen;
  dyn_spin_plant<S>(p, v, s, delta_cmd, a_cmd, grade, o, &seen);
  dyn_spin_state<S> s0 = s, e = o;
  s0.b.delta = o.b.delta;
  s0.b.a = o.b.a;
  e.b.vx = seen.vx;
  e.b.vy = seen.vy;
  e.b.r = seen.r;
  const dyn_spin_ph_rates<S> r0 = dyn_spin_ph_rates_at(p, s0, grade),
                             r1 = dyn_spin_ph_rates_at(p, e, grade);
  const S half = S(0.5) * S(v.hdt);
  b.dissipated = half * (r0.dissipated + r1.dissipated);
  b.slip = half * (r0.slip + r1.slip);
  b.supplied = half * (r0.supplied + r1.supplied);
  b.braked = half * (r0.braked + r1.braked);
  b.floor = half * (r0.floor + r1.floor);
  b.grade = half * (r0.grade + r1.grade);
  const S h0 = dyn_spin_ph_hamiltonian(p, s), he = dyn_spin_ph_hamiltonian(p, e),
          h1 = dyn_spin_ph_hamiltonian(p, o);
  b.dH = h1 - h0;
  b.limiter = h1 - he;
  b.residual =
      b.dH - (b.supplied - b.braked - b.dissipated - b.slip + b.floor + b.grade + b.limiter);
}

// The tangent dissipation of dyn_plant's Pacejka tires at the state: Dt = -d(Q_y, Q_r)/d(vy, r)
// with Q the tires' lateral force and yaw moment, = k_f a a^T + k_r b b^T where k is each tire's
// dF/dalpha times its slip's sensitivity to (vy, r) along a or b (the front's turned by delta, and
// its force with it). With a and b independent, Dt is positive semi-definite exactly when both k
// are >= 0, i.e. both tires are below their peak (the front's slip sensitivity stays positive
// while its sideways velocity is under about cot delta times its forward speed); kf and kr are
// returned with it. alpha_f and alpha_r are the slips (dyn_plant's, in the wheels' frames).
template <class S>
CVC_HD inline void dyn_ph_tangent_D(const dyn_bike_params &p, const dyn_state<S> &s, S Dt[2][2],
                                    S &kf, S &kr, S &alpha_f, S &alpha_r) {
  S Dyf, Dyr;
  dyn_ph_grip(p, s, Dyf, Dyr);
  const S lf = S(p.lf), lr = S(p.lr), C = S(p.C), Bf = S(p.B_f), Br = S(p.B_r);
  const S eps2 = S(p.eps) * S(p.eps), u = s_sqrt(s.vx * s.vx + eps2);
  const S cd = s_cos(s.delta), sd = s_sin(s.delta), vf = s.vy + lf * s.r;
  const S wf = -s.vx * sd + vf * cd, wr = s.vy - lr * s.r;
  const S uf = s_sqrt((s.vx * cd + vf * sd) * (s.vx * cd + vf * sd) + eps2);
  alpha_f = -s_atan2(wf, uf);
  alpha_r = -s_atan2(wr, u);
  // dF/dalpha of D sin(C atan(B alpha)).
  const S bf = Bf * alpha_f, br = Br * alpha_r;
  const S dFf = Dyf * C * Bf * s_cos(C * s_atan(bf)) / (S(1) + bf * bf);
  const S dFr = Dyr * C * Br * s_cos(C * s_atan(br)) / (S(1) + br * br);
  // The rear's slip moves with its w alone: dalpha/dw = -u / (u^2 + w^2). The front's w and its
  // forward speed lon (floored to uf) both move with (vy, r) along (1, lf), as cos delta and sin
  // delta, and its force reaches them through cos delta.
  const S lon = s.vx * cd + vf * sd;
  kf = dFf * cd * (cd * uf - sd * wf * lon / uf) / (uf * uf + wf * wf);
  kr = dFr * u / (u * u + wr * wr);
  Dt[0][0] = kf + kr;
  Dt[0][1] = kf * lf - kr * lr;
  Dt[1][0] = Dt[0][1];
  Dt[1][1] = kf * lf * lf + kr * lr * lr;
}

// ── The vehicle under the trained controller ──────────────────────────────────────────────

// One step: bike_control's commands from the rear axle's pose and the speed vx, through the
// dynamic plant.
CVC_HD inline void dyn_bike_step(const field &f, const dyn_state<float> &s, float gx, float gy,
                                 float al, float be, float ga, const bike_veh &v,
                                 const dyn_bike_params &p, float grade, dyn_state<float> &o,
                                 const ext_in *ext = nullptr) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd, nullptr, ext);
  dyn_plant<float>(p, v, s, delta_cmd, a_cmd, grade, o);
}

// Backward of dyn_bike_step: gradients on the next state -> on the state and (al, be, ga).
CVC_HD inline void dyn_bike_step_bwd(const field &f, const dyn_state<float> &s, float gx, float gy,
                                     float al, float be, float ga, const bike_veh &v,
                                     const dyn_bike_params &p, float grade,
                                     const dyn_state<float> &g_o, dyn_state<float> &g_s, float &gal,
                                     float &gbe, float &gga, const ext_in *ext = nullptr,
                                     float *g_fext = nullptr) {
  float delta_cmd, a_cmd;
  bike_control(f, s.x, s.y, s.psi, s.vx, gx, gy, al, be, ga, v, delta_cmd, a_cmd, nullptr, ext);
  float g_dc, g_ac;
  dyn_plant_vjp<float>(p, v, s, delta_cmd, a_cmd, grade, g_o, g_s, g_dc, g_ac);
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

#endif // __CVC_NAV_DETAIL_DYN_BIKE_H__
