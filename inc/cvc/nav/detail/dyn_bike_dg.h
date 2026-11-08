/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// dyn_bike_dg.h: a step of the dynamic single-track vehicle (dyn_bike.h) whose energy books close
// exactly, the discrete gradient of its port-Hamiltonian form. Opt-in: dyn_plant, and everything
// that steps with it, is unchanged.
//
// dyn_plant's step is first order (implicit Euler on (vy, r), a predicted then corrected vx), so
// dyn_plant_books leaves its error, second order per step, in the residual. Here H = 1/2 v^T M v is
// quadratic in the twist v = (vx, vy, r), so its discrete gradient is the midpoint twist
// vbar = (v + v1) / 2, and the step solves
//
//   M (v1 - v) / hdt = J(M vbar) vbar + Q(vbar),   Q = -R vbar + G u + f_eps - m g grade e_x,
//
// with everything evaluated at the midpoint state: the twist vbar and the actuators' outputs at the
// step's midpoint, (delta + delta1) / 2 and (a + a1) / 2, held over the step (dyn_plant holds their
// step-end values, which is first order while they move). Then H(v1) - H(v) = vbar^T M (v1 - v)
// exactly, J is skew, and
//
//   dH = hdt [-vbar^T R vbar + vbar^T G u + vbar^T f_eps - m g grade vbar_x]
//
// line for line: the books close to the solver's tolerance, not to the integrator's order. It is
// the implicit midpoint rule on the twist: second order and A-stable, and since R is positive
// semi-definite at every slip (dyn_bike.h), the solved twist never gains energy the ports and the
// grade did not supply, at any hdt and any tire stiffness. It is not L-stable: a tire much stiffer
// than the step resolves (S hdt / m >> 1, at a crawl) swings its slip past its root and back,
// decaying, where dyn_plant's implicit Euler lands on the root. The low-speed blend takes over at
// those speeds; DynBikeDG.StiffTires measures the rest.
//
// The step, given (delta_cmd, a_cmd) and the grade (sin of the slope along the heading):
//   1. Actuators: dyn_plant's step 1, to the bit (delta1, a1, the step's result).
//   2. The twist: Newton on F(v1) = M (v1 - v) - hdt (J vbar + Q(vbar)) from v1 = v, its Jacobian
//      M - hdt/2 d(J vbar + Q)/d(vbar) by forward-mode duals, with a backtracking line search on
//      |M^-1 F|, until the update is under `tol` of the twist's scale.
//   3. Limiters: dyn_plant's speed clamp to [sp_min, vmax] and low-speed blend toward the kinematic
//      bicycle, on the solved twist; dyn_plant_dg_books charges them to its limiter line.
//   4. Pose: the midpoint rule, psi1 = psi + hdt rbar, the rear axle moving with its velocity
//      averaged over the step, (vx, vy - lr r), at the midpoint heading.
//
// S is float or double (the Jacobian is dual<S>). Wheel spin keeps the form: dyn_spin_plant_dg,
// below, adds the wheels' speeds to the twist as further momenta of a quadratic H, takes Q from
// the plant's hook dyn_spin_ph_forces_at, and puts the wheels' rows at the step's end by default,
// since their slip is far stiffer than the tires' sideways slip.

#ifndef __CVC_NAV_DETAIL_DYN_BIKE_DG_H__
#define __CVC_NAV_DETAIL_DYN_BIKE_DG_H__

#include <cvc/nav/detail/dyn_bike.h>

namespace cvc {
namespace nav {
namespace diff {

// The solve's controls.
struct dyn_dg_options {
  int max_iters = 20; // Newton steps
  // Stop once an update is under tol of the twist's scale, 1 + |vx| + |vy| + L |r| (length / s).
  // 0 picks 1e-13 for a double plant and 1e-6 for a float one.
  double tol = 0.0;
};

// What one solve did.
struct dyn_dg_solve {
  int iters = 0;          // Newton steps taken
  double update = 0.0;    // the last update, relative to the twist's scale
  double residual = 0.0;  // |M^-1 F(v1)| at the end, length / s
  bool converged = false; // the update fell under tol
  int secant = 0;         // Newton steps that fell back on the secant step (dyn_spin_plant_dg)
};

// J(M v) v + Q at a state: the momenta's rate, dyn_ph_rhs's, from one dyn_ph_parts_at. With both
// slips in their wheels' frames, -R v + f_eps is the tires' forces themselves, n_f F_yf + n_r F_yr,
// less drag and rolling resistance on vx.
template <class S>
CVC_HD inline void dyn_dg_rate(const dyn_bike_params &p, const dyn_state<S> &s, S grade, S f[3]) {
  dyn_ph_parts<S> q;
  dyn_ph_parts_at(p, s, q);
  const S m = S(p.m);
  for (int i = 0; i < 3; ++i)
    f[i] = q.nf[i] * q.Fyf + q.nr[i] * q.Fyr + q.tf[i] * q.Fxf + q.tr[i] * q.Fxr;
  f[0] = f[0] + m * s.vy * s.r - q.Dx * s.vx - m * S(p.g) * grade;
  f[1] = f[1] - m * s.vx * s.r;
}

// F(v1) = M (v1 - v0) - hdt (J vbar + Q(vbar)) at the midpoint of v0 and v1; `mid` carries the
// pose, delta1 and a1.
template <class S>
CVC_HD inline void dyn_dg_residual(const dyn_bike_params &p, dyn_state<S> mid, const S v0[3],
                                   const S v1[3], S hdt, S grade, S F[3]) {
  mid.vx = S(0.5) * (v0[0] + v1[0]);
  mid.vy = S(0.5) * (v0[1] + v1[1]);
  mid.r = S(0.5) * (v0[2] + v1[2]);
  S f[3];
  dyn_dg_rate(p, mid, grade, f);
  const S M[3] = {S(p.m), S(p.m), S(p.Iz)};
  for (int i = 0; i < 3; ++i)
    F[i] = M[i] * (v1[i] - v0[i]) - hdt * f[i];
}

// |M^-1 F|, in length / s (the yaw row times L).
template <class S> CVC_HD inline S dyn_dg_norm(const dyn_bike_params &p, const S F[3]) {
  const S L = S(p.lf) + S(p.lr);
  const S a = F[0] / S(p.m), b = F[1] / S(p.m), c = L * F[2] / S(p.Iz);
  return s_sqrt(a * a + b * b + c * c);
}

// dyn_plant's step 1, to the bit: the actuators' outputs at the step's end.
template <class S>
CVC_HD inline void dyn_dg_actuators(const dyn_bike_params &p, S hdt, const dyn_state<S> &s,
                                    S delta_cmd, S a_cmd, S &delta1, S &a1) {
  const S one = S(1);
  const S k_steer = p.tau_steer > 0.0f ? one - s_exp(-hdt / S(p.tau_steer)) : one;
  S step = (delta_cmd - s.delta) * k_steer;
  if (p.steer_rate > 0.0f) {
    const S R = S(p.steer_rate) * hdt;
    step = R * s_tanh(step / R);
  }
  delta1 = s.delta + step;
  const S k_drive = p.tau_drive > 0.0f ? one - s_exp(-hdt / S(p.tau_drive)) : one;
  a1 = s.a + (a_cmd - s.a) * k_drive;
}

// Steps 3 and 4 on a solved twist v1: dyn_plant's speed clamp to [sp_min, vmax] and low-speed
// blend toward the kinematic bicycle, then the pose by the midpoint rule.
template <class S>
CVC_HD inline void dyn_dg_finish(const dyn_bike_params &p, const bike_veh &v, const dyn_state<S> &s,
                                 S delta1, S a1, const S v1[3], dyn_state<S> &o) {
  const S hdt = S(v.hdt), one = S(1), half = S(0.5);
  const S lr = S(p.lr), L = S(p.lf) + lr;
  const S sp_min = S(v.allow_reverse ? -0.25f * v.vmax : 0.0f), vmax = S(v.vmax);
  const S vx1 = dyn_clamp(v1[0], sp_min, vmax);
  S vy1 = v1[1], r1 = v1[2];
  if (p.v_blend > 0.0f) {
    const S eps2 = S(p.eps) * S(p.eps);
    const S w = one / (one + s_exp((s_sqrt(vx1 * vx1 + eps2) - S(p.v_blend)) / S(p.v_width)));
    const S r_k = vx1 * s_tan(delta1) / L, vy_k = lr * r_k;
    vy1 = vy1 + w * (vy_k - vy1);
    r1 = r1 + w * (r_k - r1);
  }
  const S rbar = half * (s.r + r1);
  const S psim = s.psi + half * hdt * rbar;
  const S ux = half * (s.vx + vx1), uy = half * ((s.vy - lr * s.r) + (vy1 - lr * r1));
  const S cm = s_cos(psim), sm = s_sin(psim);
  o.x = s.x + hdt * (ux * cm - uy * sm);
  o.y = s.y + hdt * (ux * sm + uy * cm);
  o.psi = s.psi + hdt * rbar;
  o.vx = vx1;
  o.vy = vy1;
  o.r = r1;
  o.delta = delta1;
  o.a = a1;
}

// One discrete-gradient step: dyn_plant's signature, plus the solve's report and controls.
template <class S>
CVC_HD inline void dyn_plant_dg(const dyn_bike_params &p, const bike_veh &v, const dyn_state<S> &s,
                                S delta_cmd, S a_cmd, S grade, dyn_state<S> &o,
                                dyn_step_free<S> *seen = nullptr, dyn_dg_solve *rep = nullptr,
                                const dyn_dg_options &opt = dyn_dg_options()) {
  typedef dual<S> D;
  const S hdt = S(v.hdt), one = S(1), half = S(0.5);
  const S L = S(p.lf) + S(p.lr);

  // 1. Actuators: dyn_plant's step 1.
  S delta1, a1;
  dyn_dg_actuators(p, hdt, s, delta_cmd, a_cmd, delta1, a1);

  // 2. The twist: Newton from v1 = v0, the actuators' outputs at the step's midpoint.
  dyn_state<S> mid = s;
  mid.delta = half * (s.delta + delta1);
  mid.a = half * (s.a + a1);
  const S v0[3] = {s.vx, s.vy, s.r};
  S v1[3] = {s.vx, s.vy, s.r}, F[3];
  const double tol = opt.tol > 0.0 ? opt.tol : (sizeof(S) == sizeof(float) ? 1e-6 : 1e-13);
  const double scale = 1.0 + (double)s_val(s.vx < S(0) ? -s.vx : s.vx) +
                       (double)s_val(s.vy < S(0) ? -s.vy : s.vy) +
                       (double)s_val(L * (s.r < S(0) ? -s.r : s.r));
  dyn_dg_residual(p, mid, v0, v1, hdt, grade, F);
  S fnorm = dyn_dg_norm(p, F);
  const S M[3] = {S(p.m), S(p.m), S(p.Iz)};
  dyn_dg_solve sol;
  for (int it = 0; it < opt.max_iters; ++it) {
    // A = dF/dv1 = M - hdt/2 d(rate)/d(vbar), a column per direction.
    S A[3][3];
    for (int j = 0; j < 3; ++j) {
      dyn_state<D> md;
      for (int k = 0; k < dyn_state<S>::size; ++k)
        md[k] = D(mid[k]);
      md.vx = D(half * (v0[0] + v1[0]), j == 0 ? one : S(0));
      md.vy = D(half * (v0[1] + v1[1]), j == 1 ? one : S(0));
      md.r = D(half * (v0[2] + v1[2]), j == 2 ? one : S(0));
      D f[3];
      dyn_dg_rate<D>(p, md, D(grade), f);
      for (int i = 0; i < 3; ++i)
        A[i][j] = (i == j ? M[i] : S(0)) - half * hdt * f[i].d;
    }
    // d = -A^-1 F, by Cramer's rule.
    const S c00 = A[1][1] * A[2][2] - A[1][2] * A[2][1];
    const S c01 = A[1][2] * A[2][0] - A[1][0] * A[2][2];
    const S c02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
    const S det = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02;
    if (!(s_val(det) != 0.0))
      break;
    const S inv[3][3] = {{c00 / det, (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det,
                          (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det},
                         {c01 / det, (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det,
                          (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det},
                         {c02 / det, (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det,
                          (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det}};
    S d[3];
    for (int i = 0; i < 3; ++i)
      d[i] = -(inv[i][0] * F[0] + inv[i][1] * F[1] + inv[i][2] * F[2]);
    // Backtrack until |M^-1 F| falls (a full step near the root).
    S t = one, vt[3], Ft[3], ft = fnorm;
    bool took = false;
    for (int ls = 0; ls < 12; ++ls) {
      for (int i = 0; i < 3; ++i)
        vt[i] = v1[i] + t * d[i];
      dyn_dg_residual(p, mid, v0, vt, hdt, grade, Ft);
      ft = dyn_dg_norm(p, Ft);
      if (s_val(ft) < s_val(fnorm)) {
        took = true;
        break;
      }
      t = half * t;
    }
    const double upd =
        (double)s_val(t) *
        ((double)s_val(d[0] < S(0) ? -d[0] : d[0]) + (double)s_val(d[1] < S(0) ? -d[1] : d[1]) +
         (double)s_val(L * (d[2] < S(0) ? -d[2] : d[2]))) /
        scale;
    sol.iters = it + 1;
    sol.update = upd;
    if (!took) { // at round-off: no step lowers the residual
      sol.converged = upd < 1e3 * tol;
      break;
    }
    for (int i = 0; i < 3; ++i) {
      v1[i] = vt[i];
      F[i] = Ft[i];
    }
    fnorm = ft;
    if (upd < tol) {
      sol.converged = true;
      break;
    }
  }
  sol.residual = (double)s_val(fnorm);
  if (rep)
    *rep = sol;
  if (seen) {
    seen->vx = v1[0];
    seen->vy = v1[1];
    seen->r = v1[2];
  }

  // 3 and 4. Limiters and pose.
  dyn_dg_finish(p, v, s, delta1, a1, v1, o);
}

// dyn_plant_dg's step with its books, in J: dH = supplied - dissipated + floor + grade + limiter +
// residual, as dyn_plant_books keeps them. The powers are the port-Hamiltonian rates at the step's
// midpoint state (dyn_ph_rates_at), times hdt, which is exactly what the solved twist's energy
// changed by; limiter is what the clamp and the blend then changed. So the residual is the solve's:
// vbar^T F(v1) and round-off. The step is dyn_plant_dg's, to the last bit.
template <class S>
CVC_HD inline void
dyn_plant_dg_books(const dyn_bike_params &p, const bike_veh &v, const dyn_state<S> &s, S delta_cmd,
                   S a_cmd, S grade, dyn_state<S> &o, dyn_ph_books<S> &b,
                   dyn_dg_solve *rep = nullptr, const dyn_dg_options &opt = dyn_dg_options()) {
  dyn_step_free<S> seen;
  dyn_plant_dg<S>(p, v, s, delta_cmd, a_cmd, grade, o, &seen, rep, opt);
  dyn_state<S> mid = s, e = o;
  mid.delta = S(0.5) * (s.delta + o.delta);
  mid.a = S(0.5) * (s.a + o.a);
  mid.vx = S(0.5) * (s.vx + seen.vx);
  mid.vy = S(0.5) * (s.vy + seen.vy);
  mid.r = S(0.5) * (s.r + seen.r);
  e.vx = seen.vx;
  e.vy = seen.vy;
  e.r = seen.r;
  const dyn_ph_rates<S> r = dyn_ph_rates_at(p, mid, grade);
  const S hdt = S(v.hdt);
  b.dissipated = hdt * r.dissipated;
  b.supplied = hdt * r.supplied;
  b.floor = hdt * r.floor;
  b.grade = hdt * r.grade;
  const S h0 = dyn_ph_hamiltonian(p, s), he = dyn_ph_hamiltonian(p, e),
          h1 = dyn_ph_hamiltonian(p, o);
  b.dH = h1 - h0;
  b.limiter = h1 - he;
  b.residual = b.dH - (b.supplied - b.dissipated + b.floor + b.grade + b.limiter);
}

// ── With the wheels spinning: dyn_spin_plant_dg ───────────────────────────────────────────
// dyn_spin_plant's state adds each axle's wheel speed, and its H adds their spin,
// 1/2 (J_f om_f^2 + J_r om_r^2): still quadratic, now in e = (vx, vy, r, om_f, om_r) with
// M = diag(m, m, Iz, J_f, J_r). The step solves
//
//   M (e1 - e) / h = J(M e_th) e_th + Q(e_th),   e_th = e + Theta (e1 - e),
//
// J and Q at e_th with the actuators at the step's midpoint: Q from dyn_spin_ph_forces_at, the hook
// the plant exports for this step, and J from dyn_spin_ph_J. Theta = diag(1/2, 1/2, 1/2, th, th):
// the body's rows take the midpoint, and the wheels' take theta_wheels. Then, J being skew,
//
//   H(e1) - H(e) = h e_th^T Q(e_th) - (th - 1/2) (J_f dom_f^2 + J_r dom_r^2)
//
// exactly. e_th^T Q is dyn_spin_ph_rates_at's balance, supplied - braked - dissipated - slip +
// floor + grade (plus the wrench's power), so the books close to the solver's tolerance with one
// more line, `scheme`, the second term. It is >= 0 for th >= 1/2 and zero at th = 1/2.
//
// The wheels' slip is stiff: a tire's longitudinal secant K against the wheel's spin inertia
// gives the slip a rate lambda = K (r_w^2 / J + 1 / m) of 1e3 to 1e4 / s on the Humvee, so h lambda
// is 60 to 600 at nav's 0.06 s step, and far more at a crawl. The midpoint rule (th = 1/2) is
// A-stable but not L-stable: it carries that slip past its root and back nearly undamped,
// amplification (1 - h lambda / 2) / (1 + h lambda / 2), close to -1. With th = 1 on the wheels
// alone, the slip's amplification is about (1 - b / 2) / (1 + a + b / 2), where a = h K r_w^2 / J
// and b = h K / m. On the Humvee a is about 120 b, so a disturbed slip settles in a step or two,
// about as dyn_spin_plant's does (DynSpinDG.StiffWheelSlip counts the swings). The price is first
// order in the wheels' speeds, and the books carry the damping on `scheme`. That is the default.
// `substeps` splits hdt into equal solves; the actuators move linearly across them, each substep
// taking its own midpoint. At th = 1/2 they tame the swing only once h lambda is near 2 (25
// substeps of 0.06 s on the Humvee); at th = 1 they are rarely worth their cost. The limiters and
// the pose act once, on the whole step, as in dyn_plant_dg.
//
// Newton's Jacobian is by forward-mode duals through one tire evaluation per direction. Past a
// tire's peak with a stiff slip it can turn indefinite (a crawl at 0.06 s), so a Newton direction
// that two halvings cannot make descend gives way to a secant step that always does; see the loop.
//
// The ground form (dyn_plant_ground's slope across the heading, gravity in H) is not in this step.

// The spin step's controls. A stiff slip past a tire's peak can take more Newton steps than the
// body alone, so the cap is 40.
struct dyn_spin_dg_options : dyn_dg_options {
  int substeps = 1;          // equal solves per hdt
  double theta_wheels = 1.0; // where the wheels' rows take their rate: 0.5 the midpoint, 1 the end
  dyn_spin_dg_options() { max_iters = 40; }
};

// J(M e) e + Q(e) at a state: dyn_spin_ph_rhs's, with `wrench` (the external port's (F_x, F_y,
// M_z) on the body) when given.
template <class S>
CVC_HD inline void dyn_spin_dg_rate(const dyn_bike_params &p, const dyn_spin_state<S> &s, S grade,
                                    const S *wrench, S f[5]) {
  S x[5], J[5][5], Q[5];
  dyn_spin_ph_momentum(p, s, x);
  dyn_spin_ph_J(x, J);
  dyn_spin_ph_forces_at(p, s, grade, Q, wrench);
  const S e[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
  for (int i = 0; i < 5; ++i) {
    S acc = Q[i];
    for (int j = 0; j < 3; ++j) // the structure couples nothing to the wheels
      acc = acc + J[i][j] * e[j];
    f[i] = acc;
  }
}

// The same rate from one dyn_spin_ph_parts_at, a third of the work. With each slip in its wheel's
// frame, -R e + f_eps is the tires' forces themselves: n F_y on the body, and t F_x on the body
// and -r_w F_x on its wheel (K sigma is F_x), less drag and rolling resistance on vx. The step's
// Newton Jacobian takes it; its residual and its books stay on the hook, so the solution is the
// hook's whatever this one says (DynSpinDG.TheRateIsThePortHamiltonianField holds them equal).
template <class S>
CVC_HD inline void dyn_spin_dg_rate_one(const dyn_bike_params &p, const dyn_spin_state<S> &s,
                                        S grade, const S *wrench, S f[5]) {
  dyn_spin_ph_parts<S> q;
  dyn_spin_ph_parts_at(p, s, q);
  const dyn_ph_parts<S> &b = q.b;
  const S m = S(p.m), rw = S(p.r_w);
  for (int i = 0; i < 3; ++i)
    f[i] = b.nf[i] * b.Fyf + b.nr[i] * b.Fyr + b.tf[i] * b.Fxf + b.tr[i] * b.Fxr;
  f[0] = f[0] + m * s.b.vy * s.b.r - b.Dx * s.b.vx - m * S(p.g) * grade;
  f[1] = f[1] - m * s.b.vx * s.b.r;
  f[3] = q.Tf - rw * b.Fxf;
  f[4] = q.Tr - rw * b.Fxr;
  if (wrench)
    for (int i = 0; i < 3; ++i)
      f[i] = f[i] + wrench[i];
}

template <class S> CVC_HD inline void dyn_spin_dg_set(dyn_spin_state<S> &s, const S e[5]) {
  s.b.vx = e[0];
  s.b.vy = e[1];
  s.b.r = e[2];
  s.om_f = e[3];
  s.om_r = e[4];
}

// |M^-1 F| in length / s: the yaw row times L, the wheels' times r_w.
template <class S> CVC_HD inline S dyn_spin_dg_norm(const dyn_bike_params &p, const S F[5]) {
  const S L = S(p.lf) + S(p.lr), rw = S(p.r_w);
  const S c[5] = {F[0] / S(p.m), F[1] / S(p.m), L * F[2] / S(p.Iz), rw * F[3] / S(p.J_f),
                  rw * F[4] / S(p.J_r)};
  S acc = S(0);
  for (int i = 0; i < 5; ++i)
    acc = acc + c[i] * c[i];
  return s_sqrt(acc);
}

template <class S> CVC_HD inline double dyn_dg_abs(S x) {
  const double d = (double)s_val(x);
  return d < 0.0 ? -d : d;
}

// x = A^-1 b by Gaussian elimination with partial pivoting; false if A is singular.
template <class S> CVC_HD inline bool dyn_dg_solve5(S A[5][5], S b[5], S x[5]) {
  for (int c = 0; c < 5; ++c) {
    int piv = c;
    for (int r = c + 1; r < 5; ++r)
      if (dyn_dg_abs(A[r][c]) > dyn_dg_abs(A[piv][c]))
        piv = r;
    if (!(dyn_dg_abs(A[piv][c]) > 0.0))
      return false;
    if (piv != c) {
      for (int k = 0; k < 5; ++k) {
        const S t = A[c][k];
        A[c][k] = A[piv][k];
        A[piv][k] = t;
      }
      const S t = b[c];
      b[c] = b[piv];
      b[piv] = t;
    }
    for (int r = c + 1; r < 5; ++r) {
      const S f = A[r][c] / A[c][c];
      for (int k = c; k < 5; ++k)
        A[r][k] = A[r][k] - f * A[c][k];
      b[r] = b[r] - f * b[c];
    }
  }
  for (int r = 4; r >= 0; --r) {
    S acc = b[r];
    for (int k = r + 1; k < 5; ++k)
      acc = acc - A[r][k] * x[k];
    x[r] = acc / A[r][r];
  }
  return true;
}

// The spin step's books, in J: dH = supplied - braked - dissipated - slip + floor + grade + port -
// scheme + limiter + residual. The powers are dyn_spin_ph_rates_at's at each solve's e_th, times
// its h; port the wrench's work, h W^T v_th; scheme the wheels' rows' damping, (th - 1/2) sum J
// dom^2 (zero at th = 1/2); limiter what the clamp and the blend changed. The residual is the
// solves' and round-off.
template <class S> struct dyn_spin_dg_books {
  S dH, dissipated, slip, supplied, braked, floor, grade, port, scheme, limiter, residual;
};

// F(e1) = M (e1 - e0) - h (J e_th + Q(e_th)), e_th = e0 + Theta (e1 - e0); `mid` carries the pose
// and the actuators, and gets e_th.
template <class S>
CVC_HD inline void dyn_spin_dg_residual(const dyn_bike_params &p, dyn_spin_state<S> &mid,
                                        const S e0[5], const S e1[5], const S theta[5], S h,
                                        S grade, const S *wrench, S F[5]) {
  const S M[5] = {S(p.m), S(p.m), S(p.Iz), S(p.J_f), S(p.J_r)};
  S et[5], f[5];
  for (int i = 0; i < 5; ++i)
    et[i] = e0[i] + theta[i] * (e1[i] - e0[i]);
  dyn_spin_dg_set(mid, et);
  dyn_spin_dg_rate(p, mid, grade, wrench, f);
  for (int i = 0; i < 5; ++i)
    F[i] = M[i] * (e1[i] - e0[i]) - h * f[i];
}

// One step: dyn_spin_plant's signature without `side`, plus the solve's report (over the substeps:
// the Newton steps added, the worst update and residual), its controls and, when given, its books.
template <class S>
CVC_HD inline void dyn_spin_plant_dg(const dyn_bike_params &p, const bike_veh &v,
                                     const dyn_spin_state<S> &s, S delta_cmd, S a_cmd, S grade,
                                     dyn_spin_state<S> &o, dyn_step_free<S> *seen = nullptr,
                                     const S *wrench = nullptr, dyn_dg_solve *rep = nullptr,
                                     const dyn_spin_dg_options &opt = dyn_spin_dg_options(),
                                     dyn_spin_dg_books<S> *books = nullptr) {
  typedef dual<S> D;
  const S hdt = S(v.hdt), one = S(1), half = S(0.5);
  const S L = S(p.lf) + S(p.lr), rw = S(p.r_w);
  const int n = opt.substeps > 0 ? opt.substeps : 1;
  const S h = hdt / S(n), th = S(opt.theta_wheels);
  const S theta[5] = {half, half, half, th, th};
  const S M[5] = {S(p.m), S(p.m), S(p.Iz), S(p.J_f), S(p.J_r)};
  const S wscale[5] = {one, one, L, rw, rw}; // each row in length / s
  const double tol = opt.tol > 0.0 ? opt.tol : (sizeof(S) == sizeof(float) ? 1e-6 : 1e-13);

  // 1. Actuators: dyn_plant's step 1.
  S delta1, a1;
  dyn_dg_actuators(p, hdt, s.b, delta_cmd, a_cmd, delta1, a1);

  D wd[3];
  if (wrench)
    for (int i = 0; i < 3; ++i)
      wd[i] = D(wrench[i]);
  dyn_spin_dg_books<S> bk{};
  dyn_dg_solve sol;
  sol.converged = true;
  S e0[5] = {s.b.vx, s.b.vy, s.b.r, s.om_f, s.om_r};
  for (int k = 0; k < n; ++k) {
    // 2. A substep: Newton from e1 = e0, the actuators at the substep's midpoint.
    dyn_spin_state<S> mid = s;
    const S frac = (S(k) + half) / S(n);
    mid.b.delta = s.b.delta + frac * (delta1 - s.b.delta);
    mid.b.a = s.b.a + frac * (a1 - s.b.a);
    S e1[5], F[5];
    for (int i = 0; i < 5; ++i)
      e1[i] = e0[i];
    double scale = 1.0;
    for (int i = 0; i < 5; ++i)
      scale += dyn_dg_abs(wscale[i] * e0[i]);
    dyn_spin_dg_residual(p, mid, e0, e1, theta, h, grade, wrench, F);
    S fnorm = dyn_spin_dg_norm(p, F);
    bool converged = false;
    double upd = 0.0;
    int iters = 0, secants = 0;
    for (int it = 0; it < opt.max_iters; ++it) {
      iters = it + 1;
      // A = dF/de1 = M - h d(rate)/d(e_th) Theta, a column per direction.
      S A[5][5];
      for (int j = 0; j < 5; ++j) {
        dyn_spin_state<D> md;
        for (int c = 0; c < dyn_spin_state<S>::size; ++c)
          md[c] = D(mid[c]);
        D ed[5];
        for (int i = 0; i < 5; ++i)
          ed[i] = D(e0[i] + theta[i] * (e1[i] - e0[i]), i == j ? one : S(0));
        dyn_spin_dg_set(md, ed);
        D fd[5];
        dyn_spin_dg_rate_one<D>(p, md, D(grade), wrench ? wd : nullptr, fd);
        for (int i = 0; i < 5; ++i)
          A[i][j] = (i == j ? M[i] : S(0)) - h * theta[j] * fd[i].d;
      }
      S b[5], d[5];
      for (int i = 0; i < 5; ++i)
        b[i] = -F[i];
      if (!dyn_dg_solve5(A, b, d))
        break;
      // Backtrack until |M^-1 F| falls (a full step near the root).
      S t = one, xt[5], Ft[5], ft = fnorm;
      bool took = false;
      auto search = [&](int halvings) {
        t = one;
        for (int ls = 0; ls <= halvings; ++ls) {
          for (int i = 0; i < 5; ++i)
            xt[i] = e1[i] + t * d[i];
          dyn_spin_dg_residual(p, mid, e0, xt, theta, h, grade, wrench, Ft);
          ft = dyn_spin_dg_norm(p, Ft);
          if (s_val(ft) < s_val(fnorm))
            return true;
          t = half * t;
        }
        return false;
      };
      double nsize = 0.0;
      for (int i = 0; i < 5; ++i)
        nsize += dyn_dg_abs(wscale[i] * d[i]);
      if (nsize < 1e3 * tol * scale) {
        // Newton's step is at round-off (what is left is its square): take it whole and stop,
        // rather than spend a line search finding no decrease.
        for (int i = 0; i < 5; ++i)
          e1[i] = e1[i] + d[i];
        dyn_spin_dg_residual(p, mid, e0, e1, theta, h, grade, wrench, F);
        fnorm = dyn_spin_dg_norm(p, F);
        upd = nsize / scale;
        converged = true;
        break;
      }
      // Newton's direction gets two halvings: one that needs more is climbing.
      took = search(2);
      if (!took) {
        // Past a tire's peak its force falls as its slip grows, and with a stiff slip
        // M - h dF/de can turn indefinite: Newton's direction then climbs. The secant step
        // freezes the plant's own structure and dissipation at the iterate, M - h (J - R) Theta
        // (dyn_spin_ph_J, dyn_spin_ph_R), and R >= 0 at every slip, so it pulls each slip
        // toward its root, as dyn_spin_plant's implicit step does.
        ++secants;
        S et[5], x[5], Jm[5][5], R[5][5];
        for (int i = 0; i < 5; ++i)
          et[i] = e0[i] + theta[i] * (e1[i] - e0[i]);
        dyn_spin_dg_set(mid, et);
        dyn_spin_ph_momentum(p, mid, x);
        dyn_spin_ph_J(x, Jm);
        dyn_spin_ph_R(p, mid, R);
        for (int i = 0; i < 5; ++i)
          for (int j = 0; j < 5; ++j)
            A[i][j] = (i == j ? M[i] : S(0)) - h * (Jm[i][j] - R[i][j]) * theta[j];
        for (int i = 0; i < 5; ++i)
          b[i] = -F[i];
        if (dyn_dg_solve5(A, b, d))
          took = search(11);
      }
      double size = 0.0;
      for (int i = 0; i < 5; ++i)
        size += dyn_dg_abs(wscale[i] * d[i]);
      upd = (double)s_val(t) * size / scale;
      if (!took) { // at round-off: no step lowers the residual
        converged = upd < 1e3 * tol;
        break;
      }
      for (int i = 0; i < 5; ++i) {
        e1[i] = xt[i];
        F[i] = Ft[i];
      }
      fnorm = ft;
      if (upd < tol) {
        converged = true;
        break;
      }
    }
    sol.iters += iters;
    sol.secant += secants;
    sol.update = upd > sol.update ? upd : sol.update;
    sol.residual = dyn_dg_abs(fnorm) > sol.residual ? dyn_dg_abs(fnorm) : sol.residual;
    sol.converged = sol.converged && converged;
    if (books) {
      S et[5];
      for (int i = 0; i < 5; ++i)
        et[i] = e0[i] + theta[i] * (e1[i] - e0[i]);
      dyn_spin_dg_set(mid, et);
      const dyn_spin_ph_rates<S> r = dyn_spin_ph_rates_at(p, mid, grade);
      bk.dissipated = bk.dissipated + h * r.dissipated;
      bk.slip = bk.slip + h * r.slip;
      bk.supplied = bk.supplied + h * r.supplied;
      bk.braked = bk.braked + h * r.braked;
      bk.floor = bk.floor + h * r.floor;
      bk.grade = bk.grade + h * r.grade;
      if (wrench)
        bk.port = bk.port + h * (wrench[0] * et[0] + wrench[1] * et[1] + wrench[2] * et[2]);
      for (int i = 3; i < 5; ++i)
        bk.scheme = bk.scheme + (theta[i] - half) * M[i] * (e1[i] - e0[i]) * (e1[i] - e0[i]);
    }
    for (int i = 0; i < 5; ++i)
      e0[i] = e1[i];
  }
  if (rep)
    *rep = sol;
  if (seen) {
    seen->vx = e0[0];
    seen->vy = e0[1];
    seen->r = e0[2];
  }

  // 3 and 4. Limiters and pose, on the body; the wheels keep their solved speeds.
  dyn_dg_finish(p, v, s.b, delta1, a1, e0, o.b);
  o.om_f = e0[3];
  o.om_r = e0[4];

  if (books) {
    dyn_spin_state<S> e = o;
    e.b.vx = e0[0];
    e.b.vy = e0[1];
    e.b.r = e0[2];
    const S h0 = dyn_spin_ph_hamiltonian(p, s), he = dyn_spin_ph_hamiltonian(p, e),
            h1 = dyn_spin_ph_hamiltonian(p, o);
    bk.dH = h1 - h0;
    bk.limiter = h1 - he;
    bk.residual = bk.dH - (bk.supplied - bk.braked - bk.dissipated - bk.slip + bk.floor + bk.grade +
                           bk.port - bk.scheme + bk.limiter);
    *books = bk;
  }
}

// dyn_spin_plant_dg's step with its books. The step is dyn_spin_plant_dg's, to the last bit.
template <class S>
CVC_HD inline void dyn_spin_plant_dg_books(const dyn_bike_params &p, const bike_veh &v,
                                           const dyn_spin_state<S> &s, S delta_cmd, S a_cmd,
                                           S grade, dyn_spin_state<S> &o, dyn_spin_dg_books<S> &b,
                                           const S *wrench = nullptr, dyn_dg_solve *rep = nullptr,
                                           const dyn_spin_dg_options &opt = dyn_spin_dg_options()) {
  dyn_spin_plant_dg<S>(p, v, s, delta_cmd, a_cmd, grade, o, nullptr, wrench, rep, opt, &b);
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_DYN_BIKE_DG_H__
