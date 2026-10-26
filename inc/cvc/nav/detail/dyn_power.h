/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav::diff::power_params / power_step: a differentiable power model for the dynamic
// vehicle (the vehicle-physics plan's P8c), in nav's normalized frame.
//
// What the vehicle draws from its source (fuel or battery, as the P3 ledger's `source` counts it,
// or its shaft work without one) over a step of length dt, from the dynamic vehicle's own state at
// the step's end: its drive force per unit mass `a` (dyn_state::a, the actuator after its lag), its
// speed `vx` and its yaw rate `r`. With the drive power P = m a vx,
//
//   E = dt (k_pos P+ + k_neg P- + p_idle + c_v |vx| + c_v3 |vx|^3 + c_lat m (vx r)^2 |vx|)
//
// P+ and P- are the smooth positive and negative parts of P (width `w`), |vx| a smooth magnitude
// (width `v_eps`). It is a Willans line: k_pos the source energy per joule of positive drive work
// (the driveline's and engine's efficiency together), p_idle what the source draws at rest, and
// k_neg what braking returns (<= 0: 0 for a fuel engine, which burns none on the overrun;
// negative for regeneration). c_v |vx| and c_v3 |vx|^3 are what the drive force does not carry:
// rolling resistance and drag where the dynamic vehicle leaves them out (a direct drive's, whose
// fitted command is the achieved acceleration), and losses that grow with engine speed. The last
// is cornering's: a tire's lateral force F slips it sideways at about F / C_alpha times the speed,
// so it loses about m a_lat^2 |vx| / C, a_lat = vx r; c_lat is 1 / C per unit mass.
//
// Every term is linear in its coefficient, so cvc::physics::fit_power fits them to a plant log's
// per-tick energy by least squares (physics/plant_log.h). Off (all coefficients 0) the step costs
// nothing. Header-only and allocation-free, like detail/dyn_bike.h.
//
// The tire term (power_step_tire): c_lat sees only cornering's lateral acceleration, and on a
// dense scene tire slip is about half of what the engine supplies (docs/NAV_TRAINING.md, "Energy
// in the loss"). So the model can also price what the dynamic vehicle's own tires lose to slip,
// dyn_tire_power at the step's end state (vx, vy, r, delta, a):
//
//   E += dt (tire_lat P_lat + tire_lon P_lon)
//
// P_lat is each axle's lateral force times its slip velocity u sin(alpha), P_lon its drive or
// brake force times its slip velocity. fit_power calibrates them to the ledger's `tire` term.

#ifndef __CVC_NAV_DETAIL_DYN_POWER_H__
#define __CVC_NAV_DETAIL_DYN_POWER_H__

#include <cmath>
#include <cvc/nav/detail/dyn_bike.h> // dyn_bike_params, dyn_plant's tires and the dual

namespace cvc {
namespace nav {
namespace diff {

struct power_params {
  float m = 1.0f;      // kg: the vehicle's mass (dyn_bike_params::m)
  float k_pos = 0.0f;  // source energy per unit of positive wheel work
  float k_neg = 0.0f;  // per unit of negative (braking) wheel work: <= 0 returns energy
  float p_idle = 0.0f; // source power at rest (normalized: kg length^2 / s^3)
  float c_v = 0.0f;    // source power per unit speed (kg length / s^2)
  float c_v3 = 0.0f;   // per unit speed cubed (kg / length): drag's share
  float c_lat = 0.0f;  // cornering: per m (vx r)^2 |vx| (s^2 / length)
  // Source energy per unit of the dynamic vehicle's own tire dissipation (dyn_tire_power), lateral
  // and longitudinal: power_step_tire. Dimensionless.
  float tire_lat = 0.0f, tire_lon = 0.0f;
  float w = 0.0f;      // the parts' smoothing width, in power; <= 0: 1e-3 of the full power
  float v_eps = 0.0f;  // |vx|'s smoothing width (length / s); <= 0: 1e-3 length / s
  bool on() const {
    return k_pos != 0.0f || k_neg != 0.0f || p_idle != 0.0f || c_v != 0.0f || c_v3 != 0.0f ||
           c_lat != 0.0f || has_tire();
  }
  bool has_tire() const { return tire_lat != 0.0f || tire_lon != 0.0f; }
};

// The source power at full drive force a_max and speed vmax: the scale the trainer normalizes the
// energy term by (coef_train's w_energy), and the default smoothing width's.
inline float power_full(const power_params &p, float a_max, float vmax) {
  return p.k_pos * p.m * a_max * vmax + p.p_idle + p.c_v * vmax + p.c_v3 * vmax * vmax * vmax;
}

// Smooth positive part, width w: w log(1 + e^{x / w}), and its derivative (the logistic).
inline float power_softplus(float x, float w, float &d) {
  const float z = x / w;
  if (z > 30.0f) {
    d = 1.0f;
    return x;
  }
  if (z < -30.0f) {
    d = 0.0f;
    return 0.0f;
  }
  const float e = std::exp(z);
  d = e / (1.0f + e);
  return w * std::log1p(e);
}

// The energy drawn over one step of length dt (normalized: kg length^2 / s^2), and its
// derivatives in vx, a and r. `w_default` is the width used when p.w <= 0.
inline float power_step(const power_params &p, float vx, float a, float r, float dt,
                        float w_default, float &dE_dvx, float &dE_da, float &dE_dr) {
  const float w = p.w > 0.0f ? p.w : w_default;
  const float ve = p.v_eps > 0.0f ? p.v_eps : 1e-3f;
  const float P = p.m * a * vx;
  float dpos, dneg;
  const float Ppos = power_softplus(P, w, dpos);
  const float Pneg = power_softplus(-P, w, dneg); // the braking power's magnitude
  const float speed = std::sqrt(vx * vx + ve * ve);
  // dE/dP = dt (k_pos dpos - k_neg dneg); dP/dvx = m a, dP/da = m vx.
  const float dE_dP = dt * (p.k_pos * dpos - p.k_neg * dneg);
  // Cornering: C = c_lat m (vx r)^2 speed; dC/dvx = c_lat m r^2 vx (2 speed + vx^2 / speed),
  // dC/dr = 2 c_lat m vx^2 r speed.
  const float lat = p.c_lat * p.m * vx * vx * r * r;
  // d|vx|/dvx = vx / speed; d|vx|^3/dvx = 3 speed vx.
  dE_dvx = dE_dP * p.m * a + dt * (p.c_v * vx / speed + 3.0f * p.c_v3 * speed * vx +
                                   p.c_lat * p.m * r * r * vx * (2.0f * speed + vx * vx / speed));
  dE_da = dE_dP * p.m * vx;
  dE_dr = dt * 2.0f * p.c_lat * p.m * vx * vx * r * speed;
  return dt * (p.k_pos * Ppos + p.k_neg * Pneg + p.p_idle + p.c_v * speed +
               p.c_v3 * speed * speed * speed + lat * speed);
}

// A smooth magnitude, sqrt(x^2 + w^2) - w: 0 at 0, |x| - w far from it.
template <class S> CVC_HD inline S tire_abs(S x, S w) { return s_sqrt(x * x + w * w) - w; }

// One axle's longitudinal slip power: |F| kappa u. kappa is the slip ratio at which the axle's
// Pacejka curve, D = cap and the lateral B and C (a longitudinal stiffness like the lateral's),
// gives F: kappa = tan(asin(|F| / cap) / C) / B, with |F| / cap held smoothly below 0.995 (where
// asin is still finite and kappa is about 0.3 at B = 12). u is the speed with the slip angles'
// floor, so a thrust at a standstill still slips.
template <class S> CVC_HD inline S dyn_tire_lon(S F, S cap, S B, S C, S u) {
  const S f = tire_abs(F, S(1e-3) * cap);
  const S x = dyn_sat(f / cap, S(0.995));
  const S as = s_atan2(x, s_sqrt(S(1) - x * x)); // asin(x)
  const S kappa = s_tan(as / C) / B;
  return f * kappa * u;
}

// The dynamic vehicle's tire dissipation at a state (normalized power): what dyn_plant's tires lose
// to slip, from the same loads, force split, friction ellipse and Pacejka curves it integrates.
// P_lat sums each axle's lateral force times the sideways slip velocity its slip angle implies,
// u sin(alpha): Pacejka's force has alpha's sign (C < 2), so each product is >= 0 and smooth
// everywhere, a crawl at full lock included (where the slip angles' speed floor makes the
// geometric sideways velocity disagree with the force's sign). P_lon sums each axle's
// dyn_tire_lon. `w` is unused by the lateral part, kept for the call's shape. The state is
// dyn_state's: (vx, vy) at the centre of gravity, r, the road-wheel angle delta and the drive force
// per unit mass a.
template <class S>
CVC_HD inline void dyn_tire_power(const dyn_bike_params &p, S vx, S vy, S r, S delta, S a, S w,
                                  S &P_lat, S &P_lon) {
  const S zero = S(0), one = S(1);
  const S m = S(p.m), lf = S(p.lf), lr = S(p.lr), h = S(p.h), g = S(p.g), mu = S(p.mu);
  const S Cp = S(p.C), L = lf + lr, Bf = S(p.B_f), Br = S(p.B_r);
  const S fz_floor = S(p.fz_min) * m * g;
  S Fzf = m * (g * lr - a * h) / L, Fzr = m * (g * lf + a * h) / L;
  if (s_val(Fzf) < s_val(fz_floor))
    Fzf = fz_floor;
  if (s_val(Fzr) < s_val(fz_floor))
    Fzr = fz_floor;
  const S cap_f = mu * Fzf, cap_r = mu * Fzr;
  const S F_drive = m * a;
  const S share = S(s_val(a) >= s_val(zero) ? p.front_drive : p.front_brake);
  const S Fxf = dyn_sat(F_drive * share, cap_f), Fxr = dyn_sat(F_drive * (one - share), cap_r);
  const S Dyf = dyn_lateral_cap(Fxf, cap_f), Dyr = dyn_lateral_cap(Fxr, cap_r);
  const S u = s_sqrt(vx * vx + S(p.eps) * S(p.eps)), sg = vx / u;
  const S af = sg * delta - s_atan2(vy + lf * r, u), ar = -s_atan2(vy - lr * r, u);
  const S Fyf = dyn_lateral(af, Bf, Cp, Dyf), Fyr = dyn_lateral(ar, Br, Cp, Dyr);
  P_lat = Fyf * u * s_sin(af) + Fyr * u * s_sin(ar);
  (void)w;
  P_lon = dyn_tire_lon(Fxf, cap_f, Bf, Cp, u) + dyn_tire_lon(Fxr, cap_r, Br, Cp, u);
}

// power_step with the tire term (p.tire_lat, p.tire_lon) of `dyn`'s vehicle at the step's end
// state, and the energy's derivatives in (vx, vy, r, delta, a), the tire term's by forward-mode
// duals. Without a tire term it is power_step, with zero derivatives in vy and delta.
inline float power_step_tire(const power_params &p, const dyn_bike_params &dyn, float vx, float vy,
                             float r, float delta, float a, float dt, float w_default, float g[5]) {
  float dvx, da, dr;
  float E = power_step(p, vx, a, r, dt, w_default, dvx, da, dr);
  g[0] = dvx;
  g[1] = 0.0f;
  g[2] = dr;
  g[3] = 0.0f;
  g[4] = da;
  if (!p.has_tire())
    return E;
  const float w = p.w > 0.0f ? p.w : w_default;
  const float in[5] = {vx, vy, r, delta, a};
  for (int k = 0; k < 5; ++k) {
    dual<float> x[5];
    for (int j = 0; j < 5; ++j)
      x[j] = dual<float>(in[j], j == k ? 1.0f : 0.0f);
    dual<float> pl, po;
    dyn_tire_power(dyn, x[0], x[1], x[2], x[3], x[4], dual<float>(w), pl, po);
    g[k] += dt * (p.tire_lat * pl.d + p.tire_lon * po.d);
    if (k == 0)
      E += dt * (p.tire_lat * pl.v + p.tire_lon * po.v);
  }
  return E;
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_DYN_POWER_H__
