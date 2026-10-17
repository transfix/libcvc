/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// material_bike_rollout.cpp — integrate_bicycle_material and its adjoint (see
// material.h). The material trainer's force law (integrate_surrogate_material)
// fed to the deployed kinematic bicycle (drive.cpp bicycle_rollout_material, one
// substep). The forward follows drive.cpp's op order; the adjoint walks a per-step
// record of every intermediate and every branch the forward took, so gradient flows
// through the taken branch exactly as autograd treats min/max/where.
//
// Where the deployed drive samples its SDF for the barrier, the governor's
// clearance d and normal n, this uses the trainer's obstacle discs. The barrier is
// the sum of the per-disc barriers; d and n are the nearest disc's (the footprint
// path's "binding disc" rule). Built with -ffp-contract=off. Do not "tidy" the op
// order.

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/material.h>
#include <limits>
#include <stdexcept>
#include <vector>

namespace cvc {
namespace nav {

namespace {

// Far-away clearance when an agent has no valid obstacle: the stop governor then
// never binds (its sqrt stays finite in float).
constexpr float kFar = 1e30f;

// clamp(x, lo, hi) as fminf(fmaxf(x, lo), hi), reporting which side bound:
// -1 = lo, +1 = hi, 0 = neither.
inline float clamp3(float x, float lo, float hi, int &side) {
  if (x < lo) {
    side = -1;
    return lo;
  }
  if (x > hi) {
    side = 1;
    return hi;
  }
  side = 0;
  return x;
}

// Per-agent constants.
struct agent {
  const float *Cb, *Rb, *ab;
  const std::uint8_t *mb;
  int N;
  const float *chan[6]; // risk, phi, dr/dx, dr/dy, dphi/dx, dphi/dy
  int Hp, Wp;
  float o0x, o0y, gx, gy, be, ga, ls, lh, dh, hdt, rr_eff, mass, k_sharp, d_hat_sdf;
  float L, dmax, tan_dmax, a_max, a_lat, k_steer, vmax, sp_min, v_creep_cap, kfloor;
  bool allow_reverse;
};

struct state {
  float ox, oy, th, sp;
};

// Everything one forward step computed, for the reverse.
struct rec {
  bool active;
  float ch, sh;
  // patch samples at off = o - o0
  float offx, offy, risk_raw, sdf_raw, risk, sdf, grx, gry, gpx, gpy, db;
  // geometry (sums over discs), the governor's clearance and normal
  float Fgx, Fgy, Frx, Fry, d, ngx, ngy;
  int jmin;
  // forces
  float Fmx, Fmy, Fx, Fy;
  // longitudinal command before the governors
  float a0;
  int a0_side;
  float a0c;
  // pure pursuit
  float tgx, tgy, tg_r2, Ld_raw;
  bool Ld_floor;
  float ang, sa;
  bool behind;
  float Ldeff_raw;
  bool Ldeff_floor;
  float Ldeff_cap;
  bool pick_cap; // Ldeff = the speed-based cap (else the goal distance)
  float Ldeff, delta0, S, tS, delta1;
  int delta2_side;
  float delta2;
  // governors
  float tan2, kappa_raw;
  bool kappa_floor;
  float kappa, v_corner, ds_raw;
  bool ds_floor;
  float ds, v_stop, msign, appr_raw;
  int appr_side;
  float approach;
  bool appr_dir;
  float v_stop_dir;
  bool inner_pick_stop, outer_pick_m;
  float vlim0;
  bool pick_floor;
  float v_floor, v_lim, v_creep;
  bool creep_pick_cap;
  bool hard_steer, can_move;
  float fbh_raw;
  bool fbh_live;
  float a_bh;
  bool stuck, stuck_pick;
  float a_st;
  bool rev;
  float a1;
  int a1_side;
  float a1c, lim;
  bool pick_lim;
  float a2;
  int a2_side;
  float a3, delta3, pre_sp;
  int sp_side;
  float sp1, sp2_raw;
  bool sp2_floor;
  float sp2, q, d_cap;
  int d4_side;
  float delta4, tan4, th1, c1, s1, o1x, o1y, disp, ux, uy;
};

// One step of the material bicycle (drive.cpp bicycle_rollout_material substep).
state step_fwd(const agent &a, const state &st, rec &r, float &dmin_out, int &jmin_out) {
  const float ox = st.ox, oy = st.oy, thi = st.th, spi = st.sp;
  r.ch = std::cos(thi);
  r.sh = std::sin(thi);
  const float ch = r.ch, sh = r.sh;

  // Patch samples (the surrogate's order and clamps).
  r.offx = ox - a.o0x;
  r.offy = oy - a.o0y;
  r.risk_raw = detail::patch_sample_channel(a.chan[0], a.Hp, a.Wp, r.offx, r.offy);
  r.sdf_raw = detail::patch_sample_channel(a.chan[1], a.Hp, a.Wp, r.offx, r.offy);
  r.grx = detail::patch_sample_channel(a.chan[2], a.Hp, a.Wp, r.offx, r.offy);
  r.gry = detail::patch_sample_channel(a.chan[3], a.Hp, a.Wp, r.offx, r.offy);
  r.gpx = detail::patch_sample_channel(a.chan[4], a.Hp, a.Wp, r.offx, r.offy);
  r.gpy = detail::patch_sample_channel(a.chan[5], a.Hp, a.Wp, r.offx, r.offy);
  r.risk = std::min(std::max(r.risk_raw, 0.0f), 1.0f);
  r.sdf = std::min(std::max(r.sdf_raw, 0.0f), 50.0f);

  // Geometry: the barrier summed over discs; the governor reads the nearest disc.
  r.Fgx = 0.0f;
  r.Fgy = 0.0f;
  r.Frx = 0.0f;
  r.Fry = 0.0f;
  float dmin = std::numeric_limits<float>::infinity();
  int jmin = -1;
  float nminx = 0.0f, nminy = 0.0f;
  for (int j = 0; j < a.N; ++j) {
    if (!a.mb[j])
      continue;
    const float dx = ox - a.Cb[2 * j], dy = oy - a.Cb[2 * j + 1];
    float rad = std::sqrt(dx * dx + dy * dy);
    if (rad < 1e-9f)
      rad = 1e-9f;
    const float nhx = dx / rad, nhy = dy / rad;
    const float dj = rad - (a.Rb[j] + a.rr_eff);
    const float bj = detail::ipc_dbdd_pw(dj, a.dh);
    r.Fgx += -(a.ab[j] * bj) * nhx;
    r.Fgy += -(a.ab[j] * bj) * nhy;
    const float brep = bj < 0.0f ? bj : 0.0f; // clamp(max=0)
    r.Frx += -(a.ab[j] * brep) * nhx;
    r.Fry += -(a.ab[j] * brep) * nhy;
    if (dj < dmin) {
      dmin = dj;
      jmin = j;
      nminx = nhx;
      nminy = nhy;
    }
  }
  r.jmin = jmin;
  r.d = jmin >= 0 ? dmin : kFar;
  r.ngx = nminx;
  r.ngy = nminy;
  dmin_out = dmin;
  jmin_out = jmin;
  const float d = r.d;

  // Goal spring + material forces (drive.cpp order: F = Fbar + Fgoal, then + Fmat).
  const float Fgoal_x = -a.be * (ox - a.gx), Fgoal_y = -a.be * (oy - a.gy);
  r.db = detail::sdf_barrier_db(r.sdf, a.k_sharp, a.d_hat_sdf);
  const float fsx = -a.ls * r.grx, fsy = -a.ls * r.gry;
  const float fhx = (-a.lh * r.db) * r.gpx, fhy = (-a.lh * r.db) * r.gpy;
  r.Fmx = fsx + fhx;
  r.Fmy = fsy + fhy;
  r.Fx = r.Fgx + Fgoal_x;
  r.Fy = r.Fgy + Fgoal_y;
  r.Fx = r.Fx + r.Fmx;
  r.Fy = r.Fy + r.Fmy;

  r.a0 = ((r.Fx * ch + r.Fy * sh) - a.ga * spi) / a.mass;
  r.a0c = clamp3(r.a0, -a.a_max, a.a_max, r.a0_side);

  // Pure pursuit toward the goal.
  r.tgx = a.gx - ox;
  r.tgy = a.gy - oy;
  r.tg_r2 = r.tgx * r.tgx + r.tgy * r.tgy;
  r.Ld_raw = std::sqrt(r.tg_r2);
  r.Ld_floor = r.Ld_raw < 1e-6f;
  const float L_d = r.Ld_floor ? 1e-6f : r.Ld_raw;
  r.ang = std::atan2(r.tgy, r.tgx) - thi;
  r.sa = std::sin(r.ang);
  const float cos_a = std::cos(r.ang);
  r.behind = cos_a < 0.0f;
  const float turn_sign = r.sa >= 0.0f ? 1.0f : -1.0f;
  r.Ldeff_raw = 1.2f * spi + 4.0f * a.L;
  r.Ldeff_floor = r.Ldeff_raw < 4.0f * a.L;
  r.Ldeff_cap = r.Ldeff_floor ? 4.0f * a.L : r.Ldeff_raw;
  r.pick_cap = r.Ldeff_cap < L_d; // std::min(L_d, cap)
  r.Ldeff = r.pick_cap ? r.Ldeff_cap : L_d;
  r.delta0 = r.behind ? turn_sign * a.dmax : std::atan2(2.0f * a.L * r.sa, r.Ldeff);
  // Steering bias from the repulsive barrier and the material forces.
  r.S = (r.Frx + r.Fmx) * (-sh) + (r.Fry + r.Fmy) * ch;
  r.tS = std::tanh(r.S);
  r.delta1 = r.delta0 + a.k_steer * r.tS;
  r.delta2 = clamp3(r.delta1, -a.dmax, a.dmax, r.delta2_side);

  // Governors.
  r.tan2 = std::tan(r.delta2);
  r.kappa_raw = std::fabs(r.tan2) / a.L;
  r.kappa_floor = r.kappa_raw < a.kfloor;
  r.kappa = r.kappa_floor ? a.kfloor : r.kappa_raw;
  r.v_corner = std::sqrt(a.a_lat / r.kappa);
  r.ds_raw = d - 0.5f * a.rr_eff;
  r.ds_floor = r.ds_raw < 0.0f;
  r.ds = r.ds_floor ? 0.0f : r.ds_raw;
  r.v_stop = std::sqrt(2.0f * a.a_max * r.ds);
  r.msign = spi >= 0.0f ? 1.0f : -1.0f;
  r.appr_raw = -(r.ngx * ch + r.ngy * sh) * r.msign;
  r.approach = clamp3(r.appr_raw, 0.0f, 1.0f, r.appr_side);
  r.appr_dir = r.approach > 0.05f;
  r.v_stop_dir = r.appr_dir ? r.v_stop / std::max(r.approach, 0.05f) : a.vmax;
  r.inner_pick_stop = r.v_stop_dir < r.v_corner; // std::min(v_corner, v_stop_dir)
  const float m = r.inner_pick_stop ? r.v_stop_dir : r.v_corner;
  r.outer_pick_m = m < a.vmax; // std::min(vmax, m)
  r.vlim0 = r.outer_pick_m ? m : a.vmax;
  r.hard_steer = std::fabs(r.delta2) >= 0.7f * a.dmax;
  r.can_move = d > 0.25f * a.rr_eff;
  r.v_floor = (r.hard_steer && r.can_move) ? 0.08f : 0.0f;
  r.pick_floor = r.vlim0 < r.v_floor; // std::max(v_lim, v_floor)
  r.v_lim = r.pick_floor ? r.v_floor : r.vlim0;
  r.creep_pick_cap = a.v_creep_cap < 0.5f * r.v_corner; // std::min(0.5 v_corner, cap)
  r.v_creep = r.creep_pick_cap ? a.v_creep_cap : 0.5f * r.v_corner;

  float a_long = r.a0c;
  r.fbh_raw = r.Fgx * ch + r.Fgy * sh;
  r.fbh_live = !(r.fbh_raw > 0.0f);
  if (r.behind) {
    const float fbh = r.fbh_live ? r.fbh_raw : 0.0f; // clamp(max=0)
    a_long = (r.v_creep - spi) / a.hdt + fbh;
  }
  r.a_bh = a_long;
  r.stuck = r.hard_steer && r.can_move && std::fabs(spi) < 0.06f;
  r.stuck_pick = false;
  if (r.stuck) {
    const float floor_a = (0.08f - spi) / a.hdt;
    r.stuck_pick = a_long < floor_a; // std::max(a_long, floor)
    a_long = r.stuck_pick ? floor_a : a_long;
  }
  r.a_st = a_long;
  r.rev = false;
  float delta = r.delta2;
  if (a.allow_reverse) {
    int hs;
    const float head_on = clamp3(-(r.ngx * ch + r.ngy * sh), 0.0f, 1.0f, hs);
    if (r.behind && (head_on > 0.6f) && (d < 0.5f * a.rr_eff + 0.02f)) {
      r.rev = true;
      a_long = (-0.10f - spi) / a.hdt;
      delta = -delta;
    }
  }
  r.a1 = a_long;
  r.delta3 = delta;
  r.a1c = clamp3(r.a1, -a.a_max, a.a_max, r.a1_side);
  r.lim = (r.v_lim - spi) / a.hdt;
  r.pick_lim = r.lim < r.a1c; // std::min(a_long, lim)
  r.a2 = r.pick_lim ? r.lim : r.a1c;
  r.a3 = clamp3(r.a2, -a.a_max, a.a_max, r.a2_side);

  r.pre_sp = spi + a.hdt * r.a3;
  r.sp1 = clamp3(r.pre_sp, a.sp_min, a.vmax, r.sp_side);
  r.sp2_raw = r.sp1 * r.sp1;
  r.sp2_floor = r.sp2_raw < 1e-9f;
  r.sp2 = r.sp2_floor ? 1e-9f : r.sp2_raw;
  r.q = a.a_lat * a.L / r.sp2;
  r.d_cap = std::atan(r.q);
  r.delta4 = clamp3(r.delta3, -r.d_cap, r.d_cap, r.d4_side);
  r.tan4 = std::tan(r.delta4);
  r.th1 = thi + a.hdt * (r.sp1 / a.L) * r.tan4;
  r.c1 = std::cos(r.th1);
  r.s1 = std::sin(r.th1);
  r.o1x = ox + a.hdt * r.sp1 * r.c1;
  r.o1y = oy + a.hdt * r.sp1 * r.s1;

  const float sdx = r.o1x - ox, sdy = r.o1y - oy;
  r.disp = std::sqrt(sdx * sdx + sdy * sdy);
  r.ux = r.disp > 0.0f ? sdx / r.disp : 0.0f;
  r.uy = r.disp > 0.0f ? sdy / r.disp : 0.0f;

  if (!r.active) {
    r.disp = 0.0f;
    return st; // past H: the state is frozen
  }
  return {r.o1x, r.o1y, r.th1, r.sp1};
}

// Reverse of step_fwd. In: the adjoints of the step outputs (ao = dL/d o1, ath, asp)
// and the per-step seeds of arc length and cumulative risk. Out: the adjoints of the
// step inputs, in place; coefficient grads are ADDED into gal (per disc) and the
// scalar accumulators. A frozen step (past H) is the identity.
void step_bwd(const agent &a, const state &st, const rec &r, float garc, float gcum, float &aox,
              float &aoy, float &ath, float &asp, float *gal, float &gbe, float &gga, float &gls,
              float &glh) {
  if (!r.active)
    return;
  const float ox = st.ox, oy = st.oy, spi = st.sp;
  const float ch = r.ch, sh = r.sh;

  // arc += disp, cum += risk * disp, disp = |o1 - o|.
  const float g_disp = garc + gcum * r.risk;
  const float Ao1x = aox + g_disp * r.ux, Ao1y = aoy + g_disp * r.uy;
  float Aox = -g_disp * r.ux, Aoy = -g_disp * r.uy;
  const float Arisk = gcum * r.disp;

  // o1 = o + hdt * sp1 * (cos th1, sin th1)
  Aox += Ao1x;
  Aoy += Ao1y;
  float Asp1 = asp + a.hdt * (Ao1x * r.c1 + Ao1y * r.s1);
  const float Ath1 = ath + a.hdt * r.sp1 * (-Ao1x * r.s1 + Ao1y * r.c1);
  // th1 = th + hdt * (sp1 / L) * tan(delta4)
  float Ath = Ath1;
  Asp1 += Ath1 * a.hdt * r.tan4 / a.L;
  const float Adelta4 = Ath1 * a.hdt * (r.sp1 / a.L) * (1.0f + r.tan4 * r.tan4);
  // delta4 = clamp(delta3, -d_cap, d_cap)
  const float Adelta3 = r.d4_side == 0 ? Adelta4 : 0.0f;
  const float Adcap = r.d4_side > 0 ? Adelta4 : (r.d4_side < 0 ? -Adelta4 : 0.0f);
  // d_cap = atan(q), q = a_lat * L / sp2, sp2 = max(sp1^2, 1e-9)
  const float Aq = Adcap / (1.0f + r.q * r.q);
  if (!r.sp2_floor)
    Asp1 += Aq * (-r.q / r.sp2) * 2.0f * r.sp1;
  // sp1 = clamp(sp + hdt * a3, sp_min, vmax)
  const float Apre = r.sp_side == 0 ? Asp1 : 0.0f;
  float Asp = Apre;
  const float Aa3 = Apre * a.hdt;
  // a3 = clamp(a2, -a_max, a_max)
  const float Aa2 = r.a2_side == 0 ? Aa3 : 0.0f;
  // a2 = min(a1c, lim), lim = (v_lim - sp) / hdt
  float Avlim = 0.0f, Aa1c = 0.0f;
  if (r.pick_lim) {
    Avlim = Aa2 / a.hdt;
    Asp += -Aa2 / a.hdt;
  } else {
    Aa1c = Aa2;
  }
  // a1c = clamp(a1, -a_max, a_max)
  const float Aa1 = r.a1_side == 0 ? Aa1c : 0.0f;
  // reverse: a1 = (-0.10 - sp) / hdt and delta3 = -delta2; else a1 = a_st, delta3 = delta2
  float Aast = 0.0f, Adelta2 = 0.0f;
  if (r.rev) {
    Asp += -Aa1 / a.hdt;
    Adelta2 = -Adelta3;
  } else {
    Aast = Aa1;
    Adelta2 = Adelta3;
  }
  // stuck: a_st = max(a_bh, (0.08 - sp) / hdt)
  float Aabh = 0.0f;
  if (r.stuck && r.stuck_pick)
    Asp += -Aast / a.hdt;
  else
    Aabh = Aast;
  // behind: a_bh = (v_creep - sp) / hdt + min(Fg . head, 0); else a_bh = a0c
  float Avcreep = 0.0f, Aa0c = 0.0f, AFgx = 0.0f, AFgy = 0.0f, Ach = 0.0f, Ash = 0.0f;
  if (r.behind) {
    Avcreep = Aabh / a.hdt;
    Asp += -Aabh / a.hdt;
    if (r.fbh_live) {
      AFgx += Aabh * ch;
      AFgy += Aabh * sh;
      Ach += Aabh * r.Fgx;
      Ash += Aabh * r.Fgy;
    }
  } else {
    Aa0c = Aabh;
  }
  // a0c = clamp(a0), a0 = ((F . head) - gamma * sp) / mass
  const float Aa0 = r.a0_side == 0 ? Aa0c : 0.0f;
  const float Ac = Aa0 / a.mass;
  const float AFx = Ac * ch, AFy = Ac * sh;
  Ach += Ac * r.Fx;
  Ash += Ac * r.Fy;
  gga += -Ac * spi;
  Asp += -Ac * a.ga;

  // v_lim = max(vlim0, v_floor); vlim0 = min(vmax, min(v_corner, v_stop_dir))
  const float Avlim0 = r.pick_floor ? 0.0f : Avlim;
  const float Am = r.outer_pick_m ? Avlim0 : 0.0f;
  const float Avsd = r.inner_pick_stop ? Am : 0.0f;
  float Avc = r.inner_pick_stop ? 0.0f : Am;
  // v_creep = min(0.5 * v_corner, cap)
  if (!r.creep_pick_cap)
    Avc += 0.5f * Avcreep;
  // v_stop_dir = v_stop / approach while approach > 0.05 (else the constant vmax)
  float Avstop = 0.0f, Aappr = 0.0f;
  if (r.appr_dir) {
    Avstop = Avsd / r.approach;
    Aappr = -Avsd * r.v_stop / (r.approach * r.approach);
  }
  // approach = clamp(-(n . head) * msign, 0, 1)
  float Angx = 0.0f, Angy = 0.0f;
  if (r.appr_side == 0) {
    const float Ar = Aappr * (-r.msign);
    Angx += Ar * ch;
    Angy += Ar * sh;
    Ach += Ar * r.ngx;
    Ash += Ar * r.ngy;
  }
  // v_stop = sqrt(2 a_max ds), ds = max(d - 0.5 rr_eff, 0)
  float Ad = 0.0f;
  if (!r.ds_floor && r.v_stop > 0.0f)
    Ad += Avstop * a.a_max / r.v_stop;
  // v_corner = sqrt(a_lat / kappa), kappa = max(|tan delta2| / L, kfloor)
  const float Akappa = Avc * (-0.5f * r.v_corner / r.kappa);
  if (!r.kappa_floor) {
    const float sgn = r.tan2 >= 0.0f ? 1.0f : -1.0f;
    Adelta2 += Akappa / a.L * sgn * (1.0f + r.tan2 * r.tan2);
  }

  // delta2 = clamp(delta1, -dmax, dmax); delta1 = delta0 + k_steer * tanh(S)
  const float Adelta1 = r.delta2_side == 0 ? Adelta2 : 0.0f;
  const float Adelta0 = Adelta1;
  const float AS = Adelta1 * a.k_steer * (1.0f - r.tS * r.tS);
  // S = (Fr + Fm) . left, left = (-sh, ch)
  const float AFrx = AS * (-sh), AFry = AS * ch;
  float AFmx = AS * (-sh), AFmy = AS * ch;
  Ash += -AS * (r.Frx + r.Fmx);
  Ach += AS * (r.Fry + r.Fmy);
  // delta0 = atan2(2 L sin(ang), Ldeff) unless behind (then a constant)
  if (!r.behind) {
    const float y = 2.0f * a.L * r.sa, x = r.Ldeff, den = x * x + y * y;
    if (den > 0.0f) {
      const float Asa = Adelta0 * 2.0f * a.L * x / den;
      const float ALdeff = Adelta0 * (-y) / den;
      // Ldeff = min(L_d, cap); cap = max(1.2 sp + 4L, 4L); L_d = max(|g - o|, 1e-6)
      if (r.pick_cap) {
        if (!r.Ldeff_floor)
          Asp += 1.2f * ALdeff;
      } else if (!r.Ld_floor) {
        const float k = ALdeff / r.Ld_raw;
        Aox += -k * r.tgx;
        Aoy += -k * r.tgy;
      }
      // sa = sin(ang), ang = atan2(tgy, tgx) - th, tg = g - o
      const float Aang = Asa * std::cos(r.ang);
      Ath += -Aang;
      if (r.tg_r2 > 0.0f) {
        Aox -= Aang * (-r.tgy / r.tg_r2);
        Aoy -= Aang * (r.tgx / r.tg_r2);
      }
    }
  }

  // F = Fg + Fgoal + Fm
  AFgx += AFx;
  AFgy += AFy;
  AFmx += AFx;
  AFmy += AFy;
  // Fgoal = -beta * (o - g)
  gbe += -(AFx * (ox - a.gx) + AFy * (oy - a.gy));
  Aox += -a.be * AFx;
  Aoy += -a.be * AFy;
  // Fm = -ls * gr + (-lh * db) * gp
  gls += -(AFmx * r.grx + AFmy * r.gry);
  const float Agrx = -a.ls * AFmx, Agry = -a.ls * AFmy;
  glh += -(AFmx * r.gpx + AFmy * r.gpy) * r.db;
  const float Adb = -a.lh * (AFmx * r.gpx + AFmy * r.gpy);
  const float Agpx = (-a.lh * r.db) * AFmx, Agpy = (-a.lh * r.db) * AFmy;
  const float Asdf = Adb * detail::sdf_barrier_db_grad(r.sdf, a.k_sharp, a.d_hat_sdf);

  // head = (cos th, sin th)
  Ath += Ach * (-sh) + Ash * ch;

  // Geometry: Fg = sum -(alpha_j b_j) n_j, Fr = sum -(alpha_j min(b_j, 0)) n_j, and
  // the governor reads d and n from the nearest disc.
  for (int j = 0; j < a.N; ++j) {
    if (!a.mb[j])
      continue;
    const float dx = ox - a.Cb[2 * j], dy = oy - a.Cb[2 * j + 1];
    const float r0 = std::sqrt(dx * dx + dy * dy);
    const float rad = r0 < 1e-9f ? 1e-9f : r0;
    const float nhx = dx / rad, nhy = dy / rad;
    const float dj = rad - (a.Rb[j] + a.rr_eff);
    const float bj = detail::ipc_dbdd_pw(dj, a.dh);
    const float brep = bj < 0.0f ? bj : 0.0f;
    const float pg = AFgx * nhx + AFgy * nhy, pr = AFrx * nhx + AFry * nhy;
    gal[j] += -pg * bj - pr * brep;
    float Abj = -a.ab[j] * pg;
    if (bj < 0.0f)
      Abj += -a.ab[j] * pr;
    float Anx = -(a.ab[j] * bj) * AFgx - (a.ab[j] * brep) * AFrx;
    float Any = -(a.ab[j] * bj) * AFgy - (a.ab[j] * brep) * AFry;
    float Adj = Abj * detail::ipc_dbdd_pw_grad(dj, a.dh);
    if (j == r.jmin) {
      Adj += Ad;
      Anx += Angx;
      Any += Angy;
    }
    Aox += Adj * nhx; // d d_j / d o = n_j
    Aoy += Adj * nhy;
    if (r0 > 1e-9f) { // the position Jacobian of n_j, (I - n n^T) / r
      const float dot = nhx * Anx + nhy * Any;
      Aox += (Anx - nhx * dot) / rad;
      Aoy += (Any - nhy * dot) / rad;
    }
  }

  // The patch samples depend on off = o - o0.
  const float Arisk_raw = (r.risk_raw > 0.0f && r.risk_raw < 1.0f) ? Arisk : 0.0f;
  const float Asdf_raw = (r.sdf_raw > 0.0f && r.sdf_raw < 50.0f) ? Asdf : 0.0f;
  const float gcs[6] = {Arisk_raw, Asdf_raw, Agrx, Agry, Agpx, Agpy};
  for (int c = 0; c < 6; ++c) {
    float jx, jy;
    detail::patch_sample_channel_grad(a.chan[c], a.Hp, a.Wp, r.offx, r.offy, jx, jy);
    Aox += gcs[c] * jx;
    Aoy += gcs[c] * jy;
  }

  aox = Aox;
  aoy = Aoy;
  ath = Ath;
  asp = Asp;
}

agent make_agent(int b, const float *o0, const float *goal, const float *C, const float *R,
                 const std::uint8_t *mask, const float *alphas, const float *beta,
                 const float *gamma, const float *lam_soft, const float *lam_hard,
                 const float *rollout_patch, const float *rr, const float *d_hat, const float *dt,
                 int N, int Hp, int Wp, const surrogate_material_params &p,
                 const bicycle_material_params &v) {
  agent a;
  const long chan = static_cast<long>(Hp) * Wp;
  const float *pr = rollout_patch + static_cast<long>(b) * 6 * chan;
  for (int c = 0; c < 6; ++c)
    a.chan[c] = pr + c * chan;
  a.Cb = C + static_cast<long>(b) * N * 2;
  a.Rb = R + static_cast<long>(b) * N;
  a.mb = mask + static_cast<long>(b) * N;
  a.ab = alphas + static_cast<long>(b) * N;
  a.N = N;
  a.Hp = Hp;
  a.Wp = Wp;
  a.o0x = o0[2 * b];
  a.o0y = o0[2 * b + 1];
  a.gx = goal[2 * b];
  a.gy = goal[2 * b + 1];
  a.be = beta[b];
  a.ga = gamma[b];
  a.ls = lam_soft[b];
  a.lh = lam_hard[b];
  a.dh = d_hat[b];
  a.hdt = dt[b];
  a.rr_eff = p.margin_factor * rr[b];
  a.mass = p.mass;
  a.k_sharp = p.k_sharp;
  a.d_hat_sdf = p.d_hat_sdf;
  a.L = v.L;
  a.dmax = v.delta_max;
  a.tan_dmax = std::tan(a.dmax);
  a.a_max = v.a_max;
  a.a_lat = v.a_lat_max;
  a.k_steer = v.k_steer;
  a.vmax = v.vmax;
  a.allow_reverse = v.allow_reverse;
  a.sp_min = v.allow_reverse ? -0.25f * v.vmax : 0.0f;
  a.v_creep_cap = 0.5f * std::sqrt(a.a_lat * a.L / a.tan_dmax);
  a.kfloor = a.tan_dmax / (a.L * 400.0f);
  return a;
}

int max_horizon(const int *H, int B) {
  int m = 0;
  for (int b = 0; b < B; ++b)
    m = std::max(m, H[b]);
  return m;
}

} // namespace

void validate(const bicycle_material_params &v) {
  const auto pos = [](float x) { return std::isfinite(x) && x > 0.0f; };
  if (!pos(v.L) || !pos(v.a_max) || !pos(v.a_lat_max) || !pos(v.vmax) ||
      !(v.delta_max > 0.0f && v.delta_max < 1.5707963f) || !std::isfinite(v.k_steer) ||
      v.k_steer < 0.0f)
    throw std::invalid_argument("bicycle_material_params: L, a_max, a_lat_max and vmax must be "
                                "finite and > 0, delta_max in (0, pi/2), k_steer finite and >= 0");
}

void integrate_bicycle_material(float *o, float *th, float *sp, const float *goal, const float *C,
                                const float *R, const std::uint8_t *mask, const float *alphas,
                                const float *beta, const float *gamma, const float *lam_soft,
                                const float *lam_hard, const float *rollout_patch, const float *rr,
                                const float *d_hat, const float *dt, const int *H, int B, int N,
                                int Hp, int Wp, const surrogate_material_params &p,
                                const bicycle_material_params &v, float *min_clear, float *cum_risk,
                                float *hard_count, float *arc_length, int num_threads) {
  validate(v);
  const int max_H = max_horizon(H, B);
  // The patch is centred on the start, so keep the start before o is overwritten.
  const std::vector<float> o0(o, o + 2 * B);
  detail::parallel_for(B, num_threads, [&](int b) {
    const agent a = make_agent(b, o0.data(), goal, C, R, mask, alphas, beta, gamma, lam_soft,
                               lam_hard, rollout_patch, rr, d_hat, dt, N, Hp, Wp, p, v);
    state st{o[2 * b], o[2 * b + 1], th[b], sp[b]};
    float minclr = std::numeric_limits<float>::infinity();
    float cum = 0.0f, hard = 0.0f, arc = 0.0f;
    for (int s = 0; s < max_H; ++s) {
      rec r;
      r.active = s < H[b];
      float dmin;
      int jmin;
      const state next = step_fwd(a, st, r, dmin, jmin);
      if (dmin < minclr)
        minclr = dmin; // ungated, as in the surrogate
      const float active = r.active ? 1.0f : 0.0f;
      arc += active * r.disp;
      cum += active * r.risk * r.disp;
      hard += active * (r.sdf < 1.0f ? 1.0f : 0.0f);
      st = next;
    }
    o[2 * b] = st.ox;
    o[2 * b + 1] = st.oy;
    th[b] = st.th;
    sp[b] = st.sp;
    min_clear[b] = minclr;
    cum_risk[b] = cum;
    hard_count[b] = hard;
    arc_length[b] = arc;
  });
}

void integrate_bicycle_material_vjp(
    const float *o0, const float *th0, const float *sp0, const float *goal, const float *C,
    const float *R, const std::uint8_t *mask, const float *alphas, const float *beta,
    const float *gamma, const float *lam_soft, const float *lam_hard, const float *rollout_patch,
    const float *rr, const float *d_hat, const float *dt, const int *H, int B, int N, int Hp,
    int Wp, const surrogate_material_params &p, const bicycle_material_params &v, const float *g_oT,
    const float *g_thT, const float *g_spT, const float *g_min_clear, const float *g_cum_risk,
    const float *g_arc_length, float *g_alphas, float *g_beta, float *g_gamma, float *g_lam_soft,
    float *g_lam_hard, int num_threads) {
  validate(v);
  const int max_H = max_horizon(H, B);
  detail::parallel_for(B, num_threads, [&](int b) {
    const agent a = make_agent(b, o0, goal, C, R, mask, alphas, beta, gamma, lam_soft, lam_hard,
                               rollout_patch, rr, d_hat, dt, N, Hp, Wp, p, v);
    // Forward recompute, keeping each step's input state and the min_clear argmin
    // step (the first strictly smaller clearance, as the forward takes it).
    std::vector<state> S(static_cast<size_t>(max_H) + 1);
    S[0] = {o0[2 * b], o0[2 * b + 1], th0[b], sp0[b]};
    float minclr = std::numeric_limits<float>::infinity();
    int s_star = -1;
    for (int s = 0; s < max_H; ++s) {
      rec r;
      r.active = s < H[b];
      float dmin;
      int jmin;
      S[s + 1] = step_fwd(a, S[s], r, dmin, jmin);
      if (dmin < minclr) {
        minclr = dmin;
        s_star = jmin >= 0 ? s : -1;
      }
    }

    float aox = g_oT ? g_oT[2 * b] : 0.0f, aoy = g_oT ? g_oT[2 * b + 1] : 0.0f;
    float ath = g_thT ? g_thT[b] : 0.0f, asp = g_spT ? g_spT[b] : 0.0f;
    const float gmin = g_min_clear ? g_min_clear[b] : 0.0f;
    const float gcum = g_cum_risk ? g_cum_risk[b] : 0.0f;
    const float garc = g_arc_length ? g_arc_length[b] : 0.0f;
    float gbe = 0.0f, gga = 0.0f, gls = 0.0f, glh = 0.0f;
    float *gal = g_alphas + static_cast<long>(b) * N;
    for (int s = max_H - 1; s >= 0; --s) {
      rec r;
      r.active = s < H[b];
      float dmin;
      int jmin;
      step_fwd(a, S[s], r, dmin, jmin); // rebuild the step record
      step_bwd(a, S[s], r, garc, gcum, aox, aoy, ath, asp, gal, gbe, gga, gls, glh);
      if (s == s_star) { // min_clear = d of the nearest disc at o_s
        aox += gmin * r.ngx;
        aoy += gmin * r.ngy;
      }
    }
    g_beta[b] += gbe;
    g_gamma[b] += gga;
    g_lam_soft[b] += gls;
    g_lam_hard[b] += glh;
  });
}

} // namespace nav
} // namespace cvc
