/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// diff_rollout.h — the DIFFERENTIABLE rollout primitives shared by the CPU and
// CUDA self-supervised trainers (coef_train.cpp / coef_train.cu). Every function
// is `__host__ __device__` and uses C-style float math (sqrtf/fminf/…), so the
// EXACT SAME source compiles for both the host trainer and the device kernel —
// the CPU finite-difference gradcheck therefore validates the CUDA backward too
// (write the delicate adjoint once, both paths inherit it correct).
//
// Two rollout integrators, selectable per training run (train_config::rollout):
//   * SURROGATE — the smooth point-mass `sdf_rollout` (sdf_nav.py): sample -> IPC
//     wall force + goal spring + damping -> integrate. State is (o, v). The
//     default; its gradient is clean, which is why coef_train.py trains on it.
//   * BICYCLE   — the deployment kinematic-bicycle integrator (bicycle_rollout,
//     drive.cpp), differentiated in full (atan2/tan/tanh + the corner/stop/creep/
//     reverse governors). State is (o, th, sp). Training on it closes the
//     surrogate→deployment gap at the cost of the bicycle's non-smooth branches
//     (their selection is measure-zero; the gradient flows through the taken
//     branch, exactly as autograd handles min/max/clamp/where).
//
// A "step" is ONE substep (nsub=1) for both, matching coef_train.py's training
// rollout; deployment nsub (e.g. 2) is a runtime concern the trained coefficients
// transfer across.

#ifndef __CVC_NAV_DIFF_ROLLOUT_H__
#define __CVC_NAV_DIFF_ROLLOUT_H__

#include <math.h>

#if defined(__CUDACC__)
#define CVC_HD __host__ __device__
#else
#define CVC_HD
#endif

namespace cvc {
namespace nav {
namespace diff {

// A lightweight float view of the SDF field (built from field_stack on the host,
// or directly in a kernel) — the shared sampler reads this, not field_stack, so
// host and device pass the identical struct.
struct field {
  const float *data; // [3*H*W] : phi, normal_x, normal_y planes
  int H, W;
  float S, cx, cy, mnx, mny, mxx, mxy;
};

// A cached bilinear sample (drive.cpp sample_unit) + everything its position VJP
// needs. `phi` and the unit normal (nx, ny) are the forward outputs.
struct sample {
  float phv[4], pxv[4], pyv[4]; // corner values (nw, ne, sw, se) per channel
  float wx0, wx1, wy0, wy1;
  int clx, cly;   // ix / iy clamped to the border
  float Wf1, Hf1; // W-1, H-1
  float cgx, cgy; // d gx/d onx, d gy/d ony
  float rnx, rny; // raw (pre-renorm) normal
  float r, mag;   // |raw|, |raw|+1e-6
  float phi, nx, ny;
};

CVC_HD inline float sigmoidf_(float x) { return 1.0f / (1.0f + expf(-x)); }
CVC_HD inline float siluf_(float x) { return x * sigmoidf_(x); }
CVC_HD inline float silu_grad_(float x) {
  const float s = sigmoidf_(x);
  return s + x * s * (1.0f - s);
}
CVC_HD inline float softplusf_(float x) { return x > 20.0f ? x : log1pf(expf(x)); }

// IPC barrier derivative (ipc_dbdd, drive.cpp) and its d/dd.
CVC_HD inline float ipc_(float d, float d_hat) {
  const float dc = d < 1e-6f ? 1e-6f : d;
  if (!(dc < d_hat))
    return 0.0f;
  return (d_hat - dc) * (2.0f * logf(dc / d_hat) - d_hat / dc) + 1.0f;
}
CVC_HD inline float ipc_grad_(float d, float d_hat) {
  if (d < 1e-6f)
    return 0.0f;
  if (!(d < d_hat))
    return 0.0f;
  const float A = d_hat - d;
  const float B = 2.0f * logf(d / d_hat) - d_hat / d;
  const float dB = 2.0f / d + d_hat / (d * d);
  return -B + A * dB;
}

CVC_HD inline sample sample_fwd(const field &f, float onx, float ony) {
  sample s;
  const float wx = onx / f.S + f.cx, wy = ony / f.S + f.cy;
  const float gx = 2.0f * (wx - f.mnx) / (f.mxx - f.mnx) - 1.0f;
  const float gy = 2.0f * (wy - f.mny) / (f.mxy - f.mny) - 1.0f;
  s.cgx = 2.0f / ((f.mxx - f.mnx) * f.S);
  s.cgy = 2.0f / ((f.mxy - f.mny) * f.S);
  s.Wf1 = f.W - 1;
  s.Hf1 = f.H - 1;
  float ix = (gx + 1.0f) * 0.5f * s.Wf1;
  float iy = (gy + 1.0f) * 0.5f * s.Hf1;
  s.clx = (ix < 0.0f) || (ix > s.Wf1);
  s.cly = (iy < 0.0f) || (iy > s.Hf1);
  ix = fminf(fmaxf(ix, 0.0f), s.Wf1);
  iy = fminf(fmaxf(iy, 0.0f), s.Hf1);
  const int ix0 = (int)floorf(ix), iy0 = (int)floorf(iy);
  s.wx1 = ix - ix0;
  s.wx0 = 1.0f - s.wx1;
  s.wy1 = iy - iy0;
  s.wy0 = 1.0f - s.wy1;
  const int cx0 = ix0 < 0 ? 0 : (ix0 > f.W - 1 ? f.W - 1 : ix0);
  const int cx1n = ix0 + 1, cx1 = cx1n < 0 ? 0 : (cx1n > f.W - 1 ? f.W - 1 : cx1n);
  const int cy0 = iy0 < 0 ? 0 : (iy0 > f.H - 1 ? f.H - 1 : iy0);
  const int cy1n = iy0 + 1, cy1 = cy1n < 0 ? 0 : (cy1n > f.H - 1 ? f.H - 1 : cy1n);
  const long HW = (long)f.H * f.W;
  const float *ph = f.data, *px = f.data + HW, *py = f.data + 2 * HW;
  const long nw = (long)cy0 * f.W + cx0, ne = (long)cy0 * f.W + cx1;
  const long sw = (long)cy1 * f.W + cx0, se = (long)cy1 * f.W + cx1;
  s.phv[0] = ph[nw];
  s.phv[1] = ph[ne];
  s.phv[2] = ph[sw];
  s.phv[3] = ph[se];
  s.pxv[0] = px[nw];
  s.pxv[1] = px[ne];
  s.pxv[2] = px[sw];
  s.pxv[3] = px[se];
  s.pyv[0] = py[nw];
  s.pyv[1] = py[ne];
  s.pyv[2] = py[sw];
  s.pyv[3] = py[se];
  const float nwW = s.wx0 * s.wy0, neW = s.wx1 * s.wy0, swW = s.wx0 * s.wy1, seW = s.wx1 * s.wy1;
  s.phi = s.phv[0] * nwW + s.phv[1] * neW + s.phv[2] * swW + s.phv[3] * seW;
  s.rnx = s.pxv[0] * nwW + s.pxv[1] * neW + s.pxv[2] * swW + s.pxv[3] * seW;
  s.rny = s.pyv[0] * nwW + s.pyv[1] * neW + s.pyv[2] * swW + s.pyv[3] * seW;
  s.r = sqrtf(s.rnx * s.rnx + s.rny * s.rny);
  s.mag = s.r + 1e-6f;
  s.nx = s.rnx / s.mag;
  s.ny = s.rny / s.mag;
  return s;
}

// VJP of sample_fwd: (dL/dphi, dL/dnx, dL/dny) -> (dL/donx, dL/dony).
CVC_HD inline void sample_bwd(const sample &s, float gphi, float gnx, float gny, float &gonx,
                              float &gony) {
  float grnx = 0.0f, grny = 0.0f;
  if (s.r > 0.0f) {
    const float mag2 = s.mag * s.mag;
    const float dnx_drnx = (s.mag - s.rnx * s.rnx / s.r) / mag2;
    const float dny_drny = (s.mag - s.rny * s.rny / s.r) / mag2;
    const float dcross = -(s.rnx * s.rny) / (s.r * mag2);
    grnx = gnx * dnx_drnx + gny * dcross;
    grny = gnx * dcross + gny * dny_drny;
  }
  const float dphi_dix = s.wy0 * (s.phv[1] - s.phv[0]) + s.wy1 * (s.phv[3] - s.phv[2]);
  const float dphi_diy = s.wx0 * (s.phv[2] - s.phv[0]) + s.wx1 * (s.phv[3] - s.phv[1]);
  const float drnx_dix = s.wy0 * (s.pxv[1] - s.pxv[0]) + s.wy1 * (s.pxv[3] - s.pxv[2]);
  const float drnx_diy = s.wx0 * (s.pxv[2] - s.pxv[0]) + s.wx1 * (s.pxv[3] - s.pxv[1]);
  const float drny_dix = s.wy0 * (s.pyv[1] - s.pyv[0]) + s.wy1 * (s.pyv[3] - s.pyv[2]);
  const float drny_diy = s.wx0 * (s.pyv[2] - s.pyv[0]) + s.wx1 * (s.pyv[3] - s.pyv[1]);
  const float gix = gphi * dphi_dix + grnx * drnx_dix + grny * drny_dix;
  const float giy = gphi * dphi_diy + grnx * drnx_diy + grny * drny_diy;
  const float ggx = gix * (s.clx ? 0.0f : 0.5f * s.Wf1);
  const float ggy = giy * (s.cly ? 0.0f : 0.5f * s.Hf1);
  gonx = ggx * s.cgx;
  gony = ggy * s.cgy;
}

// ── SURROGATE (point-mass) rollout step ──────────────────────────────────────
// State (o, v). One step: sample -> IPC force + goal spring + damping ->
// integrate with a vmax speed clamp.
CVC_HD inline void surr_step(const field &f, float ox, float oy, float vx, float vy, float gx,
                             float gy, float al, float be, float ga, float rr, float d_hat,
                             float vmax, float hdt, float &ox1, float &oy1, float &vx1,
                             float &vy1) {
  const sample s = sample_fwd(f, ox, oy);
  const float ipc = ipc_(s.phi - rr, d_hat);
  const float ax = -(al * ipc) * s.nx - be * (ox - gx) - ga * vx;
  const float ay = -(al * ipc) * s.ny - be * (oy - gy) - ga * vy;
  const float vpx = vx + hdt * ax, vpy = vy + hdt * ay;
  const float sp = sqrtf(vpx * vpx + vpy * vpy);
  float vcx = vpx, vcy = vpy;
  if (sp > vmax) {
    const float sc = vmax / sp;
    vcx = vpx * sc;
    vcy = vpy * sc;
  }
  vx1 = vcx;
  vy1 = vcy;
  ox1 = ox + hdt * vcx;
  oy1 = oy + hdt * vcy;
}

// Backward of surr_step: grad on (o1, v1) -> grad on (o, v) and (al, be, ga).
CVC_HD inline void surr_step_bwd(const field &f, float ox, float oy, float vx, float vy, float gx,
                                 float gy, float al, float be, float ga, float rr, float d_hat,
                                 float vmax, float hdt, float go1x, float go1y, float gv1x,
                                 float gv1y, float &gox, float &goy, float &gvx, float &gvy,
                                 float &gal, float &gbe, float &gga) {
  const sample s = sample_fwd(f, ox, oy);
  const float d = s.phi - rr;
  const float ipc = ipc_(d, d_hat);
  const float vpx = vx + hdt * (-(al * ipc) * s.nx - be * (ox - gx) - ga * vx);
  const float vpy = vy + hdt * (-(al * ipc) * s.ny - be * (oy - gy) - ga * vy);
  const float sp = sqrtf(vpx * vpx + vpy * vpy);
  const int scaled = sp > vmax;

  const float gvpp_x = gv1x + hdt * go1x, gvpp_y = gv1y + hdt * go1y;
  gox = go1x;
  goy = go1y;
  float gvp_x, gvp_y;
  if (scaled) {
    const float sp3 = sp * sp * sp;
    const float dot = gvpp_x * vpx + gvpp_y * vpy;
    gvp_x = vmax / sp * gvpp_x - vmax / sp3 * vpx * dot;
    gvp_y = vmax / sp * gvpp_y - vmax / sp3 * vpy * dot;
  } else {
    gvp_x = gvpp_x;
    gvp_y = gvpp_y;
  }
  const float gax = hdt * gvp_x, gay = hdt * gvp_y;
  gvx = gvp_x;
  gvy = gvp_y;
  gal = 0.0f;
  gbe = 0.0f;
  gga = 0.0f;
  gga += -(gax * vx + gay * vy);
  gvx += -ga * gax;
  gvy += -ga * gay;
  gbe += -(gax * (ox - gx) + gay * (oy - gy));
  gox += -be * gax;
  goy += -be * gay;
  gal += -ipc * (gax * s.nx + gay * s.ny);
  const float g_ipc = -al * (gax * s.nx + gay * s.ny);
  const float gnx = -(al * ipc) * gax, gny = -(al * ipc) * gay;
  const float gphi = g_ipc * ipc_grad_(d, d_hat);
  float sdox, sdoy;
  sample_bwd(s, gphi, gnx, gny, sdox, sdoy);
  gox += sdox;
  goy += sdoy;
}

// ── BICYCLE rollout step (deployment integrator, differentiated) ─────────────
// State (o, th, sp). Forward = one substep of bicycle_rollout (drive.cpp).
struct bike_veh {
  float rr, d_hat, vmax, L, delta_max, a_max, a_lat_max, k_steer, hdt;
  int allow_reverse;
};

CVC_HD inline void bike_step(const field &f, float ox, float oy, float thi, float spi, float gx,
                             float gy, float al, float be, float ga, const bike_veh &v, float &ox1,
                             float &oy1, float &thi1, float &spi1) {
  const float rr = v.rr, d_hat = v.d_hat, vmax = v.vmax, L = v.L, dmax = v.delta_max;
  const float a_max = v.a_max, a_lat_max = v.a_lat_max, k_steer = v.k_steer, hdt = v.hdt;
  const float tan_dmax = tanf(dmax);
  const float sp_min = v.allow_reverse ? -0.25f * vmax : 0.0f;
  const float v_creep_cap = 0.5f * sqrtf(a_lat_max * L / tan_dmax);
  const float kfloor = tan_dmax / (L * 400.0f);

  const sample s = sample_fwd(f, ox, oy);
  const float d = s.phi - rr;
  const float ipc = ipc_(d, d_hat);
  const float Fbar_x = -(al * ipc) * s.nx, Fbar_y = -(al * ipc) * s.ny;
  const float Fgoal_x = -be * (ox - gx), Fgoal_y = -be * (oy - gy);
  const float Fx = Fbar_x + Fgoal_x, Fy = Fbar_y + Fgoal_y;
  const float ch = cosf(thi), sh = sinf(thi);
  float a_long = fminf(fmaxf(Fx * ch + Fy * sh - ga * spi, -a_max), a_max);

  const float tgx = gx - ox, tgy = gy - oy;
  float L_d = sqrtf(tgx * tgx + tgy * tgy);
  if (L_d < 1e-6f)
    L_d = 1e-6f;
  const float ang = atan2f(tgy, tgx) - thi;
  const float sin_a = sinf(ang), cos_a = cosf(ang);
  const int behind = cos_a < 0.0f;
  const float turn_sign = sin_a >= 0.0f ? 1.0f : -1.0f;
  float L_d_eff = 1.2f * spi + 4.0f * L;
  if (L_d_eff < 4.0f * L)
    L_d_eff = 4.0f * L;
  L_d_eff = fminf(L_d, L_d_eff);
  float delta = behind ? turn_sign * dmax : atan2f(2.0f * L * sin_a, L_d_eff);
  const float ipc_rep = ipc < 0.0f ? ipc : 0.0f;
  const float Frep_x = -(al * ipc_rep) * s.nx, Frep_y = -(al * ipc_rep) * s.ny;
  delta = delta + k_steer * tanhf(Frep_x * (-sh) + Frep_y * ch);
  delta = fminf(fmaxf(delta, -dmax), dmax);

  float kappa = fabsf(tanf(delta)) / L;
  if (kappa < kfloor)
    kappa = kfloor;
  const float v_corner = sqrtf(a_lat_max / kappa);
  float ds = d - 0.5f * rr;
  if (ds < 0.0f)
    ds = 0.0f;
  const float v_stop = sqrtf(2.0f * a_max * ds);
  const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
  float approach = fminf(fmaxf(-(s.nx * ch + s.ny * sh) * motion_sign, 0.0f), 1.0f);
  const float v_stop_dir = approach > 0.05f ? v_stop / fmaxf(approach, 0.05f) : vmax;
  float v_lim = fminf(vmax, fminf(v_corner, v_stop_dir));
  const int hard_steer = fabsf(delta) >= 0.7f * dmax;
  const int can_move = d > 0.25f * rr;
  const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
  v_lim = fmaxf(v_lim, v_floor);
  const float v_creep = fminf(0.5f * v_corner, v_creep_cap);
  if (behind) {
    float fbh = Fbar_x * ch + Fbar_y * sh;
    if (fbh > 0.0f)
      fbh = 0.0f;
    a_long = (v_creep - spi) / hdt + fbh;
  }
  if (hard_steer && can_move && fabsf(spi) < 0.06f)
    a_long = fmaxf(a_long, (0.08f - spi) / hdt);
  if (v.allow_reverse) {
    const float head_on = fminf(fmaxf(-(s.nx * ch + s.ny * sh), 0.0f), 1.0f);
    if (behind && (head_on > 0.6f) && (d < 0.5f * rr + 0.02f)) {
      a_long = (-0.10f - spi) / hdt;
      delta = -delta;
    }
  }
  a_long = fminf(fmaxf(a_long, -a_max), a_max);
  a_long = fminf(a_long, (v_lim - spi) / hdt);
  a_long = fminf(fmaxf(a_long, -a_max), a_max);
  spi1 = fminf(fmaxf(spi + hdt * a_long, sp_min), vmax);
  float sp2 = spi1 * spi1;
  if (sp2 < 1e-9f)
    sp2 = 1e-9f;
  const float d_cap = atanf(a_lat_max * L / sp2);
  delta = fminf(fmaxf(delta, -d_cap), d_cap);
  thi1 = thi + hdt * (spi1 / L) * tanf(delta);
  ox1 = ox + hdt * spi1 * cosf(thi1);
  oy1 = oy + hdt * spi1 * sinf(thi1);
}

// Backward of bike_step: recompute the forward (capturing intermediates + which
// branch/clamp each conditional took), then one strict reverse pass. Grad flows
// through the branch that was TAKEN (its selection is measure-zero); a finite-
// difference gradcheck at interior points validates it.
CVC_HD inline void bike_step_bwd(const field &f, float ox, float oy, float thi, float spi, float gx,
                                 float gy, float al, float be, float ga, const bike_veh &v,
                                 float go1x, float go1y, float gth1, float gsp1, float &gox,
                                 float &goy, float &gth, float &gsp, float &gal, float &gbe,
                                 float &gga) {
  const float rr = v.rr, d_hat = v.d_hat, vmax = v.vmax, L = v.L, dmax = v.delta_max;
  const float a_max = v.a_max, a_lat_max = v.a_lat_max, k_steer = v.k_steer, hdt = v.hdt;
  const float tan_dmax = tanf(dmax);
  const float sp_min = v.allow_reverse ? -0.25f * vmax : 0.0f;
  const float v_creep_cap = 0.5f * sqrtf(a_lat_max * L / tan_dmax);
  const float kfloor = tan_dmax / (L * 400.0f);

  // ── recompute forward, keeping intermediates ──
  const sample s = sample_fwd(f, ox, oy);
  const float d = s.phi - rr;
  const float ipc = ipc_(d, d_hat);
  const float Fbar_x = -(al * ipc) * s.nx, Fbar_y = -(al * ipc) * s.ny;
  const float Fgoal_x = -be * (ox - gx), Fgoal_y = -be * (oy - gy);
  const float Fx = Fbar_x + Fgoal_x, Fy = Fbar_y + Fgoal_y;
  const float ch = cosf(thi), sh = sinf(thi);
  const float aL0 = Fx * ch + Fy * sh - ga * spi;
  const int c1 = aL0 > -a_max && aL0 < a_max; // a_long clamp #1 passthrough
  const float aL1 = fminf(fmaxf(aL0, -a_max), a_max);
  const float tgx = gx - ox, tgy = gy - oy;
  const float Ld0 = sqrtf(tgx * tgx + tgy * tgy);
  const int ld_pass = Ld0 >= 1e-6f;
  const float L_d = ld_pass ? Ld0 : 1e-6f;
  const float ang = atan2f(tgy, tgx) - thi;
  const float sin_a = sinf(ang), cos_a = cosf(ang);
  const int behind = cos_a < 0.0f;
  const float turn_sign = sin_a >= 0.0f ? 1.0f : -1.0f;
  const float Lde0 = 1.2f * spi + 4.0f * L;
  const int lde1_pass = Lde0 >= 4.0f * L;
  const float Lde1 = lde1_pass ? Lde0 : 4.0f * L;
  const int lde_useLd = L_d <= Lde1;
  const float Lde = lde_useLd ? L_d : Lde1;
  const float delta0 = behind ? turn_sign * dmax : atan2f(2.0f * L * sin_a, Lde);
  const int iprep_pass = ipc < 0.0f;
  const float ipc_rep = iprep_pass ? ipc : 0.0f;
  const float Frep_x = -(al * ipc_rep) * s.nx, Frep_y = -(al * ipc_rep) * s.ny;
  const float targ = Frep_x * (-sh) + Frep_y * ch;
  const float tanh_t = tanhf(targ);
  const float delta1 = delta0 + k_steer * tanh_t;
  const int c2 = delta1 > -dmax && delta1 < dmax;
  const float delta2 = fminf(fmaxf(delta1, -dmax), dmax);
  const float td2 = tanf(delta2);
  const float kappa0 = fabsf(td2) / L;
  const int kf_pass = kappa0 >= kfloor;
  const float kappa = kf_pass ? kappa0 : kfloor;
  const float v_corner = sqrtf(a_lat_max / kappa);
  const float ds0 = d - 0.5f * rr;
  const int ds_pass = ds0 >= 0.0f;
  const float ds = ds_pass ? ds0 : 0.0f;
  const float v_stop = sqrtf(2.0f * a_max * ds);
  const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
  const float appr0 = -(s.nx * ch + s.ny * sh) * motion_sign;
  const int appr_pass = appr0 > 0.0f && appr0 < 1.0f;
  const float approach = fminf(fmaxf(appr0, 0.0f), 1.0f);
  const int vsd_active = approach > 0.05f; // v_stop_dir uses v_stop/approach
  const float v_stop_dir = vsd_active ? v_stop / approach : vmax;
  const int inner_useCorner = v_corner <= v_stop_dir;
  const float inner = inner_useCorner ? v_corner : v_stop_dir;
  const int vlim0_useInner = inner < vmax;
  const float v_lim0 = vlim0_useInner ? inner : vmax;
  const int hard_steer = fabsf(delta2) >= 0.7f * dmax;
  const int can_move = d > 0.25f * rr;
  const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
  const int vlim1_useFloor = v_floor > v_lim0;
  const float v_lim1 = vlim1_useFloor ? v_floor : v_lim0;
  const int vcreep_useCorner = 0.5f * v_corner < v_creep_cap;
  const float v_creep = vcreep_useCorner ? 0.5f * v_corner : v_creep_cap;
  // behind branch
  const float fbh0 = Fbar_x * ch + Fbar_y * sh;
  const int fbh_pass = fbh0 <= 0.0f;
  const float fbh = fbh_pass ? fbh0 : 0.0f;
  const float aL_bh = behind ? (v_creep - spi) / hdt + fbh : aL1;
  // stuck branch
  const int stuck = hard_steer && can_move && fabsf(spi) < 0.06f;
  const float stuck_floor = (0.08f - spi) / hdt;
  const int stuck_useFloor = stuck && (stuck_floor > aL_bh);
  const float aL_st = stuck ? fmaxf(aL_bh, stuck_floor) : aL_bh;
  // reverse / nose-blocked branch
  int nose_blocked = 0;
  if (v.allow_reverse) {
    const float head_on = fminf(fmaxf(-(s.nx * ch + s.ny * sh), 0.0f), 1.0f);
    nose_blocked = behind && (head_on > 0.6f) && (d < 0.5f * rr + 0.02f);
  }
  const float aL_ns = nose_blocked ? (-0.10f - spi) / hdt : aL_st;
  const float delta3 = nose_blocked ? -delta2 : delta2;
  const int c3 = aL_ns > -a_max && aL_ns < a_max;
  const float aLc = fminf(fmaxf(aL_ns, -a_max), a_max);
  const float vlim_bound = (v_lim1 - spi) / hdt;
  const int aLd_useC = aLc <= vlim_bound;
  const float aLd = aLd_useC ? aLc : vlim_bound;
  const int c4 = aLd > -a_max && aLd < a_max;
  const float aLe = fminf(fmaxf(aLd, -a_max), a_max);
  const float spi1_0 = spi + hdt * aLe;
  const int c5 = spi1_0 > sp_min && spi1_0 < vmax;
  const float spi1 = fminf(fmaxf(spi1_0, sp_min), vmax);
  float sp2 = spi1 * spi1;
  const int sp2_pass = sp2 >= 1e-9f;
  if (!sp2_pass)
    sp2 = 1e-9f;
  const float dcap_arg = a_lat_max * L / sp2;
  const float d_cap = atanf(dcap_arg);
  const int c6 = delta3 > -d_cap && delta3 < d_cap;
  const float delta4 = fminf(fmaxf(delta3, -d_cap), d_cap);
  const float td4 = tanf(delta4);
  const float thi1 = thi + hdt * (spi1 / L) * td4;
  const float ch2 = cosf(thi1), sh2 = sinf(thi1);

  // ── reverse pass ──
  gox = goy = gth = gsp = gal = gbe = gga = 0.0f;
  float g_al = 0.0f;
  float g_ch = 0.0f, g_sh = 0.0f, g_nx = 0.0f, g_ny = 0.0f, g_phi = 0.0f, g_d = 0.0f, g_ipc = 0.0f;
  float g_Fbar_x = 0.0f, g_Fbar_y = 0.0f;
  float g_spi1 = gsp1; // spi1 is an output: seed its grad (was dropped -> the bug)
  float g_delta4 = 0.0f, g_delta3 = 0.0f, g_delta2 = 0.0f, g_delta0 = 0.0f;
  float g_v_corner = 0.0f, g_v_lim0 = 0.0f, g_v_lim1 = 0.0f, g_v_stop = 0.0f, g_ds = 0.0f;
  float g_appr = 0.0f, g_kappa = 0.0f, g_targ = 0.0f, g_Lde = 0.0f, g_sin_a = 0.0f, g_ang = 0.0f;
  float g_tgx = 0.0f, g_tgy = 0.0f, g_Ld = 0.0f, g_aL0 = 0.0f, g_aL1 = 0.0f, g_aL_bh = 0.0f;
  float g_aL_st = 0.0f, g_aL_ns = 0.0f, g_aLc = 0.0f, g_aLd = 0.0f, g_aLe = 0.0f, g_sp2 = 0.0f;
  float g_Fx = 0.0f, g_Fy = 0.0f, g_Fgoal_x = 0.0f, g_Fgoal_y = 0.0f, g_Frep_x = 0.0f,
        g_Frep_y = 0.0f, g_ipc_rep = 0.0f, g_v_creep = 0.0f, g_fbh = 0.0f;

  // o1 = o + hdt*spi1*ch2 ; o1y = ... sh2
  gox += go1x;
  goy += go1y;
  g_spi1 += go1x * hdt * ch2 + go1y * hdt * sh2;
  float g_ch2 = go1x * hdt * spi1, g_sh2 = go1y * hdt * spi1;
  // ch2/sh2 = cos/sin(thi1)
  float g_thi1 = gth1 + g_ch2 * (-sh2) + g_sh2 * ch2;
  // thi1 = thi + hdt*(spi1/L)*td4
  gth += g_thi1;
  const float fac = hdt / L;
  g_spi1 += g_thi1 * fac * td4;
  g_delta4 += g_thi1 * fac * spi1 * (1.0f + td4 * td4); // d tan/d delta = sec^2
  // delta4 = clamp(delta3, -d_cap, d_cap)
  float g_dcap = 0.0f;
  if (c6)
    g_delta3 += g_delta4;
  else if (delta3 >= d_cap)
    g_dcap += g_delta4;
  else
    g_dcap += -g_delta4;
  // d_cap = atan(a_lat_max*L/sp2)
  g_sp2 += g_dcap * (1.0f / (1.0f + dcap_arg * dcap_arg)) * (-a_lat_max * L / (sp2 * sp2));
  // sp2 = max(spi1^2, 1e-9)
  if (sp2_pass)
    g_spi1 += g_sp2 * 2.0f * spi1;
  // spi1 = clamp(spi1_0, sp_min, vmax)
  float g_spi1_0 = 0.0f;
  if (c5)
    g_spi1_0 += g_spi1;
  // spi1_0 = spi + hdt*aLe
  gsp += g_spi1_0;
  g_aLe += g_spi1_0 * hdt;
  // aLe = clamp(aLd, -a_max, a_max)
  if (c4)
    g_aLd += g_aLe;
  // aLd = min(aLc, (v_lim1-spi)/hdt)
  if (aLd_useC)
    g_aLc += g_aLd;
  else {
    g_v_lim1 += g_aLd / hdt;
    gsp += -g_aLd / hdt;
  }
  // aLc = clamp(aL_ns, -a_max, a_max)
  if (c3)
    g_aL_ns += g_aLc;
  // nose branch: aL_ns, delta3
  if (nose_blocked) {
    gsp += g_aL_ns * (-1.0f / hdt); // aL_ns = (-0.10 - spi)/hdt
    g_delta2 += -g_delta3;          // delta3 = -delta2
  } else {
    g_aL_st += g_aL_ns;
    g_delta2 += g_delta3;
  }
  // stuck branch: aL_st = stuck ? max(aL_bh, stuck_floor) : aL_bh
  if (stuck_useFloor)
    gsp += g_aL_st * (-1.0f / hdt); // stuck_floor = (0.08 - spi)/hdt
  else
    g_aL_bh += g_aL_st;
  // behind branch: aL_bh
  if (behind) {
    g_v_creep += g_aL_bh / hdt;
    gsp += -g_aL_bh / hdt;
    g_fbh += g_aL_bh;
    if (fbh_pass) {
      g_Fbar_x += g_fbh * ch;
      g_Fbar_y += g_fbh * sh;
      g_ch += g_fbh * Fbar_x;
      g_sh += g_fbh * Fbar_y;
    }
  } else {
    g_aL1 += g_aL_bh;
  }
  // v_creep = min(0.5*v_corner, v_creep_cap)
  if (vcreep_useCorner)
    g_v_corner += g_v_creep * 0.5f;
  // v_lim1 = max(v_lim0, v_floor)  (v_floor const)
  if (!vlim1_useFloor)
    g_v_lim0 += g_v_lim1;
  // v_lim0 = min(vmax, inner)
  float g_inner = 0.0f;
  if (vlim0_useInner)
    g_inner += g_v_lim0;
  // inner = min(v_corner, v_stop_dir)
  float g_v_stop_dir = 0.0f;
  if (inner_useCorner)
    g_v_corner += g_inner;
  else
    g_v_stop_dir += g_inner;
  // v_stop_dir = v_stop/approach (when active)
  if (vsd_active) {
    g_v_stop += g_v_stop_dir / approach;
    g_appr += g_v_stop_dir * (-v_stop / (approach * approach));
  }
  // approach = clamp(appr0, 0, 1) ; appr0 = -(nx*ch+ny*sh)*motion_sign
  if (appr_pass) {
    const float ga0 = g_appr;
    g_nx += ga0 * (-motion_sign * ch);
    g_ny += ga0 * (-motion_sign * sh);
    g_ch += ga0 * (-motion_sign * s.nx);
    g_sh += ga0 * (-motion_sign * s.ny);
  }
  // v_stop = sqrt(2*a_max*ds)
  if (v_stop > 0.0f)
    g_ds += g_v_stop * a_max / v_stop;
  // ds = max(ds0,0) ; ds0 = d - 0.5*rr
  if (ds_pass)
    g_d += g_ds;
  // v_corner = sqrt(a_lat_max/kappa)
  g_kappa += g_v_corner * (-a_lat_max / (2.0f * kappa * kappa * v_corner));
  // kappa = max(kappa0, kfloor) ; kappa0 = |tan(delta2)|/L
  if (kf_pass) {
    const float sgn = td2 >= 0.0f ? 1.0f : -1.0f;
    g_delta2 += g_kappa * sgn * (1.0f + td2 * td2) / L;
  }
  // delta2 = clamp(delta1, -dmax, dmax)
  float g_delta1 = 0.0f;
  if (c2)
    g_delta1 += g_delta2;
  // delta1 = delta0 + k_steer*tanh(targ)
  g_delta0 += g_delta1;
  g_targ += g_delta1 * k_steer * (1.0f - tanh_t * tanh_t);
  // targ = Frep_x*(-sh) + Frep_y*ch
  g_Frep_x += g_targ * (-sh);
  g_Frep_y += g_targ * ch;
  g_sh += g_targ * (-Frep_x);
  g_ch += g_targ * Frep_y;
  // Frep = -(al*ipc_rep)*n
  g_al += g_Frep_x * (-ipc_rep * s.nx) + g_Frep_y * (-ipc_rep * s.ny);
  g_ipc_rep += g_Frep_x * (-al * s.nx) + g_Frep_y * (-al * s.ny);
  g_nx += g_Frep_x * (-al * ipc_rep);
  g_ny += g_Frep_y * (-al * ipc_rep);
  if (iprep_pass)
    g_ipc += g_ipc_rep;
  // delta0 = behind ? const : atan2(2*L*sin_a, Lde)
  if (!behind) {
    const float y = 2.0f * L * sin_a, x = Lde, den = x * x + y * y;
    g_sin_a += g_delta0 * (x / den) * 2.0f * L;
    g_Lde += g_delta0 * (-y / den);
  }
  // Lde = min(L_d, Lde1)
  float g_Lde1 = 0.0f;
  if (lde_useLd)
    g_Ld += g_Lde;
  else
    g_Lde1 += g_Lde;
  // Lde1 = max(Lde0, 4L) ; Lde0 = 1.2*spi + 4L
  if (lde1_pass)
    gsp += g_Lde1 * 1.2f;
  // sin_a = sin(ang) ; cos_a -> behind only (no grad)
  g_ang += g_sin_a * cos_a;
  // ang = atan2(tgy,tgx) - thi
  gth += -g_ang;
  {
    const float den = tgx * tgx + tgy * tgy;
    if (den > 0.0f) {
      g_tgy += g_ang * (tgx / den);
      g_tgx += g_ang * (-tgy / den);
    }
  }
  // L_d = (Ld0<1e-6)? : Ld0 ; Ld0 = sqrt(tgx^2+tgy^2)
  if (ld_pass && Ld0 > 0.0f) {
    g_tgx += g_Ld * tgx / Ld0;
    g_tgy += g_Ld * tgy / Ld0;
  }
  // tg = goal - o
  gox += -g_tgx;
  goy += -g_tgy;
  // aL1 = clamp(aL0, -a_max, a_max)
  if (c1)
    g_aL0 += g_aL1;
  // aL0 = Fx*ch + Fy*sh - ga*spi
  g_Fx += g_aL0 * ch;
  g_Fy += g_aL0 * sh;
  g_ch += g_aL0 * Fx;
  g_sh += g_aL0 * Fy;
  gga += g_aL0 * (-spi);
  gsp += g_aL0 * (-ga);
  // ch = cos(thi), sh = sin(thi)  [all g_ch/g_sh contributions accumulated above]
  gth += g_ch * (-sh) + g_sh * ch;
  // Fx = Fbar_x + Fgoal_x ; Fy = ...
  g_Fbar_x += g_Fx;
  g_Fgoal_x += g_Fx;
  g_Fbar_y += g_Fy;
  g_Fgoal_y += g_Fy;
  // Fgoal = -be*(o - goal)
  gbe += g_Fgoal_x * (-(ox - gx)) + g_Fgoal_y * (-(oy - gy));
  gox += g_Fgoal_x * (-be);
  goy += g_Fgoal_y * (-be);
  // Fbar = -(al*ipc)*n
  g_al += g_Fbar_x * (-ipc * s.nx) + g_Fbar_y * (-ipc * s.ny);
  g_ipc += g_Fbar_x * (-al * s.nx) + g_Fbar_y * (-al * s.ny);
  g_nx += g_Fbar_x * (-al * ipc);
  g_ny += g_Fbar_y * (-al * ipc);
  // ipc = ipc_(d) ; d = phi - rr
  g_d += g_ipc * ipc_grad_(d, d_hat);
  g_phi += g_d;
  // sample
  float sdox, sdoy;
  sample_bwd(s, g_phi, g_nx, g_ny, sdox, sdoy);
  gox += sdox;
  goy += sdoy;

  gal += g_al;
}

} // namespace diff
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DIFF_ROLLOUT_H__
