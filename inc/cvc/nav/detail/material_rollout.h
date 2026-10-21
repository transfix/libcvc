/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// detail/material_rollout.h — CVC_HD (host+device) primitives for the
// obstacle-list material surrogate rollout (integrate_surrogate_material). The
// forward here mirrors GRL-SNAM material_nav.integrate_surrogate_material,
// itself a bit-identical port of the source method's training/eval integrator.
// Kept CVC_HD and side-effect-free so the P5 training backward and a CUDA
// twin can reuse the exact forward ops. Do not "tidy" the arithmetic.

#ifndef CVC_NAV_DETAIL_MATERIAL_ROLLOUT_H
#define CVC_NAV_DETAIL_MATERIAL_ROLLOUT_H

#include <cmath>
#include <cvc/nav/detail/nav_potential.h>

#if defined(__CUDACC__)
#define CVC_MR_HD __host__ __device__
#else
#define CVC_MR_HD
#endif

namespace cvc {
namespace nav {
namespace detail {

// IPC barrier derivative dbdd (train_coef_energy.ipc_piecewise): safe=max(d,eps),
// dbdd_in = -(2(d-dh) ln(safe/dh) + (d-dh)^2/safe) [M10: analytic b', was the
// +（d-dh)+1 attraction-band form]; d<=eps -> vp=-500; d>=dh -> 0; then
// clamp to [-max_grad, max_grad]=[-200,200]. So d<=eps yields -200 after clamp.
CVC_MR_HD inline float ipc_dbdd_pw(float d, float dh) {
  const float eps = 1e-9f, vp = -500.0f, max_grad = 200.0f;
  float out;
  if (d <= eps) {
    out = vp;
  } else if (d < dh) {
    const float safe = d < eps ? eps : d;
    // M10: analytic b' (was +（d-dh)+1, an attraction band); verified vs autograd.
    out = -(2.0f * (d - dh) * std::log(safe / dh) + (d - dh) * (d - dh) / safe);
  } else {
    out = 0.0f;
  }
  if (out < -max_grad)
    out = -max_grad;
  if (out > max_grad)
    out = max_grad;
  return out;
}

// Softplus SDF-barrier derivative db/dphi = -sigmoid(k*(d_hat_sdf - phi)).
CVC_MR_HD inline float sdf_barrier_db(float phi, float k_sharp, float d_hat_sdf) {
  return -1.0f / (1.0f + std::exp(-(k_sharp * (d_hat_sdf - phi))));
}

// Bilinear sample of one channel of a (Hp,Wp) patch at patch-local offset
// (offx,offy) in pixels from the patch centre — the bilinear_sample_patch
// coordinate map: gx = offx/((Wp-1)/2 + 1e-8), then torch grid_sample
// (align_corners=True, border), i.e. ix = clamp((gx+1)*0.5*(Wp-1), 0, Wp-1).
CVC_MR_HD inline float patch_sample_channel(const float *ch, int Hp, int Wp, float offx,
                                            float offy) {
  const float half_w = (Wp - 1) * 0.5f, half_h = (Hp - 1) * 0.5f;
  const float gx = offx / (half_w + 1e-8f), gy = offy / (half_h + 1e-8f);
  float ix = (gx + 1.0f) * 0.5f * (Wp - 1);
  float iy = (gy + 1.0f) * 0.5f * (Hp - 1);
  if (ix < 0.0f)
    ix = 0.0f;
  if (ix > Wp - 1)
    ix = static_cast<float>(Wp - 1);
  if (iy < 0.0f)
    iy = 0.0f;
  if (iy > Hp - 1)
    iy = static_cast<float>(Hp - 1);
  const int ix0 = static_cast<int>(std::floor(ix)), iy0 = static_cast<int>(std::floor(iy));
  const float wx1 = ix - ix0, wx0 = 1.0f - wx1, wy1 = iy - iy0, wy0 = 1.0f - wy1;
  const int cx0 = ix0 < 0 ? 0 : (ix0 > Wp - 1 ? Wp - 1 : ix0);
  const int cx1 = (ix0 + 1) < 0 ? 0 : ((ix0 + 1) > Wp - 1 ? Wp - 1 : ix0 + 1);
  const int cy0 = iy0 < 0 ? 0 : (iy0 > Hp - 1 ? Hp - 1 : iy0);
  const int cy1 = (iy0 + 1) < 0 ? 0 : ((iy0 + 1) > Hp - 1 ? Hp - 1 : iy0 + 1);
  const float nw = ch[cy0 * Wp + cx0], ne = ch[cy0 * Wp + cx1];
  const float sw = ch[cy1 * Wp + cx0], se = ch[cy1 * Wp + cx1];
  return nw * wx0 * wy0 + ne * wx1 * wy0 + sw * wx0 * wy1 + se * wx1 * wy1;
}

// ── VJP primitives for the P5 training backward (shared host+device) ──────────
// Each is the exact derivative of the forward op above, honouring its clamps and
// branch selection (grad is zero on a saturated clamp / dead branch, matching
// autograd on min/max/where). Validated end-to-end by the finite-difference
// gradcheck (nav_material_rollout_grad_test); do not "simplify".

// d(ipc_dbdd_pw)/dd = b''. Interior formula equals detail/diff_rollout.h ipc_grad_'s
// (the piecewise value there is this dbdd), guarded by the vp / >=dh dead
// branches and the [-200,200] saturation (zero grad where clamped).
CVC_MR_HD inline float ipc_dbdd_pw_grad(float d, float dh) {
  const float eps = 1e-9f, max_grad = 200.0f;
  if (d <= eps || !(d < dh))
    return 0.0f; // vp branch (then clamped) and the d>=dh zero branch
  // M10: forward is now b' = -(2 A ln(d/dh) + A^2/d), A = d-dh; grad is b''.
  const float lg = std::log(d / dh);
  const float A = d - dh;
  const float out = -(2.0f * A * lg + A * A / d); // corrected dbdd (= b')
  if (out < -max_grad || out > max_grad)
    return 0.0f; // saturated on the [-200,200] clamp -> zero grad
  return -2.0f * lg - 4.0f * A / d + A * A / (d * d); // b''
}

// d(sdf_barrier_db)/dphi. db = -sigmoid(z), z = k*(d_hat_sdf - phi); with
// s = sigmoid(z), d(db)/dphi = -s(1-s)*dz/dphi = k*s*(1-s).
CVC_MR_HD inline float sdf_barrier_db_grad(float phi, float k_sharp, float d_hat_sdf) {
  const float s = 1.0f / (1.0f + std::exp(-(k_sharp * (d_hat_sdf - phi))));
  return k_sharp * s * (1.0f - s);
}

// Position Jacobian of patch_sample_channel: (d value / d offx, d value / d offy)
// for one channel at (offx,offy). Zero along an axis whose index saturated on the
// border clamp (grid_sample border padding => no position gradient there). The
// sampled value itself comes from patch_sample_channel; this only adds the
// geometry, so the rollout backward recomputes indices identically here.
CVC_MR_HD inline void patch_sample_channel_grad(const float *ch, int Hp, int Wp, float offx,
                                                float offy, float &jx, float &jy) {
  const float half_w = (Wp - 1) * 0.5f, half_h = (Hp - 1) * 0.5f;
  const float gx = offx / (half_w + 1e-8f), gy = offy / (half_h + 1e-8f);
  float ix = (gx + 1.0f) * 0.5f * (Wp - 1);
  float iy = (gy + 1.0f) * 0.5f * (Hp - 1);
  const bool clx = (ix < 0.0f) || (ix > Wp - 1);
  const bool cly = (iy < 0.0f) || (iy > Hp - 1);
  if (ix < 0.0f)
    ix = 0.0f;
  if (ix > Wp - 1)
    ix = static_cast<float>(Wp - 1);
  if (iy < 0.0f)
    iy = 0.0f;
  if (iy > Hp - 1)
    iy = static_cast<float>(Hp - 1);
  const int ix0 = static_cast<int>(std::floor(ix)), iy0 = static_cast<int>(std::floor(iy));
  const float wx1 = ix - ix0, wx0 = 1.0f - wx1, wy1 = iy - iy0, wy0 = 1.0f - wy1;
  const int cx0 = ix0 < 0 ? 0 : (ix0 > Wp - 1 ? Wp - 1 : ix0);
  const int cx1 = (ix0 + 1) < 0 ? 0 : ((ix0 + 1) > Wp - 1 ? Wp - 1 : ix0 + 1);
  const int cy0 = iy0 < 0 ? 0 : (iy0 > Hp - 1 ? Hp - 1 : iy0);
  const int cy1 = (iy0 + 1) < 0 ? 0 : ((iy0 + 1) > Hp - 1 ? Hp - 1 : iy0 + 1);
  const float nw = ch[cy0 * Wp + cx0], ne = ch[cy0 * Wp + cx1];
  const float sw = ch[cy1 * Wp + cx0], se = ch[cy1 * Wp + cx1];
  // d value / d ix and d value / d iy (bilinear), then chain ix<-gx<-offx.
  const float dv_dix = wy0 * (ne - nw) + wy1 * (se - sw);
  const float dv_diy = wx0 * (sw - nw) + wx1 * (se - ne);
  const float dix_doffx = clx ? 0.0f : 0.5f * (Wp - 1) / (half_w + 1e-8f);
  const float diy_doffy = cly ? 0.0f : 0.5f * (Hp - 1) / (half_h + 1e-8f);
  jx = dv_dix * dix_doffx;
  jy = dv_diy * diy_doffy;
}

// U_nav at o for the force the material rollouts apply (detail/nav_potential.h): the
// obstacle discs' barriers (ipc_dbdd_pw's potential, summed in the forces' order), the goal
// spring and the patch's soft and hard terms, with the coefficients held fixed. The patch values
// are clamped as the forces read them (r~ to [0, 1], phi to [0, 50]). The soft and hard terms
// are the potentials of -lam_soft grad r~ and -lam_hard db grad phi when the patch's channels
// 2-5 are the gradients of channels 0-1 in the batch's units; that convention is the caller's.
CVC_MR_HD inline potential_terms
material_potential(float ox, float oy, float gx, float gy, const float *Cb, const float *Rb,
                   const unsigned char *mb, const float *ab, int N, float be, float ls, float lh,
                   float dh, float rr_eff, const float *pr_risk, const float *pr_phi, int Hp,
                   int Wp, float o0x, float o0y, float k_sharp, float d_hat_sdf) {
  potential_terms u;
  const float dx = ox - gx, dy = oy - gy;
  u.goal = 0.5f * be * (dx * dx + dy * dy);
  for (int j = 0; j < N; ++j) {
    if (!mb[j])
      continue;
    const float cx = ox - Cb[2 * j], cy = oy - Cb[2 * j + 1];
    float r = std::sqrt(cx * cx + cy * cy);
    if (r < 1e-9f)
      r = 1e-9f;
    u.barrier += ab[j] * ipc_b_pw(r - (Rb[j] + rr_eff), dh);
  }
  const float offx = ox - o0x, offy = oy - o0y;
  float risk = patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
  float sdf = patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
  risk = risk < 0.0f ? 0.0f : (risk > 1.0f ? 1.0f : risk);
  sdf = sdf < 0.0f ? 0.0f : (sdf > 50.0f ? 50.0f : sdf);
  u.soft = ls * risk;
  u.hard = lh * sdf_barrier_b(sdf, k_sharp, d_hat_sdf);
  return u;
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif
