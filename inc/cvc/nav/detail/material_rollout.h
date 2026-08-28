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

#if defined(__CUDACC__)
#define CVC_MR_HD __host__ __device__
#else
#define CVC_MR_HD
#endif

namespace cvc {
namespace nav {
namespace detail {

// IPC barrier derivative dbdd (train_coef_energy.ipc_piecewise): safe=max(d,eps),
// dbdd_in=(dh-d)(2 ln(safe/dh) - dh/safe)+1; d<=eps -> vp=-500; d>=dh -> 0; then
// clamp to [-max_grad, max_grad]=[-200,200]. So d<=eps yields -200 after clamp.
CVC_MR_HD inline float ipc_dbdd_pw(float d, float dh) {
  const float eps = 1e-9f, vp = -500.0f, max_grad = 200.0f;
  float out;
  if (d <= eps) {
    out = vp;
  } else if (d < dh) {
    const float safe = d < eps ? eps : d;
    out = (dh - d) * (2.0f * std::log(safe / dh) - dh / safe) + 1.0f;
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

} // namespace detail
} // namespace nav
} // namespace cvc

#endif
