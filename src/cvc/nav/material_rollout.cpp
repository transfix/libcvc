/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_rollout.cpp — integrate_surrogate_material (see material.h). Faithful
// float-equivalent transcription of GRL-SNAM
// material_nav.integrate_surrogate_material. Built with -ffp-contract=off. The
// per-step force primitives live in detail/material_rollout.h (CVC_HD, shared
// with the coming P5 backward + CUDA twin). Do not "tidy" the op order.

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/material.h>
#include <limits>
#include <vector>

namespace cvc {
namespace nav {

void integrate_surrogate_material(
    float *o, float *v, const float *goal, const float *C, const float *R, const std::uint8_t *mask,
    const float *alphas, const float *beta, const float *gamma, const float *lam_soft,
    const float *lam_hard, const float *rollout_patch, const float *rr, const float *d_hat,
    const float *dt, const int *H, int B, int N, int Hp, int Wp, const surrogate_material_params &p,
    float *min_clear, float *cum_risk, float *hard_count, float *arc_length, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);
  const long patch_stride = static_cast<long>(6) * Hp * Wp;
  const long chan = static_cast<long>(Hp) * Wp;

  detail::parallel_for(B, num_threads, [&](int b) {
    float ox = o[2 * b], oy = o[2 * b + 1];
    float vx = v[2 * b], vy = v[2 * b + 1];
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], ls = lam_soft[b], lh = lam_hard[b];
    const float dh = d_hat[b], dtb = dt[b];
    const float rr_eff = p.margin_factor * rr[b];
    const float o0x = ox, o0y = oy; // patch centre = rollout start
    const float *pr = rollout_patch + static_cast<long>(b) * patch_stride;
    const float *pr_risk = pr;           // ch0
    const float *pr_phi = pr + chan;     // ch1
    const float *pr_grx = pr + 2 * chan; // ch2
    const float *pr_gry = pr + 3 * chan; // ch3
    const float *pr_gpx = pr + 4 * chan; // ch4
    const float *pr_gpy = pr + 5 * chan; // ch5
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;

    float minclr = std::numeric_limits<float>::infinity();
    float cum = 0.0f, hard = 0.0f, arc = 0.0f;

    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float offx = ox - o0x, offy = oy - o0y;
      float risk_val = detail::patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
      float sdf_val = detail::patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
      const float grx = detail::patch_sample_channel(pr_grx, Hp, Wp, offx, offy);
      const float gry = detail::patch_sample_channel(pr_gry, Hp, Wp, offx, offy);
      const float gpx = detail::patch_sample_channel(pr_gpx, Hp, Wp, offx, offy);
      const float gpy = detail::patch_sample_channel(pr_gpy, Hp, Wp, offx, offy);
      risk_val = std::min(std::max(risk_val, 0.0f), 1.0f);
      sdf_val = std::min(std::max(sdf_val, 0.0f), 50.0f);

      const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);

      float fgeom_x = 0.0f, fgeom_y = 0.0f;
      float dmin = std::numeric_limits<float>::infinity();
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        const float nhx = dx / r, nhy = dy / r;
        const float d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        fgeom_x += -(ab[j] * dbdd) * nhx;
        fgeom_y += -(ab[j] * dbdd) * nhy;
        if (d < dmin)
          dmin = d;
      }
      if (dmin < minclr)
        minclr = dmin; // ungated, matching the vectorized reference

      const float fsoft_x = -ls * grx, fsoft_y = -ls * gry;
      const float db = detail::sdf_barrier_db(sdf_val, p.k_sharp, p.d_hat_sdf);
      const float fhard_x = -lh * db * gpx, fhard_y = -lh * db * gpy;

      const float ftot_x = fgoal_x + fgeom_x + fsoft_x + fhard_x - ga * vx;
      const float ftot_y = fgoal_y + fgeom_y + fsoft_y + fhard_y - ga * vy;
      const float ax = ftot_x / p.mass, ay = ftot_y / p.mass;
      const float vnx = vx + active * dtb * ax, vny = vy + active * dtb * ay;
      const float onx = ox + active * dtb * vnx, ony = oy + active * dtb * vny;

      const float sdx = onx - ox, sdy = ony - oy;
      const float disp = std::sqrt(sdx * sdx + sdy * sdy);
      arc += active * disp;
      cum += active * risk_val * disp;
      hard += active * (sdf_val < 1.0f ? 1.0f : 0.0f);

      vx = vnx;
      vy = vny;
      ox = onx;
      oy = ony;
    }

    o[2 * b] = ox;
    o[2 * b + 1] = oy;
    v[2 * b] = vx;
    v[2 * b + 1] = vy;
    min_clear[b] = minclr;
    cum_risk[b] = cum;
    hard_count[b] = hard;
    arc_length[b] = arc;
  });
}

} // namespace nav
} // namespace cvc
