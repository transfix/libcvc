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

// ── reverse-mode adjoint (P5 training backward) ──────────────────────────────
// A per-agent backward-through-time over the SAME forward above, recomputed step
// by step (state trajectory stored, per-step forces recomputed) then reversed.
// The op order and every clamp/branch/guard is a mirror of the forward; the
// per-step VJPs are the detail/material_rollout.h primitives. hard_count's grad
// is identically zero (a step-function count) so it carries no upstream seed.
void integrate_surrogate_material_vjp(
    const float *o0, const float *v0, const float *goal, const float *C, const float *R,
    const std::uint8_t *mask, const float *alphas, const float *beta, const float *gamma,
    const float *lam_soft, const float *lam_hard, const float *rollout_patch, const float *rr,
    const float *d_hat, const float *dt, const int *H, int B, int N, int Hp, int Wp,
    const surrogate_material_params &p, const float *g_oT, const float *g_vT,
    const float *g_min_clear, const float *g_cum_risk, const float *g_arc_length, float *g_alphas,
    float *g_beta, float *g_gamma, float *g_lam_soft, float *g_lam_hard, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);
  const long patch_stride = static_cast<long>(6) * Hp * Wp;
  const long chan = static_cast<long>(Hp) * Wp;

  detail::parallel_for(B, num_threads, [&](int b) {
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], ls = lam_soft[b], lh = lam_hard[b];
    const float dtb = dt[b], dh = d_hat[b], rr_eff = p.margin_factor * rr[b];
    const float o0x = o0[2 * b], o0y = o0[2 * b + 1];
    const float *pr = rollout_patch + static_cast<long>(b) * patch_stride;
    const float *pr_risk = pr, *pr_phi = pr + chan, *pr_grx = pr + 2 * chan;
    const float *pr_gry = pr + 3 * chan, *pr_gpx = pr + 4 * chan, *pr_gpy = pr + 5 * chan;
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;

    // Forward recompute, storing the (o,v) trajectory + locating the min_clear
    // argmin step (first strictly-smaller dmin, matching the forward).
    std::vector<float> Ox(max_H + 1), Oy(max_H + 1), Vx(max_H + 1), Vy(max_H + 1);
    Ox[0] = o0x;
    Oy[0] = o0y;
    Vx[0] = v0[2 * b];
    Vy[0] = v0[2 * b + 1];
    float minclr = std::numeric_limits<float>::infinity();
    int s_star = -1, j_star = -1;
    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
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
      int dmin_j = -1;
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
        if (d < dmin) {
          dmin = d;
          dmin_j = j;
        }
      }
      if (dmin < minclr) {
        minclr = dmin;
        s_star = s;
        j_star = dmin_j;
      }
      const float fsoft_x = -ls * grx, fsoft_y = -ls * gry;
      const float db = detail::sdf_barrier_db(sdf_val, p.k_sharp, p.d_hat_sdf);
      const float fhard_x = -lh * db * gpx, fhard_y = -lh * db * gpy;
      const float ftot_x = fgoal_x + fgeom_x + fsoft_x + fhard_x - ga * vx;
      const float ftot_y = fgoal_y + fgeom_y + fsoft_y + fhard_y - ga * vy;
      const float ax = ftot_x / p.mass, ay = ftot_y / p.mass;
      Vx[s + 1] = vx + active * dtb * ax;
      Vy[s + 1] = vy + active * dtb * ay;
      Ox[s + 1] = ox + active * dtb * Vx[s + 1];
      Oy[s + 1] = oy + active * dtb * Vy[s + 1];
    }

    // Upstream seeds (any may be null -> zero).
    const float goTx = g_oT ? g_oT[2 * b] : 0.0f, goTy = g_oT ? g_oT[2 * b + 1] : 0.0f;
    const float gvTx = g_vT ? g_vT[2 * b] : 0.0f, gvTy = g_vT ? g_vT[2 * b + 1] : 0.0f;
    const float gmin = g_min_clear ? g_min_clear[b] : 0.0f;
    const float gcum = g_cum_risk ? g_cum_risk[b] : 0.0f;
    const float garc = g_arc_length ? g_arc_length[b] : 0.0f;

    // Reverse pass. ao/av carry dL/d(o_{s+1}), dL/d(v_{s+1}).
    float aox = goTx, aoy = goTy, avx = gvTx, avy = gvTy;
    for (int s = max_H - 1; s >= 0; --s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
      const float offx = ox - o0x, offy = oy - o0y;
      // recompute the step's sampled fields + forces (same as forward)
      float risk_raw = detail::patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
      float sdf_raw = detail::patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
      const float grx = detail::patch_sample_channel(pr_grx, Hp, Wp, offx, offy);
      const float gry = detail::patch_sample_channel(pr_gry, Hp, Wp, offx, offy);
      const float gpx = detail::patch_sample_channel(pr_gpx, Hp, Wp, offx, offy);
      const float gpy = detail::patch_sample_channel(pr_gpy, Hp, Wp, offx, offy);
      const float risk_val = std::min(std::max(risk_raw, 0.0f), 1.0f);
      const float sdf_val = std::min(std::max(sdf_raw, 0.0f), 50.0f);
      const float db = detail::sdf_barrier_db(sdf_val, p.k_sharp, p.d_hat_sdf);

      const float onx = Ox[s + 1], ony = Oy[s + 1];
      const float sdx = onx - ox, sdy = ony - oy;
      const float disp = std::sqrt(sdx * sdx + sdy * sdy);

      // arc/cum contributions of this step (g_arc/g_cum are constant scalars).
      const float g_disp = garc * active + gcum * active * risk_val;
      float ddx = 0.0f, ddy = 0.0f;
      if (disp > 0.0f) {
        ddx = sdx / disp;
        ddy = sdy / disp;
      }
      float go_x = -g_disp * ddx, go_y = -g_disp * ddy; // o_s via -sdx
      const float g_risk_val = gcum * active * disp;    // cum uses risk_val directly

      // on = o + active*dt*vn  (on also feeds disp)
      const float aon_x = aox + g_disp * ddx, aon_y = aoy + g_disp * ddy;
      go_x += aon_x;
      go_y += aon_y;
      const float gvn_x = avx + aon_x * active * dtb; // dL/d vn (= v_{s+1})
      const float gvn_y = avy + aon_y * active * dtb;
      // vn = v + active*dt*a
      float gv_x = gvn_x, gv_y = gvn_y;        // d vn/d v_s = 1
      const float ga_x = gvn_x * active * dtb; // adjoint of a
      const float ga_y = gvn_y * active * dtb;
      const float gftot_x = ga_x / p.mass, gftot_y = ga_y / p.mass;

      // ftot = fgoal + fgeom + fsoft + fhard - gamma*v
      g_gamma[b] += -(gftot_x * vx + gftot_y * vy);
      gv_x += -ga * gftot_x;
      gv_y += -ga * gftot_y;
      // fgoal = -be*(o - goal)
      g_beta[b] += -(gftot_x * (ox - gx) + gftot_y * (oy - gy));
      go_x += -be * gftot_x;
      go_y += -be * gftot_y;
      // fsoft = -ls*[grx,gry]
      g_lam_soft[b] += -(gftot_x * grx + gftot_y * gry);
      float g_grx = -ls * gftot_x, g_gry = -ls * gftot_y;
      // fhard = -lh*db*[gpx,gpy]
      g_lam_hard[b] += -(gftot_x * db * gpx + gftot_y * db * gpy);
      const float g_db = -lh * (gftot_x * gpx + gftot_y * gpy);
      float g_gpx = -lh * db * gftot_x, g_gpy = -lh * db * gftot_y;
      // db = sdf_barrier_db(sdf_val)
      float g_sdf_val = g_db * detail::sdf_barrier_db_grad(sdf_val, p.k_sharp, p.d_hat_sdf);

      // fgeom = sum_j -(alpha_j*dbdd_j)*nhat_j
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        const float r0 = std::sqrt(dx * dx + dy * dy);
        const float r = r0 < 1e-9f ? 1e-9f : r0;
        const float nhx = dx / r, nhy = dy / r;
        const float d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        g_alphas[b * N + j] += -(gftot_x * dbdd * nhx + gftot_y * dbdd * nhy);
        const float g_dbdd = -ab[j] * (gftot_x * nhx + gftot_y * nhy);
        const float g_nhx = -(ab[j] * dbdd) * gftot_x, g_nhy = -(ab[j] * dbdd) * gftot_y;
        const float g_d = g_dbdd * detail::ipc_dbdd_pw_grad(d, dh);
        go_x += g_d * nhx; // d(d)/d o_s = nhat
        go_y += g_d * nhy;
        if (r0 > 1e-9f) { // nhat position jac (I - nn^T)/r; measure-zero if floored
          const float dot = nhx * g_nhx + nhy * g_nhy;
          go_x += (g_nhx - nhx * dot) / r;
          go_y += (g_nhy - nhy * dot) / r;
        }
      }
      // min_clear injection at the global-argmin step (d_{j*} = r - (R+rr_eff))
      if (s == s_star && j_star >= 0) {
        const float dx = ox - Cb[2 * j_star], dy = oy - Cb[2 * j_star + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        go_x += gmin * (dx / r);
        go_y += gmin * (dy / r);
      }

      // sampled-channel position grads: value(ch) depends on off = o_s - o0.
      const float g_risk_raw = g_risk_val * ((risk_raw > 0.0f && risk_raw < 1.0f) ? 1.0f : 0.0f);
      const float g_sdf_raw = g_sdf_val * ((sdf_raw > 0.0f && sdf_raw < 50.0f) ? 1.0f : 0.0f);
      const float *chs[6] = {pr_risk, pr_phi, pr_grx, pr_gry, pr_gpx, pr_gpy};
      const float gcs[6] = {g_risk_raw, g_sdf_raw, g_grx, g_gry, g_gpx, g_gpy};
      for (int c = 0; c < 6; ++c) {
        float jx, jy;
        detail::patch_sample_channel_grad(chs[c], Hp, Wp, offx, offy, jx, jy);
        go_x += gcs[c] * jx;
        go_y += gcs[c] * jy;
      }

      aox = go_x;
      aoy = go_y;
      avx = gv_x;
      avy = gv_y;
    }
  });
}

} // namespace nav
} // namespace cvc
