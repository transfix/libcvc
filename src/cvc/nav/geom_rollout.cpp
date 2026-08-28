/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// geom_rollout.cpp — see geom_rollout.h. The EXPLICIT-Euler geometry rollout
// (integrate_surrogate_v2), its reverse-mode adjoint, and the multi-start
// robustness penalty L_multi. Reuses the CVC_HD IPC primitives from
// detail/material_rollout.h. Built with -ffp-contract=off (CMake).

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/geom_rollout.h>
#include <limits>
#include <vector>

namespace cvc {
namespace nav {

void integrate_surrogate_v2(float *o, float *v, const float *goal, const float *C, const float *R,
                            const std::uint8_t *mask, const float *alphas, const float *beta,
                            const float *gamma, const float *rr, const float *d_hat,
                            const float *dt, const int *H, int B, int N,
                            const geom_rollout_params &p, float *min_clear, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);

  detail::parallel_for(B, num_threads, [&](int b) {
    float ox = o[2 * b], oy = o[2 * b + 1], vx = v[2 * b], vy = v[2 * b + 1];
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], dh = d_hat[b], dtb = dt[b];
    const float rr_eff = p.margin_factor * rr[b];
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;
    float minclr = std::numeric_limits<float>::infinity();

    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
      float fbar_x = 0.0f, fbar_y = 0.0f, dmin = std::numeric_limits<float>::infinity();
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        fbar_x += -(ab[j] * dbdd) * nhx;
        fbar_y += -(ab[j] * dbdd) * nhy;
        if (d < dmin)
          dmin = d;
      }
      if (dmin < minclr)
        minclr = dmin;
      const float ax = (fbar_x + fgoal_x - ga * vx) / p.mass;
      const float ay = (fbar_y + fgoal_y - ga * vy) / p.mass;
      const float onx = ox + active * dtb * vx; // OLD v (explicit Euler)
      const float ony = oy + active * dtb * vy;
      vx = vx + active * dtb * ax;
      vy = vy + active * dtb * ay;
      ox = onx;
      oy = ony;
    }
    o[2 * b] = ox;
    o[2 * b + 1] = oy;
    v[2 * b] = vx;
    v[2 * b + 1] = vy;
    min_clear[b] = minclr;
  });
}

void integrate_surrogate_v2_vjp(const float *o0, const float *v0, const float *goal, const float *C,
                                const float *R, const std::uint8_t *mask, const float *alphas,
                                const float *beta, const float *gamma, const float *rr,
                                const float *d_hat, const float *dt, const int *H, int B, int N,
                                const geom_rollout_params &p, const float *g_oT, const float *g_vT,
                                const float *g_min_clear, float *g_alphas, float *g_beta,
                                float *g_gamma, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);

  detail::parallel_for(B, num_threads, [&](int b) {
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], dh = d_hat[b], dtb = dt[b];
    const float rr_eff = p.margin_factor * rr[b];
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;

    // forward recompute: store trajectory + locate the min_clear argmin
    std::vector<float> Ox(max_H + 1), Oy(max_H + 1), Vx(max_H + 1), Vy(max_H + 1);
    Ox[0] = o0[2 * b];
    Oy[0] = o0[2 * b + 1];
    Vx[0] = v0[2 * b];
    Vy[0] = v0[2 * b + 1];
    float minclr = std::numeric_limits<float>::infinity();
    int s_star = -1, j_star = -1;
    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
      const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
      float fbar_x = 0.0f, fbar_y = 0.0f, dmin = std::numeric_limits<float>::infinity();
      int dmin_j = -1;
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        fbar_x += -(ab[j] * dbdd) * nhx;
        fbar_y += -(ab[j] * dbdd) * nhy;
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
      const float ax = (fbar_x + fgoal_x - ga * vx) / p.mass;
      const float ay = (fbar_y + fgoal_y - ga * vy) / p.mass;
      Ox[s + 1] = ox + active * dtb * vx;
      Oy[s + 1] = oy + active * dtb * vy;
      Vx[s + 1] = vx + active * dtb * ax;
      Vy[s + 1] = vy + active * dtb * ay;
    }

    const float gmin = g_min_clear ? g_min_clear[b] : 0.0f;
    float aox = g_oT ? g_oT[2 * b] : 0.0f, aoy = g_oT ? g_oT[2 * b + 1] : 0.0f;
    float avx = g_vT ? g_vT[2 * b] : 0.0f, avy = g_vT ? g_vT[2 * b + 1] : 0.0f;

    for (int s = max_H - 1; s >= 0; --s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float c = active * dtb;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
      // gF = adjoint of the force F (a = F/mass; v_{s+1} = v_s + c*a)
      const float gF_x = (avx * c) / p.mass, gF_y = (avy * c) / p.mass;
      // o_{s+1} = o_s + c*v_s ; v_{s+1} = v_s + c*a
      float go_x = aox, go_y = aoy;                     // d o_{s+1}/d o_s = 1
      float gv_x = aox * c + avx, gv_y = aoy * c + avy; // d o_{s+1}/d v_s + d v_{s+1}/d v_s
      // F = fbar + fgoal - ga*v
      g_gamma[b] += -(gF_x * vx + gF_y * vy);
      gv_x += -ga * gF_x;
      gv_y += -ga * gF_y;
      g_beta[b] += -(gF_x * (ox - gx) + gF_y * (oy - gy));
      go_x += -be * gF_x;
      go_y += -be * gF_y;
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        const float r0 = std::sqrt(dx * dx + dy * dy);
        const float r = r0 < 1e-9f ? 1e-9f : r0;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        g_alphas[b * N + j] += -(gF_x * dbdd * nhx + gF_y * dbdd * nhy);
        const float g_dbdd = -ab[j] * (gF_x * nhx + gF_y * nhy);
        const float g_nhx = -(ab[j] * dbdd) * gF_x, g_nhy = -(ab[j] * dbdd) * gF_y;
        const float g_d = g_dbdd * detail::ipc_dbdd_pw_grad(d, dh);
        go_x += g_d * nhx;
        go_y += g_d * nhy;
        if (r0 > 1e-9f) {
          const float dot = nhx * g_nhx + nhy * g_nhy;
          go_x += (g_nhx - nhx * dot) / r;
          go_y += (g_nhy - nhy * dot) / r;
        }
      }
      if (s == s_star && j_star >= 0) {
        const float dx = ox - Cb[2 * j_star], dy = oy - Cb[2 * j_star + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        go_x += gmin * (dx / r);
        go_y += gmin * (dy / r);
      }
      aox = go_x;
      aoy = go_y;
      avx = gv_x;
      avy = gv_y;
    }
  });
}

double multi_start_penalty(const float *alphas, const float *beta, const float *gamma,
                           const float *o0, const float *v0, const float *goal, const float *C,
                           const float *R, const std::uint8_t *mask, const float *rr,
                           const float *d_hat, const float *dt, const int *H, int B, int N,
                           const multi_start_params &p, float *g_alphas, float *g_beta,
                           float *g_gamma, int num_threads) {
  if (N == 0 || B == 0)
    return 0.0;
  // Build the (data) multi-start point o_ms: 90% toward the nearest obstacle,
  // with the feasibility fallback. v_ms = v0.
  std::vector<float> o_ms(2 * B), v_ms(v0, v0 + 2 * B), dt_ms(B);
  std::vector<int> H_ms(B, p.ms_h);
  for (int b = 0; b < B; ++b) {
    const float rr_eff = p.margin_factor * rr[b];
    const float ox = o0[2 * b], oy = o0[2 * b + 1];
    float dmin0 = std::numeric_limits<float>::infinity(), nx = 0.0f, ny = 0.0f;
    for (int j = 0; j < N; ++j) {
      if (!mask[b * N + j])
        continue;
      const float dx = ox - C[(b * N + j) * 2], dy = oy - C[(b * N + j) * 2 + 1];
      float r = std::sqrt(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      const float d = r - (R[b * N + j] + rr_eff);
      if (d < dmin0) {
        dmin0 = d;
        nx = dx / r;
        ny = dy / r;
      }
    }
    const float stepx = 0.9f * dmin0 * nx, stepy = 0.9f * dmin0 * ny;
    float omsx = ox - stepx, omsy = oy - stepy;
    // feasibility: if o_ms penetrates any obstacle, nudge back out
    float dms = std::numeric_limits<float>::infinity();
    for (int j = 0; j < N; ++j) {
      if (!mask[b * N + j])
        continue;
      const float dx = omsx - C[(b * N + j) * 2], dy = omsy - C[(b * N + j) * 2 + 1];
      float r = std::sqrt(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      const float d = r - (R[b * N + j] + rr_eff);
      if (d < dms)
        dms = d;
    }
    if (dms < 0.0f) {
      omsx = ox + 0.5f * stepx;
      omsy = oy + 0.5f * stepy;
    }
    o_ms[2 * b] = omsx;
    o_ms[2 * b + 1] = omsy;
    dt_ms[b] = p.ms_dt_mult * dt[b];
  }

  const geom_rollout_params gp{p.margin_factor, p.mass};
  std::vector<float> o_run(o_ms), v_run(v_ms), min_clear(B);
  integrate_surrogate_v2(o_run.data(), v_run.data(), goal, C, R, mask, alphas, beta, gamma, rr,
                         d_hat, dt_ms.data(), H_ms.data(), B, N, gp, min_clear.data(), num_threads);

  double L = 0.0;
  for (int b = 0; b < B; ++b) {
    const float x = -min_clear[b] / p.tau;
    L += (x > 20.0f) ? x : std::log1p(std::exp(x)); // softplus(-clr/tau)
  }
  L /= (double)B;

  if (g_alphas || g_beta || g_gamma) {
    std::vector<float> g_clr(B);
    for (int b = 0; b < B; ++b) {
      const float s = 1.0f / (1.0f + std::exp(-(-min_clear[b] / p.tau))); // sigmoid(-clr/tau)
      g_clr[b] = -s / (p.tau * (float)B);                                 // d L / d clr
    }
    integrate_surrogate_v2_vjp(o_ms.data(), v_ms.data(), goal, C, R, mask, alphas, beta, gamma, rr,
                               d_hat, dt_ms.data(), H_ms.data(), B, N, gp, nullptr, nullptr,
                               g_clr.data(), g_alphas, g_beta, g_gamma, num_threads);
  }
  return L;
}

} // namespace nav
} // namespace cvc
